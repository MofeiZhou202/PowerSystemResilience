"""High-level client for the authoritative C++ runtime."""

from __future__ import annotations

import hashlib
import json
import threading
import time
import uuid
from dataclasses import dataclass
from typing import Any, Callable, Mapping

from .errors import ApiError, BusyError, TransportError
from .models import AnalysisResult, OPFRequest, PowerFlowRequest
from .transport import Transport, TransportResponse, UrllibTransport


ANALYSIS_ROUTES: dict[str, str] = {
    "power_flow": "/api/session/pf",
    "optimal_power_flow": "/api/session/opf",
    "short_circuit": "/api/session/sc",
    "detailed_short_circuit": "/api/session/sc_detailed",
    "dc_short_circuit": "/api/session/dc_sc",
    "harmonics": "/api/session/harmonics",
    "three_phase_harmonics": "/api/session/harmonics_3ph",
    "harmonics_frequency_scan": "/api/session/harmonics_freqscan",
    "transient": "/api/session/run_transient",
    "small_signal": "/api/session/small_signal",
    "time_series_power_flow": "/api/session/run_ts_pf",
    "annual_production": "/api/session/run_annual_sim",
    "carbon_flow": "/api/session/run_carbon",
    "dynamic_carbon_flow": "/api/session/run_dynamic_carbon",
    "topology": "/api/session/topology",
    "network_reduction": "/api/session/network_reduction",
    "reconfiguration": "/api/session/run_reconfig",
    "hosting_capacity": "/api/session/run_hosting_capacity",
    "reliability": "/api/session/run_reliability",
    "resilience": "/api/session/run_distribution_resilience",
    "market_clearing": "/api/session/run_market_clearing",
    "real_time_market": "/api/session/run_real_time_market",
    "repeated_market_game": "/api/session/run_repeated_market_game",
    "integrated_energy": "/api/session/run_campus_ies",
    "ev_power_traffic": "/api/session/run_ev_traffic",
    "scenario_generation": "/api/session/generate_scenarios",
    "typhoon_faults": "/api/session/generate_typhoon_faults",
    "sppt_guard": "/api/session/sppt_guard",
}


@dataclass(frozen=True)
class ApiCallEvent:
    request_id: str
    method: str
    path: str
    model_revision: int
    request_sha256: str
    status_code: int | None
    elapsed_seconds: float
    error: str | None = None


AuditHook = Callable[[ApiCallEvent], None]


class HySimClient:
    """Typed facade over the current process-global HTTP session.

    Model-changing calls and analyses are serialized within one client by
    default. Separate clients still share the C++ process session; independent
    experiments should use independent server processes.
    """

    def __init__(
        self,
        base_url: str = "http://127.0.0.1:8088",
        *,
        transport: Transport | None = None,
        timeout: float = 600.0,
        audit_hook: AuditHook | None = None,
        serialize_model_access: bool = True,
    ) -> None:
        self.transport = transport or UrllibTransport(base_url, timeout=timeout)
        self.timeout = timeout
        self.audit_hook = audit_hook
        self._model_revision = 0
        self._state_lock = threading.Lock()
        self._operation_lock: Any = (
            threading.RLock() if serialize_model_access else _NullLock()
        )

    @property
    def model_revision(self) -> int:
        with self._state_lock:
            return self._model_revision

    def list_cases(self) -> Mapping[str, Any]:
        return self._request_json("GET", "/api/cases")

    def status(self) -> Mapping[str, Any]:
        return self._request_json("GET", "/api/session/status")

    def wait_until_idle(self, *, timeout: float = 600.0, interval: float = 0.25) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if not bool(self.status().get("busy")):
                return True
            time.sleep(interval)
        return False

    def cancel(self) -> Mapping[str, Any]:
        # Cancellation intentionally bypasses the operation lock.
        return self._request_json("POST", "/api/session/cancel", payload={})

    def new_system(self) -> Mapping[str, Any]:
        return self._modify_model("/api/session/new_empty", {})

    def load_builtin(self, case: str) -> Mapping[str, Any]:
        return self._modify_model("/api/session/load_builtin", {"case": case})

    def load_matpower(self, filename: str) -> Mapping[str, Any]:
        return self._modify_model("/api/session/load_matpower", {"filename": filename})

    def load_model(self, model: Mapping[str, Any] | str) -> Mapping[str, Any]:
        model_json = model if isinstance(model, str) else json.dumps(model, ensure_ascii=False)
        return self._modify_model(
            "/api/session/load_json_string", {"json_string": model_json}
        )

    def update_components(self, components: Mapping[str, Any]) -> Mapping[str, Any]:
        return self._modify_model("/api/session/update_components", components)

    def export_model(self) -> Mapping[str, Any]:
        response = self._request_json("POST", "/api/session/export_json", payload={})
        model_json = response.get("json_string")
        if not isinstance(model_json, str):
            raise TransportError("export_json response is missing json_string")
        parsed = json.loads(model_json)
        if not isinstance(parsed, dict):
            raise TransportError("exported model must be a JSON object")
        return parsed

    def power_flow(
        self,
        request: PowerFlowRequest | Mapping[str, Any] | None = None,
        *,
        timeout: float | None = None,
    ) -> AnalysisResult:
        if request is None:
            payload = PowerFlowRequest().to_dict()
        elif isinstance(request, PowerFlowRequest):
            payload = request.to_dict()
        else:
            payload = dict(request)
        return self._run_analysis("power_flow", payload, timeout=timeout)

    def optimal_power_flow(
        self,
        request: OPFRequest | Mapping[str, Any] | None = None,
        *,
        timeout: float | None = None,
    ) -> AnalysisResult:
        if request is None:
            payload = OPFRequest().to_dict()
        elif isinstance(request, OPFRequest):
            payload = request.to_dict()
        else:
            payload = dict(request)
        return self._run_analysis("optimal_power_flow", payload, timeout=timeout)

    def run_analysis(
        self,
        analysis: str,
        payload: Mapping[str, Any] | None = None,
        *,
        timeout: float | None = None,
    ) -> AnalysisResult:
        """Run a registered production analysis without hiding its raw schema."""

        if analysis not in ANALYSIS_ROUTES:
            choices = ", ".join(sorted(ANALYSIS_ROUTES))
            raise ValueError(f"unknown analysis {analysis!r}; choose one of: {choices}")
        return self._run_analysis(analysis, payload or {}, timeout=timeout)

    def tspf_frame(self, step: int) -> AnalysisResult:
        return self._get_frame(
            "time_series_power_flow_frame",
            "/api/session/tspf/frame",
            "step",
            step,
        )

    def transient_frame(self, index: int) -> AnalysisResult:
        return self._get_frame("transient_frame", "/api/session/transient/frame", "index", index)

    def _get_frame(self, analysis: str, route: str, key: str, index: int) -> AnalysisResult:
        request_id = str(uuid.uuid4())
        revision = self.model_revision
        data = self._request_json(
            "GET", route, query={key: index}, request_id=request_id
        )
        return AnalysisResult(analysis, route, request_id, revision, data)

    def _modify_model(self, path: str, payload: Mapping[str, Any]) -> Mapping[str, Any]:
        with self._operation_lock:
            result = self._request_json("POST", path, payload=payload)
            with self._state_lock:
                self._model_revision += 1
            return result

    def _run_analysis(
        self,
        analysis: str,
        payload: Mapping[str, Any],
        *,
        timeout: float | None,
    ) -> AnalysisResult:
        route = ANALYSIS_ROUTES[analysis]
        request_id = str(uuid.uuid4())
        with self._operation_lock:
            revision = self.model_revision
            data = self._request_json(
                "POST",
                route,
                payload=payload,
                timeout=timeout,
                request_id=request_id,
            )
        return AnalysisResult(analysis, route, request_id, revision, data)

    def _request_json(
        self,
        method: str,
        path: str,
        *,
        payload: Mapping[str, Any] | None = None,
        query: Mapping[str, str | int | float] | None = None,
        timeout: float | None = None,
        request_id: str | None = None,
    ) -> Mapping[str, Any]:
        request_id = request_id or str(uuid.uuid4())
        body_bytes = json.dumps(payload or {}, sort_keys=True, default=str).encode("utf-8")
        digest = hashlib.sha256(body_bytes).hexdigest()
        started = time.monotonic()
        response: TransportResponse | None = None
        error: str | None = None
        try:
            response = self.transport.request(
                method,
                path,
                json_body=payload if method.upper() != "GET" else None,
                query=query,
                headers={"X-HySim-Request-ID": request_id},
                timeout=timeout or self.timeout,
            )
            try:
                parsed = json.loads(response.body.decode("utf-8")) if response.body else {}
            except (UnicodeDecodeError, json.JSONDecodeError) as exc:
                raise TransportError(
                    f"{path} returned non-JSON content with status {response.status_code}"
                ) from exc
            if not isinstance(parsed, dict):
                raise TransportError(f"{path} returned a non-object JSON response")
            if response.status_code >= 400:
                message = str(parsed.get("error") or parsed.get("message") or "request failed")
                error_type = BusyError if response.status_code == 409 else ApiError
                raise error_type(response.status_code, path, message, parsed)
            return parsed
        except Exception as exc:
            error = str(exc)
            raise
        finally:
            if self.audit_hook is not None:
                event = ApiCallEvent(
                    request_id=request_id,
                    method=method.upper(),
                    path=path,
                    model_revision=self.model_revision,
                    request_sha256=digest,
                    status_code=response.status_code if response else None,
                    elapsed_seconds=time.monotonic() - started,
                    error=error,
                )
                self.audit_hook(event)


class _NullLock:
    def __enter__(self) -> "_NullLock":
        return self

    def __exit__(self, *_: object) -> None:
        return None
