"""High-level client for the authoritative C++ runtime."""

from __future__ import annotations

import hashlib
import json
import threading
import time
import uuid
from dataclasses import dataclass
from types import MappingProxyType
from typing import Any, Callable, Mapping

from .edition import EditionProfile
from .errors import (
    AnalysisDisabledError,
    ApiError,
    BusyError,
    TransportError,
    UnknownAnalysisError,
)
from .models import AnalysisResult, OPFRequest, PowerFlowRequest
from .transport import Transport, TransportResponse, UrllibTransport


ANALYSIS_ALIASES = MappingProxyType(
    {
        "resilience": "distribution_resilience",
        "integrated_energy": "campus_ies",
    }
)
ANALYSIS_ROUTES = MappingProxyType({
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
    "distribution_resilience": "/api/session/run_distribution_resilience",
    "market_clearing": "/api/session/run_market_clearing",
    "real_time_market": "/api/session/run_real_time_market",
    "repeated_market_game": "/api/session/run_repeated_market_game",
    "campus_ies": "/api/session/run_campus_ies",
    "ev_power_traffic": "/api/session/run_ev_traffic",
    "scenario_generation": "/api/session/generate_scenarios",
    "typhoon_faults": "/api/session/generate_typhoon_faults",
    "sppt_guard": "/api/session/sppt_guard",
})


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
        self._edition_profile: EditionProfile | None = None
        self._analysis_catalog: Any = None

    @property
    def edition_profile(self) -> EditionProfile:
        """Discover and cache this client's immutable edition profile."""

        self._ensure_edition_profile()
        assert self._edition_profile is not None
        return self._edition_profile

    @property
    def analysis_catalog(self) -> Any:
        """Return this client's immutable, edition-filtered analysis catalog."""

        self._ensure_edition_profile()
        return self._analysis_catalog

    def refresh_edition_profile(self) -> EditionProfile:
        """Refresh capabilities for this client without sharing process-global state."""

        from .analyses import ANALYSIS_CATALOG
        from .edition import EDITION_PROFILE_PATH

        payload = self._request_json("GET", EDITION_PROFILE_PATH)
        profile = EditionProfile.parse(payload, ANALYSIS_CATALOG)
        catalog = ANALYSIS_CATALOG.filtered(profile.enabled_names)
        with self._state_lock:
            self._edition_profile = profile
            self._analysis_catalog = catalog
        return profile

    def _ensure_edition_profile(self) -> None:
        if self._edition_profile is None:
            self.refresh_edition_profile()

    def require_analysis(self, analysis: str) -> Any:
        """Resolve aliases, distinguish unknown/disabled, and return the known spec."""

        from .analyses import ANALYSIS_CATALOG

        canonical = ANALYSIS_ALIASES.get(analysis, analysis)
        spec = ANALYSIS_CATALOG.get(canonical)
        if spec is None:
            raise UnknownAnalysisError(analysis)
        self._ensure_edition_profile()
        assert self._edition_profile is not None
        capability = self._edition_profile.capability(canonical)
        if capability is None:
            # Strict profile parsing normally catches this. Keep the gate closed if
            # a caller supplies a hand-built profile or the state is corrupted.
            raise TransportError(
                f"edition profile has no capability for SDK analysis {canonical!r}"
            )
        if not capability.enabled:
            raise AnalysisDisabledError(canonical, edition=self._edition_profile.edition)
        return spec

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
        return self._modify_model("new_empty", {})

    def load_builtin(self, case: str) -> Mapping[str, Any]:
        return self._modify_model("load_builtin", {"case": case})

    def load_matpower(self, filename: str) -> Mapping[str, Any]:
        return self._modify_model("load_matpower", {"filename": filename})

    def load_model(self, model: Mapping[str, Any] | str) -> Mapping[str, Any]:
        model_json = model if isinstance(model, str) else json.dumps(model, ensure_ascii=False)
        return self._modify_model("load_json_string", {"json_string": model_json})

    def update_components(self, components: Mapping[str, Any]) -> Mapping[str, Any]:
        return self._modify_model("update_components", components)

    def export_model(self) -> Mapping[str, Any]:
        spec = self.require_analysis("export_json")
        response = self._request_json(spec.method, spec.route, payload={})
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
        """Run an enabled production analysis after canonical alias resolution."""

        spec = self.require_analysis(analysis)
        return self.execute(
            name=spec.name,
            route=spec.route,
            method=spec.method,
            modifies_model=spec.mutates_model,
            payload=dict(payload or {}),
            timeout=timeout,
        )

    def execute(
        self,
        *,
        name: str,
        route: str,
        method: str = "POST",
        modifies_model: bool = False,
        payload: Mapping[str, Any] | None = None,
        query: Mapping[str, str | int | float] | None = None,
        timeout: float | None = None,
    ) -> AnalysisResult:
        """Run any catalog route with honest result and revision semantics.

        This is the low-level seam used by the comprehensive family facade. It
        validates the caller-provided metadata against the SDK catalog and the
        connected edition before transport, so it cannot bypass capability
        gating.
        """

        spec = self.require_analysis(name)
        if route != spec.route or method.upper() != spec.method:
            raise ValueError(
                f"analysis {spec.name!r} must use {spec.method} {spec.route}"
            )
        if modifies_model != spec.mutates_model:
            raise ValueError(
                f"analysis {spec.name!r} has inconsistent model-mutation metadata"
            )
        name = spec.name
        route = spec.route
        method = spec.method

        request_id = str(uuid.uuid4())
        with self._operation_lock:
            revision = self.model_revision
            data = self._request_json(
                method,
                route,
                payload=payload,
                query=query,
                timeout=timeout,
                request_id=request_id,
            )
            if modifies_model:
                with self._state_lock:
                    self._model_revision += 1
                revision = self._model_revision
        return AnalysisResult(name, route, request_id, revision, data)

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

    def _modify_model(self, analysis: str, payload: Mapping[str, Any]) -> Mapping[str, Any]:
        spec = self.require_analysis(analysis)
        if not spec.mutates_model:
            raise ValueError(f"analysis {spec.name!r} is not a model mutation")
        with self._operation_lock:
            result = self._request_json(spec.method, spec.route, payload=payload)
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
        spec = self.require_analysis(analysis)
        request_id = str(uuid.uuid4())
        with self._operation_lock:
            revision = self.model_revision
            data = self._request_json(
                spec.method,
                spec.route,
                payload=payload,
                timeout=timeout,
                request_id=request_id,
            )
        return AnalysisResult(spec.name, spec.route, request_id, revision, data)

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
