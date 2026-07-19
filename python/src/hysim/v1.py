"""Multi-session, revision-aware client for the /api/v1 runtime."""

from __future__ import annotations

import json
import time
from dataclasses import dataclass
from typing import Any, Mapping

from .errors import ApiError, BusyError, JobFailedError, TransportError
from .models import AnalysisResult, OPFRequest, PowerFlowRequest
from .transport import Transport, TransportResponse, UrllibTransport


TERMINAL_JOB_STATES = frozenset({"succeeded", "failed", "cancelled"})


class HySimV1Client:
    """Client for isolated sessions and asynchronous revision-bound jobs."""

    def __init__(
        self,
        base_url: str = "http://127.0.0.1:8088",
        *,
        transport: Transport | None = None,
        timeout: float = 600.0,
    ) -> None:
        self.transport = transport or UrllibTransport(base_url, timeout=timeout)
        self.timeout = timeout

    def capabilities(self) -> Mapping[str, Any]:
        body, _ = self._request("GET", "/api/v1")
        return body

    def create_session(
        self,
        *,
        case: str | None = None,
        model: Mapping[str, Any] | None = None,
    ) -> "HySimV1Session":
        if case is not None and model is not None:
            raise ValueError("provide either case or model, not both")
        payload: dict[str, Any] = {}
        if case is not None:
            payload["case"] = case
        if model is not None:
            payload["model"] = dict(model)
        body, response = self._request("POST", "/api/v1/sessions", payload=payload)
        return self._session_from(body, response)

    def get_session(self, session_id: str) -> "HySimV1Session":
        body, response = self._request("GET", f"/api/v1/sessions/{session_id}")
        return self._session_from(body, response)

    def list_sessions(self) -> tuple[Mapping[str, Any], ...]:
        body, _ = self._request("GET", "/api/v1/sessions")
        sessions = body.get("sessions", [])
        if not isinstance(sessions, list):
            raise TransportError("v1 session collection is malformed")
        return tuple(item for item in sessions if isinstance(item, Mapping))

    def get_job(self, job_id: str) -> "HySimJob":
        body, _ = self._request("GET", f"/api/v1/jobs/{job_id}")
        return HySimJob(self, job_id, body)

    def _session_from(
        self, body: Mapping[str, Any], response: TransportResponse
    ) -> "HySimV1Session":
        session_id = body.get("session_id")
        revision = body.get("model_revision")
        if not isinstance(session_id, str) or not isinstance(revision, int):
            raise TransportError("v1 session response is malformed")
        etag = _header(response, "ETag") or body.get("etag")
        if not isinstance(etag, str):
            raise TransportError("v1 session response is missing ETag")
        return HySimV1Session(self, session_id, revision, etag, body)

    def _request(
        self,
        method: str,
        path: str,
        *,
        payload: Mapping[str, Any] | None = None,
        headers: Mapping[str, str] | None = None,
        query: Mapping[str, str | int | float] | None = None,
        timeout: float | None = None,
    ) -> tuple[Mapping[str, Any], TransportResponse]:
        response = self.transport.request(
            method,
            path,
            json_body=payload if method.upper() != "GET" else None,
            query=query,
            headers=headers,
            timeout=timeout or self.timeout,
        )
        if response.status_code == 204:
            return {}, response
        try:
            parsed = json.loads(response.body.decode("utf-8")) if response.body else {}
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise TransportError(
                f"{path} returned non-JSON content with status {response.status_code}"
            ) from exc
        if not isinstance(parsed, dict):
            raise TransportError(f"{path} returned a non-object JSON response")
        if response.status_code >= 400:
            message = str(parsed.get("message") or parsed.get("error") or "request failed")
            error_type = BusyError if response.status_code == 409 else ApiError
            raise error_type(response.status_code, path, message, parsed)
        return parsed, response


@dataclass
class HySimV1Session:
    client: HySimV1Client
    session_id: str
    model_revision: int
    etag: str
    raw: Mapping[str, Any]

    @property
    def has_model(self) -> bool:
        return bool(self.raw.get("has_model"))

    def refresh(self) -> "HySimV1Session":
        current = self.client.get_session(self.session_id)
        self._adopt(current)
        return self

    def get_model(self) -> Mapping[str, Any]:
        body, response = self.client._request(
            "GET", f"/api/v1/sessions/{self.session_id}/model"
        )
        self._adopt_headers(body, response)
        model = body.get("model")
        if not isinstance(model, Mapping):
            raise TransportError("v1 model resource is malformed")
        return model

    def topology(
        self,
        *,
        lod: int = 2,
        offset: int = 0,
        limit: int = 10_000,
        viewport: tuple[float, float, float, float] | None = None,
    ) -> Mapping[str, Any]:
        query: dict[str, str | int | float] = {
            "lod": lod,
            "offset": offset,
            "limit": limit,
        }
        if viewport is not None:
            query.update(zip(("xmin", "ymin", "xmax", "ymax"), viewport))
        body, _ = self.client._request(
            "GET", f"/api/v1/sessions/{self.session_id}/topology", query=query
        )
        return body

    def subgraph(
        self,
        domain: str,
        index: int,
        *,
        depth: int = 2,
        max_nodes: int = 500,
    ) -> Mapping[str, Any]:
        body, _ = self.client._request(
            "GET",
            f"/api/v1/sessions/{self.session_id}/subgraph",
            query={
                "domain": domain,
                "index": index,
                "depth": depth,
                "max_nodes": max_nodes,
            },
        )
        return body

    def replace_model(
        self,
        *,
        case: str | None = None,
        model: Mapping[str, Any] | None = None,
    ) -> "HySimV1Session":
        if (case is None) == (model is None):
            raise ValueError("provide exactly one of case or model")
        payload = {"case": case} if case is not None else {"model": dict(model or {})}
        body, response = self.client._request(
            "PUT",
            f"/api/v1/sessions/{self.session_id}/model",
            payload=payload,
            headers={"If-Match": self.etag},
        )
        self._adopt_headers(body, response)
        self.raw = body
        return self

    def submit(
        self,
        analysis: str,
        request: Mapping[str, Any] | None = None,
    ) -> "HySimJob":
        body, _ = self.client._request(
            "POST",
            f"/api/v1/sessions/{self.session_id}/jobs",
            payload={"analysis": analysis, "request": dict(request or {})},
            headers={"If-Match": self.etag},
        )
        job_id = body.get("job_id")
        if not isinstance(job_id, str):
            raise TransportError("v1 job response is missing job_id")
        return HySimJob(self.client, job_id, body)

    def power_flow(
        self, request: PowerFlowRequest | Mapping[str, Any] | None = None
    ) -> "HySimJob":
        if request is None:
            payload = PowerFlowRequest().to_dict()
        elif isinstance(request, PowerFlowRequest):
            payload = request.to_dict()
        else:
            payload = dict(request)
        return self.submit("power_flow", payload)

    def optimal_power_flow(
        self, request: OPFRequest | Mapping[str, Any] | None = None
    ) -> "HySimJob":
        if request is None:
            payload = OPFRequest().to_dict()
        elif isinstance(request, OPFRequest):
            payload = request.to_dict()
        else:
            payload = dict(request)
        return self.submit("optimal_power_flow", payload)

    def list_jobs(self) -> tuple[Mapping[str, Any], ...]:
        body, _ = self.client._request(
            "GET", f"/api/v1/sessions/{self.session_id}/jobs"
        )
        jobs = body.get("jobs", [])
        if not isinstance(jobs, list):
            raise TransportError("v1 job collection is malformed")
        return tuple(item for item in jobs if isinstance(item, Mapping))

    def delete(self) -> None:
        self.client._request(
            "DELETE",
            f"/api/v1/sessions/{self.session_id}",
            headers={"If-Match": self.etag},
        )

    def _adopt(self, other: "HySimV1Session") -> None:
        self.model_revision = other.model_revision
        self.etag = other.etag
        self.raw = other.raw

    def _adopt_headers(
        self, body: Mapping[str, Any], response: TransportResponse
    ) -> None:
        revision = body.get("model_revision")
        etag = _header(response, "ETag")
        if isinstance(revision, int):
            self.model_revision = revision
        if isinstance(etag, str):
            self.etag = etag


@dataclass
class HySimJob:
    client: HySimV1Client
    job_id: str
    raw: Mapping[str, Any]

    @property
    def state(self) -> str:
        return str(self.raw.get("state", "unknown"))

    @property
    def terminal(self) -> bool:
        return self.state in TERMINAL_JOB_STATES

    def refresh(self) -> "HySimJob":
        body, _ = self.client._request("GET", f"/api/v1/jobs/{self.job_id}")
        self.raw = body
        return self

    def frame(
        self,
        step: int = 0,
        *,
        domain: str = "all",
        offset: int = 0,
        limit: int = 5_000,
        indices: tuple[int, ...] | None = None,
        viewport: tuple[float, float, float, float] | None = None,
        lod: int = 2,
    ) -> Mapping[str, Any]:
        query: dict[str, str | int | float] = {
            "domain": domain,
            "offset": offset,
            "limit": limit,
            "lod": lod,
        }
        if indices:
            query["indices"] = ",".join(str(value) for value in indices)
        if viewport is not None:
            query.update(zip(("xmin", "ymin", "xmax", "ymax"), viewport))
        body, _ = self.client._request(
            "GET", f"/api/v1/jobs/{self.job_id}/frames/{step}", query=query
        )
        return body

    def violations(
        self,
        step: int = 0,
        *,
        limit: int = 200,
        vmin: float = 0.9,
        vmax: float = 1.1,
        loading_limit_pct: float = 100.0,
    ) -> Mapping[str, Any]:
        body, _ = self.client._request(
            "GET",
            f"/api/v1/jobs/{self.job_id}/violations",
            query={
                "step": step,
                "limit": limit,
                "vmin": vmin,
                "vmax": vmax,
                "loading_limit_pct": loading_limit_pct,
            },
        )
        return body

    def wait(
        self,
        *,
        timeout: float = 600.0,
        poll_interval: float = 0.1,
    ) -> "HySimJob":
        deadline = time.monotonic() + timeout
        while True:
            self.refresh()
            if self.terminal:
                return self
            if time.monotonic() >= deadline:
                raise TimeoutError(f"HySim job {self.job_id} did not finish in time")
            time.sleep(poll_interval)

    def cancel(self) -> "HySimJob":
        body, _ = self.client._request(
            "POST", f"/api/v1/jobs/{self.job_id}/cancel", payload={}
        )
        self.raw = body
        return self

    def delete(self) -> None:
        self.client._request("DELETE", f"/api/v1/jobs/{self.job_id}")

    def result(self) -> AnalysisResult:
        if self.state == "failed":
            error = self.raw.get("error", {})
            message = (
                error.get("message", "analysis failed")
                if isinstance(error, Mapping)
                else error
            )
            raise JobFailedError(f"HySim job {self.job_id} failed: {message}")
        if self.state != "succeeded":
            raise JobFailedError(
                f"HySim job {self.job_id} has no result in state {self.state}"
            )
        payload = self.raw.get("result")
        if not isinstance(payload, Mapping):
            raise TransportError("succeeded v1 job is missing its result")
        analysis = str(self.raw.get("analysis", "unknown"))
        revision = self.raw.get("model_revision", 0)
        if not isinstance(revision, int):
            revision = 0
        return AnalysisResult(
            analysis=analysis,
            route=f"/api/v1/jobs/{self.job_id}",
            request_id=self.job_id,
            model_revision=revision,
            data=payload,
        )


def _header(response: TransportResponse, name: str) -> str | None:
    wanted = name.lower()
    for key, value in response.headers.items():
        if key.lower() == wanted:
            return value
    return None
