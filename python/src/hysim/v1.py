"""Multi-session, revision-aware client for the /api/v1 runtime."""

from __future__ import annotations

import hashlib
import json
import math
import time
import uuid
from dataclasses import dataclass
from typing import Any, Callable, Iterator, Mapping, Sequence

from .client import ApiCallEvent, AuditHook
from .errors import ApiError, BusyError, JobFailedError, TransportError
from .models import AnalysisResult, BusDomain, BusRef, OPFRequest, PowerFlowRequest
from .resources import ResultFrameChunk, SubgraphView, TopologyChunk, ViolationChunk
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
        audit_hook: AuditHook | None = None,
    ) -> None:
        self.transport = transport or UrllibTransport(base_url, timeout=timeout)
        self.timeout = timeout
        self.audit_hook = audit_hook

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
        _validate_identifier(session_id, "session_id")
        body, response = self._request("GET", f"/api/v1/sessions/{session_id}")
        return self._session_from(body, response)

    def list_sessions(self) -> tuple[Mapping[str, Any], ...]:
        body, _ = self._request("GET", "/api/v1/sessions")
        sessions = body.get("sessions", [])
        if not isinstance(sessions, list):
            raise TransportError("v1 session collection is malformed")
        return tuple(item for item in sessions if isinstance(item, Mapping))

    def get_job(self, job_id: str) -> "HySimJob":
        _validate_identifier(job_id, "job_id")
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
        started = time.monotonic()
        request_id = str(uuid.uuid4())
        audit_value = json.dumps(
            {"payload": payload, "query": query},
            ensure_ascii=False,
            sort_keys=True,
            separators=(",", ":"),
        ).encode("utf-8")
        digest = hashlib.sha256(audit_value).hexdigest()
        request_headers = dict(headers or {})
        request_headers.setdefault("X-HySim-Request-ID", request_id)
        response: TransportResponse | None = None
        error: str | None = None
        model_revision = 0
        try:
            response = self.transport.request(
                method,
                path,
                json_body=payload if method.upper() != "GET" else None,
                query=query,
                headers=request_headers,
                timeout=timeout or self.timeout,
            )
            if response.status_code == 204:
                return {}, response
            parsed = json.loads(response.body.decode("utf-8")) if response.body else {}
            if not isinstance(parsed, dict):
                raise TransportError(f"{path} returned a non-object JSON response")
            revision_value = parsed.get("model_revision")
            if isinstance(revision_value, int) and not isinstance(revision_value, bool):
                model_revision = revision_value
            if response.status_code >= 400:
                message = str(
                    parsed.get("message") or parsed.get("error") or "request failed"
                )
                error_type = BusyError if response.status_code == 409 else ApiError
                raise error_type(response.status_code, path, message, parsed)
            return parsed, response
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            error = str(exc)
            status = response.status_code if response is not None else "unknown"
            raise TransportError(
                f"{path} returned non-JSON content with status {status}"
            ) from exc
        except Exception as exc:
            error = str(exc)
            raise
        finally:
            if self.audit_hook is not None:
                self.audit_hook(
                    ApiCallEvent(
                        request_id=request_id,
                        method=method.upper(),
                        path=path,
                        model_revision=model_revision,
                        request_sha256=digest,
                        status_code=response.status_code if response else None,
                        elapsed_seconds=time.monotonic() - started,
                        error=error,
                    )
                )


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
        _validate_lod(lod, maximum=2)
        _validate_page(offset, limit, maximum_limit=50_000)
        query: dict[str, str | int | float] = {
            "lod": lod,
            "offset": offset,
            "limit": limit,
        }
        query.update(_viewport_query(viewport))
        body, _ = self.client._request(
            "GET", f"/api/v1/sessions/{self.session_id}/topology", query=query
        )
        return body

    def topology_chunk(
        self,
        *,
        lod: int = 2,
        offset: int = 0,
        limit: int = 10_000,
        viewport: tuple[float, float, float, float] | None = None,
    ) -> TopologyChunk:
        return TopologyChunk.from_payload(
            self.topology(lod=lod, offset=offset, limit=limit, viewport=viewport)
        )

    def topology_pages(
        self,
        *,
        lod: int = 2,
        page_size: int = 10_000,
        viewport: tuple[float, float, float, float] | None = None,
    ) -> Iterator[TopologyChunk]:
        """Iterate topology node pages without downloading the full model."""

        offset = 0
        while True:
            page = self.topology_chunk(
                lod=lod, offset=offset, limit=page_size, viewport=viewport
            )
            yield page
            if page.next_offset is None:
                return
            if page.next_offset <= offset:
                raise TransportError("topology pagination did not advance")
            offset = page.next_offset

    def iter_topology_nodes(
        self,
        *,
        lod: int = 2,
        page_size: int = 10_000,
        viewport: tuple[float, float, float, float] | None = None,
    ) -> Iterator[Mapping[str, Any]]:
        for page in self.topology_pages(
            lod=lod, page_size=page_size, viewport=viewport
        ):
            yield from page.nodes

    def subgraph(
        self,
        domain: BusDomain | str | BusRef,
        index: int | None = None,
        *,
        depth: int = 2,
        max_nodes: int = 500,
    ) -> Mapping[str, Any]:
        domain_value, index_value = _bus_identity(domain, index)
        if isinstance(depth, bool) or not isinstance(depth, int) or not 0 <= depth <= 8:
            raise ValueError("depth must be an integer in [0, 8]")
        if (
            isinstance(max_nodes, bool)
            or not isinstance(max_nodes, int)
            or not 1 <= max_nodes <= 5_000
        ):
            raise ValueError("max_nodes must be an integer in [1, 5000]")
        body, _ = self.client._request(
            "GET",
            f"/api/v1/sessions/{self.session_id}/subgraph",
            query={
                "domain": domain_value,
                "index": index_value,
                "depth": depth,
                "max_nodes": max_nodes,
            },
        )
        return body

    def subgraph_view(
        self,
        domain: BusDomain | str | BusRef,
        index: int | None = None,
        *,
        depth: int = 2,
        max_nodes: int = 500,
    ) -> SubgraphView:
        return SubgraphView.from_payload(
            self.subgraph(domain, index, depth=depth, max_nodes=max_nodes)
        )

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
        indices: Sequence[int] | None = None,
        viewport: tuple[float, float, float, float] | None = None,
        lod: int = 2,
    ) -> Mapping[str, Any]:
        _validate_step(step)
        _validate_domain(domain, allow_all=True)
        _validate_lod(lod, maximum=3)
        _validate_page(offset, limit, maximum_limit=50_000)
        query: dict[str, str | int | float] = {
            "domain": domain,
            "offset": offset,
            "limit": limit,
            "lod": lod,
        }
        checked_indices = _validate_indices(indices)
        if checked_indices:
            query["indices"] = ",".join(str(value) for value in checked_indices)
        query.update(_viewport_query(viewport))
        body, _ = self.client._request(
            "GET", f"/api/v1/jobs/{self.job_id}/frames/{step}", query=query
        )
        return body

    def frame_chunk(
        self,
        step: int = 0,
        *,
        domain: str = "all",
        offset: int = 0,
        limit: int = 5_000,
        indices: Sequence[int] | None = None,
        viewport: tuple[float, float, float, float] | None = None,
        lod: int = 2,
    ) -> ResultFrameChunk:
        return ResultFrameChunk.from_payload(
            self.frame(
                step,
                domain=domain,
                offset=offset,
                limit=limit,
                indices=indices,
                viewport=viewport,
                lod=lod,
            )
        )

    def frame_pages(
        self,
        step: int = 0,
        *,
        domain: str = "all",
        page_size: int = 5_000,
        indices: Sequence[int] | None = None,
        viewport: tuple[float, float, float, float] | None = None,
        lod: int = 2,
    ) -> Iterator[ResultFrameChunk]:
        offset = 0
        while True:
            page = self.frame_chunk(
                step,
                domain=domain,
                offset=offset,
                limit=page_size,
                indices=indices,
                viewport=viewport,
                lod=lod,
            )
            yield page
            if page.next_offset is None:
                return
            if page.next_offset <= offset:
                raise TransportError("result-frame pagination did not advance")
            offset = page.next_offset

    def iter_frame_nodes(
        self,
        step: int = 0,
        **kwargs: Any,
    ) -> Iterator[Mapping[str, Any]]:
        for page in self.frame_pages(step, **kwargs):
            yield from page.nodes

    def violations(
        self,
        step: int = 0,
        *,
        limit: int = 200,
        vmin: float = 0.9,
        vmax: float = 1.1,
        loading_limit_pct: float = 100.0,
    ) -> Mapping[str, Any]:
        _validate_step(step)
        if isinstance(limit, bool) or not isinstance(limit, int) or not 1 <= limit <= 5_000:
            raise ValueError("limit must be an integer in [1, 5000]")
        for name, value in {"vmin": vmin, "vmax": vmax,
                            "loading_limit_pct": loading_limit_pct}.items():
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise TypeError(f"{name} must be a number")
            if not math.isfinite(value):
                raise ValueError(f"{name} must be finite")
        if vmin >= vmax:
            raise ValueError("vmin must be less than vmax")
        if loading_limit_pct < 0:
            raise ValueError("loading_limit_pct must be non-negative")
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

    def violation_chunk(
        self,
        step: int = 0,
        *,
        limit: int = 200,
        vmin: float = 0.9,
        vmax: float = 1.1,
        loading_limit_pct: float = 100.0,
    ) -> ViolationChunk:
        return ViolationChunk.from_payload(
            self.violations(
                step,
                limit=limit,
                vmin=vmin,
                vmax=vmax,
                loading_limit_pct=loading_limit_pct,
            )
        )

    def wait(
        self,
        *,
        timeout: float = 600.0,
        poll_interval: float = 0.1,
        max_poll_interval: float = 2.0,
        backoff: float = 1.5,
        on_update: Callable[["HySimJob"], None] | None = None,
    ) -> "HySimJob":
        for name, value in {
            "timeout": timeout,
            "poll_interval": poll_interval,
            "max_poll_interval": max_poll_interval,
            "backoff": backoff,
        }.items():
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise TypeError(f"{name} must be a number")
            if not math.isfinite(value):
                raise ValueError(f"{name} must be finite")
        if timeout < 0:
            raise ValueError("timeout must be non-negative")
        if poll_interval < 0 or max_poll_interval < poll_interval:
            raise ValueError("poll intervals must be non-negative and ordered")
        if backoff < 1.0:
            raise ValueError("backoff must be at least 1.0")
        deadline = time.monotonic() + timeout
        interval = poll_interval
        while True:
            self.refresh()
            if on_update is not None:
                on_update(self)
            if self.terminal:
                return self
            if time.monotonic() >= deadline:
                raise TimeoutError(f"HySim job {self.job_id} did not finish in time")
            time.sleep(min(interval, max(0.0, deadline - time.monotonic())))
            interval = min(max_poll_interval, interval * backoff)

    def wait_result(self, **kwargs: Any) -> AnalysisResult:
        """Wait for terminal state and return the validated result wrapper."""

        return self.wait(**kwargs).result()

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


def _validate_identifier(value: str, name: str) -> None:
    if not isinstance(value, str) or not value or any(
        not (char.isascii() and (char.isalnum() or char in "_-")) for char in value
    ):
        raise ValueError(f"{name} must contain only ASCII letters, digits, '_' or '-'")


def _validate_lod(value: int, *, maximum: int) -> None:
    if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value <= maximum:
        raise ValueError(f"lod must be an integer in [0, {maximum}]")


def _validate_page(offset: int, limit: int, *, maximum_limit: int) -> None:
    if isinstance(offset, bool) or not isinstance(offset, int) or offset < 0:
        raise ValueError("offset must be a non-negative integer")
    if isinstance(limit, bool) or not isinstance(limit, int) or not 1 <= limit <= maximum_limit:
        raise ValueError(f"limit must be an integer in [1, {maximum_limit}]")


def _validate_step(step: int) -> None:
    if isinstance(step, bool) or not isinstance(step, int) or step < 0:
        raise ValueError("step must be a non-negative integer")


def _validate_domain(domain: str, *, allow_all: bool) -> str:
    choices = {"ac", "dc"} | ({"all"} if allow_all else set())
    if domain not in choices:
        raise ValueError(f"domain must be one of {sorted(choices)}")
    return domain


def _bus_identity(
    domain: BusDomain | str | BusRef, index: int | None
) -> tuple[str, int]:
    if isinstance(domain, BusRef):
        if index is not None:
            raise ValueError("index must be omitted when domain is a BusRef")
        return _validate_domain(domain.domain.value, allow_all=False), domain.index
    domain_value = domain.value if isinstance(domain, BusDomain) else str(domain)
    _validate_domain(domain_value, allow_all=False)
    if isinstance(index, bool) or not isinstance(index, int):
        raise TypeError("index must be an integer stable bus ID")
    return domain_value, index


def _validate_indices(indices: Sequence[int] | None) -> tuple[int, ...]:
    if indices is None:
        return ()
    checked: list[int] = []
    for value in indices:
        if isinstance(value, bool) or not isinstance(value, int):
            raise TypeError("indices must contain only integer stable IDs")
        checked.append(value)
    return tuple(checked)


def _viewport_query(
    viewport: tuple[float, float, float, float] | None,
) -> dict[str, float]:
    if viewport is None:
        return {}
    if not isinstance(viewport, tuple) or len(viewport) != 4:
        raise TypeError("viewport must be a four-number tuple")
    values: list[float] = []
    for value in viewport:
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise TypeError("viewport values must be numbers")
        number = float(value)
        if not math.isfinite(number):
            raise ValueError("viewport values must be finite")
        values.append(number)
    xmin, ymin, xmax, ymax = values
    if xmin > xmax or ymin > ymax:
        raise ValueError("viewport minima must not exceed maxima")
    return dict(zip(("xmin", "ymin", "xmax", "ymax"), values))
