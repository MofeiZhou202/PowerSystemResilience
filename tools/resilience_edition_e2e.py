#!/usr/bin/env python3
"""HTTP contract test for the fail-closed Resilience edition server.

This script intentionally uses only the Python standard library.  It starts a
fresh server on an ephemeral port, checks edition routing and runtime API v1,
and always tears the child process down.
"""

from __future__ import annotations

import argparse
import http.client
import json
import math
import socket
import subprocess
import sys
import time
import urllib.error
import urllib.parse
import urllib.request
from dataclasses import dataclass
from email.message import Message
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parent.parent
JSON_MEDIA_TYPE = "application/json"
TERMINAL_JOB_STATES = {"succeeded", "failed", "cancelled"}
ACTIVE_JOB_STATES = {"queued", "running", "cancelling"}

PF_REQUEST: dict[str, Any] = {
    "method": "ac_newton",
    "options": {"max_iter": 100, "tol": 1.0e-8},
}
OPF_REQUEST: dict[str, Any] = {
    "solver": "dc",
    "network_model": "balanced_aggregate",
}


@dataclass(frozen=True)
class Response:
    status: int
    headers: Message
    body: bytes
    method: str
    path: str

    def json(self) -> Any:
        if not self.body:
            return None
        try:
            return json.loads(self.body.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as error:
            preview = self.body[:200]
            raise AssertionError(
                f"{self.method} {self.path}: HTTP {self.status} response is not "
                f"valid UTF-8 JSON: {preview!r}"
            ) from error


def read_http_error_body(
    error: urllib.error.HTTPError,
    method: str,
    path: str,
) -> bytes:
    """Read a declared error body without waiting for a reset Windows socket."""
    raw_length = error.headers.get("Content-Length")
    expected_length: int | None = None
    if raw_length is not None:
        try:
            parsed_length = int(raw_length)
            if parsed_length >= 0:
                expected_length = parsed_length
        except ValueError:
            pass

    body = bytearray()
    try:
        if expected_length is None:
            return error.read()
        while len(body) < expected_length:
            try:
                chunk = error.read(expected_length - len(body))
            except http.client.IncompleteRead as read_error:
                body.extend(read_error.partial)
                break
            if not chunk:
                break
            body.extend(chunk)
    except (ConnectionResetError, OSError) as read_error:
        if len(body) != expected_length:
            raise AssertionError(
                f"{method} {path}: HTTP {error.code} error body was reset after "
                f"{len(body)} of {expected_length} declared bytes; "
                f"partial body={bytes(body[:200])!r}"
            ) from read_error
    finally:
        error.close()

    if len(body) != expected_length:
        raise AssertionError(
            f"{method} {path}: HTTP {error.code} error body ended after "
            f"{len(body)} of {expected_length} declared bytes; "
            f"partial body={bytes(body[:200])!r}"
        )
    return bytes(body)


def free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])


def request(
    base_url: str,
    method: str,
    path: str,
    body: dict[str, Any] | None = None,
    headers: dict[str, str] | None = None,
    timeout: float = 30.0,
) -> Response:
    """Send one HTTP request; the method never depends on body presence."""
    data = None if body is None else json.dumps(body).encode("utf-8")
    request_headers = {"Accept": JSON_MEDIA_TYPE}
    if body is not None:
        request_headers["Content-Type"] = JSON_MEDIA_TYPE
    if headers:
        request_headers.update(headers)
    req = urllib.request.Request(
        base_url + path,
        data=data,
        headers=request_headers,
        method=method,
    )
    try:
        with urllib.request.urlopen(req, timeout=timeout) as response:
            return Response(
                response.status, response.headers, response.read(), method, path
            )
    except urllib.error.HTTPError as error:
        error_body = read_http_error_body(error, method, path)
        return Response(error.code, error.headers, error_body, method, path)


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def require_status(response: Response, expected: int, label: str) -> Any:
    payload = response.json()
    require(
        response.status == expected,
        f"{label}: expected HTTP {expected}, got {response.status}: {payload}",
    )
    return payload


def require_json_headers(
    response: Response,
    label: str,
    *,
    cache_control: str | None = None,
) -> None:
    content_type = response.headers.get("Content-Type", "")
    media_type = content_type.split(";", 1)[0].strip().lower()
    require(
        media_type == JSON_MEDIA_TYPE,
        f"{label}: expected JSON Content-Type, got {content_type!r}",
    )
    if cache_control is not None:
        actual = response.headers.get("Cache-Control")
        require(
            actual == cache_control,
            f"{label}: expected Cache-Control {cache_control!r}, got {actual!r}",
        )


def require_location(response: Response, expected: str, label: str) -> None:
    actual = response.headers.get("Location")
    require(actual == expected, f"{label}: expected Location {expected!r}, got {actual!r}")


def v1_path(path: str, **query: Any) -> str:
    """Build a v1 resource path without hand-encoding query values."""
    values = {key: value for key, value in query.items() if value is not None}
    if not values:
        return path
    return f"{path}?{urllib.parse.urlencode(values)}"


def require_flat_error(
    response: Response,
    expected_status: int,
    expected_code: str,
    label: str,
) -> dict[str, Any]:
    payload = require_status(response, expected_status, label)
    require_json_headers(response, label)
    require(isinstance(payload, dict), f"{label}: response is not an object: {payload}")
    require(
        payload.get("schema") == "hysim_error_v1",
        f"{label}: wrong v1 error schema: {payload}",
    )
    require(
        payload.get("error") == expected_code,
        f"{label}: expected flat error {expected_code!r}: {payload}",
    )
    require(
        not isinstance(payload.get("error"), dict),
        f"{label}: v1 error unexpectedly used nested legacy schema: {payload}",
    )
    require(isinstance(payload.get("message"), str), f"{label}: error message is missing")
    return payload


def require_model_headers(
    response: Response,
    session_id: str,
    revision: int,
    label: str,
) -> str:
    etag = response.headers.get("ETag")
    require(isinstance(etag, str) and etag, f"{label}: ETag is missing")
    require(
        response.headers.get("X-HySim-Session-ID") == session_id,
        f"{label}: X-HySim-Session-ID is missing or wrong",
    )
    require(
        response.headers.get("X-HySim-Model-Revision") == str(revision),
        f"{label}: X-HySim-Model-Revision is missing or wrong",
    )
    return etag


def wait_job(
    base_url: str,
    job_id: str,
    label: str,
    *,
    timeout: float = 60.0,
) -> dict[str, Any]:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        response = request(base_url, "GET", f"/api/v1/jobs/{job_id}")
        payload = require_status(response, 200, label)
        require_json_headers(response, label, cache_control="no-store")
        require(
            isinstance(payload, dict) and payload.get("schema") == "hysim_job_v1",
            f"{label}: malformed job resource: {payload}",
        )
        state = payload.get("state")
        if state in TERMINAL_JOB_STATES:
            return payload
        require(state in ACTIVE_JOB_STATES, f"{label}: unknown job state: {payload}")
        time.sleep(0.05)
    raise AssertionError(f"{label}: job did not finish within {timeout:g} seconds")


def submit_job(
    base_url: str,
    session_id: str,
    session_etag: str,
    analysis: str,
    analysis_request: dict[str, Any],
    label: str,
) -> tuple[str, dict[str, Any]]:
    response = request(
        base_url,
        "POST",
        f"/api/v1/sessions/{session_id}/jobs",
        {"analysis": analysis, "request": analysis_request},
        headers={"If-Match": session_etag},
    )
    payload = require_status(response, 202, label)
    require_json_headers(response, label, cache_control="no-store")
    require(isinstance(payload, dict), f"{label}: response is not an object")
    job_id = payload.get("job_id")
    require(isinstance(job_id, str) and job_id, f"{label}: bad job id: {payload}")
    require(payload.get("analysis") == analysis, f"{label}: wrong analysis: {payload}")
    require(payload.get("state") in ACTIVE_JOB_STATES, f"{label}: bad state: {payload}")
    require("result" not in payload, f"{label}: submission inlined a result: {payload}")
    require_location(response, f"/api/v1/jobs/{job_id}", label)
    require(response.headers.get("Retry-After") == "1", f"{label}: Retry-After is missing")
    require_model_headers(response, session_id, 1, label)
    return job_id, payload


def delete_job(base_url: str, job_id: str, label: str) -> None:
    response = request(base_url, "DELETE", f"/api/v1/jobs/{job_id}")
    require_status(response, 204, label)
    require(response.body == b"", f"{label}: 204 response has a body")


def delete_session(
    base_url: str,
    session_id: str,
    session_etag: str,
    label: str,
) -> None:
    response = request(
        base_url,
        "DELETE",
        f"/api/v1/sessions/{session_id}",
        headers={"If-Match": session_etag},
    )
    require_status(response, 204, label)
    require(response.body == b"", f"{label}: 204 response has a body")


def wait_ready(base_url: str, timeout: float = 30.0) -> dict[str, Any]:
    """Wait for, and positively identify, a Resilience edition server."""
    deadline = time.monotonic() + timeout
    last_observation = "no response"
    while time.monotonic() < deadline:
        try:
            response = request(base_url, "GET", "/api/edition", timeout=1.0)
            payload = response.json()
            last_observation = f"HTTP {response.status}: {payload}"
            if (
                response.status == 200
                and isinstance(payload, dict)
                and payload.get("edition") == "resilience"
            ):
                require_json_headers(
                    response, "GET /api/edition readiness", cache_control="no-store"
                )
                return payload
            if response.status == 200 and isinstance(payload, dict):
                edition = payload.get("edition")
                if edition is not None and edition != "resilience":
                    raise AssertionError(
                        f"server is edition {edition!r}, expected 'resilience'"
                    )
        except (OSError, urllib.error.URLError):
            pass
        time.sleep(0.1)
    raise AssertionError(
        "Resilience server did not become ready via /api/edition; "
        f"last observation: {last_observation}"
    )


def expect_nested_disabled(
    base_url: str, method: str, path: str, feature: str
) -> None:
    response = request(base_url, method, path)
    payload = response.json()
    require_json_headers(response, f"{method} {path}", cache_control="no-store")
    require(
        response.status == 403,
        f"{method} {path}: expected edition HTTP 403, got {response.status}: {payload}",
    )
    require(isinstance(payload, dict), f"{method} {path}: response is not an object")
    error = payload.get("error")
    require(
        isinstance(error, dict),
        f"{method} {path}: expected nested edition error object, got {payload}",
    )
    require(
        error.get("code") == "EDITION_FEATURE_DISABLED",
        f"{method} {path}: wrong edition error code: {payload}",
    )
    require(
        error.get("edition") == "resilience",
        f"{method} {path}: wrong edition in error: {payload}",
    )
    require(
        error.get("feature") == feature,
        f"{method} {path}: feature {error.get('feature')!r}, expected {feature!r}",
    )


def expect_handler_reached(base_url: str, path: str) -> int:
    """Prove a retained route got beyond edition pre-routing into its handler."""
    response = request(base_url, "POST", path, {})
    payload = response.json()
    require_json_headers(response, f"POST {path}")
    require(
        200 <= response.status < 500,
        f"POST {path}: handler returned unexpected HTTP {response.status}: {payload}",
    )
    nested_error = payload.get("error") if isinstance(payload, dict) else None
    require(
        not (
            response.status == 403
            and isinstance(nested_error, dict)
            and nested_error.get("code") == "EDITION_FEATURE_DISABLED"
        ),
        f"POST {path}: retained Resilience route was blocked by edition guard: {payload}",
    )
    return response.status


def assert_result_contract(
    result: dict[str, Any],
    analysis: str,
    session_id: str,
    job_id: str,
    *,
    model_revision: int,
    current_model_revision: int,
    stale: bool,
) -> dict[str, Any]:
    contract = result.get("_result_contract")
    require(isinstance(contract, dict), f"{analysis} _result_contract is missing")
    require(contract.get("schema") == "hysim_result_v1", f"bad result contract: {contract}")
    require(contract.get("analysis") == analysis, f"bad result analysis: {contract}")
    require(contract.get("request_id") == job_id, f"result job id mismatch: {contract}")
    require(contract.get("session_id") == session_id, f"result session mismatch: {contract}")
    require(contract.get("model_revision") == model_revision, f"wrong result revision: {contract}")
    require(
        contract.get("current_model_revision") == current_model_revision,
        f"wrong current result revision: {contract}",
    )
    require(contract.get("stale") is stale, f"wrong stale marker: {contract}")
    require(
        contract.get("status") == ("stale" if stale else "completed"),
        f"wrong result status: {contract}",
    )
    require(isinstance(contract.get("received_at"), str), f"missing received_at: {contract}")
    return contract


def assert_scientific_power_flow(result: Any) -> None:
    require(isinstance(result, dict), "power_flow job result is not an object")
    require(result.get("schema") == "power_flow_result_v1", f"wrong PF schema: {result}")
    require(result.get("converged") is True, f"power_flow did not converge: {result}")
    require(isinstance(result.get("iterations"), int), "PF iterations is not an integer")
    residual = result.get("residual")
    require(
        isinstance(residual, (int, float))
        and not isinstance(residual, bool)
        and math.isfinite(float(residual)),
        f"PF residual is not finite: {residual!r}",
    )
    require(
        isinstance(result.get("vm"), list) and len(result["vm"]) > 0,
        "PF result lacks voltage-magnitude vector",
    )
    require(
        all(
            isinstance(value, (int, float))
            and not isinstance(value, bool)
            and math.isfinite(float(value))
            for value in result["vm"]
        ),
        "PF voltage-magnitude vector contains non-finite values",
    )
    require(
        isinstance(result.get("va"), list) and len(result["va"]) == len(result["vm"]),
        "PF voltage angle/magnitude vectors are inconsistent",
    )
    require(
        all(
            isinstance(value, (int, float))
            and not isinstance(value, bool)
            and math.isfinite(float(value))
            for value in result["va"]
        ),
        "PF voltage-angle vector contains non-finite values",
    )
    ac_rows = result.get("ac_bus_results")
    require(isinstance(ac_rows, list) and ac_rows, "PF result lacks AC bus rows")
    require(
        all(
            isinstance(row, dict)
            and row.get("domain") == "ac"
            and isinstance(row.get("index"), int)
            and not isinstance(row.get("index"), bool)
            and isinstance(row.get("vm_pu"), (int, float))
            and not isinstance(row.get("vm_pu"), bool)
            and math.isfinite(float(row["vm_pu"]))
            and isinstance(row.get("va_rad"), (int, float))
            and not isinstance(row.get("va_rad"), bool)
            and math.isfinite(float(row["va_rad"]))
            for row in ac_rows
        ),
        "PF AC bus rows do not carry finite, domain-qualified scientific values",
    )
    validity = result.get("validity_flags")
    require(isinstance(validity, dict) and validity, "PF validity_flags is missing")
    require(isinstance(result.get("model_scope"), str), "PF model_scope is missing")


def assert_scientific_opf(result: Any) -> None:
    require(isinstance(result, dict), "optimal_power_flow job result is not an object")
    require(
        result.get("schema") == "optimal_power_flow_result_v1",
        f"wrong OPF schema: {result}",
    )
    require(result.get("solver") == "dc", f"wrong OPF solver: {result}")
    require(
        result.get("network_model") == "dc_approximation",
        f"wrong OPF network model: {result}",
    )
    require(result.get("converged") is True, f"optimal_power_flow did not converge: {result}")
    require(isinstance(result.get("iterations"), int), "OPF iterations is not an integer")
    objective = result.get("objective")
    require(
        isinstance(objective, (int, float))
        and not isinstance(objective, bool)
        and math.isfinite(float(objective)),
        f"OPF objective is not finite: {objective!r}",
    )
    require(
        isinstance(result.get("va"), list) and result["va"],
        "OPF result lacks voltage-angle vector",
    )
    require(
        all(
            isinstance(value, (int, float))
            and not isinstance(value, bool)
            and math.isfinite(float(value))
            for value in result["va"]
        ),
        "OPF voltage-angle vector contains non-finite values",
    )
    require(
        isinstance(result.get("pg_mw"), list) and result["pg_mw"],
        "OPF result lacks generator-dispatch vector",
    )
    require(
        all(
            isinstance(value, (int, float))
            and not isinstance(value, bool)
            and math.isfinite(float(value))
            for value in result["pg_mw"]
        ),
        "OPF generator-dispatch vector contains non-finite values",
    )
    ac_rows = result.get("ac_bus_results")
    require(isinstance(ac_rows, list) and ac_rows, "OPF result lacks AC bus rows")
    require(
        all(
            isinstance(row, dict)
            and row.get("domain") == "ac"
            and isinstance(row.get("index"), int)
            and not isinstance(row.get("index"), bool)
            and isinstance(row.get("va_rad"), (int, float))
            and not isinstance(row.get("va_rad"), bool)
            and math.isfinite(float(row["va_rad"]))
            for row in ac_rows
        ),
        "OPF AC bus rows do not carry finite, domain-qualified angles",
    )
    dispatch = result.get("generator_dispatch")
    require(isinstance(dispatch, list) and dispatch, "OPF result lacks generator dispatch rows")
    require(
        all(
            isinstance(row, dict)
            and row.get("domain") == "ac"
            and isinstance(row.get("index"), int)
            and not isinstance(row.get("index"), bool)
            and isinstance(row.get("pg_mw"), (int, float))
            and not isinstance(row.get("pg_mw"), bool)
            and math.isfinite(float(row["pg_mw"]))
            for row in dispatch
        ),
        "OPF dispatch rows do not carry finite, domain-qualified scientific values",
    )
    validity = result.get("validity_flags")
    require(isinstance(validity, dict) and validity, "OPF validity_flags is missing")
    require(isinstance(result.get("model_scope"), str), "OPF model_scope is missing")


def assert_topology(topology: Any, session_id: str, revision: int) -> dict[str, Any]:
    require(isinstance(topology, dict), "topology response is not an object")
    require(topology.get("schema") == "hysim_topology_chunk_v1", f"bad topology: {topology}")
    require(topology.get("session_id") == session_id, f"topology session mismatch: {topology}")
    require(topology.get("model_revision") == revision, f"topology revision mismatch: {topology}")
    require(topology.get("lod") == 2, f"topology did not retain LOD2: {topology}")
    nodes = topology.get("nodes")
    require(isinstance(nodes, list) and nodes, f"topology has no nodes: {topology}")
    require(
        all(
            isinstance(node, dict)
            and isinstance(node.get("ref"), dict)
            and node["ref"].get("domain") in {"ac", "dc"}
            and isinstance(node["ref"].get("index"), int)
            and not isinstance(node["ref"].get("index"), bool)
            for node in nodes
        ),
        "topology nodes do not use domain-qualified stable references",
    )
    require(
        isinstance(topology.get("returned_nodes"), int)
        and topology["returned_nodes"] == len(nodes)
        and topology["returned_nodes"] <= topology.get("limit", -1),
        f"topology paging metadata is inconsistent: {topology}",
    )
    return nodes[0]["ref"]


def assert_frame(frame: Any, job_id: str, session_id: str, revision: int) -> None:
    require(isinstance(frame, dict), "result frame is not an object")
    require(
        frame.get("schema") == "hysim_result_frame_chunk_v1",
        f"bad frame schema: {frame}",
    )
    require(frame.get("job_id") == job_id, f"frame job mismatch: {frame}")
    require(frame.get("session_id") == session_id, f"frame session mismatch: {frame}")
    require(frame.get("model_revision") == revision, f"frame revision mismatch: {frame}")
    require(frame.get("step") == 0 and frame.get("domain") == "ac", f"bad frame: {frame}")
    nodes = frame.get("nodes")
    require(isinstance(nodes, list), f"frame nodes is not an array: {frame}")
    require(
        frame.get("returned_nodes") == len(nodes) <= 3,
        f"result frame did not honor the requested spatial limit: {frame}",
    )
    require(
        all(
            isinstance(row, dict)
            and row.get("domain") == "ac"
            and isinstance(row.get("index"), int)
            and not isinstance(row.get("index"), bool)
            and isinstance(row.get("vm_pu"), (int, float))
            and not isinstance(row.get("vm_pu"), bool)
            and math.isfinite(float(row["vm_pu"]))
            for row in nodes
        ),
        f"frame rows lack finite authored AC values: {frame}",
    )


def assert_violations(
    violations: Any, job_id: str, session_id: str, revision: int
) -> None:
    require(isinstance(violations, dict), "violation chunk is not an object")
    require(
        violations.get("schema") == "hysim_result_violation_chunk_v1",
        f"bad violation schema: {violations}",
    )
    require(violations.get("job_id") == job_id, f"violation job mismatch: {violations}")
    require(
        violations.get("session_id") == session_id,
        f"violation session mismatch: {violations}",
    )
    require(
        violations.get("model_revision") == revision,
        f"violation revision mismatch: {violations}",
    )
    items = violations.get("items")
    require(isinstance(items, list), f"violation items is not an array: {violations}")
    require(
        violations.get("returned") == len(items) <= 5,
        f"violation chunk did not honor its limit: {violations}",
    )
    require(
        all(
            isinstance(item, dict)
            and item.get("kind") in {"undervoltage", "overvoltage", "overload"}
            and isinstance(item.get("ref"), dict)
            and isinstance(item.get("value"), (int, float))
            and not isinstance(item.get("value"), bool)
            and math.isfinite(float(item["value"]))
            and isinstance(item.get("severity"), (int, float))
            and not isinstance(item.get("severity"), bool)
            and math.isfinite(float(item["severity"]))
            for item in items
        ),
        f"violation rows lack stable scientific values: {violations}",
    )


def expect_flat_v1_analysis_error(
    base_url: str,
    session_id: str,
    session_etag: str,
    analysis: str,
    expected_status: int,
    expected_code: str,
    label: str,
) -> dict[str, Any]:
    response = request(
        base_url,
        "POST",
        f"/api/v1/sessions/{session_id}/jobs",
        {"analysis": analysis, "request": {}},
        headers={"If-Match": session_etag, "Connection": "close"},
    )
    return require_flat_error(response, expected_status, expected_code, label)


def verify_recovery_metrics(base_url: str) -> None:
    """Compare the evaluator against independently integrated real solver steps."""
    require_status(request(base_url, "POST", "/api/session/load_builtin", {"case": "dist33_microgrid_der"}), 200, "load metric case")
    payload = {"model": "HeuristicSequential", "horizon_hours": 4, "time_step_hr": 0.5,
               "default_fault_count": 1, "auto_fault_start_hr": 0.5, "repair_time_hr": 1.0,
               "allow_mess_dispatch": False, "run_power_flow": False}
    artifact = require_status(request(base_url, "POST", "/api/session/run_distribution_resilience", payload), 200, "metric recovery")
    require(artifact.get("feasible") and artifact.get("completed"), f"metric run infeasible: {artifact.get('status')}")
    faults = artifact["fault_sequence"]
    require(faults, "expected automatic fault")
    require(artifact["disaster_end_hr"] == max(row["start_hr"] for row in faults), "last fault onset default mismatch")
    require(artifact["disaster_end_source"] == "last_fault_start_default", "default provenance missing")
    require(artifact["last_repair_completion_hr"] > artifact["disaster_end_hr"], "repair end was confused with disaster end")
    catalog = require_status(request(base_url, "GET", "/api/session/resilience/metric_catalog"), 200, "metric catalog")
    query = {"run_id": artifact["run_id"], "selection_revision": 7,
             "selected_metric_ids": [row["id"] for row in catalog["entries"]]}
    output = require_status(request(base_url, "POST", "/api/session/resilience/metrics", query), 200, "evaluate real metrics")
    rows = {row["id"]: row for row in output["results"]}
    require(len(rows) == 42, "incomplete evaluator result")
    for key, source in [("ens", "shed_mw"), ("served_energy", "served_mw"), ("demand_energy", "demand_mw"), ("weighted_ens", "weighted_shed_mw")]:
        expected = sum(step[source] * step["duration_hr"] for step in artifact["steps"])
        row = rows["run." + key]
        require(row["status"] == "computed" and math.isclose(row["value"], expected, abs_tol=1e-7), f"integration mismatch: {key}: {row}, expected={expected}")
    require(math.isclose(rows["run.ens"]["value"], artifact["total_shed_mwh"], abs_tol=1e-7), "ENS differs from solver")
    for key in ["critical_ens", "high_ens", "medium_ens", "low_ens"]:
        require(rows["run." + key]["status"] == "computed", f"priority metric unavailable: {key}")
    require(math.isclose(sum(rows["run." + key]["value"] for key in ["critical_ens", "high_ens", "medium_ens", "low_ens"]), rows["run.ens"]["value"], abs_tol=1e-6), "priority energy does not sum to total")
    require(rows["ch3.lolp"]["value"] is None, "single run fabricated probability")
    require(rows["ch3.apda"]["value"] is None, "proxy lacks consent")
    query["allow_apda_system_gap_approximation"] = True
    proxy = require_status(request(base_url, "POST", "/api/session/resilience/metrics", query), 200, "explicit APDA proxy")
    require(next(r for r in proxy["results"] if r["id"] == "ch3.apda")["status"] == "approximate", "proxy consent ineffective")
    query["parameters"] = {"event_window": {"disaster_end_hr": 2, "recovery_start_hr": 1}}
    require_status(request(base_url, "POST", "/api/session/resilience/metrics", query), 400, "invalid event order")
    payload["disaster_end_hr"] = 0.75
    explicit = require_status(request(base_url, "POST", "/api/session/run_distribution_resilience", payload), 200, "explicit disaster end")
    require(explicit["disaster_end_hr"] == 0.75 and explicit["disaster_end_source"] == "explicit_request", "explicit disaster end did not take precedence")
    print("Real recovery metrics: 42 rows; energy, priority balance, event defaults and explicit override passed")


def verify_weather_scenarios(base_url: str) -> None:
    """Exercise real generated faults, profiles, recovery and metric evaluation."""
    require_status(request(base_url, "POST", "/api/session/load_builtin", {"case": "dist33_weather_mixed"}), 200, "weather case")
    profile = require_status(request(base_url, "GET", "/api/edition"), 200, "weather schema")
    schemas = {row["id"]: row for row in profile["scenario_hazards"]}
    require(set(schemas) == {"typhoon", "rainstorm", "lightning"}, "hazard catalog mismatch")
    for hazard in ["rainstorm", "lightning"]:
        parameters = ({"total_mm": 500, "severity_variation": 0}
                      if hazard == "rainstorm" else
                      {"density_km2_hr": 1000, "permanent_fraction": 0.5, "severity_variation": 0})
        payload = {"regular": {"enabled": False}, "reliability": {"enabled": False},
                   "resilience": {"hazard_type": hazard, hazard: parameters,
                                  "candidates_per_intensity": 4, "default_cluster_count": 2}}
        data = require_status(request(base_url, "POST", "/api/session/generate_scenarios", payload), 200, "generate " + hazard)
        group = data["resilience"]["intensities"][0]
        require(group["hazard_type"] == hazard and group["intensity"] == hazard, "hazard identity lost")
        require(math.isclose(sum(c["probability"] for c in group["clusters"]), 1), "conditional weights invalid")
        representative = max(group["clusters"], key=lambda c: len(c["representative"]["resilience_event"]["faults"]))["representative"]
        event = representative["resilience_event"]
        require(event["faults"], "severe weather test must generate faults")
        evidence = event["hazard_evidence"]
        affected = evidence["affected_equipment"]
        require(affected["fault_count"] == len(event["faults"]), "weather fault count differs from modeled outages")
        if hazard == "lightning":
            require(all(f["equipment_type"] == "overhead_line" and f["branch_type"] == "AC"
                        for f in event["faults"]), "lightning reached cable or non-overhead asset")
        else:
            require(all(f["equipment_type"] in {"cable_accessory", "insulator", "transformer_2w"}
                        for f in event["faults"]), "rainstorm used an unmodeled asset class")
        require(set(evidence["parameters"]) == {f["key"] for f in schemas[hazard]["fields"]}, "effective parameter schema incomplete")
        for key, value in parameters.items():
            require(evidence["parameters"][key] == value, "weather override ignored: " + key)
        require(event["selected_track_max_vmax_ms"] is None and not event["track"], "weather fabricated typhoon track")
        require(evidence["model_limitations"], "weather coverage missing")
        ts = representative["standard_time_series"]
        profiles = {str(p["id"]): p["values"] for p in ts["profiles"]}
        recovery = {"model": "RAStyleStageMILP", "mip_solver": "HiGHS", "mip_time_limit_s": 10,
                    "respect_fault_windows": True,
                    "horizon_hours": 48, "time_step_hr": 1,
                    "default_fault_count": 0, "manual_faults": event["faults"],
                    "allow_mess_dispatch": False, "run_power_flow": False}
        for name in ["load", "pv", "wind", "renewable"]:
            key = ts["binding"].get("resilience_" + name + "_profile_id")
            if key is not None:
                recovery[name + "_profile"] = profiles[str(key)]
        artifact = require_status(request(base_url, "POST", "/api/session/run_distribution_resilience", recovery), 200, "recover " + hazard)
        require(artifact["completed"] and artifact["feasible"], "weather recovery not feasible")
        require(len(artifact["fault_sequence"]) == len(event["faults"]), "generated faults not consumed")
        require(artifact["disaster_end_hr"] == max(f["start_hr"] for f in event["faults"]), "weather changed disaster end convention")
        by_branch = {(f["branch_type"], f["branch_index"]): f for f in artifact["fault_sequence"]}
        for fault in event["faults"]:
            actual = by_branch[(fault["branch_type"], fault["branch_index"])]
            require(actual["start_hr"] == fault["start_hr"] and actual["repair_duration_hr"] == fault["repair_duration_hr"], f"weather outage window changed: generated={fault}, recovered={actual}")
            require(actual["equipment_type"] == fault["equipment_type"] and actual["equipment_index"] == fault["equipment_index"], "weather asset attribution lost in recovery")
        metrics = require_status(request(base_url, "POST", "/api/session/resilience/metrics", {
            "run_id": artifact["run_id"], "selection_revision": 1, "selected_metric_ids": ["run.ens", "run.served_energy"]}), 200, "weather metrics")
        ens = next(row for row in metrics["results"] if row["id"] == "run.ens")
        require(ens["status"] == "computed" and math.isclose(ens["value"], artifact["total_shed_mwh"], abs_tol=1e-6), "weather ENS mismatch")
        invalid = {**payload, "resilience": {**payload["resilience"], hazard: {"unknown": 1}}}
        require_status(request(base_url, "POST", "/api/session/generate_scenarios", invalid), 400, "unknown hazard parameter")
        invalid["resilience"][hazard] = {"duration_hr": 48, "start_hr": 1}
        require_status(request(base_url, "POST", "/api/session/generate_scenarios", invalid), 400, "event outside observation window")
        print(f"Weather {hazard}: {len(event['faults'])} faults -> real recovery -> ENS {ens['value']:.6f} MWh")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Exercise the Resilience edition HTTP and runtime v1 contracts."
    )
    parser.add_argument("--server", required=True, help="path to run_gui_server")
    parser.add_argument("--data-dir", default=str(ROOT / "data"))
    args = parser.parse_args()

    server = Path(args.server).resolve()
    require(server.is_file(), f"server binary not found: {server}")

    port = free_port()
    base_url = f"http://127.0.0.1:{port}"
    process = subprocess.Popen(
        [
            str(server),
            "--host",
            "127.0.0.1",
            "--port",
            str(port),
            "--api-job-workers",
            "1",
            "--data-dir",
            args.data_dir,
        ],
        cwd=ROOT,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )

    session_id: str | None = None
    session_etag: str | None = None
    auxiliary_sessions: dict[str, str] = {}
    cleanup_job_ids: list[str] = []
    handler_statuses: dict[str, int] = {}
    disabled_count = 0
    try:
        profile = wait_ready(base_url)
        require(profile.get("schema") == "hacdcpf.edition-profile.v1", f"bad profile: {profile}")
        require(profile.get("edition") == "resilience", f"bad edition profile: {profile}")
        require(
            profile.get("route_policy") == {
                "mode": "fail_closed",
                "unknown_api_routes": "disabled",
            },
            f"Resilience route policy is not fail-closed: {profile.get('route_policy')}",
        )
        require(
            profile.get("enabled_io_formats") == ["json", "matpower"],
            f"unexpected Resilience I/O formats: {profile.get('enabled_io_formats')}",
        )
        require(
            all(row.get("dimension") != "carbon" for row in profile.get("indicators", [])),
            "Resilience edition unexpectedly advertises a carbon indicator",
        )

        require(
            profile.get("enabled_modules")
            == [
                "model",
                "model_io",
                "parameter_validation",
                "projection",
                "attribution",
                "graph",
                "topology",
                "power_flow",
                "optimal_power_flow",
                "reliability",
                "resilience",
                "network_reconfiguration",
                "scenario_generation",
                "typhoon_faults",
                "short_circuit",
                "resilience_profiles",
                "mipsolvers",
            ],
            f"unexpected Resilience enabled modules: {profile.get('enabled_modules')}",
        )
        require(
            profile.get("frontend_modules")
            == [
                "modelIO",
                "parameterLibrary",
                "topologyAnalysis",
                "scenarioGeneration",
                "powerFlow",
                "opf",
                "resilience",
                "proactiveDefense",
                "rapidRecovery",
                "resilienceMetrics",
            ],
            f"unexpected Resilience frontend modules: {profile.get('frontend_modules')}",
        )
        require(
            [step.get("id") for step in profile.get("workflow", [])]
            == [
                "metric_selection",
                "scenario_selection",
                "proactive_defense",
                "rapid_recovery",
                "metric_output",
            ],
            f"unexpected Resilience workflow: {profile.get('workflow')}",
        )
        metric_catalog = profile.get("resilience_metric_catalog")
        require(
            isinstance(metric_catalog, dict)
            and metric_catalog.get("schema") == "resilience_metric_catalog_v1"
            and metric_catalog.get("definition_version") == "book_ch3_2026.2"
            and len(metric_catalog.get("entries", [])) == 42,
            f"unexpected Resilience metric catalog: {metric_catalog}",
        )
        require(
            profile.get("disabled_features")
            == [
                "reactive_power_optimization",
                "harmonics",
                "dynamics",
                "market",
                "carbon",
                "integrated_energy",
                "ev_traffic",
                "time_series",
                "hosting_capacity",
                "weak_links",
                "counterfactual_planning",
                "sppt_agent",
                "advanced_io",
            ],
            f"unexpected Resilience disabled features: {profile.get('disabled_features')}",
        )
        require(
            profile.get("solver_capabilities")
            == [
                "ac_power_flow",
                "dc_optimal_power_flow",
                "ac_optimal_power_flow",
                "highs",
                "native_branch_and_cut",
                "aml",
            ],
            f"unexpected Resilience solver capabilities: {profile.get('solver_capabilities')}",
        )
        require(
            isinstance(profile.get("model_scope"), str) and profile["model_scope"],
            f"Resilience model_scope is missing: {profile}",
        )
        require(
            isinstance(profile.get("limitations"), list) and profile["limitations"],
            f"Resilience limitations are missing: {profile}",
        )
        require(
            profile.get("restoration_certification")
            == {
                "ordinary_feasibility_is_certified_safe": False,
                "dynamic_certification": "not_exposed_in_first_release",
            },
            f"unexpected restoration certification: {profile.get('restoration_certification')}",
        )
        catalog = profile.get("analysis_catalog")
        require(
            isinstance(catalog, dict)
            and catalog.get("schema") == "hacdcpf.edition-analysis-catalog.v1"
            and isinstance(catalog.get("entries"), list),
            f"Resilience analysis catalog is malformed: {catalog}",
        )
        catalog_entries = {
            row.get("name"): row
            for row in catalog["entries"]
            if isinstance(row, dict) and isinstance(row.get("name"), str)
        }
        require(
            catalog_entries.get("carbon_flow", {}).get("enabled") is False,
            f"carbon_flow is not catalogued as known-disabled: {catalog}",
        )
        require(
            catalog_entries.get("reliability", {}).get("enabled") is True
            and catalog_entries.get("distribution_resilience", {}).get("enabled")
            is True,
            f"retained Resilience analyses are absent from catalog: {catalog}",
        )

        discovery_response = request(base_url, "GET", "/api/v1")
        discovery = require_status(discovery_response, 200, "GET /api/v1")
        require_json_headers(discovery_response, "GET /api/v1")
        require(discovery.get("schema") == "hysim_api_v1", f"bad v1 discovery: {discovery}")
        require(
            set(discovery.get("analyses", [])) == {"power_flow", "optimal_power_flow"},
            f"unexpected v1 analyses: {discovery.get('analyses')}",
        )
        required_features = {
            "multi_session",
            "model_revision",
            "etag",
            "asynchronous_jobs",
            "topology_lod",
            "spatial_chunks",
            "time_chunks",
        }
        require(
            required_features.issubset(set(discovery.get("features", []))),
            f"v1 discovery features are incomplete: {discovery}",
        )

        retained = (
            ("GET", "/api/cases"),
            ("GET", "/api/session/status"),
            ("GET", "/api/opf/parameter_contract"),
            ("GET", "/api/session/parameter_library"),
        )
        for method, path in retained:
            response = request(base_url, method, path)
            require_status(response, 200, f"{method} {path}")
            require_json_headers(response, f"{method} {path}")

        handler_statuses["network_reduction"] = expect_handler_reached(
            base_url, "/api/session/network_reduction"
        )
        handler_statuses["run_reconfig"] = expect_handler_reached(
            base_url, "/api/session/run_reconfig"
        )

        disabled = (
            ("POST", "/api/session/run_market_clearing", "market"),
            ("POST", "/api/session/market_ptdf", "market"),
            ("POST", "/api/session/update_carbon_factors", "carbon"),
            ("POST", "/api/session/run_carbon", "carbon"),
            ("POST", "/api/session/run_hosting_capacity", "hosting_capacity"),
            ("POST", "/api/session/run_multidimensional_weak_links", "weak_links"),
            ("GET", "/api/dynamics/model_schema", "dynamics"),
            ("POST", "/api/session/run_transient", "dynamics"),
            ("POST", "/api/session/load_bpa_dat", "advanced_io"),
            ("POST", "/api/session/load_cim_dist", "advanced_io"),
            ("POST", "/api/session/export_etap", "advanced_io"),
            ("POST", "/api/session/load_opendss", "advanced_io"),
            ("POST", "/api/session/harmonics", "harmonics"),
            ("POST", "/api/session/run_rpo", "reactive_power_optimization"),
            ("POST", "/api/session/run_ts_pf", "time_series"),
            ("POST", "/api/session/run_counterfactual_planning", "counterfactual_planning"),
            ("POST", "/api/session/run_campus_ies", "integrated_energy"),
        )
        for method, path, feature in disabled:
            expect_nested_disabled(base_url, method, path, feature)
            disabled_count += 1
        # Unknown API paths still exercise the legacy nested edition guard.  A
        # GET avoids a cpp-httplib/Windows reset observed when an unread POST
        # body is rejected before route dispatch.
        expect_nested_disabled(
            base_url, "GET", "/api/session/not_registered_in_profile", "unclassified_api"
        )
        disabled_count += 1
        expect_nested_disabled(
            base_url, "GET", "/api/v1/specialist_solver", "unclassified_api"
        )
        disabled_count += 1

        plan_response = request(
            base_url,
            "POST",
            "/api/edition/analysis_plan",
            {"indicators": ["system_economic", "system_reliability", "user_resilience"]},
        )
        plan = require_status(plan_response, 200, "valid analysis plan")
        require_json_headers(plan_response, "valid analysis plan", cache_control="no-store")
        modules = [step.get("id") for step in plan.get("steps", [])]
        require(
            modules
            == [
                "metric_selection",
                "scenario_selection",
                "proactive_defense",
                "rapid_recovery",
                "metric_output",
            ],
            f"unexpected Resilience analysis plan: {plan}",
        )
        require(plan.get("automatic_execution") is False, f"bad plan execution mode: {plan}")

        catalog_response = request(
            base_url, "GET", "/api/session/resilience/metric_catalog"
        )
        catalog = require_status(catalog_response, 200, "resilience metric catalog")
        require_json_headers(
            catalog_response, "resilience metric catalog", cache_control="no-store"
        )
        require(
            catalog.get("schema") == "resilience_metric_catalog_v1"
            and catalog.get("definition_version") == "book_ch3_2026.2"
            and len(catalog.get("entries", [])) == 42,
            f"malformed resilience metric catalog: {catalog}",
        )
        catalog_ids = [entry.get("id") for entry in catalog["entries"]]
        require(
            len(catalog_ids) == len(set(catalog_ids)) == 42,
            f"metric catalog IDs are not unique: {catalog_ids}",
        )

        unknown_run_response = request(
            base_url,
            "POST",
            "/api/session/resilience/metrics",
            {
                "schema": "resilience_metric_request_v1",
                "run_id": "rrun-not-present",
                "selection_revision": 1,
                "selected_metric_ids": ["ch3.cllp"],
            },
        )
        unknown_run = require_status(
            unknown_run_response, 404, "unknown resilience run"
        )
        require(
            isinstance(unknown_run.get("error"), dict)
            and unknown_run["error"].get("code") == "UNKNOWN_RESILIENCE_RUN",
            f"unexpected unknown-run error: {unknown_run}",
        )

        verify_recovery_metrics(base_url)
        verify_weather_scenarios(base_url)

        carbon_plan_response = request(
            base_url,
            "POST",
            "/api/edition/analysis_plan",
            {"indicators": ["user_carbon"]},
        )
        carbon_plan = require_status(carbon_plan_response, 400, "carbon analysis plan")
        require_json_headers(carbon_plan_response, "carbon analysis plan")
        carbon_error = carbon_plan.get("error") if isinstance(carbon_plan, dict) else None
        require(
            isinstance(carbon_error, dict)
            and carbon_error.get("code") == "INVALID_EDITION_PLAN",
            f"analysis-plan error must use nested INVALID_EDITION_PLAN schema: {carbon_plan}",
        )
        require(
            "not enabled in the resilience edition" in carbon_error.get("message", ""),
            f"carbon plan error lacks edition explanation: {carbon_plan}",
        )

        create_response = request(
            base_url, "POST", "/api/v1/sessions", {"case": "ieee14_acdc"}
        )
        session = require_status(create_response, 201, "create v1 session")
        require_json_headers(create_response, "create v1 session")
        session_id = session.get("session_id")
        require(isinstance(session_id, str) and session_id, f"bad session id: {session}")
        require(session.get("schema") == "hysim_session_v1", f"bad session schema: {session}")
        require(session.get("has_model") is True, f"session model was not loaded: {session}")
        require(session.get("model_revision") == 1, f"bad model revision: {session}")
        session_etag = require_model_headers(create_response, session_id, 1, "create session")
        require(session.get("etag") == session_etag, f"body/header ETag mismatch: {session}")
        require_location(create_response, f"/api/v1/sessions/{session_id}", "create session")

        session_response = request(base_url, "GET", f"/api/v1/sessions/{session_id}")
        session_resource = require_status(session_response, 200, "GET v1 session")
        require_json_headers(session_response, "GET v1 session")
        require(session_resource == session, f"GET session differs from create response: {session_resource}")
        require_model_headers(session_response, session_id, 1, "GET v1 session")

        model_response = request(
            base_url, "GET", f"/api/v1/sessions/{session_id}/model"
        )
        model_resource = require_status(model_response, 200, "GET v1 model")
        require_json_headers(model_response, "GET v1 model")
        require(
            isinstance(model_resource, dict)
            and model_resource.get("schema") == "hysim_model_resource_v1"
            and model_resource.get("session_id") == session_id
            and model_resource.get("model_revision") == 1
            and isinstance(model_resource.get("model"), dict),
            f"malformed model resource: {model_resource}",
        )
        require_model_headers(model_response, session_id, 1, "GET v1 model")
        authored_model = model_resource["model"]

        missing_match_response = request(
            base_url,
            "PUT",
            f"/api/v1/sessions/{session_id}/model",
            {"model": authored_model},
        )
        missing_match = require_flat_error(
            missing_match_response,
            428,
            "precondition_required",
            "PUT model without If-Match",
        )
        require(
            isinstance(missing_match.get("details"), dict)
            and missing_match["details"].get("current_etag") == session_etag
            and missing_match["details"].get("current_model_revision") == 1,
            f"missing If-Match details are incomplete: {missing_match}",
        )
        require_model_headers(
            missing_match_response, session_id, 1, "PUT model without If-Match"
        )

        known_disabled_job = expect_flat_v1_analysis_error(
            base_url,
            session_id,
            session_etag,
            "carbon_flow",
            403,
            "edition_feature_disabled",
            "known-disabled v1 analysis",
        )
        require(
            isinstance(known_disabled_job.get("details"), dict)
            and known_disabled_job["details"].get("edition") == "resilience"
            and known_disabled_job["details"].get("feature") == "carbon_flow",
            f"known-disabled-analysis details are incomplete: {known_disabled_job}",
        )

        unknown_job = expect_flat_v1_analysis_error(
            base_url,
            session_id,
            session_etag,
            "specialist_solver",
            400,
            "unsupported_analysis",
            "unknown v1 analysis",
        )
        require(
            isinstance(unknown_job.get("details"), dict)
            and set(unknown_job["details"].get("supported_analyses", []))
            == {"power_flow", "optimal_power_flow"},
            f"unsupported-analysis details are incomplete: {unknown_job}",
        )

        topology_response = request(
            base_url,
            "GET",
            v1_path(
                f"/api/v1/sessions/{session_id}/topology",
                lod=2,
                offset=0,
                limit=10_000,
            ),
        )
        topology = require_status(topology_response, 200, "GET topology chunk")
        require_json_headers(topology_response, "GET topology chunk")
        require_model_headers(topology_response, session_id, 1, "GET topology chunk")
        first_ref = assert_topology(topology, session_id, 1)

        subgraph_response = request(
            base_url,
            "GET",
            v1_path(
                f"/api/v1/sessions/{session_id}/subgraph",
                domain=first_ref["domain"],
                index=first_ref["index"],
                depth=1,
            ),
        )
        subgraph = require_status(subgraph_response, 200, "GET topology subgraph")
        require_json_headers(subgraph_response, "GET topology subgraph")
        require_model_headers(subgraph_response, session_id, 1, "GET topology subgraph")
        require(
            isinstance(subgraph, dict)
            and subgraph.get("schema") == "hysim_subgraph_v1"
            and subgraph.get("session_id") == session_id
            and subgraph.get("model_revision") == 1
            and subgraph.get("center") == first_ref
            and isinstance(subgraph.get("nodes"), list)
            and subgraph["nodes"],
            f"stable-ref subgraph returned the wrong center or no nodes: {subgraph}",
        )

        pf_job_id, _ = submit_job(
            base_url,
            session_id,
            session_etag,
            "power_flow",
            PF_REQUEST,
            "submit power_flow job",
        )
        cleanup_job_ids.append(pf_job_id)
        completed = wait_job(base_url, pf_job_id, "poll power_flow job")
        require(completed.get("state") == "succeeded", f"power_flow job failed: {completed}")
        require(completed.get("stale_against_current_model") is False, f"fresh PF is stale: {completed}")
        result = completed.get("result")
        assert_scientific_power_flow(result)
        assert_result_contract(
            result,
            "power_flow",
            session_id,
            pf_job_id,
            model_revision=1,
            current_model_revision=1,
            stale=False,
        )

        frame_response = request(
            base_url,
            "GET",
            v1_path(f"/api/v1/jobs/{pf_job_id}/frames/0", domain="ac", limit=3),
        )
        frame = require_status(frame_response, 200, "GET power_flow frame")
        require_json_headers(
            frame_response, "GET power_flow frame", cache_control="private, max-age=30"
        )
        assert_frame(frame, pf_job_id, session_id, 1)

        violations_response = request(
            base_url,
            "GET",
            v1_path(f"/api/v1/jobs/{pf_job_id}/violations", step=0, limit=5),
        )
        violations = require_status(
            violations_response, 200, "GET power_flow violations"
        )
        require_json_headers(
            violations_response,
            "GET power_flow violations",
            cache_control="private, max-age=30",
        )
        assert_violations(violations, pf_job_id, session_id, 1)

        opf_job_id, _ = submit_job(
            base_url,
            session_id,
            session_etag,
            "optimal_power_flow",
            OPF_REQUEST,
            "submit optimal_power_flow job",
        )
        cleanup_job_ids.append(opf_job_id)
        opf_completed = wait_job(base_url, opf_job_id, "poll optimal_power_flow job")
        require(
            opf_completed.get("state") == "succeeded",
            f"optimal_power_flow job failed: {opf_completed}",
        )
        opf_result = opf_completed.get("result")
        assert_scientific_opf(opf_result)
        assert_result_contract(
            opf_result,
            "optimal_power_flow",
            session_id,
            opf_job_id,
            model_revision=1,
            current_model_revision=1,
            stale=False,
        )

        # A dedicated large session makes the blocker observably long-running;
        # with one worker the following PF is then deterministically queued.
        cancel_session_response = request(
            base_url, "POST", "/api/v1/sessions", {"case": "case2000_acdc"}
        )
        cancel_session = require_status(
            cancel_session_response, 201, "create cancellation session"
        )
        require_json_headers(cancel_session_response, "create cancellation session")
        cancel_session_id = cancel_session.get("session_id")
        require(
            isinstance(cancel_session_id, str) and cancel_session_id,
            f"bad cancellation session id: {cancel_session}",
        )
        cancel_session_etag = require_model_headers(
            cancel_session_response,
            cancel_session_id,
            1,
            "create cancellation session",
        )
        auxiliary_sessions[cancel_session_id] = cancel_session_etag
        blocker_job_id, _ = submit_job(
            base_url,
            cancel_session_id,
            cancel_session_etag,
            "power_flow",
            PF_REQUEST,
            "submit cancellation blocker",
        )
        cleanup_job_ids.append(blocker_job_id)
        blocker_deadline = time.monotonic() + 10.0
        while True:
            blocker_response = request(base_url, "GET", f"/api/v1/jobs/{blocker_job_id}")
            blocker = require_status(blocker_response, 200, "poll cancellation blocker")
            require_json_headers(
                blocker_response, "poll cancellation blocker", cache_control="no-store"
            )
            if blocker.get("state") == "running":
                break
            require(
                blocker.get("state") == "queued",
                f"cancellation blocker ended before queueing the target: {blocker}",
            )
            require(
                time.monotonic() < blocker_deadline,
                "cancellation blocker did not start within 10 seconds",
            )
            time.sleep(0.01)

        cancel_job_id, cancel_submission = submit_job(
            base_url,
            session_id,
            session_etag,
            "power_flow",
            PF_REQUEST,
            "submit queued cancellation target",
        )
        cleanup_job_ids.append(cancel_job_id)
        require(
            cancel_submission.get("state") == "queued",
            f"cancellation target was not queued behind the blocker: {cancel_submission}",
        )
        cancel_response = request(
            base_url, "POST", f"/api/v1/jobs/{cancel_job_id}/cancel", {}
        )
        cancelled = require_status(cancel_response, 200, "cancel queued job")
        require_json_headers(cancel_response, "cancel queued job", cache_control="no-store")
        require(
            cancelled.get("state") in {"cancelled", "cancelling"}
            and cancelled.get("cancel_requested") is True
            and cancelled.get("job_id") == cancel_job_id,
            f"cancellation was not acknowledged before completion: {cancelled}",
        )
        cancelled_terminal = wait_job(base_url, cancel_job_id, "poll cancelled job")
        require(
            cancelled_terminal.get("state") == "cancelled"
            and "result" not in cancelled_terminal,
            f"cancelled job exposed a result: {cancelled_terminal}",
        )

        blocker_cancel_response = request(
            base_url, "POST", f"/api/v1/jobs/{blocker_job_id}/cancel", {}
        )
        blocker_cancel = require_status(
            blocker_cancel_response, 200, "cancel running blocker"
        )
        require_json_headers(
            blocker_cancel_response, "cancel running blocker", cache_control="no-store"
        )
        require(
            blocker_cancel.get("cancel_requested") is True
            and blocker_cancel.get("state") in {"cancelling", "cancelled"},
            f"running cancellation was not acknowledged: {blocker_cancel}",
        )
        blocker_terminal = wait_job(
            base_url, blocker_job_id, "poll cancelled blocker", timeout=120.0
        )
        require(
            blocker_terminal.get("state") == "cancelled"
            and "result" not in blocker_terminal,
            f"cancelled running job exposed a result: {blocker_terminal}",
        )

        replace_response = request(
            base_url,
            "PUT",
            f"/api/v1/sessions/{session_id}/model",
            {"case": "ieee24_3area_acdc_expanded"},
            headers={"If-Match": session_etag},
        )
        replaced = require_status(replace_response, 200, "replace v1 model")
        require_json_headers(replace_response, "replace v1 model")
        require(
            replaced.get("schema") == "hysim_session_v1"
            and replaced.get("session_id") == session_id
            and replaced.get("model_revision") == 2
            and replaced.get("has_model") is True,
            f"model revision did not increment: {replaced}",
        )
        new_etag = require_model_headers(replace_response, session_id, 2, "replace v1 model")
        require(
            new_etag != session_etag and replaced.get("etag") == new_etag,
            f"replacement did not issue a new ETag: {replaced}",
        )

        stale_put_response = request(
            base_url,
            "PUT",
            f"/api/v1/sessions/{session_id}/model",
            {"model": authored_model},
            headers={"If-Match": session_etag},
        )
        stale_put = require_flat_error(
            stale_put_response, 412, "revision_conflict", "PUT model with stale ETag"
        )
        require(
            isinstance(stale_put.get("details"), dict)
            and stale_put["details"].get("supplied_etag") == session_etag
            and stale_put["details"].get("current_etag") == new_etag
            and stale_put["details"].get("current_model_revision") == 2,
            f"stale ETag details are incomplete: {stale_put}",
        )
        require_model_headers(stale_put_response, session_id, 2, "PUT model with stale ETag")
        session_etag = new_etag

        stale_job_response = request(base_url, "GET", f"/api/v1/jobs/{pf_job_id}")
        stale_job = require_status(stale_job_response, 200, "GET stale power_flow job")
        require_json_headers(stale_job_response, "GET stale power_flow job", cache_control="no-store")
        require(
            stale_job.get("state") == "succeeded"
            and stale_job.get("model_revision") == 1
            and stale_job.get("current_model_revision") == 2
            and stale_job.get("stale_against_current_model") is True,
            f"old-revision job was not marked stale: {stale_job}",
        )
        stale_result = stale_job.get("result")
        assert_scientific_power_flow(stale_result)
        assert_result_contract(
            stale_result,
            "power_flow",
            session_id,
            pf_job_id,
            model_revision=1,
            current_model_revision=2,
            stale=True,
        )

        collection_response = request(
            base_url, "GET", f"/api/v1/sessions/{session_id}/jobs"
        )
        collection = require_status(collection_response, 200, "job collection")
        require_json_headers(collection_response, "job collection", cache_control="no-store")
        require(collection.get("schema") == "hysim_job_collection_v1", f"bad collection: {collection}")
        jobs = collection.get("jobs")
        require(isinstance(jobs, list), f"collection jobs is not an array: {collection}")
        listed_ids = {item.get("job_id") for item in jobs if isinstance(item, dict)}
        session_job_ids = [pf_job_id, opf_job_id, cancel_job_id]
        require(
            set(session_job_ids).issubset(listed_ids),
            f"jobs submitted for session {session_id} are absent from collection: "
            f"expected {session_job_ids}, got {sorted(str(value) for value in listed_ids)}",
        )
        require(
            all("result" not in item for item in jobs if isinstance(item, dict)),
            f"job collection inlined complete results: {collection}",
        )

        for retained_job_id in list(cleanup_job_ids):
            delete_job(base_url, retained_job_id, f"delete job {retained_job_id}")
            cleanup_job_ids.remove(retained_job_id)

        delete_session(
            base_url, session_id, session_etag, "delete primary session"
        )
        session_id = None
        session_etag = None
        for auxiliary_session_id, auxiliary_etag in list(auxiliary_sessions.items()):
            delete_session(
                base_url,
                auxiliary_session_id,
                auxiliary_etag,
                f"delete auxiliary session {auxiliary_session_id}",
            )
            del auxiliary_sessions[auxiliary_session_id]

        print(
            json.dumps(
                {
                    "schema": "resilience_edition_e2e_result",
                    "edition": "resilience",
                    "status": "passed",
                    "retained_routes_checked": len(retained),
                    "handler_routes_checked": handler_statuses,
                    "disabled_routes_checked": disabled_count,
                    "analysis_plan_steps": len(modules),
                    "v1": {
                        "model_get": "passed",
                        "preconditions": {"missing": 428, "stale": 412},
                        "known_disabled_analysis": 403,
                        "unknown_analysis": 400,
                        "power_flow": "succeeded",
                        "optimal_power_flow": "succeeded",
                        "topology_subgraph_frame_violations": "passed",
                        "queued_cancel": "cancelled",
                        "running_cancel": "cancelled",
                        "revision": 2,
                        "stale_old_result": True,
                    },
                    "result_contract": "hysim_result_v1",
                    "cleanup": {"jobs": "deleted", "sessions": "deleted"},
                },
                sort_keys=True,
            )
        )
        return 0
    except (AssertionError, OSError, urllib.error.URLError) as error:
        print(f"FAIL: {error}", file=sys.stderr)
        return 1
    finally:
        # Best-effort API cleanup makes a failed assertion leave less server-side
        # state, but process teardown remains unconditional and bounded.
        for retained_job_id in cleanup_job_ids:
            try:
                response = request(
                    base_url, "GET", f"/api/v1/jobs/{retained_job_id}", timeout=1.0
                )
                job = response.json()
                if isinstance(job, dict) and job.get("state") not in TERMINAL_JOB_STATES:
                    request(
                        base_url,
                        "POST",
                        f"/api/v1/jobs/{retained_job_id}/cancel",
                        {},
                        timeout=1.0,
                    )
                    deadline = time.monotonic() + 2.0
                    while time.monotonic() < deadline:
                        job_response = request(
                            base_url,
                            "GET",
                            f"/api/v1/jobs/{retained_job_id}",
                            timeout=1.0,
                        )
                        job = job_response.json()
                        if isinstance(job, dict) and job.get("state") in TERMINAL_JOB_STATES:
                            break
                        time.sleep(0.05)
                request(
                    base_url,
                    "DELETE",
                    f"/api/v1/jobs/{retained_job_id}",
                    timeout=1.0,
                )
            except Exception:
                pass
        if session_id is not None and session_etag is not None:
            try:
                delete_session(
                    base_url, session_id, session_etag, "cleanup primary session"
                )
            except Exception:
                pass
        for auxiliary_session_id, auxiliary_etag in auxiliary_sessions.items():
            try:
                delete_session(
                    base_url,
                    auxiliary_session_id,
                    auxiliary_etag,
                    f"cleanup auxiliary session {auxiliary_session_id}",
                )
            except Exception:
                pass
        process.terminate()
        try:
            process.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=5.0)


if __name__ == "__main__":
    raise SystemExit(main())
