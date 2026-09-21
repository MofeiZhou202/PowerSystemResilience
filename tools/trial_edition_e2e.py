#!/usr/bin/env python3
"""HTTP contract test for the fail-closed Trial GUI server."""

from __future__ import annotations

import argparse
import http.client
import json
import socket
import subprocess
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent


def free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])


def decode_json(raw: bytes, method: str, path: str, status: int) -> dict:
    try:
        payload = json.loads(raw.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise AssertionError(
            f"{method} {path}: HTTP {status} body is not valid UTF-8 JSON: "
            f"{raw[:200]!r}"
        ) from error
    if not isinstance(payload, dict):
        raise AssertionError(
            f"{method} {path}: HTTP {status} JSON body is not an object: {payload!r}"
        )
    return payload


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


def request(
    base_url: str,
    method: str,
    path: str,
    body: dict | None = None,
) -> tuple[int, dict]:
    data = None if body is None else json.dumps(body).encode("utf-8")
    headers = {"Accept": "application/json", "Connection": "close"}
    if body is not None:
        headers["Content-Type"] = "application/json"
    req = urllib.request.Request(
        base_url + path,
        data=data,
        headers=headers,
        method=method,
    )
    try:
        with urllib.request.urlopen(req, timeout=10.0) as response:
            raw = response.read()
            return response.status, decode_json(raw, method, path, response.status)
    except urllib.error.HTTPError as error:
        status = error.code
        raw = read_http_error_body(error, method, path)
        return status, decode_json(raw, method, path, status)


def wait_ready(base_url: str) -> bool:
    deadline = time.monotonic() + 30.0
    while time.monotonic() < deadline:
        try:
            status, profile = request(base_url, "GET", "/api/edition")
            if status == 200 and profile.get("edition") == "trial":
                return True
        except (OSError, urllib.error.URLError):
            pass
        time.sleep(0.1)
    return False


def expect_disabled(
    base_url: str,
    method: str,
    path: str,
    feature: str,
) -> None:
    status, payload = request(base_url, method, path)
    error = payload.get("error", {})
    if status != 403 or error.get("code") != "TRIAL_FEATURE_DISABLED":
        raise AssertionError(f"{path} bypassed Trial guard: {status} {payload}")
    if error.get("feature") != feature:
        raise AssertionError(f"{path} classified as {error.get('feature')}, expected {feature}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--server", required=True)
    parser.add_argument("--data-dir", default=str(ROOT / "data"))
    args = parser.parse_args()

    port = free_port()
    base_url = f"http://127.0.0.1:{port}"
    process = subprocess.Popen(
        [args.server, "--host", "127.0.0.1", "--port", str(port),
         "--data-dir", args.data_dir],
        cwd=ROOT,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    try:
        if not wait_ready(base_url):
            raise AssertionError("Trial server did not become ready")
        status, profile = request(base_url, "GET", "/api/edition")
        if status != 200 or profile.get("edition") != "trial":
            raise AssertionError(f"unexpected edition profile: {profile}")
        if profile.get("route_policy", {}).get("mode") != "fail_closed":
            raise AssertionError("Trial route policy is not fail-closed")
        if len(profile.get("workflow", [])) != 5 or len(profile.get("indicators", [])) != 8:
            raise AssertionError("Trial workflow/indicator contract is incomplete")

        retained = ("/api/cases", "/api/session/status", "/api/opf/parameter_contract", "/api/v1")
        for path in retained:
            status, _ = request(base_url, "GET", path)
            if status != 200:
                raise AssertionError(f"retained endpoint {path} returned {status}")

        status, plan = request(base_url, "POST", "/api/edition/analysis_plan", {
            "indicators": ["system_economic", "user_carbon", "system_reliability"]
        })
        modules = [step.get("module") for step in plan.get("steps", [])]
        expected = ["powerFlow", "opf", "hosting", "carbonFlow", "scenarioGeneration",
                    "reliability", "shortCircuit", "weakLinks"]
        if status != 200 or modules != expected or plan.get("automatic_execution") is not False:
            raise AssertionError(f"unexpected backend analysis plan: {status} {plan}")

        disabled = {
            ("GET", "/api/dynamics/model_schema"): "dynamics",
            ("POST", "/api/session/run_transient"): "dynamics",
            ("POST", "/api/session/harmonics"): "harmonics",
            ("POST", "/api/session/run_rpo"): "reactive_power_optimization",
            ("POST", "/api/session/set_ts_config"): "time_series",
            ("POST", "/api/session/run_ts_pf"): "time_series",
            ("POST", "/api/session/run_annual_sim"): "time_series",
            ("POST", "/api/session/run_market_clearing"): "market",
            ("POST", "/api/session/run_campus_ies"): "integrated_energy",
            ("POST", "/api/session/run_ev_traffic"): "ev_traffic",
            ("POST", "/api/session/run_reconfig"): "network_reconfiguration",
            ("POST", "/api/session/run_counterfactual_planning"): "counterfactual_planning",
            ("POST", "/api/session/sppt_agent"): "sppt_agent",
            ("POST", "/api/session/load_bpa_dat"): "advanced_io",
            ("POST", "/api/session/load_cim_dist"): "advanced_io",
            ("POST", "/api/v1/specialist_solver"): "unclassified_api",
            ("POST", "/api/session/not_registered_in_profile"): "unclassified_api",
        }
        for (method, path), feature in disabled.items():
            expect_disabled(base_url, method, path, feature)

        print(json.dumps({"edition": "trial", "retained_checked": len(retained),
                          "disabled_checked": len(disabled), "plan_steps": len(expected)}))
        return 0
    except AssertionError as error:
        print(f"FAIL: {error}", file=sys.stderr)
        return 1
    finally:
        process.terminate()
        try:
            process.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            process.kill()


if __name__ == "__main__":
    raise SystemExit(main())
