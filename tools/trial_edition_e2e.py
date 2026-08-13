#!/usr/bin/env python3
"""HTTP contract test for the fail-closed Trial GUI server."""

from __future__ import annotations

import argparse
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


def request(base_url: str, path: str, body: dict | None = None) -> tuple[int, dict]:
    data = None if body is None else json.dumps(body).encode("utf-8")
    req = urllib.request.Request(
        base_url + path,
        data=data,
        headers={"Content-Type": "application/json"},
        method="GET" if body is None else "POST",
    )
    try:
        with urllib.request.urlopen(req, timeout=10.0) as response:
            return response.status, json.loads(response.read().decode("utf-8"))
    except urllib.error.HTTPError as error:
        return error.code, json.loads(error.read().decode("utf-8"))


def wait_ready(base_url: str) -> bool:
    deadline = time.monotonic() + 30.0
    while time.monotonic() < deadline:
        try:
            if request(base_url, "/api/edition")[0] == 200:
                return True
        except (OSError, urllib.error.URLError):
            pass
        time.sleep(0.1)
    return False


def expect_disabled(base_url: str, path: str, feature: str) -> None:
    status, payload = request(base_url, path, {})
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
        status, profile = request(base_url, "/api/edition")
        if status != 200 or profile.get("edition") != "trial":
            raise AssertionError(f"unexpected edition profile: {profile}")
        if profile.get("route_policy", {}).get("mode") != "fail_closed":
            raise AssertionError("Trial route policy is not fail-closed")
        if len(profile.get("workflow", [])) != 5 or len(profile.get("indicators", [])) != 8:
            raise AssertionError("Trial workflow/indicator contract is incomplete")

        retained = ("/api/cases", "/api/session/status", "/api/opf/parameter_contract", "/api/v1")
        for path in retained:
            status, _ = request(base_url, path)
            if status != 200:
                raise AssertionError(f"retained endpoint {path} returned {status}")

        status, plan = request(base_url, "/api/edition/analysis_plan", {
            "indicators": ["system_economic", "user_carbon", "system_reliability"]
        })
        modules = [step.get("module") for step in plan.get("steps", [])]
        expected = ["powerFlow", "opf", "hosting", "carbonFlow", "scenarioGeneration",
                    "reliability", "shortCircuit", "weakLinks"]
        if status != 200 or modules != expected or plan.get("automatic_execution") is not False:
            raise AssertionError(f"unexpected backend analysis plan: {status} {plan}")

        disabled = {
            "/api/dynamics/model_schema": "dynamics",
            "/api/session/run_transient": "dynamics",
            "/api/session/harmonics": "harmonics",
            "/api/session/run_rpo": "reactive_power_optimization",
            "/api/session/set_ts_config": "time_series",
            "/api/session/run_ts_pf": "time_series",
            "/api/session/run_annual_sim": "time_series",
            "/api/session/run_market_clearing": "market",
            "/api/session/run_campus_ies": "integrated_energy",
            "/api/session/run_ev_traffic": "ev_traffic",
            "/api/session/run_reconfig": "network_reconfiguration",
            "/api/session/run_counterfactual_planning": "counterfactual_planning",
            "/api/session/sppt_agent": "sppt_agent",
            "/api/session/load_bpa_dat": "advanced_io",
            "/api/session/load_cim_dist": "advanced_io",
            "/api/v1/specialist_solver": "unclassified_api",
            "/api/session/not_registered_in_profile": "unclassified_api",
        }
        for path, feature in disabled.items():
            expect_disabled(base_url, path, feature)

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
