#!/usr/bin/env python3
"""End-to-end test for v1 sessions, ETags, and asynchronous jobs."""

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
sys.path.insert(0, str(ROOT / "python" / "src"))

from hysim import HySimV1Client, PowerFlowOptions, PowerFlowRequest  # noqa: E402


def free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])


def wait_ready(base_url: str, timeout: float = 20.0) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            with urllib.request.urlopen(base_url + "/api/v1", timeout=1.0):
                return True
        except (urllib.error.URLError, OSError):
            time.sleep(0.1)
    return False


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--server", required=True)
    parser.add_argument("--data-dir", default=str(ROOT / "data"))
    parser.add_argument("--port", type=int, default=0)
    args = parser.parse_args()

    port = args.port or free_port()
    base_url = f"http://127.0.0.1:{port}"
    process = subprocess.Popen(
        [
            args.server,
            "--host",
            "127.0.0.1",
            "--port",
            str(port),
            "--api-job-workers",
            "2",
            "--data-dir",
            args.data_dir,
        ],
        cwd=ROOT,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    sessions = []
    jobs = []
    try:
        if not wait_ready(base_url):
            print("FAIL: /api/v1 did not become ready", file=sys.stderr)
            return 1
        client = HySimV1Client(base_url)
        capabilities = client.capabilities()
        required = {
            "multi_session",
            "model_revision",
            "etag",
            "asynchronous_jobs",
            "topology_lod",
            "spatial_chunks",
            "time_chunks",
        }
        if not required.issubset(set(capabilities.get("features", []))):
            print("FAIL: v1 capability declaration is incomplete", file=sys.stderr)
            return 1

        first = client.create_session(case="ieee14_acdc")
        second = client.create_session(case="ieee24_3area_acdc_expanded")
        sessions.extend([first, second])
        if first.session_id == second.session_id:
            print("FAIL: session IDs are not unique", file=sys.stderr)
            return 1

        request = PowerFlowRequest(
            options=PowerFlowOptions(max_iter=100, tol=1.0e-8)
        )
        first_job = first.power_flow(request)
        second_job = second.power_flow(request)
        jobs.extend([first_job, second_job])
        first_job.wait(timeout=60.0)
        second_job.wait(timeout=60.0)
        first_result = first_job.result()
        second_result = second_job.result()
        if not first_result.data.get("converged") or not second_result.data.get(
            "converged"
        ):
            print("FAIL: one of the isolated PF jobs did not converge", file=sys.stderr)
            return 1

        topology = first.topology(lod=2, limit=10_000)
        topology_nodes = topology.get("nodes", [])
        if not topology_nodes or any(
            node.get("ref", {}).get("domain") not in {"ac", "dc"}
            or not isinstance(node.get("ref", {}).get("index"), int)
            for node in topology_nodes
        ):
            print("FAIL: topology chunk does not use stable bus references", file=sys.stderr)
            return 1
        first_ref = topology_nodes[0]["ref"]
        subgraph = first.subgraph(first_ref["domain"], first_ref["index"], depth=1)
        if subgraph.get("center") != first_ref or not subgraph.get("nodes"):
            print("FAIL: stable-ref subgraph query returned the wrong center", file=sys.stderr)
            return 1

        frame = first_job.frame(0, domain="ac", limit=3)
        if frame.get("returned_nodes", 0) > 3 or frame.get("step") != 0:
            print("FAIL: result frame did not honor the requested time/spatial chunk", file=sys.stderr)
            return 1
        violations = first_job.violations(0, limit=5)
        if violations.get("returned", 0) > 5:
            print("FAIL: violation chunk did not honor its limit", file=sys.stderr)
            return 1

        old_etag = first.etag
        first.replace_model(case="ieee24_3area_acdc_expanded")
        conflict = client.transport.request(
            "PUT",
            f"/api/v1/sessions/{first.session_id}/model",
            json_body={"case": "ieee14_acdc"},
            headers={"If-Match": old_etag},
        )
        if conflict.status_code != 412:
            print(f"FAIL: stale ETag returned {conflict.status_code}", file=sys.stderr)
            return 1

        first_job.refresh()
        if not first_job.result().stale:
            print("FAIL: old-revision job was not marked stale", file=sys.stderr)
            return 1
        second.refresh()
        if second.model_revision != 1 or second.raw.get("name") != (
            "IEEE24-3area AC/DC Expanded"
        ):
            print("FAIL: second session was contaminated", file=sys.stderr)
            return 1

        first_bus = first_result.data.get("ac_bus_results", [{}])[0]
        if first_bus.get("domain") != "ac" or "index" not in first_bus:
            print("FAIL: result bus reference is not domain-qualified", file=sys.stderr)
            return 1

        print(
            json.dumps(
                {
                    "schema": "hysim_v1_e2e_result",
                    "session_ids_unique": True,
                    "revisions": [first.model_revision, second.model_revision],
                    "etag_conflict": conflict.status_code,
                    "jobs": [first_job.state, second_job.state],
                    "stale_after_update": True,
                    "topology_nodes": len(topology_nodes),
                    "subgraph_nodes": len(subgraph.get("nodes", [])),
                    "frame_nodes": frame.get("returned_nodes", 0),
                    "violations": violations.get("returned", 0),
                },
                sort_keys=True,
            )
        )
        return 0
    finally:
        for job in jobs:
            try:
                if not job.terminal:
                    job.cancel()
                    job.wait(timeout=10.0)
                job.delete()
            except Exception:
                pass
        for session in sessions:
            try:
                session.delete()
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
