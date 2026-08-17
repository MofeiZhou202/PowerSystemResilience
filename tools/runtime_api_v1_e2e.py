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

        request = PowerFlowRequest(options=PowerFlowOptions(
            max_iter=100,
            tol=1.0e-8,
            robust_nonlinear={
                "enable_vsc_local_schur": False,
                "vsc_schur_min_network_dimension": 321,
                "vsc_schur_local_rcond_tolerance": 1.0e-7,
                "vsc_schur_backward_error_tolerance": 2.0e-11,
                "enable_smooth_ncp": True,
                "ncp_mu0": 2.0e-3,
                "ncp_mu_min": 3.0e-11,
                "ncp_mu_factor": 0.2,
                "ncp_mu_factor_coarse": 0.7,
                "ncp_mu_phase_transition": 0.25,
            },
        ))
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
        reactive_limits = first_result.data.get("reactive_limits", {})
        validity_flags = first_result.data.get("validity_flags", {})
        effective_robust = first_result.data.get("options_effective", {}).get(
            "robust_nonlinear", {}
        )
        if not (
            reactive_limits.get("enforcement_requested") is True
            and reactive_limits.get("certified") is True
            and reactive_limits.get("active_set_cycle_detected") is False
            and reactive_limits.get("outer_iteration_limit_reached") is False
            and validity_flags.get("generator_reactive_limits_certified") is True
            and effective_robust.get("enable_vsc_local_schur") is False
            and effective_robust.get("vsc_schur_min_network_dimension") == 321
            and effective_robust.get("vsc_schur_local_rcond_tolerance") == 1.0e-7
            and effective_robust.get("vsc_schur_backward_error_tolerance") == 2.0e-11
            and effective_robust.get("enable_smooth_ncp") is True
            and effective_robust.get("ncp_mu0") == 2.0e-3
            and effective_robust.get("ncp_mu_min") == 3.0e-11
            and effective_robust.get("ncp_mu_factor") == 0.2
            and effective_robust.get("ncp_mu_factor_coarse") == 0.7
            and effective_robust.get("ncp_mu_phase_transition") == 0.25
        ):
            print(
                f"FAIL: versioned PF Q-limit certificate is incomplete: {reactive_limits}",
                file=sys.stderr,
            )
            return 1

        opf_job = first.optimal_power_flow(
            {
                "solver": "parity",
                "network_model": "balanced_aggregate",
                "options": {
                    "max_inner_iterations": 300,
                    "allow_fallback": False,
                    "enable_phase_one": False,
                    "ac_pf_warm_start": False,
                    "ac_pf_dc_phase_one": True,
                    "ac_pf_dc_phase_one_min_buses": 0,
                    "ac_pf_dc_phase_one_time_limit_ms": 1234.0,
                },
            }
        )
        jobs.append(opf_job)
        opf_job.wait(timeout=60.0)
        opf_result = opf_job.result().data
        profiling = opf_result.get("ipm_profiling", {})
        if not opf_result.get("converged"):
            print("FAIL: v1 parity OPF contract probe did not converge", file=sys.stderr)
            return 1
        if profiling.get("phase_one_termination") != "disabled":
            print("FAIL: v1 enable_phase_one=false was not applied", file=sys.stderr)
            return 1
        if profiling.get("dc_phase_one_requested") is not False:
            print("FAIL: v1 reported a DC Phase I without an AC-PF start", file=sys.stderr)
            return 1
        if profiling.get("dc_phase_one_time_limit_ms") != 1234.0:
            print("FAIL: v1 did not apply the DC Phase-I wall budget", file=sys.stderr)
            return 1
        required_profiling = {
            "initial_primal_residual",
            "initial_dual_residual",
            "dc_phase_one_status",
            "dc_phase_one_budget_exhausted",
            "dc_phase_one_symbolic_analyze_calls",
            "parity_formulation_builds",
            "prepared_session_used",
            "formulation_reused",
            "mapping_reused",
            "symbolic_reused",
            "continuation_state_reused",
            "numeric_refactor_status",
            "prepared_session_invalidation_reason",
            "phase_one_factorizations",
            "phase_two_start_accepted",
            "factorization_calls",
        }
        if not required_profiling.issubset(profiling):
            print("FAIL: v1 OPF profiling contract is incomplete", file=sys.stderr)
            return 1

        topology = first.topology_chunk(lod=2, limit=10_000)
        topology_nodes = topology.nodes
        if not topology_nodes or any(
            node.get("ref", {}).get("domain") not in {"ac", "dc"}
            or not isinstance(node.get("ref", {}).get("index"), int)
            for node in topology_nodes
        ):
            print("FAIL: topology chunk does not use stable bus references", file=sys.stderr)
            return 1
        first_ref = topology.bus_refs[0]
        subgraph = first.subgraph_view(first_ref, depth=1)
        if subgraph.center != first_ref or not subgraph.nodes:
            print("FAIL: stable-ref subgraph query returned the wrong center", file=sys.stderr)
            return 1

        frame = first_job.frame_chunk(0, domain="ac", limit=3)
        if frame.returned_nodes > 3 or frame.step != 0:
            print(
                "FAIL: result frame did not honor the requested time/spatial chunk",
                file=sys.stderr,
            )
            return 1
        violations = first_job.violation_chunk(0, limit=5)
        if violations.returned > 5:
            print("FAIL: violation chunk did not honor its limit", file=sys.stderr)
            return 1
        paged_refs = [
            ref
            for page in first.topology_pages(lod=2, page_size=5)
            for ref in page.bus_refs
        ]
        if len(paged_refs) != topology.total_nodes or paged_refs != list(topology.bus_refs):
            print("FAIL: typed topology auto-pagination changed stable node order", file=sys.stderr)
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
                    "opf_phase_one_disabled": True,
                    "stale_after_update": True,
                    "topology_nodes": len(topology_nodes),
                    "topology_pages": (topology.total_nodes + 4) // 5,
                    "subgraph_nodes": len(subgraph.nodes),
                    "frame_nodes": frame.returned_nodes,
                    "violations": violations.returned,
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
