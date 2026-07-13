#!/usr/bin/env python3
"""End-to-end smoke test for the GUI/HTTP backend (run_gui_server).

Starts the server, then drives the same REST endpoints the web UI uses and
asserts on the responses.  Exercises the full ETAP import/export round-trip,
OpenDSS IEEE13 three-phase import/PF when available, and the analysis endpoints
so a regression in the GUI backend fails loudly in CI.

No third-party dependencies (urllib + subprocess from the standard library).

Usage:
    python3 tools/gui_api_e2e.py [--server <path/to/run_gui_server>]
                                 [--data-dir <dir>] [--port <port>]
                                 [--skip-etap]

If --server is omitted, common build locations are searched.  Exit code is 0 on
success, 1 on any failed check.
"""
from __future__ import annotations

import argparse
import json
import os
import socket
import subprocess
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent


def find_server(explicit: str | None) -> Path:
    if explicit:
        p = Path(explicit)
        if not p.exists():
            sys.exit(f"server binary not found: {p}")
        return p
    candidates = [
        REPO_ROOT / "build" / "macos-release" / "run_gui_server",
        REPO_ROOT / "build" / "macos-release" / "tests" / "run_gui_server",
        REPO_ROOT / "build" / "tests" / "run_gui_server",
        REPO_ROOT / "build" / "tests" / "run_gui_server.exe",
        REPO_ROOT / "build_rel" / "tests" / "run_gui_server",
    ]
    for c in candidates:
        if c.exists():
            return c
    sys.exit("run_gui_server not found; build it or pass --server")


def free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


class Client:
    def __init__(self, base: str) -> None:
        self.base = base

    def _req(self, path: str, *, data: bytes | None, ctype: str,
             method: str = "POST"):
        req = urllib.request.Request(self.base + path, data=data, method=method)
        req.add_header("Content-Type", ctype)
        with urllib.request.urlopen(req, timeout=60) as r:
            return r.status, r.headers, r.read()

    def get(self, path: str):
        with urllib.request.urlopen(self.base + path, timeout=30) as r:
            return r.status, json.loads(r.read())

    def post_json(self, path: str, payload: dict | None = None):
        body = json.dumps(payload or {}).encode()
        st, _, raw = self._req(path, data=body, ctype="application/json")
        return st, json.loads(raw)

    def post_bytes(self, path: str, payload: bytes):
        return self._req(path, data=payload, ctype="application/octet-stream")


class Checker:
    def __init__(self) -> None:
        self.failures = 0
        self.checks = 0

    def check(self, cond: bool, msg: str) -> None:
        self.checks += 1
        mark = "PASS" if cond else "FAIL"
        if not cond:
            self.failures += 1
        print(f"  [{mark}] {msg}")


def wait_up(base: str, timeout_s: float = 20.0) -> bool:
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        try:
            with urllib.request.urlopen(base + "/api/cases", timeout=2):
                return True
        except (urllib.error.URLError, ConnectionError, OSError):
            time.sleep(0.25)
    return False


def maybe_run_opendss_ieee13_smoke(c: Client, chk: Checker) -> None:
    ieee13 = (
        REPO_ROOT / "external_data" / "opendss_ieee_pes" /
        "opendss_reference" / "13_node" / "official_full" /
        "IEEE13Nodeckt.dss"
    )
    if not ieee13.exists():
        print("10. OpenDSS IEEE13 three-phase smoke skipped (fixture missing)")
        return

    print("10. OpenDSS IEEE13 three-phase import + PF")
    dss_text = ieee13.read_text(encoding="utf-8", errors="replace")
    st, body = c.post_json(
        "/api/session/load_opendss",
        {
            "dss_string": dss_text,
            "filename": ieee13.name,
            "dss_path": str(ieee13),
        },
    )
    native_fixture = REPO_ROOT / "tests" / "fixtures" / "td_coordination_all_components.json"
    native_model = json.loads(native_fixture.read_text(encoding="utf-8"))
    using_opendss = st == 200 and body.get("_has_three_phase_ac") is True
    if not using_opendss:
        print("   OpenDSS phase bridge unavailable; using native hybrid abc fixture")
        st, body = c.post_json(
            "/api/session/load_json_string",
            {"json_string": json.dumps(native_model)},
        )
    chk.check(
        st == 200 and (body.get("_has_three_phase_ac") is True or
                       len(body.get("tp_buses", [])) > 0),
        f"phase model loaded via {'OpenDSS' if using_opendss else 'native JSON'}",
    )

    st, pf = c.post_json(
        "/api/session/pf",
        {
            "method": "three_phase",
            "options": {
                "max_iter": 200,
                "tol": 1e-8,
                "three_phase": {
                    "algorithm": "compact",
                    "scope": "ac_only",
                    "max_control_iter": 77,
                    "include_shunts": True,
                    "vmin_pu": 0.91,
                    "compare_opendss": using_opendss,
                },
            },
        },
    )
    comp = (pf.get("opendss_reference") or {}).get("comparison") or {}
    diag = pf.get("power_balance_diagnostics") or {}
    phase_options = (pf.get("options_effective") or {}).get("three_phase") or {}
    chk.check(
        st == 200 and pf.get("converged") is True and len(pf.get("tp_bus_results", [])) >= 3,
        f"three_phase PF converged={pf.get('converged')} buses={len(pf.get('tp_bus_results', []))}",
    )
    chk.check(
        diag.get("basis") == "three_phase_abc" and diag.get("ordinary") == [],
        f"three_phase diagnostic basis={diag.get('basis')} ordinary={len(diag.get('ordinary', []))}",
    )
    chk.check(
        phase_options.get("algorithm") == "compact" and
        phase_options.get("scope_effective") == "ac_only" and
        phase_options.get("max_control_iter") == 77 and
        abs(phase_options.get("vmin_pu", 0.0) - 0.91) < 1e-12,
        f"three_phase options effective={phase_options}",
    )
    st, nr_pf = c.post_json(
        "/api/session/pf",
        {
            "method": "three_phase",
            "options": {
                "max_iter": 200,
                "tol": 1e-8,
                "three_phase": {
                    "algorithm": "newton",
                    "scope": "ac_only",
                    "max_control_iter": 55,
                    "compare_opendss": False,
                },
            },
        },
    )
    nr_options = (nr_pf.get("options_effective") or {}).get("three_phase") or {}
    chk.check(
        st == 200 and nr_options.get("algorithm") == "newton" and
        nr_options.get("max_control_iter") == 55 and
        nr_pf.get("method_actual") == "three_phase_newton_abc",
        f"three_phase Newton options effective={nr_options}",
    )
    if using_opendss:
        chk.check(
            comp.get("within_gui_tolerance") is True and
            comp.get("max_vm_error_pu", 1.0) < 1.0e-3,
            f"OpenDSS comparison max_vm={comp.get('max_vm_error_pu')} count={comp.get('count')}",
        )

    st, staged = c.post_json(
        "/api/session/pf",
        {
            "method": "three_phase",
            "options": {
                "max_iter": 200,
                "tol": 1e-8,
                "enable_converter_coordination_check": False,
                "three_phase": {
                    "algorithm": "compact",
                    "scope": "staged_hybrid",
                    "compare_opendss": False,
                },
            },
        },
    )
    scope = staged.get("analysis_scope") or {}
    boundary = staged.get("hybrid_boundary") or {}
    chk.check(
        st == 200 and scope.get("monolithic_abc_dc_jacobian") is False and
        boundary.get("ran") is True,
        f"staged hybrid scope={scope.get('model_scope')} boundary={boundary.get('converged')}",
    )

    st, _ = c.post_json("/api/session/load_matpower", {"filename": "case9.m"})
    st, exported = c.post_json("/api/session/export_json")
    case9_with_phase = json.loads(exported["json_string"])
    case9_with_phase["three_phase_ac"] = native_model["three_phase_ac"]
    st, _ = c.post_json(
        "/api/session/load_json_string",
        {"json_string": json.dumps(case9_with_phase)},
    )
    st, opf = c.post_json(
        "/api/session/opf",
        {
            "solver": "parity",
            "network_model": "balanced_with_three_phase_validation",
            "constraints": {},
            "options": {"max_inner_iterations": 400},
            "three_phase": {
                "algorithm": "compact",
                "max_iter": 200,
                "tol": 1e-8,
                "compare_opendss": False,
            },
        },
    )
    validation = opf.get("three_phase_validation") or {}
    chk.check(
        st == 200 and opf.get("analysis_scope", {}).get("monolithic_three_phase_opf") is False and
        validation.get("ran") is True and len(validation.get("bus_results", [])) >= 3,
        f"OPF abc validation ran={validation.get('ran')} converged={validation.get('converged')}",
    )


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--server")
    ap.add_argument("--data-dir", default=str(REPO_ROOT / "data"))
    ap.add_argument("--port", type=int, default=0)
    ap.add_argument(
        "--skip-etap",
        action="store_true",
        help="skip ETAP-only export/import checks for builds without ETAP support",
    )
    args = ap.parse_args()

    server = find_server(args.server)
    port = args.port or free_port()
    base = f"http://127.0.0.1:{port}"

    proc = subprocess.Popen(
        [str(server), "--port", str(port),
         "--data-dir", args.data_dir, "--matpower-dir", args.data_dir],
        cwd=str(REPO_ROOT),
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )
    chk = Checker()
    try:
        if not wait_up(base):
            print("server did not come up in time")
            return 1
        c = Client(base)

        print("1. case + matpower listings")
        st, cases = c.get("/api/cases")
        chk.check(st == 200 and len(cases.get("cases", [])) > 0,
                  f"GET /api/cases -> {len(cases.get('cases', []))} cases")

        print("1a. load and solve internal urban LVNTS benchmark")
        st, body = c.post_json(
            "/api/session/load_builtin",
            {"case": "urban_lvn_primary_secondary"},
        )
        counts = body.get("counts", {})
        chk.check(
            st == 200 and counts.get("ac_buses") == 338 and
            counts.get("tp_buses") == 338 and counts.get("dc_buses") == 3,
            "urban LVNTS load -> "
            f"AC={counts.get('ac_buses')} ABC={counts.get('tp_buses')} "
            f"DC={counts.get('dc_buses')}",
        )
        st, pf = c.post_json(
            "/api/session/pf",
            {
                "method": "ac_newton",
                "options": {
                    "max_iter": 100,
                    "tol": 1e-7,
                    "enable_converter_coordination_check": True,
                },
            },
        )
        chk.check(
            st == 200 and pf.get("converged") is True and
            len(pf.get("vm", [])) == 338 and len(pf.get("vdc", [])) == 3 and
            pf.get("converter_coordination", {}).get("feasible") is True,
            "urban LVNTS hybrid PF -> "
            f"converged={pf.get('converged')} iterations={pf.get('iterations')} "
            f"residual={pf.get('residual')}",
        )

        print("2. load built-in ieee14_acdc")
        st, body = c.post_json("/api/session/load_builtin", {"case": "ieee14_acdc"})
        counts = body.get("counts", {})
        chk.check(st == 200 and counts.get("ac_buses") == 14,
                  f"load_builtin -> {counts.get('ac_buses')} AC buses")

        print("2a. modeling parameter library validation + auto-fulfill")
        st, library = c.get("/api/session/parameter_library")
        chk.check(
            st == 200 and library.get("library_validation", {}).get("valid") is True and
            len(library.get("rules", [])) >= 20,
            f"parameter library profile={library.get('profile_id')} rules={len(library.get('rules', []))}",
        )
        sparse_model = {
            "name": "parameter-library-e2e",
            "base_mva": 10.0,
            "ac": {
                "base_mva": 10.0,
                "freq_hz": 50.0,
                "buses": [
                    {"index": 1, "bus_type": "SLACK", "base_kv": 10.0},
                    {"index": 2, "bus_type": "PQ", "base_kv": 10.0},
                ],
                "branches": [
                    {"index": 1, "from_bus": 1, "to_bus": 2,
                     "r_pu": 0.0, "x_pu": 0.0},
                ],
                "generators": [
                    {"index": 1, "bus": 1, "is_slack": True,
                     "pmax_mw": 100.0, "qmax_mvar": 100.0,
                     "qmin_mvar": -100.0},
                ],
            },
        }
        st, _ = c.post_json(
            "/api/session/load_json_string",
            {"json_string": json.dumps(sparse_model)},
        )
        st, invalid = c.post_json("/api/session/parameter_library/validate")
        invalid_codes = {item.get("code") for item in invalid.get("diagnostics", [])}
        chk.check(
            st == 200 and "branch_zero_series_impedance" in invalid_codes,
            f"parameter validation detected={sorted(invalid_codes)}",
        )
        st, fulfilled = c.post_json("/api/session/parameter_library/apply")
        applied = fulfilled.get("parameter_apply", {})
        validation = fulfilled.get("parameter_validation", {})
        fulfilled_system = json.loads(fulfilled.get("_raw_json", "{}"))
        fulfilled_branch = fulfilled_system.get("ac", {}).get("branches", [{}])[0]
        chk.check(
            st == 200 and applied.get("fields_changed", 0) >= 2 and
            validation.get("valid") is True and
            fulfilled_branch.get("r_pu", 0.0) > 0.0 and
            fulfilled_branch.get("x_pu", 0.0) > 0.0,
            f"parameter auto-fulfill changed={applied.get('fields_changed')} valid={validation.get('valid')}",
        )

        print("2b. comprehensive hybrid 3W transformer balance attribution")
        st, _ = c.post_json(
            "/api/session/load_builtin",
            {"case": "comprehensive_hybrid_acdc"},
        )
        st, comprehensive_pf = c.post_json(
            "/api/session/pf",
            {
                "method": "ac_newton",
                "options": {"enable_converter_coordination_check": True},
            },
        )
        comprehensive_balance = comprehensive_pf.get("power_balance_diagnostics", {})
        chk.check(
            st == 200 and comprehensive_pf.get("converged") is True and
            comprehensive_balance.get("ordinary_bad_count") == 0 and
            comprehensive_balance.get("ordinary") == [],
            "comprehensive hybrid PF balance -> "
            f"converged={comprehensive_pf.get('converged')} "
            f"ordinary={comprehensive_balance.get('ordinary_bad_count')}",
        )

        print("2c. Level-1 cyber-physical reliability interface matrix")
        cyber_reliability_model = {
            "name": "cyber-physical-reliability-e2e",
            "base_mva": 10.0,
            "ac": {
                "base_mva": 10.0,
                "buses": [
                    {"index": 1, "bus_type": "SLACK", "base_kv": 10.0},
                    {"index": 2, "bus_type": "PQ", "base_kv": 10.0},
                ],
                "generators": [
                    {"index": 1, "bus": 1, "in_service": True,
                     "is_slack": True, "pmax_mw": 10.0,
                     "forced_outage_rate": 0.01, "mttr_hr": 2.0},
                ],
                "loads": [
                    {"index": 1, "bus": 2, "in_service": True,
                     "p_mw": 1.0, "n_customers": 100},
                ],
                "branches": [
                    {"index": 1, "from_bus": 1, "to_bus": 2,
                     "in_service": True, "r_pu": 0.01, "x_pu": 0.1,
                     "rate_a_mva": 10.0, "failure_rate": 1.0, "mttr_hr": 4.0},
                    {"index": 2, "from_bus": 1, "to_bus": 2,
                     "in_service": False, "r_pu": 0.01, "x_pu": 0.1,
                     "rate_a_mva": 10.0},
                ],
            },
        }
        st, _ = c.post_json(
            "/api/session/load_json_string",
            {"json_string": json.dumps(cyber_reliability_model)},
        )
        st, cyber_result = c.post_json(
            "/api/session/run_reliability",
            {
                "method": "fmea",
                "execution": {"parallel": False},
                "restoration": {
                    "enable_switch_reconfiguration": True,
                    "max_switch_actions": 1,
                },
                "cyber_physical": {
                    "enabled": True,
                    "automation_availability": 0.75,
                    "automatic_switching_time_hr": 0.05,
                    "manual_switching_time_hr": 1.0,
                    "freeze_der_on_automation_loss": False,
                },
            },
        )
        cyber = cyber_result.get("cyber_physical") or {}
        validity = cyber_result.get("validity") or {}
        adjusted = float(cyber.get("eens_adjusted_mwh_yr", -1.0))
        lower = float(cyber.get("eens_perfect_cyber_mwh_yr", -1.0))
        upper = float(cyber.get("eens_no_automation_mwh_yr", -1.0))
        chk.check(
            st == 200 and cyber.get("enabled") is True and
            cyber.get("level") == 1 and lower <= adjusted <= upper and
            abs(float(cyber.get("automation_efficacy", -1.0)) - 0.75) < 1e-8 and
            validity.get("restoration_duration_cyber_conditioned") is True and
            validity.get("cyber_topology_modelled") is False and
            validity.get("cyber_power_coupling_modelled") is False,
            f"cyber-physical FMEA bounds={lower:.4f}/{adjusted:.4f}/{upper:.4f}",
        )

        print("2d. Built-in cyber-physical reliability effectiveness case")
        st, demo = c.post_json(
            "/api/session/load_builtin",
            {"case": "cyber_physical_reliability_demo"},
        )
        demo_counts = demo.get("counts", {})
        chk.check(
            st == 200 and demo_counts.get("ac_buses") == 3 and
            demo_counts.get("ac_branches") == 3 and
            demo_counts.get("loads") == 2,
            "cyber reliability demo loads as a built-in case",
        )
        st, demo_result = c.post_json(
            "/api/session/run_reliability",
            {
                "method": "fmea",
                "execution": {"parallel": False},
                "restoration": {
                    "enable_switch_reconfiguration": True,
                    "max_switch_actions": 1,
                },
                "cyber_physical": {
                    "enabled": True,
                    "automation_availability": 0.75,
                    "automatic_switching_time_hr": 0.05,
                    "manual_switching_time_hr": 1.0,
                    "freeze_der_on_automation_loss": True,
                },
            },
        )
        demo_cyber = demo_result.get("cyber_physical") or {}
        feeder_result = next(
            (
                row for row in demo_result.get("contingencies", [])
                if row.get("component_type") == "ac_branch" and
                row.get("component_index") == 0
            ),
            {},
        )
        chk.check(
            st == 200 and
            float(demo_cyber.get("delta_cyber_duration_mwh_yr", 0.0)) > 0.0 and
            float(demo_cyber.get("delta_cyber_control_mwh_yr", 0.0)) > 0.0 and
            abs(float(feeder_result.get("eens_contribution", 0.0)) - 2.075) < 1e-6,
            "built-in demo separates restoration-delay and control-loss EENS",
        )

        print("2e. Distribution CIM XML import/export round-trip")
        cim_fixtures = sorted((REPO_ROOT / "external_data" / "xml").glob("*.xml"))
        chk.check(len(cim_fixtures) == 3, "distribution CIM fixtures are available")
        cim_source = cim_fixtures[0].read_text(encoding="utf-8")
        st, cim_loaded = c.post_json(
            "/api/session/load_cim_dist",
            {"xml_string": cim_source},
        )
        cim_counts = cim_loaded.get("counts", {})
        chk.check(
            st == 200 and cim_counts.get("ac_buses", 0) > 0 and
            cim_counts.get("ac_branches", 0) > 0 and
            cim_counts.get("transformers_2w") == 1 and
            cim_counts.get("loads", 0) > 0,
            "distribution CIM fixture maps topology, transformer, and loads",
        )
        st, cim_exported = c.post_json("/api/session/export_cim_dist", {})
        cim_xml = cim_exported.get("xml_string", "")
        chk.check(
            st == 200 and "<rdf:RDF" in cim_xml and
            "<cim:PowerTransformer" in cim_xml and
            "<cim:LVBuilding" in cim_xml,
            "distribution CIM export emits the expected RDF classes",
        )
        st, cim_reloaded = c.post_json(
            "/api/session/load_cim_dist",
            {"xml_string": cim_xml},
        )
        reloaded_counts = cim_reloaded.get("counts", {})
        chk.check(
            st == 200 and
            reloaded_counts.get("ac_buses") == cim_counts.get("ac_buses") and
            reloaded_counts.get("ac_branches") == cim_counts.get("ac_branches") and
            reloaded_counts.get("loads") == cim_counts.get("loads"),
            "exported distribution CIM re-imports without topology loss",
        )

        print("3. MATPOWER OPF regression cases")
        opf_payload = {
            "solver": "parity",
            "network_model": "balanced_aggregate",
            "constraints": {
                "branch_limits": True,
                "converter_capacity": True,
                "converter_current": True,
                "converter_modulation": True,
            },
            "options": {
                "max_inner_iterations": 400,
                "max_outer_iterations": 8,
                "feasibility_tol": 1e-6,
                "stationarity_tol": 1e-6,
            },
            "check_consistency": True,
        }
        for case_name, expected_buses in (("case9.m", 9), ("case30.m", 30)):
            st, body = c.post_json("/api/session/load_matpower",
                                   {"filename": case_name})
            counts = body.get("counts", {})
            chk.check(st == 200 and counts.get("ac_buses") == expected_buses,
                      f"load_matpower {case_name} -> {counts.get('ac_buses')} AC buses")

            st, opf = c.post_json("/api/session/opf", opf_payload)
            consistent = opf.get("consistency", {}).get("consistent")
            chk.check(st == 200 and opf.get("converged") is True,
                      f"opf {case_name} converged={opf.get('converged')} status={opf.get('status')}")
            chk.check(opf.get("post_pf", {}).get("converged") is True and consistent is True,
                      f"opf->pf {case_name} post_pf={opf.get('post_pf', {}).get('converged')} consistent={consistent}")
            chk.check(opf.get("options_effective", {}).get("max_inner_iterations") == 400,
                      f"opf options echoed for {case_name}")

        print("3a. power_system Canvas reprojection contract")
        power_system_path = (
            REPO_ROOT / "external_data" / "classical_example" / "power_system.json"
        )
        power_system = power_system_path.read_text(encoding="utf-8")
        st, _ = c.post_json(
            "/api/session/load_json_string", {"json_string": power_system}
        )
        st, pf = c.post_json(
            "/api/session/pf",
            {
                "method": "ac_newton",
                "options": {"enable_converter_coordination_check": False},
            },
        )
        pf_components = pf.get("component_results", [])
        pf_grid = next(
            (row for row in pf_components if row.get("canvas_type") == "external_grid"),
            None,
        )
        pf_cb34 = next(
            (
                row
                for row in pf.get("ac_circuit_breaker_flows", [])
                if row.get("index") == 1 and row.get("position") == 1
            ),
            None,
        )
        pf_source_cb = next(
            (
                row
                for row in pf.get("ac_circuit_breaker_flows", [])
                if row.get("index") == 0 and row.get("position") == 0
            ),
            None,
        )
        chk.check(
            st == 200 and pf.get("converged") is True and pf_grid is not None,
            "PF Canvas payload contains Grid 1 attribution",
        )
        chk.check(
            pf_cb34 is not None
            and abs(float(pf_cb34.get("pf_mw", 0.0))) > 1.0e-6
            and abs(float(pf_cb34.get("qf_mvar", 0.0))) > 1.0e-6,
            f"PF CB34 terminal P/Q={pf_cb34}",
        )
        chk.check(
            pf_source_cb is not None
            and pf_source_cb.get("source") == "same_bus_external_grid_cut"
            and abs(float(pf_source_cb.get("pf_mw", 0.0))) > 1.0e-6
            and abs(float(pf_source_cb.get("qf_mvar", 0.0))) > 1.0e-6,
            f"PF source-side CB terminal P/Q={pf_source_cb}",
        )
        pf_bus1 = next(
            (
                row
                for row in (pf.get("power_balance_diagnostics", {}).get("all_buses", []))
                if row.get("domain") == "AC" and row.get("bus") == 1
            ),
            None,
        )
        chk.check(
            pf_bus1 is not None
            and abs(float(pf_bus1.get("residual_kw", 1.0))) <= 1.0
            and abs(float(pf_bus1.get("residual_kvar", 1.0))) <= 1.0,
            f"PF Bus 1 P/Q ledger residual={pf_bus1}",
        )

        st, opf = c.post_json("/api/session/opf", opf_payload)
        post_pf = opf.get("post_pf", {})
        post_components = post_pf.get("component_results", [])
        opf_grid = next(
            (row for row in post_components if row.get("canvas_type") == "external_grid"),
            None,
        )
        opf_cb34 = next(
            (
                row
                for row in post_pf.get("ac_circuit_breaker_flows", [])
                if row.get("index") == 1 and row.get("position") == 1
            ),
            None,
        )
        opf_source_cb = next(
            (
                row
                for row in post_pf.get("ac_circuit_breaker_flows", [])
                if row.get("index") == 0 and row.get("position") == 0
            ),
            None,
        )
        chk.check(
            st == 200
            and opf.get("converged") is True
            and opf_grid is not None
            and len(post_pf.get("external_grid_p_mw", [])) == 1
            and len(post_pf.get("external_grid_q_mvar", [])) == 1,
            "OPF post-PF Canvas payload contains Grid 1 P/Q",
        )
        chk.check(
            opf_cb34 is not None
            and abs(float(opf_cb34.get("pf_mw", 0.0))) > 1.0e-6
            and abs(float(opf_cb34.get("qf_mvar", 0.0))) > 1.0e-6,
            f"OPF CB34 terminal P/Q={opf_cb34}",
        )
        chk.check(
            opf_source_cb is not None
            and opf_source_cb.get("source") == "same_bus_external_grid_cut"
            and abs(float(opf_source_cb.get("pf_mw", 0.0))) > 1.0e-6
            and abs(float(opf_source_cb.get("qf_mvar", 0.0))) > 1.0e-6,
            f"OPF source-side CB terminal P/Q={opf_source_cb}",
        )
        opf_bus1 = next(
            (
                row
                for row in (post_pf.get("power_balance_diagnostics", {}).get("all_buses", []))
                if row.get("domain") == "AC" and row.get("bus") == 1
            ),
            None,
        )
        chk.check(
            opf_bus1 is not None
            and abs(float(opf_bus1.get("residual_kw", 1.0))) <= 1.0
            and abs(float(opf_bus1.get("residual_kvar", 1.0))) <= 1.0
            and abs(float(opf_bus1.get("net_injection_mvar", 0.0))) > 1.0e-6
            and abs(float(opf_bus1.get("terminal_export_mvar", 0.0))) > 1.0e-6,
            f"OPF Bus 1 P/Q ledger residual={opf_bus1}",
        )

        print("3b. cached Canvas playback frame contracts")
        st, tspf = c.post_json(
            "/api/session/run_ts_pf",
            {"num_steps": 3, "skip_uc": True, "run_opf": False,
             "enable_external_grid": True},
        )
        st_frame, tspf_frame = c.get("/api/session/tspf/frame?step=0")
        tspf_grid = next(
            (row for row in tspf_frame.get("component_results", [])
             if row.get("canvas_type") == "external_grid"),
            None,
        )
        tspf_source_cb = next(
            (row for row in tspf_frame.get("ac_circuit_breaker_flows", [])
             if row.get("source") == "same_bus_external_grid_cut"),
            None,
        )
        chk.check(
            st == 200 and st_frame == 200
            and tspf_frame.get("schema") == "canvas_frame_v1"
            and tspf_frame.get("count") == 3
            and tspf_frame.get("converged") is True,
            "TSPF frame endpoint returns a converged canvas_frame_v1",
        )
        chk.check(
            tspf_grid is not None and tspf_source_cb is not None
            and abs(float(tspf_source_cb.get("pf_mw", 0.0))) > 1.0e-6
            and abs(float(tspf_source_cb.get("qf_mvar", 0.0))) > 1.0e-6
            and len(tspf_grid.get("metrics", [])) >= 2,
            f"TSPF Grid/source-CB P/Q={tspf_source_cb}",
        )
        chk.check(
            tspf_frame.get("capabilities", {}).get("power_balance") is True
            and isinstance(tspf_frame.get("power_balance_diagnostics", {}).get("all_buses"), list),
            "TSPF frame includes authored-space bus P/Q diagnostics",
        )

        st, transient = c.post_json(
            "/api/session/run_transient",
            {"t_start_s": 0.0, "t_end_s": 0.03, "dt_s": 0.01,
             "record_every_step": True, "record_device_outputs": True,
             "run_power_flow_initialization": True,
             "enforce_voltage_health_check": False,
             "snapshot_budget": 16},
        )
        st_frame, transient_frame = c.get("/api/session/transient/frame?index=1")
        chk.check(
            st == 200 and st_frame == 200
            and transient_frame.get("schema") == "canvas_frame_v1"
            and transient_frame.get("capabilities", {}).get("branch_power") is False
            and transient_frame.get("geo_ac_branches") == []
            and transient_frame.get("geo_dc_branches") == []
            and transient_frame.get("ac_circuit_breaker_flows") == []
            and len(transient_frame.get("frequency_hz", [])) > 0
            and len(transient_frame.get("component_results", [])) > 0,
            "Transient frame exposes real telemetry without fabricated static branch flow",
        )

        if args.skip_etap:
            print("4-9. ETAP export/import checks skipped")
        else:
            print("4. reload built-in ieee14_acdc for ETAP export")
            st, body = c.post_json("/api/session/load_builtin", {"case": "ieee14_acdc"})
            counts = body.get("counts", {})
            chk.check(st == 200 and counts.get("ac_buses") == 14,
                      f"load_builtin -> {counts.get('ac_buses')} AC buses")

            print("5. export ETAP .xlsx (binary)")
            st, hdrs, xlsx = c.post_bytes("/api/session/export_etap", b"{}")
            ctype = hdrs.get("Content-Type", "")
            is_xlsx = xlsx[:2] == b"PK" and "spreadsheetml" in ctype
            chk.check(st == 200 and is_xlsx,
                      f"export_etap -> {len(xlsx)} bytes, ctype ok={is_xlsx}")

            print("6. re-import the exported workbook (.xlsx upload)")
            st, _, raw = c.post_bytes("/api/session/load_etap_xlsx", xlsx)
            re_counts = json.loads(raw).get("counts", {})
            chk.check(st == 200 and re_counts.get("ac_buses") == 14,
                      f"load_etap_xlsx -> {re_counts.get('ac_buses')} AC buses")

            print("7. import a tiny native ETAP XML")
            xml = (
                '<PROJECT><COMPONENTS>'
                '<BUS ID="B1" NominalkV="110" InService="true"/>'
                '<BUS ID="B2" NominalkV="20" InService="true"/>'
                '<UTIL ID="U1" Bus="B1" KV="110" OpVMag="100" PosR="0.1" PosX="1.0"/>'
                '<XFORM2W ID="T1" FromBus="B1" ToBus="B2" PrimkV="110" SeckV="20" '
                'AnsiMVA="40000" AnsiPosZ="10.5" AnsiPosXR="20"/>'
                '<LUMPEDLOAD ID="LD1" Bus="B2" MVA="10" PF="90"/>'
                '</COMPONENTS></PROJECT>'
            )
            st, body = c.post_json("/api/session/load_etap_xml", {"xml_string": xml})
            xml_counts = body.get("counts", {})
            chk.check(st == 200 and xml_counts.get("ac_buses") == 2,
                      f"load_etap_xml -> {xml_counts.get('ac_buses')} AC buses")

            print("8. power flow on the imported XML system")
            st, pf = c.post_json("/api/session/pf", {"method": "pure_ac", "options": {}})
            chk.check(st == 200 and pf.get("converged") is True,
                      f"pf converged={pf.get('converged')}")

            print("9. short-circuit on the imported system")
            st, sc = c.post_json("/api/session/sc",
                                 {"options": {"fault_type": "3ph", "c_factor": 1.1}})
            chk.check(st == 200 and len(sc.get("bus_results", [])) >= 1,
                      f"sc -> {len(sc.get('bus_results', []))} bus results")

        maybe_run_opendss_ieee13_smoke(c, chk)

    finally:
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()

    print(f"\n{chk.checks - chk.failures}/{chk.checks} checks passed")
    return 0 if chk.failures == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
