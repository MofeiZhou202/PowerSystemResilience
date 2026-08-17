#!/usr/bin/env python3
"""API-level test for POST /api/session/result_window (run_gui_server).

Covers the viewport-scoped power-flow result contract:
  - 409 with an explanatory body when no PF result is cached (never a silent
    empty view);
  - bbox filtering over the resident spatial index (fast path) and the
    solved-system linear scan after the model is replaced (stale path);
  - coordinate_coverage declaration for buses without coordinates;
  - vm/va/branch-flow values identical to the full /api/session/pf response
    (sampled comparison);
  - result_meta: converged flag and result_matches_current_system, including
    the lag declaration after the model is edited post-solve.

No third-party dependencies (urllib + subprocess from the standard library).

Usage:
    python3 tools/gui_result_window_e2e.py [--server <path>] [--data-dir <dir>]
"""
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

    def post_json(self, path: str, payload: dict | None = None):
        body = json.dumps(payload or {}).encode()
        req = urllib.request.Request(self.base + path, data=body, method="POST")
        req.add_header("Content-Type", "application/json")
        try:
            with urllib.request.urlopen(req, timeout=60) as r:
                return r.status, json.loads(r.read())
        except urllib.error.HTTPError as e:
            return e.code, json.loads(e.read())


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


# Three geo-referenced AC buses (slack + two PQ, one loaded), plus a fourth AC
# bus without coordinates (model-default lat/lon 0,0).  Branches chain 1-2-3-4
# so window filtering includes/excludes branches by endpoint membership.
TEST_SYSTEM = {
    "name": "result_window_test",
    "base_mva": 100.0,
    "ac": {
        "buses": [
            {"index": 1, "bus_type": "SLACK", "base_kv": 110.0,
             "latitude": 34.0, "longitude": 108.0, "area": 1, "zone": 1},
            {"index": 2, "bus_type": "PQ", "base_kv": 110.0,
             "latitude": 34.1, "longitude": 108.1, "area": 1, "zone": 1},
            {"index": 3, "bus_type": "PQ", "base_kv": 110.0,
             "latitude": 35.0, "longitude": 109.0, "area": 2, "zone": 1},
            {"index": 4, "bus_type": "PQ", "base_kv": 110.0,
             "area": 2, "zone": 2},
        ],
        "branches": [
            {"index": 1, "from_bus": 1, "to_bus": 2,
             "r_pu": 0.01, "x_pu": 0.1, "rate_a_mva": 50.0},
            {"index": 2, "from_bus": 2, "to_bus": 3,
             "r_pu": 0.01, "x_pu": 0.1, "rate_a_mva": 50.0},
            {"index": 3, "from_bus": 3, "to_bus": 4,
             "r_pu": 0.01, "x_pu": 0.1, "rate_a_mva": 50.0},
        ],
        "loads": [
            {"index": 0, "bus": 2, "p_mw": 10.0, "q_mvar": 3.0},
            {"index": 1, "bus": 3, "p_mw": 5.0, "q_mvar": 1.0},
        ],
        "generators": [
            {"index": 0, "bus": 1, "pg_mw": 15.0},
        ],
    },
}

FULL_BBOX = {"min_x": 100.0, "min_y": 30.0, "max_x": 115.0, "max_y": 40.0}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--server", default=None)
    ap.add_argument("--data-dir", default=str(REPO_ROOT / "data"))
    ap.add_argument("--port", type=int, default=0)
    args = ap.parse_args()

    server = find_server(args.server)
    port = args.port or free_port()
    base = f"http://127.0.0.1:{port}"
    proc = subprocess.Popen(
        [str(server), "--port", str(port),
         "--data-dir", args.data_dir, "--matpower-dir", args.data_dir],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        if not wait_up(base):
            sys.exit("server did not come up")
        c = Client(base)
        chk = Checker()

        print("1. no cached power flow -> 409 with explanatory body")
        st, d = c.post_json("/api/session/result_window", FULL_BBOX)
        chk.check(st == 409, f"result_window without PF -> 409 (got {st})")
        chk.check(d.get("error") == "no_cached_power_flow"
                  and "power flow" in d.get("hint", ""),
                  f"409 body explains the missing result: {d.get('error')}")

        print("2. load authored system and solve PF")
        st, d = c.post_json("/api/session/load_json_string",
                            {"json_string": json.dumps(TEST_SYSTEM)})
        chk.check(st == 200, f"load_json_string status {st}")
        st, pf = c.post_json("/api/session/pf", {})
        chk.check(st == 200 and pf.get("converged") is True,
                  f"pf converged (status {st}, converged={pf.get('converged')})")

        print("3. full bbox: window membership, coverage, result_meta")
        st, d = c.post_json("/api/session/result_window", FULL_BBOX)
        chk.check(st == 200, f"result_window status {st}")
        chk.check(d.get("schema") == "result_window_v1", "schema tag")
        cov = d.get("coordinate_coverage", {})
        chk.check(cov.get("ac_buses_total") == 4
                  and cov.get("ac_buses_with_coordinates") == 3
                  and cov.get("complete") is False,
                  f"coverage declares bus 4 without coordinates: {cov}")
        nodes = {(n["domain"], n["index"]): n for n in d.get("nodes", [])}
        chk.check(set(nodes) == {("AC", 1), ("AC", 2), ("AC", 3)},
                  f"only geo-referenced in-window buses: {sorted(nodes)}")
        meta = d.get("result_meta", {})
        chk.check(meta.get("source") == "last_power_flow"
                  and meta.get("converged") is True
                  and meta.get("result_matches_current_system") is True,
                  f"result_meta: {meta}")
        branches = {(b["from"], b["to"]): b for b in d.get("branches", [])}
        chk.check(set(branches) == {(1, 2), (2, 3)},
                  "branch to coordinate-less bus 4 excluded")

        print("4. values identical to the full PF response (sampled)")
        # vm/va/vdc arrays in the full PF response align by position with the
        # solved system's ac.buses order; branch_abs[i] == |branch_flows[i].pf_mw|.
        pos_of = {}
        for pos, b in enumerate(
                json.loads(json.dumps(TEST_SYSTEM))["ac"]["buses"]):
            pos_of[b["index"]] = pos
        ok_vm = ok_va = True
        for (dom, idx), node in nodes.items():
            if dom != "AC":
                continue
            pos = pos_of[idx]
            if abs(node["vm_pu"] - pf["vm"][pos]) > 1e-12:
                ok_vm = False
            if abs(node["va_rad"] - pf["va"][pos]) > 1e-12:
                ok_va = False
        chk.check(ok_vm, "vm_pu matches full PF vm by bus position")
        chk.check(ok_va, "va_rad matches full PF va by bus position")
        ok_br = True
        for (f, t), row in branches.items():
            bpos = next(i for i, b in enumerate(TEST_SYSTEM["ac"]["branches"])
                        if b["from_bus"] == f and b["to_bus"] == t)
            if abs(abs(row["pf_mw"]) - pf["branch_abs"][bpos]) > 1e-9:
                ok_br = False
            expected_loading = 100.0 * (
                (row["pf_mw"] ** 2 + row["qf_mvar"] ** 2) ** 0.5) / 50.0
            if abs(row.get("loading_pct", -1) - expected_loading) > 1e-9:
                ok_br = False
        chk.check(ok_br, "branch pf_mw/loading_pct consistent with full PF")

        print("5. tight bbox around bus 1")
        st, d = c.post_json("/api/session/result_window", {
            "min_x": 107.9, "min_y": 33.9, "max_x": 108.01, "max_y": 34.01})
        only = {(n["domain"], n["index"]) for n in d.get("nodes", [])}
        chk.check(st == 200 and only == {("AC", 1)},
                  f"tight bbox returns only bus 1: {only}")
        chk.check(d.get("branches") == [],
                  "no branch with both endpoints in tight bbox")

        print("6. model edited after solve -> lag declaration (stale path)")
        st, _ = c.post_json("/api/session/update_carbon_factors", {
            "generators": []})
        chk.check(st == 200, "update_carbon_factors accepted")
        st, d = c.post_json("/api/session/result_window", FULL_BBOX)
        meta = d.get("result_meta", {})
        chk.check(st == 200
                  and meta.get("result_matches_current_system") is False,
                  f"stale result flagged: {meta}")
        chk.check(any("lag behind current edits" in m
                      for m in d.get("model_limitations", [])),
                  "limitation declares results may lag")
        # Stale path still returns windowed data via the linear-scan fallback.
        chk.check(len(d.get("nodes", [])) == 3,
                  "stale path still serves windowed nodes")

        print("7. hybrid built-in: DC buses appear with vdc as vm_pu")
        st, d = c.post_json("/api/session/load_builtin",
                            {"case": "ieee24_3area_acdc_expanded"})
        chk.check(st == 200, f"load_builtin status {st}")
        st, pf = c.post_json("/api/session/pf", {})
        chk.check(st == 200 and pf.get("converged") is True,
                  f"built-in pf converged (converged={pf.get('converged')})")
        st, d = c.post_json("/api/session/result_window", {
            "min_x": -180.0, "min_y": -90.0, "max_x": 180.0, "max_y": 90.0})
        domains = {}
        for n in d.get("nodes", []):
            domains[n["domain"]] = domains.get(n["domain"], 0) + 1
        chk.check(domains.get("AC") == 24 and domains.get("DC") == 8,
                  f"world bbox returns 24 AC + 8 DC buses: {domains}")
        dc_ok = all("vm_pu" in n and "va_rad" not in n
                    for n in d.get("nodes", []) if n["domain"] == "DC")
        chk.check(dc_ok, "DC nodes carry vm_pu (vdc) and no va_rad")

        print(f"\n{chk.checks} checks, {chk.failures} failures")
        return 1 if chk.failures else 0
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()


if __name__ == "__main__":
    sys.exit(main())
