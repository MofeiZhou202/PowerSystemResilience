#!/usr/bin/env python3
"""API-level test for POST /api/session/topology_window (run_gui_server).

Covers the spatial-window contract added with the resident PowerSystemGraph +
uniform-grid bus spatial index:
  - bbox filtering returns only in-window, geo-referenced buses;
  - edges are returned only when both endpoints are inside the window;
  - lod 0/1 aggregation semantics mirror web/js/core/network_overview.js;
  - buses without coordinates are excluded and declared via
    coordinate_coverage (never a silent empty view);
  - request validation (missing/inverted bbox, bad lod).

No third-party dependencies (urllib + subprocess from the standard library).

Usage:
    python3 tools/gui_topology_window_e2e.py [--server <path>] [--data-dir <dir>]
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
            with urllib.request.urlopen(req, timeout=30) as r:
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


# Authored hybrid system: three geo-referenced AC buses (two area/zone
# groups), one AC bus and one DC bus without coordinates (model-default 0,0),
# one geo-referenced DC bus.  Branches chain 1-2-3-4 so that the window can
# include/exclude edges by endpoint membership.
TEST_SYSTEM = {
    "name": "topology_window_test",
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
            {"index": 1, "from_bus": 1, "to_bus": 2, "r_pu": 0.01, "x_pu": 0.1},
            {"index": 2, "from_bus": 2, "to_bus": 3, "r_pu": 0.01, "x_pu": 0.1},
            {"index": 3, "from_bus": 3, "to_bus": 4, "r_pu": 0.01, "x_pu": 0.1},
        ],
    },
    "dc": {
        "buses": [
            {"index": 1, "base_kv": 10.0,
             "latitude": 34.05, "longitude": 108.05, "area": 1, "zone": 1},
            {"index": 2, "base_kv": 10.0},
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

        print("1. load authored geo-referenced hybrid system")
        st, d = c.post_json("/api/session/load_json_string",
                            {"json_string": json.dumps(TEST_SYSTEM)})
        chk.check(st == 200, f"load_json_string status {st}")
        # Step-1 contract: load-path responses still embed parseable _raw_json.
        raw = json.loads(d.get("_raw_json", ""))
        chk.check(len(raw.get("ac", {}).get("buses", [])) == 4,
                  "_raw_json is parseable and carries the loaded system")

        print("2. lod=2 full bbox: window membership + coverage declaration")
        st, d = c.post_json("/api/session/topology_window", FULL_BBOX)
        chk.check(st == 200, f"topology_window status {st}")
        chk.check(d.get("schema") == "topology_window_v1", "schema tag")
        cov = d.get("coordinate_coverage", {})
        chk.check(cov.get("ac_buses_total") == 4
                  and cov.get("ac_buses_with_coordinates") == 3
                  and cov.get("dc_buses_total") == 2
                  and cov.get("dc_buses_with_coordinates") == 1
                  and cov.get("complete") is False,
                  f"coordinate_coverage declares missing coords: {cov}")
        nodes = {(n["domain"], n["index"]): n for n in d.get("nodes", [])}
        chk.check(set(nodes) == {("AC", 1), ("AC", 2), ("AC", 3), ("DC", 1)},
                  f"only geo-referenced in-window buses returned: {sorted(nodes)}")
        chk.check(nodes.get(("AC", 1), {}).get("type") == "SLACK",
                  "node type string present")
        edges = {(e["from_domain"], e["from"], e["to_domain"], e["to"])
                 for e in d.get("edges", [])}
        chk.check(edges == {("AC", 1, "AC", 2), ("AC", 2, "AC", 3)},
                  f"edge 3-4 excluded (endpoint without coords): {sorted(edges)}")
        chk.check(any("no coordinates" in m
                      for m in d.get("model_limitations", [])),
                  "limitation declares excluded buses")

        print("3. lod=2 tight bbox around bus 1")
        st, d = c.post_json("/api/session/topology_window", {
            "min_x": 107.9, "min_y": 33.9, "max_x": 108.01, "max_y": 34.01})
        chk.check(st == 200, f"tight bbox status {st}")
        only = {(n["domain"], n["index"]) for n in d.get("nodes", [])}
        chk.check(only == {("AC", 1)}, f"tight bbox returns only bus 1: {only}")
        chk.check(d.get("edges") == [], "no edge with both endpoints in tight bbox")

        print("4. lod=1 aggregation by domain/area/zone")
        st, d = c.post_json("/api/session/topology_window",
                            {**FULL_BBOX, "lod": 1})
        chk.check(st == 200, f"lod=1 status {st}")
        groups = {g["key"]: g for g in d.get("nodes", [])}
        chk.check(set(groups) == {"AC:area:1:zone:1", "AC:area:2:zone:1",
                                  "DC:area:1:zone:1"},
                  f"lod=1 groups: {sorted(groups)}")
        g11 = groups.get("AC:area:1:zone:1", {})
        chk.check(g11.get("count") == 2
                  and abs(g11.get("x", 0) - 108.05) < 1e-9
                  and abs(g11.get("y", 0) - 34.05) < 1e-9,
                  f"lod=1 centroid of buses 1+2: {g11}")
        agg = {(e["source"], e["target"]): e["count"]
               for e in d.get("edges", [])}
        chk.check(agg == {("AC:area:1:zone:1", "AC:area:2:zone:1"): 1},
                  f"lod=1 aggregate edges: {agg}")

        print("5. lod=0 aggregation by domain")
        st, d = c.post_json("/api/session/topology_window",
                            {**FULL_BBOX, "lod": 0})
        chk.check(st == 200, f"lod=0 status {st}")
        groups = {g["key"]: g for g in d.get("nodes", [])}
        chk.check(set(groups) == {"AC", "DC"}, f"lod=0 groups: {sorted(groups)}")
        chk.check(groups.get("AC", {}).get("count") == 3
                  and groups.get("DC", {}).get("count") == 1,
                  "lod=0 member counts")
        chk.check(groups.get("AC", {}).get("area") is None,
                  "lod=0 omits area/zone")
        chk.check(d.get("edges") == [],
                  "lod=0: intra-domain edges collapse into their group")

        print("6. request validation")
        st, _ = c.post_json("/api/session/topology_window",
                            {"min_x": 0.0, "min_y": 0.0, "max_x": 1.0})
        chk.check(st == 400, "missing max_y rejected")
        st, _ = c.post_json("/api/session/topology_window",
                            {"min_x": 2.0, "min_y": 0.0,
                             "max_x": 1.0, "max_y": 1.0})
        chk.check(st == 400, "inverted bbox rejected")
        st, _ = c.post_json("/api/session/topology_window",
                            {**FULL_BBOX, "lod": 3})
        chk.check(st == 400, "lod=3 rejected")

        print("7. builtin geo-referenced case: full coverage over its bounds")
        st, d = c.post_json("/api/session/load_builtin",
                            {"case": "ieee24_3area_acdc_expanded"})
        chk.check(st == 200, f"load_builtin status {st}")
        st, d = c.post_json("/api/session/topology_window", {
            "min_x": -180.0, "min_y": -90.0, "max_x": 180.0, "max_y": 90.0})
        chk.check(st == 200, f"topology_window on builtin case status {st}")
        cov = d.get("coordinate_coverage", {})
        chk.check(cov.get("ac_buses_total") == 24
                  and cov.get("ac_buses_with_coordinates") == 24
                  and cov.get("dc_buses_total") == 8
                  and cov.get("dc_buses_with_coordinates") == 8
                  and cov.get("complete") is True,
                  f"builtin case fully geo-referenced: {cov}")
        chk.check(len(d.get("nodes", [])) == 32,
                  "world bbox returns every bus")
        st, d = c.post_json("/api/session/topology_window", {
            "min_x": -179.0, "min_y": -89.0, "max_x": -178.0, "max_y": -88.0})
        chk.check(st == 200 and d.get("nodes") == [] and d.get("edges") == [],
                  "bbox outside all coordinates returns empty (declared, not "
                  "fabricated)")

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
