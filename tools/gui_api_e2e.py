#!/usr/bin/env python3
"""End-to-end smoke test for the GUI/HTTP backend (run_gui_server).

Starts the server, then drives the same REST endpoints the web UI uses and
asserts on the responses.  Exercises the full ETAP import/export round-trip and
the analysis endpoints so a regression in the GUI backend fails loudly in CI.

No third-party dependencies (urllib + subprocess from the standard library).

Usage:
    python3 tools/gui_api_e2e.py [--server <path/to/run_gui_server>]
                                 [--data-dir <dir>] [--port <port>]

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


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--server")
    ap.add_argument("--data-dir", default=str(REPO_ROOT / "data"))
    ap.add_argument("--port", type=int, default=0)
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

        print("2. load built-in ieee14_acdc")
        st, body = c.post_json("/api/session/load_builtin", {"case": "ieee14_acdc"})
        counts = body.get("counts", {})
        chk.check(st == 200 and counts.get("ac_buses") == 14,
                  f"load_builtin -> {counts.get('ac_buses')} AC buses")

        print("3. MATPOWER OPF regression cases")
        opf_payload = {
            "solver": "parity",
            "constraints": {
                "branch_limits": True,
                "converter_capacity": True,
                "converter_current": True,
                "converter_modulation": True,
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
