#!/usr/bin/env python3
"""Cross-check HySim-XJTU-HRPES AC power flow against MATPOWER.

Runs MATPOWER (via one MATLAB batch session) and the C++ tool
`matpower_pf_compare` over the same case files, then compares
  - accuracy: max/mean |dVm| (p.u.) and |dVa| (deg) on matched bus ids
  - efficiency: MATPOWER runpf elapsed time vs hacdcpf solve time

Usage:
  python3 tools/matpower_benchmark.py                 # full suite
  python3 tools/matpower_benchmark.py --cases case14 case118
  python3 tools/matpower_benchmark.py --skip-matlab   # reuse existing MATLAB CSVs

Outputs (under --out, default output/benchmarks/matpower_pf/):
  matpower_summary.csv          raw MATPOWER per-case results + timings
  <case>.matpower_bus.csv       per-bus MATPOWER voltages
  hysim_summary.csv             raw hacdcpf per-case results + timings
  comparison.csv                merged accuracy + timing comparison
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MATLAB = "/Applications/MATLAB_R2025b.app/bin/matlab"
HYSIM_BIN = os.path.join(ROOT, "build", "macos-release", "matpower_pf_compare")
CASE_DIR = os.path.join(ROOT, "external_data", "matpower")
HYSIM_TIMEOUT_S = 1800


def collect_cases(only: list[str] | None) -> list[str]:
    names = []
    for f in sorted(os.listdir(CASE_DIR)):
        if not f.endswith(".m"):
            continue
        if f.startswith(("contab_", "scenarios_")):
            continue
        names.append(f[:-2])
    if only:
        missing = [c for c in only if c not in names]
        if missing:
            sys.exit(f"unknown cases: {missing}")
        names = [c for c in names if c in set(only)]
    return names


def run_matlab(cases: list[str], out_dir: str) -> None:
    list_path = os.path.join(out_dir, "caselist.txt")
    with open(list_path, "w") as fh:
        for c in cases:
            fh.write(os.path.join(CASE_DIR, c + ".m") + "\n")
    cmd = [
        MATLAB, "-batch",
        f"addpath('{os.path.join(ROOT, 'tools')}'); "
        f"matpower_benchmark('{list_path}','{out_dir}')",
    ]
    print(f"[matlab] running {len(cases)} cases ...", flush=True)
    t0 = time.time()
    proc = subprocess.run(cmd, capture_output=True, text=True)
    print(f"[matlab] finished in {time.time()-t0:.1f}s, rc={proc.returncode}",
          flush=True)
    if proc.returncode != 0:
        print(proc.stdout[-3000:])
        print(proc.stderr[-3000:], file=sys.stderr)


def run_hysim(cases: list[str], out_dir: str) -> dict[str, dict]:
    results: dict[str, dict] = {}
    for i, c in enumerate(cases, 1):
        path = os.path.join(CASE_DIR, c + ".m")
        t0 = time.time()
        try:
            proc = subprocess.run(
                [HYSIM_BIN, path], capture_output=True, text=True,
                timeout=HYSIM_TIMEOUT_S)
            wall_s = time.time() - t0
            data = json.loads(proc.stdout)
            data["wall_s"] = wall_s
        except subprocess.TimeoutExpired:
            data = {"converged": False, "error": "timeout",
                    "wall_s": time.time() - t0}
        except json.JSONDecodeError:
            data = {"converged": False,
                    "error": f"bad output: {proc.stdout[:200]}",
                    "wall_s": time.time() - t0}
        results[c] = data
        flag = "ok" if data.get("converged") else "FAIL"
        print(f"[hysim {i}/{len(cases)}] {c}: {flag} "
              f"solve={data.get('solve_ms', float('nan')):.1f}ms", flush=True)
    with open(os.path.join(out_dir, "hysim_summary.json"), "w") as fh:
        json.dump(results, fh)
    return results


def load_matpower_summary(out_dir: str) -> dict[str, dict]:
    path = os.path.join(out_dir, "matpower_summary.csv")
    out = {}
    with open(path, newline="") as fh:
        for row in csv.DictReader(fh):
            out[row["case"]] = row
    return out


def load_matpower_bus(out_dir: str, case: str) -> dict[int, tuple[float, float]]:
    path = os.path.join(out_dir, f"{case}.matpower_bus.csv")
    out = {}
    with open(path, newline="") as fh:
        for row in csv.DictReader(fh):
            out[int(row["bus_i"])] = (float(row["vm"]), float(row["va_deg"]))
    return out


def compare(cases: list[str], out_dir: str,
            mp: dict[str, dict], hy: dict[str, dict]) -> list[dict]:
    rows = []
    for c in cases:
        row: dict = {"case": c}
        m = mp.get(c)
        h = hy.get(c, {})
        row["nbus"] = m["nbus"] if m else ""
        row["nbranch"] = m["nbranch"] if m else ""
        row["mp_success"] = m["success"] if m else ""
        row["mp_et_s"] = m["runpf_et_s"] if m else ""
        row["mp_loadcase_s"] = m["loadcase_s"] if m else ""
        row["hy_converged"] = int(bool(h.get("converged")))
        row["hy_iter"] = h.get("iterations", "")
        row["hy_solve_ms"] = h.get("solve_ms", "")
        row["hy_parse_ms"] = h.get("parse_ms", "")
        row["hy_wall_s"] = round(h.get("wall_s", float("nan")), 3)

        dvm_max = dvm_mean = dva_max = dva_mean = ""
        n_matched = 0
        if (m and m["success"] == "1" and h.get("converged")):
            ref = load_matpower_bus(out_dir, c)
            ids = h.get("bus_ids", [])
            vm, va = h.get("vm", []), h.get("va", [])
            dvm, dva = [], []
            for i, bid in enumerate(ids):
                if bid in ref and i < len(vm):
                    dvm.append(abs(vm[i] - ref[bid][0]))
                    da = abs(math.degrees(va[i]) - ref[bid][1])
                    dva.append(min(da, 360.0 - da))
            n_matched = len(dvm)
            if dvm:
                dvm_max = f"{max(dvm):.3e}"
                dvm_mean = f"{sum(dvm)/len(dvm):.3e}"
                dva_max = f"{max(dva):.3e}"
                dva_mean = f"{sum(dva)/len(dva):.3e}"
        row["n_matched_buses"] = n_matched
        row["max_abs_dvm_pu"] = dvm_max
        row["mean_abs_dvm_pu"] = dvm_mean
        row["max_abs_dva_deg"] = dva_max
        row["mean_abs_dva_deg"] = dva_mean
        try:
            speedup = (float(row["mp_et_s"]) * 1000.0) / float(row["hy_solve_ms"])
            row["speedup_solve"] = f"{speedup:.2f}"
        except (TypeError, ValueError, ZeroDivisionError):
            row["speedup_solve"] = ""
        note = []
        if m and m.get("note") and m["note"] != "ok":
            note.append("mp:" + m["note"])
        if h.get("error"):
            note.append("hy:" + h["error"])
        if m and m["success"] == "1" and h.get("converged") and \
                n_matched != int(m["nbus"]):
            note.append(f"bus_mismatch:{n_matched}/{m['nbus']}")
        row["note"] = ";".join(note)
        rows.append(row)
    return rows


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--cases", nargs="*", default=None)
    ap.add_argument("--out", default=os.path.join(
        ROOT, "output", "benchmarks", "matpower_pf"))
    ap.add_argument("--skip-matlab", action="store_true")
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)
    cases = collect_cases(args.cases)
    print(f"{len(cases)} cases")

    if not args.skip_matlab:
        run_matlab(cases, args.out)
    hy = run_hysim(cases, args.out)

    mp = load_matpower_summary(args.out)
    rows = compare(cases, args.out, mp, hy)
    out_csv = os.path.join(args.out, "comparison.csv")
    with open(out_csv, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    print(f"wrote {out_csv}")

    hdr = (f"{'case':<22}{'nbus':>6} {'mp_ok':>5} {'hy_ok':>5} "
           f"{'max|dVm|':>10} {'max|dVa|deg':>11} {'mp_et_s':>9} "
           f"{'hy_ms':>9} {'speedup':>8}")
    print(hdr)
    print("-" * len(hdr))
    for r in rows:
        print(f"{r['case']:<22}{r['nbus']:>6} {r['mp_success']:>5} "
              f"{r['hy_converged']:>5} {r['max_abs_dvm_pu']:>10} "
              f"{r['max_abs_dva_deg']:>11} {r['mp_et_s']:>9} "
              f"{str(r['hy_solve_ms'])[:9]:>9} {r['speedup_solve']:>8}")


if __name__ == "__main__":
    main()
