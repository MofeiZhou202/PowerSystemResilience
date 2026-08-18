#!/usr/bin/env python3
"""Reproducible power-flow solver benchmark harness.

Drives ``matpower_pf_compare`` over a matrix of MATPOWER cases, parses the
profiling JSON it prints, and emits a CSV plus a human-readable table. Use it to
measure linear/nonlinear solver changes instead of eyeballing single runs.

Examples
--------
    # Default case matrix, 8 warm repeats, write CSV
    python3 tools/pf_solver_benchmark.py --repeat 8 --csv output/pf_bench.csv

    # A/B a single option toggle against the baseline
    python3 tools/pf_solver_benchmark.py --cases case9241pegase \\
        --variant baseline --variant "--no-condition-monitor"

The tool is read-only w.r.t. the repository: it only executes the prebuilt
benchmark binary and reads case files under ``data/``.
"""
from __future__ import annotations

import argparse
import json
import shlex
import statistics as st
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
DEFAULT_BIN = ROOT / "build" / "macos-release" / "matpower_pf_compare"

# Representative matrix: radial distribution, small/mid transmission, large meshed.
DEFAULT_CASES = [
    "case33bw", "case69", "case85", "case141",          # radial distribution
    "case118", "case300", "case1354pegase",             # small/mid transmission
    "case2869pegase", "case9241pegase", "case13659pegase",  # large meshed
]

# (csv column, JSON key) pairs pulled from each run.
FIELDS = [
    ("backend", "linear_backend"),
    ("nbus", "n_bus"),
    ("iters", "iterations"),
    ("converged", "converged"),
    ("fact_calls", "factorization_calls"),
    ("refac_acc", "numeric_refactor_accepted"),
    ("refac_att", "numeric_refactor_attempts"),
    ("refac_fallback", "numeric_refactor_fallbacks"),
    ("pattern_rebuilds", "jacobian_pattern_rebuilds"),
    ("lin_solve_ms", "linear_solve_ms_total"),
    ("eval_jac_ms", "eval_jacobian_ms_total"),
    ("linesearch_ms", "line_search_ms_total"),
    ("scaling_ms", "scaling_ms_total"),
    ("proj_ms", "projection_ms_total"),
    ("core_ms", "solver_core_ms_total"),
    ("facade_ms", "facade_ms_total"),
]


def run_case(binary: Path, case_path: Path, repeat: int, extra: list[str]) -> dict:
    cmd = [str(binary), str(case_path), "--repeat", str(repeat), *extra]
    out = subprocess.run(cmd, capture_output=True, text=True, check=True)
    return json.loads(out.stdout)


def warm_ms(d: dict) -> float:
    r = d.get("repeat_ms") or []
    return st.median(r) if r else float(d.get("solve_ms", float("nan")))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bin", type=Path, default=DEFAULT_BIN,
                    help="Path to matpower_pf_compare (default: build/macos-release).")
    ap.add_argument("--data", type=Path, default=ROOT / "data",
                    help="Directory holding <case>.m files.")
    ap.add_argument("--cases", nargs="*", default=DEFAULT_CASES,
                    help="Case names (without .m) to benchmark.")
    ap.add_argument("--repeat", type=int, default=8,
                    help="Warm-cache repeat solves per case (median reported).")
    ap.add_argument("--variant", action="append", default=None,
                    help="Extra CLI string(s) for matpower_pf_compare; 'baseline' means none. "
                         "Repeat to compare variants side by side.")
    ap.add_argument("--csv", type=Path, default=None, help="Optional CSV output path.")
    args = ap.parse_args()

    if not args.bin.exists():
        sys.stderr.write(f"error: benchmark binary not found: {args.bin}\n"
                         "build it: cmake --build build/macos-release --target matpower_pf_compare\n")
        return 2

    variants = args.variant or ["baseline"]
    rows: list[dict] = []
    for variant in variants:
        extra = [] if variant == "baseline" else shlex.split(variant)
        for case in args.cases:
            case_path = args.data / f"{case}.m"
            if not case_path.exists():
                sys.stderr.write(f"skip (missing): {case_path}\n")
                continue
            d = run_case(args.bin, case_path, args.repeat, extra)
            row = {"case": case, "variant": variant, "warm_ms": round(warm_ms(d), 3)}
            for col, key in FIELDS:
                row[col] = d.get(key)
            # Fraction of warm solve spent in the direct linear solve.
            wm = row["warm_ms"] or float("nan")
            row["lin_solve_pct"] = round(100.0 * (row["lin_solve_ms"] or 0.0) / wm, 1) if wm else None
            rows.append(row)

    cols = ["case", "variant", "nbus", "iters", "warm_ms", "lin_solve_ms",
            "lin_solve_pct", "eval_jac_ms", "scaling_ms", "core_ms",
            "fact_calls", "refac_acc", "backend"]
    widths = {c: max(len(c), *(len(str(r.get(c, ""))) for r in rows)) for c in cols}
    print("  ".join(c.ljust(widths[c]) for c in cols))
    for r in rows:
        print("  ".join(str(r.get(c, "")).ljust(widths[c]) for c in cols))

    if args.csv:
        args.csv.parent.mkdir(parents=True, exist_ok=True)
        all_cols = ["case", "variant", "warm_ms", "lin_solve_pct"] + [c for c, _ in FIELDS]
        with args.csv.open("w") as fh:
            fh.write(",".join(all_cols) + "\n")
            for r in rows:
                fh.write(",".join(str(r.get(c, "")) for c in all_cols) + "\n")
        print(f"\nwrote {args.csv}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
