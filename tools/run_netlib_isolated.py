#!/usr/bin/env python3
"""Run NETLIB benchmark cases in isolated subprocesses with a hard timeout."""

from __future__ import annotations

import argparse
import csv
import json
import math
import platform
import statistics
import subprocess
import sys
import tempfile
import time
from datetime import datetime, timezone
from pathlib import Path


CSV_FIELDS = [
    "case", "solver", "repeat", "rows", "columns", "nonzeros",
    "available", "success", "accurate", "runtime_ms", "iterations",
    "dual_pivots", "dse_initialization_solves", "dse_initialization_ms",
    "certified_dse_btrans", "certified_dse_candidates",
    "certified_dse_rejections", "certified_dse_ms", "native_kernel_ms",
    "native_kernel_ms_per_dual_pivot", "objective", "reference_objective",
    "objective_rel_error", "max_row_violation", "max_bound_violation",
    "normalized_primal_violation", "dual_objective",
    "relative_primal_residual", "relative_dual_residual", "relative_gap",
    "status",
]

SOLVER_DISPLAY_NAMES = {
    "highs-simplex": "HiGHS-simplex",
    "highs-ipm": "HiGHS-ipm",
    "highs-pdlp": "HiGHS-pdlp",
    "scip-direct": "SCIP-direct-MPS",
    "scip": "SCIP-LP(adapter)",
    "native-dual-simplex": "Native-DualSimplex(+HiGHS-presolve)",
    "native-dual-direct": "Native-DualSimplex[direct]",
    "native-pdlp": "Native-PDLP",
    "native-ipm": "Native-IPM(+HiGHS-presolve)",
    "native-ipm-direct": "Native-IPM[centrality-step,direct]",
    "native-lcqp": "Native-LCQP",
    "ipopt": "Ipopt-LP-as-NLP",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--executable", type=Path, required=True)
    parser.add_argument("--data-dir", type=Path, default=Path("tests/data"))
    parser.add_argument("--solver", required=True)
    parser.add_argument("--cases", help="Comma-separated exact case names")
    parser.add_argument("--repeat", type=int, default=1)
    parser.add_argument("--solver-time-limit", type=float, default=15.0)
    parser.add_argument("--hard-timeout", type=float, default=30.0)
    parser.add_argument("--max-iterations", type=int, default=100000)
    parser.add_argument("--csv", dest="csv_path", type=Path, required=True)
    parser.add_argument("--json", dest="json_path", type=Path, required=True)
    parser.add_argument("--log", dest="log_path", type=Path, required=True)
    return parser.parse_args()


def load_manifest(path: Path, selected: set[str] | None) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if selected is not None:
        rows = [row for row in rows if row["name"] in selected]
        missing = selected.difference(row["name"] for row in rows)
        if missing:
            raise ValueError(f"unknown NETLIB cases: {', '.join(sorted(missing))}")
    return rows


def failure_row(case: dict[str, str], solver: str, repeat: int,
                elapsed_ms: float, status: str) -> dict[str, object]:
    return {
        "case": case["name"],
        "solver": SOLVER_DISPLAY_NAMES.get(solver, solver),
        "repeat": repeat,
        "rows": case["rows"],
        "columns": case["columns"],
        "nonzeros": case["nonzeros"],
        "available": 1,
        "success": 0,
        "accurate": 0,
        "runtime_ms": elapsed_ms,
        "iterations": 0,
        "dual_pivots": 0,
        "dse_initialization_solves": 0,
        "dse_initialization_ms": 0,
        "certified_dse_btrans": 0,
        "certified_dse_candidates": 0,
        "certified_dse_rejections": 0,
        "certified_dse_ms": 0,
        "native_kernel_ms": 0,
        "native_kernel_ms_per_dual_pivot": 0,
        "objective": "",
        "reference_objective": case["reference_objective"],
        "objective_rel_error": "",
        "max_row_violation": "",
        "max_bound_violation": "",
        "normalized_primal_violation": "",
        "status": status,
    }


def timeout_row(case: dict[str, str], solver: str, repeat: int,
                elapsed_ms: float, timeout_sec: float) -> dict[str, object]:
    return failure_row(
        case, solver, repeat, elapsed_ms,
        f"Hard timeout after {timeout_sec:g} sec",
    )


def as_bool(value: object) -> bool:
    return str(value).lower() in {"1", "true"}


def finite_float(value: object) -> float | None:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def build_summary(rows: list[dict[str, object]]) -> dict[str, object]:
    times = [float(row["runtime_ms"]) for row in rows if as_bool(row["available"])]
    return {
        "solver": rows[0]["solver"] if rows else "",
        "attempts": len(rows),
        "available": sum(as_bool(row["available"]) for row in rows),
        "successes": sum(as_bool(row["success"]) for row in rows),
        "accurate": sum(as_bool(row["accurate"]) for row in rows),
        "total_sec": sum(times) / 1000.0,
        "median_ms": statistics.median(times) if times else 0.0,
        "geometric_mean_ms": (
            math.exp(sum(math.log(max(1e-6, value)) for value in times) / len(times))
            if times else 0.0
        ),
        "hard_timeouts": sum(str(row["status"]).startswith("Hard timeout") for row in rows),
    }


def json_run(row: dict[str, object]) -> dict[str, object]:
    integer_fields = {
        "repeat", "iterations", "dual_pivots", "dse_initialization_solves",
        "certified_dse_btrans", "certified_dse_candidates",
        "certified_dse_rejections",
    }
    boolean_fields = {"available", "success", "accurate"}
    omitted_fields = {"rows", "columns", "nonzeros", "max_row_violation", "max_bound_violation"}
    result: dict[str, object] = {}
    for key, value in row.items():
        if key in omitted_fields:
            continue
        if key in boolean_fields:
            result[key] = as_bool(value)
        elif key in integer_fields:
            result[key] = int(value)
        elif key not in {"case", "solver", "status"}:
            result[key] = finite_float(value)
        else:
            result[key] = value
    return result


def main() -> int:
    args = parse_args()
    if args.hard_timeout <= 0:
        raise ValueError("--hard-timeout must be positive")
    executable = args.executable.resolve()
    data_dir = args.data_dir.resolve()
    if not executable.is_file():
        raise FileNotFoundError(executable)
    selected = set(args.cases.split(",")) if args.cases else None
    cases = load_manifest(data_dir / "mps_manifest.csv", selected)
    for path in (args.csv_path, args.json_path, args.log_path):
        path.parent.mkdir(parents=True, exist_ok=True)

    combined_rows: list[dict[str, object]] = []
    case_records: list[dict[str, object]] = []
    first_provenance: dict[str, object] | None = None
    with tempfile.TemporaryDirectory(prefix="netlib-isolated-") as temp_name, \
            args.log_path.open("w", encoding="utf-8", errors="replace") as log:
        temp_dir = Path(temp_name)
        for index, case in enumerate(cases, start=1):
            for repeat in range(1, args.repeat + 1):
                stem = f"{index:03d}-{case['name']}-r{repeat}"
                case_csv = temp_dir / f"{stem}.csv"
                case_json = temp_dir / f"{stem}.json"
                command = [
                    str(executable), "--data-dir", str(data_dir),
                    "--cases", case["name"], "--solvers", args.solver,
                    "--repeat", "1", "--time-limit", str(args.solver_time_limit),
                    "--max-iterations", str(args.max_iterations),
                    "--csv", str(case_csv), "--json", str(case_json),
                ]
                print(f"[{index}/{len(cases)}] {case['name']} repeat={repeat}", flush=True)
                log.write(f"\n$ {' '.join(command)}\n")
                log.flush()
                started = time.perf_counter()
                try:
                    completed = subprocess.run(
                        command, stdout=log, stderr=subprocess.STDOUT,
                        text=True, errors="replace", timeout=args.hard_timeout,
                        check=False,
                    )
                    elapsed_ms = (time.perf_counter() - started) * 1000.0
                    log.write(f"isolated_exit_code={completed.returncode}\n")
                    if not case_csv.is_file() or not case_json.is_file():
                        status = f"Process exit {completed.returncode} without results"
                        combined_rows.append(failure_row(
                            case, args.solver, repeat, elapsed_ms, status))
                        case_records.append({"name": case["name"], "load_ms": None})
                        log.write(f"ABNORMAL_EXIT status={status}\n")
                        log.flush()
                        print(f"  {status} {elapsed_ms:.0f} ms", flush=True)
                        continue
                    with case_csv.open(newline="", encoding="utf-8") as stream:
                        rows = list(csv.DictReader(stream))
                    if len(rows) != 1:
                        raise RuntimeError(f"{case['name']}: expected one CSV row, got {len(rows)}")
                    rows[0]["repeat"] = repeat
                    combined_rows.append(rows[0])
                    with case_json.open(encoding="utf-8") as stream:
                        payload = json.load(stream)
                    if first_provenance is None:
                        first_provenance = payload.get("provenance")
                    load_ms = payload.get("cases", [{}])[0].get("load_ms")
                    case_records.append({"name": case["name"], "load_ms": load_ms})
                    print(f"  {rows[0]['status']} {elapsed_ms:.0f} ms", flush=True)
                except subprocess.TimeoutExpired:
                    elapsed_ms = (time.perf_counter() - started) * 1000.0
                    row = timeout_row(case, args.solver, repeat, elapsed_ms, args.hard_timeout)
                    combined_rows.append(row)
                    case_records.append({"name": case["name"], "load_ms": None})
                    log.write(f"HARD_TIMEOUT elapsed_ms={elapsed_ms:.3f}\n")
                    log.flush()
                    print(f"  HARD TIMEOUT {elapsed_ms:.0f} ms", flush=True)

    with args.csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=CSV_FIELDS)
        writer.writeheader()
        writer.writerows(combined_rows)

    command_line = subprocess.list2cmdline(sys.argv)
    payload = {
        "provenance": {
            "generated_utc": datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
            "runner": "tools/run_netlib_isolated.py",
            "runner_command": command_line,
            "host": platform.platform(),
            "benchmark": first_provenance,
        },
        "configuration": {
            "data_dir": str(data_dir),
            "solver": args.solver,
            "repeats": args.repeat,
            "solver_time_limit_sec": args.solver_time_limit,
            "hard_timeout_sec": args.hard_timeout,
            "max_iterations": args.max_iterations,
            "isolation": "one benchmark subprocess per case and repeat",
        },
        "cases": case_records,
        "runs": [json_run(row) for row in combined_rows],
        "summary": [build_summary(combined_rows)],
    }
    with args.json_path.open("w", encoding="utf-8") as stream:
        json.dump(payload, stream, indent=2, ensure_ascii=True)
        stream.write("\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
