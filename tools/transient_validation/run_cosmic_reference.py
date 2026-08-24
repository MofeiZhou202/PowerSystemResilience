#!/usr/bin/env python3
"""Run and audit the authors' COSMIC IEEE 9-bus protection scenarios."""

from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import tempfile
from pathlib import Path
from typing import Any


EXPECTED_COMMIT = "6acc77e4d3f17925f1f4b79a93652eef0d1314cc"
DEFAULT_MATLAB = Path("/Applications/MATLAB_R2025b.app/bin/matlab")
DRIVER_DIR = Path(__file__).resolve().parent


def matlab_quote(value: Path) -> str:
    return str(value.resolve()).replace("'", "''")


def event_rows(case: dict[str, Any]) -> list[list[float]]:
    rows = case.get("event_record", [])
    if not rows:
        return []
    if rows and isinstance(rows[0], (int, float)):
        return [rows]
    return rows


def has_event(case: dict[str, Any], event_type: int, time_s: float,
              location_column: int | None = None, location: int = 0,
              tolerance_s: float = 5e-4) -> bool:
    for row in event_rows(case):
        if len(row) < 2 or int(row[1]) != event_type:
            continue
        if abs(float(row[0]) - time_s) > tolerance_s:
            continue
        if location_column is not None:
            if location_column >= len(row) or int(row[location_column]) != location:
                continue
        return True
    return False


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--cosmic-root", required=True, type=Path)
    parser.add_argument("--matlab", type=Path,
                        default=Path(shutil.which("matlab") or DEFAULT_MATLAB))
    parser.add_argument("--out", required=True, type=Path)
    parser.add_argument("--require-paper-match", action="store_true")
    parser.add_argument(
        "--allow-source-mismatch",
        action="store_true",
        help="run an exploratory comparison from a different or dirty COSMIC source",
    )
    args = parser.parse_args()

    if not args.matlab.is_file():
        raise SystemExit(f"MATLAB executable not found: {args.matlab}")
    if not (args.cosmic_root / "matlab/simgrid.m").is_file():
        raise SystemExit(f"not a COSMIC checkout: {args.cosmic_root}")

    actual_commit = "unavailable-non-git-source"
    source_clean = False
    if (args.cosmic_root / ".git").exists():
        actual_commit = subprocess.check_output(
            ["git", "-C", str(args.cosmic_root), "rev-parse", "HEAD"], text=True
        ).strip()
        source_clean = not subprocess.check_output(
            ["git", "-C", str(args.cosmic_root), "status", "--porcelain"],
            text=True,
        ).strip()
    source_match = actual_commit == EXPECTED_COMMIT and source_clean

    with tempfile.TemporaryDirectory(prefix="cosmic_reference_") as tmp:
        raw = Path(tmp) / "cosmic_raw.json"
        expression = (
            f"addpath('{matlab_quote(DRIVER_DIR)}'); "
            f"cosmic_reference_driver('{matlab_quote(args.cosmic_root)}',"
            f"'{matlab_quote(raw)}')"
        )
        subprocess.run([str(args.matlab), "-batch", expression],
                       cwd=tmp, check=True)
        report = json.loads(raw.read_text(encoding="utf-8"))

    public = report["public_example"]
    paper = report["paper_figure_2"]
    public_match = (
        has_event(public, 4, 10.0, 3, 6)
        and has_event(public, 13, 10.5)
        and abs(float(public["demand_lost_mw"]) - 31.25) <= 1e-6
    )
    paper_match = (
        has_event(paper, 4, 10.0, 3, 7)
        and has_event(paper, 12, 10.5, 3, 6, tolerance_s=0.05)
        and has_event(paper, 13, 10.7, tolerance_s=0.05)
    )
    report["source"] = {
        "repository": "https://github.com/ecotillasanchez/cosmic",
        "expected_commit": EXPECTED_COMMIT,
        "actual_commit": actual_commit,
        "clean_worktree": source_clean,
        "fixed_source_match": source_match,
        "matlab": str(args.matlab.resolve()),
    }
    report["cross_validation"] = {
        "public_example_match": public_match,
        "paper_figure_2_match": paper_match,
        "paper_mismatch_reason": (
            "public commit does not reproduce Fig. 2: branch-6 distance pickup "
            "ratio remains below one and no 10.7 s UVLS event is emitted"
            if not paper_match else ""
        ),
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report["cross_validation"], indent=2))
    if not source_match and not args.allow_source_mismatch:
        return 4
    if not public_match:
        return 2
    if args.require_paper_match and not paper_match:
        return 3
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
