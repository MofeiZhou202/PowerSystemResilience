#!/usr/bin/env python3
"""Generate the PSD machine/IBR component-test passability report."""

from __future__ import annotations

import argparse
import csv
import json
from collections import Counter
from pathlib import Path
from typing import Dict, List


SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_MATRIX = SCRIPT_DIR / "psd_component_test_matrix.json"

CSV_COLUMNS = [
    "group",
    "psd_tests",
    "psd_components",
    "hacdcpf_candidate",
    "status",
    "first_blocker",
    "next_action",
]

BLOCKED_STATUSES = {
    "compare-failing",
    "blocked-missing-model",
    "blocked-missing-controller",
    "blocked-missing-formulation",
    "metadata-only",
}


def load_matrix(path: Path) -> Dict[str, object]:
    with path.open("r", encoding="utf-8") as handle:
        data = json.load(handle)
    if data.get("format") != "hacdcpf_psd_component_test_matrix.v1":
        raise ValueError(f"unexpected matrix format in {path}")
    if not isinstance(data.get("rows"), list):
        raise ValueError(f"matrix rows missing in {path}")
    return data


def summarize(rows: List[Dict[str, str]]) -> Dict[str, object]:
    by_status = Counter(row.get("status", "") for row in rows)
    by_group = Counter(row.get("group", "") for row in rows)
    blocked = sum(count for status, count in by_status.items() if status in BLOCKED_STATUSES)
    return {
        "total_rows": len(rows),
        "blocked_rows": blocked,
        "by_status": dict(sorted(by_status.items())),
        "by_group": dict(sorted(by_group.items())),
        "can_pass_all_component_tests": blocked == 0,
    }


def clean_cell(value: object) -> str:
    return ("" if value is None else str(value)).replace("|", "\\|").replace("\n", " ")


def write_csv(path: Path, rows: List[Dict[str, str]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=CSV_COLUMNS,
            extrasaction="ignore",
            lineterminator="\n",
        )
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def write_markdown(path: Path, payload: Dict[str, object], rows: List[Dict[str, str]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    stats = summarize(rows)
    lines = [
        "# PSD Machine/IBR Component Test Matrix",
        "",
        f"Generated from `{DEFAULT_MATRIX.name}`. Matrix updated: {payload.get('updated', 'unknown')}.",
        "",
        "This is the first-pass gate for comparing PowerSimulationsDynamics.jl transmission-dynamics component tests against HACDCPF. It answers whether each PSD machine or IBR component test group can be compared now.",
        "",
        "## Verdict",
        "",
        f"- Can pass all listed component test groups now: `{str(stats['can_pass_all_component_tests']).lower()}`",
        f"- Rows: {stats['total_rows']}",
        f"- Blocked rows: {stats['blocked_rows']}",
    ]
    for status, count in stats["by_status"].items():
        lines.append(f"- {status}: {count}")
    lines.extend(["", "## Status Legend", ""])
    for status, text in payload.get("status_legend", {}).items():
        lines.append(f"- `{status}`: {text}")
    lines.extend(
        [
            "",
            "## Matrix",
            "",
            "| Group | PSD Tests | PSD Components | HACDCPF Candidate | Status | First Blocker | Next Action |",
            "|---|---|---|---|---|---|---|",
        ]
    )
    for row in rows:
        lines.append(
            "| "
            + " | ".join(clean_cell(row.get(col, "")) for col in CSV_COLUMNS)
            + " |"
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--matrix", type=Path, default=DEFAULT_MATRIX)
    parser.add_argument("--out-md", type=Path)
    parser.add_argument("--out-csv", type=Path)
    parser.add_argument("--fail-on-blocked", action="store_true")
    parser.add_argument("--print-summary", action="store_true")
    args = parser.parse_args()

    payload = load_matrix(args.matrix)
    rows = payload["rows"]
    stats = summarize(rows)
    if args.out_md:
        write_markdown(args.out_md, payload, rows)
    if args.out_csv:
        write_csv(args.out_csv, rows)
    if args.print_summary or not (args.out_md or args.out_csv):
        print(json.dumps(stats, indent=2, sort_keys=True))
    if args.fail_on_blocked and stats["blocked_rows"]:
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
