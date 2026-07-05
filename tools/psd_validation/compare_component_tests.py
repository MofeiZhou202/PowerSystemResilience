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
    validate_executable_cases(data)
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


def executable_cases(payload: Dict[str, object]) -> List[Dict[str, object]]:
    manifest = payload.get("execution_manifest", {})
    if not manifest:
        return []
    if not isinstance(manifest, dict):
        raise ValueError("execution_manifest must be an object")
    cases = manifest.get("cases", [])
    if not isinstance(cases, list):
        raise ValueError("execution_manifest.cases must be a list")
    return cases


def executable_signal_count(case: Dict[str, object]) -> int:
    total = 0
    for device in case.get("comparisons", []):
        if isinstance(device, dict):
            total += len(device.get("signals", []))
    return total


def executable_internal_object_count(case: Dict[str, object]) -> int:
    internal = case.get("internal_diagnostics", {})
    if not isinstance(internal, dict) or not internal.get("enabled"):
        return 0
    objects = internal.get("objects", [])
    return len(objects) if isinstance(objects, list) else 0


def summarize_execution(payload: Dict[str, object]) -> Dict[str, object]:
    cases = executable_cases(payload)
    enabled_cases = [case for case in cases if case.get("enabled")]
    return {
        "total_cases": len(cases),
        "enabled_cases": len(enabled_cases),
        "signals": sum(executable_signal_count(case) for case in enabled_cases),
        "internal_objects": sum(
            executable_internal_object_count(case) for case in enabled_cases
        ),
    }


def validate_executable_cases(payload: Dict[str, object]) -> None:
    rows = payload.get("rows", [])
    errors: List[str] = []
    for index, case in enumerate(executable_cases(payload)):
        if not isinstance(case, dict):
            errors.append(f"execution_manifest.cases[{index}] must be an object")
            continue
        case_id = str(case.get("id", f"<case {index}>"))
        for field in ("id", "enabled", "psd_case", "row_selector", "psd_reference", "hacdcpf", "comparisons"):
            if field not in case:
                errors.append(f"{case_id}: missing required field {field}")
        selector = case.get("row_selector", {})
        if not isinstance(selector, dict) or not selector:
            errors.append(f"{case_id}: row_selector must be a non-empty object")
            continue
        matches = [
            row for row in rows
            if all(row.get(key) == value for key, value in selector.items())
        ]
        if len(matches) != 1:
            errors.append(f"{case_id}: row_selector matched {len(matches)} rows")
        elif matches[0].get("execution_case_id", case_id) != case_id:
            errors.append(f"{case_id}: matrix row execution_case_id does not match")

        reference = case.get("psd_reference", {})
        if not isinstance(reference, dict):
            errors.append(f"{case_id}: psd_reference must be an object")
        else:
            for field in ("formulation", "integrator"):
                if field not in reference:
                    errors.append(f"{case_id}: psd_reference missing {field}")

        hacdcpf = case.get("hacdcpf", {})
        if not isinstance(hacdcpf, dict):
            errors.append(f"{case_id}: hacdcpf must be an object")
        else:
            for field in ("fixture", "solver", "event"):
                if field not in hacdcpf:
                    errors.append(f"{case_id}: hacdcpf missing {field}")

        comparisons = case.get("comparisons", [])
        if not isinstance(comparisons, list) or not comparisons:
            errors.append(f"{case_id}: comparisons must be a non-empty list")
        for device in comparisons if isinstance(comparisons, list) else []:
            if not isinstance(device, dict):
                errors.append(f"{case_id}: comparison device must be an object")
                continue
            for field in ("psd_ref", "local_component_index", "signals"):
                if field not in device:
                    errors.append(f"{case_id}: comparison device missing {field}")
            signals = device.get("signals", [])
            if not isinstance(signals, list) or not signals:
                errors.append(f"{case_id}: comparison device signals must be non-empty")
                continue
            for signal in signals:
                if not isinstance(signal, dict):
                    errors.append(f"{case_id}: signal must be an object")
                    continue
                for field in ("local_key", "psd_quantity", "tolerances"):
                    if field not in signal:
                        errors.append(f"{case_id}: signal missing {field}")
                tolerances = signal.get("tolerances", {})
                if not isinstance(tolerances, dict):
                    errors.append(f"{case_id}: signal tolerances must be an object")
                else:
                    for field in ("rms", "max"):
                        value = tolerances.get(field)
                        if not isinstance(value, (int, float)) or value <= 0:
                            errors.append(f"{case_id}: tolerance {field} must be positive")

        internal = case.get("internal_diagnostics")
        if internal is not None:
            if not isinstance(internal, dict):
                errors.append(f"{case_id}: internal_diagnostics must be an object")
            else:
                if not isinstance(internal.get("enabled", False), bool):
                    errors.append(f"{case_id}: internal_diagnostics.enabled must be boolean")
                if internal.get("enabled", False):
                    objects = internal.get("objects", [])
                    if not isinstance(objects, list) or not objects:
                        errors.append(
                            f"{case_id}: internal_diagnostics.objects must be non-empty when enabled"
                        )
                    step = internal.get("finite_difference_step", 0.0)
                    if not isinstance(step, (int, float)) or step <= 0:
                        errors.append(
                            f"{case_id}: internal_diagnostics.finite_difference_step must be positive"
                        )
                    tolerances = internal.get("tolerances", {})
                    if not isinstance(tolerances, dict) or not tolerances:
                        errors.append(
                            f"{case_id}: internal_diagnostics.tolerances must be non-empty"
                        )
                    else:
                        for key, value in tolerances.items():
                            if not isinstance(value, (int, float)) or value <= 0:
                                errors.append(
                                    f"{case_id}: internal tolerance {key} must be positive"
                                )
    if errors:
        raise ValueError("invalid executable manifest:\n" + "\n".join(errors))


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
    execution_stats = summarize_execution(payload)
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
        f"- Executable manifest cases: {execution_stats['enabled_cases']} enabled / {execution_stats['total_cases']} declared",
        f"- Executable comparison signals: {execution_stats['signals']}",
        f"- Executable internal diagnostic objects: {execution_stats['internal_objects']}",
    ]
    for status, count in stats["by_status"].items():
        lines.append(f"- {status}: {count}")
    lines.extend(["", "## Status Legend", ""])
    for status, text in payload.get("status_legend", {}).items():
        lines.append(f"- `{status}`: {text}")
    cases = executable_cases(payload)
    if cases:
        lines.extend(
            [
                "",
                "## Executable Gates",
                "",
                "| ID | PSD Test | PSD Reference | HACDCPF Fixture | Solver | Signals | Internal Objects | Enabled | Contract |",
                "|---|---|---|---|---|---:|---:|---|---|",
            ]
        )
        for case in cases:
            reference = case.get("psd_reference", {})
            hacdcpf = case.get("hacdcpf", {})
            reference_label = " / ".join(
                clean_cell(reference.get(field, ""))
                for field in ("formulation", "integrator")
                if reference.get(field)
            )
            lines.append(
                "| "
                + " | ".join(
                    [
                        clean_cell(case.get("id", "")),
                        clean_cell(case.get("psd_test", "")),
                        reference_label,
                        clean_cell(hacdcpf.get("fixture", "")),
                        clean_cell(hacdcpf.get("solver", "")),
                        str(executable_signal_count(case)),
                        str(executable_internal_object_count(case)),
                        clean_cell(case.get("enabled", False)),
                        clean_cell(case.get("gate_contract", "")),
                    ]
                )
                + " |"
            )
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
        summary = dict(stats)
        summary["execution_manifest"] = summarize_execution(payload)
        print(json.dumps(summary, indent=2, sort_keys=True))
    if args.fail_on_blocked and stats["blocked_rows"]:
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
