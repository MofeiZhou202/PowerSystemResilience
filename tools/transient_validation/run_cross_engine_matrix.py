#!/usr/bin/env python3
"""Build the HACDCPF/GridLAB-D transient disturbance benchmark report."""

from __future__ import annotations

import argparse
import datetime as dt
import json
import os
import shutil
import subprocess
import tempfile
from pathlib import Path
from typing import Any


REPO = Path(__file__).resolve().parents[2]
DEFAULT_GRIDLABD = Path("/tmp/gridlabd-5.3.0/GridLAB-D-5.3.0-MacOS/bin/gridlabd")
DEFAULT_OUT = REPO / "external_data/transient_validation/cross_engine_matrix.json"


def run_native(binary: Path) -> dict[str, Any]:
    with tempfile.TemporaryDirectory(prefix="transient_native_") as tmp:
        output = Path(tmp) / "native.json"
        subprocess.run([str(binary), str(output)], check=True)
        return json.loads(output.read_text(encoding="utf-8"))


def default_gridlabd() -> Path:
    configured = os.environ.get("GRIDLABD_BIN")
    discovered = shutil.which("gridlabd")
    return Path(configured or discovered or DEFAULT_GRIDLABD)


def run_gridlabd(matrix_binary: Path, gridlabd_binary: Path) -> dict[str, Any]:
    with tempfile.TemporaryDirectory(prefix="transient_gridlabd_") as tmp:
        output = Path(tmp) / "gridlabd.json"
        env = dict(os.environ)
        env["GRIDLABD_BIN"] = str(gridlabd_binary)
        subprocess.run(
            [str(matrix_binary), "--long-duration", "--require-gridlabd",
             "--gridlabd-bin", str(gridlabd_binary), "--out", str(output)],
            check=True, env=env,
        )
        return json.loads(output.read_text(encoding="utf-8"))


def coverage_status(row: dict[str, Any]) -> dict[str, Any]:
    event_types = {event["type"] for event in row["events"]}
    domain = row["domain"]
    balance = row["balance"]
    exact_stage = {"ACLoadScale", "ACBranchTrip", "ACBranchClose"}
    if domain in {"dc", "hybrid_acdc"}:
        return {"status": "native_only_gridlabd_dc_hybrid_unsupported",
                "scope": "GridLAB-D benchmark exports only the AC algebraic boundary"}
    if balance == "unbalanced":
        return {"status": "native_only_unbalanced_transient_unsupported",
                "scope": "current GridLAB-D comparison exporter is balanced AC"}
    if not event_types:
        return {"status": "sampled_ac_stage_numeric", "scope": "balanced AC base stage"}
    if domain == "ac" and balance == "balanced" and event_types <= exact_stage:
        return {"status": "sampled_ac_stage_numeric",
                "scope": "balanced AC event-stage voltage and post-event equilibrium"}
    if domain == "ac" and balance == "balanced" and event_types <= {"FaultShunt", "ClearFault"}:
        return {"status": "prefault_postfault_stage_numeric_fault_on_diagnostic",
                "scope": "balanced AC prefault/restored stages; fault-on trajectory is native-only"}
    if (domain == "ac" and balance == "balanced" and
            bool(event_types & exact_stage) and len(event_types) > 1):
        return {"status": "partial_sequence_stage_numeric",
                "scope": "load/branch stages numeric; fault-on and storage controls native-only"}
    return {"status": "native_only_dynamic_model_not_equivalent",
            "scope": "no matched GridLAB-D dynamic device/control trajectory"}


def event_taxonomy(native: dict[str, Any]) -> list[dict[str, Any]]:
    types = sorted({event["type"] for row in native["cases"] for event in row["events"]})
    sampled = {"ACBranchTrip", "ACBranchClose", "ACLoadScale"}
    rows = []
    for event_type in types:
        if event_type in sampled:
            status = "numeric_sampled_ac_stage"
        elif event_type == "FaultShunt":
            status = "numeric_pre_post_ac_stage_fault_on_native_only"
        elif event_type == "ACBranchImpedanceScale":
            status = "representable_but_not_executed_in_gridlabd_sequence"
        elif event_type in {"DCBranchTrip", "DCBranchClose", "DCLoadScale",
                            "DCDCTrip", "DCStoragePowerStep"}:
            status = "unsupported_gridlabd_dc_network"
        elif event_type == "VSCTrip":
            status = "unsupported_matched_acdc_converter_dynamics"
        elif event_type in {"GeneratorTrip", "StoragePowerStep"}:
            status = "unsupported_matched_dynamic_device_model"
        elif event_type == "ClearFault":
            status = "covered_as_restored_ac_stage"
        else:
            status = "native_only"
        rows.append({"event_type": event_type, "native": "numeric_trajectory", "gridlabd": status})
    return rows


def markdown(report: dict[str, Any]) -> str:
    summary = report["summary"]
    lines = [
        "# Transient Cross-Engine Disturbance Matrix", "",
        f"Generated: `{report['generated_at']}`", "",
        "## Results", "",
        "| Case | Domain | Balance | Events | Native | Min AC pu | Min DC pu | Frequency nadir Hz | GridLAB-D scope |",
        "|---|---|---|---:|---|---:|---:|---:|---|",
    ]
    for row in report["cases"]:
        lines.append(
            f"| `{row['id']}` | {row['domain']} | {row['balance']} | "
            f"{row['expected_event_count']} | {'PASS' if row['native_pass'] else 'FAIL'} | "
            f"{row['min_ac_voltage_pu']:.6f} | {row['min_dc_voltage_pu']:.6f} | "
            f"{row['frequency_nadir_hz']:.6f} | {row['gridlabd']['status']} |"
        )
    lines += [
        "", "## Numerical Summary", "",
        f"- Native cases: {summary['native_passed']}/{summary['native_case_count']} passed",
        f"- Native event types: {summary['native_event_type_count']}",
        f"- GridLAB-D exact sampled-stage gates: {summary['gridlabd_exact_passed']}/{summary['gridlabd_exact_count']} passed",
        f"- GridLAB-D unique electrical stages executed: {summary['gridlabd_unique_stage_runs']}",
        f"- Maximum native-transient/GridLAB-D voltage difference: `{summary['max_dynamic_voltage_error_pu']:.6e} pu`",
        f"- Maximum native/GridLAB-D equilibrium voltage-magnitude difference: `{summary['max_snapshot_vm_error_pu']:.6e} pu`",
        f"- Maximum equilibrium voltage-angle difference: `{summary['max_snapshot_va_error_deg']:.6e} deg`",
        f"- Maximum equilibrium branch-P difference: `{summary['max_snapshot_branch_p_error_mw']:.6e} MW`",
        f"- Maximum equilibrium branch-Q difference: `{summary['max_snapshot_branch_q_error_mvar']:.6e} Mvar`",
        f"- Worst native AC voltage: `{summary['native_min_ac_voltage_pu']:.6f} pu`",
        f"- Worst native DC voltage: `{summary['native_min_dc_voltage_pu']:.6f} pu`",
        f"- Worst native frequency nadir: `{summary['native_frequency_nadir_hz']:.6f} Hz`",
        f"- Maximum native ROCOF: `{summary['native_max_rocof_hz_per_s']:.6f} Hz/s`",
        f"- Maximum native bus-frequency spread: `{summary['native_max_bus_frequency_spread_hz']:.6f} Hz`",
        f"- Maximum AC voltage-deviation integral: `{summary['native_max_ac_voltage_deviation_integral_pu_s']:.6e} pu-s`",
        f"- Maximum DC voltage-deviation integral: `{summary['native_max_dc_voltage_deviation_integral_pu_s']:.6e} pu-s`",
        f"- Maximum frequency-deviation integral: `{summary['native_max_frequency_deviation_integral_hz_s']:.6e} Hz-s`",
        f"- Maximum AC time below 0.9 pu: `{summary['native_max_ac_time_below_0_9_s']:.6f} s`",
        f"- Maximum DC time below 0.9 pu: `{summary['native_max_dc_time_below_0_9_s']:.6f} s`",
        f"- Maximum qualified settling time: `{summary['native_max_settling_time_s']:.6f} s`",
        f"- Cases not settled by the 0.30 s horizon: {summary['native_unsettled_case_count']}",
        f"- Worst native measured bus-frequency range: `{summary['native_bus_frequency_nadir_hz']:.6f}` to `{summary['native_bus_frequency_zenith_hz']:.6f} Hz`",
        f"- Native bus-frequency range on the GridLAB-D anchor case: `{summary['gridlabd_anchor_bus_frequency_nadir_hz']:.6f}` to `{summary['gridlabd_anchor_bus_frequency_zenith_hz']:.6f} Hz`", "",
        "## Fidelity Boundary", "",
        "HACDCPF rows are actual time-domain dynamic simulations. GridLAB-D rows are balanced AC algebraic snapshots solved at unique event stages and compared against the native trajectory at the same times. They validate voltage envelopes and post-event equilibria, not synchronous-machine, inverter-control, protection, frequency, ROCOF, DC-link, or converter-state trajectory equivalence.", "",
        "DC, hybrid AC/DC, unbalanced, converter, and storage-control disturbances remain native-only because the current GridLAB-D bridge has no matched dynamic model and observable contract for those states. Unsupported cells are capability results, not inferred numerical comparisons.", "",
        "## Event Coverage", "",
        "| Event | Native | GridLAB-D |", "|---|---|---|",
    ]
    for row in report["event_taxonomy"]:
        lines.append(f"| `{row['event_type']}` | {row['native']} | {row['gridlabd']} |")
    lines.append("")
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--native-bin", type=Path,
                        default=REPO / "build/macos-release/transient_validation_matrix")
    parser.add_argument("--gridlabd-matrix-bin", type=Path,
                        default=REPO / "build/macos-release/gridlabd_validation_matrix")
    parser.add_argument("--gridlabd-bin", type=Path, default=default_gridlabd())
    parser.add_argument("--gridlabd-report", type=Path,
                        help="Reuse an existing long-duration JSON report")
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT)
    args = parser.parse_args()

    native = run_native(args.native_bin.resolve())
    if args.gridlabd_report:
        gridlabd = json.loads(args.gridlabd_report.read_text(encoding="utf-8"))
    else:
        gridlabd = run_gridlabd(args.gridlabd_matrix_bin.resolve(), args.gridlabd_bin.resolve())

    rows = []
    for native_row in native["cases"]:
        row = dict(native_row)
        row["native_pass"] = bool(row["success"] and
                                  row["applied_event_count"] >= row["expected_event_count"])
        row["gridlabd"] = coverage_status(row)
        rows.append(row)
    taxonomy = event_taxonomy(native)
    nonzero_dc = [r["min_dc_voltage_pu"] for r in rows if r["min_dc_voltage_pu"] > 0.0]
    settled_times = [r["settling_time_s"] for r in rows if r["settling_time_s"] >= 0.0]
    report = {
        "schema": "hacdcpf.transient.cross_engine_matrix.v1",
        "generated_at": dt.datetime.now(dt.timezone.utc).isoformat(),
        "methodology": {
            "native": "time-domain phasor/DAE transient simulation",
            "gridlabd": "balanced AC algebraic snapshot at unique event stages",
            "comparison_limit": "not a matched dynamic-state trajectory comparison",
        },
        "engines": {"native": "HACDCPF dynamics", "gridlabd": "GridLAB-D 5.3.0"},
        "cases": rows,
        "event_taxonomy": taxonomy,
        "gridlabd_sequence": gridlabd,
        "summary": {
            "native_case_count": len(rows),
            "native_passed": sum(r["native_pass"] for r in rows),
            "native_failed": sum(not r["native_pass"] for r in rows),
            "native_event_type_count": len(taxonomy),
            "gridlabd_exact_count": gridlabd["exact_gate_count"],
            "gridlabd_exact_passed": gridlabd["exact_gate_passed_count"],
            "gridlabd_unique_stage_runs": gridlabd["gridlabd_unique_stage_runs"],
            "gridlabd_stage_cache_hits": gridlabd["gridlabd_stage_cache_hits"],
            "max_dynamic_voltage_error_pu": gridlabd["max_dynamic_voltage_error_pu"],
            "max_snapshot_vm_error_pu": gridlabd["max_snapshot_vm_error_pu"],
            "max_snapshot_va_error_deg": gridlabd["max_snapshot_va_error_deg"],
            "max_snapshot_branch_p_error_mw": gridlabd["max_snapshot_branch_p_error_mw"],
            "max_snapshot_branch_q_error_mvar": gridlabd["max_snapshot_branch_q_error_mvar"],
            "gridlabd_anchor_bus_frequency_nadir_hz": gridlabd["dynamic_bus_frequency_nadir_hz"],
            "gridlabd_anchor_bus_frequency_zenith_hz": gridlabd["dynamic_bus_frequency_zenith_hz"],
            "native_min_ac_voltage_pu": min(r["min_ac_voltage_pu"] for r in rows),
            "native_min_dc_voltage_pu": min(nonzero_dc),
            "native_frequency_nadir_hz": min(r["frequency_nadir_hz"] for r in rows),
            "native_max_rocof_hz_per_s": max(r["max_abs_rocof_hz_per_s"] for r in rows),
            "native_max_bus_frequency_spread_hz": max(r["max_bus_frequency_spread_hz"] for r in rows),
            "native_bus_frequency_nadir_hz": min(r["bus_frequency_nadir_hz"] for r in rows),
            "native_bus_frequency_zenith_hz": max(r["bus_frequency_zenith_hz"] for r in rows),
            "native_max_phase_voltage_unbalance_pu": max(r["max_phase_voltage_unbalance_pu"] for r in rows),
            "native_max_ac_voltage_deviation_integral_pu_s": max(r["ac_voltage_deviation_integral_pu_s"] for r in rows),
            "native_max_dc_voltage_deviation_integral_pu_s": max(r["dc_voltage_deviation_integral_pu_s"] for r in rows),
            "native_max_frequency_deviation_integral_hz_s": max(r["frequency_deviation_integral_hz_s"] for r in rows),
            "native_max_ac_time_below_0_9_s": max(r["ac_time_below_0_9_s"] for r in rows),
            "native_max_dc_time_below_0_9_s": max(r["dc_time_below_0_9_s"] for r in rows),
            "native_max_settling_time_s": max(settled_times, default=-1.0),
            "native_unsettled_case_count": sum(not r["settled_by_end"] for r in rows),
            "native_total_elapsed_ms": sum(r["elapsed_ms"] for r in rows),
        },
    }
    output = args.out.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2), encoding="utf-8")
    output.with_suffix(".md").write_text(markdown(report), encoding="utf-8")
    print(json.dumps(report["summary"], indent=2))
    return 0 if report["summary"]["native_failed"] == 0 and gridlabd["exact_gate_passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
