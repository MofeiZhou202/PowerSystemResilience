#!/usr/bin/env python3
"""Cross-check fault kernels using OpenDSS IEEE-feeder Thevenin impedances."""

import argparse
import cmath
import json
import math
from pathlib import Path
import subprocess
from typing import Any

import dss


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_NATIVE = ROOT / "build/macos-release/short_circuit_thevenin_batch"
DEFAULT_OUTPUT = ROOT / "external_data/short_circuit_validation/ieee_feeders"
FEEDERS = {
    "ieee13": ROOT / (
        "external_data/opendss_ieee_pes/opendss_reference/13_node/"
        "official_full/IEEE13Nodeckt.dss"
    ),
    "ieee34": ROOT / (
        "external_data/opendss_ieee_pes/opendss_reference/34_node/"
        "Run_IEEE34Mod1.dss"
    ),
    "ieee123": ROOT / (
        "external_data/opendss_ieee_pes/opendss_reference/123_node/"
        "IEEE123Master.dss"
    ),
}


def complex_pair(values: Any) -> complex:
    raw = list(values)
    if len(raw) < 2:
        raise RuntimeError(f"invalid OpenDSS complex pair: {raw}")
    return complex(float(raw[0]), float(raw[1]))


def collect_thevenin_cases() -> list[dict[str, Any]]:
    cases = []
    engine = dss.DSS
    for feeder, master in FEEDERS.items():
        if not master.exists():
            raise RuntimeError(f"OpenDSS feeder is missing: {master}")
        engine.Text.Command = "Clear"
        engine.Text.Command = f'Compile "{master}"'
        if engine.Error.Number:
            raise RuntimeError(
                f"{feeder} compile failed: {engine.Error.Number}: "
                f"{engine.Error.Description}"
            )
        engine.Text.Command = "Solve mode=faultstudy"
        circuit = engine.ActiveCircuit
        if not circuit.Solution.Converged:
            raise RuntimeError(f"{feeder} OpenDSS fault study did not converge")
        for bus_name in circuit.AllBusNames:
            circuit.SetActiveBus(bus_name)
            bus = circuit.ActiveBus
            if not {1, 2, 3}.issubset(set(bus.Nodes)):
                continue
            base_kv = float(bus.kVBase) * math.sqrt(3.0)
            z1 = complex_pair(bus.Zsc1)
            z0 = complex_pair(bus.Zsc0)
            values = (base_kv, z1.real, z1.imag, z0.real, z0.imag)
            if not all(math.isfinite(value) for value in values):
                continue
            if base_kv <= 0.0 or abs(z1) <= 1e-12 or abs(z0) <= 1e-12:
                continue
            cases.append(
                {
                    "name": f"{feeder}:{bus_name}",
                    "feeder": feeder,
                    "bus": bus_name,
                    "base_mva": 100.0,
                    "base_kv": base_kv,
                    "z1_ohm": [z1.real, z1.imag],
                    "z0_ohm": [z0.real, z0.imag],
                }
            )
    return cases


def phase_currents(fault: str, base_kv: float, z1: complex, z0: complex) -> dict[str, float]:
    voltage = base_kv / math.sqrt(3.0)
    i0 = i1 = i2 = 0j
    if fault == "three_phase":
        i1 = voltage / z1
    elif fault == "single_phase_ground":
        i0 = i1 = i2 = voltage / (2.0 * z1 + z0)
    elif fault == "two_phase":
        i1 = voltage / (2.0 * z1)
        i2 = -i1
    elif fault == "two_phase_ground":
        i1 = voltage / (z1 + z1 * z0 / (z1 + z0))
        i2 = -i1 * z0 / (z1 + z0)
        i0 = -i1 * z1 / (z1 + z0)
    else:
        raise RuntimeError(f"unsupported fault type: {fault}")
    a = cmath.exp(2j * math.pi / 3.0)
    ia = i0 + i1 + i2
    ib = i0 + a * a * i1 + a * i2
    ic = i0 + a * i1 + a * a * i2
    phases = [abs(ia), abs(ib), abs(ic)]
    return {
        "ikss_ka": max(phases),
        "i_phase_a_ka": phases[0],
        "i_phase_b_ka": phases[1],
        "i_phase_c_ka": phases[2],
        "i_ground_ka": 3.0 * abs(i0),
    }


def scaled_error(actual: float, reference: float, fault_scale: float) -> float:
    # Higham (2002), sec. 2.2: use a problem-scale denominator for quantities
    # whose exact reference is zero; otherwise roundoff in a healthy phase is
    # misreported as an O(1) relative error.
    zero_threshold = max(fault_scale * 1e-10, 1e-15)
    if abs(actual) <= zero_threshold and abs(reference) <= zero_threshold:
        return 0.0
    return abs(actual - reference) / max(abs(reference), zero_threshold)


def markdown_report(report: dict[str, Any]) -> str:
    lines = [
        "# IEEE Feeder Short-Circuit Thevenin Cross-Validation",
        "",
        "This is external-Thevenin fault-kernel parity, not full phase-domain topology parity.",
        "",
        f"OpenDSS: dss-python {dss.__version__}",
        f"Three-phase buses: {report['summary']['bus_count']}",
        f"Fault comparisons: {report['summary']['comparison_count']}",
        f"Maximum relative error: {report['summary']['max_relative_error']:.9g}",
        "",
        "| Feeder | Three-phase buses | Fault comparisons | Maximum relative error |",
        "|---|---:|---:|---:|",
    ]
    for feeder, values in report["summary"]["by_feeder"].items():
        lines.append(
            f"| {feeder} | {values['bus_count']} | {values['comparison_count']} | "
            f"{values['max_relative_error']:.9g} |"
        )
    lines.extend(
        [
            "",
            "OpenDSS supplies Zsc1/Zsc0 from each complete unbalanced feeder. HACDCPF receives only that bus-level sequence equivalent; line matrices, regulator controls, and phase topology are not reassembled by this test.",
            "",
        ]
    )
    return "\n".join(lines)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--native", type=Path, default=DEFAULT_NATIVE)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--max-relative-error", type=float, default=1e-9)
    args = parser.parse_args()
    args.native = args.native.resolve()
    args.output_dir = args.output_dir.resolve()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    cases = collect_thevenin_cases()
    input_path = args.output_dir / "ieee_feeder_thevenin_input.json"
    native_path = args.output_dir / "ieee_feeder_hacdcpf.json"
    input_path.write_text(json.dumps({"cases": cases}, indent=2) + "\n", encoding="utf-8")
    subprocess.run([args.native, input_path, native_path], check=True, timeout=120)
    native = json.loads(native_path.read_text(encoding="utf-8"))
    native_by_name = {item["name"]: item for item in native["cases"]}

    comparisons = []
    by_feeder: dict[str, dict[str, Any]] = {}
    maximum = 0.0
    quantity_count = 0
    for case in cases:
        native_case = native_by_name[case["name"]]
        z1 = complex(*case["z1_ohm"])
        z0 = complex(*case["z0_ohm"])
        feeder_errors = []
        fault_results = {}
        for fault, actual in native_case["faults"].items():
            reference = phase_currents(fault, case["base_kv"], z1, z0)
            fault_scale = max(reference.values())
            errors = {
                key: scaled_error(float(actual[key]), reference[key], fault_scale)
                for key in reference
            }
            fault_max = max(errors.values())
            feeder_errors.append(fault_max)
            maximum = max(maximum, fault_max)
            quantity_count += len(errors)
            fault_results[fault] = {
                "hacdcpf": actual,
                "opendss_thevenin_reference": reference,
                "relative_error": errors,
            }
        comparisons.append({**case, "faults": fault_results})
        entry = by_feeder.setdefault(
            case["feeder"],
            {"bus_count": 0, "comparison_count": 0, "max_relative_error": 0.0},
        )
        entry["bus_count"] += 1
        entry["comparison_count"] += len(fault_results)
        entry["max_relative_error"] = max(entry["max_relative_error"], *feeder_errors)

    report = {
        "schema": "hacdcpf-ieee-feeder-short-circuit-thevenin-v1",
        "scope": "external-Thevenin fault-kernel parity; not phase-domain topology parity",
        "engines": {"opendss": f"dss-python {dss.__version__}", "hacdcpf": str(args.native)},
        "summary": {
            "bus_count": len(cases),
            "comparison_count": len(cases) * 4,
            "quantity_count": quantity_count,
            "max_relative_error": maximum,
            "by_feeder": by_feeder,
        },
        "cases": comparisons,
    }
    report_path = args.output_dir / "ieee_feeder_faultstudy.json"
    markdown_path = args.output_dir / "ieee_feeder_faultstudy.md"
    report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    markdown_path.write_text(markdown_report(report), encoding="utf-8")
    print(report_path)
    print(markdown_path)
    if maximum > args.max_relative_error:
        raise SystemExit(
            f"IEEE feeder Thevenin maximum relative error {maximum:.9g} "
            f"exceeds {args.max_relative_error:.9g}"
        )


if __name__ == "__main__":
    main()
