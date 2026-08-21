#!/usr/bin/env python3
"""Run the HACDCPF/OpenDSS/GridLAB-D short-circuit validation matrix."""

import argparse
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
from typing import Any

import dss


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_NATIVE = ROOT / "build/macos-release/short_circuit_validation_matrix"
DEFAULT_OUTPUT = ROOT / "external_data/short_circuit_validation"


def complex_pair(raw: Any) -> complex:
    values = list(raw)
    if len(values) < 2:
        raise RuntimeError(f"DSS returned an invalid complex pair: {values}")
    return complex(float(values[0]), float(values[1]))


def pu_to_ohm(value: float, base_kv: float, base_mva: float) -> float:
    return value * base_kv * base_kv / base_mva


def configure_source(engine: Any, model: dict[str, Any], base_kv: float) -> None:
    source = model["source"]
    base_mva = float(model["base_mva"])
    props = {
        key: pu_to_ohm(float(source[key]), base_kv, base_mva)
        for key in ("r1_pu", "x1_pu", "r0_pu", "x0_pu")
    }
    engine.Text.Command = (
        "Edit Vsource.source phases=3 pu=1 "
        f"basekv={base_kv} R1={props['r1_pu']} X1={props['x1_pu']} "
        f"R0={props['r0_pu']} X0={props['x0_pu']}"
    )


def fault_current_a(engine: Any) -> complex:
    engine.ActiveCircuit.SetActiveElement("Fault.benchmark_fault")
    currents = list(engine.ActiveCircuit.ActiveCktElement.Currents)
    if len(currents) < 2:
        raise RuntimeError("OpenDSS fault current result is missing")
    return complex(float(currents[0]), float(currents[1]))


def element_max_current_a(engine: Any, element_name: str) -> float:
    # DSS C-API returns the zero-based active-element index; zero is success.
    if engine.ActiveCircuit.SetActiveElement(element_name) < 0:
        raise RuntimeError(f"OpenDSS element is missing: {element_name}")
    values = list(engine.ActiveCircuit.ActiveCktElement.Currents)
    return max(
        (abs(complex(float(values[index]), float(values[index + 1])))
         for index in range(0, len(values) - 1, 2)),
        default=0.0,
    )


def build_opendss_case(case: dict[str, Any]) -> dict[str, Any]:
    engine = dss.DSS
    engine.Text.Command = "Clear"
    model = case["model"]
    kind = model["kind"]
    source_kv = float(model.get("base_kv", model.get("hv_kv")))
    engine.Text.Command = (
        f"New Circuit.{case['name']} phases=3 bus1=n1 basekv={source_kv} pu=1"
    )
    configure_source(engine, model, source_kv)

    if kind == "line_network":
        base_mva = float(model["base_mva"])
        base_kv = float(model["base_kv"])
        for index, branch in enumerate(model["branches"], start=1):
            values = {
                key: pu_to_ohm(float(branch[key]), base_kv, base_mva)
                for key in ("r1_pu", "x1_pu", "r0_pu", "x0_pu")
            }
            engine.Text.Command = (
                f"New Line.line{index} phases=3 "
                f"bus1=n{branch['from_bus']} bus2=n{branch['to_bus']} "
                f"r1={values['r1_pu']} x1={values['x1_pu']} "
                f"r0={values['r0_pu']} x0={values['x0_pu']} "
                "c1=0 c0=0 length=1 units=none"
            )
        voltage_bases = f"({base_kv})"
    elif kind == "transformer_network":
        transformer = model["transformer"]
        vk = float(transformer["vk_percent"])
        vkr = float(transformer["vkr_percent"])
        xhl = math.sqrt(max(0.0, vk * vk - vkr * vkr))
        tap = float(transformer["tap_pu"])
        taps = f"({tap},1)" if transformer["tap_side"] == "hv" else f"(1,{tap})"
        engine.Text.Command = (
            "New Transformer.transformer phases=3 windings=2 "
            "buses=(n1.1.2.3.0,n2.1.2.3.0) conns=(wye,wye) "
            f"kvs=({model['hv_kv']},{model['lv_kv']}) "
            f"kvas=({float(transformer['sn_mva']) * 1000},"
            f"{float(transformer['sn_mva']) * 1000}) "
            f"%rs=({vkr / 2},{vkr / 2}) xhl={xhl} taps={taps}"
        )
        voltage_bases = f"({model['hv_kv']},{model['lv_kv']})"
    else:
        raise RuntimeError(f"Unsupported benchmark model kind: {kind}")

    ibrs = model.get("ibrs", [])
    for index, ibr in enumerate(ibrs, start=1):
        if ibr["mode"] != "grid_forming":
            continue
        bus = int(ibr["bus"])
        kv = float(model["lv_kv"] if kind == "transformer_network" else model["base_kv"])
        zbase = kv * kv / float(ibr["rating_mva"])
        r_ohm = float(ibr["r_sc_pu"]) * zbase
        x_ohm = float(ibr["x_sc_pu"]) * zbase
        engine.Text.Command = (
            f"New Vsource.gfm{index} bus1=n{bus} phases=3 basekv={kv} pu=1 "
            f"R1={r_ohm} X1={x_ohm} R0={r_ohm} X0={x_ohm}"
        )

    engine.Text.Command = f"Set voltagebases={voltage_bases}"
    engine.Text.Command = "CalcVoltageBases"
    engine.Text.Command = "Solve mode=faultstudy"
    engine.ActiveCircuit.SetActiveBus(f"n{case['fault_bus']}")
    z1 = complex_pair(engine.ActiveCircuit.ActiveBus.Zsc1)
    z0 = complex_pair(engine.ActiveCircuit.ActiveBus.Zsc0)

    voltage_source_ikss_ka = fault_bus_kv(case) / math.sqrt(3.0) / abs(z1)
    # Re-solve an explicit bolted fault so OpenDSS independently identifies
    # which physical branches carry partial fault current. IEC 60909-0:2016
    # method A uses the minimum R/X of those branches, not the fault-point
    # Thevenin R/X.
    engine.Text.Command = "Set mode=snapshot"
    engine.Text.Command = "Solve"
    engine.Text.Command = (
        f"New Fault.benchmark_fault bus1=n{case['fault_bus']} phases=3 r=1e-6"
    )
    engine.Text.Command = "Solve"
    baseline_fault_current = fault_current_a(engine)
    participating_rx: list[float] = []

    source = model["source"]
    if element_max_current_a(engine, "Vsource.source") > 1e-6:
        source_x = float(source["x1_pu"])
        if abs(source_x) > 1e-15:
            participating_rx.append(abs(float(source["r1_pu"]) / source_x))

    if kind == "line_network":
        for index, branch in enumerate(model["branches"], start=1):
            if element_max_current_a(engine, f"Line.line{index}") <= 1e-6:
                continue
            branch_x = float(branch["x1_pu"])
            if abs(branch_x) > 1e-15:
                participating_rx.append(abs(float(branch["r1_pu"]) / branch_x))
    else:
        transformer = model["transformer"]
        if element_max_current_a(engine, "Transformer.transformer") > 1e-6:
            vk = float(transformer["vk_percent"])
            vkr = float(transformer["vkr_percent"])
            transformer_x = math.sqrt(max(0.0, vk * vk - vkr * vkr))
            if transformer_x > 1e-15:
                participating_rx.append(abs(vkr / transformer_x))

    for index, ibr in enumerate(ibrs, start=1):
        if ibr["mode"] != "grid_forming":
            continue
        if element_max_current_a(engine, f"Vsource.gfm{index}") <= 1e-6:
            continue
        x_sc = float(ibr["x_sc_pu"])
        if abs(x_sc) > 1e-15:
            participating_rx.append(abs(float(ibr["r_sc_pu"]) / x_sc))
    if not participating_rx:
        raise RuntimeError(f"{case['name']}: method A found no participating branch")

    total_ikss_ka = abs(baseline_fault_current) / 1000.0
    gfls = [ibr for ibr in ibrs if ibr["mode"] == "grid_following"]
    if gfls:
        for index, ibr in enumerate(ibrs, start=1):
            if ibr["mode"] != "grid_following":
                continue
            bus = int(ibr["bus"])
            kv = float(model["lv_kv"] if kind == "transformer_network" else model["base_kv"])
            amps = (
                float(ibr["current_limit_pu"]) * float(ibr["rating_mva"]) * 1000.0
                / (math.sqrt(3.0) * kv)
            )
            engine.Text.Command = (
                f"New Isource.gfl{index} bus1=n{bus} phases=3 amps={amps} "
                "angle=0 enabled=no"
            )
        engine.Text.Command = "Solve"
        baseline = fault_current_a(engine)
        for index, ibr in enumerate(ibrs, start=1):
            if ibr["mode"] != "grid_following":
                continue
            engine.Text.Command = f"Edit Isource.gfl{index} enabled=yes angle=0"
            engine.Text.Command = "Solve"
            response = fault_current_a(engine) - baseline
            angle = math.degrees(math.atan2(baseline.imag, baseline.real)) - math.degrees(
                math.atan2(response.imag, response.real)
            )
            ibr["external_alignment_angle_deg"] = angle
            engine.Text.Command = f"Edit Isource.gfl{index} angle={angle} enabled=no"
        for index, ibr in enumerate(ibrs, start=1):
            if ibr["mode"] == "grid_following":
                engine.Text.Command = f"Edit Isource.gfl{index} enabled=yes"
        engine.Text.Command = "Solve"
        total_ikss_ka = abs(fault_current_a(engine)) / 1000.0

    return {
        "z1": z1,
        "z0": z0,
        "voltage_source_ikss_ka": voltage_source_ikss_ka,
        "total_ikss_ka": total_ikss_ka,
        "method_a_min_rx": min(participating_rx),
    }


def fault_impedance(fault_type: str, z1: complex, z0: complex) -> complex:
    if fault_type == "three_phase":
        return z1
    if fault_type == "single_phase_ground":
        return (2.0 * z1 + z0) / 3.0
    if fault_type == "two_phase":
        return 2.0 * z1 / math.sqrt(3.0)
    raise RuntimeError(f"Unsupported comparison fault type: {fault_type}")


def fault_bus_kv(case: dict[str, Any]) -> float:
    model = case["model"]
    if model["kind"] == "transformer_network":
        return float(model["lv_kv"])
    return float(model["base_kv"])


def relative_error(actual: float, reference: float) -> float:
    return abs(actual - reference) / max(abs(reference), 1e-12)


def gridlabd_capability() -> dict[str, Any]:
    configured = os.environ.get("GRIDLABD_BIN")
    candidates = [
        Path(configured) if configured else None,
        Path(found) if (found := shutil.which("gridlabd")) else None,
        ROOT.parent / "gridlab-d" / "cmake-build" / "bin" / "gridlabd",
        ROOT.parent / "gridlab-d" / "build" / "bin" / "gridlabd",
        ROOT.parent / "gridlab-d" / "build" / "source" / "gridlabd",
    ]
    executable = next(
        (str(candidate) for candidate in candidates
         if candidate is not None and candidate.is_file()
         and os.access(candidate, os.X_OK)),
        None,
    )
    if not executable:
        return {
            "available": False,
            "short_circuit_api": False,
            "status": "unavailable",
            "detail": (
                "GridLAB-D was not found through GRIDLABD_BIN, PATH, or the "
                "supported sibling-repository build locations"
            ),
        }
    probe = subprocess.run(
        [executable, "--version"], capture_output=True, text=True, check=False,
        timeout=120,
    )
    return {
        "available": probe.returncode == 0,
        "executable": executable,
        "version": (probe.stdout or probe.stderr).strip(),
        "short_circuit_api": "balanced_shunt_emulation",
        "status": "available",
        "detail": (
            "GridLAB-D has no FaultStudy/Zsc result interface. Three-phase RMS "
            "current is extrapolated from an explicit 100 ohm balanced shunt-probe "
            "powerflow. GFM voltage-source equivalents are included; GFL current "
            "limits, unbalanced faults, and IEC peak remain unsupported by steady NR."
        ),
    }


def gridlabd_line_configuration(name: str, z1: complex, z0: complex) -> str:
    z_self = (z0 + 2.0 * z1) / 3.0
    z_mutual = (z0 - z1) / 3.0
    entries = []
    for row in range(1, 4):
        for column in range(1, 4):
            value = z_self if row == column else z_mutual
            entries.append(
                f"  z{row}{column} {value.real:+.15g}{value.imag:+.15g}j Ohm/mile;"
            )
    return "\n".join(
        ["object line_configuration {", f"  name {name};", *entries, "};"]
    )


def run_gridlabd_case(
    case: dict[str, Any], capability: dict[str, Any], fault_ohm: float = 100.0
) -> dict[str, Any]:
    if not capability.get("available"):
        return {"status": "unavailable", "ikss_ka": None, "ip_ka": None}
    if case["fault_type"] != "three_phase":
        return {
            "status": "unsupported_fault_type",
            "ikss_ka": None,
            "ip_ka": None,
        }

    model = case["model"]
    ibrs = model.get("ibrs", [])
    if any(ibr["mode"] == "grid_following" for ibr in ibrs):
        return {
            "status": "unsupported_gfl_current_limit",
            "ikss_ka": None,
            "ip_ka": None,
            "detail": (
                "GridLAB-D steady NR inverter does not enforce the declared "
                "short-circuit current limit; deltamode control/protection is required."
            ),
        }
    base_mva = float(model["base_mva"])
    source_kv = float(model.get("base_kv", model.get("hv_kv")))
    source = model["source"]
    source_z1 = complex(
        pu_to_ohm(float(source["r1_pu"]), source_kv, base_mva),
        pu_to_ohm(float(source["x1_pu"]), source_kv, base_mva),
    )
    source_z0 = complex(
        pu_to_ohm(float(source["r0_pu"]), source_kv, base_mva),
        pu_to_ohm(float(source["x0_pu"]), source_kv, base_mva),
    )
    if model["kind"] == "line_network":
        bus_count = int(model["bus_count"])
        bus_kv = {index: float(model["base_kv"]) for index in range(1, bus_count + 1)}
    else:
        bus_count = 2
        bus_kv = {1: float(model["hv_kv"]), 2: float(model["lv_kv"])}

    lines = [
        "#set iteration_limit=100000;",
        "#set double_format=%.15g;",
        "clock { timezone GMT0; starttime '2000-01-01 00:00:00'; stoptime '2000-01-01 00:00:02'; };",
        "module powerflow { solver_method NR; NR_iteration_limit 1000; nominal_frequency 50; };",
        "module tape;",
        "object meter {",
        "  name swing; phases ABCN; bustype SWING;",
        f"  nominal_voltage {source_kv * 1000.0 / math.sqrt(3.0)};",
        f"  voltage_A {source_kv * 1000.0 / math.sqrt(3.0)}+0j;",
        f"  voltage_B {source_kv * 1000.0 / math.sqrt(3.0)}-120d;",
        f"  voltage_C {source_kv * 1000.0 / math.sqrt(3.0)}+120d;",
        "};",
    ]
    for index in range(1, bus_count + 1):
        object_type = "load" if index == int(case["fault_bus"]) else "meter"
        lines.extend(
            [
                f"object {object_type} {{",
                f"  name bus{index}; phases ABCN;",
                f"  nominal_voltage {bus_kv[index] * 1000.0 / math.sqrt(3.0)};",
            ]
        )
        if object_type == "load":
            lines.extend(
                [
                    f"  constant_impedance_A {fault_ohm}+0j;",
                    f"  constant_impedance_B {fault_ohm}+0j;",
                    f"  constant_impedance_C {fault_ohm}+0j;",
                ]
            )
        lines.append("};")

    lines.append(gridlabd_line_configuration("cfg_source", source_z1, source_z0))
    lines.append(
        "object overhead_line { name source_impedance; phases ABC; from swing; "
        "to bus1; length 1 mile; configuration cfg_source; };"
    )
    for index, ibr in enumerate(ibrs, start=1):
        if ibr["mode"] != "grid_forming":
            continue
        bus = int(ibr["bus"])
        kv = bus_kv[bus]
        zbase = kv * kv / float(ibr["rating_mva"])
        z_ibr = complex(float(ibr["r_sc_pu"]) * zbase,
                        float(ibr["x_sc_pu"]) * zbase)
        voltage_ln = kv * 1000.0 / math.sqrt(3.0)
        lines.extend(
            [
                "object meter {",
                f"  name gfm_swing{index}; phases ABCN; bustype SWING;",
                f"  nominal_voltage {voltage_ln};",
                f"  voltage_A {voltage_ln}+0j;",
                f"  voltage_B {voltage_ln}-120d;",
                f"  voltage_C {voltage_ln}+120d;",
                "};",
                gridlabd_line_configuration(f"cfg_gfm{index}", z_ibr, z_ibr),
                f"object overhead_line {{ name gfm_impedance{index}; phases ABC; "
                f"from gfm_swing{index}; to bus{bus}; length 1 mile; "
                f"configuration cfg_gfm{index}; }};",
            ]
        )

    if model["kind"] == "line_network":
        base_kv = float(model["base_kv"])
        for index, branch in enumerate(model["branches"], start=1):
            z1 = complex(
                pu_to_ohm(float(branch["r1_pu"]), base_kv, base_mva),
                pu_to_ohm(float(branch["x1_pu"]), base_kv, base_mva),
            )
            z0 = complex(
                pu_to_ohm(float(branch["r0_pu"]), base_kv, base_mva),
                pu_to_ohm(float(branch["x0_pu"]), base_kv, base_mva),
            )
            lines.append(gridlabd_line_configuration(f"cfg_line{index}", z1, z0))
            lines.append(
                f"object overhead_line {{ name line{index}; phases ABC; "
                f"from bus{branch['from_bus']}; to bus{branch['to_bus']}; "
                f"length 1 mile; configuration cfg_line{index}; }};"
            )
    else:
        transformer = model["transformer"]
        tap = float(transformer["tap_pu"])
        canonical_tap = tap if transformer["tap_side"] == "hv" else 1.0 / tap
        impedance_scale = 1.0 if transformer["tap_side"] == "hv" else tap * tap
        vk = float(transformer["vk_percent"]) / 100.0
        resistance = float(transformer["vkr_percent"]) / 100.0
        reactance = math.sqrt(max(0.0, vk * vk - resistance * resistance))
        lines.extend(
            [
                "object transformer_configuration {",
                "  name cfg_transformer; connect_type WYE_WYE; install_type PADMOUNT;",
                f"  power_rating {float(transformer['sn_mva']) * 1000.0};",
                f"  primary_voltage {float(model['hv_kv']) * 1000.0 * canonical_tap};",
                f"  secondary_voltage {float(model['lv_kv']) * 1000.0};",
                f"  resistance {resistance * impedance_scale};",
                f"  reactance {reactance * impedance_scale};",
                "};",
                "object transformer { name transformer; phases ABCN; from bus1; "
                "to bus2; configuration cfg_transformer; };",
            ]
        )

    lines.extend(
        [
            "object recorder {",
            f"  parent bus{case['fault_bus']}; file fault.csv; interval 1;",
            '  property "voltage_A.real,voltage_A.imag,measured_power_A.real,measured_power_A.imag";',
            "};",
        ]
    )

    executable = Path(capability["executable"])
    root = executable.parent.parent
    environment = os.environ.copy()
    runtime_paths = [executable.parent, root, root / "lib", root / "share"]
    environment["GLPATH"] = os.pathsep.join(str(path) for path in runtime_paths if path.exists())
    with tempfile.TemporaryDirectory(prefix=f"gridlabd_sc_{case['name']}_") as directory:
        working = Path(directory)
        glm = working / "fault.glm"
        glm.write_text("\n".join(lines) + "\n", encoding="utf-8")
        result = subprocess.run(
            [str(executable), glm.name], cwd=working, env=environment,
            capture_output=True, text=True, check=False, timeout=120,
        )
        csv_path = working / "fault.csv"
        if result.returncode != 0 or not csv_path.exists():
            return {
                "status": "failed",
                "ikss_ka": None,
                "ip_ka": None,
                "exit_code": result.returncode,
                "stderr": result.stderr[-2000:],
            }
        data_lines = [
            row for row in csv_path.read_text(encoding="utf-8").splitlines()
            if row and not row.startswith("#")
        ]
        values = data_lines[-1].split(",")
        numeric = [float(value.strip().rstrip(";")) for value in values[1:5]]
        voltage = complex(numeric[0], numeric[1])
        power = complex(numeric[2], numeric[3])
        current = voltage / fault_ohm
        nominal_fault_voltage = bus_kv[int(case["fault_bus"])] * 1000.0 / math.sqrt(3.0)
        if model["kind"] == "transformer_network":
            transformer = model["transformer"]
            tap = float(transformer["tap_pu"])
            canonical_tap = tap if transformer["tap_side"] == "hv" else 1.0 / tap
            open_circuit_voltage = nominal_fault_voltage / canonical_tap
        else:
            open_circuit_voltage = nominal_fault_voltage
        z_thevenin = (complex(open_circuit_voltage, 0.0) - voltage) / current
        bolted_current_a = nominal_fault_voltage / abs(z_thevenin)
        return {
            "status": "balanced_shunt_extrapolation",
            "probe_impedance_ohm": fault_ohm,
            "fault_voltage_a_v": [voltage.real, voltage.imag],
            "fault_power_a_va": [power.real, power.imag],
            "z_thevenin_ohm": [z_thevenin.real, z_thevenin.imag],
            "ikss_ka": bolted_current_a / 1000.0,
            "ip_ka": None,
        }


def run_native(binary: Path) -> dict[str, Any]:
    if not binary.exists():
        raise RuntimeError(
            f"Native benchmark executable is missing: {binary}. Build target "
            "short_circuit_validation_matrix first."
        )
    with tempfile.TemporaryDirectory(prefix="hacdcpf_sc_matrix_") as directory:
        output = Path(directory) / "native.json"
        subprocess.run([str(binary), "--out", str(output)], check=True)
        return json.loads(output.read_text(encoding="utf-8"))


def summarize(cases: list[dict[str, Any]]) -> dict[str, Any]:
    metrics = ("ikss_ka", "ip_ka")
    summary: dict[str, Any] = {"case_count": len(cases), "metrics": {}}
    for metric in metrics:
        errors = [case["comparison"][f"{metric}_relative_error"] for case in cases]
        summary["metrics"][metric] = {
            "mean_relative_error": sum(errors) / len(errors),
            "max_relative_error": max(errors),
            "worst_case": cases[errors.index(max(errors))]["name"],
        }
    summary["within_0_1_percent"] = sum(
        case["comparison"]["ikss_ka_relative_error"] <= 1e-3
        and case["comparison"]["ip_ka_relative_error"] <= 1e-3
        for case in cases
    )
    gridlabd_cases = [case for case in cases if case["gridlabd"].get("ikss_ka") is not None]
    if gridlabd_cases:
        errors = [case["comparison"]["gridlabd_ikss_ka_relative_error"] for case in gridlabd_cases]
        summary["gridlabd_balanced_shunt"] = {
            "case_count": len(gridlabd_cases),
            "mean_relative_error": sum(errors) / len(errors),
            "max_relative_error": max(errors),
            "worst_case": gridlabd_cases[errors.index(max(errors))]["name"],
        }
    else:
        summary["gridlabd_balanced_shunt"] = {"case_count": 0}
    summary["by_category"] = {}
    for category in sorted({case["category"] for case in cases}):
        group = [case for case in cases if case["category"] == category]
        gridlabd_group = [case for case in group if case["gridlabd"].get("ikss_ka") is not None]
        entry = {
            "case_count": len(group),
            "opendss_ikss_max_relative_error": max(
                case["comparison"]["ikss_ka_relative_error"] for case in group
            ),
            "opendss_ip_max_relative_error": max(
                case["comparison"]["ip_ka_relative_error"] for case in group
            ),
            "gridlabd_case_count": len(gridlabd_group),
        }
        if gridlabd_group:
            entry["gridlabd_ikss_max_relative_error"] = max(
                case["comparison"]["gridlabd_ikss_ka_relative_error"]
                for case in gridlabd_group
            )
        summary["by_category"][category] = entry
    return summary


def markdown_report(report: dict[str, Any]) -> str:
    summary = report["summary"]
    lines = [
        "# Short-Circuit Cross-Engine Validation",
        "",
        f"Cases: {summary['case_count']}",
        f"OpenDSS: {report['engines']['opendss']['version']}",
        f"GridLAB-D: {report['engines']['gridlabd']['status']}",
        "",
        "## Aggregate Errors",
        "",
        "| Quantity | Mean relative error | Maximum relative error | Worst case |",
        "|---|---:|---:|---|",
    ]
    for metric, values in summary["metrics"].items():
        lines.append(
            f"| {metric} | {values['mean_relative_error']:.6%} | "
            f"{values['max_relative_error']:.6%} | {values['worst_case']} |"
        )
    gridlabd_summary = summary["gridlabd_balanced_shunt"]
    if gridlabd_summary["case_count"]:
        lines.append(
            f"| GridLAB-D Ikss ({gridlabd_summary['case_count']} balanced cases) | "
            f"{gridlabd_summary['mean_relative_error']:.6%} | "
            f"{gridlabd_summary['max_relative_error']:.6%} | "
            f"{gridlabd_summary['worst_case']} |"
        )
    lines.extend(
        [
            "",
            "## Category Coverage",
            "",
            "| Category | Cases | OpenDSS Ikss max error | OpenDSS ip max error | GridLAB-D cases | GridLAB-D Ikss max error |",
            "|---|---:|---:|---:|---:|---:|",
        ]
    )
    for category, values in summary["by_category"].items():
        gridlabd_error = values.get("gridlabd_ikss_max_relative_error")
        gridlabd_text = f"{gridlabd_error:.6%}" if gridlabd_error is not None else "n/a"
        lines.append(
            f"| {category} | {values['case_count']} | "
            f"{values['opendss_ikss_max_relative_error']:.6%} | "
            f"{values['opendss_ip_max_relative_error']:.6%} | "
            f"{values['gridlabd_case_count']} | {gridlabd_text} |"
        )
    lines.extend(
        [
            "",
            "## Per-Case Results",
            "",
            "| Case | Category | Fault | HACDCPF Ikss kA | HACDCPF IBR kA | OpenDSS Ikss kA | OpenDSS GFL kA | Error | GridLAB-D Ikss kA | Error | HACDCPF ip kA | OpenDSS+IEC ip kA | Error |",
            "|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
        ]
    )
    for case in report["cases"]:
        native = case["hacdcpf"]
        reference = case["opendss"]
        compare = case["comparison"]
        gridlabd = case["gridlabd"]
        gridlabd_ikss = (
            f"{gridlabd['ikss_ka']:.6f}" if gridlabd.get("ikss_ka") is not None
            else gridlabd["status"]
        )
        gridlabd_error = (
            f"{compare['gridlabd_ikss_ka_relative_error']:.4%}"
            if "gridlabd_ikss_ka_relative_error" in compare else "n/a"
        )
        lines.append(
            f"| {case['name']} | {case['category']} | {case['fault_type']} | "
            f"{native['ikss_ka']:.6f} | {native.get('ibr_contribution_ka', 0.0):.6f} | "
            f"{reference['ikss_ka']:.6f} | {reference.get('current_source_ikss_ka', 0.0):.6f} | "
            f"{compare['ikss_ka_relative_error']:.4%} | {gridlabd_ikss} | "
            f"{gridlabd_error} | {native['ip_ka']:.6f} | "
            f"{reference['ip_ka']:.6f} | {compare['ip_ka_relative_error']:.4%} |"
        )
    lines.extend(
        [
            "",
            "## GridLAB-D Scope",
            "",
            report["engines"]["gridlabd"]["detail"],
            "GridLAB-D cells are therefore capability results, not inferred short-circuit numbers.",
            "",
        ]
    )
    return "\n".join(lines)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--native", type=Path, default=DEFAULT_NATIVE)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--max-ikss-relative-error", type=float, default=1e-6)
    parser.add_argument("--max-ip-relative-error", type=float, default=2e-2)
    parser.add_argument("--require-gridlabd", action="store_true")
    parser.add_argument("--min-gridlabd-cases", type=int, default=35)
    parser.add_argument("--max-gridlabd-relative-error", type=float, default=1e-6)
    args = parser.parse_args()

    native_report = run_native(args.native)
    gridlabd = gridlabd_capability()
    cases = []
    for case in native_report["cases"]:
        dss_result = build_opendss_case(case)
        z1, z0 = dss_result["z1"], dss_result["z0"]
        zeq = fault_impedance(case["fault_type"], z1, z0)
        voltage_phase_kv = fault_bus_kv(case) / math.sqrt(3.0)
        voltage_source_ikss_ka = voltage_phase_kv / abs(zeq)
        has_gfl = any(
            ibr["mode"] == "grid_following"
            for ibr in case["model"].get("ibrs", [])
        )
        ikss_ka = dss_result["total_ikss_ka"] if has_gfl else voltage_source_ikss_ka
        rx = float(dss_result["method_a_min_rx"])
        kappa = 1.02 + 0.98 * math.exp(-3.0 * rx)
        current_source_ka = max(0.0, ikss_ka - voltage_source_ikss_ka)
        ip_ka = math.sqrt(2.0) * (
            kappa * voltage_source_ikss_ka + current_source_ka
        )
        case["opendss"] = {
            "z1_ohm": [z1.real, z1.imag],
            "z0_ohm": [z0.real, z0.imag],
            "ikss_ka": ikss_ka,
            "voltage_source_ikss_ka": voltage_source_ikss_ka,
            "current_source_ikss_ka": current_source_ka,
            "kappa_method_a": kappa,
            "ip_ka": ip_ka,
        }
        native = case["hacdcpf"]
        case["comparison"] = {
            "ikss_ka_relative_error": relative_error(native["ikss_ka"], ikss_ka),
            "ip_ka_relative_error": relative_error(native["ip_ka"], ip_ka),
        }
        case["gridlabd"] = run_gridlabd_case(case, gridlabd)
        if case["gridlabd"].get("ikss_ka") is not None:
            case["comparison"]["gridlabd_ikss_ka_relative_error"] = relative_error(
                native["ikss_ka"], case["gridlabd"]["ikss_ka"]
            )
        cases.append(case)

    report = {
        "schema": "hacdcpf-short-circuit-cross-engine-v1",
        "engines": {
            "hacdcpf": {"method": native_report["method"]},
            "opendss": {"version": f"dss-python {dss.__version__}", "mode": "faultstudy"},
            "gridlabd": gridlabd,
        },
        "summary": summarize(cases),
        "cases": cases,
    }
    args.output_dir.mkdir(parents=True, exist_ok=True)
    json_path = args.output_dir / "cross_engine_matrix.json"
    markdown_path = args.output_dir / "cross_engine_matrix.md"
    json_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    markdown_path.write_text(markdown_report(report), encoding="utf-8")
    print(json_path)
    print(markdown_path)
    ikss_error = report["summary"]["metrics"]["ikss_ka"]["max_relative_error"]
    ip_error = report["summary"]["metrics"]["ip_ka"]["max_relative_error"]
    if ikss_error > args.max_ikss_relative_error:
        raise SystemExit(
            f"OpenDSS Ikss relative error {ikss_error:.9g} exceeds "
            f"{args.max_ikss_relative_error:.9g}"
        )
    if ip_error > args.max_ip_relative_error:
        raise SystemExit(
            f"OpenDSS ip relative error {ip_error:.9g} exceeds "
            f"{args.max_ip_relative_error:.9g}"
        )
    gridlabd_summary = report["summary"]["gridlabd_balanced_shunt"]
    if args.require_gridlabd and not report["engines"]["gridlabd"]["available"]:
        raise SystemExit(report["engines"]["gridlabd"]["detail"])
    if args.require_gridlabd and gridlabd_summary["case_count"] < args.min_gridlabd_cases:
        raise SystemExit(
            f"GridLAB-D produced {gridlabd_summary['case_count']} numerical cases; "
            f"at least {args.min_gridlabd_cases} are required"
        )
    if gridlabd_summary["case_count"]:
        gridlabd_error = gridlabd_summary["max_relative_error"]
        if gridlabd_error > args.max_gridlabd_relative_error:
            raise SystemExit(
                f"GridLAB-D Ikss relative error {gridlabd_error:.9g} exceeds "
                f"{args.max_gridlabd_relative_error:.9g}"
            )


if __name__ == "__main__":
    main()
