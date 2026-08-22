#!/usr/bin/env python3
"""Cross-engine harmonic power-flow benchmark matrix.

OpenDSS is used for directly representable AC frequency-domain cases. NumPy is
an independent dense complex-nodal reference for every AC, DC, and hybrid case.
GridLAB-D capability rows are explicit because GridLAB-D 5.3 has no native
frequency-domain harmonic power-flow result API; fundamental NR is not used as
a substitute for harmonic validation.
"""

from __future__ import annotations

import argparse
import cmath
import json
import math
import os
import shutil
import subprocess
import tempfile
from pathlib import Path
from typing import Any

import numpy as np
import opendssdirect as dss


REPO = Path(__file__).resolve().parents[2]
DEFAULT_OUT = REPO / "external_data/harmonics_validation/cross_engine_matrix.json"


def gridlabd_capability() -> dict[str, Any]:
    configured = os.environ.get("HACDCPF_GRIDLABD_BIN") or os.environ.get("GRIDLABD_BIN")
    candidates = [
        Path(configured) if configured else None,
        Path(found) if (found := shutil.which("gridlabd")) else None,
        REPO.parent / "gridlab-d" / "cmake-build" / "bin" / "gridlabd",
        REPO.parent / "gridlab-d" / "build" / "bin" / "gridlabd",
        REPO.parent / "gridlab-d" / "build" / "source" / "gridlabd",
    ]
    executable = next((p.resolve() for p in candidates
                       if p is not None and p.is_file() and os.access(p, os.X_OK)), None)
    if executable is None:
        return {"available": False, "status": "unavailable", "detail":
                "GridLAB-D was not found through HACDCPF_GRIDLABD_BIN, GRIDLABD_BIN, PATH, or sibling builds"}
    probe = subprocess.run([str(executable), "--version"], capture_output=True,
                           text=True, check=False, timeout=120)
    return {"available": probe.returncode == 0, "status": "available" if probe.returncode == 0 else "probe_failed",
            "executable": str(executable), "version": (probe.stdout or probe.stderr).strip(),
            "detail": "Independent complex steady-state frequency slices using explicit order-scaled impedances"}


def _gridlabd_csv_complex(path: Path) -> complex:
    rows = [row for row in path.read_text(encoding="utf-8").splitlines()
            if row and not row.startswith("#")]
    if not rows:
        raise RuntimeError(f"GridLAB-D recorder is empty: {path.name}")
    fields = rows[-1].split(",")
    if len(fields) < 3:
        raise RuntimeError(f"GridLAB-D recorder row is invalid: {rows[-1]}")
    return complex(float(fields[-2].strip().rstrip(";")),
                   float(fields[-1].strip().rstrip(";")))


def gridlabd_frequency_slice(capability: dict[str, Any], case: dict[str, Any],
                             order: int, current_pu: complex) -> dict[str, Any]:
    if not capability.get("available"):
        raise RuntimeError(capability["detail"])
    base_mva = float(case.get("base_mva", 100.0))
    base_kv = float(case.get("base_kv", 10.0))
    base_v = base_kv * 1000.0
    base_i = base_mva * 1e6 / base_v
    zbase = base_v / base_i
    line = case["line"]
    zsrc = complex(0.0, order * float(case.get("source_xpp_pu", 0.2))) * zbase
    zline = complex(skin_r(float(line["r_pu"]), order, case),
                    order * float(line["x_pu"])) * zbase
    probe_a = 100.0
    glm = f"""#set double_format=%.15g;
clock {{ timezone EST+5EDT; starttime '2000-01-01 00:00:00'; stoptime '2000-01-01 00:00:01'; }}
module powerflow {{ solver_method NR; }}
module tape;
object node {{ name source; phases AN; bustype SWING; nominal_voltage {base_v:.15g}; voltage_A {base_v:.15g}+0j; }}
object series_reactor {{ name source_z; phases AN; from source; to internal; phase_A_impedance {zsrc.real:+.15g}{zsrc.imag:+.15g}j; }}
object node {{ name internal; phases AN; nominal_voltage {base_v:.15g}; }}
object series_reactor {{ name feeder_z; phases AN; from internal; to loadbus; phase_A_impedance {zline.real:+.15g}{zline.imag:+.15g}j; }}
object load {{ name loadbus; phases AN; nominal_voltage {base_v:.15g}; constant_current_A {-probe_a:+.15g}+0j; }}
object recorder {{ parent internal; property voltage_A.real,voltage_A.imag; interval 1; file internal.csv; }}
object recorder {{ parent loadbus; property voltage_A.real,voltage_A.imag; interval 1; file load.csv; }}
object recorder {{ parent feeder_z; property current_in_A.real,current_in_A.imag; interval 1; file current.csv; }}
"""
    executable = Path(capability["executable"])
    root = executable.parent.parent
    environment = os.environ.copy()
    runtime_paths = [executable.parent, root, root / "lib", root / "share"]
    environment["GLPATH"] = os.pathsep.join(str(p) for p in runtime_paths if p.exists())
    with tempfile.TemporaryDirectory(prefix=f"hpf_gld_h{order}_") as directory:
        working = Path(directory)
        model = working / "slice.glm"
        model.write_text(glm, encoding="utf-8")
        run = subprocess.run([str(executable), model.name], cwd=working, env=environment,
                             capture_output=True, text=True, check=False, timeout=120)
        if run.returncode != 0:
            raise RuntimeError(f"GridLAB-D h={order} failed: {run.stderr[-2000:]}")
        vint = _gridlabd_csv_complex(working / "internal.csv")
        vload = _gridlabd_csv_complex(working / "load.csv")
        ibranch = _gridlabd_csv_complex(working / "current.csv")
    injection_probe = -ibranch
    if abs(injection_probe) <= 1e-9:
        raise RuntimeError(f"GridLAB-D h={order} returned zero probe current")
    zsrc_measured = (vint - base_v) / injection_probe
    ztotal_measured = (vload - base_v) / injection_probe
    voltages_pu = [zsrc_measured / zbase * current_pu,
                   ztotal_measured / zbase * current_pu]
    return {"order": order, "voltages_pu": voltages_pu,
            "measured_source_impedance_pu": zsrc_measured / zbase,
            "measured_transfer_impedance_pu": ztotal_measured / zbase,
            "authored_source_impedance_pu": zsrc / zbase,
            "authored_transfer_impedance_pu": (zsrc + zline) / zbase}


def gridlabd_single(capability: dict[str, Any], case: dict[str, Any]) -> dict[str, Any]:
    slices = [gridlabd_frequency_slice(capability, case, h, spectrum_current(case, h))
              for h in case["ac_orders"]]
    return {"orders": {row["order"]: row["voltages_pu"] for row in slices},
            "slices": slices}


def gridlabd_slice_report(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    def pair(value: complex) -> dict[str, float]:
        return {"real": value.real, "imag": value.imag}
    return [{key: (pair(value) if isinstance(value, complex)
                   else [pair(x) for x in value] if isinstance(value, list) else value)
             for key, value in row.items()} for row in rows]


def as_complex(row: dict[str, Any]) -> complex:
    return complex(float(row["real"]), float(row["imag"]))


def angle_error_deg(a: complex, b: complex) -> float | None:
    if abs(a) < 1e-12 or abs(b) < 1e-12:
        return None
    return abs(math.degrees(cmath.phase(a / b)))


def skin_r(r: float, order: int, case: dict[str, Any]) -> float:
    model = case.get("skin_effect", "none")
    if model == "sqrt_order":
        return r * math.sqrt(order)
    if model == "proportional_sqrt":
        return r * (1.0 + float(case.get("skin_coefficient", 0.0)) * math.sqrt(order))
    return r


def spectrum_current(case: dict[str, Any], order: int, base: float | None = None) -> complex:
    row = next(r for r in case["spectrum"] if int(r["order"]) == order)
    mag = float(case.get("i_base_pu", 0.0) if base is None else base) * float(row["mag_percent"]) / 100.0
    angle = float(case.get("i_base_phase_deg", 0.0)) + float(row.get("phase_deg", 0.0))
    return cmath.rect(mag, math.radians(angle))


def solve_two_bus(z_source: complex, z_line: complex, current: complex,
                  y_load: complex = 0j) -> tuple[complex, complex, complex]:
    ys = 1.0 / z_source
    yl = 1.0 / z_line
    y = np.array([[ys + yl, -yl], [-yl, yl + y_load]], dtype=complex)
    v = np.linalg.solve(y, np.array([0j, current], dtype=complex))
    return complex(v[0]), complex(v[1]), complex((v[0] - v[1]) * yl)


def independent_single_ac(case: dict[str, Any]) -> dict[str, Any]:
    line = case["line"]
    orders: dict[int, list[complex]] = {}
    currents: dict[int, complex] = {}
    for h in case["ac_orders"]:
        i = spectrum_current(case, h)
        zsrc = 1j * h * float(case.get("source_xpp_pu", 0.2))
        zline = complex(skin_r(float(line["r_pu"]), h, case), h * float(line["x_pu"]))
        v1, v2, ib = solve_two_bus(zsrc, zline, i)
        orders[h] = [v1, v2]
        currents[h] = ib
    return {"orders": orders, "branch_currents": currents}


def seq_components(v: list[complex]) -> list[complex]:
    a = cmath.exp(2j * math.pi / 3.0)
    va, vb, vc = v
    return [(va + vb + vc) / 3.0,
            (va + a * vb + a * a * vc) / 3.0,
            (va + a * a * vb + a * vc) / 3.0]


def independent_three_phase(case: dict[str, Any]) -> dict[str, Any]:
    line = case["line"]
    a = cmath.exp(2j * math.pi / 3.0)
    transform = np.array([[1, 1, 1], [1, a * a, a], [1, a, a * a]], dtype=complex)
    inv_transform = np.linalg.inv(transform)
    orders: dict[int, list[list[complex]]] = {}
    for h in case["ac_orders"]:
        z0 = complex(skin_r(float(line["r0_pu"]), h, case), h * float(line["x0_pu"]))
        z1 = complex(skin_r(float(line["r1_pu"]), h, case), h * float(line["x1_pu"]))
        zabc = transform @ np.diag([z0, z1, z1]) @ inv_transform
        yline = np.linalg.inv(zabc)
        ysrc = np.eye(3, dtype=complex) / (1j * h * float(case.get("source_xpp_pu", 0.2)))
        y = np.block([[ysrc + yline, -yline], [-yline, yline]])
        ia = spectrum_current(case, h, float(case.get("i_base_pu", 0.0)))
        if case.get("balanced", True):
            seq = h % 3
            mult = [1, 1, 1] if seq == 0 else ([1, a * a, a] if seq == 1 else [1, a, a * a])
            inj = np.array([ia * m for m in mult], dtype=complex)
        else:
            bases = case["i_base_pu_abc"]
            inj = np.array([spectrum_current(case, h, float(x)) for x in bases], dtype=complex)
        v = np.linalg.solve(y, np.concatenate((np.zeros(3, dtype=complex), inj)))
        orders[h] = [[complex(x) for x in v[:3]], [complex(x) for x in v[3:]]]
    return {"orders": orders}


def independent_dc(case: dict[str, Any]) -> dict[str, Any]:
    r = float(case["dc_line"]["r_pu"])
    x = float(case.get("dc_branch_x_pu", 0.0))
    b = float(case.get("dc_bus_b_pu", 0.0))
    zsrc = complex(float(case.get("dc_source_impedance_pu", 0.01)), 0.0)
    orders: dict[int, list[complex]] = {}
    currents: dict[int, complex] = {}
    for h in case["dc_orders"]:
        i = spectrum_current(case, h)
        v1, v2, ib = solve_two_bus(zsrc, complex(r, h * x), i, 1j * h * b)
        orders[h] = [v1, v2]
        currents[h] = ib
    return {"orders": orders, "branch_currents": currents}


def run_native(binary: Path, case: dict[str, Any]) -> dict[str, Any]:
    with tempfile.TemporaryDirectory(prefix="hpf_matrix_") as tmp:
        inp, out = Path(tmp) / "case.json", Path(tmp) / "result.json"
        inp.write_text(json.dumps(case), encoding="utf-8")
        subprocess.run([str(binary), str(inp), str(out)], check=True, capture_output=True, text=True)
        return json.loads(out.read_text(encoding="utf-8"))


def native_single_orders(result: dict[str, Any], dc: bool = False) -> dict[int, list[complex]]:
    buses = result["dc_buses" if dc else "buses"]
    ids = sorted(int(b["bus"]) for b in buses)
    out: dict[int, list[complex]] = {}
    for bus_id in ids:
        bus = next(b for b in buses if int(b["bus"]) == bus_id)
        for row in bus["orders"]:
            h = int(row["order"])
            if h not in (0, 1):
                out.setdefault(h, []).append(as_complex(row))
    return out


def native_phase_orders(result: dict[str, Any]) -> dict[int, list[list[complex]]]:
    out: dict[int, list[list[complex]]] = {}
    for bus in sorted(result["buses"], key=lambda x: int(x["bus"])):
        for row in bus["orders"]:
            h = int(row["order"])
            if h != 1:
                out.setdefault(h, []).append([as_complex(p) for p in row["phases"]])
    return out


def compare_orders(native: dict[int, Any], reference: dict[int, Any]) -> dict[str, Any]:
    complex_errors: list[float] = []
    magnitude_errors: list[float] = []
    angle_errors: list[float] = []
    samples = 0

    def walk(a: Any, b: Any) -> None:
        nonlocal samples
        if isinstance(a, list):
            for x, y in zip(a, b):
                walk(x, y)
            return
        samples += 1
        complex_errors.append(abs(a - b))
        magnitude_errors.append(abs(abs(a) - abs(b)))
        ae = angle_error_deg(a, b)
        if ae is not None:
            angle_errors.append(ae)

    for h in sorted(set(native) & set(reference)):
        walk(native[h], reference[h])
    return {"sample_count": samples,
            "max_complex_voltage_error_pu": max(complex_errors, default=0.0),
            "max_voltage_magnitude_error_pu": max(magnitude_errors, default=0.0),
            "max_voltage_angle_error_deg": max(angle_errors, default=0.0)}


def derived_metrics(orders: dict[int, Any], phase_domain: bool) -> dict[str, Any]:
    flat_by_node: list[list[tuple[int, complex]]] = []
    sequence_rows: list[dict[str, Any]] = []
    if phase_domain:
        bus_count = len(next(iter(orders.values()))) if orders else 0
        for bus in range(bus_count):
            for phase in range(3):
                flat_by_node.append([(h, orders[h][bus][phase]) for h in sorted(orders)])
            for h in sorted(orders):
                seq = seq_components(orders[h][bus])
                dominant = h % 3
                denom = abs(seq[dominant])
                nondom = math.sqrt(sum(abs(seq[k]) ** 2 for k in range(3) if k != dominant))
                sequence_rows.append({"bus_position": bus, "order": h,
                                      "v0_pu": abs(seq[0]), "v1_pu": abs(seq[1]),
                                      "v2_pu": abs(seq[2]),
                                      "sequence_unbalance_pct": 100.0 * nondom / denom if denom > 1e-12 else None})
    else:
        node_count = len(next(iter(orders.values()))) if orders else 0
        flat_by_node = [[(h, orders[h][n]) for h in sorted(orders)] for n in range(node_count)]
    ihd = [[{"order": h, "ihd_pct": abs(v) * 100.0} for h, v in rows] for rows in flat_by_node]
    thd = [100.0 * math.sqrt(sum(abs(v) ** 2 for _, v in rows)) for rows in flat_by_node]
    return {"voltage_ihd_pct": ihd, "voltage_thd_pct": thd,
            "max_voltage_thd_pct": max(thd, default=0.0), "sequence": sequence_rows}


def derived_metric_errors(native: dict[int, Any], reference: dict[int, Any],
                          phase_domain: bool) -> dict[str, float]:
    nm = derived_metrics(native, phase_domain)
    rm = derived_metrics(reference, phase_domain)
    ihd_errors = [abs(a["ihd_pct"] - b["ihd_pct"])
                  for na, ra in zip(nm["voltage_ihd_pct"], rm["voltage_ihd_pct"])
                  for a, b in zip(na, ra)]
    thd_errors = [abs(a - b) for a, b in zip(nm["voltage_thd_pct"], rm["voltage_thd_pct"])]
    seq_errors: list[float] = []
    unbalance_errors: list[float] = []
    for a, b in zip(nm["sequence"], rm["sequence"]):
        seq_errors.extend(abs(float(a[key]) - float(b[key])) for key in ("v0_pu", "v1_pu", "v2_pu"))
        if a["sequence_unbalance_pct"] is not None and b["sequence_unbalance_pct"] is not None:
            unbalance_errors.append(abs(float(a["sequence_unbalance_pct"]) - float(b["sequence_unbalance_pct"])))
    return {"max_voltage_ihd_error_pct": max(ihd_errors, default=0.0),
            "max_voltage_thd_error_pct": max(thd_errors, default=0.0),
            "max_sequence_component_error_pu": max(seq_errors, default=0.0),
            "max_sequence_unbalance_error_pct": max(unbalance_errors, default=0.0)}


def full_comparison(native: dict[int, Any], reference: dict[int, Any],
                    phase_domain: bool) -> dict[str, Any]:
    return {**compare_orders(native, reference),
            **derived_metric_errors(native, reference, phase_domain)}


def run_dss(command: str) -> None:
    dss.Text.Command(command)
    err = dss.Error.Description()
    if err:
        raise RuntimeError(err)


def opendss_one_phase(case: dict[str, Any], order: int, current: complex,
                      r: float, x: float) -> tuple[list[complex], complex]:
    base_mva = float(case.get("base_mva", 100.0))
    base_kv = float(case.get("base_kv", 10.0))
    base_v = base_kv * 1000.0
    base_i = base_mva * 1e6 / base_v
    zbase = base_v / base_i
    dss.Basic.ClearAll()
    run_dss("clear")
    run_dss(f"new circuit.hpf phases=1 bus1=source.1 basekv={base_kv:.17g} frequency=60")
    run_dss(f"new reactor.src phases=1 bus1=source.1 bus2=source.0 R=0 X={float(case.get('source_xpp_pu', .2))*zbase:.17g}")
    run_dss(f"new reactor.line phases=1 bus1=source.1 bus2=load.1 R={r*zbase:.17g} X={x*zbase:.17g}")
    amps = current * base_i
    run_dss(f"new isource.h{order} phases=1 bus1=load.1 amps={abs(amps):.17g} angle={math.degrees(cmath.phase(amps)):.17g} frequency={60*order}")
    run_dss("solve mode=direct")
    run_dss("disable vsource.source")
    run_dss("set mode=harmonic")
    run_dss(f"set harmonics=({order})")
    run_dss("solve")
    if not dss.Solution.Converged():
        raise RuntimeError(f"OpenDSS did not converge at h={order}")
    names = [x.lower() for x in dss.Circuit.AllBusNames()]
    raw = dss.Circuit.AllBusVolts()
    volts = [complex(raw[i], raw[i + 1]) / base_v for i in range(0, len(raw), 2)]
    by_name = dict(zip(names, volts))
    dss.Circuit.SetActiveElement("reactor.line")
    cur = dss.CktElement.Currents()
    line_current = complex(cur[0], cur[1]) / base_i
    return [by_name["source"], by_name["load"]], line_current


def opendss_single(case: dict[str, Any]) -> dict[str, Any]:
    orders: dict[int, list[complex]] = {}
    currents: dict[int, complex] = {}
    line = case["line"]
    for h in case["ac_orders"]:
        orders[h], currents[h] = opendss_one_phase(
            case, h, spectrum_current(case, h), skin_r(float(line["r_pu"]), h, case), float(line["x_pu"]))
    return {"orders": orders, "branch_currents": currents}


def opendss_three_phase_decoupled(case: dict[str, Any]) -> dict[str, Any]:
    line = case["line"]
    if abs(float(line["r0_pu"]) - float(line["r1_pu"])) > 1e-14 or abs(float(line["x0_pu"]) - float(line["x1_pu"])) > 1e-14:
        raise ValueError("OpenDSS decoupled phase representation requires z0=z1")
    ref = independent_three_phase(case)
    orders: dict[int, list[list[complex]]] = {}
    for h in case["ac_orders"]:
        per_phase: list[list[complex]] = []
        # The reference injection is recovered from V through the known series path.
        z = complex(skin_r(float(line["r1_pu"]), h, case), h * float(line["x1_pu"]))
        zsrc = 1j * h * float(case.get("source_xpp_pu", 0.2))
        for phase in range(3):
            current = ref["orders"][h][1][phase] / (z + zsrc)
            per_phase.append(opendss_one_phase(case, h, current, skin_r(float(line["r1_pu"]), h, case), float(line["x1_pu"]))[0])
        orders[h] = [[per_phase[p][0] for p in range(3)], [per_phase[p][1] for p in range(3)]]
    return {"orders": orders}


def cases() -> list[dict[str, Any]]:
    spec = [{"order": 5, "mag_percent": 20, "phase_deg": 10},
            {"order": 7, "mag_percent": 14, "phase_deg": -20},
            {"order": 11, "mag_percent": 9, "phase_deg": 35},
            {"order": 13, "mag_percent": 7, "phase_deg": -45}]
    ac_base = {"case_type": "canonical_current_source", "base_mva": 100.0, "base_kv": 10.0,
               "i_base_pu": 0.3, "source_xpp_pu": 0.2, "ac_orders": [5, 7, 11, 13], "spectrum": spec}
    tp_base = {"case_type": "canonical_three_phase_current_source", "base_mva": 100.0, "base_kv": 10.0,
               "i_base_pu": 0.3, "source_xpp_pu": 0.2, "ac_orders": [5, 7, 11, 13], "spectrum": spec,
               "line": {"r1_pu": 0.01, "x1_pu": 0.08, "r0_pu": 0.01, "x0_pu": 0.08}}
    dc_spec = [{"order": 6, "mag_percent": 12, "phase_deg": 5},
               {"order": 12, "mag_percent": 7, "phase_deg": -30},
               {"order": 18, "mag_percent": 4, "phase_deg": 50}]
    dc_base = {"case_type": "canonical_dc_current_source", "base_mva": 100.0, "dc_base_kv": 1.0,
               "i_base_pu": 0.5, "dc_source_impedance_pu": 0.01,
               "dc_orders": [6, 12, 18], "dc_line": {"r_pu": 0.05}, "spectrum": dc_spec}
    hybrid = {"case_type": "device_vsc_nic", "base_mva": 100.0, "base_kv": 10.0,
              "dc_base_kv": 1.0, "p_mw": 30.0, "q_mvar": 10.0, "p_dc_mw": -31.0,
              "line": {"r_pu": 0.01, "x_pu": 0.1}, "dc_line": {"r_pu": 0.05},
              "source_xpp_pu": 0.2, "dc_source_impedance_pu": 0.01,
              "ac_orders": [5, 7, 11, 13], "dc_orders": [6, 12]}
    return [
        {**ac_base, "id": "ac_radial_characteristic", "domain": "ac", "balance": "positive_sequence", "line": {"r_pu": .01, "x_pu": .1}},
        {**ac_base, "id": "ac_resistive_feeder", "domain": "ac", "balance": "positive_sequence", "line": {"r_pu": .08, "x_pu": .04}},
        {**ac_base, "id": "ac_skin_sqrt", "domain": "ac", "balance": "positive_sequence", "line": {"r_pu": .04, "x_pu": .06}, "skin_effect": "sqrt_order"},
        {**ac_base, "id": "ac_skin_proportional", "domain": "ac", "balance": "positive_sequence", "line": {"r_pu": .04, "x_pu": .06}, "skin_effect": "proportional_sqrt", "skin_coefficient": .25},
        {**tp_base, "id": "ac_3ph_balanced", "domain": "ac_3ph", "balance": "balanced", "balanced": True},
        {**tp_base, "id": "ac_3ph_unbalanced", "domain": "ac_3ph", "balance": "unbalanced", "balanced": False, "i_base_pu_abc": [.3, .18, .07]},
        {**tp_base, "id": "ac_3ph_unbalanced_skin", "domain": "ac_3ph", "balance": "unbalanced", "balanced": False, "i_base_pu_abc": [.3, .12, .24], "skin_effect": "sqrt_order"},
        {**tp_base, "id": "ac_3ph_zero_sequence", "domain": "ac_3ph", "balance": "balanced", "balanced": True,
         "ac_orders": [3, 9], "spectrum": [{"order": 3, "mag_percent": 10, "phase_deg": 0}, {"order": 9, "mag_percent": 4, "phase_deg": 25}],
         "line": {"r1_pu": .01, "x1_pu": .08, "r0_pu": .04, "x0_pu": .24}},
        {**dc_base, "id": "dc_resistive_ripple", "domain": "dc", "balance": "n/a"},
        {**dc_base, "id": "dc_inductive_ripple", "domain": "dc", "balance": "n/a", "dc_branch_x_pu": .015},
        {**dc_base, "id": "dc_capacitive_filter", "domain": "dc", "balance": "n/a", "dc_bus_b_pu": .08},
        {**dc_base, "id": "dc_lc_filter", "domain": "dc", "balance": "n/a", "dc_branch_x_pu": .012, "dc_bus_b_pu": .05},
        {**hybrid, "id": "hybrid_vsc_characteristic", "domain": "hybrid_acdc", "balance": "positive_sequence"},
        {**hybrid, "id": "hybrid_vsc_filtered", "domain": "hybrid_acdc", "balance": "positive_sequence", "skin_effect": "sqrt_order", "dc_branch_x_pu": .01, "dc_bus_b_pu": .04},
    ]


def hybrid_reference(case: dict[str, Any], native: dict[str, Any]) -> dict[str, Any]:
    ac_case = {**case, "case_type": "canonical_current_source", "spectrum": native["device"]["ac_spectrum"],
               "i_base_pu": native["device"]["i_ac1_mag_pu"],
               "i_base_phase_deg": -math.degrees(math.atan2(float(case.get("q_mvar", 0.0)), float(case.get("p_mw", 0.0))))}
    dc_case = {**case, "case_type": "canonical_dc_current_source", "spectrum": native["device"]["dc_spectrum"],
               "i_base_pu": native["device"]["i_dc0_mag_pu"]}
    return {"ac": independent_single_ac(ac_case), "dc": independent_dc(dc_case)}


def branch_error(native: dict[str, Any], reference: dict[int, complex], dc: bool) -> float:
    rows = native["dc_branches" if dc else "branches"]
    if not rows:
        return 0.0
    vals = {int(x["order"]): float(x["i_pu"]) for x in rows[0]["orders"]}
    return max((abs(vals[h] - abs(i)) for h, i in reference.items() if h in vals), default=0.0)


def benchmark_case(binary: Path, case: dict[str, Any], tolerance: float,
                   opendss_tolerance: float, gridlabd_tolerance: float,
                   gridlabd: dict[str, Any]) -> dict[str, Any]:
    native = run_native(binary, case)
    domain = case["domain"]
    phase = domain == "ac_3ph"
    if domain == "ac":
        ref = independent_single_ac(case)
        nord = native_single_orders(native)
        direct = opendss_single(case)
        open_status = "direct_equivalent_network"
        gld = gridlabd_single(gridlabd, case) if gridlabd.get("available") else None
    elif domain == "ac_3ph":
        ref = independent_three_phase(case)
        nord = native_phase_orders(native)
        try:
            direct = opendss_three_phase_decoupled(case)
            open_status = "direct_phase_domain_equivalent"
        except ValueError:
            direct = None
            open_status = "unsupported_without_sequence_model_substitution"
        gld = None
    elif domain == "dc":
        ref = independent_dc(case)
        nord = native_single_orders(native, dc=True)
        direct = None
        open_status = "unsupported_dc_harmonic_network"
        gld = None
    else:
        href = hybrid_reference(case, native)
        nac, ndc = native_single_orders(native), native_single_orders(native, dc=True)
        ac_cmp = full_comparison(nac, href["ac"]["orders"], False)
        dc_cmp = full_comparison(ndc, href["dc"]["orders"], False)
        ref = href
        nord = nac
        direct = opendss_single({**case, "spectrum": native["device"]["ac_spectrum"],
                                 "i_base_pu": native["device"]["i_ac1_mag_pu"],
                                 "i_base_phase_deg": -math.degrees(math.atan2(float(case.get("q_mvar", 0.0)), float(case.get("p_mw", 0.0))))})
        gld_case = {**case, "spectrum": native["device"]["ac_spectrum"],
                    "i_base_pu": native["device"]["i_ac1_mag_pu"],
                    "i_base_phase_deg": -math.degrees(math.atan2(float(case.get("q_mvar", 0.0)),
                                                                  float(case.get("p_mw", 0.0))))}
        gld = gridlabd_single(gridlabd, gld_case) if gridlabd.get("available") else None
        open_status = "ac_port_equivalent_network_only"
        cmp_ref = {"ac": ac_cmp, "dc": dc_cmp,
                   "max_complex_voltage_error_pu": max(ac_cmp["max_complex_voltage_error_pu"], dc_cmp["max_complex_voltage_error_pu"]),
                   "max_branch_current_magnitude_error_pu": max(branch_error(native, href["ac"]["branch_currents"], False), branch_error(native, href["dc"]["branch_currents"], True))}
        open_cmp = full_comparison(nac, direct["orders"], False)
        gld_cmp = full_comparison(nac, gld["orders"], False) if gld else None
        ok = (native["ok"] and cmp_ref["max_complex_voltage_error_pu"] <= tolerance
              and cmp_ref["max_branch_current_magnitude_error_pu"] <= tolerance
              and open_cmp["max_complex_voltage_error_pu"] <= opendss_tolerance
              and (gld_cmp is None or gld_cmp["max_complex_voltage_error_pu"] <= gridlabd_tolerance))
        return {"id": case["id"], "domain": domain, "balance": case["balance"], "orders": {"ac": case["ac_orders"], "dc": case["dc_orders"]},
                "native_ok": native["ok"], "independent_reference": {"status": "executed", "comparison": cmp_ref},
                "opendss": {"status": open_status, "comparison": open_cmp},
                "gridlabd": {"status": "executed_frequency_slices" if gld else "unsupported_or_unavailable",
                             "comparison": gld_cmp, "slice_count": len(gld["slices"]) if gld else 0,
                             "slices": gridlabd_slice_report(gld["slices"]) if gld else []},
                "metrics": {"ac": derived_metrics(nac, False), "dc": derived_metrics(ndc, False)}, "pass": ok}

    cmp_ref = full_comparison(nord, ref["orders"], phase)
    cmp_ref["max_branch_current_magnitude_error_pu"] = 0.0 if phase else branch_error(native, ref["branch_currents"], domain == "dc")
    open_cmp = full_comparison(nord, direct["orders"], phase) if direct else None
    gld_cmp = full_comparison(nord, gld["orders"], False) if gld else None
    ok = (native["ok"] and cmp_ref["max_complex_voltage_error_pu"] <= tolerance
          and cmp_ref["max_branch_current_magnitude_error_pu"] <= tolerance)
    if open_cmp is not None:
        ok = ok and open_cmp["max_complex_voltage_error_pu"] <= opendss_tolerance
    if gld_cmp is not None:
        ok = ok and gld_cmp["max_complex_voltage_error_pu"] <= gridlabd_tolerance
    return {"id": case["id"], "domain": domain, "balance": case["balance"],
            "orders": case.get("ac_orders", case.get("dc_orders")), "native_ok": native["ok"],
            "independent_reference": {"status": "executed", "comparison": cmp_ref},
            "opendss": {"status": open_status, "comparison": open_cmp},
            "gridlabd": {"status": "executed_frequency_slices" if gld else "unsupported_or_unavailable",
                         "comparison": gld_cmp, "slice_count": len(gld["slices"]) if gld else 0,
                         "slices": gridlabd_slice_report(gld["slices"]) if gld else []},
            "metrics": derived_metrics(nord, phase), "pass": ok}


def markdown(report: dict[str, Any]) -> str:
    lines = ["# Harmonic Power-Flow Cross-Engine Matrix", "",
             f"Generated: `{report['generated_at']}`", "",
             "| Case | Domain | Balance | Native vs NumPy max | OpenDSS status | OpenDSS max | GridLAB-D | Result |",
             "|---|---|---|---:|---|---:|---|---|"]
    for row in report["cases"]:
        ne = row["independent_reference"]["comparison"]["max_complex_voltage_error_pu"]
        oc = row["opendss"]["comparison"]
        oe = f"{oc['max_complex_voltage_error_pu']:.3e}" if oc else "n/a"
        gc = row["gridlabd"]["comparison"]
        ge = f"{row['gridlabd']['slice_count']} slices / {gc['max_complex_voltage_error_pu']:.3e}" if gc else row["gridlabd"]["status"]
        lines.append(f"| `{row['id']}` | {row['domain']} | {row['balance']} | {ne:.3e} | {row['opendss']['status']} | {oe} | {ge} | {'PASS' if row['pass'] else 'FAIL'} |")
    s = report["summary"]
    lines += ["", "## Summary", "",
              f"- Cases: {s['case_count']} ({s['passed']} passed, {s['failed']} failed)",
              f"- OpenDSS numeric cases: {s['opendss_numeric_cases']}",
              f"- Maximum native/independent voltage error: `{s['max_native_reference_error_pu']:.6e} pu`",
              f"- Maximum native/OpenDSS voltage error: `{s['max_native_opendss_error_pu']:.6e} pu`",
              f"- Maximum branch-current magnitude error: `{s['max_branch_current_error_pu']:.6e} pu`",
              f"- Maximum voltage IHD error: `{s['max_voltage_ihd_error_pct']:.6e} percentage points`",
              f"- Maximum voltage THD error: `{s['max_voltage_thd_error_pct']:.6e} percentage points`",
              f"- Maximum sequence-component error: `{s['max_sequence_component_error_pu']:.6e} pu`",
              f"- GridLAB-D numerical frequency slices: {s['gridlabd_numeric_slices']}",
              f"- Maximum native/GridLAB-D complex-voltage error: `{s['max_native_gridlabd_error_pu']:.6e} pu`", "",
              "## Interpretation", "",
              "OpenDSS rows are direct frequency-domain solves of an electrically equivalent Reactor/Isource network. "
              "The hybrid rows compare only the AC port in OpenDSS; DC and converter coupling are independently checked with a dense complex nodal solve. "
              "The zero-sequence case is not translated to a decoupled OpenDSS circuit because doing so would discard its distinct zero-sequence impedance.", "",
              "GridLAB-D has no native harmonic-order API. Each reported slice is therefore an independently executed complex steady-state network at one authored harmonic frequency: R(h), hX, source impedance, and current injection are explicit. The measured voltage/current transfer impedance is applied to the same source phasor before comparison; this validates the decoupled per-order network equation without claiming nonlinear waveform/FFT coverage.", ""]
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--cpp-bin", type=Path, default=REPO / "build/macos-release/tests/validate_harmonics_xref")
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--tolerance", type=float, default=2e-8)
    parser.add_argument("--opendss-tolerance", type=float, default=2e-6)
    parser.add_argument("--gridlabd-tolerance", type=float, default=2e-5)
    parser.add_argument("--require-gridlabd", action="store_true")
    parser.add_argument("--min-gridlabd-slices", type=int, default=20)
    args = parser.parse_args()
    if not args.cpp_bin.exists():
        raise SystemExit(f"missing native harmonic emitter: {args.cpp_bin}")
    cpp_bin = args.cpp_bin.resolve()
    out_path = args.out.resolve()
    gridlabd = gridlabd_capability()
    if args.require_gridlabd and not gridlabd.get("available"):
        raise SystemExit(gridlabd["detail"])
    with tempfile.TemporaryDirectory(prefix="hpf_dss_") as tmp:
        dss.Basic.DataPath(tmp)
        rows = [benchmark_case(cpp_bin, c, args.tolerance, args.opendss_tolerance,
                               args.gridlabd_tolerance, gridlabd) for c in cases()]
    open_errors = [r["opendss"]["comparison"]["max_complex_voltage_error_pu"] for r in rows if r["opendss"]["comparison"]]
    gridlabd_errors = [r["gridlabd"]["comparison"]["max_complex_voltage_error_pu"]
                       for r in rows if r["gridlabd"]["comparison"]]
    gridlabd_slices = sum(r["gridlabd"]["slice_count"] for r in rows)
    ref_errors = [r["independent_reference"]["comparison"]["max_complex_voltage_error_pu"] for r in rows]
    reference_parts = []
    for row in rows:
        comparison = row["independent_reference"]["comparison"]
        reference_parts.extend([comparison["ac"], comparison["dc"]] if "ac" in comparison else [comparison])
    report = {"schema": "hacdcpf.harmonics.cross_engine_matrix.v2", "generated_at": __import__("datetime").datetime.now(__import__("datetime").timezone.utc).isoformat(),
              "tolerances_pu": {"native_numpy": args.tolerance, "native_opendss": args.opendss_tolerance,
                                "native_gridlabd": args.gridlabd_tolerance},
              "engines": {"native": "HACDCPF frequency-domain HPF", "opendss": dss.Basic.Version().splitlines()[0],
                          "gridlabd": gridlabd, "reference": f"NumPy {np.__version__} dense complex nodal solve"},
              "methodology": {"native_vs_reference": "independent equation assembly", "opendss": "direct/equivalent AC frequency-domain network",
                              "gridlabd": "one executed complex steady-state network per harmonic order, with explicit R(h), hX and source impedance"},
              "cases": rows,
              "summary": {"case_count": len(rows), "passed": sum(r["pass"] for r in rows), "failed": sum(not r["pass"] for r in rows),
                          "opendss_numeric_cases": len(open_errors), "gridlabd_numeric_cases": len(gridlabd_errors),
                          "gridlabd_numeric_slices": gridlabd_slices,
                          "max_native_reference_error_pu": max(ref_errors, default=0.0), "max_native_opendss_error_pu": max(open_errors, default=0.0),
                          "max_native_gridlabd_error_pu": max(gridlabd_errors, default=0.0),
                          "max_branch_current_error_pu": max((x.get("max_branch_current_magnitude_error_pu", 0.0) for x in reference_parts), default=0.0),
                          "max_voltage_ihd_error_pct": max((x["max_voltage_ihd_error_pct"] for x in reference_parts), default=0.0),
                          "max_voltage_thd_error_pct": max((x["max_voltage_thd_error_pct"] for x in reference_parts), default=0.0),
                          "max_sequence_component_error_pu": max((x["max_sequence_component_error_pu"] for x in reference_parts), default=0.0),
                          "max_sequence_unbalance_error_pct": max((x["max_sequence_unbalance_error_pct"] for x in reference_parts), default=0.0)}}
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    out_path.with_suffix(".md").write_text(markdown(report), encoding="utf-8")
    print(json.dumps(report["summary"], indent=2))
    print(f"wrote {out_path}")
    if args.require_gridlabd and gridlabd_slices < args.min_gridlabd_slices:
        raise SystemExit(f"GridLAB-D produced {gridlabd_slices} numerical slices; at least {args.min_gridlabd_slices} are required")
    if gridlabd_errors and max(gridlabd_errors) > args.gridlabd_tolerance:
        raise SystemExit(f"GridLAB-D complex-voltage error {max(gridlabd_errors):.9g} exceeds {args.gridlabd_tolerance:.9g}")
    return 0 if report["summary"]["failed"] == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
