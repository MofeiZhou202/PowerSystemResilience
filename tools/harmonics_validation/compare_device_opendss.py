#!/usr/bin/env python3
"""Device-level harmonic validation against OpenDSS where OpenDSS has a device.

The AC/DC VSC path uses HACDCPF's HarmonicNIC model and validates the resulting
AC network response against OpenDSS using the same converter harmonic current.
The script also probes OpenDSS' native VSConverter class and records whether a
native harmonic solve is available in the installed DSS C-API build.

OpenDSS does not provide a native DC/DC converter class in the DSS C-API build
used by this project. The report therefore marks DC/DC native validation as not
available instead of silently substituting a different device.
"""

from __future__ import annotations

import argparse
import cmath
import json
import math
import subprocess
import tempfile
from pathlib import Path
from typing import Any

import opendssdirect as dss


REPO = Path(__file__).resolve().parents[2]


def complex_from_row(row: dict[str, Any]) -> complex:
    if "real" in row and "imag" in row:
        return complex(float(row["real"]), float(row["imag"]))
    return cmath.rect(float(row["mag"]), math.radians(float(row["ang_deg"])))


def phasor_dict(z: complex) -> dict[str, float]:
    return {
        "real": z.real,
        "imag": z.imag,
        "mag": abs(z),
        "ang_deg": math.degrees(cmath.phase(z)),
    }


def run_dss(cmd: str) -> None:
    dss.Text.Command(cmd)


def dss_complex_array(values: list[float]) -> list[complex]:
    return [complex(values[i], values[i + 1]) for i in range(0, len(values), 2)]


def default_cpp_bin() -> Path:
    candidates = [
        REPO / "build/macos-release/tests/validate_harmonics_xref",
        REPO / "build/macos-release/validate_harmonics_xref",
        REPO / "build/linux-release/tests/validate_harmonics_xref",
        REPO / "build/linux-release/validate_harmonics_xref",
    ]
    for path in candidates:
        if path.exists():
            return path
    return candidates[0]


def run_hacdcpf(cpp_bin: Path, case: dict[str, Any]) -> dict[str, Any]:
    with tempfile.TemporaryDirectory(prefix="hpf_device_") as tmp:
        case_path = Path(tmp) / "case.json"
        out_path = Path(tmp) / "out.json"
        case_path.write_text(json.dumps(case, indent=2), encoding="utf-8")
        subprocess.run([str(cpp_bin), str(case_path), str(out_path)], check=True)
        return json.loads(out_path.read_text(encoding="utf-8"))


def set_dss_data_path(path: Path) -> None:
    """Keep OpenDSS runtime files out of the source tree."""

    dss.Basic.DataPath(str(path))


def order_voltage(result: dict[str, Any], bus: int, order: int, dc: bool = False) -> complex:
    key = "dc_buses" if dc else "buses"
    for brow in result[key]:
        if int(brow["bus"]) != bus:
            continue
        for row in brow["orders"]:
            if int(row["order"]) == order:
                return complex_from_row(row)
    raise KeyError((bus, order, dc))


def device_current(result: dict[str, Any], order: int, dc: bool = False) -> complex:
    key = "dc_current_by_order" if dc else "ac_current_by_order"
    for row in result["device"][key]:
        if int(row["order"]) == order:
            return complex_from_row(row["current"])
    raise KeyError((order, dc))


def opendss_ac_network_response(case: dict[str, Any], order: int, current: complex) -> dict[int, complex]:
    """Solve the OpenDSS AC harmonic network for one converter current injection.

    The OpenDSS circuit uses Reactor elements, not Line, so its impedance law is
    the same as HACDCPF's harmonic network: Z(h) = R + j h X.
    """

    net = case["network"]
    line = net["line"]
    source_xpp = float(net["source_xpp_pu"])
    r = float(line.get("r_pu", 0.01))
    x = float(line.get("x_pu", 0.1))
    base_mva = float(net.get("base_mva", case.get("base_mva", 100.0)))
    base_kv = float(net.get("base_kv", 10.0))
    base_volts = base_kv * 1000.0
    base_amps = base_mva * 1.0e6 / base_volts
    z_base_ohm = base_volts / base_amps
    freq = 60.0 * order
    current_amps = current * base_amps
    source_xpp_ohm = source_xpp * z_base_ohm
    r_ohm = r * z_base_ohm
    x_ohm = x * z_base_ohm

    dss.Basic.ClearAll()
    run_dss("clear")
    run_dss(f"new circuit.hpf basekv={base_kv:.17g} phases=1 bus1=source.1 pu=1 angle=0 frequency=60")
    run_dss(f"new reactor.src phases=1 bus1=source.1 bus2=source.0 R=0 X={source_xpp_ohm:.17g}")
    run_dss(f"new reactor.line phases=1 bus1=source.1 bus2=conv.1 R={r_ohm:.17g} X={x_ohm:.17g}")
    run_dss(
        "new isource.vsc_h{h} phases=1 bus1=conv.1 amps={amps:.17g} angle={ang:.17g} frequency={freq:.17g}".format(
            h=order,
            amps=abs(current_amps),
            ang=math.degrees(cmath.phase(current_amps)),
            freq=freq,
        )
    )
    run_dss("solve mode=direct")
    run_dss("disable vsource.source")
    run_dss("set mode=harmonic")
    run_dss(f"set harmonics=({order})")
    run_dss("solve")
    if not dss.Solution.Converged():
        raise RuntimeError(f"OpenDSS AC harmonic solve did not converge for h={order}")

    names = [name.lower() for name in dss.Circuit.AllBusNames()]
    volts = dss_complex_array(dss.Circuit.AllBusVolts())
    by_name = {name: volts[i] / base_volts for i, name in enumerate(names)}
    return {1: by_name["source"], 2: by_name["conv"]}


def probe_vsconverter_device() -> dict[str, Any]:
    """Probe native OpenDSS VSConverter availability and harmonic behavior.

    Some DSS C-API builds expose VSConverter but do not provide a robust harmonic
    voltage solve for a minimal converter-only test case. We report that state
    separately from the deterministic network-response cross-check.
    """

    report: dict[str, Any] = {
        "class_available": "VSConverter" in dss.Basic.Classes(),
        "instantiated": False,
        "fundamental_converged": False,
        "harmonic_solve_converged": False,
        "terminal_current_h5": None,
        "note": "",
    }
    if not report["class_available"]:
        report["note"] = "OpenDSS class list does not include VSConverter."
        return report

    try:
        dss.Basic.ClearAll()
        run_dss("clear")
        run_dss("new circuit.dev basekv=1 phases=1 bus1=source.1 pu=1 angle=0 frequency=60")
        run_dss("new load.dummy phases=1 bus1=source.1 kV=1 kW=1 kvar=0 model=1")
        run_dss("new spectrum.nic numharm=2 harmonic=(1 5) %mag=(100 20) angle=(0 0)")
        run_dss(
            "new VSConverter.vsc phases=2 Bus1=source.1.2 kVac=1 kVdc=1 kW=1 "
            "Ndc=1 Rac=0.01 Xac=0.1 m0=0.5 d0=0 VscMode=PacQac "
            "Pacref=1000 Qacref=0 spectrum=nic"
        )
        report["instantiated"] = True
        run_dss("solve mode=snap maxiterations=100 controlmode=off")
        report["fundamental_converged"] = bool(dss.Solution.Converged())
        if report["fundamental_converged"]:
            run_dss("set mode=harmonic")
            run_dss("set harmonics=(5)")
            run_dss("solve")
            report["harmonic_solve_converged"] = bool(dss.Solution.Converged())
            if dss.Circuit.SetActiveElement("VSConverter.vsc"):
                currents = dss_complex_array(dss.CktElement.Currents())
                if currents:
                    report["terminal_current_h5"] = phasor_dict(currents[0])
        if not report["harmonic_solve_converged"]:
            report["note"] = (
                "VSConverter is present and instantiable, but the minimal native "
                "harmonic solve did not converge in this DSS C-API build. The "
                "numeric pass/fail below therefore validates the deterministic "
                "network response to the converter harmonic current."
            )
    except Exception as exc:  # OpenDSS errors include useful text
        report["note"] = f"VSConverter probe failed: {exc}"
    return report


def max_ac_error_against_opendss(hacdcpf_result: dict[str, Any]) -> tuple[float, list[dict[str, Any]]]:
    rows: list[dict[str, Any]] = []
    max_err = 0.0
    orders = [int(row["order"]) for row in hacdcpf_result["device"]["ac_current_by_order"]]
    solved_orders = {int(row["order"]) for bus in hacdcpf_result["buses"] for row in bus["orders"]}
    for order in orders:
        if order <= 1 or order not in solved_orders:
            continue
        current = device_current(hacdcpf_result, order, dc=False)
        odss = opendss_ac_network_response(hacdcpf_result, order, current)
        for bus in (1, 2):
            cpp = order_voltage(hacdcpf_result, bus, order, dc=False)
            err = abs(cpp - odss[bus])
            max_err = max(max_err, err)
            rows.append(
                {
                    "order": order,
                    "bus": bus,
                    "hacdcpf": phasor_dict(cpp),
                    "opendss": phasor_dict(odss[bus]),
                    "abs_error_pu": err,
                }
            )
    return max_err, rows


def native_dcdc_status() -> dict[str, Any]:
    classes = dss.Basic.Classes()
    native = [cls for cls in classes if "dcdc" in cls.lower() or "dcconverter" in cls.lower()]
    return {
        "native_device_available": bool(native),
        "matching_classes": native,
        "note": (
            "OpenDSS DSS C-API exposes VSConverter but no native DC/DC converter "
            "class in this installation; DC/DC harmonic validation must use an "
            "equivalent ripple source or an external reference model."
        ),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--cpp-bin", type=Path, default=default_cpp_bin())
    parser.add_argument("--out", type=Path, default=REPO / "tools/harmonics_validation/device_opendss_report.json")
    parser.add_argument("--tolerance", type=float, default=1e-8)
    args = parser.parse_args()

    if not args.cpp_bin.exists():
        raise SystemExit(f"missing HACDCPF validation binary: {args.cpp_bin}")
    cpp_bin = args.cpp_bin.resolve()
    out_path = args.out.resolve()

    case = {
        "case_type": "device_vsc_nic",
        "base_mva": 100.0,
        "base_kv": 10.0,
        "dc_base_kv": 1.0,
        "p_mw": 30.0,
        "q_mvar": 0.0,
        "p_dc_mw": -30.0,
        "line": {"r_pu": 0.01, "x_pu": 0.1},
        "dc_line": {"r_pu": 0.05},
        "source_xpp_pu": 0.2,
        "dc_source_impedance_pu": 0.01,
        "ac_orders": [5, 7, 11, 13],
        "dc_orders": [6, 12],
    }

    with tempfile.TemporaryDirectory(prefix="opendss_device_") as dss_tmp:
        set_dss_data_path(Path(dss_tmp))

        hacdcpf_result = run_hacdcpf(cpp_bin, case)
        max_err, rows = max_ac_error_against_opendss(hacdcpf_result)
        report = {
            "ok": max_err <= args.tolerance,
            "tolerance": args.tolerance,
            "max_ac_network_error_pu": max_err,
            "opendss_version": dss.Basic.Version(),
            "opendss_data_path": str(Path(dss_tmp)),
            "hacdcpf": hacdcpf_result,
            "acdc_vsconverter_probe": probe_vsconverter_device(),
            "acdc_network_rows": rows,
            "dcdc_native_validation": native_dcdc_status(),
        }

        out_path.parent.mkdir(parents=True, exist_ok=True)
        out_path.write_text(json.dumps(report, indent=2), encoding="utf-8")

    print(f"wrote {out_path}")
    print(f"AC/DC converter harmonic network max |delta V| = {max_err:.3e} pu")
    print(f"OpenDSS VSConverter native probe: {report['acdc_vsconverter_probe']}")
    print(f"DC/DC native validation: {report['dcdc_native_validation']['note']}")
    print("OVERALL:", "PASS" if report["ok"] else "FAIL")
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
