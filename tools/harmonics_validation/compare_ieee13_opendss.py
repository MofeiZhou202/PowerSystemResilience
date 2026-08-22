#!/usr/bin/env python3
"""Multi-bus feeder harmonic penetration cross-check vs OpenDSS.

Runs tools/harmonics_validation/validate_harmonics_ieee13 (hacdcpf 3ph HPF on
a real OpenDSS feeder) and replays the identical balanced current injections
in OpenDSS harmonic mode (three single-phase ISources per order at the same
bus), then compares per-order per-phase complex bus voltages.

Usage:
  tools/harmonics_validation/.venv/bin/python \
      tools/harmonics_validation/compare_ieee13_opendss.py \
      [--dss feeder.dss] [--bus 675] [--i-ref-amps 100] [--out report.json]
"""
from __future__ import annotations

import argparse
import cmath
import json
import math
import subprocess
import tempfile
from pathlib import Path

import opendssdirect as dss

REPO = Path(__file__).resolve().parents[2]
DEFAULT_DSS = (REPO / "external_data/opendss_ieee_pes/opendss_reference/13_node" /
               "official_full/IEEE13Nodeckt.dss")
CPP_BIN = REPO / "build/macos-release/tests/validate_harmonics_ieee13"
ORDERS = [5, 7, 11, 13]
SPECTRUM = {5: 20.0, 7: 100.0 / 7.0, 11: 100.0 / 11.0, 13: 100.0 / 13.0}


def sequence_angles(order: int) -> dict[int, float]:
    """Balanced phase angles for harmonic order (a reference 0 deg)."""
    if order % 3 == 1:    # positive sequence
        return {0: 0.0, 1: -120.0, 2: 120.0}
    if order % 3 == 2:    # negative sequence
        return {0: 0.0, 1: 120.0, 2: -120.0}
    return {0: 0.0, 1: 0.0, 2: 0.0}


def run_dss(cmd: str) -> None:
    dss.Text.Command(cmd)


def opendss_harmonic_voltages(dss_file: Path, src_bus: str,
                              i_ref_amps: float) -> dict[int, dict[str, dict[int, complex]]]:
    """Solve OpenDSS harmonic mode with ISource injections at src_bus.

    OpenDSS reports bus voltages only for the most recently solved harmonic,
    so each order is solved and sampled separately.
    Returns {order: {bus_name: {node: complex volts}}}.
    """
    dss.Basic.ClearAll()
    run_dss("clear")
    run_dss(f'compile "{dss_file}"')
    run_dss("solve mode=snap")
    # match hacdcpf study: loads excluded from the harmonic network
    run_dss("disable load.*")
    # NOTE: the Vsource stays enabled — in harmonic mode it appears as its
    # internal impedance (R + j h X from MVAsc/X1R1), matching the
    # ExternalGrid stamping on the hacdcpf side.
    # one ISource per order per phase, each at its own harmonic frequency
    for order in ORDERS:
        mag = i_ref_amps * SPECTRUM[order] / 100.0
        ang = sequence_angles(order)
        for ph in (1, 2, 3):
            run_dss(
                f"new isource.inj_h{order}_p{ph} phases=1 "
                f"bus1={src_bus}.{ph} amps={mag:.17g} "
                f"angle={ang[ph - 1]:.17g} frequency={60.0 * order:.17g}")
    run_dss("set mode=harmonic")
    out: dict[int, dict[str, dict[int, complex]]] = {}
    for order in ORDERS:
        run_dss(f"set harmonics=({order})")
        run_dss("solve")
        if not dss.Solution.Converged():
            raise RuntimeError(f"OpenDSS harmonic solve did not converge at h={order}")
        nodes = dss.Circuit.AllNodeNames()
        raw = dss.Circuit.AllBusVolts()
        volts = [complex(raw[i], raw[i + 1]) for i in range(0, len(raw), 2)]
        for name, v in zip(nodes, volts):
            bus, node = name.rsplit(".", 1)
            out.setdefault(order, {}).setdefault(bus.lower(), {})[int(node)] = v
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dss", type=Path, default=DEFAULT_DSS)
    ap.add_argument("--bus", type=str, default="675")
    ap.add_argument("--i-ref-amps", type=float, default=100.0)
    ap.add_argument("--out", type=Path, default=None)
    ap.add_argument("--cpp-bin", type=Path, default=CPP_BIN)
    ap.add_argument("--tolerance", type=float, default=2e-3)
    args = ap.parse_args()
    args.dss = args.dss.resolve()  # loader requires an absolute master path
    if args.out is not None:
        args.out = args.out.resolve()

    with tempfile.TemporaryDirectory(prefix="feeder_hpf_") as tmp:
        hy_path = Path(tmp) / "hy.json"
        subprocess.run(
            [str(args.cpp_bin.resolve()), str(args.dss), args.bus, str(args.i_ref_amps),
             str(hy_path)], check=True)
        hy = json.loads(hy_path.read_text())
        dss.Basic.DataPath(tmp)
        od = opendss_harmonic_voltages(args.dss, args.bus, args.i_ref_amps)

    # per-bus base kV (line-neutral, as reported by OpenDSS) for pu conversion
    kvbase: dict[str, float] = {}
    for bus in next(iter(od.values())):
        dss.Circuit.SetActiveBus(bus)
        kvbase[bus] = dss.Bus.kVBase()

    rows = []
    max_err = 0.0
    for brow in hy["buses"]:
        name = brow["name"].lower()
        if name not in next(iter(od.values())):
            continue
        vbase = kvbase[name] * 1e3  # OpenDSS kVBase is already line-neutral
        for orow in brow["orders"]:
            order = int(orow["order"])
            if order == 1 or order not in od:
                continue
            for ph in (0, 1, 2):
                node = ph + 1
                if node not in od[order][name]:
                    continue
                hy_v = complex(orow["phases"][ph]["real"],
                               orow["phases"][ph]["imag"])
                od_v = od[order][name][node] / vbase
                err = abs(hy_v - od_v)
                max_err = max(max_err, err)
                rows.append({"bus": name, "node": node, "order": order,
                             "hy_mag": abs(hy_v), "od_mag": abs(od_v),
                             "hy_ang": math.degrees(cmath.phase(hy_v)),
                             "od_ang": math.degrees(cmath.phase(od_v)),
                             "err_pu": err})

    report = {"feeder": args.dss.name,
              "source_bus": args.bus,
              "i_ref_amps": args.i_ref_amps,
              "orders": ORDERS,
              "n_points": len(rows),
              "tolerance_pu": args.tolerance,
              "passed": bool(rows) and max_err <= args.tolerance,
              "max_complex_voltage_error_pu": max_err,
              "worst": sorted(rows, key=lambda r: -r["err_pu"])[:10],
              "opendss_version": dss.Basic.Version()}
    text = json.dumps(report, indent=2)
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(text + "\n")
        print(f"wrote {args.out}")
    print(f"{args.dss.name} harmonic penetration vs OpenDSS: n={len(rows)} points, "
          f"max |dV| = {max_err:.3e} pu")
    for r in report["worst"][:5]:
        print(f"  {r['bus']:>10}.{r['node']} h{r['order']:>2}: "
              f"hy={r['hy_mag']:.6f}∠{r['hy_ang']:8.2f}  "
              f"od={r['od_mag']:.6f}∠{r['od_ang']:8.2f}  err={r['err_pu']:.2e}")
    if not rows:
        raise SystemExit("IEEE13 comparison produced no common bus/phase/order points")
    return 0 if max_err <= args.tolerance else 1


if __name__ == "__main__":
    raise SystemExit(main())
