"""OpenDSS cross-check for the harmonic power-flow solver.

Uses OpenDSS (via OpenDSSDirect.py, which bundles the DSS engine — no separate
binary) as a fully independent network solver, validating the C++ solver's
per-order linear nodal solve against a completely different code base.

Modelling map (case.json -> OpenDSS), chosen so the two are apples-to-apples:

  * Self-consistent single-phase SI with Z_base = 1 ohm:  basekv = 1 kV,
    I_base = 1000 A.  Then a per-unit impedance value equals its ohm value, a
    per-unit current i maps to i*1000 A, and OpenDSS's AllBusMagPu (referred to
    the 1 kV base) reads back directly in per unit.
  * The SLACK source is a Vsource with pu = 0, i.e. a harmonic short behind its
    sub-transient reactance X1 = x'' ohms — exactly the solver's "harmonic short
    behind Z_int" grounding.
  * The nonlinear load is an Isource (ideal current source) at the load bus,
    matching the solver's Norton current-source resource model.
  * Each order h is solved with `set mode=direct` at frequency h*f1.  OpenDSS
    scales every inductive reactance by h/1 automatically, reproducing
    Z(h) = r + j*h*x; the Isource injects the order-h current only.

THD is then sqrt(sum_{h>1}|V_h|^2)/|V_1| with |V_1| = 1 pu (the solver uses the
flat stored fundamental as the reference).

Importable as solve_opendss(case) -> {'buses': {...}} or None when OpenDSS is
unavailable; runs standalone to print the OpenDSS spectrum.
"""

import json
import math
import os
import sys

I_BASE_A = 1000.0   # current base (A) for the Z_base = 1 ohm, basekv = 1 kV system
V_BASE_V = 1000.0   # voltage base (V) = Z_base * I_base


def _try_import():
    try:
        import opendssdirect as dss  # noqa: F401
        return dss
    except Exception as e:                      # pragma: no cover - env dependent
        print(f"[opendss] SKIP: OpenDSSDirect.py not available ({e})")
        return None


def solve_opendss(case):
    dss = _try_import()
    if dss is None:
        return None

    xpp = float(case.get("source_xpp_pu", 0.2))
    r_line = float(case["line"]["r_pu"])
    x_line = float(case["line"]["x_pu"])
    load_bus = int(case.get("load_bus", 2))
    i_base = float(case.get("i_base_pu", 0.3))
    base_phase = float(case.get("i_base_phase_deg", 0.0))
    f1 = float(case.get("f1_hz", 60.0))
    orders = list(case["ac_orders"])
    spectrum = {int(s["order"]): (float(s["mag_percent"]),
                                  float(s.get("phase_deg", 0.0)))
                for s in case["spectrum"]}

    def cmd(s):
        dss.Text.Command(s)

    cmd("clear")
    cmd("new circuit.xref basekv=1.0 phases=1 bus1=1 "
        f"pu=0.0 R1={0.0} X1={xpp} R0={0.0} X0={xpp}")
    # Series branch as a Reactor (R constant, X = ω·L scales linearly with order),
    # NOT a Line: OpenDSS lines carry a default frequency-dependent / earth-return
    # impedance model, whereas this solver (and the docs) use plain Z(h)=r+j·h·x.
    # A reactor reproduces exactly that, keeping the comparison apples-to-apples.
    cmd(f"new reactor.r12 phases=1 bus1=1 bus2=2 R={r_line} X={x_line}")
    # Ideal harmonic current source at the load bus (Norton resource).
    cmd(f"new isource.hload bus1={load_bus} phases=1 amps=0 angle=0 frequency={f1}")
    cmd(f"set frequency={f1}")
    cmd("set basefrequency=" + str(f1))

    node_names = None
    bus_v = {1: {1: 1.0 + 0j}, 2: {1: 1.0 + 0j}}   # fundamental reference = 1 pu

    for h in orders:
        mag_pct, ph = spectrum.get(h, (0.0, 0.0))
        amps = (mag_pct / 100.0) * i_base * I_BASE_A
        ang = ph + base_phase
        cmd(f"edit isource.hload amps={amps} angle={ang} frequency={h * f1}")
        cmd(f"set mode=direct frequency={h * f1}")
        cmd("solve")

        if node_names is None:
            node_names = [n.lower() for n in dss.Circuit.AllNodeNames()]
        volts = dss.Circuit.AllBusVolts()          # [re0, im0, re1, im1, ...]
        for idx, name in enumerate(node_names):
            busnum = name.split(".")[0]
            if busnum not in ("1", "2"):
                continue
            vc = complex(volts[2 * idx], volts[2 * idx + 1]) / V_BASE_V
            bus_v[int(busnum)][h] = vc

    def thd(vmap):
        v1 = abs(vmap.get(1, 0.0))
        if v1 < 1e-12:
            return 0.0
        acc = sum(abs(vmap[k]) ** 2 for k in vmap if k != 1)
        return math.sqrt(acc) / v1 * 100.0

    buses = {}
    for b in (1, 2):
        buses[b] = {
            "v_fund_pu": abs(bus_v[b].get(1, 0.0)),
            "thd_pct": thd(bus_v[b]),
            "orders": {h: {"mag": abs(bus_v[b][h])} for h in bus_v[b]},
        }
    return {"buses": buses, "engine": "OpenDSSDirect.py " + str(dss.__version__)}


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    case_path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(here, "case.json")
    with open(case_path) as f:
        case = json.load(f)
    res = solve_opendss(case)
    if res is None:
        sys.exit(0)
    print("OpenDSS engine:", res["engine"])
    for b, d in res["buses"].items():
        print(f"bus {b}: THD={d['thd_pct']:.4f}%")
        for h in sorted(d["orders"]):
            print(f"    h{h:>2}: |V|={d['orders'][h]['mag']:.6e} pu")


if __name__ == "__main__":
    main()
