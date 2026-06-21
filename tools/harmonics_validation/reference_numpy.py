"""Independent numpy reference for the harmonic power-flow solver.

This re-implements, from scratch and in a different language, the documented
linear nodal harmonic model used by the C++ solver
(hacdcpf::harmonics::solve_harmonic_power_flow):

    For each harmonic order h, build the bus admittance matrix Y(h) and solve
    Y(h) V(h) = I(h), where

        * line series admittance  ys(h) = 1 / (r + j*h*x)          [+ j*h*b charging]
        * slack source grounding  y_src(h) = 1 / (r_src + j*h*x'')
        * a tiny stray leakage    eps = 1e-9 on every diagonal (conditioning)
        * Norton current source   I_h = i_base * (mag%/100)
                                        * exp(j*(phase_deg + base_phase_deg)*pi/180)

    THD_V at a bus = sqrt(sum_{h>1} |V_h|^2) / |V_1| * 100, with |V_1| = 1 (flat
    stored fundamental, run_base_power_flow = false).

Because it is an entirely separate implementation, agreement with the C++ driver
(validate_harmonics_xref) to ~1e-7 is a genuine external check of the solver's
linear core, current-injection convention, branch-flow recovery, and THD formula.

Run standalone (prints the reference) or import `solve_reference(case)`.
"""

import json
import math
import os
import sys

import numpy as np

MIN_SHUNT = 1e-9  # mirrors HPFOptions::min_shunt_pu default


def solve_reference(case):
    """Return {'buses': {...}, 'branches': {...}} for the canonical 2-bus case."""
    xpp = float(case.get("source_xpp_pu", 0.2))
    r_line = float(case["line"]["r_pu"])
    x_line = float(case["line"]["x_pu"])
    b_line = float(case["line"].get("b_pu", 0.0))
    load_bus = int(case.get("load_bus", 2))
    i_base = float(case.get("i_base_pu", 0.3))
    base_phase = math.radians(float(case.get("i_base_phase_deg", 0.0)))
    orders = list(case["ac_orders"])
    spectrum = {int(s["order"]): (float(s["mag_percent"]),
                                  math.radians(float(s.get("phase_deg", 0.0))))
                for s in case["spectrum"]}

    # Node 0 = bus 1 (slack), node 1 = bus 2.  load_bus selects the injection node.
    inj_node = 0 if load_bus == 1 else 1

    bus_v = {1: {}, 2: {}}        # bus -> {order -> complex voltage}
    branch_i = {}                 # order -> |I_(1->2)|

    # Fundamental (order 1): flat stored voltage, no harmonic injection.
    bus_v[1][1] = 1.0 + 0j
    bus_v[2][1] = 1.0 + 0j

    for h in orders:
        ys = 1.0 / complex(r_line, h * x_line)         # line series
        ych = complex(0.0, h * b_line)                 # total line charging
        y_src = 1.0 / complex(0.0, h * xpp)            # slack grounding (r_src = 0)

        Y = np.zeros((2, 2), dtype=complex)
        # line between node0 (bus1) and node1 (bus2)
        Y[0, 0] += ys + 0.5 * ych
        Y[1, 1] += ys + 0.5 * ych
        Y[0, 1] += -ys
        Y[1, 0] += -ys
        # slack source grounding at node0 (bus1)
        Y[0, 0] += y_src
        # conditioning leakage on every diagonal
        Y[0, 0] += MIN_SHUNT
        Y[1, 1] += MIN_SHUNT

        I = np.zeros(2, dtype=complex)
        if h in spectrum:
            mag_pct, ph = spectrum[h]
            I[inj_node] = (mag_pct / 100.0) * i_base * np.exp(1j * (ph + base_phase))

        V = np.linalg.solve(Y, I)
        bus_v[1][h] = complex(V[0])
        bus_v[2][h] = complex(V[1])
        # series current bus1 -> bus2 at order h
        branch_i[h] = abs(ys * (V[0] - V[1]))

    def thd(vmap):
        v1 = abs(vmap[1])
        if v1 < 1e-12:
            return 0.0
        acc = sum(abs(vmap[h]) ** 2 for h in vmap if h != 1)
        return math.sqrt(acc) / v1 * 100.0

    buses = {}
    for b in (1, 2):
        buses[b] = {
            "v_fund_pu": abs(bus_v[b][1]),
            "thd_pct": thd(bus_v[b]),
            "orders": {h: {"mag": abs(bus_v[b][h]),
                           "ang_deg": math.degrees(np.angle(bus_v[b][h]))}
                       for h in bus_v[b]},
        }
    # branch THD_I referred to the fundamental branch current
    i1 = branch_i.get(1)
    if i1 is None:
        ys1 = 1.0 / complex(r_line, 1 * x_line)
        i1 = abs(ys1 * (bus_v[1][1] - bus_v[2][1]))
    thd_i = (math.sqrt(sum(branch_i[h] ** 2 for h in branch_i)) / i1 * 100.0
             if i1 > 1e-12 else 0.0)
    branches = {(1, 2): {"thd_i_pct": thd_i, "orders": dict(branch_i)}}
    return {"buses": buses, "branches": branches}


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    case_path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(here, "case.json")
    with open(case_path) as f:
        case = json.load(f)
    ref = solve_reference(case)
    for b, d in ref["buses"].items():
        print(f"bus {b}: V1={d['v_fund_pu']:.6f}  THD={d['thd_pct']:.4f}%")
        for h in sorted(d["orders"]):
            o = d["orders"][h]
            print(f"    h{h:>2}: |V|={o['mag']:.6e}  ang={o['ang_deg']:8.3f} deg")


if __name__ == "__main__":
    main()
