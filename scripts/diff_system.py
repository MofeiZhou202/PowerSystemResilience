#!/usr/bin/env python3
"""Diff two HybridPowerSystem JSON files (disk vs browser-exported canvas) to
localize why the web UI diverges from the disk file. Focuses on the elements
that drive PF/OPF convergence: generators (bus + cost_c2), bus types/bounds,
and the connective topology (branches, transformers, VSC modes)."""
import json, sys

def load(p):
    with open(p, encoding="utf-8") as f:
        return json.load(f)

def ac(s): return s.get("ac", {}) or {}
def dc(s): return s.get("dc", {}) or {}

def gens(s):
    return [(g.get("index"), g.get("bus"), g.get("cost_c2"), g.get("cost_c1"),
             g.get("pmax_mw"), g.get("is_slack"), g.get("in_service"))
            for g in ac(s).get("generators", [])]

def bus_summary(s):
    types = {}
    bounds = set()
    for b in ac(s).get("buses", []):
        types[b.get("bus_type")] = types.get(b.get("bus_type"), 0) + 1
        bounds.add((b.get("vmin_pu"), b.get("vmax_pu")))
    return types, sorted(bounds)

def counts(s):
    a, d = ac(s), dc(s)
    return {
        "ac_buses": len(a.get("buses", [])),
        "ac_branches": len(a.get("branches", [])),
        "transformers_2w": len(a.get("transformers_2w", [])),
        "circuit_breakers": len(a.get("circuit_breakers", [])),
        "switches": len(a.get("switches", [])),
        "ac_loads": len(a.get("loads", [])),
        "pv_systems": len(a.get("pv_systems", [])),
        "generators": len(a.get("generators", [])),
        "dc_buses": len(d.get("buses", [])),
        "dc_branches": len(d.get("branches", [])),
        "vsc": len(s.get("vsc_converters", [])),
        "dcdc": len(s.get("dcdc_converters", [])),
    }

def vsc_modes(s):
    out = []
    for v in s.get("vsc_converters", []):
        out.append((v.get("index"), v.get("bus_ac"), v.get("bus_dc"),
                    v.get("control_mode") or v.get("mode") or v.get("ctrl_mode")))
    return sorted(out, key=lambda t: (t[0] is None, t[0]))

def trafo_wiring(s):
    out = []
    for t in ac(s).get("transformers_2w", []):
        out.append((t.get("index"), t.get("hv_bus"), t.get("lv_bus"), t.get("in_service")))
    return sorted(out, key=lambda x: (x[0] is None, x[0]))

def main():
    A, B = load(sys.argv[1]), load(sys.argv[2])
    print(f"A = {sys.argv[1]}")
    print(f"B = {sys.argv[2]}\n")

    print("== element counts ==")
    ca, cb = counts(A), counts(B)
    for k in ca:
        flag = "" if ca[k] == cb[k] else "   <<< DIFFERS"
        print(f"  {k:18} A={ca[k]:5} B={cb[k]:5}{flag}")

    print("\n== generators (index, bus, c2, c1, pmax, slack, in_svc) ==")
    ga, gb = gens(A), gens(B)
    print("  A:", ga)
    print("  B:", gb)
    if ga != gb:
        print("  <<< GENERATORS DIFFER")

    print("\n== bus types / voltage bounds ==")
    ta, ba = bus_summary(A); tb, bb = bus_summary(B)
    print(f"  A types={ta} bounds={ba}")
    print(f"  B types={tb} bounds={bb}")
    if ta != tb: print("  <<< BUS TYPE COUNTS DIFFER")

    print("\n== VSC modes (index, bus_ac, bus_dc, mode) ==")
    va, vb = vsc_modes(A), vsc_modes(B)
    print("  A:", va)
    print("  B:", vb)
    if va != vb: print("  <<< VSC MODES DIFFER")

    print("\n== transformer wiring (index, hv_bus, lv_bus, in_svc) ==")
    wa, wb = trafo_wiring(A), trafo_wiring(B)
    if wa != wb:
        print("  <<< TRANSFORMER WIRING DIFFERS")
        sa, sb = {w[0]: w for w in wa}, {w[0]: w for w in wb}
        for k in sorted(set(sa) | set(sb), key=lambda x: (x is None, x)):
            if sa.get(k) != sb.get(k):
                print(f"    trafo {k}: A={sa.get(k)}  B={sb.get(k)}")
    else:
        print("  (identical)")

if __name__ == "__main__":
    main()
