#!/usr/bin/env python3
"""Regenerate the OpenDSS transformer-tap short-circuit reference fixture."""

import json
import math
from pathlib import Path

import dss


TAPS = (0.95, 1.0, 1.05)


def main() -> None:
    engine = dss.DSS
    engine.Text.Command = "Clear"
    engine.Text.Command = (
        "New Circuit.tap_crossval basekv=110 pu=1 phases=3 bus1=source"
    )
    engine.Text.Command = (
        "Edit Vsource.source basekv=110 pu=1 phases=3 frequency=50 "
        "mvasc3=1000 x1r1=10 mvasc1=1000 x0r0=10"
    )
    engine.Text.Command = (
        "New Transformer.t windings=2 phases=3 "
        "buses=(source.1.2.3.0,load.1.2.3.0) conns=(wye,wye) "
        "kvs=(110,20) kvas=(100000,100000) %rs=(0.5,0.5) "
        "xhl=9.949874371"
    )
    engine.Text.Command = "Set voltagebases=(110,20)"
    engine.Text.Command = "CalcVoltageBases"

    cases = []
    circuit = engine.ActiveCircuit
    for tap in TAPS:
        engine.Text.Command = f"Edit Transformer.t wdg=2 tap={tap}"
        engine.Text.Command = "Solve mode=faultstudy"
        circuit.SetActiveBus("load")
        z1 = list(circuit.ActiveBus.Zsc1)
        z0 = list(circuit.ActiveBus.Zsc0)
        ik_ka = 20.0 / math.sqrt(3.0) / math.hypot(z1[0], z1[1])
        kappa = 1.02 + 0.98 * math.exp(-3.0 * abs(z1[0] / z1[1]))
        cases.append(
            {
                "lv_tap_pu": tap,
                "z1_ohm": z1,
                "z0_ohm": z0,
                "ik3_ka": ik_ka,
                "ip_iec60909_ka": math.sqrt(2.0) * kappa * ik_ka,
            }
        )

    fixture = {
        "engine": f"DSS C-API via dss-python {dss.__version__}",
        "circuit": "110/20 kV, 100 MVA Yg-Yg transformer; 1000 MVA source",
        "notes": (
            "OpenDSS validates the symmetrical Thevenin current and R/X. "
            "Peak current is calculated from its Zsc1 using IEC 60909 method A."
        ),
        "cases": cases,
    }
    output = Path(__file__).resolve().parents[2] / (
        "external_data/short_circuit_validation/opendss_transformer_taps.json"
    )
    output.write_text(json.dumps(fixture, indent=2) + "\n", encoding="utf-8")
    print(output)


if __name__ == "__main__":
    main()
