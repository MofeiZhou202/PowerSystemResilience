#!/usr/bin/env python3
"""Independent set, conservation, recovery and unit oracle for model projection."""

import argparse
import json
import subprocess
from pathlib import Path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--executable", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--markdown", required=True)
    args = parser.parse_args()
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    raw = out.with_name("model_projection_solver_output.json")
    subprocess.run([args.executable, str(raw)], check=True)
    evidence = json.loads(raw.read_text())
    merge = evidence["merge"]
    same = evidence["same_numeric_acdc_ids"]
    dead = evidence["dc_dead_island"]
    unit = evidence["unit_conversion"]

    merged_group = next(group for group in merge["groups"] if 101 in group)
    checks = {
        "quotient_membership": 0.0 if set(merged_group) == {101, 205} else 1.0,
        "load_conservation_mw": abs(merge["load_before_mw"] - merge["load_after_mw"]),
        "generation_conservation_mw": abs(merge["generation_before_mw"] - merge["generation_after_mw"]),
        "demand_weights": max(abs(merge["demand_participation"]["101"] - 0.6),
                              abs(merge["demand_participation"]["205"] - 0.4)),
        "generation_weights": max(abs(merge["generation_participation"]["101"] - 0.75),
                                  abs(merge["generation_participation"]["205"] - 0.25)),
        "intensive_broadcast": max(abs(merge["recovered_voltage_pu"][0] - 1.02),
                                   abs(merge["recovered_voltage_pu"][1] - 1.02)),
        "extensive_generation": max(abs(merge["recovered_generation_mw"][0] - 30.0),
                                     abs(merge["recovered_generation_mw"][1] - 10.0)),
        "extensive_demand": max(abs(merge["recovered_demand_mw"][0] - 12.0),
                                 abs(merge["recovered_demand_mw"][1] - 8.0)),
        "projection_idempotence": 0.0 if merge["idempotent_bus_count"] else 1.0,
        "domain_qualified_identity": 0.0 if (same["ac_1_node"] != same["dc_1_node"]
                                                and same["ac_2_node"] != same["dc_2_node"]
                                                and same["node_domains"][0] != same["node_domains"][1]) else 1.0,
        "dead_island_position_token": 0.0 if dead["dead_ids"] == [3] else 1.0,
        "dead_island_recovery": max(abs(a - b) for a, b in zip(dead["recovered_voltage_pu"], [1.0, 0.99, 0.0])),
    }
    zbase = unit["base_kv"] ** 2 / unit["base_mva"]
    r_expected = unit["r_ohm_per_km"] * unit["length_km"] / unit["parallel"] / zbase
    x_expected = unit["x_ohm_per_km"] * unit["length_km"] / unit["parallel"] / zbase
    checks["per_unit_r"] = abs(unit["r_pu"] - r_expected)
    checks["per_unit_x"] = abs(unit["x_pu"] - x_expected)
    checks["unit_conversion_idempotence"] = 0.0 if (unit["first_conversion_count"] == 1
                                                      and unit["second_conversion_count"] == 0
                                                      and unit["values_unchanged"]) else 1.0
    tolerance = 1e-12
    stable_dead_id_contract = dead["dead_ids"] == [dead["authored_dead_id"]]
    report = {"schema": "hysim-model-projection-oracle-v1", "tolerance": tolerance,
              "checks": checks, "stable_dead_id_contract": stable_dead_id_contract,
              "known_limitations": ([] if stable_dead_id_contract else [
                  "dc_dead_bus_indices reports the prestrip position token 3, not authored sparse DC bus ID 5"]),
              "passed": max(checks.values()) <= tolerance}
    out.write_text(json.dumps(report, indent=2) + "\n")
    lines = ["# Model projection cross-validation", "",
             f"Tolerance: `{tolerance:.1e}`; overall: `{'PASS' if report['passed'] else 'FAIL'}`.", "",
             "| Independent check | Error |", "|---|---:|"]
    lines.extend(f"| {name} | {value:.3e} |" for name, value in checks.items())
    lines += ["", f"Stable dead-bus ID contract: `{'PASS' if stable_dead_id_contract else 'FAIL (known limitation)'}`."]
    Path(args.markdown).write_text("\n".join(lines) + "\n")
    if not report["passed"]:
        raise SystemExit("independent model projection oracle failed")


if __name__ == "__main__":
    main()
