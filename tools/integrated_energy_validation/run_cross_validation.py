#!/usr/bin/env python3
"""Independent equation oracle for the campus integrated-energy evidence cases."""

import argparse
import json
import math
import subprocess
from pathlib import Path


def max_abs(values):
    return max((abs(v) for v in values), default=0.0)


def vec_max(lhs, rhs):
    return max_abs(a - b for a, b in zip(lhs, rhs))


def validate_case(case):
    d, r = case["input"], case["result"]
    n, dt = d["n"], d["dt"]
    residuals = {name: [] for name in (
        "electric_balance", "heat_balance", "hydrogen_balance", "fuel_balance",
        "conversion", "storage_state", "availability", "transport", "carbon")}
    for t in range(n):
        electric_supply = (r["grid_import"][t] - r["grid_export"][t] + r["solar"][t]
                           + r["wind"][t] + r["chp_p"][t] + r["fuelcell_p"][t]
                           + r["e_discharge"][t])
        electric_demand = (d["electric_load"][t] + r["electrolyzer_h2_p"][t]
                           + r["electrolyzer_fuel_p"][t] + r["heatpump_p"][t]
                           + r["e_charge"][t] + r["ev_p"][t] + r["ccus_p"][t])
        residuals["electric_balance"].append(electric_supply - electric_demand)
        residuals["heat_balance"].append(
            r["chp_q"][t] + r["heatpump_q"][t] + r["q_discharge"][t]
            + d["eta_wasteheat"] * r["heatpump_p"][t] - d["heat_load"][t] - r["q_charge"][t])
        residuals["hydrogen_balance"].append(
            r["electrolyzer_h"][t] + r["h_discharge"][t] + r["hw_discharge"][t]
            + r["hs_discharge"][t] - d["hydrogen_load"][t] - r["fuelcell_h"][t]
            - r["hv_h"][t] - r["h_charge"][t] - r["hw_charge"][t] - r["hs_charge"][t])
        residuals["fuel_balance"].append(
            r["external_fuel"][t] + r["synthetic_fuel"][t] - r["chp_fuel"][t]
            - d["fuel_load"][t] - r["icv_f"][t])
        residuals["conversion"].extend((
            r["electrolyzer_h"][t] - d["eta_electrolysis"] * r["electrolyzer_h2_p"][t],
            r["synthetic_fuel"][t] - d["eta_power_to_fuel"] * r["electrolyzer_fuel_p"][t],
            r["fuelcell_p"][t] - d["eta_fuelcell"] * r["fuelcell_h"][t],
            r["heatpump_q"][t] - d["cop_heatpump"] * r["heatpump_p"][t]))
        residuals["availability"].extend((
            r["solar"][t] + r["solar_curtail"][t] - d["solar_available"][t],
            r["wind"][t] + r["wind_curtail"][t] - d["wind_available"][t]))
        residuals["transport"].extend((
            r["ev_km"][t] - d["ev_ratio"] * d["transport_km"][t],
            r["hv_km"][t] - d["hv_ratio"] * d["transport_km"][t],
            r["icv_km"][t] - d["icv_ratio"] * d["transport_km"][t],
            r["ev_p"][t] - d["alpha_ev"] * d["ev_ratio"] * d["transport_km"][t] / dt,
            r["hv_h"][t] - d["alpha_hv"] * d["hv_ratio"] * d["transport_km"][t] / dt,
            r["icv_f"][t] - d["alpha_icv"] * d["icv_ratio"] * d["transport_km"][t] / dt))
        gross = d["grid_carbon"][t] * dt * r["grid_import"][t] + d["fuel_carbon"] * dt * r["external_fuel"][t]
        residuals["carbon"].extend((r["emissions"][t] - gross,
                                     max(0.0, r["captured"][t] - d["capture_fraction"] * gross),
                                     max(0.0, gross - r["captured"][t] - r["residual_carbon"][t]),
                                     max(0.0, d["ccus_energy"] * r["captured"][t] / dt - r["ccus_p"][t])))

        states = (
            ("e_state", "e_charge", "e_discharge", "rho_e", 0.0),
            ("q_state", "q_charge", "q_discharge", "rho_q", 0.0),
            ("h_state", "h_charge", "h_discharge", "rho_h",
             dt * d["eta_w2d"] * r["h_w2d"][t] - dt * r["h_d2w"][t]),
            ("hw_state", "hw_charge", "hw_discharge", "rho_hw",
             dt * d["eta_d2w"] * r["h_d2w"][t] + dt * d["eta_s2w"] * r["h_s2w"][t]
             - dt * r["h_w2d"][t] - dt * r["h_w2s"][t]),
            ("hs_state", "hs_charge", "hs_discharge", "rho_hs",
             dt * d["eta_w2s"] * r["h_w2s"][t] - dt * r["h_s2w"][t]))
        for state, charge, discharge, rho, transfer in states:
            predicted = (d[rho] * r[state][t] + d["eta_charge"] * dt * r[charge][t]
                         - dt * r[discharge][t] / d["eta_discharge"] + transfer)
            residuals["storage_state"].append(r[state][t + 1] - predicted)

    cost = 0.0
    for t in range(n):
        cost += d["buy_price"][t] * dt * r["grid_import"][t] - d["sell_price"][t] * dt * r["grid_export"][t]
        cost += d["fuel_cost"] * dt * r["external_fuel"][t]
        cost += d["solar_om"] * dt * r["solar"][t] + d["wind_om"] * dt * r["wind"][t]
        cost += d["chp_om"] * dt * (r["chp_p"][t] + r["chp_q"][t])
        cost += d["heatpump_om"] * dt * r["heatpump_q"][t]
        cost += d["electrolyzer_om"] * dt * (r["electrolyzer_h2_p"][t] + r["electrolyzer_fuel_p"][t])
        cost += d["fuelcell_om"] * dt * r["fuelcell_p"][t]
        cost += d["storage_cost"] * dt * (r["e_charge"][t] + r["e_discharge"][t] + r["q_charge"][t] + r["q_discharge"][t])
        cost += d["h_storage_cost"] * dt * sum(r[name][t] for name in ("h_charge", "h_discharge", "hw_charge", "hw_discharge", "hs_charge", "hs_discharge", "h_d2w", "h_w2d", "h_w2s", "h_s2w"))
        cost += d["curtailment_cost"] * dt * (r["solar_curtail"][t] + r["wind_curtail"][t])
        cost += d["carbon_penalty"] * r["residual_carbon"][t] + d["ccus_cost"] * r["captured"][t]
        cost += d["ccus_power_cost"] * dt * r["ccus_p"][t]
    maxima = {name: max_abs(values) for name, values in residuals.items()}
    maxima["cost_recompute"] = abs(cost - r["total_cost"])
    return maxima


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--executable", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--markdown", required=True)
    args = parser.parse_args()
    output = Path(args.out)
    output.parent.mkdir(parents=True, exist_ok=True)
    raw = output.with_name("integrated_energy_solver_output.json")
    subprocess.run([args.executable, str(raw)], check=True)
    payload = json.loads(raw.read_text())
    cases = payload["cases"]
    summary = {name: validate_case(case) for name, case in cases.items()}

    one = cases["one_step_analytic"]["result"]
    storage = cases["two_step_storage"]["result"]
    analytic = {
        "one_step_dispatch": max(abs(one["solar"][0] - 2.0), abs(one["grid_import"][0] - 3.0)),
        "one_step_objective": abs(one["total_cost"] - 304.0),
        "storage_dispatch": max(abs(storage["e_charge"][0] - 1.0 / 0.9),
                                abs(storage["e_discharge"][1] - 0.9),
                                abs(storage["e_state"][1] - 1.0)),
        "storage_objective": abs(storage["total_cost"] - (20.0 * (1.0 + 1.0 / 0.9) + 120.0 * 0.1)),
    }
    tolerance = 1e-7
    observed = [v for row in summary.values() for v in row.values()] + list(analytic.values())
    report = {"schema": "hysim-integrated-energy-cross-validation-v1",
              "tolerance": tolerance, "equation_residuals": summary,
              "analytic_oracle_errors": analytic, "passed": max(observed) <= tolerance}
    output.write_text(json.dumps(report, indent=2) + "\n")
    lines = ["# Integrated-energy cross-validation", "",
             f"Tolerance: `{tolerance:.1e}`; overall: `{'PASS' if report['passed'] else 'FAIL'}`.", "",
             "| Case | Electric | Heat | Hydrogen | Fuel | Storage | Cost |", "|---|---:|---:|---:|---:|---:|---:|"]
    for name, row in summary.items():
        lines.append(f"| {name} | {row['electric_balance']:.3e} | {row['heat_balance']:.3e} | {row['hydrogen_balance']:.3e} | {row['fuel_balance']:.3e} | {row['storage_state']:.3e} | {row['cost_recompute']:.3e} |")
    lines += ["", "Analytic oracle errors: " + ", ".join(f"{k}={v:.3e}" for k, v in analytic.items()) + ".", ""]
    Path(args.markdown).write_text("\n".join(lines))
    if not report["passed"]:
        raise SystemExit("independent integrated-energy oracle failed")


if __name__ == "__main__":
    main()
