#!/usr/bin/env python3
"""Independent equation oracle for DL/T 2041-2025 hosting-capacity assessment.

This script does not import or link hacdcpf. For each deterministic
single-transformer supply area emitted by validate_hosting_xref it re-derives the
equipment-level hosting-capacity closed form

    S_d = max(0, (P - P_G + beta * n * S * cos(theta) + P_ESS + dP_ESS) / tau)

and the accessible-capacity subtractions

    C_grid = S_d - existing_DR,     C_reg = C_grid - registered_DR

from the reported supply-area aggregates and transformer parameters, and checks
them against the production figures. Case 1 is additionally the hand-verified
analytic anchor from tests/test_hosting_capacity.cpp (S_d in [10.5, 12.5],
accessible grid/registered minima 9.5 / 7.5).

The formula is documented in
docs/modules/analysis/chapters/theory_analysis_decision.tex and implemented in
src/analysis/hosting_capacity.cpp (assess_hosting_capacity).

Usage:
    run_cross_validation.py --executable <bin> --out <json> --markdown <md>
Exit status is 1 if any figure exceeds the tolerance.
"""
import argparse
import json
import subprocess
from pathlib import Path

TOLERANCE = 1e-9

ANALYTIC_ANCHORS = {
    "anchor_test_case": {
        "hosting_min_mw": 10.5,
        "hosting_max_mw": 12.5,
        "accessible_grid_min_mw": 9.5,
        "accessible_reg_min_mw": 7.5,
    },
}


def expected(case):
    s_total = case["n_parallel"] * case["sn_mva"]
    reverse_allow = case["beta"] * s_total * case["power_factor"]
    base = (case["supply_load_mw"] - case["supply_nondr_gen_mw"]
            + reverse_allow + case["supply_ess_charging_mw"])
    tau = case["tau_max"]
    sd_min = max(0.0, (base + case["dpess_min"]) / tau)
    sd_max = max(0.0, (base + case["dpess_max"]) / tau)
    if sd_min > sd_max:
        sd_min, sd_max = sd_max, sd_min
    grid_min = sd_min - case["supply_existing_dr_mw"]
    grid_max = sd_max - case["supply_existing_dr_mw"]
    return {
        "hosting_min_mw": sd_min,
        "hosting_max_mw": sd_max,
        "accessible_grid_min_mw": grid_min,
        "accessible_grid_max_mw": grid_max,
        "accessible_reg_min_mw": grid_min - case["registered_dr_mw"],
        "accessible_reg_max_mw": grid_max - case["registered_dr_mw"],
    }


def validate_case(case):
    predicted = expected(case)
    checks = {name: abs(value - case[name]) for name, value in predicted.items()}
    anchor = ANALYTIC_ANCHORS.get(case["name"])
    if anchor:
        for name, value in anchor.items():
            checks[f"analytic_{name}"] = abs(value - case[name])
    return checks


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--executable", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--markdown", required=True)
    args = parser.parse_args()

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    raw = out.with_name("hosting_capacity_solver_output.json")
    subprocess.run([args.executable, str(raw)], check=True)
    payload = json.loads(raw.read_text())

    summary = {case["name"]: validate_case(case) for case in payload["cases"]}
    observed = [value for row in summary.values() for value in row.values()]
    worst = max(observed) if observed else 0.0
    report = {
        "schema": "hysim-hosting-capacity-cross-validation-v1",
        "tolerance": TOLERANCE,
        "checks": summary,
        "max_error": worst,
        "passed": worst <= TOLERANCE,
        "scope": ["dlt2041-equipment-level-hosting-capacity"],
        "limitations": [
            "Validates the equipment-level closed form and accessible-capacity "
            "algebra from the reported supply-area aggregates; the supply-area "
            "topology walk and the optional engineering-verification (power flow, "
            "short circuit, harmonics) stage are exercised by the Catch2 suite.",
            "The oracle re-derives the same published formula; it is an "
            "independent-implementation check, not a second physical model.",
        ],
    }
    out.write_text(json.dumps(report, indent=2) + "\n")

    lines = ["# Hosting-capacity cross-validation", "",
             f"Tolerance: `{TOLERANCE:.1e}`; worst error: `{worst:.3e}`; "
             f"overall: `{'PASS' if report['passed'] else 'FAIL'}`.", "",
             "| Case | S_d,min | S_d,max | C_grid,min | C_reg,min |",
             "|---|---:|---:|---:|---:|"]
    for name, row in summary.items():
        lines.append(
            f"| {name} | {row['hosting_min_mw']:.2e} | {row['hosting_max_mw']:.2e} | "
            f"{row['accessible_grid_min_mw']:.2e} | {row['accessible_reg_min_mw']:.2e} |")
    lines += ["", "Scope: DL/T 2041-2025 equipment-level hosting-capacity and "
              "accessible-capacity closed form, independently re-derived.", ""]
    Path(args.markdown).write_text("\n".join(lines))

    if not report["passed"]:
        raise SystemExit("independent hosting-capacity oracle failed")


if __name__ == "__main__":
    main()
