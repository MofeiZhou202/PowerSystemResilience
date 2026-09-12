#!/usr/bin/env python3
"""Independent equation oracle for the day-ahead market SCED/LMP and settlement.

This script does not import or link hacdcpf. For each deterministic single-bus
"copper-plate" case emitted by validate_market_xref it re-derives, from the
generator offers and the demand alone:

  1. the merit-order dispatch stack and the marginal (price-setting) offer, whose
     price is the uniform locational marginal price on a lossless single bus;
  2. the energy balance (dispatch = demand);
  3. the settlement identities that any correct clearing must satisfy —
     resource energy revenue = sum_g LMP_bus(g) * dispatch_g, customer energy
     payment = LMP * demand, zero congestion rent on one bus, and a zero
     cash-flow residual (revenue adequacy).

These are the standard locational-marginal-pricing identities documented in
docs/theory/market_simulation_mathematical_models.md and produced by
build_pricing_model / settle_market in src/market/market_simulation.cpp.

Usage:
    run_cross_validation.py --executable <bin> --out <json> --markdown <md>
Exit status is 1 if any check exceeds the tolerance.

Negative control:
    run_cross_validation.py --executable <bin> --out <json> --markdown <md> \
        --negative-control
deliberately corrupts one LMP value in the solver payload before validation
(first case, first bus, +1.0 currency/MWh) and expects the oracle to reject it.
The exit-status logic is inverted in this mode only: exit status is 0 when the
corruption is detected (overall FAIL), and 1 when the corrupted payload
unexpectedly passes, which would mean the oracle is not sensitive to LMP
errors.
"""
import argparse
import json
import subprocess
from pathlib import Path

TOLERANCE = 1e-6


def merit_order(generators, demand):
    """Return (marginal_price, dispatch_by_index) for a lossless single bus."""
    order = sorted(range(len(generators)), key=lambda g: generators[g]["offer_price"])
    dispatch = [0.0] * len(generators)
    remaining = demand
    marginal_price = 0.0
    for g in order:
        if remaining <= 1e-12:
            break
        take = min(generators[g]["pmax_mw"], remaining)
        dispatch[g] = take
        remaining -= take
        if take > 1e-12:
            marginal_price = generators[g]["offer_price"]
    return marginal_price, dispatch, remaining


def validate_case(case):
    generators = case["generators"]
    demand = case["demand_mw"]
    lmp = case["lmp_per_bus"]
    settlement = case["settlement"]

    marginal_price, dispatch, unserved = merit_order(generators, demand)

    # 1) LMP is the marginal offer price, uniform across the single bus.
    lmp_error = max((abs(value - marginal_price) for value in lmp), default=0.0)

    # 2) Dispatch matches the merit-order stack.
    dispatch_error = max(
        (abs(generators[g]["dispatch_mw"] - dispatch[g]) for g in range(len(generators))),
        default=0.0)

    # 3) Energy balance (no shedding expected for these served cases).
    served = sum(g["dispatch_mw"] for g in generators)
    balance_error = abs(served - (demand - case["total_load_shedding_mw"]))
    unserved_error = abs(case["total_load_shedding_mw"] - unserved)

    # 4) Revenue-adequacy identities re-derived from LMP, dispatch and demand.
    lmp_value = marginal_price
    expected_resource_revenue = sum(
        lmp_value * g["dispatch_mw"] for g in generators)
    expected_customer_payment = lmp_value * (demand - case["total_load_shedding_mw"])
    resource_revenue_error = abs(
        settlement["resource_energy_revenue"] - expected_resource_revenue)
    customer_payment_error = abs(
        settlement["customer_energy_payment"] - expected_customer_payment)
    congestion_error = abs(settlement["congestion_rent"])
    cashflow_error = abs(settlement["cashflow_residual"])

    feasible_error = 0.0 if case["feasible"] else 1.0

    return {
        "marginal_lmp": lmp_error,
        "merit_dispatch": dispatch_error,
        "energy_balance": balance_error,
        "unserved": unserved_error,
        "resource_revenue_identity": resource_revenue_error,
        "customer_payment_identity": customer_payment_error,
        "congestion_rent_zero": congestion_error,
        "cashflow_residual": cashflow_error,
        "feasible_flag": feasible_error,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--executable", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--markdown", required=True)
    parser.add_argument(
        "--negative-control",
        action="store_true",
        help="corrupt one LMP value and require the oracle to FAIL; "
             "exit status is 0 only when the corruption is detected")
    args = parser.parse_args()

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    raw = out.with_name("market_solver_output.json")
    subprocess.run([args.executable, str(raw)], check=True)
    payload = json.loads(raw.read_text())

    corruption = None
    if args.negative_control:
        target = payload["cases"][0]
        corrupted_value = target["lmp_per_bus"][0] + 1.0
        corruption = {
            "case": target["name"],
            "field": "lmp_per_bus[0]",
            "original": target["lmp_per_bus"][0],
            "corrupted": corrupted_value,
        }
        target["lmp_per_bus"][0] = corrupted_value

    summary = {case["name"]: validate_case(case) for case in payload["cases"]}
    observed = [value for row in summary.values() for value in row.values()]
    worst = max(observed) if observed else 0.0
    oracle_passed = worst <= TOLERANCE
    # Positive mode passes when the oracle accepts the untouched payload.
    # Negative-control mode passes (exit 0) only when the oracle rejects the
    # deliberately corrupted payload; an unexpected PASS means the checks are
    # blind to LMP errors and must be reported as a failure.
    overall_passed = (not oracle_passed) if args.negative_control else oracle_passed
    report = {
        "schema": "hysim-market-sced-cross-validation-v1",
        "tolerance": TOLERANCE,
        "negative_control": args.negative_control,
        "checks": summary,
        "max_error": worst,
        "oracle_passed": oracle_passed,
        "passed": overall_passed,
        "scope": ["copper-plate-sced-lmp-settlement"],
        "limitations": [
            "Cases are single-bus copper-plate economic dispatch; congested "
            "network LMPs, losses, reserve co-optimization and N-1 are exercised "
            "by the market Catch2 suite, not this analytic oracle.",
            "The oracle re-derives the merit-order LMP and the revenue-adequacy "
            "identities; it is an independent-implementation check, not a second "
            "market model.",
        ],
    }
    if corruption is not None:
        report["negative_control_corruption"] = corruption
    out.write_text(json.dumps(report, indent=2) + "\n")

    verdict = "PASS" if overall_passed else "FAIL"
    if args.negative_control:
        verdict = ("PASS (negative control: corruption detected as expected)"
                   if overall_passed else
                   "FAIL (negative control: corrupted LMP unexpectedly accepted)")
    lines = ["# Market SCED/LMP cross-validation", "",
             f"Tolerance: `{TOLERANCE:.1e}`; worst error: `{worst:.3e}`; "
             f"overall: `{verdict}`.", ""]
    if corruption is not None:
        lines += [f"Negative control: `{corruption['case']}` "
                  f"`{corruption['field']}` rewritten "
                  f"{corruption['original']} -> {corruption['corrupted']}; "
                  f"oracle verdict: `{'PASS' if oracle_passed else 'FAIL'}`.", ""]
    lines += ["| Case | LMP | Dispatch | Balance | Revenue id | Cashflow |",
              "|---|---:|---:|---:|---:|---:|"]
    for name, row in summary.items():
        lines.append(
            f"| {name} | {row['marginal_lmp']:.2e} | {row['merit_dispatch']:.2e} | "
            f"{row['energy_balance']:.2e} | {row['resource_revenue_identity']:.2e} | "
            f"{row['cashflow_residual']:.2e} |")
    lines += ["", "Scope: single-bus copper-plate merit-order LMP and "
              "revenue-adequacy settlement identities, independently re-derived.", ""]
    Path(args.markdown).write_text("\n".join(lines))

    if not overall_passed:
        if args.negative_control:
            raise SystemExit(
                "negative control failed: corrupted LMP was not detected")
        raise SystemExit("independent market SCED/LMP oracle failed")


if __name__ == "__main__":
    main()
