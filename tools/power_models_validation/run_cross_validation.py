#!/usr/bin/env python3
"""Independent equation oracle for the AML power-model builder evidence cases."""

import argparse
import json
import subprocess
from pathlib import Path


def map_value(values, key):
    return float(values[key])


def branch_value(values, from_bus, to_bus):
    suffix = f"{from_bus}{to_bus}"
    matches = [float(value) for key, value in values.items()
               if key.replace("->", "").replace("\u2192", "") == suffix]
    if len(matches) != 1:
        raise ValueError(f"cannot uniquely resolve branch {from_bus}->{to_bus}: {values}")
    return matches[0]


def validate_dcopf(case):
    # DC nodal balance and f=b(theta_i-theta_j), theory_modeling_foundations.tex.
    data = case["input"]
    result = case
    cheap = map_value(result["dispatch_mw"], "cheap")
    local = map_value(result["dispatch_mw"], "local")
    theta_1 = map_value(result["angles_rad"], "b1")
    theta_2 = map_value(result["angles_rad"], "b2")
    flow = branch_value(result["flows_mw"], "b1", "b2")
    objective = data["cheap_cost"] * cheap + data["local_cost"] * local
    return {
        "balance_mw": abs(cheap + local - data["demand_b2_mw"]),
        "flow_equation_mw": abs(flow - data["susceptance"] * (theta_1 - theta_2)),
        "line_limit_mw": max(0.0, abs(flow) - data["line_limit_mw"]),
        "analytic_dispatch_mw": max(abs(cheap - 40.0), abs(local - 40.0)),
        "objective_recompute": abs(objective - result["solve"]["objective"]),
        "analytic_objective": abs(result["solve"]["objective"] - 2400.0),
    }


def validate_lindistflow(case):
    # Baran-Wu lossless balance and squared-voltage drop, same theory chapter.
    result = case
    voltage = result["voltage_sq_pu"]
    p12 = branch_value(result["branch_p_mw"], "b1", "b2")
    p23 = branch_value(result["branch_p_mw"], "b2", "b3")
    q12 = branch_value(result["branch_q_mvar"], "b1", "b2")
    q23 = branch_value(result["branch_q_mvar"], "b2", "b3")
    v1, v2, v3 = (map_value(voltage, bus) for bus in ("b1", "b2", "b3"))
    expected_v2 = 1.0 - 2.0 * (0.01 * 1.5 + 0.02 * 0.3)
    expected_v3 = expected_v2 - 2.0 * (0.015 * 0.5 + 0.025 * 0.1)
    return {
        "p_balance_mw": max(abs(p12 - p23 - 1.0), abs(p23 - 0.5)),
        "q_balance_mvar": max(abs(q12 - q23 - 0.2), abs(q23 - 0.1)),
        "voltage_drop_pu2": max(
            abs(v2 - v1 + 2.0 * (0.01 * p12 + 0.02 * q12)),
            abs(v3 - v2 + 2.0 * (0.015 * p23 + 0.025 * q23))),
        "analytic_flow": max(abs(p12 - 1.5), abs(p23 - 0.5),
                             abs(q12 - 0.3), abs(q23 - 0.1)),
        "analytic_voltage_pu2": max(abs(v1 - 1.0), abs(v2 - expected_v2),
                                    abs(v3 - expected_v3)),
    }


def validate_scuc(case):
    # Merit-order dispatch under balance/capacity/integrality, same theory chapter.
    demand = case["input"]["demand_mw"]
    cost = case["input"]["cost_per_mwh"]
    pmax = case["input"]["pmax_mw"]
    dispatch = case["dispatch_mw"]
    commitment = case["commitment"]
    balances = []
    capacity = []
    objective = 0.0
    for period in case["input"]["periods"]:
        total = 0.0
        for generator in ("cheap", "peaker"):
            power = float(dispatch[generator][period])
            online = float(commitment[generator][period])
            total += power
            objective += cost[generator] * power
            capacity.append(max(0.0, power - pmax[generator] * online))
            capacity.append(max(0.0, -power))
        balances.append(abs(total - demand[period]))
    analytic = max(abs(float(dispatch["cheap"]["t1"]) - 30.0),
                   abs(float(dispatch["peaker"]["t1"])),
                   abs(float(dispatch["cheap"]["t2"]) - 50.0),
                   abs(float(dispatch["peaker"]["t2"]) - 20.0))
    return {
        "balance_mw": max(balances),
        "capacity_mw": max(capacity),
        "integrality": max(abs(float(value) - round(float(value)))
                           for row in commitment.values() for value in row.values()),
        "analytic_dispatch_mw": analytic,
        "objective_recompute": abs(objective - case["solve"]["objective"]),
        "analytic_objective": abs(case["solve"]["objective"] - 1400.0),
        "price_contract": 0.0 if (not case["price_valid"] and
                                  "uncertified" in case["price_validity_reason"]) else 1.0,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--executable", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--markdown", required=True)
    parser.add_argument("--repetitions", type=int, default=5)
    args = parser.parse_args()
    if args.repetitions < 1:
        parser.error("--repetitions must be positive")

    output = Path(args.out)
    output.parent.mkdir(parents=True, exist_ok=True)
    checks = {}
    status_errors = {}
    backend_errors = {}
    run_reports = []
    # Explicit alternation makes backend coverage deterministic; see the
    # validation protocol in numerical_cross_validation.tex.
    dcopf_solver = "NativeDualSimplex"
    lindistflow_solver_schedule = ["NativeDualSimplex", "NativeIPMLP"]
    for run_index in range(args.repetitions):
        requested_lindistflow_solver = lindistflow_solver_schedule[
            run_index % len(lindistflow_solver_schedule)]
        raw = output.with_name(f"power_models_solver_output_run_{run_index + 1}.json")
        subprocess.run(
            [args.executable, str(raw), dcopf_solver, requested_lindistflow_solver],
            check=True)
        evidence = json.loads(raw.read_text())
        run_checks = {
            "dcopf": validate_dcopf(evidence["dcopf"]),
            "lindistflow": validate_lindistflow(evidence["lindistflow"]),
            "scuc": validate_scuc(evidence["scuc"]),
        }
        run_status_errors = {
            name: 0.0 if case["solve"]["has_primal"] and case["solve"]["optimal"] else 1.0
            for name, case in evidence.items() if name != "schema"
            and name not in {"requested_dcopf_solver", "requested_lindistflow_solver"}
        }
        expected_solver_result = {
            "NativeDualSimplex": "NativeSimplex",
            "NativeIPMLP": "NativeIPMLP",
        }
        run_backend_errors = {
            "dcopf": 0.0 if evidence["dcopf"]["solve"]["solver"]
            == expected_solver_result[dcopf_solver] else 1.0,
            "lindistflow": 0.0 if evidence["lindistflow"]["solve"]["solver"]
            == expected_solver_result[requested_lindistflow_solver] else 1.0,
            "scuc": 0.0 if evidence["scuc"]["solve"]["solver"] == "StrictHiGHS" else 1.0,
        }
        for case_name, group in run_checks.items():
            aggregate = checks.setdefault(case_name, {})
            for check_name, value in group.items():
                aggregate[check_name] = max(aggregate.get(check_name, 0.0), value)
        for case_name, value in run_status_errors.items():
            status_errors[case_name] = max(status_errors.get(case_name, 0.0), value)
        for case_name, value in run_backend_errors.items():
            backend_errors[case_name] = max(backend_errors.get(case_name, 0.0), value)
        run_observed = [value for group in run_checks.values() for value in group.values()]
        run_observed.extend(run_status_errors.values())
        run_observed.extend(run_backend_errors.values())
        run_reports.append({
            "run": run_index + 1,
            "requested_dcopf_solver": dcopf_solver,
            "requested_lindistflow_solver": requested_lindistflow_solver,
            "solvers": {
                name: case["solve"]["solver"]
                for name, case in evidence.items()
                if name not in {"schema", "requested_dcopf_solver",
                                "requested_lindistflow_solver"}
            },
            "max_error": max(run_observed),
            "checks": run_checks,
            "optimal_status_errors": run_status_errors,
            "backend_contract_errors": run_backend_errors,
        })

    # Repetition count and 1e-7 gate are fixed by the validation protocol in
    # docs/modules/power_models/chapters/numerical_cross_validation.tex.
    tolerance = 1e-7
    observed = [value for group in checks.values() for value in group.values()]
    observed.extend(status_errors.values())
    observed.extend(backend_errors.values())
    report = {
        "schema": "hysim-power-models-cross-validation-v2",
        "tolerance": tolerance,
        "repetitions": args.repetitions,
        "dcopf_solver": dcopf_solver,
        "lindistflow_solver_schedule": lindistflow_solver_schedule,
        "checks": checks,
        "optimal_status_errors": status_errors,
        "backend_contract_errors": backend_errors,
        "runs": run_reports,
        "max_error": max(observed),
        "passed": max(observed) <= tolerance,
        "scope": ["dcopf-lp", "lindistflow-lp", "scuc-milp"],
        "limitations": [
            "The nonlinear ACOPF and hybrid ACDCOPF builders are not independently certified by this oracle.",
            "SCUC has system balance only and its MILP duals are not certified prices.",
            ("NativeIPMLP DCOPF is not admitted: an audit run had 1.7402e-7 "
             "absolute objective error at the fixed 1e-7 gate."),
        ],
    }
    output.write_text(json.dumps(report, indent=2) + "\n")

    lines = ["# Power-model builder cross-validation", "",
             f"Repetitions: `{args.repetitions}`; tolerance: `{tolerance:.1e}`; "
             f"worst error: `{report['max_error']:.3e}`; "
             f"overall: `{'PASS' if report['passed'] else 'FAIL'}`.", "",
             "| Case | Check | Worst error |", "|---|---|---:|"]
    for case_name, group in checks.items():
        lines.extend(f"| {case_name} | {name} | {value:.3e} |"
                     for name, value in group.items())
    lines += ["", "Scope: DCOPF LP, LinDistFlow LP and system-balance SCUC MILP.", ""]
    Path(args.markdown).write_text("\n".join(lines))
    if not report["passed"]:
        raise SystemExit("independent power-model builder oracle failed")


if __name__ == "__main__":
    main()
