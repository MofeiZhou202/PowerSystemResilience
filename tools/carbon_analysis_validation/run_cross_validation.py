#!/usr/bin/env python3
"""Independent equation oracle for carbon-flow analysis evidence cases.

This script deliberately does not import or link hacdcpf. It reconstructs the
Kang carbon-emission-flow linear system A w = b from the network inputs emitted
by ``validate_carbon_xref`` (generator emission factors and dispatch, load
demand, and directed branch flows), re-solves it with an independent Gaussian
elimination, and checks the production nodal-intensity solution three ways:

  1. analytic closed forms  — single-source propagation (w equals the source
     emission factor everywhere), lossless dispatch-weighted mixing, and lossy
     mixing with a known load-bus intensity;
  2. independent re-solve   — |w_oracle - w_cpp|, from a from-scratch build and
     solve of the same linear system;
  3. nodal + global balance — the conservation residual ||A w_cpp - b|| and the
     system emission balance (generation = load + loss), plus the branch
     loss-allocation rule intensity = alpha w_from + (1-alpha) w_to.

The governing equations are documented in
docs/modules/carbon_analysis/chapters/theory_carbon_flow.tex and mirrored in
src/carbon_analysis/carbon_analysis.cpp (solve_carbon_matrix).

Usage:
    run_cross_validation.py --executable <bin> --out <json> --markdown <md>
Exit status is 1 if any check exceeds the tolerance.
"""
import argparse
import json
import subprocess
from pathlib import Path

TOLERANCE = 1e-7
CARBON_TRANSFER_EPS = 1e-9
REG_EPS = 1e-8


def solve_linear(matrix, rhs):
    """Solve a small dense system by Gauss-Jordan elimination with partial
    pivoting. Independent of the production sparse-QR solve."""
    n = len(matrix)
    aug = [list(matrix[i]) + [rhs[i]] for i in range(n)]
    for col in range(n):
        pivot = max(range(col, n), key=lambda r: abs(aug[r][col]))
        if abs(aug[pivot][col]) < 1e-15:
            raise ValueError("singular carbon system in the oracle")
        aug[col], aug[pivot] = aug[pivot], aug[col]
        pivot_value = aug[col][col]
        for row in range(n):
            if row == col:
                continue
            factor = aug[row][col] / pivot_value
            if factor == 0.0:
                continue
            for c in range(col, n + 1):
                aug[row][c] -= factor * aug[col][c]
    return [aug[i][n] / aug[i][i] for i in range(n)]


def build_carbon_system(case):
    """Re-derive A and b exactly as solve_carbon_matrix does, from inputs only."""
    n = case["node_count"]
    alpha = case["alpha"]
    p_out = [0.0] * n
    incoming = {}
    for edge in case["edges"]:
        src = edge["from_node_loc"]
        dst = edge["to_node_loc"]
        send = max(edge["send_mw"], 0.0)
        recv = max(edge["recv_mw"], 0.0)
        loss = min(max(edge["loss_mw"], 0.0), send)
        p_out[dst] += (1.0 - alpha) * loss
        sender_diagonal = send + (2.0 * alpha - 1.0) * loss
        p_out[src] += max(sender_diagonal, 0.0)
        carbon_transfer = recv + alpha * loss
        if carbon_transfer > CARBON_TRANSFER_EPS:
            incoming[(dst, src)] = incoming.get((dst, src), 0.0) + carbon_transfer

    rhs = [0.0] * n
    gen_at = [0.0] * n
    for source in case["sources"]:
        loc = source["node_loc"]
        rhs[loc] += source["ef"] * source["power_mw"]
        gen_at[loc] += source["power_mw"]

    load_at = [0.0] * n
    for load in case["loads"]:
        loc = load["node_loc"]
        p_out[loc] += load["demand_mw"]
        load_at[loc] += load["demand_mw"]

    row_max = [0.0] * n
    for (dst, _src), value in incoming.items():
        row_max[dst] = max(row_max[dst], abs(value))

    matrix = [[0.0] * n for _ in range(n)]
    inactive = [False] * n
    for i in range(n):
        diag = p_out[i] if abs(p_out[i]) > REG_EPS else REG_EPS
        row_max[i] = max(row_max[i], abs(diag))
        inactive[i] = load_at[i] < 1e-8 and gen_at[i] < 1e-8 and row_max[i] < 1e-6
        if inactive[i]:
            matrix[i][i] = 1.0
            rhs[i] = 0.0
        else:
            matrix[i][i] = diag
    for (dst, src), value in incoming.items():
        if not inactive[dst]:
            matrix[dst][src] += -value
    return matrix, rhs


def matvec_residual(matrix, w, rhs):
    n = len(matrix)
    worst = 0.0
    for i in range(n):
        acc = sum(matrix[i][j] * w[j] for j in range(n)) - rhs[i]
        worst = max(worst, abs(acc))
    return worst


def analytic_expectation(case):
    """Closed-form nodal intensities, derived independently of the solver."""
    name = case["name"]
    if name == "single_source_series":
        return [0.5, 0.5, 0.5]
    if name == "two_source_mixing":
        return [1.0, 0.0, 0.3]
    if name == "two_source_lossy":
        return [1.0, 0.0, 57.5 / 95.0]
    return None


def validate_case(case):
    alpha = case["alpha"]
    w_cpp = case["bus_intensity"]
    matrix, rhs = build_carbon_system(case)
    w_oracle = solve_linear(matrix, rhs)

    resolve_error = max((abs(a - b) for a, b in zip(w_oracle, w_cpp)), default=0.0)
    conservation_residual = matvec_residual(matrix, w_cpp, rhs)

    analytic = analytic_expectation(case)
    analytic_error = (
        max((abs(a - b) for a, b in zip(analytic, w_cpp)), default=0.0)
        if analytic is not None else 0.0
    )

    # Branch loss-allocation rule: intensity = alpha*w_from + (1-alpha)*w_to.
    loss_rule_error = 0.0
    for branch in case["result_branches"]:
        if branch["loss_mw"] <= 1e-12:
            continue
        expected = alpha * w_cpp[branch["from_node_loc"]] + (1.0 - alpha) * w_cpp[branch["to_node_loc"]]
        loss_rule_error = max(loss_rule_error, abs(expected - branch["intensity"]))

    # Independent system emission balance: generation = load + loss.
    total_gen = sum(s["ef"] * s["power_mw"] for s in case["sources"])
    total_load = sum(b["intensity"] * b["demand_mw"] for b in case["result_loads"])
    total_loss = sum(b["intensity"] * b["loss_mw"] for b in case["result_branches"])
    balance_error = abs(total_gen - (total_load + total_loss))

    # Load intensity equals the intensity of its host bus.
    load_bus_error = 0.0
    for load in case["result_loads"]:
        load_bus_error = max(load_bus_error, abs(load["intensity"] - w_cpp[load["node_loc"]]))

    solved_flag_error = 0.0 if case["matrix_solved"] else 1.0

    checks = {
        "analytic_intensity": analytic_error,
        "independent_resolve": resolve_error,
        "conservation_residual": conservation_residual,
        "loss_allocation_rule": loss_rule_error,
        "emission_balance": balance_error,
        "load_host_intensity": load_bus_error,
        "matrix_solved_flag": solved_flag_error,
    }
    return checks


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--executable", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--markdown", required=True)
    args = parser.parse_args()

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    raw = out.with_name("carbon_analysis_solver_output.json")
    subprocess.run([args.executable, str(raw)], check=True)
    payload = json.loads(raw.read_text())

    summary = {case["name"]: validate_case(case) for case in payload["cases"]}
    observed = [value for row in summary.values() for value in row.values()]
    worst = max(observed) if observed else 0.0
    report = {
        "schema": "hysim-carbon-analysis-cross-validation-v1",
        "tolerance": TOLERANCE,
        "checks": summary,
        "max_error": worst,
        "passed": worst <= TOLERANCE,
        "scope": ["ac-proportional-carbon-emission-flow"],
        "limitations": [
            "Cases are AC-only radial/meshed feeders; VSC/DC-DC/storage/export "
            "carbon paths are exercised only by the internal Catch2 suites.",
            "The oracle re-solves the same published linear system; it is an "
            "independent-implementation check, not a second physical model.",
        ],
    }
    out.write_text(json.dumps(report, indent=2) + "\n")

    lines = ["# Carbon-analysis cross-validation", "",
             f"Tolerance: `{TOLERANCE:.1e}`; worst error: `{worst:.3e}`; "
             f"overall: `{'PASS' if report['passed'] else 'FAIL'}`.", "",
             "| Case | Analytic | Re-solve | Conservation | Loss rule | Balance |",
             "|---|---:|---:|---:|---:|---:|"]
    for name, row in summary.items():
        lines.append(
            f"| {name} | {row['analytic_intensity']:.3e} | "
            f"{row['independent_resolve']:.3e} | {row['conservation_residual']:.3e} | "
            f"{row['loss_allocation_rule']:.3e} | {row['emission_balance']:.3e} |")
    lines += ["", "Scope: AC proportional carbon-emission-flow (Kang) matrix "
              "method, independently re-derived and re-solved.", ""]
    Path(args.markdown).write_text("\n".join(lines))

    if not report["passed"]:
        raise SystemExit("independent carbon-analysis oracle failed")


if __name__ == "__main__":
    main()
