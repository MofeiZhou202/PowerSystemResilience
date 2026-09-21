#!/usr/bin/env python3.8
"""Benchmark Phase 4 L2O solver configuration and artifact reuse on SCUC.

Default case:
  IEEE-118, T=4, wind+solar, StrictHiGHS/native branch-and-cut path.

The script first runs a source solve to populate SolverArtifactCache, then
compares the raw SCUC MIP baseline against the Phase 4 configuration policy
with compatible root-cut artifact reuse under fixed short time budgets.
"""

import argparse
import json
import statistics
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "build_mipsolvers"))

import mipsolvers  # noqa: E402


def parse_budgets(text):
    return [float(item) for item in text.split(",") if item.strip()]


def make_options(limit, gap, seed):
    opt = mipsolvers.engine.BCOptions()
    opt.time_limit_sec = limit
    opt.gap_tol = gap
    opt.num_threads = 1
    opt.deterministic_parallel = True
    opt.random_seed = seed
    return opt


def mean(rows, key):
    return statistics.mean(row[key] for row in rows)


def solve_baseline(case, options):
    start = time.perf_counter()
    result = mipsolvers.l2o.solve_scuc_mip(
        case,
        options=options,
        strict_highs=True,
        include_artifact_vectors=False,
    )
    wall = time.perf_counter() - start
    return {
        "objective": float(result["objective"]),
        "gap": float(result["mip_gap"]),
        "solver_sec": float(result.get("runtime_sec", wall)),
        "wall_sec": wall,
        "nodes": int(result.get("bc_stats", {}).get("nodes_explored", 0)),
    }


def solve_phase4(case, options, policy, cache, update_artifacts):
    start = time.perf_counter()
    result = mipsolvers.l2o.solve_scuc_mip_with_config_policy(
        case,
        options=options,
        policy_options=policy,
        artifact_cache=cache,
        update_artifacts=update_artifacts,
        strict_highs=True,
        include_artifact_vectors=False,
    )
    wall = time.perf_counter() - start
    reuse = result["solver_config"]["artifact_reuse"]
    return {
        "objective": float(result["objective"]),
        "gap": float(result["mip_gap"]),
        "solver_sec": float(result.get("runtime_sec", wall)),
        "wall_sec": wall,
        "nodes": int(result.get("bc_stats", {}).get("nodes_explored", 0)),
        "profile": result["solver_config"]["planned_decision"]["profile"],
        "root_cuts_used": bool(reuse["root_cuts_used"]),
        "root_cut_count": int(reuse["root_cut_count"]),
    }


def print_table(rows):
    print()
    print("| Budget | Baseline obj | Baseline gap | Phase4 obj | Phase4 gap | Solver s baseline/phase4 | Gap reduction |")
    print("|---:|---:|---:|---:|---:|---:|---:|")
    for row in rows:
        print(
            "| {budget:.2f}s | {baseline_objective:.6f} | {baseline_gap:.6f} | "
            "{phase4_objective:.6f} | {phase4_gap:.6f} | "
            "{baseline_solver_sec:.6f}/{phase4_solver_sec:.6f} | {gap_reduction:.6f} |".format(**row)
        )
    print()


def main():
    parser = argparse.ArgumentParser(description="Phase 4 L2O SCUC artifact-reuse benchmark")
    parser.add_argument("--budgets", default="0.25,0.5,1.0,2.0")
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--source-budget", type=float, default=2.0)
    parser.add_argument("--gap", type=float, default=1e-4)
    parser.add_argument("--seed", type=int, default=23)
    args = parser.parse_args()

    budgets = parse_budgets(args.budgets)
    case = mipsolvers.scuc.build_ieee118_case(T=4, with_wind=True, with_solar=True)
    policy = mipsolvers.l2o.SolverConfigPolicyOptions()
    cache = mipsolvers.l2o.SolverArtifactCache()

    source = solve_phase4(
        case,
        make_options(args.source_budget, args.gap, args.seed),
        policy,
        cache,
        update_artifacts=True,
    )
    print("ARTIFACT_SOURCE", json.dumps({"source": source, "cache": cache.to_dict()}, sort_keys=True))

    rows = []
    for budget in budgets:
        baseline_runs = []
        phase4_runs = []
        for _ in range(args.repeats):
            baseline_runs.append(solve_baseline(case, make_options(budget, args.gap, args.seed)))
            phase4_runs.append(solve_phase4(
                case,
                make_options(budget, args.gap, args.seed),
                policy,
                cache,
                update_artifacts=False,
            ))

        row = {
            "budget": budget,
            "baseline_objective": mean(baseline_runs, "objective"),
            "baseline_gap": mean(baseline_runs, "gap"),
            "baseline_solver_sec": mean(baseline_runs, "solver_sec"),
            "baseline_wall_sec": mean(baseline_runs, "wall_sec"),
            "baseline_nodes": mean(baseline_runs, "nodes"),
            "phase4_objective": mean(phase4_runs, "objective"),
            "phase4_gap": mean(phase4_runs, "gap"),
            "phase4_solver_sec": mean(phase4_runs, "solver_sec"),
            "phase4_wall_sec": mean(phase4_runs, "wall_sec"),
            "phase4_nodes": mean(phase4_runs, "nodes"),
            "phase4_profile": phase4_runs[-1]["profile"],
            "phase4_root_cuts_used": phase4_runs[-1]["root_cuts_used"],
            "phase4_root_cut_count": phase4_runs[-1]["root_cut_count"],
        }
        row["gap_reduction"] = row["baseline_gap"] - row["phase4_gap"]
        row["objective_delta"] = row["baseline_objective"] - row["phase4_objective"]
        rows.append(row)
        print("RESULT", json.dumps(row, sort_keys=True))

    print_table(rows)
    print("SUMMARY_JSON", json.dumps(rows, sort_keys=True))


if __name__ == "__main__":
    main()
