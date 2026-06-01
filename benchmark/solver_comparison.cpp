/// solver_comparison.cpp
///
/// Cross-solver performance comparison on SCUC MILP instances.
///
/// Solves the same synthetic SCUC MIP models with every available backend and
/// reports runtime, objective, gap and status side by side:
///
///   Native[B&C]       — this project's native branch-and-cut engine
///   Native[StrictHiGHS] — native B&C with the StrictHiGHS root pipeline
///   Gurobi            — Gurobi via the native C API adapter (if licensed)
///   HiGHS             — HiGHS via the embedded library adapter
///   SCIP              — SCIP via the MINLP (.pip) adapter
///
/// SCIP only accepts MINLP models, so each linear MIP is re-expressed as a
/// symbolic MINLP before being handed to the SCIP adapter.
///
/// Usage:
///   solver_comparison                 # 6-bus/4T + 39-bus/24T
///   solver_comparison --full          # + 39-bus/4T + 118-bus/24T
///   solver_comparison --time-limit 60 # per-solve wall-clock cap (native)

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <functional>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/SparseCore>

#include "mipsolvers/engine/bc/api.hpp"
#include "mipsolvers/engine/bc/options.hpp"
#include "mipsolvers/engine/bc/stats.hpp"
#include "mipsolvers/engine/problem_types.hpp"
#include "mipsolvers/engine/solver/external/adapters.hpp"
#include "mipsolvers/engine/solver/native/native_adapters.hpp"
#include "mipsolvers/scuc/case_builder.hpp"

using namespace mipsolvers;
using namespace mipsolvers::scuc;

namespace {

struct TestCase {
    std::string name;
    SCUCInput   inp;
};

struct ResultRow {
    std::string case_name;
    std::string solver;
    bool        available{true};
    bool        success{false};
    double      objective{0.0};
    double      gap{0.0};
    double      runtime_ms{0.0};
    std::string status;
};

// ── Symbolic helpers ──────────────────────────────────────────────────────

using SymPtr = std::shared_ptr<engine::SymExpr>;

// Build a balanced sum tree from terms to keep expression depth ~log(n),
// avoiding deep recursion when the polynomial expander walks the tree.
SymPtr balanced_sum(const std::vector<SymPtr>& terms, std::size_t lo, std::size_t hi) {
    if (lo >= hi) {
        return engine::SymExpr::constant(0.0);
    }
    if (hi - lo == 1) {
        return terms[lo];
    }
    const std::size_t mid = lo + (hi - lo) / 2;
    return engine::SymExpr::add(balanced_sum(terms, lo, mid),
                                balanced_sum(terms, mid, hi));
}

// Re-express a linear MIP as a symbolic MINLP suitable for the SCIP adapter.
engine::MINLPModel mip_to_minlp(const engine::MIPModel& mip) {
    const engine::LPModel& lp = mip.linear_part;
    engine::MINLPModel out;
    engine::NLPModel& nlp = out.nonlinear_part;

    nlp.sense = lp.sense;
    nlp.vars  = lp.vars;
    for (std::size_t j = 0; j < nlp.vars.size(); ++j) {
        if (nlp.vars[j].name.empty()) {
            nlp.vars[j].name = "x" + std::to_string(j);
        }
    }
    nlp.x0 = Eigen::VectorXd::Zero(static_cast<int>(nlp.vars.size()));

    // Objective: c' x
    {
        std::vector<SymPtr> terms;
        for (int j = 0; j < lp.c.size(); ++j) {
            if (lp.c[j] != 0.0) {
                terms.push_back(engine::SymExpr::mul(
                    engine::SymExpr::constant(lp.c[j]),
                    engine::SymExpr::variable(j)));
            }
        }
        nlp.symbolic_objective = balanced_sum(terms, 0, terms.size());
    }

    // Inequalities: A x <= b   (and  A x >= row_lhs  when finite)
    Eigen::SparseMatrix<double, Eigen::RowMajor> A_rm = lp.A;
    for (int i = 0; i < lp.A.rows(); ++i) {
        std::vector<SymPtr> terms;
        for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(A_rm, i); it; ++it) {
            terms.push_back(engine::SymExpr::mul(
                engine::SymExpr::constant(it.value()),
                engine::SymExpr::variable(static_cast<int>(it.col()))));
        }
        SymPtr expr = balanced_sum(terms, 0, terms.size());

        engine::SymbolicConstraint c_ub;
        c_ub.expr  = expr;
        c_ub.sense = engine::SymbolicSense::LessEqual;
        c_ub.rhs   = lp.b[i];
        out.nonlinear_part.symbolic_constraints.push_back(c_ub);

        const double lhs = engine::lp_row_lhs_or_neg_inf(lp, i);
        if (std::isfinite(lhs)) {
            engine::SymbolicConstraint c_lb;
            c_lb.expr  = expr;
            c_lb.sense = engine::SymbolicSense::GreaterEqual;
            c_lb.rhs   = lhs;
            out.nonlinear_part.symbolic_constraints.push_back(c_lb);
        }
    }

    // Equalities: Aeq x = beq
    Eigen::SparseMatrix<double, Eigen::RowMajor> Aeq_rm = lp.Aeq;
    for (int i = 0; i < lp.Aeq.rows(); ++i) {
        std::vector<SymPtr> terms;
        for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(Aeq_rm, i); it; ++it) {
            terms.push_back(engine::SymExpr::mul(
                engine::SymExpr::constant(it.value()),
                engine::SymExpr::variable(static_cast<int>(it.col()))));
        }
        engine::SymbolicConstraint c_eq;
        c_eq.expr  = balanced_sum(terms, 0, terms.size());
        c_eq.sense = engine::SymbolicSense::Equal;
        c_eq.rhs   = lp.beq[i];
        out.nonlinear_part.symbolic_constraints.push_back(c_eq);
    }

    out.integer_idx = mip.integer_idx;
    out.binary_idx  = mip.binary_idx;
    return out;
}

double rel_gap(double obj, double bound) {
    if (!std::isfinite(obj) || !std::isfinite(bound)) return 0.0;
    return std::abs(obj - bound) / (1.0 + std::abs(obj));
}

// ── Per-solver runners ─────────────────────────────────────────────────────

ResultRow run_native(const TestCase& tc, bool strict_highs, double time_limit_sec) {
    ResultRow r;
    r.case_name = tc.name;
    r.solver    = strict_highs ? "Native[StrictHiGHS]" : "Native[B&C]";

    engine::MIPModel mip = build_scuc_mip(tc.inp);

    engine::BCOptions opt;
    opt.time_limit_sec = time_limit_sec;
    opt.gap_tol        = 1e-3;
    opt.verbose        = false;
    // The native B&C orchestration drives its LP relaxations through the
    // vendored HiGHS LP kernel in both configurations; StrictHiGHS adds the
    // strict root pipeline (IPM seeding + dynamic cut separation) on top.
    opt.use_vendored_highs_lp_kernel = true;
    if (strict_highs) {
        opt.auto_highs_root_pipeline = true;
        opt = engine::make_strict_highs_problem_options(mip, opt);
    }

    const auto t0 = std::chrono::steady_clock::now();
    engine::BCResult res = engine::solve_milp_bc(mip, opt);
    const auto t1 = std::chrono::steady_clock::now();

    r.success    = res.stats.success;
    r.objective  = res.stats.objective;
    r.gap        = rel_gap(res.stats.objective, res.bc_stats.best_bound);
    r.runtime_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    r.status     = res.bc_stats.status;
    return r;
}

ResultRow run_external(const TestCase& tc, const std::string& label,
                       const std::function<engine::SolveResult(const engine::MIPModel&)>& solve) {
    ResultRow r;
    r.case_name = tc.name;
    r.solver    = label;

    engine::MIPModel mip = build_scuc_mip(tc.inp);
    const auto t0 = std::chrono::steady_clock::now();
    engine::SolveResult res = solve(mip);
    const auto t1 = std::chrono::steady_clock::now();

    r.success    = res.stats.success;
    r.objective  = res.stats.objective;
    r.gap        = res.stats.mip_gap;
    r.runtime_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    r.status     = res.stats.status;
    return r;
}

void print_table(const std::vector<ResultRow>& rows) {
    std::printf("%-20s %-22s %10s %14s %9s %-14s\n",
                "Case", "Solver", "Time[ms]", "Objective", "Gap[%]", "Status");
    std::printf("%s\n", std::string(94, '-').c_str());
    std::string current;
    for (const auto& r : rows) {
        if (r.case_name != current) {
            if (!current.empty()) std::printf("\n");
            current = r.case_name;
        }
        if (!r.available) {
            std::printf("%-20s %-22s %10s %14s %9s %-14s\n",
                        r.case_name.c_str(), r.solver.c_str(),
                        "-", "-", "-", "unavailable");
            continue;
        }
        std::printf("%-20s %-22s %10.1f %14.4f %9.4f %-14s\n",
                    r.case_name.c_str(), r.solver.c_str(),
                    r.runtime_ms, r.objective, 100.0 * r.gap,
                    r.success ? r.status.c_str() : ("FAIL:" + r.status).c_str());
    }
}

}  // namespace

int main(int argc, char** argv) {
    bool   full_mode      = false;
    double time_limit_sec = 120.0;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--full") == 0) {
            full_mode = true;
        } else if (std::strcmp(argv[i], "--time-limit") == 0 && i + 1 < argc) {
            time_limit_sec = std::atof(argv[++i]);
        }
    }

    std::printf("=== MIPSolvers cross-solver comparison "
                "(Native / Gurobi / HiGHS / SCIP) ===\n");
    std::printf("Mode: %s  Native time limit: %.0fs\n\n",
                full_mode ? "full" : "quick", time_limit_sec);

    // ── Build test cases ──────────────────────────────────────────────────
    std::vector<TestCase> cases;
    {
        SCUCInput inp = build_6bus_case(/*T=*/4);
        inp.config.solve_sced = false;
        inp.config.solve_lmp  = false;
        cases.push_back({"UC_6bus_3G_4T", std::move(inp)});
    }
    {
        SCUCInput inp = build_ieee39_case(/*T=*/24);
        inp.config.solve_sced = false;
        inp.config.solve_lmp  = false;
        cases.push_back({"UC_39bus_10G_24T", std::move(inp)});
    }
    if (full_mode) {
        {
            SCUCInput inp = build_ieee39_case(/*T=*/4);
            inp.config.solve_sced = false;
            inp.config.solve_lmp  = false;
            cases.push_back({"UC_39bus_10G_4T", std::move(inp)});
        }
        {
            SCUCInput inp = build_ieee118_case(/*T=*/24);
            inp.config.solve_sced = false;
            inp.config.solve_lmp  = false;
            cases.push_back({"UC_118bus_54G_24T", std::move(inp)});
        }
    }

    // Probe solver availability once.
    engine::GurobiAdapter gurobi;
    engine::HighsAdapter  highs;
    engine::ScipAdapter   scip;
    const bool have_gurobi = gurobi.available();
    const bool have_highs  = highs.available();
    const bool have_scip   = scip.available();

    std::printf("Available backends: Native=yes  Gurobi=%s  HiGHS=%s  SCIP=%s\n\n",
                have_gurobi ? "yes" : "no",
                have_highs  ? "yes" : "no",
                have_scip   ? "yes" : "no");

    std::vector<ResultRow> rows;
    for (const auto& tc : cases) {
        engine::MIPModel probe = build_scuc_mip(tc.inp);
        std::printf("--- %s: %zu vars, %ld ineq, %ld eq ---\n",
                    tc.name.c_str(), probe.linear_part.vars.size(),
                    static_cast<long>(probe.linear_part.b.size()),
                    static_cast<long>(probe.linear_part.beq.size()));
        std::fflush(stdout);

        rows.push_back(run_native(tc, /*strict_highs=*/false, time_limit_sec));
        std::printf("  Native[B&C]          done (%.1f ms)\n", rows.back().runtime_ms);
        std::fflush(stdout);

        rows.push_back(run_native(tc, /*strict_highs=*/true, time_limit_sec));
        std::printf("  Native[StrictHiGHS]  done (%.1f ms)\n", rows.back().runtime_ms);
        std::fflush(stdout);

        if (have_gurobi) {
            rows.push_back(run_external(tc, "Gurobi",
                [&](const engine::MIPModel& m) { return gurobi.solve_milp(m); }));
            std::printf("  Gurobi               done (%.1f ms)\n", rows.back().runtime_ms);
        } else {
            rows.push_back({tc.name, "Gurobi", false, false, 0, 0, 0, ""});
        }
        std::fflush(stdout);

        if (have_highs) {
            rows.push_back(run_external(tc, "HiGHS",
                [&](const engine::MIPModel& m) { return highs.solve_milp(m); }));
            std::printf("  HiGHS                done (%.1f ms)\n", rows.back().runtime_ms);
        } else {
            rows.push_back({tc.name, "HiGHS", false, false, 0, 0, 0, ""});
        }
        std::fflush(stdout);

        if (have_scip) {
            ResultRow r;
            r.case_name = tc.name;
            r.solver    = "SCIP";
            engine::MIPModel m = build_scuc_mip(tc.inp);
            engine::MINLPModel minlp = mip_to_minlp(m);
            const auto t0 = std::chrono::steady_clock::now();
            engine::SolveResult res = scip.solve_minlp(minlp);
            const auto t1 = std::chrono::steady_clock::now();
            r.success    = res.stats.success;
            r.objective  = res.stats.objective;
            r.gap        = res.stats.mip_gap;
            r.runtime_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
            r.status     = res.stats.status;
            rows.push_back(r);
            std::printf("  SCIP                 done (%.1f ms)\n", r.runtime_ms);
        } else {
            rows.push_back({tc.name, "SCIP", false, false, 0, 0, 0, ""});
        }
        std::fflush(stdout);
        std::printf("\n");
    }

    std::printf("\n=== Comparison summary ===\n");
    print_table(rows);
    return 0;
}
