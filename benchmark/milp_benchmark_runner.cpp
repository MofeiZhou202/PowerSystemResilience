/// milp_benchmark_runner.cpp
///
/// Measures the effect of StrictHiGHS solver improvements A, B, and H on
/// real SCUC MIP instances built from the synthetic test-case library.
///
/// Improvement key (from the performance plan):
///   A  — Incumbent warm-start injection via highs.setSolution()
///   B  — Objective cutoff propagation (paired with A; active when A injects
///          a feasible solution, sets objective_bound automatically)
///   H  — mip_max_stall_nodes: early termination when the best bound stops
///          improving, returning the best incumbent found so far.
///   I  — mip_lp_solver="ipm" + mip_root_crossover: pure IPM at the root LP
///          (and all LP relaxations without a valid basis).  "off" skips the
///          degenerate crossover step; fastest for large, highly degenerate
///          instances (e.g., 118-bus UC) where crossover ≈ simplex in cost.
///   J  — Bounded HiGHS IPM point + native crash-basis recovery, converted to
///          HighsBasis and injected before StrictHiGHS calls highs.run().
///
/// For each test case the runner executes six solver configurations:
///   Baseline[S]  : simplex root (old behaviour; no regression)
///   I[IPM+xov]   : IPM + crossover at root (reference; shows crossover cost)
///   I[IPM-xov]   : pure IPM, no crossover (hypothesis: fast for 118-bus)
///   I+A/B[-xov]  : pure IPM, no crossover + warm-start incumbent
///   NativeSeed[S]: HiGHS-IPM point + native crash-basis seed + simplex root
///   Seed+A/B[S]  : NativeSeed[S] + warm-start incumbent
///
/// Usage:
///   milp_benchmark_runner --smoke          # deterministic 6-bus CTest case
///   milp_benchmark_runner                  # quick (6G/4T + 10G/24T)
///   milp_benchmark_runner --full           # + 10G/4T + 39-bus/24T + 118-bus
///   milp_benchmark_runner --million-hotpath # 1M-column B&B memory/scan kernel
///   milp_benchmark_runner --json out.json  # write JSON result
///   milp_benchmark_runner --full --json out.json

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <limits>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include <Eigen/Core>

#ifdef MIPSOLVERS_HAVE_HIGHS_LIB
#include "Highs.h"
#endif

#include "mipsolvers/engine/bc/api.hpp"
#include "mipsolvers/engine/bc/options.hpp"
#include "mipsolvers/engine/bc/stats.hpp"
#include "mipsolvers/engine/detail/bc_types.hpp"
#include "mipsolvers/engine/solver/external/adapters.hpp"
#include "mipsolvers/engine/solver/native/native_adapters.hpp"
#include "mipsolvers/scuc/case_builder.hpp"

using namespace mipsolvers;
using namespace mipsolvers::scuc;
using json = nlohmann::json;

namespace {

int run_million_hotpath_benchmark() {
    constexpr int n = 1'000'000;
    constexpr int n_branchable = 20'000;
    constexpr int scan_repeats = 64;

    engine::detail::Node root;
    root.lb = Eigen::VectorXd::Zero(n);
    root.ub = Eigen::VectorXd::Constant(n, 1000.0);
    root.x_relax = Eigen::VectorXd::Constant(n, 0.25);
    root.bound = 1.0;

    std::vector<char> branchable(static_cast<std::size_t>(n), 0);
    std::vector<int> branchable_indices;
    branchable_indices.reserve(n_branchable);
    for (int k = 0; k < n_branchable; ++k) {
        const int j = static_cast<int>((static_cast<std::int64_t>(k) * n) /
                                       n_branchable);
        branchable[static_cast<std::size_t>(j)] = 1;
        branchable_indices.push_back(j);
    }
    engine::detail::CompactPseudoCostTable compact_pc(n, branchable_indices);

    volatile double checksum = 0.0;
    const auto old_branch_start = std::chrono::steady_clock::now();
    {
        auto down = root.branch_child();
        auto up = root.branch_child();
        down.ub[branchable_indices.front()] = 0.0;
        up.lb[branchable_indices.front()] = 1.0;
        checksum += down.ub[0] + up.lb[0];
    }
    const auto old_branch_end = std::chrono::steady_clock::now();

    engine::detail::Node movable_parent = root;
    const auto new_branch_start = std::chrono::steady_clock::now();
    {
        auto down = movable_parent.branch_child();
        auto up = std::move(movable_parent);
        up.x_relax.resize(0);
        up.x_seed.resize(0);
        down.ub[branchable_indices.front()] = 0.0;
        up.lb[branchable_indices.front()] = 1.0;
        checksum += down.ub[0] + up.lb[0];
    }
    const auto new_branch_end = std::chrono::steady_clock::now();

    const auto full_scan_start = std::chrono::steady_clock::now();
    for (int r = 0; r < scan_repeats; ++r) {
        for (int j = 0; j < n; ++j) {
            if (branchable[static_cast<std::size_t>(j)] != 0) {
                checksum += root.x_relax[j];
            }
        }
    }
    const auto full_scan_end = std::chrono::steady_clock::now();

    const auto compact_scan_start = std::chrono::steady_clock::now();
    for (int r = 0; r < scan_repeats; ++r) {
        for (int j : branchable_indices) checksum += root.x_relax[j];
    }
    const auto compact_scan_end = std::chrono::steady_clock::now();

    const auto ms = [](auto begin, auto end) {
        return std::chrono::duration<double, std::milli>(end - begin).count();
    };
    const double old_branch_ms = ms(old_branch_start, old_branch_end);
    const double new_branch_ms = ms(new_branch_start, new_branch_end);
    const double full_scan_ms = ms(full_scan_start, full_scan_end);
    const double compact_scan_ms = ms(compact_scan_start, compact_scan_end);

    std::printf("=== Million-column B&B hot path ===\n");
    std::printf("columns=%d branchable=%d repeats=%d\n", n, n_branchable,
                scan_repeats);
    std::printf("branch old(two full children): %.3f ms, copied %.1f MiB\n",
                old_branch_ms, 4.0 * n * sizeof(double) / 1048576.0);
    std::printf("branch new(one copy + move):   %.3f ms, copied %.1f MiB, speedup %.2fx\n",
                new_branch_ms, 2.0 * n * sizeof(double) / 1048576.0,
                old_branch_ms / std::max(1e-9, new_branch_ms));
    std::printf("candidate full scan:           %.3f ms\n", full_scan_ms);
    std::printf("candidate compact scan:        %.3f ms, speedup %.2fx\n",
                compact_scan_ms,
                full_scan_ms / std::max(1e-9, compact_scan_ms));
    std::printf("pseudocost dense/compact:      %.1f MiB / %.1f MiB (%.2fx smaller)\n",
                n * sizeof(engine::detail::PseudoCost) / 1048576.0,
                compact_pc.storage_bytes() / 1048576.0,
                static_cast<double>(n * sizeof(engine::detail::PseudoCost)) /
                    std::max<std::size_t>(1, compact_pc.storage_bytes()));
    std::printf("checksum=%.1f\n", checksum);
    return 0;
}

void set_env_var(const char* name, const char* value) {
#if defined(_WIN32)
    _putenv_s(name, value ? value : "");
#else
    if (value) {
        ::setenv(name, value, 1);
    } else {
        ::unsetenv(name);
    }
#endif
}

void unset_env_var(const char* name) {
#if defined(_WIN32)
    _putenv_s(name, "");
#else
    ::unsetenv(name);
#endif
}

}  // namespace

// ─────────────────────────────────────────────────────────────────────────────
// Solver configuration variants
// ─────────────────────────────────────────────────────────────────────────────

struct SolverConfig {
    std::string label;
    bool        warm_start;       ///< inject initial_solution from a cold run
    int         stall_nodes;      ///< highs_mip_max_stall_nodes (0 = disabled)
    std::string mip_lp_solver;    ///< "choose"=simplex; "ipm"=IPM at root
    std::string crossover;        ///< "on"/"off"/"choose"/"" (""=use BCOptions default)
    bool        seed_native_basis;///< highs_strict_seed_native_ipm_basis
    int         root_iter_limit;  ///< first root simplex iteration cap (0 = disabled)
    int         sepa_rounds{0};   ///< HiGHS hacdcpf_max_root_sepa_rounds override:
                                  ///<   0  = use engine default (50 when warm-start, unlimited otherwise)
                                  ///<  >0  = use exactly this many rounds
                                  ///<  -1  = force 0 rounds (disable all HiGHS cuts)
    engine::CutType cut_type{engine::CutType::All}; ///< cut family selector (default: all families)
};

/// Seven solver configurations:
///   Baseline[S]   — simplex root LP (mip_lp_solver="choose"; all HiGHS cuts; 50k iter cap via inline cold run)
///   I[IPM+xov]    — IPM + crossover at root (reference for crossover cost)
///   I+A/B[-xov]   — pure IPM, no crossover + warm-start incumbent injection
///   NativeSeed[S] — HiGHS-IPM point + native crash-basis recovery seeds the root basis
///   Seed+A/B[S]   — NativeSeed[S] plus warm-start incumbent injection
///   NoCuts[S]     — simplex root, 50k iter cap, hacdcpf_max_root_sepa_rounds=0 (no HiGHS cuts);
///                   fair ablation vs Baseline[S]: same iteration budget, only cuts differ
///   FCOnly[S]     — simplex root, 50k iter cap, full HiGHS cut suite (same as Baseline[S] path);
///                   NOTE: in StrictHiGHS, CutType::FlowCover only affects native B&C (not HiGHS separators)
static const SolverConfig kConfigs[] = {
    //                  label             wm  stall  lp        xov    seed   cap   sepa  cuts
    { "Baseline[S]",    false, 0, "choose", "",    false, 0,     0, engine::CutType::All },
    { "I[IPM+xov]",     false, 0, "ipm",    "on",  false, 0,     0, engine::CutType::All },
    { "I+A/B[-xov]",    true,  0, "ipm",    "off", false, 0,     0, engine::CutType::All },
    { "NativeSeed[S]",  false, 0, "choose", "",    true,  0,     0, engine::CutType::All },
    { "Seed+A/B[S]",    true,  0, "choose", "",    true,  0,     0, engine::CutType::All },
    { "NoCuts[S]",      false, 0, "choose", "",    false, 50000, -1, engine::CutType::None },
    { "FCOnly[S]",      false, 0, "choose", "",    false, 50000,  0, engine::CutType::FlowCover },
};

// ─────────────────────────────────────────────────────────────────────────────
// Test case descriptor
// ─────────────────────────────────────────────────────────────────────────────

struct TestCase {
    std::string name;
    SCUCInput   inp;
};

// ─────────────────────────────────────────────────────────────────────────────
// Result record
// ─────────────────────────────────────────────────────────────────────────────

struct RunRecord {
    std::string case_name;
    std::string solver_label;
    int  n_vars{0};
    int  n_ineq{0};
    int  n_eq{0};
    bool success{false};
    int  nodes{0};
    int  lp_solves{0};
    double objective{0.0};
    double best_bound{0.0};
    double gap{0.0};
    double runtime_ms{0.0};
    std::string status;
};

struct SolutionAudit {
    double objective{0.0};
    double max_row_violation{0.0};
    double max_bound_violation{0.0};
    double max_integrality_violation{0.0};
};

static SolutionAudit audit_solution(const engine::MIPModel& mip,
                                    const Eigen::VectorXd& x)
{
    SolutionAudit audit;
    const engine::LPModel& lp = mip.linear_part;
    if (x.size() != static_cast<int>(lp.vars.size())) {
        audit.max_row_violation = std::numeric_limits<double>::infinity();
        audit.max_bound_violation = std::numeric_limits<double>::infinity();
        audit.max_integrality_violation = std::numeric_limits<double>::infinity();
        return audit;
    }

    audit.objective = lp.c.dot(x);
    const Eigen::VectorXd ax = lp.A * x;
    for (int i = 0; i < ax.size(); ++i) {
        audit.max_row_violation = std::max(audit.max_row_violation,
                                           std::max(0.0, ax[i] - lp.b[i]));
        const double lhs = engine::lp_row_lhs_or_neg_inf(lp, i);
        if (std::isfinite(lhs)) {
            audit.max_row_violation = std::max(audit.max_row_violation,
                                               std::max(0.0, lhs - ax[i]));
        }
    }
    const Eigen::VectorXd aeqx = lp.Aeq * x;
    for (int i = 0; i < aeqx.size(); ++i) {
        audit.max_row_violation = std::max(audit.max_row_violation,
                                           std::abs(aeqx[i] - lp.beq[i]));
    }
    for (int j = 0; j < x.size(); ++j) {
        audit.max_bound_violation = std::max(
            audit.max_bound_violation,
            std::max(std::max(0.0, lp.vars[static_cast<std::size_t>(j)].lb - x[j]),
                     std::max(0.0, x[j] - lp.vars[static_cast<std::size_t>(j)].ub)));
    }
    for (int idx : mip.integer_idx) {
        if (idx >= 0 && idx < x.size()) {
            audit.max_integrality_violation = std::max(
                audit.max_integrality_violation, std::abs(x[idx] - std::round(x[idx])));
        }
    }
    for (int idx : mip.binary_idx) {
        if (idx >= 0 && idx < x.size()) {
            audit.max_integrality_violation = std::max(
                audit.max_integrality_violation, std::abs(x[idx] - std::round(x[idx])));
        }
    }
    return audit;
}

// ─────────────────────────────────────────────────────────────────────────────
// Run one configuration
// ─────────────────────────────────────────────────────────────────────────────

/// When true, ignore root_iter_limit in all SolverConfigs and the inline
/// cold run.  Set by --no-root-cap.  Lets HiGHS finish all cut rounds
/// before branching, isolating "cut quality" from "node budget".
static bool g_no_root_cap = false;

/// When true, build each test case without DC power flow constraints by
/// inflating all branch/section ratings to 2e5 MW (> the 1e5 MW threshold
/// in build_formulation that skips flow constraint rows).  Used as a quick
/// diagnostic: running --no-flow --no-root-cap shows the LP relaxation bound
/// and gap achievable without transmission constraints, which quantifies how
/// much of the current 5–7% gap comes from the flow constraint contribution
/// vs the integrality gap of the commitment variables.
static bool g_no_flow = false;

static RunRecord run_config(const TestCase& tc,
                            const SolverConfig& cfg,
                            const Eigen::VectorXd* warm_x = nullptr,
                            double time_limit_sec = 120.0)
{
    RunRecord rec;
    rec.case_name    = tc.name;
    rec.solver_label = cfg.label;

    // Build fresh MIP model for each run (cheap — just Eigen algebra)
    engine::MIPModel mip = build_scuc_mip(tc.inp);

    rec.n_vars = static_cast<int>(mip.linear_part.vars.size());
    rec.n_ineq = static_cast<int>(mip.linear_part.b.size());
    rec.n_eq   = static_cast<int>(mip.linear_part.beq.size());

    // Inject warm start solution if requested and available
    if (cfg.warm_start && warm_x
            && static_cast<int>(warm_x->size()) == rec.n_vars) {
        mip.initial_solution = *warm_x;
    }

    engine::BCOptions opt;
    opt.use_vendored_highs_lp_kernel = true;
    opt.time_limit_sec               = time_limit_sec;
    opt.gap_tol                      = 1e-3;
    opt.verbose                      = false;
    if (!cfg.mip_lp_solver.empty())
        opt.highs_mip_lp_solver = cfg.mip_lp_solver;
    if (!cfg.crossover.empty())
        opt.highs_mip_root_crossover = cfg.crossover;
    if (cfg.stall_nodes > 0)
        opt.highs_mip_max_stall_nodes = cfg.stall_nodes;
    if (cfg.seed_native_basis)
        opt.highs_strict_seed_native_ipm_basis = true;
    if (cfg.root_iter_limit > 0 && !g_no_root_cap)
        opt.highs_mip_root_simplex_iteration_limit = cfg.root_iter_limit;
    if (cfg.sepa_rounds != 0)
        opt.highs_max_root_sepa_rounds = cfg.sepa_rounds;
    opt.cuts = cfg.cut_type;

    const auto t0 = std::chrono::steady_clock::now();
    engine::BCResult res = engine::solve_milp_bc(mip, opt);
    const auto t1 = std::chrono::steady_clock::now();

    rec.success    = res.stats.success;
    rec.nodes      = res.bc_stats.nodes_explored;
    rec.lp_solves  = res.bc_stats.lp_solves;
    rec.objective  = res.stats.objective;
    rec.best_bound = res.bc_stats.best_bound;
    rec.runtime_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    rec.status     = res.bc_stats.status;

    // Relative gap: |obj - bound| / (1 + |obj|)
    if (rec.success && std::isfinite(rec.objective) && std::isfinite(rec.best_bound)) {
        rec.gap = std::abs(rec.objective - rec.best_bound)
                  / (1.0 + std::abs(rec.objective));
    }

    return rec;
}

static RunRecord run_bc_start_reference(const TestCase& tc,
                                        const char* label,
                                        const Eigen::VectorXd* warm_x,
                                        double time_limit_sec,
                                        bool strict_highs,
                                        bool force_strict_root_ipm = false)
{
    RunRecord rec;
    rec.case_name = tc.name;
    rec.solver_label = label;

    engine::MIPModel mip = build_scuc_mip(tc.inp);
    rec.n_vars = static_cast<int>(mip.linear_part.vars.size());
    rec.n_ineq = static_cast<int>(mip.linear_part.b.size());
    rec.n_eq = static_cast<int>(mip.linear_part.beq.size());
    if (warm_x != nullptr && static_cast<int>(warm_x->size()) == rec.n_vars) {
        mip.initial_solution = *warm_x;
    }

    engine::BCOptions opt;
    opt.use_vendored_highs_lp_kernel = strict_highs;
    opt.auto_highs_root_pipeline = strict_highs;
    opt.time_limit_sec = time_limit_sec;
    opt.gap_tol = 1e-3;
    opt.verbose = false;
    if (strict_highs && force_strict_root_ipm) {
        opt.highs_mip_lp_solver = "ipm";
        opt.highs_mip_root_crossover = "on";
    }
    if (strict_highs) {
        opt = engine::make_strict_highs_problem_options(mip, opt);
    }

    const auto t0 = std::chrono::steady_clock::now();
    engine::BCResult res = engine::solve_milp_bc(mip, opt);
    const auto t1 = std::chrono::steady_clock::now();

    rec.success = res.stats.success;
    rec.nodes = res.bc_stats.nodes_explored;
    rec.lp_solves = res.bc_stats.lp_solves;
    rec.objective = res.stats.objective;
    rec.best_bound = res.bc_stats.best_bound;
    rec.runtime_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    rec.status = res.bc_stats.status;
    if (rec.success && std::isfinite(rec.objective) && std::isfinite(rec.best_bound)) {
        rec.gap = std::abs(rec.objective - rec.best_bound) / (1.0 + std::abs(rec.objective));
    }
    return rec;
}

#ifdef MIPSOLVERS_HAVE_HIGHS_LIB
static const char* highs_status_label(HighsModelStatus status)
{
    switch (status) {
        case HighsModelStatus::kNotset: return "Notset";
        case HighsModelStatus::kLoadError: return "LoadError";
        case HighsModelStatus::kModelError: return "ModelError";
        case HighsModelStatus::kPresolveError: return "PresolveError";
        case HighsModelStatus::kSolveError: return "SolveError";
        case HighsModelStatus::kPostsolveError: return "PostsolveError";
        case HighsModelStatus::kModelEmpty: return "ModelEmpty";
        case HighsModelStatus::kOptimal: return "Optimal";
        case HighsModelStatus::kInfeasible: return "Infeasible";
        case HighsModelStatus::kUnboundedOrInfeasible: return "UnboundedOrInfeasible";
        case HighsModelStatus::kUnbounded: return "Unbounded";
        case HighsModelStatus::kObjectiveBound: return "ObjectiveBound";
        case HighsModelStatus::kObjectiveTarget: return "ObjectiveTarget";
        case HighsModelStatus::kTimeLimit: return "TimeLimit";
        case HighsModelStatus::kIterationLimit: return "IterationLimit";
        case HighsModelStatus::kUnknown: return "Unknown";
        case HighsModelStatus::kSolutionLimit: return "SolutionLimit";
        case HighsModelStatus::kInterrupt: return "Interrupt";
        case HighsModelStatus::kMemoryLimit: return "MemoryLimit";
        case HighsModelStatus::kHighsInterrupt: return "HighsInterrupt";
    }
    return "Unknown";
}

static RunRecord run_direct_highs_reference(const TestCase& tc,
                                            double time_limit_sec,
                                            const char* mip_lp_solver,
                                            const char* crossover,
                                            const char* label,
                                            const Eigen::VectorXd* mip_start = nullptr)
{
    RunRecord rec;
    rec.case_name    = tc.name;
    rec.solver_label = label;

    engine::MIPModel mip = build_scuc_mip(tc.inp);
    engine::LPModel lp = mip.linear_part;
    for (int idx : mip.integer_idx) {
        lp.vars[static_cast<std::size_t>(idx)].type = engine::VarType::Integer;
    }
    for (int idx : mip.binary_idx) {
        auto& var = lp.vars[static_cast<std::size_t>(idx)];
        var.type = engine::VarType::Binary;
        var.lb = std::max(0.0, var.lb);
        var.ub = std::min(1.0, var.ub);
    }

    const int ncols = static_cast<int>(lp.vars.size());
    const int m_ineq = static_cast<int>(lp.A.rows());
    const int m_eq = static_cast<int>(lp.Aeq.rows());
    const int nrows = m_ineq + m_eq;
    rec.n_vars = ncols;
    rec.n_ineq = m_ineq;
    rec.n_eq = m_eq;

    std::vector<double> col_cost(static_cast<std::size_t>(ncols), 0.0);
    std::vector<double> col_lower(static_cast<std::size_t>(ncols), -kHighsInf);
    std::vector<double> col_upper(static_cast<std::size_t>(ncols), kHighsInf);
    std::vector<HighsInt> integrality(static_cast<std::size_t>(ncols),
                                      static_cast<HighsInt>(HighsVarType::kContinuous));
    for (int j = 0; j < ncols; ++j) {
        const auto& var = lp.vars[static_cast<std::size_t>(j)];
        col_cost[static_cast<std::size_t>(j)] = lp.c[j];
        col_lower[static_cast<std::size_t>(j)] = std::isfinite(var.lb) ? var.lb : -kHighsInf;
        col_upper[static_cast<std::size_t>(j)] = std::isfinite(var.ub) ? var.ub : kHighsInf;
        if (var.type != engine::VarType::Continuous) {
            integrality[static_cast<std::size_t>(j)] =
                static_cast<HighsInt>(HighsVarType::kInteger);
        }
    }

    std::vector<double> row_lower(static_cast<std::size_t>(nrows), -kHighsInf);
    std::vector<double> row_upper(static_cast<std::size_t>(nrows), kHighsInf);
    for (int r = 0; r < m_ineq; ++r) {
        const double lhs = engine::lp_row_lhs_or_neg_inf(lp, r);
        row_lower[static_cast<std::size_t>(r)] = std::isfinite(lhs) ? lhs : -kHighsInf;
        row_upper[static_cast<std::size_t>(r)] = std::isfinite(lp.b[r]) ? lp.b[r] : kHighsInf;
    }
    for (int r = 0; r < m_eq; ++r) {
        const int rr = m_ineq + r;
        const double rhs = lp.beq[r];
        row_lower[static_cast<std::size_t>(rr)] = std::isfinite(rhs) ? rhs : kHighsInf;
        row_upper[static_cast<std::size_t>(rr)] = row_lower[static_cast<std::size_t>(rr)];
    }

    std::vector<HighsInt> start(static_cast<std::size_t>(ncols + 1), 0);
    std::vector<HighsInt> index;
    std::vector<double> value;
    index.reserve(static_cast<std::size_t>(lp.A.nonZeros() + lp.Aeq.nonZeros()));
    value.reserve(index.capacity());
    for (int j = 0; j < ncols; ++j) {
        start[static_cast<std::size_t>(j)] = static_cast<HighsInt>(index.size());
        for (Eigen::SparseMatrix<double>::InnerIterator it(lp.A, j); it; ++it) {
            if (it.value() == 0.0) continue;
            index.push_back(static_cast<HighsInt>(it.row()));
            value.push_back(it.value());
        }
        for (Eigen::SparseMatrix<double>::InnerIterator it(lp.Aeq, j); it; ++it) {
            if (it.value() == 0.0) continue;
            index.push_back(static_cast<HighsInt>(m_ineq + it.row()));
            value.push_back(it.value());
        }
    }
    start[static_cast<std::size_t>(ncols)] = static_cast<HighsInt>(index.size());

    const auto t0 = std::chrono::steady_clock::now();
    Highs::resetGlobalScheduler(true);
    Highs highs;
    highs.setOptionValue("output_flag", false);
    highs.setOptionValue("log_to_console", false);
    highs.setOptionValue("threads", static_cast<HighsInt>(1));
    highs.setOptionValue("mip_rel_gap", 1e-4);
    highs.setOptionValue("time_limit", std::max(0.001, time_limit_sec));
    if (mip_lp_solver && mip_lp_solver[0] != '\0') {
        highs.setOptionValue("mip_lp_solver", mip_lp_solver);
    }
    if (crossover && crossover[0] != '\0') {
        highs.setOptionValue("run_crossover", crossover);
    }

    const auto pass_status = highs.passModel(
        static_cast<HighsInt>(ncols), static_cast<HighsInt>(nrows),
        static_cast<HighsInt>(index.size()), static_cast<HighsInt>(MatrixFormat::kColwise),
        static_cast<HighsInt>(lp.sense == engine::Sense::Maximize ? ObjSense::kMaximize
                                                                  : ObjSense::kMinimize),
        0.0, col_cost.data(), col_lower.data(), col_upper.data(), row_lower.data(),
        row_upper.data(), start.data(), index.data(), value.data(), integrality.data());
    if (pass_status == HighsStatus::kError) {
        rec.status = "PassModelError";
        rec.runtime_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - t0).count();
        return rec;
    }

    if (mip_start != nullptr && static_cast<int>(mip_start->size()) == ncols) {
        HighsSolution start_solution;
        start_solution.value_valid = true;
        start_solution.dual_valid = false;
        start_solution.col_value.resize(static_cast<std::size_t>(ncols), 0.0);
        for (int j = 0; j < ncols; ++j) {
            double value_j = (*mip_start)[j];
            if (!std::isfinite(value_j)) value_j = col_lower[static_cast<std::size_t>(j)];
            value_j = std::min(col_upper[static_cast<std::size_t>(j)],
                               std::max(col_lower[static_cast<std::size_t>(j)], value_j));
            if (integrality[static_cast<std::size_t>(j)] !=
                static_cast<HighsInt>(HighsVarType::kContinuous)) {
                const double rounded = std::round(value_j);
                if (std::abs(value_j - rounded) <= 1e-5) value_j = rounded;
            }
            start_solution.col_value[static_cast<std::size_t>(j)] = value_j;
        }
        highs.setSolution(start_solution);
    }

    highs.run();
    const auto t1 = std::chrono::steady_clock::now();
    const HighsModelStatus status = highs.getModelStatus();
    const HighsInfo& info = highs.getInfo();
    rec.success = status == HighsModelStatus::kOptimal ||
                  (highs.getSolution().value_valid &&
                   (status == HighsModelStatus::kTimeLimit ||
                    status == HighsModelStatus::kIterationLimit ||
                    status == HighsModelStatus::kSolutionLimit));
    rec.nodes = static_cast<int>(info.mip_node_count);
    rec.lp_solves = 0;
    rec.objective = info.objective_function_value;
    rec.best_bound = info.mip_dual_bound;
    rec.gap = std::isfinite(info.mip_gap) ? info.mip_gap : 0.0;
    rec.runtime_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    rec.status = std::string("HiGHS ") + highs_status_label(status) +
                 " it=" + std::to_string(static_cast<long long>(info.simplex_iteration_count)) +
                 " ipm=" + std::to_string(static_cast<long long>(info.ipm_iteration_count));
    return rec;
}
#endif

// ─────────────────────────────────────────────────────────────────────────────
// Helpers
// ─────────────────────────────────────────────────────────────────────────────

static void print_header()
{
    std::printf("%-22s  %-22s  %5s  %5s  %5s  %2s  %6s  %6s  %13s  %13s  %10s  %9s\n",
                "Case", "Config",
                "Vars", "Ineq", "Eq",
                "OK", "Nodes", "LPs",
                "Objective", "BestBound",
                "Gap", "Time(ms)");
    std::printf("%s\n", std::string(130, '-').c_str());
}

static void print_record(const RunRecord& r)
{
    std::printf("%-22s  %-22s  %5d  %5d  %5d  %2s  %6d  %6d  %13.2f  %13.2f  %10.2e  %9.1f\n",
                r.case_name.c_str(),
                r.solver_label.c_str(),
                r.n_vars, r.n_ineq, r.n_eq,
                r.success ? "Y" : "N",
                r.nodes, r.lp_solves,
                r.objective, r.best_bound,
                r.gap, r.runtime_ms);
}

static json record_to_json(const RunRecord& r)
{
    json j;
    j["case"]          = r.case_name;
    j["config"]        = r.solver_label;
    j["n_vars"]        = r.n_vars;
    j["n_ineq"]        = r.n_ineq;
    j["n_eq"]          = r.n_eq;
    j["success"]       = r.success;
    j["nodes"]         = r.nodes;
    j["lp_solves"]     = r.lp_solves;
    j["objective"]     = r.objective;
    j["best_bound"]    = r.best_bound;
    j["gap"]           = r.gap;
    j["runtime_ms"]    = r.runtime_ms;
    j["status"]        = r.status;
    return j;
}

static int run_gurobi_start_experiment(double time_limit_sec,
                                       bool skip_native_start,
                                       bool force_strict_root_ipm)
{
    setvbuf(stdout, nullptr, _IONBF, 0);
    TestCase tc;
    tc.name = "UC_118bus_54G_24T";
    tc.inp = build_ieee118_case(/*T=*/24);
    tc.inp.config.solve_sced = false;
    tc.inp.config.solve_lmp = false;

    std::printf("=== Gurobi-start reinjection experiment ===\n");
    std::printf("Case: %s  Re-solve limit: %.0fs\n\n", tc.name.c_str(), time_limit_sec);
    print_header();

    engine::MIPModel mip = build_scuc_mip(tc.inp);
    RunRecord gurobi_rec;
    gurobi_rec.case_name = tc.name;
    gurobi_rec.solver_label = "Gurobi[base]";
    gurobi_rec.n_vars = static_cast<int>(mip.linear_part.vars.size());
    gurobi_rec.n_ineq = static_cast<int>(mip.linear_part.b.size());
    gurobi_rec.n_eq = static_cast<int>(mip.linear_part.beq.size());

    engine::GurobiAdapter gurobi;
    const auto gurobi_t0 = std::chrono::steady_clock::now();
    engine::SolveResult gurobi_res = gurobi.solve_milp(mip);
    const auto gurobi_t1 = std::chrono::steady_clock::now();
    gurobi_rec.success = gurobi_res.stats.success;
    gurobi_rec.objective = gurobi_res.stats.objective;
    gurobi_rec.best_bound = gurobi_res.stats.objective;
    gurobi_rec.gap = gurobi_res.stats.mip_gap;
    gurobi_rec.runtime_ms = std::chrono::duration<double, std::milli>(
        gurobi_t1 - gurobi_t0).count();
    gurobi_rec.status = gurobi_res.stats.status;
    print_record(gurobi_rec);
    std::printf("  status: %s\n", gurobi_rec.status.c_str());
    std::fflush(stdout);

    if (!gurobi_res.stats.success ||
        static_cast<int>(gurobi_res.x.size()) != gurobi_rec.n_vars) {
        std::printf("\nGurobi did not return a usable incumbent vector; stopping experiment.\n");
        return 2;
    }

    const SolutionAudit audit = audit_solution(mip, gurobi_res.x);
    std::printf("  Gurobi incumbent audit: obj=%.12g rowViol=%.3e boundViol=%.3e intViol=%.3e\n",
                audit.objective, audit.max_row_violation, audit.max_bound_violation,
                audit.max_integrality_violation);
    std::fflush(stdout);

#ifdef MIPSOLVERS_HAVE_HIGHS_LIB
    RunRecord highs_start = run_direct_highs_reference(
        tc, time_limit_sec, "choose", "", "HiGHS+Gurobi", &gurobi_res.x);
    print_record(highs_start);
    std::printf("  status: %s\n", highs_start.status.c_str());
    std::fflush(stdout);
#else
    std::printf("HiGHS embedded library is not available in this build.\n");
#endif

    RunRecord strict_start = run_bc_start_reference(
        tc, force_strict_root_ipm ? "StrictIPM+Gurobi" : "Strict+Gurobi",
        &gurobi_res.x, time_limit_sec, true, force_strict_root_ipm);
    print_record(strict_start);
    std::printf("  status: %s\n", strict_start.status.c_str());
    std::fflush(stdout);

    if (!skip_native_start) {
        RunRecord native_start = run_bc_start_reference(
            tc, "Native+Gurobi", &gurobi_res.x, time_limit_sec, false);
        print_record(native_start);
        std::printf("  status: %s\n", native_start.status.c_str());
        std::fflush(stdout);
    } else {
        std::printf("Native+Gurobi skipped by --skip-native-start\n");
    }

    return 0;
}

// ─────────────────────────────────────────────────────────────────────────────
// Two-phase warm-start experiment
// ─────────────────────────────────────────────────────────────────────────────

/// Runs the 118-bus SCUC case with a two-phase strategy:
///   Phase 1 (cold, phase1_frac of budget): Run the full problem cold to build
///            a warm-start bundle (incumbent x, root cuts+basis, pseudocosts).
///   Phase 2 (warm, remaining budget): Re-run the same problem injecting the
///            full bundle from Phase 1, giving HiGHS a tighter upper bound and
///            better branching hints from the start.
///
/// Compared against:
///   Baseline-cold:  Full budget, no warm start.
///   ColdP2:         Same budget as Phase 2, no warm start (controls for the
///                   shortened time limit in Phase 2).
///
/// Goal: measure whether the warm-start bundle from a short cold run lets the
/// solver cut more B&C nodes and achieve a smaller primal gap in the same
/// total wall-clock budget.
static int run_two_phase_experiment(double time_limit_sec)
{
    setvbuf(stdout, nullptr, _IONBF, 0);

    const double phase1_frac = 0.70;
    const double phase1_sec  = time_limit_sec * phase1_frac;
    const double phase2_sec  = time_limit_sec - phase1_sec;

    std::printf("=== Two-phase warm-start experiment ===\n");
    std::printf("Phase-1: %.0fs (cold)  Phase-2: %.0fs (warm)  Total: %.0fs\n\n",
                phase1_sec, phase2_sec, time_limit_sec);

    TestCase tc;
    tc.name = "UC_118bus_54G_24T";
    tc.inp  = build_ieee118_case(/*T=*/24);
    tc.inp.config.solve_sced = false;
    tc.inp.config.solve_lmp  = false;

    print_header();

    // Helper: build default BCOptions for our StrictHiGHS pipeline.
    auto make_opts = [](double tlimit) -> engine::BCOptions {
        engine::BCOptions opt;
        opt.use_vendored_highs_lp_kernel = true;
        opt.time_limit_sec               = tlimit;
        opt.gap_tol                      = 1e-3;
        opt.verbose                      = false;
        if (!g_no_root_cap)
            opt.highs_mip_root_simplex_iteration_limit = 50000;
        return opt;
    };

    // Helper: inline solve, returning both the record and the raw BCResult
    // (so the caller can extract the warm-start bundle).
    struct SolveOut { RunRecord rec; engine::BCResult res; };
    auto run_inline = [&](const char* label,
                          engine::BCOptions opt,
                          const Eigen::VectorXd* warm_x) -> SolveOut
    {
        engine::MIPModel mip = build_scuc_mip(tc.inp);
        const int nvars = static_cast<int>(mip.linear_part.vars.size());
        const int nineq = static_cast<int>(mip.linear_part.b.size());
        const int neq   = static_cast<int>(mip.linear_part.beq.size());
        if (warm_x && static_cast<int>(warm_x->size()) == nvars)
            mip.initial_solution = *warm_x;

        const auto t0 = std::chrono::steady_clock::now();
        engine::BCResult res = engine::solve_milp_bc(mip, opt);
        const auto t1 = std::chrono::steady_clock::now();

        RunRecord rec;
        rec.case_name    = tc.name;
        rec.solver_label = label;
        rec.n_vars       = nvars;
        rec.n_ineq       = nineq;
        rec.n_eq         = neq;
        rec.success      = res.stats.success;
        rec.nodes        = res.bc_stats.nodes_explored;
        rec.lp_solves    = res.bc_stats.lp_solves;
        rec.objective    = res.stats.objective;
        rec.best_bound   = res.bc_stats.best_bound;
        rec.runtime_ms   = std::chrono::duration<double, std::milli>(t1 - t0).count();
        rec.status       = res.bc_stats.status;
        if (rec.success && std::isfinite(rec.objective) && std::isfinite(rec.best_bound))
            rec.gap = std::abs(rec.objective - rec.best_bound)
                      / (1.0 + std::abs(rec.objective));
        return {std::move(rec), std::move(res)};
    };

    // 1. Baseline: full budget, cold start (reference).
    {
        auto out = run_inline("Baseline-cold[S]", make_opts(time_limit_sec), nullptr);
        print_record(out.rec);
        std::printf("  status: %s\n", out.rec.status.c_str());
        std::fflush(stdout);
    }

    // 2. Phase 1: short cold solve — builds the warm-start bundle.
    auto p1 = run_inline("2Ph-P1-cold[S]", make_opts(phase1_sec), nullptr);
    print_record(p1.rec);
    std::printf("  status: %s  incumbent: %.2f\n",
                p1.rec.status.c_str(), p1.rec.objective);
    std::fflush(stdout);

    // 3. Phase 2 (warm): remaining budget with full warm-start bundle from P1.
    //    Injects: incumbent x, root cuts+basis (skip root LP loop), pseudocosts
    //    (guide branching from node 1).
    {
        engine::BCOptions p2_opt = make_opts(phase2_sec);
        p2_opt.highs_root_cut_warm_start   = p1.res.highs_root_cuts;
        p2_opt.highs_root_basis_warm_start = p1.res.highs_root_basis;
        p2_opt.highs_pseudocost_warm_start = p1.res.highs_pseudocost_init;

        const Eigen::VectorXd* warm_p1 =
            (p1.res.x.size() > 0) ? &p1.res.x : nullptr;
        auto out = run_inline("2Ph-P2-warm[S]", p2_opt, warm_p1);
        print_record(out.rec);
        std::printf("  status: %s\n", out.rec.status.c_str());
        std::fflush(stdout);
    }

    // 4. Cold-P2: same budget as Phase 2, no warm start — controls for the
    //    shorter time available to Phase 2.
    {
        auto out = run_inline("ColdP2[S]", make_opts(phase2_sec), nullptr);
        print_record(out.rec);
        std::printf("  status: %s\n", out.rec.status.c_str());
        std::fflush(stdout);
    }

    std::printf("\n");
    return 0;
}

// ─────────────────────────────────────────────────────────────────────────────
// --heuristic-test: compare primal heuristic strategies on 118-bus
// ─────────────────────────────────────────────────────────────────────────────

static int run_heuristic_experiment(double time_limit_sec)
{
    setvbuf(stdout, nullptr, _IONBF, 0);

    std::printf("=== Primal heuristic experiment (118-bus, %.0fs) ===\n", time_limit_sec);
    std::printf("Configs: Baseline / ZIShift / RootRENS / ZIShift+RENS / Production\n");
    std::printf("ZIShift  = HiGHS ZI-Round + Shifting (LP-guided rounding after root LP)\n");
    std::printf("RootRENS = Root RENS dive over fractional binaries (~40/1296 for 118-bus)\n");
    std::printf("\n");

    TestCase tc;
    tc.name = "UC_118bus_54G_24T";
    tc.inp  = build_ieee118_case(/*T=*/24);
    tc.inp.config.solve_sced = false;
    tc.inp.config.solve_lmp  = false;

    print_header();

    auto run = [&](const char* label, engine::BCOptions opt) {
        engine::MIPModel mip = build_scuc_mip(tc.inp);
        const int nvars = static_cast<int>(mip.linear_part.vars.size());
        const int nineq = static_cast<int>(mip.linear_part.b.size());
        const int neq   = static_cast<int>(mip.linear_part.beq.size());
        const auto t0 = std::chrono::steady_clock::now();
        engine::BCResult res = engine::solve_milp_bc(mip, opt);
        const auto t1 = std::chrono::steady_clock::now();
        RunRecord rec;
        rec.case_name    = tc.name;
        rec.solver_label = label;
        rec.n_vars       = nvars;
        rec.n_ineq       = nineq;
        rec.n_eq         = neq;
        rec.success      = res.stats.success;
        rec.nodes        = res.bc_stats.nodes_explored;
        rec.lp_solves    = res.bc_stats.lp_solves;
        rec.objective    = res.stats.objective;
        rec.best_bound   = res.bc_stats.best_bound;
        rec.runtime_ms   = std::chrono::duration<double, std::milli>(t1 - t0).count();
        rec.status       = res.bc_stats.status;
        if (rec.success && std::isfinite(rec.objective) && std::isfinite(rec.best_bound))
            rec.gap = std::abs(rec.objective - rec.best_bound)
                      / (1.0 + std::abs(rec.objective));
        print_record(rec);
        std::printf("  status: %s\n", rec.status.c_str());
        std::fflush(stdout);
    };

    // Shared base: StrictHiGHS path with 50K root-LP iteration cap.
    auto make_base = [&]() -> engine::BCOptions {
        engine::BCOptions opt;
        opt.use_vendored_highs_lp_kernel = true;
        opt.time_limit_sec               = time_limit_sec;
        opt.gap_tol                      = 1e-3;
        opt.verbose                      = false;
        if (!g_no_root_cap)
            opt.highs_mip_root_simplex_iteration_limit = 50000;
        return opt;
    };

    // 1. Baseline: current benchmark config — no extra heuristics.
    run("Baseline[S]", make_base());

    // 2. ZI-Round + Shifting: LP-guided rounding that HiGHS invokes after the
    //    root LP and at regular tree intervals.  ZI-Round exploits column lock
    //    counts and LP coefficients to round fractional binaries without
    //    violating constraints; Shifting adjusts rounded values to restore
    //    row feasibility.  Both are cheap (a few passes per heuristic call)
    //    but frequently improve incumbents on degenerate UC LPs.
    {
        auto opt = make_base();
        opt.highs_mip_run_zi_round = true;
        opt.highs_mip_run_shifting = true;
        run("ZIShift[S]", opt);
    }

    // 3. Root RENS dive: after the root LP, fix the binaries that are already
    //    integral in the LP relaxation and solve a sub-MIP over only the
    //    fractional ones.  On 118-bus ~40 of 1296 binaries are fractional at
    //    the root, so the sub-MIP is tiny and usually solves within the 3s
    //    budget.  This is the "diving" heuristic.
    {
        auto opt = make_base();
        opt.enable_root_low_fractionality_rens = true;
        run("RootRENS[S]", opt);
    }

    // 4. ZI-Round + Shifting + Root RENS: combined LP rounding and diving.
    {
        auto opt = make_base();
        opt.highs_mip_run_zi_round             = true;
        opt.highs_mip_run_shifting             = true;
        opt.enable_root_low_fractionality_rens = true;
        run("ZIShift+RENS[S]", opt);
    }

    // 5. Full production options (identical to make_scuc_bc_options in scuc.cpp).
    //    Includes ZI-Round, Shifting, forced presolve, extended LP-cut age
    //    limit, and symmetry detection — the complete production pipeline.
    {
        engine::BCOptions opt;
        opt.time_limit_sec = time_limit_sec;
        opt.gap_tol        = 1e-3;
        opt.verbose        = false;
        opt = engine::make_strict_highs_production_options(opt);
        if (!g_no_root_cap)
            opt.highs_mip_root_simplex_iteration_limit = 50000;
        run("Production[S]", opt);
    }

    std::printf("\n");
    return 0;
}

// ─────────────────────────────────────────────────────────────────────────────
// --nocap-test: compare capped vs uncapped root LP
// ─────────────────────────────────────────────────────────────────────────────

static int run_nocap_test(double long_time_sec)
{
    setvbuf(stdout, nullptr, _IONBF, 0);

    std::printf("=== Root LP cap removal test (118-bus) ===\n");
    std::printf("Baseline+cap :  120s, 50K root-LP iter cap (current benchmark config)\n");
    std::printf("Baseline-cap : %.0fs, root LP runs to convergence\n", long_time_sec);
    std::printf("Production-cap: %.0fs, no cap + full production options\n\n", long_time_sec);

    TestCase tc;
    tc.name = "UC_118bus_54G_24T";
    tc.inp  = build_ieee118_case(/*T=*/24);
    tc.inp.config.solve_sced = false;
    tc.inp.config.solve_lmp  = false;

    print_header();

    auto run = [&](const char* label, engine::BCOptions opt) {
        engine::MIPModel mip = build_scuc_mip(tc.inp);
        const int nvars = static_cast<int>(mip.linear_part.vars.size());
        const int nineq = static_cast<int>(mip.linear_part.b.size());
        const int neq   = static_cast<int>(mip.linear_part.beq.size());
        const auto t0 = std::chrono::steady_clock::now();
        engine::BCResult res = engine::solve_milp_bc(mip, opt);
        const auto t1 = std::chrono::steady_clock::now();
        RunRecord rec;
        rec.case_name    = tc.name;
        rec.solver_label = label;
        rec.n_vars       = nvars;
        rec.n_ineq       = nineq;
        rec.n_eq         = neq;
        rec.success      = res.stats.success;
        rec.nodes        = res.bc_stats.nodes_explored;
        rec.lp_solves    = res.bc_stats.lp_solves;
        rec.objective    = res.stats.objective;
        rec.best_bound   = res.bc_stats.best_bound;
        rec.runtime_ms   = std::chrono::duration<double, std::milli>(t1 - t0).count();
        rec.status       = res.bc_stats.status;
        if (rec.success && std::isfinite(rec.objective) && std::isfinite(rec.best_bound))
            rec.gap = std::abs(rec.objective - rec.best_bound)
                      / (1.0 + std::abs(rec.objective));
        print_record(rec);
        std::printf("  status: %s\n", rec.status.c_str());
        std::fflush(stdout);
        return res;
    };

    // 1. Reference: Baseline with the 50K root-LP iteration cap.
    //    Always uses 120s so it matches the benchmark config exactly.
    {
        engine::BCOptions opt;
        opt.use_vendored_highs_lp_kernel           = true;
        opt.time_limit_sec                         = 120.0;
        opt.gap_tol                                = 1e-3;
        opt.verbose                                = false;
        opt.highs_mip_root_simplex_iteration_limit = 50000;
        run("Baseline[S]+cap", opt);
    }

    // 2. Baseline with no root cap: lets the simplex converge fully on the
    //    root LP, giving a tighter lower bound and a better cut set before
    //    the first branch node.  The remaining budget is available for tree.
    {
        engine::BCOptions opt;
        opt.use_vendored_highs_lp_kernel = true;
        opt.time_limit_sec               = long_time_sec;
        opt.gap_tol                      = 1e-3;
        opt.verbose                      = false;
        run("Baseline[S]-cap", opt);
    }

    // 3. Full production options with no root cap: symmetry detection,
    //    forced presolve, extended LP-cut age limit — all without the
    //    artificial root LP truncation.
    {
        engine::BCOptions opt;
        opt.time_limit_sec = long_time_sec;
        opt.gap_tol        = 1e-3;
        opt.verbose        = false;
        opt = engine::make_strict_highs_production_options(opt);
        run("Production[S]-cap", opt);
    }

    std::printf("\n");
    return 0;
}

// ─────────────────────────────────────────────────────────────────────────────
// --inject-test: warm-start a capped 120s solve with a better incumbent
//   obtained from a preceding long no-cap seed run.
// ─────────────────────────────────────────────────────────────────────────────

static int run_inject_test(double seed_time_sec)
{
    setvbuf(stdout, nullptr, _IONBF, 0);

    std::printf("=== Incumbent injection test (118-bus) ===\n");
    std::printf("Seed  : %.0fs, no root-LP cap  → x_seed (best achievable in budget)\n",
                seed_time_sec);
    std::printf("Cold  : 120s, 50K cap, no injection (control)\n");
    std::printf("Inject: 120s, 50K cap, x_seed pre-loaded as initial_solution\n");
    std::printf("        The solver sees a tight primal bound from the first simplex\n");
    std::printf("        iteration of the root LP, enabling stronger objective cutoffs\n");
    std::printf("        and reduced-cost fixing throughout root and tree phases.\n\n");

    TestCase tc;
    tc.name = "UC_118bus_54G_24T";
    tc.inp  = build_ieee118_case(/*T=*/24);
    tc.inp.config.solve_sced = false;
    tc.inp.config.solve_lmp  = false;

    print_header();

    // Helper: solve one config, print the record, return the full BCResult.
    auto run_one = [&](const char* label, engine::MIPModel mip,
                       engine::BCOptions opt) -> engine::BCResult {
        const int nvars = static_cast<int>(mip.linear_part.vars.size());
        const int nineq = static_cast<int>(mip.linear_part.b.size());
        const int neq   = static_cast<int>(mip.linear_part.beq.size());
        const auto t0 = std::chrono::steady_clock::now();
        engine::BCResult res = engine::solve_milp_bc(mip, opt);
        const auto t1 = std::chrono::steady_clock::now();
        RunRecord rec;
        rec.case_name    = tc.name;
        rec.solver_label = label;
        rec.n_vars       = nvars;
        rec.n_ineq       = nineq;
        rec.n_eq         = neq;
        rec.success      = res.stats.success;
        rec.nodes        = res.bc_stats.nodes_explored;
        rec.lp_solves    = res.bc_stats.lp_solves;
        rec.objective    = res.stats.objective;
        rec.best_bound   = res.bc_stats.best_bound;
        rec.runtime_ms   = std::chrono::duration<double, std::milli>(t1 - t0).count();
        rec.status       = res.bc_stats.status;
        if (rec.success && std::isfinite(rec.objective) && std::isfinite(rec.best_bound))
            rec.gap = std::abs(rec.objective - rec.best_bound)
                      / (1.0 + std::abs(rec.objective));
        print_record(rec);
        std::printf("  status: %s\n", rec.status.c_str());
        std::fflush(stdout);
        return res;
    };

    // Phase 0 — seed: no root-LP cap, full long budget.
    // We want the best incumbent available so x_seed tightens the cutoff
    // in the subsequent capped solves as much as possible.
    std::printf("--- Phase 0: seed solve (%.0fs, no root LP cap) ---\n", seed_time_sec);
    engine::BCOptions seed_opt;
    seed_opt.use_vendored_highs_lp_kernel = true;
    seed_opt.time_limit_sec               = seed_time_sec;
    seed_opt.gap_tol                      = 1e-3;
    seed_opt.verbose                      = false;
    engine::BCResult seed_res = run_one("Seed[S]-cap", build_scuc_mip(tc.inp), seed_opt);
    std::printf("  => x_seed objective: %.2f  best_bound: %.2f  nodes: %d\n\n",
                seed_res.stats.objective, seed_res.bc_stats.best_bound,
                seed_res.bc_stats.nodes_explored);

    // Phase 1 — cold control: standard 120s capped solve, no injection.
    std::printf("--- Phase 1: cold 120s capped solve (control) ---\n");
    {
        engine::BCOptions opt;
        opt.use_vendored_highs_lp_kernel           = true;
        opt.time_limit_sec                         = 120.0;
        opt.gap_tol                                = 1e-3;
        opt.verbose                                = false;
        opt.highs_mip_root_simplex_iteration_limit = 50000;
        run_one("Cold[S]+cap", build_scuc_mip(tc.inp), opt);
    }
    std::printf("\n");

    // Phase 2 — inject: same 120s capped solve, but mip.initial_solution is
    // pre-loaded with x_seed so the solver has a proven feasible primal bound
    // from its very first iteration.  Stronger objective cutoff → more pruning
    // during root LP (reduced-cost fixing) and tighter branching bounds.
    std::printf("--- Phase 2: inject x_seed into 120s capped solve ---\n");
    {
        engine::MIPModel mip = build_scuc_mip(tc.inp);
        const auto expected = static_cast<Eigen::Index>(mip.linear_part.vars.size());
        if (seed_res.x.size() == expected) {
            mip.initial_solution = seed_res.x;
        } else {
            std::printf("  WARNING: x_seed size %ld != model vars %ld; skipping injection\n",
                        static_cast<long>(seed_res.x.size()),
                        static_cast<long>(expected));
        }
        engine::BCOptions opt;
        opt.use_vendored_highs_lp_kernel           = true;
        opt.time_limit_sec                         = 120.0;
        opt.gap_tol                                = 1e-3;
        opt.verbose                                = false;
        opt.highs_mip_root_simplex_iteration_limit = 50000;
        run_one("Inject[S]+cap", std::move(mip), opt);
    }

    std::printf("\n");
    return 0;
}

// ─────────────────────────────────────────────────────────────────────────────
// --bound-test: isolate lower-bound strengthening strategies on 118-bus
// ─────────────────────────────────────────────────────────────────────────────

static int run_bound_test(double time_limit_sec)
{
    setvbuf(stdout, nullptr, _IONBF, 0);

    std::printf("=== Lower-bound strengthening test (118-bus, %.0fs) ===\n",
                time_limit_sec);
    std::printf("Goal: compare dual-bound movement from root cuts, cut depth,\n");
    std::printf("      production dual knobs, and IPM-root setup.\n\n");

    TestCase tc;
    tc.name = "UC_118bus_54G_24T";
    tc.inp  = build_ieee118_case(/*T=*/24);
    tc.inp.config.solve_sced = false;
    tc.inp.config.solve_lmp  = false;

    print_header();

    auto run = [&](const char* label, engine::BCOptions opt) {
        engine::MIPModel mip = build_scuc_mip(tc.inp);
        const int nvars = static_cast<int>(mip.linear_part.vars.size());
        const int nineq = static_cast<int>(mip.linear_part.b.size());
        const int neq   = static_cast<int>(mip.linear_part.beq.size());
        const auto t0 = std::chrono::steady_clock::now();
        engine::BCResult res = engine::solve_milp_bc(mip, opt);
        const auto t1 = std::chrono::steady_clock::now();
        RunRecord rec;
        rec.case_name    = tc.name;
        rec.solver_label = label;
        rec.n_vars       = nvars;
        rec.n_ineq       = nineq;
        rec.n_eq         = neq;
        rec.success      = res.stats.success;
        rec.nodes        = res.bc_stats.nodes_explored;
        rec.lp_solves    = res.bc_stats.lp_solves;
        rec.objective    = res.stats.objective;
        rec.best_bound   = res.bc_stats.best_bound;
        rec.runtime_ms   = std::chrono::duration<double, std::milli>(t1 - t0).count();
        rec.status       = res.bc_stats.status;
        if (rec.success && std::isfinite(rec.objective) && std::isfinite(rec.best_bound))
            rec.gap = std::abs(rec.objective - rec.best_bound)
                      / (1.0 + std::abs(rec.objective));
        print_record(rec);
        std::printf("  status: %s\n", rec.status.c_str());
        std::fflush(stdout);
    };

    auto make_base = [&]() -> engine::BCOptions {
        engine::BCOptions opt;
        opt.use_vendored_highs_lp_kernel = true;
        opt.time_limit_sec               = time_limit_sec;
        opt.gap_tol                      = 1e-3;
        opt.verbose                      = false;
        return opt;
    };

    // 1. Current no-cap baseline: simplex root, default 50 root separation rounds.
    run("Baseline50r[S]", make_base());

    // 2. No root cuts: measures the raw LP relaxation and dynamic cut contribution.
    {
        auto opt = make_base();
        opt.highs_max_root_sepa_rounds = -1;
        opt.cuts = engine::CutType::None;
        run("NoRootCuts[S]", opt);
    }

    // 3. More root separation: test whether the root bound is separator-starved.
    {
        auto opt = make_base();
        opt.highs_max_root_sepa_rounds = 100;
        run("Sepa100[S]", opt);
    }

    // 4. Production dual knobs without primal heuristics.  This isolates
    //    force-presolve, substitution fill-in, LP cut age, and symmetry from
    //    ZI-Round/Shifting, which worsened incumbent quality in --heuristic-test.
    {
        auto opt = make_base();
        opt = engine::make_strict_highs_production_options(opt);
        opt.highs_mip_run_zi_round = false;
        opt.highs_mip_run_shifting = false;
        run("ProdDual[S]", opt);
    }

    // 5. Production dual knobs plus deeper root separation.
    {
        auto opt = make_base();
        opt = engine::make_strict_highs_production_options(opt);
        opt.highs_mip_run_zi_round = false;
        opt.highs_mip_run_shifting = false;
        opt.highs_max_root_sepa_rounds = 100;
        run("ProdDual+Sepa100[S]", opt);
    }

    // 6. IPM root with crossover: tests whether a stronger/cleaner root LP
    //    setup improves root cut quality enough to move the lower bound.
    {
        auto opt = make_base();
        opt.highs_mip_lp_solver = "ipm";
        opt.highs_mip_root_crossover = "on";
        run("IPMroot+xov[S]", opt);
    }

    std::printf("\n");
    return 0;
}

// ─────────────────────────────────────────────────────────────────────────────
// --production-118: one 118-bus production solve for certificate testing
// ─────────────────────────────────────────────────────────────────────────────

static int run_production_118_test(double time_limit_sec,
                                   const std::string& json_path)
{
    setvbuf(stdout, nullptr, _IONBF, 0);

    std::printf("=== Production StrictHiGHS test (118-bus, %.0fs) ===\n",
                time_limit_sec);
    std::printf("Config: make_strict_highs_production_options, no root LP cap\n\n");

    TestCase tc;
    tc.name = "UC_118bus_54G_24T";
    tc.inp  = build_ieee118_case(/*T=*/24);
    tc.inp.config.solve_sced = false;
    tc.inp.config.solve_lmp  = false;

    engine::MIPModel mip = build_scuc_mip(tc.inp);
    RunRecord rec;
    rec.case_name    = tc.name;
    rec.solver_label = "Production[S]-proof";
    rec.n_vars       = static_cast<int>(mip.linear_part.vars.size());
    rec.n_ineq       = static_cast<int>(mip.linear_part.b.size());
    rec.n_eq         = static_cast<int>(mip.linear_part.beq.size());

    engine::BCOptions opt;
    opt.time_limit_sec = time_limit_sec;
    opt.gap_tol        = 1e-3;
    opt.verbose        = false;
    // Use the SAME config path as the real StrictHighsBranchAndCutAdapter:
    // make_strict_highs_problem_options applies the large-model IPM-root +
    // crossover switch (needed for the 26400-var 118-bus root) on top of the
    // base production options.  Calling make_strict_highs_production_options
    // alone leaves the root on simplex and yields a weak (NoCuts-like) bound.
    opt = engine::make_strict_highs_problem_options(mip, opt);

    print_header();
    const auto t0 = std::chrono::steady_clock::now();
    engine::BCResult res = engine::solve_milp_bc(mip, opt);
    const auto t1 = std::chrono::steady_clock::now();

    rec.success    = res.stats.success;
    rec.nodes      = res.bc_stats.nodes_explored;
    rec.lp_solves  = res.bc_stats.lp_solves;
    rec.objective  = res.stats.objective;
    rec.best_bound = res.bc_stats.best_bound;
    rec.runtime_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    rec.status     = res.bc_stats.status;
    if (rec.success && std::isfinite(rec.objective) && std::isfinite(rec.best_bound)) {
        rec.gap = std::abs(rec.objective - rec.best_bound)
                  / (1.0 + std::abs(rec.objective));
    }
    print_record(rec);
    std::printf("  status: %s\n", rec.status.c_str());
    std::fflush(stdout);

    if (!json_path.empty()) {
        json jout = json::array();
        jout.push_back(record_to_json(rec));
        std::ofstream f(json_path);
        if (f) {
            f << std::setw(2) << jout << "\n";
            std::printf("\nResults written to %s\n", json_path.c_str());
        } else {
            std::fprintf(stderr, "Cannot open %s for writing\n", json_path.c_str());
            return 1;
        }
    }

    std::printf("\n");
    return 0;
}

// ─────────────────────────────────────────────────────────────────────────────
// main
// ─────────────────────────────────────────────────────────────────────────────

int main(int argc, char** argv)
{
    bool        full_mode           = false;
    bool        smoke_mode          = false;
    bool        large_only          = false;
    bool        gurobi_start_mode   = false;
    bool        two_phase_mode      = false;
    bool        heuristic_test_mode = false;
    bool        nocap_test_mode     = false;
    bool        inject_test_mode    = false;
    bool        bound_test_mode     = false;
    bool        production_118_mode = false;
    bool        million_hotpath_mode = false;
    bool        skip_native_start   = false;
    bool        force_strict_root_ipm = false;
    double      time_limit = 120.0;
    std::string json_path;

    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--smoke") == 0) {
            smoke_mode = true;
        } else if (std::strcmp(argv[i], "--full") == 0) {
            full_mode = true;
        } else if (std::strcmp(argv[i], "--large-only") == 0) {
            full_mode  = true;   // implies --full
            large_only = true;
        } else if (std::strcmp(argv[i], "--gurobi-start") == 0) {
            gurobi_start_mode = true;
        } else if (std::strcmp(argv[i], "--two-phase") == 0) {
            two_phase_mode = true;
        } else if (std::strcmp(argv[i], "--heuristic-test") == 0) {
            heuristic_test_mode = true;
        } else if (std::strcmp(argv[i], "--nocap-test") == 0) {
            nocap_test_mode = true;
        } else if (std::strcmp(argv[i], "--inject-test") == 0) {
            inject_test_mode = true;
        } else if (std::strcmp(argv[i], "--bound-test") == 0) {
            bound_test_mode = true;
        } else if (std::strcmp(argv[i], "--production-118") == 0) {
            production_118_mode = true;
        } else if (std::strcmp(argv[i], "--million-hotpath") == 0) {
            million_hotpath_mode = true;
        } else if (std::strcmp(argv[i], "--skip-native-start") == 0) {
            skip_native_start = true;
        } else if (std::strcmp(argv[i], "--force-strict-root-ipm") == 0) {
            force_strict_root_ipm = true;
        } else if (std::strcmp(argv[i], "--no-root-cap") == 0) {
            g_no_root_cap = true;
        } else if (std::strcmp(argv[i], "--no-flow") == 0) {
            g_no_flow = true;
        } else if (std::strcmp(argv[i], "--no-highs-adapter") == 0) {
            // Accepted for backward compat with ctest invocation; no-op here
            // since this runner always uses B&C[StrictHiGHS] directly.
        } else if (std::strcmp(argv[i], "--json") == 0 && i + 1 < argc) {
            json_path = argv[++i];
        } else if (std::strcmp(argv[i], "--time-limit") == 0 && i + 1 < argc) {
            time_limit = std::atof(argv[++i]);
            if (time_limit <= 0.0) {
                std::fprintf(stderr, "--time-limit must be positive\n");
                return 1;
            }
        } else {
            std::fprintf(stderr, "Unknown argument: %s\n", argv[i]);
            return 1;
        }
    }

    if (gurobi_start_mode) {
        return run_gurobi_start_experiment(time_limit, skip_native_start,
                                           force_strict_root_ipm);
    }
    if (million_hotpath_mode) {
        return run_million_hotpath_benchmark();
    }
    if (two_phase_mode) {
        return run_two_phase_experiment(time_limit);
    }
    if (heuristic_test_mode) {
        return run_heuristic_experiment(time_limit);
    }
    if (nocap_test_mode) {
        return run_nocap_test(time_limit);
    }
    if (inject_test_mode) {
        return run_inject_test(time_limit);
    }
    if (bound_test_mode) {
        return run_bound_test(time_limit);
    }
    if (production_118_mode) {
        return run_production_118_test(time_limit, json_path);
    }

    std::printf("=== MIPSolvers MILP benchmark — A/B/H improvement test ===\n");
    std::printf("Mode: %s  Time limit: %.0fs per run\n\n",
                smoke_mode ? "smoke" : (full_mode ? "full" : "quick"),
                time_limit);

    // ── Build test cases ──────────────────────────────────────────────────────

    std::vector<TestCase> cases;

    // Quick cases (skipped when --large-only is given)
    if (!large_only) {
        SCUCInput inp = build_6bus_case(/*T=*/4);
        inp.config.solve_sced = false;
        inp.config.solve_lmp  = false;
        cases.push_back({"UC_6bus_3G_4T", std::move(inp)});
    }
    if (!large_only && !smoke_mode) {
        SCUCInput inp = build_ieee39_case(/*T=*/24);
        inp.config.solve_sced = false;
        inp.config.solve_lmp  = false;
        cases.push_back({"UC_39bus_10G_24T", std::move(inp)});
    }

    // Full cases (added when --full is given)
    if (full_mode && !smoke_mode) {
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

    // ── Run benchmark ─────────────────────────────────────────────────────────

    std::vector<RunRecord> all_records;
    int solved = 0;

    for (const auto& tc : cases) {
        std::printf("--- %s ---\n", tc.name.c_str());
        print_header();

        // ── No-flow diagnostic (--no-flow flag) ───────────────────────────────
        // Runs Baseline[S] twice: once with flow constraints intact and once
        // with all branch/section ratings inflated to 2e5 MW so that
        // build_formulation skips every line flow constraint row.  The delta
        // in n_ineq, best_bound, and gap directly measures the transmission
        // constraint contribution to the LP relaxation quality.
        if (g_no_flow) {
            // With-flow run (reference)
            RunRecord flow_rec = run_config(tc, kConfigs[0], nullptr, time_limit);
            flow_rec.solver_label = "Baseline[S]+flow";
            print_record(flow_rec);
            std::printf("  status: %s\n", flow_rec.status.c_str());
            std::fflush(stdout);
            all_records.push_back(flow_rec);
            if (flow_rec.success) ++solved;

            // No-flow run: inflate all ratings so build_formulation skips flow rows
            SCUCInput nf_inp = tc.inp;
            for (auto& br : nf_inp.branches)
                br.rating_mw = 2e5;
            for (auto& sec : nf_inp.sections) {
                sec.rating_fwd_mw = 2e5;
                sec.rating_rev_mw = 2e5;
            }
            TestCase nf_tc{tc.name, std::move(nf_inp)};
            SolverConfig noflow_cfg = kConfigs[0];
            noflow_cfg.label = "Baseline[S]-noflow";
            RunRecord noflow_rec = run_config(nf_tc, noflow_cfg, nullptr, time_limit);
            print_record(noflow_rec);
            std::printf("  status: %s\n", noflow_rec.status.c_str());
            std::fflush(stdout);
            all_records.push_back(noflow_rec);
            if (noflow_rec.success) ++solved;
            continue;
        }

#ifdef MIPSOLVERS_HAVE_HIGHS_LIB
        RunRecord direct_highs = run_direct_highs_reference(
            tc, time_limit, "choose", "", "HiGHS[direct]");
        print_record(direct_highs);
        std::printf("  status: %s\n", direct_highs.status.c_str());
        // HiGHS[noFC]: same but with our FlowCover separator disabled via env var.
        // Comparing against HiGHS[direct] isolates the contribution of
        // HighsFlowCoverSeparator to HiGHS B&B solving performance.
        set_env_var("HIGHS_NO_FLOWCOVER", "1");
        RunRecord direct_highs_nofc = run_direct_highs_reference(
            tc, time_limit, "choose", "", "HiGHS[noFC]");
        unset_env_var("HIGHS_NO_FLOWCOVER");
        print_record(direct_highs_nofc);
        std::printf("  status: %s\n", direct_highs_nofc.status.c_str());
#endif

        const SolverConfig root_fallback_12k = {
            "Fallback12k[S]", false, 0, "choose", "", false, 12000 };
        RunRecord root_fallback = run_config(tc, root_fallback_12k, nullptr, time_limit);
        print_record(root_fallback);
        std::printf("  status: %s\n", root_fallback.status.c_str());

        // Step 1: cold baseline run (also produces the warm-start solution)
        engine::MIPModel mip_cold = build_scuc_mip(tc.inp);
        engine::BCOptions cold_opt;
        cold_opt.use_vendored_highs_lp_kernel = true;
        cold_opt.time_limit_sec               = time_limit;
        cold_opt.gap_tol                      = 1e-3;
        cold_opt.verbose                      = false;
        cold_opt.highs_mip_lp_solver          = "choose";  // simplex baseline → fair warm_x
        // Cap the root LP simplex iterations.  Without this limit the root LP
        // consumes the entire time_limit for large cases (118-bus: ~60s) and
        // never finishes, producing an empty highs_root_cuts and a mid-crawl
        // basis — both useless for Baseline+Cuts[S] warm-start.
        // Raising the cap from 25000 → 50000 (~40s for 118-bus) lets HiGHS
        // complete more cut rounds before the iteration limit hits, producing
        // tighter root cuts and a better root basis for the warm run.  The cold
        // B&B still has ~80s of budget and typically finds a feasible incumbent.
        if (!g_no_root_cap)
            cold_opt.highs_mip_root_simplex_iteration_limit = 50000;

        const auto t0 = std::chrono::steady_clock::now();
        engine::BCResult cold_res = engine::solve_milp_bc(mip_cold, cold_opt);
        const auto t1 = std::chrono::steady_clock::now();

        RunRecord baseline;
        baseline.case_name    = tc.name;
        baseline.solver_label = kConfigs[0].label;  // "Baseline"
        baseline.n_vars       = static_cast<int>(mip_cold.linear_part.vars.size());
        baseline.n_ineq       = static_cast<int>(mip_cold.linear_part.b.size());
        baseline.n_eq         = static_cast<int>(mip_cold.linear_part.beq.size());
        baseline.success      = cold_res.stats.success;
        baseline.nodes        = cold_res.bc_stats.nodes_explored;
        baseline.lp_solves    = cold_res.bc_stats.lp_solves;
        baseline.objective    = cold_res.stats.objective;
        baseline.best_bound   = cold_res.bc_stats.best_bound;
        baseline.runtime_ms   = std::chrono::duration<double, std::milli>(t1 - t0).count();
        baseline.status       = cold_res.bc_stats.status;
        if (baseline.success && std::isfinite(baseline.objective)
                             && std::isfinite(baseline.best_bound)) {
            baseline.gap = std::abs(baseline.objective - baseline.best_bound)
                           / (1.0 + std::abs(baseline.objective));
        }
        print_record(baseline);
        all_records.push_back(baseline);
        if (baseline.success) ++solved;

        // Step 1a: NoCuts[S] — pure B&B with no cut generation.  Quantifies the
        //          overall benefit of cuts by comparison against Baseline[S].
        RunRecord no_cuts = run_config(tc, kConfigs[5], nullptr, time_limit);
        print_record(no_cuts);
        all_records.push_back(no_cuts);
        if (no_cuts.success) ++solved;

        // Step 1b: FCOnly[S] — FlowCover cuts only.  Isolates the contribution of
        //          the native flow-cover cut family relative to NoCuts[S] and
        //          Baseline[S] (which uses all cut families except FlowCover).
        RunRecord fc_only = run_config(tc, kConfigs[6], nullptr, time_limit);
        print_record(fc_only);
        all_records.push_back(fc_only);
        if (fc_only.success) ++solved;

        // Step 1c: Baseline+Cuts[S] — same as Step 1 but with root cuts AND the
        //          warm incumbent from that solve injected.
        //          Cuts (Improvement 6): cap sepa rounds to 50, freeing ~57s for B&B.
        //          Warm incumbent (Improvement A/B): sets upper_limit early, skipping
        //          the feasibility pump that would otherwise consume all freed time.
        {
          engine::MIPModel mip_cuts = build_scuc_mip(tc.inp);
          // Inject warm incumbent to skip feasibility pump
          if (cold_res.x.size() > 0 &&
              static_cast<int>(cold_res.x.size()) == static_cast<int>(mip_cuts.linear_part.vars.size()))
            mip_cuts.initial_solution = cold_res.x;
          engine::BCOptions cuts_opt;
          cuts_opt.use_vendored_highs_lp_kernel = true;
          cuts_opt.time_limit_sec               = time_limit;
          cuts_opt.gap_tol                      = 1e-3;
          cuts_opt.verbose                      = false;
          cuts_opt.highs_mip_lp_solver          = "choose";
          cuts_opt.highs_root_cut_warm_start      = cold_res.highs_root_cuts;
          cuts_opt.highs_root_basis_warm_start    = cold_res.highs_root_basis;
          cuts_opt.highs_pseudocost_warm_start    = cold_res.highs_pseudocost_init;

          const auto tc0 = std::chrono::steady_clock::now();
          engine::BCResult cuts_res = engine::solve_milp_bc(mip_cuts, cuts_opt);
          const auto tc1 = std::chrono::steady_clock::now();

          RunRecord cuts_rec;
          cuts_rec.case_name    = tc.name;
          cuts_rec.solver_label = "Baseline+Cuts[S]";
          cuts_rec.n_vars       = baseline.n_vars;
          cuts_rec.n_ineq       = baseline.n_ineq;
          cuts_rec.n_eq         = baseline.n_eq;
          cuts_rec.success      = cuts_res.stats.success;
          cuts_rec.nodes        = cuts_res.bc_stats.nodes_explored;
          cuts_rec.lp_solves    = cuts_res.bc_stats.lp_solves;
          cuts_rec.objective    = cuts_res.stats.objective;
          cuts_rec.best_bound   = cuts_res.bc_stats.best_bound;
          cuts_rec.runtime_ms   = std::chrono::duration<double, std::milli>(tc1 - tc0).count();
          cuts_rec.status       = cuts_res.bc_stats.status;
          if (cuts_rec.success && std::isfinite(cuts_rec.objective)
                               && std::isfinite(cuts_rec.best_bound)) {
            cuts_rec.gap = std::abs(cuts_rec.objective - cuts_rec.best_bound)
                           / (1.0 + std::abs(cuts_rec.objective));
          }
          print_record(cuts_rec);
          all_records.push_back(cuts_rec);
          if (cuts_rec.success) ++solved;
        }



        // The warm-start solution is cold_res.x (used for configs 2 and 3)
        const Eigen::VectorXd* warm_x =
            cold_res.x.size() > 0 ? &cold_res.x : nullptr;

        // Step 2: I[IPM+xov] — IPM + crossover at root (reference; shows crossover cost)
        RunRecord ipm_xov = run_config(tc, kConfigs[1], nullptr, time_limit);
        print_record(ipm_xov);
        all_records.push_back(ipm_xov);
        if (ipm_xov.success) ++solved;

        // Step 3: I+A/B[-xov] — pure IPM + warm-start incumbent
        RunRecord ipm_ab = run_config(tc, kConfigs[2], warm_x, time_limit);
        print_record(ipm_ab);
        all_records.push_back(ipm_ab);
        if (ipm_ab.success) ++solved;

        // Step 4: NativeSeed[S] — HiGHS-IPM point + native crash basis seed,
        //                          HiGHS MIP runs dual-simplex from the seed
        RunRecord native_seed = run_config(tc, kConfigs[3], nullptr, time_limit);
        print_record(native_seed);
        all_records.push_back(native_seed);
        if (native_seed.success) ++solved;

        // Step 5: Seed+A/B[S] — native crash-basis seed plus incumbent warm start
        RunRecord native_seed_ab = run_config(tc, kConfigs[4], warm_x, time_limit);
        print_record(native_seed_ab);
        all_records.push_back(native_seed_ab);
        if (native_seed_ab.success) ++solved;

        // Gurobi global-optimum reference — verifies the problem formulation is
        // intact and provides a bound to compare native gaps against.
        // Guarded by availability so the benchmark still runs without a Gurobi
        // licence.  The solve is unconstrained (no time limit) so it may take
        // several minutes on the 118-bus case.
        {
          engine::GurobiAdapter gurobi;
          if (gurobi.available()) {
            engine::MIPModel mip_grb = build_scuc_mip(tc.inp);
            const auto tg0 = std::chrono::steady_clock::now();
            engine::SolveResult grb_res = gurobi.solve_milp(mip_grb);
            const auto tg1 = std::chrono::steady_clock::now();

            RunRecord grb_rec;
            grb_rec.case_name    = tc.name;
            grb_rec.solver_label = "Gurobi[opt]";
            grb_rec.n_vars       = baseline.n_vars;
            grb_rec.n_ineq       = baseline.n_ineq;
            grb_rec.n_eq         = baseline.n_eq;
            grb_rec.success      = grb_res.stats.success;
            grb_rec.objective    = grb_res.stats.objective;
            grb_rec.best_bound   = grb_res.stats.objective;
            grb_rec.gap          = grb_res.stats.mip_gap;
            grb_rec.runtime_ms   = std::chrono::duration<double, std::milli>(
                tg1 - tg0).count();
            grb_rec.status       = grb_res.stats.status;
            grb_rec.nodes        = 0;
            grb_rec.lp_solves    = 0;
            print_record(grb_rec);
            std::printf("  status: %s\n", grb_rec.status.c_str());
            std::fflush(stdout);
          }
        }

        std::printf("\n");
    }

    // ── Summary ───────────────────────────────────────────────────────────────

    const int total = static_cast<int>(all_records.size());
    std::printf("%d/%d runs solved to optimality (or within gap tolerance).\n",
                solved, total);

    // Timing summary:
    //   Stride 8 per case:
    //     [0] Baseline[S]      — all cuts (excl. FlowCover) / simplex root
    //     [1] NoCuts[S]        — no cuts: pure B&B lower-bound reference
    //     [2] FCOnly[S]        — FlowCover cuts only (native engine)
    //     [3] Baseline+Cuts[S] — all cuts + HiGHS root-cut warm-start
    //     [4] I[IPM+xov]       — IPM+crossover at root
    //     [5] I+A/B[-xov]      — pure IPM + warm-start incumbent
    //     [6] NativeSeed[S]    — native crash-basis seed + simplex
    //     [7] Seed+A/B[S]      — NativeSeed + warm-start incumbent
    std::printf("\nTiming summary (S=simplex; +xov=IPM+crossover; +AB=warm-start incumbent; Seed=HiGHS-IPM native basis; +Cuts=cut warm-start; FC=FlowCover):\n");
    std::printf("%-22s  %10s  %10s  %9s  %12s  %10s  %12s  %10s  %10s  %6s  %6s  %6s\n",
                "Case",
                "Base[S]_ms", "NoCuts_ms", "FC_ms", "+Cuts[S]_ms",
                "IPM+xov_ms", "IPM+xov+ABms",
                "Seed[S]_ms", "Seed+AB_ms",
                "Base_N", "FC_N", "Cuts_N");
    std::printf("%s\n", std::string(142, '-').c_str());
    for (std::size_t i = 0; i + 7 < all_records.size(); i += 8) {
        const auto& base   = all_records[i];
        const auto& nocuts = all_records[i + 1];
        const auto& fconly = all_records[i + 2];
        const auto& cuts   = all_records[i + 3];
        const auto& ipmx   = all_records[i + 4];
        const auto& ipmab  = all_records[i + 5];
        const auto& seed   = all_records[i + 6];
        const auto& seedab = all_records[i + 7];
        std::printf("%-22s  %10.1f  %10.1f  %9.1f  %12.1f  %10.1f  %12.1f  %10.1f  %10.1f  %6d  %6d  %6d\n",
                    base.case_name.c_str(),
                    base.runtime_ms, nocuts.runtime_ms, fconly.runtime_ms, cuts.runtime_ms,
                    ipmx.runtime_ms, ipmab.runtime_ms,
                    seed.runtime_ms, seedab.runtime_ms,
                    base.nodes, fconly.nodes, cuts.nodes);
    }

    // ── JSON output ───────────────────────────────────────────────────────────

    if (!json_path.empty()) {
        json jout = json::array();
        for (const auto& r : all_records)
            jout.push_back(record_to_json(r));

        std::ofstream f(json_path);
        if (f) {
            f << jout.dump(2) << "\n";
            std::printf("\nResults written to %s\n", json_path.c_str());
        } else {
            std::fprintf(stderr, "Cannot open %s for writing\n", json_path.c_str());
        }
    }

    return 0;
}
