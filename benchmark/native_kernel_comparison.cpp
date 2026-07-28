/// native_kernel_comparison.cpp
///
/// Pure-native vs HiGHS comparison driver.
///
/// Unlike milp_benchmark_runner.cpp / solver_comparison.cpp — which force
/// BCOptions::lp_kernel_backend = LpKernelBackend::HiGHS and therefore exercise
/// HiGHS LP numerics inside the native B&C — this driver benchmarks the
/// *fully native* numerical stack:
///
///   MILP level : native branch-and-cut with the native dual simplex as the
///                LP relaxation kernel (ExperimentalNative),
///                optionally with the native IPM at the root
///                vs. raw-API HiGHS MIP as the reference.
///   LP level   : native dual simplex (UMFPACK LU + eta updates, Ruiz
///                scaling) and native IPM-LP vs. HiGHS LP on
///                  - the 5 NETLIB problems with published optima
///                  - the LP relaxations of the SCUC cases (large/degenerate)
///                  - adversarially diagonally-scaled NETLIB problems
///                    (dynamic range 10^k) to probe numerical stability.
///
/// Reported per run: success/status, objective, relative objective error
/// (LP level, vs published optimum or HiGHS reference), runtime, iterations
/// (LP) / nodes+LP solves (MILP), and a solution audit (max row / bound /
/// integrality violation) computed independently from the returned x.
///
/// Usage:
///   native_kernel_comparison --smoke         # deterministic 6-bus CTest case
///   native_kernel_comparison                 # quick cases
///   native_kernel_comparison --full          # + 118-bus/24T MILP + SCUC LPs
///   native_kernel_comparison --time-limit 60 # per-solve wall clock cap
///   native_kernel_comparison --skip-milp     # LP-level only
///   native_kernel_comparison --skip-lp       # MILP level only

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Sparse>

#include "Highs.h"
#include "io/HMPSIO.h"  // vendored HiGHS MPS reader

#include "mipsolvers/engine/bc/api.hpp"
#include "mipsolvers/engine/bc/options.hpp"
#include "mipsolvers/engine/bc/stats.hpp"
#include "mipsolvers/engine/kernel/ipm/ipm_lp_solver.hpp"
#include "mipsolvers/engine/kernel/linear_algebra/hfactor_backend.hpp"
#include "mipsolvers/engine/kernel/lp_kernel/dual_simplex.hpp"
#include "mipsolvers/engine/problem_types.hpp"
#include "mipsolvers/scuc/case_builder.hpp"

using namespace mipsolvers;
using namespace mipsolvers::scuc;

namespace {

// ─────────────────────────────────────────────────────────────────────────────
// Shared helpers
// ─────────────────────────────────────────────────────────────────────────────

struct TestCase {
    std::string name;
    SCUCInput   inp;
};

struct MilpRow {
    std::string case_name;
    std::string solver;
    bool        success{false};
    int         nodes{0};
    int         lp_solves{0};
    double      objective{0.0};
    double      best_bound{0.0};
    double      gap{0.0};
    double      runtime_ms{0.0};
    double      max_row_viol{0.0};
    double      max_bound_viol{0.0};
    double      max_int_viol{0.0};
    std::string status;
};

struct LpRow {
    std::string case_name;
    std::string solver;
    bool        success{false};
    int         iterations{0};
    double      objective{0.0};
    double      obj_rel_err{-1.0};   // vs reference, -1 = unknown
    double      runtime_ms{0.0};
    double      max_row_viol{0.0};
    double      max_bound_viol{0.0};
    std::string status;
};

/// Independent feasibility audit of a solution vector against the model.
void audit_lp(const engine::LPModel& lp, const Eigen::VectorXd& x,
              double& max_row_viol, double& max_bound_viol) {
    max_row_viol = 0.0;
    max_bound_viol = 0.0;
    if (x.size() != static_cast<int>(lp.vars.size())) {
        max_row_viol = max_bound_viol = std::numeric_limits<double>::infinity();
        return;
    }
    const Eigen::VectorXd ax = lp.A * x;
    for (int i = 0; i < ax.size(); ++i) {
        max_row_viol = std::max(max_row_viol, std::max(0.0, ax[i] - lp.b[i]));
        const double lhs = engine::lp_row_lhs_or_neg_inf(lp, i);
        if (std::isfinite(lhs)) {
            max_row_viol = std::max(max_row_viol, std::max(0.0, lhs - ax[i]));
        }
    }
    const Eigen::VectorXd aeqx = lp.Aeq * x;
    for (int i = 0; i < aeqx.size(); ++i) {
        max_row_viol = std::max(max_row_viol, std::abs(aeqx[i] - lp.beq[i]));
    }
    for (int j = 0; j < x.size(); ++j) {
        const auto& v = lp.vars[static_cast<std::size_t>(j)];
        max_bound_viol = std::max(max_bound_viol,
            std::max(std::max(0.0, v.lb - x[j]), std::max(0.0, x[j] - v.ub)));
    }
}

void audit_milp(const engine::MIPModel& mip, const Eigen::VectorXd& x,
                double& max_row_viol, double& max_bound_viol,
                double& max_int_viol) {
    audit_lp(mip.linear_part, x, max_row_viol, max_bound_viol);
    max_int_viol = 0.0;
    if (x.size() != static_cast<int>(mip.linear_part.vars.size())) {
        max_int_viol = std::numeric_limits<double>::infinity();
        return;
    }
    for (int idx : mip.integer_idx) {
        if (idx >= 0 && idx < x.size()) {
            max_int_viol = std::max(max_int_viol,
                                    std::abs(x[idx] - std::round(x[idx])));
        }
    }
    for (int idx : mip.binary_idx) {
        if (idx >= 0 && idx < x.size()) {
            max_int_viol = std::max(max_int_viol,
                                    std::abs(x[idx] - std::round(x[idx])));
        }
    }
}

double rel_gap(double obj, double bound) {
    if (!std::isfinite(obj) || !std::isfinite(bound)) return 0.0;
    return std::abs(obj - bound) / (1.0 + std::abs(obj));
}

// ─────────────────────────────────────────────────────────────────────────────
// HiGHS raw-API runner (MIP when integrality given, otherwise pure LP)
// ─────────────────────────────────────────────────────────────────────────────

struct HighsRunOut {
    bool        success{false};
    double      objective{0.0};
    double      best_bound{0.0};
    double      mip_gap{0.0};
    double      runtime_ms{0.0};
    int         simplex_iterations{0};
    int         nodes{0};
    std::string status;
    Eigen::VectorXd x;
};

HighsRunOut run_highs_raw(const engine::MIPModel* mip, const engine::LPModel* lp_in,
                          double time_limit_sec) {
    HighsRunOut out;
    engine::LPModel lp = mip ? mip->linear_part : *lp_in;
    if (mip) {
        for (int idx : mip->integer_idx) {
            lp.vars[static_cast<std::size_t>(idx)].type = engine::VarType::Integer;
        }
        for (int idx : mip->binary_idx) {
            auto& var = lp.vars[static_cast<std::size_t>(idx)];
            var.type = engine::VarType::Binary;
            var.lb = std::max(0.0, var.lb);
            var.ub = std::min(1.0, var.ub);
        }
    }

    const int ncols  = static_cast<int>(lp.vars.size());
    const int m_ineq = static_cast<int>(lp.A.rows());
    const int m_eq   = static_cast<int>(lp.Aeq.rows());
    const int nrows  = m_ineq + m_eq;

    std::vector<double> col_cost(static_cast<std::size_t>(ncols), 0.0);
    std::vector<double> col_lower(static_cast<std::size_t>(ncols), -kHighsInf);
    std::vector<double> col_upper(static_cast<std::size_t>(ncols), kHighsInf);
    std::vector<HighsInt> integrality(static_cast<std::size_t>(ncols),
                                      static_cast<HighsInt>(HighsVarType::kContinuous));
    for (int j = 0; j < ncols; ++j) {
        const auto& var = lp.vars[static_cast<std::size_t>(j)];
        col_cost[static_cast<std::size_t>(j)]  = lp.c[j];
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
        row_lower[static_cast<std::size_t>(rr)] = lp.beq[r];
        row_upper[static_cast<std::size_t>(rr)] = lp.beq[r];
    }

    std::vector<HighsInt> start(static_cast<std::size_t>(ncols + 1), 0);
    std::vector<HighsInt> index;
    std::vector<double>   value;
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
    highs.setOptionValue("time_limit", std::max(0.001, time_limit_sec));
    if (mip) {
        highs.setOptionValue("mip_rel_gap", 1e-4);
    }

    const auto pass_status = highs.passModel(
        static_cast<HighsInt>(ncols), static_cast<HighsInt>(nrows),
        static_cast<HighsInt>(index.size()), static_cast<HighsInt>(MatrixFormat::kColwise),
        static_cast<HighsInt>(lp.sense == engine::Sense::Maximize ? ObjSense::kMaximize
                                                                  : ObjSense::kMinimize),
        0.0, col_cost.data(), col_lower.data(), col_upper.data(), row_lower.data(),
        row_upper.data(), start.data(), index.data(), value.data(), integrality.data());
    if (pass_status == HighsStatus::kError) {
        out.status = "PassModelError";
        out.runtime_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - t0).count();
        return out;
    }

    const auto run_status = highs.run();
    out.runtime_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t0).count();
    const HighsModelStatus ms = highs.getModelStatus();
    out.status = highs.modelStatusToString(ms);
    out.success = (run_status == HighsStatus::kOk)
                  && (ms == HighsModelStatus::kOptimal
                      || ms == HighsModelStatus::kTimeLimit
                      || ms == HighsModelStatus::kIterationLimit);
    const HighsInfo& info = highs.getInfo();
    out.objective  = info.objective_function_value;
    out.best_bound = info.mip_dual_bound;
    out.mip_gap    = info.mip_gap;
    out.simplex_iterations = static_cast<int>(info.simplex_iteration_count)
                             + static_cast<int>(info.ipm_iteration_count);
    out.nodes = static_cast<int>(info.mip_node_count);
    const HighsSolution& sol = highs.getSolution();
    out.x = Eigen::VectorXd::Map(sol.col_value.data(),
                                 static_cast<int>(sol.col_value.size()));
    return out;
}

// ─────────────────────────────────────────────────────────────────────────────
// MILP-level runners
// ─────────────────────────────────────────────────────────────────────────────

MilpRow run_native_bc(const TestCase& tc, bool ipm_root, double time_limit_sec) {
    MilpRow r;
    r.case_name = tc.name;
    r.solver    = ipm_root ? "NativeBC[natIPMroot]" : "NativeBC[natSimplex]";

    engine::MIPModel mip = build_scuc_mip(tc.inp);

    engine::BCOptions opt;
    opt.lp_kernel_backend = engine::LpKernelBackend::ExperimentalNative;
    opt.use_ipm_root                 = ipm_root;
    opt.time_limit_sec               = time_limit_sec;
    opt.gap_tol                      = 1e-3;
    opt.verbose                      = false;
    // Diagnostic overrides (development only):
    //   MIPSOLVERS_NKC_VERBOSE=1            -> BCOptions::verbose (LP-STATS etc.)
    //   MIPSOLVERS_NKC_REQUIRE_TREE_CERT=1  -> force real tree exploration
    //     (disables 0-node root-certificate optimality shortcuts)
    if (std::getenv("MIPSOLVERS_NKC_VERBOSE") != nullptr) opt.verbose = true;
    if (std::getenv("MIPSOLVERS_NKC_REQUIRE_TREE_CERT") != nullptr)
        opt.require_tree_exhaustion_certificate = true;

    const auto t0 = std::chrono::steady_clock::now();
    engine::BCResult res = engine::solve_milp_bc(mip, opt);
    const auto t1 = std::chrono::steady_clock::now();

    r.success    = res.stats.success;
    r.nodes      = res.bc_stats.nodes_explored;
    r.lp_solves  = res.bc_stats.lp_solves;
    r.objective  = res.stats.objective;
    r.best_bound = res.bc_stats.best_bound;
    r.runtime_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    r.status     = res.bc_stats.status.empty() ? res.stats.status : res.bc_stats.status;
    if (r.success) {
        r.gap = rel_gap(r.objective, r.best_bound);
        audit_milp(mip, res.x, r.max_row_viol, r.max_bound_viol, r.max_int_viol);
    }
    return r;
}

MilpRow run_highs_mip(const TestCase& tc, double time_limit_sec) {
    MilpRow r;
    r.case_name = tc.name;
    r.solver    = "HiGHS[direct]";

    engine::MIPModel mip = build_scuc_mip(tc.inp);
    HighsRunOut out = run_highs_raw(&mip, nullptr, time_limit_sec);

    r.success    = out.success;
    r.nodes      = out.nodes;
    r.lp_solves  = out.simplex_iterations;
    r.objective  = out.objective;
    r.best_bound = out.best_bound;
    r.runtime_ms = out.runtime_ms;
    r.status     = out.status;
    if (r.success) {
        r.gap = rel_gap(r.objective, r.best_bound);
        audit_milp(mip, out.x, r.max_row_viol, r.max_bound_viol, r.max_int_viol);
    }
    return r;
}

// ─────────────────────────────────────────────────────────────────────────────
// LP-level runners
// ─────────────────────────────────────────────────────────────────────────────

LpRow run_native_simplex_lp(const std::string& case_name, const engine::LPModel& lp,
                            double ref_obj) {
    LpRow r;
    r.case_name = case_name;
    r.solver    = "natDualSimplex";

    engine::SimplexOptions opts;
    opts.lp_kernel_backend = engine::LpKernelBackend::ExperimentalNative;
    opts.max_iter = 100000;
    // Adaptive HiGHS presolve is a fair production comparison: HiGHS-LP presolves
    // internally, so the native kernels get the same lever.  Primal + objective
    // are all this LP-relaxation row reports, so primal-only postsolve suffices.
    // Disable for A/B with MIPSOLVERS_PRESOLVE=0.
    opts.use_highs_presolve = true;
    // [ITER-TIMING] / [SIMPLEX-DIAG] on the pure-LP rows (same env as the
    // MILP-level BCOptions::verbose plumbing).
    if (std::getenv("MIPSOLVERS_NKC_VERBOSE") != nullptr) opts.verbose = true;

    const auto t0 = std::chrono::steady_clock::now();
    engine::SimplexResult res = engine::solve_lp_with_basis(lp, opts);
    const auto t1 = std::chrono::steady_clock::now();

    r.success    = res.result.stats.success;
    r.iterations = res.result.stats.iterations;
    r.objective  = res.result.stats.objective;
    r.runtime_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    r.status     = res.result.stats.status;
    if (r.success) {
        audit_lp(lp, res.result.x, r.max_row_viol, r.max_bound_viol);
        if (std::isfinite(ref_obj)) {
            r.obj_rel_err = std::abs(r.objective - ref_obj) / (1.0 + std::abs(ref_obj));
        }
    }
    return r;
}

LpRow run_native_ipm_lp(const std::string& case_name, const engine::LPModel& lp,
                        double ref_obj) {
    LpRow r;
    r.case_name = case_name;
    r.solver    = "natIPM";

    engine::IPMLPOptions ipmopt;
    ipmopt.use_highs_presolve = true;  // fair vs HiGHS-LP; see natDualSimplex note
    engine::NativeIPMLPAdapter ipm{ipmopt};
    const auto t0 = std::chrono::steady_clock::now();
    engine::SolveResult res = ipm.solve_lp(lp);
    const auto t1 = std::chrono::steady_clock::now();

    r.success    = res.stats.success;
    r.iterations = res.stats.iterations;
    r.objective  = res.stats.objective;
    r.runtime_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    r.status     = res.stats.status;
    if (r.success) {
        audit_lp(lp, res.x, r.max_row_viol, r.max_bound_viol);
        if (std::isfinite(ref_obj)) {
            r.obj_rel_err = std::abs(r.objective - ref_obj) / (1.0 + std::abs(ref_obj));
        }
    }
    return r;
}

LpRow run_highs_lp(const std::string& case_name, const engine::LPModel& lp,
                   double ref_obj, double time_limit_sec) {
    LpRow r;
    r.case_name = case_name;
    r.solver    = "HiGHS-LP";

    HighsRunOut out = run_highs_raw(nullptr, &lp, time_limit_sec);
    r.success    = out.success;
    r.iterations = out.simplex_iterations;
    r.objective  = out.objective;
    r.runtime_ms = out.runtime_ms;
    r.status     = out.status;
    if (r.success) {
        audit_lp(lp, out.x, r.max_row_viol, r.max_bound_viol);
        if (std::isfinite(ref_obj)) {
            r.obj_rel_err = std::abs(r.objective - ref_obj) / (1.0 + std::abs(ref_obj));
        }
    }
    return r;
}

// ─────────────────────────────────────────────────────────────────────────────
// NETLIB MPS reading (mirrors tests/test_netlib_regression.cpp)
// ─────────────────────────────────────────────────────────────────────────────

std::string netlib_path(const std::string& name) {
    for (const char* prefix : {"tests/data/netlib/", "../tests/data/netlib/"}) {
        const std::string p = std::string(prefix) + name + ".mps";
        if (FILE* f = std::fopen(p.c_str(), "r")) {
            std::fclose(f);
            return p;
        }
    }
    return "tests/data/netlib/" + name + ".mps";
}

bool read_mps_as_lp(const std::string& path, engine::LPModel& lp) {
    bool output_flag = false;
    bool log_to_console = false;
    HighsInt log_dev_level = kHighsLogDevLevelNone;
    HighsLogOptions log_options;
    log_options.output_flag = &output_flag;
    log_options.log_to_console = &log_to_console;
    log_options.log_dev_level = &log_dev_level;
    HighsInt num_row = 0, num_col = 0;
    ObjSense sense = ObjSense::kMinimize;
    double offset = 0.0;
    std::vector<HighsInt> Astart, Aindex;
    std::vector<double> Avalue, colCost, colLower, colUpper, rowLower, rowUpper;
    std::vector<HighsVarType> integer;
    std::string objective_name;
    std::vector<std::string> col_names, row_names;
    HighsInt Qdim = 0, cost_row_location = 0;
    std::vector<HighsInt> Qstart, Qindex;
    std::vector<double> Qvalue;
    bool warning_issued = false;
    const FilereaderRetcode rc = readMps(
        log_options, path, -1, -1, num_row, num_col, sense, offset, Astart,
        Aindex, Avalue, colCost, colLower, colUpper, rowLower, rowUpper, integer,
        objective_name, col_names, row_names, Qdim, Qstart, Qindex, Qvalue,
        cost_row_location, warning_issued);
    if (rc != FilereaderRetcode::kOk) return false;

    lp.sense = (sense == ObjSense::kMaximize) ? engine::Sense::Maximize
                                              : engine::Sense::Minimize;
    const int n = static_cast<int>(num_col);
    const int m = static_cast<int>(num_row);
    lp.c = Eigen::VectorXd::Map(colCost.data(), n);
    lp.vars.resize(static_cast<std::size_t>(n));
    for (int j = 0; j < n; ++j) {
        lp.vars[static_cast<std::size_t>(j)].type = engine::VarType::Continuous;
        const double lo = colLower[static_cast<std::size_t>(j)];
        const double hi = colUpper[static_cast<std::size_t>(j)];
        lp.vars[static_cast<std::size_t>(j)].lb = std::isfinite(lo) ? lo : -1e20;
        lp.vars[static_cast<std::size_t>(j)].ub = std::isfinite(hi) ? hi : 1e20;
    }

    std::vector<int> ineq_row(static_cast<std::size_t>(m), -1);
    std::vector<int> eq_row(static_cast<std::size_t>(m), -1);
    int n_ineq = 0, n_eq = 0;
    for (int i = 0; i < m; ++i) {
        if (rowLower[static_cast<std::size_t>(i)] == rowUpper[static_cast<std::size_t>(i)]) {
            eq_row[static_cast<std::size_t>(i)] = n_eq++;
        } else {
            ineq_row[static_cast<std::size_t>(i)] = n_ineq++;
        }
    }
    std::vector<Eigen::Triplet<double>> trips, trips_eq;
    std::vector<double> b(static_cast<std::size_t>(n_ineq)),
        beq(static_cast<std::size_t>(n_eq)), lhs(static_cast<std::size_t>(n_ineq));
    for (int i = 0; i < m; ++i) {
        if (eq_row[static_cast<std::size_t>(i)] >= 0) {
            beq[static_cast<std::size_t>(eq_row[static_cast<std::size_t>(i)])] =
                rowLower[static_cast<std::size_t>(i)];
        } else {
            b[static_cast<std::size_t>(ineq_row[static_cast<std::size_t>(i)])] =
                rowUpper[static_cast<std::size_t>(i)];
            lhs[static_cast<std::size_t>(ineq_row[static_cast<std::size_t>(i)])] =
                rowLower[static_cast<std::size_t>(i)];
        }
    }
    for (int j = 0; j < n; ++j) {
        for (HighsInt p = Astart[j]; p < Astart[j + 1]; ++p) {
            const int i = static_cast<int>(Aindex[static_cast<std::size_t>(p)]);
            if (eq_row[static_cast<std::size_t>(i)] >= 0) {
                trips_eq.emplace_back(eq_row[static_cast<std::size_t>(i)], j,
                                      Avalue[static_cast<std::size_t>(p)]);
            } else {
                trips.emplace_back(ineq_row[static_cast<std::size_t>(i)], j,
                                   Avalue[static_cast<std::size_t>(p)]);
            }
        }
    }
    lp.A.resize(n_ineq, n);
    lp.A.setFromTriplets(trips.begin(), trips.end());
    lp.b = Eigen::VectorXd::Map(b.data(), n_ineq);
    lp.row_lhs = Eigen::VectorXd::Map(lhs.data(), n_ineq);
    lp.Aeq.resize(n_eq, n);
    lp.Aeq.setFromTriplets(trips_eq.begin(), trips_eq.end());
    lp.beq = Eigen::VectorXd::Map(beq.data(), n_eq);
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Adversarial diagonal scaling: A' = Dr * A * Dc, b' = Dr*b, c' = Dc*c,
// bounds divided by Dc.  Optimal objective is invariant; solver numerics
// are stressed by a dynamic range of ~10^(2*K).
// ─────────────────────────────────────────────────────────────────────────────

engine::LPModel scale_lp(const engine::LPModel& lp, double max_log10,
                         unsigned seed) {
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> dist(-max_log10, max_log10);

    const int m_ineq = static_cast<int>(lp.A.rows());
    const int m_eq   = static_cast<int>(lp.Aeq.rows());
    const int n      = static_cast<int>(lp.vars.size());

    Eigen::VectorXd dr_ineq(m_ineq), dr_eq(m_eq), dc(n);
    for (int i = 0; i < m_ineq; ++i) dr_ineq[i] = std::pow(10.0, dist(rng));
    for (int i = 0; i < m_eq;   ++i) dr_eq[i]   = std::pow(10.0, dist(rng));
    for (int j = 0; j < n;      ++j) dc[j]      = std::pow(10.0, dist(rng));

    engine::LPModel out = lp;
    out.A    = dr_ineq.asDiagonal() * lp.A * dc.asDiagonal();
    out.Aeq  = dr_eq.asDiagonal() * lp.Aeq * dc.asDiagonal();
    out.b    = dr_ineq.asDiagonal() * lp.b;
    out.beq  = dr_eq.asDiagonal() * lp.beq;
    if (lp.row_lhs.size() == m_ineq) {
        out.row_lhs = dr_ineq.asDiagonal() * lp.row_lhs;
    }
    out.c = dc.asDiagonal() * lp.c;
    for (int j = 0; j < n; ++j) {
        auto& v = out.vars[static_cast<std::size_t>(j)];
        const double d = dc[j];
        if (d > 0.0) {
            v.lb = v.lb / d;
            v.ub = v.ub / d;
        }
    }
    return out;
}

// ─────────────────────────────────────────────────────────────────────────────
// Reporting
// ─────────────────────────────────────────────────────────────────────────────

void print_milp_table(const std::vector<MilpRow>& rows) {
    std::printf("\n=== MILP level: native B&C (native LP kernel) vs HiGHS ===\n");
    std::printf("%-18s %-22s %9s %7s %7s %14s %14s %8s %9s %9s %9s %s\n",
                "Case", "Solver", "Time[ms]", "Nodes", "LPs", "Objective",
                "BestBound", "Gap[%]", "RowViol", "BndViol", "IntViol", "Status");
    std::printf("%s\n", std::string(150, '-').c_str());
    std::string current;
    for (const auto& r : rows) {
        if (r.case_name != current) {
            if (!current.empty()) std::printf("\n");
            current = r.case_name;
        }
        std::printf("%-18s %-22s %9.1f %7d %7d %14.4f %14.4f %8.3f %9.2e %9.2e %9.2e %s%s\n",
                    r.case_name.c_str(), r.solver.c_str(), r.runtime_ms, r.nodes,
                    r.lp_solves, r.objective, r.best_bound, 100.0 * r.gap,
                    r.max_row_viol, r.max_bound_viol, r.max_int_viol,
                    r.success ? "" : "FAIL:", r.status.c_str());
    }
}

void print_lp_table(const char* title, const std::vector<LpRow>& rows) {
    std::printf("\n=== %s ===\n", title);
    std::printf("%-22s %-15s %9s %8s %16s %11s %10s %10s %s\n",
                "Case", "Solver", "Time[ms]", "Iters", "Objective",
                "ObjRelErr", "RowViol", "BndViol", "Status");
    std::printf("%s\n", std::string(125, '-').c_str());
    std::string current;
    for (const auto& r : rows) {
        if (r.case_name != current) {
            if (!current.empty()) std::printf("\n");
            current = r.case_name;
        }
        std::printf("%-22s %-15s %9.1f %8d %16.6e %11.3e %10.2e %10.2e %s%s\n",
                    r.case_name.c_str(), r.solver.c_str(), r.runtime_ms,
                    r.iterations, r.objective, r.obj_rel_err,
                    r.max_row_viol, r.max_bound_viol,
                    r.success ? "" : "FAIL:", r.status.c_str());
    }
}

}  // namespace

// ── Warm-start (incremental node-LP) probe ──────────────────────────────────
// Mirrors exactly what a B&C tree node does: one shared Ruiz-scaled standard
// form, bound-only updates via update_standard_form_bounds, warm re-solves
// with the parent basis hint and allow_cold_start=false.  A working
// incremental solver should re-optimize each child in a handful of pivots and
// milliseconds; if every probe falls back to a cold solve (or burns thousands
// of iterations before failing), the warm-start path is broken, not the model.
namespace {
int run_warm_probe(bool use_118) {
    SCUCInput inp = use_118 ? build_ieee118_case(/*T=*/24)
                            : build_ieee39_case(/*T=*/24);
    inp.config.solve_sced = false;
    inp.config.solve_lmp  = false;
    engine::MIPModel mip = build_scuc_mip(inp);
    engine::LPModel lp = mip.linear_part;
    const int n = static_cast<int>(lp.vars.size());
    Eigen::VectorXd lb0(n), ub0(n);
    for (int j = 0; j < n; ++j) {
        lb0[j] = lp.vars[static_cast<size_t>(j)].lb;
        ub0[j] = lp.vars[static_cast<size_t>(j)].ub;
    }

    engine::StandardFormLP sf = engine::build_standard_form_lp(lp);
    engine::ruiz_scale_standard_form(sf);

    // ── HFactor backend self-test (MIPSOLVERS_HFACTOR_SELFTEST=1) ────────
    // Isolates the vendored-HFactor port from the simplex driver: factorize
    // the crash basis (slack-or-artificial per row) and check FTRAN/BTRAN
    // against an independent Eigen SparseLU reference.
    if (std::getenv("MIPSOLVERS_HFACTOR_SELFTEST") != nullptr) {
        const int m = static_cast<int>(sf.A.rows());
        const int n = static_cast<int>(sf.A.cols());
        std::vector<int> basis(m, -1);
        for (int i = 0; i < m; ++i)
            basis[i] = (sf.row_to_slack_col[i] >= 0) ? sf.row_to_slack_col[i]
                                                     : sf.row_to_artificial_col[i];
        int missing = 0;
        for (int i = 0; i < m; ++i) if (basis[i] < 0) ++missing;
        engine::HFactorBackend hfb;
        const auto tf0 = std::chrono::steady_clock::now();
        const bool ok = hfb.factorize(sf.A, basis.data(), m);
        const double fms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - tf0).count();
        std::printf("hfactor selftest: m=%d n=%d missing_basis_cols=%d "
                    "factorize ok=%d rank_def=%d time=%.1f ms\n",
                    m, n, missing, ok ? 1 : 0, hfb.rank_deficiency, fms);
        if (ok) {
            Eigen::VectorXd x(m), r(m);
            hfb.ftran(sf.b.data(), x.data());
            r = -sf.b;
            for (int i = 0; i < m; ++i)
                for (Eigen::SparseMatrix<double>::InnerIterator it(sf.A, basis[i]); it; ++it)
                    r[it.row()] += it.value() * x[i];
            std::printf("hfactor selftest: ftran residual inf-norm=%.3e\n",
                        r.cwiseAbs().maxCoeff());
            // Reference factorization (Eigen SparseLU).
            Eigen::SparseMatrix<double> B(m, m);
            std::vector<Eigen::Triplet<double>> trips;
            for (int i = 0; i < m; ++i)
                for (Eigen::SparseMatrix<double>::InnerIterator it(sf.A, basis[i]); it; ++it)
                    trips.emplace_back(it.row(), i, it.value());
            B.setFromTriplets(trips.begin(), trips.end());
            Eigen::SparseLU<Eigen::SparseMatrix<double>> lu;
            lu.analyzePattern(B);
            lu.factorize(B);
            std::printf("hfactor selftest: eigen lu %s\n",
                        lu.info() == Eigen::Success ? "ok" : "FAILED");
            if (lu.info() == Eigen::Success) {
                const Eigen::VectorXd xr = lu.solve(sf.b);
                std::printf("hfactor selftest: |x_hfactor-x_eigen|inf=%.3e\n",
                            (x - xr).cwiseAbs().maxCoeff());
                // BTRAN check: y = B^{-T} c_b on basic costs.
                Eigen::VectorXd cb(m), y(m), yr(m);
                for (int i = 0; i < m; ++i) cb[i] = sf.c_max[basis[i]];
                hfb.btran(cb.data(), y.data());
                yr = lu.transpose().solve(cb);
                std::printf("hfactor selftest: |y_hfactor-y_eigen|inf=%.3e\n",
                            (y - yr).cwiseAbs().maxCoeff());
            }
            // ── Update (Forrest-Tomlin) stress test ──────────────────────
            // Simulate simplex pivots: enter a non-basic column, leave at a
            // row with a safely nonzero pivot element, then periodically
            // compare FTRAN results against a fresh Eigen LU of the current
            // basis.  This exercises exactly the path the dual simplex uses.
            {
                std::vector<char> in_basis(n, 0);
                for (int i = 0; i < m; ++i) in_basis[basis[i]] = 1;
                std::vector<int> nonbasic;
                for (int j = 0; j < n && static_cast<int>(nonbasic.size()) < 400; ++j)
                    if (!in_basis[j]) nonbasic.push_back(j);
                Eigen::VectorXd aq(m), ep(m), e_row = Eigen::VectorXd::Zero(m);
                Eigen::SparseLU<Eigen::SparseMatrix<double>> lu_ref;
                bool lu_ref_ok = false;
                {
                    Eigen::SparseMatrix<double> B0(m, m);
                    std::vector<Eigen::Triplet<double>> tr0;
                    for (int i = 0; i < m; ++i)
                        for (Eigen::SparseMatrix<double>::InnerIterator it(sf.A, basis[i]); it; ++it)
                            tr0.emplace_back(it.row(), i, it.value());
                    B0.setFromTriplets(tr0.begin(), tr0.end());
                    lu_ref.analyzePattern(B0);
                    lu_ref.factorize(B0);
                    lu_ref_ok = (lu_ref.info() == Eigen::Success);
                }
                int done = 0, bad_update = 0;
                double worst_resid = 0.0;
                for (int t = 0; t < 50 && t < static_cast<int>(nonbasic.size()); ++t) {
                    const int q = nonbasic[static_cast<size_t>(t)];
                    // aq = B^{-1} a_q
                    Eigen::VectorXd col = Eigen::VectorXd::Zero(m);
                    for (Eigen::SparseMatrix<double>::InnerIterator it(sf.A, q); it; ++it)
                        col[it.row()] = it.value();
                    hfb.ftran_for_update(col.data(), aq.data());
                    // Verify the input aq against the reference factor BEFORE
                    // applying the update (catches ftran-after-update errors
                    // that the b-RHS check is blind to).
                    if (lu_ref_ok) {
                        const Eigen::VectorXd aq_ref = lu_ref.solve(col);
                        const double aq_err = (aq - aq_ref).cwiseAbs().maxCoeff();
                        if (aq_err > 1e-8) {
                            std::printf("hfactor selftest: BEFORE update %d ftran(a_q) WRONG err=%.3e (q=%d)\n",
                                        done + 1, aq_err, q);
                            break;
                        }
                    }
                    const bool ref_inputs =
                        std::getenv("MIPSOLVERS_HFACTOR_SELFTEST_REFINPUTS") != nullptr;
                    if (ref_inputs && lu_ref_ok) aq = lu_ref.solve(col);
                    int p = -1;
                    double best = 1e-6;
                    for (int i = 0; i < m; ++i)
                        if (std::fabs(aq[i]) > best) { best = std::fabs(aq[i]); p = i; }
                    if (p < 0) continue;  // column ~dependent on current basis
                    // ep = B^{-T} e_p
                    e_row.setZero();
                    e_row[p] = 1.0;
                    hfb.btran_for_update(e_row.data(), ep.data());
                    if (ref_inputs && lu_ref_ok) ep = lu_ref.transpose().solve(e_row);
                    const bool refact_mode =
                        std::getenv("MIPSOLVERS_HFACTOR_SELFTEST_REFACT") != nullptr;
                    if (refact_mode) {
                        // Control: fresh factorization instead of FT update.
                        basis[p] = q;
                        if (!hfb.factorize(sf.A, basis.data(), m)) { ++bad_update; break; }
                    } else {
                        if (!hfb.update(p, q, aq.data(), ep.data())) { ++bad_update; break; }
                        basis[p] = q;
                    }
                    in_basis[q] = 1;
                    ++done;
                    {
                        // Fresh reference factorization of the current basis.
                        Eigen::SparseMatrix<double> B2(m, m);
                        std::vector<Eigen::Triplet<double>> tr2;
                        for (int i = 0; i < m; ++i)
                            for (Eigen::SparseMatrix<double>::InnerIterator it(sf.A, basis[i]); it; ++it)
                                tr2.emplace_back(it.row(), i, it.value());
                        B2.setFromTriplets(tr2.begin(), tr2.end());
                        Eigen::SparseLU<Eigen::SparseMatrix<double>> lu2;
                        lu2.analyzePattern(B2);
                        lu2.factorize(B2);
                        if (lu2.info() != Eigen::Success) {
                            std::printf("hfactor selftest: update %d made basis SINGULAR\n", done);
                            break;
                        }
                        const Eigen::VectorXd xr2 = lu2.solve(sf.b);
                        Eigen::VectorXd x2(m);
                        hfb.ftran(sf.b.data(), x2.data());
                        const double res = (x2 - xr2).cwiseAbs().maxCoeff();
                        // BTRAN check too: y = B^{-T} c_b against reference.
                        Eigen::VectorXd cb2(m), y2(m);
                        for (int i = 0; i < m; ++i) cb2[i] = sf.c_max[basis[i]];
                        hfb.btran(cb2.data(), y2.data());
                        const Eigen::VectorXd yr2 = lu2.transpose().solve(cb2);
                        const double res_b = (y2 - yr2).cwiseAbs().maxCoeff();
                        worst_resid = std::max(worst_resid, std::max(res, res_b));
                        if (done <= 6 || res > 1e-6 || res_b > 1e-6)
                            std::printf("hfactor selftest: update %d ftran_resid=%.3e btran_resid=%.3e (p=%d q=%d piv=%.3e)\n",
                                        done, res, res_b, p, q, best);
                        if (res > 1e-6 || res_b > 1e-6) {
                            std::printf("hfactor selftest: update %d DIVERGED ftran=%.3e btran=%.3e\n",
                                        done, res, res_b);
                            // Top error components of the failing b-solve.
                            {
                                int n_nan = 0, n_inf = 0;
                                double max_finite = 0.0;
                                int max_finite_row = -1, first_nan_row = -1;
                                for (int i = 0; i < m; ++i) {
                                    const double dv = std::fabs(x2[i] - xr2[i]);
                                    if (std::isnan(x2[i])) { ++n_nan; if (first_nan_row < 0) first_nan_row = i; }
                                    else if (std::isinf(x2[i])) ++n_inf;
                                    else if (dv > max_finite) { max_finite = dv; max_finite_row = i; }
                                }
                                std::printf("  x2: nan=%d inf=%d max_finite_err=%.3e at row %d first_nan_row=%d\n",
                                            n_nan, n_inf, max_finite, max_finite_row, first_nan_row);
                                int n_bad = 0;
                                for (int i = 0; i < m; ++i)
                                    if (std::fabs(x2[i] - xr2[i]) > 1e-9) ++n_bad;
                                std::printf("  x2: rows with err>1e-9: %d / %d\n", n_bad, m);
                                if (max_finite_row >= 0) {
                                    Eigen::VectorXd e_w = Eigen::VectorXd::Zero(m);
                                    e_w[max_finite_row] = 1.0;
                                    Eigen::VectorXd xh2(m);
                                    hfb.ftran(e_w.data(), xh2.data());
                                    const Eigen::VectorXd xe2 = lu2.solve(e_w);
                                    const Eigen::VectorXd dv2 = (xh2 - xe2).cwiseAbs();
                                    Eigen::Index wi2;
                                    const double we2 = dv2.maxCoeff(&wi2);
                                    std::printf("  B^-1 e_%d: worst=%.3e at row %d\n",
                                                max_finite_row, we2, static_cast<int>(wi2));
                                }
                            }
                            // Localize: columns of B^{-1} via unit-vector
                            // solves; report the worst components.
                            for (int probe = 0; probe < 8; ++probe) {
                                const int j = (probe * 1447 + 1917) % m;
                                Eigen::VectorXd e_j = Eigen::VectorXd::Zero(m);
                                e_j[j] = 1.0;
                                Eigen::VectorXd xh(m);
                                hfb.ftran(e_j.data(), xh.data());
                                const Eigen::VectorXd xe = lu2.solve(e_j);
                                const Eigen::VectorXd dvec = (xh - xe).cwiseAbs();
                                Eigen::Index widx;
                                const double werr = dvec.maxCoeff(&widx);
                                std::printf("  B^-1 e_%d: worst err=%.3e at row %d (xh=%.6g xe=%.6g)\n",
                                            j, werr, static_cast<int>(widx),
                                            xh[widx], xe[widx]);
                            }
                            break;
                        }
                        lu_ref.analyzePattern(B2);
                        lu_ref.factorize(B2);
                        lu_ref_ok = (lu_ref.info() == Eigen::Success);
                    }
                }
                std::printf("hfactor selftest: updates done=%d bad=%d worst_resid=%.3e\n",
                            done, bad_update, worst_resid);
            }
        }
        return ok ? 0 : 1;
    }

    engine::SimplexOptions cold_opt;
    cold_opt.lp_kernel_backend = engine::LpKernelBackend::ExperimentalNative;
    cold_opt.max_iter = 100000;
    const auto t0 = std::chrono::steady_clock::now();
    engine::SimplexResult r0 = engine::solve_lp_from_sf(sf, cold_opt);
    const double cold_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t0).count();
    std::printf("cold root: success=%d status=%s obj=%.4f iters=%d time=%.0f ms\n",
                r0.result.stats.success ? 1 : 0, r0.result.stats.status.c_str(),
                r0.result.stats.objective, r0.result.stats.iterations, cold_ms);
    if (!r0.result.stats.success) return 1;

    // One branch per probe, always starting from the ROOT basis (B&C child
    // semantics: parent bounds + a single branching decision).  Each fixed LP
    // is cross-checked against HiGHS to distinguish genuine infeasibility
    // from a false warm-start certificate.
    std::printf("%-4s %-7s %-8s %-6s %-14s %-8s %-10s %-8s %s\n", "k", "x_root",
                "fix", "succ", "objective", "iters", "time[ms]", "hint",
                "HiGHS(status/obj)");
    const auto& bins = mip.binary_idx;
    for (int k = 0; k < 8 && k < static_cast<int>(bins.size()); ++k) {
        const int j = bins[static_cast<size_t>(k)];
        const double xr = r0.result.x[j];
        // Branch AWAY from the relaxation value (the "hard child"): this
        // genuinely changes the LP and exercises the dual warm start.
        const double v = (xr >= 0.5) ? 0.0 : 1.0;
        Eigen::VectorXd lb = lb0, ub = ub0;
        lb[j] = v;
        ub[j] = v;
        engine::StandardFormLP sf_k = sf;
        engine::update_standard_form_bounds(sf_k, lp, lb, ub);
        engine::SimplexOptions node_opt;
        node_opt.lp_kernel_backend = engine::LpKernelBackend::ExperimentalNative;
        node_opt.max_iter = 100000;
        node_opt.allow_cold_start = false;  // tree-node semantics: warm or fail
        const auto t1 = std::chrono::steady_clock::now();
        engine::SimplexResult rk =
            engine::solve_lp_from_sf(sf_k, node_opt, &r0.basis);
        const double ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - t1).count();
        std::string highs_info = "-";
        if (!use_118) {
            engine::LPModel lp_fixed = lp;
            lp_fixed.vars[static_cast<size_t>(j)].lb = v;
            lp_fixed.vars[static_cast<size_t>(j)].ub = v;
            HighsRunOut hi = run_highs_raw(nullptr, &lp_fixed, 60.0);
            char buf[96];
            std::snprintf(buf, sizeof(buf), "%s/%.2f", hi.status.c_str(),
                          hi.objective);
            highs_info = buf;
        }
        std::printf("%-4d %-7.3f %-8.0f %-6d %-14.4f %-8d %-10.1f %-8s %s [%s]\n",
                    k, xr, v, rk.result.stats.success ? 1 : 0,
                    rk.result.stats.objective, rk.result.stats.iterations, ms,
                    rk.solved_from_hint ? "yes" : "no", highs_info.c_str(),
                    rk.result.stats.status.c_str());
    }

    // ── Tree-chain probe (mirrors descending a B&C path) ─────────────────
    // Cumulative away-fixings, warm from the PREVIOUS node's basis — the
    // exact tree pattern (parent basis + accumulated domain), as opposed to
    // the single-fixing probes above which always start from the root basis.
    // In the real 118-bus tree 8/10 PATH-A warm attempts fail; if this chain
    // reproduces the failures, the bug is in the warm machinery itself, not
    // in tree bookkeeping.
    {
        std::printf("tree-chain (cumulative fixings, parent-basis hint):\n");
        std::printf("%-4s %-6s %-14s %-8s %-10s %-8s %s\n", "depth", "succ",
                    "objective", "iters", "time[ms]", "hint", "status");
        Eigen::VectorXd lb = lb0, ub = ub0;
        engine::StandardFormLP sf_chain = sf;
        const engine::SimplexBasis* hint = &r0.basis;
        engine::SimplexResult prev = r0;  // only basis/result carriers are used
        for (int d = 1; d <= 6 && d <= static_cast<int>(bins.size()); ++d) {
            const int j = bins[static_cast<size_t>(d - 1)];
            const double xr = r0.result.x[j];
            const double v = (xr >= 0.5) ? 0.0 : 1.0;
            lb[j] = v;
            ub[j] = v;
            engine::update_standard_form_bounds(sf_chain, lp, lb, ub);
            engine::SimplexOptions chain_opt;
            chain_opt.lp_kernel_backend = engine::LpKernelBackend::ExperimentalNative;
            chain_opt.max_iter = 100000;
            chain_opt.allow_cold_start = false;  // tree-node semantics
            const auto t1 = std::chrono::steady_clock::now();
            engine::SimplexResult rd =
                engine::solve_lp_from_sf(sf_chain, chain_opt, hint);
            const double ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - t1).count();
            std::printf("%-4d %-6d %-14.4f %-8d %-10.1f %-8s [%s]\n", d,
                        rd.result.stats.success ? 1 : 0,
                        rd.result.stats.objective, rd.result.stats.iterations,
                        ms, rd.solved_from_hint ? "yes" : "no",
                        rd.result.stats.status.c_str());
            if (!rd.result.stats.success) {
                // Ground-truth cross-check: a false infeasibility/failure at
                // a tree node prunes a valid subtree — verify against HiGHS
                // on the identically-fixed LP before trusting the verdict.
                engine::LPModel lp_fixed = lp;
                for (int jj = 0; jj < n; ++jj) {
                    lp_fixed.vars[static_cast<size_t>(jj)].lb = lb[jj];
                    lp_fixed.vars[static_cast<size_t>(jj)].ub = ub[jj];
                }
                HighsRunOut hi = run_highs_raw(nullptr, &lp_fixed, 600.0);
                std::printf("  -> HiGHS ground truth: %s obj=%.4f (%s)\n",
                            hi.status.c_str(), hi.objective,
                            (hi.status == "Infeasible") ? "verdict CONFIRMED"
                                                        : "FALSE CERTIFICATE!");
                break;
            }
            prev = rd;
            hint = &prev.basis;
        }
    }

    // ── Cut-appended warm re-solve (mirrors B&C cut rounds) ──────────────
    // Append violated cover cuts on fractional root binaries to the SF and
    // warm re-solve with the root basis — the exact pattern of a cut
    // re-solve in the tree (hint must be projected across the added rows).
    {
        std::vector<int> frac;
        for (int idx : bins) {
            const double x = r0.result.x[idx];
            if (x > 1e-3 && x < 1.0 - 1e-3) frac.push_back(idx);
        }
        std::printf("fractional root binaries: %zu\n", frac.size());
        for (int round = 0; round < 3 && frac.size() >= 4; ++round) {
            const int take = std::min<int>(8, static_cast<int>(frac.size()));
            Eigen::SparseVector<double> cut(n);
            double act = 0.0;
            for (int t = 0; t < take; ++t) {
                const int j = frac[static_cast<size_t>(round * 3 + t) %
                                     frac.size()];
                cut.coeffRef(j) = 1.0;
                act += r0.result.x[j];
            }
            const double rhs = std::max(1.0, std::floor(act - 0.5));
            engine::StandardFormLP sf_cut;
            std::vector<Eigen::SparseVector<double>> rows{cut};
            if (!engine::append_leq_rows_to_standard_form(sf, rows, {rhs},
                                                          sf_cut)) {
                std::printf("cut round %d: append failed\n", round);
                break;
            }
            engine::SimplexOptions cut_opt;
            cut_opt.lp_kernel_backend = engine::LpKernelBackend::ExperimentalNative;
            cut_opt.max_iter = 100000;
            cut_opt.allow_cold_start = false;  // cut re-solve: warm or fail
            const auto tc = std::chrono::steady_clock::now();
            engine::SimplexResult rc =
                engine::solve_lp_from_sf(sf_cut, cut_opt, &r0.basis);
            const double ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - tc).count();
            std::printf("cut round %d: succ=%d obj=%.4f iters=%d time=%.1f ms "
                        "hint=%s [%s]\n",
                        round, rc.result.stats.success ? 1 : 0,
                        rc.result.stats.objective, rc.result.stats.iterations,
                        ms, rc.solved_from_hint ? "yes" : "no",
                        rc.result.stats.status.c_str());
        }
    }
    return 0;
}
}  // namespace

int main(int argc, char** argv) {
    bool   full_mode      = false;
    bool   smoke_mode     = false;
    bool   skip_milp      = false;
    bool   skip_lp        = false;
    bool   check_mode     = false;
    bool   warm_probe     = false;
    bool   warm_probe_39  = false;
    int    lp_native_only = 0;  // 118 or 39: run just that SCUC relaxation
                                // through natDualSimplex (fast profiling loop)
    double time_limit_sec = 120.0;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--smoke") == 0) {
            smoke_mode = true;
        } else if (std::strcmp(argv[i], "--full") == 0) {
            full_mode = true;
        } else if (std::strcmp(argv[i], "--skip-milp") == 0) {
            skip_milp = true;
        } else if (std::strcmp(argv[i], "--skip-lp") == 0) {
            skip_lp = true;
        } else if (std::strcmp(argv[i], "--check") == 0) {
            check_mode = true;
        } else if (std::strcmp(argv[i], "--warm-probe") == 0) {
            warm_probe = true;
        } else if (std::strcmp(argv[i], "--warm-probe-39") == 0) {
            warm_probe_39 = true;
        } else if (std::strcmp(argv[i], "--lp118-native") == 0) {
            lp_native_only = 118;
        } else if (std::strcmp(argv[i], "--lp39-native") == 0) {
            lp_native_only = 39;
        } else if (std::strcmp(argv[i], "--time-limit") == 0 && i + 1 < argc) {
            time_limit_sec = std::atof(argv[++i]);
        }
    }
    if (warm_probe) return run_warm_probe(true);
    if (warm_probe_39) return run_warm_probe(false);

    if (lp_native_only != 0) {
        SCUCInput inp = (lp_native_only == 118) ? build_ieee118_case(/*T=*/24)
                                                : build_ieee39_case(/*T=*/24);
        inp.config.solve_sced = false;
        inp.config.solve_lmp  = false;
        engine::MIPModel mip = build_scuc_mip(inp);
        engine::LPModel lp = mip.linear_part;  // integrality dropped
        const char* label = (lp_native_only == 118) ? "UC_118bus_24T-relax"
                                                    : "UC_39bus_24T-relax";
        LpRow r = run_native_simplex_lp(
            label, lp, std::numeric_limits<double>::quiet_NaN());
        std::printf("%s natDualSimplex %.1f ms iters=%d obj=%.8e "
                    "rowviol=%.2e bndviol=%.2e status=%s\n",
                    label, r.runtime_ms, r.iterations, r.objective,
                    r.max_row_viol, r.max_bound_viol, r.status.c_str());
        return r.success ? 0 : 1;
    }

    // --check: exit nonzero if any must-pass row fails.  The must-pass set
    // deliberately excludes natIPM rows (known-weak kernel, informational)
    // and time-limit-sensitive cases (118-bus): natDualSimplex on NETLIB,
    // no-false-optimum on the scaled probes (honest failures allowed), and
    // NativeBC[natSimplex]-vs-HiGHS optimum agreement on the small MILPs.
    int check_failures = 0;
    auto check_fail = [&](const std::string& what) {
        ++check_failures;
        std::printf("[CHECK-FAIL] %s\n", what.c_str());
    };

    // ── MILP level ────────────────────────────────────────────────────────
    if (!skip_milp) {
        std::vector<TestCase> cases;
        {
            SCUCInput inp = build_6bus_case(/*T=*/4);
            inp.config.solve_sced = false;
            inp.config.solve_lmp  = false;
            cases.push_back({"UC_6bus_3G_4T", std::move(inp)});
        }
        if (!smoke_mode) {
            SCUCInput inp = build_ieee39_case(/*T=*/4);
            inp.config.solve_sced = false;
            inp.config.solve_lmp  = false;
            cases.push_back({"UC_39bus_10G_4T", std::move(inp)});
        }
        if (!smoke_mode) {
            SCUCInput inp = build_ieee39_case(/*T=*/24);
            inp.config.solve_sced = false;
            inp.config.solve_lmp  = false;
            cases.push_back({"UC_39bus_10G_24T", std::move(inp)});
        }
        if (full_mode && !smoke_mode) {
            SCUCInput inp = build_ieee118_case(/*T=*/24);
            inp.config.solve_sced = false;
            inp.config.solve_lmp  = false;
            cases.push_back({"UC_118bus_54G_24T", std::move(inp)});
        }

        std::vector<MilpRow> rows;
        for (const auto& tc : cases) {
            engine::MIPModel probe = build_scuc_mip(tc.inp);
            std::printf("--- %s: %zu vars (%zu bin), %ld ineq, %ld eq ---\n",
                        tc.name.c_str(), probe.linear_part.vars.size(),
                        probe.binary_idx.size(),
                        static_cast<long>(probe.linear_part.b.size()),
                        static_cast<long>(probe.linear_part.beq.size()));
            std::fflush(stdout);

            rows.push_back(run_native_bc(tc, /*ipm_root=*/false, time_limit_sec));
            std::printf("  NativeBC[natSimplex] done (%.1f ms)\n", rows.back().runtime_ms);
            std::fflush(stdout);

            rows.push_back(run_native_bc(tc, /*ipm_root=*/true, time_limit_sec));
            std::printf("  NativeBC[natIPMroot] done (%.1f ms)\n", rows.back().runtime_ms);
            std::fflush(stdout);

            rows.push_back(run_highs_mip(tc, time_limit_sec));
            std::printf("  HiGHS[direct]        done (%.1f ms)\n", rows.back().runtime_ms);
            std::fflush(stdout);
        }
        print_milp_table(rows);

        if (check_mode) {
            for (const auto& tc : cases) {
                if (tc.name.find("118bus") != std::string::npos) continue;
                const MilpRow* native = nullptr;
                const MilpRow* highs = nullptr;
                for (const auto& r : rows) {
                    if (r.case_name != tc.name) continue;
                    if (r.solver == "NativeBC[natSimplex]") native = &r;
                    if (r.solver == "HiGHS[direct]") highs = &r;
                }
                if (!native || !native->success) {
                    check_fail(tc.name + ": NativeBC[natSimplex] failed");
                    continue;
                }
                if (!highs || !highs->success) {
                    check_fail(tc.name + ": HiGHS[direct] failed");
                    continue;
                }
                const double rel = std::abs(native->objective - highs->objective) /
                                   std::max(1.0, std::abs(highs->objective));
                if (rel > 1e-4) {
                    check_fail(tc.name + ": NativeBC/HiGHS optima differ (rel " +
                               std::to_string(rel) + ")");
                }
            }
        }
    }

    // ── LP level ──────────────────────────────────────────────────────────
    if (!skip_lp) {
        // 1) NETLIB with published optima
        const std::pair<const char*, double> kNetlib[] = {
            {"afiro",    -4.6475314286e+02},
            {"adlittle",  2.2549496316e+05},
            {"share2b",  -4.1573224074e+02},
            {"stocfor1", -4.1131976219e+04},
            {"kb2",      -1.7499001299e+03},
        };
        std::vector<LpRow> netlib_rows;
        for (const auto& [name, obj] : kNetlib) {
            engine::LPModel lp;
            if (!read_mps_as_lp(netlib_path(name), lp)) {
                std::printf("WARN: cannot read %s\n", name);
                continue;
            }
            netlib_rows.push_back(run_native_simplex_lp(name, lp, obj));
            netlib_rows.push_back(run_native_ipm_lp(name, lp, obj));
            netlib_rows.push_back(run_highs_lp(name, lp, obj, time_limit_sec));
        }
        print_lp_table("LP level: NETLIB published optima", netlib_rows);
        if (check_mode) {
            for (const auto& r : netlib_rows) {
                if (r.solver != "natDualSimplex") continue;
                if (!r.success || !(r.obj_rel_err >= 0.0) ||
                    r.obj_rel_err > 1e-6) {
                    check_fail(r.case_name + ": natDualSimplex NETLIB (relerr " +
                               std::to_string(r.obj_rel_err) + ", status " +
                               r.status + ")");
                }
            }
        }

        // 2) SCUC LP relaxations (large, highly degenerate).  Reference =
        //    HiGHS LP objective (no published optimum available).
        std::vector<LpRow> relax_rows;
        auto run_relax = [&](const char* label, const SCUCInput& inp) {
            engine::MIPModel mip = build_scuc_mip(inp);
            engine::LPModel lp = mip.linear_part;  // integrality dropped
            LpRow hi = run_highs_lp(label, lp,
                                    std::numeric_limits<double>::quiet_NaN(),
                                    time_limit_sec);
            const double ref = hi.success ? hi.objective
                                          : std::numeric_limits<double>::quiet_NaN();
            relax_rows.push_back(run_native_simplex_lp(label, lp, ref));
            relax_rows.push_back(run_native_ipm_lp(label, lp, ref));
            relax_rows.push_back(hi);
        };
        {
            SCUCInput inp = build_6bus_case(4);
            inp.config.solve_sced = false;
            inp.config.solve_lmp  = false;
            run_relax("UC_6bus_4T-relax", inp);
        }
        if (!smoke_mode) {
            SCUCInput inp = build_ieee39_case(24);
            inp.config.solve_sced = false;
            inp.config.solve_lmp  = false;
            run_relax("UC_39bus_24T-relax", inp);
        }
        if (full_mode && !smoke_mode) {
            SCUCInput inp = build_ieee118_case(24);
            inp.config.solve_sced = false;
            inp.config.solve_lmp  = false;
            run_relax("UC_118bus_24T-relax", inp);
        }
        print_lp_table("LP level: SCUC LP relaxations (ref = HiGHS)", relax_rows);

        // 3) Adversarial diagonal scaling stress (numerical stability probe)
        //    over all 5 NETLIB problems with published optima (objective is
        //    invariant under the diagonal scaling).
        std::vector<LpRow> scale_rows;
        for (const auto& [name, ref_obj] : kNetlib) {
            engine::LPModel base;
            if (!read_mps_as_lp(netlib_path(name), base)) continue;
            for (double k : {2.0, 4.0, 6.0}) {
                engine::LPModel scaled = scale_lp(base, k, /*seed=*/42);
                char label[64];
                std::snprintf(label, sizeof(label), "%s-scale1e%d", name,
                              static_cast<int>(k));
                scale_rows.push_back(run_native_simplex_lp(label, scaled, ref_obj));
                scale_rows.push_back(run_native_ipm_lp(label, scaled, ref_obj));
                scale_rows.push_back(run_highs_lp(label, scaled, ref_obj,
                                                  time_limit_sec));
            }
        }
        print_lp_table("LP level: adversarial diagonal scaling (10^k dynamic range)",
                       scale_rows);
        if (check_mode) {
            for (const auto& r : scale_rows) {
                if (r.solver != "natDualSimplex") continue;
                // The enforced property on adversarial rows is "no false
                // optimum": a successful solve must be accurate, while an
                // honest failure (e.g. the residual audit demoting a garbage
                // "Optimal") is acceptable — stocfor1 at 1e6 scaling defeats
                // HiGHS too (7e17 rel-err "Optimal").
                if (r.success &&
                    (!(r.obj_rel_err >= 0.0) || r.obj_rel_err > 1e-6)) {
                    check_fail(r.case_name + ": natDualSimplex scaled false "
                               "optimal (relerr " + std::to_string(r.obj_rel_err) +
                               ", status " + r.status + ")");
                }
            }
        }
    }

    if (check_mode) {
        std::printf("\n[CHECK] %d must-pass failure(s)\n", check_failures);
        return check_failures == 0 ? 0 : 1;
    }
    return 0;
}
