/// @file bc_legacy_helpers.cpp
/// @brief Small support helpers shared by the legacy branch-and-cut modules.

#include "mipsolvers/engine/detail/bc_legacy_helpers.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <memory>
#include <thread>
#include <vector>

#include <Eigen/Sparse>
#include <fmt/format.h>

#include "mipsolvers/engine/detail/bc_env_options.hpp"
#include "mipsolvers/engine/detail/bc_utils.hpp"

namespace mipsolvers::engine {

int resolve_num_threads(int requested, int min_threads, int max_threads) {
  if (requested >= 1) return requested;
  const unsigned hw = std::thread::hardware_concurrency();
  int n = (hw == 0) ? min_threads : static_cast<int>(hw);
  if (n < min_threads) n = min_threads;
  if (max_threads > 0 && n > max_threads) n = max_threads;
  return n;
}

int resolve_num_threads(const BCOptions& opt) {
  return resolve_num_threads(opt.num_threads,
                             std::max(1, opt.auto_parallel_min_threads),
                             opt.auto_parallel_max_threads);
}

}  // namespace mipsolvers::engine

namespace mipsolvers::engine::detail {

bool bc_env_flag_enabled(const char* name) {
  const char* env = std::getenv(name);
  return env != nullptr && env[0] != '\0' && env[0] != '0';
}

int bc_conformance_trace_terms(const char* env_name, int default_value) {
  const char* env = std::getenv(env_name);
  if (env == nullptr || env[0] == '\0') return default_value;
  char* end = nullptr;
  const long parsed = std::strtol(env, &end, 10);
  if (end == env || parsed <= 0) return default_value;
  return static_cast<int>(std::min<long>(parsed, 256));
}

bool bc_frontier_conformance_enabled() {
  return bc_env_flag_enabled("MIPSOLVERS_FRONTIER_CONFORM") ||
         bc_env_flag_enabled("MIPSOLVERS_NATIVE_FRONTIER_CONFORM") ||
         bc_env_flag_enabled("MIPSOLVERS_LPSTATE_CONFORM");
}

bool bc_first_class_lp_state_conformance_enabled() {
  return bc_frontier_conformance_enabled() ||
         bc_env_flag_enabled("MIPSOLVERS_ROOT_COLDSTATE_DIAG") ||
         bc_env_flag_enabled("MIPSOLVERS_XPOOL_EXACT_DSE") ||
         bc_env_flag_enabled("MIPSOLVERS_XPOOL_PARENT_EXACT_DSE");
}

bool bc_vendored_highs_lp_kernel_enabled(const BCOptions& opt) {
  return opt.use_vendored_highs_lp_kernel ||
         bc_env_options().use_vendored_highs_lp;
}

bool bc_vendored_highs_root_frontier_enabled(const BCOptions& opt) {
  return !opt.suppress_vendored_highs_root_frontier &&
         !bc_env_options().suppress_vendored_highs_root_frontier;
}

BcStrictHighsContractState apply_bc_strict_highs_contract(BCOptions& opt) {
  BcStrictHighsContractState state;
  state.domain_heuristics = opt.enable_domain_heuristics;
  state.requested_vendored_highs_lp = bc_vendored_highs_lp_kernel_enabled(opt);
  if (state.requested_vendored_highs_lp) {
    opt.use_vendored_highs_lp_kernel = true;
  }
  state.strict_highs_lp_contract = state.requested_vendored_highs_lp;
  state.allow_vendored_root_frontier =
      bc_vendored_highs_root_frontier_enabled(opt);

  if (state.requested_vendored_highs_lp) {
    const bool strict_tree_exhaustion =
        bc_env_options().require_strict_tree_exhaustion;
    // Suppressing the vendored root frontier only removes the direct
    // HiGHS-root certificate.  It must not also disable HiGHS' normal MIP
    // optimality-limit lifecycle: HighsMipSolverData keeps upper_limit for
    // incumbent cutoff propagation and optimality_limit for gap-valid node/root
    // pruning.  A strict live-tree exhaustion audit is still available through
    // MIPSOLVERS_REQUIRE_STRICT_TREE_EXHAUSTION.
    opt.require_tree_exhaustion_certificate = strict_tree_exhaustion;
  }
  if (state.requested_vendored_highs_lp) {
    // HiGHS does not turn local proof artifacts into ad-hoc node LP rows.
    // Reduced-cost/domain proofs are resolved through the local domain trail
    // and published as reconvergence/bound-lifting certificates; when domain
    // bounds change, the LP is re-evaluated from the same HiGHS kernel state.
    // This must remain enabled in suppress-root proof-tail mode as well:
    // strict tree exhaustion removes the root MIP certificate, not the
    // ConflictSet::resolveLinearLeq/Geq-style local trail resolver.
    opt.enable_reduced_cost_conflict_learning = true;
    opt.enable_reduced_cost_proof_conflict_minimization = true;
  }
  if (state.strict_highs_lp_contract) {
    // Strict HiGHS LP-oracle mode: the native tree may consume the vendored
    // HiGHS LP kernel, reduced-cost fixing, and local-domain certificates, but
    // all proof/frontier artifacts must live in the same original column space.
    // PaPILO's active-column reduced LP is only safe when its own presolve
    // trail/postsolve owns the full MIP search, which this native tree does not.
    opt.use_papilo_presolve = false;
    opt.enable_domain_heuristics = false;
    state.domain_heuristics = false;

    // Remove legacy/native proof and primal shortcuts from the strict path.
    // HiGHS evaluateRootNode() does run root primal sources, but their lifecycle is
    // tied to HighsDomain::propagate()/evaluateRootLp(): randomized/shifting,
    // central rounding, root reduced-cost, RENS, and finally feasibility pump only
    // when no upper_limit exists.  The native objective pump, progressive
    // rounding, and generic LP diving paths are separate repair heuristics; in a
    // strict HiGHS comparison they pollute the root fixed-point timing and can
    // publish incumbents before the HiGHS-style domain/cutpool closure has reached
    // its own state.  Keep the implemented HiGHS-like central rounding and RENS
    // hooks, but disable the non-HiGHS repair loops here.
    opt.use_feasibility_pump = false;
    opt.use_progressive_rounding = false;
    opt.accept_verified_warm_start_incumbent = false;
    opt.enable_feasibility_jump = false;
    opt.use_analytic_centre = true;
    opt.use_linesearch_rounding = true;
    opt.use_lock_count_rounding = true;
    // Keep the HiGHS root pipeline switch on in strict mode.  In this codebase
    // it gates root separation/cutpool/domain fixed-point work as well as some
    // primal heuristics; the primal pieces are disabled individually above.
    opt.auto_highs_root_pipeline = true;
    opt.enable_lns = false;
    opt.enable_incumbent_local_branching = false;
    opt.enable_root_low_fractionality_rens = true;
    opt.max_dive_lps = 0;
    opt.max_probe_vars = 0;
    opt.root_split_bound_probing = false;
    opt.crossover_heuristic_freq = 0;
    opt.rins_frequency = 0;

    // Local proof artifacts may only become domain-trail/reconvergence
    // certificates. Do not turn them into native/IPM local rows or UC-specific
    // cutoff rows that HiGHS would not add to the node LP.
    opt.enable_reduced_cost_proof_cut_resolve = false;
    opt.enable_verified_reduced_cost_conflict_minimization = false;
    opt.enable_dynamic_implied_bound_probing = false;
    opt.enable_graph_implied_bound_cuts = false;
    // Keep only the HiGHS ObjectivePropagation::propagate() contract.  Native
    // rest-LP cutoff covers / weighted event cuts are separate proof artifacts:
    // they are useful experiments, but they are not HiGHS' pending-row objective
    // propagation and must not participate in strict correctness/performance
    // comparisons.
    opt.enable_objective_cutoff_conflict_cuts = false;
    opt.enable_objective_cutoff_weighted_event_cuts = false;
    opt.enable_nonviolated_cutoff_conflict_covers = false;
    // Keep objective-cutoff domain propagation enabled: this corresponds to
    // HighsDomain::ObjectivePropagation using the incumbent upper_limit.
    opt.enable_objective_cutoff_domain_fixing = true;
  }

  return state;
}

BcDeclaredIntegrality normalize_declared_integrality(const MIPModel& prob,
                                                     LPModel& base_lp) {
  const int n = static_cast<int>(base_lp.vars.size());
  BcDeclaredIntegrality out;
  out.integer_cols.assign(static_cast<std::size_t>(n), 0);
  out.binary_cols.assign(static_cast<std::size_t>(n), 0);
  for (int idx = 0; idx < n; ++idx) {
    if (is_integer_type(base_lp.vars[static_cast<std::size_t>(idx)])) {
      out.integer_cols[static_cast<std::size_t>(idx)] = 1;
      if (base_lp.vars[static_cast<std::size_t>(idx)].type ==
          VarType::Binary) {
        out.binary_cols[static_cast<std::size_t>(idx)] = 1;
      }
    }
  }

  for (int idx : prob.integer_idx) {
    if (idx >= 0 && idx < n) {
      base_lp.vars[static_cast<std::size_t>(idx)].type = VarType::Integer;
      out.integer_cols[static_cast<std::size_t>(idx)] = 1;
    }
  }
  for (int idx : prob.binary_idx) {
    if (idx >= 0 && idx < n) {
      auto& var = base_lp.vars[static_cast<std::size_t>(idx)];
      var.type = VarType::Binary;
      var.lb = std::max(var.lb, 0.0);
      var.ub = std::min(var.ub, 1.0);
      out.integer_cols[static_cast<std::size_t>(idx)] = 1;
      out.binary_cols[static_cast<std::size_t>(idx)] = 1;
    }
  }
  return out;
}

std::shared_ptr<SimplexBasis> persist_bc_node_basis_from_simplex(
    SimplexResult& simplex,
    const std::shared_ptr<SimplexBasis>& previous_basis) {
  auto basis = std::make_shared<SimplexBasis>(std::move(simplex.basis));
  if (previous_basis) basis->try_share_indices_from(*previous_basis);
  basis->compact_indices_storage();
  if (simplex.shared_binv) {
    basis->cached_inverse = simplex.shared_binv;
  } else if (simplex.basis_inverse.rows() > 0) {
    basis->cached_inverse =
        std::make_shared<const Eigen::MatrixXd>(std::move(simplex.basis_inverse));
  }
  basis->cached_reduced_costs =
      std::make_shared<const Eigen::VectorXd>(std::move(simplex.reduced_costs));
  if (simplex.form.col_scale.size() == simplex.form.A.cols()) {
    basis->cached_col_scale =
        std::make_shared<const Eigen::VectorXd>(simplex.form.col_scale);
  }
  // Tree/node SF objects are per-node workspaces. Do not persist live BasisOps
  // pointers tied to those temporary matrices across queue siblings.
  basis->cached_sparse_basis.reset();
  basis->persist_eta_count = 0;
  return basis;
}

#ifdef MIPSOLVERS_HAVE_HIGHS_LIB
const char* bc_highs_model_status_label(HighsModelStatus status) {
  switch (status) {
    case HighsModelStatus::kOptimal:
      return "Optimal";
    case HighsModelStatus::kInfeasible:
      return "Infeasible";
    case HighsModelStatus::kUnbounded:
      return "Unbounded";
    case HighsModelStatus::kUnboundedOrInfeasible:
      return "UnboundedOrInfeasible";
    case HighsModelStatus::kObjectiveBound:
      return "ObjectiveBound";
    case HighsModelStatus::kObjectiveTarget:
      return "ObjectiveTarget";
    case HighsModelStatus::kTimeLimit:
      return "TimeLimit";
    case HighsModelStatus::kIterationLimit:
      return "IterationLimit";
    case HighsModelStatus::kSolutionLimit:
      return "SolutionLimit";
    default:
      return "Other";
  }
}

bool bc_native_basis_to_highs_basis(const LPModel& lp,
                                    const SimplexBasis* basis_hint,
                                    HighsBasis& hbasis) {
  if (basis_hint == nullptr) return false;
  const int n = static_cast<int>(lp.vars.size());
  const int m = static_cast<int>(lp.A.rows() + lp.Aeq.rows());
  if (basis_hint->rows != m ||
      static_cast<int>(basis_hint->index_count()) != m ||
      static_cast<int>(basis_hint->at_upper.size()) < n) {
    return false;
  }
  hbasis.valid = false;
  hbasis.alien = true;
  hbasis.useful = true;
  hbasis.col_status.assign(static_cast<std::size_t>(n),
                           HighsBasisStatus::kLower);
  hbasis.row_status.assign(static_cast<std::size_t>(m),
                           HighsBasisStatus::kLower);
  for (int j = 0; j < n; ++j) {
    const auto& v = lp.vars[static_cast<std::size_t>(j)];
    const bool fixed = std::isfinite(v.lb) && std::isfinite(v.ub) &&
                       std::abs(v.ub - v.lb) <= 1e-12;
    if (fixed) {
      hbasis.col_status[static_cast<std::size_t>(j)] =
          HighsBasisStatus::kLower;
    } else if (basis_hint->at_upper[static_cast<std::size_t>(j)] != 0) {
      hbasis.col_status[static_cast<std::size_t>(j)] =
          HighsBasisStatus::kUpper;
    }
  }

  const int slack_begin = n;
  const int surplus_begin = n + std::max(0, basis_hint->sf_n_slack);
  const int art_begin = surplus_begin + std::max(0, basis_hint->sf_n_surplus);
  const int n_slack = std::max(0, basis_hint->sf_n_slack);
  const int n_surplus = std::max(0, basis_hint->sf_n_surplus);
  const int n_art = std::max(0, basis_hint->sf_n_artificial);
  const auto& basis_indices = basis_hint->basis_indices();
  for (int row = 0; row < m; ++row) {
    const int col = basis_indices[static_cast<std::size_t>(row)];
    if (col >= 0 && col < n) {
      hbasis.col_status[static_cast<std::size_t>(col)] =
          HighsBasisStatus::kBasic;
    } else if (col >= slack_begin && col < slack_begin + n_slack) {
      hbasis.row_status[static_cast<std::size_t>(col - slack_begin)] =
          HighsBasisStatus::kBasic;
    } else if (col >= surplus_begin && col < surplus_begin + n_surplus) {
      hbasis.row_status[static_cast<std::size_t>(col - surplus_begin)] =
          HighsBasisStatus::kBasic;
    } else if (col >= art_begin && col < art_begin + n_art) {
      hbasis.row_status[static_cast<std::size_t>(col - art_begin)] =
          HighsBasisStatus::kBasic;
    } else {
      return false;
    }
  }
  return true;
}

bool bc_pass_mip_model_to_highs(Highs& highs,
                                const LPModel& lp,
                                const Eigen::VectorXd* lb_override,
                                const Eigen::VectorXd* ub_override) {
  const int ncols = static_cast<int>(lp.vars.size());
  const int m_ineq = static_cast<int>(lp.A.rows());
  const int m_eq = static_cast<int>(lp.Aeq.rows());
  const int nrows = m_ineq + m_eq;
  if (lp.c.size() != ncols) return false;
  if (lp.b.size() != m_ineq || lp.beq.size() != m_eq) return false;
  if (lb_override != nullptr && lb_override->size() != ncols) return false;
  if (ub_override != nullptr && ub_override->size() != ncols) return false;

  const double inf = std::numeric_limits<double>::infinity();
  std::vector<double> col_cost(static_cast<std::size_t>(ncols), 0.0);
  std::vector<double> col_lower(static_cast<std::size_t>(ncols), -inf);
  std::vector<double> col_upper(static_cast<std::size_t>(ncols), inf);
  std::vector<HighsInt> integrality(static_cast<std::size_t>(ncols),
                                    static_cast<HighsInt>(HighsVarType::kContinuous));
  for (int j = 0; j < ncols; ++j) {
    col_cost[static_cast<std::size_t>(j)] =
        lp.sense == Sense::Maximize ? -lp.c[j] : lp.c[j];
    const auto& v = lp.vars[static_cast<std::size_t>(j)];
    col_lower[static_cast<std::size_t>(j)] =
        lb_override != nullptr ? (*lb_override)[j] : v.lb;
    col_upper[static_cast<std::size_t>(j)] =
        ub_override != nullptr ? (*ub_override)[j] : v.ub;
    if (v.type != VarType::Continuous) {
      integrality[static_cast<std::size_t>(j)] =
          static_cast<HighsInt>(HighsVarType::kInteger);
    }
  }

  std::vector<double> row_lower(static_cast<std::size_t>(nrows), -inf);
  std::vector<double> row_upper(static_cast<std::size_t>(nrows), inf);
  for (int r = 0; r < m_ineq; ++r) {
    row_lower[static_cast<std::size_t>(r)] = lp_row_lhs_or_neg_inf(lp, r);
    row_upper[static_cast<std::size_t>(r)] = lp.b[r];
  }
  for (int r = 0; r < m_eq; ++r) {
    const int rr = m_ineq + r;
    row_lower[static_cast<std::size_t>(rr)] = lp.beq[r];
    row_upper[static_cast<std::size_t>(rr)] = lp.beq[r];
  }

  std::vector<HighsInt> start(static_cast<std::size_t>(ncols + 1), 0);
  std::vector<HighsInt> index;
  std::vector<double> value;
  index.reserve(static_cast<std::size_t>(lp.A.nonZeros() + lp.Aeq.nonZeros()));
  value.reserve(index.capacity());
  const Eigen::SparseMatrix<double, Eigen::ColMajor> A_col(lp.A);
  const Eigen::SparseMatrix<double, Eigen::ColMajor> Aeq_col(lp.Aeq);
  for (int j = 0; j < ncols; ++j) {
    start[static_cast<std::size_t>(j)] =
        static_cast<HighsInt>(index.size());
    for (Eigen::SparseMatrix<double, Eigen::ColMajor>::InnerIterator it(A_col, j);
         it; ++it) {
      if (it.value() == 0.0) continue;
      index.push_back(static_cast<HighsInt>(it.row()));
      value.push_back(it.value());
    }
    for (Eigen::SparseMatrix<double, Eigen::ColMajor>::InnerIterator it(Aeq_col, j);
         it; ++it) {
      if (it.value() == 0.0) continue;
      index.push_back(static_cast<HighsInt>(m_ineq + it.row()));
      value.push_back(it.value());
    }
  }
  start[static_cast<std::size_t>(ncols)] =
      static_cast<HighsInt>(index.size());

  const HighsStatus pass_status = highs.passModel(
      static_cast<HighsInt>(ncols), static_cast<HighsInt>(nrows),
      static_cast<HighsInt>(index.size()),
      static_cast<HighsInt>(MatrixFormat::kColwise),
      static_cast<HighsInt>(ObjSense::kMinimize), 0.0, col_cost.data(),
      col_lower.data(), col_upper.data(), row_lower.data(), row_upper.data(),
      start.data(), index.data(), value.data(), integrality.data());
  return pass_status == HighsStatus::kOk;
}

VendoredHighsRootCertificateResult bc_try_vendored_highs_root_certificate(
    const LPModel& lp,
    const Eigen::VectorXd* lb_override,
    const Eigen::VectorXd* ub_override,
    const BCOptions& opt) {
  VendoredHighsRootCertificateResult out;
  out.attempted = true;
  const auto h0 = std::chrono::steady_clock::now();

  const int n = static_cast<int>(lp.vars.size());
  if (n <= 0 || lp.c.size() != n) {
    out.status = "invalid_model_size";
    return out;
  }
  if (lp.sense != Sense::Minimize) {
    out.status = "unsupported_sense";
    return out;
  }

  Highs highs;
  highs.setOptionValue("output_flag", false);
  highs.setOptionValue("log_to_console", false);
  highs.setOptionValue("threads", 1);
  highs.setOptionValue("presolve", "choose");
  highs.setOptionValue("solver", "choose");
  highs.setOptionValue("mip_lp_solver", "choose");
  highs.setOptionValue("mip_ipm_solver", "choose");
  highs.setOptionValue("mip_rel_gap", opt.gap_tol);

  if (!bc_pass_mip_model_to_highs(highs, lp, lb_override, ub_override)) {
    out.status = "pass_error";
    return out;
  }

  const HighsStatus run_status = highs.run();
  const HighsModelStatus model_status = highs.getModelStatus();
  const HighsInfo& info = highs.getInfo();
  out.nodes = info.mip_node_count;
  out.simplex_iterations = info.simplex_iteration_count;
  out.dual_bound = info.mip_dual_bound;
  out.status = fmt::format("{} run={} nodes={} iters={}",
                           bc_highs_model_status_label(model_status),
                           static_cast<int>(run_status), out.nodes,
                           out.simplex_iterations);

  const HighsSolution& sol = highs.getSolution();
  if (run_status != HighsStatus::kOk ||
      model_status != HighsModelStatus::kOptimal ||
      out.nodes < 0 || out.nodes > 1 ||
      static_cast<int>(sol.col_value.size()) < n ||
      !std::isfinite(out.dual_bound)) {
    out.runtime_sec =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - h0)
            .count();
    return out;
  }

  Eigen::VectorXd x(n);
  for (int j = 0; j < n; ++j) {
    x[j] = sol.col_value[static_cast<std::size_t>(j)];
  }

  Eigen::VectorXd lb(n), ub(n);
  if (lb_override != nullptr && lb_override->size() == n) {
    lb = *lb_override;
  } else {
    for (int j = 0; j < n; ++j) lb[j] = lp.vars[static_cast<std::size_t>(j)].lb;
  }
  if (ub_override != nullptr && ub_override->size() == n) {
    ub = *ub_override;
  } else {
    for (int j = 0; j < n; ++j) ub[j] = lp.vars[static_cast<std::size_t>(j)].ub;
  }

  if (!satisfies_with_bounds(lp, x, lb, ub, 1e-6)) {
    out.status += " invalid_native_recheck";
    out.runtime_sec =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - h0)
            .count();
    return out;
  }

  out.primal_obj = objective_value(lp.c, x, lp.sense);
  if (!std::isfinite(out.primal_obj)) {
    out.status += " invalid_objective";
    out.runtime_sec =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - h0)
            .count();
    return out;
  }
  out.rel_gap = std::max(
      0.0, (out.primal_obj - out.dual_bound) /
               std::max(1.0, std::abs(out.primal_obj)));
  if (out.rel_gap <= opt.gap_tol + 1e-12) {
    out.x = std::move(x);
    out.accepted = true;
  } else {
    out.status += fmt::format(" gap_reject={:.6g}", out.rel_gap);
  }
  out.runtime_sec =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - h0)
          .count();
  return out;
}
#endif

void apply_bc_first_class_simplex_state(SimplexOptions& opt,
                                        bool allow_frontier_remap) {
  if (!bc_first_class_lp_state_conformance_enabled()) return;
  opt.exact_dse_initialization = true;
  opt.perturb_degenerate_primal = false;
  opt.enable_degenerate_frontier_remap = allow_frontier_remap;
  if (!allow_frontier_remap) {
    opt.suppress_degenerate_frontier_remap = true;
  }
}

VendoredHighsSfBackendScope::VendoredHighsSfBackendScope(bool enabled)
    : old_(set_vendored_highs_sf_backend_thread_enabled(
          enabled || vendored_highs_sf_backend_thread_enabled())) {}

VendoredHighsSfBackendScope::~VendoredHighsSfBackendScope() {
  set_vendored_highs_sf_backend_thread_enabled(old_);
}



}  // namespace mipsolvers::engine::detail
