/// End-to-end MILP benchmark over MIPLIB 2017 MPS instances.
///
/// The runner keeps MPS integrality and compares five explicitly named paths:
///   cplex-mip             CPLEX owns the complete MIP solve
///   highs-mip              HiGHS owns the complete MIP solve
///   scip-mip               SCIP owns the complete MIP solve
///   native-highs-lp        native B&C with the HiGHS node-LP kernel
///   native-native-lp       native B&C with the experimental native LP kernel
///
/// Model loading is reported separately from solve time. Every incumbent is
/// audited against the original HighsLp representation, including the MPS
/// objective offset, row ranges, variable bounds, and integrality.

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <numeric>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#ifndef _WIN32
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#endif

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <nlohmann/json.hpp>
#include <zlib.h>

#include "Highs.h"

#ifdef MIPSOLVERS_HAVE_SCIP_LIB
#include "scip/scip.h"
#include "scip/scipdefplugins.h"
#include "scip/scip_event.h"
#endif

#include "mipsolvers/engine/bc/api.hpp"
#include "mipsolvers/engine/bc/options.hpp"
#include "mipsolvers/engine/bc/stats.hpp"
#include "mipsolvers/engine/problem_types.hpp"
#include "mipsolvers/engine/solver/external/adapters.hpp"
#include "mipsolvers/engine/solver/native/milp/bc/milp_presolve.hpp"

namespace fs = std::filesystem;
using json = nlohmann::json;
namespace eng = mipsolvers::engine;

namespace {

// Gleixner et al. (2021), MIPLIB 2017, Sec. 3; metric derivation and audit
// contract: docs/miplib2017_benchmark_protocol_2026-08-25.md.
constexpr double kAuditTolerance = 1e-5;
constexpr double kSummaryShiftMs = 1000.0;
constexpr double kSummaryNodeShift = 100.0;

struct Config {
  fs::path data_dir{"tests/data/miplib2017/benchmark"};
  fs::path solution_file;
  fs::path csv_path;
  fs::path json_path;
  std::vector<std::string> solvers{
      "highs-mip", "scip-mip", "native-highs-lp", "native-native-lp"};
  std::vector<std::string> case_filters;
  int repeats{1};
  int limit{0};
  int sample{0};
  int max_nodes{50000};
  int native_threads{1};
  int seed{0};
  std::vector<int> seeds;
  double time_limit_sec{60.0};
  double hard_timeout_grace_sec{5.0};
  double gap{1e-4};
  std::string native_node_estimate{"sum"};
  bool highs_verbose{false};
  bool native_verbose{false};
  bool native_papilo_presolve{true};
  bool native_presolve_probing{true};
  bool native_cuts{true};
  bool native_objective_propagation{true};
  bool native_reduced_cost_fixing{true};
  int native_row_propagation_rounds{3};
  int native_probe_max{3};
  int native_probe_reliability{2};
  fs::path native_primal_hint_file;
  bool native_audit_hint_only{false};
  bool native_tree_restart{false};
  int native_tree_restart_max{2};
  int native_tree_restart_min_nodes{256};
  int native_tree_restart_min_open_nodes{64};
  double native_tree_restart_min_improvement{0.01};
  double native_tree_restart_min_remaining_sec{0.25};
  fs::path worker_instance;
  fs::path worker_output;
  std::string worker_solver;
};

struct Reference {
  std::string status;
  double objective{std::numeric_limits<double>::quiet_NaN()};
};

struct Instance {
  std::string name;
  fs::path path;
  fs::path solver_path;
  bool solver_path_is_temporary{false};
  HighsLp source_lp;
  eng::MIPModel mip;
  double objective_offset{0.0};
  double decompress_ms{0.0};
  double load_ms{0.0};
  int rows{0};
  int columns{0};
  std::int64_t nonzeros{0};
  int integers{0};
  int binaries{0};
  int semicontinuous{0};
  int semiinteger{0};
  bool native_supported{true};
  std::string error;
};

struct Audit {
  double objective{std::numeric_limits<double>::quiet_NaN()};
  double objective_disagreement{std::numeric_limits<double>::quiet_NaN()};
  double max_row_violation{std::numeric_limits<double>::infinity()};
  double max_bound_violation{std::numeric_limits<double>::infinity()};
  double max_integrality_violation{std::numeric_limits<double>::infinity()};
  bool passed{false};
};

struct BoundEvent {
  double time_ms{0.0};
  double primal_bound{std::numeric_limits<double>::quiet_NaN()};
  double dual_bound{std::numeric_limits<double>::quiet_NaN()};
  Eigen::VectorXd incumbent;
  Audit incumbent_audit;
};

struct Result {
  std::string instance;
  std::string solver;
  std::string collection_scope{"unavailable"};
  int repeat{0};
  int seed{0};
  int execution_index{-1};
  int block_order_position{-1};
  int rows{0};
  int columns{0};
  std::int64_t nonzeros{0};
  int integers{0};
  int binaries{0};
  int semicontinuous{0};
  int semiinteger{0};
  bool available{true};
  bool has_solution{false};
  bool optimal{false};
  bool proven{false};
  bool timed_out{false};
  bool hard_timeout{false};
  bool reference_match{false};
  bool node_count_available{false};
  bool lp_solve_count_available{false};
  bool lp_iteration_count_available{false};
  bool cut_count_available{false};
  bool incumbent_timeline_available{false};
  bool bound_event_stream_available{false};
  bool native_diagnostics_available{false};
  int parallel_requested_threads{1};
  int parallel_effective_threads{1};
  int parallel_explorer_threads{0};
  bool parallel_tree_launched{false};
  std::string parallel_schedule_reason{"unavailable"};
  double read_ms{0.0};
  double shared_decompress_ms{0.0};
  double solve_ms{0.0};
  double objective{std::numeric_limits<double>::quiet_NaN()};
  double best_bound{std::numeric_limits<double>::quiet_NaN()};
  double gap{std::numeric_limits<double>::infinity()};
  double primal_dual_integral_sec{
      std::numeric_limits<double>::quiet_NaN()};
  std::int64_t nodes{-1};
  std::int64_t lp_solves{-1};
  std::int64_t lp_iterations{-1};
  int cuts{-1};
  int first_incumbent_node{-1};
  int first_incumbent_lp_solves{-1};
  bool native_presolve_attempted{false};
  bool native_presolve_adopted{false};
  int native_presolve_orig_rows{0};
  int native_presolve_final_rows{0};
  int native_presolve_orig_cols{0};
  int native_presolve_final_cols{0};
  double native_presolve_time_ms{0.0};
  std::uint64_t native_presolve_probing_trail_pushes{0};
  std::uint64_t native_presolve_probing_rows_processed{0};
  std::uint64_t native_presolve_probing_implications_learned{0};
  std::uint64_t native_presolve_probing_implications_imported{0};
  std::uint64_t native_presolve_probing_max_touched_cols{0};
  bool native_presolve_probing_truncated{false};
  std::uint64_t node_queue_domain_compactions{0};
  std::uint64_t node_queue_domain_materializations{0};
  std::uint64_t node_queue_dense_bound_values_released{0};
  std::uint64_t node_queue_compact_entries_created{0};
  std::uint64_t node_queue_terminal_compact_nodes{0};
  std::uint64_t node_queue_terminal_compact_entries{0};
  std::uint64_t node_queue_peak_compact_nodes{0};
  std::uint64_t node_queue_peak_compact_entries{0};
  std::uint64_t node_queue_domain_compaction_failures{0};
  std::uint64_t node_queue_domain_materialization_failures{0};
  std::uint64_t node_queue_serial_compactions{0};
  std::uint64_t node_queue_parallel_compactions{0};
  std::uint64_t node_queue_submip_compactions{0};
  std::uint64_t branch_payload_child_creations{0};
  std::uint64_t branch_payload_shared_vectors{0};
  std::uint64_t branch_payload_shared_elements{0};
  std::uint64_t branch_domain_dense_copies{0};
  std::uint64_t branch_domain_dense_values_copied{0};
  std::uint64_t branch_domain_moves{0};
  std::uint64_t root_domain_probe_workspace_initializations{0};
  std::uint64_t root_domain_probe_worlds{0};
  std::uint64_t root_domain_probe_trail_pushes{0};
  std::uint64_t root_domain_probe_rollbacks{0};
  std::uint64_t root_domain_probe_failures{0};
  std::uint64_t root_domain_probe_rows_processed{0};
  std::uint64_t root_domain_probe_changed_columns{0};
  std::uint64_t root_domain_probe_implication_export_columns_visited{0};
  std::uint64_t root_domain_probe_implications_learned{0};
  std::uint64_t root_domain_probe_committed_bound_changes{0};
  std::uint64_t root_lp_probe_bound_workspace_initializations{0};
  std::uint64_t root_lp_probe_bound_transactions{0};
  std::uint64_t root_lp_probe_transaction_snapshot_values{0};
  std::uint64_t root_lp_probe_transaction_rollbacks{0};
  std::uint64_t root_lp_probe_transaction_failures{0};
  std::uint64_t root_lp_probe_backend_cold_solves{0};
  std::uint64_t root_lp_probe_backend_persistent_resolves{0};
  std::uint64_t strong_probe_base_sf_materializations{0};
  std::uint64_t strong_probe_bound_transactions{0};
  std::uint64_t strong_probe_transaction_snapshot_values{0};
  std::uint64_t strong_probe_transaction_rollbacks{0};
  std::uint64_t strong_probe_transaction_failures{0};
  std::uint64_t strong_probe_backend_cold_solves{0};
  std::uint64_t strong_probe_backend_persistent_resolves{0};
  std::uint64_t separator_sparse_candidates_created{0};
  std::uint64_t separator_sparse_candidate_entries_created{0};
  std::uint64_t separator_peak_live_sparse_candidates{0};
  std::uint64_t separator_peak_live_sparse_entries{0};
  std::uint64_t separator_sparse_aggregation_snapshots{0};
  std::uint64_t separator_sparse_aggregation_entries{0};
  std::uint64_t separator_dense_workspace_materializations{0};
  std::uint64_t separator_dense_workspace_values{0};
  std::uint64_t separator_matrix_append_calls{0};
  std::uint64_t separator_matrix_appended_rows{0};
  std::uint64_t separator_matrix_appended_entries{0};
  std::uint64_t separator_matrix_prior_entries_bypassing_triplet_rebuild{0};
  std::uint64_t separator_matrix_storage_reallocations{0};
  std::uint64_t separator_matrix_peak_spare_entries{0};
  std::uint64_t tree_restarts{0};
  std::uint64_t tree_restart_nodes_discarded{0};
  std::uint64_t tree_restart_root_requeues{0};
  std::uint64_t tree_restart_cut_pool_rows_preserved{0};
  std::uint64_t tree_restart_conflicts_preserved{0};
  std::uint64_t tree_restart_implications_preserved{0};
  std::uint64_t tree_restart_clique_edges_preserved{0};
  std::uint64_t tree_restart_pseudocost_observations_preserved{0};
  int tree_restart_last_node{-1};
  std::string tree_restart_last_source;
  std::uint64_t reliability_branch_nodes{0};
  std::uint64_t strong_branch_candidates{0};
  std::uint64_t strong_branch_lp_solves{0};
  std::uint64_t strong_branch_cache_exact_hits{0};
  std::uint64_t strong_branch_cache_warm_hits{0};
  std::uint64_t strong_branch_duplicate_lp_avoided{0};
  std::uint64_t branching_regret_samples{0};
  double branching_regret_sum{0.0};
  double branching_regret_max{0.0};
  std::uint64_t strong_branch_regret_samples{0};
  double strong_branch_regret_sum{0.0};
  double strong_branch_regret_max{0.0};
  std::uint64_t node_estimate_calibration_samples{0};
  double node_estimate_predicted_lift_sum{0.0};
  double node_estimate_realized_lift_sum{0.0};
  double node_estimate_abs_error_sum{0.0};
  double node_estimate_squared_error_sum{0.0};
  double node_estimate_predicted_sq_sum{0.0};
  double node_estimate_realized_sq_sum{0.0};
  double node_estimate_cross_sum{0.0};
  std::uint64_t directional_calibration_samples{0};
  double directional_predicted_gain_sum{0.0};
  double directional_realized_gain_sum{0.0};
  double directional_abs_error_sum{0.0};
  double directional_squared_error_sum{0.0};
  std::uint64_t directional_rank_samples{0};
  std::uint64_t directional_rank_concordant{0};
  int fallback_events{-1};
  int fallback_recoveries{-1};
  double reference_objective{std::numeric_limits<double>::quiet_NaN()};
  std::string reference_status;
  Audit audit;
  std::vector<BoundEvent> bound_events;
  std::string bound_event_stream_error;
  std::uint64_t bound_events_dropped_uncertified_dual{0};
  std::string status;
  Eigen::VectorXd x;
};

struct Summary {
  std::string solver;
  int attempts{0};
  int available{0};
  int feasible{0};
  int optimal{0};
  int proven{0};
  int audited{0};
  int reference_matches{0};
  int timeouts{0};
  int benchmark_solved{0};
  int node_count_runs{0};
  int solved_node_runs{0};
  int incumbent_audit_failures{0};
  double median_solved_ms{std::numeric_limits<double>::quiet_NaN()};
  double shifted_geomean_par10_ms{std::numeric_limits<double>::quiet_NaN()};
  double shifted_geomean_solved_nodes{
      std::numeric_limits<double>::quiet_NaN()};
  double max_row_violation{0.0};
  double max_bound_violation{0.0};
  double max_integrality_violation{0.0};
  double max_objective_disagreement{0.0};
  int primal_dual_integrals{0};
  double mean_primal_dual_integral_sec{
      std::numeric_limits<double>::quiet_NaN()};
};

double progress_number(double value) {
  return std::isfinite(value) && std::abs(value) < 1e29
             ? value
             : std::numeric_limits<double>::quiet_NaN();
}

void append_bound_event(Result& result, double time_ms, double primal_bound,
                        double dual_bound,
                        const Eigen::VectorXd* incumbent = nullptr) {
  BoundEvent event;
  event.time_ms = std::max(0.0, time_ms);
  event.primal_bound = progress_number(primal_bound);
  event.dual_bound = progress_number(dual_bound);
  if (incumbent != nullptr) event.incumbent = *incumbent;
  if (!std::isfinite(event.primal_bound) &&
      !std::isfinite(event.dual_bound)) {
    return;
  }
  result.bound_events.push_back(std::move(event));
}

void retain_terminally_certified_dual_events(const Instance& instance,
                                             Result& result,
                                             double final_dual_bound) {
  if (!std::isfinite(final_dual_bound)) return;
  const double tolerance =
      1e-9 * std::max(1.0, std::abs(final_dual_bound));
  const bool minimize = instance.source_lp.sense_ == ObjSense::kMinimize;
  for (auto& event : result.bound_events) {
    if (!std::isfinite(event.dual_bound)) continue;
    const bool stronger_than_final =
        minimize ? event.dual_bound > final_dual_bound + tolerance
                 : event.dual_bound < final_dual_bound - tolerance;
    if (!stronger_than_final) continue;
    event.dual_bound = std::numeric_limits<double>::quiet_NaN();
    ++result.bound_events_dropped_uncertified_dual;
  }
  std::erase_if(result.bound_events, [](const BoundEvent& event) {
    return !std::isfinite(event.primal_bound) &&
           !std::isfinite(event.dual_bound);
  });
}

std::vector<std::string> split(const std::string& text, char delimiter) {
  std::vector<std::string> out;
  std::istringstream in(text);
  std::string item;
  while (std::getline(in, item, delimiter)) {
    if (!item.empty()) out.push_back(item);
  }
  return out;
}

std::uint64_t stable_order_key(const std::string& solver, int seed) {
  std::uint64_t hash = 1469598103934665603ULL;
  for (unsigned char byte : solver) {
    hash ^= byte;
    hash *= 1099511628211ULL;
  }
  hash ^= static_cast<std::uint64_t>(static_cast<std::uint32_t>(seed));
  hash *= 1099511628211ULL;
  return hash;
}

std::vector<std::string> base_solver_order(const Config& cfg) {
  std::vector<std::string> order = cfg.solvers;
  std::stable_sort(order.begin(), order.end(), [&](const auto& a, const auto& b) {
    return stable_order_key(a, cfg.seed) < stable_order_key(b, cfg.seed);
  });
  return order;
}

std::vector<std::string> blocked_solver_order(
    const std::vector<std::string>& base_order,
    std::size_t instance_index,
    int repeat) {
  std::vector<std::string> order = base_order;
  if (order.empty()) return order;
  const std::size_t shift =
      (instance_index + static_cast<std::size_t>(repeat)) % order.size();
  std::rotate(order.begin(), order.begin() + shift, order.end());
  return order;
}

bool is_mps_path(const fs::path& path) {
  if (path.extension() == ".mps") return true;
  return path.extension() == ".gz" && path.stem().extension() == ".mps";
}

std::string instance_name(const fs::path& path) {
  fs::path name = path.filename();
  if (name.extension() == ".gz") name = name.stem();
  if (name.extension() == ".mps") name = name.stem();
  return name.string();
}

bool matches_case(const std::string& name,
                  const std::vector<std::string>& filters) {
  if (filters.empty()) return true;
  for (const std::string& filter : filters) {
    if (name.find(filter) != std::string::npos) return true;
  }
  return false;
}

bool parse_args(int argc, char** argv, Config& cfg) {
  bool saw_seed = false;
  bool saw_seeds = false;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto value = [&](const char* flag) -> const char* {
      if (i + 1 >= argc) {
        std::cerr << "Missing value after " << flag << "\n";
        return nullptr;
      }
      return argv[++i];
    };
    if (arg == "--data-dir") {
      const char* v = value("--data-dir"); if (!v) return false; cfg.data_dir = v;
    } else if (arg == "--solu") {
      const char* v = value("--solu"); if (!v) return false; cfg.solution_file = v;
    } else if (arg == "--csv") {
      const char* v = value("--csv"); if (!v) return false; cfg.csv_path = v;
    } else if (arg == "--json") {
      const char* v = value("--json"); if (!v) return false; cfg.json_path = v;
    } else if (arg == "--solvers") {
      const char* v = value("--solvers"); if (!v) return false;
      cfg.solvers = split(v, ',');
    } else if (arg == "--case") {
      const char* v = value("--case"); if (!v) return false;
      cfg.case_filters = split(v, ',');
    } else if (arg == "--repeat") {
      const char* v = value("--repeat"); if (!v) return false;
      cfg.repeats = std::max(1, std::atoi(v));
    } else if (arg == "--limit") {
      const char* v = value("--limit"); if (!v) return false;
      cfg.limit = std::max(0, std::atoi(v));
    } else if (arg == "--sample") {
      const char* v = value("--sample"); if (!v) return false;
      cfg.sample = std::max(0, std::atoi(v));
    } else if (arg == "--max-nodes") {
      const char* v = value("--max-nodes"); if (!v) return false;
      cfg.max_nodes = std::max(1, std::atoi(v));
    } else if (arg == "--native-threads") {
      const char* v = value("--native-threads"); if (!v) return false;
      cfg.native_threads = std::max(1, std::atoi(v));
    } else if (arg == "--seed") {
      const char* v = value("--seed"); if (!v) return false;
      cfg.seed = std::max(0, std::atoi(v));
      saw_seed = true;
    } else if (arg == "--seeds") {
      const char* v = value("--seeds"); if (!v) return false;
      cfg.seeds.clear();
      std::set<int> unique_seeds;
      for (const std::string& item : split(v, ',')) {
        errno = 0;
        char* end = nullptr;
        const long parsed = std::strtol(item.c_str(), &end, 10);
        if (errno != 0 || end == item.c_str() || *end != '\0' || parsed < 0 ||
            parsed > std::numeric_limits<int>::max()) {
          std::cerr << "Invalid nonnegative seed: " << item << "\n";
          return false;
        }
        if (!unique_seeds.insert(static_cast<int>(parsed)).second) {
          std::cerr << "Duplicate seed: " << item << "\n";
          return false;
        }
        cfg.seeds.push_back(static_cast<int>(parsed));
      }
      if (cfg.seeds.empty()) {
        std::cerr << "--seeds requires at least one value\n";
        return false;
      }
      saw_seeds = true;
    } else if (arg == "--time-limit") {
      const char* v = value("--time-limit"); if (!v) return false;
      cfg.time_limit_sec = std::max(0.01, std::atof(v));
    } else if (arg == "--hard-timeout-grace") {
      const char* v = value("--hard-timeout-grace"); if (!v) return false;
      cfg.hard_timeout_grace_sec = std::max(0.0, std::atof(v));
    } else if (arg == "--gap") {
      const char* v = value("--gap"); if (!v) return false;
      cfg.gap = std::max(0.0, std::atof(v));
    } else if (arg == "--native-node-estimate") {
      const char* v = value("--native-node-estimate"); if (!v) return false;
      cfg.native_node_estimate = v;
    } else if (arg == "--highs-verbose") {
      cfg.highs_verbose = true;
    } else if (arg == "--native-verbose") {
      cfg.native_verbose = true;
    } else if (arg == "--native-no-papilo-presolve") {
      cfg.native_papilo_presolve = false;
    } else if (arg == "--native-no-presolve-probing") {
      cfg.native_presolve_probing = false;
    } else if (arg == "--native-no-cuts") {
      cfg.native_cuts = false;
    } else if (arg == "--native-no-objective-propagation") {
      cfg.native_objective_propagation = false;
    } else if (arg == "--native-no-reduced-cost-fixing") {
      cfg.native_reduced_cost_fixing = false;
    } else if (arg == "--native-no-row-propagation") {
      cfg.native_row_propagation_rounds = 0;
    } else if (arg == "--native-row-propagation-rounds") {
      const char* v = value("--native-row-propagation-rounds");
      if (!v) return false;
      cfg.native_row_propagation_rounds = std::max(0, std::atoi(v));
    } else if (arg == "--native-probe-max") {
      const char* v = value("--native-probe-max"); if (!v) return false;
      cfg.native_probe_max = std::max(0, std::atoi(v));
    } else if (arg == "--native-probe-reliability") {
      const char* v = value("--native-probe-reliability"); if (!v) return false;
      cfg.native_probe_reliability = std::max(0, std::atoi(v));
    } else if (arg == "--native-primal-hint") {
      const char* v = value("--native-primal-hint"); if (!v) return false;
      cfg.native_primal_hint_file = v;
    } else if (arg == "--native-audit-hint-only") {
      cfg.native_audit_hint_only = true;
    } else if (arg == "--native-tree-restart") {
      cfg.native_tree_restart = true;
    } else if (arg == "--native-tree-restart-max") {
      const char* v = value("--native-tree-restart-max"); if (!v) return false;
      cfg.native_tree_restart_max = std::max(0, std::atoi(v));
    } else if (arg == "--native-tree-restart-min-nodes") {
      const char* v = value("--native-tree-restart-min-nodes"); if (!v) return false;
      cfg.native_tree_restart_min_nodes = std::max(0, std::atoi(v));
    } else if (arg == "--native-tree-restart-min-open-nodes") {
      const char* v = value("--native-tree-restart-min-open-nodes"); if (!v) return false;
      cfg.native_tree_restart_min_open_nodes = std::max(1, std::atoi(v));
    } else if (arg == "--native-tree-restart-min-improvement") {
      const char* v = value("--native-tree-restart-min-improvement"); if (!v) return false;
      cfg.native_tree_restart_min_improvement = std::max(0.0, std::atof(v));
    } else if (arg == "--native-tree-restart-min-remaining") {
      const char* v = value("--native-tree-restart-min-remaining"); if (!v) return false;
      cfg.native_tree_restart_min_remaining_sec = std::max(0.0, std::atof(v));
    } else if (arg == "--worker-instance") {
      const char* v = value("--worker-instance"); if (!v) return false;
      cfg.worker_instance = v;
    } else if (arg == "--worker-output") {
      const char* v = value("--worker-output"); if (!v) return false;
      cfg.worker_output = v;
    } else if (arg == "--worker-solver") {
      const char* v = value("--worker-solver"); if (!v) return false;
      cfg.worker_solver = v;
    } else if (arg == "--help" || arg == "-h") {
      std::cout
          << "Usage: miplib2017_benchmark [options]\n"
          << "  --data-dir DIR       recursively scan DIR for .mps[.gz]\n"
          << "  --solu FILE          MIPLIB .solu reference file\n"
          << "  --solvers A,B        cplex-mip,highs-mip,scip-mip,native-highs-lp,"
             "native-native-lp,native-presolve-highs-audit\n"
          << "  --case A,B           instance-name substring filters\n"
          << "  --limit N            run first N selected instances (0 = all)\n"
          << "  --sample N           evenly sample N names after filtering (0 = all)\n"
          << "  --repeat N           repetitions per solver/instance\n"
          << "  --time-limit SEC     per-solve wall limit\n"
          << "  --hard-timeout-grace SEC  process watchdog grace after the solve limit\n"
          << "  --native-node-estimate MODE  sum or maximum\n"
          << "  --highs-verbose       enable HiGHS MIP diagnostic logging\n"
          << "  --native-verbose      enable native B&C diagnostic logging\n"
          << "  --native-no-papilo-presolve  disable PaPILO in native audit runs\n"
          << "  --native-no-presolve-probing  disable Native presolve probing\n"
          << "  --native-no-cuts     disable native cut generation in audit runs\n"
          << "  --native-no-objective-propagation  disable incumbent-cutoff rows/domain fixing\n"
          << "  --native-no-reduced-cost-fixing    disable reduced-cost domain fixing/learning\n"
          << "  --native-no-row-propagation        set the propagation-round budget to zero\n"
          << "  --native-row-propagation-rounds N  model/cut domain-propagation round budget\n"
          << "  --native-probe-max N               maximum reliability-branch candidates (0 disables probing)\n"
          << "  --native-probe-reliability N       directional pseudocost reliability threshold\n"
          << "  --native-primal-hint FILE          JSON object/array containing an original-space x vector\n"
          << "  --native-audit-hint-only           audit the hint without accepting it as an incumbent/cutoff\n"
          << "  --native-tree-restart              enable incumbent-driven sequential-tree restart\n"
          << "  --native-tree-restart-max N        maximum restarts per solve\n"
          << "  --native-tree-restart-min-nodes N  nodes required between restarts\n"
          << "  --native-tree-restart-min-open-nodes N  open-node trigger threshold\n"
          << "  --native-tree-restart-min-improvement X  relative incumbent improvement threshold\n"
          << "  --native-tree-restart-min-remaining SEC  remaining-time trigger threshold\n"
          << "  --gap VALUE          relative MIP gap (default 1e-4)\n"
          << "  --max-nodes N        native B&C node limit\n"
          << "  --native-threads N   requested native B&C threads (default 1)\n"
          << "  --seed N             deterministic backend seed\n"
          << "  --seeds A,B,C        explicit distinct backend seeds\n"
          << "  --csv FILE           raw result CSV\n"
          << "  --json FILE          raw result and summary JSON\n";
      return false;
    } else {
      std::cerr << "Unknown option: " << arg << "\n";
      return false;
    }
  }
  if (saw_seed && saw_seeds) {
    std::cerr << "Use either --seed or --seeds, not both\n";
    return false;
  }
  if (cfg.seeds.empty()) cfg.seeds.push_back(cfg.seed);
  return true;
}

std::vector<fs::path> find_instances(const Config& cfg) {
  std::vector<fs::path> paths;
  if (!fs::exists(cfg.data_dir)) return paths;
  for (const auto& entry : fs::recursive_directory_iterator(cfg.data_dir)) {
    if (!entry.is_regular_file() || !is_mps_path(entry.path())) continue;
    if (matches_case(instance_name(entry.path()), cfg.case_filters)) {
      paths.push_back(entry.path());
    }
  }
  std::sort(paths.begin(), paths.end(), [](const fs::path& a, const fs::path& b) {
    return instance_name(a) < instance_name(b);
  });
  if (cfg.sample > 0 && static_cast<int>(paths.size()) > cfg.sample) {
    std::vector<fs::path> sampled;
    sampled.reserve(static_cast<std::size_t>(cfg.sample));
    if (cfg.sample == 1) {
      sampled.push_back(paths[paths.size() / 2]);
    } else {
      for (int i = 0; i < cfg.sample; ++i) {
        const std::size_t index = static_cast<std::size_t>(i) * (paths.size() - 1) /
                                  static_cast<std::size_t>(cfg.sample - 1);
        sampled.push_back(paths[index]);
      }
    }
    paths = std::move(sampled);
  }
  if (cfg.limit > 0 && static_cast<int>(paths.size()) > cfg.limit) {
    paths.resize(static_cast<std::size_t>(cfg.limit));
  }
  return paths;
}

std::map<std::string, Reference> read_references(const fs::path& path) {
  std::map<std::string, Reference> refs;
  if (path.empty()) return refs;
  std::ifstream in(path);
  if (!in) {
    std::cerr << "Warning: cannot open reference file " << path << "\n";
    return refs;
  }
  std::string line;
  while (std::getline(in, line)) {
    if (line.empty() || line[0] == '#') continue;
    std::istringstream row(line);
    std::string tag, name, value;
    row >> tag >> name >> value;
    if (tag.empty() || name.empty()) continue;
    Reference ref;
    ref.status = tag;
    if (!value.empty()) {
      try { ref.objective = std::stod(value); } catch (...) {}
    }
    refs[name] = ref;
  }
  return refs;
}

bool highs_finite_lower(double value) {
  return std::isfinite(value) && value > -0.5 * kHighsInf;
}

bool highs_finite_upper(double value) {
  return std::isfinite(value) && value < 0.5 * kHighsInf;
}

double native_lower(double value) {
  return highs_finite_lower(value) ? value : -1e20;
}

double native_upper(double value) {
  return highs_finite_upper(value) ? value : 1e20;
}

bool materialize_mps(const fs::path& source, fs::path& destination,
                     double& elapsed_ms, std::string& error) {
  if (source.extension() != ".gz") {
    destination = source;
    return true;
  }
  const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
  destination = fs::temp_directory_path() /
      ("miplib2017_" + instance_name(source) + "_" + std::to_string(stamp) + ".mps");
  const auto start = std::chrono::steady_clock::now();
  gzFile input = gzopen(source.string().c_str(), "rb");
  if (input == nullptr) {
    error = "cannot open gzip input";
    return false;
  }
  std::ofstream output(destination, std::ios::binary);
  if (!output) {
    gzclose(input);
    error = "cannot create temporary MPS";
    return false;
  }
  std::vector<char> buffer(1U << 20);
  int count = 0;
  while ((count = gzread(input, buffer.data(), static_cast<unsigned int>(buffer.size()))) > 0) {
    output.write(buffer.data(), count);
    if (!output) {
      error = "failed writing temporary MPS";
      break;
    }
  }
  if (count < 0 && error.empty()) {
    int code = Z_OK;
    const char* message = gzerror(input, &code);
    error = message != nullptr ? message : "gzip decompression failed";
  }
  gzclose(input);
  output.close();
  elapsed_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - start).count();
  if (!error.empty()) {
    std::error_code ec;
    fs::remove(destination, ec);
    return false;
  }
  return true;
}

Instance load_instance(const fs::path& path) {
  Instance instance;
  instance.name = instance_name(path);
  instance.path = path;
  if (!materialize_mps(path, instance.solver_path, instance.decompress_ms,
                       instance.error)) {
    return instance;
  }
  instance.solver_path_is_temporary = instance.solver_path != instance.path;

  Highs reader;
  reader.setOptionValue("output_flag", false);
  reader.setOptionValue("log_to_console", false);
  const auto t0 = std::chrono::steady_clock::now();
  const HighsStatus status = reader.readModel(instance.solver_path.string());
  const auto t1 = std::chrono::steady_clock::now();
  instance.load_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
  if (status == HighsStatus::kError) {
    instance.error = "HiGHS MPS read failed";
    return instance;
  }
  if (reader.getModel().isQp()) {
    instance.error = "quadratic model is outside MILP benchmark scope";
    return instance;
  }

  instance.source_lp = reader.getLp();
  instance.source_lp.ensureColwise();
  const HighsLp& source = instance.source_lp;
  instance.rows = static_cast<int>(source.num_row_);
  instance.columns = static_cast<int>(source.num_col_);
  instance.nonzeros = static_cast<std::int64_t>(source.a_matrix_.value_.size());
  instance.objective_offset = source.offset_;

  eng::LPModel& lp = instance.mip.linear_part;
  lp.sense = source.sense_ == ObjSense::kMaximize ? eng::Sense::Maximize
                                                  : eng::Sense::Minimize;
  lp.c = Eigen::VectorXd::Map(source.col_cost_.data(), instance.columns);
  lp.vars.resize(static_cast<std::size_t>(instance.columns));

  const bool has_integrality =
      source.integrality_.size() == static_cast<std::size_t>(instance.columns);
  for (int j = 0; j < instance.columns; ++j) {
    auto& var = lp.vars[static_cast<std::size_t>(j)];
    var.lb = native_lower(source.col_lower_[static_cast<std::size_t>(j)]);
    var.ub = native_upper(source.col_upper_[static_cast<std::size_t>(j)]);
    if (source.col_names_.size() == static_cast<std::size_t>(instance.columns)) {
      var.name = source.col_names_[static_cast<std::size_t>(j)];
    }
    const HighsVarType type = has_integrality
        ? source.integrality_[static_cast<std::size_t>(j)]
        : HighsVarType::kContinuous;
    if (type == HighsVarType::kSemiContinuous) {
      ++instance.semicontinuous;
      instance.native_supported = false;
      continue;
    }
    if (type == HighsVarType::kSemiInteger) {
      ++instance.semiinteger;
      ++instance.integers;
      instance.native_supported = false;
      continue;
    }
    if (type == HighsVarType::kInteger || type == HighsVarType::kImplicitInteger) {
      ++instance.integers;
      const bool binary = var.lb >= -kAuditTolerance &&
                          var.ub <= 1.0 + kAuditTolerance;
      if (binary) {
        var.type = eng::VarType::Binary;
        instance.mip.binary_idx.push_back(j);
        ++instance.binaries;
      } else {
        var.type = eng::VarType::Integer;
        instance.mip.integer_idx.push_back(j);
      }
    }
  }

  std::vector<int> ineq_row(static_cast<std::size_t>(instance.rows), -1);
  std::vector<int> eq_row(static_cast<std::size_t>(instance.rows), -1);
  int n_ineq = 0;
  int n_eq = 0;
  for (int i = 0; i < instance.rows; ++i) {
    const double lower = source.row_lower_[static_cast<std::size_t>(i)];
    const double upper = source.row_upper_[static_cast<std::size_t>(i)];
    if (highs_finite_lower(lower) && highs_finite_upper(upper) && lower == upper) {
      eq_row[static_cast<std::size_t>(i)] = n_eq++;
    } else {
      ineq_row[static_cast<std::size_t>(i)] = n_ineq++;
    }
  }

  std::vector<double> lhs(static_cast<std::size_t>(n_ineq));
  std::vector<double> rhs(static_cast<std::size_t>(n_ineq));
  std::vector<double> eq_rhs(static_cast<std::size_t>(n_eq));
  for (int i = 0; i < instance.rows; ++i) {
    if (eq_row[static_cast<std::size_t>(i)] >= 0) {
      eq_rhs[static_cast<std::size_t>(eq_row[static_cast<std::size_t>(i)])] =
          source.row_lower_[static_cast<std::size_t>(i)];
    } else {
      const int r = ineq_row[static_cast<std::size_t>(i)];
      lhs[static_cast<std::size_t>(r)] =
          native_lower(source.row_lower_[static_cast<std::size_t>(i)]);
      rhs[static_cast<std::size_t>(r)] =
          native_upper(source.row_upper_[static_cast<std::size_t>(i)]);
    }
  }

  std::vector<Eigen::Triplet<double>> ineq_triplets;
  std::vector<Eigen::Triplet<double>> eq_triplets;
  ineq_triplets.reserve(static_cast<std::size_t>(instance.nonzeros));
  eq_triplets.reserve(static_cast<std::size_t>(instance.nonzeros / 4));
  const auto& matrix = source.a_matrix_;
  for (int j = 0; j < instance.columns; ++j) {
    const HighsInt begin = matrix.start_[static_cast<std::size_t>(j)];
    const HighsInt end = matrix.start_[static_cast<std::size_t>(j + 1)];
    for (HighsInt p = begin; p < end; ++p) {
      const int source_row = static_cast<int>(matrix.index_[static_cast<std::size_t>(p)]);
      const double value = matrix.value_[static_cast<std::size_t>(p)];
      if (eq_row[static_cast<std::size_t>(source_row)] >= 0) {
        eq_triplets.emplace_back(eq_row[static_cast<std::size_t>(source_row)], j, value);
      } else {
        ineq_triplets.emplace_back(ineq_row[static_cast<std::size_t>(source_row)], j, value);
      }
    }
  }
  lp.A.resize(n_ineq, instance.columns);
  lp.A.setFromTriplets(ineq_triplets.begin(), ineq_triplets.end());
  lp.row_lhs = Eigen::VectorXd::Map(lhs.data(), n_ineq);
  lp.b = Eigen::VectorXd::Map(rhs.data(), n_ineq);
  lp.Aeq.resize(n_eq, instance.columns);
  lp.Aeq.setFromTriplets(eq_triplets.begin(), eq_triplets.end());
  lp.beq = Eigen::VectorXd::Map(eq_rhs.data(), n_eq);

  if (instance.integers == 0) {
    instance.error = "model has no integer variables";
  }
  return instance;
}

Audit audit_solution(const Instance& instance, const Eigen::VectorXd& x,
                     double reported_objective) {
  Audit audit;
  const HighsLp& lp = instance.source_lp;
  if (x.size() != static_cast<int>(lp.num_col_)) return audit;

  audit.max_row_violation = 0.0;
  audit.max_bound_violation = 0.0;
  audit.max_integrality_violation = 0.0;
  audit.objective = lp.offset_;
  std::vector<double> activity(static_cast<std::size_t>(lp.num_row_), 0.0);
  const bool has_integrality =
      lp.integrality_.size() == static_cast<std::size_t>(lp.num_col_);

  for (int j = 0; j < static_cast<int>(lp.num_col_); ++j) {
    const double value_j = x[j];
    audit.objective += lp.col_cost_[static_cast<std::size_t>(j)] * value_j;
    const HighsVarType type = has_integrality
        ? lp.integrality_[static_cast<std::size_t>(j)]
        : HighsVarType::kContinuous;
    const bool semi_zero =
        (type == HighsVarType::kSemiContinuous || type == HighsVarType::kSemiInteger) &&
        std::abs(value_j) <= kAuditTolerance;
    if (!semi_zero && highs_finite_lower(lp.col_lower_[static_cast<std::size_t>(j)])) {
      audit.max_bound_violation = std::max(
          audit.max_bound_violation,
          std::max(0.0, lp.col_lower_[static_cast<std::size_t>(j)] - value_j));
    }
    if (!semi_zero && highs_finite_upper(lp.col_upper_[static_cast<std::size_t>(j)])) {
      audit.max_bound_violation = std::max(
          audit.max_bound_violation,
          std::max(0.0, value_j - lp.col_upper_[static_cast<std::size_t>(j)]));
    }
    if (type == HighsVarType::kInteger || type == HighsVarType::kImplicitInteger ||
        type == HighsVarType::kSemiInteger) {
      audit.max_integrality_violation = std::max(
          audit.max_integrality_violation, std::abs(value_j - std::round(value_j)));
    }
    const HighsInt begin = lp.a_matrix_.start_[static_cast<std::size_t>(j)];
    const HighsInt end = lp.a_matrix_.start_[static_cast<std::size_t>(j + 1)];
    for (HighsInt p = begin; p < end; ++p) {
      const std::size_t pp = static_cast<std::size_t>(p);
      activity[static_cast<std::size_t>(lp.a_matrix_.index_[pp])] +=
          lp.a_matrix_.value_[pp] * value_j;
    }
  }
  for (int i = 0; i < static_cast<int>(lp.num_row_); ++i) {
    const double value = activity[static_cast<std::size_t>(i)];
    if (highs_finite_lower(lp.row_lower_[static_cast<std::size_t>(i)])) {
      audit.max_row_violation = std::max(
          audit.max_row_violation,
          std::max(0.0, lp.row_lower_[static_cast<std::size_t>(i)] - value));
    }
    if (highs_finite_upper(lp.row_upper_[static_cast<std::size_t>(i)])) {
      audit.max_row_violation = std::max(
          audit.max_row_violation,
          std::max(0.0, value - lp.row_upper_[static_cast<std::size_t>(i)]));
    }
  }
  if (std::isfinite(reported_objective)) {
    audit.objective_disagreement = std::abs(audit.objective - reported_objective) /
        std::max(1.0, std::abs(audit.objective));
  }
  audit.passed = audit.max_row_violation <= kAuditTolerance &&
                 audit.max_bound_violation <= kAuditTolerance &&
                 audit.max_integrality_violation <= kAuditTolerance &&
                 audit.objective_disagreement <= kAuditTolerance;
  return audit;
}

bool minimizes_objective(const Instance& instance) {
  return instance.source_lp.sense_ == ObjSense::kMinimize;
}

double normalized_progress_gap(const Instance& instance, double primal,
                               double dual) {
  if (!std::isfinite(primal) || !std::isfinite(dual)) return 1.0;
  const double numerator = minimizes_objective(instance)
      ? std::max(0.0, primal - dual)
      : std::max(0.0, dual - primal);
  const double denominator =
      std::max({1.0, std::abs(primal), std::abs(dual)});
  return std::clamp(numerator / denominator, 0.0, 1.0);
}

void audit_and_integrate_bound_events(const Instance& instance,
                                      const Config& cfg, Result& result) {
  result.primal_dual_integral_sec =
      std::numeric_limits<double>::quiet_NaN();
  if (!result.bound_event_stream_available) return;

  const double horizon_ms = 1000.0 * cfg.time_limit_sec;
  if (!(horizon_ms > 0.0) || !std::isfinite(horizon_ms)) {
    result.bound_event_stream_available = false;
    result.bound_event_stream_error = "invalid PDI horizon";
    return;
  }
  std::stable_sort(result.bound_events.begin(), result.bound_events.end(),
                   [](const BoundEvent& lhs, const BoundEvent& rhs) {
                     return lhs.time_ms < rhs.time_ms;
                   });

  double current_primal = std::numeric_limits<double>::quiet_NaN();
  double current_dual = std::numeric_limits<double>::quiet_NaN();
  double previous_ms = 0.0;
  double integral_ms = 0.0;
  const bool minimize = minimizes_objective(instance);
  const auto finite_abs = [](double value) {
    return std::isfinite(value) ? std::abs(value) : 0.0;
  };
  for (auto& event : result.bound_events) {
    event.time_ms = std::clamp(event.time_ms, 0.0, horizon_ms);
    integral_ms += (event.time_ms - previous_ms) *
                   normalized_progress_gap(instance, current_primal,
                                           current_dual);
    previous_ms = event.time_ms;

    if (std::isfinite(event.primal_bound)) {
      if (event.incumbent.size() != instance.columns) {
        result.bound_event_stream_available = false;
        result.bound_event_stream_error =
            "primal event has no complete incumbent vector";
        break;
      }
      event.incumbent_audit =
          audit_solution(instance, event.incumbent, event.primal_bound);
      if (!event.incumbent_audit.passed) {
        result.bound_event_stream_available = false;
        result.bound_event_stream_error =
            "intermediate incumbent failed original-space audit";
        break;
      }
      event.primal_bound = event.incumbent_audit.objective;
      const double tol = 1e-8 *
          std::max({1.0, finite_abs(current_primal),
                    finite_abs(event.primal_bound)});
      if (std::isfinite(current_primal) &&
          ((minimize && event.primal_bound > current_primal + tol) ||
           (!minimize && event.primal_bound < current_primal - tol))) {
        result.bound_event_stream_available = false;
        result.bound_event_stream_error = "primal event stream regressed";
        break;
      }
      current_primal = event.primal_bound;
    }

    if (std::isfinite(event.dual_bound)) {
      const double tol = 1e-8 *
          std::max({1.0, finite_abs(current_dual),
                    finite_abs(event.dual_bound)});
      if (std::isfinite(current_dual) &&
          ((minimize && event.dual_bound < current_dual - tol) ||
           (!minimize && event.dual_bound > current_dual + tol))) {
        result.bound_event_stream_available = false;
        result.bound_event_stream_error = "dual event stream regressed";
        break;
      }
      current_dual = event.dual_bound;
    }

    if (std::isfinite(current_primal) && std::isfinite(current_dual)) {
      const double consistency_tol = 1e-7 *
          std::max({1.0, std::abs(current_primal),
                    std::abs(current_dual)});
      if ((minimize && current_dual > current_primal + consistency_tol) ||
          (!minimize && current_dual < current_primal - consistency_tol)) {
        result.bound_event_stream_available = false;
        result.bound_event_stream_error =
            "primal/dual event bounds are inconsistent";
        break;
      }
    }
  }

  if (!result.bound_event_stream_available) return;
  integral_ms += (horizon_ms - previous_ms) *
                 normalized_progress_gap(instance, current_primal,
                                         current_dual);
  result.primal_dual_integral_sec = 0.001 * integral_ms;
}

void set_dimensions(const Instance& instance, Result& result) {
  result.instance = instance.name;
  result.rows = instance.rows;
  result.columns = instance.columns;
  result.nonzeros = instance.nonzeros;
  result.integers = instance.integers;
  result.binaries = instance.binaries;
  result.semicontinuous = instance.semicontinuous;
  result.semiinteger = instance.semiinteger;
  result.shared_decompress_ms = instance.decompress_ms;
}

Result run_highs(const Instance& instance, const Config& cfg) {
  Result result;
  set_dimensions(instance, result);
  result.solver = "highs-mip";
  result.collection_scope = "highs_summary";

  Highs::resetGlobalScheduler(true);
  Highs highs;
  highs.setOptionValue("output_flag", cfg.highs_verbose);
  highs.setOptionValue("log_to_console", cfg.highs_verbose);
  if (cfg.highs_verbose) highs.setOptionValue("mip_report_level", 2);
  highs.setOptionValue("threads", 1);
  highs.setOptionValue("random_seed", cfg.seed);
  highs.setOptionValue("time_limit", cfg.time_limit_sec);
  highs.setOptionValue("mip_rel_gap", cfg.gap);

  const auto read_start = std::chrono::steady_clock::now();
  const HighsStatus read_status = highs.readModel(instance.solver_path.string());
  const auto read_end = std::chrono::steady_clock::now();
  result.read_ms = std::chrono::duration<double, std::milli>(read_end - read_start).count();
  if (read_status == HighsStatus::kError) {
    result.status = "read error";
    return result;
  }

  const HighsStatus callback_status = highs.setCallback(
      [&result, &instance](int callback_type, const std::string&,
                           const HighsCallbackOutput* output,
                           HighsCallbackInput*, void*) {
        if (output == nullptr) return;
        if (callback_type == kCallbackMipImprovingSolution) {
          if (output->mip_solution.size() !=
              static_cast<std::size_t>(instance.columns)) {
            result.bound_event_stream_available = false;
            result.bound_event_stream_error =
                "HiGHS improving-solution callback dimension mismatch";
            return;
          }
          Eigen::VectorXd x(instance.columns);
          for (int j = 0; j < instance.columns; ++j) {
            x[j] = output->mip_solution[static_cast<std::size_t>(j)];
          }
          append_bound_event(result, 1000.0 * output->running_time,
                             output->mip_primal_bound,
                             output->mip_dual_bound, &x);
        } else if (callback_type == kCallbackMipLogging) {
          // Logging callbacks do not carry the incumbent vector. Record only
          // the independently meaningful dual-bound update.
          append_bound_event(result, 1000.0 * output->running_time,
                             std::numeric_limits<double>::quiet_NaN(),
                             output->mip_dual_bound);
        }
      });
  const HighsStatus improving_callback_status =
      callback_status == HighsStatus::kOk
          ? highs.startCallback(kCallbackMipImprovingSolution)
          : HighsStatus::kError;
  const HighsStatus logging_callback_status =
      callback_status == HighsStatus::kOk
          ? highs.startCallback(kCallbackMipLogging)
          : HighsStatus::kError;
  result.bound_event_stream_available =
      callback_status == HighsStatus::kOk &&
      improving_callback_status == HighsStatus::kOk &&
      logging_callback_status == HighsStatus::kOk;
  if (!result.bound_event_stream_available &&
      result.bound_event_stream_error.empty()) {
    result.bound_event_stream_error = "HiGHS MIP callbacks unavailable";
  }

  const auto solve_start = std::chrono::steady_clock::now();
  const HighsStatus run_status = highs.run();
  const auto solve_end = std::chrono::steady_clock::now();
  result.solve_ms =
      std::chrono::duration<double, std::milli>(solve_end - solve_start).count();
  const HighsModelStatus status = highs.getModelStatus();
  result.status = highs.modelStatusToString(status);
  result.optimal = status == HighsModelStatus::kOptimal;
  result.proven = result.optimal || status == HighsModelStatus::kInfeasible ||
                  status == HighsModelStatus::kUnbounded ||
                  status == HighsModelStatus::kUnboundedOrInfeasible;
  result.timed_out = status == HighsModelStatus::kTimeLimit;
  result.has_solution = highs.getSolution().value_valid &&
      highs.getInfo().primal_solution_status == kSolutionStatusFeasible;
  if (run_status == HighsStatus::kError && !result.has_solution) {
    result.status = "solve error: " + result.status;
  }

  const HighsInfo& info = highs.getInfo();
  result.node_count_available = true;
  result.lp_iteration_count_available = true;
  result.objective = info.objective_function_value;
  result.best_bound = info.mip_dual_bound;
  result.gap = info.mip_gap;
  result.nodes = info.mip_node_count;
  result.lp_iterations = static_cast<std::int64_t>(info.simplex_iteration_count) +
                         static_cast<std::int64_t>(info.ipm_iteration_count) +
                         static_cast<std::int64_t>(info.pdlp_iteration_count);
  if (result.has_solution) {
    const auto& values = highs.getSolution().col_value;
    result.x = Eigen::VectorXd::Zero(instance.columns);
    for (int j = 0; j < instance.columns; ++j) {
      result.x[j] = values[static_cast<std::size_t>(j)];
    }
  }
  if (result.bound_event_stream_available) {
    retain_terminally_certified_dual_events(instance, result,
                                            result.best_bound);
    append_bound_event(result, result.solve_ms,
                       result.has_solution ? result.objective
                                           : std::numeric_limits<double>::quiet_NaN(),
                       result.best_bound,
                       result.has_solution ? &result.x : nullptr);
  }
  return result;
}

Result run_cplex(const Instance& instance, const Config& cfg) {
  Result result;
  set_dimensions(instance, result);
  result.solver = "cplex-mip";
  result.collection_scope = "cplex_summary";
  if (!instance.native_supported) {
    result.available = false;
    result.status =
        "MIPModel cannot represent semi-continuous/semi-integer variables";
    return result;
  }

  eng::CplexOptions options;
  options.time_limit_sec = cfg.time_limit_sec;
  options.mip_gap = cfg.gap;
  options.threads = 1;
  options.random_seed = cfg.seed;
  eng::CplexAdapter cplex(options);
  if (!cplex.available()) {
    result.available = false;
  }
  const eng::SolveResult solve = cplex.solve_milp(instance.mip);
  const eng::CplexSolveInfo info = eng::last_cplex_solve_info();
  result.read_ms = 1000.0 * info.model_import_sec.value_or(0.0);
  result.solve_ms = 1000.0 * info.optimize_sec.value_or(0.0);
  result.status = solve.stats.status;
  result.has_solution = info.has_solution && solve.stats.success &&
                        solve.x.size() == instance.columns;
  result.optimal = info.optimal;
  result.proven = info.proven;
  result.timed_out = info.timed_out;
  result.gap = result.has_solution ? solve.stats.mip_gap
                                   : std::numeric_limits<double>::infinity();
  if (info.node_count.has_value()) {
    result.node_count_available = true;
    result.nodes = *info.node_count;
  }
  if (info.best_bound.has_value()) {
    result.best_bound = *info.best_bound + instance.objective_offset;
  }
  if (result.has_solution) {
    result.objective = solve.stats.objective + instance.objective_offset;
    result.x = solve.x;
  }
  return result;
}

#ifdef MIPSOLVERS_HAVE_SCIP_LIB
std::string scip_status_string(SCIP_STATUS status) {
  switch (status) {
    case SCIP_STATUS_OPTIMAL: return "optimal";
    case SCIP_STATUS_INFEASIBLE: return "infeasible";
    case SCIP_STATUS_UNBOUNDED: return "unbounded";
    case SCIP_STATUS_INFORUNBD: return "infeasible-or-unbounded";
    case SCIP_STATUS_TIMELIMIT: return "time-limit";
    case SCIP_STATUS_GAPLIMIT: return "gap-limit";
    case SCIP_STATUS_NODELIMIT: return "node-limit";
    case SCIP_STATUS_MEMLIMIT: return "memory-limit";
    case SCIP_STATUS_USERINTERRUPT: return "interrupted";
    default: return "status-" + std::to_string(static_cast<int>(status));
  }
}

struct ScipBoundEventData {
  Result* result{nullptr};
  const Instance* instance{nullptr};
  int filter_position{-1};
};

bool scip_original_solution_vector(SCIP* scip, SCIP_SOL* solution,
                                   const Instance& instance,
                                   Eigen::VectorXd& x) {
  if (scip == nullptr || solution == nullptr) return false;
  x = Eigen::VectorXd::Zero(instance.columns);
  std::unordered_map<std::string, int> source_index;
  source_index.reserve(static_cast<std::size_t>(instance.columns));
  if (instance.source_lp.col_names_.size() ==
      static_cast<std::size_t>(instance.columns)) {
    for (int j = 0; j < instance.columns; ++j) {
      source_index.emplace(
          instance.source_lp.col_names_[static_cast<std::size_t>(j)], j);
    }
  }

  SCIP_VAR** variables = SCIPgetOrigVars(scip);
  const int variable_count = SCIPgetNOrigVars(scip);
  if (variables == nullptr || variable_count <= 0) return false;
  std::vector<char> assigned(static_cast<std::size_t>(instance.columns), 0);
  for (int i = 0; i < variable_count; ++i) {
    int index = -1;
    const char* name = SCIPvarGetName(variables[i]);
    if (name != nullptr) {
      const auto it = source_index.find(name);
      if (it != source_index.end()) index = it->second;
    }
    if (index < 0 && variable_count == instance.columns) index = i;
    if (index < 0 || index >= instance.columns) continue;
    x[index] = SCIPgetSolVal(scip, solution, variables[i]);
    assigned[static_cast<std::size_t>(index)] = 1;
  }
  return std::all_of(assigned.begin(), assigned.end(),
                     [](char value) { return value != 0; });
}

static SCIP_DECL_EVENTEXEC(scip_bound_event_exec) {
  auto* data = reinterpret_cast<ScipBoundEventData*>(
      SCIPeventhdlrGetData(eventhdlr));
  if (data == nullptr || data->result == nullptr || data->instance == nullptr) {
    return SCIP_OKAY;
  }
  const SCIP_EVENTTYPE type = SCIPeventGetType(event);
  const double time_ms = 1000.0 * SCIPgetSolvingTime(scip);
  const double dual_bound = SCIPgetDualbound(scip);
  if ((type & SCIP_EVENTTYPE_BESTSOLFOUND) != 0) {
    SCIP_SOL* solution = SCIPgetBestSol(scip);
    Eigen::VectorXd x;
    if (solution == nullptr ||
        !scip_original_solution_vector(scip, solution, *data->instance, x)) {
      data->result->bound_event_stream_available = false;
      data->result->bound_event_stream_error =
          "SCIP best-solution event could not reconstruct original columns";
      return SCIP_OKAY;
    }
    append_bound_event(*data->result, time_ms,
                       SCIPgetSolOrigObj(scip, solution), dual_bound, &x);
  } else if ((type & SCIP_EVENTTYPE_DUALBOUNDIMPROVED) != 0) {
    append_bound_event(*data->result, time_ms,
                       std::numeric_limits<double>::quiet_NaN(), dual_bound);
  }
  return SCIP_OKAY;
}

static SCIP_DECL_EVENTINITSOL(scip_bound_event_initsol) {
  auto* data = reinterpret_cast<ScipBoundEventData*>(
      SCIPeventhdlrGetData(eventhdlr));
  if (data == nullptr) return SCIP_INVALIDDATA;
  data->filter_position = -1;
  return SCIPcatchEvent(
      scip, SCIP_EVENTTYPE_BESTSOLFOUND | SCIP_EVENTTYPE_DUALBOUNDIMPROVED,
      eventhdlr, nullptr, &data->filter_position);
}

static SCIP_DECL_EVENTEXITSOL(scip_bound_event_exitsol) {
  auto* data = reinterpret_cast<ScipBoundEventData*>(
      SCIPeventhdlrGetData(eventhdlr));
  if (data != nullptr && data->filter_position >= 0) {
    const SCIP_RETCODE code = SCIPdropEvent(
        scip, SCIP_EVENTTYPE_BESTSOLFOUND | SCIP_EVENTTYPE_DUALBOUNDIMPROVED,
        eventhdlr, nullptr, data->filter_position);
    data->filter_position = -1;
    return code;
  }
  return SCIP_OKAY;
}
#endif

Result run_scip(const Instance& instance, const Config& cfg) {
  Result result;
  set_dimensions(instance, result);
  result.solver = "scip-mip";
#ifndef MIPSOLVERS_HAVE_SCIP_LIB
  result.available = false;
  result.status = "SCIP library not compiled";
  return result;
#else
  result.collection_scope = "scip_summary";
  SCIP* scip = nullptr;
  if (SCIPcreate(&scip) != SCIP_OKAY || scip == nullptr) {
    result.status = "SCIPcreate failed";
    return result;
  }
  SCIPincludeDefaultPlugins(scip);
  SCIPsetIntParam(scip, "display/verblevel", 0);
  SCIPsetIntParam(scip, "parallel/maxnthreads", 1);
  SCIPsetIntParam(scip, "randomization/randomseedshift", cfg.seed);
  SCIPsetIntParam(scip, "randomization/permutationseed", cfg.seed);
  SCIPsetRealParam(scip, "limits/time", cfg.time_limit_sec);
  SCIPsetRealParam(scip, "limits/gap", cfg.gap);

  ScipBoundEventData progress_data{&result, &instance, -1};
  SCIP_EVENTHDLR* progress_handler = nullptr;
  const SCIP_RETCODE include_progress = SCIPincludeEventhdlrBasic(
      scip, &progress_handler, "miplib_progress_audit",
      "records auditable primal/dual bound events", scip_bound_event_exec,
      reinterpret_cast<SCIP_EVENTHDLRDATA*>(&progress_data));
  SCIP_RETCODE init_progress = SCIP_ERROR;
  SCIP_RETCODE exit_progress = SCIP_ERROR;
  if (include_progress == SCIP_OKAY && progress_handler != nullptr) {
    init_progress = SCIPsetEventhdlrInitsol(
        scip, progress_handler, scip_bound_event_initsol);
    exit_progress = SCIPsetEventhdlrExitsol(
        scip, progress_handler, scip_bound_event_exitsol);
  }
  result.bound_event_stream_available =
      include_progress == SCIP_OKAY && init_progress == SCIP_OKAY &&
      exit_progress == SCIP_OKAY;
  if (!result.bound_event_stream_available) {
    result.bound_event_stream_error = "SCIP event handler unavailable";
  }

  const auto read_start = std::chrono::steady_clock::now();
  const SCIP_RETCODE read_status =
      SCIPreadProb(scip, instance.solver_path.string().c_str(), nullptr);
  const auto read_end = std::chrono::steady_clock::now();
  result.read_ms = std::chrono::duration<double, std::milli>(read_end - read_start).count();
  if (read_status != SCIP_OKAY) {
    result.status = "SCIP MPS read failed (rc=" +
                    std::to_string(static_cast<int>(read_status)) + ")";
    SCIPfree(&scip);
    return result;
  }

  const auto solve_start = std::chrono::steady_clock::now();
  const SCIP_RETCODE solve_status = SCIPsolve(scip);
  const auto solve_end = std::chrono::steady_clock::now();
  result.solve_ms =
      std::chrono::duration<double, std::milli>(solve_end - solve_start).count();
  const SCIP_STATUS status = SCIPgetStatus(scip);
  result.status = scip_status_string(status);
  result.optimal = status == SCIP_STATUS_OPTIMAL;
  // SCIP's GAPLIMIT and HiGHS' kOptimal both certify the configured relative
  // MIP gap. Normalize backend status vocabularies as derived in
  // docs/miplib2017_benchmark_protocol_2026-08-25.md,
  // "Pilot mismatch: gap-certificate normalization".
  result.proven = result.optimal || status == SCIP_STATUS_GAPLIMIT ||
                  status == SCIP_STATUS_INFEASIBLE ||
                  status == SCIP_STATUS_UNBOUNDED ||
                  status == SCIP_STATUS_INFORUNBD;
  result.timed_out = status == SCIP_STATUS_TIMELIMIT;
  result.node_count_available = true;
  result.lp_solve_count_available = true;
  result.lp_iteration_count_available = true;
  result.cut_count_available = true;
  result.nodes = static_cast<std::int64_t>(SCIPgetNNodes(scip));
  result.lp_solves = static_cast<std::int64_t>(SCIPgetNLPs(scip));
  result.lp_iterations = static_cast<std::int64_t>(SCIPgetNLPIterations(scip));
  result.cuts = SCIPgetNCutsApplied(scip);
  result.best_bound = SCIPgetDualbound(scip);
  result.gap = SCIPgetGap(scip);

  SCIP_SOL* solution = SCIPgetBestSol(scip);
  result.has_solution = solution != nullptr && SCIPgetNSols(scip) > 0;
  if (result.has_solution) {
    result.objective = SCIPgetSolOrigObj(scip, solution);
    if (!scip_original_solution_vector(scip, solution, instance, result.x)) {
      result.has_solution = false;
      result.objective = std::numeric_limits<double>::quiet_NaN();
      result.status = "SCIP final solution column reconstruction failed";
    }
  }
  if (solve_status != SCIP_OKAY && !result.has_solution) {
    result.status = "solve error: " + result.status;
  }
  SCIPfree(&scip);
  if (result.bound_event_stream_available) {
    // SCIP may deliver final solving-stage callbacks while SCIPfree() runs.
    // Apply the terminal certificate filter only after that lifecycle is over.
    retain_terminally_certified_dual_events(instance, result,
                                            result.best_bound);
    append_bound_event(result, result.solve_ms,
                       result.has_solution ? result.objective
                                           : std::numeric_limits<double>::quiet_NaN(),
                       result.best_bound,
                       result.has_solution ? &result.x : nullptr);
  }
  return result;
#endif
}

Result run_native(const Instance& instance, const Config& cfg,
                  bool experimental_native_lp) {
  Result result;
  set_dimensions(instance, result);
  result.solver = experimental_native_lp ? "native-native-lp" : "native-highs-lp";
  result.read_ms = instance.load_ms;
  if (!instance.native_supported) {
    result.available = false;
    result.status = "native model cannot represent semi-continuous/semi-integer variables";
    return result;
  }

  eng::BCOptions options;
  options.lp_kernel_backend = experimental_native_lp
      ? eng::LpKernelBackend::ExperimentalNative
      : eng::LpKernelBackend::HiGHS;
  // Achterberg (2007), Secs. 4.1-4.2; derivation and proof-ownership gate in
  // docs/native_milp_cutpool_proof_ownership_2026-08-13.md.
  options.auto_highs_root_pipeline = !experimental_native_lp;
  // HiGHS owns its coupled presolve/root state machine; the native solver
  // imports the audited root state and retains ownership of tree search.
  // Derivation: docs/native_milp_root_quality_restart_prerequisites_2026-08-13.md,
  // "Coupled HiGHS Root-Primal Ownership".
  options.highs_root_native_tree_contract = !experimental_native_lp;
  // Achterberg (2007), Sec. 9.4; Berthold (2006), Sec. 3.5; bounded RENS
  // contract in docs/native_milp_root_quality_restart_prerequisites_2026-08-13.md.
  options.enable_root_low_fractionality_rens = !experimental_native_lp;
  options.time_limit_sec = cfg.time_limit_sec;
  options.gap_tol = cfg.gap;
  options.max_nodes = cfg.max_nodes;
  options.num_threads = cfg.native_threads;
  options.random_seed = static_cast<unsigned long long>(cfg.seed);
  options.verbose = cfg.native_verbose;
  options.enable_domain_heuristics = false;
  options.use_papilo_presolve = cfg.native_papilo_presolve;
  options.native_presolve_probing = cfg.native_presolve_probing;
  options.probe_max_candidates = cfg.native_probe_max;
  options.probe_reliability = cfg.native_probe_reliability;
  if (cfg.native_audit_hint_only) {
    options.accept_verified_warm_start_incumbent = false;
    options.incumbent_quality_reject_factor = 0.1;
  }
  options.tree_restart.enabled = cfg.native_tree_restart;
  options.tree_restart.max_restarts = cfg.native_tree_restart_max;
  options.tree_restart.min_nodes_since_restart =
      cfg.native_tree_restart_min_nodes;
  options.tree_restart.min_open_nodes =
      cfg.native_tree_restart_min_open_nodes;
  options.tree_restart.min_relative_incumbent_improvement =
      cfg.native_tree_restart_min_improvement;
  options.tree_restart.min_remaining_time_sec =
      cfg.native_tree_restart_min_remaining_sec;
  if (!cfg.native_cuts) {
    options.cuts = eng::CutType::None;
    options.root_cut_rounds = 0;
    options.max_cut_depth = -1;
    options.enable_objective_cutoff_conflict_cuts = false;
    options.enable_objective_cutoff_weighted_event_cuts = false;
    options.enable_objective_cutoff_domain_fixing = false;
    options.enable_nonviolated_cutoff_conflict_covers = false;
    options.enable_graph_implied_bound_cuts = false;
    options.enable_dynamic_implied_bound_probing = false;
    options.enable_cglp_cuts = false;
  }
  if (!cfg.native_objective_propagation) {
    options.enable_objective_cutoff_conflict_cuts = false;
    options.enable_objective_cutoff_weighted_event_cuts = false;
    options.enable_objective_cutoff_domain_fixing = false;
    options.enable_nonviolated_cutoff_conflict_covers = false;
  }
  if (!cfg.native_reduced_cost_fixing) {
    options.rc_fixing_followup_propagation = false;
    options.enable_reduced_cost_conflict_learning = false;
    options.enable_reduced_cost_fixing = false;
    options.enable_reduced_cost_proof_conflict_minimization = false;
    options.enable_verified_reduced_cost_conflict_minimization = false;
    options.enable_reduced_cost_fixing_resolve = false;
    options.enable_reduced_cost_proof_cut_resolve = false;
  }
  options.bound_propagation_rounds = cfg.native_row_propagation_rounds;
  options.node_estimate_aggregation =
      cfg.native_node_estimate == "maximum"
          ? eng::NodeEstimateAggregation::Maximum
          : eng::NodeEstimateAggregation::Sum;

  eng::BCWarmStart warm_start;
  if (!cfg.native_primal_hint_file.empty()) {
    std::ifstream hint_stream(cfg.native_primal_hint_file);
    if (!hint_stream) {
      result.available = false;
      result.status = "cannot read native primal hint: " +
                      cfg.native_primal_hint_file.string();
      return result;
    }
    json hint_json;
    hint_stream >> hint_json;
    const json* values = &hint_json;
    if (hint_json.is_object()) {
      const auto x_it = hint_json.find("x");
      if (x_it == hint_json.end()) {
        result.available = false;
        result.status = "native primal hint JSON object has no x field";
        return result;
      }
      values = &*x_it;
    }
    if (!values->is_array() ||
        values->size() != static_cast<std::size_t>(instance.columns)) {
      result.available = false;
      result.status = "native primal hint dimension mismatch";
      return result;
    }
    eng::BCPrimalHint hint;
    hint.x.resize(instance.columns);
    for (int column = 0; column < instance.columns; ++column) {
      hint.x[column] = (*values)[static_cast<std::size_t>(column)].get<double>();
    }
    warm_start.primal_hints.push_back(std::move(hint));
  }

  const auto solve_start = std::chrono::steady_clock::now();
  const eng::BCResult native = warm_start.empty()
      ? eng::solve_milp_bc(instance.mip, options)
      : eng::solve_milp_bc(instance.mip, options, warm_start,
                           eng::BCCallbacks{});
  const auto solve_end = std::chrono::steady_clock::now();
  result.solve_ms =
      std::chrono::duration<double, std::milli>(solve_end - solve_start).count();
  result.status = native.bc_stats.status.empty() ? native.stats.status
                                                  : native.bc_stats.status;
  result.collection_scope = native.bc_stats.collection_scope;
  result.node_count_available = true;
  result.lp_solve_count_available = native.bc_stats.lp_solve_count_available;
  result.cut_count_available = native.bc_stats.cut_diagnostics_available;
  result.incumbent_timeline_available =
      native.bc_stats.incumbent_timeline_available;
  result.bound_event_stream_available =
      native.bc_stats.bound_event_stream_available;
  result.bound_events_dropped_uncertified_dual =
      native.bc_stats.bound_events_dropped_uncertified_dual;
  result.native_diagnostics_available =
      native.bc_stats.native_diagnostics_available;
  result.parallel_requested_threads =
      native.bc_stats.parallel_requested_threads;
  result.parallel_effective_threads =
      native.bc_stats.parallel_effective_threads;
  result.parallel_explorer_threads =
      native.bc_stats.parallel_explorer_threads;
  result.parallel_tree_launched = native.bc_stats.parallel_tree_launched;
  result.parallel_schedule_reason =
      native.bc_stats.parallel_schedule_reason;
  result.has_solution = native.stats.success && native.x.size() == instance.columns;
  result.nodes = native.bc_stats.nodes_explored;
  result.lp_solves = native.bc_stats.lp_solve_count_available
      ? native.bc_stats.lp_solves : -1;
  result.cuts = native.bc_stats.cut_diagnostics_available
      ? native.bc_stats.cuts_added : -1;
  result.first_incumbent_node = native.bc_stats.incumbent_timeline_available
      ? native.bc_stats.first_incumbent_node : -1;
  result.first_incumbent_lp_solves =
      native.bc_stats.incumbent_timeline_available
          ? native.bc_stats.first_incumbent_lp_solves : -1;
  result.native_presolve_attempted =
      native.bc_stats.native_presolve_attempted;
  result.native_presolve_adopted = native.bc_stats.native_presolve_adopted;
  result.native_presolve_orig_rows =
      native.bc_stats.native_presolve_orig_rows;
  result.native_presolve_final_rows =
      native.bc_stats.native_presolve_final_rows;
  result.native_presolve_orig_cols =
      native.bc_stats.native_presolve_orig_cols;
  result.native_presolve_final_cols =
      native.bc_stats.native_presolve_final_cols;
  result.native_presolve_time_ms = native.bc_stats.native_presolve_time_ms;
  result.native_presolve_probing_trail_pushes =
      native.bc_stats.native_presolve_probing_trail_pushes;
  result.native_presolve_probing_rows_processed =
      native.bc_stats.native_presolve_probing_rows_processed;
  result.native_presolve_probing_implications_learned =
      native.bc_stats.native_presolve_probing_implications_learned;
  result.native_presolve_probing_implications_imported =
      native.bc_stats.native_presolve_probing_implications_imported;
  result.native_presolve_probing_max_touched_cols =
      native.bc_stats.native_presolve_probing_max_touched_cols;
  result.native_presolve_probing_truncated =
      native.bc_stats.native_presolve_probing_truncated;
  result.node_queue_domain_compactions =
      native.bc_stats.node_queue_domain_compactions;
  result.node_queue_domain_materializations =
      native.bc_stats.node_queue_domain_materializations;
  result.node_queue_dense_bound_values_released =
      native.bc_stats.node_queue_dense_bound_values_released;
  result.node_queue_compact_entries_created =
      native.bc_stats.node_queue_compact_entries_created;
  result.node_queue_terminal_compact_nodes =
      native.bc_stats.node_queue_terminal_compact_nodes;
  result.node_queue_terminal_compact_entries =
      native.bc_stats.node_queue_terminal_compact_entries;
  result.node_queue_peak_compact_nodes =
      native.bc_stats.node_queue_peak_compact_nodes;
  result.node_queue_peak_compact_entries =
      native.bc_stats.node_queue_peak_compact_entries;
  result.node_queue_domain_compaction_failures =
      native.bc_stats.node_queue_domain_compaction_failures;
  result.node_queue_domain_materialization_failures =
      native.bc_stats.node_queue_domain_materialization_failures;
  result.node_queue_serial_compactions =
      native.bc_stats.node_queue_serial_compactions;
  result.node_queue_parallel_compactions =
      native.bc_stats.node_queue_parallel_compactions;
  result.node_queue_submip_compactions =
      native.bc_stats.node_queue_submip_compactions;
  result.branch_payload_child_creations =
      native.bc_stats.branch_payload_child_creations;
  result.branch_payload_shared_vectors =
      native.bc_stats.branch_payload_shared_vectors;
  result.branch_payload_shared_elements =
      native.bc_stats.branch_payload_shared_elements;
  result.branch_domain_dense_copies =
      native.bc_stats.branch_domain_dense_copies;
  result.branch_domain_dense_values_copied =
      native.bc_stats.branch_domain_dense_values_copied;
  result.branch_domain_moves = native.bc_stats.branch_domain_moves;
  result.root_domain_probe_workspace_initializations =
      native.bc_stats.root_domain_probe_workspace_initializations;
  result.root_domain_probe_worlds = native.bc_stats.root_domain_probe_worlds;
  result.root_domain_probe_trail_pushes =
      native.bc_stats.root_domain_probe_trail_pushes;
  result.root_domain_probe_rollbacks =
      native.bc_stats.root_domain_probe_rollbacks;
  result.root_domain_probe_failures =
      native.bc_stats.root_domain_probe_failures;
  result.root_domain_probe_rows_processed =
      native.bc_stats.root_domain_probe_rows_processed;
  result.root_domain_probe_changed_columns =
      native.bc_stats.root_domain_probe_changed_columns;
  result.root_domain_probe_implication_export_columns_visited =
      native.bc_stats.root_domain_probe_implication_export_columns_visited;
  result.root_domain_probe_implications_learned =
      native.bc_stats.root_domain_probe_implications_learned;
  result.root_domain_probe_committed_bound_changes =
      native.bc_stats.root_domain_probe_committed_bound_changes;
  result.root_lp_probe_bound_workspace_initializations =
      native.bc_stats.root_lp_probe_bound_workspace_initializations;
  result.root_lp_probe_bound_transactions =
      native.bc_stats.root_lp_probe_bound_transactions;
  result.root_lp_probe_transaction_snapshot_values =
      native.bc_stats.root_lp_probe_transaction_snapshot_values;
  result.root_lp_probe_transaction_rollbacks =
      native.bc_stats.root_lp_probe_transaction_rollbacks;
  result.root_lp_probe_transaction_failures =
      native.bc_stats.root_lp_probe_transaction_failures;
  result.root_lp_probe_backend_cold_solves =
      native.bc_stats.root_lp_probe_backend_cold_solves;
  result.root_lp_probe_backend_persistent_resolves =
      native.bc_stats.root_lp_probe_backend_persistent_resolves;
  result.strong_probe_base_sf_materializations =
      native.bc_stats.strong_probe_base_sf_materializations;
  result.strong_probe_bound_transactions =
      native.bc_stats.strong_probe_bound_transactions;
  result.strong_probe_transaction_snapshot_values =
      native.bc_stats.strong_probe_transaction_snapshot_values;
  result.strong_probe_transaction_rollbacks =
      native.bc_stats.strong_probe_transaction_rollbacks;
  result.strong_probe_transaction_failures =
      native.bc_stats.strong_probe_transaction_failures;
  result.strong_probe_backend_cold_solves =
      native.bc_stats.strong_probe_backend_cold_solves;
  result.strong_probe_backend_persistent_resolves =
      native.bc_stats.strong_probe_backend_persistent_resolves;
  result.separator_sparse_candidates_created =
      native.bc_stats.separator_sparse_candidates_created;
  result.separator_sparse_candidate_entries_created =
      native.bc_stats.separator_sparse_candidate_entries_created;
  result.separator_peak_live_sparse_candidates =
      native.bc_stats.separator_peak_live_sparse_candidates;
  result.separator_peak_live_sparse_entries =
      native.bc_stats.separator_peak_live_sparse_entries;
  result.separator_sparse_aggregation_snapshots =
      native.bc_stats.separator_sparse_aggregation_snapshots;
  result.separator_sparse_aggregation_entries =
      native.bc_stats.separator_sparse_aggregation_entries;
  result.separator_dense_workspace_materializations =
      native.bc_stats.separator_dense_workspace_materializations;
  result.separator_dense_workspace_values =
      native.bc_stats.separator_dense_workspace_values;
  result.separator_matrix_append_calls =
      native.bc_stats.separator_matrix_append_calls;
  result.separator_matrix_appended_rows =
      native.bc_stats.separator_matrix_appended_rows;
  result.separator_matrix_appended_entries =
      native.bc_stats.separator_matrix_appended_entries;
  result.separator_matrix_prior_entries_bypassing_triplet_rebuild =
      native.bc_stats
          .separator_matrix_prior_entries_bypassing_triplet_rebuild;
  result.separator_matrix_storage_reallocations =
      native.bc_stats.separator_matrix_storage_reallocations;
  result.separator_matrix_peak_spare_entries =
      native.bc_stats.separator_matrix_peak_spare_entries;
  result.tree_restarts = native.bc_stats.tree_restarts;
  result.tree_restart_nodes_discarded =
      native.bc_stats.tree_restart_nodes_discarded;
  result.tree_restart_root_requeues =
      native.bc_stats.tree_restart_root_requeues;
  result.tree_restart_cut_pool_rows_preserved =
      native.bc_stats.tree_restart_cut_pool_rows_preserved;
  result.tree_restart_conflicts_preserved =
      native.bc_stats.tree_restart_conflicts_preserved;
  result.tree_restart_implications_preserved =
      native.bc_stats.tree_restart_implications_preserved;
  result.tree_restart_clique_edges_preserved =
      native.bc_stats.tree_restart_clique_edges_preserved;
  result.tree_restart_pseudocost_observations_preserved =
      native.bc_stats.tree_restart_pseudocost_observations_preserved;
  result.tree_restart_last_node = native.bc_stats.tree_restart_last_node;
  result.tree_restart_last_source =
      native.bc_stats.tree_restart_last_source;
  result.reliability_branch_nodes = native.bc_stats.reliability_branch_nodes;
  result.strong_branch_candidates = native.bc_stats.strong_branch_candidates;
  result.strong_branch_lp_solves = native.bc_stats.strong_branch_lp_solves;
  result.strong_branch_cache_exact_hits =
      native.bc_stats.strong_branch_cache_exact_hits;
  result.strong_branch_cache_warm_hits =
      native.bc_stats.strong_branch_cache_warm_hits;
  result.strong_branch_duplicate_lp_avoided =
      native.bc_stats.strong_branch_duplicate_lp_avoided;
  result.branching_regret_samples = native.bc_stats.branching_regret_samples;
  result.branching_regret_sum = native.bc_stats.branching_regret_sum;
  result.branching_regret_max = native.bc_stats.branching_regret_max;
  result.strong_branch_regret_samples =
      native.bc_stats.strong_branch_regret_samples;
  result.strong_branch_regret_sum = native.bc_stats.strong_branch_regret_sum;
  result.strong_branch_regret_max = native.bc_stats.strong_branch_regret_max;
  result.node_estimate_calibration_samples =
      native.bc_stats.node_estimate_calibration_samples;
  result.node_estimate_predicted_lift_sum =
      native.bc_stats.node_estimate_predicted_lift_sum;
  result.node_estimate_realized_lift_sum =
      native.bc_stats.node_estimate_realized_lift_sum;
  result.node_estimate_abs_error_sum =
      native.bc_stats.node_estimate_abs_error_sum;
  result.node_estimate_squared_error_sum =
      native.bc_stats.node_estimate_squared_error_sum;
  result.node_estimate_predicted_sq_sum =
      native.bc_stats.node_estimate_predicted_sq_sum;
  result.node_estimate_realized_sq_sum =
      native.bc_stats.node_estimate_realized_sq_sum;
  result.node_estimate_cross_sum = native.bc_stats.node_estimate_cross_sum;
  result.directional_calibration_samples =
      native.bc_stats.directional_calibration_samples;
  result.directional_predicted_gain_sum =
      native.bc_stats.directional_predicted_gain_sum;
  result.directional_realized_gain_sum =
      native.bc_stats.directional_realized_gain_sum;
  result.directional_abs_error_sum =
      native.bc_stats.directional_abs_error_sum;
  result.directional_squared_error_sum =
      native.bc_stats.directional_squared_error_sum;
  result.directional_rank_samples = native.bc_stats.directional_rank_samples;
  result.directional_rank_concordant =
      native.bc_stats.directional_rank_concordant;
  result.fallback_events = native.bc_stats.fallback_events;
  result.fallback_recoveries = native.bc_stats.fallback_recoveries;
  result.gap = native.bc_stats.gap;
  result.timed_out = result.status.find("Time") != std::string::npos ||
                     result.status.find("time") != std::string::npos;
  if (result.has_solution) {
    result.x = native.x;
    result.objective = native.stats.objective + instance.objective_offset;
  }
  if (std::isfinite(native.bc_stats.best_bound) &&
      std::abs(native.bc_stats.best_bound) < 1e29) {
    result.best_bound = native.bc_stats.best_bound + instance.objective_offset;
  }
  if (result.bound_event_stream_available) {
    for (const auto& event : native.bc_stats.bound_events) {
      const bool has_event_incumbent =
          event.incumbent.size() == instance.columns;
      const double primal =
          has_event_incumbent && std::isfinite(event.primal_bound)
              ? event.primal_bound + instance.objective_offset
              : std::numeric_limits<double>::quiet_NaN();
      const double dual = std::isfinite(event.dual_bound)
          ? event.dual_bound + instance.objective_offset
          : std::numeric_limits<double>::quiet_NaN();
      append_bound_event(result, 1000.0 * event.time_sec, primal, dual,
                         has_event_incumbent ? &event.incumbent : nullptr);
    }
    if (result.bound_events.empty()) {
      result.bound_event_stream_available = false;
      result.bound_event_stream_error = "Native bound-event stream is empty";
    }
  } else {
    result.bound_event_stream_error = "Native bound-event stream unavailable";
  }
  result.optimal = result.has_solution && std::isfinite(result.gap) &&
                   result.gap <= cfg.gap * (1.0 + 1e-6);
  result.proven = result.optimal || result.status.find("Infeasible") != std::string::npos ||
                  result.status.find("infeasible") != std::string::npos ||
                  result.status.find("Unbounded") != std::string::npos ||
                  result.status.find("unbounded") != std::string::npos;
  return result;
}

Result run_native_presolve_highs_audit(const Instance& instance,
                                       const Config& cfg) {
  Result result;
  set_dimensions(instance, result);
  result.solver = "native-presolve-highs-audit";
  result.collection_scope = "native_presolve_audit";
  result.native_diagnostics_available = true;
  result.read_ms = instance.load_ms;
  if (!instance.native_supported) {
    result.available = false;
    result.status =
        "native model cannot represent semi-continuous/semi-integer variables";
    return result;
  }

  eng::MIPModel reduced = instance.mip;
  eng::PresolveOptions presolve_options;
  presolve_options.do_probing = cfg.native_presolve_probing;
  eng::MILPPresolve presolve(presolve_options);
  const auto start = std::chrono::steady_clock::now();
  const eng::PresolveStats stats =
      presolve.run(reduced.linear_part, reduced.binary_idx, reduced.integer_idx);
  result.native_presolve_attempted = true;
  result.native_presolve_adopted = !stats.infeasible;
  result.native_presolve_orig_rows = stats.orig_rows;
  result.native_presolve_final_rows = stats.final_rows;
  result.native_presolve_orig_cols = stats.orig_cols;
  result.native_presolve_final_cols = stats.final_cols;
  result.native_presolve_time_ms = 1000.0 * stats.presolve_time_sec;
  result.native_presolve_probing_trail_pushes = stats.probing_trail_pushes;
  result.native_presolve_probing_rows_processed = stats.probing_rows_processed;
  result.native_presolve_probing_implications_learned =
      stats.probing_implications_learned;
  result.native_presolve_probing_max_touched_cols =
      stats.probing_max_touched_cols;
  result.native_presolve_probing_truncated = stats.probing_truncated;
  if (stats.infeasible) {
    result.status = "Native presolve declared infeasible: " +
                    stats.infeasibility_reason;
    result.solve_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start).count();
    return result;
  }

  Eigen::VectorXd reduced_x;
  bool reduced_optimal = false;
  bool reduced_proven = false;
  if (reduced.linear_part.vars.empty()) {
    reduced_x.resize(0);
    reduced_optimal = true;
    reduced_proven = true;
    result.status = "Native presolve produced an empty model";
  } else {
    eng::BCOptions options;
    options.strict_highs_mip_contract = true;
    options.lp_kernel_backend = eng::LpKernelBackend::HiGHS;
    options.use_papilo_presolve = false;
    options.time_limit_sec = cfg.time_limit_sec;
    options.gap_tol = cfg.gap;
    options.max_nodes = cfg.max_nodes;
    options.num_threads = cfg.native_threads;
    const eng::BCResult solved = eng::solve_milp_bc(reduced, options);
    result.status = solved.bc_stats.status.empty() ? solved.stats.status
                                                   : solved.bc_stats.status;
    result.nodes = solved.bc_stats.nodes_explored;
    result.node_count_available = true;
    result.lp_solves = solved.bc_stats.lp_solve_count_available
                           ? solved.bc_stats.lp_solves
                           : -1;
    result.lp_solve_count_available = solved.bc_stats.lp_solve_count_available;
    result.gap = solved.bc_stats.gap;
    reduced_optimal = solved.stats.success && std::isfinite(result.gap) &&
                      result.gap <= cfg.gap * (1.0 + 1e-6);
    reduced_proven = reduced_optimal ||
                     result.status.find("Infeasible") != std::string::npos ||
                     result.status.find("infeasible") != std::string::npos;
    if (solved.stats.success &&
        solved.x.size() == static_cast<int>(reduced.linear_part.vars.size())) {
      reduced_x = solved.x;
    }
  }

  if (reduced_x.size() ==
      static_cast<int>(reduced.linear_part.vars.size())) {
    result.x = presolve.postsolve(reduced_x);
    result.objective = instance.objective_offset;
    for (int j = 0; j < result.x.size() && j < instance.mip.linear_part.c.size();
         ++j) {
      result.objective += instance.mip.linear_part.c[j] * result.x[j];
    }
    const Audit audit = audit_solution(instance, result.x, result.objective);
    result.audit = audit;
    result.has_solution = audit.passed;
    if (!audit.passed) {
      result.status = "Native presolve/postsolve audit failed: " + result.status;
    }
  }
  result.optimal = reduced_optimal && result.has_solution;
  result.proven = reduced_proven &&
                  (result.has_solution || !reduced_x.size());
  result.timed_out = result.status.find("Time") != std::string::npos ||
                     result.status.find("time") != std::string::npos;
  result.solve_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - start).count();
  return result;
}

Result run_solver(const Instance& instance, const Config& cfg,
                  const std::string& solver) {
  if (solver == "cplex-mip") return run_cplex(instance, cfg);
  if (solver == "highs-mip") return run_highs(instance, cfg);
  if (solver == "scip-mip") return run_scip(instance, cfg);
  if (solver == "native-highs-lp") return run_native(instance, cfg, false);
  if (solver == "native-native-lp") return run_native(instance, cfg, true);
  if (solver == "native-presolve-highs-audit") {
    return run_native_presolve_highs_audit(instance, cfg);
  }
  Result result;
  set_dimensions(instance, result);
  result.solver = solver;
  result.available = false;
  result.status = "unknown solver";
  return result;
}

void attach_validation(const Instance& instance, const Reference* reference,
                       const Config& cfg, Result& result) {
  if (result.has_solution) {
    result.audit = audit_solution(instance, result.x, result.objective);
  }
  audit_and_integrate_bound_events(instance, cfg, result);
  if (reference == nullptr) return;
  result.reference_status = reference->status;
  result.reference_objective = reference->objective;
  if (result.proven && result.has_solution &&
      reference->status.find("opt") != std::string::npos &&
      std::isfinite(reference->objective) &&
      std::isfinite(result.objective)) {
    const double rel_error = std::abs(result.objective - reference->objective) /
                             std::max(1.0, std::abs(reference->objective));
    // The known optimum validates a solve to the experiment's requested gap;
    // feasibility and reported-objective consistency retain the stricter
    // original-space audit. See the protocol's gap-certificate derivation.
    const double reference_tolerance = std::max(kAuditTolerance, cfg.gap);
    result.reference_match = rel_error <= reference_tolerance;
    if (!result.reference_match) {
      result.status = "Reference audit rejected optimal claim: " + result.status;
      result.optimal = false;
      result.proven = false;
    }
  } else if (result.proven && !result.has_solution &&
             reference->status.find("inf") != std::string::npos &&
             (result.status.find("infeasible") != std::string::npos ||
              result.status.find("Infeasible") != std::string::npos)) {
    result.reference_match = true;
  }
}

std::string csv_escape(const std::string& text) {
  if (text.find_first_of(",\"\n") == std::string::npos) return text;
  std::string out = "\"";
  for (char c : text) out += c == '\"' ? "\"\"" : std::string(1, c);
  return out + "\"";
}

std::string csv_number(double value) {
  if (!std::isfinite(value)) return {};
  std::ostringstream out;
  out << std::setprecision(std::numeric_limits<double>::max_digits10) << value;
  return out.str();
}

template <typename Integer>
std::string csv_count(Integer value, bool available) {
  return available ? std::to_string(value) : std::string{};
}

void write_csv(const fs::path& path, const std::vector<Result>& results) {
  if (path.empty()) return;
  if (!path.parent_path().empty()) fs::create_directories(path.parent_path());
  std::ofstream out(path);
  if (!out) throw std::runtime_error("cannot write " + path.string());
  out << "instance,solver,seed,repeat,execution_index,block_order_position,rows,columns,"
         "nonzeros,integers,binaries,semicontinuous,semiinteger,available,"
         "has_solution,optimal,proven,timed_out,hard_timeout,audit_passed,reference_match,read_ms,"
         "shared_decompress_ms,solve_ms,objective,best_bound,gap,"
         "bound_event_stream_available,primal_dual_integral_sec,"
         "bound_event_count,bound_event_stream_error,"
         "bound_events_dropped_uncertified_dual,nodes,lp_solves,lp_iterations,cuts,"
         "fallback_events,fallback_recoveries,native_presolve_attempted,"
         "native_presolve_adopted,native_presolve_orig_rows,"
         "native_presolve_final_rows,native_presolve_orig_cols,"
         "native_presolve_final_cols,native_presolve_time_ms,"
         "native_presolve_probing_trail_pushes,"
         "native_presolve_probing_rows_processed,"
         "native_presolve_probing_implications_learned,"
         "native_presolve_probing_implications_imported,"
         "native_presolve_probing_max_touched_cols,"
         "native_presolve_probing_truncated,node_queue_domain_compactions,"
         "node_queue_domain_materializations,node_queue_dense_bound_values_released,"
         "node_queue_compact_entries_created,node_queue_terminal_compact_nodes,"
         "node_queue_terminal_compact_entries,node_queue_peak_compact_nodes,"
         "node_queue_peak_compact_entries,node_queue_domain_compaction_failures,"
         "node_queue_domain_materialization_failures,node_queue_serial_compactions,"
         "node_queue_parallel_compactions,node_queue_submip_compactions,"
         "branch_payload_child_creations,branch_payload_shared_vectors,"
         "branch_payload_shared_elements,branch_domain_dense_copies,"
         "branch_domain_dense_values_copied,branch_domain_moves,"
         "root_domain_probe_workspace_initializations,"
         "root_domain_probe_worlds,root_domain_probe_trail_pushes,"
         "root_domain_probe_rollbacks,root_domain_probe_failures,"
         "root_domain_probe_rows_processed,"
         "root_domain_probe_changed_columns,"
         "root_domain_probe_implication_export_columns_visited,"
         "root_domain_probe_implications_learned,"
         "root_domain_probe_committed_bound_changes,"
         "root_lp_probe_bound_workspace_initializations,"
         "root_lp_probe_bound_transactions,"
         "root_lp_probe_transaction_snapshot_values,"
         "root_lp_probe_transaction_rollbacks,"
         "root_lp_probe_transaction_failures,"
         "root_lp_probe_backend_cold_solves,"
         "root_lp_probe_backend_persistent_resolves,"
         "strong_probe_base_sf_materializations,"
         "strong_probe_bound_transactions,"
         "strong_probe_transaction_snapshot_values,"
         "strong_probe_transaction_rollbacks,"
         "strong_probe_transaction_failures,"
         "strong_probe_backend_cold_solves,"
         "strong_probe_backend_persistent_resolves,"
         "separator_sparse_candidates_created,"
         "separator_sparse_candidate_entries_created,"
         "separator_peak_live_sparse_candidates,"
         "separator_peak_live_sparse_entries,"
         "separator_sparse_aggregation_snapshots,"
         "separator_sparse_aggregation_entries,"
         "separator_dense_workspace_materializations,"
         "separator_dense_workspace_values,separator_matrix_append_calls,"
         "separator_matrix_appended_rows,separator_matrix_appended_entries,"
         "separator_matrix_prior_entries_bypassing_triplet_rebuild,"
         "separator_matrix_storage_reallocations,"
         "separator_matrix_peak_spare_entries,tree_restarts,"
         "tree_restart_nodes_discarded,tree_restart_root_requeues,"
         "tree_restart_cut_pool_rows_preserved,tree_restart_conflicts_preserved,"
         "tree_restart_implications_preserved,tree_restart_clique_edges_preserved,"
         "tree_restart_pseudocost_observations_preserved,"
         "tree_restart_last_node,tree_restart_last_source,"
         "max_row_violation,max_bound_violation,"
         "max_integrality_violation,objective_disagreement,reference_status,"
         "reference_objective,status\n";
  out << std::setprecision(17);
  for (const Result& r : results) {
    out << csv_escape(r.instance) << ',' << csv_escape(r.solver) << ',' << r.seed
        << ',' << r.repeat << ','
        << r.execution_index << ',' << r.block_order_position << ','
        << r.rows << ',' << r.columns << ',' << r.nonzeros << ',' << r.integers << ','
        << r.binaries << ',' << r.semicontinuous << ',' << r.semiinteger << ','
        << r.available << ',' << r.has_solution << ',' << r.optimal
        << ',' << r.proven << ',' << r.timed_out << ',' << r.hard_timeout << ','
        << r.audit.passed << ','
        << r.reference_match << ',' << r.read_ms << ',' << r.shared_decompress_ms << ','
        << r.solve_ms << ','
        << csv_number(r.objective) << ',' << csv_number(r.best_bound) << ','
        << csv_number(r.gap) << ','
        << r.bound_event_stream_available << ','
        << csv_number(r.primal_dual_integral_sec) << ','
        << r.bound_events.size() << ','
        << csv_escape(r.bound_event_stream_error) << ','
        << r.bound_events_dropped_uncertified_dual << ','
        << csv_count(r.nodes, r.node_count_available) << ','
        << csv_count(r.lp_solves, r.lp_solve_count_available) << ','
        << csv_count(r.lp_iterations, r.lp_iteration_count_available) << ','
        << csv_count(r.cuts, r.cut_count_available) << ','
        << csv_count(r.fallback_events, r.native_diagnostics_available) << ','
        << csv_count(r.fallback_recoveries, r.native_diagnostics_available) << ','
        << csv_count(r.native_presolve_attempted,
                     r.native_diagnostics_available) << ','
        << csv_count(r.native_presolve_adopted,
                     r.native_diagnostics_available) << ','
        << csv_count(r.native_presolve_orig_rows,
                     r.native_diagnostics_available) << ','
        << csv_count(r.native_presolve_final_rows,
                     r.native_diagnostics_available) << ','
        << csv_count(r.native_presolve_orig_cols,
                     r.native_diagnostics_available) << ','
        << csv_count(r.native_presolve_final_cols,
                     r.native_diagnostics_available) << ','
        << (r.native_diagnostics_available
                ? csv_number(r.native_presolve_time_ms) : std::string{}) << ','
        << csv_count(r.native_presolve_probing_trail_pushes,
                     r.native_diagnostics_available) << ','
        << csv_count(r.native_presolve_probing_rows_processed,
                     r.native_diagnostics_available) << ','
        << csv_count(r.native_presolve_probing_implications_learned,
                     r.native_diagnostics_available) << ','
        << csv_count(r.native_presolve_probing_implications_imported,
                     r.native_diagnostics_available) << ','
        << csv_count(r.native_presolve_probing_max_touched_cols,
                     r.native_diagnostics_available) << ','
        << csv_count(r.native_presolve_probing_truncated,
                     r.native_diagnostics_available) << ','
        << csv_count(r.node_queue_domain_compactions,
                     r.native_diagnostics_available) << ','
        << csv_count(r.node_queue_domain_materializations,
                     r.native_diagnostics_available) << ','
        << csv_count(r.node_queue_dense_bound_values_released,
                     r.native_diagnostics_available) << ','
        << csv_count(r.node_queue_compact_entries_created,
                     r.native_diagnostics_available) << ','
        << csv_count(r.node_queue_terminal_compact_nodes,
                     r.native_diagnostics_available) << ','
        << csv_count(r.node_queue_terminal_compact_entries,
                     r.native_diagnostics_available) << ','
        << csv_count(r.node_queue_peak_compact_nodes,
                     r.native_diagnostics_available) << ','
        << csv_count(r.node_queue_peak_compact_entries,
                     r.native_diagnostics_available) << ','
        << csv_count(r.node_queue_domain_compaction_failures,
                     r.native_diagnostics_available) << ','
        << csv_count(r.node_queue_domain_materialization_failures,
                     r.native_diagnostics_available) << ','
        << csv_count(r.node_queue_serial_compactions,
                     r.native_diagnostics_available) << ','
        << csv_count(r.node_queue_parallel_compactions,
                     r.native_diagnostics_available) << ','
        << csv_count(r.node_queue_submip_compactions,
                     r.native_diagnostics_available) << ','
        << csv_count(r.branch_payload_child_creations,
                     r.native_diagnostics_available) << ','
        << csv_count(r.branch_payload_shared_vectors,
                     r.native_diagnostics_available) << ','
        << csv_count(r.branch_payload_shared_elements,
                     r.native_diagnostics_available) << ','
        << csv_count(r.branch_domain_dense_copies,
                     r.native_diagnostics_available) << ','
        << csv_count(r.branch_domain_dense_values_copied,
                     r.native_diagnostics_available) << ','
        << csv_count(r.branch_domain_moves,
                     r.native_diagnostics_available) << ','
        << csv_count(r.root_domain_probe_workspace_initializations,
                     r.native_diagnostics_available) << ','
        << csv_count(r.root_domain_probe_worlds,
                     r.native_diagnostics_available) << ','
        << csv_count(r.root_domain_probe_trail_pushes,
                     r.native_diagnostics_available) << ','
        << csv_count(r.root_domain_probe_rollbacks,
                     r.native_diagnostics_available) << ','
        << csv_count(r.root_domain_probe_failures,
                     r.native_diagnostics_available) << ','
        << csv_count(r.root_domain_probe_rows_processed,
                     r.native_diagnostics_available) << ','
        << csv_count(r.root_domain_probe_changed_columns,
                     r.native_diagnostics_available) << ','
        << csv_count(r.root_domain_probe_implication_export_columns_visited,
                     r.native_diagnostics_available) << ','
        << csv_count(r.root_domain_probe_implications_learned,
                     r.native_diagnostics_available) << ','
        << csv_count(r.root_domain_probe_committed_bound_changes,
                     r.native_diagnostics_available) << ','
        << csv_count(r.root_lp_probe_bound_workspace_initializations,
                     r.native_diagnostics_available) << ','
        << csv_count(r.root_lp_probe_bound_transactions,
                     r.native_diagnostics_available) << ','
        << csv_count(r.root_lp_probe_transaction_snapshot_values,
                     r.native_diagnostics_available) << ','
        << csv_count(r.root_lp_probe_transaction_rollbacks,
                     r.native_diagnostics_available) << ','
        << csv_count(r.root_lp_probe_transaction_failures,
                     r.native_diagnostics_available) << ','
        << csv_count(r.root_lp_probe_backend_cold_solves,
                     r.native_diagnostics_available) << ','
        << csv_count(r.root_lp_probe_backend_persistent_resolves,
                     r.native_diagnostics_available) << ','
        << csv_count(r.strong_probe_base_sf_materializations,
                     r.native_diagnostics_available) << ','
        << csv_count(r.strong_probe_bound_transactions,
                     r.native_diagnostics_available) << ','
        << csv_count(r.strong_probe_transaction_snapshot_values,
                     r.native_diagnostics_available) << ','
        << csv_count(r.strong_probe_transaction_rollbacks,
                     r.native_diagnostics_available) << ','
        << csv_count(r.strong_probe_transaction_failures,
                     r.native_diagnostics_available) << ','
        << csv_count(r.strong_probe_backend_cold_solves,
                     r.native_diagnostics_available) << ','
        << csv_count(r.strong_probe_backend_persistent_resolves,
                     r.native_diagnostics_available) << ','
        << csv_count(r.separator_sparse_candidates_created,
                     r.native_diagnostics_available) << ','
        << csv_count(r.separator_sparse_candidate_entries_created,
                     r.native_diagnostics_available) << ','
        << csv_count(r.separator_peak_live_sparse_candidates,
                     r.native_diagnostics_available) << ','
        << csv_count(r.separator_peak_live_sparse_entries,
                     r.native_diagnostics_available) << ','
        << csv_count(r.separator_sparse_aggregation_snapshots,
                     r.native_diagnostics_available) << ','
        << csv_count(r.separator_sparse_aggregation_entries,
                     r.native_diagnostics_available) << ','
        << csv_count(r.separator_dense_workspace_materializations,
                     r.native_diagnostics_available) << ','
        << csv_count(r.separator_dense_workspace_values,
                     r.native_diagnostics_available) << ','
        << csv_count(r.separator_matrix_append_calls,
                     r.native_diagnostics_available) << ','
        << csv_count(r.separator_matrix_appended_rows,
                     r.native_diagnostics_available) << ','
        << csv_count(r.separator_matrix_appended_entries,
                     r.native_diagnostics_available) << ','
        << csv_count(
               r.separator_matrix_prior_entries_bypassing_triplet_rebuild,
               r.native_diagnostics_available) << ','
        << csv_count(r.separator_matrix_storage_reallocations,
                     r.native_diagnostics_available) << ','
        << csv_count(r.separator_matrix_peak_spare_entries,
                     r.native_diagnostics_available) << ','
        << csv_count(r.tree_restarts, r.native_diagnostics_available) << ','
        << csv_count(r.tree_restart_nodes_discarded,
                     r.native_diagnostics_available) << ','
        << csv_count(r.tree_restart_root_requeues,
                     r.native_diagnostics_available) << ','
        << csv_count(r.tree_restart_cut_pool_rows_preserved,
                     r.native_diagnostics_available) << ','
        << csv_count(r.tree_restart_conflicts_preserved,
                     r.native_diagnostics_available) << ','
        << csv_count(r.tree_restart_implications_preserved,
                     r.native_diagnostics_available) << ','
        << csv_count(r.tree_restart_clique_edges_preserved,
                     r.native_diagnostics_available) << ','
        << csv_count(r.tree_restart_pseudocost_observations_preserved,
                     r.native_diagnostics_available) << ','
        << csv_count(r.tree_restart_last_node,
                     r.native_diagnostics_available &&
                         r.tree_restart_last_node >= 0) << ','
        << (r.native_diagnostics_available
                ? csv_escape(r.tree_restart_last_source) : std::string{}) << ','
        << csv_number(r.audit.max_row_violation) << ','
        << csv_number(r.audit.max_bound_violation) << ','
        << csv_number(r.audit.max_integrality_violation) << ','
        << csv_number(r.audit.objective_disagreement)
        << ',' << csv_escape(r.reference_status) << ','
        << csv_number(r.reference_objective) << ','
        << csv_escape(r.status) << '\n';
  }
}

json json_number(double value) {
  return std::isfinite(value) ? json(value) : json(nullptr);
}

json bound_events_json(const Result& result, bool include_solutions) {
  json events = json::array();
  for (const auto& event : result.bound_events) {
    json item{
        {"time_ms", event.time_ms},
        {"primal_bound", json_number(event.primal_bound)},
        {"dual_bound", json_number(event.dual_bound)},
        {"incumbent_audit", {
            {"passed", event.incumbent_audit.passed},
            {"objective", json_number(event.incumbent_audit.objective)},
            {"objective_disagreement",
             json_number(event.incumbent_audit.objective_disagreement)},
            {"max_row_violation",
             json_number(event.incumbent_audit.max_row_violation)},
            {"max_bound_violation",
             json_number(event.incumbent_audit.max_bound_violation)},
            {"max_integrality_violation",
             json_number(event.incumbent_audit.max_integrality_violation)}}}};
    if (include_solutions) {
      item["incumbent"] = json::array();
      for (int j = 0; j < event.incumbent.size(); ++j) {
        item["incumbent"].push_back(event.incumbent[j]);
      }
    }
    events.push_back(std::move(item));
  }
  return events;
}

json result_json(const Result& r) {
  return {
      {"instance", r.instance}, {"solver", r.solver}, {"seed", r.seed},
      {"repeat", r.repeat},
      {"execution_index", r.execution_index},
      {"block_order_position", r.block_order_position},
      {"collection_scope", r.collection_scope},
      {"statistics_available", {
          {"nodes", r.node_count_available},
          {"lp_solves", r.lp_solve_count_available},
          {"lp_iterations", r.lp_iteration_count_available},
          {"cuts", r.cut_count_available},
          {"incumbent_timeline", r.incumbent_timeline_available},
          {"bound_events", r.bound_event_stream_available},
          {"native_diagnostics", r.native_diagnostics_available}}},
      {"rows", r.rows}, {"columns", r.columns}, {"nonzeros", r.nonzeros},
      {"integers", r.integers}, {"binaries", r.binaries},
      {"semicontinuous", r.semicontinuous},
      {"semiinteger", r.semiinteger},
      {"available", r.available}, {"has_solution", r.has_solution},
      {"optimal", r.optimal}, {"proven", r.proven}, {"timed_out", r.timed_out},
      {"hard_timeout", r.hard_timeout},
      {"read_ms", r.read_ms}, {"shared_decompress_ms", r.shared_decompress_ms},
      {"solve_ms", r.solve_ms},
      {"objective", json_number(r.objective)},
      {"best_bound", json_number(r.best_bound)}, {"gap", json_number(r.gap)},
      {"primal_dual_integral_sec",
       json_number(r.primal_dual_integral_sec)},
      {"bound_event_stream_error", r.bound_event_stream_error},
      {"bound_events_dropped_uncertified_dual",
       r.bound_events_dropped_uncertified_dual},
      {"bound_events", bound_events_json(r, /*include_solutions=*/false)},
      {"nodes", r.node_count_available ? json(r.nodes) : json(nullptr)},
      {"lp_solves", r.lp_solve_count_available ? json(r.lp_solves) : json(nullptr)},
      {"lp_iterations", r.lp_iteration_count_available
                            ? json(r.lp_iterations) : json(nullptr)},
      {"cuts", r.cut_count_available ? json(r.cuts) : json(nullptr)},
      {"first_incumbent_node", r.incumbent_timeline_available &&
                                    r.first_incumbent_node >= 0
                                    ? json(r.first_incumbent_node) : json(nullptr)},
      {"first_incumbent_lp_solves", r.incumbent_timeline_available &&
                                        r.first_incumbent_lp_solves >= 0
                                        ? json(r.first_incumbent_lp_solves)
                                        : json(nullptr)},
      {"native_parallel", r.native_diagnostics_available ? json{
          {"requested_threads", r.parallel_requested_threads},
          {"effective_threads", r.parallel_effective_threads},
          {"explorer_threads", r.parallel_explorer_threads},
          {"tree_launched", r.parallel_tree_launched},
          {"schedule_reason", r.parallel_schedule_reason}}
          : json(nullptr)},
      {"native_presolve", r.native_diagnostics_available ? json{
          {"attempted", r.native_presolve_attempted},
          {"adopted", r.native_presolve_adopted},
          {"orig_rows", r.native_presolve_orig_rows},
          {"final_rows", r.native_presolve_final_rows},
          {"orig_cols", r.native_presolve_orig_cols},
          {"final_cols", r.native_presolve_final_cols},
          {"time_ms", r.native_presolve_time_ms},
          {"probing_trail_pushes", r.native_presolve_probing_trail_pushes},
          {"probing_rows_processed", r.native_presolve_probing_rows_processed},
          {"probing_implications_learned",
           r.native_presolve_probing_implications_learned},
          {"probing_implications_imported",
           r.native_presolve_probing_implications_imported},
          {"probing_max_touched_cols",
           r.native_presolve_probing_max_touched_cols},
          {"probing_truncated", r.native_presolve_probing_truncated}}
          : json(nullptr)},
      {"node_queue_storage", r.native_diagnostics_available ? json{
          {"domain_compactions", r.node_queue_domain_compactions},
          {"domain_materializations", r.node_queue_domain_materializations},
          {"dense_bound_values_released",
           r.node_queue_dense_bound_values_released},
          {"compact_entries_created", r.node_queue_compact_entries_created},
          {"terminal_compact_nodes", r.node_queue_terminal_compact_nodes},
          {"terminal_compact_entries", r.node_queue_terminal_compact_entries},
          {"peak_compact_nodes", r.node_queue_peak_compact_nodes},
          {"peak_compact_entries", r.node_queue_peak_compact_entries},
          {"compaction_failures", r.node_queue_domain_compaction_failures},
          {"materialization_failures",
           r.node_queue_domain_materialization_failures},
          {"serial_compactions", r.node_queue_serial_compactions},
          {"parallel_compactions", r.node_queue_parallel_compactions},
          {"submip_compactions", r.node_queue_submip_compactions}}
          : json(nullptr)},
      {"branch_payload_sharing", r.native_diagnostics_available ? json{
          {"child_creations", r.branch_payload_child_creations},
          {"shared_vectors", r.branch_payload_shared_vectors},
          {"shared_elements", r.branch_payload_shared_elements}}
          : json(nullptr)},
      {"branch_domain_transfer", r.native_diagnostics_available ? json{
          {"dense_copies", r.branch_domain_dense_copies},
          {"dense_values_copied", r.branch_domain_dense_values_copied},
          {"moves", r.branch_domain_moves}}
          : json(nullptr)},
      {"root_domain_probing", r.native_diagnostics_available ? json{
          {"workspace_initializations",
           r.root_domain_probe_workspace_initializations},
          {"worlds", r.root_domain_probe_worlds},
          {"trail_pushes", r.root_domain_probe_trail_pushes},
          {"rollbacks", r.root_domain_probe_rollbacks},
          {"failures", r.root_domain_probe_failures},
          {"rows_processed", r.root_domain_probe_rows_processed},
          {"changed_columns", r.root_domain_probe_changed_columns},
          {"implication_export_columns_visited",
           r.root_domain_probe_implication_export_columns_visited},
          {"implications_learned",
           r.root_domain_probe_implications_learned},
          {"committed_bound_changes",
           r.root_domain_probe_committed_bound_changes}}
          : json(nullptr)},
      {"root_lp_probing", r.native_diagnostics_available ? json{
          {"bound_workspace_initializations",
           r.root_lp_probe_bound_workspace_initializations},
          {"bound_transactions", r.root_lp_probe_bound_transactions},
          {"transaction_snapshot_values",
           r.root_lp_probe_transaction_snapshot_values},
          {"transaction_rollbacks",
           r.root_lp_probe_transaction_rollbacks},
          {"transaction_failures", r.root_lp_probe_transaction_failures},
          {"backend_cold_solves", r.root_lp_probe_backend_cold_solves},
          {"backend_persistent_resolves",
           r.root_lp_probe_backend_persistent_resolves}}
          : json(nullptr)},
      {"separator_storage", r.native_diagnostics_available ? json{
          {"sparse_candidates_created",
           r.separator_sparse_candidates_created},
          {"sparse_candidate_entries_created",
           r.separator_sparse_candidate_entries_created},
          {"peak_live_sparse_candidates",
           r.separator_peak_live_sparse_candidates},
          {"peak_live_sparse_entries",
           r.separator_peak_live_sparse_entries},
          {"sparse_aggregation_snapshots",
           r.separator_sparse_aggregation_snapshots},
          {"sparse_aggregation_entries",
           r.separator_sparse_aggregation_entries},
          {"dense_workspace_materializations",
           r.separator_dense_workspace_materializations},
          {"dense_workspace_values",
           r.separator_dense_workspace_values},
          {"matrix_append_calls", r.separator_matrix_append_calls},
          {"matrix_appended_rows", r.separator_matrix_appended_rows},
          {"matrix_appended_entries", r.separator_matrix_appended_entries},
          {"matrix_prior_entries_bypassing_triplet_rebuild",
           r.separator_matrix_prior_entries_bypassing_triplet_rebuild},
          {"matrix_storage_reallocations",
           r.separator_matrix_storage_reallocations},
          {"matrix_peak_spare_entries",
           r.separator_matrix_peak_spare_entries}}
          : json(nullptr)},
      {"tree_restart", r.native_diagnostics_available ? json{
          {"count", r.tree_restarts},
          {"nodes_discarded", r.tree_restart_nodes_discarded},
          {"root_requeues", r.tree_restart_root_requeues},
          {"cut_pool_rows_preserved",
           r.tree_restart_cut_pool_rows_preserved},
          {"conflicts_preserved", r.tree_restart_conflicts_preserved},
          {"implications_preserved", r.tree_restart_implications_preserved},
          {"clique_edges_preserved", r.tree_restart_clique_edges_preserved},
          {"pseudocost_observations_preserved",
           r.tree_restart_pseudocost_observations_preserved},
          {"last_node", r.tree_restart_last_node >= 0
                            ? json(r.tree_restart_last_node) : json(nullptr)},
          {"last_source", r.tree_restart_last_source}}
          : json(nullptr)},
      {"branching", r.native_diagnostics_available ? json{
          {"reliability_nodes", r.reliability_branch_nodes},
          {"strong_candidates", r.strong_branch_candidates},
          {"strong_lp_solves", r.strong_branch_lp_solves},
          {"probe_base_sf_materializations",
           r.strong_probe_base_sf_materializations},
          {"probe_bound_transactions", r.strong_probe_bound_transactions},
          {"probe_transaction_snapshot_values",
           r.strong_probe_transaction_snapshot_values},
          {"probe_transaction_rollbacks",
           r.strong_probe_transaction_rollbacks},
          {"probe_transaction_failures",
           r.strong_probe_transaction_failures},
          {"probe_backend_cold_solves",
           r.strong_probe_backend_cold_solves},
          {"probe_backend_persistent_resolves",
           r.strong_probe_backend_persistent_resolves},
          {"cache_exact_hits", r.strong_branch_cache_exact_hits},
          {"cache_warm_hits", r.strong_branch_cache_warm_hits},
          {"duplicate_lp_avoided", r.strong_branch_duplicate_lp_avoided},
          {"regret_samples", r.branching_regret_samples},
          {"regret_sum", r.branching_regret_sum},
          {"regret_max", r.branching_regret_max},
          {"strong_regret_samples", r.strong_branch_regret_samples},
          {"strong_regret_sum", r.strong_branch_regret_sum},
          {"strong_regret_max", r.strong_branch_regret_max}}
          : json(nullptr)},
      {"estimator_calibration", r.native_diagnostics_available ? json{
          {"node_samples", r.node_estimate_calibration_samples},
          {"node_predicted_lift_sum", r.node_estimate_predicted_lift_sum},
          {"node_realized_lift_sum", r.node_estimate_realized_lift_sum},
          {"node_abs_error_sum", r.node_estimate_abs_error_sum},
          {"node_squared_error_sum", r.node_estimate_squared_error_sum},
          {"node_predicted_sq_sum", r.node_estimate_predicted_sq_sum},
          {"node_realized_sq_sum", r.node_estimate_realized_sq_sum},
          {"node_cross_sum", r.node_estimate_cross_sum},
          {"directional_samples", r.directional_calibration_samples},
          {"directional_predicted_gain_sum", r.directional_predicted_gain_sum},
          {"directional_realized_gain_sum", r.directional_realized_gain_sum},
          {"directional_abs_error_sum", r.directional_abs_error_sum},
          {"directional_squared_error_sum", r.directional_squared_error_sum},
          {"directional_rank_samples", r.directional_rank_samples},
          {"directional_rank_concordant", r.directional_rank_concordant}}
          : json(nullptr)},
      {"fallback_events", r.native_diagnostics_available
                              ? json(r.fallback_events) : json(nullptr)},
      {"fallback_recoveries", r.native_diagnostics_available
                                  ? json(r.fallback_recoveries) : json(nullptr)},
      {"reference_status", r.reference_status},
      {"reference_objective", json_number(r.reference_objective)},
      {"reference_match", r.reference_match},
      {"audit", {
          {"passed", r.audit.passed},
          {"objective", json_number(r.audit.objective)},
          {"objective_disagreement", json_number(r.audit.objective_disagreement)},
          {"max_row_violation", json_number(r.audit.max_row_violation)},
          {"max_bound_violation", json_number(r.audit.max_bound_violation)},
          {"max_integrality_violation", json_number(r.audit.max_integrality_violation)}}},
      {"status", r.status}};
}

double optional_json_number(const json& object, const char* key,
                            double fallback) {
  const auto it = object.find(key);
  return it != object.end() && it->is_number() ? it->get<double>() : fallback;
}

template <typename Integer>
Integer optional_json_integer(const json& object, const char* key,
                              Integer fallback) {
  const auto it = object.find(key);
  return it != object.end() && it->is_number_integer()
      ? it->get<Integer>() : fallback;
}

json worker_result_json(const Result& result) {
  json out = result_json(result);
  out["bound_events"] =
      bound_events_json(result, /*include_solutions=*/true);
  out["x"] = json::array();
  for (int i = 0; i < result.x.size(); ++i) out["x"].push_back(result.x[i]);
  return out;
}

Result worker_result_from_json(const json& input) {
  Result result;
  result.instance = input.value("instance", "");
  result.solver = input.value("solver", "");
  result.seed = input.value("seed", 0);
  result.collection_scope = input.value("collection_scope", "unavailable");
  result.execution_index = input.value("execution_index", -1);
  result.block_order_position = input.value("block_order_position", -1);
  const json availability = input.value("statistics_available", json::object());
  result.node_count_available = availability.value("nodes", false);
  result.lp_solve_count_available = availability.value("lp_solves", false);
  result.lp_iteration_count_available = availability.value("lp_iterations", false);
  result.cut_count_available = availability.value("cuts", false);
  result.incumbent_timeline_available =
      availability.value("incumbent_timeline", false);
  result.bound_event_stream_available =
      availability.value("bound_events", false);
  result.native_diagnostics_available =
      availability.value("native_diagnostics", false);
  const json native_parallel =
      input.contains("native_parallel") && input["native_parallel"].is_object()
          ? input["native_parallel"] : json::object();
  result.parallel_requested_threads =
      native_parallel.value("requested_threads", 1);
  result.parallel_effective_threads =
      native_parallel.value("effective_threads", 1);
  result.parallel_explorer_threads =
      native_parallel.value("explorer_threads", 0);
  result.parallel_tree_launched =
      native_parallel.value("tree_launched", false);
  result.parallel_schedule_reason =
      native_parallel.value("schedule_reason", "unavailable");
  result.rows = input.value("rows", 0);
  result.columns = input.value("columns", 0);
  result.nonzeros = input.value("nonzeros", std::int64_t{0});
  result.integers = input.value("integers", 0);
  result.binaries = input.value("binaries", 0);
  result.semicontinuous = input.value("semicontinuous", 0);
  result.semiinteger = input.value("semiinteger", 0);
  result.available = input.value("available", true);
  result.has_solution = input.value("has_solution", false);
  result.optimal = input.value("optimal", false);
  result.proven = input.value("proven", false);
  result.timed_out = input.value("timed_out", false);
  result.hard_timeout = input.value("hard_timeout", false);
  result.read_ms = optional_json_number(input, "read_ms", 0.0);
  result.shared_decompress_ms =
      optional_json_number(input, "shared_decompress_ms", 0.0);
  result.solve_ms = optional_json_number(input, "solve_ms", 0.0);
  result.objective = optional_json_number(
      input, "objective", std::numeric_limits<double>::quiet_NaN());
  result.best_bound = optional_json_number(
      input, "best_bound", std::numeric_limits<double>::quiet_NaN());
  result.gap = optional_json_number(
      input, "gap", std::numeric_limits<double>::infinity());
  result.primal_dual_integral_sec = optional_json_number(
      input, "primal_dual_integral_sec",
      std::numeric_limits<double>::quiet_NaN());
  result.bound_event_stream_error =
      input.value("bound_event_stream_error", "");
  result.bound_events_dropped_uncertified_dual = optional_json_integer(
      input, "bound_events_dropped_uncertified_dual", std::uint64_t{0});
  result.nodes = optional_json_integer(input, "nodes", std::int64_t{-1});
  result.lp_solves = optional_json_integer(input, "lp_solves", std::int64_t{-1});
  result.lp_iterations =
      optional_json_integer(input, "lp_iterations", std::int64_t{-1});
  result.cuts = optional_json_integer(input, "cuts", -1);
  result.first_incumbent_node =
      optional_json_integer(input, "first_incumbent_node", -1);
  result.first_incumbent_lp_solves =
      optional_json_integer(input, "first_incumbent_lp_solves", -1);

  const auto events_it = input.find("bound_events");
  if (events_it != input.end() && events_it->is_array()) {
    for (const auto& item : *events_it) {
      if (!item.is_object()) continue;
      BoundEvent event;
      event.time_ms = optional_json_number(item, "time_ms", 0.0);
      event.primal_bound = optional_json_number(
          item, "primal_bound", std::numeric_limits<double>::quiet_NaN());
      event.dual_bound = optional_json_number(
          item, "dual_bound", std::numeric_limits<double>::quiet_NaN());
      const auto incumbent_it = item.find("incumbent");
      if (incumbent_it != item.end() && incumbent_it->is_array()) {
        event.incumbent.resize(
            static_cast<Eigen::Index>(incumbent_it->size()));
        for (std::size_t j = 0; j < incumbent_it->size(); ++j) {
          event.incumbent[static_cast<Eigen::Index>(j)] =
              (*incumbent_it)[j].get<double>();
        }
      }
      result.bound_events.push_back(std::move(event));
    }
  }

  const json native_presolve =
      input.contains("native_presolve") && input["native_presolve"].is_object()
          ? input["native_presolve"] : json::object();
  result.native_presolve_attempted =
      native_presolve.value("attempted", false);
  result.native_presolve_adopted = native_presolve.value("adopted", false);
  result.native_presolve_orig_rows = native_presolve.value("orig_rows", 0);
  result.native_presolve_final_rows = native_presolve.value("final_rows", 0);
  result.native_presolve_orig_cols = native_presolve.value("orig_cols", 0);
  result.native_presolve_final_cols = native_presolve.value("final_cols", 0);
  result.native_presolve_time_ms =
      optional_json_number(native_presolve, "time_ms", 0.0);
  result.native_presolve_probing_trail_pushes = native_presolve.value(
      "probing_trail_pushes", std::uint64_t{0});
  result.native_presolve_probing_rows_processed = native_presolve.value(
      "probing_rows_processed", std::uint64_t{0});
  result.native_presolve_probing_implications_learned = native_presolve.value(
      "probing_implications_learned", std::uint64_t{0});
  result.native_presolve_probing_implications_imported = native_presolve.value(
      "probing_implications_imported", std::uint64_t{0});
  result.native_presolve_probing_max_touched_cols = native_presolve.value(
      "probing_max_touched_cols", std::uint64_t{0});
  result.native_presolve_probing_truncated =
      native_presolve.value("probing_truncated", false);

  const json node_queue_storage =
      input.contains("node_queue_storage") &&
              input["node_queue_storage"].is_object()
          ? input["node_queue_storage"] : json::object();
  result.node_queue_domain_compactions = node_queue_storage.value(
      "domain_compactions", std::uint64_t{0});
  result.node_queue_domain_materializations = node_queue_storage.value(
      "domain_materializations", std::uint64_t{0});
  result.node_queue_dense_bound_values_released = node_queue_storage.value(
      "dense_bound_values_released", std::uint64_t{0});
  result.node_queue_compact_entries_created = node_queue_storage.value(
      "compact_entries_created", std::uint64_t{0});
  result.node_queue_terminal_compact_nodes = node_queue_storage.value(
      "terminal_compact_nodes", std::uint64_t{0});
  result.node_queue_terminal_compact_entries = node_queue_storage.value(
      "terminal_compact_entries", std::uint64_t{0});
  result.node_queue_peak_compact_nodes = node_queue_storage.value(
      "peak_compact_nodes", std::uint64_t{0});
  result.node_queue_peak_compact_entries = node_queue_storage.value(
      "peak_compact_entries", std::uint64_t{0});
  result.node_queue_domain_compaction_failures = node_queue_storage.value(
      "compaction_failures", std::uint64_t{0});
  result.node_queue_domain_materialization_failures =
      node_queue_storage.value("materialization_failures", std::uint64_t{0});
  result.node_queue_serial_compactions = node_queue_storage.value(
      "serial_compactions", std::uint64_t{0});
  result.node_queue_parallel_compactions = node_queue_storage.value(
      "parallel_compactions", std::uint64_t{0});
  result.node_queue_submip_compactions = node_queue_storage.value(
      "submip_compactions", std::uint64_t{0});

  const json branch_payload_sharing =
      input.contains("branch_payload_sharing") &&
              input["branch_payload_sharing"].is_object()
          ? input["branch_payload_sharing"] : json::object();
  result.branch_payload_child_creations = branch_payload_sharing.value(
      "child_creations", std::uint64_t{0});
  result.branch_payload_shared_vectors = branch_payload_sharing.value(
      "shared_vectors", std::uint64_t{0});
  result.branch_payload_shared_elements = branch_payload_sharing.value(
      "shared_elements", std::uint64_t{0});

  const json branch_domain_transfer =
      input.contains("branch_domain_transfer") &&
              input["branch_domain_transfer"].is_object()
          ? input["branch_domain_transfer"] : json::object();
  result.branch_domain_dense_copies = branch_domain_transfer.value(
      "dense_copies", std::uint64_t{0});
  result.branch_domain_dense_values_copied = branch_domain_transfer.value(
      "dense_values_copied", std::uint64_t{0});
  result.branch_domain_moves = branch_domain_transfer.value(
      "moves", std::uint64_t{0});

  const json separator_storage =
      input.contains("separator_storage") &&
              input["separator_storage"].is_object()
          ? input["separator_storage"] : json::object();
  result.separator_sparse_candidates_created = separator_storage.value(
      "sparse_candidates_created", std::uint64_t{0});
  result.separator_sparse_candidate_entries_created = separator_storage.value(
      "sparse_candidate_entries_created", std::uint64_t{0});
  result.separator_peak_live_sparse_candidates = separator_storage.value(
      "peak_live_sparse_candidates", std::uint64_t{0});
  result.separator_peak_live_sparse_entries = separator_storage.value(
      "peak_live_sparse_entries", std::uint64_t{0});
  result.separator_sparse_aggregation_snapshots = separator_storage.value(
      "sparse_aggregation_snapshots", std::uint64_t{0});
  result.separator_sparse_aggregation_entries = separator_storage.value(
      "sparse_aggregation_entries", std::uint64_t{0});
  result.separator_dense_workspace_materializations = separator_storage.value(
      "dense_workspace_materializations", std::uint64_t{0});
  result.separator_dense_workspace_values = separator_storage.value(
      "dense_workspace_values", std::uint64_t{0});
  result.separator_matrix_append_calls = separator_storage.value(
      "matrix_append_calls", std::uint64_t{0});
  result.separator_matrix_appended_rows = separator_storage.value(
      "matrix_appended_rows", std::uint64_t{0});
  result.separator_matrix_appended_entries = separator_storage.value(
      "matrix_appended_entries", std::uint64_t{0});
  result.separator_matrix_prior_entries_bypassing_triplet_rebuild =
      separator_storage.value(
          "matrix_prior_entries_bypassing_triplet_rebuild",
          std::uint64_t{0});
  result.separator_matrix_storage_reallocations = separator_storage.value(
      "matrix_storage_reallocations", std::uint64_t{0});
  result.separator_matrix_peak_spare_entries = separator_storage.value(
      "matrix_peak_spare_entries", std::uint64_t{0});

  const json tree_restart =
      input.contains("tree_restart") && input["tree_restart"].is_object()
          ? input["tree_restart"] : json::object();
  result.tree_restarts = tree_restart.value("count", std::uint64_t{0});
  result.tree_restart_nodes_discarded = tree_restart.value(
      "nodes_discarded", std::uint64_t{0});
  result.tree_restart_root_requeues = tree_restart.value(
      "root_requeues", std::uint64_t{0});
  result.tree_restart_cut_pool_rows_preserved = tree_restart.value(
      "cut_pool_rows_preserved", std::uint64_t{0});
  result.tree_restart_conflicts_preserved = tree_restart.value(
      "conflicts_preserved", std::uint64_t{0});
  result.tree_restart_implications_preserved = tree_restart.value(
      "implications_preserved", std::uint64_t{0});
  result.tree_restart_clique_edges_preserved = tree_restart.value(
      "clique_edges_preserved", std::uint64_t{0});
  result.tree_restart_pseudocost_observations_preserved = tree_restart.value(
      "pseudocost_observations_preserved", std::uint64_t{0});
  result.tree_restart_last_node = optional_json_integer(
      tree_restart, "last_node", -1);
  result.tree_restart_last_source = tree_restart.value("last_source", "");

  const json root_domain_probing =
      input.contains("root_domain_probing") &&
              input["root_domain_probing"].is_object()
          ? input["root_domain_probing"] : json::object();
  result.root_domain_probe_workspace_initializations =
      root_domain_probing.value("workspace_initializations", std::uint64_t{0});
  result.root_domain_probe_worlds =
      root_domain_probing.value("worlds", std::uint64_t{0});
  result.root_domain_probe_trail_pushes =
      root_domain_probing.value("trail_pushes", std::uint64_t{0});
  result.root_domain_probe_rollbacks =
      root_domain_probing.value("rollbacks", std::uint64_t{0});
  result.root_domain_probe_failures =
      root_domain_probing.value("failures", std::uint64_t{0});
  result.root_domain_probe_rows_processed =
      root_domain_probing.value("rows_processed", std::uint64_t{0});
  result.root_domain_probe_changed_columns =
      root_domain_probing.value("changed_columns", std::uint64_t{0});
  result.root_domain_probe_implication_export_columns_visited =
      root_domain_probing.value("implication_export_columns_visited",
                                std::uint64_t{0});
  result.root_domain_probe_implications_learned =
      root_domain_probing.value("implications_learned", std::uint64_t{0});
  result.root_domain_probe_committed_bound_changes =
      root_domain_probing.value("committed_bound_changes", std::uint64_t{0});

  const json root_lp_probing =
      input.contains("root_lp_probing") && input["root_lp_probing"].is_object()
          ? input["root_lp_probing"] : json::object();
  result.root_lp_probe_bound_workspace_initializations = root_lp_probing.value(
      "bound_workspace_initializations", std::uint64_t{0});
  result.root_lp_probe_bound_transactions = root_lp_probing.value(
      "bound_transactions", std::uint64_t{0});
  result.root_lp_probe_transaction_snapshot_values = root_lp_probing.value(
      "transaction_snapshot_values", std::uint64_t{0});
  result.root_lp_probe_transaction_rollbacks = root_lp_probing.value(
      "transaction_rollbacks", std::uint64_t{0});
  result.root_lp_probe_transaction_failures = root_lp_probing.value(
      "transaction_failures", std::uint64_t{0});
  result.root_lp_probe_backend_cold_solves = root_lp_probing.value(
      "backend_cold_solves", std::uint64_t{0});
  result.root_lp_probe_backend_persistent_resolves = root_lp_probing.value(
      "backend_persistent_resolves", std::uint64_t{0});

  const json branching = input.contains("branching") && input["branching"].is_object()
      ? input["branching"] : json::object();
  result.reliability_branch_nodes = branching.value("reliability_nodes", std::uint64_t{0});
  result.strong_branch_candidates = branching.value("strong_candidates", std::uint64_t{0});
  result.strong_branch_lp_solves = branching.value("strong_lp_solves", std::uint64_t{0});
  result.strong_probe_base_sf_materializations = branching.value(
      "probe_base_sf_materializations", std::uint64_t{0});
  result.strong_probe_bound_transactions = branching.value(
      "probe_bound_transactions", std::uint64_t{0});
  result.strong_probe_transaction_snapshot_values = branching.value(
      "probe_transaction_snapshot_values", std::uint64_t{0});
  result.strong_probe_transaction_rollbacks = branching.value(
      "probe_transaction_rollbacks", std::uint64_t{0});
  result.strong_probe_transaction_failures = branching.value(
      "probe_transaction_failures", std::uint64_t{0});
  result.strong_probe_backend_cold_solves = branching.value(
      "probe_backend_cold_solves", std::uint64_t{0});
  result.strong_probe_backend_persistent_resolves = branching.value(
      "probe_backend_persistent_resolves", std::uint64_t{0});
  result.strong_branch_cache_exact_hits = branching.value("cache_exact_hits", std::uint64_t{0});
  result.strong_branch_cache_warm_hits = branching.value("cache_warm_hits", std::uint64_t{0});
  result.strong_branch_duplicate_lp_avoided =
      branching.value("duplicate_lp_avoided", std::uint64_t{0});
  result.branching_regret_samples = branching.value("regret_samples", std::uint64_t{0});
  result.branching_regret_sum = optional_json_number(branching, "regret_sum", 0.0);
  result.branching_regret_max = optional_json_number(branching, "regret_max", 0.0);
  result.strong_branch_regret_samples =
      branching.value("strong_regret_samples", std::uint64_t{0});
  result.strong_branch_regret_sum =
      optional_json_number(branching, "strong_regret_sum", 0.0);
  result.strong_branch_regret_max =
      optional_json_number(branching, "strong_regret_max", 0.0);

  const json calibration = input.contains("estimator_calibration") &&
                                   input["estimator_calibration"].is_object()
      ? input["estimator_calibration"] : json::object();
  result.node_estimate_calibration_samples =
      calibration.value("node_samples", std::uint64_t{0});
  result.node_estimate_predicted_lift_sum =
      optional_json_number(calibration, "node_predicted_lift_sum", 0.0);
  result.node_estimate_realized_lift_sum =
      optional_json_number(calibration, "node_realized_lift_sum", 0.0);
  result.node_estimate_abs_error_sum =
      optional_json_number(calibration, "node_abs_error_sum", 0.0);
  result.node_estimate_squared_error_sum =
      optional_json_number(calibration, "node_squared_error_sum", 0.0);
  result.node_estimate_predicted_sq_sum =
      optional_json_number(calibration, "node_predicted_sq_sum", 0.0);
  result.node_estimate_realized_sq_sum =
      optional_json_number(calibration, "node_realized_sq_sum", 0.0);
  result.node_estimate_cross_sum =
      optional_json_number(calibration, "node_cross_sum", 0.0);
  result.directional_calibration_samples =
      calibration.value("directional_samples", std::uint64_t{0});
  result.directional_predicted_gain_sum =
      optional_json_number(calibration, "directional_predicted_gain_sum", 0.0);
  result.directional_realized_gain_sum =
      optional_json_number(calibration, "directional_realized_gain_sum", 0.0);
  result.directional_abs_error_sum =
      optional_json_number(calibration, "directional_abs_error_sum", 0.0);
  result.directional_squared_error_sum =
      optional_json_number(calibration, "directional_squared_error_sum", 0.0);
  result.directional_rank_samples =
      calibration.value("directional_rank_samples", std::uint64_t{0});
  result.directional_rank_concordant =
      calibration.value("directional_rank_concordant", std::uint64_t{0});
  result.fallback_events = optional_json_integer(input, "fallback_events", -1);
  result.fallback_recoveries =
      optional_json_integer(input, "fallback_recoveries", -1);
  result.status = input.value("status", "worker result missing status");

  const auto x_it = input.find("x");
  if (x_it != input.end() && x_it->is_array()) {
    result.x.resize(static_cast<Eigen::Index>(x_it->size()));
    for (std::size_t i = 0; i < x_it->size(); ++i) {
      result.x[static_cast<Eigen::Index>(i)] = (*x_it)[i].get<double>();
    }
  }
  return result;
}

std::string precise_number(double value) {
  std::ostringstream out;
  out << std::setprecision(std::numeric_limits<double>::max_digits10) << value;
  return out.str();
}

Result run_solver_isolated(const Instance& instance, const Config& cfg,
                           const std::string& solver,
                           const fs::path& executable) {
#ifdef _WIN32
  // Windows needs a CreateProcess implementation before this benchmark can
  // claim a hard deadline there. Keep the platform limitation explicit.
  return run_solver(instance, cfg, solver);
#else
  Result fallback;
  set_dimensions(instance, fallback);
  fallback.solver = solver;

  const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
  const fs::path output_path = fs::temp_directory_path() /
      ("miplib_worker_" + std::to_string(static_cast<long long>(getpid())) +
       "_" + std::to_string(stamp) + ".json");
  std::vector<std::string> arguments{
      executable.string(),
      "--worker-instance", instance.solver_path.string(),
      "--worker-output", output_path.string(),
      "--worker-solver", solver,
      "--time-limit", precise_number(cfg.time_limit_sec),
      "--gap", precise_number(cfg.gap),
      "--max-nodes", std::to_string(cfg.max_nodes),
      "--native-threads", std::to_string(cfg.native_threads),
      "--seed", std::to_string(cfg.seed),
      "--native-node-estimate", cfg.native_node_estimate};
  if (cfg.highs_verbose) arguments.push_back("--highs-verbose");
  if (cfg.native_verbose) arguments.push_back("--native-verbose");
  if (!cfg.native_papilo_presolve) {
    arguments.push_back("--native-no-papilo-presolve");
  }
  if (!cfg.native_presolve_probing) {
    arguments.push_back("--native-no-presolve-probing");
  }
  if (!cfg.native_cuts) arguments.push_back("--native-no-cuts");
  if (!cfg.native_objective_propagation) {
    arguments.push_back("--native-no-objective-propagation");
  }
  if (!cfg.native_reduced_cost_fixing) {
    arguments.push_back("--native-no-reduced-cost-fixing");
  }
  if (cfg.native_row_propagation_rounds == 0) {
    arguments.push_back("--native-no-row-propagation");
  } else {
    arguments.push_back("--native-row-propagation-rounds");
    arguments.push_back(
        std::to_string(cfg.native_row_propagation_rounds));
  }
  arguments.push_back("--native-probe-max");
  arguments.push_back(std::to_string(cfg.native_probe_max));
  arguments.push_back("--native-probe-reliability");
  arguments.push_back(std::to_string(cfg.native_probe_reliability));
  if (!cfg.native_primal_hint_file.empty()) {
    arguments.push_back("--native-primal-hint");
    arguments.push_back(cfg.native_primal_hint_file.string());
  }
  if (cfg.native_audit_hint_only) {
    arguments.push_back("--native-audit-hint-only");
  }
  if (cfg.native_tree_restart) {
    arguments.push_back("--native-tree-restart");
  }
  arguments.push_back("--native-tree-restart-max");
  arguments.push_back(std::to_string(cfg.native_tree_restart_max));
  arguments.push_back("--native-tree-restart-min-nodes");
  arguments.push_back(std::to_string(cfg.native_tree_restart_min_nodes));
  arguments.push_back("--native-tree-restart-min-open-nodes");
  arguments.push_back(std::to_string(cfg.native_tree_restart_min_open_nodes));
  arguments.push_back("--native-tree-restart-min-improvement");
  arguments.push_back(precise_number(
      cfg.native_tree_restart_min_improvement));
  arguments.push_back("--native-tree-restart-min-remaining");
  arguments.push_back(precise_number(
      cfg.native_tree_restart_min_remaining_sec));

  const auto process_start = std::chrono::steady_clock::now();
  const pid_t child = fork();
  if (child == 0) {
    std::vector<char*> argv;
    argv.reserve(arguments.size() + 1);
    for (const std::string& argument : arguments) {
      argv.push_back(const_cast<char*>(argument.c_str()));
    }
    argv.push_back(nullptr);
    execv(executable.c_str(), argv.data());
    _exit(127);
  }
  if (child < 0) {
    fallback.status = "worker fork failed: errno=" + std::to_string(errno);
    return fallback;
  }

  int child_status = 0;
  auto terminate_and_reap = [&](int signal) {
    kill(child, signal);
    while (waitpid(child, &child_status, 0) < 0 && errno == EINTR) {
    }
  };

  bool completed = false;
  const double hard_limit_sec = cfg.time_limit_sec + cfg.hard_timeout_grace_sec;
  while (!completed) {
    const pid_t waited = waitpid(child, &child_status, WNOHANG);
    if (waited == child) {
      completed = true;
      break;
    }
    if (waited < 0 && errno != EINTR) {
      fallback.status = "worker wait failed: errno=" + std::to_string(errno);
      terminate_and_reap(SIGKILL);
      break;
    }
    const double elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - process_start).count();
    if (elapsed >= hard_limit_sec) {
      kill(child, SIGTERM);
      for (int retry = 0; retry < 10; ++retry) {
        if (waitpid(child, &child_status, WNOHANG) == child) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
      if (waitpid(child, &child_status, WNOHANG) == 0) {
        terminate_and_reap(SIGKILL);
      }
      fallback.solve_ms = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - process_start).count();
      fallback.timed_out = true;
      fallback.hard_timeout = true;
      fallback.status = "Hard process timeout";
      std::error_code error;
      fs::remove(output_path, error);
      return fallback;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  if (!completed || !WIFEXITED(child_status) || WEXITSTATUS(child_status) != 0) {
    if (fallback.status.empty()) {
      fallback.status = WIFSIGNALED(child_status)
          ? "worker terminated by signal " + std::to_string(WTERMSIG(child_status))
          : "worker exit " + std::to_string(WEXITSTATUS(child_status));
    }
    std::error_code error;
    fs::remove(output_path, error);
    return fallback;
  }

  try {
    std::ifstream input(output_path);
    if (!input) throw std::runtime_error("worker output is missing");
    json payload;
    input >> payload;
    Result result = worker_result_from_json(payload);
    result.instance = instance.name;
    result.shared_decompress_ms = instance.decompress_ms;
    std::error_code error;
    fs::remove(output_path, error);
    return result;
  } catch (const std::exception& error) {
    fallback.status = std::string("worker result error: ") + error.what();
    std::error_code remove_error;
    fs::remove(output_path, remove_error);
    return fallback;
  }
#endif
}

std::vector<Summary> summarize(const Config& cfg,
                               const std::vector<Result>& results) {
  std::vector<Summary> summaries;
  const double par10_ms = 10.0 * cfg.time_limit_sec * 1000.0;
  for (const std::string& solver : cfg.solvers) {
    Summary summary;
    summary.solver = solver;
    std::vector<double> solved_times;
    std::vector<double> solved_nodes;
    std::vector<double> penalized_times;
    double pdi_sum_sec = 0.0;
    for (const Result& result : results) {
      if (result.solver != solver) continue;
      ++summary.attempts;
      if (result.available) ++summary.available;
      if (result.has_solution) ++summary.feasible;
      if (result.optimal) ++summary.optimal;
      if (result.proven) ++summary.proven;
      if (result.audit.passed) ++summary.audited;
      if (result.reference_match) ++summary.reference_matches;
      if (result.timed_out) ++summary.timeouts;
      if (result.bound_event_stream_available &&
          std::isfinite(result.primal_dual_integral_sec)) {
        ++summary.primal_dual_integrals;
        pdi_sum_sec += result.primal_dual_integral_sec;
      }
      const bool benchmark_solved =
          (result.proven && result.audit.passed &&
           (result.reference_status.empty() || result.reference_match)) ||
          (result.proven && !result.has_solution && result.reference_match);
      if (benchmark_solved) {
        ++summary.benchmark_solved;
        solved_times.push_back(result.solve_ms);
        penalized_times.push_back(result.solve_ms);
      } else {
        penalized_times.push_back(par10_ms);
      }
      if (result.node_count_available && result.nodes >= 0) {
        ++summary.node_count_runs;
        if (benchmark_solved) {
          ++summary.solved_node_runs;
          solved_nodes.push_back(static_cast<double>(result.nodes));
        }
      }
      if (result.has_solution) {
        if (!result.audit.passed) ++summary.incumbent_audit_failures;
        if (std::isfinite(result.audit.max_row_violation)) {
          summary.max_row_violation = std::max(
              summary.max_row_violation, result.audit.max_row_violation);
        }
        if (std::isfinite(result.audit.max_bound_violation)) {
          summary.max_bound_violation = std::max(
              summary.max_bound_violation, result.audit.max_bound_violation);
        }
        if (std::isfinite(result.audit.max_integrality_violation)) {
          summary.max_integrality_violation = std::max(
              summary.max_integrality_violation,
              result.audit.max_integrality_violation);
        }
        if (std::isfinite(result.audit.objective_disagreement)) {
          summary.max_objective_disagreement = std::max(
              summary.max_objective_disagreement,
              result.audit.objective_disagreement);
        }
      }
    }
    if (!solved_times.empty()) {
      std::sort(solved_times.begin(), solved_times.end());
      const std::size_t mid = solved_times.size() / 2;
      summary.median_solved_ms = solved_times.size() % 2
          ? solved_times[mid]
          : 0.5 * (solved_times[mid - 1] + solved_times[mid]);
    }
    if (!penalized_times.empty()) {
      double log_sum = 0.0;
      for (double value : penalized_times) log_sum += std::log(value + kSummaryShiftMs);
      summary.shifted_geomean_par10_ms =
          std::exp(log_sum / static_cast<double>(penalized_times.size())) - kSummaryShiftMs;
    }
    if (!solved_nodes.empty()) {
      double log_sum = 0.0;
      for (double value : solved_nodes) {
        log_sum += std::log(value + kSummaryNodeShift);
      }
      summary.shifted_geomean_solved_nodes =
          std::exp(log_sum / static_cast<double>(solved_nodes.size())) -
          kSummaryNodeShift;
    }
    if (summary.primal_dual_integrals > 0) {
      summary.mean_primal_dual_integral_sec =
          pdi_sum_sec / static_cast<double>(summary.primal_dual_integrals);
    }
    summaries.push_back(summary);
  }
  return summaries;
}

json summary_json(const Summary& s) {
  return {{"solver", s.solver}, {"attempts", s.attempts},
          {"available", s.available}, {"feasible", s.feasible},
          {"optimal", s.optimal}, {"proven", s.proven},
          {"audited", s.audited}, {"reference_matches", s.reference_matches},
          {"timeouts", s.timeouts},
          {"benchmark_solved", s.benchmark_solved},
          {"node_count_runs", s.node_count_runs},
          {"solved_node_runs", s.solved_node_runs},
          {"shifted_geomean_solved_nodes",
           json_number(s.shifted_geomean_solved_nodes)},
          {"incumbent_audit_failures", s.incumbent_audit_failures},
          {"max_row_violation", s.max_row_violation},
          {"max_bound_violation", s.max_bound_violation},
          {"max_integrality_violation", s.max_integrality_violation},
          {"max_objective_disagreement", s.max_objective_disagreement},
          {"primal_dual_integrals", s.primal_dual_integrals},
          {"mean_primal_dual_integral_sec",
           json_number(s.mean_primal_dual_integral_sec)},
          {"median_solved_ms", json_number(s.median_solved_ms)},
          {"shifted_geomean_par10_ms", json_number(s.shifted_geomean_par10_ms)}};
}

void write_json(const fs::path& path, const Config& cfg,
                const std::vector<Result>& results,
                const std::vector<Summary>& summaries) {
  if (path.empty()) return;
  if (!path.parent_path().empty()) fs::create_directories(path.parent_path());
  json out;
  out["benchmark"] = "MIPLIB 2017";
  out["data_dir"] = cfg.data_dir.string();
  out["solution_file"] = cfg.solution_file.string();
  out["time_limit_sec"] = cfg.time_limit_sec;
  out["hard_timeout_grace_sec"] = cfg.hard_timeout_grace_sec;
#ifdef _WIN32
  out["hard_deadline_enforced"] = false;
#else
  out["hard_deadline_enforced"] = true;
#endif
  out["gap"] = cfg.gap;
  out["audit_tolerance"] = kAuditTolerance;
  out["reference_objective_tolerance"] =
      std::max(kAuditTolerance, cfg.gap);
  out["threads"] = 1;
  out["native_threads"] = cfg.native_threads;
  out["seed"] = cfg.seeds.size() == 1 ? json(cfg.seeds.front()) : json(nullptr);
  out["seeds"] = cfg.seeds;
  out["repeats"] = cfg.repeats;
  out["repeats_per_seed"] = cfg.repeats;
  out["solvers"] = cfg.solvers;
  out["case_filters"] = cfg.case_filters;
  out["highs_verbose"] = cfg.highs_verbose;
  out["native_tree_restart"] = {
      {"enabled", cfg.native_tree_restart},
      {"max_restarts", cfg.native_tree_restart_max},
      {"min_nodes_since_restart", cfg.native_tree_restart_min_nodes},
      {"min_open_nodes", cfg.native_tree_restart_min_open_nodes},
      {"min_relative_incumbent_improvement",
       cfg.native_tree_restart_min_improvement},
      {"min_remaining_time_sec",
       cfg.native_tree_restart_min_remaining_sec}};
  out["execution_order"] = {
      {"policy", "per_seed_base_permutation_with_block_rotation"},
      {"balance_unit", "instance_seed_repeat_block"},
      {"base_orders", json::array()}};
  for (int seed : cfg.seeds) {
    Config seeded_cfg = cfg;
    seeded_cfg.seed = seed;
    out["execution_order"]["base_orders"].push_back(
        {{"seed", seed}, {"order", base_solver_order(seeded_cfg)}});
  }
  Highs version_probe;
  out["solver_versions"] = {{"highs", version_probe.version()}};
#ifdef MIPSOLVERS_HAVE_SCIP_LIB
  out["solver_versions"]["scip"] =
      std::to_string(SCIPmajorVersion()) + "." +
      std::to_string(SCIPminorVersion()) + "." +
      std::to_string(SCIPtechVersion());
#else
  out["solver_versions"]["scip"] = nullptr;
#endif
#if defined(__clang__)
  out["compiler_id"] = "Clang " __clang_version__;
#elif defined(__GNUC__)
  out["compiler_id"] = "GCC " __VERSION__;
#elif defined(_MSC_VER)
  out["compiler_id"] = "MSVC " + std::to_string(_MSC_VER);
#else
  out["compiler_id"] = "unknown";
#endif
  out["max_nodes_native"] = cfg.max_nodes;
  out["native_node_estimate"] = cfg.native_node_estimate;
  out["native_papilo_presolve"] = cfg.native_papilo_presolve;
  out["native_presolve_probing"] = cfg.native_presolve_probing;
  out["native_top_level_presolve_policy"] =
      "run when PaPILO does not publish a reduced model";
  out["native_cuts"] = cfg.native_cuts;
  out["native_objective_propagation"] =
      cfg.native_cuts && cfg.native_objective_propagation;
  out["native_reduced_cost_fixing"] = cfg.native_reduced_cost_fixing;
  out["native_row_propagation_rounds"] =
      cfg.native_row_propagation_rounds;
  out["native_reliability_branching"] = {
      {"probe_max_candidates", cfg.native_probe_max},
      {"directional_reliability_threshold",
       cfg.native_probe_reliability}};
  out["native_primal_hint_file"] = cfg.native_primal_hint_file.empty()
      ? json(nullptr)
      : json(cfg.native_primal_hint_file.string());
  out["native_audit_hint_only"] = cfg.native_audit_hint_only;
  out["summary_policy"] = {
      {"solved", "proven and independently audited incumbent"},
      {"timeout_penalty", "PAR-10"}, {"shift_ms", kSummaryShiftMs},
      {"solved_node_shift", kSummaryNodeShift},
      {"node_scope", "solved runs with backend node count available"},
      {"primal_dual_integral",
       "piecewise-constant normalized gap over the fixed backend time limit; "
       "gap=1 before both bounds are available; every primal event is "
       "original-space audited"}};
  out["results"] = json::array();
  for (const Result& result : results) out["results"].push_back(result_json(result));
  out["summaries"] = json::array();
  for (const Summary& summary : summaries) out["summaries"].push_back(summary_json(summary));
  std::ofstream file(path);
  if (!file) throw std::runtime_error("cannot write " + path.string());
  file << std::setw(2) << out << '\n';
}

void print_result(const Result& result) {
  std::printf("%-24s %-20s %9.1f %9lld %10.3g %5s %5s %9.1e %s\n",
              result.instance.c_str(), result.solver.c_str(), result.solve_ms,
              static_cast<long long>(result.nodes), result.gap,
              result.optimal ? "yes" : "no", result.audit.passed ? "yes" : "no",
              result.audit.max_row_violation, result.status.c_str());
}

void print_summary(const std::vector<Summary>& summaries) {
  std::printf("\n%-20s %7s %7s %7s %8s %10s %14s %14s %10s\n",
              "Solver", "Runs", "Solved", "Feas", "AuditFail", "Timeouts",
              "PAR10-shift ms", "Solved-node sg", "Node runs");
  std::printf("%s\n", std::string(119, '-').c_str());
  for (const Summary& s : summaries) {
    std::printf("%-20s %7d %7d %7d %8d %10d %14.1f %14.1f %10d\n",
                s.solver.c_str(),
                s.attempts, s.benchmark_solved, s.feasible,
                s.incumbent_audit_failures, s.timeouts,
                s.shifted_geomean_par10_ms,
                s.shifted_geomean_solved_nodes, s.solved_node_runs);
  }
}

}  // namespace

int main(int argc, char** argv) {
  Config cfg;
  if (!parse_args(argc, argv, cfg)) return argc > 1 ? 1 : 0;
  if (cfg.native_node_estimate != "sum" &&
      cfg.native_node_estimate != "maximum") {
    std::cerr << "Unknown native node estimate: "
              << cfg.native_node_estimate << "\n";
    return 2;
  }
  const std::set<std::string> valid_solvers{
      "cplex-mip", "highs-mip", "scip-mip", "native-highs-lp",
      "native-native-lp",
      "native-presolve-highs-audit"};

  const bool any_worker_option = !cfg.worker_instance.empty() ||
                                 !cfg.worker_output.empty() ||
                                 !cfg.worker_solver.empty();
  if (any_worker_option) {
    if (cfg.worker_instance.empty() || cfg.worker_output.empty() ||
        cfg.worker_solver.empty()) {
      std::cerr << "Worker mode requires --worker-instance, --worker-output, "
                   "and --worker-solver\n";
      return 2;
    }
    if (!valid_solvers.count(cfg.worker_solver)) {
      std::cerr << "Unknown worker solver: " << cfg.worker_solver << "\n";
      return 2;
    }
    Instance instance = load_instance(cfg.worker_instance);
    if (!instance.error.empty()) {
      std::cerr << "Worker model load failed: " << instance.error << "\n";
      return 3;
    }
    Result result = run_solver(instance, cfg, cfg.worker_solver);
    try {
      if (!cfg.worker_output.parent_path().empty()) {
        fs::create_directories(cfg.worker_output.parent_path());
      }
      std::ofstream output(cfg.worker_output);
      if (!output) {
        throw std::runtime_error("cannot write " + cfg.worker_output.string());
      }
      output << std::setw(2) << worker_result_json(result) << '\n';
    } catch (const std::exception& error) {
      std::cerr << error.what() << "\n";
      return 3;
    }
    return 0;
  }

  for (const std::string& solver : cfg.solvers) {
    if (!valid_solvers.count(solver)) {
      std::cerr << "Unknown solver: " << solver << "\n";
      return 2;
    }
  }

  if (cfg.solution_file.empty()) {
    for (const fs::path& candidate : {
             cfg.data_dir / "benchmark.solu",
             cfg.data_dir.parent_path() / "benchmark.solu",
             cfg.data_dir.parent_path() / "miplib2017-v36.solu",
             fs::path("tests/data/miplib2017/miplib2017-v36.solu"),
             fs::path("tests/data/miplib2017/benchmark.solu")}) {
      if (fs::exists(candidate)) {
        cfg.solution_file = candidate;
        break;
      }
    }
  }
  const auto references = read_references(cfg.solution_file);
  const auto paths = find_instances(cfg);
  if (paths.empty()) {
    std::cerr << "No .mps or .mps.gz instances under " << cfg.data_dir << "\n";
    return 2;
  }

  std::printf("MIPLIB 2017: %zu instances, %zu solvers, %zu seed(s), "
              "%d repeat(s)/seed, %.3g s, gap %.3g\n",
              paths.size(), cfg.solvers.size(), cfg.seeds.size(), cfg.repeats,
              cfg.time_limit_sec, cfg.gap);
  std::printf("%-24s %-20s %9s %9s %10s %5s %5s %9s %s\n",
              "Instance", "Solver", "Solve ms", "Nodes", "Gap", "Opt", "Audit",
              "RowViol", "Status");
  std::printf("%s\n", std::string(125, '-').c_str());

  std::error_code executable_error;
  fs::path executable = fs::weakly_canonical(fs::absolute(argv[0]),
                                             executable_error);
  if (executable_error || executable.empty()) {
    executable = fs::absolute(argv[0]);
  }

  std::vector<Result> results;
  int execution_index = 0;
  for (std::size_t path_index = 0; path_index < paths.size(); ++path_index) {
    const fs::path& path = paths[path_index];
    Instance instance = load_instance(path);
    if (!instance.error.empty()) {
      std::fprintf(stderr, "%s: %s\n", instance.name.c_str(), instance.error.c_str());
      for (std::size_t seed_index = 0; seed_index < cfg.seeds.size();
           ++seed_index) {
        Config seeded_cfg = cfg;
        seeded_cfg.seed = cfg.seeds[seed_index];
        const auto base_order = base_solver_order(seeded_cfg);
        for (int repeat = 0; repeat < cfg.repeats; ++repeat) {
          const int block = static_cast<int>(seed_index) * cfg.repeats + repeat;
          const auto order = blocked_solver_order(base_order, path_index, block);
          for (std::size_t position = 0; position < order.size(); ++position) {
            const std::string& solver = order[position];
            Result result;
            set_dimensions(instance, result);
            result.instance = instance.name;
            result.solver = solver;
            result.seed = seeded_cfg.seed;
            result.repeat = repeat;
            result.execution_index = execution_index++;
            result.block_order_position = static_cast<int>(position);
            result.available = false;
            result.status = "model load: " + instance.error;
            print_result(result);
            results.push_back(std::move(result));
          }
        }
      }
      if (instance.solver_path_is_temporary) {
        std::error_code ec;
        fs::remove(instance.solver_path, ec);
      }
      continue;
    }
    const auto ref_it = references.find(instance.name);
    const Reference* reference = ref_it == references.end() ? nullptr : &ref_it->second;
    for (std::size_t seed_index = 0; seed_index < cfg.seeds.size();
         ++seed_index) {
      Config seeded_cfg = cfg;
      seeded_cfg.seed = cfg.seeds[seed_index];
      const auto base_order = base_solver_order(seeded_cfg);
      for (int repeat = 0; repeat < cfg.repeats; ++repeat) {
        const int block = static_cast<int>(seed_index) * cfg.repeats + repeat;
        const auto order = blocked_solver_order(base_order, path_index, block);
        for (std::size_t position = 0; position < order.size(); ++position) {
          const std::string& solver = order[position];
          Result result = run_solver_isolated(
              instance, seeded_cfg, solver, executable);
          result.seed = seeded_cfg.seed;
          result.repeat = repeat;
          result.execution_index = execution_index++;
          result.block_order_position = static_cast<int>(position);
          attach_validation(instance, reference, seeded_cfg, result);
          print_result(result);
          results.push_back(std::move(result));
        }
      }
    }
    if (instance.solver_path_is_temporary) {
      std::error_code ec;
      fs::remove(instance.solver_path, ec);
    }
  }

  const std::vector<Summary> summaries = summarize(cfg, results);
  print_summary(summaries);
  try {
    write_csv(cfg.csv_path, results);
    write_json(cfg.json_path, cfg, results, summaries);
  } catch (const std::exception& error) {
    std::cerr << error.what() << "\n";
    return 3;
  }
  return results.empty() ? 2 : 0;
}
