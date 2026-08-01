/**
 * @file mipsolvers_py.cpp
 * @brief pybind11 Python bindings for MIPSolvers: SCUC market dispatch module
 *        and SolverEngine LP/MILP interface.
 *
 * Python 3.8+ compatible. Build with CMake option -DMIPSOLVERS_BUILD_PYTHON=ON.
 */

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>

#include <cctype>
#include <algorithm>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

// AML forward declaration — defined in aml_bindings.cpp
void bind_aml(pybind11::module_& parent);

// SCUC public API
#include "mipsolvers/scuc/case_builder.hpp"
#include "mipsolvers/scuc/scuc.hpp"

// Engine public API
#include "mipsolvers/engine/branch_and_cut.hpp"
#include "mipsolvers/engine/engine.hpp"
#include "mipsolvers/engine/api/options.hpp"
#include "mipsolvers/engine/api/result.hpp"
#include "mipsolvers/engine/problem_types.hpp"
#include "mipsolvers/engine/solver/native/native_adapters.hpp"
#include "mipsolvers/l2o/branching_policy.hpp"
#include "mipsolvers/l2o/model_fingerprint.hpp"
#include "mipsolvers/l2o/policy.hpp"
#include "mipsolvers/l2o/scuc_warm_start.hpp"
#include "mipsolvers/l2o/solver_config_policy.hpp"

namespace py = pybind11;
using namespace pybind11::literals;

// ─────────────────────────────────────────────────────────────────────────────
// Helpers: numpy <-> Eigen
// ─────────────────────────────────────────────────────────────────────────────
namespace {

[[noreturn]] void rethrow_scuc_exception(const std::string& context,
                                         const std::exception& ex) {
  throw std::runtime_error(context + ": " + ex.what());
}

mipsolvers::scuc::SCUCInput scuc_from_json_checked(const std::string& json_str) {
  try {
    return mipsolvers::scuc::scuc_from_json(json_str);
  } catch (const nlohmann::json::parse_error& ex) {
    rethrow_scuc_exception("SCUC JSON parse error", ex);
  } catch (const nlohmann::json::type_error& ex) {
    rethrow_scuc_exception("SCUC JSON type error", ex);
  } catch (const nlohmann::json::out_of_range& ex) {
    rethrow_scuc_exception("SCUC JSON field error", ex);
  } catch (const nlohmann::json::exception& ex) {
    rethrow_scuc_exception("SCUC JSON error", ex);
  }
}

std::string scuc_output_to_json_checked(const mipsolvers::scuc::SCUCOutput& out,
                                        const mipsolvers::scuc::SCUCInput& inp,
                                        int indent) {
  try {
    return mipsolvers::scuc::scuc_output_to_json(out, inp, indent);
  } catch (const nlohmann::json::exception& ex) {
    rethrow_scuc_exception("SCUC JSON serialization error", ex);
  }
}

void require_vector_size(const Eigen::VectorXd& vec,
                         int expected,
                         const std::string& name) {
  if (vec.size() != expected) {
    throw std::invalid_argument(name + " length must equal c.size");
  }
}

void require_matrix_cols(const Eigen::SparseMatrix<double>& matrix,
                         int expected,
                         const std::string& name) {
  if (matrix.cols() != expected) {
    throw std::invalid_argument(name + ".shape[1] must equal c.size");
  }
}

/// 1-D numpy float64 array -> Eigen::VectorXd
Eigen::VectorXd vec_from_numpy(const py::array_t<double>& arr) {
  py::buffer_info info = arr.request();
  if (info.ndim != 1)
    throw std::invalid_argument("Expected 1-D float64 array");
  Eigen::VectorXd v(static_cast<Eigen::Index>(info.shape[0]));
  const double* ptr = static_cast<const double*>(info.ptr);
  for (Eigen::Index i = 0; i < v.size(); ++i) v[i] = ptr[i];
  return v;
}

/// Eigen::VectorXd -> 1-D numpy float64 array
py::array_t<double> vec_to_numpy(const Eigen::VectorXd& v) {
  py::array_t<double> arr(static_cast<py::ssize_t>(v.size()));
  double* ptr = arr.mutable_data();
  for (Eigen::Index i = 0; i < v.size(); ++i) ptr[i] = v[i];
  return arr;
}

/// Dense 2-D numpy float64 [rows, cols] -> Eigen::SparseMatrix<double> (row-major scan)
Eigen::SparseMatrix<double> sparse_from_dense(const py::array_t<double>& arr) {
  py::buffer_info info = arr.request();
  if (info.ndim != 2)
    throw std::invalid_argument("Expected 2-D float64 array for matrix A");
  const Eigen::Index rows = static_cast<Eigen::Index>(info.shape[0]);
  const Eigen::Index cols = static_cast<Eigen::Index>(info.shape[1]);
  const double* ptr = static_cast<const double*>(info.ptr);
  const Eigen::Index row_stride = static_cast<Eigen::Index>(info.strides[0] / sizeof(double));
  const Eigen::Index col_stride = static_cast<Eigen::Index>(info.strides[1] / sizeof(double));

  std::vector<Eigen::Triplet<double>> triplets;
  triplets.reserve(static_cast<size_t>(rows * cols / 4));  // rough estimate
  for (Eigen::Index r = 0; r < rows; ++r)
    for (Eigen::Index c = 0; c < cols; ++c) {
      const double v = ptr[r * row_stride + c * col_stride];
      if (v != 0.0) triplets.emplace_back(r, c, v);
    }
  Eigen::SparseMatrix<double> sp(rows, cols);
  sp.setFromTriplets(triplets.begin(), triplets.end());
  return sp;
}

/// COO triplets (rows[], cols[], data[]) + shape -> Eigen::SparseMatrix<double>
/// (reserved for future scipy.sparse interop)
[[maybe_unused]]
Eigen::SparseMatrix<double> sparse_from_coo(
    const py::array_t<int>& row_arr,
    const py::array_t<int>& col_arr,
    const py::array_t<double>& data_arr,
    int n_rows, int n_cols) {
  py::buffer_info ri = row_arr.request();
  py::buffer_info ci = col_arr.request();
  py::buffer_info di = data_arr.request();
  if (ri.ndim != 1 || ci.ndim != 1 || di.ndim != 1)
    throw std::invalid_argument("COO arrays must be 1-D");
  const py::ssize_t nnz = ri.shape[0];
  if (ci.shape[0] != nnz || di.shape[0] != nnz)
    throw std::invalid_argument("COO arrays must have equal length");

  const int* rp = static_cast<const int*>(ri.ptr);
  const int* cp = static_cast<const int*>(ci.ptr);
  const double* dp = static_cast<const double*>(di.ptr);

  std::vector<Eigen::Triplet<double>> triplets;
  triplets.reserve(static_cast<size_t>(nnz));
  for (py::ssize_t k = 0; k < nnz; ++k)
    triplets.emplace_back(rp[k], cp[k], dp[k]);

  Eigen::SparseMatrix<double> sp(n_rows, n_cols);
  sp.setFromTriplets(triplets.begin(), triplets.end());
  return sp;
}

mipsolvers::scuc::Matrix2D matrix_from_python_2d(const py::object& obj,
                                                const std::string& name) {
  if (obj.is_none()) return {};
  if (py::isinstance<py::array>(obj)) {
    auto arr = py::array_t<double, py::array::c_style | py::array::forcecast>::ensure(obj);
    if (!arr) throw std::invalid_argument(name + " must be convertible to a 2-D float64 array");
    py::buffer_info info = arr.request();
    if (info.ndim != 2) throw std::invalid_argument(name + " must be 2-D");
    const auto rows = static_cast<size_t>(info.shape[0]);
    const auto cols = static_cast<size_t>(info.shape[1]);
    const double* data = static_cast<const double*>(info.ptr);
    mipsolvers::scuc::Matrix2D out(rows, std::vector<double>(cols, 0.0));
    for (size_t r = 0; r < rows; ++r) {
      for (size_t c = 0; c < cols; ++c) out[r][c] = data[r * cols + c];
    }
    return out;
  }
  return obj.cast<mipsolvers::scuc::Matrix2D>();
}

py::array_t<double> matrix_to_numpy_2d(const mipsolvers::scuc::Matrix2D& matrix) {
  const py::ssize_t rows = static_cast<py::ssize_t>(matrix.size());
  py::ssize_t cols = 0;
  for (const auto& row : matrix) cols = std::max(cols, static_cast<py::ssize_t>(row.size()));
  py::array_t<double> arr({rows, cols});
  auto data = arr.mutable_unchecked<2>();
  for (py::ssize_t r = 0; r < rows; ++r) {
    for (py::ssize_t c = 0; c < cols; ++c) {
      data(r, c) = c < static_cast<py::ssize_t>(matrix[static_cast<size_t>(r)].size())
                       ? matrix[static_cast<size_t>(r)][static_cast<size_t>(c)]
                       : 0.0;
    }
  }
  return arr;
}

double profile_value(const std::vector<std::vector<double>>& profiles,
                     int row,
                     int t,
                     double fallback) {
  if (row >= 0 && row < static_cast<int>(profiles.size()) &&
      t >= 0 && t < static_cast<int>(profiles[static_cast<size_t>(row)].size())) {
    return profiles[static_cast<size_t>(row)][static_cast<size_t>(t)];
  }
  return fallback;
}

double total_load_mw(const mipsolvers::scuc::SCUCInput& inp, int t) {
  double total = 0.0;
  for (int d = 0; d < static_cast<int>(inp.loads.size()); ++d) {
    const double multiplier = profile_value(inp.profiles.load, d, t, 1.0);
    total += inp.loads[static_cast<size_t>(d)].p_mw * multiplier;
  }
  return total;
}

double total_wind_mw(const mipsolvers::scuc::SCUCInput& inp, int t) {
  double total = 0.0;
  for (int w = 0; w < static_cast<int>(inp.wind.size()); ++w) {
    total += profile_value(inp.profiles.wind, w, t, inp.wind[static_cast<size_t>(w)].pmax);
  }
  return total;
}

double total_solar_mw(const mipsolvers::scuc::SCUCInput& inp, int t) {
  double total = 0.0;
  for (int s = 0; s < static_cast<int>(inp.solar.size()); ++s) {
    total += profile_value(inp.profiles.solar, s, t, inp.solar[static_cast<size_t>(s)].pmax);
  }
  return total;
}

py::dict scuc_warm_start_report_to_py(const mipsolvers::l2o::SCUCWarmStartReport& report) {
  py::dict d;
  d["success"] = report.success;
  d["message"] = report.message;
  d["ng"] = report.ng;
  d["T"] = report.T;
  d["num_ig_set"] = report.num_ig_set;
  d["num_su_set"] = report.num_su_set;
  d["num_sd_set"] = report.num_sd_set;
  d["num_missing_cols"] = report.num_missing_cols;
  d["num_out_of_range_predictions"] = report.num_out_of_range_predictions;
  d["used_existing_seed"] = report.used_existing_seed;
  d["inferred_startup"] = report.inferred_startup;
  d["inferred_shutdown"] = report.inferred_shutdown;
  return d;
}

mipsolvers::l2o::SCUCWarmStartOptions make_scuc_warm_start_options(
    double binary_threshold,
    bool infer_missing_transitions,
    bool initialize_from_existing_seed) {
  mipsolvers::l2o::SCUCWarmStartOptions options;
  options.binary_threshold = binary_threshold;
  options.infer_missing_transitions = infer_missing_transitions;
  options.initialize_from_existing_seed = initialize_from_existing_seed;
  return options;
}

py::object json_to_py(const nlohmann::json& j) {
  if (j.is_null()) return py::none();
  if (j.is_boolean()) return py::bool_(j.get<bool>());
  if (j.is_number_integer()) return py::int_(j.get<long long>());
  if (j.is_number_unsigned()) return py::int_(j.get<unsigned long long>());
  if (j.is_number_float()) return py::float_(j.get<double>());
  if (j.is_string()) return py::str(j.get<std::string>());
  if (j.is_array()) {
    py::list out;
    for (const auto& item : j) out.append(json_to_py(item));
    return std::move(out);
  }
  py::dict out;
  for (auto it = j.begin(); it != j.end(); ++it) {
    out[py::str(it.key())] = json_to_py(it.value());
  }
  return std::move(out);
}

void set_initial_solution_from_python(mipsolvers::engine::MIPModel& mip,
                                      const py::object& initial_solution) {
  if (initial_solution.is_none()) return;
  auto x0 = vec_from_numpy(initial_solution.cast<py::array_t<double>>());
  require_vector_size(x0, static_cast<int>(mip.linear_part.c.size()), "initial_solution");
  mip.initial_solution = std::move(x0);
}

template <typename Stats>
nlohmann::json solve_stats_to_json(const Stats& stats) {
  return nlohmann::json{
      {"success", stats.success},
      {"iterations", stats.iterations},
      {"objective", stats.objective},
      {"residual_inf", stats.residual_inf},
      {"primal_feas", stats.primal_feas},
      {"dual_feas", stats.dual_feas},
      {"complementarity", stats.complementarity},
      {"mip_gap", stats.mip_gap},
      {"runtime_sec", stats.runtime_sec},
      {"status", stats.status},
      {"solver_name", stats.solver_name},
      {"cglp_cuts_added", stats.cglp_cuts_added},
      {"has_farkas_certificate", stats.has_farkas_certificate}};
}

nlohmann::json bc_stats_to_json(const mipsolvers::engine::BCStats& stats) {
  return nlohmann::json{
      {"status", stats.status},
      {"nodes_explored", stats.nodes_explored},
      {"lp_solves", stats.lp_solves},
      {"cuts_added", stats.cuts_added},
      {"root_cuts_added", stats.root_cuts_added},
      {"tree_cuts_added", stats.tree_cuts_added},
      {"root_gomory_cuts", stats.root_gomory_cuts},
      {"root_mir_cuts", stats.root_mir_cuts},
      {"root_cover_cuts", stats.root_cover_cuts},
      {"root_clique_cuts", stats.root_clique_cuts},
      {"root_zerohalf_cuts", stats.root_zerohalf_cuts},
      {"root_flowcover_cuts", stats.root_flowcover_cuts},
      {"root_impliedbound_cuts", stats.root_impliedbound_cuts},
      {"root_presolved_rows", stats.root_presolved_rows},
      {"root_presolved_cols", stats.root_presolved_cols},
      {"root_cut_bound_lift", stats.root_cut_bound_lift},
      {"best_bound", stats.best_bound},
      {"best_obj", stats.best_obj},
      {"gap", stats.gap},
      {"root_only_bound", stats.root_only_bound},
      {"runtime_sec", stats.runtime_sec},
      {"incumbent_updates", stats.incumbent_updates},
      {"first_incumbent_node", stats.first_incumbent_node},
      {"last_incumbent_node", stats.last_incumbent_node},
      {"first_incumbent_lp_solves", stats.first_incumbent_lp_solves},
      {"last_incumbent_lp_solves", stats.last_incumbent_lp_solves},
      {"best_bound_at_last_incumbent", stats.best_bound_at_last_incumbent},
      {"best_bound_lift_after_last_incumbent", stats.best_bound_lift_after_last_incumbent},
      {"incumbent_repair_lp_attempts", stats.incumbent_repair_lp_attempts},
      {"incumbent_repair_improvements", stats.incumbent_repair_improvements},
      {"incumbent_repair_time_ms", stats.incumbent_repair_time_ms},
      {"reliability_branch_nodes", stats.reliability_branch_nodes},
      {"strong_branch_candidates", stats.strong_branch_candidates},
      {"strong_branch_lp_solves", stats.strong_branch_lp_solves},
      {"strong_branch_cache_exact_hits", stats.strong_branch_cache_exact_hits},
      {"strong_branch_cache_warm_hits", stats.strong_branch_cache_warm_hits},
      {"strong_branch_duplicate_lp_avoided", stats.strong_branch_duplicate_lp_avoided},
      {"branching_regret_samples", stats.branching_regret_samples},
      {"branching_regret_sum", stats.branching_regret_sum},
      {"branching_regret_max", stats.branching_regret_max},
      {"strong_branch_regret_samples", stats.strong_branch_regret_samples},
      {"strong_branch_regret_sum", stats.strong_branch_regret_sum},
      {"strong_branch_regret_max", stats.strong_branch_regret_max},
      {"parallel_requested_threads", stats.parallel_requested_threads},
      {"parallel_effective_threads", stats.parallel_effective_threads},
      {"parallel_tree_launched", stats.parallel_tree_launched},
      {"parallel_schedule_reason", stats.parallel_schedule_reason},
      {"pool_cuts_separated", stats.pool_cuts_separated},
      {"cut_worker_cuts", stats.cut_worker_cuts},
      {"fallback_events", stats.fallback_events},
      {"fallback_recoveries", stats.fallback_recoveries},
      {"cglp_candidates", stats.cglp_candidates},
      {"cglp_lp_solved", stats.cglp_lp_solved},
      {"cglp_accepted", stats.cglp_accepted},
      {"work_steals", stats.work_steals},
      {"propagation_calls", stats.propagation_calls},
      {"propagation_bound_tightenings", stats.propagation_bound_tightenings},
      {"local_node_cuts_generated", stats.local_node_cuts_generated},
      {"local_node_cuts_added", stats.local_node_cuts_added},
      {"lazy_constraints_added", stats.lazy_constraints_added},
      {"invalid_cut_coefficients", stats.invalid_cut_coefficients}};
}

nlohmann::json root_cuts_to_json(const mipsolvers::engine::BCRootCuts& cuts,
                                 bool include_vectors) {
  nlohmann::json j{
      {"num_cuts", cuts.numCuts()},
      {"num_nonzeros", static_cast<int>(cuts.index.size())}};
  if (include_vectors) {
    j["start"] = cuts.start;
    j["index"] = cuts.index;
    j["value"] = cuts.value;
    j["lower"] = cuts.lower;
    j["upper"] = cuts.upper;
  }
  return j;
}

nlohmann::json root_basis_to_json(const mipsolvers::engine::BCRootBasis& basis,
                                  bool include_vectors) {
  nlohmann::json j{
      {"valid", basis.valid},
      {"num_col_status", static_cast<int>(basis.col_status.size())},
      {"num_row_status", static_cast<int>(basis.row_status.size())}};
  if (include_vectors) {
    j["col_status"] = basis.col_status;
    j["row_status"] = basis.row_status;
  }
  return j;
}

nlohmann::json pseudocost_to_json(const mipsolvers::engine::BCPseudocostInit& pc,
                                  bool include_vectors) {
  nlohmann::json j{
      {"n_orig_cols", pc.n_orig_cols},
      {"nsamplestotal", pc.nsamplestotal},
      {"ninferencestotal", pc.ninferencestotal},
      {"cost_total", pc.cost_total},
      {"inferences_total", pc.inferences_total},
      {"conflict_avg_score", pc.conflict_avg_score}};
  if (include_vectors) {
    j["pseudocostup"] = pc.pseudocostup;
    j["pseudocostdown"] = pc.pseudocostdown;
    j["nsamplesup"] = pc.nsamplesup;
    j["nsamplesdown"] = pc.nsamplesdown;
    j["inferencesup"] = pc.inferencesup;
    j["inferencesdown"] = pc.inferencesdown;
    j["ninferencesup"] = pc.ninferencesup;
    j["ninferencesdown"] = pc.ninferencesdown;
    j["conflictscoreup"] = pc.conflictscoreup;
    j["conflictscoredown"] = pc.conflictscoredown;
  }
  return j;
}

py::dict solve_result_to_py_dict(const mipsolvers::engine::SolveResult& res) {
  py::dict d;
  d["success"] = res.stats.success;
  d["objective"] = res.stats.objective;
  d["status"] = res.stats.status;
  d["solver"] = res.stats.solver_name;
  d["mip_gap"] = res.stats.mip_gap;
  d["runtime_sec"] = res.stats.runtime_sec;
  d["x"] = vec_to_numpy(res.x);
  d["stats"] = json_to_py(solve_stats_to_json(res.stats));
  if (res.constraint_duals.size() > 0) d["duals"] = vec_to_numpy(res.constraint_duals);
  if (res.box_dual_lb.size() > 0) d["box_dual_lb"] = vec_to_numpy(res.box_dual_lb);
  if (res.box_dual_ub.size() > 0) d["box_dual_ub"] = vec_to_numpy(res.box_dual_ub);
  return d;
}

py::dict api_result_to_py_dict(const mipsolvers::engine::api::Result& res) {
  py::dict d;
  d["success"] = res.stats.success;
  d["objective"] = res.stats.objective;
  d["status"] = res.stats.status;
  d["solver"] = res.stats.solver_name;
  d["iterations"] = res.stats.iterations;
  d["mip_gap"] = res.stats.mip_gap;
  d["runtime_sec"] = res.stats.runtime_sec;
  d["x"] = vec_to_numpy(res.x);
  d["stats"] = json_to_py(solve_stats_to_json(res.stats));
  if (res.constraint_duals.size() > 0) d["duals"] = vec_to_numpy(res.constraint_duals);
  if (res.box_dual_lb.size() > 0) d["box_dual_lb"] = vec_to_numpy(res.box_dual_lb);
  if (res.box_dual_ub.size() > 0) d["box_dual_ub"] = vec_to_numpy(res.box_dual_ub);
  return d;
}

py::dict bc_result_to_py_dict(const mipsolvers::engine::BCResult& res,
                              bool include_artifact_vectors) {
  py::dict d;
  d["success"] = res.stats.success;
  d["objective"] = res.stats.objective;
  d["status"] = res.stats.status;
  d["solver"] = res.stats.solver_name;
  d["mip_gap"] = res.stats.mip_gap;
  d["runtime_sec"] = res.stats.runtime_sec;
  d["x"] = vec_to_numpy(res.x);
  d["stats"] = json_to_py(solve_stats_to_json(res.stats));
  d["bc_stats"] = json_to_py(bc_stats_to_json(res.bc_stats));

  nlohmann::json artifacts = nlohmann::json::object();
  artifacts["has_root_cuts"] = static_cast<bool>(res.highs_root_cuts) && !res.highs_root_cuts->empty();
  artifacts["has_root_basis"] = static_cast<bool>(res.highs_root_basis) && !res.highs_root_basis->empty();
  artifacts["has_pseudocost_init"] = static_cast<bool>(res.highs_pseudocost_init) && !res.highs_pseudocost_init->empty();
  if (res.highs_root_cuts) {
    artifacts["root_cuts"] = root_cuts_to_json(*res.highs_root_cuts, include_artifact_vectors);
  }
  if (res.highs_root_basis) {
    artifacts["root_basis"] = root_basis_to_json(*res.highs_root_basis, include_artifact_vectors);
  }
  if (res.highs_pseudocost_init) {
    artifacts["pseudocost_init"] = pseudocost_to_json(*res.highs_pseudocost_init, include_artifact_vectors);
  }
  d["artifacts"] = json_to_py(artifacts);
  return d;
}

bool is_native_milp_solver_name(const std::string& solver) {
  return solver.empty() || solver == "Auto" || solver == "StrictHiGHS" ||
         solver == "NativeBranchAndCut" || solver == "B&C" || solver == "BC";
}

mipsolvers::engine::BCOptions make_python_bc_options(double mip_gap,
                                                     double time_limit_sec) {
  mipsolvers::engine::BCOptions opt;
  opt.gap_tol = mip_gap;
  opt.time_limit_sec = time_limit_sec;
  return opt;
}

mipsolvers::engine::SolveResult solve_native_milp_with_options(
    const mipsolvers::engine::MIPModel& mip,
    const std::string& solver,
    const mipsolvers::engine::BCOptions& opt) {
  if (solver == "NativeBranchAndCut" || solver == "B&C" || solver == "BC") {
    mipsolvers::engine::NativeBranchAndCutAdapter adapter(opt);
    return adapter.solve_milp(mip);
  }
  mipsolvers::engine::StrictHighsBranchAndCutAdapter adapter(opt);
  return adapter.solve_milp(mip);
}

nlohmann::json uc_hint_to_json(const mipsolvers::engine::MIPModel::UCGenHint& h) {
  return nlohmann::json{
      {"ng", h.ng},
      {"T", h.T},
      {"period_hours", h.period_hours},
      {"ig_start", h.ig_start},
      {"su_start", h.su_start},
      {"sd_start", h.sd_start},
      {"pg_start", h.pg_start},
      {"ig_cols", h.ig_cols},
      {"su_cols", h.su_cols},
      {"sd_cols", h.sd_cols},
      {"pg_cols", h.pg_cols},
      {"min_up", h.min_up},
      {"min_down", h.min_down},
      {"ig0", h.ig0},
      {"pmin", h.pmin},
      {"pmax", h.pmax},
      {"ramp", h.ramp},
      {"demand", h.demand},
      {"reserve_requirement", h.reserve_requirement},
      {"spinning_requirement", h.spinning_requirement},
      {"regulation_up_requirement", h.regulation_up_requirement},
      {"regulation_down_requirement", h.regulation_down_requirement},
      {"n_areas", h.n_areas},
      {"gen_area", h.gen_area},
      {"area_demand", h.area_demand},
      {"area_reserve_requirement", h.area_reserve_requirement},
      {"network_line_count", h.network_line_count},
      {"line_gsf", h.line_gsf},
      {"line_fwd_rhs", h.line_fwd_rhs},
      {"line_rev_rhs", h.line_rev_rhs},
      {"section_count", h.section_count},
      {"section_gsf", h.section_gsf},
      {"section_fwd_rhs", h.section_fwd_rhs},
      {"section_rev_rhs", h.section_rev_rhs},
      {"n_segments", h.n_segments},
      {"segment_cols", h.segment_cols},
      {"segment_cap", h.segment_cap},
      {"n_storage", h.n_storage},
      {"storage_charge_cols", h.storage_charge_cols},
      {"storage_discharge_cols", h.storage_discharge_cols},
      {"storage_energy_capacity", h.storage_energy_capacity},
      {"storage_efficiency", h.storage_efficiency},
      {"storage_initial_energy", h.storage_initial_energy},
      {"storage_cycle_limit", h.storage_cycle_limit},
      {"certifies_power_balance_rows", h.certifies_power_balance_rows},
      {"certifies_generation_capacity_rows", h.certifies_generation_capacity_rows},
      {"certifies_ramping_rows", h.certifies_ramping_rows},
      {"certifies_min_up_down_rows", h.certifies_min_up_down_rows},
      {"certifies_segment_bound_rows", h.certifies_segment_bound_rows},
      {"certifies_system_reserve_rows", h.certifies_system_reserve_rows},
      {"certifies_area_import_rows", h.certifies_area_import_rows},
      {"certifies_hard_network_flow_rows", h.certifies_hard_network_flow_rows},
      {"certifies_hard_section_flow_rows", h.certifies_hard_section_flow_rows},
      {"certifies_storage_cycle_rows", h.certifies_storage_cycle_rows}};
}

nlohmann::json mip_summary_to_json(
    const mipsolvers::engine::MIPModel& mip,
    const mipsolvers::l2o::FingerprintOptions& fingerprint_options) {
  const auto fingerprint = mipsolvers::l2o::fingerprint_mip(mip, fingerprint_options);
  nlohmann::json j = nlohmann::json::object();
  j["fingerprint"] = fingerprint;
  j["num_variables"] = static_cast<int>(mip.linear_part.c.size());
  j["num_ineq_rows"] = static_cast<int>(mip.linear_part.A.rows());
  j["num_eq_rows"] = static_cast<int>(mip.linear_part.Aeq.rows());
  j["num_ineq_nonzeros"] = static_cast<int>(mip.linear_part.A.nonZeros());
  j["num_eq_nonzeros"] = static_cast<int>(mip.linear_part.Aeq.nonZeros());
  j["num_integer_vars"] = static_cast<int>(mip.integer_idx.size());
  j["num_binary_vars"] = static_cast<int>(mip.binary_idx.size());
  j["integer_idx"] = mip.integer_idx;
  j["binary_idx"] = mip.binary_idx;
  j["branching_priority"] = mip.branching_priority;
  j["has_initial_solution"] = mip.initial_solution.size() > 0;
  j["has_uc_hint"] = mip.uc_hint.has_value();
  if (mip.uc_hint) j["uc_hint"] = uc_hint_to_json(*mip.uc_hint);
  return j;
}

/// Build engine::LPModel from numpy arrays.
/// A, Aeq may be None (passed as py::none), or 2-D numpy arrays.
/// lb / ub may be None (defaults to -inf / +inf per variable).
mipsolvers::engine::LPModel build_lp_model(
    const py::array_t<double>& c,
    const py::object& A,
    const py::object& b,
    const py::object& Aeq,
    const py::object& beq,
    const py::object& lb,
    const py::object& ub,
    bool maximize) {
  using namespace mipsolvers::engine;
  LPModel lp;
  lp.sense = maximize ? Sense::Maximize : Sense::Minimize;
  lp.c = vec_from_numpy(c);
  const int n = static_cast<int>(lp.c.size());

  if (n <= 0) {
    throw std::invalid_argument("c must be a non-empty 1-D float64 array");
  }

  if (A.is_none() != b.is_none()) {
    throw std::invalid_argument("A and b must be provided together");
  }
  if (Aeq.is_none() != beq.is_none()) {
    throw std::invalid_argument("Aeq and beq must be provided together");
  }

  if (!A.is_none() && !b.is_none()) {
    lp.A   = sparse_from_dense(A.cast<py::array_t<double>>());
    lp.b   = vec_from_numpy(b.cast<py::array_t<double>>());
    require_matrix_cols(lp.A, n, "A");
    if (lp.b.size() != lp.A.rows()) {
      throw std::invalid_argument("b length must equal A.shape[0]");
    }
  }
  if (!Aeq.is_none() && !beq.is_none()) {
    lp.Aeq  = sparse_from_dense(Aeq.cast<py::array_t<double>>());
    lp.beq  = vec_from_numpy(beq.cast<py::array_t<double>>());
    require_matrix_cols(lp.Aeq, n, "Aeq");
    if (lp.beq.size() != lp.Aeq.rows()) {
      throw std::invalid_argument("beq length must equal Aeq.shape[0]");
    }
  }

  if (lp.A.rows() == 0 && lp.A.cols() != n) lp.A.resize(0, n);
  if (lp.Aeq.rows() == 0 && lp.Aeq.cols() != n) lp.Aeq.resize(0, n);

  lp.vars.resize(static_cast<size_t>(n));
  if (!lb.is_none()) {
    auto lbv = vec_from_numpy(lb.cast<py::array_t<double>>());
    require_vector_size(lbv, n, "lb");
    for (int i = 0; i < n; ++i) lp.vars[static_cast<size_t>(i)].lb = lbv[i];
  }
  if (!ub.is_none()) {
    auto ubv = vec_from_numpy(ub.cast<py::array_t<double>>());
    require_vector_size(ubv, n, "ub");
    for (int i = 0; i < n; ++i) lp.vars[static_cast<size_t>(i)].ub = ubv[i];
  }
  return lp;
}

/// Build engine::MIPModel from numpy arrays.
/// vartypes: list of "C"/"I"/"B" strings (or single char), length n.
mipsolvers::engine::MIPModel build_mip_model(
    const py::array_t<double>& c,
    const py::object& A,
    const py::object& b,
    const py::object& Aeq,
    const py::object& beq,
    const py::object& lb,
    const py::object& ub,
    const py::list& vartypes,
    bool maximize) {
  using namespace mipsolvers::engine;
  MIPModel mip;
  mip.linear_part = build_lp_model(c, A, b, Aeq, beq, lb, ub, maximize);
  const int n = static_cast<int>(mip.linear_part.c.size());

  if (!vartypes.empty() && static_cast<int>(vartypes.size()) != n) {
    throw std::invalid_argument("vartypes length must equal c.size");
  }

  for (int i = 0; i < n; ++i) {
    const std::string vt = vartypes.empty()
                               ? "C"
                               : py::str(vartypes[i]).cast<std::string>();
    const char ch = vt.empty() ? 'C' : static_cast<char>(std::toupper(static_cast<unsigned char>(vt[0])));
    auto& meta = mip.linear_part.vars[static_cast<size_t>(i)];
    if (ch == 'B') {
      mip.binary_idx.push_back(i);
      meta.type = VarType::Binary;
      meta.lb = 0.0;
      meta.ub = 1.0;
    } else if (ch == 'I') {
      mip.integer_idx.push_back(i);
      meta.type = VarType::Integer;
    } else {
      meta.type = VarType::Continuous;
    }
  }
  return mip;
}

}  // anonymous namespace

// ─────────────────────────────────────────────────────────────────────────────
// Module definition
// ─────────────────────────────────────────────────────────────────────────────
PYBIND11_MODULE(mipsolvers, m) {
  m.doc() = R"doc(
MIPSolvers Python bindings.

Submodules
----------
scuc   : Short-term Unit Commitment (SCUC/SCED/LMP) market dispatch module.
engine : Low-level LP/MILP/NLP solver engine.
)doc";

  // ═══════════════════════════════════════════════════════════════════════════
  // SCUC submodule
  // ═══════════════════════════════════════════════════════════════════════════
  py::module_ m_scuc = m.def_submodule("scuc", R"doc(
Short-Term Unit Commitment (SCUC) market dispatch module.

Quick start (JSON API — recommended)
--------------------------------------
>>> import mipsolvers
>>> result_json = mipsolvers.scuc.solve_json(input_json_str)

Quick start (struct API)
------------------------
>>> inp = mipsolvers.scuc.SCUCInput()
>>> inp.config.num_periods = 24
>>> g = mipsolvers.scuc.Generator()
>>> g.name = "G1"; g.bus = 0; g.pmin = 100.0; g.pmax = 400.0
>>> g.bid_segments = [mipsolvers.scuc.BidSegment(20.0, 300.0)]
>>> inp.generators.append(g)
>>> out = mipsolvers.scuc.solve(inp)
>>> print(out.scuc.converged, out.scuc.total_cost)
)doc");

  // ── BidSegment ─────────────────────────────────────────────────────────────
  py::class_<mipsolvers::scuc::BidSegment>(m_scuc, "BidSegment", R"doc(
A single price–quantity segment of a generator's offer curve.

Attributes
----------
price    : float  Offer price ($/MWh). **First field.**
quantity : float  Incremental capacity above Pmin (MW). **Second field.**
)doc")
    .def(py::init<>())
    .def(py::init([](double price, double quantity) {
      mipsolvers::scuc::BidSegment s;
      s.price = price;
      s.quantity = quantity;
      return s;
    }), "price"_a, "quantity"_a)
    .def_readwrite("price",    &mipsolvers::scuc::BidSegment::price)
    .def_readwrite("quantity", &mipsolvers::scuc::BidSegment::quantity)
    .def("__repr__", [](const mipsolvers::scuc::BidSegment& s) {
      return "BidSegment(price=" + std::to_string(s.price) +
             ", quantity=" + std::to_string(s.quantity) + ")";
    });

  // ── Generator ──────────────────────────────────────────────────────────────
  py::class_<mipsolvers::scuc::Generator>(m_scuc, "Generator", R"doc(
Thermal generator parameters.
)doc")
    .def(py::init<>())
    .def_readwrite("name",                   &mipsolvers::scuc::Generator::name)
    .def_readwrite("bus",                    &mipsolvers::scuc::Generator::bus)
    .def_readwrite("pmin",                   &mipsolvers::scuc::Generator::pmin)
    .def_readwrite("pmax",                   &mipsolvers::scuc::Generator::pmax)
    .def_readwrite("ramp_up_mw_min",         &mipsolvers::scuc::Generator::ramp_up_mw_min)
    .def_readwrite("ramp_dn_mw_min",         &mipsolvers::scuc::Generator::ramp_dn_mw_min)
    .def_readwrite("min_up_time_hr",         &mipsolvers::scuc::Generator::min_up_time_hr)
    .def_readwrite("min_dn_time_hr",         &mipsolvers::scuc::Generator::min_dn_time_hr)
    .def_readwrite("must_run",               &mipsolvers::scuc::Generator::must_run)
    .def_readwrite("max_startups",           &mipsolvers::scuc::Generator::max_startups)
    .def_readwrite("max_shutdowns",          &mipsolvers::scuc::Generator::max_shutdowns)
    .def_readwrite("startup_cost",           &mipsolvers::scuc::Generator::startup_cost)
    .def_readwrite("startup_cost_warm",      &mipsolvers::scuc::Generator::startup_cost_warm)
    .def_readwrite("startup_cost_cold",      &mipsolvers::scuc::Generator::startup_cost_cold)
    .def_readwrite("hot_start_threshold_hr", &mipsolvers::scuc::Generator::hot_start_threshold_hr)
    .def_readwrite("warm_start_threshold_hr",&mipsolvers::scuc::Generator::warm_start_threshold_hr)
    .def_readwrite("ud_periods",             &mipsolvers::scuc::Generator::ud_periods)
    .def_readwrite("dd_periods",             &mipsolvers::scuc::Generator::dd_periods)
    .def_readwrite("no_load_cost",           &mipsolvers::scuc::Generator::no_load_cost)
    .def_readwrite("spinning_reserve_price", &mipsolvers::scuc::Generator::spinning_reserve_price)
    .def_readwrite("regulation_up_price",    &mipsolvers::scuc::Generator::regulation_up_price)
    .def_readwrite("regulation_down_price",  &mipsolvers::scuc::Generator::regulation_down_price)
    .def_readwrite("pfr_alpha",              &mipsolvers::scuc::Generator::pfr_alpha)
    .def_readwrite("group_id",               &mipsolvers::scuc::Generator::group_id)
    .def_readwrite("bid_segments",           &mipsolvers::scuc::Generator::bid_segments);

  // ── Branch ─────────────────────────────────────────────────────────────────
  py::class_<mipsolvers::scuc::Branch>(m_scuc, "Branch", R"doc(
AC transmission branch (line or transformer).
)doc")
    .def(py::init<>())
    .def_readwrite("from_bus",   &mipsolvers::scuc::Branch::from)
    .def_readwrite("to_bus",     &mipsolvers::scuc::Branch::to)
    .def_readwrite("reactance",  &mipsolvers::scuc::Branch::reactance)
    .def_readwrite("rating_mw",  &mipsolvers::scuc::Branch::rating_mw)
    .def_readwrite("in_service", &mipsolvers::scuc::Branch::in_service);

  // ── Load ───────────────────────────────────────────────────────────────────
  py::class_<mipsolvers::scuc::Load>(m_scuc, "Load")
    .def(py::init<>())
    .def_readwrite("bus",  &mipsolvers::scuc::Load::bus)
    .def_readwrite("p_mw", &mipsolvers::scuc::Load::p_mw);

  // ── WindUnit / SolarUnit ───────────────────────────────────────────────────
  py::class_<mipsolvers::scuc::WindUnit>(m_scuc, "WindUnit")
    .def(py::init<>())
    .def_readwrite("bus",  &mipsolvers::scuc::WindUnit::bus)
    .def_readwrite("pmax", &mipsolvers::scuc::WindUnit::pmax);

  py::class_<mipsolvers::scuc::SolarUnit>(m_scuc, "SolarUnit")
    .def(py::init<>())
    .def_readwrite("bus",  &mipsolvers::scuc::SolarUnit::bus)
    .def_readwrite("pmax", &mipsolvers::scuc::SolarUnit::pmax);

  // ── StorageUnit ────────────────────────────────────────────────────────────
  py::class_<mipsolvers::scuc::StorageUnit>(m_scuc, "StorageUnit", R"doc(
Battery / pumped-hydro storage unit.
)doc")
    .def(py::init<>())
    .def_readwrite("name",                  &mipsolvers::scuc::StorageUnit::name)
    .def_readwrite("bus",                   &mipsolvers::scuc::StorageUnit::bus)
    .def_readwrite("pmax_charge",           &mipsolvers::scuc::StorageUnit::pmax_charge)
    .def_readwrite("pmax_discharge",        &mipsolvers::scuc::StorageUnit::pmax_discharge)
    .def_readwrite("pmin_charge",           &mipsolvers::scuc::StorageUnit::pmin_charge)
    .def_readwrite("pmin_discharge",        &mipsolvers::scuc::StorageUnit::pmin_discharge)
    .def_readwrite("energy_capacity_mwh",   &mipsolvers::scuc::StorageUnit::energy_capacity_mwh)
    .def_readwrite("efficiency",            &mipsolvers::scuc::StorageUnit::efficiency)
    .def_readwrite("eta_charge",            &mipsolvers::scuc::StorageUnit::eta_charge)
    .def_readwrite("eta_discharge",         &mipsolvers::scuc::StorageUnit::eta_discharge)
    .def_readwrite("soc_init",              &mipsolvers::scuc::StorageUnit::soc_init)
    .def_readwrite("soc_min",               &mipsolvers::scuc::StorageUnit::soc_min)
    .def_readwrite("soc_final",             &mipsolvers::scuc::StorageUnit::soc_final)
    .def_readwrite("charge_bid_price",      &mipsolvers::scuc::StorageUnit::charge_bid_price)
    .def_readwrite("discharge_bid_price",   &mipsolvers::scuc::StorageUnit::discharge_bid_price)
    .def_readwrite("cycle_limit",           &mipsolvers::scuc::StorageUnit::cycle_limit)
    .def_readwrite("use_binary_indicators", &mipsolvers::scuc::StorageUnit::use_binary_indicators);

  // ── DCLine ─────────────────────────────────────────────────────────────────
  py::class_<mipsolvers::scuc::DCLine>(m_scuc, "DCLine")
    .def(py::init<>())
    .def_readwrite("from_bus", &mipsolvers::scuc::DCLine::from)
    .def_readwrite("to_bus",   &mipsolvers::scuc::DCLine::to)
    .def_readwrite("pmin",     &mipsolvers::scuc::DCLine::pmin)
    .def_readwrite("pmax",     &mipsolvers::scuc::DCLine::pmax)
    .def_readwrite("ramp_up",  &mipsolvers::scuc::DCLine::ramp_up)
    .def_readwrite("ramp_dn",  &mipsolvers::scuc::DCLine::ramp_dn);

  // ── GeneratorGroup ─────────────────────────────────────────────────────────
  py::class_<mipsolvers::scuc::GeneratorGroup>(m_scuc, "GeneratorGroup")
    .def(py::init<>())
    .def_readwrite("id",          &mipsolvers::scuc::GeneratorGroup::id)
    .def_readwrite("name",        &mipsolvers::scuc::GeneratorGroup::name)
    .def_readwrite("gen_indices", &mipsolvers::scuc::GeneratorGroup::gen_indices)
    .def_readwrite("pmin_t",      &mipsolvers::scuc::GeneratorGroup::pmin_t)
    .def_readwrite("pmax_t",      &mipsolvers::scuc::GeneratorGroup::pmax_t)
    .def_readwrite("emin",        &mipsolvers::scuc::GeneratorGroup::emin)
    .def_readwrite("emax",        &mipsolvers::scuc::GeneratorGroup::emax);

  // ── Section ────────────────────────────────────────────────────────────────
  py::class_<mipsolvers::scuc::Section>(m_scuc, "Section", R"doc(
Monitored network section (aggregate transmission corridor).
line_weights: list of (branch_index, weight) tuples.
)doc")
    .def(py::init<>())
    .def_readwrite("name",           &mipsolvers::scuc::Section::name)
    .def_readwrite("line_weights",   &mipsolvers::scuc::Section::line_weights)
    .def_readwrite("rating_fwd_mw",  &mipsolvers::scuc::Section::rating_fwd_mw)
    .def_readwrite("rating_rev_mw",  &mipsolvers::scuc::Section::rating_rev_mw);

  // ── SCUCConfig ─────────────────────────────────────────────────────────────
  py::class_<mipsolvers::scuc::SCUCConfig>(m_scuc, "SCUCConfig", R"doc(
Solver and model configuration.

Key attributes
--------------
solver          : str   "Auto" | "Gurobi" | "HiGHS" | "NativeBranchAndCut"
num_periods     : int   Number of dispatch periods T (default 24)
period_length_hr: float Hours per period (default 1.0)
n_segments      : int   Offer curve segments per generator (default 3)
mip_gap         : float Relative MIP optimality gap (default 0.001)
time_limit_sec  : float Solver time limit in seconds (default 300.0)
voll            : float Value of lost load $/MWh (default 10000.0)
solve_sced      : bool  Run SCED LP after SCUC (default True)
solve_lmp       : bool  Compute nodal LMP after SCED (default True)
)doc")
    .def(py::init<>())
    .def_readwrite("solver",                      &mipsolvers::scuc::SCUCConfig::solver)
    .def_readwrite("allow_fallback",              &mipsolvers::scuc::SCUCConfig::allow_fallback)
    .def_readwrite("num_periods",                 &mipsolvers::scuc::SCUCConfig::num_periods)
    .def_readwrite("period_length_hr",            &mipsolvers::scuc::SCUCConfig::period_length_hr)
    .def_readwrite("n_segments",                  &mipsolvers::scuc::SCUCConfig::n_segments)
    .def_readwrite("mip_gap",                     &mipsolvers::scuc::SCUCConfig::mip_gap)
    .def_readwrite("time_limit_sec",              &mipsolvers::scuc::SCUCConfig::time_limit_sec)
    .def_readwrite("verbose",                     &mipsolvers::scuc::SCUCConfig::verbose)
    .def_readwrite("spinning_reserve_req",        &mipsolvers::scuc::SCUCConfig::spinning_reserve_req)
    .def_readwrite("regulation_up_req",           &mipsolvers::scuc::SCUCConfig::regulation_up_req)
    .def_readwrite("regulation_down_req",         &mipsolvers::scuc::SCUCConfig::regulation_down_req)
    .def_readwrite("neg_reserve_req",             &mipsolvers::scuc::SCUCConfig::neg_reserve_req)
    .def_readwrite("pfr_reserve_req_mw",          &mipsolvers::scuc::SCUCConfig::pfr_reserve_req_mw)
    .def_readwrite("voll",                        &mipsolvers::scuc::SCUCConfig::voll)
    .def_readwrite("vocc",                        &mipsolvers::scuc::SCUCConfig::vocc)
    .def_readwrite("renewable_min_output_coeff",  &mipsolvers::scuc::SCUCConfig::renewable_min_output_coeff)
    .def_readwrite("M2_renewable_curtail_penalty",&mipsolvers::scuc::SCUCConfig::M2_renewable_curtail_penalty)
    .def_readwrite("M1_line_slack_penalty",       &mipsolvers::scuc::SCUCConfig::M1_line_slack_penalty)
    .def_readwrite("wheeling_fee_per_mwh",        &mipsolvers::scuc::SCUCConfig::wheeling_fee_per_mwh)
    .def_readwrite("enable_market_cuts",          &mipsolvers::scuc::SCUCConfig::enable_market_cuts)
    .def_readwrite("solve_sced",                  &mipsolvers::scuc::SCUCConfig::solve_sced)
    .def_readwrite("solve_lmp",                   &mipsolvers::scuc::SCUCConfig::solve_lmp)
    .def_readwrite("lmp_delta",                   &mipsolvers::scuc::SCUCConfig::lmp_delta);

  // ── SCUCProfiles ───────────────────────────────────────────────────────────
  py::class_<mipsolvers::scuc::SCUCProfiles>(m_scuc, "SCUCProfiles", R"doc(
Time-series profiles.

Attributes
----------
load  : list[list[float]]  load[d][t] = per-unit multiplier for Load[d] at period t
wind  : list[list[float]]  wind[w][t] = forecast MW for WindUnit w at period t
solar : list[list[float]]  solar[s][t] = forecast MW for SolarUnit s at period t
)doc")
    .def(py::init<>())
    .def_readwrite("load",  &mipsolvers::scuc::SCUCProfiles::load)
    .def_readwrite("wind",  &mipsolvers::scuc::SCUCProfiles::wind)
    .def_readwrite("solar", &mipsolvers::scuc::SCUCProfiles::solar);

  // ── SCUCInitialStatus ──────────────────────────────────────────────────────
  py::class_<mipsolvers::scuc::SCUCInitialStatus>(m_scuc, "SCUCInitialStatus", R"doc(
Generator and storage state at the start of the planning horizon.
)doc")
    .def(py::init<>())
    .def_readwrite("commitment",    &mipsolvers::scuc::SCUCInitialStatus::commitment)
    .def_readwrite("dispatch",      &mipsolvers::scuc::SCUCInitialStatus::dispatch)
    .def_readwrite("storage_soc",   &mipsolvers::scuc::SCUCInitialStatus::storage_soc)
    .def_readwrite("time_in_state", &mipsolvers::scuc::SCUCInitialStatus::time_in_state);

  // ── SCUCInput ──────────────────────────────────────────────────────────────
  py::class_<mipsolvers::scuc::SCUCInput>(m_scuc, "SCUCInput", R"doc(
Full SCUC problem input.  All fields have sensible defaults; populate only
what you need.

Example
-------
>>> inp = mipsolvers.scuc.SCUCInput()
>>> inp.config.num_periods = 24
>>> inp.config.solver = "HiGHS"
)doc")
    .def(py::init<>())
    .def_readwrite("config",            &mipsolvers::scuc::SCUCInput::config)
    .def_readwrite("num_buses",         &mipsolvers::scuc::SCUCInput::num_buses)
    .def_readwrite("generators",        &mipsolvers::scuc::SCUCInput::generators)
    .def_readwrite("branches",          &mipsolvers::scuc::SCUCInput::branches)
    .def_readwrite("loads",             &mipsolvers::scuc::SCUCInput::loads)
    .def_readwrite("wind",              &mipsolvers::scuc::SCUCInput::wind)
    .def_readwrite("solar",             &mipsolvers::scuc::SCUCInput::solar)
    .def_readwrite("storage",           &mipsolvers::scuc::SCUCInput::storage)
    .def_readwrite("dc_lines",          &mipsolvers::scuc::SCUCInput::dc_lines)
    .def_readwrite("generator_groups",  &mipsolvers::scuc::SCUCInput::generator_groups)
    .def_readwrite("sections",          &mipsolvers::scuc::SCUCInput::sections)
    .def_readwrite("profiles",          &mipsolvers::scuc::SCUCInput::profiles)
    .def_readwrite("initial_status",    &mipsolvers::scuc::SCUCInput::initial_status);

  // ── SCUCSolveResult ────────────────────────────────────────────────────────
  py::class_<mipsolvers::scuc::SCUCSolveResult>(m_scuc, "SCUCSolveResult", R"doc(
SCUC or SCED solve result.

Key attributes
--------------
converged       : bool
objective       : float  Optimal objective value ($)
solver_name     : str
solve_time_sec  : float
mip_gap         : float
commitment      : list[list[float]]  [ng][T_commit]
dispatch        : list[list[float]]  [ng][T]
line_flows      : list[list[float]]  [nl][T]
nodal_lmp       : (in SCUCLMPResult) [nb][T]
load_shedding   : list[float]        [T]
total_cost      : float
energy_cost     : float
startup_cost    : float
no_load_cost    : float
reserve_cost    : float
penalty_cost    : float
)doc")
    .def(py::init<>())
    .def_readonly("converged",         &mipsolvers::scuc::SCUCSolveResult::converged)
    .def_readonly("objective",         &mipsolvers::scuc::SCUCSolveResult::objective)
    .def_readonly("solver_name",       &mipsolvers::scuc::SCUCSolveResult::solver_name)
    .def_readonly("solve_time_sec",    &mipsolvers::scuc::SCUCSolveResult::solve_time_sec)
    .def_readonly("mip_gap",           &mipsolvers::scuc::SCUCSolveResult::mip_gap)
    .def_readonly("n_cuts_added",      &mipsolvers::scuc::SCUCSolveResult::n_cuts_added)
    .def_readonly("commitment",        &mipsolvers::scuc::SCUCSolveResult::commitment)
    .def_readonly("startup",           &mipsolvers::scuc::SCUCSolveResult::startup)
    .def_readonly("shutdown",          &mipsolvers::scuc::SCUCSolveResult::shutdown)
    .def_readonly("dispatch",          &mipsolvers::scuc::SCUCSolveResult::dispatch)
    .def_readonly("spinning_reserve",  &mipsolvers::scuc::SCUCSolveResult::spinning_reserve)
    .def_readonly("regulation_up",     &mipsolvers::scuc::SCUCSolveResult::regulation_up)
    .def_readonly("regulation_down",   &mipsolvers::scuc::SCUCSolveResult::regulation_down)
    .def_readonly("segment_dispatch",  &mipsolvers::scuc::SCUCSolveResult::segment_dispatch)
    .def_readonly("wind_generation",   &mipsolvers::scuc::SCUCSolveResult::wind_generation)
    .def_readonly("solar_generation",  &mipsolvers::scuc::SCUCSolveResult::solar_generation)
    .def_readonly("wind_curtailment",  &mipsolvers::scuc::SCUCSolveResult::wind_curtailment)
    .def_readonly("solar_curtailment", &mipsolvers::scuc::SCUCSolveResult::solar_curtailment)
    .def_readonly("storage_charging",     &mipsolvers::scuc::SCUCSolveResult::storage_charging)
    .def_readonly("storage_discharging",  &mipsolvers::scuc::SCUCSolveResult::storage_discharging)
    .def_readonly("storage_soc",          &mipsolvers::scuc::SCUCSolveResult::storage_soc)
    .def_readonly("line_flows",           &mipsolvers::scuc::SCUCSolveResult::line_flows)
    .def_readonly("section_flows",        &mipsolvers::scuc::SCUCSolveResult::section_flows)
    .def_readonly("load_shedding",        &mipsolvers::scuc::SCUCSolveResult::load_shedding)
    .def_readonly("gen_curtailment",      &mipsolvers::scuc::SCUCSolveResult::gen_curtailment)
    .def_readonly("energy_cost",          &mipsolvers::scuc::SCUCSolveResult::energy_cost)
    .def_readonly("startup_cost",         &mipsolvers::scuc::SCUCSolveResult::startup_cost)
    .def_readonly("no_load_cost",         &mipsolvers::scuc::SCUCSolveResult::no_load_cost)
    .def_readonly("reserve_cost",         &mipsolvers::scuc::SCUCSolveResult::reserve_cost)
    .def_readonly("penalty_cost",         &mipsolvers::scuc::SCUCSolveResult::penalty_cost)
    .def_readonly("total_cost",           &mipsolvers::scuc::SCUCSolveResult::total_cost);

  // ── SCUCLMPResult ──────────────────────────────────────────────────────────
  py::class_<mipsolvers::scuc::SCUCLMPResult>(m_scuc, "SCUCLMPResult", R"doc(
Locational Marginal Price (LMP) result.

Attributes
----------
converged       : bool
solve_time_sec  : float
nodal_lmp       : list[list[float]]  [nb][T]  $/MWh
energy_lmp      : list[list[float]]  [nb][T]
congestion_lmp  : list[list[float]]  [nb][T]
avg_lmp         : float
max_lmp         : float
min_lmp         : float
)doc")
    .def(py::init<>())
    .def_readonly("converged",       &mipsolvers::scuc::SCUCLMPResult::converged)
    .def_readonly("solve_time_sec",  &mipsolvers::scuc::SCUCLMPResult::solve_time_sec)
    .def_readonly("nodal_lmp",       &mipsolvers::scuc::SCUCLMPResult::nodal_lmp)
    .def_readonly("energy_lmp",      &mipsolvers::scuc::SCUCLMPResult::energy_lmp)
    .def_readonly("congestion_lmp",  &mipsolvers::scuc::SCUCLMPResult::congestion_lmp)
    .def_readonly("avg_lmp",         &mipsolvers::scuc::SCUCLMPResult::avg_lmp)
    .def_readonly("max_lmp",         &mipsolvers::scuc::SCUCLMPResult::max_lmp)
    .def_readonly("min_lmp",         &mipsolvers::scuc::SCUCLMPResult::min_lmp);

  // ── SCUCOutput ─────────────────────────────────────────────────────────────
  py::class_<mipsolvers::scuc::SCUCOutput>(m_scuc, "SCUCOutput", R"doc(
Aggregated output of the full SCUC pipeline (SCUC → SCED → LMP).

Attributes
----------
scuc : SCUCSolveResult   Day-ahead unit commitment (MILP)
sced : SCUCSolveResult   Economic dispatch LP (only if config.solve_sced=True)
lmp  : SCUCLMPResult     Nodal prices (only if config.solve_lmp=True)
)doc")
    .def(py::init<>())
    .def_readonly("scuc", &mipsolvers::scuc::SCUCOutput::scuc)
    .def_readonly("sced", &mipsolvers::scuc::SCUCOutput::sced)
    .def_readonly("lmp",  &mipsolvers::scuc::SCUCOutput::lmp);

  // ── Free functions ─────────────────────────────────────────────────────────
  m_scuc.def("from_json", &scuc_from_json_checked,
    py::arg("json_str"),
    R"doc(
Deserialize an SCUCInput from a JSON string.

Parameters
----------
json_str : str  JSON representation of the input (see data_format_spec.md).

Returns
-------
SCUCInput

Raises
------
RuntimeError  If the JSON is malformed or a field type is wrong.
)doc");

  m_scuc.def("input_to_json", &mipsolvers::scuc::scuc_input_to_json,
    py::arg("input"),
    py::arg("indent") = 2,
    R"doc(
Serialize an SCUCInput to a JSON string. Use indent=-1 for compact JSON.
)doc");

  m_scuc.def("build_3bus_case", &mipsolvers::scuc::build_3bus_case,
    py::arg("T") = 3,
    py::arg("dt") = 1.0,
    R"doc(Build the built-in 3-bus SCUC case for dataset generation.)doc");

  m_scuc.def("build_6bus_case", &mipsolvers::scuc::build_6bus_case,
    py::arg("T") = 24,
    py::arg("dt") = 1.0,
    py::arg("with_wind") = false,
    py::arg("with_storage") = false,
    R"doc(Build the built-in 6-bus SCUC case for dataset generation.)doc");

  m_scuc.def("build_ieee39_case", &mipsolvers::scuc::build_ieee39_case,
    py::arg("T") = 24,
    py::arg("dt") = 1.0,
    py::arg("with_wind") = false,
    py::arg("with_solar") = false,
    R"doc(Build the IEEE-39 SCUC case for dataset generation.)doc");

  m_scuc.def("build_ieee118_case", &mipsolvers::scuc::build_ieee118_case,
    py::arg("T") = 24,
    py::arg("dt") = 1.0,
    py::arg("with_wind") = false,
    py::arg("with_solar") = false,
    R"doc(Build the IEEE-118 SCUC case for dataset generation.)doc");

  m_scuc.def("build_mip_summary",
    [](const mipsolvers::scuc::SCUCInput& inp,
       bool include_matrix_values,
       bool include_rhs_values,
       bool include_objective_values) -> py::dict {
      auto mip = mipsolvers::scuc::build_scuc_mip(inp);
      mipsolvers::l2o::FingerprintOptions fp;
      fp.include_matrix_values = include_matrix_values;
      fp.include_rhs_values = include_rhs_values;
      fp.include_objective_values = include_objective_values;
      return json_to_py(mip_summary_to_json(mip, fp)).cast<py::dict>();
    },
    py::arg("input"),
    py::arg("include_matrix_values") = false,
    py::arg("include_rhs_values") = false,
    py::arg("include_objective_values") = false,
    R"doc(
Build the raw SCUC MIP in C++ and return a Python dict with dimensions,
integer/binary columns, SCUC column maps, branch priorities, and an L2O
structural fingerprint. This is the safe Python training surface: the full
solver model stays in C++, while Python receives stable metadata for datasets.
)doc");

  m_scuc.def("solve",
    [](const mipsolvers::scuc::SCUCInput& inp) {
      try {
        return mipsolvers::scuc::scuc_solve(inp);
      } catch (const std::invalid_argument&) {
        throw;
      } catch (const std::exception& ex) {
        rethrow_scuc_exception("SCUC solve failed", ex);
      }
    },
    py::arg("input"),
    py::call_guard<py::gil_scoped_release>(),
    R"doc(
Run the full SCUC pipeline (SCUC MILP → SCED LP → LMP).

Parameters
----------
input : SCUCInput

Returns
-------
SCUCOutput

Notes
-----
The GIL is released during the solve; other Python threads can run.
)doc");

  m_scuc.def("output_to_json",
    [](const mipsolvers::scuc::SCUCOutput& out,
       const mipsolvers::scuc::SCUCInput& inp,
       int indent) {
      return scuc_output_to_json_checked(out, inp, indent);
    },
    py::arg("output"),
    py::arg("input"),
    py::arg("indent") = 2,
    R"doc(
Serialize an SCUCOutput to a JSON string.

Parameters
----------
output : SCUCOutput
input  : SCUCInput   (used for metadata echo)
indent : int         JSON indentation (default 2; use -1 for compact)

Returns
-------
str  JSON string
)doc");

  // ── Convenience: JSON-in → JSON-out ────────────────────────────────────────
  m_scuc.def("solve_json",
    [](const std::string& json_str, int indent) -> std::string {
      const auto inp = scuc_from_json_checked(json_str);
      mipsolvers::scuc::SCUCOutput out;
      {
        py::gil_scoped_release rel;
        try {
          out = mipsolvers::scuc::scuc_solve(inp);
        } catch (const std::invalid_argument&) {
          throw;
        } catch (const std::exception& ex) {
          rethrow_scuc_exception("SCUC solve failed", ex);
        }
      }
      return scuc_output_to_json_checked(out, inp, indent);
    },
    py::arg("json_str"),
    py::arg("indent") = 2,
    R"doc(
One-shot JSON API: parse input, run SCUC pipeline, return JSON result.

This is the simplest way to call SCUC from Python.

Parameters
----------
json_str : str  Input JSON string (see data_format_spec.md for schema).
indent   : int  Output JSON indentation (default 2; use -1 for compact).

Returns
-------
str  JSON result string containing "scuc", "sced", "lmp", "meta" keys.

Example
-------
>>> import json, mipsolvers
>>> inp = {"config": {"num_periods": 3}, "generators": [...], "loads": [...]}
>>> result = json.loads(mipsolvers.scuc.solve_json(json.dumps(inp)))
>>> print(result["scuc"]["converged"], result["scuc"]["cost"]["total"])
)doc");

  // ═══════════════════════════════════════════════════════════════════════════
  // Engine submodule
  // ═══════════════════════════════════════════════════════════════════════════
  py::module_ m_eng = m.def_submodule("engine", R"doc(
Low-level LP and MILP solver engine.

Quick start
-----------
>>> import numpy as np, mipsolvers
>>> # Minimize  c'x  subject to  A*x <= b,  lb <= x <= ub
>>> res = mipsolvers.engine.solve_lp(c, A, b, lb=lb, ub=ub, solver="Auto")
>>> print(res["success"], res["objective"], res["x"])

>>> # MILP: same as LP but pass vartypes list
>>> res = mipsolvers.engine.solve_milp(c, A, b, lb, ub, vartypes=["C","I","B",...])
)doc");

  py::enum_<mipsolvers::engine::BranchingStrategy>(m_eng, "BranchingStrategy")
    .value("MostInfeasible", mipsolvers::engine::BranchingStrategy::MostInfeasible)
    .value("Pseudocost", mipsolvers::engine::BranchingStrategy::Pseudocost)
    .value("FirstFractional", mipsolvers::engine::BranchingStrategy::FirstFractional);

  py::enum_<mipsolvers::engine::NodeSelection>(m_eng, "NodeSelection")
    .value("BestFirst", mipsolvers::engine::NodeSelection::BestFirst)
    .value("DepthFirst", mipsolvers::engine::NodeSelection::DepthFirst)
    .value("Hybrid", mipsolvers::engine::NodeSelection::Hybrid);

  py::enum_<mipsolvers::engine::CutType>(m_eng, "CutType")
    .value("None", mipsolvers::engine::CutType::None)
    .value("IntRounding", mipsolvers::engine::CutType::IntRounding)
    .value("MIR", mipsolvers::engine::CutType::MIR)
    .value("Gomory", mipsolvers::engine::CutType::Gomory)
    .value("Cover", mipsolvers::engine::CutType::Cover)
    .value("FlowCover", mipsolvers::engine::CutType::FlowCover)
    .value("ImpliedBound", mipsolvers::engine::CutType::ImpliedBound)
    .value("All", mipsolvers::engine::CutType::All);

  py::enum_<mipsolvers::engine::LpKernelBackend>(m_eng, "LpKernelBackend")
    .value("HiGHS", mipsolvers::engine::LpKernelBackend::HiGHS)
    .value("ExperimentalNative",
           mipsolvers::engine::LpKernelBackend::ExperimentalNative);

  py::class_<mipsolvers::engine::BCOptions>(m_eng, "BCOptions", R"doc(
Focused branch-and-cut options for L2O experiments and policy evaluation.
The production solver remains C++; Python only chooses safe public knobs.
)doc")
    .def(py::init<>())
    .def_readwrite("max_nodes", &mipsolvers::engine::BCOptions::max_nodes)
    .def_readwrite("max_lp_iter", &mipsolvers::engine::BCOptions::max_lp_iter)
    .def_readwrite("time_limit_sec", &mipsolvers::engine::BCOptions::time_limit_sec)
    .def_readwrite("int_tol", &mipsolvers::engine::BCOptions::int_tol)
    .def_readwrite("gap_tol", &mipsolvers::engine::BCOptions::gap_tol)
    .def_readwrite("lp_tol", &mipsolvers::engine::BCOptions::lp_tol)
    .def_readwrite("root_cut_rounds", &mipsolvers::engine::BCOptions::root_cut_rounds)
    .def_readwrite("cuts_per_round", &mipsolvers::engine::BCOptions::cuts_per_round)
    .def_readwrite("max_cut_depth", &mipsolvers::engine::BCOptions::max_cut_depth)
    .def_readwrite("cuts", &mipsolvers::engine::BCOptions::cuts)
    .def_readwrite("branching", &mipsolvers::engine::BCOptions::branching)
    .def_readwrite("node_sel", &mipsolvers::engine::BCOptions::node_sel)
    .def_readwrite("use_simplex_lp_nodes", &mipsolvers::engine::BCOptions::use_simplex_lp_nodes)
    .def_readwrite("use_ipm_root", &mipsolvers::engine::BCOptions::use_ipm_root)
    .def_readwrite("use_ipm_nodes", &mipsolvers::engine::BCOptions::use_ipm_nodes)
    .def_readwrite("lp_kernel_backend",
                   &mipsolvers::engine::BCOptions::lp_kernel_backend)
    .def_property(
        "use_vendored_highs_lp_kernel",
        [](const mipsolvers::engine::BCOptions& opt) {
          return mipsolvers::engine::uses_highs_lp_kernel(
              opt.lp_kernel_backend);
        },
        [](mipsolvers::engine::BCOptions& opt, bool enabled) {
          opt.lp_kernel_backend =
              enabled
                  ? mipsolvers::engine::LpKernelBackend::HiGHS
                  : mipsolvers::engine::LpKernelBackend::ExperimentalNative;
        },
        "Deprecated compatibility alias; use lp_kernel_backend.")
    .def_readwrite("strict_highs_mip_contract", &mipsolvers::engine::BCOptions::strict_highs_mip_contract)
    .def_readwrite("verbose", &mipsolvers::engine::BCOptions::verbose)
    .def_readwrite("accept_verified_warm_start_incumbent", &mipsolvers::engine::BCOptions::accept_verified_warm_start_incumbent)
    .def_readwrite("enable_feasibility_jump", &mipsolvers::engine::BCOptions::enable_feasibility_jump)
    .def_readwrite("feasibility_jump_max_flips", &mipsolvers::engine::BCOptions::feasibility_jump_max_flips)
    .def_readwrite("feasibility_jump_repair_attempts", &mipsolvers::engine::BCOptions::feasibility_jump_repair_attempts)
    .def_readwrite("enable_domain_heuristics", &mipsolvers::engine::BCOptions::enable_domain_heuristics)
    .def_readwrite("num_threads", &mipsolvers::engine::BCOptions::num_threads)
    .def_readwrite("deterministic_parallel", &mipsolvers::engine::BCOptions::deterministic_parallel)
    .def_readwrite("random_seed", &mipsolvers::engine::BCOptions::random_seed)
    .def_readwrite("feasibility_pump_iters", &mipsolvers::engine::BCOptions::feasibility_pump_iters)
    .def_readwrite("max_dive_lps", &mipsolvers::engine::BCOptions::max_dive_lps)
    .def_readwrite("max_probe_vars", &mipsolvers::engine::BCOptions::max_probe_vars)
    .def_readwrite("enable_lns", &mipsolvers::engine::BCOptions::enable_lns)
    .def_readwrite("lns_fix_ratio", &mipsolvers::engine::BCOptions::lns_fix_ratio)
    .def_readwrite("lns_node_limit", &mipsolvers::engine::BCOptions::lns_node_limit)
    .def_readwrite("lns_time_limit", &mipsolvers::engine::BCOptions::lns_time_limit)
    .def_readwrite("lns_max_iters", &mipsolvers::engine::BCOptions::lns_max_iters)
    .def_readwrite("highs_mip_detect_symmetry", &mipsolvers::engine::BCOptions::highs_mip_detect_symmetry)
    .def_readwrite("highs_mip_heuristic_effort", &mipsolvers::engine::BCOptions::highs_mip_heuristic_effort)
    .def_readwrite("highs_mip_pscost_minreliable", &mipsolvers::engine::BCOptions::highs_mip_pscost_minreliable)
    .def_readwrite("highs_mip_max_stall_nodes", &mipsolvers::engine::BCOptions::highs_mip_max_stall_nodes)
    .def_readwrite("highs_mip_run_zi_round", &mipsolvers::engine::BCOptions::highs_mip_run_zi_round)
    .def_readwrite("highs_mip_run_shifting", &mipsolvers::engine::BCOptions::highs_mip_run_shifting)
    .def_readwrite("highs_force_presolve_on", &mipsolvers::engine::BCOptions::highs_force_presolve_on)
    .def_readwrite("highs_presolve_substitution_maxfillin", &mipsolvers::engine::BCOptions::highs_presolve_substitution_maxfillin)
    .def_readwrite("highs_mip_lp_age_limit", &mipsolvers::engine::BCOptions::highs_mip_lp_age_limit)
    .def_readwrite("highs_mip_lp_solver", &mipsolvers::engine::BCOptions::highs_mip_lp_solver)
    .def_readwrite("highs_mip_root_crossover", &mipsolvers::engine::BCOptions::highs_mip_root_crossover)
    .def_readwrite("highs_mip_root_simplex_iteration_limit", &mipsolvers::engine::BCOptions::highs_mip_root_simplex_iteration_limit)
    .def_readwrite("highs_strict_auto_ipm_root_for_large_models", &mipsolvers::engine::BCOptions::highs_strict_auto_ipm_root_for_large_models)
    .def_readwrite("highs_strict_auto_ipm_root_min_cols", &mipsolvers::engine::BCOptions::highs_strict_auto_ipm_root_min_cols)
    .def_readwrite("highs_strict_auto_ipm_root_min_rows", &mipsolvers::engine::BCOptions::highs_strict_auto_ipm_root_min_rows)
    .def_readwrite("highs_strict_auto_ipm_root_min_time_sec", &mipsolvers::engine::BCOptions::highs_strict_auto_ipm_root_min_time_sec)
    .def_readwrite("highs_max_root_sepa_rounds", &mipsolvers::engine::BCOptions::highs_max_root_sepa_rounds)
    .def("to_dict", [](const mipsolvers::engine::BCOptions& opt) {
      nlohmann::json j{
          {"max_nodes", opt.max_nodes},
          {"time_limit_sec", opt.time_limit_sec},
          {"gap_tol", opt.gap_tol},
          {"root_cut_rounds", opt.root_cut_rounds},
          {"cuts_per_round", opt.cuts_per_round},
          {"num_threads", opt.num_threads},
          {"verbose", opt.verbose},
          {"highs_mip_lp_solver", opt.highs_mip_lp_solver},
          {"highs_mip_root_crossover", opt.highs_mip_root_crossover}};
      return json_to_py(j);
    });

  // ── solve_lp ───────────────────────────────────────────────────────────────
  m_eng.def("solve_lp",
    [](const py::array_t<double>& c,
       const py::object& A,
       const py::object& b,
       const py::object& Aeq,
       const py::object& beq,
       const py::object& lb,
       const py::object& ub,
       const std::string& solver,
       bool maximize) -> py::dict {

      auto lp = build_lp_model(c, A, b, Aeq, beq, lb, ub, maximize);
      mipsolvers::engine::SolverEngine eng;
      mipsolvers::engine::SolveOptions opts;
      opts.preferred_solver = solver;
      opts.allow_fallback = true;

      py::gil_scoped_release rel;
      auto res = eng.solve_lp(lp, opts);
      py::gil_scoped_acquire acq;

      py::dict d;
      d["success"]    = res.stats.success;
      d["objective"]  = res.stats.objective;
      d["status"]     = res.stats.status;
      d["solver"]     = res.stats.solver_name;
      d["iterations"] = res.stats.iterations;
      d["runtime_sec"]= res.stats.runtime_sec;
      d["x"]          = vec_to_numpy(res.x);
      if (res.constraint_duals.size() > 0)
        d["duals"]    = vec_to_numpy(res.constraint_duals);
      return d;
    },
    py::arg("c"),
    py::arg("A")    = py::none(),
    py::arg("b")    = py::none(),
    py::arg("Aeq")  = py::none(),
    py::arg("beq")  = py::none(),
    py::arg("lb")   = py::none(),
    py::arg("ub")   = py::none(),
    py::arg("solver")   = "Auto",
    py::arg("maximize") = false,
    R"doc(
Solve a Linear Program.

  minimize    c @ x
  subject to  A  @ x <= b       (inequality, optional)
              Aeq @ x  = beq    (equality, optional)
              lb <= x <= ub     (box bounds, optional)

Parameters
----------
c        : np.ndarray  (n,)   Objective vector
A        : np.ndarray  (m,n)  Inequality constraint matrix (optional)
b        : np.ndarray  (m,)   Inequality RHS (optional)
Aeq      : np.ndarray  (p,n)  Equality constraint matrix (optional)
beq      : np.ndarray  (p,)   Equality RHS (optional)
lb       : np.ndarray  (n,)   Variable lower bounds (default -inf)
ub       : np.ndarray  (n,)   Variable upper bounds (default +inf)
solver   : str                "Auto" | "Gurobi" | "HiGHS" | "NativeIPMLP" |
                               "NativePDLP" | "NativeLCQP"
maximize : bool               If True, maximize c@x (default False)

Returns
-------
dict with keys:
  success    : bool
  objective  : float
  x          : np.ndarray  (n,)  Optimal solution
  duals      : np.ndarray  Constraint duals (if available)
  status     : str
  solver     : str
  iterations : int
  runtime_sec: float
)doc");

  // ── solve_milp ─────────────────────────────────────────────────────────────
  m_eng.def("solve_milp",
    [](const py::array_t<double>& c,
       const py::object& A,
       const py::object& b,
       const py::object& Aeq,
       const py::object& beq,
       const py::object& lb,
       const py::object& ub,
       const py::list& vartypes,
       const std::string& solver,
       double mip_gap,
       double time_limit_sec,
       const py::object& initial_solution,
       bool maximize) -> py::dict {

      auto mip = build_mip_model(c, A, b, Aeq, beq, lb, ub, vartypes, maximize);
      set_initial_solution_from_python(mip, initial_solution);

      if (is_native_milp_solver_name(solver)) {
        auto opt = make_python_bc_options(mip_gap, time_limit_sec);
        py::gil_scoped_release rel;
        auto res = solve_native_milp_with_options(mip, solver, opt);
        py::gil_scoped_acquire acq;
        return solve_result_to_py_dict(res);
      }

      mipsolvers::engine::SolverEngine eng;
      mipsolvers::engine::SolveOptions opts;
      opts.preferred_solver = solver;
      opts.allow_fallback = true;

      py::gil_scoped_release rel;
      auto res = eng.solve_milp(mip, opts);
      py::gil_scoped_acquire acq;

      auto d = api_result_to_py_dict(res);
      d["warning"] = "mip_gap and time_limit_sec are currently enforced only for Auto, StrictHiGHS, and NativeBranchAndCut";
      return d;
    },
    py::arg("c"),
    py::arg("A")    = py::none(),
    py::arg("b")    = py::none(),
    py::arg("Aeq")  = py::none(),
    py::arg("beq")  = py::none(),
    py::arg("lb")   = py::none(),
    py::arg("ub")   = py::none(),
    py::arg("vartypes")      = py::list(),
    py::arg("solver")        = "Auto",
    py::arg("mip_gap")       = 1e-4,
    py::arg("time_limit_sec")= 300.0,
    py::arg("initial_solution") = py::none(),
    py::arg("maximize")      = false,
    R"doc(
Solve a Mixed-Integer Linear Program.

  minimize    c @ x
  subject to  A  @ x <= b       (inequality, optional)
              Aeq @ x  = beq    (equality, optional)
              lb <= x <= ub
              x[i] ∈ {0,1}  for i in binary_idx
              x[i] ∈ ℤ       for i in integer_idx

Parameters
----------
c        : np.ndarray  (n,)
A, b, Aeq, beq, lb, ub : same as solve_lp
vartypes : list[str]   Per-variable type: "C" continuous, "I" integer, "B" binary.
                       Default: all continuous. Length must equal n if provided.
solver   : str         "Auto" | "Gurobi" | "HiGHS" | "NativeBranchAndCut"
mip_gap       : float  Relative MIP gap tolerance (default 1e-4)
time_limit_sec: float  Time limit (default 300.0)
initial_solution: np.ndarray  Optional incumbent or repair seed (n,)
maximize      : bool

Returns
-------
dict with keys:
  success    : bool
  objective  : float
  x          : np.ndarray  (n,)
  mip_gap    : float
  status     : str
  solver     : str
  runtime_sec: float
)doc");

  m_eng.def("mip_summary",
    [](const py::array_t<double>& c,
       const py::object& A,
       const py::object& b,
       const py::object& Aeq,
       const py::object& beq,
       const py::object& lb,
       const py::object& ub,
       const py::list& vartypes,
       bool maximize,
       bool include_matrix_values,
       bool include_rhs_values,
       bool include_objective_values) -> py::dict {
      auto mip = build_mip_model(c, A, b, Aeq, beq, lb, ub, vartypes, maximize);
      mipsolvers::l2o::FingerprintOptions fp;
      fp.include_matrix_values = include_matrix_values;
      fp.include_rhs_values = include_rhs_values;
      fp.include_objective_values = include_objective_values;
      return json_to_py(mip_summary_to_json(mip, fp)).cast<py::dict>();
    },
    py::arg("c"),
    py::arg("A") = py::none(),
    py::arg("b") = py::none(),
    py::arg("Aeq") = py::none(),
    py::arg("beq") = py::none(),
    py::arg("lb") = py::none(),
    py::arg("ub") = py::none(),
    py::arg("vartypes") = py::list(),
    py::arg("maximize") = false,
    py::arg("include_matrix_values") = false,
    py::arg("include_rhs_values") = false,
    py::arg("include_objective_values") = false,
    R"doc(
Return dimensions, integer columns, branch metadata, and an L2O fingerprint for
a NumPy-defined MILP without solving it.
)doc");

  m_eng.def("solve_milp_bc",
    [](const py::array_t<double>& c,
       const py::object& A,
       const py::object& b,
       const py::object& Aeq,
       const py::object& beq,
       const py::object& lb,
       const py::object& ub,
       const py::list& vartypes,
       const mipsolvers::engine::BCOptions& options,
       const py::object& initial_solution,
       bool strict_highs,
       bool maximize,
       bool include_artifact_vectors) -> py::dict {
      auto mip = build_mip_model(c, A, b, Aeq, beq, lb, ub, vartypes, maximize);
      set_initial_solution_from_python(mip, initial_solution);
      auto opt = options;
      if (strict_highs) opt = mipsolvers::engine::make_strict_highs_problem_options(mip, opt);

      py::gil_scoped_release rel;
      auto res = mipsolvers::engine::solve_milp_bc(mip, opt);
      py::gil_scoped_acquire acq;
      return bc_result_to_py_dict(res, include_artifact_vectors);
    },
    py::arg("c"),
    py::arg("A") = py::none(),
    py::arg("b") = py::none(),
    py::arg("Aeq") = py::none(),
    py::arg("beq") = py::none(),
    py::arg("lb") = py::none(),
    py::arg("ub") = py::none(),
    py::arg("vartypes") = py::list(),
    py::arg("options") = mipsolvers::engine::BCOptions{},
    py::arg("initial_solution") = py::none(),
    py::arg("strict_highs") = true,
    py::arg("maximize") = false,
    py::arg("include_artifact_vectors") = false,
    R"doc(
Solve a NumPy-defined MILP through the native branch-and-cut API and return
standard stats, detailed BCStats, and optional warm-start artifacts. This is the
main Python research/evaluation surface for learned warm starts and option
tuning; production solving still executes in C++.
)doc");

  // ── list_solvers ───────────────────────────────────────────────────────────
  m_eng.def("list_solvers",
    [](const std::string& problem_class) -> std::vector<std::string> {
      mipsolvers::engine::SolverEngine eng;
      using PC = mipsolvers::engine::ProblemClass;
      PC cls = PC::MILP;
      if (problem_class == "LP")    cls = PC::LP;
      else if (problem_class == "MILP")  cls = PC::MILP;
      else if (problem_class == "QP")    cls = PC::QP;
      else if (problem_class == "NLP")   cls = PC::NLP;
      else if (problem_class == "MINLP") cls = PC::MINLP;
      else if (problem_class == "LE")    cls = PC::LE;
      else if (problem_class == "NLE")   cls = PC::NLE;
      return eng.list_solvers(cls);
    },
    py::arg("problem_class") = "MILP",
    R"doc(
List registered solver adapters for a given problem class.

Parameters
----------
problem_class : str  "LP" | "MILP" | "QP" | "NLP" | "MINLP" | "LE" | "NLE"

Returns
-------
list[str]  Available solver names (in priority order for "Auto")
)doc");

  // ═══════════════════════════════════════════════════════════════════════════
  // L2O submodule
  // ═══════════════════════════════════════════════════════════════════════════
  py::module_ m_l2o = m.def_submodule("l2o", R"doc(
Learning-to-optimize dataset and policy metadata helpers.

This submodule exposes stable fingerprints and metadata structures for Python
training workflows while keeping model construction and solving in C++.
)doc");

  py::enum_<mipsolvers::l2o::PolicyMode>(m_l2o, "PolicyMode")
    .value("Off", mipsolvers::l2o::PolicyMode::Off)
    .value("TraceOnly", mipsolvers::l2o::PolicyMode::TraceOnly)
    .value("Advisory", mipsolvers::l2o::PolicyMode::Advisory)
    .value("Guarded", mipsolvers::l2o::PolicyMode::Guarded)
    .value("Experimental", mipsolvers::l2o::PolicyMode::Experimental);

  py::class_<mipsolvers::l2o::FingerprintOptions>(m_l2o, "FingerprintOptions")
    .def(py::init<>())
    .def_readwrite("include_objective_sense", &mipsolvers::l2o::FingerprintOptions::include_objective_sense)
    .def_readwrite("include_variable_names", &mipsolvers::l2o::FingerprintOptions::include_variable_names)
    .def_readwrite("include_bounds", &mipsolvers::l2o::FingerprintOptions::include_bounds)
    .def_readwrite("include_matrix_values", &mipsolvers::l2o::FingerprintOptions::include_matrix_values)
    .def_readwrite("include_rhs_values", &mipsolvers::l2o::FingerprintOptions::include_rhs_values)
    .def_readwrite("include_objective_values", &mipsolvers::l2o::FingerprintOptions::include_objective_values)
    .def_readwrite("include_uc_hint", &mipsolvers::l2o::FingerprintOptions::include_uc_hint)
    .def("to_dict", [](const mipsolvers::l2o::FingerprintOptions& opt) {
      nlohmann::json j = opt;
      return json_to_py(j);
    });

  py::class_<mipsolvers::l2o::PolicyMetadata>(m_l2o, "PolicyMetadata")
    .def(py::init<>())
    .def_readwrite("name", &mipsolvers::l2o::PolicyMetadata::name)
    .def_readwrite("version", &mipsolvers::l2o::PolicyMetadata::version)
    .def_readwrite("model_hash", &mipsolvers::l2o::PolicyMetadata::model_hash)
    .def_readwrite("feature_schema_version", &mipsolvers::l2o::PolicyMetadata::feature_schema_version)
    .def_readwrite("training_distribution", &mipsolvers::l2o::PolicyMetadata::training_distribution)
    .def_readwrite("max_inference_time_sec", &mipsolvers::l2o::PolicyMetadata::max_inference_time_sec)
    .def_readwrite("mode", &mipsolvers::l2o::PolicyMetadata::mode)
    .def("to_dict", [](const mipsolvers::l2o::PolicyMetadata& metadata) {
      nlohmann::json j = metadata;
      return json_to_py(j);
    });

  py::class_<mipsolvers::l2o::SolverConfigPolicyOptions>(m_l2o, "SolverConfigPolicyOptions", R"doc(
Policy knobs for Phase 4 solver-configuration learning and artifact reuse.
The default is conservative and only changes solver options through existing
branch-and-cut safety surfaces.
)doc")
    .def(py::init<>())
    .def_readwrite("enable_config_tuning", &mipsolvers::l2o::SolverConfigPolicyOptions::enable_config_tuning)
    .def_readwrite("enable_artifact_reuse", &mipsolvers::l2o::SolverConfigPolicyOptions::enable_artifact_reuse)
    .def_readwrite("deterministic_evaluation", &mipsolvers::l2o::SolverConfigPolicyOptions::deterministic_evaluation)
    .def_readwrite("short_time_limit_sec", &mipsolvers::l2o::SolverConfigPolicyOptions::short_time_limit_sec)
    .def_readwrite("short_budget_heuristic_effort", &mipsolvers::l2o::SolverConfigPolicyOptions::short_budget_heuristic_effort)
    .def_readwrite("short_budget_root_sepa_rounds", &mipsolvers::l2o::SolverConfigPolicyOptions::short_budget_root_sepa_rounds)
    .def_readwrite("artifact_root_sepa_rounds", &mipsolvers::l2o::SolverConfigPolicyOptions::artifact_root_sepa_rounds)
    .def_readwrite("scuc_min_binary_vars", &mipsolvers::l2o::SolverConfigPolicyOptions::scuc_min_binary_vars)
    .def_readwrite("scuc_min_rows", &mipsolvers::l2o::SolverConfigPolicyOptions::scuc_min_rows)
    .def_readwrite("presolve_substitution_maxfillin", &mipsolvers::l2o::SolverConfigPolicyOptions::presolve_substitution_maxfillin)
    .def_readwrite("mip_lp_age_limit", &mipsolvers::l2o::SolverConfigPolicyOptions::mip_lp_age_limit)
    .def_readwrite("short_budget_max_stall_nodes", &mipsolvers::l2o::SolverConfigPolicyOptions::short_budget_max_stall_nodes)
    .def("to_dict", [](const mipsolvers::l2o::SolverConfigPolicyOptions& options) {
      nlohmann::json j = options;
      return json_to_py(j);
    });

  py::class_<mipsolvers::l2o::SolverArtifactCache>(m_l2o, "SolverArtifactCache", R"doc(
In-memory cache of root cuts, root basis, and pseudocost artifacts keyed by a
structural L2O model fingerprint. Artifacts are reused only when fingerprint
and dimensional validation succeed.
)doc")
    .def(py::init<>())
    .def("empty", &mipsolvers::l2o::SolverArtifactCache::empty)
    .def("size", &mipsolvers::l2o::SolverArtifactCache::size)
    .def("clear", &mipsolvers::l2o::SolverArtifactCache::clear)
    .def("to_dict", [](const mipsolvers::l2o::SolverArtifactCache& cache) {
      return json_to_py(cache.summary());
    });

  py::class_<mipsolvers::l2o::SCUCBranchingPolicyOptions>(m_l2o, "SCUCBranchingPolicyOptions", R"doc(
Policy knobs for Phase 5 learned SCUC branching priorities. Static priorities
are installed on the MIP model; dynamic priors are advisory callbacks consumed
by the native branch selector when that path is active.
)doc")
    .def(py::init<>())
    .def_readwrite("enable_static_priorities", &mipsolvers::l2o::SCUCBranchingPolicyOptions::enable_static_priorities)
    .def_readwrite("enable_dynamic_priors", &mipsolvers::l2o::SCUCBranchingPolicyOptions::enable_dynamic_priors)
    .def_readwrite("clear_existing_priorities", &mipsolvers::l2o::SCUCBranchingPolicyOptions::clear_existing_priorities)
    .def_readwrite("include_startup_shutdown", &mipsolvers::l2o::SCUCBranchingPolicyOptions::include_startup_shutdown)
    .def_readwrite("include_dispatch_priorities", &mipsolvers::l2o::SCUCBranchingPolicyOptions::include_dispatch_priorities)
    .def_readwrite("prefer_earlier_periods", &mipsolvers::l2o::SCUCBranchingPolicyOptions::prefer_earlier_periods)
    .def_readwrite("dynamic_scale_by_fractionality", &mipsolvers::l2o::SCUCBranchingPolicyOptions::dynamic_scale_by_fractionality)
    .def_readwrite("commitment_base_priority", &mipsolvers::l2o::SCUCBranchingPolicyOptions::commitment_base_priority)
    .def_readwrite("transition_base_priority", &mipsolvers::l2o::SCUCBranchingPolicyOptions::transition_base_priority)
    .def_readwrite("dispatch_base_priority", &mipsolvers::l2o::SCUCBranchingPolicyOptions::dispatch_base_priority)
    .def_readwrite("time_priority_scale", &mipsolvers::l2o::SCUCBranchingPolicyOptions::time_priority_scale)
    .def_readwrite("learned_priority_scale", &mipsolvers::l2o::SCUCBranchingPolicyOptions::learned_priority_scale)
    .def_readwrite("transition_score_scale", &mipsolvers::l2o::SCUCBranchingPolicyOptions::transition_score_scale)
    .def_readwrite("dispatch_score_scale", &mipsolvers::l2o::SCUCBranchingPolicyOptions::dispatch_score_scale)
    .def_readwrite("dynamic_prior_weight", &mipsolvers::l2o::SCUCBranchingPolicyOptions::dynamic_prior_weight)
    .def_readwrite("score_epsilon", &mipsolvers::l2o::SCUCBranchingPolicyOptions::score_epsilon)
    .def("to_dict", [](const mipsolvers::l2o::SCUCBranchingPolicyOptions& options) {
      nlohmann::json j = options;
      return json_to_py(j);
    });

  m_l2o.def("scuc_mip_summary",
    [](const mipsolvers::scuc::SCUCInput& inp,
       const mipsolvers::l2o::FingerprintOptions& fp) -> py::dict {
      auto mip = mipsolvers::scuc::build_scuc_mip(inp);
      return json_to_py(mip_summary_to_json(mip, fp)).cast<py::dict>();
    },
    py::arg("input"),
    py::arg("fingerprint_options") = mipsolvers::l2o::FingerprintOptions{},
    R"doc(
Build an SCUC MIP in C++ and return the L2O summary/fingerprint dictionary.
)doc");

  m_l2o.def("fingerprint_scuc_input",
    [](const mipsolvers::scuc::SCUCInput& inp,
       const mipsolvers::l2o::FingerprintOptions& fp) -> py::dict {
      auto mip = mipsolvers::scuc::build_scuc_mip(inp);
      nlohmann::json j = mipsolvers::l2o::fingerprint_mip(mip, fp);
      return json_to_py(j).cast<py::dict>();
    },
    py::arg("input"),
    py::arg("fingerprint_options") = mipsolvers::l2o::FingerprintOptions{},
    R"doc(Return only the structural/numeric fingerprint for an SCUC input.)doc");

  m_l2o.def("scuc_solver_config_features",
    [](const mipsolvers::scuc::SCUCInput& inp,
       const mipsolvers::engine::BCOptions& options,
       bool strict_highs) -> py::dict {
      auto mip = mipsolvers::scuc::build_scuc_mip(inp);
      auto opt = options;
      if (strict_highs) opt = mipsolvers::engine::make_strict_highs_problem_options(mip, opt);
      const auto features = mipsolvers::l2o::extract_solver_config_features(mip, opt);
      nlohmann::json j = features;
      return json_to_py(j).cast<py::dict>();
    },
    py::arg("input"),
    py::arg("options") = mipsolvers::engine::BCOptions{},
    py::arg("strict_highs") = true,
    R"doc(Return the compact Phase 4 solver-configuration feature dictionary for an SCUC input.)doc");

  m_l2o.def("scuc_generator_time_features",
    [](const mipsolvers::scuc::SCUCInput& inp) -> py::dict {
      const int ng = static_cast<int>(inp.generators.size());
      const int T = std::max(0, inp.config.num_periods);
      const std::vector<std::string> feature_names{
          "bias",
          "time_index",
          "time_fraction",
          "pmin_mw",
          "pmax_mw",
          "pmin_pmax_ratio",
          "ramp_up_mw_min",
          "ramp_down_mw_min",
          "min_up_time_hr",
          "min_down_time_hr",
          "startup_cost",
          "no_load_cost",
          "initial_commitment",
          "time_in_state_hr",
          "demand_mw",
          "net_load_mw",
          "load_ratio",
          "net_load_ratio",
          "up_reserve_mw",
          "generator_pmax_share",
          "must_run"};
      const int nf = static_cast<int>(feature_names.size());
      py::array_t<double> features({static_cast<py::ssize_t>(ng),
                                    static_cast<py::ssize_t>(T),
                                    static_cast<py::ssize_t>(nf)});
      auto data = features.mutable_unchecked<3>();

      double total_pmax = 0.0;
      for (const auto& gen : inp.generators) total_pmax += std::max(0.0, gen.pmax);
      const double scale = std::max(1.0, total_pmax);

      for (int t = 0; t < T; ++t) {
        const double load = total_load_mw(inp, t);
        const double net_load = load - total_wind_mw(inp, t) - total_solar_mw(inp, t);
        const double up_reserve = (inp.config.spinning_reserve_req + inp.config.regulation_up_req) * load;
        for (int g = 0; g < ng; ++g) {
          const auto& gen = inp.generators[static_cast<size_t>(g)];
          const double init = (g < static_cast<int>(inp.initial_status.commitment.size()))
                                  ? inp.initial_status.commitment[static_cast<size_t>(g)]
                                  : 0.0;
          const double time_in_state = (g < static_cast<int>(inp.initial_status.time_in_state.size()))
                                           ? inp.initial_status.time_in_state[static_cast<size_t>(g)]
                                           : 0.0;
          int k = 0;
          data(g, t, k++) = 1.0;
          data(g, t, k++) = static_cast<double>(t);
          data(g, t, k++) = T > 1 ? static_cast<double>(t) / static_cast<double>(T - 1) : 0.0;
          data(g, t, k++) = gen.pmin;
          data(g, t, k++) = gen.pmax;
          data(g, t, k++) = gen.pmax > 0.0 ? gen.pmin / gen.pmax : 0.0;
          data(g, t, k++) = gen.ramp_up_mw_min;
          data(g, t, k++) = gen.ramp_dn_mw_min;
          data(g, t, k++) = gen.min_up_time_hr;
          data(g, t, k++) = gen.min_dn_time_hr;
          data(g, t, k++) = gen.startup_cost;
          data(g, t, k++) = gen.no_load_cost;
          data(g, t, k++) = init;
          data(g, t, k++) = time_in_state;
          data(g, t, k++) = load;
          data(g, t, k++) = net_load;
          data(g, t, k++) = load / scale;
          data(g, t, k++) = net_load / scale;
          data(g, t, k++) = up_reserve;
          data(g, t, k++) = gen.pmax / scale;
          data(g, t, k++) = gen.must_run ? 1.0 : 0.0;
        }
      }

      py::dict out;
      out["features"] = features;
      out["feature_names"] = feature_names;
      out["shape"] = py::make_tuple(ng, T, nf);
      return out;
    },
    py::arg("input"),
    R"doc(
Return a generator-time feature tensor with shape [ng, T, n_features]. These
features are intentionally simple and dependency-free so Python training code
can use NumPy, scikit-learn, PyTorch, or any other model stack outside C++.
)doc");

  m_l2o.def("scuc_commitment_labels",
    [](const mipsolvers::scuc::SCUCSolveResult& result) -> py::dict {
      py::dict out;
      out["commitment"] = matrix_to_numpy_2d(result.commitment);
      out["startup"] = matrix_to_numpy_2d(result.startup);
      out["shutdown"] = matrix_to_numpy_2d(result.shutdown);
      out["objective"] = result.objective;
      out["mip_gap"] = result.mip_gap;
      out["converged"] = result.converged;
      return out;
    },
    py::arg("result"),
    R"doc(Extract NumPy label arrays from an SCUCSolveResult.)doc");

  m_l2o.def("make_scuc_commitment_warm_start",
    [](const mipsolvers::scuc::SCUCInput& inp,
       const py::object& commitment,
       const py::object& startup,
       const py::object& shutdown,
       const py::object& initial_solution,
       double binary_threshold,
       bool infer_missing_transitions,
       bool initialize_from_existing_seed) -> py::dict {
      auto mip = mipsolvers::scuc::build_scuc_mip(inp);
      set_initial_solution_from_python(mip, initial_solution);
      const auto commitment_matrix = matrix_from_python_2d(commitment, "commitment");
      const auto startup_matrix = matrix_from_python_2d(startup, "startup");
      const auto shutdown_matrix = matrix_from_python_2d(shutdown, "shutdown");
      const auto options = make_scuc_warm_start_options(
          binary_threshold, infer_missing_transitions, initialize_from_existing_seed);
      auto warm = mipsolvers::l2o::make_scuc_commitment_warm_start(
          mip, commitment_matrix, startup_matrix, shutdown_matrix, options);

      py::dict out;
      out["x"] = vec_to_numpy(warm.x);
      out["report"] = scuc_warm_start_report_to_py(warm.report);
      out["mip_summary"] = json_to_py(mip_summary_to_json(mip, mipsolvers::l2o::FingerprintOptions{}));
      return out;
    },
    py::arg("input"),
    py::arg("commitment"),
    py::arg("startup") = py::none(),
    py::arg("shutdown") = py::none(),
    py::arg("initial_solution") = py::none(),
    py::arg("binary_threshold") = 0.5,
    py::arg("infer_missing_transitions") = true,
    py::arg("initialize_from_existing_seed") = true,
    R"doc(
Convert learned SCUC commitment probabilities or binaries into a full MIP
initial-solution vector. Startup/shutdown are inferred from commitment and the
initial unit status unless explicitly supplied.
)doc");

  m_l2o.def("make_scuc_branching_priorities",
    [](const mipsolvers::scuc::SCUCInput& inp,
       const py::object& generator_time_scores,
       const mipsolvers::l2o::SCUCBranchingPolicyOptions& policy_options) -> py::dict {
      auto mip = mipsolvers::scuc::build_scuc_mip(inp);
      const auto score_matrix = matrix_from_python_2d(generator_time_scores, "generator_time_scores");
      mipsolvers::l2o::SCUCBranchingPriorityReport report;
      const auto priorities = mipsolvers::l2o::make_scuc_branching_priorities(
          mip, score_matrix, policy_options, &report);
      py::dict out;
      out["priority_vector"] = priorities;
      nlohmann::json report_json = report;
      out["report"] = json_to_py(report_json);
      out["mip_summary"] = json_to_py(mip_summary_to_json(mip, mipsolvers::l2o::FingerprintOptions{}));
      return out;
    },
    py::arg("input"),
    py::arg("generator_time_scores") = py::none(),
    py::arg("policy_options") = mipsolvers::l2o::SCUCBranchingPolicyOptions{},
    R"doc(
Build the SCUC MIP and return the Phase 5 learned branching-priority vector
without solving. generator_time_scores should have shape [ng, T]; larger values
mean branch earlier for the corresponding unit commitment variables.
)doc");

  m_l2o.def("solve_scuc_mip",
    [](const mipsolvers::scuc::SCUCInput& inp,
       const mipsolvers::engine::BCOptions& options,
       bool strict_highs,
       bool include_artifact_vectors) -> py::dict {
      auto mip = mipsolvers::scuc::build_scuc_mip(inp);
      auto opt = options;
      if (strict_highs) opt = mipsolvers::engine::make_strict_highs_problem_options(mip, opt);

      py::gil_scoped_release rel;
      auto result = mipsolvers::engine::solve_milp_bc(mip, opt);
      py::gil_scoped_acquire acq;

      return bc_result_to_py_dict(result, include_artifact_vectors);
    },
    py::arg("input"),
    py::arg("options") = mipsolvers::engine::BCOptions{},
    py::arg("strict_highs") = true,
    py::arg("include_artifact_vectors") = false,
    R"doc(
Solve the raw SCUC MIP through the same branch-and-cut path used by
solve_scuc_mip_with_warm_start, but without replacing the model's default
initial solution. This is the baseline evaluation surface for learned starts.
)doc");

  m_l2o.def("solve_scuc_mip_with_config_policy",
    [](const mipsolvers::scuc::SCUCInput& inp,
       const mipsolvers::engine::BCOptions& options,
       const mipsolvers::l2o::SolverConfigPolicyOptions& policy_options,
       const py::object& artifact_cache,
       bool update_artifacts,
       bool strict_highs,
       bool include_artifact_vectors) -> py::dict {
      auto mip = mipsolvers::scuc::build_scuc_mip(inp);
      auto opt = options;
      if (strict_highs) opt = mipsolvers::engine::make_strict_highs_problem_options(mip, opt);

      mipsolvers::l2o::SolverArtifactCache* cache = nullptr;
      if (!artifact_cache.is_none()) {
        cache = artifact_cache.cast<mipsolvers::l2o::SolverArtifactCache*>();
      }

      nlohmann::json cache_before = cache ? cache->summary() : nlohmann::json{{"size", 0}};
      mipsolvers::l2o::SolverArtifactReuseReport reuse_report;
      reuse_report.attempted = cache != nullptr && policy_options.enable_artifact_reuse;
      if (cache && policy_options.enable_artifact_reuse) {
        reuse_report = cache->apply(mip, opt);
      } else {
        reuse_report.message = cache ? "artifact reuse disabled" : "no artifact cache supplied";
      }

      const auto planned_features = mipsolvers::l2o::extract_solver_config_features(mip, opt);
      const auto planned_decision = mipsolvers::l2o::choose_solver_config(
          planned_features, opt, policy_options);
      std::vector<mipsolvers::l2o::SolverConfigRunRecord> records;
      auto callbacks = mipsolvers::l2o::make_solver_config_callbacks(
          policy_options, &records, "l2o-phase4-config");

      py::gil_scoped_release rel;
      auto result = mipsolvers::engine::solve_milp_bc(
          mip, opt, mipsolvers::engine::BCWarmStart{}, callbacks);
      py::gil_scoped_acquire acq;

      if (cache && update_artifacts) cache->store(mip, result);

      auto out = bc_result_to_py_dict(result, include_artifact_vectors);
      nlohmann::json config = nlohmann::json::object();
      config["planned_decision"] = planned_decision;
      config["artifact_reuse"] = reuse_report;
      config["cache_before"] = cache_before;
      config["cache_after"] = cache ? cache->summary() : nlohmann::json{{"size", 0}};
      config["run_records"] = records;
      out["solver_config"] = json_to_py(config);
      return out;
    },
    py::arg("input"),
    py::arg("options") = mipsolvers::engine::BCOptions{},
    py::arg("policy_options") = mipsolvers::l2o::SolverConfigPolicyOptions{},
    py::arg("artifact_cache") = py::none(),
    py::arg("update_artifacts") = true,
    py::arg("strict_highs") = true,
    py::arg("include_artifact_vectors") = false,
    R"doc(
Solve an SCUC MIP with the Phase 4 configuration policy and optional artifact
reuse. The policy is deployed through the branch-and-cut hyperparameter tuner;
post-solve records are returned for offline training tables. When an artifact
cache is supplied, compatible root cuts, root basis, and pseudocost artifacts
are installed through BCOptions before solving and refreshed after the run when
update_artifacts is true.
)doc");

  m_l2o.def("solve_scuc_mip_with_branching_policy",
    [](const mipsolvers::scuc::SCUCInput& inp,
       const py::object& generator_time_scores,
       const mipsolvers::engine::BCOptions& options,
       const mipsolvers::l2o::SCUCBranchingPolicyOptions& policy_options,
       bool strict_highs,
       bool include_artifact_vectors) -> py::dict {
      auto mip = mipsolvers::scuc::build_scuc_mip(inp);
      const auto score_matrix = matrix_from_python_2d(generator_time_scores, "generator_time_scores");
      auto report = mipsolvers::l2o::apply_scuc_branching_priorities(
          mip, score_matrix, policy_options);
      auto callbacks = mipsolvers::l2o::make_scuc_branching_callbacks(
          mip, score_matrix, policy_options, "l2o-phase5-branching");
      report.installed_dynamic_prior = static_cast<bool>(callbacks.branching_prior);

      auto opt = options;
      if (strict_highs) opt = mipsolvers::engine::make_strict_highs_problem_options(mip, opt);

      py::gil_scoped_release rel;
      auto result = callbacks.branching_prior
                        ? mipsolvers::engine::solve_milp_bc(
                              mip, opt, mipsolvers::engine::BCWarmStart{}, callbacks)
                        : mipsolvers::engine::solve_milp_bc(mip, opt);
      py::gil_scoped_acquire acq;

      auto out = bc_result_to_py_dict(result, include_artifact_vectors);
      nlohmann::json policy_json = nlohmann::json::object();
      policy_json["report"] = report;
      policy_json["options"] = policy_options;
      out["branching_policy"] = json_to_py(policy_json);
      return out;
    },
    py::arg("input"),
    py::arg("generator_time_scores") = py::none(),
    py::arg("options") = mipsolvers::engine::BCOptions{},
    py::arg("policy_options") = mipsolvers::l2o::SCUCBranchingPolicyOptions{},
    py::arg("strict_highs") = true,
    py::arg("include_artifact_vectors") = false,
    R"doc(
Solve an SCUC MIP with Phase 5 learned branching priorities. The static
priority vector is installed directly on the C++ MIP model; the dynamic prior is
passed as an advisory branch-and-cut callback and ignored safely by solver paths
that do not request branch-callback scores.
)doc");

  m_l2o.def("solve_scuc_mip_with_warm_start",
    [](const mipsolvers::scuc::SCUCInput& inp,
       const py::object& commitment,
       const py::object& startup,
       const py::object& shutdown,
       const py::object& initial_solution,
       const mipsolvers::engine::BCOptions& options,
       double binary_threshold,
       bool infer_missing_transitions,
       bool initialize_from_existing_seed,
       bool strict_highs,
       bool include_artifact_vectors) -> py::dict {
      auto mip = mipsolvers::scuc::build_scuc_mip(inp);
      set_initial_solution_from_python(mip, initial_solution);
      const auto commitment_matrix = matrix_from_python_2d(commitment, "commitment");
      const auto startup_matrix = matrix_from_python_2d(startup, "startup");
      const auto shutdown_matrix = matrix_from_python_2d(shutdown, "shutdown");
      const auto warm_options = make_scuc_warm_start_options(
          binary_threshold, infer_missing_transitions, initialize_from_existing_seed);
      auto warm = mipsolvers::l2o::make_scuc_commitment_warm_start(
          mip, commitment_matrix, startup_matrix, shutdown_matrix, warm_options);
      mip.initial_solution = warm.x;
      auto opt = options;
      opt.time_limit_sec = options.time_limit_sec;
      opt.gap_tol = options.gap_tol;
      if (strict_highs) opt = mipsolvers::engine::make_strict_highs_problem_options(mip, opt);

      py::gil_scoped_release rel;
      auto result = mipsolvers::engine::solve_milp_bc(mip, opt);
      py::gil_scoped_acquire acq;

      auto out = bc_result_to_py_dict(result, include_artifact_vectors);
      out["warm_start_report"] = scuc_warm_start_report_to_py(warm.report);
      return out;
    },
    py::arg("input"),
    py::arg("commitment"),
    py::arg("startup") = py::none(),
    py::arg("shutdown") = py::none(),
    py::arg("initial_solution") = py::none(),
    py::arg("options") = mipsolvers::engine::BCOptions{},
    py::arg("binary_threshold") = 0.5,
    py::arg("infer_missing_transitions") = true,
    py::arg("initialize_from_existing_seed") = true,
    py::arg("strict_highs") = true,
    py::arg("include_artifact_vectors") = false,
    R"doc(
Evaluate a learned SCUC commitment warm start by building the SCUC MIP in C++,
installing the predicted initial solution, and solving through branch-and-cut.
The solver still validates and repairs the seed before accepting an incumbent.
)doc");

  // ── AML submodule ──────────────────────────────────────────────────────────
  bind_aml(m);
}
