/// Cross-algorithm benchmark over the NETLIB MPS data in tests/data/netlib.
///
/// The comparison is deliberately end-to-end at the adapter boundary: model
/// setup, backend presolve, and solve time are included, while the shared MPS
/// parse is measured separately. Results are independently audited against the
/// original LPModel and the reference objectives in mps_manifest.csv.
///
/// Usage:
///   netlib_solver_benchmark --data-dir tests/data --repeat 1
///       --time-limit 30 --csv reports/netlib_benchmark.csv
///       --json reports/netlib_benchmark.json
///   netlib_solver_benchmark --case afiro --solvers highs-simplex,ipopt
/// Fully native comparison key: native-dual-direct (no HiGHS presolve).
/// DSE comparison keys: native-dual-devex, native-dual-structural-dse,
/// native-dual-exact-dse, native-dual-certified-dse.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <numeric>
#include <sstream>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <nlohmann/json.hpp>

#include "Highs.h"
#include "io/HMPSIO.h"

#include "mipsolvers/engine/kernel/ipm/ipm_lp_solver.hpp"
#include "mipsolvers/engine/kernel/ipm/lcqp_solver.hpp"
#include "mipsolvers/engine/kernel/lp_kernel/dual_simplex.hpp"
#include "mipsolvers/engine/problem_types.hpp"
#include "mipsolvers/engine/solver/external/adapters.hpp"
#include "mipsolvers/engine/solver/native/lp/pdlp_solver.hpp"
#include "mipsolvers/engine/solver/native/native_lp_selector.hpp"
#include "../src/engine/kernel/lp_kernel/native_dual_core.hpp"

#ifdef HACDCPF_HAVE_SCIP_LIB
#include "scip/scip.h"
#include "scip/scipdefplugins.h"
#endif

namespace fs = std::filesystem;
using json = nlohmann::json;
namespace eng = mipsolvers::engine;

namespace {

constexpr double kObjectiveTolerance = 1e-5;
constexpr double kFeasibilityTolerance = 1e-7;
// S1 measurement-contract schema version. Bump when the report field set or
// aggregation semantics change so downstream comparisons stay well-defined.
constexpr const char* kSchemaVersion = "s1-measurement-contract-2";

struct Config {
  fs::path data_dir{"tests/data"};
  fs::path csv_path;
  fs::path json_path;
  std::string case_filter;
  std::vector<std::string> case_names;
  std::vector<std::string> solvers;
  int repeats{1};
  int max_iterations{100000};
  double time_limit_sec{30.0};
  bool warm_cohort{false};
  int warm_branch_vars{4};
  double warm_branch_frac{0.5};
};

struct CaseInfo {
  std::string name;
  fs::path path;
  double reference_objective{std::numeric_limits<double>::quiet_NaN()};
  eng::LPModel lp;
  double load_ms{0.0};
};

struct RunResult {
  std::string case_name;
  std::string solver;
  int repeat{0};
  int rows{0};
  int columns{0};
  int nonzeros{0};
  bool available{true};
  bool success{false};
  bool accurate{false};
  int iterations{0};
  double runtime_ms{0.0};
  bool native_telemetry{false};
  int dual_pivots{0};
  int dse_initialization_solves{0};
  int certified_dse_btrans{0};
  int certified_dse_candidates{0};
  int certified_dse_rejections{0};
  double dse_initialization_ms{0.0};
  double certified_dse_ms{0.0};
  double native_kernel_ms{0.0};
  double native_kernel_ms_per_dual_pivot{0.0};
  double objective{std::numeric_limits<double>::quiet_NaN()};
  double reference_objective{std::numeric_limits<double>::quiet_NaN()};
  double objective_rel_error{std::numeric_limits<double>::infinity()};
  double max_row_violation{std::numeric_limits<double>::infinity()};
  double max_bound_violation{std::numeric_limits<double>::infinity()};
  double normalized_primal_violation{std::numeric_limits<double>::infinity()};
  double relative_primal_residual{std::numeric_limits<double>::quiet_NaN()};
  double relative_dual_residual{std::numeric_limits<double>::quiet_NaN()};
  double relative_gap{std::numeric_limits<double>::quiet_NaN()};
  double dual_objective{std::numeric_limits<double>::quiet_NaN()};
  // LP presolve telemetry (SolveStats; design §3.2).  -1 = presolve did not
  // run; presolve_used: 0 none, 1 native, 2 highs bridge, 3 fallback direct.
  double presolve_ms{0.0};
  long presolve_orig_rows{-1};
  long presolve_orig_cols{-1};
  long presolve_orig_nnz{-1};
  long presolve_reduced_rows{-1};
  long presolve_reduced_cols{-1};
  long presolve_reduced_nnz{-1};
  int presolve_used{0};
  std::string status;
  Eigen::VectorXd x;
};

struct Summary {
  std::string solver;
  int attempts{0};
  int available{0};
  int successes{0};
  int accurate{0};
  double total_sec{0.0};
  double median_ms{0.0};
  double geometric_mean_ms{0.0};
  int native_telemetry_count{0};
  double mean_dual_pivots{0.0};
  double mean_dse_initialization_ms{0.0};
  double geometric_mean_kernel_ms_per_dual_pivot{0.0};
  double geometric_speedup_vs_highs_simplex{
      std::numeric_limits<double>::quiet_NaN()};
};

std::vector<std::string> split(const std::string& text, char delimiter) {
  std::vector<std::string> out;
  std::istringstream in(text);
  std::string item;
  while (std::getline(in, item, delimiter)) {
    if (!item.empty()) out.push_back(item);
  }
  return out;
}

std::string csv_escape(const std::string& text) {
  if (text.find_first_of(",\"\n") == std::string::npos) return text;
  std::string out = "\"";
  for (char c : text) out += (c == '\"') ? "\"\"" : std::string(1, c);
  return out + "\"";
}

bool parse_args(int argc, char** argv, Config& cfg) {
  const std::vector<std::string> defaults = {
      "highs-simplex", "highs-ipm", "highs-pdlp", "scip-direct", "scip",
      "native-dual-simplex", "native-ipm", "native-ipm-direct", "native-pdlp",
      "native-lcqp", "ipopt"};
  cfg.solvers = defaults;
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
    } else if (arg == "--csv") {
      const char* v = value("--csv"); if (!v) return false; cfg.csv_path = v;
    } else if (arg == "--json") {
      const char* v = value("--json"); if (!v) return false; cfg.json_path = v;
    } else if (arg == "--case") {
      const char* v = value("--case"); if (!v) return false; cfg.case_filter = v;
    } else if (arg == "--cases") {
      const char* v = value("--cases"); if (!v) return false;
      cfg.case_names = split(v, ',');
    } else if (arg == "--solvers") {
      const char* v = value("--solvers"); if (!v) return false;
      cfg.solvers = split(v, ',');
    } else if (arg == "--repeat") {
      const char* v = value("--repeat"); if (!v) return false;
      cfg.repeats = std::max(1, std::atoi(v));
    } else if (arg == "--time-limit") {
      const char* v = value("--time-limit"); if (!v) return false;
      cfg.time_limit_sec = std::max(0.001, std::atof(v));
    } else if (arg == "--max-iterations") {
      const char* v = value("--max-iterations"); if (!v) return false;
      cfg.max_iterations = std::max(1, std::atoi(v));
    } else if (arg == "--warm-cohort") {
      cfg.warm_cohort = true;
    } else if (arg == "--warm-branch-vars") {
      const char* v = value("--warm-branch-vars"); if (!v) return false;
      cfg.warm_branch_vars = std::max(1, std::atoi(v));
    } else if (arg == "--warm-branch-frac") {
      const char* v = value("--warm-branch-frac"); if (!v) return false;
      cfg.warm_branch_frac = std::min(0.99, std::max(0.01, std::atof(v)));
    } else if (arg == "--help" || arg == "-h") {
      std::cout
          << "Usage: netlib_solver_benchmark [options]\n"
          << "  --data-dir DIR       tests/data directory\n"
          << "  --case TEXT          run matching instance names only\n"
          << "  --cases A,B          run an exact comma-separated case list\n"
          << "  --solvers A,B        select algorithms\n"
          << "                       IPM keys: native-ipm-direct,\n"
          << "                       native-ipm-legacy-step\n"
          << "                       native DSE keys: native-dual-devex,\n"
          << "                       full-native key: native-dual-direct\n"
          << "                       native-dual-structural-dse,\n"
          << "                       native-dual-exact-dse,\n"
          << "                       native-dual-certified-dse\n"
          << "  --repeat N           repetitions per case/algorithm\n"
          << "  --time-limit SEC     supported backend wall limit\n"
          << "  --max-iterations N   iterative algorithm limit\n"
          << "  --warm-cohort        S5 warm node-reopt cohort (native kernel)\n"
          << "  --warm-branch-vars N vars to tighten per node (default 4)\n"
          << "  --warm-branch-frac F tighten to F*x* toward lb (default 0.5)\n"
          << "  --csv PATH           raw result CSV\n"
          << "  --json PATH          raw and summary JSON\n";
      return false;
    } else {
      std::cerr << "Unknown option: " << arg << "\n";
      return false;
    }
  }
  return true;
}

std::map<std::string, double> read_references(const fs::path& manifest) {
  std::ifstream in(manifest);
  if (!in) throw std::runtime_error("cannot open " + manifest.string());
  std::map<std::string, double> refs;
  std::string line;
  std::getline(in, line);
  while (std::getline(in, line)) {
    const auto fields = split(line, ',');
    if (fields.size() >= 8 && fields[0] == "netlib") {
      refs[fields[1]] = std::stod(fields[6]);
    }
  }
  return refs;
}

bool read_mps_as_lp(const fs::path& path, eng::LPModel& lp, std::string& error) {
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
  std::vector<HighsInt> a_start, a_index, q_start, q_index;
  std::vector<double> a_value, col_cost, col_lower, col_upper;
  std::vector<double> row_lower, row_upper, q_value;
  std::vector<HighsVarType> integer;
  std::string objective_name;
  std::vector<std::string> col_names, row_names;
  HighsInt q_dim = 0, cost_row_location = 0;
  bool warning_issued = false;
  const FilereaderRetcode rc = readMps(
      log_options, path.string(), -1, -1, num_row, num_col, sense, offset,
      a_start, a_index, a_value, col_cost, col_lower, col_upper, row_lower,
      row_upper, integer, objective_name, col_names, row_names, q_dim, q_start,
      q_index, q_value, cost_row_location, warning_issued);
  if (rc != FilereaderRetcode::kOk || q_dim != 0) {
    error = rc != FilereaderRetcode::kOk ? "MPS parse failed" : "quadratic MPS";
    return false;
  }

  lp = {};
  lp.sense = sense == ObjSense::kMaximize ? eng::Sense::Maximize
                                          : eng::Sense::Minimize;
  const int n = static_cast<int>(num_col);
  const int m = static_cast<int>(num_row);
  lp.c = Eigen::VectorXd::Map(col_cost.data(), n);
  lp.vars.resize(static_cast<std::size_t>(n));
  for (int j = 0; j < n; ++j) {
    auto& var = lp.vars[static_cast<std::size_t>(j)];
    var.type = eng::VarType::Continuous;
    var.lb = std::isfinite(col_lower[static_cast<std::size_t>(j)])
                 ? col_lower[static_cast<std::size_t>(j)] : -1e20;
    var.ub = std::isfinite(col_upper[static_cast<std::size_t>(j)])
                 ? col_upper[static_cast<std::size_t>(j)] : 1e20;
    if (j < static_cast<int>(col_names.size())) var.name = col_names[j];
  }

  std::vector<int> ineq_row(static_cast<std::size_t>(m), -1);
  std::vector<int> eq_row(static_cast<std::size_t>(m), -1);
  int n_ineq = 0, n_eq = 0;
  for (int i = 0; i < m; ++i) {
    if (row_lower[static_cast<std::size_t>(i)] ==
        row_upper[static_cast<std::size_t>(i)]) {
      eq_row[static_cast<std::size_t>(i)] = n_eq++;
    } else {
      ineq_row[static_cast<std::size_t>(i)] = n_ineq++;
    }
  }
  std::vector<Eigen::Triplet<double>> ineq_trips, eq_trips;
  std::vector<double> upper(static_cast<std::size_t>(n_ineq));
  std::vector<double> lower(static_cast<std::size_t>(n_ineq));
  std::vector<double> equal(static_cast<std::size_t>(n_eq));
  for (int i = 0; i < m; ++i) {
    if (eq_row[static_cast<std::size_t>(i)] >= 0) {
      equal[static_cast<std::size_t>(eq_row[static_cast<std::size_t>(i)])] =
          row_lower[static_cast<std::size_t>(i)];
    } else {
      const int r = ineq_row[static_cast<std::size_t>(i)];
      lower[static_cast<std::size_t>(r)] = row_lower[static_cast<std::size_t>(i)];
      upper[static_cast<std::size_t>(r)] = row_upper[static_cast<std::size_t>(i)];
    }
  }
  for (int j = 0; j < n; ++j) {
    for (HighsInt p = a_start[j]; p < a_start[j + 1]; ++p) {
      const int i = static_cast<int>(a_index[static_cast<std::size_t>(p)]);
      const double v = a_value[static_cast<std::size_t>(p)];
      if (eq_row[static_cast<std::size_t>(i)] >= 0) {
        eq_trips.emplace_back(eq_row[static_cast<std::size_t>(i)], j, v);
      } else {
        ineq_trips.emplace_back(ineq_row[static_cast<std::size_t>(i)], j, v);
      }
    }
  }
  lp.A.resize(n_ineq, n);
  lp.A.setFromTriplets(ineq_trips.begin(), ineq_trips.end());
  lp.row_lhs = Eigen::VectorXd::Map(lower.data(), n_ineq);
  lp.b = Eigen::VectorXd::Map(upper.data(), n_ineq);
  lp.Aeq.resize(n_eq, n);
  lp.Aeq.setFromTriplets(eq_trips.begin(), eq_trips.end());
  lp.beq = Eigen::VectorXd::Map(equal.data(), n_eq);
  return true;
}

std::vector<CaseInfo> load_cases(const Config& cfg) {
  const auto refs = read_references(cfg.data_dir / "mps_manifest.csv");
  std::vector<CaseInfo> cases;
  for (const auto& [name, reference] : refs) {
    if (!cfg.case_filter.empty() && name.find(cfg.case_filter) == std::string::npos)
      continue;
    if (!cfg.case_names.empty() &&
        std::find(cfg.case_names.begin(), cfg.case_names.end(), name) ==
            cfg.case_names.end())
      continue;
    CaseInfo info;
    info.name = name;
    info.path = cfg.data_dir / "netlib" / (name + ".mps");
    info.reference_objective = reference;
    const auto t0 = std::chrono::steady_clock::now();
    std::string error;
    if (!read_mps_as_lp(info.path, info.lp, error)) {
      throw std::runtime_error(info.path.string() + ": " + error);
    }
    info.load_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t0).count();
    cases.push_back(std::move(info));
  }
  return cases;
}

double rhs_scale(const eng::LPModel& lp) {
  double scale = 1.0;
  for (int i = 0; i < lp.b.size(); ++i)
    if (std::isfinite(lp.b[i])) scale = std::max(scale, std::abs(lp.b[i]));
  for (int i = 0; i < lp.row_lhs.size(); ++i)
    if (std::isfinite(lp.row_lhs[i]))
      scale = std::max(scale, std::abs(lp.row_lhs[i]));
  for (int i = 0; i < lp.beq.size(); ++i)
    scale = std::max(scale, std::abs(lp.beq[i]));
  return scale;
}

void audit(const eng::LPModel& lp, RunResult& row) {
  if (row.x.size() != lp.c.size() || !row.x.allFinite()) return;
  row.objective = lp.c.dot(row.x);
  row.objective_rel_error = std::abs(row.objective - row.reference_objective) /
                            std::max(1.0, std::abs(row.reference_objective));
  row.max_row_violation = 0.0;
  if (lp.A.rows() > 0) {
    const Eigen::VectorXd ax = lp.A * row.x;
    for (int i = 0; i < ax.size(); ++i) {
      if (std::isfinite(lp.b[i]))
        row.max_row_violation = std::max(row.max_row_violation, ax[i] - lp.b[i]);
      const double lhs = eng::lp_row_lhs_or_neg_inf(lp, i);
      if (std::isfinite(lhs))
        row.max_row_violation = std::max(row.max_row_violation, lhs - ax[i]);
    }
  }
  if (lp.Aeq.rows() > 0) {
    row.max_row_violation = std::max(
        row.max_row_violation, (lp.Aeq * row.x - lp.beq).lpNorm<Eigen::Infinity>());
  }
  row.max_bound_violation = 0.0;
  for (int j = 0; j < row.x.size(); ++j) {
    const auto& v = lp.vars[static_cast<std::size_t>(j)];
    row.max_bound_violation = std::max(
        row.max_bound_violation,
        std::max(std::max(0.0, v.lb - row.x[j]), std::max(0.0, row.x[j] - v.ub)));
  }
  row.normalized_primal_violation =
      std::max(row.max_row_violation, row.max_bound_violation) / rhs_scale(lp);
  row.accurate = row.success && row.objective_rel_error <= kObjectiveTolerance &&
                 row.normalized_primal_violation <= kFeasibilityTolerance;
}

eng::NLPModel lp_to_nlp(const eng::LPModel& lp, int max_iterations) {
  eng::NLPModel nlp;
  nlp.sense = lp.sense;
  nlp.vars = lp.vars;
  nlp.solver_options.max_iterations = std::min(max_iterations, 2000);
  nlp.solver_options.tolerance = 1e-8;
  nlp.solver_options.acceptable_tolerance = 1e-6;
  nlp.x0 = Eigen::VectorXd::Zero(lp.c.size());
  for (int j = 0; j < nlp.x0.size(); ++j) {
    const auto& v = lp.vars[static_cast<std::size_t>(j)];
    if (v.lb > 0.0 && v.lb < 1e19) nlp.x0[j] = v.lb;
    if (v.ub < 0.0 && v.ub > -1e19) nlp.x0[j] = v.ub;
  }
  const Eigen::VectorXd cost = lp.sense == eng::Sense::Minimize ? lp.c : -lp.c;
  nlp.f = [cost](const Eigen::VectorXd& x) { return cost.dot(x); };
  nlp.grad = [cost](const Eigen::VectorXd&, Eigen::VectorXd& out) { out = cost; };

  const Eigen::SparseMatrix<double> aeq = lp.Aeq;
  const Eigen::VectorXd beq = lp.beq;
  if (aeq.rows() > 0) {
    nlp.g = [aeq, beq](const Eigen::VectorXd& x, Eigen::VectorXd& out) {
      out = aeq * x - beq;
    };
    nlp.jac_g = [aeq](const Eigen::VectorXd&, Eigen::SparseMatrix<double>& out) {
      out = aeq;
    };
  }

  std::vector<int> upper_row(static_cast<std::size_t>(lp.A.rows()), -1);
  std::vector<int> lower_row(static_cast<std::size_t>(lp.A.rows()), -1);
  int h_rows = 0;
  for (int i = 0; i < lp.A.rows(); ++i) {
    if (std::isfinite(lp.b[i])) upper_row[static_cast<std::size_t>(i)] = h_rows++;
    if (std::isfinite(eng::lp_row_lhs_or_neg_inf(lp, i)))
      lower_row[static_cast<std::size_t>(i)] = h_rows++;
  }
  Eigen::SparseMatrix<double> hmat(h_rows, lp.A.cols());
  Eigen::VectorXd hrhs(h_rows);
  std::vector<Eigen::Triplet<double>> trips;
  trips.reserve(static_cast<std::size_t>(2 * lp.A.nonZeros()));
  for (int j = 0; j < lp.A.outerSize(); ++j) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(lp.A, j); it; ++it) {
      const int ur = upper_row[static_cast<std::size_t>(it.row())];
      const int lr = lower_row[static_cast<std::size_t>(it.row())];
      if (ur >= 0) trips.emplace_back(ur, it.col(), it.value());
      if (lr >= 0) trips.emplace_back(lr, it.col(), -it.value());
    }
  }
  for (int i = 0; i < lp.A.rows(); ++i) {
    const int ur = upper_row[static_cast<std::size_t>(i)];
    const int lr = lower_row[static_cast<std::size_t>(i)];
    if (ur >= 0) hrhs[ur] = lp.b[i];
    if (lr >= 0) hrhs[lr] = -eng::lp_row_lhs_or_neg_inf(lp, i);
  }
  hmat.setFromTriplets(trips.begin(), trips.end());
  if (h_rows > 0) {
    nlp.h = [hmat, hrhs](const Eigen::VectorXd& x, Eigen::VectorXd& out) {
      out = hmat * x - hrhs;
    };
    nlp.jac_h = [hmat](const Eigen::VectorXd&, Eigen::SparseMatrix<double>& out) {
      out = hmat;
    };
  }
  return nlp;
}

RunResult run_highs(const CaseInfo& kase, const std::string& algorithm,
                    double time_limit_sec) {
  RunResult row;
  row.solver = "HiGHS-" + algorithm;
  const auto t0 = std::chrono::steady_clock::now();
  Highs::resetGlobalScheduler(true);
  Highs highs;
  if (std::getenv("MIPSOLVERS_HIGHS_TIMER") != nullptr) {
    // Emit HiGHS' own per-operation simplex timer report (kHighsAnalysisLevel
    // SolverTime = 8) for a direct comparison against native's [DS-PROFILE].
    highs.setOptionValue("output_flag", true);
    highs.setOptionValue("log_to_console", true);
    highs.setOptionValue("highs_analysis_level", 8);
  } else {
    highs.setOptionValue("output_flag", false);
    highs.setOptionValue("log_to_console", false);
  }
  highs.setOptionValue("threads", static_cast<HighsInt>(1));
  highs.setOptionValue("time_limit", time_limit_sec);
  highs.setOptionValue("solver", algorithm);
  if (algorithm == "ipm") highs.setOptionValue("run_crossover", "off");
  const eng::LPModel& lp = kase.lp;
  const int ncols = static_cast<int>(lp.vars.size());
  const int nineq = lp.A.rows();
  const int nrows = nineq + lp.Aeq.rows();
  std::vector<double> col_cost(static_cast<std::size_t>(ncols));
  std::vector<double> col_lower(static_cast<std::size_t>(ncols));
  std::vector<double> col_upper(static_cast<std::size_t>(ncols));
  for (int j = 0; j < ncols; ++j) {
    col_cost[static_cast<std::size_t>(j)] = lp.c[j];
    col_lower[static_cast<std::size_t>(j)] = lp.vars[static_cast<std::size_t>(j)].lb;
    col_upper[static_cast<std::size_t>(j)] = lp.vars[static_cast<std::size_t>(j)].ub;
  }
  std::vector<double> row_lower(static_cast<std::size_t>(nrows), -kHighsInf);
  std::vector<double> row_upper(static_cast<std::size_t>(nrows), kHighsInf);
  for (int i = 0; i < nineq; ++i) {
    row_lower[static_cast<std::size_t>(i)] = eng::lp_row_lhs_or_neg_inf(lp, i);
    row_upper[static_cast<std::size_t>(i)] = lp.b[i];
  }
  for (int i = 0; i < lp.Aeq.rows(); ++i) {
    row_lower[static_cast<std::size_t>(nineq + i)] = lp.beq[i];
    row_upper[static_cast<std::size_t>(nineq + i)] = lp.beq[i];
  }
  std::vector<HighsInt> start(static_cast<std::size_t>(ncols + 1), 0);
  std::vector<HighsInt> index;
  std::vector<double> value;
  index.reserve(static_cast<std::size_t>(lp.A.nonZeros() + lp.Aeq.nonZeros()));
  value.reserve(index.capacity());
  for (int j = 0; j < ncols; ++j) {
    start[static_cast<std::size_t>(j)] = static_cast<HighsInt>(index.size());
    for (Eigen::SparseMatrix<double>::InnerIterator it(lp.A, j); it; ++it) {
      index.push_back(static_cast<HighsInt>(it.row()));
      value.push_back(it.value());
    }
    for (Eigen::SparseMatrix<double>::InnerIterator it(lp.Aeq, j); it; ++it) {
      index.push_back(static_cast<HighsInt>(nineq + it.row()));
      value.push_back(it.value());
    }
  }
  start[static_cast<std::size_t>(ncols)] = static_cast<HighsInt>(index.size());
  const HighsStatus pass_status = highs.passModel(
      ncols, nrows, static_cast<HighsInt>(index.size()),
      static_cast<HighsInt>(MatrixFormat::kColwise),
      static_cast<HighsInt>(lp.sense == eng::Sense::Maximize
                                ? ObjSense::kMaximize : ObjSense::kMinimize),
      0.0, col_cost.data(), col_lower.data(), col_upper.data(), row_lower.data(),
      row_upper.data(), start.data(), index.data(), value.data(), nullptr);
  if (pass_status == HighsStatus::kError) {
    row.success = false;
    row.status = "passModel failed";
  } else {
    const HighsStatus run_status = highs.run();
    const HighsModelStatus status = highs.getModelStatus();
    row.success = run_status != HighsStatus::kError &&
                  status == HighsModelStatus::kOptimal;
    row.status = highs.modelStatusToString(status);
    const HighsInfo& info = highs.getInfo();
    row.iterations = static_cast<int>(info.simplex_iteration_count +
                                      info.ipm_iteration_count +
                                      info.pdlp_iteration_count);
    const auto& values = highs.getSolution().col_value;
    if (static_cast<int>(values.size()) == kase.lp.c.size())
      row.x = Eigen::VectorXd::Map(values.data(), static_cast<int>(values.size()));
  }
  row.runtime_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - t0).count();
  return row;
}

RunResult run_scip_direct(const CaseInfo& kase, double time_limit_sec) {
  RunResult row;
  row.solver = "SCIP-direct-MPS";
#ifdef HACDCPF_HAVE_SCIP_LIB
  const auto t0 = std::chrono::steady_clock::now();
  SCIP* scip = nullptr;
  SCIP_RETCODE rc = SCIPcreate(&scip);
  if (rc != SCIP_OKAY || scip == nullptr) {
    row.status = "SCIPcreate failed";
  } else {
    SCIPincludeDefaultPlugins(scip);
    SCIPsetIntParam(scip, "display/verblevel", 0);
    SCIPsetRealParam(scip, "limits/time", time_limit_sec);
    rc = SCIPreadProb(scip, kase.path.string().c_str(), nullptr);
    if (rc != SCIP_OKAY) {
      row.status = "SCIPreadProb failed";
    } else {
      rc = SCIPsolve(scip);
      const SCIP_STATUS status = SCIPgetStatus(scip);
      SCIP_SOL* sol = SCIPgetBestSol(scip);
      row.success = rc == SCIP_OKAY && status == SCIP_STATUS_OPTIMAL && sol != nullptr;
      switch (status) {
        case SCIP_STATUS_OPTIMAL: row.status = "Optimal"; break;
        case SCIP_STATUS_TIMELIMIT: row.status = "Time limit"; break;
        case SCIP_STATUS_INFEASIBLE: row.status = "Infeasible"; break;
        case SCIP_STATUS_UNBOUNDED: row.status = "Unbounded"; break;
        default: row.status = "SCIP status " + std::to_string(static_cast<int>(status));
      }
      if (sol != nullptr) {
        std::unordered_map<std::string, int> name_to_column;
        for (int j = 0; j < static_cast<int>(kase.lp.vars.size()); ++j)
          name_to_column[kase.lp.vars[static_cast<std::size_t>(j)].name] = j;
        row.x = Eigen::VectorXd::Zero(kase.lp.c.size());
        SCIP_VAR** vars = SCIPgetOrigVars(scip);
        const int nvars = SCIPgetNOrigVars(scip);
        for (int j = 0; j < nvars; ++j) {
          const char* name = SCIPvarGetName(vars[j]);
          const auto it = name == nullptr ? name_to_column.end()
                                          : name_to_column.find(name);
          if (it != name_to_column.end())
            row.x[it->second] = SCIPgetSolVal(scip, sol, vars[j]);
        }
      }
    }
    SCIPfree(&scip);
  }
  row.runtime_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - t0).count();
#else
  (void)kase;
  (void)time_limit_sec;
  row.available = false;
  row.status = "embedded SCIP unavailable";
#endif
  return row;
}

RunResult run_adapter(const CaseInfo& kase, const std::string& solver,
                      const Config& cfg) {
  RunResult row;
  row.solver = solver;
  eng::SolveResult result;
  const auto t0 = std::chrono::steady_clock::now();
  if (solver == "native-dual-simplex" || solver == "native-dual-direct" ||
      solver == "native-dual-devex" ||
      solver == "native-dual-structural-dse" ||
      solver == "native-dual-exact-dse" ||
      solver == "native-dual-certified-dse") {
    eng::SimplexOptions opt;
    opt.lp_kernel_backend = eng::LpKernelBackend::ExperimentalNative;
    opt.max_iter = cfg.max_iterations;
    opt.time_limit_sec = cfg.time_limit_sec;
    opt.use_highs_presolve = solver != "native-dual-direct";
    if (solver == "native-dual-devex") {
      opt.dual_edge_weight_initialization =
          eng::DualEdgeWeightInitialization::Devex;
    } else if (solver == "native-dual-structural-dse") {
      opt.dual_edge_weight_initialization =
          eng::DualEdgeWeightInitialization::StructuralExact;
    } else if (solver == "native-dual-exact-dse") {
      opt.dual_edge_weight_initialization =
          eng::DualEdgeWeightInitialization::FullExact;
    } else if (solver == "native-dual-certified-dse") {
      opt.dual_edge_weight_initialization =
          eng::DualEdgeWeightInitialization::CertifiedExact;
    }
    result = eng::solve_lp_with_basis(kase.lp, opt).result;
    if (solver == "native-dual-direct") {
      row.solver = "Native-DualSimplex[direct]";
    } else if (solver == "native-dual-devex") {
      row.solver = "Native-DualSimplex[Devex](+HiGHS-presolve)";
    } else if (solver == "native-dual-structural-dse") {
      row.solver = "Native-DualSimplex[StructuralDSE](+HiGHS-presolve)";
    } else if (solver == "native-dual-exact-dse") {
      row.solver = "Native-DualSimplex[ExactDSE](+HiGHS-presolve)";
    } else if (solver == "native-dual-certified-dse") {
      row.solver = "Native-DualSimplex[CertifiedDSE](+HiGHS-presolve)";
    } else {
      row.solver = "Native-DualSimplex(+HiGHS-presolve)";
    }
  } else if (solver == "native-ipm" || solver == "native-ipm-direct" ||
             solver == "native-ipm-legacy-step") {
    eng::IPMLPOptions opt;
    opt.max_iter = std::min(cfg.max_iterations, 2000);
    opt.time_limit_sec = cfg.time_limit_sec;
    // P1: native-ipm opts into the native presolve explicitly
    // (native_presolve_lp_2026-08-18.md §4); native-ipm-direct stays
    // presolve-free as the h8-comparable control arm. Env
    // MIPSOLVERS_NATIVE_PRESOLVE=0 force-disables per run.
    opt.presolve = solver != "native-ipm-direct";
    opt.use_highs_presolve = solver != "native-ipm-direct";
    opt.centrality_step_control = solver != "native-ipm-legacy-step";
    result = eng::NativeIPMLPAdapter(opt).solve_lp(kase.lp);
    if (!opt.use_highs_presolve) {
      row.solver = "Native-IPM[centrality-step,direct]";
    } else {
      row.solver = opt.centrality_step_control
                       ? "Native-IPM[centrality-step](+native-presolve)"
                       : "Native-IPM[legacy-step](+native-presolve)";
    }
  } else if (solver == "native-auto") {
    eng::NativeAutoLPAdapter adapter(cfg.time_limit_sec);
    result = adapter.solve_lp(kase.lp);
    row.solver = "Native-Auto[selector]";
  } else if (solver == "native-pdlp") {
    eng::PDLPOptions opt;
    opt.max_iter = cfg.max_iterations;
    result = eng::NativePDLPAdapter(opt).solve_lp(kase.lp);
    row.solver = "Native-PDLP";
  } else if (solver == "native-lcqp") {
    eng::LCQPOptions opt;
    opt.max_iter = std::min(cfg.max_iterations, 2000);
    result = eng::NativeLCQPAdapter(opt).solve_lp(kase.lp);
    row.solver = "Native-LCQP";
  } else if (solver == "scip") {
    eng::ScipAdapter adapter;
    row.available = adapter.available();
    if (row.available) {
      eng::MIPModel mip;
      mip.linear_part = kase.lp;
      result = adapter.solve_milp(mip);
    } else {
      row.status = "SCIP unavailable";
    }
    row.solver = "SCIP-LP(adapter)";
  } else if (solver == "ipopt") {
    eng::IpoptAdapter adapter;
    row.available = adapter.available();
    if (row.available) result = adapter.solve_nlp(lp_to_nlp(kase.lp, cfg.max_iterations));
    else row.status = "Ipopt unavailable";
    row.solver = "Ipopt-LP-as-NLP";
  } else {
    row.available = false;
    row.status = "unknown solver key";
  }
  row.runtime_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - t0).count();
  if (row.available) {
    row.success = result.stats.success;
    row.status = result.stats.status;
    row.iterations = result.stats.iterations;
    row.native_telemetry = result.stats.native_dual_kernel_time_sec > 0.0;
    row.dual_pivots = result.stats.dual_phase_one_iterations +
                      result.stats.dual_phase_two_iterations;
    row.dse_initialization_solves = result.stats.dse_initialization_solves;
    row.certified_dse_btrans = result.stats.certified_dse_btrans;
    row.certified_dse_candidates = result.stats.certified_dse_candidates;
    row.certified_dse_rejections = result.stats.certified_dse_rejections;
    row.dse_initialization_ms =
        1000.0 * result.stats.dse_initialization_time_sec;
    row.certified_dse_ms = 1000.0 * result.stats.certified_dse_time_sec;
    row.native_kernel_ms = 1000.0 * result.stats.native_dual_kernel_time_sec;
    row.relative_primal_residual = result.stats.relative_primal_residual;
    row.relative_dual_residual = result.stats.relative_dual_residual;
    row.relative_gap = result.stats.relative_gap;
    row.dual_objective = result.stats.dual_objective;
    row.presolve_ms = result.stats.presolve_ms;
    row.presolve_orig_rows = result.stats.presolve_orig_rows;
    row.presolve_orig_cols = result.stats.presolve_orig_cols;
    row.presolve_orig_nnz = result.stats.presolve_orig_nnz;
    row.presolve_reduced_rows = result.stats.presolve_reduced_rows;
    row.presolve_reduced_cols = result.stats.presolve_reduced_cols;
    row.presolve_reduced_nnz = result.stats.presolve_reduced_nnz;
    row.presolve_used = result.stats.presolve_used;
    if (row.dual_pivots > 0) {
      row.native_kernel_ms_per_dual_pivot =
          std::max(0.0, row.native_kernel_ms - row.dse_initialization_ms) /
          row.dual_pivots;
    }
    row.x = std::move(result.x);
  }
  return row;
}

RunResult run_one(const CaseInfo& kase, const std::string& solver,
                  const Config& cfg, int repeat) {
  RunResult row;
  if (solver == "highs-simplex") row = run_highs(kase, "simplex", cfg.time_limit_sec);
  else if (solver == "highs-ipm") row = run_highs(kase, "ipm", cfg.time_limit_sec);
  else if (solver == "highs-pdlp") row = run_highs(kase, "pdlp", cfg.time_limit_sec);
  else if (solver == "scip-direct") row = run_scip_direct(kase, cfg.time_limit_sec);
  else row = run_adapter(kase, solver, cfg);
  row.case_name = kase.name;
  row.repeat = repeat;
  row.rows = kase.lp.A.rows() + kase.lp.Aeq.rows();
  row.columns = kase.lp.c.size();
  row.nonzeros = kase.lp.A.nonZeros() + kase.lp.Aeq.nonZeros();
  row.reference_objective = kase.reference_objective;
  audit(kase.lp, row);
  return row;
}

double median(std::vector<double> values) {
  if (values.empty()) return 0.0;
  std::sort(values.begin(), values.end());
  const std::size_t n = values.size();
  return n % 2 ? values[n / 2] : 0.5 * (values[n / 2 - 1] + values[n / 2]);
}

std::vector<Summary> summarize(const std::vector<RunResult>& rows) {
  std::map<std::string, std::vector<const RunResult*>> groups;
  for (const auto& row : rows) groups[row.solver].push_back(&row);
  std::map<std::tuple<std::string, int>, double> baseline;
  for (const auto& row : rows)
    if (row.solver == "HiGHS-simplex" && row.accurate)
      baseline[{row.case_name, row.repeat}] = row.runtime_ms;
  std::vector<Summary> out;
  for (const auto& [solver, members] : groups) {
    Summary s;
    s.solver = solver;
    std::vector<double> times;
    std::vector<double> speedups;
    std::vector<double> kernel_ms_per_pivot;
    double log_sum = 0.0;
    for (const RunResult* row : members) {
      ++s.attempts;
      s.available += row->available ? 1 : 0;
      s.successes += row->success ? 1 : 0;
      s.accurate += row->accurate ? 1 : 0;
      s.total_sec += row->runtime_ms / 1000.0;
      if (row->available) {
        times.push_back(row->runtime_ms);
        log_sum += std::log(std::max(1e-6, row->runtime_ms));
      }
      if (row->native_telemetry) {
        ++s.native_telemetry_count;
        s.mean_dual_pivots += row->dual_pivots;
        s.mean_dse_initialization_ms += row->dse_initialization_ms;
        if (row->dual_pivots > 0) {
          kernel_ms_per_pivot.push_back(
              row->native_kernel_ms_per_dual_pivot);
        }
      }
      const auto it = baseline.find({row->case_name, row->repeat});
      if (row->accurate && it != baseline.end())
        speedups.push_back(it->second / std::max(1e-6, row->runtime_ms));
    }
    s.median_ms = median(times);
    s.geometric_mean_ms = times.empty()
                             ? 0.0
                             : std::exp(log_sum / times.size());
    if (s.native_telemetry_count > 0) {
      s.mean_dual_pivots /= s.native_telemetry_count;
      s.mean_dse_initialization_ms /= s.native_telemetry_count;
    }
    if (!kernel_ms_per_pivot.empty()) {
      double kernel_log_sum = 0.0;
      for (double value : kernel_ms_per_pivot) {
        kernel_log_sum += std::log(std::max(1e-9, value));
      }
      s.geometric_mean_kernel_ms_per_dual_pivot =
          std::exp(kernel_log_sum / kernel_ms_per_pivot.size());
    }
    if (!speedups.empty()) {
      double log_speedup = 0.0;
      for (double value : speedups) log_speedup += std::log(value);
      s.geometric_speedup_vs_highs_simplex =
          std::exp(log_speedup / speedups.size());
    }
    out.push_back(std::move(s));
  }
  return out;
}

json finite_or_null(double value) {
  return std::isfinite(value) ? json(value) : json(nullptr);
}

// S1 reproducibility provenance embedded in every report so a result is tied to
// the exact binary, build, host architecture and invocation that produced it.
// git_commit is best-effort (null when git is unavailable or out of tree).
json build_provenance(const std::string& command_line) {
  std::string git_commit;
  // Deterministic override first (CI / sandboxed runs where a git subprocess is
  // unavailable), then a best-effort git query.
  if (const char* env = std::getenv("MIPSOLVERS_BENCH_GIT_COMMIT")) {
    git_commit = env;
  }
#if defined(__unix__) || defined(__APPLE__)
  if (git_commit.empty()) {
    if (FILE* pipe = ::popen("git rev-parse HEAD 2>/dev/null", "r")) {
      char buffer[128];
      while (std::fgets(buffer, sizeof(buffer), pipe) != nullptr) {
        git_commit += buffer;
      }
      ::pclose(pipe);
    }
  }
#endif
  while (!git_commit.empty() &&
         (git_commit.back() == '\n' || git_commit.back() == '\r' ||
          git_commit.back() == ' ')) {
    git_commit.pop_back();
  }

  char timestamp[32] = {0};
  const std::time_t now = std::time(nullptr);
  if (const std::tm* utc = std::gmtime(&now)) {
    std::strftime(timestamp, sizeof(timestamp), "%Y-%m-%dT%H:%M:%SZ", utc);
  }

#if defined(__clang__)
  const std::string compiler = std::string("clang ") + __clang_version__;
#elif defined(__GNUC__)
  const std::string compiler = std::string("gcc ") + __VERSION__;
#elif defined(_MSC_VER)
  const std::string compiler = "msvc " + std::to_string(_MSC_VER);
#else
  const std::string compiler = "unknown";
#endif

#ifdef NDEBUG
  const char* build_type = "Release/NDEBUG";
#else
  const char* build_type = "Debug";
#endif

#if defined(__aarch64__) || defined(_M_ARM64)
  const char* target_arch = "arm64";
#elif defined(__x86_64__) || defined(_M_X64)
  const char* target_arch = "x86_64";
#else
  const char* target_arch = "unknown";
#endif

  json provenance;
  provenance["schema_version"] = kSchemaVersion;
  provenance["generated_utc"] = timestamp;
  provenance["git_commit"] =
      git_commit.empty() ? json(nullptr) : json(git_commit);
  provenance["build_type"] = build_type;
  provenance["compiler"] = compiler;
  provenance["target_arch"] = target_arch;
  provenance["command"] = command_line;
  return provenance;
}

void write_csv(const fs::path& path, const std::vector<RunResult>& rows) {
  if (path.empty()) return;
  if (!path.parent_path().empty()) fs::create_directories(path.parent_path());
  std::ofstream out(path);
  out << "case,solver,repeat,rows,columns,nonzeros,available,success,accurate,"
         "runtime_ms,iterations,dual_pivots,dse_initialization_solves,"
         "dse_initialization_ms,certified_dse_btrans,"
         "certified_dse_candidates,certified_dse_rejections,"
         "certified_dse_ms,native_kernel_ms,"
         "native_kernel_ms_per_dual_pivot,objective,reference_objective,objective_rel_error,"
         "max_row_violation,max_bound_violation,normalized_primal_violation,"
         "relative_primal_residual,relative_dual_residual,relative_gap,"
         "dual_objective,status\n";
  out << std::setprecision(17);
  for (const auto& r : rows) {
    out << r.case_name << ',' << csv_escape(r.solver) << ',' << r.repeat << ','
        << r.rows << ',' << r.columns << ',' << r.nonzeros << ',' << r.available
        << ',' << r.success << ',' << r.accurate << ',' << r.runtime_ms << ','
        << r.iterations << ',' << r.dual_pivots << ','
        << r.dse_initialization_solves << ',' << r.dse_initialization_ms << ','
        << r.certified_dse_btrans << ',' << r.certified_dse_candidates << ','
        << r.certified_dse_rejections << ',' << r.certified_dse_ms << ','
        << r.native_kernel_ms << ',' << r.native_kernel_ms_per_dual_pivot << ','
        << r.objective << ',' << r.reference_objective
        << ',' << r.objective_rel_error << ',' << r.max_row_violation << ','
        << r.max_bound_violation << ',' << r.normalized_primal_violation << ','
        << r.relative_primal_residual << ',' << r.relative_dual_residual << ','
        << r.relative_gap << ',' << r.dual_objective << ','
        << csv_escape(r.status) << '\n';
  }
}

void write_json(const fs::path& path, const Config& cfg,
                const std::vector<CaseInfo>& cases,
                const std::vector<RunResult>& rows,
                const std::vector<Summary>& summaries,
                const std::string& command_line) {
  if (path.empty()) return;
  if (!path.parent_path().empty()) fs::create_directories(path.parent_path());
  json root;
  root["provenance"] = build_provenance(command_line);
  root["configuration"] = {{"data_dir", cfg.data_dir.string()},
                           {"repeats", cfg.repeats},
                           {"time_limit_sec", cfg.time_limit_sec},
                           {"max_iterations", cfg.max_iterations},
                           {"objective_tolerance", kObjectiveTolerance},
                           {"normalized_feasibility_tolerance", kFeasibilityTolerance},
                           {"single_threaded_highs", true}};
  for (const auto& c : cases) {
    root["cases"].push_back({{"name", c.name}, {"path", c.path.string()},
                              {"load_ms", c.load_ms}});
  }
  for (const auto& r : rows) {
    root["runs"].push_back({
        {"case", r.case_name}, {"solver", r.solver}, {"repeat", r.repeat},
        {"available", r.available}, {"success", r.success},
        {"accurate", r.accurate}, {"runtime_ms", r.runtime_ms},
        {"iterations", r.iterations}, {"dual_pivots", r.dual_pivots},
        {"dse_initialization_solves", r.dse_initialization_solves},
        {"dse_initialization_ms", r.dse_initialization_ms},
        {"certified_dse_btrans", r.certified_dse_btrans},
        {"certified_dse_candidates", r.certified_dse_candidates},
        {"certified_dse_rejections", r.certified_dse_rejections},
        {"certified_dse_ms", r.certified_dse_ms},
        {"native_kernel_ms", r.native_kernel_ms},
        {"native_kernel_ms_per_dual_pivot",
         r.native_kernel_ms_per_dual_pivot},
        {"objective", finite_or_null(r.objective)},
        {"reference_objective", r.reference_objective},
        {"objective_rel_error", finite_or_null(r.objective_rel_error)},
        {"normalized_primal_violation", finite_or_null(r.normalized_primal_violation)},
        {"relative_primal_residual", finite_or_null(r.relative_primal_residual)},
        {"relative_dual_residual", finite_or_null(r.relative_dual_residual)},
        {"relative_gap", finite_or_null(r.relative_gap)},
        {"dual_objective", finite_or_null(r.dual_objective)},
        {"presolve_ms", r.presolve_ms},
        {"presolve_orig_rows", r.presolve_orig_rows},
        {"presolve_orig_cols", r.presolve_orig_cols},
        {"presolve_orig_nnz", r.presolve_orig_nnz},
        {"presolve_reduced_rows", r.presolve_reduced_rows},
        {"presolve_reduced_cols", r.presolve_reduced_cols},
        {"presolve_reduced_nnz", r.presolve_reduced_nnz},
        {"presolve_used", r.presolve_used},
        {"status", r.status}});
  }
  for (const auto& s : summaries) {
    root["summary"].push_back({
        {"solver", s.solver}, {"attempts", s.attempts},
        {"available", s.available},
        {"successes", s.successes}, {"accurate", s.accurate},
        {"total_sec", s.total_sec}, {"median_ms", s.median_ms},
        {"geometric_mean_ms", s.geometric_mean_ms},
        {"mean_dual_pivots", s.mean_dual_pivots},
        {"mean_dse_initialization_ms", s.mean_dse_initialization_ms},
        {"geometric_mean_kernel_ms_per_dual_pivot",
         s.geometric_mean_kernel_ms_per_dual_pivot},
        {"geometric_speedup_vs_highs_simplex",
         finite_or_null(s.geometric_speedup_vs_highs_simplex)}});
  }
  std::ofstream(path) << std::setw(2) << root << '\n';
}

void print_summary(const std::vector<Summary>& summaries) {
  std::cout << "\nSummary (accuracy requires success, rel.obj<=1e-5, "
               "normalized primal violation<=1e-7)\n";
  std::printf("%-40s %9s %9s %9s %12s %12s %11s\n", "Algorithm", "Available",
              "Success", "Accurate", "Median ms", "GeoMean ms", "vs HiGHS");
  std::printf("%s\n", std::string(110, '-').c_str());
  for (const auto& s : summaries) {
    char speedup[32] = "-";
    if (std::isfinite(s.geometric_speedup_vs_highs_simplex))
      std::snprintf(speedup, sizeof(speedup), "%.3fx",
                    s.geometric_speedup_vs_highs_simplex);
    std::printf("%-40s %4d/%-4d %4d/%-4d %4d/%-4d %12.3f %12.3f %11s\n",
                s.solver.c_str(), s.available, s.attempts, s.successes,
                s.attempts, s.accurate, s.attempts, s.median_ms,
                s.geometric_mean_ms, speedup);
  }
  bool have_native_telemetry = false;
  for (const auto& s : summaries) {
    have_native_telemetry = have_native_telemetry || s.native_telemetry_count > 0;
  }
  if (have_native_telemetry) {
    std::cout << "\nNative DSE initialization diagnostics\n";
    std::printf("%-55s %12s %14s %18s\n", "Algorithm", "Mean pivots",
                "Mean init ms", "Kernel ms/pivot");
    std::printf("%s\n", std::string(103, '-').c_str());
    for (const auto& s : summaries) {
      if (s.native_telemetry_count == 0) continue;
      std::printf("%-55s %12.1f %14.6f %18.6f\n", s.solver.c_str(),
                  s.mean_dual_pivots, s.mean_dse_initialization_ms,
                  s.geometric_mean_kernel_ms_per_dual_pivot);
    }
  }
}

// ---------------------------------------------------------------------------
// S5 warm re-optimization cohort.
//
// Measures the native dual-simplex kernel's warm-start value on a synthetic
// B&C node: solve the root LP cold, tighten a few basic structural variables'
// upper bounds (a branching decision), then re-solve the node twice - cold and
// warm from the root basis + inherited DSE norms. Reports the warm/cold pivot
// and wall ratios. Both node solves share the same StandardFormLP (presolve
// free), so the ratio isolates the warm-start effect.
// ---------------------------------------------------------------------------
struct WarmCohortRow {
  std::string name;
  int rows{0};
  int cols{0};
  int root_pivots{0};
  int cold_pivots{0};
  int warm_pivots{0};
  double cold_ms{0.0};
  double warm_ms{0.0};
  int warm_dse_init{0};
  int branched{0};
  bool usable{false};
  std::string note;
};

double timed_native_solve(const eng::StandardFormLP& sf,
                          const eng::SimplexOptions& options,
                          const eng::SimplexBasis* hint, int repeats,
                          eng::native_dual::Result& out) {
  double best_ms = std::numeric_limits<double>::infinity();
  for (int r = 0; r < std::max(1, repeats); ++r) {
    const auto t0 = std::chrono::steady_clock::now();
    out = eng::native_dual::solve(sf, options, hint);
    const double ms = std::chrono::duration<double, std::milli>(
                          std::chrono::steady_clock::now() - t0)
                          .count();
    best_ms = std::min(best_ms, ms);
  }
  return best_ms;
}

void run_warm_cohort(const Config& cfg, const std::vector<CaseInfo>& cases) {
  eng::SimplexOptions options;
  options.lp_kernel_backend = eng::LpKernelBackend::ExperimentalNative;
  options.max_iter = cfg.max_iterations;
  options.time_limit_sec = cfg.time_limit_sec;

  std::cout << "S5 warm re-optimization cohort (native dual-simplex kernel, "
               "presolve-free)\n"
            << cases.size() << " cases, branch<=" << cfg.warm_branch_vars
            << " vars to " << cfg.warm_branch_frac << "*x*, min of "
            << cfg.repeats << " wall sample(s)\n\n";
  std::printf("%-12s %6s %6s %8s %8s %8s %8s %9s %9s %8s %6s\n", "case", "m",
              "n", "rootPiv", "coldPiv", "warmPiv", "piv x", "coldMs", "warmMs",
              "ms x", "dseIn");
  std::printf("%s\n", std::string(104, '-').c_str());

  std::vector<WarmCohortRow> table;
  double log_piv_sum = 0.0;
  double log_ms_sum = 0.0;
  int usable = 0;
  int mismatches = 0;
  long total_cold_piv = 0;
  long total_warm_piv = 0;
  for (const auto& kase : cases) {
    WarmCohortRow row;
    row.name = kase.name;

    eng::StandardFormLP sf = eng::build_standard_form_lp(kase.lp);
    row.rows = static_cast<int>(sf.A.rows());
    row.cols = static_cast<int>(sf.A.cols());

    eng::native_dual::Result root;
    timed_native_solve(sf, options, nullptr, 1, root);
    if (root.status != eng::native_dual::Status::Optimal) {
      row.note = "root-not-optimal";
      table.push_back(row);
      continue;
    }
    row.root_pivots = root.statistics.iterations;

    // Build a node: tighten up to K basic structural variables toward their
    // lower bound (shifted lb = 0), cutting the root optimum so the node needs
    // a genuine re-optimization.
    eng::StandardFormLP sf_node = sf;
    int branched = 0;
    for (std::size_t pos = 0;
         pos < root.basis.size() && branched < cfg.warm_branch_vars; ++pos) {
      const int col = root.basis[pos];
      if (col < 0 || col >= sf.n_original) continue;  // structural only
      const double xval = (pos < static_cast<std::size_t>(root.x_basic.size()))
                              ? root.x_basic[static_cast<int>(pos)]
                              : 0.0;
      if (!(xval > 1e-6)) continue;  // need room to tighten below x*
      sf_node.var_ub[col] = cfg.warm_branch_frac * xval;
      ++branched;
    }
    row.branched = branched;
    if (branched == 0) {
      row.note = "no-branch-candidate";
      table.push_back(row);
      continue;
    }

    eng::native_dual::Result cold;
    row.cold_ms =
        timed_native_solve(sf_node, options, nullptr, cfg.repeats, cold);
    if (cold.status != eng::native_dual::Status::Optimal) {
      row.note = "node-not-optimal";
      table.push_back(row);
      continue;
    }
    row.cold_pivots = cold.statistics.iterations;

    eng::SimplexBasis hint;
    hint.bound_domain_version = eng::SimplexBasis::kBoundDomainVersion;
    hint.rows = static_cast<int>(sf.A.rows());
    hint.cols = static_cast<int>(sf.A.cols());
    hint.indices = root.basis;
    hint.at_upper = root.at_upper;
    hint.cached_dse_weights =
        std::make_shared<const std::vector<double>>(root.edge_weights);
    hint.cached_dse_basis =
        std::make_shared<const std::vector<int>>(root.basis);

    eng::native_dual::Result warm;
    row.warm_ms =
        timed_native_solve(sf_node, options, &hint, cfg.repeats, warm);
    if (warm.status != eng::native_dual::Status::Optimal) {
      row.note = "warm-not-optimal";
      table.push_back(row);
      continue;
    }
    row.warm_pivots = warm.statistics.iterations;
    row.warm_dse_init = warm.statistics.dse_initialization_solves;

    const double obj_gap = std::abs(warm.max_objective - cold.max_objective);
    if (obj_gap > 1e-6 * (1.0 + std::abs(cold.max_objective))) {
      row.note = "OBJECTIVE-MISMATCH";
      ++mismatches;
      table.push_back(row);
      continue;
    }

    row.usable = true;
    ++usable;
    total_cold_piv += row.cold_pivots;
    total_warm_piv += row.warm_pivots;
    log_piv_sum += std::log((row.warm_pivots + 1.0) / (row.cold_pivots + 1.0));
    if (row.cold_ms > 0.0 && row.warm_ms > 0.0)
      log_ms_sum += std::log(row.warm_ms / row.cold_ms);
    table.push_back(row);
  }

  for (const auto& row : table) {
    if (row.usable) {
      std::printf("%-12s %6d %6d %8d %8d %8d %8.3f %9.3f %9.3f %8.3f %6d\n",
                  row.name.c_str(), row.rows, row.cols, row.root_pivots,
                  row.cold_pivots, row.warm_pivots,
                  (row.warm_pivots + 1.0) / (row.cold_pivots + 1.0), row.cold_ms,
                  row.warm_ms, row.warm_ms > 0.0 ? row.warm_ms / row.cold_ms : 0.0,
                  row.warm_dse_init);
    } else {
      std::printf("%-12s %6d %6d %8d %8s %8s %8s %9s %9s %8s %6s  (%s)\n",
                  row.name.c_str(), row.rows, row.cols, row.root_pivots, "-",
                  "-", "-", "-", "-", "-", "-", row.note.c_str());
    }
  }

  std::printf("%s\n", std::string(104, '-').c_str());
  const double geo_piv = usable > 0 ? std::exp(log_piv_sum / usable) : 0.0;
  const double geo_ms = usable > 0 ? std::exp(log_ms_sum / usable) : 0.0;
  std::cout << "\nSummary: " << usable << " usable / " << cases.size()
            << " cases, mismatches=" << mismatches << "\n";
  if (usable > 0) {
    std::printf("  geomean warm/cold pivots = %.4f  (%.2fx fewer)\n", geo_piv,
                geo_piv > 0.0 ? 1.0 / geo_piv : 0.0);
    std::printf("  geomean warm/cold wall   = %.4f  (%.2fx faster)\n", geo_ms,
                geo_ms > 0.0 ? 1.0 / geo_ms : 0.0);
    std::printf("  total pivots cold=%ld warm=%ld  (%.1f%% reduction)\n",
                total_cold_piv, total_warm_piv,
                total_cold_piv > 0
                    ? 100.0 * (1.0 - static_cast<double>(total_warm_piv) /
                                         static_cast<double>(total_cold_piv))
                    : 0.0);
  }
}

}  // namespace

int main(int argc, char** argv) {
  Config cfg;
  if (!parse_args(argc, argv, cfg)) return argc > 1 ? 1 : 0;
  std::string command_line;
  for (int i = 0; i < argc; ++i) {
    if (i > 0) command_line += ' ';
    command_line += argv[i];
  }
  try {
    const std::vector<CaseInfo> cases = load_cases(cfg);
    if (cases.empty()) throw std::runtime_error("no matching NETLIB cases");
    if (cfg.warm_cohort) {
      run_warm_cohort(cfg, cases);
      return 0;
    }
    std::cout << "NETLIB cross-algorithm benchmark: " << cases.size()
              << " cases, " << cfg.solvers.size() << " algorithms, "
              << cfg.repeats << " repeat(s)\n";
    std::vector<RunResult> rows;
    rows.reserve(cases.size() * cfg.solvers.size() * cfg.repeats);
    for (const auto& kase : cases) {
      std::cout << "\n[" << kase.name << "] "
                << kase.lp.A.rows() + kase.lp.Aeq.rows() << "x"
                << kase.lp.c.size() << ", nnz="
                << kase.lp.A.nonZeros() + kase.lp.Aeq.nonZeros() << '\n';
      for (const auto& solver : cfg.solvers) {
        for (int repeat = 1; repeat <= cfg.repeats; ++repeat) {
          RunResult row = run_one(kase, solver, cfg, repeat);
          std::printf("  %-38s %9.2f ms  %-8s rel=%9.2e feas=%9.2e  %s\n",
                      row.solver.c_str(), row.runtime_ms,
                      row.accurate ? "ACCURATE" : (row.success ? "WRONG" : "FAIL"),
                      row.objective_rel_error, row.normalized_primal_violation,
                      row.status.c_str());
          if (std::isfinite(row.relative_dual_residual) ||
              std::isfinite(row.relative_gap)) {
            std::printf("    original-kkt primal=%.3e dual=%.3e gap=%.3e\n",
                        row.relative_primal_residual,
                        row.relative_dual_residual, row.relative_gap);
          }
          if (row.native_telemetry) {
            std::printf("    dual_pivots=%d init=%.6f ms/%d solves "
                        "kernel=%.3f ms %.6f ms/pivot\n",
                        row.dual_pivots, row.dse_initialization_ms,
                        row.dse_initialization_solves, row.native_kernel_ms,
                        row.native_kernel_ms_per_dual_pivot);
            if (row.certified_dse_btrans > 0) {
              std::printf("    certified_dse=%d BTRAN, %d candidates, "
                          "%d rejected, %.3f ms\n",
                          row.certified_dse_btrans,
                          row.certified_dse_candidates,
                          row.certified_dse_rejections,
                          row.certified_dse_ms);
            }
          }
          std::fflush(stdout);
          rows.push_back(std::move(row));
        }
      }
    }
    const std::vector<Summary> summaries = summarize(rows);
    print_summary(summaries);
    write_csv(cfg.csv_path, rows);
    write_json(cfg.json_path, cfg, cases, rows, summaries, command_line);
    if (!cfg.csv_path.empty()) std::cout << "CSV:  " << cfg.csv_path << '\n';
    if (!cfg.json_path.empty()) std::cout << "JSON: " << cfg.json_path << '\n';
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "netlib_solver_benchmark: " << e.what() << '\n';
    return 2;
  }
}
