/// End-to-end MILP benchmark over MIPLIB 2017 MPS instances.
///
/// The runner keeps MPS integrality and compares four explicitly named paths:
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
#endif

#include "mipsolvers/engine/bc/api.hpp"
#include "mipsolvers/engine/bc/options.hpp"
#include "mipsolvers/engine/bc/stats.hpp"
#include "mipsolvers/engine/problem_types.hpp"

namespace fs = std::filesystem;
using json = nlohmann::json;
namespace eng = mipsolvers::engine;

namespace {

constexpr double kAuditTolerance = 1e-5;
constexpr double kSummaryShiftMs = 1000.0;

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
  int seed{0};
  double time_limit_sec{60.0};
  double hard_timeout_grace_sec{5.0};
  double gap{1e-4};
  std::string native_node_estimate{"sum"};
  bool native_verbose{false};
  bool native_presolve{true};
  bool native_cuts{true};
  bool native_objective_propagation{true};
  bool native_reduced_cost_fixing{true};
  bool native_row_propagation{true};
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

struct Result {
  std::string instance;
  std::string solver;
  std::string collection_scope{"unavailable"};
  int repeat{0};
  int rows{0};
  int columns{0};
  std::int64_t nonzeros{0};
  int integers{0};
  int binaries{0};
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
  bool native_diagnostics_available{false};
  double read_ms{0.0};
  double shared_decompress_ms{0.0};
  double solve_ms{0.0};
  double objective{std::numeric_limits<double>::quiet_NaN()};
  double best_bound{std::numeric_limits<double>::quiet_NaN()};
  double gap{std::numeric_limits<double>::infinity()};
  std::int64_t nodes{-1};
  std::int64_t lp_solves{-1};
  std::int64_t lp_iterations{-1};
  int cuts{-1};
  int first_incumbent_node{-1};
  int first_incumbent_lp_solves{-1};
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
  double median_solved_ms{std::numeric_limits<double>::quiet_NaN()};
  double shifted_geomean_par10_ms{std::numeric_limits<double>::quiet_NaN()};
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
    } else if (arg == "--seed") {
      const char* v = value("--seed"); if (!v) return false;
      cfg.seed = std::max(0, std::atoi(v));
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
    } else if (arg == "--native-verbose") {
      cfg.native_verbose = true;
    } else if (arg == "--native-no-presolve") {
      cfg.native_presolve = false;
    } else if (arg == "--native-no-cuts") {
      cfg.native_cuts = false;
    } else if (arg == "--native-no-objective-propagation") {
      cfg.native_objective_propagation = false;
    } else if (arg == "--native-no-reduced-cost-fixing") {
      cfg.native_reduced_cost_fixing = false;
    } else if (arg == "--native-no-row-propagation") {
      cfg.native_row_propagation = false;
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
          << "  --solvers A,B        highs-mip,scip-mip,native-highs-lp,native-native-lp\n"
          << "  --case A,B           instance-name substring filters\n"
          << "  --limit N            run first N selected instances (0 = all)\n"
          << "  --sample N           evenly sample N names after filtering (0 = all)\n"
          << "  --repeat N           repetitions per solver/instance\n"
          << "  --time-limit SEC     per-solve wall limit\n"
          << "  --hard-timeout-grace SEC  process watchdog grace after the solve limit\n"
          << "  --native-node-estimate MODE  sum or maximum\n"
          << "  --native-verbose      enable native B&C diagnostic logging\n"
          << "  --native-no-presolve disable PaPILO in native audit runs\n"
          << "  --native-no-cuts     disable native cut generation in audit runs\n"
          << "  --native-no-objective-propagation  disable incumbent-cutoff rows/domain fixing\n"
          << "  --native-no-reduced-cost-fixing    disable reduced-cost domain fixing/learning\n"
          << "  --native-no-row-propagation        disable model/cut row domain propagation\n"
          << "  --gap VALUE          relative MIP gap (default 1e-4)\n"
          << "  --max-nodes N        native B&C node limit\n"
          << "  --seed N             deterministic backend seed\n"
          << "  --csv FILE           raw result CSV\n"
          << "  --json FILE          raw result and summary JSON\n";
      return false;
    } else {
      std::cerr << "Unknown option: " << arg << "\n";
      return false;
    }
  }
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

void set_dimensions(const Instance& instance, Result& result) {
  result.instance = instance.name;
  result.rows = instance.rows;
  result.columns = instance.columns;
  result.nonzeros = instance.nonzeros;
  result.integers = instance.integers;
  result.binaries = instance.binaries;
  result.shared_decompress_ms = instance.decompress_ms;
}

Result run_highs(const Instance& instance, const Config& cfg) {
  Result result;
  set_dimensions(instance, result);
  result.solver = "highs-mip";
  result.collection_scope = "highs_summary";

  Highs::resetGlobalScheduler(true);
  Highs highs;
  highs.setOptionValue("output_flag", false);
  highs.setOptionValue("log_to_console", false);
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
  result.proven = result.optimal || status == SCIP_STATUS_INFEASIBLE ||
                  status == SCIP_STATUS_UNBOUNDED || status == SCIP_STATUS_INFORUNBD;
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
    result.x = Eigen::VectorXd::Zero(instance.columns);
    std::unordered_map<std::string, int> source_index;
    source_index.reserve(static_cast<std::size_t>(instance.columns));
    if (instance.source_lp.col_names_.size() ==
        static_cast<std::size_t>(instance.columns)) {
      for (int j = 0; j < instance.columns; ++j) {
        source_index.emplace(instance.source_lp.col_names_[static_cast<std::size_t>(j)], j);
      }
    }
    SCIP_VAR** variables = SCIPgetOrigVars(scip);
    const int variable_count = SCIPgetNOrigVars(scip);
    for (int i = 0; i < variable_count; ++i) {
      int index = -1;
      const char* name = SCIPvarGetName(variables[i]);
      if (name != nullptr) {
        const auto it = source_index.find(name);
        if (it != source_index.end()) index = it->second;
      }
      if (index < 0 && variable_count == instance.columns) index = i;
      if (index >= 0 && index < instance.columns) {
        result.x[index] = SCIPgetSolVal(scip, solution, variables[i]);
      }
    }
  }
  if (solve_status != SCIP_OKAY && !result.has_solution) {
    result.status = "solve error: " + result.status;
  }
  SCIPfree(&scip);
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
  options.time_limit_sec = cfg.time_limit_sec;
  options.gap_tol = cfg.gap;
  options.max_nodes = cfg.max_nodes;
  options.num_threads = 1;
  options.verbose = cfg.native_verbose;
  options.enable_domain_heuristics = false;
  options.use_papilo_presolve = cfg.native_presolve;
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
  if (!cfg.native_row_propagation) {
    options.bound_propagation_rounds = 0;
  }
  options.node_estimate_aggregation =
      cfg.native_node_estimate == "maximum"
          ? eng::NodeEstimateAggregation::Maximum
          : eng::NodeEstimateAggregation::Sum;

  const auto solve_start = std::chrono::steady_clock::now();
  const eng::BCResult native = eng::solve_milp_bc(instance.mip, options);
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
  result.native_diagnostics_available =
      native.bc_stats.native_diagnostics_available;
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
  result.optimal = result.has_solution && std::isfinite(result.gap) &&
                   result.gap <= cfg.gap * (1.0 + 1e-6);
  result.proven = result.optimal || result.status.find("Infeasible") != std::string::npos ||
                  result.status.find("infeasible") != std::string::npos ||
                  result.status.find("Unbounded") != std::string::npos ||
                  result.status.find("unbounded") != std::string::npos;
  return result;
}

Result run_solver(const Instance& instance, const Config& cfg,
                  const std::string& solver) {
  if (solver == "highs-mip") return run_highs(instance, cfg);
  if (solver == "scip-mip") return run_scip(instance, cfg);
  if (solver == "native-highs-lp") return run_native(instance, cfg, false);
  if (solver == "native-native-lp") return run_native(instance, cfg, true);
  Result result;
  set_dimensions(instance, result);
  result.solver = solver;
  result.available = false;
  result.status = "unknown solver";
  return result;
}

void attach_validation(const Instance& instance, const Reference* reference,
                       Result& result) {
  if (result.has_solution) {
    result.audit = audit_solution(instance, result.x, result.objective);
  }
  if (reference == nullptr) return;
  result.reference_status = reference->status;
  result.reference_objective = reference->objective;
  if (result.optimal && reference->status.find("opt") != std::string::npos &&
      std::isfinite(reference->objective) &&
      std::isfinite(result.objective)) {
    const double rel_error = std::abs(result.objective - reference->objective) /
                             std::max(1.0, std::abs(reference->objective));
    result.reference_match = rel_error <= kAuditTolerance;
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
  out << "instance,solver,repeat,rows,columns,nonzeros,integers,binaries,available,"
         "has_solution,optimal,proven,timed_out,hard_timeout,audit_passed,reference_match,read_ms,"
         "shared_decompress_ms,solve_ms,objective,best_bound,gap,nodes,lp_solves,lp_iterations,cuts,"
         "fallback_events,fallback_recoveries,max_row_violation,max_bound_violation,"
         "max_integrality_violation,objective_disagreement,reference_status,"
         "reference_objective,status\n";
  out << std::setprecision(17);
  for (const Result& r : results) {
    out << csv_escape(r.instance) << ',' << csv_escape(r.solver) << ',' << r.repeat << ','
        << r.rows << ',' << r.columns << ',' << r.nonzeros << ',' << r.integers << ','
        << r.binaries << ',' << r.available << ',' << r.has_solution << ',' << r.optimal
        << ',' << r.proven << ',' << r.timed_out << ',' << r.hard_timeout << ','
        << r.audit.passed << ','
        << r.reference_match << ',' << r.read_ms << ',' << r.shared_decompress_ms << ','
        << r.solve_ms << ','
        << csv_number(r.objective) << ',' << csv_number(r.best_bound) << ','
        << csv_number(r.gap) << ','
        << csv_count(r.nodes, r.node_count_available) << ','
        << csv_count(r.lp_solves, r.lp_solve_count_available) << ','
        << csv_count(r.lp_iterations, r.lp_iteration_count_available) << ','
        << csv_count(r.cuts, r.cut_count_available) << ','
        << csv_count(r.fallback_events, r.native_diagnostics_available) << ','
        << csv_count(r.fallback_recoveries, r.native_diagnostics_available) << ','
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

json result_json(const Result& r) {
  return {
      {"instance", r.instance}, {"solver", r.solver}, {"repeat", r.repeat},
      {"collection_scope", r.collection_scope},
      {"statistics_available", {
          {"nodes", r.node_count_available},
          {"lp_solves", r.lp_solve_count_available},
          {"lp_iterations", r.lp_iteration_count_available},
          {"cuts", r.cut_count_available},
          {"incumbent_timeline", r.incumbent_timeline_available},
          {"native_diagnostics", r.native_diagnostics_available}}},
      {"rows", r.rows}, {"columns", r.columns}, {"nonzeros", r.nonzeros},
      {"integers", r.integers}, {"binaries", r.binaries},
      {"available", r.available}, {"has_solution", r.has_solution},
      {"optimal", r.optimal}, {"proven", r.proven}, {"timed_out", r.timed_out},
      {"hard_timeout", r.hard_timeout},
      {"read_ms", r.read_ms}, {"shared_decompress_ms", r.shared_decompress_ms},
      {"solve_ms", r.solve_ms},
      {"objective", json_number(r.objective)},
      {"best_bound", json_number(r.best_bound)}, {"gap", json_number(r.gap)},
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
      {"branching", r.native_diagnostics_available ? json{
          {"reliability_nodes", r.reliability_branch_nodes},
          {"strong_candidates", r.strong_branch_candidates},
          {"strong_lp_solves", r.strong_branch_lp_solves},
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
  out["x"] = json::array();
  for (int i = 0; i < result.x.size(); ++i) out["x"].push_back(result.x[i]);
  return out;
}

Result worker_result_from_json(const json& input) {
  Result result;
  result.instance = input.value("instance", "");
  result.solver = input.value("solver", "");
  result.collection_scope = input.value("collection_scope", "unavailable");
  const json availability = input.value("statistics_available", json::object());
  result.node_count_available = availability.value("nodes", false);
  result.lp_solve_count_available = availability.value("lp_solves", false);
  result.lp_iteration_count_available = availability.value("lp_iterations", false);
  result.cut_count_available = availability.value("cuts", false);
  result.incumbent_timeline_available =
      availability.value("incumbent_timeline", false);
  result.native_diagnostics_available =
      availability.value("native_diagnostics", false);
  result.rows = input.value("rows", 0);
  result.columns = input.value("columns", 0);
  result.nonzeros = input.value("nonzeros", std::int64_t{0});
  result.integers = input.value("integers", 0);
  result.binaries = input.value("binaries", 0);
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
  result.nodes = optional_json_integer(input, "nodes", std::int64_t{-1});
  result.lp_solves = optional_json_integer(input, "lp_solves", std::int64_t{-1});
  result.lp_iterations =
      optional_json_integer(input, "lp_iterations", std::int64_t{-1});
  result.cuts = optional_json_integer(input, "cuts", -1);
  result.first_incumbent_node =
      optional_json_integer(input, "first_incumbent_node", -1);
  result.first_incumbent_lp_solves =
      optional_json_integer(input, "first_incumbent_lp_solves", -1);

  const json branching = input.contains("branching") && input["branching"].is_object()
      ? input["branching"] : json::object();
  result.reliability_branch_nodes = branching.value("reliability_nodes", std::uint64_t{0});
  result.strong_branch_candidates = branching.value("strong_candidates", std::uint64_t{0});
  result.strong_branch_lp_solves = branching.value("strong_lp_solves", std::uint64_t{0});
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
      "--seed", std::to_string(cfg.seed),
      "--native-node-estimate", cfg.native_node_estimate};
  if (cfg.native_verbose) arguments.push_back("--native-verbose");
  if (!cfg.native_presolve) arguments.push_back("--native-no-presolve");
  if (!cfg.native_cuts) arguments.push_back("--native-no-cuts");
  if (!cfg.native_objective_propagation) {
    arguments.push_back("--native-no-objective-propagation");
  }
  if (!cfg.native_reduced_cost_fixing) {
    arguments.push_back("--native-no-reduced-cost-fixing");
  }
  if (!cfg.native_row_propagation) {
    arguments.push_back("--native-no-row-propagation");
  }

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
    std::vector<double> penalized_times;
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
      const bool benchmark_solved =
          (result.proven && result.audit.passed &&
           (result.reference_status.empty() || result.reference_match)) ||
          (result.proven && !result.has_solution && result.reference_match);
      if (benchmark_solved) {
        solved_times.push_back(result.solve_ms);
        penalized_times.push_back(result.solve_ms);
      } else {
        penalized_times.push_back(par10_ms);
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
  out["threads"] = 1;
  out["seed"] = cfg.seed;
  out["max_nodes_native"] = cfg.max_nodes;
  out["native_node_estimate"] = cfg.native_node_estimate;
  out["native_presolve"] = cfg.native_presolve;
  out["native_cuts"] = cfg.native_cuts;
  out["native_objective_propagation"] =
      cfg.native_cuts && cfg.native_objective_propagation;
  out["native_reduced_cost_fixing"] = cfg.native_reduced_cost_fixing;
  out["native_row_propagation"] = cfg.native_row_propagation;
  out["summary_policy"] = {
      {"solved", "proven and independently audited incumbent"},
      {"timeout_penalty", "PAR-10"}, {"shift_ms", kSummaryShiftMs}};
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
  std::printf("\n%-20s %8s %8s %8s %8s %10s %14s\n",
              "Solver", "Runs", "Feas", "Optimal", "Audit", "Timeouts", "PAR10-shift ms");
  std::printf("%s\n", std::string(92, '-').c_str());
  for (const Summary& s : summaries) {
    std::printf("%-20s %8d %8d %8d %8d %10d %14.1f\n", s.solver.c_str(),
                s.attempts, s.feasible, s.optimal, s.audited, s.timeouts,
                s.shifted_geomean_par10_ms);
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
      "highs-mip", "scip-mip", "native-highs-lp", "native-native-lp"};

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

  std::printf("MIPLIB 2017: %zu instances, %zu solvers, %d repeat(s), %.3g s, gap %.3g\n",
              paths.size(), cfg.solvers.size(), cfg.repeats,
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
  for (const fs::path& path : paths) {
    Instance instance = load_instance(path);
    if (!instance.error.empty()) {
      std::fprintf(stderr, "%s: %s\n", instance.name.c_str(), instance.error.c_str());
      for (int repeat = 0; repeat < cfg.repeats; ++repeat) {
        for (const std::string& solver : cfg.solvers) {
          Result result;
          set_dimensions(instance, result);
          result.instance = instance.name;
          result.solver = solver;
          result.repeat = repeat;
          result.available = false;
          result.status = "model load: " + instance.error;
          print_result(result);
          results.push_back(std::move(result));
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
    for (int repeat = 0; repeat < cfg.repeats; ++repeat) {
      for (const std::string& solver : cfg.solvers) {
        Result result = run_solver_isolated(instance, cfg, solver, executable);
        result.repeat = repeat;
        attach_validation(instance, reference, result);
        print_result(result);
        results.push_back(std::move(result));
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
