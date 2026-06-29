#include "mipsolvers/engine/solver/external/adapters.hpp"

#include <chrono>
#include <cmath>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <algorithm>
#include <map>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include "mipsolvers/core/string_utils.hpp"
#include "mipsolvers/engine/util/problem_validation.hpp"

#ifdef HACDCPF_HAVE_HIGHS_LIB
#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunused-parameter"
#elif defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#endif
#include "Highs.h"
#if defined(__clang__)
#pragma clang diagnostic pop
#elif defined(__GNUC__)
#pragma GCC diagnostic pop
#endif
#endif

#ifdef HACDCPF_HAVE_IPOPT
#include "IpIpoptApplication.hpp"
#include "IpTNLP.hpp"
#endif

#ifdef HACDCPF_HAVE_SCIP_LIB
#include <scip/scip.h>
#include <scip/scipdefplugins.h>
#endif

namespace fs = std::filesystem;

namespace mipsolvers::engine {
namespace {

std::string shell_quote(const fs::path& p) {
  return std::string("\"") + p.string() + "\"";
}

std::string shell_quote(const std::string& s) {
  return std::string("\"") + s + "\"";
}

const char* shell_null_device() {
#ifdef _WIN32
  return "NUL";
#else
  return "/dev/null";
#endif
}

bool file_exists(const std::string& p) {
  return !p.empty() && fs::exists(fs::path(p));
}

std::string resolve_explicit_executable(const std::string& explicit_path) {
  if (!explicit_path.empty() && file_exists(explicit_path)) {
    return explicit_path;
  }
  return "";
}

bool env_flag_enabled(const char* name) {
  const char* env = std::getenv(name);
  return env != nullptr && env[0] != '\0' && env[0] != '0';
}

void relay_highs_conformance_lines(const fs::path& log_path) {
  const bool relay_conf = env_flag_enabled("HACDCPF_HIGHS_CONF");
  const bool relay_frontier =
      env_flag_enabled("HACDCPF_HIGHS_FRONTIER_CONFORM");
  const bool relay_timeline = env_flag_enabled("HACDCPF_HIGHS_TIMELINE");
  const bool relay_xrow = env_flag_enabled("HIGHS_XROW_TRACE") ||
                          env_flag_enabled("HACDCPF_XTAB_ROW_TRACE");
  const bool relay_lpbasis = env_flag_enabled("HIGHS_LP_BASIS_TRACE") ||
                             env_flag_enabled("HACDCPF_LP_BASIS_TRACE") ||
                             relay_xrow;
  const bool relay_analysis = std::getenv("HACDCPF_HIGHS_ANALYSIS") != nullptr;
  if (!relay_conf && !relay_frontier && !relay_timeline && !relay_xrow &&
      !relay_lpbasis && !relay_analysis)
    return;
  std::ifstream in(log_path);
  std::string line;
  while (std::getline(in, line)) {
    if ((relay_conf && (line.find("[HIGHS-CONF]") != std::string::npos ||
                        line.find("[HIGHS-SEP]") != std::string::npos ||
                        line.find("[HIGHS-PRESOLVE-STATE]") != std::string::npos ||
                        line.find("[HIGHS-VBSTATE]") != std::string::npos ||
                        line.find("[HIGHS-LPSTATE]") != std::string::npos)) ||
        (relay_timeline && line.find("[HIGHS-TL]") != std::string::npos) ||
        (relay_frontier &&
         (line.find("[HIGHS-FRONTIER]") != std::string::npos ||
          line.find("[HIGHS-LPSTATE]") != std::string::npos ||
          line.find("[HIGHS-PRESOLVE-STATE]") != std::string::npos ||
          line.find("[HIGHS-VBSTATE]") != std::string::npos ||
          line.find("[HIGHS-REPAIR]") != std::string::npos ||
          line.find("[HIGHS-REPAIR-CAND]") != std::string::npos)) ||
        (relay_xrow && line.find("[HIGHS-XROW]") != std::string::npos)) {
      std::fprintf(stderr, "%s\n", line.c_str());
    } else if (relay_lpbasis &&
               line.find("[HIGHS-LPBASIS]") != std::string::npos) {
      std::fprintf(stderr, "%s\n", line.c_str());
    } else if (relay_analysis &&
               (line.find("MipCore_") != std::string::npos ||
                line.find("MipLevl1") != std::string::npos ||
                line.find("MipRootNode") != std::string::npos ||
                line.find("MipSerch") != std::string::npos ||
                line.find("MipDive") != std::string::npos ||
                line.find("MipNodeSearch") != std::string::npos ||
                line.find("MipSeparation") != std::string::npos ||
                line.find("MipSlvLp") != std::string::npos ||
                line.find("MipSubMip") != std::string::npos ||
                line.find("MipPrslv") != std::string::npos ||
                line.find("MipRootSeparation") != std::string::npos)) {
      std::fprintf(stderr, "[HIGHS-CLK] %s\n", line.c_str());
    }
  }
}

void write_mps_bounds(std::ofstream& out, const std::vector<VariableMeta>& vars) {
  out << "BOUNDS\n";
  for (size_t j = 0; j < vars.size(); ++j) {
    const std::string v = "X" + std::to_string(j + 1);
    out << " LO BND       " << v << " " << vars[j].lb << "\n";
    out << " UP BND       " << v << " " << vars[j].ub << "\n";
  }
}

void write_mps_rhs(std::ofstream& out,
                   const Eigen::VectorXd& b,
                   const std::string& prefix) {
  for (int i = 0; i < b.size(); ++i) {
    out << "    RHS1      " << prefix << (i + 1) << " " << b[i] << "\n";
  }
}

void write_mps_ranges(std::ofstream& out, const LPModel& lp) {
  if (!lp_has_row_lhs(lp)) return;
  bool opened = false;
  for (int i = 0; i < lp.A.rows(); ++i) {
    const double lhs = lp_row_lhs_or_neg_inf(lp, i);
    if (!std::isfinite(lhs) || !std::isfinite(lp.b[i])) continue;
    const double range = lp.b[i] - lhs;
    if (range < -1e-9) continue;
    if (!opened) {
      out << "RANGES\n";
      opened = true;
    }
    out << "    RNG1      C" << (i + 1) << " "
        << std::max(0.0, range) << "\n";
  }
}

bool write_lp_as_mps(const LPModel& lp, const fs::path& mps_path, bool with_integer_markers) {
  std::ofstream out(mps_path);
  if (!out.is_open()) {
    return false;
  }

  out << "NAME HACDCPF\n";
  if (lp.sense == Sense::Maximize) {
    out << "OBJSENSE\n    MAX\n";
  }
  out << "ROWS\n";
  out << " N  OBJ\n";
  for (int i = 0; i < lp.A.rows(); ++i) {
    out << " L  C" << (i + 1) << "\n";
  }
  for (int i = 0; i < lp.Aeq.rows(); ++i) {
    out << " E  E" << (i + 1) << "\n";
  }

  out << "COLUMNS\n";
  bool in_mark = false;
  const Eigen::SparseMatrix<double, Eigen::ColMajor> A_col(lp.A);
  const Eigen::SparseMatrix<double, Eigen::ColMajor> Aeq_col(lp.Aeq);
  for (int j = 0; j < lp.c.size(); ++j) {
    if (with_integer_markers) {
      const bool is_int = lp.vars[j].type == VarType::Integer ||
                          lp.vars[j].type == VarType::Binary;
      if (is_int && !in_mark) {
        out << "    MARK0000  'MARKER'                 'INTORG'\n";
        in_mark = true;
      }
      if (!is_int && in_mark) {
        out << "    MARK0001  'MARKER'                 'INTEND'\n";
        in_mark = false;
      }
    }

    const std::string xname = "X" + std::to_string(j + 1);
    if (lp.c[j] != 0.0) {
      out << "    " << xname << "  OBJ " << lp.c[j] << "\n";
    }
    for (Eigen::SparseMatrix<double, Eigen::ColMajor>::InnerIterator it(A_col, j);
         it; ++it) {
      out << "    " << xname << "  C" << (it.row() + 1) << " "
          << it.value() << "\n";
    }
    for (Eigen::SparseMatrix<double, Eigen::ColMajor>::InnerIterator it(Aeq_col, j);
         it; ++it) {
      out << "    " << xname << "  E" << (it.row() + 1) << " "
          << it.value() << "\n";
    }
  }
  if (with_integer_markers && in_mark) {
    out << "    MARK0002  'MARKER'                 'INTEND'\n";
  }

  out << "RHS\n";
  write_mps_rhs(out, lp.b, "C");
  write_mps_rhs(out, lp.beq, "E");
  write_mps_ranges(out, lp);

  write_mps_bounds(out, lp.vars);
  out << "ENDATA\n";

  return true;
}

std::optional<double> parse_first_number(const std::string& text,
                                         bool* is_percent = nullptr) {
  std::stringstream ss(text);
  std::string tok;
  while (ss >> tok) {
    try {
      size_t p = 0;
      const double v = std::stod(tok, &p);
      if (p > 0) {
        if (is_percent) {
          *is_percent = tok.find('%') != std::string::npos;
        }
        return v;
      }
    } catch (const std::exception&) {
    }
  }
  return std::nullopt;
}

struct HighsRunReport {
  std::optional<std::string> status;
  std::optional<double> primal_bound;
  std::optional<double> dual_bound;
  std::optional<double> gap;
};

std::optional<std::string> parse_highs_report_status_line(const std::string& line) {
  const std::string t = trim(line);
  constexpr const char* kStatus = "Status";
  if (t.rfind(kStatus, 0) != 0) return std::nullopt;
  const std::string rest =
      trim(t.substr(std::char_traits<char>::length(kStatus)));
  return rest.empty() ? std::nullopt : std::optional<std::string>(rest);
}

HighsRunReport parse_highs_run_report(const fs::path& log_path) {
  HighsRunReport report;
  std::ifstream in(log_path);
  if (!in.is_open()) return report;

  std::string line;
  bool in_solving_report = false;
  while (std::getline(in, line)) {
    const std::string t = trim(line);
    if (t == "Solving report") {
      in_solving_report = true;
      continue;
    }
    if (!in_solving_report) continue;

    if (!report.status) {
      report.status = parse_highs_report_status_line(t);
    }
    if (t.rfind("Primal bound", 0) == 0) {
      report.primal_bound = parse_first_number(t.substr(12));
    } else if (t.rfind("Dual bound", 0) == 0) {
      report.dual_bound = parse_first_number(t.substr(10));
    } else if (t.rfind("Gap", 0) == 0) {
      bool is_percent = false;
      if (auto v = parse_first_number(t.substr(3), &is_percent)) {
        report.gap = is_percent ? (*v / 100.0) : *v;
      }
    }
  }
  return report;
}

SolveResult unavailable_result(const std::string& solver_name, const std::string& reason) {
  SolveResult out;
  out.stats.success = false;
  out.stats.solver_name = solver_name;
  out.stats.status = "Unavailable: " + reason;
  return out;
}

std::optional<double> parse_highs_solution_objective(const fs::path& sol_path) {
  std::ifstream in(sol_path);
  if (!in.is_open()) {
    return std::nullopt;
  }

  std::string line;
  while (std::getline(in, line)) {
    if (line.find("Objective") != std::string::npos || line.find("objective") != std::string::npos) {
      std::stringstream ss(line);
      std::string tok;
      while (ss >> tok) {
        try {
          size_t p = 0;
          const double v = std::stod(tok, &p);
          if (p == tok.size()) {
            return v;
          }
        } catch (const std::exception&) {
        }
      }
    }
  }
  return std::nullopt;
}

std::optional<std::string> parse_highs_model_status(const fs::path& sol_path) {
  std::ifstream in(sol_path);
  if (!in.is_open()) {
    return std::nullopt;
  }

  std::string line;
  while (std::getline(in, line)) {
    if (trim(line) == "Model status") {
      while (std::getline(in, line)) {
        const std::string s = trim(line);
        if (!s.empty()) {
          return s;
        }
      }
      break;
    }
  }
  return std::nullopt;
}

#ifdef HACDCPF_HAVE_HIGHS_LIB
std::string highs_model_status_label(HighsModelStatus status) {
  switch (status) {
    case HighsModelStatus::kNotset:
      return "notset";
    case HighsModelStatus::kLoadError:
      return "load_error";
    case HighsModelStatus::kModelError:
      return "model_error";
    case HighsModelStatus::kPresolveError:
      return "presolve_error";
    case HighsModelStatus::kSolveError:
      return "solve_error";
    case HighsModelStatus::kPostsolveError:
      return "postsolve_error";
    case HighsModelStatus::kModelEmpty:
      return "empty";
    case HighsModelStatus::kOptimal:
      return "optimal";
    case HighsModelStatus::kInfeasible:
      return "infeasible";
    case HighsModelStatus::kUnboundedOrInfeasible:
      return "unbounded_or_infeasible";
    case HighsModelStatus::kUnbounded:
      return "unbounded";
    case HighsModelStatus::kObjectiveBound:
      return "objective_bound";
    case HighsModelStatus::kObjectiveTarget:
      return "objective_target";
    case HighsModelStatus::kTimeLimit:
      return "time_limit";
    case HighsModelStatus::kIterationLimit:
      return "iteration_limit";
    case HighsModelStatus::kUnknown:
      return "unknown";
    case HighsModelStatus::kSolutionLimit:
      return "solution_limit";
    case HighsModelStatus::kInterrupt:
      return "interrupt";
    case HighsModelStatus::kMemoryLimit:
      return "memory_limit";
    case HighsModelStatus::kHighsInterrupt:
      return "highs_interrupt";
  }
  return "unknown";
}

bool highs_status_has_solution(HighsModelStatus status) {
  return status == HighsModelStatus::kOptimal ||
         status == HighsModelStatus::kTimeLimit ||
         status == HighsModelStatus::kIterationLimit ||
         status == HighsModelStatus::kSolutionLimit;
}

std::optional<SolveResult> solve_lp_with_embedded_highs(const LPModel& prob,
                                                        bool with_integer_markers,
                                                        const std::string& solver_name,
                                                        const Eigen::VectorXd* mip_start) {
  const auto t0 = std::chrono::steady_clock::now();
  SolveResult out;
  out.stats.solver_name = solver_name;

  const ValidationReport vr = validate(prob);
  if (!vr.valid) {
    out.stats.status = vr.errors.empty() ? "Invalid model" : vr.errors.front();
    return out;
  }

  const int ncols = static_cast<int>(prob.vars.size());
  const int m_ineq = static_cast<int>(prob.A.rows());
  const int m_eq = static_cast<int>(prob.Aeq.rows());
  const int nrows = m_ineq + m_eq;
  if (ncols == 0) {
    out.stats.success = true;
    out.stats.status = "HiGHS empty";
    out.x = Eigen::VectorXd::Zero(0);
    return out;
  }

  std::vector<double> col_cost(static_cast<std::size_t>(ncols), 0.0);
  std::vector<double> col_lower(static_cast<std::size_t>(ncols), -kHighsInf);
  std::vector<double> col_upper(static_cast<std::size_t>(ncols), kHighsInf);
  std::vector<HighsInt> integrality(static_cast<std::size_t>(ncols),
                                    static_cast<HighsInt>(HighsVarType::kContinuous));
  for (int j = 0; j < ncols; ++j) {
    const auto& var = prob.vars[static_cast<std::size_t>(j)];
    col_cost[static_cast<std::size_t>(j)] = prob.c[j];
    col_lower[static_cast<std::size_t>(j)] =
        std::isfinite(var.lb) ? var.lb : -kHighsInf;
    col_upper[static_cast<std::size_t>(j)] =
        std::isfinite(var.ub) ? var.ub : kHighsInf;
    if (with_integer_markers && var.type != VarType::Continuous) {
      integrality[static_cast<std::size_t>(j)] =
          static_cast<HighsInt>(HighsVarType::kInteger);
    }
  }

  std::vector<double> row_lower(static_cast<std::size_t>(nrows), -kHighsInf);
  std::vector<double> row_upper(static_cast<std::size_t>(nrows), kHighsInf);
  for (int r = 0; r < m_ineq; ++r) {
    const double lhs = lp_row_lhs_or_neg_inf(prob, r);
    row_lower[static_cast<std::size_t>(r)] =
        std::isfinite(lhs) ? lhs : -kHighsInf;
    row_upper[static_cast<std::size_t>(r)] =
        std::isfinite(prob.b[r]) ? prob.b[r] : kHighsInf;
  }
  for (int r = 0; r < m_eq; ++r) {
    const int rr = m_ineq + r;
    const double rhs = prob.beq[r];
    row_lower[static_cast<std::size_t>(rr)] =
        std::isfinite(rhs) ? rhs : (rhs < 0.0 ? -kHighsInf : kHighsInf);
    row_upper[static_cast<std::size_t>(rr)] = row_lower[static_cast<std::size_t>(rr)];
  }

  std::vector<HighsInt> start(static_cast<std::size_t>(ncols + 1), 0);
  std::vector<HighsInt> index;
  std::vector<double> value;
  index.reserve(static_cast<std::size_t>(prob.A.nonZeros() + prob.Aeq.nonZeros()));
  value.reserve(index.capacity());
  for (int j = 0; j < ncols; ++j) {
    start[static_cast<std::size_t>(j)] = static_cast<HighsInt>(index.size());
    for (Eigen::SparseMatrix<double>::InnerIterator it(prob.A, j); it; ++it) {
      if (it.value() == 0.0) continue;
      index.push_back(static_cast<HighsInt>(it.row()));
      value.push_back(it.value());
    }
    for (Eigen::SparseMatrix<double>::InnerIterator it(prob.Aeq, j); it; ++it) {
      if (it.value() == 0.0) continue;
      index.push_back(static_cast<HighsInt>(m_ineq + it.row()));
      value.push_back(it.value());
    }
  }
  start[static_cast<std::size_t>(ncols)] = static_cast<HighsInt>(index.size());

  Highs highs;
  const bool highs_log_on = std::getenv("MIPSOLVERS_HIGHS_ADAPTER_LOG") != nullptr;
  highs.setOptionValue("output_flag", highs_log_on);
  highs.setOptionValue("log_to_console", highs_log_on);
  highs.setOptionValue("threads", 1);
  if (with_integer_markers) {
    highs.setOptionValue("mip_rel_gap", 1e-4);
  }

  const auto pass_status = highs.passModel(
      static_cast<HighsInt>(ncols), static_cast<HighsInt>(nrows),
      static_cast<HighsInt>(index.size()),
      static_cast<HighsInt>(MatrixFormat::kColwise),
      static_cast<HighsInt>(prob.sense == Sense::Maximize ? ObjSense::kMaximize
                                                          : ObjSense::kMinimize),
      0.0, col_cost.data(), col_lower.data(), col_upper.data(), row_lower.data(),
      row_upper.data(), start.data(), index.data(), value.data(),
      with_integer_markers ? integrality.data() : nullptr);
  if (pass_status == HighsStatus::kError) {
    out.stats.status = "HiGHS passModel failed";
    return out;
  }

  if (with_integer_markers && mip_start != nullptr &&
      static_cast<int>(mip_start->size()) == ncols) {
    HighsSolution start;
    start.value_valid = true;
    start.dual_valid = false;
    start.col_value.resize(static_cast<std::size_t>(ncols), 0.0);
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
      start.col_value[static_cast<std::size_t>(j)] = value_j;
    }
    highs.setSolution(start);
  }

  const auto run_status = highs.run();
  const HighsModelStatus model_status = highs.getModelStatus();
  out.stats.status = "HiGHS " + highs_model_status_label(model_status);

  const bool optimal = model_status == HighsModelStatus::kOptimal;
  const bool feasible_with_limit =
      highs_status_has_solution(model_status) && highs.getSolution().value_valid;
  out.stats.success = optimal || feasible_with_limit;
  if (!out.stats.success && run_status == HighsStatus::kError) {
    out.stats.status = "HiGHS solve failed";
    return out;
  }

  const HighsInfo& info = highs.getInfo();
  out.stats.objective = info.objective_function_value;
  out.stats.iterations = static_cast<int>(info.simplex_iteration_count +
                                          info.ipm_iteration_count +
                                          info.pdlp_iteration_count);
  out.stats.primal_feas = info.max_primal_infeasibility;
  out.stats.dual_feas = info.max_dual_infeasibility;
  out.stats.residual_inf =
      std::max(info.max_primal_infeasibility, info.max_dual_infeasibility);
  if (with_integer_markers && std::isfinite(info.mip_gap)) {
    out.stats.mip_gap = info.mip_gap;
  }

  const HighsSolution& sol = highs.getSolution();
  if (static_cast<int>(sol.col_value.size()) >= ncols) {
    out.x = Eigen::VectorXd::Zero(ncols);
    for (int j = 0; j < ncols; ++j) {
      out.x[j] = sol.col_value[static_cast<std::size_t>(j)];
    }
  }

  if (!with_integer_markers &&
      static_cast<int>(sol.row_dual.size()) >= nrows &&
      static_cast<int>(sol.col_dual.size()) >= ncols) {
    out.constraint_duals = Eigen::VectorXd::Zero(nrows);
    for (int i = 0; i < nrows; ++i) {
      out.constraint_duals[i] = sol.row_dual[static_cast<std::size_t>(i)];
    }
    out.box_dual_lb = Eigen::VectorXd::Zero(ncols);
    out.box_dual_ub = Eigen::VectorXd::Zero(ncols);
    for (int j = 0; j < ncols; ++j) {
      const double dual = sol.col_dual[static_cast<std::size_t>(j)];
      if (dual > 0.0) {
        out.box_dual_lb[j] = dual;
      } else if (dual < 0.0) {
        out.box_dual_ub[j] = -dual;
      }
    }
  }

  const auto t1 = std::chrono::steady_clock::now();
  out.stats.runtime_sec = std::chrono::duration<double>(t1 - t0).count();
  return out;
}
#endif

// ---------------------------------------------------------------------------
// Parse HiGHS solution file for variable values
// HiGHS .sol format has a "Columns" section with lines like:
//   <index> <name> <value>   or   <name> <value>
// Variables are named X1, X2, ... (1-indexed) matching write_lp_as_mps().
// ---------------------------------------------------------------------------
std::unordered_map<std::string, double> parse_highs_solution_values(
    const fs::path& sol_path) {
  std::unordered_map<std::string, double> values;
  std::ifstream in(sol_path);
  if (!in.is_open()) return values;

  std::string line;
  bool in_columns = false;

  const auto normalize_section_line = [](std::string text) {
    text = trim(text);
    if (!text.empty() && text.front() == '#') {
      text.erase(text.begin());
      text = trim(text);
    }
    return text;
  };

  while (std::getline(in, line)) {
    const std::string t = trim(line);
    if (t.empty()) continue;

    const std::string normalized = normalize_section_line(t);

    // Detect start of Columns section
    if (normalized.rfind("Columns", 0) == 0 || normalized.rfind("columns", 0) == 0) {
      in_columns = true;
      continue;
    }

    // Stop at next named section (e.g., "Rows")
    if (in_columns && (normalized.rfind("Rows", 0) == 0 ||
                       normalized.rfind("rows", 0) == 0 ||
                       normalized.rfind("Dual", 0) == 0 ||
                       normalized.rfind("dual", 0) == 0 ||
                       normalized.rfind("Basis", 0) == 0 ||
                       normalized.rfind("basis", 0) == 0)) {
      break;
    }

    if (!in_columns) continue;

    // Parse: possibly "<int> <name> <value>" or "<name> <value>"
    std::istringstream iss(t);
    std::string tok1, tok2, tok3;
    iss >> tok1;
    if (!iss) continue;

    // Try 3-token format: index name value
    if (iss >> tok2 && iss >> tok3) {
      // tok1 might be integer index, tok2 = name, tok3 = value
      try {
        double val = std::stod(tok3);
        values[tok2] = val;
        continue;
      } catch (const std::exception&) {}
    }

    // Try 2-token format: name value
    if (!tok2.empty()) {
      try {
        double val = std::stod(tok2);
        values[tok1] = val;
        continue;
      } catch (const std::exception&) {}
    }
  }

  return values;
}

// ---------------------------------------------------------------------------
// Polynomial expansion for PIP export
// ---------------------------------------------------------------------------
// A Monomial is coefficient * product of (variable_index, power) pairs.
// A Polynomial is a vector of Monomials.

struct Monomial {
  double coeff{1.0};
  std::map<int, int> vars;  // var_index -> integer power (>= 1)
};

using Polynomial = std::vector<Monomial>;

Polynomial poly_constant(double c) {
  return {{c, {}}};
}

Polynomial poly_variable(int idx) {
  Monomial m;
  m.coeff = 1.0;
  m.vars[idx] = 1;
  return {m};
}

Polynomial poly_negate(const Polynomial& p) {
  Polynomial out;
  out.reserve(p.size());
  for (auto m : p) {
    m.coeff = -m.coeff;
    out.push_back(std::move(m));
  }
  return out;
}

Polynomial poly_add(const Polynomial& a, const Polynomial& b) {
  Polynomial out = a;
  out.insert(out.end(), b.begin(), b.end());
  return out;
}

Polynomial poly_sub(const Polynomial& a, const Polynomial& b) {
  return poly_add(a, poly_negate(b));
}

Monomial mono_mul(const Monomial& a, const Monomial& b) {
  Monomial out;
  out.coeff = a.coeff * b.coeff;
  out.vars = a.vars;
  for (const auto& [idx, pow] : b.vars) {
    out.vars[idx] += pow;
  }
  return out;
}

Polynomial poly_mul(const Polynomial& a, const Polynomial& b) {
  Polynomial out;
  out.reserve(a.size() * b.size());
  for (const auto& ma : a) {
    for (const auto& mb : b) {
      out.push_back(mono_mul(ma, mb));
    }
  }
  return out;
}

Polynomial poly_pow(const Polynomial& base, int n) {
  if (n <= 0) {
    return poly_constant(1.0);
  }
  Polynomial result = poly_constant(1.0);
  for (int i = 0; i < n; ++i) {
    result = poly_mul(result, base);
  }
  return result;
}

// Collect like terms to keep the polynomial compact.
Polynomial poly_simplify(const Polynomial& p) {
  // Key: sorted vector of (var_index, power)
  std::map<std::vector<std::pair<int, int>>, double> bucket;
  for (const auto& m : p) {
    std::vector<std::pair<int, int>> key(m.vars.begin(), m.vars.end());
    bucket[key] += m.coeff;
  }
  Polynomial out;
  out.reserve(bucket.size());
  for (auto& [key, c] : bucket) {
    if (std::abs(c) < 1e-15) {
      continue;
    }
    Monomial m;
    m.coeff = c;
    m.vars.insert(key.begin(), key.end());
    out.push_back(std::move(m));
  }
  return out;
}

bool expand_to_polynomial(const std::shared_ptr<SymExpr>& e,
                          Polynomial& out,
                          std::string& err) {
  if (!e) {
    err = "null symbolic expression";
    return false;
  }

  switch (e->op) {
    case SymOp::Constant:
      out = poly_constant(e->value);
      return true;
    case SymOp::Variable:
      out = poly_variable(e->var_index);
      return true;
    case SymOp::Neg: {
      Polynomial a;
      if (!expand_to_polynomial(e->lhs, a, err)) return false;
      out = poly_negate(a);
      return true;
    }
    case SymOp::Add: {
      Polynomial a, b;
      if (!expand_to_polynomial(e->lhs, a, err)) return false;
      if (!expand_to_polynomial(e->rhs, b, err)) return false;
      out = poly_add(a, b);
      return true;
    }
    case SymOp::Sub: {
      Polynomial a, b;
      if (!expand_to_polynomial(e->lhs, a, err)) return false;
      if (!expand_to_polynomial(e->rhs, b, err)) return false;
      out = poly_sub(a, b);
      return true;
    }
    case SymOp::Mul: {
      Polynomial a, b;
      if (!expand_to_polynomial(e->lhs, a, err)) return false;
      if (!expand_to_polynomial(e->rhs, b, err)) return false;
      out = poly_mul(a, b);
      return true;
    }
    case SymOp::Pow2: {
      Polynomial a;
      if (!expand_to_polynomial(e->lhs, a, err)) return false;
      out = poly_pow(a, 2);
      return true;
    }
    case SymOp::PowN: {
      Polynomial a;
      if (!expand_to_polynomial(e->lhs, a, err)) return false;
      const int n = e->var_index;
      if (n < 0) {
        err = "PowN exponent must be non-negative";
        return false;
      }
      out = poly_pow(a, n);
      return true;
    }
  }

  err = "unsupported symbolic operation";
  return false;
}

// ---------------------------------------------------------------------------
// PIP format serialization
// ---------------------------------------------------------------------------

std::string sanitize_var_name(const std::string& base, int idx) {
  if (base.empty()) {
    return "x" + std::to_string(idx + 1);
  }
  std::string out;
  out.reserve(base.size());
  for (char c : base) {
    if (std::isalnum(static_cast<unsigned char>(c)) || c == '_') {
      out.push_back(c);
    } else {
      out.push_back('_');
    }
  }
  if (out.empty()) {
    out = "x" + std::to_string(idx + 1);
  }
  return out;
}

// Format a simplified polynomial in PIP syntax (e.g. "3 x^2 y - 2 z + 7").
std::string poly_to_pip(const Polynomial& p,
                        const std::vector<std::string>& var_names) {
  if (p.empty()) {
    return "0";
  }

  std::ostringstream os;
  bool first = true;
  for (const auto& m : p) {
    const double c = m.coeff;
    if (std::abs(c) < 1e-15) continue;

    if (first) {
      if (m.vars.empty()) {
        os << c;
      } else {
        if (c == -1.0) {
          os << "- ";
        } else if (c != 1.0) {
          if (c < 0.0) {
            os << "- " << std::abs(c) << " ";
          } else {
            os << c << " ";
          }
        }
        bool var_first = true;
        for (const auto& [idx, pow] : m.vars) {
          if (!var_first) os << " ";
          os << var_names[idx];
          if (pow > 1) os << "^" << pow;
          var_first = false;
        }
      }
      first = false;
    } else {
      if (m.vars.empty()) {
        os << (c >= 0.0 ? " + " : " - ") << std::abs(c);
      } else {
        const double ac = std::abs(c);
        os << (c >= 0.0 ? " + " : " - ");
        if (ac != 1.0) {
          os << ac << " ";
        }
        bool var_first = true;
        for (const auto& [idx, pow] : m.vars) {
          if (!var_first) os << " ";
          os << var_names[idx];
          if (pow > 1) os << "^" << pow;
          var_first = false;
        }
      }
    }
  }

  if (first) {
    return "0";
  }
  return os.str();
}

bool write_minlp_as_scip_pip(const MINLPModel& prob,
                             const fs::path& pip_path,
                             std::vector<std::string>& var_names,
                             std::string& err,
                             bool& maximize_objective) {
  const auto& nlp = prob.nonlinear_part;
  const int n = static_cast<int>(nlp.vars.size());
  if (!nlp.symbolic_objective) {
    err = "symbolic objective is required for native SCIP MINLP export";
    return false;
  }

  var_names.clear();
  var_names.reserve(n);
  for (int i = 0; i < n; ++i) {
    var_names.push_back(sanitize_var_name(nlp.vars[i].name, i));
  }

  std::ofstream out(pip_path);
  if (!out.is_open()) {
    err = "failed to open temporary PIP file";
    return false;
  }

  maximize_objective = (nlp.sense == Sense::Maximize);

  Polynomial obj_poly;
  if (!expand_to_polynomial(nlp.symbolic_objective, obj_poly, err)) {
    return false;
  }
  obj_poly = poly_simplify(obj_poly);
  if (maximize_objective) {
    obj_poly = poly_negate(obj_poly);
  }

  out << "Minimize\n";
  out << " obj: " << poly_to_pip(obj_poly, var_names) << "\n";

  out << "Subject To\n";
  int cid = 1;
  for (const auto& c : nlp.symbolic_constraints) {
    Polynomial cpoly;
    if (!expand_to_polynomial(c.expr, cpoly, err)) {
      err = "constraint conversion failed: " + err;
      return false;
    }
    cpoly = poly_simplify(cpoly);

    // Separate constant from non-constant terms: move constant to RHS.
    double lhs_constant = 0.0;
    Polynomial lhs_terms;
    for (const auto& m : cpoly) {
      if (m.vars.empty()) {
        lhs_constant += m.coeff;
      } else {
        lhs_terms.push_back(m);
      }
    }
    const double rhs = c.rhs - lhs_constant;

    const std::string cname = c.name.empty() ? ("c" + std::to_string(cid++)) : c.name;
    out << " " << cname << ": " << poly_to_pip(lhs_terms, var_names);

    switch (c.sense) {
      case SymbolicSense::LessEqual:    out << " <= " << rhs; break;
      case SymbolicSense::GreaterEqual: out << " >= " << rhs; break;
      case SymbolicSense::Equal:        out << " = " << rhs;  break;
    }
    out << "\n";
  }

  out << "Bounds\n";
  for (int i = 0; i < n; ++i) {
    out << " " << nlp.vars[i].lb << " <= " << var_names[i] << " <= " << nlp.vars[i].ub << "\n";
  }

  if (!prob.integer_idx.empty()) {
    out << "General\n";
    for (int idx : prob.integer_idx) {
      out << " " << var_names[idx] << "\n";
    }
  }
  if (!prob.binary_idx.empty()) {
    out << "Binary\n";
    for (int idx : prob.binary_idx) {
      out << " " << var_names[idx] << "\n";
    }
  }

  out << "End\n";
  return true;
}

#ifndef HACDCPF_HAVE_SCIP_LIB
struct ScipSolution {
  bool success{false};
  double objective{0.0};
  std::unordered_map<std::string, double> values;
  std::string status;
};

ScipSolution parse_scip_solution(const fs::path& sol_path) {
  ScipSolution s;
  std::ifstream in(sol_path);
  if (!in.is_open()) {
    s.status = "solution file missing";
    return s;
  }

  std::string line;
  while (std::getline(in, line)) {
    const std::string t = trim(line);
    if (t.rfind("solution status:", 0) == 0) {
      s.status = trim(t.substr(std::string("solution status:").size()));
      std::string l = s.status;
      std::transform(l.begin(), l.end(), l.begin(),
                     [](unsigned char c) { return std::tolower(c); });
      s.success = (l.find("optimal") != std::string::npos ||
                   l.find("feasible") != std::string::npos);
    } else if (t.rfind("objective value:", 0) == 0) {
      const std::string v = trim(t.substr(std::string("objective value:").size()));
      try {
        s.objective = std::stod(v);
      } catch (const std::exception&) {
      }
    } else if (!t.empty() && t.find(':') == std::string::npos) {
      std::istringstream iss(t);
      std::string name;
      double val = 0.0;
      if (iss >> name >> val) {
        s.values[name] = val;
      }
    }
  }

  return s;
}
#endif

#ifdef HACDCPF_HAVE_IPOPT
class CallbackTNLP final : public Ipopt::TNLP {
 public:
  explicit CallbackTNLP(const NLPModel& prob)
      : prob_(prob),
        n_(static_cast<int>(prob.vars.size())),
        meq_(0),
        mineq_(0),
        m_(0),
        iters_(0),
        primal_inf_(0.0),
        dual_inf_(0.0),
        complementarity_(0.0),
        objective_(0.0) {
    if (prob_.g) {
      Eigen::VectorXd geq;
      prob_.g(prob_.x0, geq);
      meq_ = static_cast<int>(geq.size());
    }
    if (prob_.h) {
      Eigen::VectorXd h;
      prob_.h(prob_.x0, h);
      mineq_ = static_cast<int>(h.size());
    }
    m_ = meq_ + mineq_;

    nnz_jac_ = 0;
    // Sparse Jacobian: detect the structural sparsity pattern by evaluating the
    // Jacobian callbacks at a probe point with all variables shifted away from
    // the flat-start. The AML jac_g/jac_h lambdas now emit ALL symbolic
    // nonzeros (including structurally nonzero entries that are zero at flat
    // start), so the probe detects the complete pattern reliably.
    //
    // Replacing nnz_jac_ = m_ * n_ with the true structural nnz reduces
    // memory and compute per Ipopt iteration from O(m*n) → O(nnz) ≈ O(n)
    // for sparse power networks — the main scalability bottleneck for large cases.
    {
      // Probe point: shift each variable by 0.05*(i+1), clamped to bounds.
      Eigen::VectorXd x_probe(n_);
      for (int i = 0; i < n_; ++i) {
        double lo = prob_.vars[static_cast<std::size_t>(i)].lb;
        double hi = prob_.vars[static_cast<std::size_t>(i)].ub;
        double v  = prob_.x0.size() == n_ ? prob_.x0[i]
                                           : 0.5 * (lo + hi);
        v += 0.05 * (i + 1);
        x_probe[i] = std::min(hi, std::max(lo, v));
      }

      auto collect = [&](const auto& jac_cb, int row_offset) {
        Eigen::SparseMatrix<double> J;
        jac_cb(x_probe, J);
        J.makeCompressed();
        for (int k = 0; k < J.outerSize(); ++k)
          for (Eigen::SparseMatrix<double>::InnerIterator it(J, k); it; ++it) {
            jac_rows_.push_back(row_offset + static_cast<int>(it.row()));
            jac_cols_.push_back(static_cast<int>(it.col()));
          }
      };

      if (prob_.jac_g) collect(prob_.jac_g, 0);
      if (prob_.jac_h) collect(prob_.jac_h, meq_);

      nnz_jac_ = static_cast<int>(jac_rows_.size());
    }
    // Ipopt requires nnz_jac_g >= 1 even for unconstrained problems.
    if (nnz_jac_ == 0) nnz_jac_ = 1;
  }

  bool get_nlp_info(Ipopt::Index& n,
                    Ipopt::Index& m,
                    Ipopt::Index& nnz_jac_g,
                    Ipopt::Index& nnz_h_lag,
                    Ipopt::TNLP::IndexStyleEnum& index_style) override {
    n = n_;
    m = m_;
    nnz_jac_g = nnz_jac_;
    nnz_h_lag = 0;
    index_style = Ipopt::TNLP::C_STYLE;
    return true;
  }

  bool get_bounds_info(Ipopt::Index n,
                       Ipopt::Number* x_l,
                       Ipopt::Number* x_u,
                       Ipopt::Index m,
                       Ipopt::Number* g_l,
                       Ipopt::Number* g_u) override {
    if (n != n_ || m != m_) {
      return false;
    }

    for (int i = 0; i < n_; ++i) {
      x_l[i] = prob_.vars[i].lb;
      x_u[i] = prob_.vars[i].ub;
    }

    for (int i = 0; i < meq_; ++i) {
      g_l[i] = 0.0;
      g_u[i] = 0.0;
    }
    for (int i = 0; i < mineq_; ++i) {
      g_l[meq_ + i] = -1e19;
      g_u[meq_ + i] = 0.0;
    }
    return true;
  }

  bool get_starting_point(Ipopt::Index n,
                          bool init_x,
                          Ipopt::Number* x,
                          bool init_z,
                          Ipopt::Number* z_L,
                          Ipopt::Number* z_U,
                          Ipopt::Index m,
                          bool init_lambda,
                          Ipopt::Number* lambda) override {
    if (!init_x || n != n_ || x == nullptr) {
      return false;
    }

    for (int i = 0; i < n_; ++i) {
      x[i] = std::min(prob_.vars[i].ub, std::max(prob_.vars[i].lb, prob_.x0[i]));
    }

    if (init_z) {
      if (z_L == nullptr || z_U == nullptr) {
        return false;
      }
      for (int i = 0; i < n_; ++i) {
        z_L[i] = 0.0;
        z_U[i] = 0.0;
      }
    }

    if (init_lambda) {
      if (lambda == nullptr || m != m_) {
        return false;
      }
      for (int i = 0; i < m_; ++i) {
        lambda[i] = 0.0;
      }
    }
    return true;
  }

  bool eval_f(Ipopt::Index n,
              const Ipopt::Number* x,
              bool new_x,
              Ipopt::Number& obj_value) override {
    (void)new_x;
    if (n != n_ || x == nullptr) {
      return false;
    }
    Eigen::Map<const Eigen::VectorXd> xv(x, n_);
    obj_value = prob_.f(xv);
    return std::isfinite(obj_value);
  }

  bool eval_grad_f(Ipopt::Index n,
                   const Ipopt::Number* x,
                   bool new_x,
                   Ipopt::Number* grad_f) override {
    (void)new_x;
    if (n != n_ || x == nullptr || grad_f == nullptr) {
      return false;
    }
    Eigen::Map<const Eigen::VectorXd> xv(x, n_);
    Eigen::VectorXd g;
    prob_.grad(xv, g);
    if (g.size() != n_) {
      return false;
    }
    for (int i = 0; i < n_; ++i) {
      grad_f[i] = g[i];
    }
    return true;
  }

  bool eval_g(Ipopt::Index n,
              const Ipopt::Number* x,
              bool new_x,
              Ipopt::Index m,
              Ipopt::Number* g) override {
    (void)new_x;
    if (n != n_ || m != m_ || x == nullptr || (m_ > 0 && g == nullptr)) {
      return false;
    }

    Eigen::Map<const Eigen::VectorXd> xv(x, n_);
    if (meq_ > 0) {
      Eigen::VectorXd geq;
      prob_.g(xv, geq);
      if (geq.size() != meq_) {
        return false;
      }
      for (int i = 0; i < meq_; ++i) {
        g[i] = geq[i];
      }
    }
    if (mineq_ > 0) {
      Eigen::VectorXd h;
      prob_.h(xv, h);
      if (h.size() != mineq_) {
        return false;
      }
      for (int i = 0; i < mineq_; ++i) {
        g[meq_ + i] = h[i];
      }
    }
    return true;
  }

  bool eval_jac_g(Ipopt::Index n,
                  const Ipopt::Number* x,
                  bool new_x,
                  Ipopt::Index m,
                  Ipopt::Index nele_jac,
                  Ipopt::Index* iRow,
                  Ipopt::Index* jCol,
                  Ipopt::Number* values) override {
    (void)new_x;
    if (n != n_ || m != m_) {
      return false;
    }
    int cursor = 0;

    if (values == nullptr) {
      // Structure pass: emit the pre-computed sparse pattern.
      if (nele_jac > 0 && (iRow == nullptr || jCol == nullptr)) {
        return false;
      }
      for (int k = 0; k < static_cast<int>(jac_rows_.size()); ++k) {
        iRow[cursor] = jac_rows_[static_cast<std::size_t>(k)];
        jCol[cursor] = jac_cols_[static_cast<std::size_t>(k)];
        ++cursor;
      }
    } else {
      // Value pass: evaluate sparse Jacobian and fill in the same (row,col) order.
      if (x == nullptr) {
        return false;
      }
      Eigen::Map<const Eigen::VectorXd> xv(x, n_);

      // Build a map (row,col) → index in jac_rows_ for lookup.
      // For efficiency, materialise both sparse Jacobians and walk them.
      // Use a flat parallel value array: values_flat[k] corresponds to
      // (jac_rows_[k], jac_cols_[k]).
      //
      // Strategy: build sparse matrices, convert to a lookup dictionary,
      // then fill the values array in declared order.
      // For typical AC OPF nnz << m*n this is O(nnz) rather than O(m*n).
      std::vector<double> sparse_vals(static_cast<std::size_t>(nnz_jac_), 0.0);

      // Helper: accumulate from a sparse Jacobian block into sparse_vals.
      // `block_offset` is the row offset for this block in the global pattern.
      // We need to match (row+block_offset, col) → position in jac_rows_/jac_cols_.
      // Since jac_rows_/jac_cols_ are stored in the same order as produced by
      // the pattern probe (compressed column-major), we can use binary search
      // or a hash map. Use a small hash map keyed by encoded (row,col).
      auto fill_block = [&](const Eigen::SparseMatrix<double>& J, int row_offset) {
        for (int k = 0; k < J.outerSize(); ++k)
          for (Eigen::SparseMatrix<double>::InnerIterator it(J, k); it; ++it) {
            const int gr = row_offset + static_cast<int>(it.row());
            const int gc = static_cast<int>(it.col());
            // Find the position in jac_rows_/jac_cols_ — linear scan over
            // the nnz entries (fast because nnz is small for sparse networks).
            // The probe and value passes produce the same column-major ordering,
            // so we can match by the same traversal order.
            for (int p = 0; p < nnz_jac_; ++p) {
              if (jac_rows_[static_cast<std::size_t>(p)] == gr &&
                  jac_cols_[static_cast<std::size_t>(p)] == gc) {
                sparse_vals[static_cast<std::size_t>(p)] = it.value();
                break;
              }
            }
          }
      };

      if (meq_ > 0 && prob_.jac_g) {
        Eigen::SparseMatrix<double> jg;
        prob_.jac_g(xv, jg);
        fill_block(jg, 0);
      }
      if (mineq_ > 0 && prob_.jac_h) {
        Eigen::SparseMatrix<double> jh;
        prob_.jac_h(xv, jh);
        fill_block(jh, meq_);
      }

      for (int k = 0; k < nnz_jac_; ++k) {
        values[cursor++] = sparse_vals[static_cast<std::size_t>(k)];
      }
    }

    return cursor == nele_jac;
  }

  bool eval_h(Ipopt::Index n,
              const Ipopt::Number* x,
              bool new_x,
              Ipopt::Number obj_factor,
              Ipopt::Index m,
              const Ipopt::Number* lambda,
              bool new_lambda,
              Ipopt::Index nele_hess,
              Ipopt::Index* iRow,
              Ipopt::Index* jCol,
              Ipopt::Number* values) override {
    (void)n;
    (void)x;
    (void)new_x;
    (void)obj_factor;
    (void)m;
    (void)lambda;
    (void)new_lambda;
    (void)nele_hess;
    (void)iRow;
    (void)jCol;
    (void)values;
    return true;
  }

  void finalize_solution(Ipopt::SolverReturn status,
                         Ipopt::Index n,
                         const Ipopt::Number* x,
                         const Ipopt::Number* z_L,
                         const Ipopt::Number* z_U,
                         Ipopt::Index m,
                         const Ipopt::Number* g,
                         const Ipopt::Number* lambda,
                         Ipopt::Number obj_value,
                         const Ipopt::IpoptData* ip_data,
                         Ipopt::IpoptCalculatedQuantities* ip_cq) override {
    (void)z_L;
    (void)z_U;
    (void)m;
    (void)g;
    (void)lambda;
    (void)ip_data;
    (void)ip_cq;
    (void)status;
    if (x != nullptr && n > 0) {
      solution_ = Eigen::Map<const Eigen::VectorXd>(x, n);
    } else {
      solution_ = Eigen::VectorXd::Zero(n_);
    }
    objective_ = obj_value;
  }

  bool intermediate_callback(Ipopt::AlgorithmMode mode,
                             Ipopt::Index iter,
                             Ipopt::Number obj_value,
                             Ipopt::Number inf_pr,
                             Ipopt::Number inf_du,
                             Ipopt::Number mu,
                             Ipopt::Number d_norm,
                             Ipopt::Number regularization_size,
                             Ipopt::Number alpha_du,
                             Ipopt::Number alpha_pr,
                             Ipopt::Index ls_trials,
                             const Ipopt::IpoptData* ip_data,
                             Ipopt::IpoptCalculatedQuantities* ip_cq) override {
    (void)mode;
    (void)obj_value;
    (void)d_norm;
    (void)regularization_size;
    (void)alpha_du;
    (void)alpha_pr;
    (void)ls_trials;
    (void)ip_data;
    (void)ip_cq;
    iters_ = static_cast<int>(iter);
    primal_inf_ = inf_pr;
    dual_inf_ = inf_du;
    complementarity_ = mu;
    return true;
  }

  SolveResult build_result(Ipopt::ApplicationReturnStatus app_status) const {
    SolveResult out;
    out.x = solution_;
    out.stats.objective = objective_;
    out.stats.iterations = iters_;
    out.stats.primal_feas = primal_inf_;
    out.stats.dual_feas = dual_inf_;
    out.stats.complementarity = complementarity_;
    out.stats.residual_inf = std::max(primal_inf_, dual_inf_);

    switch (app_status) {
      case Ipopt::Solve_Succeeded:
      case Ipopt::Solved_To_Acceptable_Level:
      case Ipopt::Feasible_Point_Found:
        out.stats.success = true;
        out.stats.status = "Converged";
        break;
      case Ipopt::Maximum_Iterations_Exceeded:
        out.stats.success = false;
        out.stats.status = "Max iterations exceeded";
        break;
      default:
        out.stats.success = false;
        out.stats.status = "Ipopt failed";
        break;
    }
    return out;
  }

 private:
  const NLPModel& prob_;
  int n_;
  int meq_;
  int mineq_;
  int m_;
  int nnz_jac_;
  std::vector<int> jac_rows_;   ///< Sparse Jacobian row indices (structural pattern)
  std::vector<int> jac_cols_;   ///< Sparse Jacobian column indices (structural pattern)

  Eigen::VectorXd solution_;
  int iters_;
  double primal_inf_;
  double dual_inf_;
  double complementarity_;
  double objective_;
};
#endif

}  // namespace

HighsAdapter::HighsAdapter(std::string executable)
    : executable_(resolve_explicit_executable(executable)) {}

std::string HighsAdapter::name() const {
  return "HiGHS";
}

bool HighsAdapter::supports(ProblemClass cls) const {
  return cls == ProblemClass::LP || cls == ProblemClass::MILP;
}

bool HighsAdapter::available() const {
#ifdef HACDCPF_HAVE_HIGHS_LIB
  return true;
#else
  return !executable_.empty();
#endif
}

const std::string& HighsAdapter::executable() const {
  return executable_;
}

SolveResult HighsAdapter::solve_lp(const LPModel& prob) const {
  const auto t0 = std::chrono::steady_clock::now();
#ifdef HACDCPF_HAVE_HIGHS_LIB
  if (auto embedded = solve_lp_with_embedded_highs(prob, false, name(), nullptr)) {
    return *embedded;
  }
#endif
  if (!available()) {
    return unavailable_result(name(), "embedded HiGHS library not compiled");
  }

  const ValidationReport vr = validate(prob);
  if (!vr.valid) {
    return unavailable_result(name(), vr.errors.empty() ? "invalid LP model" : vr.errors.front());
  }

  const auto stamp = std::to_string(
      std::chrono::duration_cast<std::chrono::nanoseconds>(t0.time_since_epoch()).count());
  const fs::path mps_path = fs::temp_directory_path() / ("mipsolvers_lp_" + stamp + ".mps");
  const fs::path sol_path = fs::temp_directory_path() / ("mipsolvers_lp_" + stamp + ".sol");
  const fs::path opt_path = fs::temp_directory_path() / ("mipsolvers_lp_" + stamp + ".opt");
  const fs::path log_path = fs::temp_directory_path() / ("mipsolvers_lp_" + stamp + ".log");

  SolveResult out;
  out.stats.solver_name = name();

  if (!write_lp_as_mps(prob, mps_path, false)) {
    out.stats.status = "Unavailable: failed to write MPS";
    return out;
  }

  {
    std::ofstream ofs(opt_path);
    ofs << "log_file = " << log_path.string() << "\n";
  }

  const std::string cmd = shell_quote(executable_) + " --model_file " + shell_quote(mps_path) +
                          " --solution_file " + shell_quote(sol_path) +
                          " --options_file " + shell_quote(opt_path) +
                          (env_flag_enabled("HIGHS_XROW_TRACE") ||
                                   env_flag_enabled("HACDCPF_XTAB_ROW_TRACE")
                               ? " >> " + shell_quote(log_path) + " 2>&1"
                   : std::string(" > ") + shell_null_device() + " 2>&1");

  const int rc = std::system(cmd.c_str());
  const HighsRunReport run_report = parse_highs_run_report(log_path);
  auto model_status = parse_highs_model_status(sol_path);
  if (!model_status && run_report.status) model_status = run_report.status;
  const bool solved = model_status.has_value() &&
                      (*model_status == "Optimal" || *model_status == "Feasible");
  if (!solved && rc != 0) {
    out.stats.status = "HiGHS process failed";
    std::error_code ec;
    fs::remove(mps_path, ec);
    fs::remove(sol_path, ec);
    fs::remove(opt_path, ec);
    fs::remove(log_path, ec);
    return out;
  }

  out.stats.success = solved;
  out.stats.status = solved ? ("HiGHS " + *model_status)
                            : "HiGHS finished without solution";
  if (const auto obj = parse_highs_solution_objective(sol_path)) {
    out.stats.objective = *obj;
  } else if (run_report.primal_bound) {
    out.stats.objective = *run_report.primal_bound;
  }
  if (run_report.gap) {
    out.stats.mip_gap = *run_report.gap;
  }

  // Parse variable values from solution file (X1..Xn, 1-indexed)
  if (solved) {
    const int n = static_cast<int>(prob.vars.size());
    const auto vals = parse_highs_solution_values(sol_path);
    out.x = Eigen::VectorXd::Zero(n);
    for (int i = 0; i < n; ++i) {
      auto it = vals.find("X" + std::to_string(i + 1));
      if (it != vals.end()) out.x[i] = it->second;
    }
  }

  const auto t1 = std::chrono::steady_clock::now();
  out.stats.runtime_sec = std::chrono::duration<double>(t1 - t0).count();

  relay_highs_conformance_lines(log_path);

  std::error_code ec;
  fs::remove(mps_path, ec);
  fs::remove(sol_path, ec);
  fs::remove(opt_path, ec);
  fs::remove(log_path, ec);
  return out;
}

SolveResult HighsAdapter::solve_milp(const MIPModel& prob) const {
  const auto t0 = std::chrono::steady_clock::now();
  const ValidationReport vr = validate(prob);
  if (!vr.valid) {
    return unavailable_result(name(), vr.errors.empty() ? "invalid MILP model" : vr.errors.front());
  }

  LPModel lp = prob.linear_part;
  for (int idx : prob.integer_idx) {
    lp.vars[idx].type = VarType::Integer;
  }
  for (int idx : prob.binary_idx) {
    lp.vars[idx].type = VarType::Binary;
    lp.vars[idx].lb = std::max(0.0, lp.vars[idx].lb);
    lp.vars[idx].ub = std::min(1.0, lp.vars[idx].ub);
  }

#ifdef HACDCPF_HAVE_HIGHS_LIB
  // The native B&C solver may have already initialized HiGHS's global thread
  // scheduler (with num_threads > 1).  Resetting it before constructing a new
  // Highs instance allows solve_lp_with_embedded_highs to set its own thread
  // count without triggering the "scheduler already initialized" error.
  Highs::resetGlobalScheduler(/*blocking=*/true);
  const Eigen::VectorXd* mip_start =
      prob.initial_solution.size() == static_cast<int>(lp.vars.size())
          ? &prob.initial_solution
          : nullptr;
  if (auto embedded = solve_lp_with_embedded_highs(lp, true, name(), mip_start)) {
    return *embedded;
  }
#endif
  if (!available()) {
    return unavailable_result(name(), "embedded HiGHS library not compiled");
  }

  SolveResult out;
  out.stats.solver_name = name();

  const auto stamp = std::to_string(
      std::chrono::duration_cast<std::chrono::nanoseconds>(t0.time_since_epoch()).count());
  const fs::path mps_path = fs::temp_directory_path() / ("mipsolvers_milp_" + stamp + ".mps");
  const fs::path sol_path = fs::temp_directory_path() / ("mipsolvers_milp_" + stamp + ".sol");
  const fs::path opt_path = fs::temp_directory_path() / ("mipsolvers_milp_" + stamp + ".opt");
  const fs::path log_path = fs::temp_directory_path() / ("mipsolvers_milp_" + stamp + ".log");

  if (!write_lp_as_mps(lp, mps_path, true)) {
    out.stats.status = "Unavailable: failed to write MPS";
    return out;
  }

  // Write options file to set MIP gap tolerance (not available as CLI flag).
  {
    std::ofstream ofs(opt_path);
    ofs << "mip_rel_gap = 1e-4\n";
    ofs << "log_file = " << log_path.string() << "\n";
    // Optional: enable HiGHS' built-in MIP timer (per-clock report) so we can
    // compare phase-by-phase against the native B&C diagnostics. Activated
    // by HACDCPF_HIGHS_ANALYSIS=<level> (e.g. 128 for kHighsAnalysisLevelMipTime).
    if (const char* env_lvl = std::getenv("HACDCPF_HIGHS_ANALYSIS")) {
      if (env_lvl[0] != '\0') ofs << "highs_analysis_level = " << env_lvl << "\n";
    }
  }

  // When HACDCPF_HIGHS_TIMELINE/CONF/SEP are on we want HiGHS' stderr/stdout
  // captured so the relay can pick those tagged lines up.
  const bool capture_log =
      env_flag_enabled("HIGHS_XROW_TRACE") ||
      env_flag_enabled("HACDCPF_XTAB_ROW_TRACE") ||
      env_flag_enabled("HACDCPF_HIGHS_TIMELINE") ||
      env_flag_enabled("HACDCPF_HIGHS_FRONTIER_CONFORM") ||
      env_flag_enabled("HACDCPF_HIGHS_CONF") ||
      std::getenv("HACDCPF_HIGHS_ANALYSIS") != nullptr;

  const std::string cmd = shell_quote(executable_) + " --model_file " + shell_quote(mps_path) +
                          " --solution_file " + shell_quote(sol_path) +
                          " --options_file " + shell_quote(opt_path) +
                          (capture_log
                               ? " >> " + shell_quote(log_path) + " 2>&1"
               : std::string(" > ") + shell_null_device() + " 2>&1");

  const int rc = std::system(cmd.c_str());
  const HighsRunReport run_report = parse_highs_run_report(log_path);
  auto model_status = parse_highs_model_status(sol_path);
  if (!model_status && run_report.status) model_status = run_report.status;
  const bool solved = model_status.has_value() &&
                      (*model_status == "Optimal" || *model_status == "Feasible");
  if (!solved && rc != 0) {
    out.stats.status = "HiGHS process failed";
    std::error_code ec;
    fs::remove(mps_path, ec);
    fs::remove(sol_path, ec);
    fs::remove(opt_path, ec);
    fs::remove(log_path, ec);
    return out;
  }

  out.stats.success = solved;
  out.stats.status = solved ? ("HiGHS " + *model_status)
                            : "HiGHS finished without solution";
  if (const auto obj = parse_highs_solution_objective(sol_path)) {
    out.stats.objective = *obj;
  } else if (run_report.primal_bound) {
    out.stats.objective = *run_report.primal_bound;
  }
  if (run_report.gap) {
    out.stats.mip_gap = *run_report.gap;
  }

  // Parse variable values from solution file (X1..Xn, 1-indexed)
  if (solved) {
    const int n = static_cast<int>(lp.vars.size());
    const auto vals = parse_highs_solution_values(sol_path);
    out.x = Eigen::VectorXd::Zero(n);
    for (int i = 0; i < n; ++i) {
      auto it = vals.find("X" + std::to_string(i + 1));
      if (it != vals.end()) out.x[i] = it->second;
    }
  }

  const auto t1 = std::chrono::steady_clock::now();
  out.stats.runtime_sec = std::chrono::duration<double>(t1 - t0).count();

  relay_highs_conformance_lines(log_path);

  std::error_code ec;
  fs::remove(mps_path, ec);
  fs::remove(sol_path, ec);
  fs::remove(opt_path, ec);
  fs::remove(log_path, ec);
  return out;
}

IpoptAdapter::IpoptAdapter(std::string executable)
    : executable_(resolve_explicit_executable(executable)) {}

ScipAdapter::ScipAdapter(std::string executable)
  : executable_(resolve_explicit_executable(executable)) {}

std::string IpoptAdapter::name() const {
  return "Ipopt";
}

std::string ScipAdapter::name() const {
  return "SCIP";
}

bool IpoptAdapter::supports(ProblemClass cls) const {
  return cls == ProblemClass::NLP;
}

bool ScipAdapter::supports(ProblemClass cls) const {
  return cls == ProblemClass::MINLP || cls == ProblemClass::MILP;
}

bool IpoptAdapter::available() const {
#ifdef HACDCPF_HAVE_IPOPT
  return true;
#else
  return !executable_.empty();
#endif
}

const std::string& IpoptAdapter::executable() const {
  return executable_;
}

bool ScipAdapter::available() const {
#ifdef HACDCPF_HAVE_SCIP_LIB
  return true;
#else
  return !executable_.empty();
#endif
}

const std::string& ScipAdapter::executable() const {
  return executable_;
}

SolveResult ScipAdapter::solve_milp(const MIPModel& prob) const {
  const auto t0 = std::chrono::steady_clock::now();
  SolveResult out;
  out.stats.solver_name = name();

  const ValidationReport vr = validate(prob);
  if (!vr.valid) {
    return unavailable_result(name(), vr.errors.empty() ? "invalid MILP model"
                                                        : vr.errors.front());
  }
  if (!available()) {
    return unavailable_result(name(), "embedded SCIP library not compiled");
  }

  // Promote integrality onto the linear part so the MPS export carries the
  // INTORG/INTEND markers SCIP reads.  Mirrors HighsAdapter::solve_milp().
  LPModel lp = prob.linear_part;
  for (int idx : prob.integer_idx) {
    lp.vars[idx].type = VarType::Integer;
  }
  for (int idx : prob.binary_idx) {
    lp.vars[idx].type = VarType::Binary;
    lp.vars[idx].lb = std::max(0.0, lp.vars[idx].lb);
    lp.vars[idx].ub = std::min(1.0, lp.vars[idx].ub);
  }
  const int n = static_cast<int>(lp.vars.size());

  std::error_code ec;
  const auto stamp = std::to_string(
      std::chrono::duration_cast<std::chrono::nanoseconds>(t0.time_since_epoch()).count());
  const fs::path mps_path =
      fs::temp_directory_path() / ("mipsolvers_scip_milp_" + stamp + ".mps");
  const fs::path sol_path =
      fs::temp_directory_path() / ("mipsolvers_scip_milp_" + stamp + ".sol");

  // write_lp_as_mps names columns X1..Xn (1-indexed); the solution is mapped
  // back through that convention below.
  if (!write_lp_as_mps(lp, mps_path, true)) {
    out.stats.status = "Unavailable: failed to write MPS for SCIP";
    return out;
  }

#ifdef HACDCPF_HAVE_SCIP_LIB
  // ── In-process SCIP via libscip ────────────────────────────────────────────
  SCIP* scip_env = nullptr;
  SCIP_RETCODE scip_rc = SCIPcreate(&scip_env);
  if (scip_rc != SCIP_OKAY || scip_env == nullptr) {
    out.stats.status =
        "SCIPcreate failed (rc=" + std::to_string(static_cast<int>(scip_rc)) + ")";
    fs::remove(mps_path, ec);
    out.stats.runtime_sec =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    return out;
  }
  SCIPincludeDefaultPlugins(scip_env);
  SCIPsetIntParam(scip_env, "display/verblevel", 0);
  // 0.1% relative gap matches the native B&C / HiGHS UC tolerance; the time cap
  // guards against pathological instances while still returning any incumbent.
  SCIPsetRealParam(scip_env, "limits/gap", 1e-3);
  SCIPsetRealParam(scip_env, "limits/time", 300.0);

  scip_rc = SCIPreadProb(scip_env, mps_path.string().c_str(), nullptr);
  if (scip_rc != SCIP_OKAY) {
    SCIPfree(&scip_env);
    out.stats.status =
        "SCIPreadProb(MPS) failed (rc=" + std::to_string(static_cast<int>(scip_rc)) + ")";
    fs::remove(mps_path, ec);
    out.stats.runtime_sec =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    return out;
  }

  SCIPsolve(scip_env);
  const SCIP_STATUS scip_status = SCIPgetStatus(scip_env);
  SCIP_SOL* scip_sol = SCIPgetBestSol(scip_env);
  if (scip_sol != nullptr && SCIPgetNSols(scip_env) > 0) {
    out.stats.success = true;
    out.stats.status = (scip_status == SCIP_STATUS_OPTIMAL) ? "Solved"
                                                            : "Feasible (limit)";
    out.stats.objective = SCIPgetSolOrigObj(scip_env, scip_sol);
    out.stats.mip_gap = SCIPgetGap(scip_env);

    std::unordered_map<std::string, int> name_to_idx;
    name_to_idx.reserve(static_cast<std::size_t>(n));
    for (int i = 0; i < n; ++i) name_to_idx.emplace("X" + std::to_string(i + 1), i);

    // Iterate the ORIGINAL variables (named X1..Xn by write_lp_as_mps).  After
    // presolve, SCIPgetVars() returns transformed/aggregated columns whose names
    // no longer match the MPS export, which would silently zero the solution.
    // SCIPgetBestSol() lives in the original space, so original vars map cleanly.
    const int nvars_scip = SCIPgetNOrigVars(scip_env);
    SCIP_VAR** scip_vars = SCIPgetOrigVars(scip_env);
    out.x = Eigen::VectorXd::Zero(n);
    for (int vi = 0; vi < nvars_scip; ++vi) {
      const char* vname = SCIPvarGetName(scip_vars[vi]);
      if (vname == nullptr) continue;
      auto it = name_to_idx.find(vname);
      if (it != name_to_idx.end()) {
        out.x[it->second] = SCIPgetSolVal(scip_env, scip_sol, scip_vars[vi]);
      }
    }
  } else {
    out.stats.success = false;
    out.stats.status = (scip_status == SCIP_STATUS_INFEASIBLE)
                           ? "Infeasible"
                           : "SCIP finished without solution";
  }

  SCIPfree(&scip_env);
  fs::remove(mps_path, ec);
  out.stats.runtime_sec =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  return out;
#else
  // ── External SCIP subprocess ────────────────────────────────────────────────
  const std::string cmd = shell_quote(executable_) +
                          " -c \"set limits gap 0.001\" -c \"read " + shell_quote(mps_path) +
                          "\" -c \"optimize\" -c \"write solution " + shell_quote(sol_path) +
                          "\" -c \"quit\" > " + shell_null_device() + " 2>&1";
  const int rc = std::system(cmd.c_str());
  const ScipSolution sol = parse_scip_solution(sol_path);
  if (!sol.success && rc != 0) {
    out.stats.status = "SCIP process failed";
    fs::remove(mps_path, ec);
    fs::remove(sol_path, ec);
    out.stats.runtime_sec =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    return out;
  }
  out.stats.success = sol.success;
  out.stats.status = sol.success
                         ? "Solved"
                         : (sol.status.empty() ? "SCIP finished without solution" : sol.status);
  out.stats.objective = sol.objective;
  out.x = Eigen::VectorXd::Zero(n);
  for (int i = 0; i < n; ++i) {
    auto it = sol.values.find("X" + std::to_string(i + 1));
    if (it != sol.values.end()) out.x[i] = it->second;
  }
  fs::remove(mps_path, ec);
  fs::remove(sol_path, ec);
  out.stats.runtime_sec =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  return out;
#endif  // HACDCPF_HAVE_SCIP_LIB
}

SolveResult ScipAdapter::solve_minlp(const MINLPModel& prob_in) const {
  const auto t0 = std::chrono::steady_clock::now();
  SolveResult out;
  out.stats.solver_name = name();

  // Default an absent/wrong-sized nonlinear x0 to a bounds-aware interior
  // point so callers (the AML converter) need not pre-size it; validation and
  // the NLP relaxation both require x0 to match the variable count.
  MINLPModel prob = prob_in;
  {
    const int n_vars = static_cast<int>(prob.nonlinear_part.vars.size());
    if (prob.nonlinear_part.x0.size() != n_vars && n_vars > 0) {
      prob.nonlinear_part.x0 = Eigen::VectorXd::Zero(n_vars);
      for (int i = 0; i < n_vars; ++i) {
        const double lo = prob.nonlinear_part.vars[static_cast<std::size_t>(i)].lb;
        const double hi = prob.nonlinear_part.vars[static_cast<std::size_t>(i)].ub;
        double v = 0.0;
        if (std::isfinite(lo) && std::isfinite(hi)) {
          v = 0.5 * (lo + hi);
        } else if (std::isfinite(lo)) {
          v = lo;
        } else if (std::isfinite(hi)) {
          v = hi;
        }
        prob.nonlinear_part.x0[i] = v;
      }
    }
  }

  const ValidationReport vr = validate(prob);
  if (!vr.valid) {
    out.stats.status = vr.errors.empty() ? "Invalid MINLP model" : vr.errors.front();
    return out;
  }

  if (!available()) {
    out.stats.status = "Unavailable: embedded SCIP library not compiled";
    return out;
  }

  auto solve_relaxation_rounding = [&]() {
    NLPModel relaxed = prob.nonlinear_part;
    for (int idx : prob.integer_idx) {
      relaxed.vars[idx].type = VarType::Continuous;
    }
    for (int idx : prob.binary_idx) {
      relaxed.vars[idx].type = VarType::Continuous;
      relaxed.vars[idx].lb = std::max(0.0, relaxed.vars[idx].lb);
      relaxed.vars[idx].ub = std::min(1.0, relaxed.vars[idx].ub);
    }

    IpoptAdapter nlp_fallback;
    const SolveResult rel = nlp_fallback.solve_nlp(relaxed);
    if (!rel.stats.success || rel.x.size() != static_cast<int>(relaxed.vars.size())) {
      out.stats.status = "MINLP fallback failed: " + rel.stats.status;
      return;
    }

    Eigen::VectorXd x = rel.x;
    for (int idx : prob.integer_idx) {
      x[idx] = std::round(x[idx]);
    }
    for (int idx : prob.binary_idx) {
      x[idx] = (x[idx] >= 0.5) ? 1.0 : 0.0;
    }
    for (int i = 0; i < x.size(); ++i) {
      x[i] = std::min(relaxed.vars[i].ub, std::max(relaxed.vars[i].lb, x[i]));
    }

    double feas = 0.0;
    if (relaxed.g) {
      Eigen::VectorXd geq;
      relaxed.g(x, geq);
      if (geq.size() > 0) {
        feas = std::max(feas, geq.cwiseAbs().maxCoeff());
      }
    }
    if (relaxed.h) {
      Eigen::VectorXd h;
      relaxed.h(x, h);
      for (int i = 0; i < h.size(); ++i) {
        feas = std::max(feas, h[i]);
      }
    }

    out.x = x;
    out.stats.objective = relaxed.f ? relaxed.f(x) : rel.stats.objective;
    out.stats.iterations = rel.stats.iterations;
    out.stats.primal_feas = std::max(0.0, feas);
    out.stats.residual_inf = std::max(out.stats.primal_feas, rel.stats.residual_inf);
    out.stats.success = out.stats.primal_feas <= 1e-4;
    out.stats.status = out.stats.success ? "Solved (NLP-relaxation+rounding)"
                                         : "Infeasible after rounding";
  };

  if (!prob.nonlinear_part.symbolic_objective) {
    solve_relaxation_rounding();
    const auto t1 = std::chrono::steady_clock::now();
    out.stats.runtime_sec = std::chrono::duration<double>(t1 - t0).count();
    return out;
  }

  std::error_code ec;
  const auto stamp = std::to_string(
      std::chrono::duration_cast<std::chrono::nanoseconds>(t0.time_since_epoch()).count());
  const fs::path pip_path = fs::temp_directory_path() / ("mipsolvers_minlp_" + stamp + ".pip");
  const fs::path sol_path = fs::temp_directory_path() / ("mipsolvers_minlp_" + stamp + ".sol");

  std::vector<std::string> var_names;
  bool was_maximize = false;
  std::string export_err;
  if (!write_minlp_as_scip_pip(prob, pip_path, var_names, export_err, was_maximize)) {
    if (prob.nonlinear_part.f && prob.nonlinear_part.grad) {
      solve_relaxation_rounding();
      if (!out.stats.success) {
        out.stats.status = "SCIP export failed: " + export_err + "; " + out.stats.status;
      }
      const auto t1 = std::chrono::steady_clock::now();
      out.stats.runtime_sec = std::chrono::duration<double>(t1 - t0).count();
      return out;
    }
    out.stats.status = "SCIP export failed: " + export_err;
    const auto t1 = std::chrono::steady_clock::now();
    out.stats.runtime_sec = std::chrono::duration<double>(t1 - t0).count();
    return out;
  }

  const std::string cmd = shell_quote(executable_) +
                          " -c \"set limits time 30\" -c \"read " + pip_path.string() +
                          "\" -c \"optimize\" -c \"write solution " + sol_path.string() +
                          "\" -c \"quit\" > " + shell_null_device() + " 2>&1";

#ifdef HACDCPF_HAVE_SCIP_LIB
  // ── In-process SCIP via libscip ────────────────────────────────────────────
  // Load the PIP file into the embedded SCIP library instance, solve in-process,
  // and read back the solution without spawning a subprocess.
  SCIP* scip_env = nullptr;
  SCIP_RETCODE scip_rc = SCIPcreate(&scip_env);
  if (scip_rc != SCIP_OKAY || scip_env == nullptr) {
    out.stats.status = "SCIPcreate failed (rc=" + std::to_string(static_cast<int>(scip_rc)) + ")";
    fs::remove(pip_path, ec);
    const auto t1 = std::chrono::steady_clock::now();
    out.stats.runtime_sec = std::chrono::duration<double>(t1 - t0).count();
    return out;
  }
  SCIPincludeDefaultPlugins(scip_env);
  SCIPsetIntParam(scip_env, "display/verblevel", 0);
  SCIPsetRealParam(scip_env, "limits/time", 30.0);

  scip_rc = SCIPreadProb(scip_env, pip_path.string().c_str(), nullptr);
  if (scip_rc != SCIP_OKAY) {
    SCIPfree(&scip_env);
    out.stats.status = "SCIPreadProb failed (rc=" + std::to_string(static_cast<int>(scip_rc)) + ")";
    fs::remove(pip_path, ec);
    const auto t1 = std::chrono::steady_clock::now();
    out.stats.runtime_sec = std::chrono::duration<double>(t1 - t0).count();
    return out;
  }

  SCIPsolve(scip_env);

  SCIP_STATUS scip_status = SCIPgetStatus(scip_env);
  const bool scip_solved = (scip_status == SCIP_STATUS_OPTIMAL ||
                             scip_status == SCIP_STATUS_TIMELIMIT ||
                             scip_status == SCIP_STATUS_NODELIMIT);
  SCIP_SOL* scip_sol = SCIPgetBestSol(scip_env);

  if (scip_sol != nullptr) {
    out.stats.success = (scip_status == SCIP_STATUS_OPTIMAL ||
                          SCIPgetNSols(scip_env) > 0);
    out.stats.status = (scip_status == SCIP_STATUS_OPTIMAL) ? "Solved"
                     : (scip_solved ? "Feasible (time/node limit)" : "No solution");
    out.stats.objective = was_maximize
                              ? -SCIPgetSolOrigObj(scip_env, scip_sol)
                              : SCIPgetSolOrigObj(scip_env, scip_sol);

    // Map SCIP variable names back to our variable index.
    const int nvars_scip = SCIPgetNVars(scip_env);
    SCIP_VAR** scip_vars = SCIPgetVars(scip_env);
    const int n = static_cast<int>(prob.nonlinear_part.vars.size());
    out.x = Eigen::VectorXd::Zero(n);
    for (int vi = 0; vi < nvars_scip; ++vi) {
      const char* vname = SCIPvarGetName(scip_vars[vi]);
      for (int i = 0; i < n; ++i) {
        if (var_names[static_cast<std::size_t>(i)] == vname) {
          out.x[i] = SCIPgetSolVal(scip_env, scip_sol, scip_vars[vi]);
          break;
        }
      }
    }
  } else {
    out.stats.success = false;
    out.stats.status = (scip_status == SCIP_STATUS_INFEASIBLE) ? "Infeasible"
                       : "SCIP finished without solution";
  }

  SCIPfree(&scip_env);
  fs::remove(pip_path, ec);
  const auto t1_lib = std::chrono::steady_clock::now();
  out.stats.runtime_sec = std::chrono::duration<double>(t1_lib - t0).count();
  return out;
#else
  // ── External SCIP subprocess ────────────────────────────────────────────────
  const int rc = std::system(cmd.c_str());

  const ScipSolution sol = parse_scip_solution(sol_path);
  if (!sol.success && rc != 0) {
    out.stats.status = "SCIP process failed";
    fs::remove(pip_path, ec);
    fs::remove(sol_path, ec);
    const auto t1 = std::chrono::steady_clock::now();
    out.stats.runtime_sec = std::chrono::duration<double>(t1 - t0).count();
    return out;
  }

  out.stats.success = sol.success;
  out.stats.status = sol.success ? "Solved" : (sol.status.empty() ? "SCIP finished without solution"
                                                                     : sol.status);

  const int n = static_cast<int>(prob.nonlinear_part.vars.size());
  out.x = Eigen::VectorXd::Zero(n);
  for (int i = 0; i < n; ++i) {
    auto it = sol.values.find(var_names[i]);
    if (it != sol.values.end()) {
      out.x[i] = it->second;
    }
  }

  out.stats.objective = was_maximize ? -sol.objective : sol.objective;

  const auto t1 = std::chrono::steady_clock::now();
  out.stats.runtime_sec = std::chrono::duration<double>(t1 - t0).count();

  fs::remove(pip_path, ec);
  fs::remove(sol_path, ec);
  return out;
#endif  // HACDCPF_HAVE_SCIP_LIB
}

SolveResult IpoptAdapter::solve_nlp(const NLPModel& prob_in) const {
#ifdef HACDCPF_HAVE_IPOPT
  const auto t0 = std::chrono::steady_clock::now();
#endif
  SolveResult out;
  out.stats.solver_name = name();

  // Ensure an initial point sized to the variable count. Validation and the
  // constraint callbacks (g/h, evaluated at x0 during CallbackTNLP
  // construction) reject a mismatched x0, so default an absent/wrong-sized x0
  // to a bounds-aware interior point here rather than requiring every caller
  // (e.g. the SCIP MINLP relaxation) to supply one.
  const int n_vars = static_cast<int>(prob_in.vars.size());
  NLPModel prob = prob_in;
  if (prob.x0.size() != n_vars && n_vars > 0) {
    prob.x0 = Eigen::VectorXd::Zero(n_vars);
    for (int i = 0; i < n_vars; ++i) {
      const double lo = prob.vars[static_cast<std::size_t>(i)].lb;
      const double hi = prob.vars[static_cast<std::size_t>(i)].ub;
      double v = 0.0;
      if (std::isfinite(lo) && std::isfinite(hi)) {
        v = 0.5 * (lo + hi);
      } else if (std::isfinite(lo)) {
        v = lo;
      } else if (std::isfinite(hi)) {
        v = hi;
      }
      prob.x0[i] = v;
    }
  }

  const ValidationReport vr = validate(prob);
  if (!vr.valid) {
    out.stats.status = vr.errors.empty() ? "Invalid NLP model" : vr.errors.front();
    return out;
  }

#ifdef HACDCPF_HAVE_IPOPT
  if (!prob.f || !prob.grad) {
    out.stats.status = "Unavailable: NLP model missing objective callbacks";
    return out;
  }
  if (prob.g && !prob.jac_g) {
    out.stats.status = "Unavailable: NLP model missing jac_g callback";
    return out;
  }
  if (prob.h && !prob.jac_h) {
    out.stats.status = "Unavailable: NLP model missing jac_h callback";
    return out;
  }

  Ipopt::SmartPtr<Ipopt::TNLP> nlp = new CallbackTNLP(prob);
  Ipopt::SmartPtr<Ipopt::IpoptApplication> app = IpoptApplicationFactory();

  app->Options()->SetIntegerValue("print_level", 0);
  app->Options()->SetStringValue("sb", "yes");
  app->Options()->SetStringValue("hessian_approximation", "limited-memory");
  app->Options()->SetIntegerValue("max_iter", 500);
  app->Options()->SetNumericValue("tol", 1e-8);
  app->Options()->SetNumericValue("acceptable_tol", 1e-6);

  const Ipopt::ApplicationReturnStatus init_status = app->Initialize();
  if (init_status != Ipopt::Solve_Succeeded) {
    out.stats.status = "Ipopt initialization failed";
    return out;
  }

  const Ipopt::ApplicationReturnStatus solve_status = app->OptimizeTNLP(nlp);

  const CallbackTNLP* cb = dynamic_cast<const CallbackTNLP*>(GetRawPtr(nlp));
  if (!cb) {
    out.stats.status = "Ipopt internal callback error";
    return out;
  }
  out = cb->build_result(solve_status);
  out.stats.solver_name = name();

  const auto t1 = std::chrono::steady_clock::now();
  out.stats.runtime_sec = std::chrono::duration<double>(t1 - t0).count();
  return out;
#else
  if (executable_.empty()) {
    out.stats.status = "Unavailable: embedded Ipopt TNLP bridge not compiled";
    return out;
  }

  out.stats.status = "Unavailable: external Ipopt executable adapter is disabled by default; build embedded Ipopt";
  return out;
#endif
}

// ---------------------------------------------------------------------------
// GurobiAdapter — native C API adapter
// ---------------------------------------------------------------------------
#ifdef HACDCPF_HAVE_GUROBI
#include "gurobi_c.h"

namespace {

// Per-solve context passed to the Gurobi MIP callback for dynamic cut injection.
struct GurobiScucCutData {
  int n{0};                              // total variable count
  const MIPModel::UCGenHint* uc{nullptr}; // SCUC structural hints
  double pair_floor{0.05};              // min fractional value to trigger cut check
  int max_cuts_per_call{256};           // cap on cuts injected per MIPNODE event
  int total_cuts_added{0};             // accumulator reported in SolveStats
  bool enable_dynamic_cuts{true};       // false when callback is diagnostics-only
  bool separate_network{false};         // add PTDF rows through lazy/user cuts
  double network_cut_tol{1e-6};
  int max_network_user_cuts_per_call{128};
  int max_network_lazy_cuts_per_call{4096};
  int network_user_cuts_added{0};
  int network_lazy_cuts_added{0};
  std::vector<unsigned char> network_user_cut_seen;
  std::vector<unsigned char> network_lazy_cut_seen;
  bool analyze_gap{false};              // emit one relaxation violation report
  bool analyze_done{false};
  double analyze_gap_trigger{0.014};    // default: sample near the observed 1.35% gap
  int analyze_top{20};
};

struct ScucNetworkCutCandidate {
  int row{-1};
  double violation{0.0};
};

struct ScucRelaxationViolation {
  std::string family;
  double violation{0.0};
  int g{-1};
  int t{-1};
  int aux{-1};
  double lhs{0.0};
  double rhs{0.0};
};

struct ScucRelaxationSummary {
  int count{0};
  double max_violation{0.0};
  double sum_violation{0.0};
};

struct ScucCoverItem {
  int g{-1};
  double weight{0.0};
  double z_value{0.0};
  bool complemented{false};
  bool s_class{false};
};

struct ScucPtdfRowCandidate {
  int line{-1};
  int direction{0};
  double rhs{0.0};
  double slack{0.0};
  std::vector<double> coeff;
};

static int scuc_col(const std::vector<int>& cols, int ng, int T, int g, int t) {
  if (g < 0 || g >= ng || t < 0 || t >= T) return -1;
  const auto pos = static_cast<std::size_t>(t * ng + g);
  return pos < cols.size() ? cols[pos] : -1;
}

static double scuc_val(const std::vector<double>& sol, int col) {
  if (col < 0 || col >= static_cast<int>(sol.size())) return 0.0;
  return sol[static_cast<std::size_t>(col)];
}

static void record_scuc_relaxation_violation(
    std::vector<ScucRelaxationViolation>& top,
    std::map<std::string, ScucRelaxationSummary>& summary,
    const std::string& family,
    double violation,
    int g,
    int t,
    int aux,
    double lhs,
    double rhs) {
  constexpr double kTol = 1e-6;
  if (!(violation > kTol)) return;
  auto& s = summary[family];
  ++s.count;
  s.max_violation = std::max(s.max_violation, violation);
  s.sum_violation += violation;
  top.push_back({family, violation, g, t, aux, lhs, rhs});
}

static void analyze_scuc_relaxation_at_gap(
    const MIPModel::UCGenHint& uc,
    const std::vector<double>& sol,
    double incumbent,
    double bound,
    double rel_gap,
    double runtime,
    double node_count,
    int top_limit) {
  const int ng = uc.ng;
  const int T = uc.T;
  if (ng <= 0 || T <= 0) return;

  std::vector<ScucRelaxationViolation> top;
  std::map<std::string, ScucRelaxationSummary> summary;

  auto get_ig = [&](int g, int t) {
    return scuc_val(sol, scuc_col(uc.ig_cols, ng, T, g, t));
  };
  auto get_su = [&](int g, int t) {
    return scuc_val(sol, scuc_col(uc.su_cols, ng, T, g, t));
  };
  auto get_sd = [&](int g, int t) {
    return scuc_val(sol, scuc_col(uc.sd_cols, ng, T, g, t));
  };
  auto get_pg = [&](int g, int t) {
    return scuc_val(sol, scuc_col(uc.pg_cols, ng, T, g, t));
  };
  auto record = [&](const std::string& family, double lhs, double rhs,
                    int g, int t, int aux = -1) {
    record_scuc_relaxation_violation(top, summary, family, lhs - rhs,
                                     g, t, aux, lhs, rhs);
  };
  auto record_violation = [&](const std::string& family, double violation,
                              int g, int t, int aux, double lhs, double rhs) {
    record_scuc_relaxation_violation(top, summary, family, violation,
                                     g, t, aux, lhs, rhs);
  };
  auto record_fractional = [&](const std::string& family, double value,
                               int g, int t) {
    const double rounded = (value >= 0.5) ? 1.0 : 0.0;
    const double frac = std::abs(value - rounded);
    record_scuc_relaxation_violation(top, summary, family, frac,
                                     g, t, -1, value, rounded);
  };
  auto frac_distance = [](double value) {
    const double rounded = (value >= 0.5) ? 1.0 : 0.0;
    return std::abs(value - rounded);
  };
  auto pmax_tier = [](double pmax) {
    return (pmax > 600.0) ? 6
         : (pmax > 500.0) ? 5
         : (pmax > 400.0) ? 4
         : (pmax > 250.0) ? 3
         : (pmax > 150.0) ? 2
         : (pmax > 100.0) ? 1
         : 0;
  };
  const char* tier_label[7] = {
      "<=100", "100-150", "150-250", "250-400", "400-500", "500-600", ">600"};
  std::vector<double> gen_ig_frac(static_cast<std::size_t>(ng), 0.0);
  std::vector<double> gen_su_frac(static_cast<std::size_t>(ng), 0.0);
  std::vector<double> gen_sd_frac(static_cast<std::size_t>(ng), 0.0);
  std::vector<double> gen_u_sum(static_cast<std::size_t>(ng), 0.0);
  std::vector<double> time_ig_frac(static_cast<std::size_t>(T), 0.0);
  std::vector<double> time_u_sum(static_cast<std::size_t>(T), 0.0);
  std::vector<double> time_pmin_u(static_cast<std::size_t>(T), 0.0);
  std::vector<double> time_pmax_u(static_cast<std::size_t>(T), 0.0);
  int tier_count[7] = {0, 0, 0, 0, 0, 0, 0};
  double tier_ig_frac[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  double tier_u_sum[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  double tier_pmax_u[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  int s_group_id = -1;
  int s_group_size = 0;
  if (uc.identical_group_count > 0 &&
      uc.group_member_start.size() >= static_cast<std::size_t>(uc.identical_group_count + 1) &&
      uc.group_is_s_class.size() >= static_cast<std::size_t>(uc.identical_group_count)) {
    for (int q = 0; q < uc.identical_group_count; ++q) {
      if (uc.group_is_s_class[static_cast<std::size_t>(q)] == 0) continue;
      const int begin = uc.group_member_start[static_cast<std::size_t>(q)];
      const int end = uc.group_member_start[static_cast<std::size_t>(q + 1)];
      const int size = std::max(0, end - begin);
      if (size > s_group_size) {
        s_group_id = q;
        s_group_size = size;
      }
    }
  }

  // Binary fractionality is not a valid cut by itself, but it tells us where
  // the relaxation is escaping the commitment polytope at the sampled gap.
  for (int g = 0; g < ng; ++g) {
    const double pmin = uc.pmin.size() > static_cast<std::size_t>(g)
                            ? std::max(0.0, uc.pmin[static_cast<std::size_t>(g)]) : 0.0;
    const double pmax = uc.pmax.size() > static_cast<std::size_t>(g)
                            ? std::max(0.0, uc.pmax[static_cast<std::size_t>(g)]) : 0.0;
    const int tier = pmax_tier(pmax);
    if (tier >= 0 && tier < 7) ++tier_count[tier];
    for (int t = 0; t < T; ++t) {
      const double u = get_ig(g, t);
      const double su = get_su(g, t);
      const double sd = get_sd(g, t);
      const double ig_frac = frac_distance(u);
      const double su_frac = frac_distance(su);
      const double sd_frac = frac_distance(sd);
      gen_ig_frac[static_cast<std::size_t>(g)] += ig_frac;
      gen_su_frac[static_cast<std::size_t>(g)] += su_frac;
      gen_sd_frac[static_cast<std::size_t>(g)] += sd_frac;
      gen_u_sum[static_cast<std::size_t>(g)] += u;
      time_ig_frac[static_cast<std::size_t>(t)] += ig_frac;
      time_u_sum[static_cast<std::size_t>(t)] += u;
      time_pmin_u[static_cast<std::size_t>(t)] += pmin * u;
      time_pmax_u[static_cast<std::size_t>(t)] += pmax * u;
      if (tier >= 0 && tier < 7) {
        tier_ig_frac[tier] += ig_frac;
        tier_u_sum[tier] += u;
        tier_pmax_u[tier] += pmax * u;
      }
      record_fractional("fractional_IG", u, g, t);
      record_fractional("fractional_SU", su, g, t);
      record_fractional("fractional_SD", sd, g, t);
    }
  }

  // Family T (transition hull), currently disabled for large UC instances.
  if (uc.ig_cols.size() >= static_cast<std::size_t>(ng * T) &&
      uc.su_cols.size() >= static_cast<std::size_t>(ng * T) &&
      uc.sd_cols.size() >= static_cast<std::size_t>(ng * T)) {
    for (int g = 0; g < ng; ++g) {
      const double ig0 =
          (uc.ig0.size() > static_cast<std::size_t>(g) && uc.ig0[static_cast<std::size_t>(g)] != 0)
              ? 1.0 : 0.0;
      for (int t = 0; t < T; ++t) {
        const double u = get_ig(g, t);
        const double su = get_su(g, t);
        const double sd = get_sd(g, t);
        record("T:SU<=IG", su, u, g, t);
        record("T:SD<=1-IG", sd + u, 1.0, g, t);
        if (t == 0) {
          record("T:SU<=1-IG0", su, 1.0 - ig0, g, t);
          record("T:SD<=IG0", sd, ig0, g, t);
        } else {
          const double up = get_ig(g, t - 1);
          record("T:SU+IGprev<=1", su + up, 1.0, g, t);
          record("T:SD<=IGprev", sd, up, g, t);
        }
      }
    }
  }

  // Pairwise SU/SD conflict rows should already be present in the tight Gurobi
  // path.  Keeping them in the report is a quick sanity check that the sampled
  // relaxation is not escaping already-added min-up/down user cuts.
  if (uc.min_up.size() >= static_cast<std::size_t>(ng) &&
      uc.min_down.size() >= static_cast<std::size_t>(ng)) {
    for (int g = 0; g < ng; ++g) {
      const int mu = uc.min_up[static_cast<std::size_t>(g)];
      const int md = uc.min_down[static_cast<std::size_t>(g)];
      if (mu > 1) {
        for (int t1 = 0; t1 < T; ++t1) {
          const double su = get_su(g, t1);
          for (int t2 = t1 + 1; t2 <= std::min(T - 1, t1 + mu - 1); ++t2)
            record("H:SU+SD<=1", su + get_sd(g, t2), 1.0, g, t1, t2);
        }
      }
      if (md > 1) {
        for (int t1 = 0; t1 < T; ++t1) {
          const double sd = get_sd(g, t1);
          for (int t2 = t1 + 1; t2 <= std::min(T - 1, t1 + md - 1); ++t2)
            record("H:SD+SU<=1", sd + get_su(g, t2), 1.0, g, t1, t2);
        }
      }
    }
  }

  // Ramp-perspective and multi-lag startup/shutdown capacity cuts, using the
  // conservative ramp value available in UCGenHint.  These are valid but weaker
  // than a direction-specific test if ramp-up and ramp-down differ.
  if (!uc.pg_cols.empty() && uc.pg_cols.size() >= static_cast<std::size_t>(ng * T) &&
      uc.pmax.size() >= static_cast<std::size_t>(ng) &&
      uc.pmin.size() >= static_cast<std::size_t>(ng) &&
      uc.ramp.size() >= static_cast<std::size_t>(ng)) {
    for (int g = 0; g < ng; ++g) {
      const double pmax = std::max(0.0, uc.pmax[static_cast<std::size_t>(g)]);
      const double pmin = std::max(0.0, uc.pmin[static_cast<std::size_t>(g)]);
      const double ramp = std::min(pmax, std::max(0.0, uc.ramp[static_cast<std::size_t>(g)]));
      if (!(pmax > 1e-9)) continue;
      const double su_cap = std::min(pmax, pmin + ramp);
      const double sd_cap = std::min(pmax, pmin + ramp);
      for (int t = 1; t < T; ++t) {
        const double pg = get_pg(g, t);
        const double pg_prev = get_pg(g, t - 1);
        record("R:up_perspective", pg - pg_prev, ramp * get_ig(g, t - 1) + su_cap * get_su(g, t), g, t);
        record("R:down_perspective", pg_prev - pg, ramp * get_ig(g, t) + sd_cap * get_sd(g, t), g, t);
      }

      const int tu = uc.min_up.size() > static_cast<std::size_t>(g)
                         ? uc.min_up[static_cast<std::size_t>(g)] : 0;
      const int td = uc.min_down.size() > static_cast<std::size_t>(g)
                         ? uc.min_down[static_cast<std::size_t>(g)] : 0;
      for (int k = 1; k <= std::min(std::max(0, tu - 1), 8); ++k) {
        const double excess = pmax - pmin - static_cast<double>(k) * ramp;
        if (!(excess > 1e-9)) break;
        for (int t = k - 1; t < T; ++t) {
          const int su_t = t - k + 1;
          record("P:startup_multilag", get_pg(g, t) + excess * get_su(g, su_t),
                 pmax * get_ig(g, t), g, t, k);
        }
      }
      for (int k = 1; k <= std::min(std::max(0, td - 1), 8); ++k) {
        const double excess = pmax - pmin - static_cast<double>(k) * ramp;
        if (!(excess > 1e-9)) break;
        for (int t = 0; t + k < T; ++t) {
          const int sd_t = t + k;
          record("Q:shutdown_multilag", get_pg(g, t) + excess * get_sd(g, sd_t),
                 pmax * get_ig(g, t), g, t, k);
        }
      }
    }
  }

  // Full-density Family C check.  The production cut currently drops small
  // positive coefficients below 1 MW; violations here measure whether those
  // dropped valid terms still matter at the sampled relaxation.
  if (uc.certifies_hard_network_flow_rows && uc.network_line_count > 0 &&
      !uc.line_gsf.empty() && !uc.line_fwd_rhs.empty() && !uc.line_rev_rhs.empty() &&
      uc.pmin.size() >= static_cast<std::size_t>(ng) &&
      uc.pmax.size() >= static_cast<std::size_t>(ng)) {
    const int nl = uc.network_line_count;
    for (int l = 0; l < nl; ++l) {
      for (int t = 0; t < T; ++t) {
        const double fwd_rhs = uc.line_fwd_rhs[static_cast<std::size_t>(l * T + t)];
        const double rev_rhs = uc.line_rev_rhs[static_cast<std::size_t>(l * T + t)];
        if (fwd_rhs < 1e8) {
          double lhs = 0.0;
          for (int g = 0; g < ng; ++g) {
            const double gsf = uc.line_gsf[static_cast<std::size_t>(l * ng + g)];
            if (std::abs(gsf) < 1e-10) continue;
            const double coeff = (gsf > 0.0 ? gsf * uc.pmin[static_cast<std::size_t>(g)]
                                            : gsf * uc.pmax[static_cast<std::size_t>(g)]);
            lhs += coeff * get_ig(g, t);
          }
          record("C_full:fwd", lhs, fwd_rhs, -1, t, l);
        }
        if (rev_rhs < 1e8) {
          double lhs = 0.0;
          for (int g = 0; g < ng; ++g) {
            const double neg_gsf = -uc.line_gsf[static_cast<std::size_t>(l * ng + g)];
            if (std::abs(neg_gsf) < 1e-10) continue;
            const double coeff = (neg_gsf > 0.0 ? neg_gsf * uc.pmin[static_cast<std::size_t>(g)]
                                                : neg_gsf * uc.pmax[static_cast<std::size_t>(g)]);
            lhs += coeff * get_ig(g, t);
          }
          record("C_full:rev", lhs, rev_rhs, -1, t, l);
        }
      }
    }

    auto scan_s_group_cover = [&](const std::string& family,
                                  const std::vector<double>& row_coeff,
                                  double rhs,
                                  int line,
                                  int t) {
      if (s_group_id < 0 || rhs >= 1e8) return;
      if (uc.identical_group_id.size() < static_cast<std::size_t>(ng)) return;
      double transformed_rhs = rhs;
      for (int g = 0; g < ng; ++g) {
        const double coeff = row_coeff[static_cast<std::size_t>(g)];
        if (coeff < -1e-10) transformed_rhs -= coeff;
      }
      std::vector<ScucCoverItem> items;
      std::vector<ScucCoverItem> s_items;
      items.reserve(static_cast<std::size_t>(ng));
      s_items.reserve(static_cast<std::size_t>(s_group_size));
      for (int g = 0; g < ng; ++g) {
        const bool is_s = uc.identical_group_id[static_cast<std::size_t>(g)] == s_group_id;
        const double coeff = row_coeff[static_cast<std::size_t>(g)];
        if (std::abs(coeff) <= 1e-10) continue;
        ScucCoverItem item;
        item.g = g;
        item.s_class = is_s;
        if (coeff > 0.0) {
          item.weight = coeff;
          item.z_value = get_ig(g, t);
          item.complemented = false;
        } else {
          item.weight = -coeff;
          item.z_value = 1.0 - get_ig(g, t);
          item.complemented = true;
        }
        items.push_back(item);
        if (is_s) s_items.push_back(item);
      }
      if (items.empty() || transformed_rhs < -1e-8) return;

      auto evaluate_order = [&](std::vector<ScucCoverItem> ordered,
                                const std::string& tag,
                                bool require_s_member) {
        double weight_sum = 0.0;
        double z_sum = 0.0;
        int complement_count = 0;
        int s_count = 0;
        for (int k = 0; k < static_cast<int>(ordered.size()); ++k) {
          weight_sum += ordered[static_cast<std::size_t>(k)].weight;
          z_sum += ordered[static_cast<std::size_t>(k)].z_value;
          complement_count += ordered[static_cast<std::size_t>(k)].complemented ? 1 : 0;
          s_count += ordered[static_cast<std::size_t>(k)].s_class ? 1 : 0;
          if (weight_sum <= transformed_rhs + 1e-8) continue;
          const double cover_rhs = static_cast<double>(k);
          const double violation = z_sum - cover_rhs;
          if (violation > 1e-6 && (!require_s_member || s_count > 0)) {
            const std::string name = family + tag + (complement_count > 0 ? ":mixed" : ":pos");
            record_violation(name, violation, s_group_id, t, line, z_sum, cover_rhs);
          }
          break;
        }
      };

      auto evaluate_family = [&](const std::vector<ScucCoverItem>& source,
                                 const std::string& tag,
                                 bool require_s_member) {
        if (source.empty()) return;
        auto by_z = source;
        std::sort(by_z.begin(), by_z.end(), [](const auto& a, const auto& b) {
          if (std::abs(a.z_value - b.z_value) > 1e-12) return a.z_value > b.z_value;
          return a.weight > b.weight;
        });
        evaluate_order(std::move(by_z), tag, require_s_member);

        auto by_weight = source;
        std::sort(by_weight.begin(), by_weight.end(), [](const auto& a, const auto& b) {
          if (std::abs(a.weight - b.weight) > 1e-12) return a.weight > b.weight;
          return a.z_value > b.z_value;
        });
        evaluate_order(std::move(by_weight), tag, require_s_member);

        auto by_score = source;
        std::sort(by_score.begin(), by_score.end(), [](const auto& a, const auto& b) {
          const double sa = a.z_value * std::max(1e-9, a.weight);
          const double sb = b.z_value * std::max(1e-9, b.weight);
          if (std::abs(sa - sb) > 1e-12) return sa > sb;
          return a.z_value > b.z_value;
        });
        evaluate_order(std::move(by_score), tag, require_s_member);
      };

      evaluate_family(s_items, ":SOnly", false);
      evaluate_family(items, ":WithS", true);
    };

    std::vector<double> row_coeff(static_cast<std::size_t>(ng), 0.0);
    std::vector<std::vector<ScucPtdfRowCandidate>> rows_by_time(static_cast<std::size_t>(T));
    for (int l = 0; l < nl; ++l) {
      for (int t = 0; t < T; ++t) {
        const double fwd_rhs = uc.line_fwd_rhs[static_cast<std::size_t>(l * T + t)];
        const double rev_rhs = uc.line_rev_rhs[static_cast<std::size_t>(l * T + t)];
        if (fwd_rhs < 1e8) {
          double lhs = 0.0;
          for (int g = 0; g < ng; ++g) {
            const double gsf = uc.line_gsf[static_cast<std::size_t>(l * ng + g)];
            row_coeff[static_cast<std::size_t>(g)] =
                (gsf > 0.0 ? gsf * uc.pmin[static_cast<std::size_t>(g)]
                           : gsf * uc.pmax[static_cast<std::size_t>(g)]);
            lhs += row_coeff[static_cast<std::size_t>(g)] * get_ig(g, t);
          }
          scan_s_group_cover("SGroupCover:C_fwd", row_coeff, fwd_rhs, l, t);
          rows_by_time[static_cast<std::size_t>(t)].push_back(
              {l, 1, fwd_rhs, fwd_rhs - lhs, row_coeff});
        }
        if (rev_rhs < 1e8) {
          double lhs = 0.0;
          for (int g = 0; g < ng; ++g) {
            const double neg_gsf = -uc.line_gsf[static_cast<std::size_t>(l * ng + g)];
            row_coeff[static_cast<std::size_t>(g)] =
                (neg_gsf > 0.0 ? neg_gsf * uc.pmin[static_cast<std::size_t>(g)]
                               : neg_gsf * uc.pmax[static_cast<std::size_t>(g)]);
            lhs += row_coeff[static_cast<std::size_t>(g)] * get_ig(g, t);
          }
          scan_s_group_cover("SGroupCover:C_rev", row_coeff, rev_rhs, l, t);
          rows_by_time[static_cast<std::size_t>(t)].push_back(
              {l, -1, rev_rhs, rev_rhs - lhs, row_coeff});
        }
      }
    }

    std::vector<double> agg_coeff(static_cast<std::size_t>(ng), 0.0);
    for (int t = 0; t < T; ++t) {
      auto& rows = rows_by_time[static_cast<std::size_t>(t)];
      if (rows.size() < 2) continue;
      std::sort(rows.begin(), rows.end(), [](const auto& a, const auto& b) {
        return a.slack < b.slack;
      });
      const int limit = std::min<int>(12, static_cast<int>(rows.size()));
      for (int i = 0; i < limit; ++i) {
        for (int j = i + 1; j < limit; ++j) {
          for (int g = 0; g < ng; ++g) {
            agg_coeff[static_cast<std::size_t>(g)] =
                rows[static_cast<std::size_t>(i)].coeff[static_cast<std::size_t>(g)] +
                rows[static_cast<std::size_t>(j)].coeff[static_cast<std::size_t>(g)];
          }
          const double agg_rhs = rows[static_cast<std::size_t>(i)].rhs +
                                 rows[static_cast<std::size_t>(j)].rhs;
          const int aux = rows[static_cast<std::size_t>(i)].line * 1000 +
                          rows[static_cast<std::size_t>(j)].line;
          scan_s_group_cover("SGroupCover:C_pair", agg_coeff, agg_rhs, aux, t);
        }
      }
    }
  }

  std::sort(top.begin(), top.end(), [](const auto& a, const auto& b) {
    return a.violation > b.violation;
  });

  std::fprintf(stderr,
               "[Gurobi][SCUC gap analysis] node=%.0f time=%.2fs incumbent=%.8g bound=%.8g rel_gap=%.4f\n",
               node_count, runtime, incumbent, bound, rel_gap);
  if (summary.empty()) {
    std::fprintf(stderr, "[Gurobi][SCUC gap analysis] no tested violations above tolerance\n");
    return;
  }
  std::fprintf(stderr, "[Gurobi][SCUC gap analysis] violation summary:\n");
  for (const auto& [family, s] : summary) {
    std::fprintf(stderr,
                 "  %-24s count=%5d max=%11.6g sum=%11.6g\n",
                 family.c_str(), s.count, s.max_violation, s.sum_violation);
  }
  if (s_group_id >= 0) {
    const double pmin = uc.group_pmin.size() > static_cast<std::size_t>(s_group_id)
                            ? uc.group_pmin[static_cast<std::size_t>(s_group_id)] : 0.0;
    const double pmax = uc.group_pmax.size() > static_cast<std::size_t>(s_group_id)
                            ? uc.group_pmax[static_cast<std::size_t>(s_group_id)] : 0.0;
    std::fprintf(stderr,
                 "[Gurobi][SCUC gap analysis] S-class group q=%d size=%d pmin=%.2f pmax=%.2f\n",
                 s_group_id, s_group_size, pmin, pmax);
  }
  std::fprintf(stderr, "[Gurobi][SCUC gap analysis] IG fractionality by pmax tier:\n");
  for (int tier = 0; tier < 7; ++tier) {
    if (tier_count[tier] == 0) continue;
    std::fprintf(stderr,
                 "  tier=%-7s gens=%2d ig_frac_sum=%10.6f u_sum=%10.6f pmax_u=%12.3f\n",
                 tier_label[tier], tier_count[tier], tier_ig_frac[tier],
                 tier_u_sum[tier], tier_pmax_u[tier]);
  }

  std::vector<int> gen_order;
  gen_order.reserve(static_cast<std::size_t>(ng));
  for (int g = 0; g < ng; ++g) gen_order.push_back(g);
  std::sort(gen_order.begin(), gen_order.end(), [&](int a, int b) {
    return gen_ig_frac[static_cast<std::size_t>(a)] >
           gen_ig_frac[static_cast<std::size_t>(b)];
  });
  std::fprintf(stderr, "[Gurobi][SCUC gap analysis] top generators by IG fractionality:\n");
  const int gen_limit = std::min(12, ng);
  for (int i = 0; i < gen_limit; ++i) {
    const int g = gen_order[static_cast<std::size_t>(i)];
    const double pmin = uc.pmin.size() > static_cast<std::size_t>(g) ? uc.pmin[static_cast<std::size_t>(g)] : 0.0;
    const double pmax = uc.pmax.size() > static_cast<std::size_t>(g) ? uc.pmax[static_cast<std::size_t>(g)] : 0.0;
    std::fprintf(stderr,
                 "  g=%3d pmin=%7.2f pmax=%7.2f ig_frac_sum=%9.6f u_sum=%9.6f su_frac=%9.6f sd_frac=%9.6f\n",
                 g, pmin, pmax, gen_ig_frac[static_cast<std::size_t>(g)],
                 gen_u_sum[static_cast<std::size_t>(g)],
                 gen_su_frac[static_cast<std::size_t>(g)],
                 gen_sd_frac[static_cast<std::size_t>(g)]);
  }

  std::vector<int> time_order;
  time_order.reserve(static_cast<std::size_t>(T));
  for (int t = 0; t < T; ++t) time_order.push_back(t);
  std::sort(time_order.begin(), time_order.end(), [&](int a, int b) {
    return time_ig_frac[static_cast<std::size_t>(a)] >
           time_ig_frac[static_cast<std::size_t>(b)];
  });
  std::fprintf(stderr, "[Gurobi][SCUC gap analysis] top periods by IG fractionality:\n");
  const int time_limit = std::min(12, T);
  for (int i = 0; i < time_limit; ++i) {
    const int t = time_order[static_cast<std::size_t>(i)];
    const double demand = uc.demand.size() > static_cast<std::size_t>(t) ? uc.demand[static_cast<std::size_t>(t)] : 0.0;
    const double up_req = uc.reserve_requirement.size() > static_cast<std::size_t>(t)
                              ? uc.reserve_requirement[static_cast<std::size_t>(t)] : 0.0;
    std::fprintf(stderr,
                 "  t=%3d ig_frac_sum=%9.6f u_sum=%9.6f pmin_u=%10.3f pmax_u=%10.3f demand=%10.3f up_req=%9.3f cap_slack=%10.3f\n",
                 t, time_ig_frac[static_cast<std::size_t>(t)],
                 time_u_sum[static_cast<std::size_t>(t)],
                 time_pmin_u[static_cast<std::size_t>(t)],
                 time_pmax_u[static_cast<std::size_t>(t)], demand, up_req,
                 time_pmax_u[static_cast<std::size_t>(t)] - demand - up_req);
  }
  const int limit = std::max(0, std::min(top_limit, static_cast<int>(top.size())));
  std::fprintf(stderr, "[Gurobi][SCUC gap analysis] top %d violations:\n", limit);
  for (int i = 0; i < limit; ++i) {
    const auto& v = top[static_cast<std::size_t>(i)];
    std::fprintf(stderr,
                 "  %-24s viol=%11.6g g=%3d t=%3d aux=%3d lhs=%12.6f rhs=%12.6f\n",
                 v.family.c_str(), v.violation, v.g, v.t, v.aux, v.lhs, v.rhs);
  }
}

static int separate_scuc_network_rows(void* cbdata, const std::vector<double>& sol,
                                      GurobiScucCutData& data, bool as_lazy) {
  const auto* uc_ptr = data.uc;
  if (uc_ptr == nullptr) return 0;
  const auto& uc = *uc_ptr;
  const int row_count = uc.network_flow_row_count;
  if (row_count <= 0 || data.n <= 0) return 0;
  if (uc.network_flow_row_start.size() != static_cast<std::size_t>(row_count + 1) ||
      uc.network_flow_rhs.size() < static_cast<std::size_t>(row_count)) {
    return 0;
  }

  auto& seen = as_lazy ? data.network_lazy_cut_seen : data.network_user_cut_seen;
  if (seen.size() < static_cast<std::size_t>(row_count))
    seen.assign(static_cast<std::size_t>(row_count), 0);

  std::vector<ScucNetworkCutCandidate> candidates;
  candidates.reserve(64);
  for (int r = 0; r < row_count; ++r) {
    if (seen[static_cast<std::size_t>(r)] != 0) continue;
    const int begin = uc.network_flow_row_start[static_cast<std::size_t>(r)];
    const int end = uc.network_flow_row_start[static_cast<std::size_t>(r + 1)];
    if (begin < 0 || end < begin ||
        end > static_cast<int>(uc.network_flow_col.size()) ||
        end > static_cast<int>(uc.network_flow_value.size())) {
      continue;
    }
    double activity = 0.0;
    for (int k = begin; k < end; ++k) {
      const int col = uc.network_flow_col[static_cast<std::size_t>(k)];
      if (col < 0 || col >= data.n) continue;
      activity += uc.network_flow_value[static_cast<std::size_t>(k)] *
                  sol[static_cast<std::size_t>(col)];
    }
    const double rhs = uc.network_flow_rhs[static_cast<std::size_t>(r)];
    const double violation = activity - rhs;
    if (violation > data.network_cut_tol)
      candidates.push_back({r, violation});
  }

  if (candidates.empty()) return 0;
  std::sort(candidates.begin(), candidates.end(), [](const auto& a, const auto& b) {
    return a.violation > b.violation;
  });

  const int max_cuts = as_lazy ? data.max_network_lazy_cuts_per_call
                               : data.max_network_user_cuts_per_call;
  if (max_cuts <= 0) return 0;
  int cuts_added = 0;
  std::vector<int> ind;
  std::vector<double> val;
  for (const auto& candidate : candidates) {
    if (cuts_added >= max_cuts) break;
    const int r = candidate.row;
    const int begin = uc.network_flow_row_start[static_cast<std::size_t>(r)];
    const int end = uc.network_flow_row_start[static_cast<std::size_t>(r + 1)];
    ind.clear();
    val.clear();
    ind.reserve(static_cast<std::size_t>(end - begin));
    val.reserve(static_cast<std::size_t>(end - begin));
    for (int k = begin; k < end; ++k) {
      const int col = uc.network_flow_col[static_cast<std::size_t>(k)];
      if (col < 0 || col >= data.n) continue;
      const double coeff = uc.network_flow_value[static_cast<std::size_t>(k)];
      if (std::abs(coeff) <= 1e-15) continue;
      ind.push_back(col);
      val.push_back(coeff);
    }
    if (ind.empty()) continue;
    const double rhs = uc.network_flow_rhs[static_cast<std::size_t>(r)];
    int err = 0;
    if (as_lazy) {
      err = GRBcblazy(cbdata, static_cast<int>(ind.size()), ind.data(),
                      val.data(), GRB_LESS_EQUAL, rhs);
    } else {
      err = GRBcbcut(cbdata, static_cast<int>(ind.size()), ind.data(),
                     val.data(), GRB_LESS_EQUAL, rhs);
    }
    if (err == 0) {
      seen[static_cast<std::size_t>(r)] = 1;
      ++cuts_added;
    }
  }

  data.total_cuts_added += cuts_added;
  if (as_lazy) data.network_lazy_cuts_added += cuts_added;
  else data.network_user_cuts_added += cuts_added;
  return cuts_added;
}

// Gurobi MIP callback: inject violated pairwise min-up/down conflict cuts as
// user cuts at every MIPNODE LP-optimal node during branch-and-bound.
//
//   SU(g,t1) + SD(g,t2) <= 1   for 0 < t2 - t1 < min_up[g]   (min-up conflict)
//   SD(g,t1) + SU(g,t2) <= 1   for 0 < t2 - t1 < min_down[g] (min-down conflict)
//
// These are valid implied cuts derived from the min-up/down constraints that
// are NOT present in the base SCUC formulation, mirroring the native B&C
// append_strict_scuc_dynamic_cuts() logic.
static int gurobi_scuc_cut_callback(
    GRBmodel* /*model*/, void* cbdata, int where, void* usrdata) {
  auto* data = static_cast<GurobiScucCutData*>(usrdata);
  if (!data || !data->uc) return 0;
  const auto& uc = *data->uc;

  if (where == GRB_CB_MIPSOL) {
    if (!data->separate_network) return 0;
    std::vector<double> sol(static_cast<std::size_t>(data->n), 0.0);
    if (GRBcbget(cbdata, where, GRB_CB_MIPSOL_SOL, sol.data()) != 0) return 0;
    separate_scuc_network_rows(cbdata, sol, *data, true);
    return 0;
  }

  if (where != GRB_CB_MIPNODE) return 0;

  // Only proceed when the LP relaxation is LP-optimal at this node.
  int node_status = 0;
  if (GRBcbget(cbdata, where, GRB_CB_MIPNODE_STATUS, &node_status) != 0)
    return 0;
  if (node_status != GRB_OPTIMAL) return 0;

  const int ng = uc.ng, T = uc.T, n = data->n;
  if (ng <= 0 || T <= 0 || n <= 0) return 0;
  // (su_cols/sd_cols size guards are per-block below; absent data causes those
  // blocks to be skipped gracefully via internal bounds checks.)

  // Fetch LP relaxation solution at this node.
  std::vector<double> sol(static_cast<std::size_t>(n), 0.0);
  if (GRBcbget(cbdata, where, GRB_CB_MIPNODE_REL, sol.data()) != 0) return 0;

  if (data->separate_network)
    separate_scuc_network_rows(cbdata, sol, *data, false);

  if (data->analyze_gap && !data->analyze_done) {
    double incumbent = 0.0;
    double bound = 0.0;
    double node_count = 0.0;
    double runtime = 0.0;
    const int ok_best = GRBcbget(cbdata, where, GRB_CB_MIPNODE_OBJBST, &incumbent);
    const int ok_bound = GRBcbget(cbdata, where, GRB_CB_MIPNODE_OBJBND, &bound);
    GRBcbget(cbdata, where, GRB_CB_MIPNODE_NODCNT, &node_count);
    GRBcbget(cbdata, where, GRB_CB_RUNTIME, &runtime);
    if (ok_best == 0 && ok_bound == 0 && std::isfinite(incumbent) &&
        std::isfinite(bound) && std::abs(incumbent) < 1e90) {
      const double rel_gap = std::abs(incumbent - bound) /
                             std::max(1.0, std::abs(incumbent));
      if (rel_gap <= data->analyze_gap_trigger) {
        analyze_scuc_relaxation_at_gap(uc, sol, incumbent, bound, rel_gap,
                                       runtime, node_count, data->analyze_top);
        data->analyze_done = true;
      }
    }
  }

  if (!data->enable_dynamic_cuts) return 0;
  if (!uc.certifies_min_up_down_rows) return 0;

  const double pf = data->pair_floor;
  const int max_cuts = data->max_cuts_per_call;
  int cuts_added = 0;

  auto get_col = [&](const std::vector<int>& cols, int g, int t) -> int {
    if (g < 0 || g >= ng || t < 0 || t >= T) return -1;
    const auto idx = static_cast<std::size_t>(t * ng + g);
    if (idx >= cols.size()) return -1;
    return cols[idx];
  };
  auto get_val = [&](int col) -> double {
    if (col < 0 || col >= n) return 0.0;
    return sol[static_cast<std::size_t>(col)];
  };

  // Min-up violations: SU(g,t1) + SD(g,t2) > 1
  if (uc.min_up.size() >= static_cast<std::size_t>(ng)) {
    for (int g = 0; g < ng && cuts_added < max_cuts; ++g) {
      const int mu = uc.min_up[static_cast<std::size_t>(g)];
      if (mu <= 1) continue;
      for (int t1 = 0; t1 < T && cuts_added < max_cuts; ++t1) {
        const int su_col = get_col(uc.su_cols, g, t1);
        if (su_col < 0) continue;
        const double su_v = get_val(su_col);
        if (su_v <= pf) continue;
        const int last = std::min(T - 1, t1 + mu - 1);
        for (int t2 = t1 + 1; t2 <= last && cuts_added < max_cuts; ++t2) {
          const int sd_col = get_col(uc.sd_cols, g, t2);
          if (sd_col < 0) continue;
          const double sd_v = get_val(sd_col);
          if (sd_v <= pf || su_v + sd_v <= 1.0 + pf) continue;
          const int ind[2] = {su_col, sd_col};
          const double val[2] = {1.0, 1.0};
          GRBcbcut(cbdata, 2, ind, val, GRB_LESS_EQUAL, 1.0);
          ++cuts_added;
        }
      }
    }
  }

  // Min-down violations: SD(g,t1) + SU(g,t2) > 1
  if (uc.min_down.size() >= static_cast<std::size_t>(ng)) {
    for (int g = 0; g < ng && cuts_added < max_cuts; ++g) {
      const int md = uc.min_down[static_cast<std::size_t>(g)];
      if (md <= 1) continue;
      for (int t1 = 0; t1 < T && cuts_added < max_cuts; ++t1) {
        const int sd_col = get_col(uc.sd_cols, g, t1);
        if (sd_col < 0) continue;
        const double sd_v = get_val(sd_col);
        if (sd_v <= pf) continue;
        const int last = std::min(T - 1, t1 + md - 1);
        for (int t2 = t1 + 1; t2 <= last && cuts_added < max_cuts; ++t2) {
          const int su_col = get_col(uc.su_cols, g, t2);
          if (su_col < 0) continue;
          const double su_v = get_val(su_col);
          if (su_v <= pf || sd_v + su_v <= 1.0 + pf) continue;
          const int ind[2] = {sd_col, su_col};
          const double val[2] = {1.0, 1.0};
          GRBcbcut(cbdata, 2, ind, val, GRB_LESS_EQUAL, 1.0);
          ++cuts_added;
        }
      }
    }
  }

  // -------------------------------------------------------------------
  // Family B (dynamic): reserve-commitment cover violations
  //   sum_g cap[g] * u_lp(g,t) < requirement[t]  → inject GEQ cut
  // -------------------------------------------------------------------
  if (uc.certifies_system_reserve_rows &&
      uc.ig_cols.size() >= static_cast<std::size_t>(ng * T)) {
    auto check_reserve_cover = [&](const std::vector<double>& caps,
                                   const std::vector<double>& req) {
      for (int t = 0; t < T && cuts_added < max_cuts; ++t) {
        if (req.size() <= static_cast<std::size_t>(t)) continue;
        const double requirement = req[static_cast<std::size_t>(t)];
        if (!(requirement > 0.0)) continue;
        if (caps.size() < static_cast<std::size_t>(ng)) continue;
        std::vector<int>    r_ind;
        std::vector<double> r_val;
        double lhs = 0.0;
        for (int g = 0; g < ng; ++g) {
          const int ig = uc.ig_cols[static_cast<std::size_t>(t * ng + g)];
          if (ig < 0 || ig >= n) continue;
          const double cap = std::max(0.0, caps[static_cast<std::size_t>(g)]);
          if (!(cap > 0.0)) continue;
          const double u_v = get_val(ig);
          lhs += cap * u_v;
          r_ind.push_back(ig); r_val.push_back(cap);
        }
        if (!r_ind.empty() && lhs < requirement - pf) {
          GRBcbcut(cbdata, static_cast<int>(r_ind.size()), r_ind.data(),
                   r_val.data(), GRB_GREATER_EQUAL, requirement);
          ++cuts_added;
        }
      }
    };

    if (!uc.up_reserve_headroom_cap.empty() && !uc.reserve_requirement.empty())
      check_reserve_cover(uc.up_reserve_headroom_cap, uc.reserve_requirement);
    if (!uc.spinning_reserve_cap.empty() && !uc.spinning_requirement.empty())
      check_reserve_cover(uc.spinning_reserve_cap, uc.spinning_requirement);
    if (!uc.regulation_up_cap.empty() && !uc.regulation_up_requirement.empty())
      check_reserve_cover(uc.regulation_up_cap, uc.regulation_up_requirement);
    if (!uc.regulation_down_cap.empty() &&
        !uc.regulation_down_requirement.empty())
      check_reserve_cover(uc.regulation_down_cap, uc.regulation_down_requirement);
  }

  // -------------------------------------------------------------------
  // Family G (extended clique): sum_{t'=t}^{t+T^U+T^D-1} SU(g,t') <= 1
  // Check each startup window; inject cut when LP sum > 1.
  // When static tight SCUC is enabled these rows are already in the model
  // and no violations will be found; cost is O(ng*T) dot product per node.
  // -------------------------------------------------------------------
  if (uc.min_up.size() >= static_cast<std::size_t>(ng) &&
      uc.min_down.size() >= static_cast<std::size_t>(ng)) {
    for (int g = 0; g < ng && cuts_added < max_cuts; ++g) {
      const int mu = uc.min_up[static_cast<std::size_t>(g)];
      const int md = uc.min_down[static_cast<std::size_t>(g)];
      const int L = mu + md;
      if (L <= 1) continue;
      for (int t = 0; t < T && cuts_added < max_cuts; ++t) {
        const int last = std::min(T - 1, t + L - 1);
        if (last == t) continue;
        std::vector<int>    w_ind;
        std::vector<double> w_val;
        double sum_su = 0.0;
        for (int t2 = t; t2 <= last; ++t2) {
          const int col = get_col(uc.su_cols, g, t2);
          if (col < 0) continue;
          const double v = get_val(col);
          w_ind.push_back(col); w_val.push_back(1.0); sum_su += v;
        }
        if (static_cast<int>(w_ind.size()) >= 2 && sum_su > 1.0 + pf) {
          GRBcbcut(cbdata, static_cast<int>(w_ind.size()), w_ind.data(),
                   w_val.data(), GRB_LESS_EQUAL, 1.0);
          ++cuts_added;
        }
      }
    }
  }

  data->total_cuts_added += cuts_added;
  return 0;
}

}  // anonymous namespace

#endif  // HACDCPF_HAVE_GUROBI

GurobiAdapter::GurobiAdapter() {
#ifdef HACDCPF_HAVE_GUROBI
  GRBenv* env = nullptr;
  if (GRBloadenv(&env, nullptr) == 0 && env != nullptr) {
    // Suppress console output.
    GRBsetintparam(env, "OutputFlag", 0);
    env_ = env;
  }
#endif
}

GurobiAdapter::~GurobiAdapter() {
#ifdef HACDCPF_HAVE_GUROBI
  if (env_) {
    GRBfreeenv(static_cast<GRBenv*>(env_));
  }
#endif
}

std::string GurobiAdapter::name() const { return "Gurobi"; }

bool GurobiAdapter::supports(ProblemClass cls) const {
  return available() && (cls == ProblemClass::LP || cls == ProblemClass::QP || cls == ProblemClass::MILP);
}

bool GurobiAdapter::available() const {
#ifdef HACDCPF_HAVE_GUROBI
  return env_ != nullptr;
#else
  return false;
#endif
}

SolveResult GurobiAdapter::solve_lp(const LPModel& prob) const {
  SolveResult out;
  out.stats.solver_name = name();
#ifndef HACDCPF_HAVE_GUROBI
  (void)prob;  // Gurobi disabled at compile time; param unused in stub
#endif
#ifdef HACDCPF_HAVE_GUROBI
  const auto t0 = std::chrono::steady_clock::now();
  if (!available()) {
    out.stats.status = "Unavailable: Gurobi env not initialized";
    return out;
  }
  auto* env = static_cast<GRBenv*>(env_);
  GRBmodel* model = nullptr;
  if (GRBnewmodel(env, &model, "lp", 0, nullptr, nullptr, nullptr, nullptr, nullptr) != 0) {
    out.stats.status = "Gurobi: failed to create model";
    return out;
  }

  const int n = static_cast<int>(prob.vars.size());
  const int m_ineq = static_cast<int>(prob.A.rows());
  const int m_eq = static_cast<int>(prob.Aeq.rows());
  const double obj_sign = (prob.sense == Sense::Maximize) ? -1.0 : 1.0;

  // Add variables.
  std::vector<double> obj_coeff(n), lb(n), ub(n);
  for (int j = 0; j < n; ++j) {
    obj_coeff[j] = obj_sign * prob.c[j];
    lb[j] = prob.vars[j].lb;
    ub[j] = prob.vars[j].ub;
  }
  GRBaddvars(model, n, 0, nullptr, nullptr, nullptr,
             obj_coeff.data(), lb.data(), ub.data(), nullptr, nullptr);
  GRBsetintattr(model, "ModelSense", GRB_MINIMIZE);

  // Inequality constraints: Ax <= b.
  for (int i = 0; i < m_ineq; ++i) {
    std::vector<int> ind;
    std::vector<double> val;
    for (Eigen::SparseMatrix<double>::InnerIterator it(prob.A, 0); it; ++it) {
      // Sparse col-major iteration — need row-by-row, so iterate all columns.
    }
    // Use dense row extraction for correctness.
    for (int j = 0; j < n; ++j) {
      const double v = prob.A.coeff(i, j);
      if (std::abs(v) > 1e-15) {
        ind.push_back(j);
        val.push_back(v);
      }
    }
    GRBaddconstr(model, static_cast<int>(ind.size()), ind.data(), val.data(),
                 GRB_LESS_EQUAL, prob.b[i], nullptr);
    const double lhs = lp_row_lhs_or_neg_inf(prob, i);
    if (std::isfinite(lhs)) {
      GRBaddconstr(model, static_cast<int>(ind.size()), ind.data(), val.data(),
                   GRB_GREATER_EQUAL, lhs, nullptr);
    }
  }

  // Equality constraints: Aeq x = beq.
  for (int i = 0; i < m_eq; ++i) {
    std::vector<int> ind;
    std::vector<double> val;
    for (int j = 0; j < n; ++j) {
      const double v = prob.Aeq.coeff(i, j);
      if (std::abs(v) > 1e-15) {
        ind.push_back(j);
        val.push_back(v);
      }
    }
    GRBaddconstr(model, static_cast<int>(ind.size()), ind.data(), val.data(),
                 GRB_EQUAL, prob.beq[i], nullptr);
  }

  GRBupdatemodel(model);
  GRBoptimize(model);

  int status = 0;
  GRBgetintattr(model, "Status", &status);

  if (status == GRB_OPTIMAL) {
    out.stats.success = true;
    out.stats.status = "Optimal";
    double objval = 0.0;
    GRBgetdblattr(model, "ObjVal", &objval);
    out.stats.objective = obj_sign * objval;
    out.x.resize(n);
    GRBgetdblattrarray(model, "X", 0, n, out.x.data());

    // Extract constraint duals (Pi) for LP.
    const int m_total = m_ineq + m_eq;
    out.constraint_duals.resize(m_total);
    if (GRBgetdblattrarray(model, "Pi", 0, m_total,
                           out.constraint_duals.data()) == 0) {
      out.constraint_duals *= obj_sign;
    } else {
      out.constraint_duals.resize(0);
    }
  } else if (status == GRB_INFEASIBLE) {
    out.stats.status = "Infeasible";
    // Compute Farkas certificate (IIS-based dual ray).
    GRBsetintparam(GRBgetenv(model), "InfUnbdInfo", 1);
    GRBoptimize(model);
    int recheck = 0;
    GRBgetintattr(model, "Status", &recheck);
    if (recheck == GRB_INFEASIBLE) {
      out.stats.farkas_ray.resize(m_ineq);
      out.stats.farkas_ray_eq.resize(m_eq);
      // Gurobi stores Farkas dual on constraints via FarkasDual attribute.
      std::vector<double> fdual(m_ineq + m_eq);
      if (GRBgetdblattrarray(model, "FarkasDual", 0, m_ineq + m_eq, fdual.data()) == 0) {
        for (int i = 0; i < m_ineq; ++i) out.stats.farkas_ray[i] = fdual[i];
        for (int i = 0; i < m_eq; ++i) out.stats.farkas_ray_eq[i] = fdual[m_ineq + i];
        out.stats.has_farkas_certificate = true;
      }
    }
  } else {
    out.stats.status = "Gurobi status=" + std::to_string(status);
  }

  const auto t1 = std::chrono::steady_clock::now();
  out.stats.runtime_sec = std::chrono::duration<double>(t1 - t0).count();
  GRBfreemodel(model);
#else
  out.stats.status = "Unavailable: Gurobi not linked";
#endif
  return out;
}

SolveResult GurobiAdapter::solve_qp(const QPModel& prob) const {
  SolveResult out;
  out.stats.solver_name = name();
#ifndef HACDCPF_HAVE_GUROBI
  (void)prob;
#endif
#ifdef HACDCPF_HAVE_GUROBI
  const auto t0 = std::chrono::steady_clock::now();
  if (!available()) {
    out.stats.status = "Unavailable: Gurobi env not initialized";
    return out;
  }
  auto* env = static_cast<GRBenv*>(env_);
  GRBmodel* model = nullptr;
  if (GRBnewmodel(env, &model, "qp", 0, nullptr, nullptr, nullptr, nullptr, nullptr) != 0) {
    out.stats.status = "Gurobi: failed to create model";
    return out;
  }

  const int n = static_cast<int>(prob.vars.size());
  const int m_ineq = static_cast<int>(prob.A.rows());
  const int m_eq = static_cast<int>(prob.Aeq.rows());
  const double obj_sign = (prob.sense == Sense::Maximize) ? -1.0 : 1.0;

  // Add variables with linear objective coefficients
  std::vector<double> obj_coeff(n), lb(n), ub(n);
  for (int j = 0; j < n; ++j) {
    obj_coeff[j] = obj_sign * prob.c[j];
    lb[j] = prob.vars[j].lb;
    ub[j] = prob.vars[j].ub;
  }
  GRBaddvars(model, n, 0, nullptr, nullptr, nullptr,
             obj_coeff.data(), lb.data(), ub.data(), nullptr, nullptr);
  GRBsetintattr(model, "ModelSense", GRB_MINIMIZE);

  // Add quadratic objective: Q is stored as the Hessian (we need 0.5*x'Qx)
  // Gurobi expects quadratic terms as coefficient pairs
  for (int k = 0; k < prob.Q.outerSize(); ++k) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(prob.Q, k); it; ++it) {
      int i = it.row();
      int j = it.col();
      double qij = it.value();
      if (std::abs(qij) > 1e-15) {
        // GRBaddqpterms uses raw coefficient: contributes qij*xi*xj directly.
        // QPModel convention is 0.5*x'Qx: halve here so Gurobi sees the
        // right effective coefficient. Apply obj_sign for maximize support.
        double qij_half = 0.5 * obj_sign * qij;
        GRBaddqpterms(model, 1, &i, &j, &qij_half);
      }
    }
  }

  // Inequality constraints: Ax <= b
  for (int i = 0; i < m_ineq; ++i) {
    std::vector<int> ind;
    std::vector<double> val;
    for (int j = 0; j < n; ++j) {
      const double v = prob.A.coeff(i, j);
      if (std::abs(v) > 1e-15) {
        ind.push_back(j);
        val.push_back(v);
      }
    }
    GRBaddconstr(model, static_cast<int>(ind.size()), ind.data(), val.data(),
                 GRB_LESS_EQUAL, prob.b[i], nullptr);
  }

  // Equality constraints: Aeq x = beq
  for (int i = 0; i < m_eq; ++i) {
    std::vector<int> ind;
    std::vector<double> val;
    for (int j = 0; j < n; ++j) {
      const double v = prob.Aeq.coeff(i, j);
      if (std::abs(v) > 1e-15) {
        ind.push_back(j);
        val.push_back(v);
      }
    }
    GRBaddconstr(model, static_cast<int>(ind.size()), ind.data(), val.data(),
                 GRB_EQUAL, prob.beq[i], nullptr);
  }

  GRBupdatemodel(model);
  GRBoptimize(model);

  int status = 0;
  GRBgetintattr(model, "Status", &status);

  if (status == GRB_OPTIMAL) {
    out.stats.success = true;
    out.stats.status = "Optimal";
    double objval = 0.0;
    GRBgetdblattr(model, "ObjVal", &objval);
    out.stats.objective = obj_sign * objval;
    out.x.resize(n);
    GRBgetdblattrarray(model, "X", 0, n, out.x.data());
  } else if (status == GRB_INFEASIBLE) {
    out.stats.status = "Infeasible";
  } else {
    out.stats.status = "Gurobi status=" + std::to_string(status);
  }

  const auto t1 = std::chrono::steady_clock::now();
  out.stats.runtime_sec = std::chrono::duration<double>(t1 - t0).count();
  GRBfreemodel(model);
#else
  out.stats.status = "Unavailable: Gurobi not linked";
#endif
  return out;
}

SolveResult GurobiAdapter::solve_milp(const MIPModel& prob) const {
  SolveResult out;
  out.stats.solver_name = name();
#ifndef HACDCPF_HAVE_GUROBI
  (void)prob;
#endif
#ifdef HACDCPF_HAVE_GUROBI
  const auto t0 = std::chrono::steady_clock::now();
  if (!available()) {
    out.stats.status = "Unavailable: Gurobi env not initialized";
    return out;
  }
  auto* env = static_cast<GRBenv*>(env_);
  GRBmodel* model = nullptr;
  if (GRBnewmodel(env, &model, "milp", 0, nullptr, nullptr, nullptr, nullptr, nullptr) != 0) {
    out.stats.status = "Gurobi: failed to create model";
    return out;
  }

  const auto& lp = prob.linear_part;
  const int n = static_cast<int>(lp.vars.size());
  const int m_ineq = static_cast<int>(lp.A.rows());
  const int m_eq = static_cast<int>(lp.Aeq.rows());
  const double obj_sign = (lp.sense == Sense::Maximize) ? -1.0 : 1.0;

  // Prepare variable metadata.
  std::vector<double> obj_coeff(n), lb_vec(n), ub_vec(n);
  std::vector<char> vtype(n, GRB_CONTINUOUS);
  for (int j = 0; j < n; ++j) {
    obj_coeff[j] = obj_sign * lp.c[j];
    lb_vec[j] = lp.vars[j].lb;
    ub_vec[j] = lp.vars[j].ub;
  }
  for (int idx : prob.integer_idx) {
    if (idx >= 0 && idx < n) {
      vtype[idx] = GRB_INTEGER;
    }
  }
  for (int idx : prob.binary_idx) {
    if (idx >= 0 && idx < n) {
      vtype[idx] = GRB_BINARY;
      lb_vec[idx] = std::max(0.0, lb_vec[idx]);
      ub_vec[idx] = std::min(1.0, ub_vec[idx]);
    }
  }

  GRBaddvars(model, n, 0, nullptr, nullptr, nullptr,
             obj_coeff.data(), lb_vec.data(), ub_vec.data(), vtype.data(), nullptr);
  GRBsetintattr(model, "ModelSense", GRB_MINIMIZE);

  const bool separate_gurobi_network =
      prob.uc_hint.has_value() && prob.uc_hint->certifies_hard_network_flow_rows &&
      prob.uc_hint->network_flow_row_count > 0 &&
      prob.uc_hint->network_flow_row_start.size() ==
          static_cast<std::size_t>(prob.uc_hint->network_flow_row_count + 1) &&
      (std::getenv("MIPSOLVERS_GUROBI_SEPARATE_NETWORK") != nullptr ||
       std::getenv("MIPSOLVERS_SCUC_LAZY_NETWORK") != nullptr);
  std::vector<unsigned char> skip_ineq_row(static_cast<std::size_t>(m_ineq), 0);
  int skipped_network_rows = 0;
  if (separate_gurobi_network) {
    for (int row : prob.uc_hint->network_flow_original_row) {
      if (row >= 0 && row < m_ineq && skip_ineq_row[static_cast<std::size_t>(row)] == 0) {
        skip_ineq_row[static_cast<std::size_t>(row)] = 1;
        ++skipped_network_rows;
      }
    }
  }

  // Load inequality constraints using row-major sparse iteration.
  // The dense lp.A.coeff(i,j) approach is O(n) per entry (O(m*n) total),
  // which is prohibitive for large SCUC instances (26k vars × 32k rows).
  // Converting to row-major once and iterating over nonzeros is O(nnz).
  {
    Eigen::SparseMatrix<double, Eigen::RowMajor> A_rm(lp.A);
    for (int i = 0; i < m_ineq; ++i) {
      if (skip_ineq_row[static_cast<std::size_t>(i)] != 0) continue;
      std::vector<int> ind;
      std::vector<double> val;
      for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(A_rm, i); it; ++it) {
        if (std::abs(it.value()) > 1e-15) {
          ind.push_back(static_cast<int>(it.col()));
          val.push_back(it.value());
        }
      }
      GRBaddconstr(model, static_cast<int>(ind.size()), ind.data(), val.data(),
                   GRB_LESS_EQUAL, lp.b[i], nullptr);
      const double lhs = lp_row_lhs_or_neg_inf(lp, i);
      if (std::isfinite(lhs)) {
        GRBaddconstr(model, static_cast<int>(ind.size()), ind.data(), val.data(),
                     GRB_GREATER_EQUAL, lhs, nullptr);
      }
    }
  }

  // Load equality constraints using row-major sparse iteration.
  {
    Eigen::SparseMatrix<double, Eigen::RowMajor> Aeq_rm(lp.Aeq);
    for (int i = 0; i < m_eq; ++i) {
      std::vector<int> ind;
      std::vector<double> val;
      for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(Aeq_rm, i); it; ++it) {
        if (std::abs(it.value()) > 1e-15) {
          ind.push_back(static_cast<int>(it.col()));
          val.push_back(it.value());
        }
      }
      GRBaddconstr(model, static_cast<int>(ind.size()), ind.data(), val.data(),
                   GRB_EQUAL, lp.beq[i], nullptr);
    }
  }

  // Tight SCUC formulation: add pairwise min-up/down conflict cuts as static
  // constraints before solve.  These are valid implied cuts not in the base
  // formulation that tighten the LP relaxation root bound.
  //   SU(g,t1) + SD(g,t2) <= 1  for 0 < t2-t1 < min_up[g]
  //   SD(g,t1) + SU(g,t2) <= 1  for 0 < t2-t1 < min_down[g]
  // Enable by setting MIPSOLVERS_GUROBI_TIGHT_SCUC=1.
  const bool add_tight_scuc =
      prob.uc_hint.has_value() &&
      prob.uc_hint->certifies_min_up_down_rows &&
      std::getenv("MIPSOLVERS_GUROBI_TIGHT_SCUC") != nullptr;
  if (add_tight_scuc) {
    const auto& uc = *prob.uc_hint;
    const int ng = uc.ng, T_uc = uc.T;
    int static_cuts = 0;
    auto add_pair = [&](int col_a, int col_b) {
      if (col_a < 0 || col_b < 0 || col_a >= n || col_b >= n) return;
      int ind[2] = {col_a, col_b};
      double val[2] = {1.0, 1.0};
      GRBaddconstr(model, 2, ind, val, GRB_LESS_EQUAL, 1.0, nullptr);
      ++static_cuts;
    };
    if (uc.su_cols.size() >= static_cast<std::size_t>(ng * T_uc) &&
        uc.sd_cols.size() >= static_cast<std::size_t>(ng * T_uc) &&
        uc.min_up.size() >= static_cast<std::size_t>(ng)) {
      for (int g = 0; g < ng; ++g) {
        const int mu = uc.min_up[static_cast<std::size_t>(g)];
        if (mu <= 1) continue;
        for (int t1 = 0; t1 < T_uc; ++t1) {
          const int su = uc.su_cols[static_cast<std::size_t>(t1 * ng + g)];
          if (su < 0) continue;
          for (int t2 = t1 + 1; t2 <= std::min(T_uc - 1, t1 + mu - 1); ++t2)
            add_pair(su, uc.sd_cols[static_cast<std::size_t>(t2 * ng + g)]);
        }
      }
    }
    if (uc.sd_cols.size() >= static_cast<std::size_t>(ng * T_uc) &&
        uc.su_cols.size() >= static_cast<std::size_t>(ng * T_uc) &&
        uc.min_down.size() >= static_cast<std::size_t>(ng)) {
      for (int g = 0; g < ng; ++g) {
        const int md = uc.min_down[static_cast<std::size_t>(g)];
        if (md <= 1) continue;
        for (int t1 = 0; t1 < T_uc; ++t1) {
          const int sd = uc.sd_cols[static_cast<std::size_t>(t1 * ng + g)];
          if (sd < 0) continue;
          for (int t2 = t1 + 1; t2 <= std::min(T_uc - 1, t1 + md - 1); ++t2)
            add_pair(sd, uc.su_cols[static_cast<std::size_t>(t2 * ng + g)]);
        }
      }
    }
    // -------------------------------------------------------------------
    // Family G: Extended startup clique (no two startups within T^U+T^D window)
    //   sum_{t'=t}^{t+T^U+T^D-1} SU(g,t') <= 1   for each g, t
    // Also: total startup count bound
    //   sum_t SU(g,t) <= floor(T / (T^U + T^D))
    // These dominate the pairwise SU+SD cuts and tighten the LP relaxation
    // by 25-30% expected gap closure.
    // -------------------------------------------------------------------
    if (uc.su_cols.size() >= static_cast<std::size_t>(ng * T_uc) &&
        uc.min_up.size() >= static_cast<std::size_t>(ng) &&
        uc.min_down.size() >= static_cast<std::size_t>(ng)) {
      for (int g = 0; g < ng; ++g) {
        const int mu = uc.min_up[static_cast<std::size_t>(g)];
        const int md = uc.min_down[static_cast<std::size_t>(g)];
        const int L = mu + md;
        if (L <= 1) continue;

        // Clique cuts: at most 1 startup in each window [t, t+L-1].
        for (int t = 0; t < T_uc; ++t) {
          const int last = std::min(T_uc - 1, t + L - 1);
          if (last == t) continue;
          std::vector<int> ind;
          std::vector<double> val;
          for (int t2 = t; t2 <= last; ++t2) {
            const int col = uc.su_cols[static_cast<std::size_t>(t2 * ng + g)];
            if (col >= 0 && col < n) { ind.push_back(col); val.push_back(1.0); }
          }
          if (static_cast<int>(ind.size()) >= 2) {
            GRBaddconstr(model, static_cast<int>(ind.size()), ind.data(), val.data(),
                         GRB_LESS_EQUAL, 1.0, nullptr);
            ++static_cuts;
          }
        }

        // Total startup count bound: sum_t SU(g,t) <= floor(T / L).
        const int max_starts = T_uc / L;
        if (max_starts >= 1) {
          std::vector<int> ind;
          std::vector<double> val;
          for (int t = 0; t < T_uc; ++t) {
            const int col = uc.su_cols[static_cast<std::size_t>(t * ng + g)];
            if (col >= 0 && col < n) { ind.push_back(col); val.push_back(1.0); }
          }
          if (!ind.empty()) {
            GRBaddconstr(model, static_cast<int>(ind.size()), ind.data(), val.data(),
                         GRB_LESS_EQUAL, static_cast<double>(max_starts), nullptr);
            ++static_cuts;
          }
        }
      }
    }

    // -------------------------------------------------------------------
    // Family A: Bid-segment cumulative coupling
    //   sum_{m=0}^{k-1} seg(g,t,m) <= cumcap[g,k] * u(g,t)   for each g,t,k
    // Prevents the LP from using fractional commitment to spread dispatch
    // evenly across bid segments (dispatch fills sequentially).
    // -------------------------------------------------------------------
    if (uc.n_segments > 1 &&
        uc.segment_cols.size() >=
            static_cast<std::size_t>(ng * T_uc * uc.n_segments) &&
        uc.segment_cap.size() >= static_cast<std::size_t>(ng * uc.n_segments) &&
        uc.ig_cols.size() >= static_cast<std::size_t>(ng * T_uc)) {
      const int ns = uc.n_segments;
      // Precompute cumulative capacities per generator.
      std::vector<double> cumcap(static_cast<std::size_t>(ng * ns), 0.0);
      for (int g = 0; g < ng; ++g)
        for (int m = 1; m < ns; ++m)
          cumcap[static_cast<std::size_t>(g * ns + m)] =
              cumcap[static_cast<std::size_t>(g * ns + m - 1)] +
              uc.segment_cap[static_cast<std::size_t>(g * ns + m - 1)];

      for (int g = 0; g < ng; ++g) {
        for (int k = 1; k < ns; ++k) {   // prefix 1..ns-1
          const double cc = cumcap[static_cast<std::size_t>(g * ns + k)];
          if (cc <= 0.0) continue;
          for (int t = 0; t < T_uc; ++t) {
            const int ig_col = uc.ig_cols[static_cast<std::size_t>(t * ng + g)];
            if (ig_col < 0 || ig_col >= n) continue;
            std::vector<int> ind;
            std::vector<double> val;
            for (int m = 0; m < k; ++m) {
              const int sc = uc.segment_cols[
                  static_cast<std::size_t>((t * ng + g) * ns + m)];
              if (sc >= 0 && sc < n) { ind.push_back(sc); val.push_back(1.0); }
            }
            // sum seg - cc * u(g,t) <= 0
            ind.push_back(ig_col);
            val.push_back(-cc);
            if (static_cast<int>(ind.size()) >= 2) {
              GRBaddconstr(model, static_cast<int>(ind.size()), ind.data(),
                           val.data(), GRB_LESS_EQUAL, 0.0, nullptr);
              ++static_cuts;
            }
          }
        }
      }
    }

    if (std::getenv("MIPSOLVERS_GUROBI_VERBOSE") != nullptr)
      std::fprintf(stderr, "[Gurobi] tight SCUC: %d static cuts added"
                   " (pairwise SU/SD + Family G clique + Family A segment)\n",
                   static_cuts);

    // -------------------------------------------------------------------
    // Family B: Reserve-commitment cover inequalities
    //   sum_g cap[g] * u(g,t) >= requirement[t]   for each reserve type, t
    // Mirrors native branch_and_cut.cpp: add_reserve_cover_cut().
    // These implied rows (derivable from headroom bound + reserve balance)
    // are explicitly added to strengthen the LP at root and all nodes.
    // -------------------------------------------------------------------
    if (uc.certifies_system_reserve_rows &&
        uc.ig_cols.size() >= static_cast<std::size_t>(ng * T_uc)) {
      int b_cuts = 0;

      auto add_reserve_cover_static = [&](const std::vector<double>& caps,
                                          const std::vector<double>& req, int t) {
        if (req.size() <= static_cast<std::size_t>(t) ||
            !(req[static_cast<std::size_t>(t)] > 0.0)) return;
        if (caps.size() < static_cast<std::size_t>(ng)) return;
        const double rhs = req[static_cast<std::size_t>(t)];
        std::vector<int> ind;
        std::vector<double> val;
        for (int g = 0; g < ng; ++g) {
          const int ig = uc.ig_cols[static_cast<std::size_t>(t * ng + g)];
          if (ig < 0 || ig >= n) continue;
          const double cap = std::max(0.0, caps[static_cast<std::size_t>(g)]);
          if (!(cap > 0.0)) continue;
          ind.push_back(ig); val.push_back(cap);
        }
        if (!ind.empty()) {
          GRBaddconstr(model, static_cast<int>(ind.size()), ind.data(), val.data(),
                       GRB_GREATER_EQUAL, rhs, nullptr);
          ++static_cuts; ++b_cuts;
        }
      };

      for (int t = 0; t < T_uc; ++t) {
        if (!uc.up_reserve_headroom_cap.empty() && !uc.reserve_requirement.empty())
          add_reserve_cover_static(uc.up_reserve_headroom_cap,
                                   uc.reserve_requirement, t);
        if (!uc.spinning_reserve_cap.empty() && !uc.spinning_requirement.empty())
          add_reserve_cover_static(uc.spinning_reserve_cap,
                                   uc.spinning_requirement, t);
        if (!uc.regulation_up_cap.empty() &&
            !uc.regulation_up_requirement.empty())
          add_reserve_cover_static(uc.regulation_up_cap,
                                   uc.regulation_up_requirement, t);
        if (!uc.regulation_down_cap.empty() &&
            !uc.regulation_down_requirement.empty())
          add_reserve_cover_static(uc.regulation_down_cap,
                                   uc.regulation_down_requirement, t);
      }

      if (std::getenv("MIPSOLVERS_GUROBI_VERBOSE") != nullptr)
        std::fprintf(stderr, "[Gurobi] tight SCUC: %d Family B reserve-cover"
                     " rows added statically\n", b_cuts);
    }

    // Family C: network line capacity commitment cuts.
    // For each monitored line l and period t, the maximum possible flow from
    // committed generators must not exceed the line thermal limit:
    //   Forward: sum_g coeff_fwd(g,l) * u(g,t) <= line_fwd_rhs[l,t]
    //   Reverse: sum_g coeff_rev(g,l) * u(g,t) <= line_rev_rhs[l,t]
    // where coeff_fwd(g,l) = max(0,gsf)*pmin[g] + min(0,gsf)*pmax[g] and
    //       coeff_rev(g,l) = max(0,-gsf)*pmin[g] + min(0,-gsf)*pmax[g].
    // Lines with sentinel RHS (1e8+) are unconstrained and skipped.
    if (uc.certifies_hard_network_flow_rows &&
        uc.network_line_count > 0 && !uc.line_gsf.empty() &&
        !uc.line_fwd_rhs.empty() && !uc.line_rev_rhs.empty() &&
        !uc.pmax.empty() && !uc.pmin.empty()) {
      const int nl = uc.network_line_count;
      int c_cuts = 0;
      std::vector<int>    cidx;
      std::vector<double> cval;
      for (int l = 0; l < nl; ++l) {
        for (int t = 0; t < T_uc; ++t) {
          const double fwd_rhs = uc.line_fwd_rhs[static_cast<size_t>(l * T_uc + t)];
          const double rev_rhs = uc.line_rev_rhs[static_cast<size_t>(l * T_uc + t)];

          // Forward capacity cut: sum_{gsf>0} gsf*pmin*u + sum_{gsf<0} gsf*pmax*u <= fwd_rhs
          // Derived from: flow_min_from_committed <= actual_flow <= fwd_rhs
          //
          // Density filter: drop positive terms with coeff < 1 MW.  Dropping a
          // positive LHS term only weakens (relaxes) the cut, so validity is
          // preserved.  Negative terms are always kept (dropping them would
          // tighten the cut and could cut off feasible solutions).
          // All non-empty cuts are added regardless of trivial satisfaction;
          // trivially-satisfied cuts still provide LP dual information that
          // helps Gurobi's internal cut separation.
          if (fwd_rhs < 1e8) {
            cidx.clear(); cval.clear();
            for (int g = 0; g < ng; ++g) {
              const double gsf = uc.line_gsf[static_cast<size_t>(l * ng + g)];
              if (std::abs(gsf) < 1e-10) continue;
              const int ig_col = uc.ig_cols[static_cast<size_t>(t * ng + g)];
              if (ig_col < 0) continue;
              const double coeff = (gsf > 0.0 ? gsf * uc.pmin[g] : gsf * uc.pmax[g]);
              if (std::abs(coeff) < 1e-10) continue;
              if (coeff > 0.0 && coeff < 1.0) continue;  // drop small positive terms (< 1 MW)
              cidx.push_back(ig_col);
              cval.push_back(coeff);
            }
            if (!cidx.empty()) {
              GRBaddconstr(model, static_cast<int>(cidx.size()),
                           cidx.data(), cval.data(),
                           GRB_LESS_EQUAL, fwd_rhs, nullptr);
              ++c_cuts;
            }
          }

          // Reverse capacity cut: sum_{gsf<0} (-gsf)*pmin*u + sum_{gsf>0} (-gsf)*pmax*u <= rev_rhs
          if (rev_rhs < 1e8) {
            cidx.clear(); cval.clear();
            for (int g = 0; g < ng; ++g) {
              const double neg_gsf = -uc.line_gsf[static_cast<size_t>(l * ng + g)];
              if (std::abs(neg_gsf) < 1e-10) continue;
              const int ig_col = uc.ig_cols[static_cast<size_t>(t * ng + g)];
              if (ig_col < 0) continue;
              const double coeff = (neg_gsf > 0.0 ? neg_gsf * uc.pmin[g] : neg_gsf * uc.pmax[g]);
              if (std::abs(coeff) < 1e-10) continue;
              if (coeff > 0.0 && coeff < 1.0) continue;  // drop small positive terms (< 1 MW)
              cidx.push_back(ig_col);
              cval.push_back(coeff);
            }
            if (!cidx.empty()) {
              GRBaddconstr(model, static_cast<int>(cidx.size()),
                           cidx.data(), cval.data(),
                           GRB_LESS_EQUAL, rev_rhs, nullptr);
              ++c_cuts;
            }
          }
        }
      }
      if (std::getenv("MIPSOLVERS_GUROBI_VERBOSE") != nullptr)
        std::fprintf(stderr, "[Gurobi] tight SCUC: %d Family C network-line"
                     " capacity cuts added statically\n", c_cuts);
    }
  }

  GRBupdatemodel(model);

  // --- Solver options ---
  if (GRBenv* model_env = GRBgetenv(model)) {
    // Tighten default 1e-4 MIPGap for oracle-quality solves.
    GRBsetdblparam(model_env, "MIPGap", 1e-9);
    GRBsetdblparam(model_env, "MIPGapAbs", 1e-6);

    // BranchDir=1: branch toward commitment (u=1) first.  For UC problems the
    // optimal solution has most generators committed; branching up first finds
    // good primal solutions earlier, enabling tighter pruning of the B&B tree.
    // Heuristics=0.15: modest increase from default 0.05 to allocate more of
    // the B&B time to primal improvement heuristics (RINS, local branching)
    // so the optimal incumbent is found earlier, enabling tighter pruning.
    if (std::getenv("MIPSOLVERS_GUROBI_TIGHT_SCUC") != nullptr) {
      GRBsetintparam(model_env, "BranchDir", 1);
      GRBsetdblparam(model_env, "Heuristics", 0.15);
    }

    // Optional time limit from env var MIPSOLVERS_GUROBI_TIME_LIMIT (seconds).
    if (const char* tl_env = std::getenv("MIPSOLVERS_GUROBI_TIME_LIMIT")) {
      char* endp = nullptr;
      const double tl = std::strtod(tl_env, &endp);
      if (endp != tl_env && tl > 0.0)
        GRBsetdblparam(model_env, "TimeLimit", tl);
    }

    // Output to stderr when MIPSOLVERS_GUROBI_VERBOSE is set.
    if (std::getenv("MIPSOLVERS_GUROBI_VERBOSE") != nullptr)
      GRBsetintparam(model_env, "OutputFlag", 1);

    // PreCrush=1 allows Gurobi to presolve user cuts (required for GRBcbcut).
    // Only set when the callback is expected.
    if (prob.uc_hint.has_value() &&
        (prob.uc_hint->certifies_min_up_down_rows ||
         prob.uc_hint->certifies_hard_network_flow_rows))
      GRBsetintparam(model_env, "PreCrush", 1);
    if (separate_gurobi_network)
      GRBsetintparam(model_env, "LazyConstraints", 1);
  }

  // Branching priorities: set from MIPModel::branching_priority when provided.
  if (!prob.branching_priority.empty()) {
    const int prio_n = std::min(n, static_cast<int>(prob.branching_priority.size()));
    std::vector<int> priorities(static_cast<std::size_t>(prio_n));
    for (int j = 0; j < prio_n; ++j)
      priorities[static_cast<std::size_t>(j)] = prob.branching_priority[static_cast<std::size_t>(j)];
    GRBsetintattrarray(model, "BranchPriority", 0, prio_n, priorities.data());
  }

  // Warm start.
  if (static_cast<int>(prob.initial_solution.size()) == n) {
    GRBsetdblattrarray(model, "Start", 0, n,
                       const_cast<double*>(prob.initial_solution.data()));
  }

  // Dynamic SCUC callback: inject violated min-up/down conflict cuts as user
  // cuts at every LP-optimal MIPNODE.  Disable with MIPSOLVERS_GUROBI_NO_DYNAMIC_CUTS.
  GurobiScucCutData cut_data;
  const bool analyze_scuc_gap =
      prob.uc_hint.has_value() &&
      std::getenv("MIPSOLVERS_GUROBI_ANALYZE_GAP") != nullptr;
  const bool enable_dynamic_scuc_cuts =
      prob.uc_hint.has_value() &&
      (prob.uc_hint->certifies_min_up_down_rows ||
       (prob.uc_hint->certifies_hard_network_flow_rows &&
        prob.uc_hint->network_line_count > 0 &&
        !prob.uc_hint->line_gsf.empty())) &&
      std::getenv("MIPSOLVERS_GUROBI_NO_DYNAMIC_CUTS") == nullptr;
  const bool enable_cb = enable_dynamic_scuc_cuts || analyze_scuc_gap || separate_gurobi_network;
  if (enable_cb) {
    cut_data.n = n;
    cut_data.uc = &(*prob.uc_hint);
    cut_data.enable_dynamic_cuts = enable_dynamic_scuc_cuts;
    cut_data.separate_network = separate_gurobi_network;
    if (const char* tol_env = std::getenv("MIPSOLVERS_GUROBI_NETWORK_CUT_TOL")) {
      char* endp = nullptr;
      const double value = std::strtod(tol_env, &endp);
      if (endp != tol_env && value > 0.0)
        cut_data.network_cut_tol = value;
    }
    if (const char* cap_env = std::getenv("MIPSOLVERS_GUROBI_NETWORK_USER_CUTS")) {
      char* endp = nullptr;
      const long value = std::strtol(cap_env, &endp, 10);
      if (endp != cap_env && value >= 0)
        cut_data.max_network_user_cuts_per_call = static_cast<int>(std::min<long>(value, 100000));
    }
    if (const char* cap_env = std::getenv("MIPSOLVERS_GUROBI_NETWORK_LAZY_CUTS")) {
      char* endp = nullptr;
      const long value = std::strtol(cap_env, &endp, 10);
      if (endp != cap_env && value >= 0)
        cut_data.max_network_lazy_cuts_per_call = static_cast<int>(std::min<long>(value, 100000));
    }
    if (separate_gurobi_network) {
      const int row_count = prob.uc_hint->network_flow_row_count;
      cut_data.network_user_cut_seen.assign(static_cast<std::size_t>(row_count), 0);
      cut_data.network_lazy_cut_seen.assign(static_cast<std::size_t>(row_count), 0);
      if (std::getenv("MIPSOLVERS_GUROBI_VERBOSE") != nullptr) {
        std::fprintf(stderr,
                     "[Gurobi] lazy network: skipped %d upfront PTDF rows, callback rows=%d\n",
                     skipped_network_rows, row_count);
      }
    }
    cut_data.analyze_gap = analyze_scuc_gap;
    if (const char* gap_env = std::getenv("MIPSOLVERS_GUROBI_ANALYZE_GAP_TRIGGER")) {
      char* endp = nullptr;
      const double value = std::strtod(gap_env, &endp);
      if (endp != gap_env && value > 0.0)
        cut_data.analyze_gap_trigger = value;
    }
    if (const char* top_env = std::getenv("MIPSOLVERS_GUROBI_ANALYZE_TOP")) {
      char* endp = nullptr;
      const long value = std::strtol(top_env, &endp, 10);
      if (endp != top_env && value > 0)
        cut_data.analyze_top = static_cast<int>(std::min<long>(value, 200));
    }
    GRBsetcallbackfunc(model, gurobi_scuc_cut_callback, &cut_data);
  }

  GRBoptimize(model);

  int status = 0;
  GRBgetintattr(model, "Status", &status);
  int sol_count = 0;
  GRBgetintattr(model, "SolCount", &sol_count);
  const bool has_incumbent = sol_count > 0;

  if (status == GRB_OPTIMAL) {
    out.stats.success = true;
    out.stats.status = "Optimal";
  } else if ((status == GRB_TIME_LIMIT || status == GRB_NODE_LIMIT) && has_incumbent) {
    out.stats.success = true;
    out.stats.status = (status == GRB_TIME_LIMIT) ? "Time limit" : "Node limit";
  } else if (status == GRB_INFEASIBLE) {
    out.stats.status = "Infeasible";
  } else {
    out.stats.status = "Gurobi status=" + std::to_string(status);
  }

  if (has_incumbent && out.stats.success) {
    double objval = 0.0;
    GRBgetdblattr(model, "ObjVal", &objval);
    out.stats.objective = obj_sign * objval;
    out.x.resize(n);
    GRBgetdblattrarray(model, "X", 0, n, out.x.data());
    double mip_gap = 0.0;
    if (GRBgetdblattr(model, "MIPGap", &mip_gap) == 0)
      out.stats.mip_gap = mip_gap;

    // Extract constraint duals (Pi) for pure LP problems.
    if (prob.binary_idx.empty() && prob.integer_idx.empty()) {
      const int m_total = m_ineq + m_eq;
      out.constraint_duals.resize(m_total);
      if (GRBgetdblattrarray(model, "Pi", 0, m_total,
                             out.constraint_duals.data()) == 0) {
        out.constraint_duals *= obj_sign;
      } else {
        out.constraint_duals.resize(0);
      }
    }
  }

  // Report dynamic cuts injected via callback.
  if (enable_cb && cut_data.total_cuts_added > 0)
    out.stats.cglp_cuts_added = cut_data.total_cuts_added;
  if (separate_gurobi_network && std::getenv("MIPSOLVERS_GUROBI_VERBOSE") != nullptr) {
    std::fprintf(stderr,
                 "[Gurobi] lazy network: user_cuts=%d lazy_cuts=%d total_callback_rows=%d\n",
                 cut_data.network_user_cuts_added, cut_data.network_lazy_cuts_added,
                 cut_data.total_cuts_added);
  }

  const auto t1 = std::chrono::steady_clock::now();
  out.stats.runtime_sec = std::chrono::duration<double>(t1 - t0).count();
  GRBfreemodel(model);
#else
  out.stats.status = "Unavailable: Gurobi not linked";
#endif
  return out;
}

}  // namespace mipsolvers::engine
