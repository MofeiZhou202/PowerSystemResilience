#include "mipsolvers/engine/decomposition/benders.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "mipsolvers/engine/bc/api.hpp"

namespace mipsolvers::engine {
namespace {

using Clock = std::chrono::steady_clock;

double relative_gap(double upper, double lower) {
  if (!std::isfinite(upper) || !std::isfinite(lower))
    return std::numeric_limits<double>::infinity();
  return std::max(0.0, upper - lower) /
         std::max(1.0, std::abs(upper));
}

bool master_proved_optimal(const BCResult& result) {
  return result.stats.success &&
         result.stats.status.find("Optimal") != std::string::npos;
}

std::optional<BendersCut> make_binary_no_good_cut(
    const MIPModel& master, int theta_col, const Eigen::VectorXd& x) {
  const int cols = static_cast<int>(master.linear_part.c.size());
  if (x.size() != cols) return std::nullopt;
  std::vector<char> binary(static_cast<std::size_t>(cols), 0);
  for (int col : master.binary_idx) {
    if (col >= 0 && col < cols) binary[static_cast<std::size_t>(col)] = 1;
  }

  BendersCut cut;
  cut.terms.reserve(static_cast<std::size_t>(std::max(0, cols - 1)));
  int ones = 0;
  for (int col = 0; col < cols; ++col) {
    if (col == theta_col) continue;
    const bool is_binary = binary[static_cast<std::size_t>(col)] ||
        (col < static_cast<int>(master.linear_part.vars.size()) &&
         master.linear_part.vars[static_cast<std::size_t>(col)].type ==
             VarType::Binary);
    if (!is_binary) return std::nullopt;
    if (x[col] >= 0.5) {
      cut.terms.emplace_back(col, 1.0);
      ++ones;
    } else {
      cut.terms.emplace_back(col, -1.0);
    }
  }
  if (cut.terms.empty()) return std::nullopt;
  cut.rhs = static_cast<double>(ones - 1);
  return cut;
}

void append_cut(MIPModel& master, const BendersCut& cut) {
  auto& lp = master.linear_part;
  const int row = static_cast<int>(lp.A.rows());
  lp.A.conservativeResize(row + 1, lp.c.size());
  lp.b.conservativeResize(row + 1);
  lp.b[row] = cut.rhs;

  constexpr int kColumnReserveChunk = 4;
  Eigen::VectorXi additional = Eigen::VectorXi::Zero(lp.A.outerSize());
  bool needs_reserve = false;
  const bool compressed = lp.A.isCompressed();
  const auto* outer = lp.A.outerIndexPtr();
  const auto* used = lp.A.innerNonZeroPtr();
  for (const auto& [col, value] : cut.terms) {
    if (col < 0 || col >= lp.c.size() || value == 0.0) continue;
    const bool column_full =
        compressed || outer[col + 1] - outer[col] <= used[col];
    if (column_full && additional[col] == 0) {
      additional[col] = kColumnReserveChunk;
      needs_reserve = true;
    }
  }
  if (needs_reserve) lp.A.reserve(additional);

  for (const auto& [col, value] : cut.terms) {
    if (col >= 0 && col < lp.c.size() && value != 0.0)
      lp.A.coeffRef(row, col) += value;
  }
  // Solver and presolve backends consume canonical CSC.  Eigen's
  // uncompressed representation exposes reserved gaps through outer offsets,
  // which is not a portable input contract for those backends.
  lp.A.makeCompressed();
}

void sparsify_cut(BendersCut& cut, const LPModel& master,
                  double coefficient_tolerance) {
  if (coefficient_tolerance <= 0.0) return;
  std::vector<std::pair<int, double>> kept;
  kept.reserve(cut.terms.size());
  for (const auto& term : cut.terms) {
    const int col = term.first;
    const double value = term.second;
    if (std::abs(value) > coefficient_tolerance || col < 0 ||
        col >= static_cast<int>(master.vars.size())) {
      kept.push_back(term);
      continue;
    }
    const auto& var = master.vars[static_cast<std::size_t>(col)];
    const double minimizing_bound = value >= 0.0 ? var.lb : var.ub;
    if (!std::isfinite(minimizing_bound)) {
      kept.push_back(term);
      continue;
    }
    // Projecting a removed term over its box requires the weakest valid RHS.
    cut.rhs -= value * minimizing_bound;
  }
  cut.terms = std::move(kept);
}

void update_standard_form_rhs(StandardFormLP& sf, int row, double rhs) {
  if (row < 0 || row >= sf.b.size() || row >= sf.row_rhs_value.size())
    throw std::out_of_range("Benders coupling row is outside standard form");
  const double old_rhs = sf.row_rhs_value[row];
  if (old_rhs == rhs) return;
  const double row_scale =
      sf.row_scale.size() == sf.b.size() ? sf.row_scale[row] : 1.0;
  const double row_sign = row < static_cast<int>(sf.row_sign.size())
                              ? static_cast<double>(sf.row_sign[row])
                              : 1.0;
  sf.b[row] += row_sign * row_scale * (rhs - old_rhs);
  sf.row_rhs_value[row] = rhs;
}

std::string validate_model(const BendersModel& model) {
  const auto& master = model.master.linear_part;
  const auto& subproblem = model.subproblem;
  const int master_cols = static_cast<int>(model.master.linear_part.c.size());
  const int sub_cols = static_cast<int>(model.subproblem.c.size());
  if (master_cols <= 0) return "Benders master has no columns";
  if (sub_cols <= 0) return "Benders subproblem has no columns";
  if (master.vars.size() != static_cast<std::size_t>(master_cols) ||
      master.A.cols() != master_cols || master.Aeq.cols() != master_cols ||
      master.b.size() != master.A.rows() ||
      master.beq.size() != master.Aeq.rows() ||
      (master.row_lhs.size() != 0 &&
       master.row_lhs.size() != master.A.rows()))
    return "Benders master dimensions are inconsistent";
  if (subproblem.vars.size() != static_cast<std::size_t>(sub_cols) ||
      subproblem.A.cols() != sub_cols ||
      subproblem.Aeq.cols() != sub_cols ||
      subproblem.b.size() != subproblem.A.rows() ||
      subproblem.beq.size() != subproblem.Aeq.rows() ||
      (subproblem.row_lhs.size() != 0 &&
       subproblem.row_lhs.size() != subproblem.A.rows()))
    return "Benders subproblem dimensions are inconsistent";
  if (master.sense != Sense::Minimize ||
      subproblem.sense != Sense::Minimize)
    return "Benders currently requires minimization master and subproblem";
  if (model.theta_col < 0 || model.theta_col >= master_cols)
    return "Benders theta_col is outside the master";
  if (std::abs(master.c[model.theta_col] - 1.0) > 1e-12)
    return "Benders theta objective coefficient must equal 1";
  if (model.theta_col >=
          static_cast<int>(master.vars.size()) ||
      master.vars[static_cast<std::size_t>(model.theta_col)]
              .type != VarType::Continuous)
    return "Benders theta variable must be continuous";
  for (const auto& var : subproblem.vars) {
    if (var.type != VarType::Continuous)
      return "Benders subproblem must contain only continuous variables";
  }
  if (model.coupling_ineq.rows() != model.subproblem.A.rows() ||
      model.coupling_ineq.cols() != master_cols)
    return "Benders inequality coupling dimensions do not match";
  if (model.coupling_eq.rows() != model.subproblem.Aeq.rows() ||
      model.coupling_eq.cols() != master_cols)
    return "Benders equality coupling dimensions do not match";
  for (Eigen::SparseMatrix<double>::InnerIterator it(
           model.coupling_ineq, model.theta_col); it; ++it) {
    if (it.value() != 0.0) return "Benders theta column cannot couple to the LP";
  }
  for (Eigen::SparseMatrix<double>::InnerIterator it(
           model.coupling_eq, model.theta_col); it; ++it) {
    if (it.value() != 0.0) return "Benders theta column cannot couple to the LP";
  }
  return {};
}

}  // namespace

BendersPartition partition_benders_model(
    MIPModel source, const std::vector<int>& master_columns,
    double coefficient_tolerance) {
  BendersPartition out;
  auto fail = [&](std::string message) {
    out.status = std::move(message);
    return std::move(out);
  };

  LPModel& source_lp = source.linear_part;
  const int original_cols = static_cast<int>(source_lp.c.size());
  out.original_variables = original_cols;
  if (source_lp.sense != Sense::Minimize)
    return fail("Benders partition currently requires minimization");
  if (original_cols <= 0 || source_lp.vars.size() !=
                                static_cast<std::size_t>(original_cols))
    return fail("Benders partition source columns are inconsistent");
  if (source_lp.A.cols() != original_cols ||
      source_lp.Aeq.cols() != original_cols ||
      source_lp.b.size() != source_lp.A.rows() ||
      source_lp.beq.size() != source_lp.Aeq.rows())
    return fail("Benders partition source matrix dimensions are inconsistent");
  if (master_columns.empty())
    return fail("Benders partition requires at least one master column");
  if (coefficient_tolerance < 0.0 ||
      !std::isfinite(coefficient_tolerance))
    return fail("Benders partition coefficient tolerance is invalid");

  const int master_cols = static_cast<int>(master_columns.size());
  std::vector<int> original_to_master(static_cast<std::size_t>(original_cols),
                                      -1);
  out.master_to_original = master_columns;
  for (int col = 0; col < master_cols; ++col) {
    const int original = master_columns[static_cast<std::size_t>(col)];
    if (original < 0 || original >= original_cols)
      return fail("Benders partition master column is out of range");
    if (original_to_master[static_cast<std::size_t>(original)] >= 0)
      return fail("Benders partition master columns contain a duplicate");
    original_to_master[static_cast<std::size_t>(original)] = col;
  }

  std::vector<char> discrete(static_cast<std::size_t>(original_cols), 0);
  std::vector<char> binary(static_cast<std::size_t>(original_cols), 0);
  for (int col : source.integer_idx) {
    if (col >= 0 && col < original_cols)
      discrete[static_cast<std::size_t>(col)] = 1;
  }
  for (int col : source.binary_idx) {
    if (col >= 0 && col < original_cols) {
      discrete[static_cast<std::size_t>(col)] = 1;
      binary[static_cast<std::size_t>(col)] = 1;
    }
  }

  std::vector<int> original_to_subproblem(
      static_cast<std::size_t>(original_cols), -1);
  out.subproblem_to_original.reserve(
      static_cast<std::size_t>(original_cols - master_cols));
  for (int original = 0; original < original_cols; ++original) {
    if (original_to_master[static_cast<std::size_t>(original)] >= 0) continue;
    const auto& var = source_lp.vars[static_cast<std::size_t>(original)];
    if (discrete[static_cast<std::size_t>(original)] ||
        var.type != VarType::Continuous)
      return fail("Benders recourse contains a non-continuous variable");
    original_to_subproblem[static_cast<std::size_t>(original)] =
        static_cast<int>(out.subproblem_to_original.size());
    out.subproblem_to_original.push_back(original);
  }
  if (out.subproblem_to_original.empty())
    return fail("Benders partition has no recourse columns");

  BendersModel& model = out.model;
  LPModel& master = model.master.linear_part;
  LPModel& subproblem = model.subproblem;
  model.theta_col = master_cols;
  master.sense = Sense::Minimize;
  master.c = Eigen::VectorXd::Zero(master_cols + 1);
  master.vars.resize(static_cast<std::size_t>(master_cols + 1));
  for (int col = 0; col < master_cols; ++col) {
    const int original = master_columns[static_cast<std::size_t>(col)];
    master.c[col] = source_lp.c[original];
    master.vars[static_cast<std::size_t>(col)] =
        std::move(source_lp.vars[static_cast<std::size_t>(original)]);
    if (binary[static_cast<std::size_t>(original)] ||
        master.vars[static_cast<std::size_t>(col)].type == VarType::Binary)
      model.master.binary_idx.push_back(col);
    else if (discrete[static_cast<std::size_t>(original)] ||
             master.vars[static_cast<std::size_t>(col)].type == VarType::Integer)
      model.master.integer_idx.push_back(col);
  }

  double recourse_lower_bound = 0.0;
  bool finite_recourse_lower_bound = true;
  const int sub_cols = static_cast<int>(out.subproblem_to_original.size());
  subproblem.sense = Sense::Minimize;
  subproblem.c.resize(sub_cols);
  subproblem.vars.resize(static_cast<std::size_t>(sub_cols));
  for (int col = 0; col < sub_cols; ++col) {
    const int original =
        out.subproblem_to_original[static_cast<std::size_t>(col)];
    auto& var = source_lp.vars[static_cast<std::size_t>(original)];
    const double cost = source_lp.c[original];
    subproblem.c[col] = cost;
    const double bound = cost >= 0.0 ? var.lb : var.ub;
    if (!std::isfinite(bound)) {
      finite_recourse_lower_bound = false;
    } else if (finite_recourse_lower_bound) {
      recourse_lower_bound += cost * bound;
    }
    subproblem.vars[static_cast<std::size_t>(col)] = std::move(var);
  }
  master.c[model.theta_col] = 1.0;
  master.vars[static_cast<std::size_t>(model.theta_col)] = {
      VarType::Continuous,
      finite_recourse_lower_bound ? recourse_lower_bound : -1.0e20, 1.0e20,
      "benders_theta"};

  auto partition_rows = [&](const Eigen::SparseMatrix<double>& source_matrix,
                            const Eigen::VectorXd& source_rhs,
                            const Eigen::VectorXd* source_lhs,
                            Eigen::SparseMatrix<double>& master_matrix,
                            Eigen::VectorXd& master_rhs,
                            Eigen::VectorXd* master_lhs,
                            Eigen::SparseMatrix<double>& sub_matrix,
                            Eigen::VectorXd& sub_rhs,
                            Eigen::VectorXd* sub_lhs,
                            Eigen::SparseMatrix<double>& coupling) {
    const int source_rows = static_cast<int>(source_matrix.rows());
    std::vector<char> has_master(static_cast<std::size_t>(source_rows), 0);
    std::vector<char> has_recourse(static_cast<std::size_t>(source_rows), 0);
    for (int original = 0; original < source_matrix.outerSize(); ++original) {
      const bool is_master =
          original_to_master[static_cast<std::size_t>(original)] >= 0;
      for (Eigen::SparseMatrix<double>::InnerIterator it(source_matrix,
                                                          original);
           it; ++it) {
        if (std::abs(it.value()) <= coefficient_tolerance) continue;
        if (is_master)
          has_master[static_cast<std::size_t>(it.row())] = 1;
        else
          has_recourse[static_cast<std::size_t>(it.row())] = 1;
      }
    }

    std::vector<int> master_row(static_cast<std::size_t>(source_rows), -1);
    std::vector<int> sub_row(static_cast<std::size_t>(source_rows), -1);
    int master_rows = 0;
    int sub_rows = 0;
    for (int row = 0; row < source_rows; ++row) {
      if (has_master[static_cast<std::size_t>(row)] &&
          !has_recourse[static_cast<std::size_t>(row)])
        master_row[static_cast<std::size_t>(row)] = master_rows++;
      else
        sub_row[static_cast<std::size_t>(row)] = sub_rows++;
    }

    master_rhs.resize(master_rows);
    sub_rhs.resize(sub_rows);
    if (source_lhs && master_lhs) master_lhs->resize(master_rows);
    if (source_lhs && sub_lhs) sub_lhs->resize(sub_rows);
    for (int row = 0; row < source_rows; ++row) {
      const int mr = master_row[static_cast<std::size_t>(row)];
      if (mr >= 0) {
        master_rhs[mr] = source_rhs[row];
        if (source_lhs && master_lhs) (*master_lhs)[mr] = (*source_lhs)[row];
      } else {
        const int sr = sub_row[static_cast<std::size_t>(row)];
        sub_rhs[sr] = source_rhs[row];
        if (source_lhs && sub_lhs) (*sub_lhs)[sr] = (*source_lhs)[row];
      }
    }

    std::size_t master_nnz = 0;
    std::size_t sub_nnz = 0;
    std::size_t coupling_nnz = 0;
    for (int original = 0; original < source_matrix.outerSize(); ++original) {
      const bool is_master =
          original_to_master[static_cast<std::size_t>(original)] >= 0;
      for (Eigen::SparseMatrix<double>::InnerIterator it(source_matrix,
                                                          original);
           it; ++it) {
        if (std::abs(it.value()) <= coefficient_tolerance) continue;
        if (master_row[static_cast<std::size_t>(it.row())] >= 0)
          ++master_nnz;
        else if (is_master)
          ++coupling_nnz;
        else
          ++sub_nnz;
      }
    }
    std::vector<Eigen::Triplet<double>> master_terms;
    std::vector<Eigen::Triplet<double>> sub_terms;
    std::vector<Eigen::Triplet<double>> coupling_terms;
    master_terms.reserve(master_nnz);
    sub_terms.reserve(sub_nnz);
    coupling_terms.reserve(coupling_nnz);
    for (int original = 0; original < source_matrix.outerSize(); ++original) {
      const int mc = original_to_master[static_cast<std::size_t>(original)];
      const int sc = original_to_subproblem[static_cast<std::size_t>(original)];
      for (Eigen::SparseMatrix<double>::InnerIterator it(source_matrix,
                                                          original);
           it; ++it) {
        if (std::abs(it.value()) <= coefficient_tolerance) continue;
        const int mr = master_row[static_cast<std::size_t>(it.row())];
        if (mr >= 0) {
          if (mc >= 0) master_terms.emplace_back(mr, mc, it.value());
          continue;
        }
        const int sr = sub_row[static_cast<std::size_t>(it.row())];
        if (mc >= 0)
          coupling_terms.emplace_back(sr, mc, it.value());
        else if (sc >= 0)
          sub_terms.emplace_back(sr, sc, it.value());
      }
    }

    master_matrix.resize(master_rows, master_cols + 1);
    master_matrix.setFromTriplets(master_terms.begin(), master_terms.end());
    sub_matrix.resize(sub_rows, sub_cols);
    sub_matrix.setFromTriplets(sub_terms.begin(), sub_terms.end());
    coupling.resize(sub_rows, master_cols + 1);
    coupling.setFromTriplets(coupling_terms.begin(), coupling_terms.end());
  };

  const Eigen::VectorXd* source_lhs =
      lp_has_row_lhs(source_lp) ? &source_lp.row_lhs : nullptr;
  partition_rows(source_lp.A, source_lp.b, source_lhs, master.A, master.b,
                 &master.row_lhs, subproblem.A, subproblem.b,
                 &subproblem.row_lhs, model.coupling_ineq);
  source_lp.A = {};
  source_lp.b.resize(0);
  source_lp.row_lhs.resize(0);
  partition_rows(source_lp.Aeq, source_lp.beq, nullptr, master.Aeq, master.beq,
                 nullptr, subproblem.Aeq, subproblem.beq, nullptr,
                 model.coupling_eq);
  source_lp.Aeq = {};
  source_lp.beq.resize(0);

  out.success = true;
  out.status = "Partitioned";
  return out;
}

BendersResult solve_benders(BendersModel model,
                            const BendersOptions& options) {
  BendersResult out;
  const std::string validation_error = validate_model(model);
  if (!validation_error.empty()) {
    out.status = validation_error;
    return out;
  }

  const auto started = Clock::now();
  const int master_cols = static_cast<int>(model.master.linear_part.c.size());
  const int sub_ineq_rows = static_cast<int>(model.subproblem.A.rows());
  const int sub_eq_rows = static_cast<int>(model.subproblem.Aeq.rows());
  const int sub_cols = static_cast<int>(model.subproblem.c.size());

  std::vector<int> coupled_cols;
  coupled_cols.reserve(static_cast<std::size_t>(master_cols));
  for (int col = 0; col < master_cols; ++col) {
    const bool coupled_ineq =
        Eigen::SparseMatrix<double>::InnerIterator(model.coupling_ineq, col);
    const bool coupled_eq =
        Eigen::SparseMatrix<double>::InnerIterator(model.coupling_eq, col);
    if (coupled_ineq || coupled_eq) {
      coupled_cols.push_back(col);
    }
  }
  out.stats.coupled_master_variables = static_cast<int>(coupled_cols.size());
  out.stats.coupling_nonzeros =
      static_cast<long long>(model.coupling_ineq.nonZeros()) +
      static_cast<long long>(model.coupling_eq.nonZeros());

  StandardFormLP sub_sf = build_standard_form_lp(model.subproblem);
  if (options.scaling_rounds > 0)
    ruiz_scale_standard_form(sub_sf, options.scaling_rounds);

  Eigen::VectorXd previous_master = Eigen::VectorXd::Zero(master_cols);
  Eigen::VectorXd latest_master;
  Eigen::VectorXd best_master;
  Eigen::VectorXd best_subproblem;
  std::shared_ptr<const BCPseudocostInit> pseudocost;
  SimplexBasis sub_basis;
  bool have_sub_basis = false;
  bool rebuild_subproblem = false;

  auto remaining_seconds = [&]() {
    const double elapsed =
        std::chrono::duration<double>(Clock::now() - started).count();
    return options.time_limit_sec > 0.0
               ? std::max(0.0, options.time_limit_sec - elapsed)
               : std::numeric_limits<double>::infinity();
  };

  const int max_iterations = std::max(1, options.max_iterations);
  for (int iteration = 0; iteration < max_iterations; ++iteration) {
    if (remaining_seconds() <= 0.0) {
      out.status = "Benders time limit";
      break;
    }
    ++out.stats.iterations;

    const Eigen::VectorXd* incumbent =
        options.reuse_master_incumbent && best_master.size() == master_cols
            ? &best_master
            : nullptr;
    if (incumbent) {
      model.master.initial_solution = *incumbent;
      ++out.stats.master_incumbent_warm_starts;
    } else {
      model.master.initial_solution.resize(0);
    }

    BCOptions master_options = options.master_options;
    master_options.verbose = options.verbose || master_options.verbose;
    master_options.time_limit_sec = remaining_seconds();
    master_options.highs_pseudocost_warm_start = pseudocost;
    if (pseudocost) ++out.stats.master_pseudocost_reuses;
    if (incumbent) {
      master_options.root_cut_rounds = options.warm_master_cut_rounds;
      master_options.accept_verified_warm_start_incumbent = true;
      if (options.disable_redundant_warm_heuristics) {
        master_options.use_feasibility_pump = false;
        master_options.use_progressive_rounding = false;
        master_options.enable_feasibility_jump = false;
      }
    } else {
      master_options.root_cut_rounds = options.cold_master_cut_rounds;
    }

    const auto master_started = Clock::now();
    BCResult master_result = solve_milp_bc(model.master, master_options);
    out.stats.master_solve_time_sec +=
        std::chrono::duration<double>(Clock::now() - master_started).count();
    out.stats.master_nodes += master_result.bc_stats.nodes_explored;
    out.stats.master_lp_solves += master_result.bc_stats.lp_solves;
    if (master_result.highs_pseudocost_init &&
        !master_result.highs_pseudocost_init->empty()) {
      pseudocost = std::move(master_result.highs_pseudocost_init);
    }
    if (!master_result.stats.success || master_result.x.size() != master_cols) {
      out.status = "Benders master failed: " + master_result.stats.status;
      break;
    }
    latest_master = std::move(master_result.x);
    if (master_proved_optimal(master_result)) {
      out.lower_bound = std::max(out.lower_bound,
                                 master_result.stats.objective);
    }
    out.relative_gap = relative_gap(out.upper_bound, out.lower_bound);
    if (master_proved_optimal(master_result) && out.has_incumbent &&
        out.relative_gap <= options.gap_tolerance) {
      out.success = true;
      out.status = "Optimal";
      break;
    }

    if (rebuild_subproblem) {
      LPModel current_subproblem = model.subproblem;
      const Eigen::VectorXd coupled_ineq_rhs =
          model.coupling_ineq * latest_master;
      current_subproblem.b -= coupled_ineq_rhs;
      if (current_subproblem.row_lhs.size() ==
          current_subproblem.b.size()) {
        current_subproblem.row_lhs -= coupled_ineq_rhs;
      }
      current_subproblem.beq -= model.coupling_eq * latest_master;
      sub_sf = build_standard_form_lp(current_subproblem);
      if (options.scaling_rounds > 0)
        ruiz_scale_standard_form(sub_sf, options.scaling_rounds);
      previous_master = latest_master;
      have_sub_basis = false;
      rebuild_subproblem = false;
    } else {
      for (int col : coupled_cols) {
        const double delta = latest_master[col] - previous_master[col];
        if (delta == 0.0) continue;
        for (Eigen::SparseMatrix<double>::InnerIterator it(
                 model.coupling_ineq, col); it; ++it) {
          update_standard_form_rhs(
              sub_sf, it.row(),
              sub_sf.row_rhs_value[it.row()] - it.value() * delta);
          ++out.stats.incremental_rhs_updates;
        }
        for (Eigen::SparseMatrix<double>::InnerIterator it(
                 model.coupling_eq, col); it; ++it) {
          const int sf_row = sub_ineq_rows + it.row();
          update_standard_form_rhs(
              sub_sf, sf_row,
              sub_sf.row_rhs_value[sf_row] - it.value() * delta);
          ++out.stats.incremental_rhs_updates;
        }
      }
      for (int col : coupled_cols) previous_master[col] = latest_master[col];
    }

    SimplexOptions sub_options = options.subproblem_options;
    sub_options.verbose = options.verbose || sub_options.verbose;
    sub_options.time_limit_sec = remaining_seconds();
    sub_options.prefer_dual_simplex_reopt = true;
    sub_options.allow_cold_start = true;
    sub_options.reuse_factorization = have_sub_basis;
    const auto sub_started = Clock::now();
    SimplexResult sub_result =
        solve_lp_from_sf(sub_sf, sub_options,
                         have_sub_basis ? &sub_basis : nullptr);
    out.stats.subproblem_solve_time_sec +=
        std::chrono::duration<double>(Clock::now() - sub_started).count();
    ++out.stats.subproblem_solves;
    out.stats.subproblem_simplex_iterations +=
        sub_result.result.stats.iterations;
    if (sub_result.solved_from_hint)
      ++out.stats.subproblem_basis_warm_starts;
    if (sub_result.result.stats.success &&
        sub_result.basis.rows == sub_sf.A.rows() &&
        sub_result.basis.cols == sub_sf.A.cols() &&
        sub_result.basis.index_count() ==
            static_cast<std::size_t>(sub_sf.A.rows())) {
      sub_basis = std::move(sub_result.basis);
      have_sub_basis = true;
    } else if (!sub_result.result.stats.success) {
      // A Phase-I/infeasibility basis is not a valid reoptimization hint for
      // the next master assignment. Reusing it can publish a stale feasible
      // solution after the coupled RHS changes.
      have_sub_basis = false;
    }

    const SolveResult& sub_solve = sub_result.result;
    if (sub_solve.stats.success && sub_solve.x.size() == sub_cols) {
      const double fixed_master_objective =
          model.master.linear_part.c.dot(latest_master) -
          latest_master[model.theta_col];
      const double candidate_objective =
          fixed_master_objective + sub_solve.stats.objective;
      if (candidate_objective < out.upper_bound) {
        out.upper_bound = candidate_objective;
        out.objective = candidate_objective;
        out.has_incumbent = true;
        best_master = latest_master;
        best_master[model.theta_col] =
            sub_solve.stats.objective +
            1e-6 * (1.0 + std::abs(sub_solve.stats.objective));
        best_subproblem = sub_solve.x;
      }

      if (sub_solve.constraint_duals.size() <
          sub_ineq_rows + sub_eq_rows) {
        out.status = "Benders LP optimum has no dual certificate";
        break;
      }
      BendersCut cut;
      cut.terms.reserve(coupled_cols.size() + 1);
      double rhs = -sub_solve.stats.objective;
      for (int col : coupled_cols) {
        double gradient = 0.0;
        for (Eigen::SparseMatrix<double>::InnerIterator it(
                 model.coupling_ineq, col); it; ++it) {
          gradient -= it.value() * sub_solve.constraint_duals[it.row()];
        }
        for (Eigen::SparseMatrix<double>::InnerIterator it(
                 model.coupling_eq, col); it; ++it) {
          gradient -= it.value() *
              sub_solve.constraint_duals[sub_ineq_rows + it.row()];
        }
        rhs += gradient * latest_master[col];
        if (gradient != 0.0)
          cut.terms.emplace_back(col, gradient);
      }
      cut.terms.emplace_back(model.theta_col, -1.0);
      cut.rhs = rhs;
      sparsify_cut(cut, model.master.linear_part,
                   options.coefficient_tolerance);
      if (options.verbose) {
        double activity = 0.0;
        for (const auto& [col, value] : cut.terms)
          activity += value * latest_master[col];
        std::fprintf(stderr,
                     "[Benders] optimality cut iteration=%d q=%.12g "
                     "activity=%.12g rhs=%.12g terms=%zu\n",
                     iteration + 1, sub_solve.stats.objective, activity,
                     cut.rhs, cut.terms.size());
      }
      append_cut(model.master, cut);
      ++out.stats.cuts_added;
    } else {
      const bool subproblem_infeasible =
          sub_solve.stats.has_farkas_certificate ||
          sub_solve.stats.status.find("infeasible") != std::string::npos ||
          sub_solve.stats.status.find("Infeasible") != std::string::npos;
      if (!subproblem_infeasible) {
        out.status = "Benders subproblem failed: " + sub_solve.stats.status;
        break;
      }
      std::optional<BendersCut> cut;
      if (sub_solve.stats.has_farkas_certificate &&
          sub_solve.stats.farkas_ray.size() >= sub_ineq_rows &&
          sub_solve.stats.farkas_ray_eq.size() >= sub_eq_rows) {
        // Reconstruct the Phase-I ray in the signed, scaled standard-form row
        // space.  The public ray has row_sign removed but intentionally keeps
        // Ruiz row scaling implicit.
        Eigen::VectorXd y_scaled(sub_sf.A.rows());
        for (int row = 0; row < sub_ineq_rows; ++row) {
          const double sign =
              row < static_cast<int>(sub_sf.row_sign.size())
                  ? static_cast<double>(sub_sf.row_sign[row])
                  : 1.0;
          y_scaled[row] = sign * sub_solve.stats.farkas_ray[row];
        }
        for (int row = 0; row < sub_eq_rows; ++row) {
          const int sf_row = sub_ineq_rows + row;
          const double sign =
              sf_row < static_cast<int>(sub_sf.row_sign.size())
                  ? static_cast<double>(sub_sf.row_sign[sf_row])
                  : 1.0;
          y_scaled[sf_row] =
              sign * sub_solve.stats.farkas_ray_eq[row];
        }

        // A Farkas row ray alone is insufficient when recourse columns or
        // ranged-row slacks have finite bounds.  Compute the minimum of
        // (A' y)'z over every non-artificial standard-form column.  This adds
        // the missing box-bound support directly from the standard-form bounds.
        const Eigen::VectorXd ray_column = sub_sf.A.transpose() * y_scaled;
        std::vector<char> artificial(
            static_cast<std::size_t>(sub_sf.A.cols()), 0);
        for (int col : sub_sf.row_to_artificial_col) {
          if (col >= 0 && col < sub_sf.A.cols())
            artificial[static_cast<std::size_t>(col)] = 1;
        }
        bool valid_certificate = ray_column.allFinite();
        double box_minimum = 0.0;
        for (int col = 0; valid_certificate && col < ray_column.size(); ++col) {
          if (artificial[static_cast<std::size_t>(col)] ||
              ray_column[col] >= -options.cut_tolerance)
            continue;
          const double upper =
              col < sub_sf.var_ub.size()
                  ? sub_sf.var_ub[col]
                  : std::numeric_limits<double>::infinity();
          if (!std::isfinite(upper)) {
            valid_certificate = false;
            break;
          }
          box_minimum += ray_column[col] * upper;
        }

        const double current_rhs = y_scaled.dot(sub_sf.b);
        const double violation = box_minimum - current_rhs;
        if (valid_certificate && std::isfinite(violation) &&
            violation > options.cut_tolerance) {
          BendersCut farkas_cut;
          farkas_cut.terms.reserve(coupled_cols.size());
          double base_rhs = current_rhs;
          for (int col : coupled_cols) {
            double rhs_coefficient = 0.0;
            for (Eigen::SparseMatrix<double>::InnerIterator it(
                     model.coupling_ineq, col); it; ++it) {
              const double row_scale =
                  sub_sf.row_scale.size() == sub_sf.b.size()
                      ? sub_sf.row_scale[it.row()]
                      : 1.0;
              rhs_coefficient -= it.value() * row_scale *
                  sub_solve.stats.farkas_ray[it.row()];
            }
            for (Eigen::SparseMatrix<double>::InnerIterator it(
                     model.coupling_eq, col); it; ++it) {
              const int sf_row = sub_ineq_rows + it.row();
              const double row_scale =
                  sub_sf.row_scale.size() == sub_sf.b.size()
                      ? sub_sf.row_scale[sf_row]
                      : 1.0;
              rhs_coefficient -= it.value() * row_scale *
                  sub_solve.stats.farkas_ray_eq[it.row()];
            }
            base_rhs -= rhs_coefficient * latest_master[col];
            if (rhs_coefficient != 0.0) {
              farkas_cut.terms.emplace_back(col, -rhs_coefficient);
            }
          }
          farkas_cut.rhs = base_rhs - box_minimum;
          sparsify_cut(farkas_cut, model.master.linear_part,
                       options.coefficient_tolerance);
          cut = std::move(farkas_cut);
        }
        if (options.verbose && cut) {
          std::fprintf(stderr,
                       "[Benders] feasibility cut iteration=%d "
                       "violation=%.12g box_min=%.12g rhs=%.12g terms=%zu\n",
                       iteration + 1, violation, box_minimum, cut->rhs,
                       cut->terms.size());
        }
      }
      if (!cut && options.infeasible_fallback)
        cut = options.infeasible_fallback(latest_master);
      if (!cut && options.use_binary_no_good_fallback)
        cut = make_binary_no_good_cut(model.master, model.theta_col,
                                      latest_master);
      if (!cut) {
        out.status = "Benders infeasible LP has no valid certificate cut";
        break;
      }
      append_cut(model.master, *cut);
      ++out.stats.cuts_added;
      // The native Phase-I solve may leave artificial columns active in the
      // reusable standard form. Rebuild at the next master assignment so a
      // stale artificial solution cannot be accepted as feasible recourse.
      rebuild_subproblem = true;
    }
  }

  out.relative_gap = relative_gap(out.upper_bound, out.lower_bound);
  if (!out.success && out.has_incumbent &&
      out.relative_gap <= options.gap_tolerance) {
    out.success = true;
    out.status = "Optimal";
  }
  if (out.status.empty())
    out.status = out.success ? "Optimal" : "Benders iteration limit";
  if (out.has_incumbent) {
    out.master_x = std::move(best_master);
    out.subproblem_x = std::move(best_subproblem);
  }
  return out;
}

}  // namespace mipsolvers::engine
