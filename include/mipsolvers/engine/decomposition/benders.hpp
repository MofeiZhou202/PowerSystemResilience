#pragma once
/// \file benders.hpp
/// \brief Generic Branch-and-Benders-Cut decomposition for MILP + LP models.

#include <functional>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/SparseCore>

#include "mipsolvers/engine/bc/options.hpp"
#include "mipsolvers/engine/kernel/lp_kernel/dual_simplex.hpp"
#include "mipsolvers/engine/problem_types.hpp"

namespace mipsolvers::engine {

/// Sparse master inequality sum(value_j * x_j) <= rhs.
struct BendersCut {
  std::vector<std::pair<int, double>> terms;
  double rhs{0.0};
};

/// Parameterized two-stage model:
///   min f'x + theta
///   s.t. x in master,
///        min c'y: A y <= b - coupling_ineq*x,
///                  E y  = d - coupling_eq*x.
///
/// Coupling matrices use master-column indices directly and therefore have
/// exactly master.linear_part.c.size() columns.  They may couple binary,
/// integer, or continuous master variables; the master solver owns their
/// domains.  The theta column must not occur in either coupling matrix.
struct BendersModel {
  MIPModel master;
  int theta_col{-1};
  LPModel subproblem;
  Eigen::SparseMatrix<double> coupling_ineq;
  Eigen::SparseMatrix<double> coupling_eq;
};

/// Result of automatically partitioning a monolithic MILP.  The caller only
/// selects first-stage columns; pure first-stage rows are moved to the compact
/// master and every row containing a recourse column is parameterized in the
/// continuous subproblem.  theta is not included in master_to_original.
struct BendersPartition {
  bool success{false};
  std::string status;
  BendersModel model;
  std::vector<int> master_to_original;
  std::vector<int> subproblem_to_original;
  int original_variables{0};
};

/// Split a monolithic minimization MILP into a compact first-stage master and
/// a continuous recourse LP.  master_columns may contain binary, integer, or
/// continuous variables, but every remaining column must be continuous.
/// Column order follows master_columns; recourse columns retain source order.
/// The input is accepted by value so large callers can transfer ownership.
BendersPartition partition_benders_model(
    MIPModel source, const std::vector<int>& master_columns,
    double coefficient_tolerance = 0.0);

/// Called only when an infeasible subproblem has no usable Farkas ray.
/// A returned cut must be globally valid.  For an all-binary master, a
/// no-good cut excluding the current assignment is valid; returning nullopt
/// terminates without claiming convergence.
using BendersInfeasibleFallback =
    std::function<std::optional<BendersCut>(const Eigen::VectorXd& master_x)>;

struct BendersOptions {
  int max_iterations{200};
  double time_limit_sec{300.0};
  double gap_tolerance{1e-4};
  double cut_tolerance{1e-6};
  double coefficient_tolerance{0.0};
  int scaling_rounds{10};
  bool verbose{false};

  BCOptions master_options;
  SimplexOptions subproblem_options;
  int cold_master_cut_rounds{4};
  int warm_master_cut_rounds{2};
  bool reuse_master_incumbent{false};
  bool disable_redundant_warm_heuristics{true};
  /// If every non-theta master column is binary, an infeasible assignment can
  /// be excluded safely when the LP backend cannot provide a Farkas ray.
  bool use_binary_no_good_fallback{true};
  BendersInfeasibleFallback infeasible_fallback;
};

struct BendersStats {
  int iterations{0};
  int cuts_added{0};
  int coupled_master_variables{0};
  long long coupling_nonzeros{0};
  long long incremental_rhs_updates{0};

  long long master_nodes{0};
  long long master_lp_solves{0};
  int master_incumbent_warm_starts{0};
  int master_pseudocost_reuses{0};
  double master_solve_time_sec{0.0};

  int subproblem_solves{0};
  int subproblem_basis_warm_starts{0};
  long long subproblem_simplex_iterations{0};
  double subproblem_solve_time_sec{0.0};
};

struct BendersResult {
  bool success{false};       ///< Proven within gap_tolerance.
  bool has_incumbent{false}; ///< A feasible master/subproblem pair exists.
  std::string status;
  Eigen::VectorXd master_x;
  Eigen::VectorXd subproblem_x;
  double objective{0.0};
  double lower_bound{-std::numeric_limits<double>::infinity()};
  double upper_bound{std::numeric_limits<double>::infinity()};
  double relative_gap{std::numeric_limits<double>::infinity()};
  BendersStats stats;
};

/// Solve a generic parameterized MILP + continuous LP decomposition.
/// The model is accepted by value so large callers can move ownership and
/// avoid retaining duplicate sparse matrices during the solve.
BendersResult solve_benders(BendersModel model,
                            const BendersOptions& options = {});

}  // namespace mipsolvers::engine
