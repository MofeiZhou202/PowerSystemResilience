#pragma once
/// \file two_stage.hpp
/// \brief Two-stage decomposition: L-shaped/Benders for stochastic programs and
///        Column-and-Constraint Generation (CCG) for robust optimization.
///
/// Standard form (see docs/archive/two_stage_decomposition_design.md):
///   first stage   min c'x  s.t. A x <= b, Aeq x = beq, x in X (bounds+integrality)
///   recourse s    Q_s(x) = min d_s'y  s.t. W_s y >= h_s - T_s x, y in Y_s
///   stochastic    min c'x + sum_s p_s Q_s(x)
///   robust        min c'x + max_s Q_s(x)
///
/// The module depends only on mipsolvers::engine::SolverEngine (solve_lp /
/// solve_milp) and carries no domain (SCUC/power-system) coupling.

#include <limits>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/SparseCore>

#include "mipsolvers/engine/problem_types.hpp"

namespace mipsolvers::engine::decomposition {

/// First-stage (here-and-now) description.
///   min c'x  s.t.  A x <= b,  Aeq x = beq,  x in vars (bounds + integrality)
struct FirstStage {
  Eigen::VectorXd c;
  Eigen::SparseMatrix<double> A;     ///< inequality rows, A x <= b
  Eigen::VectorXd b;
  Eigen::SparseMatrix<double> Aeq;   ///< equality rows, Aeq x = beq
  Eigen::VectorXd beq;
  std::vector<VariableMeta> vars;    ///< size n1; encodes bounds and integrality
};

/// One second-stage recourse block (a scenario / uncertainty realization).
///   Q(x) = min d'y  s.t.  W y >= h - T x,  y in vars
/// The `>=` orientation matches the DecompositionAlgorithms MATLAB reference.
struct Recourse {
  Eigen::VectorXd d;                 ///< recourse cost, size n_s
  Eigen::SparseMatrix<double> T;     ///< technology matrix, m_s x n1
  Eigen::SparseMatrix<double> W;     ///< recourse matrix,   m_s x n_s
  Eigen::VectorXd h;                 ///< recourse rhs,      m_s
  std::vector<VariableMeta> vars;    ///< size n_s; bounds and (robust) integrality
  double probability{1.0};           ///< p_s for stochastic expectation; ignored by robust
};

/// Complete two-stage instance shared by all three solvers.
struct TwoStageModel {
  FirstStage first;
  std::vector<Recourse> scenarios;
};

/// Aggregation of scenario epigraphs in the Benders master.
enum class BendersCutMode {
  MultiCut,       ///< one theta_s per scenario (tighter master, S rows/iter)
  SingleCut,      ///< single aggregated theta = sum_s p_s theta_s (smaller master)
  IntegerLShaped, ///< Laporte-Louveaux integer optimality cuts; pure-binary
                  ///< first stage, exact for integer recourse (design doc §3.3)
  Lagrangian,     ///< Lagrangian/SDDiP cuts; pure-binary first stage, tight for
                  ///< integer recourse via an inner dual loop (design doc §3.6)
};

struct BendersOptions {
  int max_iterations{500};
  double gap_tolerance{1e-6};
  double time_limit_sec{0.0};        ///< 0 = unlimited
  BendersCutMode cut_mode{BendersCutMode::MultiCut};
  /// Worker threads for the per-scenario recourse solves. 1 = sequential
  /// (bit-identical to the serial path); >1 uses one SolverEngine per worker.
  int threads{1};
  /// In-out stabilization (Ben-Ameur & Neto 2007): separate cuts at
  /// x_sep = alpha*x_center + (1-alpha)*x_master. 0 disables it (pure Kelley).
  double stabilization_alpha{0.0};
  /// Fallback valid lower bound on each Q_s when the automatic under-estimator
  /// LP is proven unbounded below. Must be supplied by the caller and be a true
  /// under-estimate to stay exact; NaN means no certified fallback is available.
  double theta_lower_bound{std::numeric_limits<double>::quiet_NaN()};
  /// Lagrangian mode (design doc §3.6): inner cutting-plane iterations for the
  /// Lagrangian dual, and the box bound B on the multipliers lambda.
  int lagrangian_inner_iterations{20};
  double lagrangian_dual_bound{1e3};
  std::string master_solver;         ///< "" = engine default (e.g. NativeBranchAndCut)
  std::string subproblem_solver;     ///< "" = engine default LP solver
  bool verbose{false};
};

struct CCGOptions {
  int max_iterations{200};
  double gap_tolerance{1e-6};
  double time_limit_sec{0.0};        ///< 0 = unlimited
  /// Worker threads for the per-scenario oracle evaluations. 1 = sequential.
  int threads{1};
  std::string master_solver;         ///< "" = engine default
  std::string recourse_solver;       ///< "" = engine default; MILP if recourse has integrality
  bool verbose{false};
};

struct ExtensiveFormOptions {
  double time_limit_sec{0.0};
  std::string solver;                ///< "" = engine default
};

/// Unified result for every solver in this module.
struct DecompositionResult {
  bool success{false};               ///< proven optimal within gap_tolerance
  std::string status;
  Eigen::VectorXd x;                 ///< first-stage optimum
  std::vector<Eigen::VectorXd> y;    ///< recourse optimum per scenario (empty if not recovered)
  double objective{std::numeric_limits<double>::quiet_NaN()};
  double lower_bound{-std::numeric_limits<double>::infinity()};
  double upper_bound{std::numeric_limits<double>::infinity()};
  double relative_gap{std::numeric_limits<double>::infinity()};
  int iterations{0};                 ///< Benders/CCG major iterations
  int cuts_added{0};                 ///< Benders optimality + feasibility cuts
  int scenarios_generated{0};        ///< CCG active-set size at termination
};

// ─────────────────────────────────────────────────────────────────────────────
// Solvers
// ─────────────────────────────────────────────────────────────────────────────

/// Deterministic-equivalent (extensive form) of the stochastic program:
///   min c'x + sum_s p_s d_s'y_s  s.t. first-stage + W_s y_s >= h_s - T_s x.
/// Exact monolithic baseline / correctness oracle.
DecompositionResult solve_extensive_form_stochastic(
    const TwoStageModel& model, const ExtensiveFormOptions& options = {});

/// Epigraph (extensive form) of the robust program:
///   min c'x + eta  s.t. first-stage + eta >= d_s'y_s + W_s y_s >= h_s - T_s x.
/// Exact monolithic baseline / correctness oracle for CCG.
DecompositionResult solve_extensive_form_robust(
    const TwoStageModel& model, const ExtensiveFormOptions& options = {});

/// L-shaped / Benders decomposition for the two-stage stochastic program.
/// Requires continuous recourse (Assumption A1); integer recourse is rejected.
DecompositionResult solve_benders_stochastic(
    const TwoStageModel& model, const BendersOptions& options = {});

/// Column-and-Constraint Generation for two-stage robust optimization over a
/// finite uncertainty set. Supports continuous or integer recourse.
DecompositionResult solve_ccg_robust(
    const TwoStageModel& model, const CCGOptions& options = {});

// ─────────────────────────────────────────────────────────────────────────────
// Polyhedral (continuous) uncertainty — robust CCG with a KKT max–min oracle
// ─────────────────────────────────────────────────────────────────────────────

/// Bounded polyhedral uncertainty set  U = { u : u_lb <= u <= u_ub, G u <= g }.
struct UncertaintySet {
  Eigen::VectorXd u_lb;              ///< size nu
  Eigen::VectorXd u_ub;              ///< size nu
  Eigen::SparseMatrix<double> G;     ///< budget/linking rows (mu x nu), may be 0 rows
  Eigen::VectorXd g;                 ///< size mu
};

/// Continuous LP recourse with right-hand-side uncertainty:
///   Q(x,u) = min d'y  s.t.  W y >= h0 + P u - T x,  y >= 0
struct RobustRecourse {
  Eigen::VectorXd d;                 ///< recourse cost, size n
  Eigen::SparseMatrix<double> T;     ///< technology matrix, m x n1
  Eigen::SparseMatrix<double> W;     ///< recourse matrix,   m x n
  Eigen::VectorXd h0;                ///< nominal rhs, size m
  Eigen::SparseMatrix<double> P;     ///< uncertainty map,   m x nu   (h(u)=h0+P u)
  std::vector<VariableMeta> vars;    ///< size n; continuous, lb=0, no finite ub
};

struct PolyhedralRobustModel {
  FirstStage first;
  RobustRecourse recourse;
  UncertaintySet uncertainty;
};

struct PolyhedralCCGOptions {
  int max_iterations{200};
  double gap_tolerance{1e-6};
  double time_limit_sec{0.0};
  std::string master_solver;         ///< "" = engine default
  std::string oracle_solver;         ///< "" = engine default MILP solver
  /// Big-M for the KKT complementarity linearization in (ORACLE). Validated at
  /// runtime: a solve whose duals/primals reach 0.99*big_m is rejected.
  double big_m{1e6};
  bool verbose{false};
};

/// Column-and-Constraint Generation for two-stage robust optimization over a
/// continuous polyhedral uncertainty set (RHS uncertainty, continuous recourse).
/// The adversarial worst case is found by the KKT max–min oracle (design doc
/// §3.4); relatively complete recourse is assumed.
DecompositionResult solve_ccg_polyhedral_robust(
    const PolyhedralRobustModel& model, const PolyhedralCCGOptions& options = {});

}  // namespace mipsolvers::engine::decomposition
