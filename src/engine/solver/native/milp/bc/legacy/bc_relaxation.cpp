/// @file bc_relaxation.cpp
/// @brief LP/NLP relaxation solving and rounding heuristics for the B&C solver.

#include "mipsolvers/engine/detail/bc_utils.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Sparse>

#include "mipsolvers/engine/kernel/ipm/ipm_lp_solver.hpp"
#include "mipsolvers/engine/kernel/ipm/ipm_solver.hpp"

#ifdef MIPSOLVERS_HAVE_HIGHS_LIB
#include "Highs.h"
#include "util/HVector.h"
#endif

namespace mipsolvers::engine::detail {

namespace {

constexpr double kHighsDefaultKktTolerance = 1e-7;
constexpr double kHighsDefaultMipTolerance = 1e-6;

bool lp_basis_trace_enabled() {
  return std::getenv("MIPSOLVERS_LP_BASIS_TRACE") != nullptr;
}

bool root_simplex_conformance_enabled() {
  return std::getenv("MIPSOLVERS_ROOT_COLDSTATE_DIAG") != nullptr ||
         std::getenv("MIPSOLVERS_LPSTATE_CONFORM") != nullptr ||
         std::getenv("MIPSOLVERS_FRONTIER_CONFORM") != nullptr ||
         std::getenv("MIPSOLVERS_XPOOL_EXACT_DSE") != nullptr ||
         std::getenv("MIPSOLVERS_XPOOL_PARENT_EXACT_DSE") != nullptr;
}

bool vendored_highs_lp_kernel_enabled(const BCOptions& opt) {
  return uses_highs_lp_kernel(opt.lp_kernel_backend);
}

void apply_root_simplex_conformance_options(SimplexOptions& opt,
                                            bool force = false) {
  if (!force && !root_simplex_conformance_enabled()) return;
  opt.exact_dse_initialization = true;
  opt.enable_degenerate_frontier_remap = true;
  opt.use_partial_pricing = false;
  opt.perturb_degenerate_primal = false;
}

int lp_basis_trace_terms() {
  const char* env = std::getenv("MIPSOLVERS_LP_BASIS_TRACE_TERMS");
  if (env == nullptr) return 8;
  char* end = nullptr;
  long val = std::strtol(env, &end, 10);
  if (end == env) return 8;
  return static_cast<int>(std::clamp<long>(val, 0, 64));
}

struct BasisComposition {
  int orig{0};
  int slack{0};
  int surplus{0};
  int artificial{0};
  int invalid{0};
  int duplicate{0};
  int upper{0};
};

const char* basis_col_kind(const StandardFormLP& sf, int col) {
  if (col < 0 || col >= static_cast<int>(sf.A.cols())) return "invalid";
  if (col < sf.n_original) return "orig";
  if (col < sf.n_original + sf.n_slack) return "slack";
  if (col < sf.n_original + sf.n_slack + sf.n_surplus) return "surplus";
  if (col < sf.n_original + sf.n_slack + sf.n_surplus + sf.n_artificial) return "art";
  return "unknown";
}

BasisComposition basis_composition(const StandardFormLP& sf,
                                   const std::vector<int>& basis,
                                   const std::vector<char>& at_upper) {
  const int n = static_cast<int>(sf.A.cols());
  BasisComposition comp;
  std::vector<char> seen(static_cast<std::size_t>(std::max(0, n)), 0);
  for (int idx : basis) {
    if (idx < 0 || idx >= n) {
      ++comp.invalid;
      continue;
    }
    if (seen[static_cast<std::size_t>(idx)]) ++comp.duplicate;
    seen[static_cast<std::size_t>(idx)] = 1;
    const char* kind = basis_col_kind(sf, idx);
    if (std::strcmp(kind, "orig") == 0) ++comp.orig;
    else if (std::strcmp(kind, "slack") == 0) ++comp.slack;
    else if (std::strcmp(kind, "surplus") == 0) ++comp.surplus;
    else if (std::strcmp(kind, "art") == 0) ++comp.artificial;
  }
  for (int j = 0; j < n && j < static_cast<int>(at_upper.size()); ++j) {
    if (j < static_cast<int>(seen.size()) && seen[static_cast<std::size_t>(j)]) continue;
    if (at_upper[static_cast<std::size_t>(j)]) ++comp.upper;
  }
  return comp;
}

void trace_basis_source(const char* stage,
                        const StandardFormLP& sf,
                        const SimplexBasis& basis,
                        const char* detail = "") {
  if (!lp_basis_trace_enabled()) return;
  const auto indices = basis.basis_indices();
  const BasisComposition comp = basis_composition(sf, indices, basis.at_upper);
  fprintf(stderr,
          "[B&C-LPBASIS-SOURCE] stage=%s m=%d n=%d nOrig=%d nSlack=%d "
          "nSurplus=%d nArt=%d basic=orig%d:slack%d:surplus%d:art%d "
          "invalid=%d dup=%d nonbasicUpper=%d %s sample=[",
          stage == nullptr ? "unknown" : stage,
          static_cast<int>(sf.A.rows()), static_cast<int>(sf.A.cols()),
          sf.n_original, sf.n_slack, sf.n_surplus, sf.n_artificial,
          comp.orig, comp.slack, comp.surplus, comp.artificial, comp.invalid,
          comp.duplicate, comp.upper, detail == nullptr ? "" : detail);
  const int max_terms = lp_basis_trace_terms();
  for (int i = 0; i < static_cast<int>(indices.size()) && i < max_terms; ++i) {
    if (i > 0) std::fprintf(stderr, ";");
    const int col = indices[static_cast<std::size_t>(i)];
    std::fprintf(stderr, "%d:%d:%s", i, col, basis_col_kind(sf, col));
  }
  std::fprintf(stderr, "]\n");
}

// Validate LP feasibility against variable bounds and linear constraints
// (without integer-integrality checks, since this is LP relaxation).
bool lp_solution_feasible(const LPModel& lp, const SolveResult& res, double tol) {
  const int n = static_cast<int>(lp.vars.size());
  if (!res.stats.success || res.x.size() != n) {
    return false;
  }
  for (int i = 0; i < n; ++i) {
    if (res.x[i] < lp.vars[i].lb - tol || res.x[i] > lp.vars[i].ub + tol) {
      return false;
    }
  }
  // Use efficient matrix-vector multiplication instead of row-by-row traversal
  // (ColMajor sparse matrix row(i).dot(x) is O(NNZ) per row, not O(row_nnz))
  if (lp.A.rows() > 0) {
    Eigen::VectorXd Ax = lp.A * res.x;
    for (int i = 0; i < static_cast<int>(lp.A.rows()); ++i) {
      if (Ax[i] > lp.b[i] + tol) {
        return false;
      }
    }
  }
  if (lp.Aeq.rows() > 0) {
    Eigen::VectorXd Aeq_x = lp.Aeq * res.x;
    for (int i = 0; i < static_cast<int>(lp.Aeq.rows()); ++i) {
      if (std::abs(Aeq_x[i] - lp.beq[i]) > tol) {
        return false;
      }
    }
  }
  return true;
}

bool ipm_handoff_audit_enabled(const BCOptions& opt) {
  return opt.verbose || std::getenv("MIPSOLVERS_IPM_HANDOFF_AUDIT") != nullptr;
}

struct PrimalAuditMetrics {
  double bound_violation{0.0};
  double ineq_upper_violation{0.0};
  double ineq_lower_violation{0.0};
  double eq_violation{0.0};
  double objective_recomputed{0.0};
  double objective_gap{0.0};
};

PrimalAuditMetrics audit_original_primal(const LPModel& lp,
                                         const SolveResult& res) {
  PrimalAuditMetrics m;
  const int n = static_cast<int>(lp.vars.size());
  if (res.x.size() != n) {
    m.bound_violation = std::numeric_limits<double>::infinity();
    m.ineq_upper_violation = std::numeric_limits<double>::infinity();
    m.ineq_lower_violation = std::numeric_limits<double>::infinity();
    m.eq_violation = std::numeric_limits<double>::infinity();
    m.objective_recomputed = std::numeric_limits<double>::quiet_NaN();
    m.objective_gap = std::numeric_limits<double>::infinity();
    return m;
  }
  for (int j = 0; j < n; ++j) {
    m.bound_violation = std::max(m.bound_violation,
                                 lp.vars[j].lb - res.x[j]);
    m.bound_violation = std::max(m.bound_violation,
                                 res.x[j] - lp.vars[j].ub);
  }
  if (lp.A.rows() > 0) {
    Eigen::VectorXd ax = lp.A * res.x;
    for (int i = 0; i < static_cast<int>(lp.A.rows()); ++i) {
      m.ineq_upper_violation = std::max(m.ineq_upper_violation,
                                        ax[i] - lp.b[i]);
      const double lhs = lp_row_lhs_or_neg_inf(lp, i);
      if (std::isfinite(lhs)) {
        m.ineq_lower_violation = std::max(m.ineq_lower_violation,
                                          lhs - ax[i]);
      }
    }
  }
  if (lp.Aeq.rows() > 0) {
    Eigen::VectorXd aeqx = lp.Aeq * res.x;
    for (int i = 0; i < static_cast<int>(lp.Aeq.rows()); ++i) {
      m.eq_violation = std::max(m.eq_violation,
                                std::abs(aeqx[i] - lp.beq[i]));
    }
  }
  m.objective_recomputed = lp.c.dot(res.x);
  m.objective_gap = std::abs(m.objective_recomputed - res.stats.objective);
  return m;
}

double row_col_coefficient(const StandardFormLP& sf, int row, int col);

Eigen::VectorXd primal_to_scaled_standard_form(const StandardFormLP& sf,
                                               const Eigen::VectorXd& x_orig);

std::shared_ptr<SimplexBasis>
make_first_class_basis_hint_from_simplex(const SimplexResult& simplex) {
  auto basis = std::make_shared<SimplexBasis>(simplex.basis);
  basis->sf_n_slack = simplex.form.n_slack;
  basis->sf_n_surplus = simplex.form.n_surplus;
  basis->sf_n_artificial = simplex.form.n_artificial;
  if (simplex.reduced_costs.size() == simplex.form.A.cols()) {
    basis->cached_reduced_costs =
        std::make_shared<const Eigen::VectorXd>(simplex.reduced_costs);
  }
  if (simplex.form.col_scale.size() == simplex.form.A.cols()) {
    basis->cached_col_scale =
        std::make_shared<const Eigen::VectorXd>(simplex.form.col_scale);
  }
  if (simplex.basis.cached_sparse_basis) {
    basis->cached_sparse_basis = simplex.basis.cached_sparse_basis;
    basis->persist_eta_count = simplex.basis.persist_eta_count;
  }
  return basis;
}

#ifdef MIPSOLVERS_HAVE_HIGHS_LIB
const char* highs_model_status_label(HighsModelStatus status) {
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
    default:
      return "Other";
  }
}

class DirectHighsLpBasisOps : public BasisOps {
 public:
  DirectHighsLpBasisOps(std::shared_ptr<Highs> highs,
                        int rows,
                        int cols,
                        const Eigen::SparseMatrix<double>* A)
      : highs_(std::move(highs)),
        m_(rows),
        n_(cols),
        A_owned_(A != nullptr ? *A : Eigen::SparseMatrix<double>()) {}

  BasisOpsKind kind() const override { return BasisOpsKind::VendoredHighs; }

  Eigen::VectorXd ftran(const Eigen::VectorXd& rhs) const override {
    Eigen::VectorXd out = Eigen::VectorXd::Zero(m_);
    if (!highs_ || rhs.size() != m_) return out;
    std::vector<double> h_rhs(static_cast<std::size_t>(m_), 0.0);
    std::vector<double> h_out(static_cast<std::size_t>(m_), 0.0);
    for (int i = 0; i < m_; ++i) h_rhs[static_cast<std::size_t>(i)] = rhs[i];
    if (highs_->getBasisSolve(h_rhs.data(), h_out.data()) !=
        HighsStatus::kOk) {
      return Eigen::VectorXd::Constant(m_,
                                       std::numeric_limits<double>::quiet_NaN());
    }
    for (int i = 0; i < m_; ++i) out[i] = h_out[static_cast<std::size_t>(i)];
    return out;
  }

  Eigen::VectorXd btran(const Eigen::VectorXd& rhs) const override {
    Eigen::VectorXd out = Eigen::VectorXd::Zero(m_);
    if (!highs_ || rhs.size() != m_) return out;
    std::vector<double> h_rhs(static_cast<std::size_t>(m_), 0.0);
    std::vector<double> h_out(static_cast<std::size_t>(m_), 0.0);
    for (int i = 0; i < m_; ++i) h_rhs[static_cast<std::size_t>(i)] = rhs[i];
    if (highs_->getBasisTransposeSolve(h_rhs.data(), h_out.data()) !=
        HighsStatus::kOk) {
      return Eigen::VectorXd::Constant(m_,
                                       std::numeric_limits<double>::quiet_NaN());
    }
    for (int i = 0; i < m_; ++i) out[i] = h_out[static_cast<std::size_t>(i)];
    return out;
  }

  bool basis_inverse_row(int row, Eigen::VectorXd& out) const override {
    out = Eigen::VectorXd::Zero(m_);
    if (!highs_ || row < 0 || row >= m_) return false;
    std::vector<double> row_vec(static_cast<std::size_t>(m_), 0.0);
    HighsInt row_num_nz = 0;
    std::vector<HighsInt> row_indices(static_cast<std::size_t>(m_), 0);
    const HighsStatus st = highs_->getBasisInverseRow(
        static_cast<HighsInt>(row), row_vec.data(), &row_num_nz,
        row_indices.data());
    if (st != HighsStatus::kOk || row_num_nz < 0 || row_num_nz > m_) {
      return false;
    }
    for (HighsInt k = 0; k < row_num_nz; ++k) {
      const int r = static_cast<int>(row_indices[static_cast<std::size_t>(k)]);
      if (r < 0 || r >= m_) return false;
      out[r] = row_vec[static_cast<std::size_t>(r)];
    }
    return out.allFinite();
  }

  bool basis_inverse_row_sparse_entries(
      int row,
      std::vector<std::pair<int, double>>& out) const override {
    out.clear();
    if (!highs_ || row < 0 || row >= m_) return false;
    HVector row_ep;
    row_ep.setup(static_cast<HighsInt>(m_));
    if (highs_->getBasisInverseRowSparse(static_cast<HighsInt>(row),
                                         row_ep) != HighsStatus::kOk) {
      return false;
    }
    out.reserve(static_cast<std::size_t>(std::max<HighsInt>(0, row_ep.count)));
    for (HighsInt k = 0; k < row_ep.count; ++k) {
      const int r = static_cast<int>(row_ep.index[static_cast<std::size_t>(k)]);
      if (r < 0 || r >= m_) return false;
      const double value = row_ep.array[static_cast<std::size_t>(r)];
      if (!std::isfinite(value)) return false;
      out.emplace_back(r, value);
    }
    return true;
  }

  bool tableau_row(int row, Eigen::RowVectorXd& out) const override {
    if (!highs_ || row < 0 || row >= m_ || n_ <= 0) return false;
    Eigen::VectorXd y;
    if (!basis_inverse_row(row, y)) return false;
    if (A_owned_.rows() != m_ || A_owned_.cols() != n_) return false;
    out = y.transpose() * A_owned_;
    return out.allFinite();
  }

  SparseFactorTelemetry factor_telemetry() const override {
    SparseFactorTelemetry t;
    t.ft_valid = true;
    return t;
  }

  void rebind_A(const Eigen::SparseMatrix<double>& A) override { A_owned_ = A; }

  bool bound_to_A(const Eigen::SparseMatrix<double>& A) const override {
    return A.rows() == m_ && A.cols() == n_;
  }

  std::shared_ptr<Highs> highs_handle() const override { return highs_; }
  int row_count() const { return m_; }

 private:
  std::shared_ptr<Highs> highs_;
  int m_{0};
  int n_{0};
  Eigen::SparseMatrix<double> A_owned_;
};

bool highs_pass_lp_model(Highs& highs, const LPModel& lp) {
  const int ncols = static_cast<int>(lp.vars.size());
  const int m_ineq = static_cast<int>(lp.A.rows());
  const int m_eq = static_cast<int>(lp.Aeq.rows());
  const int nrows = m_ineq + m_eq;

  std::vector<double> col_cost(static_cast<std::size_t>(ncols), 0.0);
  std::vector<double> col_lower(static_cast<std::size_t>(ncols), 0.0);
  std::vector<double> col_upper(static_cast<std::size_t>(ncols), 0.0);
  for (int j = 0; j < ncols; ++j) {
    col_cost[static_cast<std::size_t>(j)] = lp.c[j];
    col_lower[static_cast<std::size_t>(j)] = lp.vars[static_cast<std::size_t>(j)].lb;
    col_upper[static_cast<std::size_t>(j)] = lp.vars[static_cast<std::size_t>(j)].ub;
  }

  std::vector<double> row_lower(static_cast<std::size_t>(nrows),
                                -std::numeric_limits<double>::infinity());
  std::vector<double> row_upper(static_cast<std::size_t>(nrows),
                                std::numeric_limits<double>::infinity());
  for (int i = 0; i < m_ineq; ++i) {
    row_lower[static_cast<std::size_t>(i)] = lp_row_lhs_or_neg_inf(lp, i);
    row_upper[static_cast<std::size_t>(i)] = lp.b[i];
  }
  for (int i = 0; i < m_eq; ++i) {
    row_lower[static_cast<std::size_t>(m_ineq + i)] = lp.beq[i];
    row_upper[static_cast<std::size_t>(m_ineq + i)] = lp.beq[i];
  }

  std::vector<HighsInt> start(static_cast<std::size_t>(ncols + 1), 0);
  std::vector<HighsInt> index;
  std::vector<double> value;
  index.reserve(static_cast<std::size_t>(lp.A.nonZeros() + lp.Aeq.nonZeros()));
  value.reserve(index.capacity());

  // Avoid an O(nnz) copy when the matrices are already in compressed ColMajor
  // format (the default for Eigen::SparseMatrix<double>).  Only copy when
  // compression is required, which is rare after normal matrix assembly.
  const Eigen::SparseMatrix<double>* A_ptr = &lp.A;
  const Eigen::SparseMatrix<double>* Aeq_ptr = &lp.Aeq;
  Eigen::SparseMatrix<double> A_compressed, Aeq_compressed;
  if (!lp.A.isCompressed()) {
    A_compressed = lp.A;
    A_compressed.makeCompressed();
    A_ptr = &A_compressed;
  }
  if (!lp.Aeq.isCompressed()) {
    Aeq_compressed = lp.Aeq;
    Aeq_compressed.makeCompressed();
    Aeq_ptr = &Aeq_compressed;
  }
  const auto& A_col = *A_ptr;
  const auto& Aeq_col = *Aeq_ptr;
  for (int j = 0; j < ncols; ++j) {
    start[static_cast<std::size_t>(j)] =
        static_cast<HighsInt>(index.size());
    for (Eigen::SparseMatrix<double>::InnerIterator it(A_col, j);
         it; ++it) {
      if (it.value() == 0.0) continue;
      index.push_back(static_cast<HighsInt>(it.row()));
      value.push_back(it.value());
    }
    for (Eigen::SparseMatrix<double>::InnerIterator it(Aeq_col, j);
         it; ++it) {
      if (it.value() == 0.0) continue;
      index.push_back(static_cast<HighsInt>(m_ineq + it.row()));
      value.push_back(it.value());
    }
  }
  start[static_cast<std::size_t>(ncols)] =
      static_cast<HighsInt>(index.size());

  const ObjSense sense = lp.sense == Sense::Maximize
                             ? ObjSense::kMaximize
                             : ObjSense::kMinimize;
  const HighsStatus pass_status = highs.passModel(
      static_cast<HighsInt>(ncols), static_cast<HighsInt>(nrows),
      static_cast<HighsInt>(index.size()),
      static_cast<HighsInt>(MatrixFormat::kColwise),
      static_cast<HighsInt>(sense), 0.0, col_cost.data(),
      col_lower.data(), col_upper.data(), row_lower.data(), row_upper.data(),
      start.data(), index.data(), value.data(), nullptr);
  return pass_status == HighsStatus::kOk;
}

bool build_native_simplex_state_from_highs(const LPModel& lp,
                                           const std::shared_ptr<Highs>& highs,
                                           const SolveResult& primal,
                                           SimplexResult& simplex) {
  if (!highs) return false;
  const int n_orig = static_cast<int>(lp.vars.size());
  const int m_ineq = static_cast<int>(lp.A.rows());
  const int m_eq = static_cast<int>(lp.Aeq.rows());
  const int m = m_ineq + m_eq;
  const int n_sf = n_orig + m;

  simplex.form = StandardFormLP{};
  simplex.form.n_original = n_orig;
  simplex.form.original_types.reserve(static_cast<std::size_t>(n_orig));
  simplex.form.lb_shift = Eigen::VectorXd::Zero(n_orig);
  for (int j = 0; j < n_orig; ++j) {
    simplex.form.original_types.push_back(lp.vars[static_cast<std::size_t>(j)].type);
    simplex.form.lb_shift[j] = lp.vars[static_cast<std::size_t>(j)].lb;
  }
  simplex.form.n_slack = m;
  simplex.form.n_surplus = 0;
  simplex.form.n_artificial = 0;
  simplex.form.row_to_slack_col.assign(static_cast<std::size_t>(m), -1);
  simplex.form.row_to_surplus_col.assign(static_cast<std::size_t>(m), -1);
  simplex.form.row_to_artificial_col.assign(static_cast<std::size_t>(m), -1);
  simplex.form.row_sign.assign(static_cast<std::size_t>(m), 1);
  simplex.form.row_rhs_value = Eigen::VectorXd::Zero(m);
  simplex.form.ub_row_var.assign(static_cast<std::size_t>(m), -1);
  simplex.form.source_highs_row.assign(static_cast<std::size_t>(m), -1);
  if (static_cast<int>(lp.highs_row_to_native_row.size()) >= m &&
      static_cast<int>(lp.highs_row_start.size()) >=
          static_cast<int>(lp.highs_row_to_native_row.size()) + 1) {
    for (int highs_row = 0;
         highs_row < static_cast<int>(lp.highs_row_to_native_row.size());
         ++highs_row) {
      const int native_row =
          lp.highs_row_to_native_row[static_cast<std::size_t>(highs_row)];
      if (native_row >= 0 && native_row < m) {
        simplex.form.source_highs_row[static_cast<std::size_t>(native_row)] =
            highs_row;
      }
    }
    simplex.form.source_row_start = lp.highs_row_start;
    simplex.form.source_row_index = lp.highs_row_index;
    simplex.form.source_row_value = lp.highs_row_value;
  }
  for (int row = 0; row < m; ++row) {
    simplex.form.row_to_slack_col[static_cast<std::size_t>(row)] = n_orig + row;
  }
  for (int row = 0; row < m_ineq; ++row) {
    simplex.form.row_rhs_value[row] = lp.b[row];
  }
  for (int row = 0; row < m_eq; ++row) {
    simplex.form.row_rhs_value[m_ineq + row] = lp.beq[row];
  }

  std::vector<Eigen::Triplet<double>> triplets;
  triplets.reserve(static_cast<std::size_t>(lp.A.nonZeros() +
                                            lp.Aeq.nonZeros() + m));
  for (int col = 0; col < lp.A.outerSize(); ++col) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(lp.A, col); it; ++it) {
      if (std::abs(it.value()) > 1e-15) {
        triplets.emplace_back(static_cast<int>(it.row()), col, it.value());
      }
    }
  }
  for (int col = 0; col < lp.Aeq.outerSize(); ++col) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(lp.Aeq, col); it; ++it) {
      if (std::abs(it.value()) > 1e-15) {
        triplets.emplace_back(m_ineq + static_cast<int>(it.row()), col,
                              it.value());
      }
    }
  }
  for (int row = 0; row < m; ++row) {
    triplets.emplace_back(row, n_orig + row, -1.0);
  }
  simplex.form.A.resize(m, n_sf);
  simplex.form.A.setFromTriplets(triplets.begin(), triplets.end());
  simplex.form.A.makeCompressed();
  simplex.form.A_row = simplex.form.A;
  simplex.form.b = Eigen::VectorXd::Zero(m);
  simplex.form.c_max = Eigen::VectorXd::Zero(n_sf);
  const Eigen::VectorXd c_min = lp.sense == Sense::Minimize ? lp.c : (-lp.c);
  simplex.form.c_max.head(n_orig) = -c_min;
  simplex.form.objective_const = 0.0;
  simplex.form.var_ub =
      Eigen::VectorXd::Constant(n_sf, std::numeric_limits<double>::infinity());
  for (int j = 0; j < n_orig; ++j) {
    if (std::isfinite(lp.vars[static_cast<std::size_t>(j)].ub) &&
        std::isfinite(lp.vars[static_cast<std::size_t>(j)].lb)) {
      simplex.form.var_ub[j] =
          lp.vars[static_cast<std::size_t>(j)].ub -
          lp.vars[static_cast<std::size_t>(j)].lb;
    }
  }
  for (int row = 0; row < m_ineq; ++row) {
    const double lhs = lp_row_lhs_or_neg_inf(lp, row);
    const double rhs = lp.b[row];
    simplex.form.b[row] = 0.0;
    if (std::isfinite(lhs) && std::isfinite(rhs)) {
      simplex.form.var_ub[n_orig + row] = std::max(0.0, rhs - lhs);
    }
  }
  for (int row = 0; row < m_eq; ++row) {
    simplex.form.var_ub[n_orig + m_ineq + row] = 0.0;
  }

  simplex.result = primal;
  simplex.result.stats.solver_name = "VendoredHighsLpKernel";
  simplex.x_std = Eigen::VectorXd::Zero(n_sf);
  if (primal.x.size() != n_orig) return false;
  for (int j = 0; j < n_orig; ++j) {
    const double lb = lp.vars[static_cast<std::size_t>(j)].lb;
    simplex.x_std[j] = primal.x[j] - lb;
  }
  const HighsSolution& hsol = highs->getSolution();
  if (static_cast<int>(hsol.row_value.size()) < m) return false;
  for (int row = 0; row < m; ++row) {
    double lower = -std::numeric_limits<double>::infinity();
    double upper = std::numeric_limits<double>::infinity();
    if (row < m_ineq) {
      lower = lp_row_lhs_or_neg_inf(lp, row);
      upper = lp.b[row];
    } else {
      lower = lp.beq[row - m_ineq];
      upper = lower;
    }
    const double row_value = hsol.row_value[static_cast<std::size_t>(row)];
    if (std::isfinite(lower)) {
      simplex.x_std[n_orig + row] = row_value - lower;
    } else if (std::isfinite(upper)) {
      simplex.x_std[n_orig + row] = row_value - upper;
    } else {
      simplex.x_std[n_orig + row] = row_value;
    }
  }
  if (simplex.x_std.size() != n_sf) return false;

  simplex.x_basic = Eigen::VectorXd::Zero(m);
  simplex.basis.rows = m;
  simplex.basis.cols = n_sf;
  simplex.basis.indices.assign(static_cast<std::size_t>(m), -1);
  simplex.basis.at_upper.assign(static_cast<std::size_t>(n_sf), 0);
  simplex.basis.sf_n_slack = simplex.form.n_slack;
  simplex.basis.sf_n_surplus = simplex.form.n_surplus;
  simplex.basis.sf_n_artificial = simplex.form.n_artificial;

  const HighsBasis& hbasis = highs->getBasis();
  if (!hbasis.valid ||
      static_cast<int>(hbasis.col_status.size()) < n_orig ||
      static_cast<int>(hbasis.row_status.size()) < m) {
    return false;
  }

  for (int j = 0; j < n_orig; ++j) {
    const auto st = hbasis.col_status[static_cast<std::size_t>(j)];
    simplex.basis.at_upper[static_cast<std::size_t>(j)] =
        st == HighsBasisStatus::kUpper ? 1 : 0;
  }
  for (int row = 0; row < m; ++row) {
    const auto st = hbasis.row_status[static_cast<std::size_t>(row)];
    const int logical_col = n_orig + row;
    if (st == HighsBasisStatus::kUpper && logical_col < n_sf) {
      simplex.basis.at_upper[static_cast<std::size_t>(logical_col)] = 1;
    }
  }

  std::vector<HighsInt> basic(static_cast<std::size_t>(std::max(0, m)), 0);
  if (m > 0 &&
      highs->getBasicVariables(basic.data()) != HighsStatus::kOk) {
    return false;
  }
  for (int row = 0; row < m; ++row) {
    const HighsInt hbasic = basic[static_cast<std::size_t>(row)];
    const int col =
        hbasic >= 0 ? static_cast<int>(hbasic)
                    : n_orig + static_cast<int>(-hbasic - 1);
    if (col < 0 || col >= n_sf) return false;
    simplex.basis.indices[static_cast<std::size_t>(row)] = col;
    simplex.x_basic[row] = simplex.x_std[col];
  }

  simplex.basis_inverse.resize(0, 0);

  simplex.reduced_costs = Eigen::VectorXd::Zero(n_sf);
  if (primal.box_dual_lb.size() == n_orig &&
      primal.box_dual_ub.size() == n_orig) {
    std::vector<char> is_basic(static_cast<std::size_t>(n_sf), 0);
    for (int col : simplex.basis.indices) {
      if (col >= 0 && col < n_sf) is_basic[static_cast<std::size_t>(col)] = 1;
    }
    for (int j = 0; j < n_orig; ++j) {
      if (is_basic[static_cast<std::size_t>(j)]) continue;
      const double lb_dual = primal.box_dual_lb[j];
      const double ub_dual = primal.box_dual_ub[j];
      simplex.reduced_costs[j] = -(lb_dual - ub_dual);
    }
  }

  simplex.max_objective = -simplex.result.stats.objective;
  simplex.exact_optimal = simplex.result.stats.success &&
                          highs->getModelStatus() == HighsModelStatus::kOptimal;
  simplex.basis.cached_sparse_basis =
      std::make_shared<DirectHighsLpBasisOps>(highs, m, n_sf,
                                              &simplex.form.A);
  simplex.basis.persist_eta_count = 0;
  return true;
}

bool finish_vendored_highs_relaxation(const LPModel& lp,
                                      const std::shared_ptr<Highs>& highs,
                                      HighsStatus run_status,
                                      HighsModelStatus model_status,
                                      double runtime_sec,
                                      const char* label,
                                      const BCOptions& opt,
                                      LPRelaxationResult& out) {
  if (!highs) return false;
  const int m = static_cast<int>(lp.A.rows() + lp.Aeq.rows());
  const int n = static_cast<int>(lp.vars.size());
  const HighsInfo& info = highs->getInfo();

  SolveResult primal;
  primal.stats.solver_name =
      (label != nullptr && std::strstr(label, "live append") != nullptr)
          ? "VendoredHighsLiveAppendLpKernel"
          : "VendoredHighsDirectLpKernel";
  primal.stats.iterations = static_cast<int>(info.simplex_iteration_count);
  primal.stats.objective = info.objective_function_value;
  primal.stats.status = std::string("HiGHS ") +
                        highs_model_status_label(model_status);
  primal.stats.success = run_status == HighsStatus::kOk &&
                         model_status == HighsModelStatus::kOptimal;
  primal.stats.runtime_sec = runtime_sec;

  if (model_status == HighsModelStatus::kInfeasible) {
    primal.stats.status = "HiGHS Infeasible";
    primal.stats.has_farkas_certificate = true;
    primal.stats.success = false;
    out.primal = std::move(primal);
    out.dual_bound = std::numeric_limits<double>::infinity();
    return true;
  }
  if (!primal.stats.success) {
    out.primal = std::move(primal);
    out.dual_bound = std::numeric_limits<double>::infinity();
    return true;
  }

  const auto finish_t0 = std::chrono::steady_clock::now();
  const HighsSolution& sol = highs->getSolution();
  primal.x = Eigen::VectorXd::Zero(n);
  for (int j = 0; j < n && j < static_cast<int>(sol.col_value.size()); ++j) {
    primal.x[j] = sol.col_value[static_cast<std::size_t>(j)];
  }
  primal.constraint_duals = Eigen::VectorXd::Zero(m);
  for (int i = 0; i < m && i < static_cast<int>(sol.row_dual.size()); ++i) {
    primal.constraint_duals[i] = sol.row_dual[static_cast<std::size_t>(i)];
  }
  primal.box_dual_lb = Eigen::VectorXd::Zero(n);
  primal.box_dual_ub = Eigen::VectorXd::Zero(n);
  if (static_cast<int>(sol.col_dual.size()) >= n) {
    for (int j = 0; j < n; ++j) {
      const double dj = sol.col_dual[static_cast<std::size_t>(j)];
      if (dj >= 0.0) {
        primal.box_dual_lb[j] = dj;
      } else {
        primal.box_dual_ub[j] = -dj;
      }
    }
  }

  auto simplex = std::make_shared<SimplexResult>();
  const auto import_t0 = std::chrono::steady_clock::now();
  if (!build_native_simplex_state_from_highs(lp, highs, primal, *simplex)) {
    if (opt.verbose ||
        std::getenv("MIPSOLVERS_HIGHS_LP_KERNEL_TRACE") != nullptr) {
      std::fprintf(stderr,
                   "[BC_RELAX] VendoredHiGHS %s solved but basis import "
                   "failed\n",
                   label != nullptr ? label : "direct LP");
    }
    return false;
  }
  const double import_ms =
      std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - import_t0)
          .count();
  const double finish_ms =
      std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - finish_t0)
          .count();

  out.simplex = simplex;
  out.primal = std::move(primal);
  out.dual_bound = out.primal.stats.objective;
  out.row_duals = out.primal.constraint_duals;
  out.basis_hint = make_first_class_basis_hint_from_simplex(*simplex);
  if (opt.verbose ||
      std::getenv("MIPSOLVERS_HIGHS_LP_KERNEL_TRACE") != nullptr) {
    std::fprintf(stderr,
                 "[BC_RELAX] VendoredHiGHS %s: success=1 iter=%d "
                 "obj=%.12g rows=%d cols=%d basisRows=%d time=%.3fms "
                 "finishMs=%.3f importMs=%.3f\n",
                 label != nullptr ? label : "direct LP",
                 out.primal.stats.iterations, out.primal.stats.objective, m,
                 n, out.basis_hint ? out.basis_hint->rows : -1,
                 out.primal.stats.runtime_sec * 1000.0, finish_ms, import_ms);
  }
  return true;
}

bool native_basis_to_highs_basis(const LPModel& lp,
                                 const SimplexBasis* basis_hint,
                                 HighsBasis& hbasis) {
  if (basis_hint == nullptr) return false;
  const int n = static_cast<int>(lp.vars.size());
  const int m = static_cast<int>(lp.A.rows() + lp.Aeq.rows());
  if (basis_hint->rows <= 0 || basis_hint->rows > m ||
      static_cast<int>(basis_hint->index_count()) != basis_hint->rows ||
      static_cast<int>(basis_hint->at_upper.size()) < n) {
    return false;
  }
  const auto direct_lp_basis =
      std::dynamic_pointer_cast<DirectHighsLpBasisOps>(
          basis_hint->cached_sparse_basis);
  if (direct_lp_basis &&
      basis_hint->cols == n + basis_hint->rows &&
      basis_hint->sf_n_slack == basis_hint->rows &&
      basis_hint->sf_n_surplus == 0 &&
      basis_hint->sf_n_artificial == 0) {
    const int m_eq = static_cast<int>(lp.Aeq.rows());
    const int old_rows = basis_hint->rows;
    const int new_rows = m - old_rows;
    const int old_m_ineq = old_rows - m_eq;
    if (new_rows < 0 || old_m_ineq < 0) return false;
    auto map_old_row = [&](int old_row) -> int {
      if (old_row < 0 || old_row >= old_rows) return -1;
      return old_row < old_m_ineq ? old_row : old_row + new_rows;
    };

    std::vector<int> mapped_basis(static_cast<std::size_t>(m), -1);
    for (int row = 0; row < m; ++row) {
      mapped_basis[static_cast<std::size_t>(row)] = n + row;
    }
    const auto& hint = basis_hint->basis_indices();
    for (int old_basis_row = 0; old_basis_row < old_rows; ++old_basis_row) {
      const int new_basis_row = map_old_row(old_basis_row);
      if (new_basis_row < 0 || new_basis_row >= m) return false;
      int col = hint[static_cast<std::size_t>(old_basis_row)];
      if (col >= n) {
        const int old_logical_row = col - n;
        const int new_logical_row = map_old_row(old_logical_row);
        if (new_logical_row < 0 || new_logical_row >= m) return false;
        col = n + new_logical_row;
      }
      if (col < 0 || col >= n + m) return false;
      mapped_basis[static_cast<std::size_t>(new_basis_row)] = col;
    }

    hbasis.valid = false;
    hbasis.alien = true;
    hbasis.useful = true;
    hbasis.col_status.assign(static_cast<std::size_t>(n),
                             HighsBasisStatus::kLower);
    hbasis.row_status.assign(static_cast<std::size_t>(m),
                             HighsBasisStatus::kLower);
    for (int j = 0; j < n; ++j) {
      const bool fixed =
          std::isfinite(lp.vars[static_cast<std::size_t>(j)].lb) &&
          std::isfinite(lp.vars[static_cast<std::size_t>(j)].ub) &&
          std::abs(lp.vars[static_cast<std::size_t>(j)].ub -
                   lp.vars[static_cast<std::size_t>(j)].lb) <= 1e-12;
      if (!fixed && basis_hint->at_upper[static_cast<std::size_t>(j)] != 0) {
        hbasis.col_status[static_cast<std::size_t>(j)] =
            HighsBasisStatus::kUpper;
      }
    }
    const int old_at_upper_n =
        std::min(basis_hint->cols,
                 static_cast<int>(basis_hint->at_upper.size()));
    for (int old_col = n; old_col < old_at_upper_n; ++old_col) {
      const int new_row = map_old_row(old_col - n);
      if (new_row >= 0 && new_row < m) {
        hbasis.row_status[static_cast<std::size_t>(new_row)] =
            HighsBasisStatus::kUpper;
      }
    }

    std::vector<char> seen(static_cast<std::size_t>(n + m), 0);
    for (int row = 0; row < m; ++row) {
      const int col = mapped_basis[static_cast<std::size_t>(row)];
      if (col < 0 || col >= n + m) return false;
      if (seen[static_cast<std::size_t>(col)] != 0) return false;
      seen[static_cast<std::size_t>(col)] = 1;
      if (col < n) {
        hbasis.col_status[static_cast<std::size_t>(col)] =
            HighsBasisStatus::kBasic;
      } else {
        hbasis.row_status[static_cast<std::size_t>(col - n)] =
            HighsBasisStatus::kBasic;
      }
    }
    hbasis.valid = true;
    return true;
  }

  const StandardFormLP sf = build_standard_form_lp(lp);
  const int sf_n = static_cast<int>(sf.A.cols());
  if (sf.n_original != n || sf.A.rows() != m || sf_n < n) {
    return false;
  }

  std::vector<int> mapped_basis(static_cast<std::size_t>(m), -1);
  auto adjusted_col = [&](int col, int n_slack_old,
                          int n_surplus_old) -> int {
    if (col < 0) return -1;
    const int old_surplus_begin = n + n_slack_old;
    const int old_art_begin = old_surplus_begin + n_surplus_old;
    const int delta_slack = sf.n_slack - n_slack_old;
    const int delta_surplus = sf.n_surplus - n_surplus_old;
    if (col < old_surplus_begin) return col;
    if (col < old_art_begin) return col + delta_slack;
    return col + delta_slack + delta_surplus;
  };

  if (basis_hint->rows < m) {
    const int m_eq = static_cast<int>(lp.Aeq.rows());
    const int old_rows = basis_hint->rows;
    const int new_rows = m - old_rows;
    const int old_m_ineq = old_rows - m_eq;
    if (old_m_ineq < 0) return false;
    const int n_slack_old =
        basis_hint->sf_n_slack >= 0
            ? basis_hint->sf_n_slack
            : basis_hint->cols - n - sf.n_surplus - sf.n_artificial;
    const int n_surplus_old =
        basis_hint->sf_n_surplus >= 0 ? basis_hint->sf_n_surplus
                                      : sf.n_surplus;
    if (n_slack_old < 0 || n_surplus_old < 0 ||
        sf.n_slack - n_slack_old != new_rows) {
      return false;
    }
    const auto& hint = basis_hint->basis_indices();
    for (int row = 0; row < old_m_ineq; ++row) {
      mapped_basis[static_cast<std::size_t>(row)] =
          adjusted_col(hint[static_cast<std::size_t>(row)], n_slack_old,
                       n_surplus_old);
    }
    for (int k = 0; k < new_rows; ++k) {
      const int row = old_m_ineq + k;
      const int slack =
          row < static_cast<int>(sf.row_to_slack_col.size())
              ? sf.row_to_slack_col[static_cast<std::size_t>(row)]
              : -1;
      mapped_basis[static_cast<std::size_t>(row)] = slack;
    }
    for (int row = old_m_ineq; row < old_rows; ++row) {
      mapped_basis[static_cast<std::size_t>(row + new_rows)] =
          adjusted_col(hint[static_cast<std::size_t>(row)], n_slack_old,
                       n_surplus_old);
    }
  } else {
    const auto& hint = basis_hint->basis_indices();
    for (int row = 0; row < m; ++row) {
      mapped_basis[static_cast<std::size_t>(row)] =
          hint[static_cast<std::size_t>(row)];
    }
  }

  std::vector<int> sf_col_to_row(static_cast<std::size_t>(sf_n), -1);
  for (int row = 0; row < m; ++row) {
    const int slack =
        row < static_cast<int>(sf.row_to_slack_col.size())
            ? sf.row_to_slack_col[static_cast<std::size_t>(row)]
            : -1;
    const int surplus =
        row < static_cast<int>(sf.row_to_surplus_col.size())
            ? sf.row_to_surplus_col[static_cast<std::size_t>(row)]
            : -1;
    const int artificial =
        row < static_cast<int>(sf.row_to_artificial_col.size())
            ? sf.row_to_artificial_col[static_cast<std::size_t>(row)]
            : -1;
    if (slack >= 0 && slack < sf_n) sf_col_to_row[slack] = row;
    if (surplus >= 0 && surplus < sf_n) sf_col_to_row[surplus] = row;
    if (artificial >= 0 && artificial < sf_n) sf_col_to_row[artificial] = row;
  }

  hbasis.valid = false;
  hbasis.alien = true;
  hbasis.useful = true;
  hbasis.col_status.assign(static_cast<std::size_t>(n),
                           HighsBasisStatus::kLower);
  hbasis.row_status.assign(static_cast<std::size_t>(m),
                           HighsBasisStatus::kLower);
  for (int j = 0; j < n; ++j) {
    const bool fixed = std::isfinite(lp.vars[static_cast<std::size_t>(j)].lb) &&
                       std::isfinite(lp.vars[static_cast<std::size_t>(j)].ub) &&
                       std::abs(lp.vars[static_cast<std::size_t>(j)].ub -
                                lp.vars[static_cast<std::size_t>(j)].lb) <=
                           1e-12;
    if (fixed) {
      hbasis.col_status[static_cast<std::size_t>(j)] = HighsBasisStatus::kLower;
    } else if (basis_hint->at_upper[static_cast<std::size_t>(j)] != 0) {
      hbasis.col_status[static_cast<std::size_t>(j)] = HighsBasisStatus::kUpper;
    }
  }
  for (int row = 0; row < m; ++row) {
    const int col = mapped_basis[static_cast<std::size_t>(row)];
    if (col >= 0 && col < n) {
      hbasis.col_status[static_cast<std::size_t>(col)] =
          HighsBasisStatus::kBasic;
    } else if (col >= n && col < sf_n) {
      const int logical_row = sf_col_to_row[static_cast<std::size_t>(col)];
      if (logical_row < 0 || logical_row >= m) return false;
      hbasis.row_status[static_cast<std::size_t>(logical_row)] =
          HighsBasisStatus::kBasic;
    } else {
      return false;
    }
  }
  const int at_upper_n =
      std::min(sf_n, static_cast<int>(basis_hint->at_upper.size()));
  for (int col = n; col < at_upper_n; ++col) {
    if (basis_hint->at_upper[static_cast<std::size_t>(col)] == 0) continue;
    const int logical_row = sf_col_to_row[static_cast<std::size_t>(col)];
    if (logical_row >= 0 && logical_row < m &&
        hbasis.row_status[static_cast<std::size_t>(logical_row)] !=
            HighsBasisStatus::kBasic) {
      hbasis.row_status[static_cast<std::size_t>(logical_row)] =
          HighsBasisStatus::kUpper;
    }
  }
  hbasis.valid = true;
  return true;
}

bool solve_lp_relaxation_with_vendored_highs(const LPModel& lp,
                                             const SimplexBasis* basis_hint,
                                             const BCOptions& opt,
                                             LPRelaxationResult& out) {
  const auto t0 = std::chrono::steady_clock::now();
  std::optional<std::string> live_append_reject;
  const int live_append_candidate_rows =
      static_cast<int>(lp.A.rows() + lp.Aeq.rows());
  const bool has_appended_rows =
      basis_hint != nullptr && basis_hint->rows >= 0 &&
      basis_hint->rows < live_append_candidate_rows;
  const bool require_live_append =
      opt.auto_highs_root_pipeline && has_appended_rows;
  if (require_live_append && basis_hint != nullptr) {
    const int n = static_cast<int>(lp.vars.size());
    const int m_ineq = static_cast<int>(lp.A.rows());
    const int m_eq = static_cast<int>(lp.Aeq.rows());
    const int m = m_ineq + m_eq;
    const auto direct_lp_basis =
        std::dynamic_pointer_cast<DirectHighsLpBasisOps>(
            basis_hint->cached_sparse_basis);
    if (!direct_lp_basis || !direct_lp_basis->highs_handle()) {
      live_append_reject = "missing direct HiGHS basis";
    } else if (m_eq != 0) {
      live_append_reject = "equalities not supported by live append";
    } else if (basis_hint->rows != direct_lp_basis->row_count()) {
      live_append_reject = "basis row count changed";
    } else if (basis_hint->rows < 0 || basis_hint->rows >= m) {
      live_append_reject = "no appended rows";
    } else if (basis_hint->cols != n + basis_hint->rows) {
      live_append_reject = "basis column count changed";
    } else {
      const bool live_perf_trace =
          std::getenv("MIPSOLVERS_XPOOL_LEDGER") != nullptr ||
          std::getenv("MIPSOLVERS_HIGHS_LP_KERNEL_TRACE") != nullptr;
      const int old_rows = basis_hint->rows;
      const int new_rows = m - old_rows;
      const auto pack_t0 = std::chrono::steady_clock::now();
      std::vector<double> lower(static_cast<std::size_t>(new_rows));
      std::vector<double> upper(static_cast<std::size_t>(new_rows));
      std::vector<HighsInt> starts(static_cast<std::size_t>(new_rows + 1), 0);
      std::vector<HighsInt> indices;
      std::vector<double> values;
      std::vector<std::vector<std::pair<HighsInt, double>>> row_entries(
          static_cast<std::size_t>(new_rows));
      for (int col = 0; col < lp.A.outerSize(); ++col) {
        for (Eigen::SparseMatrix<double>::InnerIterator it(lp.A, col); it;
             ++it) {
          const int row = static_cast<int>(it.row());
          if (row < old_rows || row >= m_ineq || it.value() == 0.0) continue;
          row_entries[static_cast<std::size_t>(row - old_rows)].emplace_back(
              static_cast<HighsInt>(col), it.value());
        }
      }
      for (int row = old_rows; row < m_ineq; ++row) {
        const int local = row - old_rows;
        starts[static_cast<std::size_t>(local)] =
            static_cast<HighsInt>(indices.size());
        lower[static_cast<std::size_t>(local)] = lp_row_lhs_or_neg_inf(lp, row);
        upper[static_cast<std::size_t>(local)] = lp.b[row];
        for (const auto& entry : row_entries[static_cast<std::size_t>(local)]) {
          indices.push_back(entry.first);
          values.push_back(entry.second);
        }
      }
      starts[static_cast<std::size_t>(new_rows)] =
          static_cast<HighsInt>(indices.size());
      auto highs = direct_lp_basis->highs_handle();
      const double pack_ms =
          std::chrono::duration<double, std::milli>(
              std::chrono::steady_clock::now() - pack_t0)
              .count();
      const auto add_t0 = std::chrono::steady_clock::now();
      const HighsStatus add_status = highs->addRows(
          static_cast<HighsInt>(new_rows), lower.data(), upper.data(),
          static_cast<HighsInt>(indices.size()), starts.data(), indices.data(),
          values.data());
      const double add_ms =
          std::chrono::duration<double, std::milli>(
              std::chrono::steady_clock::now() - add_t0)
              .count();
      if (add_status == HighsStatus::kOk || add_status == HighsStatus::kWarning) {
        const auto run_t0 = std::chrono::steady_clock::now();
        const HighsStatus run_status = highs->run();
        const double run_ms =
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - run_t0)
                .count();
        const HighsModelStatus model_status = highs->getModelStatus();
        const double runtime_sec =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - t0)
                .count();
        const bool finished = finish_vendored_highs_relaxation(
                lp, highs, run_status, model_status, runtime_sec,
                "live append LP", opt, out);
        if (live_perf_trace) {
          std::fprintf(stderr,
                       "[BC_RELAX-LIVEAPPEND-PERF] rows=%d nnz=%zu packMs=%.6f "
                       "addRowsMs=%.6f runMs=%.6f totalMs=%.6f finished=%d\n",
                       new_rows, values.size(), pack_ms, add_ms, run_ms,
                       runtime_sec * 1000.0, finished ? 1 : 0);
        }
        if (finished) {
          return true;
        }
      }
      live_append_reject = "Highs::addRows/run failed";
    }
    if (require_live_append) {
      if (opt.verbose ||
          std::getenv("MIPSOLVERS_HIGHS_LP_KERNEL_TRACE") != nullptr) {
        std::fprintf(stderr,
                     "[BC_RELAX] VendoredHiGHS live append required but "
                     "unavailable: %s\n",
                     live_append_reject ? live_append_reject->c_str()
                                        : "unknown");
      }
      return false;
    }
  }
  const bool use_direct_highs_lp =
      opt.auto_highs_root_pipeline ||
      (basis_hint == nullptr &&
       std::getenv("MIPSOLVERS_BC_NO_DIRECT_HIGHS_LP") == nullptr);
  if (use_direct_highs_lp) {
    auto highs = std::make_shared<Highs>();
    highs->setOptionValue("output_flag", false);
    highs->setOptionValue("log_to_console", false);
    highs->setOptionValue("random_seed", 0);
    highs->setOptionValue("threads", 1);
    // L2a: presolve the COLD root LP (no incoming basis) so HiGHS reduces the
    // full model before the simplex solve.  On root-gap-closable SCUC the
    // un-presolved 10151-row solve (~120ms) collapses to the presolved path
    // (~17ms).  Node LPs (basis_hint present) keep presolve OFF so the
    // original-space warm basis is applied directly.  Env escape hatch for A/B.
    const bool cold_root_solve = (basis_hint == nullptr);
    const bool cold_root_presolve =
        cold_root_solve &&
        std::getenv("MIPSOLVERS_BC_NO_COLD_ROOT_PRESOLVE") == nullptr;
    highs->setOptionValue("presolve", cold_root_presolve ? "on" : "off");
    highs->setOptionValue("parallel", "off");
    highs->setOptionValue("solver", "simplex");
    highs->setOptionValue("simplex_strategy", 1);
    highs->setOptionValue("simplex_scale_strategy", 2);
    highs->setOptionValue("simplex_initial_condition_check", false);
    highs->setOptionValue("primal_feasibility_tolerance",
                          kHighsDefaultMipTolerance);
    highs->setOptionValue("dual_feasibility_tolerance",
                          kHighsDefaultKktTolerance);
    highs->setOptionValue("mip_feasibility_tolerance",
                          kHighsDefaultMipTolerance);
    highs->setOptionValue("simplex_iteration_limit",
                          std::max(opt.max_lp_iter * 50, 10000));

    if (!highs_pass_lp_model(*highs, lp)) {
      if (opt.verbose ||
          std::getenv("MIPSOLVERS_HIGHS_LP_KERNEL_TRACE") != nullptr) {
        std::fprintf(stderr,
                     "[BC_RELAX] VendoredHiGHS direct LP failed: pass_model\n");
      }
      return false;
    }
    HighsBasis warm_basis;
    if (native_basis_to_highs_basis(lp, basis_hint, warm_basis)) {
      (void)highs->setBasis(warm_basis, "MIPSOLVERS direct LP basis hint");
    }

    const HighsStatus run_status = highs->run();
    const HighsModelStatus model_status = highs->getModelStatus();
    const HighsInfo& info = highs->getInfo();

    SolveResult primal;
    primal.stats.solver_name = "VendoredHighsDirectLpKernel";
    primal.stats.iterations =
        static_cast<int>(info.simplex_iteration_count);
    primal.stats.objective = info.objective_function_value;
      primal.stats.status =
        std::string("HiGHS ") + highs_model_status_label(model_status);
    primal.stats.success = run_status == HighsStatus::kOk &&
                           model_status == HighsModelStatus::kOptimal;
    primal.stats.runtime_sec =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - t0)
            .count();

    if (model_status == HighsModelStatus::kInfeasible) {
      primal.stats.status = "HiGHS Infeasible";
      primal.stats.has_farkas_certificate = true;
      out.primal = std::move(primal);
      out.dual_bound = std::numeric_limits<double>::infinity();
      return true;
    }
    if (!primal.stats.success) {
      if (opt.verbose ||
          std::getenv("MIPSOLVERS_HIGHS_LP_KERNEL_TRACE") != nullptr) {
        std::fprintf(stderr,
                     "[BC_RELAX] VendoredHiGHS direct LP rejected: run=%d "
                     "status=%s iter=%d\n",
                     static_cast<int>(run_status),
                     highs_model_status_label(model_status),
                     primal.stats.iterations);
      }
      return false;
    }

    const int n = static_cast<int>(lp.vars.size());
    const int m = static_cast<int>(lp.A.rows() + lp.Aeq.rows());
    const HighsSolution& sol = highs->getSolution();
    if (static_cast<int>(sol.col_value.size()) < n) return false;

    primal.x = Eigen::VectorXd::Zero(n);
    primal.box_dual_lb = Eigen::VectorXd::Zero(n);
    primal.box_dual_ub = Eigen::VectorXd::Zero(n);
    for (int j = 0; j < n; ++j) {
      primal.x[j] = sol.col_value[static_cast<std::size_t>(j)];
      const double cd = static_cast<int>(sol.col_dual.size()) > j
                            ? sol.col_dual[static_cast<std::size_t>(j)]
                            : 0.0;
      if (cd > 0.0) {
        primal.box_dual_lb[j] = cd;
      } else if (cd < 0.0) {
        primal.box_dual_ub[j] = -cd;
      }
    }
    if (static_cast<int>(sol.row_dual.size()) >= m) {
      primal.constraint_duals = Eigen::VectorXd::Zero(m);
      for (int i = 0; i < m; ++i) {
        primal.constraint_duals[i] =
            sol.row_dual[static_cast<std::size_t>(i)];
      }
    }

    auto simplex = std::make_shared<SimplexResult>();
    if (!build_native_simplex_state_from_highs(lp, highs, primal, *simplex)) {
      if (opt.verbose ||
          std::getenv("MIPSOLVERS_HIGHS_LP_KERNEL_TRACE") != nullptr) {
        std::fprintf(stderr,
                     "[BC_RELAX] VendoredHiGHS direct LP solved but basis "
                     "import failed\n");
      }
      return false;
    }

    out.simplex = simplex;
    out.primal = std::move(primal);
    out.dual_bound = out.primal.stats.objective;
    out.row_duals = out.primal.constraint_duals;
    out.basis_hint = make_first_class_basis_hint_from_simplex(*simplex);
    if (opt.verbose ||
        std::getenv("MIPSOLVERS_HIGHS_LP_KERNEL_TRACE") != nullptr) {
      std::fprintf(stderr,
                   "[BC_RELAX] VendoredHiGHS direct LP: success=1 iter=%d "
                   "obj=%.12g rows=%d cols=%d basisRows=%d time=%.3fms\n",
                   out.primal.stats.iterations, out.primal.stats.objective, m,
                   n, out.basis_hint ? out.basis_hint->rows : -1,
                   out.primal.stats.runtime_sec * 1000.0);
    }
    return true;
  }

  StandardFormLP sf = build_standard_form_lp(lp);
  ruiz_scale_standard_form(sf);
  SimplexOptions sf_opt;
  sf_opt.max_iter = std::max(opt.max_lp_iter * 50, 10000);
  sf_opt.feasibility_tol = kHighsDefaultKktTolerance;
  sf_opt.optimality_tol = kHighsDefaultKktTolerance;
  sf_opt.verbose = false;
  sf_opt.allow_cold_start = true;
  sf_opt.use_partial_pricing = false;
  sf_opt.perturb_degenerate_primal = false;
  sf_opt.lp_kernel_backend = LpKernelBackend::HiGHS;
  sf_opt.factor_backend = simplex_factor_backend_from_id(opt.simplex_factor_backend);
  apply_root_simplex_conformance_options(sf_opt, /*force=*/true);

  auto simplex = std::make_shared<SimplexResult>(
      solve_lp_from_sf(sf, sf_opt, basis_hint));
  if (simplex->result.stats.status == "LP infeasible") {
    out.primal = simplex->result;
    out.dual_bound = std::numeric_limits<double>::infinity();
    out.simplex = simplex;
    return true;
  }
  if (simplex->result.stats.success &&
      simplex->result.x.size() == static_cast<int>(lp.vars.size())) {
    out.simplex = simplex;
    out.primal = simplex->result;
    out.dual_bound = out.primal.stats.objective;
    out.row_duals = out.primal.constraint_duals;
    out.basis_hint = make_first_class_basis_hint_from_simplex(*simplex);
    if (opt.verbose || std::getenv("MIPSOLVERS_HIGHS_LP_KERNEL_TRACE") != nullptr) {
      std::fprintf(stderr,
                   "[BC_RELAX] VendoredHiGHS SF LP: success=1 iter=%d obj=%.12g "
                   "rows=%d cols=%d basisRows=%d time=%.3fms\n",
                   out.primal.stats.iterations, out.primal.stats.objective,
                   static_cast<int>(sf.A.rows()), static_cast<int>(sf.A.cols()),
                   out.basis_hint ? out.basis_hint->rows : -1,
                   std::chrono::duration<double, std::milli>(
                       std::chrono::steady_clock::now() - t0).count());
    }
    return true;
  }

  if (opt.verbose || std::getenv("MIPSOLVERS_HIGHS_LP_KERNEL_TRACE") != nullptr) {
    std::fprintf(stderr,
                 "[BC_RELAX] VendoredHiGHS SF LP failed: status=%s rows=%d cols=%d\n",
                 simplex->result.stats.status.c_str(),
                 static_cast<int>(sf.A.rows()), static_cast<int>(sf.A.cols()));
  }
  return false;
}
#else
bool solve_lp_relaxation_with_vendored_highs(const LPModel&,
                                             const SimplexBasis*,
                                             const BCOptions&,
                                             LPRelaxationResult&) {
  return false;
}
#endif

Eigen::VectorXd primal_to_scaled_standard_form(const StandardFormLP& sf,
                                               const Eigen::VectorXd& x_orig) {
  const int sf_m = static_cast<int>(sf.A.rows());
  const int sf_n = static_cast<int>(sf.A.cols());
  const bool have_col_scale = (sf.col_scale.size() == sf_n);
  Eigen::VectorXd x_sf = Eigen::VectorXd::Zero(sf_n);
  for (int j = 0; j < sf.n_original && j < x_orig.size(); ++j) {
    double x_shifted = x_orig[j] - sf.lb_shift[j];
    x_sf[j] = have_col_scale ? (x_shifted / sf.col_scale[j]) : x_shifted;
  }
  Eigen::VectorXd ax = sf.A * x_sf;
  for (int i = 0; i < sf_m; ++i) {
    int sc = sf.row_to_slack_col[i];
    if (sc >= 0) {
      const double col_scale = have_col_scale ? sf.col_scale[sc] : 1.0;
      const double a_sc = row_col_coefficient(sf, i, sc);
      if (std::abs(a_sc) > 1e-15) {
        x_sf[sc] = std::max(0.0, (sf.b[i] - ax[i]) / a_sc);
      } else {
        x_sf[sc] = have_col_scale
            ? std::max(0.0, sf.b[i] - ax[i]) / col_scale
            : std::max(0.0, sf.b[i] - ax[i]);
      }
    }
    int su = sf.row_to_surplus_col[i];
    if (su >= 0) {
      const double col_scale = have_col_scale ? sf.col_scale[su] : 1.0;
      const double a_su = row_col_coefficient(sf, i, su);
      if (std::abs(a_su) > 1e-15) {
        x_sf[su] = std::max(0.0, (sf.b[i] - ax[i]) / a_su);
      } else {
        x_sf[su] = have_col_scale
            ? std::max(0.0, ax[i] - sf.b[i]) / col_scale
            : std::max(0.0, ax[i] - sf.b[i]);
      }
    }
  }
  return x_sf;
}

struct StandardFormHandoffAudit {
  double primal_residual_inf{std::numeric_limits<double>::infinity()};
  double objective_from_standard_form{std::numeric_limits<double>::quiet_NaN()};
  double objective_gap_to_solver{std::numeric_limits<double>::infinity()};
  double legacy_rc_inf{std::numeric_limits<double>::infinity()};
  double legacy_stationarity_best_inf{std::numeric_limits<double>::infinity()};
  double public_rc_inf{std::numeric_limits<double>::infinity()};
  double public_stationarity_best_inf{std::numeric_limits<double>::infinity()};
  bool have_row_duals{false};
  bool have_box_duals{false};
};

void update_stationarity_audit(const StandardFormLP& sf,
                               const SolveResult& ipm_res,
                               const Eigen::VectorXd& y_sf,
                               double& rc_inf,
                               double& stationarity_best_inf) {
  const int sf_n = static_cast<int>(sf.A.cols());
  const bool have_col_scale = sf.col_scale.size() == sf_n;
  Eigen::VectorXd rc = sf.c_max - Eigen::VectorXd(sf.A.transpose() * y_sf);
  if (rc.size() != sf_n || !rc.allFinite()) return;
  rc_inf = rc.cwiseAbs().maxCoeff();
  Eigen::VectorXd stat_minus_plus = rc;
  Eigen::VectorXd stat_plus_minus = rc;
  const bool have_box = ipm_res.box_dual_lb.size() >= sf.n_original &&
                        ipm_res.box_dual_ub.size() >= sf.n_original;
  if (have_box) {
    for (int j = 0; j < sf.n_original; ++j) {
      const double col_scale = have_col_scale ? sf.col_scale[j] : 1.0;
      const double zl = ipm_res.box_dual_lb[j] * col_scale;
      const double zu = ipm_res.box_dual_ub[j] * col_scale;
      stat_minus_plus[j] = rc[j] - zl + zu;
      stat_plus_minus[j] = rc[j] + zl - zu;
    }
  }
  double best = std::numeric_limits<double>::infinity();
  if (stat_minus_plus.allFinite()) {
    best = std::min(best, stat_minus_plus.cwiseAbs().maxCoeff());
  }
  if (stat_plus_minus.allFinite()) {
    best = std::min(best, stat_plus_minus.cwiseAbs().maxCoeff());
  }
  stationarity_best_inf = best;
}

StandardFormHandoffAudit audit_ipm_standard_form_handoff(
    const StandardFormLP& sf,
    const SolveResult& ipm_res) {
  StandardFormHandoffAudit a;
  const int sf_m = static_cast<int>(sf.A.rows());
  const int sf_n = static_cast<int>(sf.A.cols());
  if (ipm_res.x.size() < sf.n_original || sf_m <= 0 || sf_n <= 0) return a;

  Eigen::VectorXd x_sf = primal_to_scaled_standard_form(sf, ipm_res.x);
  if (x_sf.size() == sf_n) {
    Eigen::VectorXd r = sf.A * x_sf - sf.b;
    if (r.size() > 0 && r.allFinite()) {
      a.primal_residual_inf = r.cwiseAbs().maxCoeff();
    }
    a.objective_from_standard_form = sf.objective_const - sf.c_max.dot(x_sf);
    a.objective_gap_to_solver =
        std::abs(a.objective_from_standard_form - ipm_res.stats.objective);
  }

  a.have_row_duals = ipm_res.constraint_duals.size() >= sf_m;
  a.have_box_duals = ipm_res.box_dual_lb.size() >= sf.n_original &&
                     ipm_res.box_dual_ub.size() >= sf.n_original;
  if (!a.have_row_duals) return a;

  const bool have_row_scale = sf.row_scale.size() == sf_m;
  Eigen::VectorXd y_legacy_sf = Eigen::VectorXd::Zero(sf_m);
  Eigen::VectorXd y_public_sf = Eigen::VectorXd::Zero(sf_m);
  for (int i = 0; i < sf_m; ++i) {
    const double row_sign =
        (i < static_cast<int>(sf.row_sign.size()))
            ? static_cast<double>(sf.row_sign[static_cast<std::size_t>(i)])
            : 1.0;
    const double row_scale = have_row_scale ? sf.row_scale[i] : 1.0;
    if (row_scale != 0.0 && std::isfinite(row_scale)) {
      // Legacy IPM crash-basis convention currently used below.
      y_legacy_sf[i] = row_sign * ipm_res.constraint_duals[i] / row_scale;
      // Public simplex convention from populate_dual_certificate:
      // row_dual = -row_sign * row_scale * y_scaled.
      y_public_sf[i] = -row_sign * ipm_res.constraint_duals[i] / row_scale;
    }
  }
  update_stationarity_audit(sf, ipm_res, y_legacy_sf,
                            a.legacy_rc_inf,
                            a.legacy_stationarity_best_inf);
  update_stationarity_audit(sf, ipm_res, y_public_sf,
                            a.public_rc_inf,
                            a.public_stationarity_best_inf);
  return a;
}

void trace_ipm_handoff_audit(const char* stage,
                             const LPModel& lp,
                             const SolveResult& ipm_res,
                             const StandardFormLP* sf,
                             const SimplexResult* simplex) {
  const PrimalAuditMetrics p = audit_original_primal(lp, ipm_res);
  std::fprintf(stderr,
               "[BC_IPM_HANDOFF] stage=%s ipmSuccess=%d ipmStatus='%s' "
               "ipmObj=%.12g objRe=%.12g objGap=%.3e "
               "origViol=bnd%.3e:ineqU%.3e:ineqL%.3e:eq%.3e",
               stage == nullptr ? "unknown" : stage,
               ipm_res.stats.success ? 1 : 0,
               ipm_res.stats.status.c_str(),
               ipm_res.stats.objective,
               p.objective_recomputed,
               p.objective_gap,
               p.bound_violation,
               p.ineq_upper_violation,
               p.ineq_lower_violation,
               p.eq_violation);
  if (sf != nullptr) {
    const StandardFormHandoffAudit s =
        audit_ipm_standard_form_handoff(*sf, ipm_res);
    std::fprintf(stderr,
                 " sf_m=%d sf_n=%d sfPr=%.3e sfObj=%.12g sfObjGap=%.3e "
                 "dual=rows%d:box%d legacyRc=%.3e legacyStat=%.3e "
                 "publicRc=%.3e publicStat=%.3e",
                 static_cast<int>(sf->A.rows()),
                 static_cast<int>(sf->A.cols()),
                 s.primal_residual_inf,
                 s.objective_from_standard_form,
                 s.objective_gap_to_solver,
                 s.have_row_duals ? 1 : 0,
                 s.have_box_duals ? 1 : 0,
                 s.legacy_rc_inf,
                 s.legacy_stationarity_best_inf,
                 s.public_rc_inf,
                 s.public_stationarity_best_inf);
  }
  if (simplex != nullptr) {
    const PrimalAuditMetrics sp = audit_original_primal(lp, simplex->result);
    double sx_sf_resid = std::numeric_limits<double>::infinity();
    if (simplex->form.A.rows() > 0 &&
        simplex->x_std.size() == simplex->form.A.cols()) {
      Eigen::VectorXd sr = simplex->form.A * simplex->x_std - simplex->form.b;
      if (sr.size() > 0 && sr.allFinite()) sx_sf_resid = sr.cwiseAbs().maxCoeff();
    }
    std::fprintf(stderr,
                 " sxSuccess=%d sxStatus='%s' sxObj=%.12g "
                 "sxObjRe=%.12g xoverObjGap=%.3e sxSfPr=%.3e "
                 "sxViol=bnd%.3e:ineqU%.3e:ineqL%.3e:eq%.3e",
                 simplex->result.stats.success ? 1 : 0,
                 simplex->result.stats.status.c_str(),
                 simplex->result.stats.objective,
                 sp.objective_recomputed,
                 std::abs(simplex->result.stats.objective -
                          ipm_res.stats.objective),
                 sx_sf_resid,
                 sp.bound_violation,
                 sp.ineq_upper_violation,
                 sp.ineq_lower_violation,
                 sp.eq_violation);
  }
  std::fprintf(stderr, "\n");
}

double row_col_coefficient(const StandardFormLP& sf, int row, int col) {
  for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(sf.A_row, row); it; ++it) {
    if (it.col() == col) return it.value();
  }
  return 0.0;
}

CrashBasisRecoveryStats recover_primal_activity_basis_impl(const StandardFormLP& sf,
                                                           const Eigen::VectorXd& x_orig,
                                                           const SolveResult* ipm_res,
                                                           const SimplexBasis* seed_basis,
                                                           SimplexBasis& out_basis) {
  const int sf_m = static_cast<int>(sf.A.rows());
  const int sf_n = static_cast<int>(sf.A.cols());
  Eigen::VectorXd x_sf = primal_to_scaled_standard_form(sf, x_orig);

  CrashBasisRecoveryStats stats;
  out_basis.rows = sf_m;
  out_basis.cols = sf_n;
  out_basis.indices.resize(sf_m);
  out_basis.at_upper.assign(sf_n, 0);
  out_basis.cached_sparse_basis.reset();
  out_basis.cached_reduced_costs.reset();
  out_basis.persist_eta_count = 0;

  std::vector<double> interior_score(static_cast<size_t>(sf.n_original), 0.0);
  std::vector<double> partition_score(static_cast<size_t>(sf.n_original), 0.0);
  std::vector<double> row_dual_score(static_cast<size_t>(sf_m), 1.0);
  const double crash_rel_tol = 1e-5;
  const bool have_col_scale = (sf.col_scale.size() == sf_n);
  const bool have_row_scale = (sf.row_scale.size() == sf_m);

  Eigen::VectorXd s_sf = Eigen::VectorXd::Zero(sf_n);
  bool have_rc = false;
  bool have_box = false;
  if (ipm_res != nullptr) {
    if (ipm_res->constraint_duals.size() > 0) {
      Eigen::VectorXd y_sf = Eigen::VectorXd::Zero(sf_m);
      for (int i = 0; i < sf_m && i < static_cast<int>(ipm_res->constraint_duals.size()); ++i) {
        double y_orig = sf.row_sign[i] * ipm_res->constraint_duals[i];
        y_sf[i] = have_row_scale ? (y_orig / sf.row_scale[i]) : y_orig;
        row_dual_score[static_cast<size_t>(i)] = std::abs(y_sf[i]) / (1.0 + std::abs(y_sf[i]));
      }
      s_sf = sf.c_max - Eigen::VectorXd(sf.A.transpose() * y_sf);
      have_rc = true;
    }
    have_box = (ipm_res->box_dual_lb.size() >= sf.n_original &&
                ipm_res->box_dual_ub.size() >= sf.n_original);
  }

  for (int j = 0; j < sf_n; ++j) {
    double val = x_sf[j];
    double ub_j = sf.var_ub[j];
    double range = std::isfinite(ub_j) ? ub_j : std::max(1.0, std::abs(val));
    double rel_from_lb = val / std::max(range, 1e-10);
    double rel_from_ub = std::isfinite(ub_j)
        ? (ub_j - val) / std::max(range, 1e-10) : 1.0;
    if (rel_from_ub < crash_rel_tol && std::isfinite(ub_j)) {
      out_basis.at_upper[static_cast<size_t>(j)] = 1;
    }
    if (j < sf.n_original && rel_from_lb >= crash_rel_tol && rel_from_ub >= crash_rel_tol) {
      interior_score[static_cast<size_t>(j)] = std::min(rel_from_lb, rel_from_ub);
      if (have_box) {
        const double col_sc = have_col_scale ? sf.col_scale[j] : 1.0;
        const double zl_j = ipm_res->box_dual_lb(j) * col_sc;
        const double zu_j = ipm_res->box_dual_ub(j) * col_sc;
        const double gap_lb = val;
        const double gap_ub = std::isfinite(ub_j) ? (ub_j - val) : 1.0;
        const double pi_lb = gap_lb / std::sqrt(gap_lb * gap_lb + zl_j * zl_j + 1e-20);
        const double pi_ub = gap_ub / std::sqrt(gap_ub * gap_ub + zu_j * zu_j + 1e-20);
        partition_score[static_cast<size_t>(j)] = pi_lb * pi_ub;
      } else if (have_rc) {
        partition_score[static_cast<size_t>(j)] = val / (val + std::abs(s_sf[j]) + 1e-10);
      } else {
        partition_score[static_cast<size_t>(j)] = interior_score[static_cast<size_t>(j)];
      }
    }
  }

  const bool seeded =
      (seed_basis != nullptr && seed_basis->rows == sf_m && seed_basis->cols == sf_n &&
       static_cast<int>(seed_basis->index_count()) == sf_m);
  if (seeded) {
    out_basis.indices = seed_basis->basis_indices();
    if (seed_basis->at_upper.size() == static_cast<size_t>(sf_n)) {
      for (int j = sf.n_original; j < sf_n; ++j) {
        out_basis.at_upper[static_cast<size_t>(j)] =
            seed_basis->at_upper[static_cast<size_t>(j)];
      }
    }
  } else {
    for (int i = 0; i < sf_m; ++i) {
      if (sf.row_to_slack_col[i] >= 0) {
        out_basis.indices[static_cast<size_t>(i)] = sf.row_to_slack_col[i];
      } else if (sf.row_to_artificial_col[i] >= 0) {
        out_basis.indices[static_cast<size_t>(i)] = sf.row_to_artificial_col[i];
      } else {
        out_basis.indices[static_cast<size_t>(i)] = -1;
      }
    }
  }

  struct RowCandidate {
    double score;
    double partition;
    double row_dual;
    double activity;
    int row;
    int col;
    bool operator<(const RowCandidate& other) const { return score > other.score; }
  };

  std::vector<RowCandidate> candidates;
  candidates.reserve(sf_m * 2);
  std::vector<char> col_used(static_cast<size_t>(sf_n), 0);
  for (int i = 0; i < sf_m; ++i) {
    const int idx = out_basis.indices[static_cast<size_t>(i)];
    if (idx >= 0 && idx < sf_n) col_used[static_cast<size_t>(idx)] = 1;
  }

  const int max_candidates_per_row = 3;
  const double min_partition_for_swap = (have_box || have_rc) ? 0.15 : 1e-6;
  const double min_row_dual_for_swap = (ipm_res != nullptr && ipm_res->constraint_duals.size() > 0)
      ? 1e-2 : 0.0;
  for (int row = 0; row < sf_m; ++row) {
    const int current_basis_col = out_basis.indices[static_cast<size_t>(row)];
    const bool row_is_seed_slack = (current_basis_col == sf.row_to_slack_col[row] ||
                                    current_basis_col == sf.row_to_artificial_col[row] ||
                                    current_basis_col < 0);
    if (seeded && !row_is_seed_slack) continue;

    const int slack_col = sf.row_to_slack_col[row];
    const int art_col = sf.row_to_artificial_col[row];
    const double row_score = row_dual_score[static_cast<size_t>(row)];
    const bool tight_row = (slack_col >= 0)
        ? x_sf[slack_col] <= 1e-6 * std::max(1.0, std::abs(sf.b[row]))
        : (art_col >= 0);
    if (!tight_row && row_score < min_row_dual_for_swap) continue;

    std::array<RowCandidate, 3> best{{
        { -std::numeric_limits<double>::infinity(), 0.0, 0.0, 0.0, row, -1 },
        { -std::numeric_limits<double>::infinity(), 0.0, 0.0, 0.0, row, -1 },
        { -std::numeric_limits<double>::infinity(), 0.0, 0.0, 0.0, row, -1 },
    }};
    int row_candidates = 0;
    for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(sf.A_row, row); it; ++it) {
      const int col = it.col();
      if (col >= sf.n_original || col_used[static_cast<size_t>(col)]) continue;
      const double part_score = partition_score[static_cast<size_t>(col)];
      if (part_score < min_partition_for_swap) continue;
      const double coeff = std::abs(it.value());
      const double activity = coeff * std::abs(x_sf[col]);
      const double score = activity * (0.25 + part_score) * (0.25 + row_score);
      RowCandidate cand{score, part_score, row_score, activity, row, col};
      ++row_candidates;
      for (int pos = 0; pos < max_candidates_per_row; ++pos) {
        if (cand.score > best[static_cast<size_t>(pos)].score) {
          for (int shift = max_candidates_per_row - 1; shift > pos; --shift) {
            best[static_cast<size_t>(shift)] = best[static_cast<size_t>(shift - 1)];
          }
          best[static_cast<size_t>(pos)] = cand;
          break;
        }
      }
    }
    if (row_candidates == 0) continue;
    ++stats.candidate_rows;
    for (const auto& cand : best) {
      if (cand.col >= 0) candidates.push_back(cand);
    }
  }
  std::sort(candidates.begin(), candidates.end());

  const int max_structural_swaps = std::min({sf_m, sf.n_original, seeded ? 64 : 256});
  std::vector<int> selected_rows;
  std::vector<int> selected_cols;
  std::vector<char> row_swapped(static_cast<size_t>(sf_m), 0);
  Eigen::MatrixXd selected_submatrix(0, 0);
  double selected_partition_sum = 0.0;
  double selected_row_dual_sum = 0.0;
  double selected_activity_sum = 0.0;

  for (const auto& cand : candidates) {
    if (static_cast<int>(selected_rows.size()) >= max_structural_swaps) break;
    if (row_swapped[static_cast<size_t>(cand.row)] || col_used[static_cast<size_t>(cand.col)]) continue;

    const int k = static_cast<int>(selected_rows.size());
    Eigen::MatrixXd trial = Eigen::MatrixXd::Zero(k + 1, k + 1);
    if (k > 0) {
      trial.topLeftCorner(k, k) = selected_submatrix;
      for (int idx = 0; idx < k; ++idx) {
        trial(idx, k) = row_col_coefficient(sf, selected_rows[static_cast<size_t>(idx)], cand.col);
        trial(k, idx) = row_col_coefficient(sf, cand.row, selected_cols[static_cast<size_t>(idx)]);
      }
    }
    trial(k, k) = row_col_coefficient(sf, cand.row, cand.col);

    Eigen::FullPivLU<Eigen::MatrixXd> lu(trial);
    if (lu.rank() != k + 1) continue;

    const int old_col = out_basis.indices[static_cast<size_t>(cand.row)];
    if (old_col >= 0 && old_col < sf_n) {
      col_used[static_cast<size_t>(old_col)] = 0;
    }
    selected_submatrix = std::move(trial);
    selected_rows.push_back(cand.row);
    selected_cols.push_back(cand.col);
    row_swapped[static_cast<size_t>(cand.row)] = 1;
    col_used[static_cast<size_t>(cand.col)] = 1;
    out_basis.indices[static_cast<size_t>(cand.row)] = cand.col;
    selected_partition_sum += cand.partition;
    selected_row_dual_sum += cand.row_dual;
    selected_activity_sum += cand.activity;
  }

  stats.selected_swaps = static_cast<int>(selected_rows.size());
  if (stats.selected_swaps > 0) {
    stats.mean_partition_score = selected_partition_sum / stats.selected_swaps;
    stats.mean_row_dual_score = selected_row_dual_sum / stats.selected_swaps;
    stats.mean_activity_score = selected_activity_sum / stats.selected_swaps;
  }
  const int min_required_swaps = std::min(seeded ? 3 : 8, stats.candidate_rows);
  stats.passes_screen =
      stats.selected_swaps > 0 &&
      stats.selected_swaps >= min_required_swaps &&
      stats.mean_partition_score >= ((have_box || have_rc) ? 0.2 : 0.05) &&
      stats.mean_row_dual_score >= ((ipm_res != nullptr && ipm_res->constraint_duals.size() > 0) ? 0.02 : 0.0);
  if (lp_basis_trace_enabled()) {
    char detail[256];
    std::snprintf(detail, sizeof(detail),
                  "seeded=%d haveBox=%d haveRc=%d candRows=%d cand=%zu "
                  "swaps=%d minReq=%d part=%.6g rowdual=%.6g activity=%.6g "
                  "screen=%d",
                  seeded ? 1 : 0, have_box ? 1 : 0, have_rc ? 1 : 0,
                  stats.candidate_rows, candidates.size(), stats.selected_swaps,
                  min_required_swaps, stats.mean_partition_score,
                  stats.mean_row_dual_score, stats.mean_activity_score,
                  stats.passes_screen ? 1 : 0);
    trace_basis_source("primal_activity_recovered", sf, out_basis, detail);
  }
  return stats;
}

SimplexBasis build_primal_crash_basis(const StandardFormLP& sf,
                                      const Eigen::VectorXd& x_orig) {
  SimplexBasis crash_basis;
  (void)recover_primal_activity_basis_impl(sf, x_orig, nullptr, nullptr, crash_basis);
  trace_basis_source("primal_crash_basis", sf, crash_basis, "from=x0");
  return crash_basis;
}

int count_fractional_integer_vars(const LPModel& lp, const SolveResult& res,
                                  double tol) {
  const int n = static_cast<int>(lp.vars.size());
  if (!res.stats.success || res.x.size() != n) {
    return std::numeric_limits<int>::max();
  }
  int count = 0;
  for (int i = 0; i < n; ++i) {
    if (!is_integer_type(lp.vars[i])) continue;
    const double dist = std::abs(res.x[i] - std::round(res.x[i]));
    if (dist > tol) ++count;
  }
  return count;
}

}  // namespace

CrashBasisRecoveryStats recover_primal_activity_basis(const StandardFormLP& sf,
                                                      const Eigen::VectorXd& x_orig,
                                                      const SolveResult* ipm_res,
                                                      const SimplexBasis* seed_basis,
                                                      SimplexBasis& out_basis) {
  return recover_primal_activity_basis_impl(sf, x_orig, ipm_res, seed_basis, out_basis);
}

LPRelaxationResult solve_lp_relaxation(const LPModel& lp,
                                       const Eigen::VectorXd* x0,
                                       const SimplexBasis* basis_hint,
                                       const BCOptions& opt) {
  LPRelaxationResult out;
  if (opt.use_simplex_lp_nodes && vendored_highs_lp_kernel_enabled(opt)) {
    if (solve_lp_relaxation_with_vendored_highs(lp, basis_hint, opt, out)) {
      return out;
    }
    if (opt.verbose || std::getenv("MIPSOLVERS_HIGHS_LP_KERNEL_TRACE") != nullptr) {
      std::fprintf(stderr,
                   "[BC_RELAX] VendoredHiGHS LP unavailable/failed; "
                   "strict HiGHS LP contract rejects native fallback\n");
    }
    out.primal.stats.solver_name = "VendoredHighsLpKernel";
    out.primal.stats.success = false;
    out.primal.stats.status = "VendoredHiGHS LP rejected";
    out.dual_bound = std::numeric_limits<double>::infinity();
    return out;
  }
  const bool trace_huge_root_relax = opt.verbose;
  auto relax_t0 = std::chrono::steady_clock::now();
  const double lp_wall_budget = opt.time_limit_sec > 0.0 ? opt.time_limit_sec : 0.0;
  // Each solver stage (IPM root, simplex, IPM repair) must run against what
  // is LEFT of the budget, not the full budget again — otherwise a failing
  // multi-stage solve overruns the caller's limit severalfold.  0 keeps the
  // "unlimited" convention of {Simplex,IPMLP}Options::time_limit_sec; the
  // 1 ms floor keeps an exhausted budget from turning back into "unlimited".
  auto lp_remaining_budget = [&]() -> double {
    if (lp_wall_budget <= 0.0) return 0.0;
    const double elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - relax_t0).count();
    return std::max(0.001, lp_wall_budget - elapsed);
  };
  auto lp_budget_exhausted = [&]() -> bool {
    return lp_wall_budget > 0.0 && lp_remaining_budget() <= 0.001;
  };
  auto apply_simplex_budget = [&](SimplexOptions& simplex_opt) {
    simplex_opt.time_limit_sec = lp_remaining_budget();
  };
  auto apply_ipm_budget = [&](IPMLPOptions& ipm_opt) {
    ipm_opt.time_limit_sec = lp_remaining_budget();
  };

  const int n_lp = static_cast<int>(lp.vars.size());
  const bool huge_warmstart_root =
      (x0 != nullptr && x0->size() == n_lp && n_lp >= 30000);

  if (huge_warmstart_root) {
    StandardFormLP warm_sf = build_standard_form_lp(lp);
    ruiz_scale_standard_form(warm_sf);
    SimplexBasis warm_basis = build_primal_crash_basis(warm_sf, *x0);

    SimplexOptions warm_opt;
    warm_opt.max_iter = std::min(std::max(static_cast<int>(warm_sf.A.rows()) / 2, 2000), 12000);
    warm_opt.feasibility_tol = 1e-6;
    warm_opt.optimality_tol = 1e-6;
    warm_opt.verbose = false;
    warm_opt.allow_cold_start = false;
    warm_opt.use_partial_pricing = true;
    warm_opt.factor_backend = simplex_factor_backend_from_id(opt.simplex_factor_backend);
    warm_opt.lp_kernel_backend = opt.lp_kernel_backend;
    apply_simplex_budget(warm_opt);
    apply_root_simplex_conformance_options(warm_opt, opt.use_simplex_lp_nodes);

    auto warm_simplex = std::make_shared<SimplexResult>(
        solve_lp_from_sf(warm_sf, warm_opt, &warm_basis));
    if (opt.verbose || warm_sf.A.rows() > 10000) {
      fprintf(stderr,
          "[BC_RELAX] simplex with basis: path=warm-start success=%d iters=%d obj=%.2f\n",
              warm_simplex->result.stats.success ? 1 : 0,
              warm_simplex->result.stats.iterations,
              warm_simplex->result.stats.objective);
    }
    bool accept_warm_basis = false;
    if (warm_simplex->result.stats.success &&
        warm_simplex->x_std.size() == warm_sf.A.cols()) {
      Eigen::VectorXd sf_resid = warm_sf.A * warm_simplex->x_std - warm_sf.b;
      double max_resid = std::max(sf_resid.maxCoeff(), -sf_resid.minCoeff());
      accept_warm_basis = max_resid < 1e-4 && std::isfinite(warm_simplex->result.stats.objective);
    }
    if (accept_warm_basis) {
      out.simplex = warm_simplex;
      out.primal = warm_simplex->result;
      if (warm_simplex->result.constraint_duals.size() == lp.A.rows() + lp.Aeq.rows()) {
        out.row_duals = warm_simplex->result.constraint_duals;
      }
      out.dual_bound = out.primal.stats.objective;
      out.basis_hint = make_first_class_basis_hint_from_simplex(*warm_simplex);
      return out;
    }
  }

  // P2-short: IPM health probe.  The IPM-LP kernel hard-fails on degenerate
  // and badly scaled instances (NumericalError / Cholesky failed /
  // ProblemTooLarge); a few relaxed-tolerance iterations on the exact LP the
  // IPM would see diagnose that for a fraction of a full doomed solve.  This
  // single choke point covers auto-enabled, forced-huge-warmstart, and
  // caller-set IPM-root routes.
  bool ipm_probe_diagnosed_failure = false;
  Eigen::VectorXd ipm_probe_seed;
  if (opt.use_ipm_root && opt.ipm_root_probe && !lp_budget_exhausted()) {
    IPMLPOptions probe_opt;
    probe_opt.max_iter = std::max(1, opt.ipm_probe_max_iter);
    probe_opt.tol_primal = 1e-6;
    probe_opt.tol_dual = 1e-6;
    probe_opt.tol_gap = 1e-6;
    probe_opt.verbose = false;
    const double rem = lp_remaining_budget();
    probe_opt.time_limit_sec =
        rem > 0.0 ? std::min(opt.ipm_probe_time_sec, 0.15 * rem)
                  : opt.ipm_probe_time_sec;
    NativeIPMLPAdapter probe_solver(probe_opt);
    SolveResult probe_res = probe_solver.solve_lp(lp);
    const std::string& probe_status = probe_res.stats.status;
    if (!probe_res.stats.success &&
        (probe_status == "NumericalError" ||
         probe_status == "Cholesky failed" ||
         probe_status == "ProblemTooLarge")) {
      ipm_probe_diagnosed_failure = true;
      if (opt.verbose) {
        fprintf(stderr,
                "[BC_RELAX] IPM probe unhealthy (status='%s', %d iters) — "
                "skipping IPM root, routing to simplex\n",
                probe_status.c_str(), probe_res.stats.iterations);
      }
    } else if (probe_res.x.size() == n_lp) {
      // Healthy (converged, MaxIter, or Time limit with an iterate): reuse
      // the probe point to warm-start the full solve.
      ipm_probe_seed = std::move(probe_res.x);
    }
  }

  // ---- Interior-point root-LP path ----
  // Uses the new LP IPM adapter (based on Mehrotra LCQP core) to solve
  // the root relaxation directly, then crosses over to simplex to get
  // a basis for Gomory cut generation and warm-started child nodes.
  if (opt.use_ipm_root && !ipm_probe_diagnosed_failure) {
    if (lp_budget_exhausted()) {
      out.primal.stats.success = false;
      out.primal.stats.status = "Time limit";
      return out;
    }
    const bool require_simplex_crossover = opt.use_simplex_lp_nodes;
    IPMLPOptions ipm_opt;
    ipm_opt.max_iter = std::max(opt.max_lp_iter, 200);
    ipm_opt.tol_primal = std::max(1e-9, opt.lp_tol * 0.1);
    ipm_opt.tol_dual = std::max(1e-9, opt.lp_tol * 0.1);
    ipm_opt.tol_gap = std::max(1e-9, opt.lp_tol * 0.1);
    ipm_opt.verbose = false;
    apply_ipm_budget(ipm_opt);

    NativeIPMLPAdapter ipm_solver(ipm_opt);
    SolveResult ipm_res = ipm_probe_seed.size() == n_lp
                              ? ipm_solver.solve_lp(lp, ipm_probe_seed)
                              : ipm_solver.solve_lp(lp);
    if (ipm_res.stats.status == "Time limit") {
      out.primal = std::move(ipm_res);
      out.dual_bound = out.primal.stats.objective;
      return out;
    }
    if (ipm_res.constraint_duals.size() == lp.A.rows() + lp.Aeq.rows()) {
      out.row_duals = ipm_res.constraint_duals;
    }
    auto relax_t_ipm = std::chrono::steady_clock::now();
    if (opt.verbose) {
      fprintf(stderr, "[BC_RELAX] IPM root: success=%d status='%s' iter=%d obj=%.4f time=%.3fs\n",
              ipm_res.stats.success ? 1 : 0,
              ipm_res.stats.status.c_str(),
              ipm_res.stats.iterations,
              ipm_res.stats.objective,
              ipm_res.stats.runtime_sec);
    }
    const bool audit_ipm_handoff = ipm_handoff_audit_enabled(opt);
    if (audit_ipm_handoff) {
      trace_ipm_handoff_audit("ipm_raw", lp, ipm_res, nullptr, nullptr);
    }
    if (ipm_res.stats.success) {
      // ═══════════════════════════════════════════════════════════════════
      // 3-Phase IPM→Simplex Crossover
      //
      // Phase 1: Basis Identification — partition variables by distance from
      //   bounds using relative tolerance. Variables far from both bounds are
      //   "basic candidates". Use complementarity (reduced costs) if available.
      //
      // Phase 2: Push Phase — run simplex with LOOSE feasibility tolerance
      //   to quickly restore primal feasibility from the crash basis. The
      //   loose tolerance prevents cycling on large degenerate problems.
      //
      // Phase 3: Cleanup — re-optimize with TIGHT tolerances from the
      //   Phase 2 basis to reach exact vertex with zero duality gap.
      // ═══════════════════════════════════════════════════════════════════
      const int ipm_frac_int_count = huge_warmstart_root
          ? count_fractional_integer_vars(lp, ipm_res, std::max(1e-6, opt.lp_tol * 10.0))
          : std::numeric_limits<int>::max();
      if (!require_simplex_crossover &&
          huge_warmstart_root && ipm_frac_int_count <= 32) {
        if (opt.verbose) {
          fprintf(stderr,
                  "[BC_RELAX] huge near-integral IPM root: n_frac_int=%d — skipping crossover\n",
                  ipm_frac_int_count);
        }
        out.primal = ipm_res;
        out.dual_bound = ipm_res.stats.objective;
        return out;
      }
      StandardFormLP sf = build_standard_form_lp(lp);
      auto relax_t_sf = std::chrono::steady_clock::now();
      ruiz_scale_standard_form(sf);
      auto relax_t_scale = std::chrono::steady_clock::now();
      const int sf_m = static_cast<int>(sf.A.rows());
      const int sf_n = static_cast<int>(sf.A.cols());
      if (audit_ipm_handoff) {
        trace_ipm_handoff_audit("sf_scaled", lp, ipm_res, &sf, nullptr);
      }

      std::shared_ptr<SimplexResult> simplex;

      // P6.1: Skip crossover entirely on very large problems — the 3-phase
      // IPM→simplex crossover dominates root time (8-39s on 500-1000 gen UC).
      // Return IPM result directly and use IPM for node solves.
      if (!require_simplex_crossover &&
          sf_m > opt.xlarge_ipm_only_threshold) {
        if (opt.verbose) {
          fprintf(stderr, "[BC_RELAX] P6.1: sf_m=%d > %d — skipping crossover (IPM-only mode)\n",
                  sf_m, opt.xlarge_ipm_only_threshold);
        }
        out.primal = ipm_res;
        out.dual_bound = ipm_res.stats.objective;
        // No simplex, no basis_hint — tree exploration uses IPMDiver.
        return out;
      }

      // HiGHS' MIP LP relaxation does not build tableau rows from an
      // interior-only optimal partition basis. It starts from a logical basis
      // (rows basic, columns nonbasic at bounds) and lets simplex/recoverBasis
      // determine the degenerate vertex used by separators. For non-xlarge
      // roots, recover the simplex basis in that same way. The old IPM crash
      // partition is kept only for the exceptional path where simplex node LPs
      // are not requested.
      const bool highs_style_logical_recovery = opt.use_simplex_lp_nodes;

      if (sf_m > 200 && !highs_style_logical_recovery) {
        // ══════════════════════════════════════════════════════════════════
        // Phase 1: Basis Identification via Optimal Partition
        // ══════════════════════════════════════════════════════════════════
        // Convert IPM solution (original/unscaled space) to SCALED standard-form.
        //
        // After ruiz_scale_standard_form(sf):
        //   - sf.A, sf.b, sf.c_max, sf.var_ub: SCALED
        //   - sf.lb_shift: UNSCALED (original lower bounds)
        //   - sf.col_scale[j]: column scaling factor
        //   - sf.row_scale[i]: row scaling factor
        //
        // Transformations:
        //   x_scaled[j] = (x_orig[j] - lb[j]) / col_scale[j]
        //   y_scaled[i] = y_orig[i] / row_scale[i]   (dual prices)
        //   z_scaled[j] = z_orig[j] * col_scale[j]   (bound duals, complementary to x)
        // ══════════════════════════════════════════════════════════════════
        const bool have_col_scale = (sf.col_scale.size() == sf_n);
        const bool have_row_scale = (sf.row_scale.size() == sf_m);

        // Step 1: Convert IPM primal solution to scaled standard-form
        Eigen::VectorXd x_sf = primal_to_scaled_standard_form(sf, ipm_res.x);

        // Step 2: Convert IPM constraint duals to scaled space
        // Scaled reduced cost: s_s = c_s - A_s^T y_s
        Eigen::VectorXd s_sf = Eigen::VectorXd::Zero(sf_n);
        bool have_rc = false;
        if (ipm_res.constraint_duals.size() > 0) {
          Eigen::VectorXd y_sf = Eigen::VectorXd::Zero(sf_m);
          for (int i = 0; i < sf_m && i < static_cast<int>(ipm_res.constraint_duals.size()); ++i) {
            double y_orig = sf.row_sign[i] * ipm_res.constraint_duals[i];
            // Scale: y_scaled = y_orig / row_scale
            y_sf[i] = have_row_scale ? (y_orig / sf.row_scale[i]) : y_orig;
          }
          s_sf = sf.c_max - Eigen::VectorXd(sf.A.transpose() * y_sf);
          have_rc = true;
        }

        // Step 3: Convert IPM bound duals to scaled space (if available)
        // z_scaled = z_orig * col_scale (complementary to x_scaled)
        const bool have_box = (ipm_res.box_dual_lb.size() >= sf.n_original &&
                               ipm_res.box_dual_ub.size() >= sf.n_original);

        // ══════════════════════════════════════════════════════════════════
        // Optimal partition scoring (all quantities now in scaled space)
        //   π_j = (x_j) / sqrt(x_j^2 + z_l_j^2) × (ub_j - x_j) / sqrt((ub_j-x_j)^2 + z_u_j^2)
        // π_j → 1 ⟹ clearly basic (interior, small multipliers)
        // π_j → 0 ⟹ clearly nonbasic at a bound
        // ══════════════════════════════════════════════════════════════════
        std::vector<std::pair<double, int>> scored;
        scored.reserve(sf_n);
        std::vector<char> at_upper_vec(sf_n, 0);
        int trace_orig_at_lb = 0;
        int trace_orig_at_ub = 0;
        int trace_orig_interior = 0;
        int trace_orig_fixed = 0;
        int trace_orig_inf_ub = 0;
        int trace_orig_at_bound_rc_tie = 0;
        int trace_orig_at_lb_rc_tie = 0;
        int trace_orig_at_ub_rc_tie = 0;
        double trace_orig_at_bound_rc_tie_abs_sum = 0.0;
        std::vector<int> trace_sample_lb_tie;
        std::vector<int> trace_sample_ub_tie;
        std::vector<int> trace_sample_ub_binary_like;
        const int trace_max_terms = lp_basis_trace_terms();
        const double trace_rc_tie_tol =
            std::max(1e-8, 10.0 * opt.lp_tol);

        for (int j = 0; j < sf_n; ++j) {
          double val = x_sf[j];
          double ub_j = sf.var_ub[j];
          double range = std::isfinite(ub_j) ? ub_j : std::max(1.0, std::abs(val));
          double rel_from_lb = val / std::max(range, 1e-10);
          double rel_from_ub = std::isfinite(ub_j)
              ? (ub_j - val) / std::max(range, 1e-10) : 1.0;
          const double crash_rel_tol = 1e-4;

          if (rel_from_ub < crash_rel_tol && std::isfinite(ub_j)) {
            at_upper_vec[j] = 1;  // Nonbasic at upper bound
            if (j < sf.n_original) {
              ++trace_orig_at_ub;
              const double rc_abs = have_rc ? std::abs(s_sf[j]) : std::numeric_limits<double>::infinity();
              if (rc_abs <= trace_rc_tie_tol) {
                ++trace_orig_at_bound_rc_tie;
                ++trace_orig_at_ub_rc_tie;
                trace_orig_at_bound_rc_tie_abs_sum += rc_abs;
                if (static_cast<int>(trace_sample_ub_tie.size()) < trace_max_terms)
                  trace_sample_ub_tie.push_back(j);
              }
              if (std::abs(ub_j - 1.0) <= 1e-8 &&
                  static_cast<int>(trace_sample_ub_binary_like.size()) < trace_max_terms) {
                trace_sample_ub_binary_like.push_back(j);
              }
            }
          } else if (rel_from_lb < crash_rel_tol) {
            // Nonbasic at lower bound (default)
            if (j < sf.n_original) {
              ++trace_orig_at_lb;
              if (std::isfinite(ub_j) && std::abs(ub_j) <= 1e-12) ++trace_orig_fixed;
              if (!std::isfinite(ub_j)) ++trace_orig_inf_ub;
              const double rc_abs = have_rc ? std::abs(s_sf[j]) : std::numeric_limits<double>::infinity();
              if (rc_abs <= trace_rc_tie_tol) {
                ++trace_orig_at_bound_rc_tie;
                ++trace_orig_at_lb_rc_tie;
                trace_orig_at_bound_rc_tie_abs_sum += rc_abs;
                if (static_cast<int>(trace_sample_lb_tie.size()) < trace_max_terms)
                  trace_sample_lb_tie.push_back(j);
              }
            }
          } else {
            // Basic candidate — score by optimal partition indicator
            if (j < sf.n_original) ++trace_orig_interior;
            double score;
            if (have_box && j < sf.n_original) {
              // Scale bound duals: z_scaled = z_orig * col_scale
              double col_sc = have_col_scale ? sf.col_scale[j] : 1.0;
              const double zl_j = ipm_res.box_dual_lb(j) * col_sc;
              const double zu_j = ipm_res.box_dual_ub(j) * col_sc;
              const double gap_lb = val;
              const double gap_ub = std::isfinite(ub_j) ? (ub_j - val) : 1.0;
              const double pi_lb = gap_lb / std::sqrt(gap_lb * gap_lb + zl_j * zl_j + 1e-20);
              const double pi_ub = gap_ub / std::sqrt(gap_ub * gap_ub + zu_j * zu_j + 1e-20);
              score = pi_lb * pi_ub;
            } else if (have_rc && j < sf.n_original) {
              // Complementarity score (large x / small |s| → basic)
              score = val / (std::abs(s_sf[j]) + 1e-10);
            } else {
              // Slack/surplus/artificial: distance from nearest bound
              score = std::min(rel_from_lb, rel_from_ub);
            }
            scored.emplace_back(score, j);
          }
        }

        // Only the top sf_m candidates are used in the crash basis.
	        if (static_cast<int>(scored.size()) > sf_m) {
	          std::nth_element(scored.begin(), scored.begin() + sf_m, scored.end(), std::greater<>());
	        } else {
	          std::sort(scored.begin(), scored.end(), std::greater<>());
	        }
	        int scored_orig = 0;
	        int scored_slack = 0;
	        int scored_surplus = 0;
	        int scored_art = 0;
	        for (const auto& item : scored) {
	          const char* kind = basis_col_kind(sf, item.second);
	          if (std::strcmp(kind, "orig") == 0) ++scored_orig;
	          else if (std::strcmp(kind, "slack") == 0) ++scored_slack;
	          else if (std::strcmp(kind, "surplus") == 0) ++scored_surplus;
	          else if (std::strcmp(kind, "art") == 0) ++scored_art;
	        }

	        SimplexBasis crash_basis;
	        crash_basis.rows = sf_m;
	        crash_basis.cols = sf_n;
	        crash_basis.indices.resize(sf_m);
        crash_basis.at_upper = at_upper_vec;

        if (static_cast<int>(scored.size()) >= sf_m) {
          for (int i = 0; i < sf_m; ++i)
            crash_basis.indices[i] = scored[i].second;
        } else {
          // Not enough basic candidates — fill with scored first, then slacks.
          int filled = 0;
          for (auto& [d, j] : scored) {
            if (filled >= sf_m) break;
            crash_basis.indices[filled++] = j;
          }
          std::vector<bool> used(sf_n, false);
          for (int i = 0; i < filled; ++i) used[crash_basis.indices[i]] = true;
          for (int i = 0; i < sf_m && filled < sf_m; ++i) {
            int sc = sf.row_to_slack_col[i];
            if (sc >= 0 && !used[sc]) { crash_basis.indices[filled++] = sc; used[sc] = true; }
          }
	          for (int j = 0; j < sf_n && filled < sf_m; ++j) {
	            if (!used[j]) { crash_basis.indices[filled++] = j; used[j] = true; }
	          }
	        }
	        if (lp_basis_trace_enabled()) {
	          char detail[256];
	          std::snprintf(detail, sizeof(detail),
	                        "source=ipm_crash haveBox=%d haveRc=%d scored=%zu "
	                        "scoredKind=orig%d:slack%d:surplus%d:art%d",
	                        have_box ? 1 : 0, have_rc ? 1 : 0,
	                        scored.size(), scored_orig, scored_slack,
	                        scored_surplus, scored_art);
	          trace_basis_source("ipm_crash_basis", sf, crash_basis, detail);
            auto print_sample = [&](const char* label, const std::vector<int>& cols) {
              std::fprintf(stderr, " %s=[", label);
              for (int k = 0; k < static_cast<int>(cols.size()); ++k) {
                if (k > 0) std::fprintf(stderr, ";");
                const int col = cols[static_cast<std::size_t>(k)];
                const double val = (col >= 0 && col < x_sf.size()) ? x_sf[col] : 0.0;
                const double ub = (col >= 0 && col < sf.var_ub.size())
                    ? sf.var_ub[col] : std::numeric_limits<double>::infinity();
                const double rc = (have_rc && col >= 0 && col < s_sf.size()) ? s_sf[col] : 0.0;
                std::fprintf(stderr, "%d:val%.12g:ub%.12g:rc%.12g",
                             col, val, ub, rc);
              }
              std::fprintf(stderr, "]");
            };
            std::fprintf(stderr,
                         "[B&C-LPBASIS-CRASH] stage=ipm_partition m=%d n=%d nOrig=%d "
                         "origClass=atlb%d:atub%d:interior%d:fixed%d:infub%d "
                         "rcTie=total%d:lb%d:ub%d:absSum%.12g tol%.3g "
                         "scoredOrig=%d scoredSlack=%d",
                         sf_m, sf_n, sf.n_original, trace_orig_at_lb,
                         trace_orig_at_ub, trace_orig_interior, trace_orig_fixed,
                         trace_orig_inf_ub, trace_orig_at_bound_rc_tie,
                         trace_orig_at_lb_rc_tie, trace_orig_at_ub_rc_tie,
                         trace_orig_at_bound_rc_tie_abs_sum, trace_rc_tie_tol,
                         scored_orig, scored_slack);
            print_sample("lbTie", trace_sample_lb_tie);
            print_sample("ubTie", trace_sample_ub_tie);
            print_sample("ub01", trace_sample_ub_binary_like);
            std::fprintf(stderr, "\n");
	        }

	        // ── Phase 2: Push Phase — loose tolerances for fast feasibility ──
        // The crash basis from IPM may need many pivots.  Allow cold-start
        // fallback: a one-time ~2s cold-start is far cheaper than losing
        // the simplex basis (which would cause dozens of cold starts + zero
        // cuts in the B&C tree).
        SimplexOptions push_opt;
        push_opt.max_iter = huge_warmstart_root
          ? std::min(std::max(sf_m / 2, 3000), 12000)
          : std::min(std::max(sf_m * 3, 5000), 50000);
        push_opt.feasibility_tol = 1e-6;   // Loose: prevent cycling
        push_opt.optimality_tol = 1e-6;    // Loose: fast convergence
        push_opt.verbose = false;
        push_opt.allow_cold_start = true;   // Fallback to cold-start if warm-start fails
        push_opt.use_partial_pricing = huge_warmstart_root;
        push_opt.factor_backend = simplex_factor_backend_from_id(opt.simplex_factor_backend);
        push_opt.lp_kernel_backend = opt.lp_kernel_backend;
        apply_simplex_budget(push_opt);
        apply_root_simplex_conformance_options(push_opt, require_simplex_crossover);

	        simplex = std::make_shared<SimplexResult>(
	            solve_lp_from_sf(sf, push_opt, &crash_basis));
        auto relax_t_push = std::chrono::steady_clock::now();

        // ── Validate crossover result: check scaled SF residual ──
        // If Ax - b has large residual, the crash basis failed. Fall back to cold-start.
        bool crossover_valid = false;
        if (simplex->result.stats.success && simplex->x_std.size() == sf_n) {
          Eigen::VectorXd sf_resid = sf.A * simplex->x_std - sf.b;
          double max_resid = std::max(sf_resid.maxCoeff(), -sf_resid.minCoeff());
          crossover_valid = (max_resid < 1e-4);
          if (opt.verbose || !crossover_valid) {
            fprintf(stderr, "[BC_RELAX] simplex without basis: path=ipm-crash iters=%d obj=%.2f sf_resid=%.2e valid=%d\n",
                    simplex->result.stats.iterations, simplex->result.stats.objective,
                    max_resid, crossover_valid ? 1 : 0);
          }
        }

        // ═══════════════════════════════════════════════════════════════════
        // BUG FIX: Do NOT attempt cold-start simplex on large LPs if crossover
        // fails. Cold-start on a 10k+ constraint LP may need 100k+ iterations
        // (hours of CPU time). Instead, return the IPM interior-point solution
        // directly — B&C will proceed without Gomory cuts but still with valid
        // bounds and branching.
        // ═══════════════════════════════════════════════════════════════════
        if (!crossover_valid && !require_simplex_crossover) {
          if (opt.verbose) {
            fprintf(stderr, "[BC_RELAX] crossover invalid on large LP (m=%d) — skipping cold-start, using IPM result\n",
                    sf_m);
          }
          // Return IPM result directly — no simplex basis, no Gomory cuts.
          if (lp_solution_feasible(lp, ipm_res, 1e-6)) {
            out.primal = ipm_res;
            out.dual_bound = ipm_res.stats.objective;
            return out;
          }
          // IPM result also infeasible (rare) — continue to fallback paths.
        }

        if (opt.verbose || sf_m > 10000) {
          fprintf(stderr, "[BC_RELAX] simplex without basis: path=ipm-crash iters=%d success=%d obj=%.2f ms=%.1f\n",
                  simplex->result.stats.iterations, simplex->result.stats.success ? 1 : 0,
                  simplex->result.stats.objective,
                  simplex->result.stats.runtime_sec * 1000.0);
        }

        // ── Phase 3: Cleanup — tight tolerances from Phase 2 basis ──
	        if (simplex->result.stats.success && crossover_valid) {
	          SimplexBasis phase2_basis = simplex->basis;
	          trace_basis_source("ipm_crash_phase2_basis", sf, phase2_basis,
	                             "source=phase2_cleanup_hint");
	          SimplexOptions cleanup_opt;
          cleanup_opt.max_iter = std::max(opt.max_lp_iter * 5, 2000);
          cleanup_opt.feasibility_tol = std::max(1e-10, opt.lp_tol * 0.1);
          cleanup_opt.optimality_tol = std::max(1e-10, opt.lp_tol * 0.1);
          cleanup_opt.verbose = false;
          cleanup_opt.allow_cold_start = false;
          cleanup_opt.use_partial_pricing = false; // Full pricing for consistent basis
          cleanup_opt.factor_backend = simplex_factor_backend_from_id(opt.simplex_factor_backend);
          cleanup_opt.lp_kernel_backend = opt.lp_kernel_backend;
          apply_simplex_budget(cleanup_opt);
          apply_root_simplex_conformance_options(cleanup_opt, require_simplex_crossover);

          auto cleanup_res = std::make_shared<SimplexResult>(
              solve_lp_from_sf(sf, cleanup_opt, &phase2_basis));
          auto relax_t_cleanup = std::chrono::steady_clock::now();
          if (sf_m > 10000) {
            fprintf(stderr, "[BC_RELAX] crossover cleanup: iters=%d success=%d obj=%.2f\n",
                    cleanup_res->result.stats.iterations, cleanup_res->result.stats.success ? 1 : 0,
                    cleanup_res->result.stats.objective);
          }
          if (trace_huge_root_relax) {
            fprintf(stderr,
                    "[BC_RELAX-TIMING] ipm=%.0f sf=%.0f scale=%.0f push=%.0f cleanup=%.0f total=%.0f\n",
                    std::chrono::duration<double, std::milli>(relax_t_ipm - relax_t0).count(),
                    std::chrono::duration<double, std::milli>(relax_t_sf - relax_t_ipm).count(),
                    std::chrono::duration<double, std::milli>(relax_t_scale - relax_t_sf).count(),
                    std::chrono::duration<double, std::milli>(relax_t_push - relax_t_scale).count(),
                    std::chrono::duration<double, std::milli>(relax_t_cleanup - relax_t_push).count(),
                    std::chrono::duration<double, std::milli>(relax_t_cleanup - relax_t0).count());
          }
          if (cleanup_res->result.stats.success) {
            simplex = cleanup_res;
          }
          // If cleanup fails, keep the Phase 2 result — still has a valid basis.
        }
      } else {
        // HiGHS-style logical-basis recovery: dual simplex from previous
        // basis when available (xpool warm-start), or cold-start from the
        // slack/artificial logical basis otherwise.
        // solve_lp_from_sf has built-in basis extension when basis_hint->rows
        // < sf_m (the dimension mismatch that occurs after cut rows are added).
        // Passing basis_hint activates PATH A sparse warm-start and restores
        // primal feasibility via dual simplex in ~N_cuts iterations.
        SimplexOptions sx_opt;
        sx_opt.max_iter = std::max(opt.max_lp_iter * 10, 2000);
        sx_opt.feasibility_tol = std::max(1e-10, opt.lp_tol * 0.1);
        sx_opt.optimality_tol = std::max(1e-10, opt.lp_tol * 0.1);
        sx_opt.verbose = false;
        sx_opt.allow_cold_start = true;  // Fall back to cold-start if warm fails
        sx_opt.use_partial_pricing = false;
        sx_opt.factor_backend = simplex_factor_backend_from_id(opt.simplex_factor_backend);
        sx_opt.lp_kernel_backend = opt.lp_kernel_backend;
        apply_simplex_budget(sx_opt);
        apply_root_simplex_conformance_options(sx_opt, require_simplex_crossover);
        if (lp_basis_trace_enabled()) {
          std::fprintf(stderr,
                       "[B&C-LPBASIS-RECOVER] stage=ipm_logical_recovery "
                       "m=%d n=%d nOrig=%d nSlack=%d reason=highs_logical_basis "
                       "hint_rows=%d hint_cols=%d\n",
                       sf_m, sf_n, sf.n_original, sf.n_slack,
                       basis_hint ? basis_hint->rows : -1,
                       basis_hint ? basis_hint->cols : -1);
        }
        simplex = std::make_shared<SimplexResult>(
            solve_lp_from_sf(sf, sx_opt, basis_hint));
      }

      // Store StandardFormLP for root Gomory cut generation.
      // NOTE: Use copy (not move) so sf remains valid for validation and diagnostics below.
      if (simplex->form.A.rows() == 0 && sf.A.rows() > 0) {
        simplex->form = sf;  // Copy, not move
      }

      // IPM-root with simplex nodes is a first-class handoff: accept only a
      // real simplex vertex/basis.  The relaxed tolerance remains only for
      // the legacy IPM-only fallback mode where no simplex state is required.
      const double base_feas_tol = require_simplex_crossover
          ? std::max(1e-8, opt.lp_tol * 10.0)
          : std::max(1e-3, opt.lp_tol * 1000.0);
      const double feas_tol = require_simplex_crossover
          ? base_feas_tol
          : ((n_lp > 10000) ? std::max(base_feas_tol, 1e-2)
             : (n_lp > 5000) ? std::max(base_feas_tol, 5e-3)
                              : base_feas_tol);
      bool accept_crossover = false;
      if (simplex->result.stats.success) {
        if (huge_warmstart_root && simplex->x_std.size() == sf_n) {
          Eigen::VectorXd sf_resid = sf.A * simplex->x_std - sf.b;
          double max_resid = std::max(sf_resid.maxCoeff(), -sf_resid.minCoeff());
          double obj_gap = std::abs(simplex->result.stats.objective - ipm_res.stats.objective);
          double obj_tol = std::max(1e-4, 1e-8 * std::max(1.0, std::abs(ipm_res.stats.objective)));
          accept_crossover = (max_resid < 1e-4 && obj_gap <= obj_tol);
        } else {
          accept_crossover = lp_solution_feasible(lp, simplex->result, feas_tol);
        }
      }
      if (accept_crossover) {
        if (audit_ipm_handoff) {
          trace_ipm_handoff_audit("crossover_accept", lp, ipm_res, &sf, simplex.get());
        }
        if (opt.verbose) {
          fprintf(stderr, "[BC_RELAX] IPM→simplex crossover OK: obj=%.4f\n",
                  simplex->result.stats.objective);
        }
        if (simplex->result.constraint_duals.size() != lp.A.rows() + lp.Aeq.rows() &&
            ipm_res.constraint_duals.size() == lp.A.rows() + lp.Aeq.rows()) {
          simplex->result.constraint_duals = ipm_res.constraint_duals;
        }
        if (simplex->result.box_dual_lb.size() != static_cast<int>(lp.vars.size()) &&
            ipm_res.box_dual_lb.size() == static_cast<int>(lp.vars.size())) {
          simplex->result.box_dual_lb = ipm_res.box_dual_lb;
        }
        if (simplex->result.box_dual_ub.size() != static_cast<int>(lp.vars.size()) &&
            ipm_res.box_dual_ub.size() == static_cast<int>(lp.vars.size())) {
          simplex->result.box_dual_ub = ipm_res.box_dual_ub;
        }
        out.simplex = simplex;
        out.primal = simplex->result;
        if (simplex->result.constraint_duals.size() == lp.A.rows() + lp.Aeq.rows()) {
          out.row_duals = simplex->result.constraint_duals;
        }
        out.dual_bound = out.primal.stats.objective;
        out.basis_hint = make_first_class_basis_hint_from_simplex(*simplex);
        return out;
      }
      if (audit_ipm_handoff) {
        trace_ipm_handoff_audit("crossover_reject", lp, ipm_res, &sf, simplex.get());
      }
      if (opt.verbose) {
        fprintf(stderr, "[BC_RELAX] IPM→simplex crossover FAILED: success=%d status='%s' — falling through to simplex\n",
                simplex->result.stats.success ? 1 : 0,
                simplex->result.stats.status.c_str());
        // Diagnostic: find worst violation in BOTH scaled SF and original space.
        if (simplex->result.stats.success && simplex->x_std.size() > 0) {
          // Check scaled SF residual: A_scaled * x_std - b_scaled
          // Use simplex->form (copy of sf)
          Eigen::VectorXd sf_resid = simplex->form.A * simplex->x_std - simplex->form.b;
          double worst_sf_ineq = sf_resid.maxCoeff();
          fprintf(stderr, "[BC_RELAX] SCALED SF residual: max=%.2e (should be ~0 if simplex correct)\n",
                  worst_sf_ineq);
        }
        // Check original space violation
        if (simplex->result.stats.success && simplex->result.x.size() == n_lp) {
          double worst_bnd = 0, worst_ineq = 0, worst_eq = 0;
          for (int i = 0; i < n_lp; ++i) {
            worst_bnd = std::max(worst_bnd, lp.vars[i].lb - simplex->result.x[i]);
            worst_bnd = std::max(worst_bnd, simplex->result.x[i] - lp.vars[i].ub);
          }
          // Use efficient matrix-vector multiplication
          if (lp.A.rows() > 0) {
            Eigen::VectorXd Ax = lp.A * simplex->result.x;
            for (int i = 0; i < static_cast<int>(lp.A.rows()); ++i) {
              worst_ineq = std::max(worst_ineq, Ax[i] - lp.b[i]);
            }
          }
          if (lp.Aeq.rows() > 0) {
            Eigen::VectorXd Aeq_x = lp.Aeq * simplex->result.x;
            for (int i = 0; i < static_cast<int>(lp.Aeq.rows()); ++i) {
              worst_eq = std::max(worst_eq, std::abs(Aeq_x[i] - lp.beq[i]));
            }
          }
          fprintf(stderr, "[BC_RELAX] crossover violations: bounds=%.2e ineq=%.2e eq=%.2e\n",
                  worst_bnd, worst_ineq, worst_eq);
        }
      }
      // Legacy IPM-only fallback only.  When simplex nodes are enabled, falling
      // through to the pure simplex path below is mandatory so the root always
      // publishes a valid basis/nonbasic side/reduced-cost state.
      if (!require_simplex_crossover && lp_solution_feasible(lp, ipm_res, 1e-6)) {
        if (opt.verbose) {
          fprintf(stderr, "[BC_RELAX] crossover failed — using raw IPM result (no basis)\n");
        }
        out.primal = ipm_res;
        out.dual_bound = ipm_res.stats.objective;
        // No simplex, no basis_hint — tree exploration uses IPMDiver.
        return out;
      }
    }
    // If IPM fails or crossover fails, continue to existing simplex paths.
  }

  if (opt.use_simplex_lp_nodes) {
    if (lp_budget_exhausted()) {
      out.primal.stats.success = false;
      out.primal.stats.status = "Time limit";
      return out;
    }
    SimplexOptions simplex_opt;
    // Cold-start solves (roots, repair re-solves) on large degenerate LPs can
    // need orders of magnitude more pivots than a warm node LP — the
    // unpresolved 118-bus relaxation takes ~1e5 pivots (~40 s), far above
    // max_lp_iter*10 = 5000, which turned a solvable root into a MaxIter
    // failure.  The wall-clock budget (apply_simplex_budget) is the real
    // governor for cold solves, so give them a generous iteration cap.
    const bool cold_root_solve = (basis_hint == nullptr);
    simplex_opt.max_iter =
        cold_root_solve ? std::max(opt.max_lp_iter * 10, 100000)
                        : std::max(opt.max_lp_iter * 10, 2000);
    simplex_opt.feasibility_tol = std::max(1e-10, opt.lp_tol * 0.1);
    simplex_opt.optimality_tol = std::max(1e-10, opt.lp_tol * 0.1);
    simplex_opt.verbose = false;
    simplex_opt.factor_backend = simplex_factor_backend_from_id(opt.simplex_factor_backend);
    simplex_opt.lp_kernel_backend = opt.lp_kernel_backend;
    apply_simplex_budget(simplex_opt);
    apply_root_simplex_conformance_options(simplex_opt, opt.use_simplex_lp_nodes);

    auto simplex = std::make_shared<SimplexResult>(solve_lp_with_basis(lp, simplex_opt, basis_hint));
    out.simplex = simplex;
    out.primal = simplex->result;
    if (!out.primal.stats.success) {
      if (lp_wall_budget > 0.0 && lp_wall_budget <= 10.0) {
        out.primal.stats.success = false;
        out.primal.stats.status = "Time limit";
        out.dual_bound = out.primal.stats.objective;
        return out;
      }
      if (lp_budget_exhausted()) {
        out.primal.stats.status = "Time limit";
        return out;
      }
      if (ipm_probe_diagnosed_failure) {
        // The probe already showed the IPM cannot handle this LP; a repair
        // attempt would burn the remaining budget on a doomed solve.
        if (opt.verbose) {
          fprintf(stderr,
                  "[BC_RELAX] Simplex failed and IPM probe was unhealthy — "
                  "skipping IPM-LP repair\n");
        }
        return out;
      }
      if (opt.verbose) {
        fprintf(stderr, "[BC_RELAX] Pure simplex FAILED: status='%s' — trying IPM-LP fallback\n",
                simplex->result.stats.status.c_str());
      }
      // HiGHS-style repair: when simplex fails, solve the LP with IPM only to
      // obtain a robust primal point, then cross it back to a simplex basis and
      // continue the MIP flow with simplex certificates.  Do not hand a raw
      // interior solution to the proof/frontier path when simplex nodes are
      // enabled.
      IPMLPOptions ipm_fb;
      ipm_fb.max_iter = std::max(opt.max_lp_iter, 200);
      ipm_fb.tol_primal = std::max(1e-9, opt.lp_tol * 0.1);
      ipm_fb.tol_dual   = std::max(1e-9, opt.lp_tol * 0.1);
      ipm_fb.tol_gap    = std::max(1e-9, opt.lp_tol * 0.1);
      ipm_fb.verbose = false;
      apply_ipm_budget(ipm_fb);
      NativeIPMLPAdapter ipm_fb_solver(ipm_fb);
      SolveResult ipm_fb_res = ipm_fb_solver.solve_lp(lp);
      if (ipm_fb_res.stats.status == "Time limit") {
        out.primal = std::move(ipm_fb_res);
        out.dual_bound = out.primal.stats.objective;
        return out;
      }
      if (ipm_fb_res.constraint_duals.size() == lp.A.rows() + lp.Aeq.rows()) {
        out.row_duals = ipm_fb_res.constraint_duals;
      }
      if (opt.verbose) {
        fprintf(stderr, "[BC_RELAX] IPM-LP fallback: success=%d status='%s' iter=%d obj=%.4f time=%.3fs\n",
                ipm_fb_res.stats.success ? 1 : 0,
                ipm_fb_res.stats.status.c_str(),
                ipm_fb_res.stats.iterations,
                ipm_fb_res.stats.objective,
                ipm_fb_res.stats.runtime_sec);
      }
      if (ipm_fb_res.stats.success) {
        const bool audit_ipm_handoff = ipm_handoff_audit_enabled(opt);
        if (audit_ipm_handoff) {
          trace_ipm_handoff_audit("simplex_repair_ipm_raw", lp, ipm_fb_res,
                                  nullptr, nullptr);
        }

        StandardFormLP repair_sf = build_standard_form_lp(lp);
        ruiz_scale_standard_form(repair_sf);
        if (audit_ipm_handoff) {
          trace_ipm_handoff_audit("simplex_repair_sf_scaled", lp,
                                  ipm_fb_res, &repair_sf, nullptr);
        }

        SimplexBasis repair_basis;
        auto repair_stats = recover_primal_activity_basis_impl(
            repair_sf, ipm_fb_res.x, &ipm_fb_res, basis_hint, repair_basis);
        if (opt.verbose || lp_basis_trace_enabled()) {
          fprintf(stderr,
                  "[BC_RELAX] IPM repair crossover seed: swaps=%d candRows=%d "
                  "screen=%d\n",
                  repair_stats.selected_swaps, repair_stats.candidate_rows,
                  repair_stats.passes_screen ? 1 : 0);
        }

        SimplexOptions repair_opt = simplex_opt;
        repair_opt.max_iter = std::max(opt.max_lp_iter * 20, 5000);
        repair_opt.allow_cold_start = true;
        repair_opt.use_partial_pricing = false;
        apply_simplex_budget(repair_opt);
        apply_root_simplex_conformance_options(repair_opt, true);

        auto repair_simplex = std::make_shared<SimplexResult>(
            solve_lp_from_sf(repair_sf, repair_opt, &repair_basis));
        if (repair_simplex->form.A.rows() == 0) {
          repair_simplex->form = repair_sf;
        }

        bool accept_repair = false;
        if (repair_simplex->result.stats.success &&
            repair_simplex->result.x.size() == n_lp) {
          const double obj_gap = std::abs(repair_simplex->result.stats.objective -
                                          ipm_fb_res.stats.objective);
          const double obj_tol = std::max(
              1e-4, 1e-8 * std::max(1.0, std::abs(ipm_fb_res.stats.objective)));
          accept_repair =
              lp_solution_feasible(lp, repair_simplex->result,
                                   std::max(1e-8, opt.lp_tol * 10.0)) &&
              obj_gap <= obj_tol;
        }
        if (audit_ipm_handoff) {
          trace_ipm_handoff_audit(accept_repair
                                      ? "simplex_repair_crossover_accept"
                                      : "simplex_repair_crossover_reject",
                                  lp, ipm_fb_res, &repair_sf,
                                  repair_simplex.get());
        }
        if (accept_repair) {
          out.simplex = repair_simplex;
          out.primal = repair_simplex->result;
          if (out.primal.constraint_duals.size() != lp.A.rows() + lp.Aeq.rows() &&
              ipm_fb_res.constraint_duals.size() == lp.A.rows() + lp.Aeq.rows()) {
            out.primal.constraint_duals = ipm_fb_res.constraint_duals;
          }
          out.dual_bound = out.primal.stats.objective;
          if (out.primal.constraint_duals.size() == lp.A.rows() + lp.Aeq.rows()) {
            out.row_duals = out.primal.constraint_duals;
          }
          out.basis_hint = make_first_class_basis_hint_from_simplex(*repair_simplex);
          return out;
        }

        if (!opt.use_simplex_lp_nodes &&
            lp_solution_feasible(lp, ipm_fb_res, 1e-6)) {
          out.primal = std::move(ipm_fb_res);
          out.dual_bound = out.primal.stats.objective;
          return out;
        }
      }
      // Both simplex and IPM-LP failed.
      return out;
    } else {
      out.dual_bound = out.primal.stats.objective;
      if (simplex->result.constraint_duals.size() == lp.A.rows() + lp.Aeq.rows()) {
        out.row_duals = simplex->result.constraint_duals;
      }
      out.basis_hint = make_first_class_basis_hint_from_simplex(*simplex);
      return out;
    }
  }

  // ═══════════════════════════════════════════════════════════════════════════
  // BUG FIX: Do NOT wrap LP as NLP for IPM solving. The std::function callback
  // overhead makes it 10-100x slower than direct sparse matrix operations.
  // Use NativeIPMLPAdapter (the LP-specific IPM) as a fallback instead.
  // ═══════════════════════════════════════════════════════════════════════════
  if (opt.verbose) {
    fprintf(stderr, "[BC_RELAX] Fallback: using NativeIPMLPAdapter (not NLP wrapper)\n");
  }
  
  IPMLPOptions ipm_opt;
  ipm_opt.max_iter = std::max(opt.max_lp_iter, 200);
  ipm_opt.tol_primal = std::max(1e-9, opt.lp_tol * 0.1);
  ipm_opt.tol_dual = std::max(1e-9, opt.lp_tol * 0.1);
  ipm_opt.tol_gap = std::max(1e-9, opt.lp_tol * 0.1);
  ipm_opt.verbose = opt.verbose;
  apply_ipm_budget(ipm_opt);
  
  NativeIPMLPAdapter ipm_solver(ipm_opt);
  out.primal = ipm_solver.solve_lp(lp);
  if (out.primal.constraint_duals.size() == lp.A.rows() + lp.Aeq.rows()) {
    out.row_duals = out.primal.constraint_duals;
  }
  
  if (opt.verbose) {
    fprintf(stderr, "[BC_RELAX] IPM-LP fallback: success=%d status='%s' iter=%d obj=%.4f time=%.3fs\n",
            out.primal.stats.success ? 1 : 0,
            out.primal.stats.status.c_str(),
            out.primal.stats.iterations,
            out.primal.stats.objective,
            out.primal.stats.runtime_sec);
  }
  
  if (out.primal.stats.success) {
    out.dual_bound = out.primal.stats.objective;
  }
  return out;
}

SolveResult solve_nlp_relaxation(const NLPModel& nlp_base,
                                 const Eigen::VectorXd& lb,
                                 const Eigen::VectorXd& ub,
                                 const Eigen::VectorXd* x0,
                                 const BCOptions& opt) {
  NLPModel nlp = nlp_base;
  apply_node_bounds(nlp.vars, lb, ub);

  if (x0 != nullptr && x0->size() == static_cast<int>(nlp.vars.size())) {
    nlp.x0 = *x0;
  } else if (nlp.x0.size() != static_cast<int>(nlp.vars.size())) {
    nlp.x0 = Eigen::VectorXd::Zero(static_cast<int>(nlp.vars.size()));
  }
  nlp.x0 = clamp_to_bounds(nlp.x0, lb, ub);

  IPMOptions ipm;
  ipm.max_iter = opt.max_lp_iter;
  ipm.tol_primal = opt.lp_tol;
  ipm.tol_dual = opt.lp_tol;
  ipm.tol_complementarity = std::max(1e-8, opt.lp_tol);
  ipm.alpha_max = 0.95;
  ipm.verbose = false;

  NativeIPMAdapter solver(ipm);
  return solver.solve_nlp(nlp);
}

bool try_rounding_heuristic(const LPModel& base_lp,
                            const Eigen::VectorXd& x_relax,
                            const Eigen::VectorXd& node_lb,
                            const Eigen::VectorXd& node_ub,
                            double int_tol,
                            bool& has_incumbent,
                            double& incumbent_obj,
                            Eigen::VectorXd& incumbent_x,
                            SolutionPool* sol_pool) {
  const int n = static_cast<int>(base_lp.vars.size());

  int frac_count = 0;
  for (int i = 0; i < n; ++i) {
    if (is_integer_type(base_lp.vars[i]) && !is_integral(x_relax[i], int_tol)) {
      ++frac_count;
    }
  }
  if (frac_count > 20) return false;

  // ═══════════════════════════════════════════════════════════════════════════
  // BUG FIX: Avoid deep-copying entire LPModel (including sparse matrices A, Aeq)
  // on every node. Instead, check feasibility directly against base_lp constraints
  // and node-local bounds.
  // ═══════════════════════════════════════════════════════════════════════════
  const double tol = kNodeFeasibilityTol;
  
  // Lambda to check solution feasibility without copying LPModel
  auto check_feasibility = [&](const Eigen::VectorXd& x) -> bool {
    // Check node-local variable bounds
    for (int i = 0; i < n; ++i) {
      if (x[i] < node_lb[i] - tol || x[i] > node_ub[i] + tol) {
        return false;
      }
      // Check integrality for integer variables
      if (is_integer_type(base_lp.vars[i]) && !is_integral(x[i], tol)) {
        return false;
      }
    }
    // Check inequality constraints: A*x <= b
    if (base_lp.A.rows() > 0) {
      Eigen::VectorXd Ax = base_lp.A * x;
      for (int i = 0; i < static_cast<int>(base_lp.A.rows()); ++i) {
        if (Ax[i] > base_lp.b[i] + tol) {
          return false;
        }
      }
    }
    // Check equality constraints: Aeq*x == beq
    if (base_lp.Aeq.rows() > 0) {
      Eigen::VectorXd Aeq_x = base_lp.Aeq * x;
      for (int i = 0; i < static_cast<int>(base_lp.Aeq.rows()); ++i) {
        if (std::abs(Aeq_x[i] - base_lp.beq[i]) > tol) {
          return false;
        }
      }
    }
    return true;
  };

  if (sol_pool && sol_pool->size() >= 3) {
    Eigen::VectorXd xg = sol_pool->guided_rounding(x_relax, base_lp.vars);
    xg = clamp_to_bounds(xg, node_lb, node_ub);
    if (check_feasibility(xg)) {
      const double obj = objective_value(base_lp.c, xg, base_lp.sense);
      if (!has_incumbent || obj < incumbent_obj) {
        has_incumbent = true;
        incumbent_x = xg;
        incumbent_obj = obj;
        sol_pool->add(xg, obj);
        return true;
      }
    }
  }

  const Eigen::VectorXd xr = project_integer_solution(base_lp.vars, x_relax, node_lb, node_ub);

  if (!check_feasibility(xr)) return false;

  const double obj = objective_value(base_lp.c, xr, base_lp.sense);
  if (!has_incumbent || obj < incumbent_obj) {
    has_incumbent = true;
    incumbent_x = xr;
    incumbent_obj = obj;
    if (sol_pool) sol_pool->add(xr, obj);
    return true;
  }
  return false;
}

}  // namespace mipsolvers::engine::detail
