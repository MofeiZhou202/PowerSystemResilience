#pragma once

#include <memory>
#include <cstdint>

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/SparseLU>

namespace mipsolvers::engine {

/// Abstract sparse linear solver interface (self-contained in solver module).
class SparseLinearSolver {
 public:
  virtual ~SparseLinearSolver() = default;
  virtual const char* backend_name() const = 0;
  virtual void analyze_pattern(const Eigen::SparseMatrix<double>& a) = 0;
  virtual bool factorize(const Eigen::SparseMatrix<double>& a) = 0;
  virtual bool supports_numeric_refactor() const { return false; }
  virtual bool refactorize(const Eigen::SparseMatrix<double>& a) {
    return factorize(a);
  }
  virtual bool solve(const Eigen::VectorXd& rhs, Eigen::VectorXd& x) = 0;
  /// Number of pivots perturbed by the last numerical factorization.
  virtual int perturbed_pivots() const { return -1; }
  /// Backend-reported symbolic factor nonzeros and factorization work.
  virtual std::int64_t factor_nonzeros() const { return -1; }
  virtual std::int64_t factor_work() const { return -1; }
  /// Factor inertia when exposed by a symmetric-indefinite backend.
  virtual int negative_eigenvalues() const { return -1; }
  virtual int estimated_deficiency() const { return -1; }
  /// Solve multiple right-hand sides against the current factorization.
  /// The default preserves source compatibility for third-party backends by
  /// dispatching one column at a time; native backends override this method.
  virtual bool solve_many(const Eigen::MatrixXd& rhs, Eigen::MatrixXd& x);
};

class EigenSparseLUSolver final : public SparseLinearSolver {
 public:
  const char* backend_name() const override;
  void analyze_pattern(const Eigen::SparseMatrix<double>& a) override;
  bool factorize(const Eigen::SparseMatrix<double>& a) override;
  bool solve(const Eigen::VectorXd& rhs, Eigen::VectorXd& x) override;
  bool solve_many(const Eigen::MatrixXd& rhs, Eigen::MatrixXd& x) override;

 private:
  Eigen::SparseLU<Eigen::SparseMatrix<double>> solver_;
  bool empty_system_{false};
  bool analysis_done_{false};
};

#ifdef HACDCPF_HAVE_UMFPACK
class EigenUmfPackSolver final : public SparseLinearSolver {
 public:
  EigenUmfPackSolver();
  ~EigenUmfPackSolver() override;
  const char* backend_name() const override;
  void analyze_pattern(const Eigen::SparseMatrix<double>& a) override;
  bool factorize(const Eigen::SparseMatrix<double>& a) override;
  bool solve(const Eigen::VectorXd& rhs, Eigen::VectorXd& x) override;
  bool solve_many(const Eigen::MatrixXd& rhs, Eigen::MatrixXd& x) override;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
  bool empty_system_{false};
};
#endif

#ifdef HACDCPF_HAVE_KLU
class EigenKluSolver final : public SparseLinearSolver {
 public:
  EigenKluSolver();
  ~EigenKluSolver() override;
  const char* backend_name() const override;
  void analyze_pattern(const Eigen::SparseMatrix<double>& a) override;
  bool factorize(const Eigen::SparseMatrix<double>& a) override;
  bool supports_numeric_refactor() const override { return true; }
  bool refactorize(const Eigen::SparseMatrix<double>& a) override;
  bool solve(const Eigen::VectorXd& rhs, Eigen::VectorXd& x) override;
  bool solve_many(const Eigen::MatrixXd& rhs, Eigen::MatrixXd& x) override;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
  bool empty_system_{false};
};
#endif

#ifdef HACDCPF_HAVE_SUPERLU
class SuperLUSolver final : public SparseLinearSolver {
 public:
  SuperLUSolver();
  ~SuperLUSolver() override;
  const char* backend_name() const override;
  void analyze_pattern(const Eigen::SparseMatrix<double>& a) override;
  bool factorize(const Eigen::SparseMatrix<double>& a) override;
  bool solve(const Eigen::VectorXd& rhs, Eigen::VectorXd& x) override;
  bool solve_many(const Eigen::MatrixXd& rhs, Eigen::MatrixXd& x) override;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
  bool empty_system_{false};
};
#endif

#ifdef HACDCPF_HAVE_MKL_PARDISO
class MKLPardisoSolver final : public SparseLinearSolver {
 public:
  MKLPardisoSolver();
  ~MKLPardisoSolver() override;
  const char* backend_name() const override;
  void analyze_pattern(const Eigen::SparseMatrix<double>& a) override;
  bool factorize(const Eigen::SparseMatrix<double>& a) override;
  bool solve(const Eigen::VectorXd& rhs, Eigen::VectorXd& x) override;
  bool solve_many(const Eigen::MatrixXd& rhs, Eigen::MatrixXd& x) override;
  int perturbed_pivots() const override;
  std::int64_t factor_nonzeros() const override;
  std::int64_t factor_work() const override;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
  bool empty_system_{false};
};

/// MKL PARDISO sparse Cholesky backend for symmetric positive-definite
/// normal equations. The lower triangle is authoritative.
class MKLPardisoLLTSolver final : public SparseLinearSolver {
 public:
  MKLPardisoLLTSolver();
  ~MKLPardisoLLTSolver() override;
  const char* backend_name() const override;
  void analyze_pattern(const Eigen::SparseMatrix<double>& a) override;
  bool factorize(const Eigen::SparseMatrix<double>& a) override;
  bool solve(const Eigen::VectorXd& rhs, Eigen::VectorXd& x) override;
  bool solve_many(const Eigen::MatrixXd& rhs, Eigen::MatrixXd& x) override;
  int perturbed_pivots() const override;
  std::int64_t factor_nonzeros() const override;
  std::int64_t factor_work() const override;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
  bool empty_system_{false};
};

/// MKL PARDISO symmetric-indefinite LDL^T backend for augmented KKT systems.
/// The lower triangle is authoritative; callers may still provide a full
/// symmetric matrix because Eigen's PardisoLDLT wrapper selects that triangle.
class MKLPardisoLDLTSolver final : public SparseLinearSolver {
 public:
  MKLPardisoLDLTSolver();
  ~MKLPardisoLDLTSolver() override;
  const char* backend_name() const override;
  void analyze_pattern(const Eigen::SparseMatrix<double>& a) override;
  bool factorize(const Eigen::SparseMatrix<double>& a) override;
  bool solve(const Eigen::VectorXd& rhs, Eigen::VectorXd& x) override;
  bool solve_many(const Eigen::MatrixXd& rhs, Eigen::MatrixXd& x) override;
  int perturbed_pivots() const override;
  std::int64_t factor_nonzeros() const override;
  std::int64_t factor_work() const override;
  int negative_eigenvalues() const override;
  int estimated_deficiency() const override;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
  bool empty_system_{false};
};

/// One-shot numerical portfolio for symmetric-indefinite KKT matrices.
/// The first factorization/solve is evaluated with both PARDISO LU and LDLT;
/// subsequent factorizations reuse the fastest backward-stable backend.
class MKLPardisoAdaptiveSolver final : public SparseLinearSolver {
 public:
  MKLPardisoAdaptiveSolver();
  ~MKLPardisoAdaptiveSolver() override;
  const char* backend_name() const override;
  void analyze_pattern(const Eigen::SparseMatrix<double>& a) override;
  bool factorize(const Eigen::SparseMatrix<double>& a) override;
  bool solve(const Eigen::VectorXd& rhs, Eigen::VectorXd& x) override;
  int perturbed_pivots() const override;
  std::int64_t factor_nonzeros() const override;
  std::int64_t factor_work() const override;
 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
  bool empty_system_{false};
};
#endif

#ifdef HACDCPF_HAVE_MUMPS
/// MUMPS multifrontal direct solver (Bunch–Kaufman LDLᵀ) for symmetric
/// indefinite systems — the open-source counterpart to MA57/Pardiso for the
/// KKT/Newton systems in the IPM paths.  Symmetric-indefinite factorization
/// exploits symmetry (≈half the fill and flops of a generic unsymmetric LU)
/// and uses nested-dissection ordering, which is near-optimal for grid-type
/// graphs (power networks, meshes).  The analyze/factorize/solve contract
/// matches the IPM's "analyze once, factorize many" pattern: the sparsity
/// pattern is analyzed once (lower triangle is extracted then), the numeric
/// values are refilled per iteration.
class MumpsSolver final : public SparseLinearSolver {
 public:
  MumpsSolver();
  ~MumpsSolver() override;
  const char* backend_name() const override;
  void analyze_pattern(const Eigen::SparseMatrix<double>& a) override;
  bool factorize(const Eigen::SparseMatrix<double>& a) override;
  bool solve(const Eigen::VectorXd& rhs, Eigen::VectorXd& x) override;
  bool solve_many(const Eigen::MatrixXd& rhs, Eigen::MatrixXd& x) override;

  /// Number of negative eigenvalues of the factored matrix — the negative
  /// pivots of the LDLᵀ factorization (INFOG(12)), which equal the matrix
  /// inertia by Sylvester's congruence (K = LDLᵀ ⇒ inertia(K) = inertia(D)).
  /// Free with the factorization; this is what the Wächter–Biegler δ_W
  /// inertia-correction loop queries each IPM iteration.  Returns -1 when no
  /// factorization has run yet.
  int negative_eigenvalues() const override;

  /// Estimated numerical rank deficiency from INFOG(28). Returns -1 before
  /// numeric factorization. A trustworthy nonsingular factor has value zero.
  int estimated_deficiency() const override;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
  bool empty_system_{false};
};
#endif

std::unique_ptr<SparseLinearSolver> make_default_sparse_solver();

}  // namespace mipsolvers::engine
