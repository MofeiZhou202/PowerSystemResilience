#pragma once

#include <memory>

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
  virtual bool solve(const Eigen::VectorXd& rhs, Eigen::VectorXd& x) = 0;
  /// Factor inertia when exposed by a symmetric-indefinite backend.
  virtual int negative_eigenvalues() const { return -1; }
  virtual int estimated_deficiency() const { return -1; }
};

class EigenSparseLUSolver final : public SparseLinearSolver {
 public:
  const char* backend_name() const override;
  void analyze_pattern(const Eigen::SparseMatrix<double>& a) override;
  bool factorize(const Eigen::SparseMatrix<double>& a) override;
  bool solve(const Eigen::VectorXd& rhs, Eigen::VectorXd& x) override;

 private:
  Eigen::SparseLU<Eigen::SparseMatrix<double>> solver_;
  bool empty_system_{false};
  bool analysis_done_{false};
};

#ifdef HACDCPF_HAVE_UMFPACK
class EigenUmfPackSolver final : public SparseLinearSolver {
 public:
  ~EigenUmfPackSolver() override;
  const char* backend_name() const override;
  void analyze_pattern(const Eigen::SparseMatrix<double>& a) override;
  bool factorize(const Eigen::SparseMatrix<double>& a) override;
  bool solve(const Eigen::VectorXd& rhs, Eigen::VectorXd& x) override;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
  bool empty_system_{false};
};
#endif

#ifdef HACDCPF_HAVE_KLU
class EigenKluSolver final : public SparseLinearSolver {
 public:
  ~EigenKluSolver() override;
  const char* backend_name() const override;
  void analyze_pattern(const Eigen::SparseMatrix<double>& a) override;
  bool factorize(const Eigen::SparseMatrix<double>& a) override;
  bool solve(const Eigen::VectorXd& rhs, Eigen::VectorXd& x) override;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
  bool empty_system_{false};
};
#endif

#ifdef HACDCPF_HAVE_SUPERLU
class SuperLUSolver final : public SparseLinearSolver {
 public:
  ~SuperLUSolver() override;
  const char* backend_name() const override;
  void analyze_pattern(const Eigen::SparseMatrix<double>& a) override;
  bool factorize(const Eigen::SparseMatrix<double>& a) override;
  bool solve(const Eigen::VectorXd& rhs, Eigen::VectorXd& x) override;

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
