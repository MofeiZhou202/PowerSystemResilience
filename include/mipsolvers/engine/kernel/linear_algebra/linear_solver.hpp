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

std::unique_ptr<SparseLinearSolver> make_default_sparse_solver();

}  // namespace mipsolvers::engine
