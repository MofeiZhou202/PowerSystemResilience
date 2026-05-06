#include "hacdcpf/engine/kernel/linear_algebra/linear_solver.hpp"

#ifdef HACDCPF_HAVE_UMFPACK
#include <Eigen/UmfPackSupport>
#endif
#ifdef HACDCPF_HAVE_KLU
#include <Eigen/KLUSupport>
#endif

namespace hacdcpf::engine {

const char* EigenSparseLUSolver::backend_name() const {
  return "EigenSparseLU";
}

void EigenSparseLUSolver::analyze_pattern(const Eigen::SparseMatrix<double>& a) {
  solver_.analyzePattern(a);
}

bool EigenSparseLUSolver::factorize(const Eigen::SparseMatrix<double>& a) {
  solver_.factorize(a);
  return solver_.info() == Eigen::Success;
}

bool EigenSparseLUSolver::solve(const Eigen::VectorXd& rhs, Eigen::VectorXd& x) {
  x = solver_.solve(rhs);
  return solver_.info() == Eigen::Success;
}

#ifdef HACDCPF_HAVE_UMFPACK
class EigenUmfPackSolver::Impl {
 public:
  Eigen::UmfPackLU<Eigen::SparseMatrix<double>> solver;
};

const char* EigenUmfPackSolver::backend_name() const {
  return "SuiteSparse-UMFPACK(Eigen)";
}

void EigenUmfPackSolver::analyze_pattern(const Eigen::SparseMatrix<double>& a) {
  if (!impl_) impl_ = std::make_unique<Impl>();
  impl_->solver.analyzePattern(a);
}

bool EigenUmfPackSolver::factorize(const Eigen::SparseMatrix<double>& a) {
  if (!impl_) impl_ = std::make_unique<Impl>();
  impl_->solver.factorize(a);
  return impl_->solver.info() == Eigen::Success;
}

bool EigenUmfPackSolver::solve(const Eigen::VectorXd& rhs, Eigen::VectorXd& x) {
  if (!impl_) return false;
  x = impl_->solver.solve(rhs);
  return impl_->solver.info() == Eigen::Success;
}
#endif

#ifdef HACDCPF_HAVE_KLU
class EigenKluSolver::Impl {
 public:
  Eigen::KLU<Eigen::SparseMatrix<double>> solver;
};

const char* EigenKluSolver::backend_name() const {
  return "SuiteSparse-KLU(Eigen)";
}

void EigenKluSolver::analyze_pattern(const Eigen::SparseMatrix<double>& a) {
  if (!impl_) impl_ = std::make_unique<Impl>();
  impl_->solver.analyzePattern(a);
}

bool EigenKluSolver::factorize(const Eigen::SparseMatrix<double>& a) {
  if (!impl_) impl_ = std::make_unique<Impl>();
  impl_->solver.factorize(a);
  return impl_->solver.info() == Eigen::Success;
}

bool EigenKluSolver::solve(const Eigen::VectorXd& rhs, Eigen::VectorXd& x) {
  if (!impl_) return false;
  x = impl_->solver.solve(rhs);
  return impl_->solver.info() == Eigen::Success;
}
#endif

std::unique_ptr<SparseLinearSolver> make_default_sparse_solver() {
#ifdef HACDCPF_HAVE_KLU
  return std::make_unique<EigenKluSolver>();
#elif defined(HACDCPF_HAVE_UMFPACK)
  return std::make_unique<EigenUmfPackSolver>();
#else
  return std::make_unique<EigenSparseLUSolver>();
#endif
}

}  // namespace hacdcpf::engine
