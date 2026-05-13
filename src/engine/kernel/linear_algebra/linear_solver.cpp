#include "mipsolvers/engine/kernel/linear_algebra/linear_solver.hpp"

#ifdef HACDCPF_HAVE_UMFPACK
#include <Eigen/UmfPackSupport>
#endif
#ifdef HACDCPF_HAVE_KLU
#include <Eigen/KLUSupport>
#endif
#ifdef HACDCPF_HAVE_SUPERLU
#include <Eigen/SuperLUSupport>
#endif
#ifdef HACDCPF_HAVE_MKL_PARDISO
#include <Eigen/PardisoSupport>
#endif

namespace mipsolvers::engine {

namespace {

bool is_empty_square_system(const Eigen::SparseMatrix<double>& a) {
  return a.rows() == 0 && a.cols() == 0;
}

}  // namespace

const char* EigenSparseLUSolver::backend_name() const {
  return "EigenSparseLU";
}

void EigenSparseLUSolver::analyze_pattern(const Eigen::SparseMatrix<double>& a) {
  empty_system_ = is_empty_square_system(a);
  if (empty_system_) return;
  solver_.analyzePattern(a);
  analysis_done_ = true;
}

bool EigenSparseLUSolver::factorize(const Eigen::SparseMatrix<double>& a) {
  empty_system_ = is_empty_square_system(a);
  if (empty_system_) return true;
  if (!analysis_done_) {
    // analyze_pattern() was not called; run it now so that
    // Eigen::SparseLU::m_analysisIsOk is set before factorize() is called.
    solver_.analyzePattern(a);
    analysis_done_ = true;
  }
  solver_.factorize(a);
  return solver_.info() == Eigen::Success;
}

bool EigenSparseLUSolver::solve(const Eigen::VectorXd& rhs, Eigen::VectorXd& x) {
  if (empty_system_) {
    if (rhs.size() != 0) return false;
    x.resize(0);
    return true;
  }
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
  empty_system_ = is_empty_square_system(a);
  if (empty_system_) return;
  if (!impl_) impl_ = std::make_unique<Impl>();
  impl_->solver.analyzePattern(a);
}

bool EigenUmfPackSolver::factorize(const Eigen::SparseMatrix<double>& a) {
  empty_system_ = is_empty_square_system(a);
  if (empty_system_) return true;
  const bool needs_analyze = !impl_;
  if (!impl_) impl_ = std::make_unique<Impl>();
  if (needs_analyze) {
    // analyze_pattern() was not called; run it now so that UMFPACK
    // has a symbolic factorization before the numerical step.
    impl_->solver.analyzePattern(a);
  }
  impl_->solver.factorize(a);
  return impl_->solver.info() == Eigen::Success;
}

bool EigenUmfPackSolver::solve(const Eigen::VectorXd& rhs, Eigen::VectorXd& x) {
  if (empty_system_) {
    if (rhs.size() != 0) return false;
    x.resize(0);
    return true;
  }
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
  empty_system_ = is_empty_square_system(a);
  if (empty_system_) return;
  if (!impl_) impl_ = std::make_unique<Impl>();
  impl_->solver.analyzePattern(a);
}

bool EigenKluSolver::factorize(const Eigen::SparseMatrix<double>& a) {
  empty_system_ = is_empty_square_system(a);
  if (empty_system_) return true;
  const bool needs_analyze = !impl_;
  if (!impl_) impl_ = std::make_unique<Impl>();
  if (needs_analyze) {
    // analyze_pattern() was not called; run it now so that KLU
    // has a symbolic factorization before the numerical step.
    impl_->solver.analyzePattern(a);
  }
  impl_->solver.factorize(a);
  return impl_->solver.info() == Eigen::Success;
}

bool EigenKluSolver::solve(const Eigen::VectorXd& rhs, Eigen::VectorXd& x) {
  if (empty_system_) {
    if (rhs.size() != 0) return false;
    x.resize(0);
    return true;
  }
  if (!impl_) return false;
  x = impl_->solver.solve(rhs);
  return impl_->solver.info() == Eigen::Success;
}
#endif

#ifdef HACDCPF_HAVE_SUPERLU
class SuperLUSolver::Impl {
 public:
  Eigen::SuperLU<Eigen::SparseMatrix<double>> solver;
};

const char* SuperLUSolver::backend_name() const {
  return "SuperLU(Eigen)";
}

void SuperLUSolver::analyze_pattern(const Eigen::SparseMatrix<double>& a) {
  empty_system_ = is_empty_square_system(a);
  if (empty_system_) return;
  if (!impl_) impl_ = std::make_unique<Impl>();
  impl_->solver.analyzePattern(a);
}

bool SuperLUSolver::factorize(const Eigen::SparseMatrix<double>& a) {
  empty_system_ = is_empty_square_system(a);
  if (empty_system_) return true;
  if (!impl_) {
    // analyze_pattern() was not called; run it now so that
    // Eigen::SuperLU::m_analysisIsOk is set before factorize() is called.
    impl_ = std::make_unique<Impl>();
    impl_->solver.analyzePattern(a);
  }
  impl_->solver.factorize(a);
  return impl_->solver.info() == Eigen::Success;
}

bool SuperLUSolver::solve(const Eigen::VectorXd& rhs, Eigen::VectorXd& x) {
  if (empty_system_) {
    if (rhs.size() != 0) return false;
    x.resize(0);
    return true;
  }
  if (!impl_) return false;
  x = impl_->solver.solve(rhs);
  return impl_->solver.info() == Eigen::Success;
}
#endif

#ifdef HACDCPF_HAVE_MKL_PARDISO
class MKLPardisoSolver::Impl {
 public:
  Eigen::PardisoLU<Eigen::SparseMatrix<double>> solver;
};

const char* MKLPardisoSolver::backend_name() const {
  return "Intel-MKL-PARDISO(Eigen)";
}

void MKLPardisoSolver::analyze_pattern(const Eigen::SparseMatrix<double>& a) {
  empty_system_ = is_empty_square_system(a);
  if (empty_system_) return;
  if (!impl_) impl_ = std::make_unique<Impl>();
  impl_->solver.analyzePattern(a);
}

bool MKLPardisoSolver::factorize(const Eigen::SparseMatrix<double>& a) {
  empty_system_ = is_empty_square_system(a);
  if (empty_system_) return true;
  if (!impl_) {
    // analyze_pattern() was not called; run it now so that
    // Eigen::PardisoLU::m_analysisIsOk is set before factorize() is called.
    impl_ = std::make_unique<Impl>();
    impl_->solver.analyzePattern(a);
  }
  impl_->solver.factorize(a);
  return impl_->solver.info() == Eigen::Success;
}

bool MKLPardisoSolver::solve(const Eigen::VectorXd& rhs, Eigen::VectorXd& x) {
  if (empty_system_) {
    if (rhs.size() != 0) return false;
    x.resize(0);
    return true;
  }
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
#elif defined(HACDCPF_HAVE_MKL_PARDISO)
  return std::make_unique<MKLPardisoSolver>();
#elif defined(HACDCPF_HAVE_SUPERLU)
  return std::make_unique<SuperLUSolver>();
#else
  return std::make_unique<EigenSparseLUSolver>();
#endif
}

}  // namespace mipsolvers::engine
