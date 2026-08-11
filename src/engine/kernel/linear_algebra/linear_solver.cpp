#include "mipsolvers/engine/kernel/linear_algebra/linear_solver.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <vector>

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
#include <mkl_service.h>
#endif

namespace mipsolvers::engine {

int mkl_max_threads() {
#ifdef HACDCPF_HAVE_MKL_PARDISO
  return mkl_get_max_threads();
#else
  return 1;
#endif
}

void set_mkl_num_threads(int threads) {
#ifdef HACDCPF_HAVE_MKL_PARDISO
  mkl_set_num_threads(threads);
#else
  (void)threads;
#endif
}

namespace {

bool is_empty_square_system(const Eigen::SparseMatrix<double>& a) {
  return a.rows() == 0 && a.cols() == 0;
}

template <typename Solver>
bool solve_many_with_eigen(Solver& solver, bool empty_system,
                           const Eigen::MatrixXd& rhs, Eigen::MatrixXd& x) {
  if (empty_system) {
    if (rhs.rows() != 0) return false;
    x.resize(0, rhs.cols());
    return true;
  }
  if (rhs.rows() != solver.rows() || !rhs.allFinite()) return false;
  if (rhs.cols() == 0) {
    x.resize(rhs.rows(), 0);
    return true;
  }
  x = solver.solve(rhs);
  return solver.info() == Eigen::Success && x.rows() == rhs.rows() &&
         x.cols() == rhs.cols() && x.allFinite();
}

}  // namespace

bool SparseLinearSolver::solve_many(const Eigen::MatrixXd& rhs,
                                    Eigen::MatrixXd& x) {
  if (!rhs.allFinite()) return false;
  x.resize(rhs.rows(), rhs.cols());
  for (Eigen::Index col = 0; col < rhs.cols(); ++col) {
    Eigen::VectorXd solution;
    if (!solve(rhs.col(col), solution) || solution.size() != rhs.rows() ||
        !solution.allFinite()) {
      return false;
    }
    x.col(col) = solution;
  }
  return true;
}

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

bool EigenSparseLUSolver::solve_many(const Eigen::MatrixXd& rhs,
                                     Eigen::MatrixXd& x) {
  return solve_many_with_eigen(solver_, empty_system_, rhs, x);
}

#ifdef HACDCPF_HAVE_UMFPACK
class EigenUmfPackSolver::Impl {
 public:
  Eigen::UmfPackLU<Eigen::SparseMatrix<double>> solver;
};

EigenUmfPackSolver::EigenUmfPackSolver() = default;
EigenUmfPackSolver::~EigenUmfPackSolver() = default;

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

bool EigenUmfPackSolver::solve_many(const Eigen::MatrixXd& rhs,
                                    Eigen::MatrixXd& x) {
  if (!impl_ && !empty_system_) return false;
  if (empty_system_) {
    if (rhs.rows() != 0) return false;
    x.resize(0, rhs.cols());
    return true;
  }
  return solve_many_with_eigen(impl_->solver, false, rhs, x);
}
#endif

#ifdef HACDCPF_HAVE_KLU
class EigenKluSolver::Impl {
 public:
  Eigen::KLU<Eigen::SparseMatrix<double>> solver;
};

EigenKluSolver::EigenKluSolver() = default;
EigenKluSolver::~EigenKluSolver() = default;

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

bool EigenKluSolver::solve_many(const Eigen::MatrixXd& rhs,
                                Eigen::MatrixXd& x) {
  if (!impl_ && !empty_system_) return false;
  if (empty_system_) {
    if (rhs.rows() != 0) return false;
    x.resize(0, rhs.cols());
    return true;
  }
  return solve_many_with_eigen(impl_->solver, false, rhs, x);
}
#endif

#ifdef HACDCPF_HAVE_SUPERLU
class SuperLUSolver::Impl {
 public:
  Eigen::SuperLU<Eigen::SparseMatrix<double>> solver;
};

SuperLUSolver::SuperLUSolver() = default;
SuperLUSolver::~SuperLUSolver() = default;

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

bool SuperLUSolver::solve_many(const Eigen::MatrixXd& rhs,
                               Eigen::MatrixXd& x) {
  if (!impl_ && !empty_system_) return false;
  if (empty_system_) {
    if (rhs.rows() != 0) return false;
    x.resize(0, rhs.cols());
    return true;
  }
  return solve_many_with_eigen(impl_->solver, false, rhs, x);
}
#endif

#ifdef HACDCPF_HAVE_MKL_PARDISO
class MKLPardisoSolver::Impl {
 public:
  Eigen::PardisoLU<Eigen::SparseMatrix<double>> solver;
};

MKLPardisoSolver::MKLPardisoSolver() = default;
MKLPardisoSolver::~MKLPardisoSolver() = default;

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

bool MKLPardisoSolver::solve_many(const Eigen::MatrixXd& rhs,
                                  Eigen::MatrixXd& x) {
  if (!impl_ && !empty_system_) return false;
  if (empty_system_) {
    if (rhs.rows() != 0) return false;
    x.resize(0, rhs.cols());
    return true;
  }
  return solve_many_with_eigen(impl_->solver, false, rhs, x);
}

int MKLPardisoSolver::perturbed_pivots() const {
  return impl_ ? static_cast<int>(impl_->solver.pardisoParameterArray()[13])
               : -1;
}

std::int64_t MKLPardisoSolver::factor_nonzeros() const {
  return impl_ ? static_cast<std::int64_t>(
                     impl_->solver.pardisoParameterArray()[17])
               : -1;
}

std::int64_t MKLPardisoSolver::factor_work() const {
  return impl_ ? static_cast<std::int64_t>(
                     impl_->solver.pardisoParameterArray()[18])
               : -1;
}

class MKLPardisoLLTSolver::Impl {
 public:
  Eigen::PardisoLLT<Eigen::SparseMatrix<double>, Eigen::Lower> solver;
};

MKLPardisoLLTSolver::MKLPardisoLLTSolver() = default;
MKLPardisoLLTSolver::~MKLPardisoLLTSolver() = default;

const char* MKLPardisoLLTSolver::backend_name() const {
  return "Intel-MKL-PARDISO-LLT(Eigen)";
}

void MKLPardisoLLTSolver::analyze_pattern(
    const Eigen::SparseMatrix<double>& a) {
  empty_system_ = is_empty_square_system(a);
  if (empty_system_) return;
  if (!impl_) impl_ = std::make_unique<Impl>();
  impl_->solver.analyzePattern(a);
}

bool MKLPardisoLLTSolver::factorize(
    const Eigen::SparseMatrix<double>& a) {
  empty_system_ = is_empty_square_system(a);
  if (empty_system_) return true;
  if (!impl_) {
    impl_ = std::make_unique<Impl>();
    impl_->solver.analyzePattern(a);
  }
  impl_->solver.factorize(a);
  return impl_->solver.info() == Eigen::Success;
}

bool MKLPardisoLLTSolver::solve(const Eigen::VectorXd& rhs,
                                Eigen::VectorXd& x) {
  if (empty_system_) {
    if (rhs.size() != 0) return false;
    x.resize(0);
    return true;
  }
  if (!impl_) return false;
  x = impl_->solver.solve(rhs);
  return impl_->solver.info() == Eigen::Success && x.allFinite();
}

bool MKLPardisoLLTSolver::solve_many(const Eigen::MatrixXd& rhs,
                                     Eigen::MatrixXd& x) {
  if (!impl_ && !empty_system_) return false;
  if (empty_system_) {
    if (rhs.rows() != 0) return false;
    x.resize(0, rhs.cols());
    return true;
  }
  return solve_many_with_eigen(impl_->solver, false, rhs, x);
}

int MKLPardisoLLTSolver::perturbed_pivots() const {
  return impl_ ? static_cast<int>(impl_->solver.pardisoParameterArray()[13])
               : -1;
}

std::int64_t MKLPardisoLLTSolver::factor_nonzeros() const {
  return impl_ ? static_cast<std::int64_t>(
                     impl_->solver.pardisoParameterArray()[17])
               : -1;
}

std::int64_t MKLPardisoLLTSolver::factor_work() const {
  return impl_ ? static_cast<std::int64_t>(
                     impl_->solver.pardisoParameterArray()[18])
               : -1;
}

class MKLPardisoLDLTSolver::Impl {
 public:
  Eigen::PardisoLDLT<Eigen::SparseMatrix<double>, Eigen::Lower> solver;
};

MKLPardisoLDLTSolver::MKLPardisoLDLTSolver() = default;
MKLPardisoLDLTSolver::~MKLPardisoLDLTSolver() = default;

const char* MKLPardisoLDLTSolver::backend_name() const {
  return "Intel-MKL-PARDISO-LDLT(Eigen)";
}

void MKLPardisoLDLTSolver::analyze_pattern(
    const Eigen::SparseMatrix<double>& a) {
  empty_system_ = is_empty_square_system(a);
  if (empty_system_) return;
  if (!impl_) impl_ = std::make_unique<Impl>();
  impl_->solver.analyzePattern(a);
}

bool MKLPardisoLDLTSolver::factorize(
    const Eigen::SparseMatrix<double>& a) {
  empty_system_ = is_empty_square_system(a);
  if (empty_system_) return true;
  if (!impl_) {
    impl_ = std::make_unique<Impl>();
    impl_->solver.analyzePattern(a);
  }
  impl_->solver.factorize(a);
  return impl_->solver.info() == Eigen::Success;
}

bool MKLPardisoLDLTSolver::solve(const Eigen::VectorXd& rhs,
                                 Eigen::VectorXd& x) {
  if (empty_system_) {
    if (rhs.size() != 0) return false;
    x.resize(0);
    return true;
  }
  if (!impl_) return false;
  x = impl_->solver.solve(rhs);
  return impl_->solver.info() == Eigen::Success && x.allFinite();
}

bool MKLPardisoLDLTSolver::solve_many(const Eigen::MatrixXd& rhs,
                                      Eigen::MatrixXd& x) {
  if (!impl_ && !empty_system_) return false;
  if (empty_system_) {
    if (rhs.rows() != 0) return false;
    x.resize(0, rhs.cols());
    return true;
  }
  return solve_many_with_eigen(impl_->solver, false, rhs, x);
}

int MKLPardisoLDLTSolver::perturbed_pivots() const {
  return impl_ ? static_cast<int>(impl_->solver.pardisoParameterArray()[13])
               : -1;
}

std::int64_t MKLPardisoLDLTSolver::factor_nonzeros() const {
  return impl_ ? static_cast<std::int64_t>(
                     impl_->solver.pardisoParameterArray()[17])
               : -1;
}

std::int64_t MKLPardisoLDLTSolver::factor_work() const {
  return impl_ ? static_cast<std::int64_t>(
                     impl_->solver.pardisoParameterArray()[18])
               : -1;
}

int MKLPardisoLDLTSolver::negative_eigenvalues() const {
  if (empty_system_) return 0;
  // PARDISO IPARM(23), zero-based slot 22: negative eigenvalues for mtype=-2.
  return impl_ ? static_cast<int>(
                     impl_->solver.pardisoParameterArray()[22])
               : -1;
}

int MKLPardisoLDLTSolver::estimated_deficiency() const {
  if (empty_system_) return 0;
  if (!impl_) return -1;
  const auto& iparm = impl_->solver.pardisoParameterArray();
  // IPARM(22)/(23) are the positive/negative inertia counts. Any missing
  // pivots after a successful symmetric-indefinite factor are numerical zeros.
  const int positive = static_cast<int>(iparm[21]);
  const int negative = static_cast<int>(iparm[22]);
  const int missing_inertia = std::max(
      0, static_cast<int>(impl_->solver.rows()) - positive - negative);
  const int perturbed = static_cast<int>(iparm[13]);
  if (std::getenv("MIPSOLVERS_IPM_VERBOSE") != nullptr) {
    std::fprintf(stderr,
                 "PARDISO LDLT inertia: dim=%lld positive=%d negative=%d "
                 "missing=%d perturbed=%d\n",
                 static_cast<long long>(impl_->solver.rows()), positive,
                 negative, missing_inertia, perturbed);
  }
  return missing_inertia;
}

class MKLPardisoAdaptiveSolver::Impl {
 public:
  enum class Backend { Undecided, LU, LDLT };

  MKLPardisoSolver lu;
  MKLPardisoLDLTSolver ldlt;
  Eigen::SparseMatrix<double> matrix;
  Backend selected{Backend::Undecided};
  bool lu_factorized{false};
  bool ldlt_factorized{false};
  double lu_factor_sec{std::numeric_limits<double>::infinity()};
  double ldlt_factor_sec{std::numeric_limits<double>::infinity()};
};

MKLPardisoAdaptiveSolver::MKLPardisoAdaptiveSolver() = default;
MKLPardisoAdaptiveSolver::~MKLPardisoAdaptiveSolver() = default;

const char* MKLPardisoAdaptiveSolver::backend_name() const {
  if (!impl_ || impl_->selected == Impl::Backend::Undecided)
    return "Intel-MKL-PARDISO-Adaptive(Eigen)";
  return impl_->selected == Impl::Backend::LDLT
             ? "Intel-MKL-PARDISO-Adaptive[LDLT](Eigen)"
             : "Intel-MKL-PARDISO-Adaptive[LU](Eigen)";
}

void MKLPardisoAdaptiveSolver::analyze_pattern(
    const Eigen::SparseMatrix<double>& a) {
  empty_system_ = is_empty_square_system(a);
  if (empty_system_) return;
  if (!impl_) impl_ = std::make_unique<Impl>();
  impl_->selected = Impl::Backend::Undecided;
  impl_->lu_factorized = false;
  impl_->ldlt_factorized = false;
  impl_->lu.analyze_pattern(a);
  impl_->ldlt.analyze_pattern(a);
}

bool MKLPardisoAdaptiveSolver::factorize(
    const Eigen::SparseMatrix<double>& a) {
  empty_system_ = is_empty_square_system(a);
  if (empty_system_) return true;
  if (!impl_) {
    impl_ = std::make_unique<Impl>();
    impl_->lu.analyze_pattern(a);
    impl_->ldlt.analyze_pattern(a);
  }
  auto timed_factor = [&](SparseLinearSolver& solver, double& elapsed) {
    const auto start = std::chrono::steady_clock::now();
    const bool ok = solver.factorize(a);
    elapsed = std::chrono::duration<double>(
                  std::chrono::steady_clock::now() - start)
                  .count();
    return ok;
  };

  if (impl_->selected == Impl::Backend::LU) {
    impl_->lu_factorized = timed_factor(impl_->lu, impl_->lu_factor_sec);
    if (impl_->lu_factorized) return true;
    impl_->ldlt_factorized =
        timed_factor(impl_->ldlt, impl_->ldlt_factor_sec);
    if (impl_->ldlt_factorized) impl_->selected = Impl::Backend::LDLT;
    return impl_->ldlt_factorized;
  }
  if (impl_->selected == Impl::Backend::LDLT) {
    impl_->ldlt_factorized =
        timed_factor(impl_->ldlt, impl_->ldlt_factor_sec);
    if (impl_->ldlt_factorized) return true;
    impl_->lu_factorized = timed_factor(impl_->lu, impl_->lu_factor_sec);
    if (impl_->lu_factorized) impl_->selected = Impl::Backend::LU;
    return impl_->lu_factorized;
  }

  // LDLT is the structure-preserving method, so evaluate it first. LU remains
  // lazy until solve() observes a failed/backward-unstable or perturbed LDLT
  // factor. This avoids paying for two numeric factors when LDLT already gives
  // a certified zero-perturbation solve.
  impl_->ldlt_factorized =
      timed_factor(impl_->ldlt, impl_->ldlt_factor_sec);
  impl_->matrix = a;
  if (impl_->ldlt_factorized) return true;
  impl_->lu_factorized = timed_factor(impl_->lu, impl_->lu_factor_sec);
  return impl_->lu_factorized;
}

bool MKLPardisoAdaptiveSolver::solve(const Eigen::VectorXd& rhs,
                                     Eigen::VectorXd& x) {
  if (empty_system_) {
    if (rhs.size() != 0) return false;
    x.resize(0);
    return true;
  }
  if (!impl_) return false;
  if (impl_->selected == Impl::Backend::LU)
    return impl_->lu.solve(rhs, x);
  if (impl_->selected == Impl::Backend::LDLT)
    return impl_->ldlt.solve(rhs, x);

  Eigen::VectorXd x_lu, x_ldlt;
  double lu_solve_sec = std::numeric_limits<double>::infinity();
  double ldlt_solve_sec = std::numeric_limits<double>::infinity();
  auto timed_solve = [&](SparseLinearSolver& solver, Eigen::VectorXd& solution,
                         double& elapsed) {
    const auto start = std::chrono::steady_clock::now();
    const bool ok = solver.solve(rhs, solution);
    elapsed = std::chrono::duration<double>(
                  std::chrono::steady_clock::now() - start)
                  .count();
    return ok && solution.allFinite();
  };
  const bool ldlt_ok = impl_->ldlt_factorized &&
                       timed_solve(impl_->ldlt, x_ldlt, ldlt_solve_sec);

  std::vector<double> row_sums(static_cast<std::size_t>(impl_->matrix.rows()),
                               0.0);
  for (int col = 0; col < impl_->matrix.outerSize(); ++col) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(impl_->matrix, col); it;
         ++it) {
      row_sums[static_cast<std::size_t>(it.row())] += std::abs(it.value());
    }
  }
  const double matrix_norm = row_sums.empty()
                                 ? 0.0
                                 : *std::max_element(row_sums.begin(),
                                                     row_sums.end());
  auto backward_error = [&](const Eigen::VectorXd& solution) {
    const double residual =
        (impl_->matrix * solution - rhs).lpNorm<Eigen::Infinity>();
    const double scale = 1.0 + rhs.lpNorm<Eigen::Infinity>() +
                         matrix_norm * solution.lpNorm<Eigen::Infinity>();
    return residual / scale;
  };
  const double ldlt_error = ldlt_ok ? backward_error(x_ldlt)
                                    : std::numeric_limits<double>::infinity();
  const double stable_limit = std::sqrt(std::numeric_limits<double>::epsilon());
  const bool ldlt_stable = ldlt_ok && ldlt_error <= stable_limit;
  const int ldlt_perturbed = impl_->ldlt.perturbed_pivots();

  if (ldlt_stable && ldlt_perturbed == 0) {
    impl_->selected = Impl::Backend::LDLT;
    x = std::move(x_ldlt);
    if (std::getenv("MIPSOLVERS_IPM_VERBOSE") != nullptr) {
      std::fprintf(stderr,
                   "PARDISO portfolio: ldlt_factor=%.6g ldlt_solve=%.6g "
                   "ldlt_error=%.3e ldlt_perturbed=0 lu=skipped\n",
                   impl_->ldlt_factor_sec, ldlt_solve_sec, ldlt_error);
    }
    return x.allFinite();
  }

  if (!impl_->lu_factorized) {
    const auto start = std::chrono::steady_clock::now();
    impl_->lu_factorized = impl_->lu.factorize(impl_->matrix);
    impl_->lu_factor_sec =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start)
            .count();
  }
  const bool lu_ok = impl_->lu_factorized &&
                     timed_solve(impl_->lu, x_lu, lu_solve_sec);
  if (!ldlt_ok && !lu_ok) return false;
  const double lu_error = lu_ok ? backward_error(x_lu)
                                : std::numeric_limits<double>::infinity();
  const bool lu_stable = lu_ok && lu_error <= stable_limit;
  const int lu_perturbed = impl_->lu.perturbed_pivots();

  if (std::getenv("MIPSOLVERS_IPM_VERBOSE") != nullptr) {
    std::fprintf(stderr,
                 "PARDISO portfolio: ldlt_factor=%.6g ldlt_solve=%.6g "
                 "ldlt_error=%.3e ldlt_perturbed=%d lu_factor=%.6g "
                 "lu_solve=%.6g lu_error=%.3e lu_perturbed=%d\n",
                 impl_->ldlt_factor_sec, ldlt_solve_sec, ldlt_error,
                 ldlt_perturbed, impl_->lu_factor_sec, lu_solve_sec, lu_error,
                 lu_perturbed);
  }

  if (ldlt_ok && lu_ok && ldlt_perturbed >= 0 && lu_perturbed >= 0 &&
      ldlt_perturbed != lu_perturbed) {
    impl_->selected = ldlt_perturbed < lu_perturbed ? Impl::Backend::LDLT
                                                    : Impl::Backend::LU;
  } else if (ldlt_stable != lu_stable) {
    impl_->selected = ldlt_stable ? Impl::Backend::LDLT : Impl::Backend::LU;
  } else if (ldlt_stable && lu_stable) {
    impl_->selected =
        impl_->ldlt_factor_sec + ldlt_solve_sec <=
                impl_->lu_factor_sec + lu_solve_sec
            ? Impl::Backend::LDLT
            : Impl::Backend::LU;
  } else {
    impl_->selected = ldlt_error <= lu_error ? Impl::Backend::LDLT
                                             : Impl::Backend::LU;
  }
  x = impl_->selected == Impl::Backend::LDLT ? std::move(x_ldlt)
                                              : std::move(x_lu);
  return x.allFinite();
}

int MKLPardisoAdaptiveSolver::perturbed_pivots() const {
  if (!impl_) return -1;
  if (impl_->selected == Impl::Backend::LDLT)
    return impl_->ldlt.perturbed_pivots();
  if (impl_->selected == Impl::Backend::LU)
    return impl_->lu.perturbed_pivots();
  return -1;
}

std::int64_t MKLPardisoAdaptiveSolver::factor_nonzeros() const {
  if (!impl_) return -1;
  return impl_->selected == Impl::Backend::LU ? impl_->lu.factor_nonzeros()
                                               : impl_->ldlt.factor_nonzeros();
}

std::int64_t MKLPardisoAdaptiveSolver::factor_work() const {
  if (!impl_) return -1;
  return impl_->selected == Impl::Backend::LU ? impl_->lu.factor_work()
                                               : impl_->ldlt.factor_work();
}

#endif

#ifdef HACDCPF_HAVE_MUMPS
#include <mpi.h>

#include <dmumps_c.h>

#include <mutex>

namespace {

// Homebrew/system MUMPS runs through the mpiseq sequential compatibility layer, whose
// MPI_Init is a no-op but which MUMPS expects to have been called before the
// first dmumps_c call. mpiseq has no MPI_Initialized, so we track it here.
void ensure_mpi_initialized() {
  static bool done = false;
  if (!done) {
    MPI_Init(nullptr, nullptr);
    done = true;
  }
}

// Serialises ALL MUMPS API calls process-wide.  MUMPS keeps mutable state in
// Fortran modules (not only in DMUMPS_STRUC_C), so concurrently driving two
// MumpsSolver instances from different threads is not safe; KLU/UMFPACK/Eigen
// are per-instance and stay fully parallel.  Uncontended lock cost is ~20 ns
// and irrelevant next to a factorization.  Recursive because factorize() may
// fall through to analyze_pattern() on the same thread.
std::recursive_mutex g_mumps_api_mutex;

}  // namespace

// MUMPS driver state: the DMUMPS_STRUC_C control block plus the 1-based
// lower-triangle CSC of the analyzed pattern.  The pattern is extracted once
// at analyze_pattern(); factorize() refills only the values (the sparsity
// pattern is invariant across IPM iterations).
class MumpsSolver::Impl {
 public:
  DMUMPS_STRUC_C par{};
  MUMPS_INT n{0};
  MUMPS_INT nnz_lower{0};
  std::vector<MUMPS_INT> irn, jcn;  // 1-based, lower triangle (row >= col)
  std::vector<double> a_vals;       // lower-triangle values in (irn,jcn) order
  std::vector<double> rhs_buf;
  double matrix_inf_norm{0.0};
  bool analyzed{false};
  bool numeric_attempted{false};
  bool factorized{false};

  ~Impl() {
    if (analyzed) {
      std::lock_guard<std::recursive_mutex> _lk(g_mumps_api_mutex);
      par.job = -2;  // JOB_END: free the factorization instance
      dmumps_c(&par);
    }
  }
};

MumpsSolver::MumpsSolver() = default;
MumpsSolver::~MumpsSolver() = default;

const char* MumpsSolver::backend_name() const {
  return "MUMPS (multifrontal LDL^T, symmetric indefinite)";
}

void MumpsSolver::analyze_pattern(const Eigen::SparseMatrix<double>& a) {
  std::lock_guard<std::recursive_mutex> _lk(g_mumps_api_mutex);
  empty_system_ = is_empty_square_system(a);
  if (empty_system_) return;
  ensure_mpi_initialized();
  if (!impl_) impl_ = std::make_unique<Impl>();
  auto& p = *impl_;

  if (p.analyzed) {
    p.par.job = -2;
    dmumps_c(&p.par);
    p.par = DMUMPS_STRUC_C{};
    p.analyzed = false;
    p.numeric_attempted = false;
    p.factorized = false;
  }

  // Extract the lower triangle (row >= col) in column-major order — this is
  // the deterministic value order refilled by factorize().
  const int n = static_cast<int>(a.rows());
  p.irn.clear();
  p.jcn.clear();
  p.a_vals.clear();
  p.irn.reserve(static_cast<size_t>(a.nonZeros()));
  p.jcn.reserve(static_cast<size_t>(a.nonZeros()));
  p.a_vals.reserve(static_cast<size_t>(a.nonZeros()));
  for (int col = 0; col < a.outerSize(); ++col) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(a, col); it; ++it) {
      if (it.row() >= it.col()) {
        p.irn.push_back(static_cast<MUMPS_INT>(it.row()) + 1);
        p.jcn.push_back(static_cast<MUMPS_INT>(it.col()) + 1);
        p.a_vals.push_back(it.value());
      }
    }
  }
  p.n = n;
  p.nnz_lower = static_cast<MUMPS_INT>(p.irn.size());

  p.par.comm_fortran = -987654;  // USE_COMM_WORLD (sequential via mpiseq)
  p.par.par = 1;                 // host participates in the computation
  p.par.sym = 2;                 // general symmetric indefinite (LDL^T)
  p.par.job = -1;                // JOB_INIT
  dmumps_c(&p.par);
  // Silence all MUMPS diagnostic output.
  p.par.icntl[0] = -1;
  p.par.icntl[1] = -1;
  p.par.icntl[2] = -1;
  p.par.icntl[3] = 0;
  p.par.icntl[6] = 7;   // ICNTL(7) ordering: auto (METIS/PORD nested dissection)
  p.par.icntl[7] = 0;   // ICNTL(8) scaling: none (caller equilibrates already)
  p.par.icntl[19] = 0;  // ICNTL(20): centralized dense RHS
  p.par.icntl[20] = 0;  // ICNTL(21): centralized solution
  p.par.icntl[23] = 1;  // ICNTL(24): detect numerical null pivots
  p.par.cntl[0] = 0.01; // CNTL(1) pivot threshold for indefinite systems

  p.par.n = p.n;
  p.par.nnz = p.nnz_lower;
  p.par.irn = p.irn.data();
  p.par.jcn = p.jcn.data();
  p.par.a = p.a_vals.data();
  p.par.job = 1;  // JOB_ANALYZE (symbolic analysis, once per pattern)
  dmumps_c(&p.par);
  p.analyzed = (p.par.infog[0] == 0);
  p.numeric_attempted = false;
  p.factorized = false;
}

bool MumpsSolver::factorize(const Eigen::SparseMatrix<double>& a) {
  std::lock_guard<std::recursive_mutex> _lk(g_mumps_api_mutex);
  empty_system_ = is_empty_square_system(a);
  if (empty_system_) return true;
  if (!impl_ || !impl_->analyzed) {
    analyze_pattern(a);
    if (!impl_ || !impl_->analyzed) return false;
  }
  auto& p = *impl_;
  if (a.rows() != p.n || a.cols() != p.n) return false;
  p.factorized = false;
  // Refill the lower-triangle values in the same column-major order used at
  // analysis time (the pattern is unchanged).
  MUMPS_INT k = 0;
  for (int col = 0; col < a.outerSize(); ++col) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(a, col); it; ++it) {
      if (it.row() >= it.col()) {
        if (k >= p.nnz_lower ||
            p.irn[static_cast<size_t>(k)] != it.row() + 1 ||
            p.jcn[static_cast<size_t>(k)] != it.col() + 1) {
          return false;
        }
        p.a_vals[static_cast<size_t>(k++)] = it.value();
      }
    }
  }
  if (k != p.nnz_lower) return false;

  std::vector<double> row_sums(static_cast<size_t>(p.n), 0.0);
  for (MUMPS_INT entry = 0; entry < p.nnz_lower; ++entry) {
    const int row = p.irn[static_cast<size_t>(entry)] - 1;
    const int col = p.jcn[static_cast<size_t>(entry)] - 1;
    const double abs_value = std::abs(p.a_vals[static_cast<size_t>(entry)]);
    row_sums[static_cast<size_t>(row)] += abs_value;
    if (row != col) row_sums[static_cast<size_t>(col)] += abs_value;
  }
  p.matrix_inf_norm = row_sums.empty()
      ? 0.0 : *std::max_element(row_sums.begin(), row_sums.end());
  p.par.a = p.a_vals.data();
  p.par.job = 2;  // JOB_FACTOR (numeric factorization)
  dmumps_c(&p.par);
  p.numeric_attempted = true;
  p.factorized = p.par.infog[0] == 0 && p.par.infog[27] == 0;
  return p.factorized;
}

bool MumpsSolver::solve(const Eigen::VectorXd& rhs, Eigen::VectorXd& x) {
  std::lock_guard<std::recursive_mutex> _lk(g_mumps_api_mutex);
  if (empty_system_) {
    if (rhs.size() != 0) return false;
    x.resize(0);
    return true;
  }
  if (!impl_ || !impl_->analyzed || !impl_->factorized) return false;
  auto& p = *impl_;
  if (rhs.size() != p.n || !rhs.allFinite()) return false;
  p.rhs_buf.assign(rhs.data(), rhs.data() + rhs.size());
  p.par.rhs = p.rhs_buf.data();
  p.par.nrhs = 1;
  p.par.lrhs = p.n;
  p.par.job = 3;  // JOB_SOLVE
  dmumps_c(&p.par);
  if (p.par.infog[0] != 0) return false;
  x = Eigen::VectorXd::Map(p.rhs_buf.data(), p.n);
  if (!x.allFinite()) return false;

  Eigen::VectorXd residual = -rhs;
  for (MUMPS_INT entry = 0; entry < p.nnz_lower; ++entry) {
    const int row = p.irn[static_cast<size_t>(entry)] - 1;
    const int col = p.jcn[static_cast<size_t>(entry)] - 1;
    const double value = p.a_vals[static_cast<size_t>(entry)];
    residual[row] += value * x[col];
    if (row != col) residual[col] += value * x[row];
  }
  const double scale = std::max(
      1.0, rhs.lpNorm<Eigen::Infinity>() +
               p.matrix_inf_norm * x.lpNorm<Eigen::Infinity>());
  return residual.lpNorm<Eigen::Infinity>() <= 1e-9 * scale;
}

bool MumpsSolver::solve_many(const Eigen::MatrixXd& rhs, Eigen::MatrixXd& x) {
  std::lock_guard<std::recursive_mutex> _lk(g_mumps_api_mutex);
  if (empty_system_) {
    if (rhs.rows() != 0) return false;
    x.resize(0, rhs.cols());
    return true;
  }
  if (!impl_ || !impl_->analyzed || !impl_->factorized) return false;
  auto& p = *impl_;
  if (rhs.rows() != p.n ||
      rhs.cols() > std::numeric_limits<MUMPS_INT>::max() ||
      !rhs.allFinite()) {
    return false;
  }
  if (rhs.cols() == 0) {
    x.resize(p.n, 0);
    return true;
  }

  p.rhs_buf.assign(rhs.data(), rhs.data() + rhs.size());
  p.par.rhs = p.rhs_buf.data();
  p.par.nrhs = static_cast<MUMPS_INT>(rhs.cols());
  p.par.lrhs = p.n;
  p.par.job = 3;  // JOB_SOLVE, all column-major right-hand sides at once
  dmumps_c(&p.par);
  if (p.par.infog[0] != 0) return false;
  x = Eigen::Map<const Eigen::MatrixXd>(p.rhs_buf.data(), p.n, rhs.cols());
  if (!x.allFinite()) return false;

  Eigen::MatrixXd residual = -rhs;
  for (MUMPS_INT entry = 0; entry < p.nnz_lower; ++entry) {
    const int row = p.irn[static_cast<size_t>(entry)] - 1;
    const int col = p.jcn[static_cast<size_t>(entry)] - 1;
    const double value = p.a_vals[static_cast<size_t>(entry)];
    residual.row(row) += value * x.row(col);
    if (row != col) residual.row(col) += value * x.row(row);
  }
  const double scale = std::max(
      1.0, rhs.cwiseAbs().maxCoeff() +
               p.matrix_inf_norm * x.cwiseAbs().maxCoeff());
  return residual.cwiseAbs().maxCoeff() <= 1e-9 * scale;
}

int MumpsSolver::negative_eigenvalues() const {
  if (empty_system_ || !impl_ || !impl_->factorized) return -1;
  // INFOG(12): number of negative pivots in the LDLᵀ factorization.
  return static_cast<int>(impl_->par.infog[11]);
}

int MumpsSolver::estimated_deficiency() const {
  if (empty_system_ || !impl_ || !impl_->numeric_attempted) return -1;
  // INFOG(28): estimated deficiency when ICNTL(24)=1.
  return static_cast<int>(impl_->par.infog[27]);
}
#endif

// AUDIT-NAV: 所有原生 Newton/KKT 路径的默认稀疏后端在此决定；审核部署差异时
// 先核对编译宏和返回顺序，再核对各后端的 analyze/factorize/solve 契约。
std::unique_ptr<SparseLinearSolver> make_default_sparse_solver() {
  // Note: MumpsSolver is deliberately NOT the generic default — it factors
  // singular rank-deficient systems without flagging (it accepts tiny
  // pivots), which would defeat the δ_W/δ_C inertia-escalation machinery the
  // IPM relies on.  It is used selectively where the KKT is well-posed
  // (e.g. the OPF parity path) via explicit construction.
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
