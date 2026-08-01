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
#endif

#ifdef HACDCPF_HAVE_KLU
class EigenKluSolver::Impl {
 public:
  Eigen::KLU<Eigen::SparseMatrix<double>> solver;
};

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
#endif

#ifdef HACDCPF_HAVE_SUPERLU
class SuperLUSolver::Impl {
 public:
  Eigen::SuperLU<Eigen::SparseMatrix<double>> solver;
};

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
  bool analyzed{false};

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
  p.par.cntl[0] = 0.01; // CNTL(1) pivot threshold for indefinite systems

  p.par.n = p.n;
  p.par.nnz = p.nnz_lower;
  p.par.irn = p.irn.data();
  p.par.jcn = p.jcn.data();
  p.par.a = p.a_vals.data();
  p.par.job = 1;  // JOB_ANALYZE (symbolic analysis, once per pattern)
  dmumps_c(&p.par);
  p.analyzed = (p.par.infog[0] == 0);
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
  // Refill the lower-triangle values in the same column-major order used at
  // analysis time (the pattern is unchanged).
  MUMPS_INT k = 0;
  for (int col = 0; col < a.outerSize(); ++col) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(a, col); it; ++it) {
      if (it.row() >= it.col()) {
        p.a_vals[static_cast<size_t>(k++)] = it.value();
      }
    }
  }
  p.par.a = p.a_vals.data();
  p.par.job = 2;  // JOB_FACTOR (numeric factorization)
  dmumps_c(&p.par);
  return p.par.infog[0] >= 0;  // >= 0: success or handled warning (e.g. null pivot)
}

bool MumpsSolver::solve(const Eigen::VectorXd& rhs, Eigen::VectorXd& x) {
  std::lock_guard<std::recursive_mutex> _lk(g_mumps_api_mutex);
  if (empty_system_) {
    if (rhs.size() != 0) return false;
    x.resize(0);
    return true;
  }
  if (!impl_ || !impl_->analyzed) return false;
  auto& p = *impl_;
  p.rhs_buf.assign(rhs.data(), rhs.data() + rhs.size());
  p.par.rhs = p.rhs_buf.data();
  p.par.nrhs = 1;
  p.par.lrhs = p.n;
  p.par.job = 3;  // JOB_SOLVE
  dmumps_c(&p.par);
  if (p.par.infog[0] < 0) return false;
  x = Eigen::VectorXd::Map(p.rhs_buf.data(), p.n);
  return true;
}

int MumpsSolver::negative_eigenvalues() const {
  if (empty_system_ || !impl_ || !impl_->analyzed) return -1;
  // INFOG(12): number of negative pivots in the LDLᵀ factorization.
  return static_cast<int>(impl_->par.infog[11]);
}
#endif

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
