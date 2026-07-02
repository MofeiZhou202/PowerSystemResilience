#include "hacdcpf/dynamics/solvers/SparseLinearSolver.hpp"

#if defined(HACDCPF_OPF_HAVE_UMFPACK)
#include <Eigen/UmfPackSupport>
#endif
#if defined(HACDCPF_OPF_HAVE_KLU)
#include <Eigen/KLUSupport>
#endif
#include <Eigen/IterativeLinearSolvers>
#include <Eigen/SparseLU>

namespace hacdcpf::dynamics {

namespace {

std::string solver_name(DynamicLinearSolverType type) {
  switch (type) {
    case DynamicLinearSolverType::EigenSparseLU: return "EigenSparseLU";
    case DynamicLinearSolverType::EigenSparseQR: return "EigenSparseQR";
    case DynamicLinearSolverType::EigenBiCGSTAB: return "EigenBiCGSTAB";
    case DynamicLinearSolverType::KLU: return "KLU";
    case DynamicLinearSolverType::UMFPACK: return "UMFPACK";
    case DynamicLinearSolverType::Pardiso: return "Pardiso";
    case DynamicLinearSolverType::PETSc: return "PETSc";
  }
  return "Unknown";
}

}  // namespace

SparseLinearSolveResult SparseLinearSolver::solve(
    const Eigen::SparseMatrix<double>& a,
    const Eigen::VectorXd& b,
    Eigen::VectorXd& x) const {
  if (type_ == DynamicLinearSolverType::EigenBiCGSTAB) {
    Eigen::BiCGSTAB<Eigen::SparseMatrix<double>> solver;
    solver.compute(a);
    x = solver.solve(b);
    if (solver.info() == Eigen::Success && x.allFinite()) return {true, {}};
    return {false, "Real BiCGSTAB sparse solve failed"};
  }
#if defined(HACDCPF_OPF_HAVE_UMFPACK)
  if (type_ == DynamicLinearSolverType::UMFPACK) {
    Eigen::UmfPackLU<Eigen::SparseMatrix<double>> solver;
    solver.compute(a);
    if (solver.info() != Eigen::Success) return {false, "UMFPACK factorization failed"};
    x = solver.solve(b);
    if (solver.info() == Eigen::Success && x.allFinite()) return {true, {}};
    return {false, "UMFPACK solve failed"};
  }
#endif
#if defined(HACDCPF_OPF_HAVE_KLU)
  if (type_ == DynamicLinearSolverType::KLU) {
    Eigen::KLU<Eigen::SparseMatrix<double>> solver;
    solver.compute(a);
    if (solver.info() != Eigen::Success) return {false, "KLU factorization failed"};
    x = solver.solve(b);
    if (solver.info() == Eigen::Success && x.allFinite()) return {true, {}};
    return {false, "KLU solve failed"};
  }
#endif
  if (type_ == DynamicLinearSolverType::KLU ||
      type_ == DynamicLinearSolverType::UMFPACK ||
      type_ == DynamicLinearSolverType::Pardiso ||
      type_ == DynamicLinearSolverType::PETSc ||
      type_ == DynamicLinearSolverType::EigenSparseQR) {
    // Fall through to the always-available baseline. The result message remains
    // empty on success so callers can use optional backend requests portably.
    (void)solver_name(type_);
  }
  Eigen::SparseLU<Eigen::SparseMatrix<double>> solver;
  solver.compute(a);
  if (solver.info() != Eigen::Success) {
    return {false, "Real sparse factorization failed"};
  }
  x = solver.solve(b);
  if (solver.info() != Eigen::Success || !x.allFinite()) {
    return {false, "Real sparse solve failed"};
  }
  return {true, {}};
}

SparseLinearSolveResult SparseLinearSolver::solve(
    const Eigen::SparseMatrix<std::complex<double>>& a,
    const Eigen::VectorXcd& b,
    Eigen::VectorXcd& x) const {
  if (type_ == DynamicLinearSolverType::EigenBiCGSTAB) {
    Eigen::BiCGSTAB<Eigen::SparseMatrix<std::complex<double>>> solver;
    solver.compute(a);
    x = solver.solve(b);
    if (solver.info() == Eigen::Success && x.allFinite()) return {true, {}};
    return {false, "Complex BiCGSTAB sparse solve failed"};
  }
#if defined(HACDCPF_OPF_HAVE_UMFPACK)
  if (type_ == DynamicLinearSolverType::UMFPACK) {
    Eigen::UmfPackLU<Eigen::SparseMatrix<std::complex<double>>> solver;
    solver.compute(a);
    if (solver.info() != Eigen::Success) return {false, "Complex UMFPACK factorization failed"};
    x = solver.solve(b);
    if (solver.info() == Eigen::Success && x.allFinite()) return {true, {}};
    return {false, "Complex UMFPACK solve failed"};
  }
#endif
#if defined(HACDCPF_OPF_HAVE_KLU)
  if (type_ == DynamicLinearSolverType::KLU) {
    Eigen::KLU<Eigen::SparseMatrix<std::complex<double>>> solver;
    solver.compute(a);
    if (solver.info() != Eigen::Success) return {false, "Complex KLU factorization failed"};
    x = solver.solve(b);
    if (solver.info() == Eigen::Success && x.allFinite()) return {true, {}};
    return {false, "Complex KLU solve failed"};
  }
#endif
  if (type_ == DynamicLinearSolverType::KLU ||
      type_ == DynamicLinearSolverType::UMFPACK ||
      type_ == DynamicLinearSolverType::Pardiso ||
      type_ == DynamicLinearSolverType::PETSc ||
      type_ == DynamicLinearSolverType::EigenSparseQR) {
    (void)solver_name(type_);
  }
  Eigen::SparseLU<Eigen::SparseMatrix<std::complex<double>>> solver;
  solver.compute(a);
  if (solver.info() != Eigen::Success) {
    return {false, "Complex sparse factorization failed"};
  }
  x = solver.solve(b);
  if (solver.info() != Eigen::Success || !x.allFinite()) {
    return {false, "Complex sparse solve failed"};
  }
  return {true, {}};
}

}  // namespace hacdcpf::dynamics
