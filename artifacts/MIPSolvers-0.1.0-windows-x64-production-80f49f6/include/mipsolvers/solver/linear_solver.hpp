#pragma once
#include "mipsolvers/engine/kernel/linear_algebra/linear_solver.hpp"
namespace mipsolvers::solver {
using mipsolvers::engine::SparseLinearSolver;
using mipsolvers::engine::EigenSparseLUSolver;
#ifdef HACDCPF_HAVE_UMFPACK
using mipsolvers::engine::EigenUmfPackSolver;
#endif
#ifdef HACDCPF_HAVE_KLU
using mipsolvers::engine::EigenKluSolver;
#endif
#ifdef HACDCPF_HAVE_SUPERLU
using mipsolvers::engine::SuperLUSolver;
#endif
#ifdef HACDCPF_HAVE_MKL_PARDISO
using mipsolvers::engine::MKLPardisoSolver;
using mipsolvers::engine::MKLPardisoLLTSolver;
using mipsolvers::engine::MKLPardisoLDLTSolver;
using mipsolvers::engine::MKLPardisoAdaptiveSolver;
#endif
using mipsolvers::engine::make_default_sparse_solver;
}  // namespace mipsolvers::solver
