#pragma once
#include <mipsolvers/engine/kernel/linear_algebra/linear_solver.hpp>
namespace hacdcpf::engine {
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
#endif
  using mipsolvers::engine::make_default_sparse_solver;
}
