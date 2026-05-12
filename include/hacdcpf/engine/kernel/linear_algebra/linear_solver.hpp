#pragma once
// Forwarding header: hacdcpf::engine linear solvers → mipsolvers::engine
#include <mipsolvers/engine/kernel/linear_algebra/linear_solver.hpp>

namespace hacdcpf::engine {
  using mipsolvers::engine::SparseLinearSolver;
  using mipsolvers::engine::EigenSparseLUSolver;
  using mipsolvers::engine::make_default_sparse_solver;
#ifdef HACDCPF_HAVE_UMFPACK
  using mipsolvers::engine::EigenUmfPackSolver;
#endif
#ifdef HACDCPF_HAVE_KLU
  using mipsolvers::engine::EigenKluSolver;
#endif
}
