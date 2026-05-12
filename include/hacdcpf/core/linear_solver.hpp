#pragma once
#include "hacdcpf/engine/kernel/linear_algebra/linear_solver.hpp"
namespace hacdcpf::core {
using hacdcpf::engine::SparseLinearSolver;
using hacdcpf::engine::EigenSparseLUSolver;
#ifdef HACDCPF_HAVE_UMFPACK
using hacdcpf::engine::EigenUmfPackSolver;
#endif
#ifdef HACDCPF_HAVE_KLU
using hacdcpf::engine::EigenKluSolver;
#endif
using hacdcpf::engine::make_default_sparse_solver;
}  // namespace hacdcpf::core
