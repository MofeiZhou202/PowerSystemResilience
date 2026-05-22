#pragma once

/// backend/linear_solver.hpp
/// ==========================
/// Public interface to the sparse linear solver backend.
/// Replaces: power_flow/linear_solver.hpp.

#include "hacdcpf/engine/kernel/linear_algebra/linear_solver.hpp"

namespace hacdcpf::backend {

using hacdcpf::engine::SparseLinearSolver;
using hacdcpf::engine::EigenSparseLUSolver;
using hacdcpf::engine::make_default_sparse_solver;

}  // namespace hacdcpf::backend

// Also expose in the powerflow namespace for backward compatibility.
namespace hacdcpf::powerflow {
using hacdcpf::engine::SparseLinearSolver;
using hacdcpf::engine::EigenSparseLUSolver;
using hacdcpf::engine::make_default_sparse_solver;
}  // namespace hacdcpf::powerflow
