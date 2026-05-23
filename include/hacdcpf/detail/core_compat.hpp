#pragma once

/// detail/core_compat.hpp — Backward-compat aliases in hacdcpf::core.
/// Replaces the deleted core/ compat-stubs directory.

#include "hacdcpf/assembly/solver_data.hpp"
#include "hacdcpf/engine/kernel/linear_algebra/linear_solver.hpp"
#include "hacdcpf/engine/solver/native/nle/newton_solver.hpp"
#include "hacdcpf/power_flow/jacobian_builder.hpp"
#include "hacdcpf/power_flow/assembly/jacobian_builder.hpp"

namespace hacdcpf::core {

using SolverData                    = hacdcpf::powerflow::SolverData;
using JacobianContext               = hacdcpf::powerflow::JacobianContext;
using JacobianPattern               = hacdcpf::powerflow::JacobianPattern;
using SparseLinearSolver            = hacdcpf::engine::SparseLinearSolver;
using NewtonSolver                  = hacdcpf::engine::NewtonSolver;
using hacdcpf::powerflow::make_solver_data;
using hacdcpf::powerflow::make_solver_data_projected;
using hacdcpf::powerflow::build_jacobian_pattern;
using hacdcpf::powerflow::evaluate_residual_and_jacobian;
using hacdcpf::powerflow::evaluate_residual_only;
using hacdcpf::engine::make_default_sparse_solver;

}  // namespace hacdcpf::core
