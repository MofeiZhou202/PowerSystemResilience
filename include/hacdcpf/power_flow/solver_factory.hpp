#pragma once

/// PowerFlowSolverFactory
/// ======================
/// Creates the requested IPowerFlowSolver from an explicit method selector.
/// Centralises solver-selection logic so the facade functions and the
/// robust pipeline have a single routing point.
///
/// Usage:
///   auto solver = PowerFlowSolverFactory::create(PowerFlowMethod::Newton);
///   auto result = solver->solve(problem);

#include <memory>

#include "hacdcpf/power_flow/power_flow_options.hpp"
#include "hacdcpf/power_flow/solver_interface.hpp"

namespace hacdcpf {

// ── Solver method selector ────────────────────────────────────────────────────

enum class PowerFlowMethod {
    /// Full AC/DC Newton-Raphson with optional coupled Jacobian (default).
    Newton,
    /// AC fast-decoupled (XB variant). Fast for weakly-meshed networks.
    FDPF,
    /// DC linearised PF (active power only).
    DC,
    /// Island-aware adaptive selection (Newton per island, DC fallback).
    Adaptive,
    /// Continuation power flow — traces the P-V curve.
    Continuation,
    /// Newton–GMRES with Schur-complement preconditioner (Phase 5 fallback).
    NewtonKrylov,
};

// ── Factory ───────────────────────────────────────────────────────────────────

class PowerFlowSolverFactory {
public:
    /// Construct and return the solver specified by \p method.
    /// If \p method is unrecognised, returns a Newton solver.
    [[nodiscard]] static std::unique_ptr<IPowerFlowSolver>
    create(PowerFlowMethod method);

};

}  // namespace hacdcpf
