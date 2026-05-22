#pragma once

/// PowerFlowSolverFactory
/// ======================
/// Creates the appropriate IPowerFlowSolver based on PowerFlowOptions.
/// Centralises solver-selection logic so the facade functions and the
/// robust pipeline have a single routing point.
///
/// Usage:
///   PowerFlowOptions opt;
///   opt.method = PowerFlowMethod::Newton;
///   auto solver = PowerFlowSolverFactory::create(opt);
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

    /// Convenience overload: inspect \p options to pick the solver.
    /// Rules (in priority order):
    ///   1. options.method (if not Newton) → use as-is
    ///   2. options.enable_semi_smooth_newton → Newton
    ///   3. options.enable_coupled_jacobian  → Newton (full coupling)
    ///   4. else → Newton (default)
    [[nodiscard]] static std::unique_ptr<IPowerFlowSolver>
    create(const PowerFlowOptions& options);
};

}  // namespace hacdcpf
