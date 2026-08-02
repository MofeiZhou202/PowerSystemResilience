#pragma once

/// IPowerFlowSolver — abstract interface for all PF solver implementations.
/// =========================================================================
///
/// Every concrete solver (Newton, FDPF, DC-only, CPF, etc.) implements this
/// interface so the solver factory and upper layers can swap them uniformly.

#include "hacdcpf/power_flow/power_flow_options.hpp"
#include "hacdcpf/power_flow/power_flow_result.hpp"
#include "hacdcpf/power_flow/power_flow_problem.hpp"

namespace hacdcpf {

// ── Abstract solver interface ─────────────────────────────────────────────────

class IPowerFlowSolver {
public:
    virtual ~IPowerFlowSolver() = default;

    /// Solve the power-flow problem and return a result.
    /// Options embedded in \p problem.options take precedence over any
    /// solver-internal defaults.
    [[nodiscard]] virtual PowerFlowResult solve(
        const PowerFlowProblem& problem) = 0;

    /// Human-readable solver name (used in diagnostics/logging).
    [[nodiscard]] virtual const char* name() const noexcept = 0;
};

// ── Concrete solver forward declarations ─────────────────────────────────────
// Each class lives in its own .hpp/cpp under include/hacdcpf/power_flow/.

/// Hybrid AC/DC Newton-Raphson (default, full Jacobian).
class NewtonHybridSolver final : public IPowerFlowSolver {
public:
    [[nodiscard]] PowerFlowResult solve(const PowerFlowProblem& prob) override;
    [[nodiscard]] const char* name() const noexcept override { return "NewtonHybrid"; }
};

/// AC fast-decoupled PF (XB variant).
class FDPFSolver final : public IPowerFlowSolver {
public:
    [[nodiscard]] PowerFlowResult solve(const PowerFlowProblem& prob) override;
    [[nodiscard]] const char* name() const noexcept override { return "FDPF"; }
};

/// Lossless balanced-AC B-theta linearised PF (not the independent DC grid solver).
class DCPowerFlowSolver final : public IPowerFlowSolver {
public:
    [[nodiscard]] PowerFlowResult solve(const PowerFlowProblem& prob) override;
    [[nodiscard]] const char* name() const noexcept override { return "DC"; }
};

/// Island-aware adaptive solver (wraps Newton + DC + FDPF per island).
class AdaptiveHybridSolver final : public IPowerFlowSolver {
public:
    [[nodiscard]] PowerFlowResult solve(const PowerFlowProblem& prob) override;
    [[nodiscard]] const char* name() const noexcept override { return "AdaptiveHybrid"; }
};

/// Continuation PF (voltage stability tracing).
class ContinuationPowerFlowSolver final : public IPowerFlowSolver {
public:
    [[nodiscard]] PowerFlowResult solve(const PowerFlowProblem& prob) override;
    [[nodiscard]] const char* name() const noexcept override { return "ContinuationPF"; }
};

/// Newton–GMRES Krylov fallback.
class NewtonKrylovSolver final : public IPowerFlowSolver {
public:
    [[nodiscard]] PowerFlowResult solve(const PowerFlowProblem& prob) override;
    [[nodiscard]] const char* name() const noexcept override { return "NewtonKrylov"; }
};

}  // namespace hacdcpf
