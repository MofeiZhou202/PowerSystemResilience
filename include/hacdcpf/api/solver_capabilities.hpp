#pragma once

/// api/solver_capabilities.hpp
/// ==============================
/// Runtime query for optional backend availability.
///
/// Usage:
///   auto caps = hacdcpf::get_solver_capabilities();
///   if (caps.has_gurobi) { ... }

namespace hacdcpf {

/// Describes which optional solver backends are available in this build.
/// Query at runtime via get_solver_capabilities().
struct SolverCapabilities {
    // ── Linear algebra backends ───────────────────────────────────────────────
    bool has_sparse_lu{true};              ///< Eigen SparseLU (always available)
    bool has_klu{false};                   ///< KLU from SuiteSparse
    bool has_umfpack{false};              ///< UMFPACK from SuiteSparse
    bool has_accelerate{false};           ///< Apple Accelerate framework (macOS only)

    // ── MIP / QP / NLP solver backends ───────────────────────────────────────
    bool has_highs{false};                ///< HiGHS LP/MIP solver
    bool has_gurobi{false};               ///< Gurobi LP/QP/MIP solver
    bool has_ipopt{false};                ///< Ipopt NLP solver
    bool has_scip{false};                 ///< SCIP MIP solver
    bool has_papilo{false};               ///< PaPILO presolve library
    bool has_native_ipm{true};            ///< Built-in primal-dual IPM (always)

    // ── I/O backends ─────────────────────────────────────────────────────────
    bool has_excel{false};                ///< Excel I/O via OpenXLSX
    bool has_opendss{false};              ///< OpenDSS bridge

    // ── Solver feature flags ─────────────────────────────────────────────────
    bool supports_quadratic_objective{true};  ///< QP objective (native + HiGHS)
    bool supports_integer_variables{false};   ///< MIP variables (requires HiGHS/Gurobi/SCIP)
    bool supports_ac_opf{true};               ///< Nonlinear AC OPF (always)
    bool supports_dc_opf{true};               ///< Linear DC OPF (always)
    bool supports_three_phase{true};          ///< Three-phase unbalanced PF (always)
};

/// Returns a SolverCapabilities struct reflecting what was compiled in.
/// The result is inexpensive to compute; it is safe to call repeatedly.
SolverCapabilities get_solver_capabilities() noexcept;

}  // namespace hacdcpf
