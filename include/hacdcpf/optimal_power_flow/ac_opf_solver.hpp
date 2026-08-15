#pragma once

/// optimal_power_flow/ac_opf_solver.hpp
/// =====================================
/// Forward declarations of AC OPF solver entry points.
/// Replaces: optimal_power_flow/ac_opf.hpp

#include <memory>

#include "hacdcpf/model/hybrid_power_system.hpp"
#include "hacdcpf/optimal_power_flow/opf_options.hpp"
#include "hacdcpf/optimal_power_flow/opf_result.hpp"

namespace hacdcpf::opf {

/// Owning repeated-solve context for a fixed-layout native Parity ACOPF.
///
/// The session keeps the assembled formulation and sparse symbolic ordering
/// alive across calls.  A complete native-IPM continuation state is carried
/// only when the exact variable/equality/inequality/bound layout signature
/// matches.  Structural changes invalidate the cached assets and rebuild them.
class PreparedACOPFSession {
 public:
  explicit PreparedACOPFSession(ACOPFOptions options = {});
  ~PreparedACOPFSession();

  PreparedACOPFSession(const PreparedACOPFSession&) = delete;
  PreparedACOPFSession& operator=(const PreparedACOPFSession&) = delete;
  PreparedACOPFSession(PreparedACOPFSession&&) noexcept;
  PreparedACOPFSession& operator=(PreparedACOPFSession&&) noexcept;

  ACOPFResult solve(const HybridPowerSystem& sys);
  void reset();

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

/// Solve the AC optimal power flow problem for a hybrid AC/DC system.
///
/// The solver canonicalizes `sys`, assembles a nonlinear program, and returns
/// dispatch and voltage vectors in user-facing component order.  For the native
/// AC path the core equality model is the polar AC balance
///
/// @htmlonly
/// <div>\[
///   P_i^g - P_i^d - P_i^{conv}
///     = V_i \sum_j V_j (G_{ij}\cos\theta_{ij}
///       + B_{ij}\sin\theta_{ij})
/// \]</div>
/// <div>\[
///   Q_i^g - Q_i^d - Q_i^{conv}
///     = V_i \sum_j V_j (G_{ij}\sin\theta_{ij}
///       - B_{ij}\cos\theta_{ij})
/// \]</div>
/// @endhtmlonly
///
/// with generator, voltage, branch-flow, converter, and optional load-shedding
/// bounds.  If `ACOPFOptions::use_parity_ipm` is true, the full-space parity
/// formulation in `hacdcpf::opf::parity` is used instead.
///
/// @param sys Rich hybrid power-system model. The input is not modified.
/// @param opt Interior-point tolerances, fallback policy, and constraint-family
/// toggles.
/// @return ACOPFResult containing voltages, dispatch, converter powers,
/// convergence metrics, infeasibility hints, and solver-path metadata.
ACOPFResult solve_ac_opf(const HybridPowerSystem& sys,
                         const ACOPFOptions& opt = {});

/// Compare the analytical AC OPF Jacobian against finite differences.
///
/// The diagnostic perturbs the same state variables used by `solve_ac_opf` and
/// reports the largest absolute/relative mismatch between analytical residual
/// derivatives and finite-difference estimates.
///
/// @param sys System at which the OPF residual/Jacobian model is assembled.
/// @param opt Solver options controlling the same formulation toggles as OPF.
/// @return Worst-row/column error summary for code-review and regression checks.
ACOPFJacobianDiagnostics compute_jacobian_diagnostics(const HybridPowerSystem& sys,
                                                       const ACOPFOptions& opt = {});

}  // namespace hacdcpf::opf
