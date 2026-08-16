#pragma once

/// model/converter_model_scope.hpp
/// =================================
/// Per-engine declaration of the unified AC/DC (VSC) and DC/DC converter-model
/// features that a given result actually honored. Historical theory is retained
/// in docs/archive/theory/multiple_converter.md.
///
/// Different analyses (snapshot power flow, OPF, harmonics, time-series,
/// reliability) deliberately model converters at different fidelity.  Attaching
/// this scope to each result lets downstream code branch on what was *actually*
/// modelled instead of assuming every engine implements the full converter
/// physics.  This mirrors the `model_scope` / `ValidityFlags` pattern already
/// used by the reliability and resilience results.

#include <string>

namespace hacdcpf {

/// Honest, machine-checkable description of converter-model fidelity for a result.
struct ConverterModelScope {
  /// Short human-readable scope tag, e.g.
  /// "steady-state-newton:vsc-3mode+dcdc-power-transfer".
  std::string model_scope{"unset"};

  /// Per-feature flags — true only when the producing engine actually enforced
  /// or modelled the feature for this result (not merely stored the field).
  struct ValidityFlags {
    /// VSC switching/conduction loss vs. the DC bus (linear or quadratic model).
    bool vsc_loss_modelled{false};
    /// AC-side conduction loss drawn from the DC bus (r_conv_ac_pu coupling).
    bool vsc_ac_conduction_loss_modelled{false};
    /// Apparent-power capacity circle P^2 + Q^2 <= S^2 enforced as a constraint.
    bool vsc_capacity_circle_enforced{false};
    /// AC and/or DC current limits enforced as constraints (not post-hoc only).
    bool vsc_current_limits_enforced{false};
    /// A grid-forming VSC internal voltage behind virtual impedance was an
    /// explicit solved Norton/Thevenin port, rather than a terminal-bus source.
    bool vsc_gfm_norton_modelled{false};
    /// P/Q-priority current limiting changed the solved GFM operating point
    /// through the nonsmooth equation block (not a post-solve projection).
    bool vsc_gfm_priority_limit_enforced{false};
    /// At least one AC island without a terminal SLACK was angle-anchored by
    /// fixed GFM internal-voltage phasor(s), with every terminal Vm/Va retained.
    bool vsc_gfm_island_reference_modelled{false};
    /// Modulation-index feasibility (m_min <= m <= m_max, Vac = Km*m*Vdc).
    bool vsc_modulation_limits_enforced{false};
    /// VDC_Q / VDC_VAC DC-voltage control (equality or stiff droop) honored.
    bool vsc_vdc_control_modelled{false};
    /// Multi-source DC-voltage coordination (droop / participation / master-slave)
    /// actually affects the solved operating point (not just pre-solve checks).
    bool dc_multisource_coordination_modelled{false};
    /// DC/DC efficiency + I^2R loss applied to the port power balance.
    bool dcdc_loss_modelled{false};
    /// DC/DC topology duty-ratio feasibility (d_min <= D <= d_max) enforced.
    bool dcdc_duty_ratio_enforced{false};
    /// LCC quasi-steady stations (U_d0/cos(alpha)/cos(gamma) characteristics,
    /// Q = P*tan(phi)) consumed by the solve; no overlap-angle iteration.
    bool lcc_quasi_steady_modelled{false};
    /// Converter-transformer taps were iterated against the LCC alpha/gamma
    /// targets within their declared R-card ranges.
    bool lcc_transformer_tap_control_modelled{false};
    /// Equation/variable closure (N_eq == N_var) and structural rank verified
    /// before the numerical solve.
    bool equation_closure_checked{false};
  };
  ValidityFlags validity{};
};

}  // namespace hacdcpf
