#pragma once

/// model/device_control_role.hpp
///
/// Resolved steady-state control role of an AC/DC (VSC) converter
/// (multi-converter model r1 §7).  This turns a converter's stored control mode
/// into an explicit set of role flags — which quantities it fixes, which it
/// releases, and which references it provides — so the coordination checks and
/// island-reference logic can reason about converters uniformly instead of
/// re-deriving behaviour from the raw enum at every call site.

#include <cmath>

#include "hacdcpf/model/converter_components.hpp"
#include "hacdcpf/model/enum_strings.hpp"
#include "hacdcpf/model/enums/converter_enums.hpp"

namespace hacdcpf {

struct DeviceControlRole {
  // AC-side controlled quantities.
  bool controls_ac_p{false};
  bool controls_ac_q{false};
  bool controls_ac_v{false};
  bool controls_ac_angle{false};

  // DC-side controlled quantities.
  bool controls_dc_p{false};
  bool controls_dc_v_rigid{false};
  bool controls_dc_v_droop{false};

  // Grid-forming designation.
  bool is_ac_grid_forming{false};
  bool is_dc_grid_forming{false};

  // References this device provides to its island(s).
  bool provides_ac_angle_reference{false};
  bool provides_ac_voltage_reference{false};
  bool provides_dc_v_reference{false};

  // Free (released) device unknowns.
  bool ac_p_is_free{false};
  bool ac_q_is_free{false};
  bool dc_p_is_free{false};

  // Dual-side grid-forming gate (advanced, requires an explicit energy buffer).
  bool has_energy_buffer{false};
  bool allow_dual_side_grid_forming{false};

  // The seven-mode taxonomy classification of this converter.
  ACDCControlMode acdc_mode{ACDCControlMode::AC_PQ};
};

/// Derive the control role of a VSC converter from its control mode and flags.
inline DeviceControlRole resolve_device_control_role(const VSCConverter& conv) {
  DeviceControlRole r;
  r.acdc_mode = to_acdc_control_mode(conv.control_mode);
  r.has_energy_buffer = conv.has_energy_buffer;
  r.allow_dual_side_grid_forming = conv.allow_dual_side_grid_forming;

  // A converter forms the DC voltage when it actively regulates Vdc through the
  // k_vdc droop (VDC_Q / VDC_VAC / DC_V_DROOP_AC_V) or is declared DC-side
  // grid-forming.
  const bool dc_droop_mode = (conv.control_mode == ConverterMode::VDC_Q ||
                              conv.control_mode == ConverterMode::VDC_VAC ||
                              conv.control_mode == ConverterMode::DC_V_DROOP_AC_V);
  const bool forms_dc_voltage =
      (dc_droop_mode && std::abs(conv.k_vdc) > 1e-12) || conv.grid_forming;

  switch (conv.control_mode) {
    case ConverterMode::PQ_MODE:  // Mode 3: AC_PQ
      r.controls_ac_p = true;
      r.controls_ac_q = true;
      r.controls_dc_p = true;
      break;
    case ConverterMode::AC_PV:  // Mode 2: Ps + Vs
      r.controls_ac_p = true;
      r.controls_ac_v = true;
      r.controls_dc_p = true;
      r.ac_q_is_free = true;
      r.provides_ac_voltage_reference = true;
      break;
    case ConverterMode::VDC_Q:  // Mode 4/7: Udc + Qs
      r.controls_dc_v_droop = true;
      r.controls_ac_q = true;
      r.ac_p_is_free = true;
      r.provides_dc_v_reference = forms_dc_voltage;
      break;
    case ConverterMode::VDC_VAC:  // Mode 5: Udc + Vs (legacy qac=0 realization)
      r.controls_dc_v_droop = true;
      r.ac_p_is_free = true;
      r.provides_dc_v_reference = forms_dc_voltage;
      break;
    case ConverterMode::DC_V_DROOP_AC_V:  // Mode 6: droop Udc + Vs
      r.controls_dc_v_droop = true;
      r.controls_ac_v = true;
      r.ac_p_is_free = true;
      r.ac_q_is_free = true;
      r.provides_dc_v_reference = forms_dc_voltage;
      r.provides_ac_voltage_reference = true;
      break;
    case ConverterMode::AC_GRID_FORMING:  // Mode 1: δs + Vs
      r.controls_ac_angle = true;
      r.controls_ac_v = true;
      r.ac_p_is_free = true;
      r.ac_q_is_free = true;
      r.is_ac_grid_forming = true;
      r.provides_ac_angle_reference = true;
      r.provides_ac_voltage_reference = true;
      break;
  }

  // Reconcile with the explicit opt-in flags carried on the converter.
  if (conv.ac_grid_forming) {
    r.is_ac_grid_forming = true;
    r.provides_ac_angle_reference = true;
    r.provides_ac_voltage_reference = true;
  }
  if (forms_dc_voltage) {
    r.is_dc_grid_forming = true;
    r.provides_dc_v_reference = true;
  }
  return r;
}

}  // namespace hacdcpf
