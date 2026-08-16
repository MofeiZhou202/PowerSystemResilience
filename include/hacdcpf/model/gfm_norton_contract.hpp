#pragma once

#include <complex>
#include <numbers>

#include "hacdcpf/model/converter_components.hpp"

namespace hacdcpf::model {

struct GFMNortonParameters {
  double internal_voltage_pu{1.0};
  double internal_angle_rad{0.0};
  double virtual_r_pu{0.0};
  double virtual_x_pu{0.0};

  std::complex<double> internal_voltage() const {
    return std::polar(internal_voltage_pu, internal_angle_rad);
  }

  std::complex<double> virtual_impedance() const {
    return {virtual_r_pu, virtual_x_pu};
  }
};

inline GFMNortonParameters resolve_gfm_norton_parameters(
    const VSCConverter& converter) {
  const bool has_explicit_internal_reference =
      converter.gfm_internal_voltage_set_pu > 0.0 ||
      converter.gfm_internal_angle_set_deg != 0.0;
  return GFMNortonParameters{
      .internal_voltage_pu = converter.gfm_internal_voltage_set_pu > 0.0
          ? converter.gfm_internal_voltage_set_pu
          : converter.v_ac_set_pu,
      .internal_angle_rad =
          (has_explicit_internal_reference
               ? converter.gfm_internal_angle_set_deg
               : converter.v_ac_angle_set_deg) *
          std::numbers::pi / 180.0,
      .virtual_r_pu = converter.gfm_virtual_r_pu != 0.0
          ? converter.gfm_virtual_r_pu
          : converter.r_conv_ac_pu,
      .virtual_x_pu = converter.gfm_virtual_x_pu != 0.0
          ? converter.gfm_virtual_x_pu
          : converter.x_sc_pu};
}

}  // namespace hacdcpf::model
