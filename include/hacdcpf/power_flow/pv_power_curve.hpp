#pragma once

#include <cmath>

#include "hacdcpf/model/ac_components.hpp"

namespace hacdcpf::powerflow {

// PV array empirical power model (Julia DistributionPowerFlow parity).
// P(V) = Isc * (1 - (V/Voc)^a)^b * (1 - c*(V/Vmpp)^2)
// Constants: a = 10.0, b = 0.547596, c = 0.023812

inline double pv_module_current(double isc, double voc, double vmpp, double v_terminal) {
  constexpr double a = 10.0;
  constexpr double b = 0.547596;
  constexpr double c = 0.023812;

  const double v_ratio = v_terminal / voc;
  const double v_mpp_ratio = v_terminal / vmpp;
  return isc * std::pow(1.0 - std::pow(v_ratio, a), b)
             * (1.0 - c * v_mpp_ratio * v_mpp_ratio);
}

// Compute total PV system output in MW.
// At MPPT: V_terminal = Vmpp.
// Falls back to pv.p_mw when array parameters are not specified.
inline double compute_pv_power_mw(const PVSystem& pv) {
  if (pv.voc <= 0.0 || pv.isc <= 0.0 || pv.vmpp <= 0.0) {
    return pv.p_mw;
  }

  const double i_module = pv_module_current(pv.isc, pv.voc, pv.vmpp, pv.vmpp);
  const double p_module_w = i_module * pv.vmpp;  // single module power in W
  const double p_array_w = p_module_w
                           * static_cast<double>(pv.num_series)
                           * static_cast<double>(pv.num_parallel);
  const double irr_scale = pv.irradiance / 1000.0;
  return p_array_w * irr_scale * pv.inverter_eff / 1.0e6;  // W → MW
}

}  // namespace hacdcpf::powerflow
