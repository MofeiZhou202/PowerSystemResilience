#include "hacdcpf/power_flow/converter_model.hpp"

#include <algorithm>
#include <cmath>

namespace hacdcpf::powerflow {

double converter_loss(const VSCConverter& conv,
                      double p,
                      double vdc,
                      double base_mva,
                      LossModelType loss_model) {
  if (loss_model == LossModelType::Linear) {
    return (1.0 - conv.eta) * std::abs(p);
  }

  const double a = conv.loss_mw / base_mva;    // fixed / no-load loss
  const double b = conv.loss_percent / 100.0;   // switching loss ∝ I
  const double c = 1.0 - conv.eta;              // conduction loss ∝ I²
  const double vdc_safe = std::max(vdc, 0.1);
  const double current = std::abs(p) / vdc_safe;
  return a + b * current + c * current * current;
}

std::pair<double, double> converter_loss_jacobian(const VSCConverter& conv,
                                                  double p,
                                                  double vdc,
                                                  double base_mva,
                                                  LossModelType loss_model) {
  if (loss_model == LossModelType::Linear) {
    const double sign = (p >= 0.0) ? 1.0 : -1.0;
    return {(1.0 - conv.eta) * sign, 0.0};
  }

  const double a = conv.loss_mw / base_mva;    // fixed / no-load loss
  const double b = conv.loss_percent / 100.0;   // switching loss ∝ I
  const double c = 1.0 - conv.eta;              // conduction loss ∝ I²
  const double vdc_safe = std::max(vdc, 0.1);
  const double abs_p = std::abs(p);
  const double sign = (p >= 0.0) ? 1.0 : -1.0;

  // loss = a + b*I + c*I², where I = |p| / vdc
  // d(loss)/dp:   constant term 'a' has zero derivative w.r.t. p and vdc.
  const double dploss_dp = b * sign / vdc_safe + 2.0 * c * p / (vdc_safe * vdc_safe);
  const double dploss_dvdc =
      -b * abs_p / (vdc_safe * vdc_safe) - 2.0 * c * p * p / (vdc_safe * vdc_safe * vdc_safe);
  (void)a;

  return {dploss_dp, dploss_dvdc};
}

std::pair<double, double> converter_ac_injection(const VSCConverter& conv,
                                                 const Eigen::VectorXd& vm,
                                                 const Eigen::VectorXd& va,
                                                 const Eigen::VectorXd& vdc,
                                                 double base_mva,
                                                 LossModelType loss_model) {
  (void)vm;
  (void)va;

  const double pset = conv.p_set_mw / base_mva;
  const double qset = conv.q_set_mvar / base_mva;
  const int dc_idx = conv.bus_dc - 1;
  const double vdc_bus = (dc_idx >= 0 && dc_idx < vdc.size()) ? vdc[dc_idx] : 1.0;

  if (conv.control_mode == ConverterMode::PQ_MODE) {
    return {pset, qset};
  }

  const double p_transfer = conv.k_vdc * (vdc_bus * vdc_bus - conv.v_dc_set_pu * conv.v_dc_set_pu);
  const double ploss = converter_loss(conv, p_transfer, vdc_bus, base_mva, loss_model);
  const double pac = p_transfer - ploss;

  if (conv.control_mode == ConverterMode::VDC_Q) {
    return {pac, qset};
  }
  return {pac, 0.0};
}

double converter_dc_injection(const VSCConverter& conv,
                              const Eigen::VectorXd& vm,
                              const Eigen::VectorXd& va,
                              const Eigen::VectorXd& vdc,
                              double base_mva,
                              LossModelType loss_model) {
  (void)va;

  const double pset = conv.p_set_mw / base_mva;
  const int dc_idx = conv.bus_dc - 1;
  const double vdc_bus = (dc_idx >= 0 && dc_idx < vdc.size()) ? vdc[dc_idx] : 1.0;

  double pdc_base = 0.0;
  if (conv.control_mode == ConverterMode::PQ_MODE) {
    const double ploss = converter_loss(conv, pset, vdc_bus, base_mva, loss_model);
    pdc_base = -(pset + ploss);
  } else {
    pdc_base = -conv.k_vdc * (vdc_bus * vdc_bus - conv.v_dc_set_pu * conv.v_dc_set_pu);
  }

  return pdc_base;
}

// ═══════════════════════════════════════════════════════════════════════
// Cross-coupling Jacobian functions (Direction 1)
// ═══════════════════════════════════════════════════════════════════════

ConverterJacobianAC converter_ac_jacobian_vdc(const VSCConverter& conv,
                                               const Eigen::VectorXd& vdc,
                                               double base_mva,
                                               LossModelType loss_model) {
  ConverterJacobianAC jac;
  if (!conv.in_service) return jac;

  const int dc_idx = conv.bus_dc - 1;
  const double vdc_bus = (dc_idx >= 0 && dc_idx < vdc.size()) ? vdc[dc_idx] : 1.0;

  if (conv.control_mode == ConverterMode::PQ_MODE) {
    // AC injection is constant (pset, qset) — no Vdc dependence.
    return jac;
  }

  // VDC_Q or VDC_VAC: pac = p_transfer - loss(p_transfer, Vdc)
  // where p_transfer = k*(Vdc² - Vdc_set²)
  const double k = conv.k_vdc;
  const double p_transfer = k * (vdc_bus * vdc_bus - conv.v_dc_set_pu * conv.v_dc_set_pu);
  const auto [dploss_dp, dploss_dvdc] =
      converter_loss_jacobian(conv, p_transfer, vdc_bus, base_mva, loss_model);

  // dp_transfer/dVdc = 2*k*Vdc
  const double dp_transfer_dvdc = 2.0 * k * vdc_bus;

  // pac = p_transfer - loss(p_transfer, Vdc)
  // dpac/dVdc = dp_transfer/dVdc * (1 - dploss/dp) - dploss/dVdc
  jac.dpac_dvdc = dp_transfer_dvdc * (1.0 - dploss_dp) - dploss_dvdc;

  // Reactive: VDC_Q uses qset (constant), VDC_VAC uses 0 (constant).
  jac.dqac_dvdc = 0.0;
  return jac;
}

ConverterJacobianDC converter_dc_jacobian_vdc(const VSCConverter& conv,
                                               const Eigen::VectorXd& vdc,
                                               double base_mva,
                                               LossModelType loss_model) {
  ConverterJacobianDC jac;
  if (!conv.in_service) return jac;

  const int dc_idx = conv.bus_dc - 1;
  const double vdc_bus = (dc_idx >= 0 && dc_idx < vdc.size()) ? vdc[dc_idx] : 1.0;

  if (conv.control_mode == ConverterMode::PQ_MODE) {
    // pdc = -(pset + loss(pset, Vdc))  →  dpdc/dVdc = -dploss/dVdc
    const double pset = conv.p_set_mw / base_mva;
    const auto [dploss_dp, dploss_dvdc] =
        converter_loss_jacobian(conv, pset, vdc_bus, base_mva, loss_model);
    jac.dpdc_dvdc = -dploss_dvdc;
  } else {
    // VDC modes: pdc = -k*(Vdc² - Vdc_set²)  →  dpdc/dVdc = -2*k*Vdc
    jac.dpdc_dvdc = -2.0 * conv.k_vdc * vdc_bus;
  }
  return jac;
}

DCDCJacobian dcdc_jacobian_vdc(const DCDCConverter& dcdc,
                                const Eigen::VectorXd& vdc,
                                double base_mva) {
  DCDCJacobian jac;
  if (!dcdc.in_service) return jac;
  const DCDCPowerTransfer transfer = dcdc_power_transfer(dcdc, vdc, base_mva);
  jac.dpin_dvdc_in = 0.0;
  jac.dpin_dvdc_out = transfer.dpin_dvdc_out;
  jac.dpout_dvdc_in = transfer.dpout_dvdc_in;
  jac.dpout_dvdc_out = transfer.dpout_dvdc_out;

  return jac;
}

DCDCPowerTransfer dcdc_power_transfer(const DCDCConverter& dcdc,
                                      const Eigen::VectorXd& vdc,
                                      double base_mva) {
  DCDCPowerTransfer out;
  if (!dcdc.in_service) {
    return out;
  }

  const int bin = dcdc.bus_in - 1;
  const int bout = dcdc.bus_out - 1;
  const int ndc = static_cast<int>(vdc.size());

  out.eta = std::clamp(dcdc.eta, 0.01, 1.0);
  out.p_out_ref_pu = dcdc.p_ref_mw / base_mva;
  if (dcdc.control_mode == DCDCControlMode::Droop && dcdc.k_droop != 0.0) {
    const double vdc_out = (bout >= 0 && bout < ndc) ? vdc[bout] : dcdc.v_ref_pu;
    out.p_out_ref_pu += dcdc.k_droop * (vdc_out - dcdc.v_ref_pu);
    out.dpout_ref_dvdc_out = dcdc.k_droop;
  }

  out.dpin_dpout_ref = (out.p_out_ref_pu >= 0.0) ? (1.0 / out.eta) : out.eta;
  out.p_in_pu = out.p_out_ref_pu * out.dpin_dpout_ref;
  out.dpin_dvdc_out = out.dpin_dpout_ref * out.dpout_ref_dvdc_out;

  double dloss_dpin = 0.0;
  if (dcdc.r_eq_pu > 0.0 && bin >= 0 && bin < ndc) {
    const double vdc_in = std::max(vdc[bin], 0.1);
    const double vdc2 = vdc_in * vdc_in;
    out.i2r_loss_pu = dcdc.r_eq_pu * out.p_in_pu * out.p_in_pu / vdc2;
    dloss_dpin = 2.0 * dcdc.r_eq_pu * out.p_in_pu / vdc2;
    out.dpout_dvdc_in = 2.0 * dcdc.r_eq_pu * out.p_in_pu * out.p_in_pu / (vdc2 * vdc_in);
  }

  out.p_out_pu = out.p_out_ref_pu - out.i2r_loss_pu;
  out.dpout_dvdc_out = out.dpout_ref_dvdc_out - dloss_dpin * out.dpin_dvdc_out;
  return out;
}

ConverterJacobianDCVmAC converter_dc_jacobian_vm_ac(const VSCConverter& conv,
                                                     const Eigen::VectorXd& vm,
                                                     double pac0,
                                                     double /*base_mva*/) {
  ConverterJacobianDCVmAC jac;
  (void)conv; (void)vm; (void)pac0;
  // r_conv_ac_pu removed from VSCConverter; this Jacobian entry is always zero.
  return jac;
}

}  // namespace hacdcpf::powerflow
