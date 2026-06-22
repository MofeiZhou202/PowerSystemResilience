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
  double pac = 0.0;  // AC-side injection used by the conduction-loss coupling below.
  if (conv.control_mode == ConverterMode::PQ_MODE) {
    const double ploss = converter_loss(conv, pset, vdc_bus, base_mva, loss_model);
    pdc_base = -(pset + ploss);
    pac = pset;
  } else {
    const double p_transfer =
        conv.k_vdc * (vdc_bus * vdc_bus - conv.v_dc_set_pu * conv.v_dc_set_pu);
    pdc_base = -p_transfer;
    const double ploss = converter_loss(conv, p_transfer, vdc_bus, base_mva, loss_model);
    pac = p_transfer - ploss;
  }

  // AC-side conduction loss supplied by the DC bus (multi-converter model §2.3/3.1.2):
  //   ploss_AC = r_conv_ac_pu · pac² / Vm_AC²
  // The converter draws this extra power from the DC side, so its DC-bus injection
  // becomes more negative.  Opt-in: r_conv_ac_pu = 0 (default) leaves legacy behaviour
  // unchanged and keeps this term out of the residual/Jacobian.
  if (conv.r_conv_ac_pu > 0.0) {
    const int ac_idx = conv.bus_ac - 1;
    const double vm_ac =
        (ac_idx >= 0 && ac_idx < vm.size()) ? std::max(vm[ac_idx], 0.1) : 1.0;
    pdc_base -= conv.r_conv_ac_pu * pac * pac / (vm_ac * vm_ac);
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

DCDCDutyResult dcdc_duty_ratio(const DCDCConverter& dcdc, double v_in, double v_out) {
  DCDCDutyResult r;
  // Generic topology imposes no duty-ratio law -> nothing to evaluate.
  if (dcdc.topology == DCDCTopology::Generic) return r;
  if (!(v_in > 1e-9) || !(v_out > 1e-9)) return r;  // undefined for non-positive Vdc
  r.voltage_ratio = v_out / v_in;
  r.defined = true;
  switch (dcdc.topology) {
    case DCDCTopology::Buck:
      // Ideal CCM Buck: Vout = D·Vin (step-down, D in (0,1]).
      r.duty = v_out / v_in;
      break;
    case DCDCTopology::Boost:
      // Ideal CCM Boost: Vout = Vin/(1-D) (step-up).
      r.duty = 1.0 - v_in / v_out;
      break;
    case DCDCTopology::BuckBoost:
      // Non-inverting Buck-Boost: Vout/Vin = D/(1-D).
      r.duty = v_out / (v_in + v_out);
      break;
    case DCDCTopology::Isolated: {
      // Isolated: Vout = n·M(D)·Vin -> report the modulation gain M.
      const double n = (std::abs(dcdc.n_ratio) > 1e-9) ? dcdc.n_ratio : 1.0;
      r.duty = v_out / (n * v_in);
      break;
    }
    default:
      r.defined = false;
      return r;
  }
  r.feasible = std::isfinite(r.duty) && r.duty >= dcdc.d_min - 1e-9 &&
               r.duty <= dcdc.d_max + 1e-9;
  return r;
}

ConverterJacobianDCVmAC converter_dc_jacobian_vm_ac(const VSCConverter& conv,
                                                     const Eigen::VectorXd& vm,
                                                     double pac0,
                                                     double /*base_mva*/) {
  ConverterJacobianDCVmAC jac;
  if (!conv.in_service || conv.r_conv_ac_pu <= 0.0) return jac;
  // DC injection carries the AC conduction loss: pdc -= r·pac²/Vm_AC².
  // Hence ∂pdc/∂Vm_AC = +2·r·pac² / Vm_AC³ (loss falls as the AC voltage rises).
  const int ac_idx = conv.bus_ac - 1;
  const double vm_ac =
      (ac_idx >= 0 && ac_idx < vm.size()) ? std::max(vm[ac_idx], 0.1) : 1.0;
  jac.dpdc_dvm_ac =
      2.0 * conv.r_conv_ac_pu * pac0 * pac0 / (vm_ac * vm_ac * vm_ac);
  return jac;
}

}  // namespace hacdcpf::powerflow
