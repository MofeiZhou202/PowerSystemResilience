#pragma once

#include <utility>

#include <Eigen/Core>

#include "hacdcpf/assembly/solver_data.hpp"

namespace hacdcpf::powerflow {

// ── Cross-coupling Jacobian structs (Direction 1) ──────────────────────

/// Derivatives of VSC AC-side injection w.r.t. DC voltage.
struct ConverterJacobianAC {
  double dpac_dvdc{0.0};
  double dqac_dvdc{0.0};
};

/// Derivatives of VSC DC-side injection w.r.t. DC voltage (converter contribution only).
struct ConverterJacobianDC {
  double dpdc_dvdc{0.0};
};

/// Derivatives of VSC DC-side injection w.r.t. AC terminal voltage magnitude.
/// Arises from the AC-conduction-loss coupling: ploss_AC = r_conv_ac * pac² / Vm_AC².
/// The DC bus must supply these losses, so ∂pdc/∂Vm_AC = 2·r·pac²/Vm_AC³.
struct ConverterJacobianDCVmAC {
  double dpdc_dvm_ac{0.0};
};

/// Derivatives of DCDC converter injections w.r.t. input/output DC voltages.
struct DCDCJacobian {
  double dpin_dvdc_in{0.0};
  double dpin_dvdc_out{0.0};
  double dpout_dvdc_in{0.0};
  double dpout_dvdc_out{0.0};
};

/// Shared DCDC power transfer state for the PF convention.
/// p_out_ref_pu is the droop-adjusted output-side power command before I2R loss.
/// p_out_pu is the delivered output-side injection after I2R loss.
struct DCDCPowerTransfer {
  double eta{1.0};
  double p_out_ref_pu{0.0};
  double p_in_pu{0.0};
  double i2r_loss_pu{0.0};
  double p_out_pu{0.0};
  double dpin_dpout_ref{1.0};
  double dpout_ref_dvdc_out{0.0};
  double dpin_dvdc_out{0.0};
  double dpout_dvdc_in{0.0};
  double dpout_dvdc_out{0.0};
};

ConverterJacobianAC converter_ac_jacobian_vdc(const VSCConverter& conv,
                                               const Eigen::VectorXd& vdc,
                                               double base_mva,
                                               LossModelType loss_model);

ConverterJacobianDC converter_dc_jacobian_vdc(const VSCConverter& conv,
                                               const Eigen::VectorXd& vdc,
                                               double base_mva,
                                               LossModelType loss_model);

/// Jacobian of DC power balance w.r.t. AC terminal voltage magnitude.
/// @param pac0  Nominal AC injection (pre-computed, in pu on base_mva).
ConverterJacobianDCVmAC converter_dc_jacobian_vm_ac(const VSCConverter& conv,
                                                     const Eigen::VectorXd& vm,
                                                     double pac0,
                                                     double base_mva);

DCDCJacobian dcdc_jacobian_vdc(const DCDCConverter& dcdc,
                                const Eigen::VectorXd& vdc,
                                double base_mva);

DCDCPowerTransfer dcdc_power_transfer(const DCDCConverter& dcdc,
                                      const Eigen::VectorXd& vdc,
                                      double base_mva);

double converter_loss(const VSCConverter& conv,
                      double p,
                      double vdc,
                      double base_mva,
                      LossModelType loss_model);

std::pair<double, double> converter_loss_jacobian(const VSCConverter& conv,
                                                  double p,
                                                  double vdc,
                                                  double base_mva,
                                                  LossModelType loss_model);

std::pair<double, double> converter_ac_injection(const VSCConverter& conv,
                                                 const Eigen::VectorXd& vm,
                                                 const Eigen::VectorXd& va,
                                                 const Eigen::VectorXd& vdc,
                                                 double base_mva,
                                                 LossModelType loss_model);

double converter_dc_injection(const VSCConverter& conv,
                              const Eigen::VectorXd& vm,
                              const Eigen::VectorXd& va,
                              const Eigen::VectorXd& vdc,
                              double base_mva,
                              LossModelType loss_model);

}  // namespace hacdcpf::powerflow
