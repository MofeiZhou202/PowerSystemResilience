#pragma once

#include <array>
#include <string>

#include <Eigen/Core>

#include "hacdcpf/model/converter_components.hpp"

namespace hacdcpf::powerflow {

constexpr int kVSCLimitStateSize = 6;

struct VSCLimitCommand {
  double p_ref_pu{0.0};
  double q_ref_pu{0.0};
  bool droop_saturated{false};
};

struct VSCLimitEvaluation {
  Eigen::Matrix<double, kVSCLimitStateSize, 1> equation{
      Eigen::Matrix<double, kVSCLimitStateSize, 1>::Zero()};
  Eigen::Matrix<double, kVSCLimitStateSize, kVSCLimitStateSize> jacobian{
      Eigen::Matrix<double, kVSCLimitStateSize, kVSCLimitStateSize>::Zero()};
  Eigen::Matrix<double, kVSCLimitStateSize, 1> derivative_vm{
      Eigen::Matrix<double, kVSCLimitStateSize, 1>::Zero()};
  Eigen::Matrix<double, kVSCLimitStateSize, 1> derivative_va{
      Eigen::Matrix<double, kVSCLimitStateSize, 1>::Zero()};
  Eigen::Matrix<double, kVSCLimitStateSize, 1> derivative_vdc{
      Eigen::Matrix<double, kVSCLimitStateSize, 1>::Zero()};
  double current_pu{0.0};
  double current_margin_pu{0.0};
  double complementarity_residual{0.0};
  bool current_limit_active{false};
  bool droop_saturated{false};
};

bool supports_vsc_limit_ncp(const VSCConverter& converter);
bool uses_gfm_limit_ncp(const VSCConverter& converter);

VSCLimitCommand vsc_limit_command(const VSCConverter& converter,
                                   double vdc_pu,
                                   double base_mva);

Eigen::Matrix<double, kVSCLimitStateSize, 1> initialize_vsc_limit_state(
    const VSCConverter& converter,
    double vm_pu,
    double va_rad,
    double vdc_pu,
    double base_mva,
    LossModelType loss_model);

VSCLimitEvaluation evaluate_vsc_limit_ncp(
    const VSCConverter& converter,
    double vm_pu,
    double va_rad,
    double vdc_pu,
    double base_mva,
    LossModelType loss_model,
    const Eigen::Matrix<double, kVSCLimitStateSize, 1>& state,
    double ncp_mu = 0.0);

const char* vsc_current_limit_priority_str(VSCCurrentLimitPriority priority);
VSCCurrentLimitPriority vsc_current_limit_priority_from_str(
    const std::string& value);

}  // namespace hacdcpf::powerflow
