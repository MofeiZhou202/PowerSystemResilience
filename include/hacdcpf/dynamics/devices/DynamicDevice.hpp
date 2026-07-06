#pragma once

#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Sparse>

#include "hacdcpf/dynamics/DynamicEvent.hpp"
#include "hacdcpf/dynamics/DynamicResults.hpp"
#include "hacdcpf/dynamics/DynamicStamp.hpp"
#include "hacdcpf/dynamics/DynamicState.hpp"
#include "hacdcpf/dynamics/NetworkState.hpp"
#include "hacdcpf/power_flow/power_flow_result.hpp"

namespace hacdcpf::dynamics {

struct DynamicJacobianContext {
  int n_x{0};
  int n_ac{0};
  int n_dc{0};
  int ac_real_offset{0};
  int ac_imag_offset{0};
  int dc_offset{0};
  int total_size{0};
  double dt{0.0};
  double theta{1.0};
  bool include_differential_derivatives{false};

  [[nodiscard]] bool validStateIndex(int idx) const noexcept {
    return idx >= 0 && idx < n_x;
  }

  [[nodiscard]] bool validAcNode(int node) const noexcept {
    return node >= 0 && node < n_ac;
  }

  [[nodiscard]] bool validDcNode(int node) const noexcept {
    return node >= 0 && node < n_dc;
  }

  [[nodiscard]] int acRealRow(int node) const noexcept {
    return ac_real_offset + node;
  }

  [[nodiscard]] int acImagRow(int node) const noexcept {
    return ac_imag_offset + node;
  }

  [[nodiscard]] int acRealCol(int node) const noexcept {
    return ac_real_offset + node;
  }

  [[nodiscard]] int acImagCol(int node) const noexcept {
    return ac_imag_offset + node;
  }

  [[nodiscard]] int dcRow(int node) const noexcept {
    return dc_offset + node;
  }

  [[nodiscard]] int dcCol(int node) const noexcept {
    return dc_offset + node;
  }
};

class DynamicDevice {
 public:
  virtual ~DynamicDevice() = default;

  virtual void assignStateIndices(int& offset) = 0;

  virtual void initializeFromPowerFlow(const PowerFlowResult& pf,
                                       DynamicState& x,
                                       NetworkState& y) = 0;

  virtual bool trimToNetworkEquilibrium(DynamicState& x,
                                        NetworkState& y) {
    (void)x;
    (void)y;
    return false;
  }

  virtual void computeDerivatives(double t,
                                  const DynamicState& x,
                                  const NetworkState& y,
                                  Eigen::Ref<Eigen::VectorXd> dxdt) const = 0;

  virtual void maskSlowStateResidual(Eigen::Ref<Eigen::VectorXd> dxdt) const {
    (void)dxdt;
  }

  virtual void stamp(double t,
                     const DynamicState& x,
                     const NetworkState& y,
                     DynamicStamp& stamp) const = 0;

  virtual void updateAlgebraicOutputs(const DynamicState& x,
                                      const NetworkState& y) {
    (void)x;
    (void)y;
  }

  [[nodiscard]] virtual DynamicDeviceOutput output(const DynamicState& x,
                                                   const NetworkState& y) const {
    (void)x;
    (void)y;
    DynamicDeviceOutput out;
    out.name = name();
    out.type = type();
    out.component_index = componentIndex();
    out.model_standard = modelStandard();
    out.model_name = modelName();
    out.parameter_set = parameterSet();
    out.model_profiles = modelProfiles();
    return out;
  }

  virtual void handleEvent(const DynamicEvent& event,
                           DynamicState& x,
                           NetworkState& y) {
    (void)event;
    (void)x;
    (void)y;
  }

  virtual void addJacobian(double t,
                           const DynamicState& x,
                           const NetworkState& y,
                           std::vector<Eigen::Triplet<double>>& triplets) const {
    (void)t;
    (void)x;
    (void)y;
    (void)triplets;
  }

  virtual void addJacobian(double t,
                           const DynamicState& x,
                           const NetworkState& y,
                           const DynamicJacobianContext& context,
                           std::vector<Eigen::Triplet<double>>& triplets) const {
    (void)context;
    addJacobian(t, x, y, triplets);
  }

  [[nodiscard]] virtual std::string name() const = 0;
  [[nodiscard]] virtual std::string type() const = 0;
  [[nodiscard]] virtual int componentIndex() const = 0;
  [[nodiscard]] virtual std::string modelStandard() const { return "HACDCPF"; }
  [[nodiscard]] virtual std::string modelName() const { return type(); }
  [[nodiscard]] virtual std::string parameterSet() const { return {}; }
  [[nodiscard]] virtual std::vector<DynamicModelProfile> modelProfiles() const {
    DynamicModelProfile profile;
    profile.standard = modelStandard();
    profile.model_name = modelName();
    profile.parameter_set = parameterSet();
    return {profile};
  }
};

}  // namespace hacdcpf::dynamics
