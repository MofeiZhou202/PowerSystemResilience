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

  [[nodiscard]] virtual std::string name() const = 0;
  [[nodiscard]] virtual std::string type() const = 0;
  [[nodiscard]] virtual int componentIndex() const = 0;
};

}  // namespace hacdcpf::dynamics
