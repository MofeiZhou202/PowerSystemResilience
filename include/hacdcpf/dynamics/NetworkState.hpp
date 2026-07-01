#pragma once

#include <complex>

#include <Eigen/Core>

namespace hacdcpf::dynamics {

struct NetworkState {
  Eigen::VectorXcd Vac_abc;
  Eigen::VectorXd Vdc;

  Eigen::VectorXcd Iac_abc;
  Eigen::VectorXd Idc;

  void resize(int ac_phase_nodes, int dc_buses) {
    Vac_abc = Eigen::VectorXcd::Zero(ac_phase_nodes);
    Iac_abc = Eigen::VectorXcd::Zero(ac_phase_nodes);
    Vdc = Eigen::VectorXd::Ones(dc_buses);
    Idc = Eigen::VectorXd::Zero(dc_buses);
  }

  [[nodiscard]] int ac_phase_nodes() const noexcept {
    return static_cast<int>(Vac_abc.size());
  }

  [[nodiscard]] int dc_bus_count() const noexcept {
    return static_cast<int>(Vdc.size());
  }
};

}  // namespace hacdcpf::dynamics
