#pragma once

#include <cstddef>

#include <Eigen/Core>

namespace hacdcpf::dynamics {

struct StateIndexRange {
  int offset{0};
  int size{0};

  [[nodiscard]] bool empty() const noexcept { return size <= 0; }
};

struct DynamicState {
  Eigen::VectorXd x;
  Eigen::VectorXd dxdt;
  double time_s{0.0};

  void resize(std::size_t n) {
    x = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(n));
    dxdt = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(n));
  }

  [[nodiscard]] int size() const noexcept {
    return static_cast<int>(x.size());
  }
};

}  // namespace hacdcpf::dynamics
