#pragma once
#include <string>
#include <vector>

#include "hacdcpf/model/system.hpp"

namespace hacdcpf::analysis {

struct DistributionPFOptions {
  int max_iterations{100};
  double convergence_tolerance{1e-6};
  bool backward_forward_sweep{true};
};

struct DistributionBusResult {
  int bus_index{0};
  double vm_pu{1.0};
  double va_deg{0.0};
  double p_injection_mw{0.0};
  double q_injection_mvar{0.0};
};

struct DistributionPFResult {
  bool converged{false};
  int iterations{0};
  double residual{0.0};
  std::vector<DistributionBusResult> buses;
  std::string status;
};

DistributionPFResult solve_distribution_power_flow(
    const HybridPowerSystem& sys,
    const DistributionPFOptions& opt = {});

}  // namespace hacdcpf::analysis
