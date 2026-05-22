#pragma once
#include <string>
#include <vector>

#include "hacdcpf/model/hybrid_power_system.hpp"

namespace hacdcpf::analysis {

struct DistributionResilienceOptions {
  /// Fault scenario duration in hours
  double fault_duration_hr{1.0};
  /// Load curtailment penalty weight
  double curtailment_penalty{1000.0};
  /// Enable microgrid islanding
  bool enable_islanding{true};
  /// Enable storage dispatch during outages
  bool enable_storage_dispatch{true};
  /// Maximum restoration time (hours)
  double max_restoration_hr{8.0};
  bool verbose{false};
};

struct ResilienceBusCurtailment {
  int bus_index{0};
  double curtailed_load_mw{0.0};
  double duration_hr{0.0};
  double energy_not_served_mwh{0.0};
};

struct DistributionResilienceResult {
  bool completed{false};
  double total_ens_mwh{0.0};     ///< Total energy not served (MWh)
  double max_curtailment_mw{0.0};///< Peak curtailment power
  double restoration_time_hr{0.0};
  std::vector<ResilienceBusCurtailment> bus_curtailments;
  std::string status;
};

DistributionResilienceResult run_distribution_resilience_assessment(
    const HybridPowerSystem& sys,
    const DistributionResilienceOptions& opt = {});

}  // namespace hacdcpf::analysis
