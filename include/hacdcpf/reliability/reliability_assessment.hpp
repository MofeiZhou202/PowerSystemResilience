#pragma once
#include <string>
#include <vector>

namespace hacdcpf::analysis {

struct ReliabilityComponent {
  int index{0};
  double failure_rate_per_year{0.0};
  double repair_time_hr{0.0};
};

struct ReliabilityAssessmentOptions {
  int num_simulations{10000};
  double time_horizon_years{1.0};
  int random_seed{42};
};

struct ReliabilityAssessmentResult {
  bool completed{false};
  double lolp{0.0};      ///< Loss of Load Probability
  double eens_mwh{0.0};  ///< Expected Energy Not Supplied (MWh/year)
  double saidi_hr{0.0};  ///< System Average Interruption Duration Index
  double saifi{0.0};     ///< System Average Interruption Frequency Index
  int num_simulations{0};
  std::string status;
};

}  // namespace hacdcpf::analysis
