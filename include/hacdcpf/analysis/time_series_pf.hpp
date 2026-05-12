#pragma once
#include <string>
#include <vector>

// Time-series data types used by io and analysis modules.
// TimeSeriesData/TimeSeriesProfile are in the global hacdcpf namespace for
// backward-compatibility with the json_io serialisation functions.

namespace hacdcpf {

struct TimeSeriesProfile {
  int id{0};
  std::string name;
  std::vector<double> values;
};

struct TimeSeriesData {
  int num_steps{24};
  double step_duration_hr{1.0};
  std::vector<TimeSeriesProfile> profiles;
};

}  // namespace hacdcpf

namespace hacdcpf::analysis {

struct TimeSeriesPFSnapshot {
  int step{0};
  double time_hr{0.0};
  bool converged{false};
  std::vector<double> vm;
  std::vector<double> va;
  std::vector<double> vdc;
};

struct TimeSeriesPFResult {
  bool all_converged{false};
  int num_steps{0};
  int num_failed{0};
  std::vector<TimeSeriesPFSnapshot> snapshots;
};

struct TimeSeriesPFOptions {
  int num_threads{1};
  bool abort_on_failure{false};
};

}  // namespace hacdcpf::analysis
