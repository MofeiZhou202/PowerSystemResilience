#pragma once

#include <vector>

namespace hacdcpf {

// ═══════════════════════════════════════════════════════════════════════
// Island tracking (solver internal)
// ═══════════════════════════════════════════════════════════════════════
struct IslandInfo {
  int id{0};
  bool has_ac_slack{false};
  bool has_dc_slack{false};
  int ac_slack_bus{0};
  int dc_slack_bus{0};
  bool has_generators{false};

  // Global bus indices.
  std::vector<int> ac_buses;
  std::vector<int> dc_buses;
  std::vector<int> converters;
};

}  // namespace hacdcpf
