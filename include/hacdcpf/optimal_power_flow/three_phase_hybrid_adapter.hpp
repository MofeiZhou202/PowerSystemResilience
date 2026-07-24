#pragma once

#include <array>
#include <string>
#include <vector>

#include "hacdcpf/model/hybrid_power_system.hpp"
#include "hacdcpf/optimal_power_flow/three_phase_hybrid_opf.hpp"

namespace hacdcpf::opf::phase_hybrid {

struct ThreePhaseHybridAdapterOptions {
  bool include_shunts{true};
  bool enforce_converter_capacity{true};
  bool enforce_converter_current{true};
  bool allow_constant_power_load_equivalent{true};
  double vuf_max{0.03};
};

struct ThreePhaseHybridGeneratorBinding {
  int component_index{0};
  int bus_id{0};
  int phase{0};
  std::string name;
  std::string source_type;
  double ramp_up_mw_min{0.0};
  double ramp_down_mw_min{0.0};
};

struct ThreePhaseHybridModel {
  ThreePhaseHybridOPFCase opf;

  // Stable component IDs are kept alongside solver positions so results can
  // be attributed without confusing bus IDs, vector positions, and nodes.
  std::vector<int> ac_bus_ids;
  std::vector<std::array<int, 3>> bus_phase_nodes;
  std::vector<int> dc_bus_ids;
  std::vector<int> vsc_component_indices;
  std::vector<ThreePhaseHybridGeneratorBinding> generator_bindings;
  std::vector<std::string> model_limitations;

  int phase_node(int ac_bus_id, int phase) const;
};

/// Convert a rich three-phase AC/DC HybridPowerSystem into the monolithic
/// nonlinear phase-domain OPF representation. Unsupported load voltage
/// characteristics can be represented by their imported per-phase constant-
/// power operating-point equivalent when explicitly enabled in options.
ThreePhaseHybridModel build_three_phase_hybrid_model(
    const HybridPowerSystem& system,
    const ThreePhaseHybridAdapterOptions& options = {});

}  // namespace hacdcpf::opf::phase_hybrid
