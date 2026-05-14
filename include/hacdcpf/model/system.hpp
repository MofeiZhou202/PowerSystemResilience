#pragma once

#include <optional>
#include <string>
#include <vector>

#include "hacdcpf/model/bus_merge_map.hpp"
#include "hacdcpf/model/components.hpp"

namespace hacdcpf {

struct ACSystem {
  std::vector<ACBus> buses;
  std::vector<ACBranch> branches;
  std::vector<Generator> generators;
  std::vector<StaticGenerator> static_generators;
  std::vector<Load> loads;
  std::vector<FlexibleLoad> flexible_loads;
  std::vector<AsymmetricLoad> asymmetric_loads;
  std::vector<Shunt> shunts;
  std::vector<Storage> storage;
  std::vector<RenewableGen> renewable_gens;
  std::vector<PVSystem> pv_systems;
  std::vector<ExternalGrid> external_grids;
  std::vector<Transformer2W> transformers_2w;
  std::vector<Transformer3W> transformers_3w;
  std::vector<Switch> switches;
  std::vector<CircuitBreaker> circuit_breakers;
  std::vector<ChargingStation> charging_stations;
  std::vector<Charger> chargers;
  std::vector<AsynchronousMotor> motors;
  double base_mva{100.0};
  double freq_hz{50.0};
  std::string name{"AC System"};
};

struct DCSystem {
  std::vector<DCBus> buses;
  std::vector<DCBranch> branches;
  std::vector<DCLoad> loads;
  std::vector<Storage> storage;
  std::vector<StaticGenerator> static_generators;
  std::vector<StaticGeneratorDC> dc_static_generators;
  std::vector<PVArrayDC> pv_arrays;
  std::vector<DCDCConverter> dcdc_converters;
  std::vector<DCCircuitBreaker> dc_circuit_breakers;
  double base_mva{100.0};
  std::string name{"DC System"};
};

struct HybridPowerSystem {
  ACSystem ac;
  DCSystem dc;
  std::vector<VSCConverter> vsc_converters;
  std::vector<EnergyRouter> energy_routers;
  std::vector<MobileStorage> mobile_storage;
  std::vector<VirtualPowerPlant> vpps;
  std::vector<Microgrid> microgrids;
  double base_mva{100.0};
  std::string name{"Hybrid AC/DC System"};

  // Optional three-phase AC subsystem (for unbalanced analysis)
  std::optional<ThreePhaseACSystem> three_phase_ac;

  // Bus merge map populated by project_to_canonical_models() when
  // zero-impedance branches cause bus contraction.
  std::optional<BusMergeMap> bus_merge_map;

  // Branch expansion map populated by project_to_canonical_models() to
  // record which ACBranch entries were synthesised from Transformer2W/3W
  // or Switch elements.  Used to map branch-flow results back to the
  // original rich component.
  std::optional<BranchExpandMap> branch_expand_map;
};

}  // namespace hacdcpf
