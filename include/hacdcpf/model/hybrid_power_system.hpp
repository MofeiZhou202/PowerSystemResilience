#pragma once

/// model/hybrid_power_system.hpp
/// ==============================
/// Top-level power system container types: ACSystem, DCSystem,
/// HybridPowerSystem, VirtualPowerPlant, Microgrid.
/// Replaces: system.hpp, aggregation.hpp.

#include <optional>
#include <string>
#include <vector>

#include "hacdcpf/model/ac_components.hpp"
#include "hacdcpf/model/converter_components.hpp"
#include "hacdcpf/model/dc_components.hpp"
#include "hacdcpf/model/enums/grid_enums.hpp"
#include "hacdcpf/projection/canonical_network.hpp"

namespace hacdcpf {

// ═══════════════════════════════════════════════════════════════════════
// Virtual Power Plant (aggregation of DERs)
// ═══════════════════════════════════════════════════════════════════════
struct VirtualPowerPlant {
  int index{0};
  std::string name;
  std::string description;
  int aggregation_bus{0};
  bool in_service{true};

  int n_pv_systems{0};
  int n_wind_turbines{0};
  int n_battery_systems{0};
  int n_ev_chargers{0};
  int n_controllable_loads{0};
  int n_chp{0};
  int n_biomass{0};
  int n_thermal_storage{0};
  int n_hvac{0};
  int n_industrial{0};

  double p_generation_sum_mw{0.0};
  double e_storage_sum_mwh{0.0};
  double p_load_controllable_mw{0.0};
  double p_pv_sum_mw{0.0};
  double p_wind_sum_mw{0.0};

  double p_regulation_up_mw{0.0};
  double p_regulation_down_mw{0.0};

  double p_output_mw{0.0};
  double q_output_mvar{0.0};
  double pmax_mw{0.0};
  double pmin_mw{0.0};
  double ramp_up_max_mw_min{0.0};
  double ramp_down_max_mw_min{0.0};

  double mtbf_hours{0.0};
  double mttr_hours{0.0};
  double t_scheduled_hr{0.0};
};

// ═══════════════════════════════════════════════════════════════════════
// Microgrid
// ═══════════════════════════════════════════════════════════════════════
struct Microgrid {
  int index{0};
  std::string name;
  std::string description;
  bool in_service{true};

  int aggregation_bus{0};
  std::vector<int> internal_buses;

  MicrogridMode operating_mode{MicrogridMode::GridConnected};
  bool islanding_capability{false};
  bool auto_reconnection{false};

  double p_exchange_max_mw{0.0};
  double p_exchange_min_mw{0.0};
  double p_import_max_mw{0.0};
  double p_export_max_mw{0.0};
  double p_exchange_mw{0.0};

  double total_generation_mw{0.0};
  double total_storage_mwh{0.0};
  double total_load_mw{0.0};
  double capacity_mw{0.0};
  double peak_load_mw{0.0};
  double total_dg_capacity_mw{0.0};
  double total_diesel_capacity_mw{0.0};

  double f_set_hz{50.0};
  double v_set_pu{1.0};
  double k_droop{0.0};
  double p_set_mw{0.0};
  std::string control_area;

  double f_max_hz{52.0};
  double f_min_hz{47.0};
  double v_max_pu{1.1};
  double v_min_pu{0.9};

  int area{0};

  double mtbf_hours{0.0};
  double mttr_hours{0.0};
  double t_scheduled_hr{0.0};
};

// ═══════════════════════════════════════════════════════════════════════
// AC System container
// ═══════════════════════════════════════════════════════════════════════
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

// ═══════════════════════════════════════════════════════════════════════
// DC System container
// ═══════════════════════════════════════════════════════════════════════
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

// ═══════════════════════════════════════════════════════════════════════
// Hybrid AC/DC Power System
// ═══════════════════════════════════════════════════════════════════════
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

  std::optional<ThreePhaseACSystem> three_phase_ac;

  std::optional<BusMergeMap> bus_merge_map;
  std::optional<BranchExpandMap> branch_expand_map;
};

}  // namespace hacdcpf
