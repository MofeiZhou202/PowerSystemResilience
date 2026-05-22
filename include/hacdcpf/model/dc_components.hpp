#pragma once

/// model/dc_components.hpp
/// =======================
/// Consolidated DC component types for the hacdcpf power system model.
/// Replaces: dc_network.hpp, loads.hpp (DCLoad), switching.hpp (DCCircuitBreaker).

#include <string>

#include "hacdcpf/model/enums/bus_types.hpp"
#include "hacdcpf/model/enums/load_enums.hpp"
#include "hacdcpf/model/enums/switching_enums.hpp"

namespace hacdcpf {

// ═══════════════════════════════════════════════════════════════════════
// DC Bus
// ═══════════════════════════════════════════════════════════════════════
struct DCBus {
  int index{0};
  DCBusType bus_type{DCBusType::DC_P};
  double vm_pu{1.0};
  double vmax_pu{1.1};
  double vmin_pu{0.9};
  double pd_mw{0.0};
  bool in_service{true};
  std::string name;

  double base_kv{0.0};
  int area{0};
  int zone{0};

  int n_customers{0};
  double importance{1.0};
  bool is_load{false};

  double latitude{0.0};
  double longitude{0.0};
};

// ═══════════════════════════════════════════════════════════════════════
// DC Branch
// ═══════════════════════════════════════════════════════════════════════
struct DCBranch {
  int index{0};
  int from_bus{0};
  int to_bus{0};
  double r_pu{0.0};
  bool in_service{true};
  std::string name;

  double rate_a_mva{0.0};
  double length_km{0.0};
  double base_kv{0.0};
  double s_max_mva{0.0};
  int n_parallel{1};

  double mtbf_hours{0.0};
  double mttr_hours{0.0};
  double t_scheduled_hr{0.0};
};

// ═══════════════════════════════════════════════════════════════════════
// DC Static Generator (PV, wind, ESS on DC bus)
// ═══════════════════════════════════════════════════════════════════════
struct StaticGeneratorDC {
  int index{0};
  int bus{0};
  bool in_service{true};
  std::string name;
  std::string type;

  double p_set_mw{0.0};
  double scaling{1.0};
  int profile_id{-1};
  double pmax_mw{0.0};
  double pmin_mw{0.0};

  bool controllable{false};

  double mtbf_hours{0.0};
  double mttr_hours{0.0};
  double t_scheduled_hr{0.0};
};

// ═══════════════════════════════════════════════════════════════════════
// DC PV Array
// ═══════════════════════════════════════════════════════════════════════
struct PVArrayDC {
  int index{0};
  int bus{0};
  bool in_service{true};
  std::string name;

  double p_set_mw{0.0};
  int profile_id{-1};

  int num_series{0};
  int num_parallel{0};
  double vmpp{0.0};
  double impp{0.0};
  double voc{0.0};
  double isc{0.0};
  double alpha_isc{0.0};
  double beta_voc{0.0};

  double temperature{25.0};
  double irradiance{1000.0};

  double mtbf_hours{0.0};
  double mttr_hours{0.0};
  double t_scheduled_hr{0.0};
};

// ═══════════════════════════════════════════════════════════════════════
// DC Load
// ═══════════════════════════════════════════════════════════════════════
struct DCLoad {
  int index{0};
  int bus{0};
  bool in_service{true};
  std::string name;
  std::string type;

  double p_mw{0.0};
  double p_rated_mw{0.0};
  double scaling{1.0};

  double z_percent{0.0};
  double i_percent{0.0};
  double p_percent{100.0};

  bool controllable{false};
  double p_min_mw{0.0};
  double cost_mw{0.0};
  int profile_id{-1};
  LoadPriority priority{LoadPriority::Medium};
};

// ═══════════════════════════════════════════════════════════════════════
// DC Circuit Breaker
// ═══════════════════════════════════════════════════════════════════════
struct DCCircuitBreaker {
  int index{0};
  std::string name;
  int bus_from{0};
  int bus_to{0};
  bool in_service{true};

  BreakerType breaker_type{BreakerType::CB};
  bool closed{true};

  double r_ohm{0.0};
  double rated_voltage_kv{0.0};
  double i_rated_ka{0.0};
  double i_breaking_ka{0.0};

  std::string element_type;
  int element_id{0};
};

}  // namespace hacdcpf
