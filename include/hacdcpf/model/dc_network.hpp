#pragma once

#include <string>

#include "hacdcpf/model/enums/bus_types.hpp"

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

  double base_kv{0.0};           // rated voltage (kV)
  int area{0};
  int zone{0};

  // Customer / resilience data
  int n_customers{0};
  double importance{1.0};
  bool is_load{false};

  // Geographical (for planning / visualization)
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
  double base_kv{0.0};           // rated voltage (kV)
  double s_max_mva{0.0};         // max transmission power (MVA)
  int n_parallel{1};

  // Reliability
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
  std::string type;               // "PV", "Wind", "ESS"

  double p_set_mw{0.0};          // DC active power setpoint
  double scaling{1.0};
  int profile_id{-1};            // time-series hook (index into profile, -1 = none)
  double pmax_mw{0.0};
  double pmin_mw{0.0};

  bool controllable{false};

  // Reliability
  double mtbf_hours{0.0};
  double mttr_hours{0.0};
  double t_scheduled_hr{0.0};
};

// ═══════════════════════════════════════════════════════════════════════
// DC PV Array (photovoltaic panel array on DC bus)
// ═══════════════════════════════════════════════════════════════════════
struct PVArrayDC {
  int index{0};
  int bus{0};
  bool in_service{true};
  std::string name;

  double p_set_mw{0.0};          // rated output (MW)
  int profile_id{-1};            // time-series hook (index into profile, -1 = none)

  // PV cell parameters
  int num_series{0};
  int num_parallel{0};
  double vmpp{0.0};              // voltage at MPP (V)
  double impp{0.0};              // current at MPP (A)
  double voc{0.0};               // open-circuit voltage (V)
  double isc{0.0};               // short-circuit current (A)
  double alpha_isc{0.0};         // temp coefficient of Isc (%/C)
  double beta_voc{0.0};          // temp coefficient of Voc (%/C)

  // Environmental
  double temperature{25.0};      // C
  double irradiance{1000.0};     // W/m2

  // Reliability
  double mtbf_hours{0.0};
  double mttr_hours{0.0};
  double t_scheduled_hr{0.0};
};

}  // namespace hacdcpf
