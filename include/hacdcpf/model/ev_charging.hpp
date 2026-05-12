#pragma once

#include <string>

#include "hacdcpf/model/enums/charging_enums.hpp"

namespace hacdcpf {

// ═══════════════════════════════════════════════════════════════════════
// Charger (individual EV charging unit)
// ═══════════════════════════════════════════════════════════════════════
struct Charger {
  int index{0};
  std::string name;
  int station_id{0};               // parent ChargingStation index
  ChargerType charger_type{ChargerType::AC_L2};
  bool in_service{true};

  double p_rated_kw{0.0};
  double p_ch_max_kw{0.0};
  double p_ch_min_kw{0.0};
  double eta{0.95};

  bool v2g_capable{false};
  double p_dis_max_kw{0.0};       // V2G discharge max

  // EV connected
  std::string connector_type;      // e.g. "CCS", "CHAdeMO", "Type2"
  std::string status;              // e.g. "idle", "charging", "fault"
  double p_ev_kw{0.0};            // current EV power demand

  // Reliability
  double mtbf_hr{0.0};
  double mttr_hr{0.0};
  double t_scheduled_hr{0.0};
};

// ═══════════════════════════════════════════════════════════════════════
// Charging Station (EV charging site, aggregation of chargers)
// ═══════════════════════════════════════════════════════════════════════
struct ChargingStation {
  int index{0};
  std::string name;
  int bus{0};
  std::string location;
  bool in_service{true};

  int n_fast{0};
  int n_slow{0};
  int num_chargers{0};

  double p_fast_max_kw{0.0};
  double p_slow_max_kw{0.0};
  double max_power_kw{0.0};

  double simultaneity_factor{1.0};
  double power_factor{0.95};
  double utilization_rate{0.0};

  double p_total_kw{0.0};
  double q_total_kvar{0.0};

  int n_cars{0};                   // number of currently connected EVs

  // Reliability
  double mtbf_hr{0.0};
  double mttr_hr{0.0};
  double t_scheduled_hr{0.0};
};

}  // namespace hacdcpf
