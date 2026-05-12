#pragma once

#include <string>

#include "hacdcpf/model/enums/switching_enums.hpp"

namespace hacdcpf {

// ═══════════════════════════════════════════════════════════════════════
// Switch (circuit breaker, disconnector, sectionalizer, ...)
// ═══════════════════════════════════════════════════════════════════════
struct Switch {
  int index{0};
  std::string name;
  int bus_from{0};
  int bus_to{0};
  bool in_service{true};

  SwitchType switch_type{SwitchType::CircuitBreaker};
  bool closed{true};

  // Electrical parameters
  double r_contact_ohm{0.0};
  double z_ohm{0.0};
  double i_rated_ka{0.0};
  double i_breaking_ka{0.0};

  // Protection coordination
  int element_type{0};            // type of protected element
  int element_id{0};              // index of protected element

  // Automation
  bool is_remote{false};
  bool is_automated{false};
  double t_operation_s{0.0};      // operation time (seconds)

  // Reliability
  double p_sw_fail{0.0};          // switching failure probability
  double mtbf_hours{0.0};
  double mttr_hours{0.0};
  double t_scheduled_hr{0.0};     // scheduled maintenance (hours/year)
  double t_tp_hr{0.0};            // topology reconfiguration time (hours)
};

// ═══════════════════════════════════════════════════════════════════════
// Circuit Breaker (HVCB) — high-voltage circuit breaker
// Corresponds to Julia HighVoltageCircuitBreaker in DistributionPowerFlow
// ═══════════════════════════════════════════════════════════════════════
struct CircuitBreaker {
  int index{0};
  std::string name;
  int bus_from{0};
  int bus_to{0};
  bool in_service{true};

  BreakerType breaker_type{BreakerType::CB};
  bool closed{true};

  // Electrical parameters
  double z_ohm{0.0};             // series impedance (ohms)
  double rated_voltage_kv{0.0};  // rated voltage (kV)
  double i_rated_ka{0.0};        // rated current (kA)
  double i_breaking_ka{0.0};     // breaking current capacity (kA)

  // Protected element association
  std::string element_type;      // "l"=line, "t"=transformer, "b"=bus
  int element_id{0};             // index of protected element
};

// ═══════════════════════════════════════════════════════════════════════
// DC Circuit Breaker — breaker on a DC bus pair
// ═══════════════════════════════════════════════════════════════════════
struct DCCircuitBreaker {
  int index{0};
  std::string name;
  int bus_from{0};
  int bus_to{0};
  bool in_service{true};

  BreakerType breaker_type{BreakerType::CB};
  bool closed{true};

  // Electrical parameters
  double r_ohm{0.0};              // series resistance (ohms)
  double rated_voltage_kv{0.0};   // rated voltage (kV)
  double i_rated_ka{0.0};         // rated current (kA)
  double i_breaking_ka{0.0};      // breaking current capacity (kA)

  // Protected element association
  std::string element_type;       // "l"=line, "c"=converter, "b"=bus
  int element_id{0};              // index of protected element
};

// ═══════════════════════════════════════════════════════════════════════
// Shunt (fixed or switchable)
// ═══════════════════════════════════════════════════════════════════════
struct Shunt {
  int index{0};
  int bus{0};
  bool in_service{true};
  std::string name;

  double gs_mw{0.0};         // conductance at 1.0 pu voltage (MW)
  double bs_mvar{0.0};       // susceptance at 1.0 pu voltage (Mvar)

  // Switchable shunt
  bool switchable{false};
  int n_steps{1};             // number of switchable steps
  int current_step{1};        // current step (1..n_steps)
  double bs_per_step{0.0};    // reactive per step (Mvar)
};

}  // namespace hacdcpf
