#pragma once

#include <string>

#include "hacdcpf/model/enums/load_enums.hpp"

namespace hacdcpf {

// ═══════════════════════════════════════════════════════════════════════
// Load (first-class, decoupled from bus)
// ═══════════════════════════════════════════════════════════════════════
struct Load {
  int index{0};
  int bus{0};
  bool in_service{true};
  std::string name;

  double p_mw{0.0};               // active demand (positive = consumption from bus)
  double q_mvar{0.0};             // reactive demand (positive = consumption from bus)
  double scaling{1.0};            // scaling factor

  // Load model
  LoadModel model{LoadModel::ConstantPower};

  // ZIP coefficients: P = P0*(Zp*V^2 + Ip*V + Pp), Q = Q0*(Zq*V^2 + Iq*V + Pq)
  double z_percent_p{0.0};
  double i_percent_p{0.0};
  double p_percent_p{100.0};
  double z_percent_q{0.0};
  double i_percent_q{0.0};
  double p_percent_q{100.0};

  // Controllability (for demand response / UC)
  bool controllable{false};
  double p_min_mw{0.0};       // min load if controllable
  double cost_mw{0.0};        // cost of load shedding ($/MWh)

  // Classification
  LoadPriority priority{LoadPriority::Medium};
  int n_customers{0};

  // Time-series hook (index into external profile, -1 = none)
  int profile_id{-1};

  // Short-circuit motor-fraction (IEC 60909)
  double sn_mva{0.0};         // rated apparent power (MVA)
  double motor_percent{0.0};  // percentage of motor load [0..1]
  double x_sub_pu{0.0};       // subtransient reactance of motor fraction (pu)
  double r_sc_pu{0.0};        // resistance of motor fraction (pu)
};

// ═══════════════════════════════════════════════════════════════════════
// Flexible Load (demand response)
// ═══════════════════════════════════════════════════════════════════════
struct FlexibleLoad {
  int index{0};
  int bus{0};
  bool in_service{true};
  std::string name;

  double p_mw{0.0};               // baseline demand (positive = consumption from bus)
  double q_mvar{0.0};             // baseline reactive demand (positive = consumption from bus)

  // Flexibility range
  double flex_up_mw{0.0};         // max increase from baseline
  double flex_down_mw{0.0};       // max decrease from baseline
  double flex_duration_h{0.0};    // how long flexibility can be sustained

  // Response characteristics
  double response_time_s{0.0};    // activation time
  double ramp_rate_mw_min{0.0};   // ramp rate
  double availability_pct{100.0}; // availability percentage

  bool controllable{true};
  LoadPriority priority{LoadPriority::Medium};
  std::string control_area;
};

// ═══════════════════════════════════════════════════════════════════════
// Asymmetric Load (three-phase unbalanced)
// ═══════════════════════════════════════════════════════════════════════
struct AsymmetricLoad {
  int index{0};
  int bus{0};
  bool in_service{true};
  std::string name;

  std::string connection;          // "wye" or "delta"
  bool grounded{true};

  // Per-phase rated power (positive = consumption from bus)
  double pa_rated_mw{0.0};  double qa_rated_mvar{0.0};
  double pb_rated_mw{0.0};  double qb_rated_mvar{0.0};
  double pc_rated_mw{0.0};  double qc_rated_mvar{0.0};

  // Per-phase actual power
  double pa_mw{0.0};  double qa_mvar{0.0};
  double pb_mw{0.0};  double qb_mvar{0.0};
  double pc_mw{0.0};  double qc_mvar{0.0};

  double scaling{1.0};

  // ZIP composition
  double const_z_percent{0.0};
  double const_i_percent{0.0};
  double const_p_percent{100.0};

  bool controllable{false};
  LoadPriority priority{LoadPriority::Medium};
};

// ═══════════════════════════════════════════════════════════════════════
// DC Load (first-class DC demand table)
// ═══════════════════════════════════════════════════════════════════════
struct DCLoad {
  int index{0};
  int bus{0};
  bool in_service{true};
  std::string name;
  std::string type;                // load type (industrial, commercial, residential)

  // Active power demand on DC network (positive = consumption from bus).
  double p_mw{0.0};
  double p_rated_mw{0.0};         // rated demand
  double scaling{1.0};

  // ZIP model
  double z_percent{0.0};
  double i_percent{0.0};
  double p_percent{100.0};

  // Optional metadata / control hooks.
  bool controllable{false};
  double p_min_mw{0.0};
  double cost_mw{0.0};
  int profile_id{-1};
  LoadPriority priority{LoadPriority::Medium};
};

}  // namespace hacdcpf
