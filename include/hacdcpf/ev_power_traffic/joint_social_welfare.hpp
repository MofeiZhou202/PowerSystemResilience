#pragma once

// ev_power_traffic/joint_social_welfare.hpp
// ──────────────────────────────────────────────────────────────────────────
// Joint Power-Traffic Social Welfare Maximisation.
//
// Iteratively coordinates a DC-OPF (power side) with a system-optimal LP
// traffic assignment (traffic side) via locational marginal price (LMP)
// feedback.  Converges when nodal electricity prices at charging stations
// stabilise between iterations.
//
// Social welfare:  W = B_EV − C_gen − C_delay
//   B_EV   = Σ_d  WTP_d · served_d          (gross consumer benefit)
//   C_gen  = Σ_k  opf_objective_k           (generation dispatch cost)
//   C_delay= Σ_(d,r) VOT · travel_time · x  (traffic delay externality)

#include <unordered_map>
#include <vector>

#include "hacdcpf/ev_power_traffic/simulation.hpp"
#include "hacdcpf/optimal_power_flow/opf_options.hpp"
#include "hacdcpf/optimal_power_flow/opf_result.hpp"

namespace hacdcpf::evpt {

struct JointSocialWelfareOptions {
  // Iterative coordination parameters
  int max_iterations{30};
  double price_convergence_tol{1e-4};  ///< $/kWh; stop when max LMP change < tol
  double price_update_step{0.6};       ///< α in π^{new} = α·λ_{LMP} + (1-α)·π^{old}

  // When false the DC-OPF step is skipped and fixed default prices are used
  // (degenerates to a plain system-optimal LP with no power coupling).
  bool use_dcopf_prices{true};

  bool verbose{false};  ///< print per-iteration summary to stdout

  // Sub-problem options
  EVPowerTrafficOptions evpt_opts;
  opf::DCOPFOptions     dcopf_opts;
};

/// One row in the convergence history.
struct JointIterationRecord {
  int    iteration{0};
  double social_welfare{0.0};
  double ev_gross_benefit{0.0};   ///< Σ WTP_d · served_d
  double generation_cost{0.0};    ///< Σ_k opf_cost_k
  double traffic_delay_cost{0.0}; ///< VOT · Σ travel_time
  double price_change{0.0};       ///< max |π^{new} − π^{old}| over all stations/steps
  bool   opf_converged{false};
};

struct JointSocialWelfareResult {
  bool        converged{false};
  int         iterations{0};
  std::string status;

  // Social welfare at the final (converged / last) iteration
  double social_welfare{0.0};        ///< B_EV - C_gen - C_delay
  double ev_gross_benefit{0.0};      ///< Σ WTP_d · served_d
  double ev_consumer_surplus{0.0};   ///< Σ (WTP_d − price_paid_d) · served_d
  double generation_cost{0.0};       ///< total generation dispatch cost ($/step)
  double traffic_delay_cost{0.0};    ///< VOT · total travel time

  // Converged station prices ($/kWh) per step: [station_id][step]
  std::unordered_map<int, std::vector<double>> station_prices_per_step;

  // LMPs ($/MWh) at each charging-station bus, per step: [step][bus]
  std::vector<std::unordered_map<int, double>> lmp_by_step;

  // Final traffic assignment result
  EVPowerTrafficResult final_evpt;

  // DC-OPF result for each time step (empty when use_dcopf_prices=false)
  std::vector<opf::DCOPFResult> opf_by_step;

  // Per-iteration convergence history
  std::vector<JointIterationRecord> history;
};

JointSocialWelfareResult solve_joint_social_welfare(
    EVPowerTrafficProblem problem,   // taken by value; prices mutated internally
    const JointSocialWelfareOptions& options = {});

}  // namespace hacdcpf::evpt
