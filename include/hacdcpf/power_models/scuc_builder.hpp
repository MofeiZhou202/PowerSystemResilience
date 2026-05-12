#pragma once

/// Security-Constrained Unit Commitment (SCUC) Builder
/// =====================================================
/// Builds and solves a SCUC MILP via the AML.
///
/// Mathematical form (simplified UC):
///
///   min  Σ_t Σ_g  [ c_g*p_{g,t}  +  S_g*s_{g,t}  +  C_g*u_{g,t} ]
///   s.t. Σ_g p_{g,t} = D_t                      ∀ t          (balance)
///        p_g^min*u_{g,t} ≤ p_{g,t} ≤ p_g^max*u_{g,t}  ∀ g,t  (capacity)
///        p_{g,t} − p_{g,t−1} ≤ R_g^up            ∀ g, t≠t_0  (ramp up)
///        p_{g,t−1} − p_{g,t} ≤ R_g^dn            ∀ g, t≠t_0  (ramp dn)
///        s_{g,t} ≥ u_{g,t} − u_{g,t−1}           ∀ g, t≠t_0  (startup)
///        s_{g,t} ≥ 0,  u_{g,t} ∈ {0,1},  p_{g,t} ≥ 0

#include <map>
#include <string>
#include <vector>

#include "hacdcpf/aml/aml.hpp"

namespace hacdcpf::power_models {

// ════════════════════════════════════════════════════════════════════════════
// Input data
// ════════════════════════════════════════════════════════════════════════════

struct SCUCData {
  /// Generator ids (ordered by index, used as set keys).
  std::vector<std::string> generator_ids;

  /// Time period ids (ordered chronologically — defines prev/next).
  std::vector<std::string> period_ids;

  /// Hourly system demand [MW], keyed by period_id.
  std::map<std::string, double> demand_MW;

  // ── Per-generator parameters ───────────────────────────────────────────

  /// Variable fuel cost [$/MWh], keyed by generator_id.
  std::map<std::string, double> cost_per_MWh;

  /// Minimum stable generation [MW], keyed by generator_id.
  std::map<std::string, double> pmin_MW;

  /// Maximum generation capacity [MW], keyed by generator_id.
  std::map<std::string, double> pmax_MW;

  /// Ramp-up limit [MW/h], keyed by generator_id.
  /// If absent or 0, no ramp-up constraint is imposed.
  std::map<std::string, double> ramp_up_MW;

  /// Ramp-down limit [MW/h], keyed by generator_id.
  std::map<std::string, double> ramp_dn_MW;

  /// Startup cost [$], keyed by generator_id.
  std::map<std::string, double> startup_cost;

  /// No-load (commitment) cost [$/h], keyed by generator_id.
  std::map<std::string, double> commit_cost;

  // ── Beta extensions ─────────────────────────────────────────────────────

  /// Minimum consecutive on-periods after startup. Key: generator_id.
  /// If absent, no minimum-up constraint is applied.
  std::map<std::string, int> min_up_periods;

  /// Minimum consecutive off-periods after shutdown. Key: generator_id.
  std::map<std::string, int> min_dn_periods;

  /// Initial commitment status u_{g, t=-1} before the first period.
  /// Value: 1.0 = on, 0.0 = off. Default = off if absent.
  std::map<std::string, double> initial_commitment;

  /// Per-period spinning reserve requirement [MW]. Key: period_id.
  /// Adds: Σ_g pmax_g * u_{g,t} ≥ demand_t + reserve_t  for each period t.
  std::map<std::string, double> spinning_reserve_MW;
};

// ════════════════════════════════════════════════════════════════════════════
// Output
// ════════════════════════════════════════════════════════════════════════════

struct SCUCResult {
  aml::SolveResult solve_result;

  /// Commitment status (0 or 1), keyed by (gen_id, period_id).
  std::map<std::pair<std::string, std::string>, double> commitment;

  /// Dispatch [MW], keyed by (gen_id, period_id).
  std::map<std::pair<std::string, std::string>, double> dispatch_MW;

  /// Startup event (0 or 1), keyed by (gen_id, period_id).
  std::map<std::pair<std::string, std::string>, double> startup_event;

  /// Shadow price of balance constraint [$/MWh], keyed by period_id.
  std::map<std::string, double> price_per_MWh;
};

// ════════════════════════════════════════════════════════════════════════════
// Builder function
// ════════════════════════════════════════════════════════════════════════════

SCUCResult solve_scuc(const SCUCData& data,
                       const aml::SolveOptions& opts = {});

}  // namespace hacdcpf::power_models
