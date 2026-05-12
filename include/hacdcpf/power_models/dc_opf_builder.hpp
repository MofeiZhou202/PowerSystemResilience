#pragma once

/// DC Optimal Power Flow Builder
/// ==============================
/// Builds and solves a DC OPF problem via the AML.
///
/// Mathematical form:
///
///   min   Σ_g  c_g * p_g
///   s.t.  Σ_{g∈g(i)} p_g − P_d^i − Σ_{ℓ=(i,j)} b_ℓ(θ_i−θ_j)
///              + Σ_{ℓ=(j,i)} b_ℓ(θ_j−θ_i) = 0   ∀ i ∈ B
///         p_g^min ≤ p_g ≤ p_g^max   ∀ g
///         −P_ℓ^max ≤ b_ℓ(θ_i−θ_j) ≤ P_ℓ^max   ∀ ℓ=(i,j)
///         θ_ref = 0
///
/// LMP_i = −dual(balance_i)   (generation-demand-export form)

#include <map>
#include <string>
#include <tuple>
#include <vector>

#include "hacdcpf/aml/aml.hpp"

namespace hacdcpf::power_models {

// ════════════════════════════════════════════════════════════════════════════
// Input data
// ════════════════════════════════════════════════════════════════════════════

struct DCOPFData {
  /// 1-D list of bus identifiers (strings).
  std::vector<std::string> bus_ids;

  /// The reference (slack) bus — its angle is fixed to zero.
  std::string slack_bus;

  /// Generators: (gen_id, bus_id, pmin_MW, pmax_MW, cost_per_MWh)
  using GenTuple = std::tuple<std::string, std::string,
                               double, double, double>;
  std::vector<GenTuple> generators;

  /// Branches: (from_bus, to_bus, susceptance_1_per_ohm, max_flow_MW)
  /// max_flow_MW == 0 means uncongested (no flow limit imposed).
  using BranchTuple = std::tuple<std::string, std::string,
                                  double, double>;
  std::vector<BranchTuple> branches;

  /// Nodal demand in MW, keyed by bus_id.
  std::map<std::string, double> demands_MW;
};

// ════════════════════════════════════════════════════════════════════════════
// Output
// ════════════════════════════════════════════════════════════════════════════

struct DCOPFResult {
  aml::SolveResult solve_result;

  /// Optimal dispatch in MW, keyed by generator id.
  std::map<std::string, double> gen_dispatch_MW;

  /// Locational Marginal Prices in $/MWh, keyed by bus id.
  /// LMP_i = −dual(nodal balance at i)
  std::map<std::string, double> lmp_per_MWh;

  /// Voltage angles in radians, keyed by bus id.
  std::map<std::string, double> voltage_angle_rad;

  /// Branch flows in MW, keyed by "from_bus->to_bus".
  std::map<std::string, double> branch_flow_MW;
};

// ════════════════════════════════════════════════════════════════════════════
// Builder function
// ════════════════════════════════════════════════════════════════════════════

DCOPFResult solve_dc_opf(const DCOPFData& data,
                          const aml::SolveOptions& opts = {});

}  // namespace hacdcpf::power_models
