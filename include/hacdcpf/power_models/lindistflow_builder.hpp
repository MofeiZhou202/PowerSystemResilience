#pragma once

/// LinDistFlow Builder
/// ===================
/// Builds and solves a linearised DistFlow LP for radial distribution networks.
///
/// Mathematical model (per Baran & Wu linearisation, ignoring I²r losses):
///
///   Variables:
///     P_ij  — real power flow on branch (i→j)  [MW]
///     Q_ij  — reactive power flow on branch (i→j) [MVAr]
///     v_i   — squared voltage magnitude at bus i [p.u.²]
///
///   Power balance at each non-root bus i:
///     Σ_{j: (i,j)∈E} P_ij - Σ_{k: (k,i)∈E} P_ki = -p_load_i
///     Σ_{j: (i,j)∈E} Q_ij - Σ_{k: (k,i)∈E} Q_ki = -q_load_i
///
///   LinDistFlow voltage drop for branch (i→j):
///     v_j = v_i - 2*(r_ij*P_ij + x_ij*Q_ij)
///
///   Thermal limit:
///     |P_ij| ≤ S_max_ij,  |Q_ij| ≤ S_max_ij  (approximate)
///
///   Voltage limits:
///     v_min² ≤ v_i ≤ v_max²
///
///   Objective: minimize total real power loss = Σ p_load_i - v_root = 0 (feasibility)
///              Default: minimise Σ (v_max - v_i) as a proxy for voltage regulation.
///
/// Scope: this basic AML builder has fixed loads only; it does not create DG,
/// controllable source, or load-shedding variables.

#include <map>
#include <string>
#include <tuple>
#include <vector>

#include "hacdcpf/aml/aml.hpp"

namespace hacdcpf::power_models {

// ════════════════════════════════════════════════════════════════════════════
// Input data
// ════════════════════════════════════════════════════════════════════════════

struct LinDistFlowData {
  /// Ordered list of bus ids. The root bus must be included.
  std::vector<std::string> bus_ids;

  /// Root bus id (feeder head). Voltage is fixed: v_root = v_root_pu_sq.
  std::string root_bus;

  /// Fixed root voltage [p.u.²], default 1.0.
  double root_v_sq_pu = 1.0;

  /// Branch definition: (from_bus, to_bus, r_pu, x_pu, s_max_mva).
  using BranchTuple = std::tuple<std::string, std::string, double, double, double>;
  std::vector<BranchTuple> branches;

  /// Real load at each bus [MW]. Default 0 if absent.
  std::map<std::string, double> p_load_mw;

  /// Reactive load at each bus [MVAr]. Default 0 if absent.
  std::map<std::string, double> q_load_mvar;

  /// Voltage lower bound [p.u.²]. Default 0.81 = (0.9)².
  double v_min_pu_sq = 0.81;

  /// Voltage upper bound [p.u.²]. Default 1.21 = (1.1)².
  double v_max_pu_sq = 1.21;
};

// ════════════════════════════════════════════════════════════════════════════
// Output
// ════════════════════════════════════════════════════════════════════════════

struct LinDistFlowResult {
  aml::SolveResult solve_result;

  /// V_i² [p.u.²], keyed by bus_id.
  std::map<std::string, double> voltage_sq_pu;

  /// Real power flow P_ij [MW], keyed by "{from}→{to}".
  std::map<std::string, double> branch_p_mw;

  /// Reactive power flow Q_ij [MVAr], keyed by "{from}→{to}".
  std::map<std::string, double> branch_q_mvar;
};

// ════════════════════════════════════════════════════════════════════════════
// Builder function
// ════════════════════════════════════════════════════════════════════════════

LinDistFlowResult solve_lindistflow(const LinDistFlowData& data,
                                     const aml::SolveOptions& opts = {});

}  // namespace hacdcpf::power_models
