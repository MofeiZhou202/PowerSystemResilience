#pragma once

// Topology Analysis — connectivity, radiality, and optimal network
// reconfiguration (ONR) via a specialised branch-and-cut MILP.
// Equivalent to DistributionPowerFlow.jl TopologyAnalysis module.
//
// The ONR entry point (solve_optimal_reconfiguration) is a compatibility
// wrapper: it routes the historical AC-only ONR API through the maintained
// hybrid-aware reconfiguration solver (run_topology_reconfiguration, invoked
// with solver="highs" → HiGHS with a SCIP fallback) and projects the result
// back into the legacy ONRResult shape. It is no longer a separate inline
// branch-and-cut. When the core MILP is infeasible but the unchanged connected
// topology still solves a power flow, the wrapper reports that base topology
// with ONRResult::fallback_used=true so callers do not mistake it for a proven
// ONR incumbent.
//
// Cross-validated by:
//   1. Connectivity / radiality checks on known networks.
//   2. ONR on a 6-bus test case verified against exhaustive enumeration.
//   3. ONR on IEEE 33bw verified by post-solve Newton power flow.

#include <string>
#include <vector>

#include "hacdcpf/api/hacdcpf.hpp"        // PowerFlowResult
#include "hacdcpf/model/system.hpp"
#include "hacdcpf/solver/branch_and_cut.hpp" // BCStats

namespace hacdcpf::analysis {

// ---------------------------------------------------------------------------
// Simple topology queries  (reuse island-detector DFS pattern)
// ---------------------------------------------------------------------------

/// True iff the AC network (considering only in-service branches) is connected.
bool is_connected(const ACSystem& ac_sys);

/// True iff the AC network is radial:
///   connected  AND  #in-service-branches == #buses - 1
bool is_radial(const ACSystem& ac_sys);

/// Number of connected islands (0 if no buses, 1 if fully connected).
int count_islands(const ACSystem& ac_sys);

// ---------------------------------------------------------------------------
// Optimal Network Reconfiguration options
// ---------------------------------------------------------------------------

struct ONROptions {
  /// Stable ACBranch::index values that the ONR may open or close.
  /// An empty vector means ALL branches in ac_sys.branches are candidates.
  std::vector<int> switchable_branch_ids;

  // Voltage limits (applied to the squared-voltage variable in LinDistFlow)
  double v_min_pu{0.95};
  double v_max_pu{1.05};

  // Branch thermal rating used when ACBranch::rate_a_mva == 0
  double default_rate_mva{10.0};

  // Big-M constant for LinDistFlow relaxation (units: pu²)
  double big_m{4.0};

  // B&C solver parameters
  double mip_gap{0.01};    ///< Relative optimality gap tolerance
  int    max_time_s{300};  ///< Wall-clock time limit [s]
  bool   verbose{false};
};

// ---------------------------------------------------------------------------
// ONR result
// ---------------------------------------------------------------------------

struct ONRResult {
  /// Stable branch identifiers (ACBranch::index) that should be OPEN.
  /// These are the original component indices from the ACSystem branch table,
  /// NOT positional offsets into the (possibly projected) branch vector.
  std::vector<int> open_branch_ids;

  /// Stable branch identifiers (ACBranch::index) that should be CLOSED.
  std::vector<int> closed_branch_ids;

  // Objective (linearised loss proxy  Σ r[b]·|P[b]|  [p.u.])
  double milp_objective{0.0};
  double estimated_loss_mw{0.0};

  // Verification power flow run on the optimal configuration
  PowerFlowResult verification_pf;

  bool feasible{false};
  bool optimal{false};

  /// True when the core reconfiguration MILP did not produce a feasible
  /// solution and this result instead reports the unchanged base topology
  /// (which still solved a verification power flow). `feasible` is true and
  /// `optimal` is false in that case, `milp_objective` is 0, and the branch
  /// sets echo the input topology — it is a connectivity fallback, not an ONR
  /// incumbent.
  bool fallback_used{false};

  // B&C diagnostics
  solver::BCStats bc_stats;

  std::string summary() const;
};

// ---------------------------------------------------------------------------
// Primary interface
// ---------------------------------------------------------------------------

/// Solve the Optimal Network Reconfiguration problem.
///
/// The MILP is formulated as:
///   Variables : α[b]∈{0,1}, P[b], Q[b], v[i], f[b] (commodity), t[b] (|P| aux)
///   Constraints:
///     - Spanning-tree edge count: Σ α[b] = n_bus − 1
///     - Connectivity (single-commodity flow with big-M capacity on α)
///     - Active / reactive power balance (LinDistFlow, all non-root buses)
///     - LinDistFlow voltage drop (big-M relaxed for open branches)
///     - Branch thermal limits (big-M: forced zero when α=0)
///     - Auxiliary: t[b] ≥ |P[b]|
///   Objective: min Σ_b r[b]·t[b]   (linearised I²R loss proxy)
///
/// The B&C is invoked via solver::solve_milp_bc() with MIR cutting planes.
///
/// After finding the optimal α*, the function builds a modified ACSystem with
/// in_service flags set accordingly and runs a Newton power flow for
/// verification.
ONRResult solve_optimal_reconfiguration(const ACSystem& ac_sys,
                                        const ONROptions& opt = {});

/// Overload accepting a full HybridPowerSystem.
/// The system is first projected through project_to_canonical_models() so that
/// Transformer2W/3W, Switches, FlexibleLoads, AsymmetricLoads, and Chargers
/// are expanded into canonical flat tables before building the ONR MILP.
ONRResult solve_optimal_reconfiguration(const HybridPowerSystem& sys,
                                        const ONROptions& opt = {});

}  // namespace hacdcpf::analysis
