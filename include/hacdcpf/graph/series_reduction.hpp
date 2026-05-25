#pragma once

/// graph/series_reduction.hpp
/// ===========================
/// Series reduction (passive degree-2 node elimination) and
/// pendant branch folding.
///
/// Series reduction theory:
///   For a node j with degree=2, no injection (S_j=0), no shunt, no control:
///     Z_ik_new = Z_ij + Z_jk   (series aiding)
///     B_ik_new ≈ B_ij + B_jk   (line charging approximation)
///
/// Pendant reduction theory:
///   For a leaf node j with only load (no gen, no control):
///     P_i_new = P_i + P_j + R_ij * (P_j² + Q_j²) / |V_i|²   (approx)
///     Q_i_new = Q_i + Q_j + X_ij * (P_j² + Q_j²) / |V_i|²   (approx)
///   |V_i| = 1.0 pu is used when exact value is unknown.

#include <vector>

#include "hacdcpf/graph/power_system_graph.hpp"
#include "hacdcpf/graph/reduction_mapping.hpp"
#include "hacdcpf/graph/reduction_plan.hpp"
#include "hacdcpf/graph/topology_analysis.hpp"
#include "hacdcpf/model/hybrid_power_system.hpp"

namespace hacdcpf::graph {

// ═══════════════════════════════════════════════════════════════════════
// Series reduction result
// ═══════════════════════════════════════════════════════════════════════

struct SeriesReductionResult {
  PowerSystemGraph   reduced_graph;
  HybridPowerSystem  reduced_system;
  ReductionMapping   mapping;
  std::vector<Diagnostic> diagnostics;
};

// ═══════════════════════════════════════════════════════════════════════
// Pendant reduction result
// ═══════════════════════════════════════════════════════════════════════

struct PendantReductionResult {
  PowerSystemGraph   reduced_graph;
  HybridPowerSystem  reduced_system;
  ReductionMapping   mapping;
  std::vector<Diagnostic> diagnostics;
};

// ═══════════════════════════════════════════════════════════════════════
// API
// ═══════════════════════════════════════════════════════════════════════

/// Apply series reduction to all eligible degree-2 passive nodes.
/// Only nodes classified as ZeroInjectionDegree2 in \p plan are processed.
SeriesReductionResult apply_series_reduction(
    const PowerSystemGraph&      graph,
    const HybridPowerSystem&     system,
    const ReductionPlan&         plan,
    const GraphReductionOptions& options = {});

/// Apply pendant (leaf-node) folding.
/// Uses |V_parent| = 1.0 pu as the approximation for branch loss calculation.
/// Only nodes classified as PendantLoad in \p plan are processed.
PendantReductionResult apply_pendant_reduction(
    const PowerSystemGraph&      graph,
    const HybridPowerSystem&     system,
    const ReductionPlan&         plan,
    const GraphReductionOptions& options = {});

}  // namespace hacdcpf::graph
