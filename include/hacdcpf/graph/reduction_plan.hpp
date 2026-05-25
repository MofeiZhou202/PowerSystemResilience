#pragma once

/// graph/reduction_plan.hpp
/// =========================
/// Options, candidate classification, and reduction plan for the
/// multi-strategy graph reduction pipeline.
///
/// Reduction strategies (in application order):
///   1. SwitchContraction   – merge zero-impedance / closed-switch supernodes
///   2. SeriesReduction     – eliminate passive degree-2 nodes
///   3. PendantReduction    – fold terminal load nodes (approximate)
///   4. KronPassiveOnly     – Schur complement for passive interior nodes
///
/// The plan is generated before any modification so it can be audited,
/// printed, and optionally rejected by the caller.

#include <string>
#include <vector>

namespace hacdcpf::graph {

// ═══════════════════════════════════════════════════════════════════════
// Reduction mode / method enumerations
// ═══════════════════════════════════════════════════════════════════════

enum class ReductionMode {
  Safe,        ///< Only provably lossless reductions (default)
  Moderate,    ///< Safe + passive Kron reduction
  Aggressive,  ///< All reductions including approximate pendant folding
};

enum class ReductionMethod {
  None,
  SwitchContraction,
  SeriesReduction,
  PendantReduction,
  KronPassiveOnly,
  KronWithCurrentInjection,
  WardEquivalent,
  HybridSafeReduction,
};

// ═══════════════════════════════════════════════════════════════════════
// Reduction options
// ═══════════════════════════════════════════════════════════════════════

struct GraphReductionOptions {
  // --- Which reductions to enable ---
  bool enable_switch_contraction{true};
  bool enable_series_reduction{true};
  bool enable_pendant_reduction{false};   ///< Approximate; off by default
  bool enable_kron_reduction{false};      ///< Off by default; enable explicitly

  // --- Which nodes / edges must be preserved ---
  bool preserve_all_load_buses{true};
  bool preserve_all_generator_buses{true};
  bool preserve_all_voltage_constrained_buses{true};
  bool preserve_all_monitored_buses{true};
  bool preserve_branch_flow_limited_edges{true};

  // --- Numerical thresholds ---
  double zero_impedance_threshold{1e-8};   ///< |Z| < threshold → zero-Z
  double voltage_base_tolerance{1e-6};     ///< base_kv tolerance for merges
  double max_fill_ratio{2.0};              ///< Kron fill-in guard

  ReductionMode mode{ReductionMode::Safe};
};

// ═══════════════════════════════════════════════════════════════════════
// Candidate classification
// ═══════════════════════════════════════════════════════════════════════

enum class CandidateType {
  MustRetain,                ///< Must not be eliminated
  ZeroInjectionDegree2,      ///< Passive, degree-2 → series reduction
  ZeroInjectionPassiveInterior, ///< Passive, interior → Kron candidate
  PendantLoad,               ///< Leaf node with load only
  RadialFeederSegment,       ///< Segment of a radial feeder
  KronPassiveNode,           ///< Pure passive node for Kron elimination
};

struct BusCandidate {
  int           bus_id{0};
  CandidateType type{CandidateType::MustRetain};
  std::string   reason;
};

struct ReductionCandidates {
  std::vector<BusCandidate> candidates;
};

// ═══════════════════════════════════════════════════════════════════════
// Reduction plan
// ═══════════════════════════════════════════════════════════════════════

enum class ReductionActionType {
  SwitchContraction,
  SeriesReduction,
  PendantFold,
  KronEliminate,
  Retain,
};

struct ReductionAction {
  ReductionActionType    type{ReductionActionType::Retain};
  std::vector<int>       eliminated_buses;
  std::vector<int>       eliminated_branches;
  std::vector<int>       retained_buses;
  std::string            reason;
};

struct ReductionPlan {
  std::vector<ReductionAction> actions;
  int n_buses_eliminated{0};
  int n_branches_eliminated{0};
};

// ═══════════════════════════════════════════════════════════════════════
// API (implementations in src/graph/reduction_plan.cpp)
// ═══════════════════════════════════════════════════════════════════════

struct PowerSystemGraph;  // forward (defined in power_system_graph.hpp)

}  // namespace hacdcpf::graph

// Forward declaration in the correct namespace to avoid shadowing
// hacdcpf::HybridPowerSystem inside hacdcpf::graph.
namespace hacdcpf { struct HybridPowerSystem; }

namespace hacdcpf::graph {

/// Classify every bus in the (contracted) graph into candidate types.
ReductionCandidates classify_reduction_candidates(
    const PowerSystemGraph&          graph,
    const hacdcpf::HybridPowerSystem& system,
    const GraphReductionOptions&     options = {});

/// Build an auditable reduction plan from the classified candidates.
ReductionPlan make_reduction_plan(
    const PowerSystemGraph&      graph,
    const ReductionCandidates&   candidates,
    const GraphReductionOptions& options = {});

}  // namespace hacdcpf::graph
