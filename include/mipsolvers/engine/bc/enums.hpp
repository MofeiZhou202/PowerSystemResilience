#pragma once
/// \file enums.hpp
/// \brief Strategy enumerations for the native branch-and-cut engine.
///
/// This header is deliberately dependency-free so it can be included
/// from any public header (including ML-integration code) without
/// dragging in the rest of the B&C engine.

namespace mipsolvers::engine {

/// Branching-variable selection strategy (mirrors Julia's BranchingParams).
enum class BranchingStrategy {
  MostInfeasible,   ///< Variable whose fraction is closest to 0.5
  Pseudocost,       ///< Benichou pseudocost estimates
  FirstFractional,  ///< First fractional variable (debug/baseline)
};

/// Node selection strategy (mirrors Julia's NodeSelectionStrategy).
enum class NodeSelection {
  BestFirst,   ///< Estimate-guided best-first with exact LB tracking
  DepthFirst,  ///< LIFO (fast incumbent discovery, small footprint)
  Hybrid,      ///< DFS until first incumbent, then estimate-guided best-first
};

enum class NodeEstimateAggregation {
  Sum,      ///< Sum each fractional variable's cheaper directional lift
  Maximum,  ///< Use only the largest cheaper directional lift
};

/// Families of cutting planes the engine can generate.
enum class CutType {
  None,
  IntRounding,
  MIR,
  Gomory,
  Cover,
  All,
};

/// Validity/lifetime of a generated row.
///
/// GlobalCut rows are valid for the original model relaxation and may be
/// stored in the global cut pool. LocalNodeCut rows are valid only under the
/// branch/domain state of the generating node and descendants that inherit
/// that domain. LazyConstraint rows are model-valid constraints checked
/// against integer incumbents; they must not be treated as node-local cuts.
enum class ValidityScope {
  GlobalCut,
  LocalNodeCut,
  LazyConstraint,
};

}  // namespace mipsolvers::engine
