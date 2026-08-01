#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

#include <Eigen/Core>

#include "../native_dual_core.hpp"

namespace mipsolvers::engine::native_dual::detail {

enum class Phase { One, DualOne, Two };

enum class EdgeWeightMode { SteepestEdge, Devex };

enum class RebuildReason {
  Initial,
  UpdateLimit,
  SyntheticWork,
  NumericalTrouble,
  PossiblyOptimal,
  PossiblyPrimalInfeasible,
  Cleanup,
};

enum class Move : std::int8_t {
  Down = -1,
  Fixed = 0,
  Up = 1,
};

inline int sign(Move move) { return static_cast<int>(move); }

struct Bounds {
  Eigen::VectorXd lower;
  Eigen::VectorXd upper;
  std::vector<char> enterable;
};

class BasisFactor;

struct State {
  struct LeavingHeapEntry {
    double merit{0.0};
    int row{-1};
    std::uint64_t version{0};
  };

  const StandardFormLP* sf{nullptr};
  const SimplexOptions* options{nullptr};
  Phase phase{Phase::Two};
  int m{0};
  int n{0};
  Bounds bounds;
  std::vector<int> basis;
  std::vector<char> basic;
  std::vector<Move> move;
  Eigen::VectorXd cost;
  Eigen::VectorXd original_cost;
  Eigen::VectorXd cost_perturbation;
  Eigen::VectorXd cost_shift;
  // Feasible anchor Ax0=b used to express dual Phase I in the homogeneous
  // coordinates z=x-x0. Empty outside dual Phase I.
  Eigen::VectorXd dual_phase_one_anchor;
  Eigen::VectorXd x_basic;
  Eigen::VectorXd reduced_costs;
  std::vector<double> edge_weight;
  EdgeWeightMode edge_weight_mode{EdgeWeightMode::SteepestEdge};
  std::vector<char> devex_reference;
  int devex_iterations{0};
  double objective{0.0};
  std::shared_ptr<BasisFactor> factor;
  bool fresh_rebuild{false};
  bool costs_perturbed{false};
  bool costs_shifted{false};
  bool perturbation_disabled{false};
  int updates_since_rebuild{0};
  int canonical_primal_corrections{0};
  bool reinvert_after_pivot{false};
  std::uint64_t cycle_signature_a{0};
  std::uint64_t cycle_signature_b{0};
  // Incrementally maintained signature of the CURRENT (basis, nonbasic-side)
  // state.  Updated by O(1) token XORs at each pivot commit and fully resynced
  // by resync_cycle_signature whenever basis/move change outside the minor
  // iteration (major rebuilds reclassify every nonbasic side).
  // record_cycle_arrival publishes it into cycle_signature_a/b.
  std::uint64_t cycle_signature_live_a{0};
  std::uint64_t cycle_signature_live_b{0};
  int pivot_sequence{0};
  int pricing_epoch{0};
  // Lazy CHUZR heap. Each touched row gets a new version and, when infeasible,
  // a new heap entry. Stale entries are discarded when they reach the top.
  // Dense primal changes and reconstructions invalidate the whole structure.
  std::vector<LeavingHeapEntry> leaving_heap;
  std::vector<std::uint64_t> leaving_row_version;
  bool leaving_heap_valid{false};

  struct CycleRecord {
    std::uint64_t signature_a{0};
    std::uint64_t signature_b{0};
    int leaving_col{-1};
    int entering_col{-1};
    int last_seen{0};
  };
  std::map<std::pair<std::uint64_t, std::uint64_t>, CycleRecord> cycle_history;
  std::map<std::tuple<std::uint64_t, std::uint64_t, int, int>, int>
      taboo_changes;
  std::map<std::tuple<std::uint64_t, std::uint64_t, int>, int> taboo_rows;
};

struct Audit {
  bool ok{false};
  std::string failure;
  double equation_residual{0.0};
  double max_primal_infeasibility{0.0};
  double max_dual_infeasibility{0.0};
  double max_basic_reduced_cost{0.0};
  int worst_primal_row{-1};
  int worst_dual_col{-1};
};

struct IndexedVector {
  int dimension{0};
  std::vector<int> index;
  std::vector<double> value;
  // Coordinate-lookup table, built lazily by at(): most solve results are
  // only iterated over their support, while the few queried vectors amortize
  // one build across their queries. Entries need not be sorted.
  mutable std::vector<int> lookup_slot;

  void clear(int size) {
    dimension = size;
    index.clear();
    value.clear();
    lookup_slot.clear();
  }
  double at(int target) const {
    if (lookup_slot.empty()) {
      if (index.size() < 8) {
        for (std::size_t k = 0; k < index.size(); ++k) {
          if (index[k] == target) return value[k];
        }
        return 0.0;
      }
      build_lookup();
    }
    const std::size_t mask = lookup_slot.size() - 1;
    std::size_t slot =
        (static_cast<std::uint32_t>(target) * 0x9e3779b1u) & mask;
    for (;;) {
      const int position = lookup_slot[slot];
      if (position < 0) return 0.0;
      if (index[static_cast<std::size_t>(position)] == target)
        return value[static_cast<std::size_t>(position)];
      slot = (slot + 1) & mask;
    }
  }
  void build_lookup() const {
    std::size_t slot_count = 4;
    while (slot_count < index.size() * 2) slot_count *= 2;
    lookup_slot.assign(slot_count, -1);
    const std::size_t mask = slot_count - 1;
    for (std::size_t k = 0; k < index.size(); ++k) {
      std::size_t slot =
          (static_cast<std::uint32_t>(index[k]) * 0x9e3779b1u) & mask;
      while (lookup_slot[slot] >= 0) slot = (slot + 1) & mask;
      lookup_slot[slot] = static_cast<int>(k);
    }
  }
  bool finite() const {
    if (index.size() != value.size()) return false;
    for (double entry : value) {
      if (!std::isfinite(entry)) return false;
    }
    return true;
  }
};

struct Leaving {
  int row{-1};
  int side{0};
  double delta{0.0};
  double violation{0.0};
  IndexedVector row_ep;
  bool released_taboo_row{false};
  int taboo_row_rejections{0};
  bool row_ep_refined{false};
};

struct Entering {
  int col{-1};
  double pivot{0.0};
  double alpha{0.0};
  double theta{0.0};
};

struct BoundFlip {
  int col{-1};
  Move old_move{Move::Fixed};
  double range{0.0};
};

struct WorkingCostShift {
  int col{-1};
  double delta{0.0};
};

struct PivotTransaction {
  Entering entering;
  std::vector<BoundFlip> flips;
  std::vector<WorkingCostShift> cost_shifts;
  IndexedVector bfrt_rhs;
  double covered_violation{0.0};
  double dual_step_lower{0.0};
  double dual_step_upper{0.0};
  int taboo_rejections{0};
  bool all_harris_candidates_taboo{false};
  bool stability_blocked{false};
  int unstable_pivot_rejections{0};
  int harris_second_pass_candidates{0};
  // CHUZC terminal evidence. The three nested candidate domains distinguish
  // the mathematical sign condition from the dot-product certification and
  // the stronger pivot-stability threshold used for an actual basis update.
  int positive_candidate_count{0};
  int certified_candidate_count{0};
  int stable_candidate_count{0};
  double positive_capacity{0.0};
  double certified_capacity{0.0};
  double stable_capacity{0.0};
  double stable_capacity_error{0.0};
  // BFRT profiling evidence. These counters describe the exact candidate
  // ordering used by this transaction and do not participate in the pivot.
  int bfrt_candidate_count{0};
  int bfrt_group_count{0};
  int bfrt_selected_group_size{0};
  int bfrt_stability_prefiltered{0};
  double bfrt_sort_time_sec{0.0};
  double bfrt_order_time_sec{0.0};
};

struct Certificate {
  bool valid{false};
  Eigen::VectorXd multiplier;
  double attainable_lower{0.0};
  double attainable_upper{0.0};
  double rhs{0.0};
  double error_bound{0.0};
  double margin{0.0};
};

}  // namespace mipsolvers::engine::native_dual::detail
