#pragma once

#include <cstdint>
#include <cstdlib>
#include <map>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

#include <Eigen/Core>

#include "../native_dual_core.hpp"

namespace mipsolvers::engine::native_dual::detail {

enum class Phase { One, Two };

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

// Cadence (in pivots since the last refactorization) of the periodic
// full-primal-residual audit in minor_iteration.  The tiered per-solve
// backward-error validation aligns to this stride so that on every audit pivot
// the hot-path solves are also fully verified.
inline constexpr int kNativeDualAuditStride = 16;

// Whether the current pivot's hot-path checked solves (choose_leaving row_ep
// BTRAN, pivotal-column FTRAN) should run the O(nnz) backward-error residual
// audit.  Strict posture (tier_checked_solves == false) always verifies; the
// tiered posture verifies only on the periodic audit stride, trusting the raw
// LU solve in between (finiteness is still guarded and drift is caught by the
// periodic full-residual audit + reinvert).  Env
// MIPSOLVERS_DS_TIER_CHECKED_SOLVES (0 = force strict, 1 = force tiered)
// overrides the option, mirroring MIPSOLVERS_DS_PARANOID.
inline bool should_verify_checked_solve(const State& state) {
  static const int env_override = [] {
    const char* e = std::getenv("MIPSOLVERS_DS_TIER_CHECKED_SOLVES");
    return e ? (e[0] == '0' ? 0 : 1) : -1;
  }();
  const bool tier =
      env_override >= 0
          ? (env_override != 0)
          : (state.options != nullptr && state.options->tier_checked_solves);
  return !tier || (state.updates_since_rebuild % kNativeDualAuditStride == 0);
}

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

struct Leaving {
  int row{-1};
  int side{0};
  double delta{0.0};
  double violation{0.0};
  Eigen::VectorXd row_ep;
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
  Eigen::VectorXd bfrt_rhs;
  double covered_violation{0.0};
  double dual_step_lower{0.0};
  double dual_step_upper{0.0};
  int taboo_rejections{0};
  bool all_harris_candidates_taboo{false};
  bool stability_blocked{false};
  int unstable_pivot_rejections{0};
  int harris_second_pass_candidates{0};
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
