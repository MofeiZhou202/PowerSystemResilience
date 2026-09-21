/// @file bc_branching.cpp
/// @brief Branch variable selection strategies for the B&C solver.

#include "mipsolvers/engine/detail/bc_utils.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iterator>
#include <limits>
#include <optional>
#include <vector>

#include <Eigen/Core>
#include "mipsolvers/engine/detail/bc_fallback.hpp"
#include "mipsolvers/engine/kernel/lp_kernel/dual_simplex.hpp"

namespace mipsolvers::engine::detail {

namespace {

template <typename PseudoCosts>
double pseudocost_branch_score(int j,
                               const Eigen::VectorXd& x,
                               const PseudoCosts& pc) {
  const double frac = x[j] - std::floor(x[j]);
  return compute_branch_product_score(pc[j], frac);
}

const BranchDirectionalObservation* find_observation(
    const std::vector<BranchDirectionalObservation>& observations,
    int var,
    bool is_up) {
  const auto it = std::find_if(
      observations.begin(), observations.end(), [&](const auto& observation) {
        return observation.var == var && observation.is_up == is_up;
      });
  return it == observations.end() ? nullptr : &*it;
}

int branch_priority_value(int j, const std::vector<int>& priority) {
  return (j >= 0 && j < static_cast<int>(priority.size())) ? priority[j] : 0;
}

bool collect_dynamic_prior_scores(const std::vector<int>& cand,
                                  const Eigen::VectorXd& x,
                                  const BCBranchingPriorFn& dynamic_prior,
                                  const BCBranchContext& context,
                                  std::vector<double>& mapped_scores) {
  if (!dynamic_prior || cand.empty()) return false;

  BCBranchContext local_context = context;
  if (local_context.candidates == nullptr) {
    local_context.candidates = &cand;
  }
  if (local_context.lp_x == nullptr || local_context.lp_x_size == 0) {
    local_context.lp_x = x.data();
    local_context.lp_x_size = static_cast<std::size_t>(x.size());
  }
  if (local_context.candidates->size() != cand.size()) return false;
  const auto& score_columns = *local_context.candidates;

  std::vector<double> raw_scores;
  try {
    dynamic_prior(local_context, raw_scores);
  } catch (...) {
    return false;
  }
  if (raw_scores.empty()) return false;

  mapped_scores.assign(cand.size(), 0.0);
  if (raw_scores.size() == cand.size()) {
    bool any = false;
    for (std::size_t k = 0; k < cand.size(); ++k) {
      const double score = raw_scores[k];
      if (!std::isfinite(score)) continue;
      mapped_scores[k] = score;
      any = any || std::abs(score) > 0.0;
    }
    return any;
  }

  int max_col = -1;
  for (int j : score_columns) max_col = std::max(max_col, j);
  if (max_col < 0 || static_cast<int>(raw_scores.size()) <= max_col) {
    return false;
  }
  bool any = false;
  for (std::size_t k = 0; k < cand.size(); ++k) {
    const int col = score_columns[k];
    if (col < 0) continue;
    const double score = raw_scores[static_cast<std::size_t>(col)];
    if (!std::isfinite(score)) continue;
    mapped_scores[k] = score;
    any = any || std::abs(score) > 0.0;
  }
  return any;
}

double most_infeasible_score(int j, const Eigen::VectorXd& x) {
  const double frac = std::abs(x[j] - std::floor(x[j]));
  return 0.5 - std::abs(frac - 0.5);
}

template <typename PseudoCosts>
double base_branch_score(BranchingStrategy strategy,
                         int ordinal,
                         int j,
                         const Eigen::VectorXd& x,
                         const PseudoCosts& pc) {
  switch (strategy) {
    case BranchingStrategy::FirstFractional:
      return -static_cast<double>(ordinal) * 1e-9;
    case BranchingStrategy::MostInfeasible:
      return most_infeasible_score(j, x);
    case BranchingStrategy::Pseudocost:
    default:
      return std::log1p(std::max(0.0, pseudocost_branch_score(j, x, pc)));
  }
}

template <typename PseudoCosts>
int choose_branch_var_with_dynamic_prior(const BCOptions& opt,
                                         const std::vector<int>& cand,
                                         const Eigen::VectorXd& x,
                                         const PseudoCosts& pc,
                                         const std::vector<int>& priority,
                                         const BCBranchingPriorFn& dynamic_prior,
                                         const BCBranchContext& context) {
  std::vector<double> prior_scores;
  if (!collect_dynamic_prior_scores(cand, x, dynamic_prior, context, prior_scores)) {
    return priority.empty() ? choose_branch_var(opt, cand, x, pc)
                            : choose_branch_var(opt, cand, x, pc, priority);
  }

  int best = cand.front();
  int best_priority = priority.empty()
                          ? std::numeric_limits<int>::min()
                          : branch_priority_value(best, priority);
  double best_score = -std::numeric_limits<double>::infinity();
  for (std::size_t k = 0; k < cand.size(); ++k) {
    const int j = cand[k];
    const int prio = priority.empty() ? 0 : branch_priority_value(j, priority);
    if (!priority.empty() && prio < best_priority) continue;
    const double base = base_branch_score(opt.branching, static_cast<int>(k), j, x, pc);
    const double combined = base + prior_scores[k];
    if ((!priority.empty() && prio > best_priority) ||
        combined > best_score ||
        (std::abs(combined - best_score) <= 1e-12 && j < best)) {
      best = j;
      best_priority = prio;
      best_score = combined;
    }
  }
  return best;
}

}  // namespace

double compute_directional_branch_score(
    const PseudoCost& pseudocost,
    double branch_distance,
    bool is_up,
    const BranchDirectionalObservation* observation,
    double prediction_offset) {
  constexpr double eps = 1e-6;
  const double history = is_up ? pseudocost.up_history_multiplier()
                               : pseudocost.down_history_multiplier();
  if (observation != nullptr) {
    switch (observation->state) {
      case BranchEvidenceState::Measured:
      case BranchEvidenceState::ExactOptimal:
        if (std::isfinite(observation->gain)) {
          return std::max(std::max(0.0, observation->gain) * history, eps);
        }
        break;
      case BranchEvidenceState::ProvenCutoff:
        return std::numeric_limits<double>::infinity();
      case BranchEvidenceState::Predicted:
      case BranchEvidenceState::UnknownFailure:
        break;
    }
  }
  const double base = is_up ? pseudocost.up_lcb() : pseudocost.down_lcb();
  return std::max(std::abs(branch_distance) *
                      std::max(base + prediction_offset, eps) * history,
                  eps);
}

double compute_branch_product_score(
    const PseudoCost& pseudocost,
    double fractionality,
    const BranchDirectionalObservation* down_observation,
    const BranchDirectionalObservation* up_observation) {
  const double down = compute_directional_branch_score(
      pseudocost, fractionality, false, down_observation);
  const double up = compute_directional_branch_score(
      pseudocost, 1.0 - fractionality, true, up_observation);
  return down * up;
}

bool choose_up_branch_first(
    const PseudoCost& pseudocost,
    double fractionality,
    const BranchDirectionalObservation* down_observation,
    const BranchDirectionalObservation* up_observation,
    bool has_incumbent,
    bool existing_prefer_up,
    int advisory_direction) {
  const bool down_cutoff =
      down_observation != nullptr &&
      down_observation->state == BranchEvidenceState::ProvenCutoff;
  const bool up_cutoff =
      up_observation != nullptr &&
      up_observation->state == BranchEvidenceState::ProvenCutoff;
  if (has_incumbent && down_cutoff != up_cutoff) return up_cutoff;

  if (!has_incumbent) {
    const double down = compute_directional_branch_score(
        pseudocost, fractionality, false, down_observation);
    const double up = compute_directional_branch_score(
        pseudocost, 1.0 - fractionality, true, up_observation);
    if (up < down) return true;
    if (down < up) return false;
  }
  if (advisory_direction != 0) return advisory_direction > 0;
  return existing_prefer_up;
}

BranchEstimatorCalibration compute_branch_estimator_calibration(
    double parent_bound,
    double parent_estimate,
    bool down_prediction_available,
    double predicted_down_gain,
    bool up_prediction_available,
    double predicted_up_gain,
    bool down_bound_available,
    double down_bound,
    bool up_bound_available,
    double up_bound) {
  BranchEstimatorCalibration calibration;
  if (!std::isfinite(parent_bound)) return calibration;

  const bool down_bound_usable =
      down_bound_available && std::isfinite(down_bound);
  const bool up_bound_usable = up_bound_available && std::isfinite(up_bound);
  const bool down_usable = down_bound_usable && down_prediction_available &&
                           std::isfinite(predicted_down_gain);
  const bool up_usable = up_bound_usable && up_prediction_available &&
                         std::isfinite(predicted_up_gain);
  auto add_direction = [&](double predicted, double child_bound) {
    const double realized = std::max(0.0, child_bound - parent_bound);
    const double error = predicted - realized;
    ++calibration.directional_samples;
    calibration.directional_predicted_gain_sum += predicted;
    calibration.directional_realized_gain_sum += realized;
    calibration.directional_abs_error_sum += std::abs(error);
    calibration.directional_squared_error_sum += error * error;
  };
  if (down_usable) add_direction(predicted_down_gain, down_bound);
  if (up_usable) add_direction(predicted_up_gain, up_bound);

  if (down_usable && up_usable) {
    const double realized_down = std::max(0.0, down_bound - parent_bound);
    const double realized_up = std::max(0.0, up_bound - parent_bound);
    ++calibration.directional_rank_samples;
    const double predicted_delta = predicted_down_gain - predicted_up_gain;
    const double realized_delta = realized_down - realized_up;
    if (predicted_delta == 0.0 || realized_delta == 0.0 ||
        std::signbit(predicted_delta) == std::signbit(realized_delta)) {
      ++calibration.directional_rank_concordant;
    }
  }

  if (std::isfinite(parent_estimate) &&
      (down_bound_usable || up_bound_usable)) {
    const double predicted = std::max(0.0, parent_estimate - parent_bound);
    double best_child_bound = std::numeric_limits<double>::infinity();
    if (down_bound_usable) {
      best_child_bound = std::min(best_child_bound, down_bound);
    }
    if (up_bound_usable) best_child_bound = std::min(best_child_bound, up_bound);
    const double realized = std::max(0.0, best_child_bound - parent_bound);
    const double error = predicted - realized;
    calibration.node_samples = 1;
    calibration.node_predicted_lift_sum = predicted;
    calibration.node_realized_lift_sum = realized;
    calibration.node_abs_error_sum = std::abs(error);
    calibration.node_squared_error_sum = error * error;
    calibration.node_predicted_sq_sum = predicted * predicted;
    calibration.node_realized_sq_sum = realized * realized;
    calibration.node_cross_sum = predicted * realized;
  }
  return calibration;
}

void update_pseudocost_from_branch_evidence(
    PseudoCost& pseudocost,
    bool is_up,
    BranchEvidenceState state,
    double gain,
    double branch_distance,
    bool certified_conflict) {
  if (state == BranchEvidenceState::Measured ||
      state == BranchEvidenceState::ExactOptimal) {
    const double unit_gain = normalized_pseudocost_gain(gain, branch_distance);
    if (is_up) pseudocost.add_up(unit_gain);
    else pseudocost.add_down(unit_gain);
    return;
  }
  if (state != BranchEvidenceState::ProvenCutoff) return;
  if (is_up) {
    pseudocost.add_up_cutoff();
    if (certified_conflict) pseudocost.add_up_conflict();
  } else {
    pseudocost.add_down_cutoff();
    if (certified_conflict) pseudocost.add_down_conflict();
  }
}

namespace {

std::uint64_t mix_reuse_signature(std::uint64_t seed, std::uint64_t value) {
  value += 0x9e3779b97f4a7c15ULL;
  value = (value ^ (value >> 30)) * 0xbf58476d1ce4e5b9ULL;
  value = (value ^ (value >> 27)) * 0x94d049bb133111ebULL;
  value ^= value >> 31;
  return seed ^ (value + 0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2));
}

std::uint64_t double_signature_bits(double value) {
  if (value == 0.0) value = 0.0;
  std::uint64_t bits = 0;
  static_assert(sizeof(bits) == sizeof(value));
  std::memcpy(&bits, &value, sizeof(bits));
  return bits;
}

bool same_reuse_bound(double lhs, double rhs, double tolerance) {
  if (!std::isfinite(lhs) || !std::isfinite(rhs)) {
    return std::isfinite(lhs) == std::isfinite(rhs) &&
           std::signbit(lhs) == std::signbit(rhs);
  }
  return std::abs(lhs - rhs) <= tolerance;
}

}  // namespace

std::uint64_t ordered_local_cut_signature(const std::vector<PoolCut>& cuts) {
  std::uint64_t signature = mix_reuse_signature(0, cuts.size());
  for (std::size_t row = 0; row < cuts.size(); ++row) {
    const PoolCut& cut = cuts[row];
    signature = mix_reuse_signature(signature, row);
    signature = mix_reuse_signature(signature,
                                    static_cast<std::uint64_t>(cut.coeff.size()));
    signature = mix_reuse_signature(
        signature, static_cast<std::uint64_t>(cut.coeff.nonZeros()));
    for (Eigen::SparseVector<double>::InnerIterator it(cut.coeff); it; ++it) {
      signature = mix_reuse_signature(
          signature, static_cast<std::uint64_t>(it.index()));
      signature = mix_reuse_signature(signature,
                                      double_signature_bits(it.value()));
    }
    signature = mix_reuse_signature(signature, double_signature_bits(cut.rhs));
    signature = mix_reuse_signature(signature,
                                    static_cast<std::uint64_t>(cut.domain_only));
    signature = mix_reuse_signature(
        signature, static_cast<std::uint64_t>(cut.validity_scope));
  }
  return signature;
}

BranchProbeReuseKind classify_branch_probe_reuse(
    bool available,
    bool consumed,
    int probe_var,
    bool probe_is_up,
    int branch_var,
    bool child_is_up,
    const Eigen::VectorXd& probe_lb,
    const Eigen::VectorXd& probe_ub,
    const Eigen::VectorXd& child_lb,
    const Eigen::VectorXd& child_ub,
    std::uint64_t probe_row_signature,
    std::uint64_t child_row_signature,
    std::uint64_t probe_model_epoch,
    std::uint64_t child_model_epoch,
    std::uint64_t probe_objective_epoch,
    std::uint64_t child_objective_epoch,
    bool proof_usable,
    bool warm_state_usable,
    double tolerance) {
  if (!available || consumed || probe_var != branch_var ||
      probe_is_up != child_is_up || probe_row_signature != child_row_signature ||
      probe_model_epoch != child_model_epoch ||
      probe_objective_epoch != child_objective_epoch ||
      probe_lb.size() != child_lb.size() || probe_ub.size() != child_ub.size() ||
      probe_lb.size() != probe_ub.size()) {
    return BranchProbeReuseKind::None;
  }

  const double tol = std::max(0.0, tolerance);
  bool same_domain = true;
  bool child_is_subset = true;
  for (int j = 0; j < probe_lb.size(); ++j) {
    const bool same_lb = same_reuse_bound(probe_lb[j], child_lb[j], tol);
    const bool same_ub = same_reuse_bound(probe_ub[j], child_ub[j], tol);
    same_domain = same_domain && same_lb && same_ub;
    if (!same_lb) {
      if (!std::isfinite(child_lb[j])) {
        child_is_subset = false;
      } else if (std::isfinite(probe_lb[j]) &&
                 child_lb[j] < probe_lb[j] - tol) {
        child_is_subset = false;
      }
    }
    if (!same_ub) {
      if (!std::isfinite(child_ub[j])) {
        child_is_subset = false;
      } else if (std::isfinite(probe_ub[j]) &&
                 child_ub[j] > probe_ub[j] + tol) {
        child_is_subset = false;
      }
    }
  }
  if (same_domain && proof_usable) return BranchProbeReuseKind::Exact;
  if (child_is_subset && warm_state_usable) return BranchProbeReuseKind::Warm;
  return BranchProbeReuseKind::None;
}

template <typename PseudoCosts>
double branch_var_score_with_overlay_impl(
    int j,
    const Eigen::VectorXd& x,
    const PseudoCosts& pc,
    const std::vector<BranchDirectionalObservation>& observations) {
  const double frac = x[j] - std::floor(x[j]);
  return compute_branch_product_score(
      pc[j], frac, find_observation(observations, j, false),
      find_observation(observations, j, true));
}

double compute_branch_var_score_with_overlay(
    int j,
    const Eigen::VectorXd& x,
    const std::vector<PseudoCost>& pc,
    const std::vector<BranchDirectionalObservation>& observations) {
  return branch_var_score_with_overlay_impl(j, x, pc, observations);
}

double compute_branch_var_score_with_overlay(
    int j,
    const Eigen::VectorXd& x,
    const CompactPseudoCostTable& pc,
    const std::vector<BranchDirectionalObservation>& observations) {
  return branch_var_score_with_overlay_impl(j, x, pc, observations);
}

template <typename PseudoCosts>
int choose_branch_var_pseudocost_with_overlay_impl(
    const std::vector<int>& cand,
    const Eigen::VectorXd& x,
    const PseudoCosts& pc,
    const std::vector<int>& priority,
    const std::vector<BranchDirectionalObservation>& observations,
    const BCBranchingPriorFn& dynamic_prior,
    const BCBranchContext& context) {
  if (cand.empty()) return -1;
  std::vector<double> prior_scores;
  const bool has_dynamic_prior = collect_dynamic_prior_scores(
      cand, x, dynamic_prior, context, prior_scores);
  int best = cand.front();
  int best_priority = priority.empty()
                          ? 0
                          : branch_priority_value(best, priority);
  double best_score = -std::numeric_limits<double>::infinity();
  for (std::size_t k = 0; k < cand.size(); ++k) {
    const int j = cand[k];
    const int candidate_priority =
        priority.empty() ? 0 : branch_priority_value(j, priority);
    if (!priority.empty() && candidate_priority < best_priority) continue;
    const double product =
        branch_var_score_with_overlay_impl(j, x, pc, observations);
    const double score = std::log1p(std::max(0.0, product)) +
                         (has_dynamic_prior ? prior_scores[k] : 0.0);
    if ((!priority.empty() && candidate_priority > best_priority) ||
        score > best_score ||
        (has_dynamic_prior && std::abs(score - best_score) <= 1e-12 &&
         j < best)) {
      best = j;
      best_priority = candidate_priority;
      best_score = score;
    }
  }
  return best;
}

int choose_branch_var_pseudocost_with_overlay(
    const std::vector<int>& cand,
    const Eigen::VectorXd& x,
    const std::vector<PseudoCost>& pc,
    const std::vector<int>& priority,
    const std::vector<BranchDirectionalObservation>& observations,
    const BCBranchingPriorFn& dynamic_prior,
    const BCBranchContext& context) {
  return choose_branch_var_pseudocost_with_overlay_impl(
      cand, x, pc, priority, observations, dynamic_prior, context);
}

int choose_branch_var_pseudocost_candidates(
    const std::vector<int>& cand,
    const Eigen::VectorXd& x,
    const std::vector<PseudoCost>& candidate_pc,
    const std::vector<int>& priority,
    const BCBranchingPriorFn& dynamic_prior,
    const BCBranchContext& context,
    const std::vector<BranchDirectionalObservation>& observations) {
  if (cand.empty() || candidate_pc.size() != cand.size()) return -1;

  std::vector<double> prior_scores;
  const bool has_dynamic_prior = collect_dynamic_prior_scores(
      cand, x, dynamic_prior, context, prior_scores);
  int best = cand.front();
  int best_priority =
      priority.empty() ? 0 : branch_priority_value(best, priority);
  double best_score = -std::numeric_limits<double>::infinity();
  for (std::size_t k = 0; k < cand.size(); ++k) {
    const int candidate = cand[k];
    const int candidate_priority =
        priority.empty() ? 0 : branch_priority_value(candidate, priority);
    if (!priority.empty() && candidate_priority < best_priority) continue;
    const double fractionality = x[candidate] - std::floor(x[candidate]);
    const double product = compute_branch_product_score(
        candidate_pc[k], fractionality,
        find_observation(observations, candidate, false),
        find_observation(observations, candidate, true));
    const double score = std::log1p(std::max(0.0, product)) +
                         (has_dynamic_prior ? prior_scores[k] : 0.0);
    if ((!priority.empty() && candidate_priority > best_priority) ||
        score > best_score ||
        (has_dynamic_prior && std::abs(score - best_score) <= 1e-12 &&
         candidate < best)) {
      best = candidate;
      best_priority = candidate_priority;
      best_score = score;
    }
  }
  return best;
}

int choose_branch_var_pseudocost_with_overlay(
    const std::vector<int>& cand,
    const Eigen::VectorXd& x,
    const CompactPseudoCostTable& pc,
    const std::vector<int>& priority,
    const std::vector<BranchDirectionalObservation>& observations,
    const BCBranchingPriorFn& dynamic_prior,
    const BCBranchContext& context) {
  return choose_branch_var_pseudocost_with_overlay_impl(
      cand, x, pc, priority, observations, dynamic_prior, context);
}

double compute_branch_var_score(int j,
                                const Eigen::VectorXd& x,
                                const std::vector<PseudoCost>& pc,
                                const std::vector<int>* priority) {
  (void)priority;
  return pseudocost_branch_score(j, x, pc);
}

double compute_branch_var_score(int j,
                                const Eigen::VectorXd& x,
                                const CompactPseudoCostTable& pc,
                                const std::vector<int>* priority) {
  (void)priority;
  return pseudocost_branch_score(j, x, pc);
}

int choose_branch_var_most_infeasible(const std::vector<int>& cand,
                                      const Eigen::VectorXd& x) {
  int best = cand.front();
  double best_score = -1.0;
  for (int j : cand) {
    const double frac = std::abs(x[j] - std::floor(x[j]));
    const double s = 0.5 - std::abs(frac - 0.5);
    if (s > best_score) {
      best_score = s;
      best = j;
    }
  }
  return best;
}

int choose_branch_var_pseudocost(const std::vector<int>& cand,
                                 const Eigen::VectorXd& x,
                                 const std::vector<PseudoCost>& pc) {
  int best = cand.front();
  double best_score = -1.0;
  for (int j : cand) {
    const double score = pseudocost_branch_score(j, x, pc);
    if (score > best_score) {
      best_score = score;
      best = j;
    }
  }
  return best;
}

int choose_branch_var_pseudocost(const std::vector<int>& cand,
                                 const Eigen::VectorXd& x,
                                 const CompactPseudoCostTable& pc) {
  int best = cand.front();
  double best_score = -1.0;
  for (int j : cand) {
    const double score = pseudocost_branch_score(j, x, pc);
    if (score > best_score) {
      best_score = score;
      best = j;
    }
  }
  return best;
}

int choose_branch_var_pseudocost(const std::vector<int>& cand,
                                 const Eigen::VectorXd& x,
                                 const std::vector<PseudoCost>& pc,
                                 const std::vector<int>& priority) {
  if (priority.empty()) return choose_branch_var_pseudocost(cand, x, pc);
  int best = cand.front();
  double best_score = -1.0;
  int best_priority = branch_priority_value(best, priority);
  for (int j : cand) {
    const int prio = branch_priority_value(j, priority);
    const double score = pseudocost_branch_score(j, x, pc);
    if (prio > best_priority || (prio == best_priority && score > best_score)) {
      best_score = score;
      best = j;
      best_priority = prio;
    }
  }
  return best;
}

int choose_branch_var_pseudocost(const std::vector<int>& cand,
                                 const Eigen::VectorXd& x,
                                 const CompactPseudoCostTable& pc,
                                 const std::vector<int>& priority) {
  if (priority.empty()) return choose_branch_var_pseudocost(cand, x, pc);
  int best = cand.front();
  double best_score = -1.0;
  int best_priority = branch_priority_value(best, priority);
  for (int j : cand) {
    const int prio = branch_priority_value(j, priority);
    const double score = pseudocost_branch_score(j, x, pc);
    if (prio > best_priority || (prio == best_priority && score > best_score)) {
      best_score = score;
      best = j;
      best_priority = prio;
    }
  }
  return best;
}

int choose_branch_var_pseudocost(const std::vector<int>& cand,
                                 const Eigen::VectorXd& x,
                                 const std::vector<PseudoCost>& pc,
                                 const std::vector<int>& priority,
                                 const BCBranchingPriorFn& dynamic_prior,
                                 const BCBranchContext& context) {
  if (cand.empty()) return -1;
  if (!dynamic_prior) return choose_branch_var_pseudocost(cand, x, pc, priority);
  BCOptions opt;
  opt.branching = BranchingStrategy::Pseudocost;
  return choose_branch_var_with_dynamic_prior(
      opt, cand, x, pc, priority, dynamic_prior, context);
}

int choose_branch_var_pseudocost(const std::vector<int>& cand,
                                 const Eigen::VectorXd& x,
                                 const CompactPseudoCostTable& pc,
                                 const std::vector<int>& priority,
                                 const BCBranchingPriorFn& dynamic_prior,
                                 const BCBranchContext& context) {
  if (cand.empty()) return -1;
  if (!dynamic_prior) return choose_branch_var_pseudocost(cand, x, pc, priority);
  BCOptions opt;
  opt.branching = BranchingStrategy::Pseudocost;
  return choose_branch_var_with_dynamic_prior(
      opt, cand, x, pc, priority, dynamic_prior, context);
}

int choose_branch_var(const BCOptions& opt,
                      const std::vector<int>& cand,
                      const Eigen::VectorXd& x,
                      const std::vector<PseudoCost>& pc) {
  if (cand.empty()) {
    return -1;
  }
  switch (opt.branching) {
    case BranchingStrategy::FirstFractional:
      return cand.front();
    case BranchingStrategy::MostInfeasible:
      return choose_branch_var_most_infeasible(cand, x);
    case BranchingStrategy::Pseudocost:
    default:
      return choose_branch_var_pseudocost(cand, x, pc);
  }
}

int choose_branch_var(const BCOptions& opt,
                      const std::vector<int>& cand,
                      const Eigen::VectorXd& x,
                      const CompactPseudoCostTable& pc) {
  if (cand.empty()) return -1;
  switch (opt.branching) {
    case BranchingStrategy::FirstFractional:
      return cand.front();
    case BranchingStrategy::MostInfeasible:
      return choose_branch_var_most_infeasible(cand, x);
    case BranchingStrategy::Pseudocost:
    default:
      return choose_branch_var_pseudocost(cand, x, pc);
  }
}

int choose_branch_var(const BCOptions& opt,
                      const std::vector<int>& cand,
                      const Eigen::VectorXd& x,
                      const std::vector<PseudoCost>& pc,
                      const std::vector<int>& priority) {
  if (cand.empty()) {
    return -1;
  }
  switch (opt.branching) {
    case BranchingStrategy::FirstFractional:
      return cand.front();
    case BranchingStrategy::MostInfeasible:
      return choose_branch_var_most_infeasible(cand, x);
    case BranchingStrategy::Pseudocost:
    default:
      return choose_branch_var_pseudocost(cand, x, pc, priority);
  }
}

int choose_branch_var(const BCOptions& opt,
                      const std::vector<int>& cand,
                      const Eigen::VectorXd& x,
                      const CompactPseudoCostTable& pc,
                      const std::vector<int>& priority) {
  if (cand.empty()) return -1;
  switch (opt.branching) {
    case BranchingStrategy::FirstFractional:
      return cand.front();
    case BranchingStrategy::MostInfeasible:
      return choose_branch_var_most_infeasible(cand, x);
    case BranchingStrategy::Pseudocost:
    default:
      return choose_branch_var_pseudocost(cand, x, pc, priority);
  }
}

int choose_branch_var(const BCOptions& opt,
                      const std::vector<int>& cand,
                      const Eigen::VectorXd& x,
                      const std::vector<PseudoCost>& pc,
                      const std::vector<int>& priority,
                      const BCBranchingPriorFn& dynamic_prior,
                      const BCBranchContext& context) {
  if (cand.empty()) {
    return -1;
  }
  if (!dynamic_prior) {
    return choose_branch_var(opt, cand, x, pc, priority);
  }
  return choose_branch_var_with_dynamic_prior(
      opt, cand, x, pc, priority, dynamic_prior, context);
}

int choose_branch_var(const BCOptions& opt,
                      const std::vector<int>& cand,
                      const Eigen::VectorXd& x,
                      const CompactPseudoCostTable& pc,
                      const std::vector<int>& priority,
                      const BCBranchingPriorFn& dynamic_prior,
                      const BCBranchContext& context) {
  if (cand.empty()) return -1;
  if (!dynamic_prior) return choose_branch_var(opt, cand, x, pc, priority);
  return choose_branch_var_with_dynamic_prior(
      opt, cand, x, pc, priority, dynamic_prior, context);
}

template <typename PseudoCosts>
int choose_branch_var_reliability_impl(
    const std::vector<int>& cand,
    const Eigen::VectorXd& x,
    PseudoCosts& pc,
    const LPModel& base_lp,
    const StandardFormLP& base_sf,
    const Eigen::VectorXd& node_lb,
    const Eigen::VectorXd& node_ub,
    const SimplexBasis* basis_hint,
    const SimplexOptions& simplex_opt,
    double parent_bound,
    int& lp_solves,
    int node_depth,
    int reliability_limit,
    int max_probes,
    const std::vector<int>& priority,
    const BCBranchingPriorFn& dynamic_prior,
    const BCBranchContext& context,
    std::vector<BranchDirectionalObservation>* selected_observations,
    StrongBranchProbeStorageStats* storage_stats) {

  if (selected_observations != nullptr) selected_observations->clear();

  const std::vector<BranchDirectionalObservation> no_observations;
  auto select_with_observations =
      [&](const std::vector<BranchDirectionalObservation>& observations) {
        return choose_branch_var_pseudocost_with_overlay(
            cand, x, pc, priority, observations, dynamic_prior, context);
      };

  if (node_depth > 6) {
    return select_with_observations(no_observations);
  }

  // max_probes=0 means skip probing entirely (pure pseudocost).
  if (max_probes == 0) {
    return select_with_observations(no_observations);
  }

  // Scale probing budget with depth: aggressive at root, lighter deeper.
  const int effective_probes = (node_depth == 0) ? max_probes
                             : std::max(2, max_probes / 2);

  std::vector<BranchDirectionalObservation> observations;
  observations.reserve(static_cast<std::size_t>(2 * effective_probes));
  std::optional<StandardFormLP> active_probe_sf;
  std::optional<SimplexBasis> active_probe_basis;
  SimplexOptions persistent_probe_opt = simplex_opt;
  persistent_probe_opt.allow_persistent_lp_state = true;

  auto has_observation = [&](int var, bool is_up) {
    return find_observation(observations, var, is_up) != nullptr;
  };
  auto reliability_limit_for = [&](int var) {
    return (pc[var].down_cnt >= 1 && pc[var].up_cnt >= 1)
               ? std::max(2, reliability_limit / 2)
               : reliability_limit;
  };
  auto direction_reliable = [&](int var, bool is_up) {
    const int count = is_up ? pc[var].up_cnt : pc[var].down_cnt;
    return count >= reliability_limit_for(var);
  };
  auto probe_direction = [&](int var, bool is_up) {
    const double xj = x[var];
    const double floor_xj = std::floor(xj);
    const double distance = is_up ? floor_xj + 1.0 - xj : xj - floor_xj;
    Eigen::VectorXd probe_lb = node_lb;
    Eigen::VectorXd probe_ub = node_ub;
    if (is_up) probe_lb[var] = std::max(probe_lb[var], floor_xj + 1.0);
    else probe_ub[var] = std::min(probe_ub[var], floor_xj);

    BranchDirectionalObservation observation;
    observation.var = var;
    observation.is_up = is_up;
    if (probe_lb[var] > probe_ub[var] + 1e-12) {
      observation.state = BranchEvidenceState::ProvenCutoff;
      update_pseudocost_from_branch_evidence(
          pc[var], is_up, observation.state, 0.0, distance);
      observations.push_back(observation);
      return;
    }

    if (!active_probe_sf.has_value()) {
      active_probe_sf.emplace(base_sf);
      update_standard_form_bounds(
          *active_probe_sf, base_lp, node_lb, node_ub);
      if (storage_stats != nullptr) {
        ++storage_stats->base_sf_materializations;
      }
    }
    const double old_value = is_up ? node_lb[var] : node_ub[var];
    const double new_value = is_up ? probe_lb[var] : probe_ub[var];
    const std::vector<BoundChangeInfo> branch_change{
        {var, 0.0, is_up, old_value, new_value}};
    StandardFormBoundTransaction transaction(*active_probe_sf);
    if (!transaction.apply(base_lp, branch_change)) {
      if (storage_stats != nullptr) {
        ++storage_stats->transaction_failures;
      }
      observation.state = BranchEvidenceState::UnknownFailure;
      update_pseudocost_from_branch_evidence(
          pc[var], is_up, observation.state, 0.0, distance);
      observations.push_back(observation);
      return;
    }
    if (storage_stats != nullptr) {
      ++storage_stats->bound_transactions;
      storage_stats->transaction_snapshot_values +=
          transaction.snapshot_value_count();
    }
    const SimplexBasis* effective_basis_hint =
        active_probe_basis.has_value() ? &*active_probe_basis : basis_hint;
    auto res = solve_lp_from_sf(
        *active_probe_sf, persistent_probe_opt, effective_basis_hint);
    ++lp_solves;
    if (storage_stats != nullptr) {
      if (res.result.stats.solver_name ==
          "VendoredHighsPersistentLpKernel") {
        ++storage_stats->backend_persistent_resolves;
      } else {
        ++storage_stats->backend_cold_solves;
      }
    }
    if (res.basis.cached_sparse_basis) {
      active_probe_basis = res.basis;
    }
    if (transaction.rollback()) {
      if (storage_stats != nullptr) {
        ++storage_stats->transaction_rollbacks;
      }
    } else {
      if (storage_stats != nullptr) {
        ++storage_stats->transaction_failures;
      }
      active_probe_sf.reset();
    }
    const auto failure =
        classify_lp_result(res, static_cast<int>(base_lp.vars.size()));
    if (failure == LPFailureType::Success) {
      observation.state = res.exact_optimal ? BranchEvidenceState::ExactOptimal
                                            : BranchEvidenceState::Measured;
      observation.gain =
          std::max(0.0, res.result.stats.objective - parent_bound);
    } else if (failure == LPFailureType::ObjectiveCutoff ||
               (failure == LPFailureType::Infeasible &&
                res.result.stats.has_farkas_certificate)) {
      observation.state = BranchEvidenceState::ProvenCutoff;
    } else {
      observation.state = BranchEvidenceState::UnknownFailure;
    }
    update_pseudocost_from_branch_evidence(
        pc[var], is_up, observation.state, observation.gain, distance,
        failure == LPFailureType::Infeasible &&
            res.result.stats.has_farkas_certificate);
    observations.push_back(observation);
  };

  int candidate_probes = 0;
  while (candidate_probes < effective_probes) {
    const int current = select_with_observations(observations);
    if (current < 0) break;
    const bool need_down =
        !direction_reliable(current, false) && !has_observation(current, false);
    const bool need_up =
        !direction_reliable(current, true) && !has_observation(current, true);
    if (!need_down && !need_up) break;
    if (need_down) probe_direction(current, false);
    if (need_up) probe_direction(current, true);
    ++candidate_probes;
  }

  const int selected = select_with_observations(observations);
  if (selected_observations != nullptr && selected >= 0) {
    std::copy_if(observations.begin(), observations.end(),
                 std::back_inserter(*selected_observations),
                 [&](const auto& observation) {
                   return observation.var == selected;
                 });
  }
  return selected;
}

int choose_branch_var_reliability(
    const std::vector<int>& cand,
    const Eigen::VectorXd& x,
    std::vector<PseudoCost>& pc,
    const LPModel& base_lp,
    const StandardFormLP& base_sf,
    const Eigen::VectorXd& node_lb,
    const Eigen::VectorXd& node_ub,
    const SimplexBasis* basis_hint,
    const SimplexOptions& simplex_opt,
    double parent_bound,
    int& lp_solves,
    int node_depth,
    int reliability_limit,
    int max_probes,
    const std::vector<int>& priority,
    const BCBranchingPriorFn& dynamic_prior,
    const BCBranchContext& context,
    std::vector<BranchDirectionalObservation>* selected_observations,
    StrongBranchProbeStorageStats* storage_stats) {
  return choose_branch_var_reliability_impl(
      cand, x, pc, base_lp, base_sf, node_lb, node_ub, basis_hint,
      simplex_opt, parent_bound, lp_solves, node_depth, reliability_limit,
      max_probes, priority, dynamic_prior, context, selected_observations,
      storage_stats);
}

int choose_branch_var_reliability(
    const std::vector<int>& cand,
    const Eigen::VectorXd& x,
    CompactPseudoCostTable& pc,
    const LPModel& base_lp,
    const StandardFormLP& base_sf,
    const Eigen::VectorXd& node_lb,
    const Eigen::VectorXd& node_ub,
    const SimplexBasis* basis_hint,
    const SimplexOptions& simplex_opt,
    double parent_bound,
    int& lp_solves,
    int node_depth,
    int reliability_limit,
    int max_probes,
    const std::vector<int>& priority,
    const BCBranchingPriorFn& dynamic_prior,
    const BCBranchContext& context,
    std::vector<BranchDirectionalObservation>* selected_observations,
    StrongBranchProbeStorageStats* storage_stats) {
  return choose_branch_var_reliability_impl(
      cand, x, pc, base_lp, base_sf, node_lb, node_ub, basis_hint,
      simplex_opt, parent_bound, lp_solves, node_depth, reliability_limit,
      max_probes, priority, dynamic_prior, context, selected_observations,
      storage_stats);
}

double compute_node_estimate(const std::vector<VariableMeta>& vars,
                             const Eigen::VectorXd& x,
                             const std::vector<PseudoCost>& pc,
                             double int_tol,
                             double node_bound,
                             NodeEstimateAggregation aggregation) {
  if (!std::isfinite(node_bound)) {
    return kInf;
  }

  std::vector<int> frac;
  if (!fractional_indices(vars, x, int_tol, frac)) {
    return node_bound;
  }

  const double offset = std::max(
      1e-6,
      int_tol * std::max(1.0, std::abs(node_bound)) /
          std::max(1, static_cast<int>(frac.size())));

  double lift = 0.0;
  for (int j : frac) {
    const double frac_part = x[j] - std::floor(x[j]);
    const double down = compute_directional_branch_score(
        pc[j], frac_part, false, nullptr, offset);
    const double up = compute_directional_branch_score(
        pc[j], 1.0 - frac_part, true, nullptr, offset);
    const double variable_lift = std::min(down, up);
    if (aggregation == NodeEstimateAggregation::Maximum) {
      lift = std::max(lift, variable_lift);
    } else {
      lift += variable_lift;
    }
  }

  return std::max(node_bound, node_bound + lift);
}

double compute_node_estimate(const std::vector<int>& branchable_cols,
                             const Eigen::VectorXd& x,
                             const std::vector<PseudoCost>& pc,
                             double int_tol,
                             double node_bound,
                             NodeEstimateAggregation aggregation) {
  if (!std::isfinite(node_bound)) return kInf;

  int fractional_count = 0;
  for (int j : branchable_cols) {
    if (j >= 0 && j < x.size() && j < static_cast<int>(pc.size()) &&
        !is_integral(x[j], int_tol)) {
      ++fractional_count;
    }
  }
  if (fractional_count == 0) return node_bound;

  const double offset = std::max(
      1e-6,
      int_tol * std::max(1.0, std::abs(node_bound)) / fractional_count);
  double lift = 0.0;
  for (int j : branchable_cols) {
    if (j < 0 || j >= x.size() || j >= static_cast<int>(pc.size()) ||
        is_integral(x[j], int_tol)) {
      continue;
    }
    const double frac_part = x[j] - std::floor(x[j]);
    const double down = compute_directional_branch_score(
        pc[j], frac_part, false, nullptr, offset);
    const double up = compute_directional_branch_score(
        pc[j], 1.0 - frac_part, true, nullptr, offset);
    const double variable_lift = std::min(down, up);
    if (aggregation == NodeEstimateAggregation::Maximum) {
      lift = std::max(lift, variable_lift);
    } else {
      lift += variable_lift;
    }
  }
  return std::max(node_bound, node_bound + lift);
}

double compute_node_estimate(const std::vector<int>& branchable_cols,
                             const Eigen::VectorXd& x,
                             const CompactPseudoCostTable& pc,
                             double int_tol,
                             double node_bound,
                             NodeEstimateAggregation aggregation) {
  if (!std::isfinite(node_bound)) return kInf;

  int fractional_count = 0;
  for (int j : branchable_cols) {
    if (j >= 0 && j < x.size() && j < static_cast<int>(pc.size()) &&
        !is_integral(x[j], int_tol)) {
      ++fractional_count;
    }
  }
  if (fractional_count == 0) return node_bound;

  const double offset = std::max(
      1e-6,
      int_tol * std::max(1.0, std::abs(node_bound)) / fractional_count);
  double lift = 0.0;
  for (int j : branchable_cols) {
    if (j < 0 || j >= x.size() || j >= static_cast<int>(pc.size()) ||
        is_integral(x[j], int_tol)) {
      continue;
    }
    const double frac_part = x[j] - std::floor(x[j]);
    const double down = compute_directional_branch_score(
        pc[j], frac_part, false, nullptr, offset);
    const double up = compute_directional_branch_score(
        pc[j], 1.0 - frac_part, true, nullptr, offset);
    const double variable_lift = std::min(down, up);
    if (aggregation == NodeEstimateAggregation::Maximum) {
      lift = std::max(lift, variable_lift);
    } else {
      lift += variable_lift;
    }
  }
  return std::max(node_bound, node_bound + lift);
}

}  // namespace mipsolvers::engine::detail
