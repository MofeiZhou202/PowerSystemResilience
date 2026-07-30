#include "pricing.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <iomanip>
#include <sstream>
#include <vector>

namespace mipsolvers::engine::native_dual::detail {
namespace {

struct Candidate {
  int col{-1};
  double pivot{0.0};
  double alpha{0.0};
  double margin{0.0};
  long double breakpoint{0.0};
  double range{std::numeric_limits<double>::infinity()};
  bool taboo{false};
  int taboo_expiry{-1};
};

double dot_error_bound(const State& state, const Eigen::VectorXd& row_ep,
                       int col) {
  double absolute_dot = 0.0;
  int terms = 0;
  for (Eigen::SparseMatrix<double>::InnerIterator it(state.sf->A, col); it;
       ++it) {
    absolute_dot += std::abs(row_ep[it.row()] * it.value());
    ++terms;
  }
  const double eps = std::numeric_limits<double>::epsilon();
  const double product = terms * eps;
  const double gamma = product < 0.5 ? product / (1.0 - product) : 1.0;
  if (absolute_dot == 0.0) return 0.0;
  return gamma * absolute_dot + 256.0 * eps * absolute_dot;
}

}  // namespace

bool choose_leaving(State& state, Leaving& leaving, std::string& failure) {
  leaving = {};
  double best_merit = -1.0;
  double taboo_merit = -1.0;
  int taboo_expiry = std::numeric_limits<int>::max();
  Leaving taboo_fallback;
  for (int row = 0; row < state.m; ++row) {
    int side = 0;
    const double violation = primal_infeasibility(state, row, side);
    if (side == 0) continue;
    const double weight = state.edge_weight[static_cast<std::size_t>(row)];
    if (!(weight > 0.0) || !std::isfinite(weight)) {
      failure = "CHUZR encountered an invalid DSE weight";
      return false;
    }
    const double merit = violation * violation / weight;
    const int leaving_col = state.basis[static_cast<std::size_t>(row)];
    int expiry = -1;
    if (is_taboo_row(state, leaving_col, &expiry)) {
      ++leaving.taboo_row_rejections;
      if (expiry < taboo_expiry ||
          (expiry == taboo_expiry &&
           (merit > taboo_merit ||
            (merit == taboo_merit &&
             (taboo_fallback.row < 0 || row < taboo_fallback.row))))) {
        taboo_expiry = expiry;
        taboo_merit = merit;
        taboo_fallback.row = row;
        taboo_fallback.side = side;
        taboo_fallback.violation = violation;
        taboo_fallback.delta = side * violation;
      }
      continue;
    }
    if (merit > best_merit ||
        (merit == best_merit && (leaving.row < 0 || row < leaving.row))) {
      best_merit = merit;
      leaving.row = row;
      leaving.side = side;
      leaving.violation = violation;
      leaving.delta = side * violation;
    }
  }
  if (leaving.row < 0 && taboo_fallback.row >= 0) {
    const int rejections = leaving.taboo_row_rejections;
    leaving = taboo_fallback;
    leaving.released_taboo_row = true;
    leaving.taboo_row_rejections = rejections;
  }
  if (leaving.row < 0) return true;

  Eigen::VectorXd unit = Eigen::VectorXd::Zero(state.m);
  unit[leaving.row] = 1.0;
  const SolveEvidence row_solve =
      state.factor->checked_btran(unit, true, should_verify_checked_solve(state));
  leaving.row_ep = row_solve.solution;
  leaving.row_ep_refined = row_solve.refined;
  if (!row_solve.accepted) {
    failure = "CHUZR BTRAN failed backward-error validation (" +
              state.factor->last_solve_diagnostics() + ')';
    return false;
  }
  if (state.edge_weight_mode == EdgeWeightMode::Devex) return true;

  const double exact_weight = leaving.row_ep.squaredNorm();
  const double stored_weight =
      state.edge_weight[static_cast<std::size_t>(leaving.row)];
  const double error = std::abs(exact_weight - stored_weight);
  const double error_limit =
      2048.0 * std::numeric_limits<double>::epsilon() *
      std::max(1.0, exact_weight);
  if (error > error_limit) {
    // DSE weights rank eligible leaving rows; they are not a feasibility
    // certificate. The selected row has now been recomputed by exact BTRAN,
    // so replace its advisory cached value before PRICE. Pivot validity is
    // checked independently by row/column agreement and reconstruction.
    state.edge_weight[static_cast<std::size_t>(leaving.row)] = exact_weight;
  }
  return true;
}

bool choose_entering_bfrt(const State& state, const Leaving& leaving,
                          const Eigen::VectorXd& pivot_row,
                          PivotTransaction& transaction,
                          std::string& failure) {
  transaction = {};
  transaction.bfrt_rhs = Eigen::VectorXd::Zero(state.m);
  if (pivot_row.size() != state.n || !pivot_row.allFinite()) {
    failure = "PRICE produced an invalid pivotal row";
    return false;
  }
  std::vector<Candidate> candidates;
  candidates.reserve(static_cast<std::size_t>(state.n / 8 + 8));
  const double stable_pivot_tolerance =
      state.updates_since_rebuild < 10
          ? 1e-9
          : (state.updates_since_rebuild < 20 ? 3e-8 : 1e-6);
  for (int j = 0; j < state.n; ++j) {
    if (state.basic[static_cast<std::size_t>(j)]) continue;
    const int direction = sign(state.move[static_cast<std::size_t>(j)]);
    if (direction == 0) continue;
    const double pivot = pivot_row[j];
    const double signed_alpha = leaving.side * direction * pivot;
    const double error = dot_error_bound(state, leaving.row_ep, j);
    if (!(signed_alpha > error)) {
      if (signed_alpha + error > 0.0) {
        transaction.stability_blocked = true;
        ++transaction.unstable_pivot_rejections;
      }
      continue;
    }
    if (!(signed_alpha > stable_pivot_tolerance)) {
      transaction.stability_blocked = true;
      ++transaction.unstable_pivot_rejections;
      continue;
    }
    const double margin =
        std::max(0.0, -direction * state.reduced_costs[j]);
    const long double breakpoint =
        static_cast<long double>(margin) /
        static_cast<long double>(signed_alpha);
    if (!std::isfinite(breakpoint)) {
      failure = "BFRT breakpoint is not finite";
      return false;
    }
    const double range = state.bounds.upper[j] - state.bounds.lower[j];
    int taboo_expiry = -1;
    const int leaving_col =
        state.basis[static_cast<std::size_t>(leaving.row)];
    const bool taboo =
        is_taboo_change(state, leaving_col, j, &taboo_expiry);
    candidates.push_back({j, pivot, signed_alpha, margin, breakpoint, range,
                          taboo, taboo_expiry});
  }
  if (candidates.empty()) return true;

  std::sort(candidates.begin(), candidates.end(),
            [](const Candidate& lhs, const Candidate& rhs) {
              if (lhs.breakpoint != rhs.breakpoint)
                return lhs.breakpoint < rhs.breakpoint;
              return lhs.col < rhs.col;
            });

  std::size_t selected_begin = 0;
  std::size_t selected_end = 0;
  long double covered = 0.0;
  for (std::size_t begin = 0; begin < candidates.size();) {
    std::size_t end = begin + 1;
    while (end < candidates.size() &&
           candidates[end].breakpoint == candidates[begin].breakpoint) {
      ++end;
    }
    long double group_change = 0.0;
    bool group_has_unbounded_range = false;
    for (std::size_t i = begin; i < end; ++i) {
      if (std::isfinite(candidates[i].range)) {
        group_change += static_cast<long double>(candidates[i].alpha) *
                        static_cast<long double>(candidates[i].range);
      } else {
        group_has_unbounded_range = true;
      }
    }
    if (group_has_unbounded_range ||
        covered + group_change >=
            static_cast<long double>(leaving.violation)) {
      selected_begin = begin;
      selected_end = end;
      break;
    }
    covered += group_change;
    begin = end;
  }

  // Finite bound ranges cannot span the leaving violation. Leave the
  // transaction empty so the caller can require a checked row certificate.
  if (selected_end <= selected_begin) return true;

  std::vector<char> flipped_col(static_cast<std::size_t>(state.n), 0);
  for (std::size_t i = 0; i < selected_begin; ++i) {
    if (std::isfinite(candidates[i].range)) {
      flipped_col[static_cast<std::size_t>(candidates[i].col)] = 1;
    }
  }

  // Harris pass 1 uses the same numerically certified pivot domain as pass 2.
  // A raw tableau coefficient whose sign is inside its dot-product error bound
  // is not an eligible pivot and cannot define a trustworthy ratio bound.
  // Earlier finite-range candidates have already been flipped, so only the
  // remaining certified suffix constrains the admissible dual step. Use the
  // current signed reduced cost directly: replacing it by max(0, -d*r) would
  // discard already-consumed tolerance when d*r>0.
  const long double dual_tolerance = state.options->optimality_tol;
  long double harris_step_upper =
      std::numeric_limits<long double>::infinity();
  for (std::size_t i = selected_begin; i < candidates.size(); ++i) {
    const Candidate& candidate = candidates[i];
    const int direction =
        sign(state.move[static_cast<std::size_t>(candidate.col)]);
    const long double alpha = static_cast<long double>(candidate.alpha);
    const long double signed_reduced_cost = static_cast<long double>(direction) *
                                            state.reduced_costs[candidate.col];
    harris_step_upper = std::min(
        harris_step_upper,
        (dual_tolerance - signed_reduced_cost) / alpha);
  }

  // Harris pass 2: among every candidate feasible at the relaxed first-pass
  // step, take the largest signed pivot. This separates stability from the
  // exact minimum-ratio ordering without relaxing the terminal dual audit.
  const Candidate* selected = nullptr;
  const Candidate* released = nullptr;
  for (std::size_t i = selected_begin; i < candidates.size(); ++i) {
    const Candidate& candidate = candidates[i];
    if (candidate.breakpoint > harris_step_upper) continue;
    ++transaction.harris_second_pass_candidates;
    if (candidate.taboo) {
      ++transaction.taboo_rejections;
      if (released == nullptr ||
          candidate.taboo_expiry < released->taboo_expiry ||
          (candidate.taboo_expiry == released->taboo_expiry &&
           (candidate.alpha > released->alpha ||
            (candidate.alpha == released->alpha &&
             candidate.col < released->col)))) {
        released = &candidate;
      }
      continue;
    }
    if (selected == nullptr || candidate.alpha > selected->alpha ||
        (candidate.alpha == selected->alpha &&
         candidate.col < selected->col)) {
      selected = &candidate;
    }
  }
  if (selected == nullptr && released != nullptr) {
    transaction.all_harris_candidates_taboo = true;
    return true;
  }
  if (selected == nullptr) {
    failure = "Harris BFRT second pass has no entering column";
    return false;
  }
  transaction.entering.col = selected->col;
  transaction.entering.pivot = selected->pivot;
  transaction.entering.alpha = selected->alpha;
  transaction.entering.theta = static_cast<double>(selected->breakpoint);
  const long double theta = selected->breakpoint;

  long double step_lower = -std::numeric_limits<long double>::infinity();
  long double step_upper = harris_step_upper;
  int step_upper_col = -1;
  long double step_upper_alpha = 0.0L;
  long double step_upper_signed_reduced_cost = 0.0L;
  for (std::size_t i = 0; i < candidates.size(); ++i) {
    const Candidate& candidate = candidates[i];
    const bool flip = i < selected_begin && std::isfinite(candidate.range);
    if (flip) {
      transaction.flips.push_back(
          {candidate.col,
           state.move[static_cast<std::size_t>(candidate.col)],
           candidate.range});
      const int direction =
          sign(state.move[static_cast<std::size_t>(candidate.col)]);
      const double delta = direction * candidate.range;
      for (Eigen::SparseMatrix<double>::InnerIterator it(state.sf->A,
                                                          candidate.col);
           it; ++it) {
        transaction.bfrt_rhs[it.row()] += it.value() * delta;
      }
    }
  }

  // A coefficient whose sign is inside its dot-product error bound cannot be
  // used as a pivot, but a large accepted dual step can still carry its
  // reduced cost across zero. Boxed columns join the BFRT bound transaction;
  // one-sided columns receive a deterministic working-cost shift that the
  // mandatory original-cost cleanup later removes.
  std::vector<char> shifted_col(static_cast<std::size_t>(state.n), 0);
  for (int col = 0; col < state.n; ++col) {
    if (state.basic[static_cast<std::size_t>(col)] || col == selected->col ||
        flipped_col[static_cast<std::size_t>(col)]) {
      continue;
    }
    const int direction = sign(state.move[static_cast<std::size_t>(col)]);
    if (direction == 0) continue;
    const long double alpha =
        static_cast<long double>(leaving.side) * direction * pivot_row[col];
    const long double signed_reduced_cost =
        static_cast<long double>(direction) * state.reduced_costs[col];
    const long double updated_signed_reduced_cost =
        signed_reduced_cost + alpha * theta;
    if (!(updated_signed_reduced_cost > dual_tolerance)) continue;

    const double range = state.bounds.upper[col] - state.bounds.lower[col];
    if (std::isfinite(range) && range > 0.0) {
      flipped_col[static_cast<std::size_t>(col)] = 1;
      transaction.flips.push_back(
          {col, state.move[static_cast<std::size_t>(col)], range});
      const double delta = direction * range;
      for (Eigen::SparseMatrix<double>::InnerIterator it(state.sf->A, col); it;
           ++it) {
        transaction.bfrt_rhs[it.row()] += it.value() * delta;
      }
      continue;
    }

    const double updated_reduced_cost =
        state.reduced_costs[col] +
        leaving.side * static_cast<double>(theta) * pivot_row[col];
    if (!std::isfinite(updated_reduced_cost)) {
      failure = "Harris BFRT requires a non-finite working-cost shift";
      return false;
    }
    transaction.cost_shifts.push_back({col, -updated_reduced_cost});
    shifted_col[static_cast<std::size_t>(col)] = 1;
  }

  for (int col = 0; col < state.n; ++col) {
    if (state.basic[static_cast<std::size_t>(col)] ||
        shifted_col[static_cast<std::size_t>(col)]) {
      continue;
    }
    const int direction = sign(state.move[static_cast<std::size_t>(col)]);
    if (direction == 0) continue;
    const long double alpha =
        static_cast<long double>(leaving.side) * direction * pivot_row[col];
    if (!(alpha > 0.0L)) continue;
    const long double signed_reduced_cost =
        static_cast<long double>(direction) * state.reduced_costs[col];
    if (flipped_col[static_cast<std::size_t>(col)]) {
      step_lower =
          std::max(step_lower,
                   (-dual_tolerance - signed_reduced_cost) / alpha);
    } else {
      const long double column_step_upper =
          (dual_tolerance - signed_reduced_cost) / alpha;
      if (column_step_upper < step_upper) {
        step_upper = column_step_upper;
        step_upper_col = col;
        step_upper_alpha = alpha;
        step_upper_signed_reduced_cost = signed_reduced_cost;
      }
    }
  }
  if (theta < step_lower || theta > step_upper) {
    std::ostringstream message;
    message << std::setprecision(18)
            << "exact-breakpoint BFRT violates its dual-feasibility interval"
            << " (theta=" << theta << ", lower=" << step_lower
            << ", upper=" << step_upper
            << ", entering_col=" << selected->col
            << ", entering_alpha=" << selected->alpha
            << ", limiting_col=" << step_upper_col
            << ", limiting_alpha=" << step_upper_alpha
            << ", limiting_signed_rc=" << step_upper_signed_reduced_cost;
    if (step_upper_col >= 0) {
      message << ", limiting_dot_error="
              << dot_error_bound(state, leaving.row_ep, step_upper_col)
              << ", limiting_lower=" << state.bounds.lower[step_upper_col]
              << ", limiting_upper=" << state.bounds.upper[step_upper_col];
    }
    message << ')';
    failure = message.str();
    return false;
  }
  transaction.dual_step_lower = static_cast<double>(step_lower);
  transaction.dual_step_upper = static_cast<double>(step_upper);
  long double rhs_projection = 0.0L;
  for (int row = 0; row < state.m; ++row) {
    rhs_projection +=
        static_cast<long double>(leaving.row_ep[row]) *
        static_cast<long double>(transaction.bfrt_rhs[row]);
  }
  const long double materialized_coverage =
      static_cast<long double>(leaving.side) * rhs_projection;
  if (!(materialized_coverage >= 0.0L) ||
      !(materialized_coverage <=
        static_cast<long double>(leaving.violation)) ||
      !std::isfinite(materialized_coverage)) {
    std::ostringstream message;
    message << std::setprecision(18)
            << "materialized BFRT flips do not leave a valid entering step"
            << " (coverage=" << materialized_coverage
            << ", violation=" << leaving.violation << ')';
    failure = message.str();
    return false;
  }
  transaction.covered_violation =
      static_cast<double>(materialized_coverage);
  return true;
}

bool compute_dse_weights(const State& state, const Leaving& leaving,
                         const Eigen::VectorXd& pivot_row,
                         const Eigen::VectorXd& direction, double pivot,
                         std::vector<double>& updated,
                         bool& restart_devex,
                         std::string& failure) {
  restart_devex = false;
  if (direction.size() != state.m || pivot == 0.0) {
    failure = "edge-weight update received an invalid pivotal column";
    return false;
  }
  if (state.edge_weight_mode == EdgeWeightMode::Devex) {
    if (pivot_row.size() != state.n || !pivot_row.allFinite() ||
        state.devex_reference.size() != static_cast<std::size_t>(state.n)) {
      failure = "Devex update received an invalid reference row";
      return false;
    }
    long double exact_weight = 0.0L;
    for (int col = 0; col < state.n; ++col) {
      if (!state.devex_reference[static_cast<std::size_t>(col)]) continue;
      const long double value = static_cast<long double>(pivot_row[col]);
      exact_weight += value * value;
    }
    exact_weight = std::max(1.0L, exact_weight);
    if (!std::isfinite(exact_weight)) {
      failure = "Devex pivotal reference weight is non-finite";
      return false;
    }
    const double computed = static_cast<double>(exact_weight);
    const double stored =
        state.edge_weight[static_cast<std::size_t>(leaving.row)];
    if (!(stored > 0.0) || !std::isfinite(stored)) {
      failure = "Devex stored pivotal weight is invalid";
      return false;
    }
    const double ratio = std::max(stored / computed, computed / stored);
    restart_devex = ratio > 9.0;

    const double new_pivotal =
        std::max(1.0, computed / (pivot * pivot));
    if (!std::isfinite(new_pivotal)) {
      failure = "Devex new pivotal weight is non-finite";
      return false;
    }
    updated = state.edge_weight;
    for (int row = 0; row < state.m; ++row) {
      const double candidate =
          new_pivotal * direction[row] * direction[row];
      if (!std::isfinite(candidate)) {
        failure = "Devex recurrence produced a non-finite weight";
        return false;
      }
      updated[static_cast<std::size_t>(row)] =
          std::max(updated[static_cast<std::size_t>(row)], candidate);
    }
    updated[static_cast<std::size_t>(leaving.row)] = new_pivotal;
    return true;
  }

  const Eigen::VectorXd rho = state.factor->ftran(leaving.row_ep);
  if (rho.size() != state.m || !rho.allFinite()) {
    failure = "DSE FTRAN failed backward-error validation";
    return false;
  }
  const double old_pivotal_weight =
      state.edge_weight[static_cast<std::size_t>(leaving.row)];
  updated = state.edge_weight;
  const double inv_pivot = 1.0 / pivot;
  for (int row = 0; row < state.m; ++row) {
    if (row == leaving.row) continue;
    const double ratio = direction[row] * inv_pivot;
    const long double ratio_ld = static_cast<long double>(ratio);
    const long double value_ld =
        static_cast<long double>(updated[static_cast<std::size_t>(row)]) -
        2.0L * ratio_ld * static_cast<long double>(rho[row]) +
        ratio_ld * ratio_ld *
            static_cast<long double>(old_pivotal_weight);
    const double value = static_cast<double>(value_ld);
    const double roundoff =
        1024.0 * std::numeric_limits<double>::epsilon() *
        std::max({1.0, std::abs(updated[static_cast<std::size_t>(row)]),
                  std::abs(2.0 * ratio * rho[row]),
                  std::abs(ratio * ratio * old_pivotal_weight)});
    if (!std::isfinite(value)) {
      failure = "Goldfarb-Reid update produced a non-finite DSE weight";
      return false;
    }
    if (value <= roundoff) {
      Eigen::VectorXd unit = Eigen::VectorXd::Zero(state.m);
      unit[row] = 1.0;
      const Eigen::VectorXd old_inverse_row = state.factor->btran(unit);
      if (old_inverse_row.size() != state.m ||
          !old_inverse_row.allFinite()) {
        failure = "DSE cancellation-row BTRAN failed";
        return false;
      }
      long double exact_weight = 0.0L;
      for (int col = 0; col < state.m; ++col) {
        const long double component =
            static_cast<long double>(old_inverse_row[col]) -
            ratio_ld * static_cast<long double>(leaving.row_ep[col]);
        exact_weight += component * component;
      }
      if (!(exact_weight > 0.0L) || !std::isfinite(exact_weight)) {
        failure = "DSE exact cancellation-row reconstruction failed";
        return false;
      }
      updated[static_cast<std::size_t>(row)] =
          static_cast<double>(exact_weight);
      continue;
    }
    updated[static_cast<std::size_t>(row)] = value;
  }
  updated[static_cast<std::size_t>(leaving.row)] =
      old_pivotal_weight * inv_pivot * inv_pivot;
  if (!(updated[static_cast<std::size_t>(leaving.row)] > 0.0) ||
      !std::isfinite(updated[static_cast<std::size_t>(leaving.row)])) {
    failure = "pivotal DSE weight is invalid";
    return false;
  }
  return true;
}

}  // namespace mipsolvers::engine::native_dual::detail
