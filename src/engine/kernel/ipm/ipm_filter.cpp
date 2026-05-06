#include "hacdcpf/engine/kernel/ipm/ipm_filter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace hacdcpf::engine {

bool Filter::is_acceptable(double theta_trial, double phi_trial,
                           double gamma_theta, double gamma_phi) const {
  if (!std::isfinite(theta_trial) || !std::isfinite(phi_trial)) {
    return false;
  }
  for (const auto& e : entries_) {
    const double theta_bound = (1.0 - gamma_theta) * e.theta;
    const double phi_bound = e.phi - gamma_phi * e.theta;
    if (theta_trial >= theta_bound && phi_trial >= phi_bound) {
      return false;
    }
  }
  return true;
}

void Filter::add_entry(double theta, double phi,
                       double gamma_theta, double gamma_phi) {
  if (!std::isfinite(theta) || !std::isfinite(phi)) return;
  FilterEntry incoming{theta, phi};
  // Prune entries that the incoming one dominates under the Wächter–Biegler
  // margins: we remove any existing entry (θ_k, φ_k) for which
  //   θ_k > (1 − γ_θ) θ_incoming  AND  φ_k > φ_incoming − γ_φ θ_incoming.
  // That forbidden region is a strict superset of what (θ_k, φ_k) ever
  // forbade, so removing them is safe.
  const double theta_cut = (1.0 - gamma_theta) * incoming.theta;
  const double phi_cut = incoming.phi - gamma_phi * incoming.theta;
  entries_.erase(
      std::remove_if(entries_.begin(), entries_.end(),
                     [&](const FilterEntry& e) {
                       return e.theta > theta_cut && e.phi > phi_cut;
                     }),
      entries_.end());
  entries_.push_back(incoming);
}

void Filter::reset_with_theta_upper_bound(double theta_max) {
  entries_.clear();
  if (std::isfinite(theta_max) && theta_max > 0.0) {
    // Hard upper bound forbidding any iterate with θ ≥ theta_max.
    // Represented as an entry with φ = −∞ so the φ coordinate is effectively
    // non-restrictive — only θ matters.
    entries_.push_back({theta_max, -std::numeric_limits<double>::infinity()});
  }
}

void Filter::clear() { entries_.clear(); }

}  // namespace hacdcpf::engine
