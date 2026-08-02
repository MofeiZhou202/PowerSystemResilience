#pragma once

#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "hacdcpf/power_flow/power_flow_options.hpp"
#include "hacdcpf/assembly/solver_data.hpp"

namespace hacdcpf::powerflow {

/// Infinity norm of a vector (returns 0 for empty vectors).
inline double inf_norm(const Eigen::VectorXd& v) {
  return (v.size() == 0) ? 0.0 : v.cwiseAbs().maxCoeff();
}

/// Return internal index of first AC slack bus, or 0 if none found.
inline int first_slack_or_default(const SolverData& data) {
  for (int i = 0; i < static_cast<int>(data.ac_buses.size()); ++i) {
    if (data.ac_buses[static_cast<size_t>(i)].bus_type == BusType::SLACK) {
      return i;
    }
  }
  return data.ac_buses.empty() ? -1 : 0;
}

/// Return internal index of the DC voltage reference bus.
/// Prefers a DC_V bus; otherwise the first non-isolated bus; otherwise 0.
/// DC_ISOLATED buses are de-energized and are never chosen as an anchor for an
/// energized network unless no other bus exists.
inline int first_dc_slack_or_default(const SolverData& data) {
  const int ndc = static_cast<int>(data.dc_buses.size());
  if (ndc == 0) return -1;
  int first_non_isolated = -1;
  for (int i = 0; i < ndc; ++i) {
    const auto& b = data.dc_buses[static_cast<size_t>(i)];
    if (b.bus_type == DCBusType::DC_V) return i;
    if (first_non_isolated < 0 && b.bus_type != DCBusType::DC_ISOLATED) {
      first_non_isolated = i;
    }
  }
  return first_non_isolated >= 0 ? first_non_isolated : 0;
}

/// Load DC voltages from InitialState into @p vdc (size ndc).
/// Uses bus defaults when init is nullptr or size mismatches.
/// Throws std::invalid_argument if any entry is non-finite.
inline void load_dc_initial_state(const InitialState* init,
                                  const std::vector<DCBus>& dc_buses,
                                  Eigen::VectorXd& vdc) {
  const int ndc = static_cast<int>(dc_buses.size());
  if (init != nullptr && static_cast<int>(init->vdc.size()) == ndc) {
    for (int i = 0; i < ndc; ++i) {
      if (!std::isfinite(init->vdc[static_cast<size_t>(i)])) {
        throw std::invalid_argument(
            "InitialState::vdc[" + std::to_string(i) + "] is not finite (NaN or Inf)");
      }
      vdc[i] = init->vdc[static_cast<size_t>(i)];
    }
  } else {
    for (int i = 0; i < ndc; ++i) {
      vdc[i] = dc_buses[static_cast<size_t>(i)].vm_pu;
    }
  }
}

/// AC bus classification result (excludes every fixed SLACK reference).
struct AcBusSets {
  int slack{0};                 ///< Index of the primary AC slack bus.
  std::vector<int> pv;         ///< PV buses.
  std::vector<int> pq;         ///< PQ buses.
  std::vector<int> non_slack;  ///< pv ∪ pq (all buses with Newton equations).
};

/// Classify AC buses into PV / PQ / non-slack groups.
/// Every SLACK bus is an angle-and-voltage reference for its connected area and
/// therefore contributes neither a P nor Q equation.
inline AcBusSets classify_ac_buses(const SolverData& data) {
  AcBusSets sets;
  const int n = static_cast<int>(data.ac_buses.size());
  sets.slack = first_slack_or_default(data);
  sets.pv.reserve(static_cast<size_t>(n));
  sets.pq.reserve(static_cast<size_t>(n));
  sets.non_slack.reserve(static_cast<size_t>(n));
  for (int i = 0; i < n; ++i) {
    const BusType bt = data.ac_buses[static_cast<size_t>(i)].bus_type;
    if (bt == BusType::SLACK) continue;
    if (bt == BusType::PQ) {
      sets.pq.push_back(i);
    } else {
      sets.pv.push_back(i);
    }
    sets.non_slack.push_back(i);
  }
  return sets;
}

}  // namespace hacdcpf::powerflow
