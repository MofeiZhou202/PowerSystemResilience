#pragma once

#include <vector>

#include "hacdcpf/dynamics/DynamicResults.hpp"

namespace hacdcpf::dynamics {

struct DynamicSystem;  // forward declaration (defined in DynamicSystem.hpp)

// Aggregate frequency observability for one instant of a transient run
// (design doc §7 "Frequency in a phasor simulator", §6.2 island detection).
//
// Frequency is not a single physical unknown in a phasor simulator; this report
// derives the observable quantities from the per-device speed states:
//   * a per-island center-of-inertia (COI) frequency, inertia-weighted over the
//     rotor/virtual speeds of the generation in each connected AC component;
//   * a single system COI frequency (inertia-weighted across all islands);
//   * detection of energized islands that have lost their frequency anchor
//     (sources present but no machine / grid-forming / slack device), which is
//     precisely the case that makes the network algebraic block singular.
struct DynamicFrequencyReport {
  double nominal_frequency_hz{50.0};
  double system_coi_frequency_hz{50.0};
  std::vector<DynamicIslandFrequency> islands;  // energized islands only
  bool has_anchorless_source_island{false};
};

// Computes the frequency report for the current state (`sys.x`) and network
// voltages (`sys.y`). The AC network is partitioned into connected components
// over the in-service branches; each generation-capable device is bucketed into
// the component of its terminal bus via DynamicDevice::frequencyParticipation().
[[nodiscard]] DynamicFrequencyReport computeFrequencyReport(const DynamicSystem& sys);

}  // namespace hacdcpf::dynamics
