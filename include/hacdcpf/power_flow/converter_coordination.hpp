#pragma once

#include <string>
#include <vector>

#include "hacdcpf/model/hybrid_power_system.hpp"

namespace hacdcpf::powerflow {

enum class CoordinationSeverity {
  Info,
  Warning,
  Error,
  Fatal,
};

struct CoordinationIssue {
  CoordinationSeverity severity{CoordinationSeverity::Info};
  std::string rule_id;
  std::string component_type;
  int component_index{-1};
  int island_index{-1};
  std::string message;
};

struct DCVoltageControlSource {
  std::string component_type;
  int component_index{-1};
  int bus{-1};
  double v_set_pu{0.0};
  bool has_v_set{false};
  bool droop{false};
};

struct DCIslandCoordinationSummary {
  int island_index{-1};
  std::vector<int> dc_buses;
  std::vector<int> declared_v_buses;
  std::vector<DCVoltageControlSource> voltage_sources;
  int hard_vdc_sources{0};
  int droop_sources{0};
  // In-service VSC converters in PQ mode that the unified solver can
  // auto-promote to VDC_Q to form the DC-island voltage reference (mirrors
  // plan_dc_island_references in the Newton solver).  Their presence means the
  // island is solvable even without an explicitly declared voltage source.
  int promotable_vsc_sources{0};
  // True when the island contains at least one DC_V-typed bus.  The solver pins
  // such a bus as a fixed-voltage slack, so the island has a (declared,
  // possibly non-physical) reference even with no voltage-forming device.
  bool has_declared_v_bus{false};
  int fixed_power_devices{0};
  double fixed_power_mw{0.0};
  double flexible_up_mw{0.0};
  double flexible_down_mw{0.0};
};

struct ConverterCoordinationReport {
  bool enabled{false};
  bool feasible{true};
  std::vector<CoordinationIssue> issues;
  std::vector<DCIslandCoordinationSummary> dc_islands;

  [[nodiscard]] bool has_blocking_issue() const noexcept;
  [[nodiscard]] bool has_fatal() const noexcept;
  [[nodiscard]] int blocking_count() const noexcept;
  [[nodiscard]] int fatal_count() const noexcept;
  [[nodiscard]] int error_count() const noexcept;
  [[nodiscard]] int warning_count() const noexcept;
};

const char* coordination_severity_str(CoordinationSeverity severity) noexcept;

ConverterCoordinationReport evaluate_converter_coordination(
    const HybridPowerSystem& sys,
    bool enabled = true);

}  // namespace hacdcpf::powerflow
