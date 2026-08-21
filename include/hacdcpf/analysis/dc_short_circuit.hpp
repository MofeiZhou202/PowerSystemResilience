#pragma once

/// analysis/dc_short_circuit.hpp
/// =============================
/// Steady-state DC fault-level (bolted-fault current) estimate for a DC bus.

#include <functional>
#include <string>
#include <vector>

#include "hacdcpf/model/hybrid_power_system.hpp"

namespace hacdcpf::analysis {

/// Options for the resistive DC fault-level estimate.
struct DCFaultOptions {
  /// Bolted fault by default; a positive value adds fault-path resistance (pu).
  double fault_resistance_pu{0.0};
  /// DC voltage assumed at the voltage-reference (DC_V) source buses, in pu.
  /// 0 => use each source bus's own vm_pu.
  double source_voltage_pu{0.0};
  /// Include DCCircuitBreaker state in the DC conductance graph.
  bool consider_dc_breakers{true};
  /// Let a DCCB with matching branch id or unique same terminals control the
  /// corresponding DCBranch. Open breakers block the branch; closed breakers
  /// add their contact resistance in series.
  bool dc_breakers_control_branches{true};
  /// Add closed DCCBs that are not assigned to a branch as standalone DC edges.
  bool add_unassigned_closed_breaker_edges{true};
  /// Minimum resistance used for ideal closed DCCBs to avoid singular graphs.
  double min_closed_breaker_resistance_pu{1e-6};
  /// Cooperative cancellation checked between selected sparse solves.
  std::function<bool()> cancellation_requested;
};

/// Fault-duty estimate for a DC circuit breaker participating in the DC
/// short-circuit topology.
struct DCBreakerDutyResult {
  int breaker_index{0};
  std::string name;
  int from_bus{0};
  int to_bus{0};
  bool in_service{true};
  bool closed{true};
  bool controls_branch{false};
  int controlled_branch_index{0};
  double r_pu{0.0};
  double i_duty_ka{0.0};
  double i_breaking_ka{0.0};
  bool breaking_rating_ok{true};
  std::string model;
};

/// Result of a DC bus bolted-fault calculation.
struct DCFaultResult {
  bool solved{false};
  int fault_bus_id{0};
  double v_prefault_pu{0.0};  ///< Pre-fault voltage at the faulted bus [pu]
  double r_thevenin_pu{0.0};  ///< Thevenin resistance to the DC source(s) [pu]
  double i_fault_pu{0.0};     ///< Steady-state fault current [pu]
  double i_fault_ka{0.0};     ///< Steady-state fault current [kA]
  int dc_branch_edges_used{0};
  int dccb_edges_used{0};
  int dccb_open_count{0};
  int dccb_blocked_branch_count{0};
  std::vector<DCBreakerDutyResult> breaker_duties;
  std::string message;
  std::string status{"not_run"};
  std::string model_scope{"dc-resistive-quasi-static-v2"};
  std::vector<std::string> model_limitations;
  double max_linear_residual{0.0};
};

/// Estimate the steady-state bolted-fault current at a DC bus.
///
/// Resistive, stiff-source model: the DC network is reduced to its branch
/// resistances and voltage-reference (DC_V) buses are treated as ideal voltage
/// sources.  Closed DCCircuitBreaker elements can either control matching
/// DCBranch entries or act as standalone conductive edges, depending on the
/// options above. The fault current at a non-source bus k is
/// `I_f = V_k / (R_kk + Rf)`, where `R_kk` is the Thevenin resistance from bus k
/// to the source(s).  This is the line-resistance-limited fault level; it does
/// NOT model converter current limiting, capacitor-discharge transients, or
/// source internal impedance, so it is a conservative upper bound useful for DC
/// breaker/cable screening.
DCFaultResult dc_bus_fault_level(const HybridPowerSystem& sys, int dc_bus_id,
                                 const DCFaultOptions& opt = {});

/// Batch variant. Topology construction and sparse component factorizations
/// are reused for every requested fault bus.
std::vector<DCFaultResult> dc_bus_fault_levels(
    const HybridPowerSystem& sys, const std::vector<int>& dc_bus_ids,
    const DCFaultOptions& opt = {});

}  // namespace hacdcpf::analysis
