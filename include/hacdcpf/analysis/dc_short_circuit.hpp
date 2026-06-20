#pragma once

/// analysis/dc_short_circuit.hpp
/// =============================
/// Steady-state DC fault-level (bolted-fault current) estimate for a DC bus.

#include <string>

#include "hacdcpf/model/hybrid_power_system.hpp"

namespace hacdcpf::analysis {

/// Options for the resistive DC fault-level estimate.
struct DCFaultOptions {
  /// Bolted fault by default; a positive value adds fault-path resistance (pu).
  double fault_resistance_pu{0.0};
  /// DC voltage assumed at the voltage-reference (DC_V) source buses, in pu.
  /// 0 => use each source bus's own vm_pu.
  double source_voltage_pu{0.0};
};

/// Result of a DC bus bolted-fault calculation.
struct DCFaultResult {
  bool solved{false};
  int fault_bus_id{0};
  double v_prefault_pu{0.0};  ///< Pre-fault voltage at the faulted bus [pu]
  double r_thevenin_pu{0.0};  ///< Thevenin resistance to the DC source(s) [pu]
  double i_fault_pu{0.0};     ///< Steady-state fault current [pu]
  double i_fault_ka{0.0};     ///< Steady-state fault current [kA]
  std::string message;
};

/// Estimate the steady-state bolted-fault current at a DC bus.
///
/// Resistive, stiff-source model: the DC network is reduced to its branch
/// resistances and voltage-reference (DC_V) buses are treated as ideal voltage
/// sources.  The fault current at a non-source bus k is
/// `I_f = V_k / (R_kk + Rf)`, where `R_kk` is the Thevenin resistance from bus k
/// to the source(s).  This is the line-resistance-limited fault level; it does
/// NOT model converter current limiting, capacitor-discharge transients, or
/// source internal impedance (the data model carries none of these), so it is a
/// conservative upper bound useful for DC breaker/cable screening.
DCFaultResult dc_bus_fault_level(const HybridPowerSystem& sys, int dc_bus_id,
                                 const DCFaultOptions& opt = {});

}  // namespace hacdcpf::analysis
