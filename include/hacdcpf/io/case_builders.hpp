#pragma once
#include <string>
#include <vector>

#include "hacdcpf/model/hybrid_power_system.hpp"

namespace hacdcpf::io {

// ── Pre-built test cases ──────────────────────────────────────────────────────

HybridPowerSystem build_ieee14_acdc();
HybridPowerSystem build_ieee24_3area_acdc();
HybridPowerSystem build_ieee24_3area_acdc_expanded();
HybridPowerSystem build_ieee118_acdc();
HybridPowerSystem build_ac_only_version(const HybridPowerSystem& sys);
HybridPowerSystem build_case33bw_acdc();
HybridPowerSystem build_case33mg_acdc();
HybridPowerSystem build_case69_acdc();
HybridPowerSystem build_case300_acdc();
HybridPowerSystem build_demo_multizone_acdc();
HybridPowerSystem build_case2000_acdc();
HybridPowerSystem build_dist33_microgrid_der();
HybridPowerSystem build_comprehensive_hybrid_acdc();
HybridPowerSystem build_market_3bus_toy();
HybridPowerSystem build_market_5bus_acdc_toy();

/// ETAP/OpenDSS-style example specified entirely in *actual* engineering values
/// (cable ohm/km + length, DC link ohm/km, per-bus base kV).  No per-unit
/// impedance is supplied; convert_actual_to_per_unit fills it in during
/// projection.  Useful for exercising the actual-value path end to end.
HybridPowerSystem build_actual_value_demo_acdc();

}  // namespace hacdcpf::io

