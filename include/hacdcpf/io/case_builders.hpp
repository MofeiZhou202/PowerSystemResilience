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
HybridPowerSystem build_multiscale_comprehensive_acdc();
HybridPowerSystem build_hybrid_acdc_microgrid_island();
HybridPowerSystem build_networked_microgrids_islanding();
HybridPowerSystem build_market_3bus_toy();
HybridPowerSystem build_market_5bus_acdc_toy();

/// LVNT-inspired urban benchmark with a 230/13.8 kV primary substation,
/// four 13.8 kV feeders, 24 distribution transformers, detailed 0.48 kV
/// secondary streets, unbalanced wye/delta services, DER, storage, and a
/// converter-fed DC charging corridor. Both balanced and phase-domain models
/// are authored from the same deterministic component data.
HybridPowerSystem build_urban_lvn_primary_secondary();

/// ETAP/OpenDSS-style example specified entirely in *actual* engineering values
/// (cable ohm/km + length, DC link ohm/km, per-bus base kV).  No per-unit
/// impedance is supplied; convert_actual_to_per_unit fills it in during
/// projection.  Useful for exercising the actual-value path end to end.
HybridPowerSystem build_actual_value_demo_acdc();

}  // namespace hacdcpf::io
