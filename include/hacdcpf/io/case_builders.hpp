#pragma once
#include <string>
#include <vector>

#include "hacdcpf/model/system.hpp"

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

}  // namespace hacdcpf::io

