#pragma once

#include "hacdcpf/model/system.hpp"

namespace hacdcpf::io {

HybridPowerSystem build_ieee14_acdc();
HybridPowerSystem build_ieee24_3area_acdc();
HybridPowerSystem build_ieee24_3area_acdc_expanded();
HybridPowerSystem build_ieee118_acdc();
HybridPowerSystem build_ac_only_version(const HybridPowerSystem& sys);
HybridPowerSystem build_case33bw_acdc();
HybridPowerSystem build_case33mg_acdc();
HybridPowerSystem build_case69_acdc();
HybridPowerSystem build_case300_acdc();
HybridPowerSystem build_case2000_acdc();
HybridPowerSystem build_demo_multizone_acdc();
HybridPowerSystem build_dist33_microgrid_der();
HybridPowerSystem build_comprehensive_hybrid_acdc();

/// 3-bus toy case for market simulation verification.
/// Cheap gen on bus 1, mid-cost gen on bus 2, peaker on bus 3.
/// Line 1→3 has a tight 30 MW rating to force congestion in LMP.
HybridPowerSystem build_market_3bus_toy();

/// 5-bus AC/DC toy case for hybrid market simulation debugging.
/// 3 AC buses (area 1) + 2 AC buses (area 2) connected by 1 DC link via VSC.
/// Demonstrates DC line congestion, inter-area price separation, and
/// the effect of VSC converter losses on LMP.
HybridPowerSystem build_market_5bus_acdc_toy();

/// 60-bus Southern China five-province AC/DC case for large-scale market
/// simulation.  Provinces: Guangdong (GD), Guangxi (GX), Yunnan (YN),
/// Guizhou (GZ), Hainan (HN).  15 Balancing Areas, 75+ generators
/// (coal/gas/hydro), 3 HVDC links (GD-YN 5 GW, GD-GZ 3.2 GW, GD-HN 1.2 GW),
/// plus 16 AC tielines.  Geographic coordinates for GIS rendering.
HybridPowerSystem build_five_province_acdc();

}  // namespace hacdcpf::io
