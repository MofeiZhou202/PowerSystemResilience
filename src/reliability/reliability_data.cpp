// reliability_data.cpp
//
// Default reliability data sets and load profiles for standard test cases.
// These are "case builder" companions: they populate component-level failure
// parameters on pre-built HybridPowerSystem objects so that reliability
// algorithms can be called immediately without manual data entry.
//
// Contents:
//   - IEEE RTS-24 hourly/daily/weekly load profile tables
//   - build_ieee_rts24_load_profile()
//   - apply_ieee24_reliability_data()       — IEEE RTS-24 generators + branches
//   - apply_comprehensive_reliability_data() — full hybrid AC/DC distribution case

#include "hacdcpf/reliability/reliability_assessment.hpp"

#include <spdlog/spdlog.h>

#include <cmath>
#include <stdexcept>
#include <string>

namespace hacdcpf::analysis {

// ═══════════════════════════════════════════════════════════════════════
// IEEE RTS-24 Load Profile Tables
// ═══════════════════════════════════════════════════════════════════════
namespace {

// Weekly peak-load factors (52 weeks, normalised to 1.0 = system peak).
// Source: IEEE RTS Technical Report, Table A.1.
const double kWeeklyFactors[52] = {
    0.862, 0.900, 0.878, 0.834,   // Weeks  1- 4  (Winter)
    0.880, 0.841, 0.832, 0.806,   // Weeks  5- 8
    0.740, 0.737, 0.715, 0.727,   // Weeks  9-12  (Spring)
    0.704, 0.750, 0.721, 0.800,   // Weeks 13-16
    0.754, 0.837, 0.870, 0.880,   // Weeks 17-20  (Summer start)
    0.856, 0.811, 0.900, 0.887,   // Weeks 21-24
    0.896, 0.861, 0.755, 0.816,   // Weeks 25-28
    0.801, 0.880, 0.722, 0.776,   // Weeks 29-32
    0.800, 0.729, 0.726, 0.705,   // Weeks 33-36  (Fall)
    0.780, 0.695, 0.724, 0.723,   // Weeks 37-40
    0.743, 0.744, 0.800, 0.881,   // Weeks 41-44
    0.885, 0.909, 0.940, 0.890,   // Weeks 45-48  (Winter)
    0.942, 0.970, 1.000, 0.952    // Weeks 49-52  (peak at week 51)
};

// Daily peak-load factors (Mon=0 … Sun=6).
const double kDailyFactors[7] = {
    0.93, 1.00, 0.98, 0.96, 0.94, 0.77, 0.75
};

// Hourly shape factors [24 hours × 6 columns].
// Columns: 0=WinterWkdy, 1=WinterWknd, 2=SummerWkdy, 3=SummerWknd,
//          4=SprFallWkdy, 5=SprFallWknd
const double kHourlyFactors[24][6] = {
    {0.67, 0.78, 0.64, 0.74, 0.63, 0.75},  // 00:00
    {0.63, 0.72, 0.60, 0.70, 0.62, 0.73},
    {0.60, 0.68, 0.58, 0.66, 0.60, 0.69},
    {0.59, 0.66, 0.56, 0.65, 0.58, 0.66},
    {0.59, 0.64, 0.56, 0.64, 0.59, 0.65},
    {0.60, 0.65, 0.58, 0.62, 0.65, 0.65},
    {0.74, 0.66, 0.64, 0.62, 0.72, 0.68},
    {0.86, 0.70, 0.76, 0.66, 0.85, 0.74},
    {0.95, 0.80, 0.87, 0.81, 0.95, 0.83},
    {0.96, 0.88, 0.95, 0.86, 0.99, 0.89},
    {0.96, 0.90, 0.99, 0.91, 1.00, 0.92},
    {0.95, 0.91, 1.00, 0.93, 0.99, 0.94},
    {0.95, 0.90, 0.99, 0.93, 0.93, 0.91},
    {0.95, 0.88, 1.00, 0.92, 0.92, 0.90},
    {0.93, 0.87, 1.00, 0.91, 0.90, 0.90},
    {0.94, 0.87, 0.97, 0.91, 0.88, 0.86},
    {0.99, 0.91, 0.96, 0.92, 0.90, 0.85},
    {1.00, 1.00, 0.96, 0.94, 0.92, 0.88},  // 17:00  peak hour
    {1.00, 0.99, 0.93, 0.95, 0.96, 0.92},
    {0.96, 0.97, 0.92, 0.95, 0.98, 1.00},
    {0.91, 0.94, 0.92, 1.00, 0.96, 0.97},
    {0.83, 0.92, 0.93, 0.93, 0.90, 0.95},
    {0.73, 0.87, 0.87, 0.88, 0.80, 0.90},
    {0.63, 0.81, 0.72, 0.80, 0.70, 0.85}   // 23:00
};

enum class Season { Winter, Summer, SpringFall };

Season get_season(int week_of_year) {
  // week_of_year: 1-52
  if (week_of_year <= 8 || week_of_year >= 44) return Season::Winter;
  if (week_of_year >= 18 && week_of_year <= 30) return Season::Summer;
  return Season::SpringFall;
}

int get_hourly_col(Season season, bool is_weekend) {
  switch (season) {
    case Season::Winter:    return is_weekend ? 1 : 0;
    case Season::Summer:    return is_weekend ? 3 : 2;
    case Season::SpringFall: return is_weekend ? 5 : 4;
  }
  return 0;
}

// ═══════════════════════════════════════════════════════════════════════
// IEEE RTS-24 Generator Reliability Data
// IEEE RTS Task Force (1979), generator table; row order is the MATPOWER
// case24_ieee_rts.m order. The 33rd record count includes the Pmax=0
// synchronous condenser at row 15; there are 32 active-power generating units.
// ═══════════════════════════════════════════════════════════════════════
struct GenReliabilityData {
  int bus;
  double pmax_mw;
  double mttf_hr;
  double mttr_hr;
};

const GenReliabilityData kIeee24UnitReliability[] = {
    { 1,  20.0,  450.0,  50.0}, { 1,  20.0,  450.0,  50.0},
    { 1,  76.0, 1960.0,  40.0}, { 1,  76.0, 1960.0,  40.0},
    { 2,  20.0,  450.0,  50.0}, { 2,  20.0,  450.0,  50.0},
    { 2,  76.0, 1960.0,  40.0}, { 2,  76.0, 1960.0,  40.0},
    { 7, 100.0, 1200.0,  50.0}, { 7, 100.0, 1200.0,  50.0},
    { 7, 100.0, 1200.0,  50.0},
    {13, 197.0,  950.0,  50.0}, {13, 197.0,  950.0,  50.0},
    {13, 197.0,  950.0,  50.0},
    {14,   0.0, 10000.0,  0.1},  // synchronous condenser
    {15,  12.0, 2940.0,  60.0}, {15,  12.0, 2940.0,  60.0},
    {15,  12.0, 2940.0,  60.0}, {15,  12.0, 2940.0,  60.0},
    {15,  12.0, 2940.0,  60.0},
    {15, 155.0,  960.0,  40.0}, {16, 155.0,  960.0,  40.0},
    {18, 400.0, 1100.0, 150.0}, {21, 400.0, 1100.0, 150.0},
    {22,  50.0, 1980.0,  20.0}, {22,  50.0, 1980.0,  20.0},
    {22,  50.0, 1980.0,  20.0}, {22,  50.0, 1980.0,  20.0},
    {22,  50.0, 1980.0,  20.0}, {22,  50.0, 1980.0,  20.0},
    {23, 155.0,  960.0,  40.0}, {23, 155.0,  960.0,  40.0},
    {23, 350.0, 1150.0, 100.0},
};

// The native hybrid builder aggregates each bus's fleet into one generator.
// These rows preserve the builder's documented screening model, but are not
// used by the unit-level MATPOWER RTS-24 benchmark.
const GenReliabilityData kIeee24AggregateGenReliability[] = {
    { 1, 300.0, 2940.0, 60.0},
    { 2, 300.0, 2940.0, 60.0},
    { 7, 400.0, 1200.0, 40.0},
    {13, 800.0, 1100.0, 45.0},
    {15, 400.0,  960.0, 50.0},
    {16, 300.0, 1100.0, 45.0},
    {18, 600.0, 1100.0, 50.0},
    {21, 600.0, 1100.0, 50.0},
    {22, 500.0, 1100.0, 50.0},
    {23, 800.0, 1100.0, 50.0},
};

// ═══════════════════════════════════════════════════════════════════════
// IEEE RTS-24 Branch Reliability Data
// lambda = failures/yr, repair_hr = mean repair duration (hours).
// ═══════════════════════════════════════════════════════════════════════
struct BranchReliabilityData {
  int from_bus;
  int to_bus;
  double lambda;
  double repair_hr;
};

const BranchReliabilityData kIeee24UnitBranchReliability[] = {
    { 1,  2, 0.24, 16.0},  { 1,  3, 0.51, 10.0},  { 1,  5, 0.33, 10.0},
    { 2,  4, 0.39, 10.0},  { 2,  6, 0.48, 10.0},  { 3,  9, 0.38, 10.0},
    { 3, 24, 0.02, 768.0}, { 4,  9, 0.36, 10.0},  { 5, 10, 0.34, 10.0},
    { 6, 10, 0.33, 35.0},  { 7,  8, 0.30, 10.0},  { 8,  9, 0.44, 10.0},
    { 8, 10, 0.44, 10.0},  { 9, 11, 0.02, 768.0}, // transformer
    { 9, 12, 0.02, 768.0},                         // transformer
    {10, 11, 0.02, 768.0},                         // transformer
    {10, 12, 0.02, 768.0},                         // transformer
    {11, 13, 0.40, 11.0},  {11, 14, 0.39, 11.0},  {12, 13, 0.40, 11.0},
    {12, 23, 0.52, 11.0},  {13, 23, 0.49, 11.0},  {14, 16, 0.38, 11.0},
    {15, 16, 0.33, 11.0},  {15, 21, 0.41, 11.0},  {15, 21, 0.41, 11.0},
    {15, 24, 0.41, 11.0},
    {16, 17, 0.35, 11.0},  {16, 19, 0.34, 11.0},  {17, 18, 0.32, 11.0},
    {17, 22, 0.54, 11.0},  {18, 21, 0.35, 11.0},  {18, 21, 0.35, 11.0},
    {19, 20, 0.38, 11.0},  {19, 20, 0.38, 11.0},  {20, 23, 0.34, 11.0},
    {20, 23, 0.34, 11.0},  {21, 22, 0.45, 11.0},
};

const BranchReliabilityData kIeee24AggregateBranchReliability[] = {
    { 1,  2, 0.24, 16.0},  { 1,  3, 0.51, 10.0},  { 1,  5, 0.33, 10.0},
    { 2,  4, 0.39, 10.0},  { 2,  6, 0.48, 10.0},  { 3,  9, 0.38, 10.0},
    { 4,  9, 0.36, 10.0},  { 5, 10, 0.34, 10.0},  { 6, 10, 0.33, 35.0},
    { 7,  8, 0.30, 10.0},  { 3, 24, 0.02, 768.0}, { 8, 10, 0.44, 10.0},
    { 9, 11, 0.02, 768.0}, { 9, 12, 0.02, 768.0}, {10, 11, 0.02, 768.0},
    {10, 12, 0.02, 768.0}, {11, 13, 0.40, 11.0},  {11, 14, 0.39, 11.0},
    {12, 13, 0.40, 11.0},  {12, 23, 0.52, 11.0},  {13, 23, 0.49, 11.0},
    {14, 16, 0.38, 11.0},  {15, 16, 0.33, 11.0},  {15, 21, 0.41, 11.0},
    {15, 24, 0.41, 11.0},  {16, 17, 0.35, 11.0},  {16, 19, 0.34, 11.0},
    {17, 18, 0.32, 11.0},  {17, 22, 0.54, 11.0},  {18, 21, 0.35, 11.0},
    {19, 20, 0.38, 11.0},  {20, 23, 0.34, 11.0},  {21, 22, 0.45, 11.0},
};

double unavail_from_mttf(double mttf_hr, double mttr_hr) {
  return mttr_hr / (mttf_hr + mttr_hr);
}

}  // namespace

// ═══════════════════════════════════════════════════════════════════════
// build_ieee_rts24_load_profile
// ═══════════════════════════════════════════════════════════════════════
LoadProfile build_ieee_rts24_load_profile(int hours_per_year) {
  LoadProfile profile;
  if (hours_per_year <= 0) {
    spdlog::warn("build_ieee_rts24_load_profile: hours_per_year={} <= 0; returning empty profile",
                 hours_per_year);
    return profile;
  }
  profile.factors.resize(hours_per_year);

  for (int h = 0; h < hours_per_year; ++h) {
    int week        = std::min((h / 168) + 1, 52);
    int day         = (h / 24) % 7;          // 0=Mon … 6=Sun
    int hour_of_day = h % 24;

    double week_factor = kWeeklyFactors[week - 1];
    double day_factor  = kDailyFactors[day];
    Season season      = get_season(week);
    bool   is_weekend  = (day >= 5);
    int    col         = get_hourly_col(season, is_weekend);
    double hour_factor = kHourlyFactors[hour_of_day][col];

    profile.factors[h] = week_factor * day_factor * hour_factor;
  }
  return profile;
}

// ═══════════════════════════════════════════════════════════════════════
// apply_ieee24_reliability_data
// ═══════════════════════════════════════════════════════════════════════
void apply_ieee24_reliability_data(HybridPowerSystem& sys) {
  const bool unit_level =
      sys.ac.generators.size() == std::size(kIeee24UnitReliability) &&
      sys.ac.branches.size() == std::size(kIeee24UnitBranchReliability);
  const bool aggregate =
      sys.ac.generators.size() == std::size(kIeee24AggregateGenReliability) &&
      sys.ac.branches.size() == std::size(kIeee24AggregateBranchReliability);
  if (!unit_level && !aggregate) {
    throw std::invalid_argument(
        "apply_ieee24_reliability_data: system is neither the 33-row/38-branch "
        "MATPOWER RTS-24 nor the explicit 10-generator/33-branch aggregate builder");
  }

  const auto apply_generators = [&](const auto& table, const char* scope) {
    for (size_t i = 0; i < sys.ac.generators.size(); ++i) {
      auto& gen = sys.ac.generators[i];
      const auto& rd = table[i];
      if (gen.bus != rd.bus || std::abs(gen.pmax_mw - rd.pmax_mw) > 1e-6) {
        throw std::invalid_argument(
            "apply_ieee24_reliability_data: " + std::string(scope) +
            " generator row " + std::to_string(i + 1) +
            " does not match the RTS-24 bus/Pmax signature");
      }
      gen.mttr_hr = rd.mttr_hr;
      gen.forced_outage_rate = unavail_from_mttf(rd.mttf_hr, rd.mttr_hr);
    }
  };
  const auto apply_branches = [&](const auto& table, const char* scope) {
    for (size_t i = 0; i < sys.ac.branches.size(); ++i) {
      auto& branch = sys.ac.branches[i];
      const auto& rd = table[i];
      const bool same_endpoints =
          (branch.from_bus == rd.from_bus && branch.to_bus == rd.to_bus) ||
          (branch.from_bus == rd.to_bus && branch.to_bus == rd.from_bus);
      if (!same_endpoints) {
        throw std::invalid_argument(
            "apply_ieee24_reliability_data: " + std::string(scope) +
            " branch row " + std::to_string(i + 1) +
            " does not match the RTS-24 endpoint signature");
      }
      branch.failure_rate = rd.lambda;
      branch.mttr_hr = rd.repair_hr;
    }
  };

  if (unit_level) {
    apply_generators(kIeee24UnitReliability, "unit-level");
    apply_branches(kIeee24UnitBranchReliability, "unit-level");
  } else {
    apply_generators(kIeee24AggregateGenReliability, "aggregate");
    apply_branches(kIeee24AggregateBranchReliability, "aggregate");
  }
}

// ═══════════════════════════════════════════════════════════════════════
// apply_comprehensive_reliability_data
// Sets typical distribution-level failure data on the comprehensive
// hybrid AC/DC test system (build_comprehensive_hybrid_acdc).
// ═══════════════════════════════════════════════════════════════════════
void apply_comprehensive_reliability_data(HybridPowerSystem& sys) {
  spdlog::info("Applying comprehensive reliability data to {} buses, "
               "{} branches, {} generators",
               sys.ac.buses.size(), sys.ac.branches.size(),
               sys.ac.generators.size());

  // ── Generators (slack / dispatchable) ──
  for (auto& g : sys.ac.generators) {
    g.forced_outage_rate = 0.02;
    g.mttr_hr            = 40.0;
  }

  // ── AC Branches (distribution feeders / cables) ──
  for (auto& br : sys.ac.branches) {
    br.failure_rate = 0.30;
    br.mttr_hr      = 6.0;
  }
  // Overhead lines on Feeder 3 (residential) — higher failure rate
  for (auto& br : sys.ac.branches) {
    if (br.from_bus >= 15 || br.to_bus >= 15) {
      br.failure_rate = 0.50;
      br.mttr_hr      = 8.0;
    }
  }
  // Underground cables on Feeder 1 (industrial) — lower rate, longer repair
  for (auto& br : sys.ac.branches) {
    if ((br.from_bus >= 4 && br.from_bus <= 8) &&
        (br.to_bus   >= 4 && br.to_bus   <= 8)) {
      br.failure_rate = 0.08;
      br.mttr_hr      = 24.0;
    }
  }
  // HV / OLTC branches (buses 1–3): very reliable
  for (auto& br : sys.ac.branches) {
    if (br.from_bus <= 3 && br.to_bus <= 3) {
      br.failure_rate = 0.04;
      br.mttr_hr      = 12.0;
    }
  }

  // ── Static Generators (DG: diesel, gas peaker, fuel cell) ──
  for (auto& sg : sys.ac.static_generators) {
    sg.mtbf_hours = 3000.0;
    sg.mttr_hours =   24.0;
  }

  // ── Renewable Generators (wind, hydro) ──
  for (auto& rg : sys.ac.renewable_gens) {
    rg.mtbf_hours = 4000.0;
    rg.mttr_hours =   48.0;
  }

  // ── PV Systems ──
  for (auto& pv : sys.ac.pv_systems) {
    pv.mtbf_hours          =  8000.0;
    pv.mttr_hours          =    12.0;
    pv.mtbf_panel_hours    = 200000.0;
    pv.mttr_panel_hours    =    24.0;
    pv.mtbf_inverter_hours =  20000.0;
    pv.mttr_inverter_hours =     8.0;
  }

  // ── Storage (BESS) ──
  for (auto& st : sys.ac.storage) {
    st.forced_outage_rate = 0.015;
    st.mttr_hr            =  24.0;
    st.mtbf_battery_hr    = 40000.0;
    st.mttr_battery_hr    =    48.0;
    st.mtbf_pcs_hr        = 15000.0;
    st.mttr_pcs_hr        =    12.0;
    st.mtbf_bms_hr        = 50000.0;
    st.mttr_bms_hr        =     4.0;
  }

  // ── Transformers 2W ──
  for (auto& t : sys.ac.transformers_2w) {
    t.mtbf_hours = 300000.0;
    t.mttr_hours =    200.0;
  }

  // ── Transformers 3W ──
  for (auto& t : sys.ac.transformers_3w) {
    t.mtbf_hours = 250000.0;
    t.mttr_hours =    250.0;
  }

  // ── VSC Converters ──
  for (auto& v : sys.vsc_converters) {
    v.forced_outage_rate = 0.01;
    v.mttr_hr            = 48.0;
  }

  // ── DC Branches ──
  for (auto& db : sys.dc.branches) {
    db.mtbf_hours = 50000.0;
    db.mttr_hours =    24.0;
  }

  // ── DC-DC Converters ──
  for (auto& dc : sys.dc.dcdc_converters) {
    dc.mtbf_hours = 20000.0;
    dc.mttr_hours =    36.0;
  }

  // ── AC Loads — set customer counts ──
  for (auto& ld : sys.ac.loads) {
    if (ld.bus >= 4 && ld.bus <= 8) {
      // Industrial: fewer large customers
      ld.n_customers = static_cast<int>(ld.p_mw * 5);
    } else if (ld.bus >= 9 && ld.bus <= 14) {
      // Commercial: moderate density
      ld.n_customers = static_cast<int>(ld.p_mw * 50);
    } else if (ld.bus >= 15) {
      // Residential: many small customers
      ld.n_customers = static_cast<int>(ld.p_mw * 500);
    } else {
      ld.n_customers = std::max(1, static_cast<int>(ld.p_mw * 10));
    }
  }

  spdlog::info("Comprehensive reliability data applied: "
               "{} loads with customer counts set",
               sys.ac.loads.size());
}

}  // namespace hacdcpf::analysis
