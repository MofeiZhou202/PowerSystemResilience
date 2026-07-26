/// case_builder.cpp — Synthetic SCUC test case generators
///
/// Three canonical test cases:
///   build_3bus_case()   — tiny 3-bus/2-gen problem for unit tests
///   build_6bus_case()   — standard 6-bus IEEE case
///   build_ieee39_case() — IEEE 39-bus New England system (10 generators)
///
/// Also provides scuc_input_to_json() for golden-file export.

#include "mipsolvers/scuc/case_builder.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <nlohmann/json.hpp>

namespace mipsolvers::scuc {

using json = nlohmann::json;

// ─────────────────────────────────────────────────────────────────────────────
// Helper: replicate a single-period load as a T-period sinusoidal shape
//   peak_pu = 1.0, trough_pu = min_frac (at midnight/noon etc.)
// ─────────────────────────────────────────────────────────────────────────────
static std::vector<double> make_load_profile(int T, double dt, double p_mw,
                                             double min_frac = 0.65) {
  std::vector<double> prof(static_cast<size_t>(T));
  // Simple double-peak daily shape centred at hours 10 and 19 (relative to T*dt hours)
  const double total_hr = T * dt;
  const double peak1_hr = total_hr * 10.0 / 24.0;
  const double peak2_hr = total_hr * 19.0 / 24.0;
  const double sigma = total_hr * 2.0 / 24.0;
  for (int t = 0; t < T; ++t) {
    const double hr = (t + 0.5) * dt;
    const double g1 = std::exp(-0.5 * ((hr - peak1_hr) / sigma) * ((hr - peak1_hr) / sigma));
    const double g2 = std::exp(-0.5 * ((hr - peak2_hr) / sigma) * ((hr - peak2_hr) / sigma));
    const double pu = min_frac + (1.0 - min_frac) * std::max(g1, g2);
    prof[static_cast<size_t>(t)] = pu;  // multiplied by p_mw in the load struct
  }
  (void)p_mw;
  return prof;
}

static std::vector<double> make_wind_profile(int T, double dt) {
  std::vector<double> prof(static_cast<size_t>(T));
  // Higher output at night/early morning (typical northern China pattern)
  const double total_hr = T * dt;
  const double low_hr = total_hr * 13.0 / 24.0;
  const double sigma = total_hr * 4.0 / 24.0;
  for (int t = 0; t < T; ++t) {
    const double hr = (t + 0.5) * dt;
    const double dip = std::exp(-0.5 * ((hr - low_hr) / sigma) * ((hr - low_hr) / sigma));
    prof[static_cast<size_t>(t)] = std::max(0.1, 1.0 - 0.6 * dip);
  }
  return prof;
}

static std::vector<double> make_solar_profile(int T, double dt) {
  std::vector<double> prof(static_cast<size_t>(T));
  const double total_hr = T * dt;
  const double noon = total_hr * 12.0 / 24.0;
  const double sigma = total_hr * 3.0 / 24.0;
  for (int t = 0; t < T; ++t) {
    const double hr = (t + 0.5) * dt;
    const double v = std::exp(-0.5 * ((hr - noon) / sigma) * ((hr - noon) / sigma));
    prof[static_cast<size_t>(t)] = v > 0.01 ? v : 0.0;
  }
  return prof;
}

// ─────────────────────────────────────────────────────────────────────────────
// 3-bus case  (2 generators, 1 load, 2 branches)
// ─────────────────────────────────────────────────────────────────────────────
//    Bus 0 ─── Branch 0 ──── Bus 1 ─── Branch 1 ──── Bus 2
//     G1(0)                   G2(1)                   Load(2)
SCUCInput build_3bus_case(int T, double dt) {
  SCUCInput inp;

  inp.config.num_periods      = T;
  inp.config.period_length_hr = dt;
  inp.config.n_segments       = 2;
  inp.config.spinning_reserve_req = 0.05;
  inp.config.regulation_up_req    = 0.02;
  inp.config.voll = 5000.0;
  inp.config.vocc = 100.0;
  inp.config.M1_line_slack_penalty = 1e4;
  inp.config.enable_market_cuts = true;

  inp.num_buses = 3;

  // ── Generators ───────────────────────────────────────────────────────────
  {
    Generator g;
    g.name = "G1"; g.bus = 0;
    g.pmin = 30.0; g.pmax = 200.0;
    g.ramp_up_mw_min = 3.0; g.ramp_dn_mw_min = 3.0;
    g.min_up_time_hr = 2.0; g.min_dn_time_hr = 2.0;
    g.startup_cost = 800.0; g.no_load_cost = 50.0;
    g.bid_segments = {{30.0, 20.0}, {170.0, 28.0}};  // {quantity, price}
    g.spinning_reserve_price = 3.0;
    g.regulation_up_price    = 4.0;
    g.regulation_down_price  = 2.0;
    inp.generators.push_back(g);
  }
  {
    Generator g;
    g.name = "G2"; g.bus = 1;
    g.pmin = 20.0; g.pmax = 120.0;
    g.ramp_up_mw_min = 2.5; g.ramp_dn_mw_min = 2.5;
    g.min_up_time_hr = 1.0; g.min_dn_time_hr = 1.0;
    g.startup_cost = 400.0; g.no_load_cost = 30.0;
    g.bid_segments = {{20.0, 25.0}, {100.0, 35.0}};
    g.spinning_reserve_price = 4.0;
    g.regulation_up_price    = 5.0;
    g.regulation_down_price  = 3.0;
    inp.generators.push_back(g);
  }

  // ── Branches ─────────────────────────────────────────────────────────────
  {
    Branch b; b.from = 0; b.to = 1; b.reactance = 0.1; b.rating_mw = 150.0;
    inp.branches.push_back(b);
  }
  {
    Branch b; b.from = 1; b.to = 2; b.reactance = 0.12; b.rating_mw = 150.0;
    inp.branches.push_back(b);
  }

  // ── Load ─────────────────────────────────────────────────────────────────
  {
    Load l; l.bus = 2; l.p_mw = 180.0;
    inp.loads.push_back(l);
  }
  // Load profile (multiplier applied to l.p_mw inside scuc.cpp)
  inp.profiles.load.push_back(make_load_profile(T, dt, 180.0));

  // ── Initial status ────────────────────────────────────────────────────────
  inp.initial_status.commitment = {1.0, 0.0};
  inp.initial_status.dispatch   = {80.0, 0.0};

  return inp;
}

// ─────────────────────────────────────────────────────────────────────────────
// 6-bus case  (3 generators, 6 buses, 7 branches)
// ─────────────────────────────────────────────────────────────────────────────
// Classic IEEE 6-bus system (Garver's 6-bus).
// Bus layout:
//   Bus 0: G1 (coal)
//   Bus 1: G2 (gas)
//   Bus 2: Load 1
//   Bus 3: G3 (gas peaker), Load 2, optional Wind
//   Bus 4: Load 3
//   Bus 5: Load 4, optional Storage
SCUCInput build_6bus_case(int T, double dt, bool with_wind, bool with_storage) {
  SCUCInput inp;

  inp.config.num_periods      = T;
  inp.config.period_length_hr = dt;
  inp.config.n_segments       = 3;
  inp.config.spinning_reserve_req = 0.05;
  inp.config.regulation_up_req    = 0.03;
  inp.config.regulation_down_req  = 0.02;
  inp.config.voll = 8000.0;
  inp.config.vocc = 200.0;
  inp.config.M1_line_slack_penalty = 1e5;
  inp.config.enable_market_cuts = true;

  inp.num_buses = 6;

  // ── Generators ───────────────────────────────────────────────────────────
  auto add_gen = [&](const char* name, int bus,
                     double pmin, double pmax,
                     double rup, double rdn,
                     double mup, double mdn,
                     double su_cost, double nl_cost,
                     std::initializer_list<std::pair<double,double>> segs,
                     double sp_price = 3.0) {
    Generator g;
    g.name = name; g.bus = bus;
    g.pmin = pmin; g.pmax = pmax;
    g.ramp_up_mw_min = rup; g.ramp_dn_mw_min = rdn;
    g.min_up_time_hr = mup; g.min_dn_time_hr = mdn;
    g.startup_cost = su_cost; g.no_load_cost = nl_cost;
    for (const auto& [q, p] : segs) g.bid_segments.push_back({q, p});
    g.spinning_reserve_price = sp_price;
    g.regulation_up_price    = sp_price + 1.0;
    g.regulation_down_price  = sp_price - 1.0;
    inp.generators.push_back(std::move(g));
  };

  add_gen("Coal_G1", 0,  80.0, 400.0, 6.0, 6.0, 4.0, 4.0,
          3000.0, 150.0,
          {{80.0,22.0},{200.0,27.0},{120.0,33.0}}, 3.0);
  add_gen("Gas_G2",  1,  50.0, 200.0, 5.0, 5.0, 2.0, 2.0,
          1200.0, 60.0,
          {{50.0,28.0},{100.0,35.0},{50.0,42.0}},  4.0);
  add_gen("Peaker",  3,  10.0,  80.0, 8.0, 8.0, 1.0, 1.0,
          500.0, 20.0,
          {{10.0,38.0},{50.0,52.0},{20.0,65.0}},  5.0);

  // ── Branches ─────────────────────────────────────────────────────────────
  struct BrSpec { int f, t; double x, r; };
  for (const auto& bs : std::initializer_list<BrSpec>{
        {0, 1, 0.04, 200.0}, {0, 3, 0.06, 160.0}, {0, 4, 0.06, 160.0},
        {1, 2, 0.04, 200.0}, {1, 3, 0.04, 200.0}, {2, 4, 0.08, 130.0},
        {3, 5, 0.08, 130.0}}) {
    Branch b;
    b.from = bs.f; b.to = bs.t; b.reactance = bs.x; b.rating_mw = bs.r;
    inp.branches.push_back(b);
  }

  // ── Loads ─────────────────────────────────────────────────────────────────
  const double total_load = 500.0;
  std::vector<std::pair<int,double>> load_spec = {{2,0.30},{3,0.25},{4,0.25},{5,0.20}};
  for (const auto& [bus, frac] : load_spec) {
    Load l; l.bus = bus; l.p_mw = total_load * frac;
    inp.loads.push_back(l);
  }
  for (int d = 0; d < static_cast<int>(inp.loads.size()); ++d)
    inp.profiles.load.push_back(make_load_profile(T, dt, inp.loads[static_cast<size_t>(d)].p_mw));

  // ── Wind (optional) ────────────────────────────────────────────────────────
  if (with_wind) {
    WindUnit w; w.bus = 3; w.pmax = 100.0;
    inp.wind.push_back(w);
    inp.profiles.wind.push_back(make_wind_profile(T, dt));
    // Scale to fractions of pmax
    for (double& v : inp.profiles.wind.back()) v *= w.pmax;
    inp.config.M2_renewable_curtail_penalty = 50.0;
  }

  // ── Storage (optional) ─────────────────────────────────────────────────────
  if (with_storage) {
    StorageUnit s;
    s.bus = 5;
    s.pmax_charge    = 60.0;
    s.pmax_discharge = 60.0;
    s.energy_capacity_mwh = 240.0;
    s.efficiency = 0.85;
    s.soc_init   = 0.50;
    s.soc_min    = 0.10;
    s.soc_final  = 0.50;
    s.cycle_limit = 1.0;
    s.charge_bid_price    = -5.0;   // charging revenue (negative = net payment)
    s.discharge_bid_price =  8.0;   // discharging revenue
    inp.storage.push_back(s);
  }

  // ── Initial status ────────────────────────────────────────────────────────
  inp.initial_status.commitment = {1.0, 0.0, 0.0};
  inp.initial_status.dispatch   = {180.0, 0.0, 0.0};

  return inp;
}

// ─────────────────────────────────────────────────────────────────────────────
// IEEE 39-bus New England test case
// ─────────────────────────────────────────────────────────────────────────────
// 10 generators, 39 buses, 46 branches.
// Generator data from standard IEEE 39-bus test case (Pai, 1989), adapted for
// SCUC with piecewise-linear costs from Bergen & Vittal (2000).
// Load data scaled to approximately 6000 MW peak system load.
SCUCInput build_ieee39_case(int T, double dt, bool with_wind, bool with_solar) {
  SCUCInput inp;

  inp.config.num_periods      = T;
  inp.config.period_length_hr = dt;
  inp.config.n_segments       = 3;
  inp.config.spinning_reserve_req = 0.05;
  inp.config.regulation_up_req    = 0.03;
  inp.config.regulation_down_req  = 0.02;
  inp.config.voll = 10000.0;
  inp.config.vocc = 300.0;
  inp.config.M1_line_slack_penalty = 1e6;
  inp.config.enable_market_cuts = true;

  inp.num_buses = 39;

  // ── Generators ───────────────────────────────────────────────────────────
  // Bus (0-indexed), Pmin, Pmax, RampUp, RampDn (MW/min), MinUp, MinDn (hr),
  // StartupCost ($), NoLoad ($/hr), BidSegs [{qty,price}]×3
  // Bus indices: standard IEEE 39-bus 1-indexed bus N = 0-indexed bus N-1.
  // Generator buses: 31-39 (1-indexed) = 30-38 (0-indexed), plus bus 30 (1-indexed) = 29 (0-indexed).
  struct GenSpec {
    const char* name;
    int bus;
    double pmin, pmax, rup, rdn, mup, mdn, su_cost, nl_cost;
    double seg1_q, seg1_p, seg2_q, seg2_p, seg3_q, seg3_p;
    double pfr_alpha;
  };
  // Data adapted from PSS/E tutorials and published IEEE 39-bus SCUC datasets.
  const GenSpec gens[] = {
    // name  bus  pmin  pmax  rup  rdn mup mdn su_cost nl_cost  s1q  s1p  s2q  s2p  s3q  s3p  pfr
    {"G1",   38,  250,  1100,  15,  15,  5,  5,  15000, 400,  250, 20,  550, 25,  300, 31, 0.05},
    {"G2",   30,  100,   650,  10,  10,  4,  4,   8000, 220,  100, 22,  350, 28,  200, 34, 0.04},
    {"G3",   31,  150,   725,  12,  12,  4,  4,   9000, 250,  150, 21,  375, 27,  200, 33, 0.04},
    {"G4",   32,  150,   650,  12,  12,  4,  4,   9000, 240,  150, 23,  300, 29,  200, 35, 0.04},
    {"G5",   33,  100,   508,  10,  10,  3,  3,   7500, 180,  100, 24,  250, 30,  158, 38, 0.03},
    {"G6",   34,   50,   687,   8,   8,  3,  3,   8500, 200,   50, 26,  350, 33,  287, 41, 0.03},
    {"G7",   35,  100,   580,  10,  10,  3,  3,   8000, 190,  100, 25,  280, 31,  200, 39, 0.03},
    {"G8",   36,   50,   564,   8,   8,  2,  2,   6000, 150,   50, 27,  300, 34,  214, 43, 0.03},
    {"G9",   37,   50,   865,  10,  10,  2,  2,   9000, 220,   50, 22,  450, 28,  365, 36, 0.04},
    // G10 bus = 29 (0-indexed) = bus 30 (1-indexed): main network bus with direct tie lines
    // {29,0} and {29,2} already in the branch list.  Do NOT add a separate transformer.
    {"G10",  29,  300,  1100,  15,  15,  6,  6,  16000, 450,  300, 19,  550, 24,  250, 29, 0.06},
  };

  for (const auto& gs : gens) {
    Generator g;
    g.name = gs.name;
    g.bus = gs.bus;
    g.pmin = gs.pmin; g.pmax = gs.pmax;
    g.ramp_up_mw_min = gs.rup; g.ramp_dn_mw_min = gs.rdn;
    g.min_up_time_hr = gs.mup; g.min_dn_time_hr = gs.mdn;
    g.startup_cost = gs.su_cost; g.no_load_cost = gs.nl_cost;
    g.bid_segments.push_back({gs.seg1_p, gs.seg1_q});  // BidSegment{price, quantity}
    g.bid_segments.push_back({gs.seg2_p, gs.seg2_q});
    g.bid_segments.push_back({gs.seg3_p, gs.seg3_q});
    g.spinning_reserve_price = 4.0;
    g.regulation_up_price    = 5.0;
    g.regulation_down_price  = 3.0;
    g.pfr_alpha = gs.pfr_alpha;
    inp.generators.push_back(std::move(g));
  }
  inp.config.pfr_reserve_req_mw = 300.0;  // 5% of ~6 GW peak

  // ── Branches (51 lines) ───────────────────────────────────────────────────
  // Standard IEEE 39-bus (0-indexed) topology + reactances.
  // Generator step-up transformers follow the standard mapping
  //   (1-indexed): 31→6, 32→10, 33→19, 34→20, 35→22, 36→23, 37→25, 38→29, 39→1
  //   (0-indexed): 30→5,  31→9,  32→18, 33→19, 34→21, 35→22, 36→24, 37→28, 38→0
  struct BrSpec { int f, t; double x, r; };
  const BrSpec branches[] = {
    // ── Transmission lines (standard IEEE 39-bus, 0-indexed) ──────────────
    // Ratings calibrated for full thermal dispatch (~7.4 GW) to avoid artificial congestion.
    // Key corridors {0,38}, {29,0}, {4,5}, {15,16} sized to carry full cross-zonal flow.
    { 0, 1,  0.0357, 1000}, { 0,38, 0.0250, 2200}, { 1, 2,  0.0411, 900},
    { 2, 3,  0.0085, 1100}, { 2,24, 0.0382, 900},  { 3, 4,  0.0302, 1000},
    { 3,13,  0.0129, 1100}, { 4, 5,  0.0022, 1000}, { 4,11,  0.0222, 700},
    { 5, 6,  0.0324, 900},  { 6, 7,  0.0014,  800}, { 6,10,  0.0105, 700},
    { 7, 8,  0.0106, 800},  { 7,24,  0.0053,  800}, { 8, 9,  0.0272, 700},
    { 9,11,  0.0138, 700},  { 9,38,  0.0182, 1500}, {10,12,  0.0302, 700},
    {11,12,  0.0173, 700},  {12,13,  0.0203, 900},  {13,14,  0.0591, 1100},
    {14,15,  0.0132, 700},  {15,16,  0.0176,  900}, {16,17,  0.0137, 700},
    {16,18,  0.0007, 900},  {16,20,  0.0050, 900},  {17,26,  0.0616, 600},
    {18,19,  0.0129, 700},  {20,21,  0.0140, 700},  {20,22,  0.0066, 800},
    {21,22,  0.0096, 800},  {22,23,  0.0350, 700},  {23,24,  0.0966, 500},
    {24,25,  0.0394, 600},  {24,37,  0.0423, 600},  {25,26,  0.0820, 500},
    {25,27,  0.0596, 500},  {26,27,  0.0323, 700},  {26,28,  0.0514, 600},
    {27,28,  0.0229, 700},  {27,29,  0.0755, 1000},
    // {28,37}: G9 step-up transformer; rating must cover G9 Pmax (865 MW) + margin
    {28,37,  0.0131, 1000},
    // {29,0},{29,2}: G10 tie lines; sized for full G10 Pmax (1100 MW) cross-zonal flow
    {29, 2,  0.0200, 1800},  {29, 0,  0.0181, 2200},
    // ── Generator step-up transformers (G2-G8) ──────────────────────────────
    // G1  (bus38): connected via {0,38}(2200MW) and {9,38}(1500MW) — no transformer needed
    // G9  (bus37): connected via {28,37}(1000MW) above              — no transformer needed
    // G10 (bus29): connected via {29,0}(2200MW) and {29,2}(1800MW) — no transformer needed
    // Reactances ~0.018-0.022 pu; ratings ≥ generator Pmax
    { 5, 30,  0.0180,  750},  // G2  (bus30, 650 MW) → bus5  (bus31→bus6  in 1-indexed)
    { 9, 31,  0.0200,  850},  // G3  (bus31, 725 MW) → bus9  (bus32→bus10 in 1-indexed)
    {18, 32,  0.0180,  750},  // G4  (bus32, 650 MW) → bus18 (bus33→bus19 in 1-indexed)
    {19, 33,  0.0200,  600},  // G5  (bus33, 508 MW) → bus19 (bus34→bus20 in 1-indexed)
    {21, 34,  0.0180,  800},  // G6  (bus34, 687 MW) → bus21 (bus35→bus22 in 1-indexed)
    {22, 35,  0.0200,  700},  // G7  (bus35, 580 MW) → bus22 (bus36→bus23 in 1-indexed)
    {24, 36,  0.0200,  650},  // G8  (bus36, 564 MW) → bus24 (bus37→bus25 in 1-indexed)
  };
  for (const auto& bs : branches) {
    Branch b;
    b.from = bs.f; b.to = bs.t; b.reactance = bs.x; b.rating_mw = bs.r;
    inp.branches.push_back(b);
  }

  // ── Loads (21 load buses, ~6 GW peak aggregate) ───────────────────────────
  struct LoadSpec { int bus; double p_mw; };
  const LoadSpec loads[] = {
    { 0, 97.6}, { 1, 0.0},  { 2, 322.0}, { 3, 500.0}, { 4, 0.0},
    { 6, 233.8},{ 7, 522.0},{ 8, 0.0},   { 9,  6.0},  {11, 8.5},
    {12, 0.0},  {14,320.0}, {15,329.0},  {16,158.0},  {18,  0.0},
    {19,628.0}, {21,274.0}, {22,247.5},  {24,308.6},  {25,224.0},
    {27,139.0}, {28,281.0}, {30,  0.0},  {35, 0.0},   {36,  0.0},
    {38, 0.0}
  };
  for (const auto& ls : loads) {
    if (ls.p_mw < 1.0) continue;
    Load l; l.bus = ls.bus; l.p_mw = ls.p_mw;
    inp.loads.push_back(l);
  }
  // Load profile (same shape for all buses)
  for (size_t d = 0; d < inp.loads.size(); ++d)
    inp.profiles.load.push_back(make_load_profile(T, dt, inp.loads[d].p_mw));

  // ── Wind (optional, 3 sites) ───────────────────────────────────────────────
  if (with_wind) {
    struct WindSpec { int bus; double pmax; };
    const WindSpec wsites[] = {{5, 400.0}, {14, 300.0}, {26, 350.0}};
    for (const auto& ws : wsites) {
      WindUnit w; w.bus = ws.bus; w.pmax = ws.pmax;
      inp.wind.push_back(w);
      auto prof = make_wind_profile(T, dt);
      for (double& v : prof) v *= ws.pmax;
      inp.profiles.wind.push_back(prof);
    }
    inp.config.M2_renewable_curtail_penalty = 80.0;
  }

  // ── Solar (optional, 2 sites) ──────────────────────────────────────────────
  if (with_solar) {
    struct SolarSpec { int bus; double pmax; };
    const SolarSpec ssites[] = {{21, 250.0}, {36, 200.0}};
    for (const auto& ss : ssites) {
      SolarUnit s; s.bus = ss.bus; s.pmax = ss.pmax;
      inp.solar.push_back(s);
      auto prof = make_solar_profile(T, dt);
      for (double& v : prof) v *= ss.pmax;
      inp.profiles.solar.push_back(prof);
    }
    if (!with_wind)
      inp.config.M2_renewable_curtail_penalty = 80.0;
  }

  // ── Initial status (4 of 10 units online) ────────────────────────────────
  inp.initial_status.commitment = {1,1,0,0,1,0,0,0,0,1};
  inp.initial_status.dispatch   = {500,300,0,0,200,0,0,0,0,600};

  return inp;
}

// ─────────────────────────────────────────────────────────────────────────────
// IEEE 118-bus system (MATPOWER case118)
//   54 generators, 186 branches, 99 loads, ~4242 MW peak aggregate load
//   Bus numbering: 0-indexed (MATPOWER 1-indexed − 1)
//   Branch ratings: scaled from standard 100 MVA base to MW capacity
//   Bid curves: 3-segment piecewise-linear fit to quadratic cost a*p²+b*p
//   Startup/no-load/ramp params: synthetic, calibrated by unit class
// ─────────────────────────────────────────────────────────────────────────────
SCUCInput build_ieee118_case(int T, double dt, bool with_wind, bool with_solar) {
  SCUCInput inp;

  inp.config.num_periods      = T;
  inp.config.period_length_hr = dt;
  inp.config.n_segments       = 3;
  inp.config.spinning_reserve_req = 0.05;
  inp.config.regulation_up_req    = 0.03;
  inp.config.regulation_down_req  = 0.02;
  inp.config.voll             = 10000.0;
  inp.config.vocc             = 500.0;
  inp.config.mip_gap          = 0.003;
  inp.config.time_limit_sec   = 600.0;
  inp.config.M1_line_slack_penalty = 1e6;
  inp.config.enable_market_cuts    = true;
  inp.config.solve_sced = true;
  inp.config.solve_lmp  = true;
  inp.num_buses = 118;

  // ── Helper: linearise quadratic cost a*p^2 + b*p into 3 equal segments ──
  // Segments are defined above Pmin (i.e., for p in [Pmin, Pmax]).
  // avg marginal cost of each segment = b + 2*a*(Pmin + (2k-1)*(Pmax-Pmin)/6)
  // for k=1,2,3.  segment quantity = (Pmax-Pmin)/3.
  auto make_segments = [](double pmin, double pmax, double a_coeff, double b_coeff)
      -> std::vector<BidSegment> {
    const double range = pmax - pmin;
    const double w = range / 3.0;
    std::vector<BidSegment> segs;
    for (int k = 1; k <= 3; ++k) {
      const double mc = b_coeff + 2.0 * a_coeff * (pmin + (2 * k - 1) * w / 2.0);
      segs.push_back({mc, w});
    }
    return segs;
  };

  // ── Generator data ────────────────────────────────────────────────────────
  // Format: name, bus(0-idx), pmin, pmax, rup, rdn (MW/min),
  //         mup, mdn (hr), su_cost ($), nl_cost ($/hr),
  //         a_coeff ($/MW^2h), b_coeff ($/MWh at Pmin)
  // Unit classes:
  //   S  = small peaker  (Pmax ≤ 107)  : ramp 2%/min, mup/mdn=1h, su=1000
  //   M  = mid merit     (108-320 MW)  : ramp 1.5%/min, mup/mdn=3h, su=3000
  //   L  = large baseload (> 320 MW)   : ramp 1%/min,  mup/mdn=5h, su=10000
  struct GenSpec {
    const char* name;
    int bus;
    double pmin, pmax, rup, rdn, mup, mdn, su_cost, nl_cost, a_coeff, b_coeff;
  };
  // MATPOWER case118 gencost (type-2, quadratic): mostly a=0.00697, b varies 8.0-9.5
  // ($/MWh²·h, $/MWh). Startup / min-time / no-load: synthetic class-based.
  const GenSpec gens[] = {
    // name   bus  pmin  pmax   rup   rdn  mup mdn  su    nl    a        b
    // S-class (pmax<=107): pmin=20 MW (20% of rated), mup/mdn=2h for realistic gas peakers
    {"G01",    0,  20,  100,  2.0,  2.0,  2,  2,  1000, 40,  0.00697, 8.0},
    {"G02",    3,  20,  100,  2.0,  2.0,  2,  2,  1000, 40,  0.00697, 8.0},
    {"G03",    5,  20,  100,  2.0,  2.0,  2,  2,  1000, 40,  0.00697, 8.0},
    {"G04",    7,  20,  100,  2.0,  2.0,  2,  2,  1000, 40,  0.00697, 8.0},
    {"G05",    9, 150,  550,  5.5,  5.5,  5,  4, 12000,280,  0.00658, 9.0},
    {"G06",   11,   0,  185,  2.8,  2.8,  2,  2,  2500, 80,  0.00697, 8.0},
    {"G07",   14,  20,  100,  2.0,  2.0,  2,  2,  1000, 40,  0.00697, 8.0},
    {"G08",   17,  20,  100,  2.0,  2.0,  2,  2,  1000, 40,  0.00697, 8.0},
    {"G09",   18,  20,  100,  2.0,  2.0,  2,  2,  1000, 40,  0.00697, 8.0},
    {"G10",   23,  20,  100,  2.0,  2.0,  2,  2,  1000, 40,  0.00697, 8.0},
    {"G11",   24, 140,  320,  3.2,  3.2,  3,  3,  5000,140,  0.00697, 8.5},
    {"G12",   25,   0,  414,  4.1,  4.1,  4,  3,  7000,180,  0.00697, 9.0},
    {"G13",   26,  20,  100,  2.0,  2.0,  2,  2,  1000, 40,  0.00697, 8.0},
    {"G14",   30,  20,  107,  2.1,  2.1,  2,  2,  1000, 40,  0.00697, 8.5},
    {"G15",   31,  20,  100,  2.0,  2.0,  2,  2,  1000, 40,  0.00697, 8.0},
    {"G16",   33,  20,  100,  2.0,  2.0,  2,  2,  1000, 40,  0.00697, 8.0},
    {"G17",   35,  20,  100,  2.0,  2.0,  2,  2,  1000, 40,  0.00697, 8.0},
    {"G18",   39,  20,  100,  2.0,  2.0,  2,  2,  1000, 40,  0.00697, 8.0},
    {"G19",   41,  20,  100,  2.0,  2.0,  2,  2,  1000, 40,  0.00697, 8.0},
    {"G20",   45,  19,  119,  1.8,  1.8,  2,  2,  2000, 55,  0.00697, 8.5},
    {"G21",   48, 204,  304,  3.0,  3.0,  4,  3,  6000,150,  0.00697, 9.5},
    {"G22",   53,  48,  148,  1.5,  1.5,  2,  2,  2000, 65,  0.00697, 8.5},
    {"G23",   54,  20,  100,  2.0,  2.0,  2,  2,  1000, 40,  0.00697, 8.0},
    {"G24",   55,  20,  100,  2.0,  2.0,  2,  2,  1000, 40,  0.00697, 8.0},
    {"G25",   58, 155,  255,  2.6,  2.6,  3,  3,  4500,120,  0.00697, 9.5},
    {"G26",   60,   0,  160,  2.4,  2.4,  2,  2,  2500, 70,  0.00697, 8.5},
    {"G27",   61,   0,  160,  2.4,  2.4,  2,  2,  2500, 70,  0.00697, 8.5},
    {"G28",   64, 391,  491,  4.9,  4.9,  5,  4, 11000,220,  0.00697, 8.5},
    {"G29",   65, 392,  492,  4.9,  4.9,  5,  4, 11000,220,  0.00697, 8.5},
    {"G30",   68, 516,  805,  8.1,  8.1,  6,  5, 18000,400,  0.00450, 9.0},
    {"G31",   69,  20,  100,  2.0,  2.0,  2,  2,  1000, 40,  0.00697, 8.0},
    {"G32",   71,  20,  100,  2.0,  2.0,  2,  2,  1000, 40,  0.00697, 8.0},
    {"G33",   72,  20,  100,  2.0,  2.0,  2,  2,  1000, 40,  0.00697, 8.0},
    {"G34",   73,  20,  100,  2.0,  2.0,  2,  2,  1000, 40,  0.00697, 8.0},
    {"G35",   75,  20,  100,  2.0,  2.0,  2,  2,  1000, 40,  0.00697, 8.0},
    {"G36",   76,  20,  100,  2.0,  2.0,  2,  2,  1000, 40,  0.00697, 8.0},
    {"G37",   79, 477,  577,  5.8,  5.8,  5,  4, 12000,260,  0.00697, 9.5},
    {"G38",   84,  20,  100,  2.0,  2.0,  2,  2,  1000, 40,  0.00697, 8.0},
    {"G39",   86,  20,  104,  1.6,  1.6,  2,  2,  1000, 45,  0.00697, 8.5},
    {"G40",   88, 607,  707,  7.1,  7.1,  6,  5, 16000,360,  0.00450, 9.5},
    {"G41",   89,  20,  100,  2.0,  2.0,  2,  2,  1000, 40,  0.00697, 8.0},
    {"G42",   90,  20,  100,  2.0,  2.0,  2,  2,  1000, 40,  0.00697, 8.0},
    {"G43",   91,  20,  100,  2.0,  2.0,  2,  2,  1000, 40,  0.00697, 8.0},
    {"G44",   98,  20,  100,  2.0,  2.0,  2,  2,  1000, 40,  0.00697, 8.0},
    {"G45",   99, 252,  352,  3.5,  3.5,  4,  3,  6500,160,  0.00697, 9.0},
    {"G46",  102,  40,  140,  1.4,  1.4,  2,  2,  2000, 60,  0.00697, 8.5},
    {"G47",  103,  20,  100,  2.0,  2.0,  2,  2,  1000, 40,  0.00697, 8.0},
    {"G48",  104,  20,  100,  2.0,  2.0,  2,  2,  1000, 40,  0.00697, 8.0},
    {"G49",  106,  20,  100,  2.0,  2.0,  2,  2,  1000, 40,  0.00697, 8.0},
    {"G50",  109,  20,  100,  2.0,  2.0,  2,  2,  1000, 40,  0.00697, 8.0},
    {"G51",  110,  36,  136,  1.4,  1.4,  2,  2,  2000, 60,  0.00697, 8.5},
    {"G52",  111,  20,  100,  2.0,  2.0,  2,  2,  1000, 40,  0.00697, 8.0},
    {"G53",  112,  20,  100,  2.0,  2.0,  2,  2,  1000, 40,  0.00697, 8.0},
    {"G54",  115,  20,  100,  2.0,  2.0,  2,  2,  1000, 40,  0.00697, 8.0},
  };

  for (const auto& gs : gens) {
    Generator g;
    g.name = gs.name;
    g.bus  = gs.bus;
    g.pmin = gs.pmin;  g.pmax = gs.pmax;
    g.ramp_up_mw_min = gs.rup;  g.ramp_dn_mw_min = gs.rdn;
    g.min_up_time_hr = gs.mup;  g.min_dn_time_hr = gs.mdn;
    g.startup_cost   = gs.su_cost;
    g.no_load_cost   = gs.nl_cost;
    g.bid_segments   = make_segments(gs.pmin, gs.pmax, gs.a_coeff, gs.b_coeff);
    g.spinning_reserve_price  = 4.0;
    g.regulation_up_price     = 5.0;
    g.regulation_down_price   = 3.0;
    inp.generators.push_back(std::move(g));
  }

  // ── Branches (186 lines) ─────────────────────────────────────────────────
  // MATPOWER case118 branch data (0-indexed buses).
  // Reactance in pu; rating in MW (= rateA_MVA × 1.0 for 100 MVA base, then
  // scaled: main 230-kV corridors 300 MW, others 200 MW).
  struct BrSpec { int f, t; double x; double r; };
  const BrSpec branches[] = {
    // ── Northern zone (buses 0-11) ─────────────────────────────────────────
    { 0,  1, 0.0999, 200}, { 0,  2, 0.0424, 200}, { 3,  4, 0.0080, 200},
    { 2,  4, 0.1080, 200}, { 4,  5, 0.0540, 200}, { 5,  6, 0.0208, 200},
    { 7,  8, 0.0305, 300}, { 7,  4, 0.0267, 300}, { 8,  9, 0.0322, 300},
    { 3, 10, 0.0688, 200}, { 4, 10, 0.0682, 200}, {10, 11, 0.0196, 200},
    { 1, 11, 0.0616, 200}, { 2, 11, 0.1600, 200}, { 6, 11, 0.0340, 200},
    // ── Buses 12-23 ────────────────────────────────────────────────────────
    {10, 12, 0.0731, 200}, {11, 13, 0.0707, 200}, {12, 14, 0.2444, 150},
    {13, 14, 0.1950, 150}, {11, 15, 0.0834, 200}, {14, 16, 0.0437, 200},
    {15, 16, 0.1801, 200}, {16, 17, 0.0505, 200}, {17, 18, 0.0493, 200},
    {18, 19, 0.1170, 200}, {14, 18, 0.0394, 200}, {19, 20, 0.0849, 200},
    {20, 21, 0.0970, 200}, {21, 22, 0.1590, 200}, {22, 23, 0.0492, 200},
    // ── Buses 23-33 ────────────────────────────────────────────────────────
    {22, 24, 0.0800, 200}, {25, 24, 0.0382, 200}, {24, 26, 0.1630, 200},
    {26, 27, 0.0855, 200}, {27, 28, 0.0943, 200}, {29, 16, 0.0388, 200},
    { 7, 29, 0.0504, 300}, {25, 29, 0.0860, 300}, {16, 30, 0.1563, 150},
    {28, 30, 0.0331, 200}, {22, 31, 0.1153, 200}, {30, 31, 0.0985, 200},
    {26, 31, 0.0755, 200}, {14, 32, 0.1244, 200}, {18, 33, 0.2470, 150},
    // ── Buses 34-46 ────────────────────────────────────────────────────────
    {34, 35, 0.0102, 300}, {34, 36, 0.0497, 300}, {32, 36, 0.1420, 200},
    {33, 35, 0.0268, 200}, {33, 36, 0.0094, 300}, {37, 36, 0.0375, 300},
    {36, 38, 0.1060, 200}, {36, 39, 0.1680, 200}, {29, 37, 0.0540, 300},
    {38, 39, 0.0605, 200}, {39, 40, 0.0487, 200}, {39, 41, 0.1830, 200},
    {40, 41, 0.1350, 200}, {42, 43, 0.2454, 150}, {33, 42, 0.1681, 150},
    {43, 44, 0.0901, 200}, {44, 45, 0.1356, 200}, {45, 46, 0.1270, 200},
    {45, 47, 0.1890, 200}, {46, 48, 0.0625, 200}, {41, 48, 0.3230, 150},
    {41, 48, 0.3230, 150}, {44, 48, 0.1860, 150}, {47, 48, 0.0505, 200},
    // ── Buses 48-64 ────────────────────────────────────────────────────────
    {48, 49, 0.0752, 300}, {48, 50, 0.1370, 300}, {50, 51, 0.0588, 200},
    {51, 52, 0.1635, 200}, {52, 53, 0.1220, 200}, {48, 53, 0.2890, 150},
    {48, 53, 0.2890, 150}, {53, 54, 0.0707, 200}, {53, 55, 0.00955,300},
    {54, 55, 0.0151, 300}, {55, 56, 0.0966, 200}, {49, 56, 0.1340, 200},
    {55, 57, 0.0966, 200}, {50, 57, 0.0719, 200}, {53, 58, 0.2293, 200},
    {55, 58, 0.2510, 150}, {55, 58, 0.2390, 150}, {54, 58, 0.2158, 150},
    {58, 59, 0.1450, 200}, {58, 60, 0.1500, 200}, {59, 60, 0.0135, 300},
    {59, 61, 0.0561, 300}, {60, 61, 0.0376, 300}, {62, 58, 0.0386, 300},
    {62, 63, 0.0200, 300}, {63, 60, 0.0268, 300}, {37, 64, 0.0986, 300},
    {63, 64, 0.0302, 300},
    // ── Buses 64-79 (main 230 kV backbone) ────────────────────────────────
    {48, 65, 0.0919, 300}, {48, 65, 0.0919, 300}, {61, 65, 0.2180, 200},
    {61, 66, 0.1170, 200}, {64, 65, 0.0370, 300}, {65, 66, 0.1015, 300},
    {64, 67, 0.0160, 300}, {46, 68, 0.2778, 150}, {48, 68, 0.3240, 150},
    {67, 68, 0.0370, 300}, {68, 69, 0.1270, 300}, {23, 69, 0.4115, 200},
    {69, 70, 0.0355, 300}, {23, 71, 0.1960, 200}, {70, 71, 0.1800, 200},
    {70, 72, 0.0454, 300}, {69, 74, 0.1323, 200}, {69, 75, 0.1410, 200},
    {68, 75, 0.1220, 300}, {73, 74, 0.0406, 200}, {75, 76, 0.1480, 200},
    {68, 76, 0.1010, 200}, {74, 76, 0.1999, 200}, {76, 77, 0.0124, 300},
    {77, 78, 0.0244, 300}, {76, 79, 0.0485, 300}, {76, 79, 0.1050, 300},
    {78, 79, 0.0704, 200},
    // ── Buses 79-96 ────────────────────────────────────────────────────────
    {67, 80, 0.0202, 300}, {80, 79, 0.0370, 300}, {76, 81, 0.0853, 200},
    {81, 82, 0.03665,200}, {82, 83, 0.1320, 200}, {82, 84, 0.1480, 200},
    {83, 84, 0.0641, 200}, {84, 85, 0.1230, 200}, {85, 86, 0.2074, 150},
    {84, 87, 0.1020, 200}, {84, 88, 0.1730, 200}, {87, 88, 0.0712, 200},
    {88, 89, 0.1880, 200}, {88, 90, 0.0997, 200}, {89, 90, 0.0836, 200},
    {88, 91, 0.0505, 200}, {88, 91, 0.1581, 200}, {90, 91, 0.1272, 200},
    {91, 92, 0.0848, 200}, {91, 93, 0.1580, 200}, {92, 93, 0.0732, 200},
    {93, 94, 0.0434, 200}, {79, 95, 0.1820, 200}, {81, 95, 0.0530, 200},
    {93, 95, 0.0869, 200}, {79, 96, 0.0934, 200}, {79, 97, 0.1080, 200},
    {79, 98, 0.2060, 200}, {91, 99, 0.2950, 150}, {93, 99, 0.0580, 200},
    {94, 95, 0.0547, 200}, {95, 96, 0.0885, 200}, {97, 99, 0.1790, 200},
    {98, 99, 0.0813, 200},
    // ── Buses 99-117 ───────────────────────────────────────────────────────
    {99,100, 0.1262, 200}, {91,101, 0.0559, 200}, {100,101, 0.1120, 200},
    {99,102, 0.0525, 200}, {99,103, 0.2040, 200}, {102,103, 0.1584, 200},
    {102,104, 0.1625, 200}, {99,105, 0.2290, 200}, {103,104, 0.0378, 200},
    {104,105, 0.0547, 200}, {104,106, 0.1830, 200}, {104,107, 0.0703, 200},
    {105,106, 0.1830, 200}, {107,108, 0.0288, 200}, {102,109, 0.1813, 200},
    {108,109, 0.0762, 200}, {109,110, 0.0755, 200}, {109,111, 0.0640, 200},
    { 16,112, 0.0301, 200}, { 31,112, 0.2030, 150}, { 31,113, 0.0612, 200},
    { 26,114, 0.0741, 200}, {113,114, 0.0104, 200}, { 67,115, 0.00405,300},
    { 11,116, 0.1400, 200}, { 74,117, 0.0481, 200}, { 75,117, 0.0544, 200},
  };
  static_assert(sizeof(branches)/sizeof(branches[0]) == 186,
                "IEEE 118-bus: expected 186 branches");
  for (const auto& bs : branches) {
    Branch b;
    b.from = bs.f; b.to = bs.t; b.reactance = bs.x; b.rating_mw = bs.r;
    inp.branches.push_back(b);
  }

  // ── Loads (99 load buses, ~4242 MW peak aggregate) ────────────────────────
  // Bus (0-indexed), base_mw — MATPOWER case118 PD column (MW)
  struct LoadSpec { int bus; double p_mw; };
  const LoadSpec loads[] = {
    { 1, 20.0}, { 2, 39.0}, { 4,  5.0}, { 5, 52.0}, { 6, 19.0},
    { 7,100.0}, { 8, 28.0}, { 9, 34.0}, {10, 23.0}, {11, 63.0},
    {12, 25.0}, {14, 33.0}, {15, 46.0}, {17, 60.0}, {18, 25.0},
    {19, 14.0}, {20, 14.0}, {21, 12.0}, {22, 18.0}, {23, 97.0},
    {26, 71.0}, {27, 12.0}, {28, 27.0}, {29, 17.0}, {30,  5.0},
    {31, 43.0}, {32, 26.0}, {33, 34.0}, {35, 59.0}, {36, 33.0},
    {38, 31.0}, {39, 41.0}, {40, 37.0}, {41, 10.0}, {42, 14.0},
    {43,  9.0}, {44, 18.0}, {45, 14.0}, {46, 10.0}, {47, 33.0},
    {48, 68.0}, {49, 20.0}, {50, 68.0}, {51, 76.0}, {52, 20.0},
    {53, 13.0}, {54,  8.0}, {55, 22.0}, {56, 31.0}, {57, 43.0},
    {58, 34.0}, {59, 15.0}, {60,  8.0}, {61, 23.0}, {62, 31.0},
    {63, 27.0}, {64, 12.0}, {65, 22.0}, {66, 18.0}, {67,100.0},
    {68, 55.0}, {70, 33.0}, {73, 23.0}, {74, 28.0}, {75, 16.0},
    {76, 16.0}, {77, 45.0}, {78, 11.0}, {79, 28.0}, {81, 44.0},
    {82, 10.0}, {83, 37.0}, {84,  4.0}, {85, 37.0}, {86, 10.0},
    {87, 25.0}, {88, 25.0}, {89, 17.0}, {90, 18.0}, {91, 37.0},
    {92, 56.0}, {93, 18.0}, {94, 28.0}, {95, 12.0}, {96, 35.0},
    {98, 30.0}, {99, 25.0}, {100,37.0}, {101, 8.0}, {102,43.0},
    {103,26.0}, {104,19.0}, {105,23.0}, {106, 8.0}, {107,28.0},
    {108,13.0}, {109,17.0}, {110,15.0},
  };
  for (const auto& ls : loads) {
    Load l; l.bus = ls.bus; l.p_mw = ls.p_mw;
    inp.loads.push_back(l);
  }
  // Load profile for each bus
  for (size_t d = 0; d < inp.loads.size(); ++d)
    inp.profiles.load.push_back(make_load_profile(T, dt, inp.loads[d].p_mw));

  // ── Wind (optional, 4 sites at high-wind buses) ───────────────────────────
  if (with_wind) {
    struct WindSpec { int bus; double pmax; };
    const WindSpec wsites[] = {{5, 500.0}, {23, 400.0}, {67, 600.0}, {84, 350.0}};
    for (const auto& ws : wsites) {
      WindUnit w; w.bus = ws.bus; w.pmax = ws.pmax;
      inp.wind.push_back(w);
      auto prof = make_wind_profile(T, dt);
      for (double& v : prof) v *= ws.pmax;
      inp.profiles.wind.push_back(prof);
    }
    inp.config.M2_renewable_curtail_penalty = 80.0;
  }

  // ── Solar (optional, 3 sites) ─────────────────────────────────────────────
  if (with_solar) {
    struct SolarSpec { int bus; double pmax; };
    const SolarSpec ssites[] = {{23, 300.0}, {60, 250.0}, {99, 200.0}};
    for (const auto& ss : ssites) {
      SolarUnit s; s.bus = ss.bus; s.pmax = ss.pmax;
      inp.solar.push_back(s);
      auto prof = make_solar_profile(T, dt);
      for (double& v : prof) v *= ss.pmax;
      inp.profiles.solar.push_back(prof);
    }
    if (!with_wind)
      inp.config.M2_renewable_curtail_penalty = 80.0;
  }

  // ── Initial status: large baseload units on, peakers off ─────────────────
  // Select units s.t. sum(pmin) < min_load (~1879 MW) AND sum(pmax) > peak_load (~2890 MW)
  //   on_units (0-idx): G05(4) G11(10) G12(11) G29(28) G37(36) G40(39)
  //   sum pmin = 150+140+0+392+477+607 = 1766 MW  < 1879 MW (min load) ✓
  //   sum pmax = 550+320+414+492+577+707 = 3060 MW > 2890 MW (peak load) ✓
  const int ngens = static_cast<int>(inp.generators.size());
  inp.initial_status.commitment.assign(static_cast<size_t>(ngens), 0);
  inp.initial_status.dispatch.assign(static_cast<size_t>(ngens), 0.0);
  const int on_units[] = {4, 10, 11, 28, 36, 39};  // 0-indexed in gens array
  for (int gi : on_units) {
    inp.initial_status.commitment[static_cast<size_t>(gi)] = 1;
    inp.initial_status.dispatch[static_cast<size_t>(gi)] = inp.generators[static_cast<size_t>(gi)].pmin;
  }

  return inp;
}

// ─────────────────────────────────────────────────────────────────────────────
// scuc_input_to_json  — serialise SCUCInput to JSON string
// ─────────────────────────────────────────────────────────────────────────────
std::string scuc_input_to_json(const SCUCInput& inp, int indent) {
  json j;

  // Config
  {
    auto& jc = j["config"];
    jc["num_periods"]               = inp.config.num_periods;
    jc["period_length_hr"]          = inp.config.period_length_hr;
    jc["n_segments"]                = inp.config.n_segments;
    jc["solver"]                    = inp.config.solver;
    jc["allow_fallback"]            = inp.config.allow_fallback;
    jc["mip_gap"]                   = inp.config.mip_gap;
    jc["spinning_reserve_req"]      = inp.config.spinning_reserve_req;
    jc["regulation_up_req"]         = inp.config.regulation_up_req;
    jc["regulation_down_req"]       = inp.config.regulation_down_req;
    jc["neg_reserve_req"]           = inp.config.neg_reserve_req;
    jc["pfr_reserve_req_mw"]        = inp.config.pfr_reserve_req_mw;
    jc["voll"]                      = inp.config.voll;
    jc["vocc"]                      = inp.config.vocc;
    jc["renewable_min_output_coeff"]= inp.config.renewable_min_output_coeff;
    jc["M1_line_slack_penalty"]     = inp.config.M1_line_slack_penalty;
    jc["M2_renewable_curtail_penalty"] = inp.config.M2_renewable_curtail_penalty;
    jc["wheeling_fee_per_mwh"]      = inp.config.wheeling_fee_per_mwh;
    jc["enable_market_cuts"]        = inp.config.enable_market_cuts;
    jc["enable_primal_repair"]      = inp.config.enable_primal_repair;
    jc["enable_benders_decomposition"] = inp.config.enable_benders_decomposition;
    jc["benders_auto_min_variables"] = inp.config.benders_auto_min_variables;
    jc["benders_max_iterations"]    = inp.config.benders_max_iterations;
    jc["benders_cut_tolerance"]     = inp.config.benders_cut_tolerance;
    jc["solve_sced"]                = inp.config.solve_sced;
    jc["solve_lmp"]                 = inp.config.solve_lmp;
    jc["lmp_delta"]                 = inp.config.lmp_delta;
  }

  j["num_buses"] = inp.num_buses;

  // Generators
  for (const auto& g : inp.generators) {
    json jg;
    jg["name"] = g.name; jg["bus"] = g.bus;
    jg["pmin"] = g.pmin; jg["pmax"] = g.pmax;
    jg["ramp_up_mw_min"] = g.ramp_up_mw_min;
    jg["ramp_dn_mw_min"] = g.ramp_dn_mw_min;
    jg["min_up_time_hr"] = g.min_up_time_hr;
    jg["min_dn_time_hr"] = g.min_dn_time_hr;
    jg["must_run"] = g.must_run;
    jg["max_startups"] = g.max_startups;
    jg["max_shutdowns"] = g.max_shutdowns;
    jg["startup_cost"] = g.startup_cost;
    jg["startup_cost_warm"] = g.startup_cost_warm;
    jg["startup_cost_cold"] = g.startup_cost_cold;
    jg["no_load_cost"] = g.no_load_cost;
    jg["spinning_reserve_price"] = g.spinning_reserve_price;
    jg["regulation_up_price"]    = g.regulation_up_price;
    jg["regulation_down_price"]  = g.regulation_down_price;
    jg["pfr_alpha"] = g.pfr_alpha;
    jg["group_id"]  = g.group_id;
    json segs = json::array();
    for (const auto& s : g.bid_segments)
      segs.push_back({{"quantity", s.quantity}, {"price", s.price}});
    jg["bid_segments"] = segs;
    j["generators"].push_back(jg);
  }

  // Branches
  for (const auto& b : inp.branches) {
    json jb;
    jb["from"] = b.from; jb["to"] = b.to;
    jb["reactance"] = b.reactance; jb["rating_mw"] = b.rating_mw;
    jb["in_service"] = b.in_service;
    j["branches"].push_back(jb);
  }

  // Loads
  for (const auto& l : inp.loads) {
    json jl;
    jl["bus"] = l.bus; jl["p_mw"] = l.p_mw;
    j["loads"].push_back(jl);
  }

  // Wind
  for (const auto& w : inp.wind) {
    json jw;
    jw["bus"] = w.bus; jw["pmax"] = w.pmax;
    j["wind"].push_back(jw);
  }

  // Solar
  for (const auto& s : inp.solar) {
    json js;
    js["bus"] = s.bus; js["pmax"] = s.pmax;
    j["solar"].push_back(js);
  }

  // Storage
  for (const auto& s : inp.storage) {
    json js;
    js["bus"] = s.bus;
    js["pmax_charge"] = s.pmax_charge;
    js["pmax_discharge"] = s.pmax_discharge;
    js["pmin_charge"] = s.pmin_charge;
    js["pmin_discharge"] = s.pmin_discharge;
    js["energy_capacity_mwh"] = s.energy_capacity_mwh;
    js["efficiency"] = s.efficiency;
    js["eta_charge"] = s.eta_charge;
    js["eta_discharge"] = s.eta_discharge;
    js["soc_init"] = s.soc_init;
    js["soc_min"] = s.soc_min;
    js["soc_final"] = s.soc_final;
    js["cycle_limit"] = s.cycle_limit;
    js["charge_bid_price"] = s.charge_bid_price;
    js["discharge_bid_price"] = s.discharge_bid_price;
    js["use_binary_indicators"] = s.use_binary_indicators;
    j["storage"].push_back(js);
  }

  // DC lines
  for (const auto& d : inp.dc_lines) {
    json jd;
    jd["from"] = d.from; jd["to"] = d.to;
    jd["pmin"] = d.pmin; jd["pmax"] = d.pmax;
    jd["ramp_up"] = d.ramp_up; jd["ramp_dn"] = d.ramp_dn;
    j["dc_lines"].push_back(jd);
  }

  // Generator groups
  for (const auto& grp : inp.generator_groups) {
    json jgrp;
    jgrp["id"] = grp.id; jgrp["name"] = grp.name;
    jgrp["gen_indices"] = grp.gen_indices;
    jgrp["pmin_t"] = grp.pmin_t;
    jgrp["pmax_t"] = grp.pmax_t;
    jgrp["emin"] = grp.emin;
    jgrp["emax"] = grp.emax;
    j["generator_groups"].push_back(jgrp);
  }

  // Sections
  for (const auto& sec : inp.sections) {
    json jsec;
    jsec["name"] = sec.name;
    jsec["rating_fwd_mw"] = sec.rating_fwd_mw;
    jsec["rating_rev_mw"] = sec.rating_rev_mw;
    json lw = json::array();
    for (const auto& [li, wt] : sec.line_weights)
      lw.push_back(json::array({li, wt}));
    jsec["line_weights"] = lw;
    j["sections"].push_back(jsec);
  }

  // Profiles
  if (!inp.profiles.load.empty())  j["profiles"]["load"]  = inp.profiles.load;
  if (!inp.profiles.wind.empty())  j["profiles"]["wind"]  = inp.profiles.wind;
  if (!inp.profiles.solar.empty()) j["profiles"]["solar"] = inp.profiles.solar;

  // Initial status
  {
    auto& ji = j["initial_status"];
    ji["commitment"]  = inp.initial_status.commitment;
    ji["dispatch"]    = inp.initial_status.dispatch;
    ji["storage_soc"] = inp.initial_status.storage_soc;
    if (!inp.initial_status.time_in_state.empty())
      ji["time_in_state"] = inp.initial_status.time_in_state;
  }

  return indent >= 0 ? j.dump(indent) : j.dump();
}

}  // namespace mipsolvers::scuc
