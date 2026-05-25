/// @file test_resilience_assessment.cpp
/// @brief Unit tests for the distribution resilience assessment module.
///
/// Tests cover:
///  - Basic API (empty / minimal system rejection)
///  - Single-fault radial feeder: shedding and restoration
///  - Fully-connected (no-fault) system: zero shed
///  - Time-stepping: multi-step horizon accumulation
///  - Reconfiguration: tie-switch closes to restore load
///  - MESS dispatch: mobile storage dispatches to isolated island
///  - Demo data injection
///  - summary() string non-empty
///  - Progress callback cancellation
///  - Custom load/RES profiles
///  - Resilience index in [0, 1]
///  - Peak shed non-negative
///  - Fault sequence populated
///  - Legacy field sync (total_ens_mwh == total_shed_mwh)

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <cmath>
#include <string>
#include <vector>

#include "hacdcpf/model/hybrid_power_system.hpp"
#include "hacdcpf/resilience/resilience_assessment.hpp"

using namespace hacdcpf;
using namespace hacdcpf::analysis;
using Approx = Catch::Approx;

// ── Helpers ──────────────────────────────────────────────────────────────────

/// Build a simple 3-bus radial AC system:
///   Bus1 (slack) --- Branch1 (1 km) --- Bus2 --- Branch2 (1 km) --- Bus3
/// Bus2 and Bus3 have 1 MW load each.
static HybridPowerSystem make_radial_3bus() {
  HybridPowerSystem sys;

  ACBus b1, b2, b3;
  b1.index = 1; b1.bus_type = BusType::SLACK; b1.vm_pu = 1.0; b1.in_service = true;
  b2.index = 2; b2.bus_type = BusType::PQ;    b2.pd_mw = 1.0; b2.in_service = true;
  b3.index = 3; b3.bus_type = BusType::PQ;    b3.pd_mw = 1.0; b3.in_service = true;
  sys.ac.buses = {b1, b2, b3};

  ACBranch br1, br2;
  br1.index = 1; br1.from_bus = 1; br1.to_bus = 2; br1.in_service = true;
  br1.r_pu = 0.01; br1.x_pu = 0.01; br1.length_km = 1.0; br1.mttr_hr = 6.0;
  br2.index = 2; br2.from_bus = 2; br2.to_bus = 3; br2.in_service = true;
  br2.r_pu = 0.01; br2.x_pu = 0.01; br2.length_km = 1.0; br2.mttr_hr = 6.0;
  sys.ac.branches = {br1, br2};

  return sys;
}

/// Build a 4-bus ring with one tie switch (branch 4, normally open):
///   Bus1 (slack) -1- Bus2 -2- Bus3 -3- Bus4
///   Bus4 -----4 (tie, open)------- Bus1
/// Bus2, Bus3, Bus4 have 1 MW each.
static HybridPowerSystem make_ring_4bus() {
  HybridPowerSystem sys;

  for (int i = 1; i <= 4; ++i) {
    ACBus b;
    b.index = i;
    b.bus_type = (i == 1) ? BusType::SLACK : BusType::PQ;
    b.vm_pu = 1.0;
    b.in_service = true;
    if (i > 1) b.pd_mw = 1.0;
    sys.ac.buses.push_back(b);
  }

  auto make_br = [](int idx, int f, int t, bool open = false) {
    ACBranch br;
    br.index = idx; br.from_bus = f; br.to_bus = t;
    br.in_service = !open;
    br.r_pu = 0.01; br.x_pu = 0.01; br.length_km = 1.0; br.mttr_hr = 6.0;
    return br;
  };
  sys.ac.branches = {make_br(1, 1, 2), make_br(2, 2, 3), make_br(3, 3, 4), make_br(4, 4, 1, /*open=*/true)};
  return sys;
}

// ── Test cases ────────────────────────────────────────────────────────────────

TEST_CASE("Resilience: empty system returns error", "[resilience]") {
  HybridPowerSystem sys;
  auto r = run_distribution_resilience_assessment(sys);
  CHECK_FALSE(r.feasible);
  CHECK_FALSE(r.status.empty());
}

TEST_CASE("Resilience: invalid options return error", "[resilience]") {
  auto sys = make_radial_3bus();
  DistributionResilienceOptions opts;
  opts.horizon_hours = 0;
  auto r = run_distribution_resilience_assessment(sys, opts);
  CHECK_FALSE(r.feasible);
}

TEST_CASE("Resilience: no-fault system has zero shed", "[resilience]") {
  auto sys = make_radial_3bus();
  DistributionResilienceOptions opts;
  opts.horizon_hours = 4;
  opts.time_step_hr = 1.0;
  opts.default_fault_count = 0;   // no auto-faults
  opts.faults.clear();
  opts.allow_mess_dispatch = false;

  auto r = run_distribution_resilience_assessment(sys, opts);
  REQUIRE(r.feasible);
  CHECK(r.total_shed_mwh == Approx(0.0).margin(1e-9));
  CHECK(r.resilience_index == Approx(1.0).margin(1e-6));
  CHECK(r.peak_shed_mw == Approx(0.0).margin(1e-9));
}

TEST_CASE("Resilience: single fault causes shedding on downstream island", "[resilience]") {
  auto sys = make_radial_3bus();
  DistributionResilienceOptions opts;
  opts.horizon_hours = 6;
  opts.time_step_hr = 1.0;
  opts.allow_reconfiguration = false;
  opts.allow_mess_dispatch = false;

  // Fault branch 1 for the entire horizon.
  DistributionResilienceFault f;
  f.ac_branch_index = 1;
  f.outage_start_hr = 0.0;
  f.repair_duration_hr = 24.0;  // longer than horizon
  opts.faults = {f};
  opts.default_fault_count = 0;

  auto r = run_distribution_resilience_assessment(sys, opts);
  REQUIRE(r.feasible);
  CHECK(r.status == "Completed");
  CHECK(r.total_shed_mwh > 0.0);
  CHECK(r.resilience_index < 1.0);
  CHECK(r.resilience_index >= 0.0);
  CHECK(r.peak_shed_mw > 0.0);
  REQUIRE_FALSE(r.steps.empty());
  for (const auto& s : r.steps) {
    CHECK(s.shed_mw >= 0.0);
    CHECK(s.restoration_ratio >= 0.0);
    CHECK(s.restoration_ratio <= 1.0 + 1e-9);
  }
}

TEST_CASE("Resilience: fault repair with reconfiguration restores load", "[resilience]") {
  auto sys = make_radial_3bus();
  DistributionResilienceOptions opts;
  opts.horizon_hours = 10;
  opts.time_step_hr = 1.0;
  opts.allow_reconfiguration = true;   // needed for greedy re-close after repair
  opts.allow_mess_dispatch = false;
  opts.default_fault_count = 0;

  DistributionResilienceFault f;
  f.ac_branch_index = 1;
  f.outage_start_hr = 0.0;
  f.repair_duration_hr = 5.0;  // repaired at hour 5
  opts.faults = {f};

  auto r = run_distribution_resilience_assessment(sys, opts);
  REQUIRE(r.feasible);
  REQUIRE(r.steps.size() == 10);

  // After repair (step 5+), reconfiguration re-closes the branch: ratio == 1.
  for (int i = 5; i < 10; ++i) {
    CHECK(r.steps[static_cast<size_t>(i)].restoration_ratio == Approx(1.0).margin(1e-6));
  }
}

TEST_CASE("Resilience: reconfiguration closes tie switch to restore island", "[resilience]") {
  auto sys = make_ring_4bus();
  DistributionResilienceOptions opts;
  opts.horizon_hours = 4;
  opts.time_step_hr = 1.0;
  opts.allow_reconfiguration = true;
  opts.allow_mess_dispatch = false;
  opts.default_fault_count = 0;

  // Fault branch 1 (Bus1 -> Bus2), leaving Bus2–4 isolated.
  // Tie switch (branch 4) can restore them.
  DistributionResilienceFault f;
  f.ac_branch_index = 1;
  f.outage_start_hr = 0.0;
  f.repair_duration_hr = 24.0;
  opts.faults = {f};

  auto r = run_distribution_resilience_assessment(sys, opts);
  REQUIRE(r.feasible);
  // With reconfiguration, shed should be zero.
  CHECK(r.total_shed_mwh == Approx(0.0).margin(1e-6));
}

TEST_CASE("Resilience: MESS dispatch reduces shedding on isolated island", "[resilience]") {
  auto sys = make_radial_3bus();

  // Add a MESS at Bus1 (source bus).
  MobileStorage mess;
  mess.index = 1; mess.name = "MESS-1"; mess.bus = 1; mess.in_service = true;
  mess.status = MobileStorageStatus::Stationary;
  mess.pmax_mw = 2.0; mess.p_rated_mw = 2.0; mess.pmin_mw = 0.0;
  mess.e_rated_mwh = 10.0; mess.soc_init = 0.9; mess.soc_min = 0.1; mess.soc_max = 1.0;
  mess.e_mwh = mess.soc_init * mess.e_rated_mwh;
  mess.eta_charge = 0.95; mess.eta_discharge = 0.95;
  mess.e_consumption_mwh_km = 0.01; mess.max_travel_distance_km = 50.0;
  sys.mobile_storage = {mess};

  DistributionResilienceOptions opts;
  opts.horizon_hours = 4;
  opts.time_step_hr = 1.0;
  opts.allow_reconfiguration = false;
  opts.allow_mess_dispatch = true;
  opts.use_electrical_graph_as_transport_proxy = true;
  opts.mess_travel_speed_kmph = 40.0;
  opts.default_fault_count = 0;

  // Fault branch 1: isolates Bus2 and Bus3.
  DistributionResilienceFault f;
  f.ac_branch_index = 1;
  f.outage_start_hr = 0.0;
  f.repair_duration_hr = 24.0;
  opts.faults = {f};

  // Run WITHOUT MESS dispatch for reference.
  opts.allow_mess_dispatch = false;
  auto r_no_mess = run_distribution_resilience_assessment(sys, opts);

  // Run WITH MESS dispatch.
  opts.allow_mess_dispatch = true;
  auto r_mess = run_distribution_resilience_assessment(sys, opts);

  REQUIRE(r_no_mess.feasible);
  REQUIRE(r_mess.feasible);
  // MESS should reduce total shedding (or at worst equal it).
  CHECK(r_mess.total_shed_mwh <= r_no_mess.total_shed_mwh + 1e-6);
}

TEST_CASE("Resilience: demo data injection runs without crash", "[resilience]") {
  auto sys = make_radial_3bus();
  apply_distribution_resilience_demo_data(sys);
  CHECK_FALSE(sys.mobile_storage.empty());

  DistributionResilienceOptions opts;
  opts.horizon_hours = 6;
  opts.time_step_hr = 1.0;
  opts.default_fault_count = 1;
  auto r = run_distribution_resilience_assessment(sys, opts);
  CHECK(r.feasible);
}

TEST_CASE("Resilience: summary() returns non-empty string", "[resilience]") {
  auto sys = make_radial_3bus();
  DistributionResilienceOptions opts;
  opts.horizon_hours = 2;
  opts.time_step_hr = 1.0;
  opts.default_fault_count = 0;
  auto r = run_distribution_resilience_assessment(sys, opts);
  CHECK_FALSE(r.summary().empty());
}

TEST_CASE("Resilience: progress callback can cancel simulation", "[resilience]") {
  auto sys = make_radial_3bus();
  DistributionResilienceOptions opts;
  opts.horizon_hours = 10;
  opts.time_step_hr = 1.0;
  opts.default_fault_count = 0;

  int callback_calls = 0;
  // Cancel after step 3.
  opts.progress_callback = [&](int step, double /*ratio*/, double /*shed*/) -> bool {
    ++callback_calls;
    return step < 3;
  };

  auto r = run_distribution_resilience_assessment(sys, opts);
  CHECK(r.status == "Cancelled");
  CHECK(callback_calls > 0);
  CHECK(r.steps.size() <= 4);  // steps 0..3
}

TEST_CASE("Resilience: custom flat load/RES profiles", "[resilience]") {
  auto sys = make_radial_3bus();
  DistributionResilienceOptions opts;
  opts.horizon_hours = 4;
  opts.time_step_hr = 1.0;
  opts.default_fault_count = 0;
  opts.load_profile = {0.5, 0.5, 0.5, 0.5};
  opts.renewable_profile = {0.0, 0.0, 0.0, 0.0};

  auto r = run_distribution_resilience_assessment(sys, opts);
  REQUIRE(r.feasible);
  for (const auto& s : r.steps) {
    CHECK(s.load_multiplier == Approx(0.5).margin(1e-6));
  }
}

TEST_CASE("Resilience: resilience index is in [0, 1]", "[resilience]") {
  auto sys = make_radial_3bus();
  DistributionResilienceOptions opts;
  opts.horizon_hours = 8;
  opts.time_step_hr = 1.0;
  opts.default_fault_count = 1;
  auto r = run_distribution_resilience_assessment(sys, opts);
  REQUIRE(r.feasible);
  CHECK(r.resilience_index >= 0.0);
  CHECK(r.resilience_index <= 1.0 + 1e-9);
}

TEST_CASE("Resilience: fault sequence populated for auto faults", "[resilience]") {
  auto sys = make_radial_3bus();
  DistributionResilienceOptions opts;
  opts.horizon_hours = 8;
  opts.time_step_hr = 1.0;
  opts.default_fault_count = 1;
  auto r = run_distribution_resilience_assessment(sys, opts);
  REQUIRE(r.feasible);
  CHECK_FALSE(r.fault_sequence.empty());
  for (const auto& fs : r.fault_sequence) {
    CHECK(fs.branch_index > 0);
    CHECK(fs.repair_hr > 0.0);
  }
}

TEST_CASE("Resilience: legacy fields synced to new fields", "[resilience]") {
  auto sys = make_radial_3bus();
  DistributionResilienceOptions opts;
  opts.horizon_hours = 4;
  opts.time_step_hr = 1.0;
  opts.default_fault_count = 0;
  auto r = run_distribution_resilience_assessment(sys, opts);
  REQUIRE(r.feasible);
  CHECK(r.total_ens_mwh == Approx(r.total_shed_mwh).margin(1e-12));
  CHECK(r.max_curtailment_mw == Approx(r.peak_shed_mw).margin(1e-12));
  CHECK(r.completed == r.feasible);
}

TEST_CASE("Resilience: step count matches horizon/timestep", "[resilience]") {
  auto sys = make_radial_3bus();
  DistributionResilienceOptions opts;
  opts.horizon_hours = 12;
  opts.time_step_hr = 2.0;
  opts.default_fault_count = 0;
  auto r = run_distribution_resilience_assessment(sys, opts);
  REQUIRE(r.feasible);
  CHECK(r.steps.size() == 6u);
}

TEST_CASE("Resilience: MIP solver returns feasible on zero-fault system", "[resilience]") {
  auto sys = make_radial_3bus();
  DistributionResilienceOptions opts;
  opts.horizon_hours = 4;
  opts.time_step_hr = 1.0;
  opts.default_fault_count = 0;
  auto r = run_distribution_resilience_mip_assessment(sys, opts);
  CHECK(r.feasible);
}

TEST_CASE("Resilience: peak shed non-negative", "[resilience]") {
  auto sys = make_radial_3bus();
  DistributionResilienceOptions opts;
  opts.horizon_hours = 6;
  opts.time_step_hr = 1.0;
  opts.default_fault_count = 1;
  auto r = run_distribution_resilience_assessment(sys, opts);
  CHECK(r.peak_shed_mw >= 0.0);
  CHECK(r.total_shed_mwh >= 0.0);
  CHECK(r.total_served_mwh >= 0.0);
  CHECK(r.total_demand_mwh >= 0.0);
  CHECK(r.total_served_mwh <= r.total_demand_mwh + 1e-6);
}
