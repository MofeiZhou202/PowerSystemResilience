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
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#ifdef _WIN32
#include <io.h>
#else
#include <unistd.h>
#endif

#ifndef STDOUT_FILENO
#define STDOUT_FILENO 1
#endif
#ifndef STDERR_FILENO
#define STDERR_FILENO 2
#endif

#ifdef _WIN32
#define HACDCPF_DUP _dup
#define HACDCPF_DUP2 _dup2
#define HACDCPF_CLOSE _close
#define HACDCPF_FILENO _fileno
#else
#define HACDCPF_DUP dup
#define HACDCPF_DUP2 dup2
#define HACDCPF_CLOSE close
#define HACDCPF_FILENO fileno
#endif

#include "hacdcpf/model/hybrid_power_system.hpp"
#include "hacdcpf/resilience/resilience_assessment.hpp"
#include "hacdcpf/reliability/reliability_assessment.hpp"

using namespace hacdcpf;
using namespace hacdcpf::analysis;
using Approx = Catch::Approx;

// ── Helpers ──────────────────────────────────────────────────────────────────

class ScopedStdStreamCapture {
public:
  ScopedStdStreamCapture()
      : stdout_file_(std::tmpfile()), stderr_file_(std::tmpfile()) {
    if (stdout_file_ == nullptr || stderr_file_ == nullptr) {
      throw std::runtime_error("failed to create temporary stream capture files");
    }
    flush_all();
    stdout_saved_ = HACDCPF_DUP(STDOUT_FILENO);
    stderr_saved_ = HACDCPF_DUP(STDERR_FILENO);
    if (stdout_saved_ < 0 || stderr_saved_ < 0) {
      throw std::runtime_error("failed to duplicate stdout/stderr");
    }
    if (HACDCPF_DUP2(HACDCPF_FILENO(stdout_file_), STDOUT_FILENO) < 0 ||
        HACDCPF_DUP2(HACDCPF_FILENO(stderr_file_), STDERR_FILENO) < 0) {
      throw std::runtime_error("failed to redirect stdout/stderr");
    }
    active_ = true;
  }

  ~ScopedStdStreamCapture() {
    restore();
    if (stdout_file_ != nullptr) std::fclose(stdout_file_);
    if (stderr_file_ != nullptr) std::fclose(stderr_file_);
  }

  ScopedStdStreamCapture(const ScopedStdStreamCapture&) = delete;
  ScopedStdStreamCapture& operator=(const ScopedStdStreamCapture&) = delete;

  void restore() {
    if (!active_) return;
    flush_all();
    HACDCPF_DUP2(stdout_saved_, STDOUT_FILENO);
    HACDCPF_DUP2(stderr_saved_, STDERR_FILENO);
    HACDCPF_CLOSE(stdout_saved_);
    HACDCPF_CLOSE(stderr_saved_);
    stdout_saved_ = -1;
    stderr_saved_ = -1;
    active_ = false;
  }

  std::string stdout_text() const { return read_stream(stdout_file_); }
  std::string stderr_text() const { return read_stream(stderr_file_); }

private:
  static void flush_all() {
    std::cout.flush();
    std::cerr.flush();
    std::fflush(nullptr);
  }

  static std::string read_stream(std::FILE* stream) {
    std::fflush(stream);
    std::rewind(stream);
    std::string text;
    char buffer[4096];
    while (const std::size_t read_count =
               std::fread(buffer, 1, sizeof(buffer), stream)) {
      text.append(buffer, read_count);
    }
    return text;
  }

  std::FILE* stdout_file_{nullptr};
  std::FILE* stderr_file_{nullptr};
  int stdout_saved_{-1};
  int stderr_saved_{-1};
  bool active_{false};
};

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

static HybridPowerSystem make_single_bus_shortage() {
  HybridPowerSystem sys;

  ACBus b;
  b.index = 1;
  b.bus_type = BusType::SLACK;
  b.pd_mw = 2.0;
  b.vm_pu = 1.0;
  b.in_service = true;
  sys.ac.buses = {b};

  Generator g;
  g.index = 1;
  g.bus = 1;
  g.in_service = true;
  g.is_slack = true;
  g.pmax_mw = 1.0;
  g.pmin_mw = 0.0;
  g.qmax_mvar = 10.0;
  g.qmin_mvar = -10.0;
  g.cost_c1 = 1.0;
  sys.ac.generators = {g};

  return sys;
}

static bool has_mess_travel_evidence(const MESSStateStep& ms) {
  return ms.status == "InTransit" || ms.target_bus != ms.bus || ms.remaining_travel_hr > 1e-9;
}

static HybridPowerSystem make_hybrid_no_switch_bus_load_case() {
  HybridPowerSystem sys;

  ACBus ac1, ac2, ac3;
  ac1.index = 1; ac1.bus_type = BusType::SLACK; ac1.vm_pu = 1.0; ac1.in_service = true;
  ac2.index = 2; ac2.bus_type = BusType::PQ; ac2.pd_mw = 1.2; ac2.in_service = true;
  ac3.index = 3; ac3.bus_type = BusType::PQ; ac3.pd_mw = 0.8; ac3.in_service = true;
  sys.ac.buses = {ac1, ac2, ac3};

  ACBranch acb1, acb2;
  acb1.index = 1; acb1.from_bus = 1; acb1.to_bus = 2; acb1.in_service = true; acb1.rate_a_mva = 10.0; acb1.mttr_hr = 6.0;
  acb2.index = 2; acb2.from_bus = 2; acb2.to_bus = 3; acb2.in_service = true; acb2.rate_a_mva = 10.0; acb2.mttr_hr = 6.0;
  sys.ac.branches = {acb1, acb2};

  DCBus dc1, dc2;
  dc1.index = 1; dc1.bus_type = DCBusType::DC_P; dc1.in_service = true;
  dc2.index = 2; dc2.bus_type = DCBusType::DC_P; dc2.pd_mw = 0.4; dc2.in_service = true;
  sys.dc.buses = {dc1, dc2};

  DCBranch dcb;
  dcb.index = 1; dcb.from_bus = 1; dcb.to_bus = 2; dcb.in_service = true; dcb.rate_a_mva = 5.0; dcb.mttr_hours = 6.0;
  sys.dc.branches = {dcb};

  VSCConverter vsc;
  vsc.index = 1; vsc.bus_ac = 2; vsc.bus_dc = 1; vsc.in_service = true; vsc.pmax_mw = 5.0;
  sys.vsc_converters = {vsc};

  return sys;
}

static HybridPowerSystem make_hybrid_transfer_component_case() {
  auto sys = make_hybrid_no_switch_bus_load_case();

  DCBus dc3;
  dc3.index = 3;
  dc3.bus_type = DCBusType::DC_P;
  dc3.in_service = true;
  sys.dc.buses.push_back(dc3);

  DCLoad dc_load;
  dc_load.index = 10;
  dc_load.bus = 3;
  dc_load.p_mw = 0.3;
  dc_load.priority = LoadPriority::Critical;
  sys.dc.loads.push_back(dc_load);

  DCDCConverter dcdc;
  dcdc.index = 20;
  dcdc.bus_in = 2;
  dcdc.bus_out = 3;
  dcdc.pmax_mw = 2.0;
  dcdc.pmin_mw = -2.0;
  dcdc.sn_mva = 2.0;
  sys.dc.dcdc_converters.push_back(dcdc);

  DCStorage storage;
  storage.index = 30;
  storage.bus = 3;
  storage.pmax_mw = 0.2;
  storage.p_rated_mw = 0.2;
  storage.e_rated_mwh = 0.5;
  storage.e_mwh = 0.4;
  storage.soc_min = 0.1;
  storage.eta_discharge = 0.95;
  sys.dc.dc_storage.push_back(storage);

  return sys;
}

static DistributionResilienceOptions make_hybrid_dc_fault_options(
    DistributionResilienceMIPSolver solver) {
  DistributionResilienceFault fault;
  fault.branch_kind = ResilienceBranchKind::DC;
  fault.branch_index = 1;
  fault.outage_start_hr = 0.0;
  fault.repair_duration_hr = 1.0;
  fault.name = "dc_feeder_fault";

  DistributionResilienceOptions opts;
  opts.horizon_hours = 2;
  opts.time_step_hr = 1.0;
  opts.default_fault_count = 0;
  opts.faults = {fault};
  opts.mip.solver = solver;
  opts.mip.max_time_s = 20;
  return opts;
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
  INFO(r.status);
  INFO(r.model_stats.solver_status);
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
  opts.allow_reconfiguration = true;
  opts.allow_mess_dispatch = false;

  // Fault branch 1 for the entire horizon.
  DistributionResilienceFault f;
  f.ac_branch_index = 1;
  f.outage_start_hr = 0.0;
  f.repair_duration_hr = 24.0;  // longer than horizon
  opts.faults = {f};
  opts.default_fault_count = 0;

  auto r = run_distribution_resilience_assessment(sys, opts);
  INFO(r.status);
  INFO(r.model_stats.solver_status);
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
  INFO(r.status);
  INFO(r.model_stats.solver_status);
  REQUIRE(r.feasible);
  REQUIRE(r.steps.size() == 10);

  // After repair (step 5+), reconfiguration re-closes the branch: ratio == 1.
  for (int i = 5; i < 10; ++i) {
    CHECK(r.steps[static_cast<size_t>(i)].restoration_ratio == Approx(1.0).margin(1e-6));
  }
}

TEST_CASE("Resilience stage MILP: no-switch bus-load case uses virtual branch breakers", "[resilience]") {
  auto sys = make_hybrid_no_switch_bus_load_case();
  DistributionResilienceOptions opts;
  opts.model = DistributionResilienceModel::RAStyleStageMILP;
  opts.use_ra_style_stage_milp = true;
  opts.enable_disaster_stages = true;
  opts.horizon_hours = 4;
  opts.time_step_hr = 1.0;
  opts.allow_mess_dispatch = false;
  opts.default_fault_count = 0;
  opts.require_switch_for_nonfault_branch_operation = true;
  opts.allow_branch_operation_without_switch = false;

  DistributionResilienceFault f;
  f.branch_kind = ResilienceBranchKind::AC;
  f.branch_index = 1;
  f.ac_branch_index = 1;
  f.outage_start_hr = 1.0;
  f.repair_duration_hr = 2.0;
  opts.faults = {f};

  auto r = run_distribution_resilience_assessment(sys, opts);
  INFO(r.status);
  INFO(r.model_stats.solver_status);
  REQUIRE(r.feasible);
  CHECK(r.model == DistributionResilienceModel::RAStyleStageMILP);
  REQUIRE(r.steps.size() >= 2);
  CHECK(r.model_stats.formulation_notes.find("virtual operable AC/DC branch breakers") != std::string::npos);

  for (const auto& step : r.steps) {
    REQUIRE_FALSE(step.bus_supply_index.empty());
    REQUIRE(step.bus_supply_demand_mw.size() == step.bus_supply_index.size());
    CHECK(std::any_of(step.bus_supply_demand_mw.begin(), step.bus_supply_demand_mw.end(),
                      [](double demand) { return demand > 1e-9; }));
  }
  const auto& isolation_step = r.steps[1];
  REQUIRE(isolation_step.disaster_stage == "DisasterIsolation");
  CHECK(isolation_step.active_faults == 1);
  CHECK(isolation_step.switch_actions == 1);
  CHECK(isolation_step.isolation_switch_actions == 1);
  CHECK(isolation_step.open_ac_branch_ids == std::vector<int>{1});
  CHECK(isolation_step.open_switch_ids == std::vector<int>{-100001});
}

TEST_CASE("Resilience stage MILP: greedy repair restores repaired line", "[resilience]") {
  auto sys = make_radial_3bus();
  DistributionResilienceOptions opts;
  opts.model = DistributionResilienceModel::RAStyleStageMILP;
  opts.use_ra_style_stage_milp = true;
  opts.enable_disaster_stages = true;
  opts.horizon_hours = 6;
  opts.time_step_hr = 1.0;
  opts.allow_reconfiguration = true;
  opts.allow_mess_dispatch = false;
  opts.default_fault_count = 0;

  DistributionResilienceFault f;
  f.ac_branch_index = 1;
  f.outage_start_hr = 0.0;
  f.repair_duration_hr = 3.0;
  opts.faults = {f};

  auto r = run_distribution_resilience_assessment(sys, opts);
  INFO(r.status);
  INFO(r.model_stats.solver_status);
  REQUIRE(r.feasible);
  REQUIRE(r.steps.size() == 6);
  REQUIRE(r.fault_sequence.size() == 1);

  CHECK(r.fault_sequence[0].repair_hr == Approx(3.0).margin(1e-6));
  CHECK(r.steps[0].restoration_ratio < 1.0);
  CHECK(r.steps[3].active_faults == 0);
  CHECK(r.steps[3].repaired_faults == 1);
  for (int i = 3; i < 6; ++i) {
    CHECK(r.steps[static_cast<size_t>(i)].disaster_stage == "Normal");
    CHECK(r.steps[static_cast<size_t>(i)].restoration_ratio == Approx(1.0).margin(1e-6));
  }
}

TEST_CASE("Resilience stage MILP: custom scenario profiles are sampled", "[resilience]") {
  auto sys = make_radial_3bus();
  DistributionResilienceOptions opts;
  opts.model = DistributionResilienceModel::RAStyleStageMILP;
  opts.use_ra_style_stage_milp = true;
  opts.enable_disaster_stages = true;
  opts.horizon_hours = 4;
  opts.time_step_hr = 1.0;
  opts.default_fault_count = 0;
  opts.allow_mess_dispatch = false;
  opts.load_profile = {0.25, 0.50, 0.75, 1.00};
  opts.renewable_profile = {0.10, 0.20, 0.30, 0.40};
  opts.pv_profile = {0.90, 0.80, 0.70, 0.60};
  opts.wind_profile = {0.40, 0.50, 0.60, 0.70};

  auto r = run_distribution_resilience_assessment(sys, opts);
  REQUIRE(r.feasible);
  REQUIRE(r.steps.size() == 4);
  for (size_t i = 0; i < r.steps.size(); ++i) {
    CHECK(r.steps[i].load_multiplier == Approx(opts.load_profile[i]).margin(1e-9));
    CHECK(r.steps[i].res_multiplier == Approx(opts.renewable_profile[i]).margin(1e-9));
    CHECK(r.steps[i].pv_multiplier == Approx(opts.pv_profile[i]).margin(1e-9));
    CHECK(r.steps[i].wind_multiplier == Approx(opts.wind_profile[i]).margin(1e-9));
    CHECK(r.steps[i].pv_multiplier != Approx(r.steps[i].wind_multiplier).margin(1e-9));
  }
}

TEST_CASE("Resilience stage MILP: split profiles fall back to aggregate renewable profile", "[resilience]") {
  auto sys = make_radial_3bus();
  DistributionResilienceOptions opts;
  opts.model = DistributionResilienceModel::RAStyleStageMILP;
  opts.use_ra_style_stage_milp = true;
  opts.enable_disaster_stages = true;
  opts.horizon_hours = 3;
  opts.time_step_hr = 1.0;
  opts.default_fault_count = 0;
  opts.allow_mess_dispatch = false;
  opts.renewable_profile = {0.15, 0.25, 0.35};

  auto r = run_distribution_resilience_assessment(sys, opts);
  REQUIRE(r.feasible);
  REQUIRE(r.steps.size() == 3);
  for (size_t i = 0; i < r.steps.size(); ++i) {
    CHECK(r.steps[i].res_multiplier == Approx(opts.renewable_profile[i]).margin(1e-9));
    CHECK(r.steps[i].pv_multiplier == Approx(opts.renewable_profile[i]).margin(1e-9));
    CHECK(r.steps[i].wind_multiplier == Approx(opts.renewable_profile[i]).margin(1e-9));
  }
}

TEST_CASE("Resilience stage MILP: final repair restores initial AC tie state", "[resilience]") {
  auto sys = make_ring_4bus();
  Switch sw;
  sw.index = 1;
  sw.name = "Tie 4";
  sw.bus_from = 4;
  sw.bus_to = 1;
  sw.in_service = true;
  sw.closed = false;
  sw.is_remote = true;
  sys.ac.switches = {sw};

  DistributionResilienceOptions opts;
  opts.model = DistributionResilienceModel::RAStyleStageMILP;
  opts.use_ra_style_stage_milp = true;
  opts.enable_disaster_stages = true;
  opts.horizon_hours = 5;
  opts.time_step_hr = 1.0;
  opts.allow_mess_dispatch = false;
  opts.default_fault_count = 0;
  opts.require_switch_for_nonfault_branch_operation = true;
  opts.allow_branch_operation_without_switch = false;
  opts.allow_stage2_close_ties = true;

  DistributionResilienceFault f;
  f.ac_branch_index = 1;
  f.outage_start_hr = 0.0;
  f.repair_duration_hr = 3.0;
  opts.faults = {f};

  auto r = run_distribution_resilience_assessment(sys, opts);
  INFO(r.status);
  INFO(r.model_stats.solver_status);
  REQUIRE(r.feasible);
  CHECK(r.model_stats.formulation_notes.find("virtual operable AC/DC branch breakers") == std::string::npos);
  REQUIRE(r.steps.size() == 5);
  REQUIRE(r.steps[3].disaster_stage == "Normal");
  CHECK(r.steps[3].active_faults == 0);
  CHECK(r.steps[3].repaired_faults == 1);
  CHECK(r.steps[3].closed_tie_branch_ids.empty());
  CHECK(std::find(r.steps[3].open_ac_branch_ids.begin(), r.steps[3].open_ac_branch_ids.end(), 4) != r.steps[3].open_ac_branch_ids.end());
  CHECK(std::find(r.steps[3].open_switch_ids.begin(), r.steps[3].open_switch_ids.end(), 1) != r.steps[3].open_switch_ids.end());
  CHECK(r.steps[3].restoration_ratio == Approx(1.0).margin(1e-6));
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

TEST_CASE("Resilience stage MILP: MESS dispatch emits traces and energy", "[resilience]") {
  auto sys = make_radial_3bus();

  MobileStorage mess;
  mess.index = 1;
  mess.name = "MESS-1";
  mess.bus = 2;
  mess.in_service = true;
  mess.status = MobileStorageStatus::Stationary;
  mess.pmax_mw = 1.0;
  mess.p_rated_mw = 1.0;
  mess.e_rated_mwh = 4.0;
  mess.soc_init = 1.0;
  mess.soc_min = 0.1;
  mess.soc_max = 1.0;
  mess.e_mwh = 4.0;
  mess.eta_discharge = 1.0;
  sys.mobile_storage = {mess};

  DistributionResilienceOptions opts;
  opts.model = DistributionResilienceModel::RAStyleStageMILP;
  opts.use_ra_style_stage_milp = true;
  opts.enable_disaster_stages = true;
  opts.horizon_hours = 3;
  opts.time_step_hr = 1.0;
  opts.allow_mess_dispatch = true;
  opts.use_strict_mip_for_mess = false;
  opts.default_fault_count = 0;

  DistributionResilienceFault f;
  f.ac_branch_index = 1;
  f.outage_start_hr = 0.0;
  f.repair_duration_hr = 24.0;
  opts.faults = {f};

  auto r = run_distribution_resilience_assessment(sys, opts);
  INFO(r.status);
  INFO(r.model_stats.solver_status);
  REQUIRE(r.feasible);
  REQUIRE_FALSE(r.steps.empty());
  CHECK(r.mess_energy_delivered_mwh > 0.0);
  bool saw_dispatch = false;
  bool saw_energy_drop = false;
  for (const auto& step : r.steps) {
    REQUIRE_FALSE(step.mess_states.empty());
    if (step.mess_states[0].dispatch_mw > 0.0) saw_dispatch = true;
    if (step.mess_states[0].energy_mwh < mess.e_mwh - 1e-9) saw_energy_drop = true;
  }
  CHECK(saw_dispatch);
  CHECK(saw_energy_drop);
}

TEST_CASE("Resilience stage MILP: strict MIP MESS routing can move and dispatch", "[resilience]") {
  auto sys = make_radial_3bus();

  MobileStorage mess;
  mess.index = 7;
  mess.name = "MESS-7";
  mess.bus = 1;
  mess.target_bus = 1;
  mess.in_service = true;
  mess.status = MobileStorageStatus::Stationary;
  mess.pmax_mw = 1.0;
  mess.p_rated_mw = 1.0;
  mess.e_rated_mwh = 8.0;
  mess.soc_init = 1.0;
  mess.soc_min = 0.1;
  mess.soc_max = 1.0;
  mess.e_mwh = 8.0;
  mess.eta_discharge = 1.0;
  mess.e_consumption_mwh_km = 0.01;
  mess.max_travel_distance_km = 10.0;
  sys.mobile_storage = {mess};

  DistributionResilienceOptions opts;
  opts.model = DistributionResilienceModel::RAStyleStageMILP;
  opts.use_ra_style_stage_milp = true;
  opts.enable_disaster_stages = true;
  opts.horizon_hours = 4;
  opts.time_step_hr = 1.0;
  opts.allow_mess_dispatch = true;
  opts.use_strict_mip_for_mess = true;
  opts.fallback_to_stage_mess_dispatch = false;
  opts.default_fault_count = 0;
  opts.mip.solver = DistributionResilienceMIPSolver::Native;
  opts.mip.max_time_s = 20;
  opts.mip.max_nodes = 20000;

  DistributionResilienceFault f;
  f.ac_branch_index = 1;
  f.branch_kind = ResilienceBranchKind::AC;
  f.branch_index = 1;
  f.outage_start_hr = 0.0;
  f.repair_duration_hr = 24.0;
  opts.faults = {f};

  auto r = run_distribution_resilience_assessment(sys, opts);
  INFO(r.status);
  INFO(r.model_stats.solver_status);
  REQUIRE(r.feasible);
  CHECK(r.mess_dispatch_model == "ra_residual_mess_milp");
  bool saw_travel = false;
  bool saw_dispatch = false;
  bool saw_energy_drop = false;
  bool have_prev = false;
  MESSStateStep prev_ms;
  for (const auto& step : r.steps) {
    REQUIRE_FALSE(step.mess_states.empty());
    const auto& ms = step.mess_states.front();
    const bool travel_evidence = has_mess_travel_evidence(ms);
    if (travel_evidence) saw_travel = true;
    if (ms.dispatch_mw > 0.0) saw_dispatch = true;
    if (ms.energy_mwh < mess.e_mwh - 1e-9) saw_energy_drop = true;
    if (have_prev) {
      if (ms.bus != prev_ms.bus) {
        CHECK((has_mess_travel_evidence(prev_ms) || travel_evidence));
      }
      if (ms.energy_mwh < prev_ms.energy_mwh - 1e-9 && prev_ms.dispatch_mw <= 1e-9) {
        CHECK(has_mess_travel_evidence(prev_ms));
      }
    }
    prev_ms = ms;
    have_prev = true;
  }
  CHECK(saw_travel);
  CHECK(saw_dispatch);
  CHECK(saw_energy_drop);
  CHECK(r.mess_travel_distance_km > 0.0);
  CHECK(r.mess_energy_delivered_mwh > 0.0);
}

TEST_CASE("Resilience stage MILP: no-fault MESS does not move without RA residual shed", "[resilience]") {
  auto sys = make_radial_3bus();

  MobileStorage mess;
  mess.index = 9;
  mess.name = "MESS-9";
  mess.bus = 1;
  mess.target_bus = 1;
  mess.in_service = true;
  mess.status = MobileStorageStatus::Stationary;
  mess.pmax_mw = 1.0;
  mess.p_rated_mw = 1.0;
  mess.e_rated_mwh = 4.0;
  mess.soc_init = 1.0;
  mess.soc_min = 0.1;
  mess.soc_max = 1.0;
  mess.e_mwh = 4.0;
  mess.eta_discharge = 1.0;
  mess.e_consumption_mwh_km = 0.01;
  mess.max_travel_distance_km = 10.0;
  sys.mobile_storage = {mess};

  DistributionResilienceOptions opts;
  opts.model = DistributionResilienceModel::RAStyleStageMILP;
  opts.use_ra_style_stage_milp = true;
  opts.enable_disaster_stages = true;
  opts.horizon_hours = 4;
  opts.time_step_hr = 1.0;
  opts.allow_mess_dispatch = true;
  opts.use_strict_mip_for_mess = true;
  opts.fallback_to_stage_mess_dispatch = false;
  opts.default_fault_count = 0;
  opts.faults.clear();
  opts.mip.solver = DistributionResilienceMIPSolver::Native;
  opts.mip.max_time_s = 20;
  opts.mip.max_nodes = 20000;

  auto r = run_distribution_resilience_assessment(sys, opts);
  REQUIRE(r.feasible);
  CHECK(r.mess_dispatch_model == "ra_residual_mess_milp");
  CHECK(r.total_shed_mwh == Approx(0.0).margin(1e-9));
  CHECK(r.mess_travel_distance_km == Approx(0.0).margin(1e-9));
  CHECK(r.mess_energy_delivered_mwh == Approx(0.0).margin(1e-9));
  for (const auto& step : r.steps) {
    REQUIRE_FALSE(step.mess_states.empty());
    const auto& ms = step.mess_states.front();
    CHECK(ms.bus == mess.bus);
    CHECK(ms.target_bus == mess.bus);
    CHECK(ms.status != "InTransit");
    CHECK(ms.remaining_travel_hr == Approx(0.0).margin(1e-9));
    CHECK(ms.dispatch_mw == Approx(0.0).margin(1e-9));
    CHECK(ms.energy_mwh == Approx(mess.e_mwh).margin(1e-9));
  }
}

TEST_CASE("Resilience strict MIP: no-fault MESS does not move without benefit", "[resilience]") {
  auto sys = make_radial_3bus();

  MobileStorage mess;
  mess.index = 8;
  mess.name = "MESS-8";
  mess.bus = 1;
  mess.target_bus = 1;
  mess.in_service = true;
  mess.status = MobileStorageStatus::Stationary;
  mess.pmax_mw = 1.0;
  mess.p_rated_mw = 1.0;
  mess.e_rated_mwh = 4.0;
  mess.soc_init = 1.0;
  mess.soc_min = 0.1;
  mess.soc_max = 1.0;
  mess.e_mwh = 4.0;
  mess.eta_discharge = 1.0;
  mess.e_consumption_mwh_km = 0.01;
  mess.max_travel_distance_km = 10.0;
  sys.mobile_storage = {mess};

  DistributionResilienceOptions opts;
  opts.model = DistributionResilienceModel::MultiPeriodMIPLinDistFlow;
  opts.horizon_hours = 4;
  opts.time_step_hr = 1.0;
  opts.allow_mess_dispatch = true;
  opts.default_fault_count = 0;
  opts.faults.clear();
  opts.mip.solver = DistributionResilienceMIPSolver::Native;
  opts.mip.max_time_s = 20;
  opts.mip.max_nodes = 20000;

  auto r = run_distribution_resilience_mip_assessment(sys, opts);
  REQUIRE(r.feasible);
  CHECK(r.total_shed_mwh == Approx(0.0).margin(1e-9));
  CHECK(r.mess_travel_distance_km == Approx(0.0).margin(1e-9));
  CHECK(r.mess_energy_delivered_mwh == Approx(0.0).margin(1e-9));

  for (const auto& step : r.steps) {
    REQUIRE_FALSE(step.mess_states.empty());
    const auto& ms = step.mess_states.front();
    CHECK(ms.bus == mess.bus);
    CHECK(ms.target_bus == mess.bus);
    CHECK(ms.status != "InTransit");
    CHECK(ms.remaining_travel_hr == Approx(0.0).margin(1e-9));
    CHECK(ms.dispatch_mw == Approx(0.0).margin(1e-9));
    CHECK(ms.energy_mwh == Approx(mess.e_mwh).margin(1e-9));
  }
}

TEST_CASE("Resilience strict MIP honors external MESS availability",
          "[resilience][mess]") {
  auto sys = make_radial_3bus();

  MobileStorage mess;
  mess.index = 18;
  mess.name = "Externally routed MESS";
  mess.bus = 2;
  mess.target_bus = 2;
  mess.in_service = true;
  mess.pmax_mw = 1.0;
  mess.p_rated_mw = 1.0;
  mess.e_rated_mwh = 4.0;
  mess.e_mwh = 4.0;
  mess.soc_init = 1.0;
  mess.soc_min = 0.0;
  mess.soc_max = 1.0;
  mess.eta_discharge = 1.0;
  mess.grid_forming = true;
  sys.mobile_storage = {mess};

  DistributionResilienceOptions opts;
  opts.model = DistributionResilienceModel::MultiPeriodMIPLinDistFlow;
  opts.horizon_hours = 4;
  opts.time_step_hr = 1.0;
  opts.allow_reconfiguration = false;
  opts.allow_mess_dispatch = true;
  opts.default_fault_count = 0;
  opts.faults = {{1, 2.0, 2.0, "upstream outage"}};
  opts.mobile_storage_available_from_hr[mess.index] = 2.0;
  opts.mip.solver = DistributionResilienceMIPSolver::Native;
  opts.mip.max_nodes = 20000;
  opts.mip.max_time_s = 20;

  const auto result = run_distribution_resilience_mip_assessment(sys, opts);
  INFO(result.status);
  REQUIRE(result.feasible);
  REQUIRE(result.steps.size() == 4u);
  REQUIRE(result.steps[0].mess_states.size() == 1u);
  CHECK(result.steps[0].mess_states[0].dispatch_mw == Approx(0.0).margin(1e-9));
  CHECK(result.steps[1].mess_states[0].dispatch_mw == Approx(0.0).margin(1e-9));
  CHECK(result.steps[2].mess_states[0].dispatch_mw > 0.0);
}

TEST_CASE("Resilience sequential model honors external MESS availability",
          "[resilience][mess]") {
  auto sys = make_radial_3bus();
  MobileStorage mess;
  mess.index = 19;
  mess.bus = 2;
  mess.target_bus = 2;
  mess.in_service = true;
  mess.pmax_mw = 1.0;
  mess.p_rated_mw = 1.0;
  mess.e_rated_mwh = 4.0;
  mess.e_mwh = 4.0;
  mess.soc_init = 1.0;
  mess.soc_min = 0.0;
  mess.soc_max = 1.0;
  mess.eta_discharge = 1.0;
  mess.max_travel_distance_km = 1e-6;
  sys.mobile_storage = {mess};

  DistributionResilienceOptions opts;
  opts.model = DistributionResilienceModel::HeuristicSequential;
  opts.horizon_hours = 4;
  opts.time_step_hr = 1.0;
  opts.default_fault_count = 0;
  opts.faults = {{1, 0.0, 4.0, "upstream outage"}};
  opts.mobile_storage_available_from_hr[mess.index] = 2.0;

  const auto result = run_distribution_resilience_assessment(sys, opts);
  REQUIRE(result.feasible);
  REQUIRE(result.steps.size() == 4u);
  REQUIRE(result.steps[0].mess_states.size() == 1u);
  CHECK(result.steps[0].mess_states[0].status == "AwaitingArrival");
  CHECK(result.steps[1].mess_states[0].dispatch_mw == Approx(0.0).margin(1e-9));
  CHECK(result.steps[2].mess_states[0].dispatch_mw > 0.0);
  CHECK(result.steps[0].shed_mw > result.steps[2].shed_mw);
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

  ScopedStdStreamCapture capture;
  const auto r = run_distribution_resilience_mip_assessment(sys, opts);
  capture.restore();
  const std::string stdout_text = capture.stdout_text();
  const std::string stderr_text = capture.stderr_text();

  CHECK(r.feasible);
  CHECK(r.model == DistributionResilienceModel::MultiPeriodMIPLinDistFlow);
  CHECK(r.model_stats.model_scope == "hybrid-acdc-restoration-milp");
  CHECK_FALSE(r.model_stats.validity.dc_network_modelled);
  CHECK_FALSE(r.model_stats.validity.vsc_dispatch_modelled);
  CHECK(r.model_stats.validity.ac_branch_flow_limits_enforced);
  CHECK(r.model_stats.validity.lindistflow_voltage_envelope_enforced);
  CHECK(r.model_stats.validity.radial_topology_enforced);
  CHECK(stdout_text.empty());
  CHECK(stderr_text.empty());
}

TEST_CASE("Resilience: strict MIP models hybrid transfer components and DC faults",
          "[resilience][mip][hybrid]") {
  const auto sys = make_hybrid_transfer_component_case();
  const auto opts =
      make_hybrid_dc_fault_options(DistributionResilienceMIPSolver::HiGHS);

  const auto r = run_distribution_resilience_mip_assessment(sys, opts);
  INFO(r.status);
  INFO(r.model_stats.solver_status);
  REQUIRE(r.feasible);
  REQUIRE(r.steps.size() == 2);
  REQUIRE(r.fault_sequence.size() == 1);
  CHECK(r.fault_sequence.front().branch_kind == ResilienceBranchKind::DC);
  CHECK(r.model_stats.model_scope == "hybrid-acdc-restoration-milp");
  CHECK(r.model_stats.validity.canonical_projection_used);
  CHECK(r.model_stats.validity.dc_network_modelled);
  CHECK(r.model_stats.validity.vsc_dispatch_modelled);
  CHECK(r.model_stats.validity.dcdc_dispatch_modelled);
  CHECK(r.model_stats.validity.dc_branch_flow_limits_enforced);
  CHECK(r.model_stats.validity.converter_transfer_limits_enforced);
  CHECK(r.model_stats.validity.ac_dc_storage_modelled);
  CHECK_FALSE(r.model_stats.validity.converter_losses_modelled);
  CHECK_FALSE(r.model_stats.validity.reactive_power_modelled);
  CHECK_FALSE(r.model_stats.validity.protection_logic_modelled);
  CHECK_FALSE(r.model_stats.validity.transient_limits_modelled);

  const auto has_component = [&](const std::string& type) {
    return std::any_of(
        r.steps.back().component_states.begin(),
        r.steps.back().component_states.end(),
        [&](const ResilienceComponentStateStep& state) {
          return state.component_type == type;
        });
  };
  CHECK(has_component("ac_branch"));
  CHECK(has_component("dc_branch"));
  CHECK(has_component("vsc_converter"));
  CHECK(has_component("dcdc_converter"));
  CHECK(std::all_of(r.steps.back().component_states.begin(),
                    r.steps.back().component_states.end(),
                    [](const ResilienceComponentStateStep& state) {
                      return state.active_power_valid;
                    }));

  CHECK(std::find(r.steps.front().open_dc_branch_ids.begin(),
                  r.steps.front().open_dc_branch_ids.end(),
                  1) != r.steps.front().open_dc_branch_ids.end());
  CHECK(std::find(r.steps.back().open_dc_branch_ids.begin(),
                  r.steps.back().open_dc_branch_ids.end(),
                  1) == r.steps.back().open_dc_branch_ids.end());
  CHECK(std::find(r.steps.back().bus_supply_kind.begin(),
                  r.steps.back().bus_supply_kind.end(),
                  "AC") != r.steps.back().bus_supply_kind.end());
  CHECK(std::find(r.steps.back().bus_supply_kind.begin(),
                  r.steps.back().bus_supply_kind.end(),
                  "DC") != r.steps.back().bus_supply_kind.end());
  CHECK(std::any_of(r.steps.back().bus_voltages.begin(),
                    r.steps.back().bus_voltages.end(),
                    [](const BusVoltageStep& voltage) {
                      return voltage.domain == "DC" && voltage.bus_index == 1;
                    }));
}

TEST_CASE("Resilience B&C backends are independent across ordered calls",
          "[resilience][mip][backend-state]") {
  const auto native_sys = make_radial_3bus();
  DistributionResilienceOptions native_opts;
  native_opts.horizon_hours = 2;
  native_opts.time_step_hr = 1.0;
  native_opts.default_fault_count = 0;
  native_opts.mip.solver = DistributionResilienceMIPSolver::Native;
  native_opts.mip.max_time_s = 20;

  const auto highs_sys = make_hybrid_transfer_component_case();
  const auto highs_opts =
      make_hybrid_dc_fault_options(DistributionResilienceMIPSolver::HiGHS);

  for (int iteration = 0; iteration < 5; ++iteration) {
    CAPTURE(iteration);
    const auto native_result =
        run_distribution_resilience_mip_assessment(native_sys, native_opts);
    INFO(native_result.status);
    INFO(native_result.model_stats.solver_status);
    REQUIRE(native_result.feasible);

    const auto highs_result =
        run_distribution_resilience_mip_assessment(highs_sys, highs_opts);
    INFO(highs_result.status);
    INFO(highs_result.model_stats.solver_status);
    REQUIRE(highs_result.feasible);
    CHECK(highs_result.model_stats.validity.dc_network_modelled);
    CHECK(highs_result.model_stats.validity.vsc_dispatch_modelled);
  }
}

TEST_CASE("Resilience: strict MIP rejects unmatched domain-qualified faults",
          "[resilience][mip][fault-validation]") {
  auto sys = make_hybrid_no_switch_bus_load_case();
  DistributionResilienceFault fault;
  fault.branch_kind = ResilienceBranchKind::DC;
  fault.branch_index = 999;

  DistributionResilienceOptions opts;
  opts.horizon_hours = 1;
  opts.time_step_hr = 1.0;
  opts.default_fault_count = 0;
  opts.faults = {fault};

  const auto r = run_distribution_resilience_mip_assessment(sys, opts);
  CHECK_FALSE(r.feasible);
  CHECK(r.fault_sequence.empty());
  CHECK(r.status.find("do not match") != std::string::npos);
}

TEST_CASE("Resilience: strict MIP canonicalizes rich distribution components",
          "[resilience][mip][projection]") {
  HybridPowerSystem sys;
  for (int index = 1; index <= 4; ++index) {
    ACBus bus;
    bus.index = index;
    bus.bus_type = index == 1 ? BusType::SLACK : BusType::PQ;
    bus.base_kv = index == 1 ? 20.0 : 10.0;
    sys.ac.buses.push_back(bus);
  }

  Transformer2W transformer;
  transformer.index = 101;
  transformer.hv_bus = 1;
  transformer.lv_bus = 2;
  transformer.sn_mva = 20.0;
  transformer.vn_hv_kv = 20.0;
  transformer.vn_lv_kv = 10.0;
  transformer.vk_percent = 6.0;
  transformer.vkr_percent = 0.6;
  sys.ac.transformers_2w.push_back(transformer);

  Transformer3W transformer_3w;
  transformer_3w.index = 104;
  transformer_3w.hv_bus = 1;
  transformer_3w.mv_bus = 3;
  transformer_3w.lv_bus = 4;
  transformer_3w.sn_hv_mva = 15.0;
  transformer_3w.sn_mv_mva = 10.0;
  transformer_3w.sn_lv_mva = 5.0;
  transformer_3w.vn_hv_kv = 20.0;
  transformer_3w.vn_mv_kv = 10.0;
  transformer_3w.vn_lv_kv = 10.0;
  transformer_3w.vk_hv_mv_percent = 7.0;
  transformer_3w.vk_hv_lv_percent = 8.0;
  transformer_3w.vk_mv_lv_percent = 5.0;
  transformer_3w.vkr_hv_mv_percent = 0.7;
  transformer_3w.vkr_hv_lv_percent = 0.8;
  transformer_3w.vkr_mv_lv_percent = 0.5;
  sys.ac.transformers_3w.push_back(transformer_3w);

  Switch sw;
  sw.index = 102;
  sw.bus_from = 2;
  sw.bus_to = 3;
  sw.closed = true;
  sw.r_contact_ohm = 0.001;
  sys.ac.switches.push_back(sw);

  CircuitBreaker breaker;
  breaker.index = 103;
  breaker.bus_from = 3;
  breaker.bus_to = 4;
  breaker.closed = true;
  breaker.z_ohm = 0.001;
  sys.ac.circuit_breakers.push_back(breaker);

  FlexibleLoad flexible;
  flexible.index = 201;
  flexible.bus = 2;
  flexible.p_mw = 0.4;
  flexible.priority = LoadPriority::High;
  sys.ac.flexible_loads.push_back(flexible);

  AsymmetricLoad asymmetric;
  asymmetric.index = 202;
  asymmetric.bus = 3;
  asymmetric.pa_mw = 0.1;
  asymmetric.pb_mw = 0.1;
  asymmetric.pc_mw = 0.1;
  asymmetric.priority = LoadPriority::Critical;
  sys.ac.asymmetric_loads.push_back(asymmetric);

  AsynchronousMotor motor;
  motor.index = 203;
  motor.bus = 4;
  motor.sn_mva = 0.25;
  motor.cos_phi = 0.9;
  motor.efficiency = 0.95;
  sys.ac.motors.push_back(motor);

  VirtualPowerPlant vpp;
  vpp.index = 301;
  vpp.pcc_bus = 3;
  vpp.p_output_mw = 0.2;
  vpp.pmax_mw = 0.3;
  sys.vpps.push_back(vpp);

  Microgrid microgrid;
  microgrid.index = 302;
  microgrid.pcc_bus = 4;
  microgrid.operating_mode = MicrogridMode::GridConnected;
  microgrid.p_exchange_mw = 0.1;
  microgrid.p_exchange_max_mw = 0.2;
  microgrid.p_exchange_min_mw = -0.2;
  sys.microgrids.push_back(microgrid);

  EnergyRouter router;
  router.index = 401;
  router.name = "restoration_router";
  router.num_ports = 2;
  router.p_rated_mw = 0.5;
  router.pmax_mw = 0.5;
  router.pmin_mw = -0.5;
  EnergyRouterPort port_a;
  port_a.index = 1;
  port_a.bus = 2;
  port_a.side = 0;
  port_a.port_type = ERPortType::AC;
  port_a.pmax_mw = 0.5;
  port_a.pmin_mw = -0.5;
  EnergyRouterPort port_b = port_a;
  port_b.index = 2;
  port_b.bus = 4;
  port_b.side = 1;
  router.ports = {port_a, port_b};
  sys.energy_routers.push_back(router);

  DCBus dc_bus_a;
  dc_bus_a.index = 901;
  dc_bus_a.bus_type = DCBusType::DC_P;
  DCBus dc_bus_b = dc_bus_a;
  dc_bus_b.index = 902;
  dc_bus_b.pd_mw = 0.05;
  sys.dc.buses = {dc_bus_a, dc_bus_b};

  LCCConverter lcc;
  lcc.index = 501;
  lcc.ac_bus = 1;
  lcc.dc_bus = 901;
  lcc.p_set_mw = 0.2;
  sys.lcc_converters.push_back(lcc);

  DCCircuitBreaker dc_breaker;
  dc_breaker.index = 502;
  dc_breaker.bus_from = 901;
  dc_breaker.bus_to = 902;
  dc_breaker.closed = true;
  dc_breaker.rated_voltage_kv = 10.0;
  dc_breaker.i_rated_ka = 0.1;
  sys.dc.dc_circuit_breakers.push_back(dc_breaker);

  DistributionResilienceOptions opts;
  opts.horizon_hours = 1;
  opts.time_step_hr = 1.0;
  opts.default_fault_count = 0;

  const auto r = run_distribution_resilience_mip_assessment(sys, opts);
  INFO(r.status);
  REQUIRE(r.feasible);
  CHECK(r.model_stats.validity.canonical_projection_used);
  CHECK(r.model_stats.validity.energy_router_modelled);
  CHECK(r.model_stats.validity.transformers_modelled);
  CHECK(r.model_stats.validity.switches_and_breakers_modelled);
  CHECK(r.model_stats.validity.rich_loads_modelled);
  CHECK(r.model_stats.validity.aggregated_resources_modelled);
  CHECK(r.model_stats.validity.vsc_dispatch_modelled);
  CHECK(r.model_stats.validity.lcc_dispatch_modelled);
  CHECK(r.model_stats.validity.dcdc_dispatch_modelled);
  CHECK(std::any_of(r.steps.front().component_states.begin(),
                    r.steps.front().component_states.end(),
                    [](const ResilienceComponentStateStep& state) {
                      return state.component_type == "vsc_converter";
                    }));
  CHECK(std::any_of(r.steps.front().component_states.begin(),
                    r.steps.front().component_states.end(),
                    [](const ResilienceComponentStateStep& state) {
                      return state.component_type == "dcdc_converter";
                    }));
  CHECK(std::none_of(r.steps.front().component_states.begin(),
                     r.steps.front().component_states.end(),
                     [](const ResilienceComponentStateStep& state) {
                       return state.active_power_valid;
                     }));
  CHECK(std::any_of(r.steps.front().component_states.begin(),
                    r.steps.front().component_states.end(),
                    [](const ResilienceComponentStateStep& state) {
                      return state.component_type == "transformer_2w" &&
                             state.component_index == 101 &&
                             state.canonical_component_index != 0;
                    }));
  CHECK(std::any_of(r.steps.front().component_states.begin(),
                    r.steps.front().component_states.end(),
                    [](const ResilienceComponentStateStep& state) {
                      return state.component_type == "switch" &&
                             state.component_index == 102;
                    }));
  CHECK(std::any_of(r.steps.front().component_states.begin(),
                    r.steps.front().component_states.end(),
                    [](const ResilienceComponentStateStep& state) {
                      return state.component_type == "circuit_breaker" &&
                             state.component_index == 103;
                    }));
  CHECK(std::any_of(r.steps.front().component_states.begin(),
                    r.steps.front().component_states.end(),
                    [](const ResilienceComponentStateStep& state) {
                      return state.component_type == "transformer_3w" &&
                             state.component_index == 104 &&
                             state.pair_number >= 0 && state.pair_number <= 2;
                    }));
  CHECK(std::any_of(r.steps.front().component_states.begin(),
                    r.steps.front().component_states.end(),
                    [](const ResilienceComponentStateStep& state) {
                      return state.component_type == "lcc_converter" &&
                             state.component_index == 501 &&
                             state.domain == "AC-DC";
                    }));
  CHECK(std::any_of(r.steps.front().component_states.begin(),
                    r.steps.front().component_states.end(),
                    [](const ResilienceComponentStateStep& state) {
                      return state.component_type == "dc_circuit_breaker" &&
                             state.component_index == 502 &&
                             state.domain == "DC";
                    }));
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

TEST_CASE("Sequential MC: N-0 baseline curtailment is counted every hour",
          "[reliability][sequential]") {
  auto sys = make_single_bus_shortage();

  LoadProfile profile;
  profile.factors = {1.0, 1.0, 1.0, 1.0};

  ReliabilityOptions opts;
  opts.max_iterations = 1;
  opts.hours_per_year = 4;
  opts.seed = 1234;
  opts.compute_tail_risk = false;

  const auto r = run_sequential_mc(sys, profile, opts);

  CHECK(r.iterations_used == 1);
  CHECK(r.eens_mwh_yr == Approx(4.0).margin(1e-6));
  CHECK(r.lole_hr_yr == Approx(4.0).margin(1e-6));
  CHECK(r.baseline_eens_mwh_yr == Approx(4.0).margin(1e-6));
  CHECK(r.incremental_eens_mwh_yr == Approx(0.0).margin(1e-9));
  REQUIRE(r.annual_eens.size() == 1);
  CHECK(r.annual_eens.front() == Approx(4.0).margin(1e-6));
}

TEST_CASE("Sequential MC: per-load spatial factors preserve distinct load points",
          "[reliability][sequential][load-profile]") {
  auto sys = make_single_bus_shortage();
  sys.ac.buses.front().pd_mw = 0.0;
  Load first;
  first.index = 11;
  first.bus = 1;
  first.name = "Residential";
  first.p_mw = 1.0;
  Load second = first;
  second.index = 12;
  second.name = "Industrial";
  sys.ac.loads = {first, second};

  LoadProfile profile;
  profile.factors = {1.0, 1.0, 1.0, 1.0};
  profile.ac_load_factors = {0.5, 1.0};

  ReliabilityOptions opts;
  opts.max_iterations = 1;
  opts.hours_per_year = 4;
  opts.seed = 1234;
  opts.compute_tail_risk = false;

  const auto r = run_sequential_mc(sys, profile, opts);

  // Effective demand is 0.5 + 1.0 MW against 1.0 MW capacity.
  CHECK(r.eens_mwh_yr == Approx(2.0).margin(1e-6));
  CHECK(r.lole_hr_yr == Approx(4.0).margin(1e-6));
  CHECK(r.baseline_eens_mwh_yr == Approx(2.0).margin(1e-6));
  CHECK(r.incremental_eens_mwh_yr == Approx(0.0).margin(1e-9));
}

TEST_CASE("Sequential MC: inactive baseline components are not sampled as failures",
          "[reliability][sequential]") {
  auto sys = make_single_bus_shortage();

  ACBranch inactive;
  inactive.index = 99;
  inactive.from_bus = 1;
  inactive.to_bus = 1;
  inactive.in_service = false;
  inactive.failure_rate = 1.0e9;
  inactive.mttr_hr = 1000.0;
  sys.ac.branches.push_back(inactive);

  LoadProfile profile;
  profile.factors = {1.0, 1.0, 1.0, 1.0};

  ReliabilityOptions opts;
  opts.max_iterations = 1;
  opts.hours_per_year = 4;
  opts.seed = 1234;
  opts.compute_tail_risk = false;

  const auto r = run_sequential_mc(sys, profile, opts);

  for (const auto& component : r.critical_components) {
    CHECK(component.component_type != "ACBranch");
  }
}

// ── FMEA budget-truncation tests ─────────────────────────────────────────────

/// When max_repair_opf_calls=1 the repair-stage topology search is capped to
/// a single OPF evaluation (the baseline with no switch actions).  With budget
/// exhausted at call 1, budget_exhausted() is true, so repair_search_truncated
/// must be set on every contingency that reaches the repair stage.
TEST_CASE("FMEA: repair_search_truncated set when budget=1 with switch reconfiguration",
          "[fmea][reliability][truncation]") {
  // Build the 3-bus radial system and add a normally-open tie switch so the
  // repair stage has at least one candidate to enumerate.
  auto sys = make_radial_3bus();

  // Generator at the slack bus to supply load in the healthy state.
  Generator g;
  g.index = 1; g.bus = 1; g.in_service = true;
  g.pg_mw = 5.0; g.pmax_mw = 5.0; g.pmin_mw = 0.0;
  sys.ac.generators = {g};

  // Normally-open automated tie switch Bus3 → Bus1 (ring-back path).
  Switch sw;
  sw.index = 1; sw.bus_from = 3; sw.bus_to = 1;
  sw.in_service = true; sw.closed = false;
  sw.is_automated = true;
  sys.ac.switches = {sw};

  analysis::FMEAOptions opts;
  opts.enable_switch_reconfiguration = true;
  opts.max_repair_switch_actions = 1;
  opts.max_repair_opf_calls = 1;  // only baseline OPF allowed → every repair search truncates

  const auto result = analysis::run_distribution_fmea(sys, opts);

  REQUIRE(result.n_contingencies > 0);

  // Every contingency that visits the repair reconfiguration path must be
  // flagged (budget exhausted immediately because baseline used all 1 call).
  bool any_truncated = false;
  for (const auto& det : result.contingencies) {
    if (det.repair_search_truncated) { any_truncated = true; break; }
  }
  CHECK(any_truncated);
}

/// With unlimited budget (max_repair_opf_calls=0) no contingency should be
/// flagged as truncated on this small system.
TEST_CASE("FMEA: repair_search_truncated false with unlimited budget",
          "[fmea][reliability][truncation]") {
  auto sys = make_radial_3bus();

  Generator g;
  g.index = 1; g.bus = 1; g.in_service = true;
  g.pg_mw = 5.0; g.pmax_mw = 5.0; g.pmin_mw = 0.0;
  sys.ac.generators = {g};

  Switch sw;
  sw.index = 1; sw.bus_from = 3; sw.bus_to = 1;
  sw.in_service = true; sw.closed = false; sw.is_automated = true;
  sys.ac.switches = {sw};

  analysis::FMEAOptions opts;
  opts.enable_switch_reconfiguration = true;
  opts.max_repair_switch_actions = 1;
  opts.max_repair_opf_calls = 0;  // 0 = unlimited

  const auto result = analysis::run_distribution_fmea(sys, opts);

  for (const auto& det : result.contingencies) {
    CHECK_FALSE(det.repair_search_truncated);
  }
}

TEST_CASE("FMEA: hybrid catalog optimizes DC load and VSC transfer",
          "[fmea][reliability][regression]") {
  auto sys = make_radial_3bus();

  Generator g;
  g.index = 1;
  g.bus = 1;
  g.in_service = true;
  g.pg_mw = 5.0;
  g.pmax_mw = 5.0;
  g.pmin_mw = 0.0;
  sys.ac.generators = {g};

  DCBus dc_bus;
  dc_bus.index = 101;
  dc_bus.in_service = true;
  dc_bus.pd_mw = 0.5;
  dc_bus.is_load = true;
  sys.dc.buses = {dc_bus};

  DCLoad dc_load;
  dc_load.index = 1;
  dc_load.bus = 101;
  dc_load.in_service = true;
  dc_load.p_mw = 0.5;
  sys.dc.loads = {dc_load};

  VSCConverter vsc;
  vsc.index = 1;
  vsc.bus_ac = 1;
  vsc.bus_dc = 101;
  vsc.in_service = true;
  vsc.controllable = true;
  vsc.p_rated_mw = 2.0;
  vsc.pmin_mw = -2.0;
  vsc.pmax_mw = 2.0;
  vsc.mttr_hr = 10.0;
  vsc.forced_outage_rate = 0.01;
  sys.vsc_converters = {vsc};

  analysis::FMEAOptions opts;
  opts.enable_switch_reconfiguration = false;
  const auto result = analysis::run_distribution_fmea(sys, opts);

  CHECK(result.model_scope == "hybrid-acdc-network-lp");
  CHECK_FALSE(result.model_limitations.empty());
  CHECK(result.model_limitations.find("DC load shedding") != std::string::npos);
  CHECK(result.model_limitations.find("AC switches/branches only") != std::string::npos);
  CHECK(result.validity.dc_load_curtailment_included);
  CHECK(result.validity.vsc_dc_power_flow_modelled);
  CHECK(result.validity.ac_opf_curtailment);
  CHECK_FALSE(result.validity.ac_voltage_reactive_feasibility_certified);
  CHECK_FALSE(result.validity.repair_ac_switch_reconfiguration_modelled);
  CHECK_FALSE(result.validity.repair_dc_side_reconfiguration_modelled);
  CHECK(result.nodal_eens_mwh_yr.size() == sys.ac.buses.size() + sys.dc.buses.size());

  bool saw_vsc_loss = false;
  for (const auto& detail : result.contingencies) {
    if (detail.component_type != "vsc_converter") continue;
    saw_vsc_loss = true;
    CHECK(detail.shed_rep_mw >= Approx(1.0).margin(1e-6));
    REQUIRE(detail.nodal_shed_rep_mw.size() == sys.ac.buses.size() + sys.dc.buses.size());
    CHECK(detail.nodal_shed_rep_mw.back() >= Approx(1.0).margin(1e-6));
  }
  CHECK(saw_vsc_loss);
}

TEST_CASE("FMEA: load_scale_factor scales DC bus-level demand",
          "[fmea][reliability][regression]") {
  auto sys = make_radial_3bus();

  Generator g;
  g.index = 1;
  g.bus = 1;
  g.in_service = true;
  g.pg_mw = 10.0;
  g.pmax_mw = 10.0;
  g.pmin_mw = 0.0;
  sys.ac.generators = {g};

  DCBus dc_bus;
  dc_bus.index = 101;
  dc_bus.in_service = true;
  dc_bus.pd_mw = 0.75;
  dc_bus.is_load = true;
  sys.dc.buses = {dc_bus};

  VSCConverter vsc;
  vsc.index = 1;
  vsc.bus_ac = 1;
  vsc.bus_dc = 101;
  vsc.in_service = true;
  vsc.controllable = true;
  vsc.p_rated_mw = 5.0;
  vsc.pmin_mw = -5.0;
  vsc.pmax_mw = 5.0;
  vsc.mttr_hr = 10.0;
  vsc.forced_outage_rate = 0.01;
  sys.vsc_converters = {vsc};

  analysis::FMEAOptions opts;
  opts.enable_switch_reconfiguration = false;
  opts.load_scale_factor = 2.0;
  const auto result = analysis::run_distribution_fmea(sys, opts);

  bool saw_vsc_loss = false;
  for (const auto& detail : result.contingencies) {
    if (detail.component_type != "vsc_converter") continue;
    saw_vsc_loss = true;
    CHECK(detail.shed_rep_mw == Approx(1.5).margin(1e-6));
    REQUIRE(detail.nodal_shed_rep_mw.size() == sys.ac.buses.size() + sys.dc.buses.size());
    CHECK(detail.nodal_shed_rep_mw.back() == Approx(1.5).margin(1e-6));
  }
  CHECK(saw_vsc_loss);
}
