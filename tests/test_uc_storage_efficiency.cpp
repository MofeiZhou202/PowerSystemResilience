#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include "hacdcpf/time_series/time_series_pf.hpp"

using namespace hacdcpf;

namespace {

HybridPowerSystem make_ac_storage_case(double fixed_net_load_mw) {
  HybridPowerSystem sys;
  sys.base_mva = 100.0;

  ACBus bus;
  bus.index = 1;
  bus.bus_type = BusType::SLACK;
  bus.base_kv = 110.0;
  sys.ac.buses.push_back(bus);

  if (fixed_net_load_mw > 0.0) {
    Load load;
    load.index = 1;
    load.bus = 1;
    load.p_mw = fixed_net_load_mw;
    load.in_service = true;
    sys.ac.loads.push_back(load);
  } else if (fixed_net_load_mw < 0.0) {
    PVSystem pv;
    pv.index = 1;
    pv.bus = 1;
    pv.p_mw = -fixed_net_load_mw;
    pv.in_service = true;
    sys.ac.pv_systems.push_back(pv);
  }

  Storage storage;
  storage.index = 1;
  storage.bus = 1;
  storage.pmin_mw = -20.0;
  storage.pmax_mw = 20.0;
  storage.e_rated_mwh = 100.0;
  storage.soc_init = 0.5;
  storage.soc_min = 0.1;
  storage.soc_max = 0.9;
  storage.eta_charge = 0.8;
  storage.eta_discharge = 0.9;
  storage.self_discharge_pct = 10.0;
  storage.in_service = true;
  sys.ac.storage.push_back(storage);

  return sys;
}

TimeSeriesData one_hour() {
  TimeSeriesData ts;
  ts.num_steps = 1;
  ts.step_duration_hr = 1.0;
  return ts;
}

TimeSeriesPFOptions uc_options() {
  TimeSeriesPFOptions opts;
  opts.uc_solver = UCSolverChoice::HiGHS;
  opts.run_opf = false;
  return opts;
}

}  // namespace

TEST_CASE("UC storage SOC applies discharge efficiency in the correct direction",
          "[time_series][storage][efficiency]") {
  const UCSchedule schedule =
      solve_unit_commitment(make_ac_storage_case(9.0), one_hour(), uc_options());

  REQUIRE(schedule.feasible);
  REQUIRE(schedule.ess_dispatch.size() == 1);
  REQUIRE(schedule.ess_soc.size() == 1);
  CHECK_FALSE(schedule.solver_status.empty());
  CHECK(schedule.mip_gap_target_met);
  CHECK(schedule.optimality_proven);
  CHECK(schedule.ess_dispatch[0][0] == Catch::Approx(9.0).margin(1e-8));
  CHECK(schedule.ess_soc[0][0] == Catch::Approx(0.35).margin(1e-8));
}

TEST_CASE("UC storage SOC applies charging efficiency in the correct direction",
          "[time_series][storage][efficiency]") {
  const UCSchedule schedule =
      solve_unit_commitment(make_ac_storage_case(-10.0), one_hour(), uc_options());

  REQUIRE(schedule.feasible);
  REQUIRE(schedule.ess_dispatch.size() == 1);
  REQUIRE(schedule.ess_soc.size() == 1);
  CHECK(schedule.ess_dispatch[0][0] == Catch::Approx(-10.0).margin(1e-8));
  CHECK(schedule.ess_soc[0][0] == Catch::Approx(0.53).margin(1e-8));
}
