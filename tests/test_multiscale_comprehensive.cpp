#include <algorithm>
#include <cmath>
#include <vector>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "hacdcpf/api/hacdcpf.hpp"
#include "hacdcpf/io/case_builders.hpp"
#include "hacdcpf/io/json_io.hpp"
#include "hacdcpf/model/standard_parameter_library.hpp"
#include "hacdcpf/time_series/annual_production_sim.hpp"
#include "hacdcpf/time_series/time_series_pf.hpp"

using Catch::Approx;
using namespace hacdcpf;

namespace {

const std::vector<double> kDailyLoad = {
    0.62, 0.58, 0.55, 0.54, 0.56, 0.63, 0.72, 0.82,
    0.90, 0.94, 0.96, 0.95, 0.92, 0.89, 0.88, 0.91,
    0.97, 1.05, 1.10, 1.06, 0.98, 0.88, 0.76, 0.68};

const std::vector<double> kDailyWind = {
    0.72, 0.75, 0.77, 0.74, 0.68, 0.60, 0.48, 0.38,
    0.31, 0.28, 0.30, 0.35, 0.42, 0.48, 0.52, 0.55,
    0.51, 0.47, 0.45, 0.48, 0.55, 0.62, 0.67, 0.70};

const std::vector<double> kDailySolar = {
    0.0, 0.0, 0.0, 0.0, 0.0, 0.02, 0.12, 0.34,
    0.60, 0.80, 0.94, 1.00, 0.98, 0.88, 0.70, 0.48,
    0.24, 0.08, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0};

void bind_profiles(HybridPowerSystem& sys) {
  for (auto& load : sys.ac.loads) load.profile_id = 0;
  for (auto& load : sys.dc.loads) load.profile_id = 0;
  for (auto& wind : sys.ac.renewable_gens) wind.profile_id = 1;
  for (auto& pv : sys.ac.pv_systems) pv.profile_id = 2;
  for (auto& pv : sys.dc.pv_arrays) pv.profile_id = 2;
  for (auto& source : sys.dc.dc_static_generators) {
    if (source.type == "Wind") source.profile_id = 1;
  }
}

TimeSeriesData daily_profiles() {
  TimeSeriesData data;
  data.num_steps = 24;
  data.step_duration_hr = 1.0;
  data.profiles = {
      {0, "load", kDailyLoad},
      {1, "wind", kDailyWind},
      {2, "solar", kDailySolar},
  };
  return data;
}

TimeSeriesData weekly_profiles() {
  TimeSeriesData data;
  data.num_steps = 168;
  data.step_duration_hr = 1.0;
  data.profiles = {{0, "load", {}}, {1, "wind", {}}, {2, "solar", {}}};
  const std::vector<double> load_scale = {0.95, 1.00, 1.04, 1.08,
                                          1.02, 0.92, 0.88};
  const std::vector<double> wind_scale = {1.05, 0.92, 0.84, 0.78,
                                          0.90, 1.08, 1.15};
  const std::vector<double> solar_scale = {0.82, 0.90, 1.00, 1.08,
                                           1.02, 0.95, 0.88};
  for (size_t day = 0; day < 7; ++day) {
    for (size_t hour = 0; hour < 24; ++hour) {
      data.profiles[0].values.push_back(kDailyLoad[hour] * load_scale[day]);
      data.profiles[1].values.push_back(kDailyWind[hour] * wind_scale[day]);
      data.profiles[2].values.push_back(kDailySolar[hour] * solar_scale[day]);
    }
  }
  return data;
}

TimeSeriesData annual_daily_profiles() {
  TimeSeriesData data;
  data.num_steps = 365;
  data.step_duration_hr = 24.0;
  data.profiles = {{0, "daily_load", {}},
                   {1, "daily_wind", {}},
                   {2, "daily_solar", {}}};
  for (int day = 0; day < data.num_steps; ++day) {
    const double season = std::sin(2.0 * 3.14159265358979323846 *
                                   (static_cast<double>(day) - 30.0) / 365.0);
    data.profiles[0].values.push_back(0.82 + 0.13 * std::abs(season));
    data.profiles[1].values.push_back(0.48 + 0.14 * season);
    data.profiles[2].values.push_back(0.22 + 0.12 * season);
  }
  return data;
}

TimeSeriesPFOptions rich_dispatch_options() {
  TimeSeriesPFOptions options;
  options.uc_solver = UCSolverChoice::HiGHS;
  options.run_opf = false;
  options.enable_network_constraints = true;
  options.enable_dc_network_constraints = true;
  options.enable_dc_branch_flows = true;
  options.enable_vpp = true;
  options.enable_energy_router = true;
  options.enable_microgrid = true;
  options.enable_mobile_storage = true;
  options.enable_external_grid = true;
  options.enforce_terminal_soc_cyclic = true;
  options.pf_options.max_iter = 150;
  options.pf_options.tol = 1e-7;
  return options;
}

struct ReplayStats {
  double max_pf_residual{0.0};
  double max_voltage_violation{0.0};
  double max_branch_loading_pct{0.0};
  double max_vsc_loading_pct{0.0};
};

ReplayStats replay_stats(const HybridPowerSystem& sys,
                         const TimeSeriesPFResult& result) {
  ReplayStats stats;
  for (const auto& pf : result.pf_results) {
    if (!pf.converged) continue;
    stats.max_pf_residual = std::max(stats.max_pf_residual, pf.residual);
    for (size_t i = 0; i < pf.vm.size() && i < sys.ac.buses.size(); ++i) {
      stats.max_voltage_violation = std::max(
          stats.max_voltage_violation,
          std::max({sys.ac.buses[i].vmin_pu - pf.vm[i],
                    pf.vm[i] - sys.ac.buses[i].vmax_pu, 0.0}));
    }
    for (size_t i = 0; i < pf.vdc.size() && i < sys.dc.buses.size(); ++i) {
      stats.max_voltage_violation = std::max(
          stats.max_voltage_violation,
          std::max({sys.dc.buses[i].vmin_pu - pf.vdc[i],
                    pf.vdc[i] - sys.dc.buses[i].vmax_pu, 0.0}));
    }
    for (size_t i = 0;
         i < pf.branch_flows.size() && i < sys.ac.branches.size(); ++i) {
      const double rating = sys.ac.branches[i].rate_a_mva;
      if (rating <= 0.0) continue;
      const auto& flow = pf.branch_flows[i];
      const double apparent = std::max(
          std::hypot(flow.pf_mw, flow.qf_mvar),
          std::hypot(flow.pt_mw, flow.qt_mvar));
      stats.max_branch_loading_pct = std::max(
          stats.max_branch_loading_pct, 100.0 * apparent / rating);
    }
    for (const auto& transfer : pf.vsc_transfers) {
      const auto converter = std::find_if(
          sys.vsc_converters.begin(), sys.vsc_converters.end(),
          [&](const auto& item) { return item.index == transfer.index; });
      if (converter == sys.vsc_converters.end()) continue;
      const double rating = converter->p_rated_mw > 0.0
                                ? converter->p_rated_mw
                                : std::max(std::abs(converter->pmax_mw),
                                           std::abs(converter->pmin_mw));
      if (rating <= 0.0) continue;
      stats.max_vsc_loading_pct = std::max(
          stats.max_vsc_loading_pct,
          100.0 * std::hypot(transfer.p_ac_mw, transfer.q_ac_mvar) / rating);
    }
  }
  return stats;
}

}  // namespace

TEST_CASE("Multiscale comprehensive AC/DC constrained OPF converges",
          "[integration][multiscale][comprehensive][opf]") {
  const HybridPowerSystem sys = io::from_json(
      io::to_json(io::build_multiscale_comprehensive_acdc(), 0));

  opf::ACOPFOptions options;
  // This is an end-to-end model-coverage test, not a native-IPM trajectory
  // test. Use the production selector so a numerically stalled ParityIPM
  // iterate can be completed by Ipopt on the same full-space formulation.
  options.ac_solver_backend = opf::ACOPFSolverBackend::Auto;
  options.max_inner_iterations = 400;
  options.max_outer_iterations = 8;
  options.feasibility_tol = 1e-6;
  options.stationarity_tol = 1e-6;
  options.ac_pf_warm_start = true;
  options.allow_fallback = true;
  options.enforce_branch_limits = true;
  options.enforce_converter_capacity = true;
  options.enforce_converter_current_limits = true;
  options.enforce_converter_modulation_limits = true;

  const opf::ACOPFResult result = solve_ac_opf(sys, options);
  INFO(result.status);
  INFO("constraint violation=" << result.max_constraint_violation);
  INFO("stationarity=" << result.max_stationarity);
  INFO("linear solver=" << result.profiling.linear_solver_backend);
  INFO("AC P balance=" << result.profiling.max_ac_p_balance_residual_pu);
  INFO("AC Q balance=" << result.profiling.max_ac_q_balance_residual_pu);
  INFO("DC balance=" << result.profiling.max_dc_balance_residual_pu);
  INFO("converter balance="
       << result.profiling.max_converter_balance_residual_pu);
  INFO("other equalities="
       << result.profiling.max_other_equality_residual_pu);
  INFO("nonlinear inequalities="
       << result.profiling.max_nonlinear_inequality_violation_pu);
  INFO("complementarity=" << result.profiling.final_barrier_mu);
  if (!result.vm.empty()) {
    const auto ac_range = std::minmax_element(result.vm.begin(), result.vm.end());
    INFO("OPF AC voltage range=" << *ac_range.first << "," << *ac_range.second);
  }
  if (!result.vdc.empty()) {
    const auto dc_range = std::minmax_element(result.vdc.begin(), result.vdc.end());
    INFO("OPF DC voltage range=" << *dc_range.first << "," << *dc_range.second);
  }
  REQUIRE(result.converged);
  CHECK(result.iterations < options.max_inner_iterations *
                                options.max_outer_iterations);
  CHECK(result.max_constraint_violation < 100.0 * options.feasibility_tol);
  CHECK(result.max_stationarity < 100.0 * options.stationarity_tol);
  REQUIRE(result.vm.size() == sys.ac.buses.size());
  REQUIRE(result.vdc.size() == sys.dc.buses.size());
  REQUIRE(result.external_grid_p_mw.size() == sys.ac.external_grids.size());

  const ParameterValidationReport validation =
      validate_component_parameters(sys);
  CHECK(validation.error_count() == 0);
  CHECK(std::none_of(
      validation.diagnostics.begin(), validation.diagnostics.end(),
      [](const auto& diagnostic) {
        return diagnostic.component_type == "Mobile storage" &&
               diagnostic.code.rfind("opf_", 0) == 0;
      }));
}

TEST_CASE("Multiscale comprehensive AC/DC case runs from milliseconds to a year",
          "[integration][multiscale][comprehensive][slow]") {
  HybridPowerSystem authored = io::build_multiscale_comprehensive_acdc();
  bind_profiles(authored);

  REQUIRE(authored.ac.buses.size() >= 20);
  REQUIRE(authored.dc.buses.size() >= 4);
  REQUIRE(authored.vsc_converters.size() >= 3);
  REQUIRE(authored.dc.dcdc_converters.size() >= 2);
  REQUIRE_FALSE(authored.ac.storage.empty());
  REQUIRE_FALSE(authored.dc.storage.empty());
  REQUIRE_FALSE(authored.ac.circuit_breakers.empty());
  REQUIRE(authored.dc.dc_circuit_breakers.size() >= 2);
  REQUIRE(authored.energy_routers.size() == 3);
  std::vector<int> router_port_counts;
  for (const auto& router : authored.energy_routers) {
    REQUIRE(router.num_ports == static_cast<int>(router.ports.size()));
    router_port_counts.push_back(router.num_ports);
  }
  std::sort(router_port_counts.begin(), router_port_counts.end());
  CHECK(router_port_counts == std::vector<int>{2, 3, 4});
  REQUIRE_FALSE(authored.vpps.empty());
  REQUIRE_FALSE(authored.mobile_storage.empty());
  REQUIRE_FALSE(authored.microgrids.empty());

  const HybridPowerSystem sys =
      io::from_json(io::to_json(authored, 0));
  CHECK(sys.energy_routers.size() == authored.energy_routers.size());
  CHECK(sys.vpps.size() == authored.vpps.size());
  CHECK(sys.dc.dc_circuit_breakers.size() ==
        authored.dc.dc_circuit_breakers.size());

  PowerFlowOptions pf_options;
  pf_options.max_iter = 150;
  pf_options.tol = 1e-7;
  const PowerFlowResult steady = solve_power_flow(sys, pf_options);
  INFO("steady residual=" << steady.residual);
  REQUIRE(steady.converged);
  REQUIRE(steady.vm.size() == sys.ac.buses.size());
  REQUIRE(steady.vdc.size() == sys.dc.buses.size());
  CHECK(std::all_of(steady.vm.begin(), steady.vm.end(),
                    [](double value) { return std::isfinite(value); }));
  CHECK(std::all_of(steady.vdc.begin(), steady.vdc.end(),
                    [](double value) { return std::isfinite(value); }));

  dynamics::DynamicSolverOptions dynamic_options;
  dynamic_options.t_end_s = 0.010;
  dynamic_options.dt_s = 0.001;
  dynamic_options.run_power_flow_initialization = true;
  dynamic_options.project_to_canonical = true;
  dynamic_options.dynamic_dc_link = true;
  dynamic_options.record_device_outputs = true;
  const dynamics::DynamicResults transient =
      hacdcpf::run_transient_simulation(sys, dynamic_options);
  INFO(transient.message);
  REQUIRE(transient.success);
  REQUIRE(transient.steps >= 10);
  REQUIRE(transient.final_snapshot() != nullptr);
  CHECK(transient.final_snapshot()->time_s == Approx(0.010).margin(1e-9));

  const TimeSeriesPFResult daily =
      solve_time_series_pf(sys, daily_profiles(), rich_dispatch_options());
  INFO("daily converged=" << daily.num_converged << "/" << daily.num_steps);
  const ReplayStats daily_stats = replay_stats(sys, daily);
  INFO("maximum PF residual=" << daily_stats.max_pf_residual);
  INFO("maximum voltage violation=" << daily_stats.max_voltage_violation);
  INFO("maximum AC branch loading (%)="
       << daily_stats.max_branch_loading_pct);
  INFO("maximum VSC loading (%)=" << daily_stats.max_vsc_loading_pct);
  REQUIRE(daily.num_steps == 24);
  REQUIRE(daily.num_converged == 24);
  REQUIRE(daily.uc_schedule.feasible);
  REQUIRE_FALSE(daily.uc_schedule.solver_name.empty());

  const TimeSeriesPFResult weekly =
      solve_time_series_pf(sys, weekly_profiles(), rich_dispatch_options());
  const ReplayStats weekly_stats = replay_stats(sys, weekly);
  INFO("weekly converged=" << weekly.num_converged << "/" << weekly.num_steps);
  INFO("weekly maximum PF residual=" << weekly_stats.max_pf_residual);
  INFO("weekly maximum voltage violation="
       << weekly_stats.max_voltage_violation);
  INFO("weekly maximum AC branch loading (%)="
       << weekly_stats.max_branch_loading_pct);
  INFO("weekly maximum VSC loading (%)="
       << weekly_stats.max_vsc_loading_pct);
  REQUIRE(weekly.num_steps == 168);
  REQUIRE(weekly.num_converged == 168);
  REQUIRE(weekly.uc_schedule.feasible);
  CHECK(weekly_stats.max_voltage_violation <= 1e-9);
  CHECK(weekly_stats.max_branch_loading_pct < 100.0);
  CHECK(weekly_stats.max_vsc_loading_pct < 100.0);

  // Use schedule-only replay for the full calendar year so this regression
  // remains fast. Nodal AC/DC balance is checked above by the 168 full PF runs;
  // the annual stage validates chronology, dispatch feasibility, and energy.
  analysis::AnnualProductionSimOptions annual_options;
  annual_options.skip_replay = true;
  annual_options.enforce_cyclic_soc = true;
  annual_options.pf_snapshot_interval = 0;
  annual_options.ts_pf_options = rich_dispatch_options();
  annual_options.ts_pf_options.run_opf = false;
  const TimeSeriesData annual_input = annual_daily_profiles();
  const analysis::AnnualProductionSimResult annual =
      analysis::solve_annual_production_simulation(
          sys, annual_input, annual_options);
  INFO(annual.summary());
  REQUIRE(annual.feasible);
  REQUIRE(annual.num_steps == 365);
  CHECK(annual.step_duration_hr * annual.num_steps ==
        Approx(8760.0).margin(1e-9));
  REQUIRE(annual.step_results.size() == 365);
  CHECK(annual.num_opf_converged == 0);
  CHECK(annual.num_pf_converged == 0);
  CHECK(std::none_of(annual.step_results.begin(), annual.step_results.end(),
                     [](const analysis::AnnualStepResult& step) {
                       return step.opf_converged || step.pf_converged;
                     }));
  CHECK(annual.total_load_mwh > 0.0);
  CHECK(std::isfinite(annual.total_cost));
  CHECK(std::isfinite(annual.power_balance_error_mwh));
  CHECK(annual.total_ens_mwh == Approx(0.0).margin(1e-9));
}
