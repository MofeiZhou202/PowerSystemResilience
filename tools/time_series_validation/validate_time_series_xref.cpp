#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "hacdcpf/io/gridlabd_bridge.hpp"
#include "hacdcpf/time_series/time_series_pf.hpp"

namespace {

using nlohmann::json;
using namespace hacdcpf;

constexpr int kSteps = 4;

HybridPowerSystem build_uc_case() {
  HybridPowerSystem sys;
  sys.name = "time_series_uc_xref";
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;

  ACBus bus;
  bus.index = 1;
  bus.name = "UC_BUS";
  bus.bus_type = BusType::SLACK;
  bus.base_kv = 110.0;
  sys.ac.buses.push_back(bus);

  Load load;
  load.index = 1;
  load.name = "UC_LOAD";
  load.bus = 1;
  load.p_mw = 100.0;
  load.profile_id = 101;
  sys.ac.loads.push_back(load);

  Generator base;
  base.index = 1;
  base.name = "BASE";
  base.bus = 1;
  base.is_slack = true;
  base.pg_mw = 50.0;
  base.pmin_mw = 0.0;
  base.pmax_mw = 100.0;
  base.cost_c1 = 18.0;
  base.cost_c0 = 45.0;
  base.startup_cost = 420.0;
  base.min_up_time_hr = 2.0;
  base.min_dn_time_hr = 1.0;
  base.ramp_up_mw_min = 2.0;
  base.ramp_dn_mw_min = 2.0;
  sys.ac.generators.push_back(base);

  Generator peak;
  peak.index = 2;
  peak.name = "PEAK";
  peak.bus = 1;
  peak.pg_mw = 0.0;
  peak.pmin_mw = 0.0;
  peak.pmax_mw = 65.0;
  peak.cost_c1 = 43.0;
  peak.cost_c0 = 25.0;
  peak.startup_cost = 310.0;
  peak.min_up_time_hr = 2.0;
  peak.min_dn_time_hr = 2.0;
  peak.ramp_up_mw_min = 2.0;
  peak.ramp_dn_mw_min = 2.0;
  sys.ac.generators.push_back(peak);

  Storage storage;
  storage.index = 1;
  storage.name = "BESS";
  storage.bus = 1;
  storage.pmin_mw = -20.0;
  storage.pmax_mw = 20.0;
  storage.e_rated_mwh = 40.0;
  storage.soc_init = 0.50;
  storage.soc_min = 0.10;
  storage.soc_max = 0.90;
  storage.eta_charge = 0.90;
  storage.eta_discharge = 0.90;
  sys.ac.storage.push_back(storage);
  return sys;
}

TimeSeriesData build_uc_series() {
  TimeSeriesData ts;
  ts.num_steps = kSteps;
  ts.step_duration_hr = 1.0;
  ts.profiles.push_back(
      TimeSeriesProfile{101, "load_scale", {0.60, 1.30, 0.85, 1.40}});
  return ts;
}

HybridPowerSystem build_pf_case(double scale) {
  HybridPowerSystem sys;
  sys.name = "time_series_pf_xref";
  sys.base_mva = 10.0;
  sys.ac.base_mva = 10.0;
  sys.ac.freq_hz = 50.0;

  ACBus slack;
  slack.index = 1;
  slack.name = "SOURCEBUS";
  slack.bus_type = BusType::SLACK;
  slack.base_kv = 12.47;
  slack.vm_pu = 1.0;
  sys.ac.buses.push_back(slack);

  ACBus load;
  load.index = 2;
  load.name = "LOADBUS";
  load.bus_type = BusType::PQ;
  load.base_kv = 12.47;
  load.pd_mw = 1.2 * scale;
  load.qd_mvar = 0.45 * scale;
  sys.ac.buses.push_back(load);

  ACBranch line;
  line.index = 1;
  line.name = "L1";
  line.from_bus = 1;
  line.to_bus = 2;
  line.r_pu = 0.012;
  line.x_pu = 0.032;
  line.rate_a_mva = 10.0;
  sys.ac.branches.push_back(line);

  Generator source;
  source.index = 1;
  source.name = "SOURCE";
  source.bus = 1;
  source.is_slack = true;
  source.vg_pu = 1.0;
  source.pmin_mw = -100.0;
  source.pmax_mw = 100.0;
  source.qmin_mvar = -100.0;
  source.qmax_mvar = 100.0;
  sys.ac.generators.push_back(source);
  return sys;
}

json vector_matrix(const std::vector<std::vector<double>>& values) {
  return values;
}

json int_vector_matrix(const std::vector<std::vector<int>>& values) {
  return values;
}

json uc_result() {
  TimeSeriesPFOptions options;
  options.uc_solver = UCSolverChoice::HiGHS;
  options.run_opf = false;
  options.enable_external_grid = false;
  options.enforce_terminal_soc_cyclic = true;
  const auto result = solve_unit_commitment(build_uc_case(), build_uc_series(), options);

  return {
      {"case", "TS-UC-4H"},
      {"num_steps", kSteps},
      {"step_duration_hr", 1.0},
      {"load_mw", {60.0, 130.0, 85.0, 140.0}},
      {"generator", {
          {{"name", "BASE"}, {"pmin_mw", 0.0}, {"pmax_mw", 100.0},
           {"cost_c1", 18.0}, {"cost_c0", 45.0}, {"startup_cost", 420.0},
           {"initial_on", true}, {"min_up_hr", 2.0}, {"min_down_hr", 1.0},
           {"ramp_mw_per_period", 120.0}},
          {{"name", "PEAK"}, {"pmin_mw", 0.0}, {"pmax_mw", 65.0},
           {"cost_c1", 43.0}, {"cost_c0", 25.0}, {"startup_cost", 310.0},
           {"initial_on", false}, {"min_up_hr", 2.0}, {"min_down_hr", 2.0},
           {"ramp_mw_per_period", 120.0}}
      }},
      {"storage", {{"p_charge_max_mw", 20.0}, {"p_discharge_max_mw", 20.0},
                   {"energy_mwh", 40.0}, {"soc_init", 0.5}, {"soc_min", 0.1},
                   {"soc_max", 0.9}, {"eta_charge", 0.9},
                   {"eta_discharge", 0.9}, {"terminal_cyclic", true}}},
      {"hysim", {{"feasible", result.feasible}, {"solver_name", result.solver_name},
                 {"solver_status", result.solver_status}, {"mip_gap", result.mip_gap},
                 {"mip_gap_target_met", result.mip_gap_target_met},
                 {"optimality_proven", result.optimality_proven},
                 {"objective", result.total_cost},
                 {"gen_dispatch_mw", vector_matrix(result.gen_dispatch)},
                 {"gen_commit", int_vector_matrix(result.gen_commit)},
                 {"storage_dispatch_mw", vector_matrix(result.ess_dispatch)},
                 {"storage_soc", vector_matrix(result.ess_soc)}}}
  };
}

json pf_matrix() {
  const std::vector<double> scales{0.65, 0.80, 1.00, 1.15, 1.30, 0.90};
  json rows = json::array();
  bool all_passed = true;
  std::string executable;

  for (int t = 0; t < static_cast<int>(scales.size()); ++t) {
    io::GridLABDComparisonOptions options;
    options.require_gridlabd = true;
    options.vm_tolerance_pu = 5e-3;
    options.va_tolerance_deg = 0.5;
    options.branch_p_tolerance_mw = 2e-2;
    options.branch_q_tolerance_mvar = 2e-2;
    options.run_options.keep_working_files = false;
    const auto report = io::compare_gridlabd_snapshot(build_pf_case(scales[t]), options);
    all_passed = all_passed && report.equivalence_passed;
    executable = report.gridlabd_result.executable.string();

    json items = json::array();
    for (const auto& item : report.items) {
      items.push_back({{"kind", item.kind}, {"key", item.key},
                       {"hysim", item.hacdcpf_value},
                       {"gridlabd", item.gridlabd_value},
                       {"abs_error", std::abs(item.difference)},
                       {"tolerance", item.tolerance}, {"passed", item.passed}});
    }
    const auto& pf = report.hacdcpf_power_flow;
    rows.push_back({
        {"step", t}, {"scale", scales[t]}, {"load_p_mw", 1.2 * scales[t]},
        {"load_q_mvar", 0.45 * scales[t]}, {"hysim_converged", pf.converged},
        {"hysim_vm_pu", pf.vm}, {"hysim_va_rad", pf.va},
        {"hysim_branch", pf.branch_flows.empty() ? json::object() : json{
            {"pf_mw", pf.branch_flows[0].pf_mw}, {"qf_mvar", pf.branch_flows[0].qf_mvar},
            {"pt_mw", pf.branch_flows[0].pt_mw}, {"qt_mvar", pf.branch_flows[0].qt_mvar}}},
        {"gridlabd_run_success", report.gridlabd_run_success},
        {"gridlabd_equivalence_passed", report.equivalence_passed},
        {"gridlabd_items", std::move(items)}
    });
  }
  return {{"case", "TS-PF-6STEP"}, {"scales", scales},
          {"gridlabd_executable", executable}, {"all_gridlabd_passed", all_passed},
          {"steps", std::move(rows)}};
}

}  // namespace

int main() {
  try {
    const json result{{"schema", "hysim-time-series-cross-validation-v1"},
                      {"uc", uc_result()}, {"pf", pf_matrix()}};
    std::cout << result.dump(2) << '\n';
    return result["uc"]["hysim"]["feasible"].get<bool>() &&
                   result["pf"]["all_gridlabd_passed"].get<bool>()
               ? 0
               : 2;
  } catch (const std::exception& error) {
    std::cerr << "time-series validation driver failed: " << error.what() << '\n';
    return 1;
  }
}
