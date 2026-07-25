#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <numeric>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "hacdcpf/analysis/counterfactual_planning.hpp"
#include "hacdcpf/analysis/multidimensional_weak_link.hpp"
#include "hacdcpf/analysis/typhoon_resilience.hpp"
#include "hacdcpf/graph/power_system_graph.hpp"
#include "hacdcpf/graph/topology_analysis.hpp"
#include "hacdcpf/io/case_builders.hpp"
#include "hacdcpf/reliability/reliability_assessment.hpp"
#include "hacdcpf/resilience/resilience_dynamic_certification.hpp"
#include "hacdcpf/time_series/time_series_pf.hpp"

namespace fs = std::filesystem;
using json = nlohmann::json;
using namespace hacdcpf;
using namespace hacdcpf::analysis;

namespace {

using Clock = std::chrono::steady_clock;

double elapsed_seconds(Clock::time_point start) {
  return std::chrono::duration<double>(Clock::now() - start).count();
}

std::string csv_escape(const std::string& text) {
  if (text.find_first_of(",\"\n") == std::string::npos) return text;
  std::string out = "\"";
  for (const char ch : text) out += ch == '"' ? "\"\"" : std::string(1, ch);
  out += '"';
  return out;
}

void write_text(const fs::path& path, const std::string& text) {
  std::ofstream out(path);
  if (!out) throw std::runtime_error("cannot write " + path.string());
  out << text;
}

HybridPowerSystem build_review_case() {
  HybridPowerSystem sys = hacdcpf::io::build_cyber_physical_reliability_demo();
  sys.name = "Review benchmark: networked hybrid microgrids";
  sys.base_mva = sys.ac.base_mva = sys.dc.base_mva = 10.0;
  sys.ac.freq_hz = 50.0;

  const std::vector<std::pair<double, double>> coordinates = {
      {22.50, 113.72}, {22.54, 113.78}, {22.58, 113.84}};
  for (std::size_t i = 0; i < sys.ac.buses.size(); ++i) {
    sys.ac.buses[i].latitude = coordinates[i].first;
    sys.ac.buses[i].longitude = coordinates[i].second;
    sys.ac.buses[i].base_kv = 10.0;
  }

  sys.ac.external_grids.clear();
  Generator source;
  source.index = 1;
  source.name = "Utility equivalent machine";
  source.bus = 1;
  source.is_slack = true;
  source.in_service = true;
  source.pg_mw = 3.1;
  source.pmin_mw = 0.0;
  source.pmax_mw = 8.0;
  source.qmin_mvar = -5.0;
  source.qmax_mvar = 5.0;
  source.vg_pu = 1.0;
  source.xdpp_pu = 0.20;
  source.forced_outage_rate = 0.02;
  source.mttr_hr = 8.0;
  source.cost_c1 = 90.0;
  source.emission_factor_tco2_mwh = 0.55;
  sys.ac.generators = {source};

  sys.ac.loads[0].priority = LoadPriority::Critical;
  sys.ac.buses[1].importance = 3.0;
  sys.ac.loads[0].name = "Emergency coordination center";
  sys.ac.loads[1].priority = LoadPriority::High;
  sys.ac.buses[2].importance = 1.5;
  for (auto& branch : sys.ac.branches) {
    branch.length_km = branch.index == 1 ? 8.0 : 7.0;
    branch.rate_a_mva = std::max(2.5, branch.rate_a_mva);
    branch.mttr_hr = branch.index == 1 ? 8.0 : 6.0;
  }

  ACBus bus4;
  bus4.index = 4;
  bus4.name = "Hospital microgrid bus";
  bus4.bus_type = BusType::PQ;
  bus4.base_kv = 10.0;
  bus4.vm_pu = 1.0;
  bus4.latitude = 22.60;
  bus4.longitude = 113.77;
  sys.ac.buses.push_back(bus4);

  ACBranch lateral;
  lateral.index = 4;
  lateral.name = "Hospital lateral";
  lateral.from_bus = 2;
  lateral.to_bus = 4;
  lateral.r_pu = 0.015;
  lateral.x_pu = 0.08;
  lateral.rate_a_mva = 2.0;
  lateral.length_km = 9.0;
  lateral.failure_rate = 0.6;
  lateral.mttr_hr = 7.0;
  sys.ac.branches.push_back(lateral);

  ACBranch microgrid_tie;
  microgrid_tie.index = 5;
  microgrid_tie.name = "Normally open inter-microgrid tie";
  microgrid_tie.from_bus = 3;
  microgrid_tie.to_bus = 4;
  microgrid_tie.r_pu = 0.02;
  microgrid_tie.x_pu = 0.10;
  microgrid_tie.rate_a_mva = 1.5;
  microgrid_tie.length_km = 5.0;
  microgrid_tie.in_service = false;
  microgrid_tie.mttr_hr = 4.0;
  sys.ac.branches.push_back(microgrid_tie);

  Load hospital;
  hospital.index = 3;
  hospital.name = "Hospital critical load";
  hospital.bus = 4;
  hospital.p_mw = 0.6;
  hospital.q_mvar = 0.12;
  hospital.priority = LoadPriority::Critical;
  sys.ac.buses.back().importance = 5.0;
  hospital.n_customers = 1;
  sys.ac.loads.push_back(hospital);

  sys.ac.storage.front().grid_forming = true;
  sys.ac.storage.front().soc_init = 0.75;
  sys.ac.storage.front().soc_max = 0.95;
  sys.ac.storage.front().e_mwh = 7.5;
  sys.ac.storage.front().dynamic_model.model_name = "GridFormingBattery";

  Storage hospital_bess;
  hospital_bess.index = 2;
  hospital_bess.name = "Hospital GFM battery";
  hospital_bess.bus = 4;
  hospital_bess.in_service = true;
  hospital_bess.controllable = true;
  hospital_bess.grid_forming = true;
  hospital_bess.p_rated_mw = 0.8;
  hospital_bess.pmax_mw = 0.8;
  hospital_bess.pmin_mw = -0.8;
  hospital_bess.qmax_mvar = 0.4;
  hospital_bess.qmin_mvar = -0.4;
  hospital_bess.e_rated_mwh = 3.2;
  hospital_bess.e_mwh = 2.4;
  hospital_bess.soc_init = 0.75;
  hospital_bess.soc_min = 0.15;
  hospital_bess.soc_max = 0.95;
  hospital_bess.eta_charge = 0.95;
  hospital_bess.eta_discharge = 0.95;
  hospital_bess.dynamic_model.model_name = "GridFormingBattery";
  sys.ac.storage.push_back(hospital_bess);

  RenewableGen pv;
  pv.index = 1;
  pv.name = "Microgrid PV aggregate";
  pv.bus = 3;
  pv.type = RenewableType::SolarPV;
  pv.in_service = true;
  pv.p_mw = 0.35;
  pv.p_rated_mw = 0.8;
  pv.capacity_factor = 0.45;
  pv.curtailable = true;
  sys.ac.renewable_gens.push_back(pv);

  DCBus dc1;
  dc1.index = 101;
  dc1.name = "DC microgrid PCC";
  dc1.bus_type = DCBusType::DC_V;
  dc1.base_kv = 0.75;
  dc1.latitude = 22.60;
  dc1.longitude = 113.77;
  DCBus dc2 = dc1;
  dc2.index = 102;
  dc2.name = "DC critical service bus";
  dc2.bus_type = DCBusType::DC_P;
  dc2.latitude = 22.62;
  dc2.longitude = 113.80;
  sys.dc.buses = {dc1, dc2};

  DCBranch dc_line;
  dc_line.index = 101;
  dc_line.name = "DC service cable";
  dc_line.from_bus = 101;
  dc_line.to_bus = 102;
  dc_line.r_pu = 0.02;
  dc_line.rate_a_mva = 1.0;
  dc_line.s_max_mva = 1.0;
  dc_line.length_km = 3.0;
  dc_line.mttr_hours = 5.0;
  sys.dc.branches = {dc_line};

  DCLoad dc_critical;
  dc_critical.index = 101;
  dc_critical.name = "Emergency DC service";
  dc_critical.bus = 102;
  dc_critical.p_mw = 0.25;
  dc_critical.priority = LoadPriority::Critical;
  sys.dc.buses[1].importance = 4.0;
  dc_critical.n_customers = 1;
  sys.dc.loads = {dc_critical};

  VSCConverter vsc;
  vsc.index = 101;
  vsc.name = "Hospital AC/DC interlink";
  vsc.bus_ac = 4;
  vsc.bus_dc = 101;
  vsc.in_service = true;
  vsc.control_mode = ConverterMode::PQ_MODE;
  vsc.p_set_mw = 0.25;
  vsc.p_schedule_mw = 0.25;
  vsc.pmin_mw = -1.0;
  vsc.pmax_mw = 1.0;
  vsc.qmin_mvar = -0.5;
  vsc.qmax_mvar = 0.5;
  vsc.p_rated_mw = 1.0;
  vsc.eta = 0.98;
  vsc.i_ac_max_pu = 1.2;
  vsc.i_dc_max_pu = 1.2;
  sys.vsc_converters = {vsc};

  Microgrid mg1;
  mg1.index = 1;
  mg1.name = "Community microgrid";
  mg1.pcc_bus = 2;
  mg1.internal_buses = {2, 3};
  mg1.islanding_capability = true;
  mg1.auto_reconnection = true;
  mg1.p_import_max_mw = 2.5;
  mg1.p_export_max_mw = 1.0;
  mg1.capacity_mw = 1.8;
  mg1.total_storage_mwh = 10.0;

  Microgrid mg2;
  mg2.index = 2;
  mg2.name = "Hybrid hospital microgrid";
  mg2.pcc_bus = 4;
  mg2.internal_buses = {4};
  mg2.islanding_capability = true;
  mg2.auto_reconnection = true;
  mg2.p_import_max_mw = 1.5;
  mg2.p_export_max_mw = 0.8;
  mg2.capacity_mw = 1.8;
  mg2.total_storage_mwh = 3.2;
  sys.microgrids = {mg1, mg2};
  return sys;
}

TimeSeriesPFResult run_pre_event(const HybridPowerSystem& system) {
  TimeSeriesData data;
  data.num_steps = 24;
  data.step_duration_hr = 1.0;
  TimeSeriesPFOptions options;
  options.uc_solver = UCSolverChoice::HiGHS;
  options.run_opf = false;
  options.enable_network_constraints = true;
  options.enable_dc_network_constraints = true;
  options.enable_dc_branch_flows = true;
  options.enable_microgrid = true;
  options.enforce_terminal_soc_cyclic = true;
  options.keep_system_snapshots = false;
  return solve_time_series_pf(system, data, options);
}

TyphoonScenarioOptions hazard_options(unsigned seed, bool stochastic) {
  TyphoonScenarioOptions options;
  options.horizon_hours = 24;
  options.time_step_hr = 1.0;
  options.seed = seed;
  options.stochastic = stochastic;
  options.use_month_defaults = false;
  options.use_sst_resource = false;
  options.initial_latitude = 22.38;
  options.initial_longitude = 113.55;
  options.initial_delta_p_hpa = 82.0;
  options.initial_rmw_km = 32.0;
  options.initial_heading_deg = 48.0;
  options.initial_translation_speed_kmph = 18.0;
  options.target_segment_length_km = 2.0;
  options.default_overhead_fraction = 0.80;
  options.default_cable_fraction = 0.20;
  options.default_repair_time_hr = 6.0;
  options.max_repair_time_hr = 18.0;
  options.post_disaster_repair_delay_hr = 1.0;
  return options;
}

DistributionResilienceOptions restoration_options(
    const TyphoonFaultSequenceResult& hazard) {
  DistributionResilienceOptions options;
  options.horizon_hours = 24;
  options.time_step_hr = 1.0;
  options.default_fault_count = 0;
  options.faults = hazard.faults;
  options.renewable_profile = hazard.renewable_profile_multiplier;
  options.allow_reconfiguration = true;
  options.allow_mess_dispatch = true;
  options.mip.solver = DistributionResilienceMIPSolver::HiGHS;
  options.mip.mip_gap = 0.0;
  options.mip.max_time_s = 30;
  return options;
}

double tier_unserved_mwh(const DistributionResilienceResult& result,
                         double dt_hr, int priority_tier,
                         const std::string& domain = {}) {
  double total = 0.0;
  for (const auto& step : result.steps) {
    for (std::size_t i = 0; i < step.bus_supply_shed_mw.size(); ++i) {
      if (i < step.bus_supply_priority_tier.size() &&
          step.bus_supply_priority_tier[i] == priority_tier &&
          (domain.empty() ||
           (i < step.bus_supply_kind.size() &&
            step.bus_supply_kind[i] == domain))) {
        total += step.bus_supply_shed_mw[i] * dt_hr;
      }
    }
  }
  return total;
}

double quantile(std::vector<double> values, double probability) {
  if (values.empty()) return 0.0;
  std::sort(values.begin(), values.end());
  const double location = probability * static_cast<double>(values.size() - 1);
  const auto lower = static_cast<std::size_t>(std::floor(location));
  const auto upper = static_cast<std::size_t>(std::ceil(location));
  const double weight = location - static_cast<double>(lower);
  return values[lower] * (1.0 - weight) + values[upper] * weight;
}

double empirical_cvar(std::vector<double> values, double probability) {
  if (values.empty()) return 0.0;
  std::sort(values.begin(), values.end(), std::greater<double>());
  const double tail_count =
      std::max(1.0e-12, (1.0 - probability) * values.size());
  const std::size_t full = std::min(
      values.size(), static_cast<std::size_t>(std::floor(tail_count)));
  double tail_sum =
      std::accumulate(values.begin(), values.begin() + full, 0.0);
  const double fractional = tail_count - static_cast<double>(full);
  if (fractional > 1.0e-12 && full < values.size()) {
    tail_sum += fractional * values[full];
  }
  return tail_sum / tail_count;
}

struct RiskStats {
  double mean{0.0};
  double var95{0.0};
  double cvar95{0.0};
};

RiskStats risk_stats(const std::vector<double>& values) {
  RiskStats out;
  if (values.empty()) return out;
  out.mean = std::accumulate(values.begin(), values.end(), 0.0) /
             static_cast<double>(values.size());
  out.var95 = quantile(values, 0.95);
  out.cvar95 = empirical_cvar(values, 0.95);
  return out;
}

double relative_change(double current, double previous) {
  return std::abs(current - previous) /
         std::max(1.0e-9, std::abs(previous));
}

double maximum_branch_loading(const DistributionResilienceResult& result,
                              int branch_index) {
  double loading = 0.0;
  for (const auto& step : result.steps) {
    for (const auto& flow : step.branch_flows) {
      if (flow.component_type == "ac_branch" &&
          flow.branch_index == branch_index) {
        loading = std::max(loading, flow.loading_percent);
      }
    }
  }
  return loading;
}

json resilience_json(const DistributionResilienceResult& result) {
  json limitations = {
      {"converter_losses_modelled",
       result.model_stats.validity.converter_losses_modelled},
      {"reactive_power_modelled",
       result.model_stats.validity.reactive_power_modelled},
      {"protection_logic_modelled",
       result.model_stats.validity.protection_logic_modelled},
      {"transient_limits_modelled",
       result.model_stats.validity.transient_limits_modelled}};
  return {{"feasible", result.feasible},
          {"status", result.status},
          {"model_scope", result.model_stats.model_scope},
          {"solver", result.model_stats.solver_name},
          {"solver_status", result.model_stats.solver_status},
          {"runtime_sec", result.model_stats.runtime_sec},
          {"mip_gap", result.model_stats.mip_gap},
          {"total_demand_mwh", result.total_demand_mwh},
          {"total_served_mwh", result.total_served_mwh},
          {"total_shed_mwh", result.total_shed_mwh},
          {"priority_weighted_unserved_objective", result.weighted_unserved_mwh},
          {"priority_weighted_objective_unit",
           "priority-penalty times MWh; not physical energy"},
          {"resilience_index", result.resilience_index},
          {"peak_shed_mw", result.peak_shed_mw},
          {"switch_actions", result.total_switch_actions},
          {"limitations", std::move(limitations)}};
}

json certificate_json(const DistributionResilienceDAECertificateResult& result) {
  json transitions = json::array();
  for (const auto& row : result.transitions) {
    double min_frequency = std::numeric_limits<double>::infinity();
    double max_rocof = 0.0;
    double min_voltage = std::numeric_limits<double>::infinity();
    for (const auto& snapshot : row.certificate.trajectory.snapshots) {
      if (std::isfinite(snapshot.coi_frequency_hz))
        min_frequency = std::min(min_frequency, snapshot.coi_frequency_hz);
      if (std::isfinite(snapshot.coi_rocof_hz_s))
        max_rocof = std::max(max_rocof, std::abs(snapshot.coi_rocof_hz_s));
      if (std::isfinite(snapshot.min_ac_voltage_pu))
        min_voltage = std::min(min_voltage, snapshot.min_ac_voltage_pu);
    }
    transitions.push_back(
        {{"from_step", row.from_step},
         {"to_step", row.to_step},
         {"action_id", row.action.id},
         {"event_count", row.action.dynamic_events.size()},
         {"cyber_executable", row.executability.cyber_executable},
         {"executable", row.executability.executable},
         {"label", to_string(row.certificate.label)},
         {"proof_valid", row.certificate.proof_valid},
         {"simulation_success", row.certificate.simulation_success},
         {"simulation_message", row.certificate.trajectory.message},
         {"failed_step", row.certificate.trajectory.failed_step},
         {"warnings", row.certificate.trajectory.warnings},
         {"min_frequency_hz",
          std::isfinite(min_frequency) ? json(min_frequency) : json(nullptr)},
         {"max_abs_rocof_hz_s", max_rocof},
         {"min_ac_voltage_pu",
          std::isfinite(min_voltage) ? json(min_voltage) : json(nullptr)},
         {"unsupported_transitions", row.unsupported_transitions},
         {"limitations", row.certificate.limitations}});
  }
  json feedback = json::array();
  for (const auto& row : result.feedback) {
    feedback.push_back(
        {{"to_step", row.to_step},
         {"action_id", row.action_id},
         {"feedback_type", row.feedback_type},
         {"reason", row.reason},
         {"event_labels", row.event_labels},
         {"recommended_master_action", row.recommended_master_action},
         {"applied_to_restoration_mip", row.applied_to_restoration_mip}});
  }
  return {{"attempted", result.attempted},
          {"all_transitions_safe", result.all_transitions_safe},
          {"proof_valid", result.proof_valid},
          {"status", result.status},
          {"model_scope", result.model_scope},
          {"limitations", result.limitations},
          {"transitions", std::move(transitions)},
          {"feedback", std::move(feedback)},
          {"feedback_closed_loop_complete", false}};
}

std::vector<double> normalized(const std::vector<double>& values) {
  const double maximum = values.empty()
                             ? 0.0
                             : *std::max_element(values.begin(), values.end());
  std::vector<double> out(values.size(), 0.0);
  if (maximum <= 1e-12) return out;
  std::transform(values.begin(), values.end(), out.begin(),
                 [&](double value) { return value / maximum; });
  return out;
}

std::string latex_escape(std::string text) {
  const std::vector<std::pair<std::string, std::string>> replacements = {
      {"\\", "\\textbackslash{}"}, {"_", "\\_"}, {"%", "\\%"},
      {"&", "\\&"}, {"#", "\\#"}};
  for (const auto& [from, to] : replacements) {
    std::size_t pos = 0;
    while ((pos = text.find(from, pos)) != std::string::npos) {
      text.replace(pos, from.size(), to);
      pos += to.size();
    }
  }
  return text;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const fs::path output_dir = argc > 1
                                    ? fs::path(argv[1])
                                    : fs::path(HACDCPF_PROJECT_ROOT) /
                                          "docs/latex/paper/distribution_resilience_transient_microgrids_review/results";
    fs::create_directories(output_dir);
    json report;
    report["schema"] = "hacdcpf.distribution_resilience_review_case.v1";
    report["case"] = {{"name", "networked hybrid microgrid benchmark"},
                      {"ac_buses", 4}, {"dc_buses", 2},
                      {"microgrids", 2}, {"critical_loads", 3}};

    HybridPowerSystem system = build_review_case();

    auto started = Clock::now();
    const auto pre_event = run_pre_event(system);
    report["pre_event"] = {
        {"horizon_hr", 24},
        {"uc_feasible", pre_event.uc_schedule.feasible},
        {"solver", pre_event.uc_schedule.solver_name},
        {"solver_status", pre_event.uc_schedule.solver_status},
        {"mip_gap", pre_event.uc_schedule.mip_gap},
        {"terminal_soc_cyclic", true},
        {"ac_storage_soc", pre_event.uc_schedule.ess_soc},
        {"runtime_sec", elapsed_seconds(started)}};

    started = Clock::now();
    const auto hazard = generate_typhoon_fault_sequence(
        system, hazard_options(203007, false));
    report["hazard"] = {
        {"model", "Holland wind field + segment fragility + sampled branch failures"},
        {"seed", hazard.seed},
        {"status", hazard.status},
        {"track_points", hazard.track.size()},
        {"segments", hazard.generated_segments.size()},
        {"fault_count", hazard.faults.size()},
        {"max_track_wind_ms", hazard.selected_track_max_vmax_ms},
        {"used_synthetic_segments", hazard.used_synthetic_segments},
        {"used_fallback_coordinates", hazard.used_fallback_coordinates},
        {"runtime_sec", elapsed_seconds(started)}};

    json risks = json::array();
    for (const auto& risk : hazard.branch_risks) {
      risks.push_back({{"domain", to_string(risk.branch_kind)},
                       {"branch_index", risk.branch_index},
                       {"peak_wind_ms", risk.peak_wind_ms},
                       {"peak_rain_mm_hr", risk.peak_rain_mm_hr},
                       {"peak_failure_probability",
                        risk.peak_failure_probability}});
    }
    report["hazard"]["branch_risks"] = risks;

    auto event_options = restoration_options(hazard);
    started = Clock::now();
    const auto heuristic = run_distribution_resilience_assessment(
        system, event_options);
    const double heuristic_runtime = elapsed_seconds(started);
    started = Clock::now();
    const auto strict_mip = run_distribution_resilience_mip_assessment(
        system, event_options);
    const double strict_runtime = elapsed_seconds(started);
    report["restoration"] = {
        {"heuristic", resilience_json(heuristic)},
        {"strict_mip", resilience_json(strict_mip)}};
    report["restoration"]["heuristic"]["wall_runtime_sec"] = heuristic_runtime;
    report["restoration"]["strict_mip"]["wall_runtime_sec"] = strict_runtime;
    report["restoration"]["strict_mip"]["critical_unserved_mwh"] =
        tier_unserved_mwh(strict_mip, event_options.time_step_hr, 0);
    report["restoration"]["strict_mip"]["critical_ac_unserved_mwh"] =
        tier_unserved_mwh(strict_mip, event_options.time_step_hr, 0, "AC");
    report["restoration"]["strict_mip"]["critical_dc_unserved_mwh"] =
        tier_unserved_mwh(strict_mip, event_options.time_step_hr, 0, "DC");

    dynamics::DynamicSolverOptions dynamic_options;
    dynamic_options.t_end_s = 0.25;
    dynamic_options.dt_s = 0.005;
    dynamic_options.solver_type = dynamics::DynamicSolverType::TrapezoidalNewton;
    dynamic_options.run_power_flow_initialization = true;
    dynamic_options.record_device_outputs = true;
    MultiFidelityCertificateOptions certificate_options;
    certificate_options.min_frequency_hz = 49.0;
    certificate_options.max_rocof_hz_s = 1.0;
    certificate_options.min_ac_voltage_pu = 0.90;
    certificate_options.require_converter_current_observation = true;
    certificate_options.max_residual_samples = 12;
    certificate_options.max_jacobian_samples = 2;
    DistributionResilienceDAECertificationOptions bridge_options;
    bridge_options.fidelity_level = 3;
    bridge_options.switching_time_s = 0.05;
    bridge_options.max_transitions = 12;

    started = Clock::now();
    const auto dae = certify_distribution_resilience_dynamics(
        system, strict_mip, dynamic_options, certificate_options,
        bridge_options);
    report["dynamic_certificate"] = certificate_json(dae);
    report["dynamic_certificate"]["runtime_sec"] = elapsed_seconds(started);

    auto cyber_options = bridge_options;
    cyber_options.command_path_available = false;
    const auto cyber_blocked = certify_distribution_resilience_dynamics(
        system, strict_mip, dynamic_options, certificate_options,
        cyber_options);
    report["cyber_blocked_certificate"] = certificate_json(cyber_blocked);
    report["cyber_assumption"] = {
        {"outage", "restoration command path unavailable"},
        {"scope", "executability gate; no packet-level network simulation"}};

    FMEAOptions fmea_options;
    fmea_options.enable_parallel = false;
    fmea_options.cyber_physical.enabled = true;
    fmea_options.cyber_physical.information_enabled = true;
    fmea_options.cyber_physical.automation_availability = 0.90;
    fmea_options.cyber_physical.automatic_switching_time_hr = 0.05;
    fmea_options.cyber_physical.manual_switching_time_hr = 1.0;
    fmea_options.cyber_physical.intelligent.enabled = true;
    started = Clock::now();
    const auto fmea = run_distribution_fmea(system, fmea_options);
    report["cyber_physical_fmea"] = {
        {"model_scope", fmea.model_scope},
        {"model_limitations", fmea.model_limitations},
        {"contingencies", fmea.n_contingencies},
        {"loss_contingencies", fmea.n_loss_contingencies},
        {"eens_mwh_yr", fmea.eens_mwh_yr},
        {"lole_hr_yr", fmea.lole_hr_yr},
        {"saifi", fmea.distribution_idx.saifi},
        {"saidi_hr", fmea.distribution_idx.saidi},
        {"cyber_delta_eens_mwh_yr",
         fmea.cyber_physical.eens_adjusted_mwh_yr -
             fmea.cyber_physical.eens_perfect_cyber_mwh_yr},
        {"automation_efficacy", fmea.cyber_physical.automation_efficacy},
        {"runtime_sec", elapsed_seconds(started)}};

    const auto graph = hacdcpf::graph::build_power_system_graph(system);
    const auto topology = hacdcpf::graph::analyze_topology(graph);
    std::set<int> bridge_component_ids;
    for (const int edge_id : topology.bridge_edge_ids) {
      if (edge_id >= 0 && edge_id < static_cast<int>(graph.edges.size()))
        bridge_component_ids.insert(graph.edges[static_cast<std::size_t>(edge_id)].comp_index);
    }

    std::map<int, double> fragility;
    for (const auto& risk : hazard.branch_risks) {
      if (risk.branch_kind == ResilienceBranchKind::AC)
        fragility[risk.branch_index] = risk.peak_failure_probability;
    }
    std::map<int, double> annual_eens;
    for (const auto& row : fmea.contingencies) {
      if (row.component_type != "ac_branch") continue;
      if (row.component_index >= 0 &&
          row.component_index < static_cast<int>(system.ac.branches.size())) {
        annual_eens[system.ac.branches[static_cast<std::size_t>(row.component_index)].index] =
            row.eens_contribution;
      }
    }

    std::vector<int> candidate_ids;
    std::vector<double> loading_values;
    std::vector<double> reliability_values;
    std::vector<double> consequence_values;
    std::map<int, double> matched_consequence;
    for (const auto& branch : system.ac.branches) {
      if (!branch.in_service) continue;
      candidate_ids.push_back(branch.index);
      loading_values.push_back(maximum_branch_loading(strict_mip, branch.index));
      reliability_values.push_back(annual_eens[branch.index]);
      DistributionResilienceFault fault(
          branch.index, 0.0, 6.0, "matched weak-link contingency");
      auto matched_options = event_options;
      matched_options.horizon_hours = 12;
      matched_options.faults = {fault};
      matched_options.renewable_profile.assign(12, 0.8);
      const auto matched = run_distribution_resilience_mip_assessment(
          system, matched_options);
      matched_consequence[branch.index] = matched.total_shed_mwh;
      consequence_values.push_back(matched.total_shed_mwh);
    }
    const auto loading_norm = normalized(loading_values);
    const auto reliability_norm = normalized(reliability_values);
    const auto consequence_norm = normalized(consequence_values);

    std::vector<WeakLinkEntityEvidence> weak_evidence;
    for (std::size_t i = 0; i < candidate_ids.size(); ++i) {
      WeakLinkEntityEvidence row;
      row.key = "ac_branch:" + std::to_string(candidate_ids[i]);
      row.name = row.key;
      row.canvas_type = "ac_branch";
      row.canvas_index = candidate_ids[i];
      row.comparison_group = "network";
      row.domain = "AC";
      row.dimensions[0].pressure = loading_norm[i];
      row.dimensions[0].detail = "maximum strict-MIP loading";
      row.dimensions[2].pressure = reliability_norm[i];
      row.dimensions[2].detail = "normalized FMEA EENS contribution";
      row.dimensions[3].pressure = consequence_norm[i];
      row.dimensions[3].detail = "matched strict-MIP weighted ENS";
      weak_evidence.push_back(std::move(row));
    }
    MultidimensionalWeakLinkOptions weak_options;
    weak_options.minimum_dimensions = 2;
    weak_options.top_k = 20;
    const auto weak_links = run_multidimensional_weak_link_assessment(
        weak_evidence, {}, weak_options);

    json weak_rows = json::array();
    for (const auto& row : weak_links.entities) {
      const int id = row.canvas_index;
      weak_rows.push_back(
          {{"branch_index", id},
           {"fragility_probability", fragility[id]},
           {"graph_bridge", bridge_component_ids.count(id) != 0},
           {"max_loading_percent", maximum_branch_loading(strict_mip, id)},
           {"fmea_eens_contribution_mwh_yr", annual_eens[id]},
           {"matched_unserved_mwh", matched_consequence[id]},
           {"multidimensional_severity", row.severity},
           {"consensus", row.consensus},
           {"pareto", row.pareto},
           {"dominant_dimension", row.dominant_dimension},
           {"relation", row.relation}});
    }
    report["weak_links"] = {
        {"method", "separate fragility, Tarjan bridge, FMEA, matched MIP counterfactual, and multidimensional evidence"},
        {"rows", weak_rows}};

    CounterfactualPlanningOptions planning_options;
    planning_options.max_measures = 5;
    planning_options.include_pairs = true;
    planning_options.max_pairs = 10;
    // The economic and carbon evaluators currently enter an AC-only OPF/PF
    // path. Keep the hybrid benchmark on the two domain-safe dimensions.
    planning_options.include_economic = false;
    planning_options.include_carbon = false;
    planning_options.resilience_horizon_hours = 12;
    planning_options.resilience_fault_count = 0;
    if (!hazard.faults.empty()) {
      for (const auto& fault : hazard.faults) {
        if (fault.branch_kind == ResilienceBranchKind::AC)
          planning_options.resilience_fault_branch_indices.push_back(
              fault.branch_index);
      }
    }
    const auto measures = generate_counterfactual_measures(system, planning_options);
    started = Clock::now();
    const auto planning = run_counterfactual_planning_assessment(
        system, measures, planning_options);
    json measure_rows = json::array();
    int best_measure = -1;
    double best_resilience_benefit = -std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < planning.measures.size(); ++i) {
      const auto& row = planning.measures[i];
      const auto benefit = row.benefit.absolute[3];
      measure_rows.push_back(
          {{"id", row.measure.id},
           {"name", row.measure.name},
           {"type", counterfactual_measure_type_name(row.measure.type)},
           {"applied", row.applied},
           {"status", row.apply_status},
           {"economic_benefit_per_year",
            row.benefit.absolute[0].has_value() ? json(*row.benefit.absolute[0])
                                                : json(nullptr)},
           {"carbon_benefit_tco2_per_year",
            row.benefit.absolute[1].has_value() ? json(*row.benefit.absolute[1])
                                                : json(nullptr)},
           {"reliability_benefit_mwh_per_year",
            row.benefit.absolute[2].has_value() ? json(*row.benefit.absolute[2])
                                                : json(nullptr)},
           {"resilience_benefit_mwh",
            benefit.has_value() ? json(*benefit) : json(nullptr)},
           {"relation", row.benefit.relation}});
      if (row.applied && benefit.has_value() && *benefit > best_resilience_benefit) {
        best_resilience_benefit = *benefit;
        best_measure = static_cast<int>(i);
      }
    }
    report["counterfactual_planning"] = {
        {"methodology", planning.methodology},
        {"evaluated_dimensions", {"reliability", "resilience"}},
        {"excluded_dimensions",
         {"economic and carbon paths are AC-only for this evaluator and are not claimed for the hybrid benchmark"}},
        {"evaluated_systems", planning.evaluated_systems},
        {"failed_systems", planning.failed_systems},
        {"runtime_sec", elapsed_seconds(started)},
        {"measures", measure_rows}};

    HybridPowerSystem improved = system;
    std::string generic_best_measure = "none";
    if (best_measure >= 0) {
      const auto& selected = planning.measures[static_cast<std::size_t>(best_measure)];
      generic_best_measure = selected.measure.id;
    }
    report["counterfactual_planning"]["generic_best_measure"] =
        generic_best_measure;

    Storage dc_resilience_storage;
    dc_resilience_storage.index = 201;
    dc_resilience_storage.bus = 102;
    dc_resilience_storage.name = "Critical DC bus resilience storage";
    dc_resilience_storage.in_service = true;
    dc_resilience_storage.controllable = true;
    dc_resilience_storage.p_rated_mw = 0.30;
    dc_resilience_storage.pmax_mw = 0.30;
    dc_resilience_storage.pmin_mw = -0.30;
    dc_resilience_storage.e_rated_mwh = 2.0;
    dc_resilience_storage.e_mwh = 1.8;
    dc_resilience_storage.soc_init = 0.90;
    dc_resilience_storage.soc_min = 0.10;
    dc_resilience_storage.soc_max = 0.95;
    dc_resilience_storage.grid_forming = true;
    improved.dc.storage.push_back(dc_resilience_storage);
    const std::string selected_measure = "dc_storage:DC:102";
    report["counterfactual_planning"]["selected_measure_applied"] = true;
    report["counterfactual_planning"]["selected_measure_status"] =
        "0.30 MW / 2.0 MWh grid-forming DC storage added at the critical DC load bus";
    report["counterfactual_planning"]["selected_measure_basis"] =
        "domain-qualified consequence tracing: all benchmark ENS occurs at critical DC bus 102 after DC branch 101 outages";
    report["counterfactual_planning"]["selected_measure"] = selected_measure;

    std::vector<double> baseline_samples;
    std::vector<double> improved_samples;
    json sample_rows = json::array();
    json convergence_rows = json::array();
    const std::vector<unsigned> convergence_checkpoints = {16, 32, 64, 128, 256};
    constexpr unsigned minimum_convergence_samples = 64;
    constexpr double mean_relative_tolerance = 0.05;
    constexpr double cvar_relative_tolerance = 0.10;
    constexpr int required_consecutive_checkpoints = 2;
    std::optional<RiskStats> previous_baseline_stats;
    std::optional<RiskStats> previous_improved_stats;
    int consecutive_converged_checkpoints = 0;
    bool risk_converged = false;
    for (unsigned sample = 0; sample < convergence_checkpoints.back(); ++sample) {
      const unsigned seed = 203100 + sample;
      const auto sampled_hazard = generate_typhoon_fault_sequence(
          system, hazard_options(seed, true));
      auto sampled_options = restoration_options(sampled_hazard);
      sampled_options.mip.solver = DistributionResilienceMIPSolver::HiGHS;
      sampled_options.mip.shed_penalty_critical = 1000.0;
      sampled_options.mip.shed_penalty_high = 100.0;
      sampled_options.mip.shed_penalty_medium = 10.0;
      sampled_options.mip.shed_penalty_low = 1.0;
      sampled_options.mip.max_time_s = 20;
      const auto baseline = run_distribution_resilience_mip_assessment(
          system, sampled_options);
      const auto intervention = run_distribution_resilience_mip_assessment(
          improved, sampled_options);
      if (baseline.feasible && intervention.feasible) {
        baseline_samples.push_back(baseline.total_shed_mwh);
        improved_samples.push_back(intervention.total_shed_mwh);
      }
      sample_rows.push_back(
          {{"seed", seed},
           {"fault_count", sampled_hazard.faults.size()},
           {"baseline_unserved_mwh", baseline.total_shed_mwh},
           {"improved_unserved_mwh", intervention.total_shed_mwh},
           {"baseline_critical_unserved_mwh",
            tier_unserved_mwh(baseline, sampled_options.time_step_hr, 0)},
           {"improved_critical_unserved_mwh",
            tier_unserved_mwh(intervention, sampled_options.time_step_hr, 0)},
           {"baseline_priority_weighted_objective",
            baseline.weighted_unserved_mwh},
           {"improved_priority_weighted_objective",
            intervention.weighted_unserved_mwh},
           {"baseline_feasible", baseline.feasible},
           {"improved_feasible", intervention.feasible}});

      const unsigned requested_samples = sample + 1;
      if (std::find(convergence_checkpoints.begin(),
                    convergence_checkpoints.end(),
                    requested_samples) == convergence_checkpoints.end()) {
        continue;
      }
      const auto baseline_checkpoint = risk_stats(baseline_samples);
      const auto improved_checkpoint = risk_stats(improved_samples);
      double baseline_mean_change = 0.0;
      double improved_mean_change = 0.0;
      double baseline_cvar_change = 0.0;
      double improved_cvar_change = 0.0;
      bool has_previous = previous_baseline_stats.has_value() &&
                          previous_improved_stats.has_value();
      if (has_previous) {
        baseline_mean_change = relative_change(
            baseline_checkpoint.mean, previous_baseline_stats->mean);
        improved_mean_change = relative_change(
            improved_checkpoint.mean, previous_improved_stats->mean);
        baseline_cvar_change = relative_change(
            baseline_checkpoint.cvar95, previous_baseline_stats->cvar95);
        improved_cvar_change = relative_change(
            improved_checkpoint.cvar95, previous_improved_stats->cvar95);
      }
      const double maximum_mean_change =
          std::max(baseline_mean_change, improved_mean_change);
      const double maximum_cvar_change =
          std::max(baseline_cvar_change, improved_cvar_change);
      const bool checkpoint_within_tolerance =
          has_previous && requested_samples >= minimum_convergence_samples &&
          maximum_mean_change < mean_relative_tolerance &&
          maximum_cvar_change < cvar_relative_tolerance;
      consecutive_converged_checkpoints = checkpoint_within_tolerance
                                              ? consecutive_converged_checkpoints + 1
                                              : 0;
      risk_converged = consecutive_converged_checkpoints >=
                       required_consecutive_checkpoints;
      convergence_rows.push_back(
          {{"requested_samples", requested_samples},
           {"paired_feasible_samples", baseline_samples.size()},
           {"infeasible_pairs", requested_samples - baseline_samples.size()},
           {"baseline_mean_unserved_mwh", baseline_checkpoint.mean},
           {"improved_mean_unserved_mwh", improved_checkpoint.mean},
           {"baseline_var95_unserved_mwh", baseline_checkpoint.var95},
           {"improved_var95_unserved_mwh", improved_checkpoint.var95},
           {"baseline_cvar95_unserved_mwh", baseline_checkpoint.cvar95},
           {"improved_cvar95_unserved_mwh", improved_checkpoint.cvar95},
           {"baseline_mean_relative_change", baseline_mean_change},
           {"improved_mean_relative_change", improved_mean_change},
           {"baseline_cvar_relative_change", baseline_cvar_change},
           {"improved_cvar_relative_change", improved_cvar_change},
           {"maximum_mean_relative_change", maximum_mean_change},
           {"maximum_cvar_relative_change", maximum_cvar_change},
           {"within_tolerance", checkpoint_within_tolerance},
           {"consecutive_within_tolerance", consecutive_converged_checkpoints},
           {"converged", risk_converged}});
      previous_baseline_stats = baseline_checkpoint;
      previous_improved_stats = improved_checkpoint;
      if (risk_converged) break;
    }
    if (baseline_samples.empty())
      throw std::runtime_error("no paired feasible held-out scenarios");
    const auto baseline_risk = risk_stats(baseline_samples);
    const auto improved_risk = risk_stats(improved_samples);
    const double baseline_mean = baseline_risk.mean;
    const double improved_mean = improved_risk.mean;
    const double baseline_var = baseline_risk.var95;
    const double improved_var = improved_risk.var95;
    const unsigned requested_sample_count =
        static_cast<unsigned>(sample_rows.size());
    report["held_out_risk"] = {
        {"samples", sample_rows},
        {"convergence", convergence_rows},
        {"sample_count", baseline_samples.size()},
        {"requested_sample_count", requested_sample_count},
        {"infeasible_pair_count",
         requested_sample_count - baseline_samples.size()},
        {"converged", risk_converged},
        {"convergence_status",
         risk_converged
             ? "mean and CVaR relative-change tolerances met at two consecutive checkpoints"
             : "maximum sample count reached before the declared convergence rule was met"},
        {"minimum_convergence_samples", minimum_convergence_samples},
        {"mean_relative_tolerance", mean_relative_tolerance},
        {"cvar_relative_tolerance", cvar_relative_tolerance},
        {"required_consecutive_checkpoints", required_consecutive_checkpoints},
        {"solver", "HiGHS (paired runs; priority penalties scaled 1000:100:10:1)"},
        {"selected_measure", selected_measure},
        {"baseline_mean_unserved_mwh", baseline_mean},
        {"improved_mean_unserved_mwh", improved_mean},
        {"baseline_var95_unserved_mwh", baseline_var},
        {"improved_var95_unserved_mwh", improved_var},
        {"baseline_cvar95_unserved_mwh", baseline_risk.cvar95},
        {"improved_cvar95_unserved_mwh", improved_risk.cvar95},
        {"scope", "event-conditioned empirical risk over paired synthetic typhoon seeds; not an annual probability estimate"}};

    int first_unproven_step = std::numeric_limits<int>::max();
    for (const auto& row : dae.transitions) {
      if (!(row.certificate.proof_valid &&
            row.certificate.label == CertificateLabel::Safe)) {
        first_unproven_step = std::min(first_unproven_step, row.to_step);
      }
    }
    double admissible_service = 0.0;
    for (const auto& step : strict_mip.steps) {
      if (step.step_index < first_unproven_step)
        admissible_service += step.served_mw * event_options.time_step_hr;
    }
    if (first_unproven_step == std::numeric_limits<int>::max())
      admissible_service = strict_mip.total_served_mwh;
    report["metrics"] = {
        {"ordinary_served_mwh", strict_mip.total_served_mwh},
        {"dynamically_admissible_served_mwh", admissible_service},
        {"dynamic_credit_rule", "zero service credit from the first unproven represented transition onward"},
        {"critical_unserved_mwh",
         tier_unserved_mwh(strict_mip, event_options.time_step_hr, 0)},
        {"critical_ac_unserved_mwh",
         tier_unserved_mwh(strict_mip, event_options.time_step_hr, 0, "AC")},
        {"critical_dc_unserved_mwh",
         tier_unserved_mwh(strict_mip, event_options.time_step_hr, 0, "DC")},
        {"resilience_index", strict_mip.resilience_index},
        {"priority_weighted_unserved_objective",
         strict_mip.weighted_unserved_mwh},
        {"empirical_cvar95_unserved_mwh",
         baseline_risk.cvar95}};

    std::ostringstream service_csv;
    service_csv << "step,hour,demand_mw,served_mw,shed_mw,critical_shed_mw,active_faults,switch_actions\n";
    for (const auto& step : strict_mip.steps) {
      double critical_shed = 0.0;
      for (std::size_t i = 0; i < step.bus_supply_shed_mw.size(); ++i) {
        if (i < step.bus_supply_priority_tier.size() &&
            step.bus_supply_priority_tier[i] == 0)
          critical_shed += step.bus_supply_shed_mw[i];
      }
      service_csv << step.step_index << ',' << step.hour << ','
                  << step.total_demand_mw << ',' << step.served_mw << ','
                  << step.shed_mw << ',' << critical_shed << ','
                  << step.active_faults << ',' << step.switch_actions << '\n';
    }
    write_text(output_dir / "service_curve.csv", service_csv.str());

    std::ostringstream hazard_csv;
    hazard_csv << "domain,branch_index,peak_wind_ms,peak_rain_mm_hr,peak_failure_probability\n";
    for (const auto& risk : hazard.branch_risks)
      hazard_csv << to_string(risk.branch_kind) << ',' << risk.branch_index << ','
                 << risk.peak_wind_ms << ',' << risk.peak_rain_mm_hr << ','
                 << risk.peak_failure_probability << '\n';
    write_text(output_dir / "hazard_branch_risk.csv", hazard_csv.str());

    std::ostringstream weak_csv;
    weak_csv << "branch_index,fragility_probability,graph_bridge,max_loading_percent,fmea_eens_mwh_yr,matched_unserved_mwh,severity,consensus,pareto,dominant_dimension,relation\n";
    for (const auto& row : weak_rows)
      weak_csv << row["branch_index"] << ',' << row["fragility_probability"]
               << ',' << (row["graph_bridge"].get<bool>() ? 1 : 0) << ','
               << row["max_loading_percent"] << ','
               << row["fmea_eens_contribution_mwh_yr"] << ','
               << row["matched_unserved_mwh"] << ','
               << row["multidimensional_severity"] << ',' << row["consensus"]
               << ',' << (row["pareto"].get<bool>() ? 1 : 0) << ','
               << csv_escape(row["dominant_dimension"].get<std::string>()) << ','
               << csv_escape(row["relation"].get<std::string>()) << '\n';
    write_text(output_dir / "weak_link_ranking.csv", weak_csv.str());

    std::ostringstream dynamic_csv;
    dynamic_csv << "from_step,to_step,action_id,event_count,cyber_executable,executable,label,proof_valid,simulation_success,min_frequency_hz,max_abs_rocof_hz_s,min_ac_voltage_pu\n";
    for (const auto& row : report["dynamic_certificate"]["transitions"]) {
      dynamic_csv << row["from_step"] << ',' << row["to_step"] << ','
                  << csv_escape(row["action_id"].get<std::string>()) << ','
                  << row["event_count"] << ','
                  << (row["cyber_executable"].get<bool>() ? 1 : 0) << ','
                  << (row["executable"].get<bool>() ? 1 : 0) << ','
                  << row["label"].get<std::string>() << ','
                  << (row["proof_valid"].get<bool>() ? 1 : 0) << ','
                  << (row["simulation_success"].get<bool>() ? 1 : 0) << ','
                  << (row["min_frequency_hz"].is_null() ? "" : row["min_frequency_hz"].dump()) << ','
                  << row["max_abs_rocof_hz_s"] << ','
                  << (row["min_ac_voltage_pu"].is_null() ? "" : row["min_ac_voltage_pu"].dump()) << '\n';
    }
    write_text(output_dir / "dynamic_transitions.csv", dynamic_csv.str());

    std::ostringstream risk_csv;
    risk_csv << "seed,fault_count,baseline_unserved_mwh,improved_unserved_mwh,baseline_critical_unserved_mwh,improved_critical_unserved_mwh,baseline_priority_weighted_objective,improved_priority_weighted_objective,baseline_feasible,improved_feasible\n";
    for (const auto& row : sample_rows)
      risk_csv << row["seed"] << ',' << row["fault_count"] << ','
               << row["baseline_unserved_mwh"] << ','
               << row["improved_unserved_mwh"] << ','
               << row["baseline_critical_unserved_mwh"] << ','
               << row["improved_critical_unserved_mwh"] << ','
               << row["baseline_priority_weighted_objective"] << ','
               << row["improved_priority_weighted_objective"] << ','
               << (row["baseline_feasible"].get<bool>() ? 1 : 0) << ','
               << (row["improved_feasible"].get<bool>() ? 1 : 0) << '\n';
    write_text(output_dir / "held_out_risk.csv", risk_csv.str());

    std::ostringstream convergence_csv;
    convergence_csv << "requested_samples,paired_feasible_samples,infeasible_pairs,baseline_mean_unserved_mwh,improved_mean_unserved_mwh,baseline_var95_unserved_mwh,improved_var95_unserved_mwh,baseline_cvar95_unserved_mwh,improved_cvar95_unserved_mwh,baseline_mean_relative_change,improved_mean_relative_change,baseline_cvar_relative_change,improved_cvar_relative_change,maximum_mean_relative_change,maximum_cvar_relative_change,within_tolerance,consecutive_within_tolerance,converged\n";
    for (const auto& row : convergence_rows) {
      convergence_csv
          << row["requested_samples"] << ','
          << row["paired_feasible_samples"] << ','
          << row["infeasible_pairs"] << ','
          << row["baseline_mean_unserved_mwh"] << ','
          << row["improved_mean_unserved_mwh"] << ','
          << row["baseline_var95_unserved_mwh"] << ','
          << row["improved_var95_unserved_mwh"] << ','
          << row["baseline_cvar95_unserved_mwh"] << ','
          << row["improved_cvar95_unserved_mwh"] << ','
          << row["baseline_mean_relative_change"] << ','
          << row["improved_mean_relative_change"] << ','
          << row["baseline_cvar_relative_change"] << ','
          << row["improved_cvar_relative_change"] << ','
          << row["maximum_mean_relative_change"] << ','
          << row["maximum_cvar_relative_change"] << ','
          << (row["within_tolerance"].get<bool>() ? 1 : 0) << ','
          << row["consecutive_within_tolerance"] << ','
          << (row["converged"].get<bool>() ? 1 : 0) << '\n';
    }
    write_text(output_dir / "risk_convergence.csv", convergence_csv.str());

#if 0
    std::ostringstream tex;
    tex << std::fixed << std::setprecision(3);
    tex << "% Generated by distribution_resilience_review_case; do not edit by hand.\n";
    tex << "\\begin{table*}[t]\n\\centering\n"
           "\\caption{End-to-end review benchmark: computed results and declared scope.}\n"
           "\\label{tab:review-simulation-summary}\n"
           "\\begin{tabular}{p{0.18\\textwidth}p{0.18\\textwidth}p{0.22\\textwidth}p{0.34\\textwidth}}\n"
           "\\toprule\nLayer & Method & Computed result & Interpretation boundary \\\\\n+           "\\midrule\n";
    tex << "Pre-event & 24-h UC with cyclic SOC & feasible="
        << (pre_event.uc_schedule.feasible ? "yes" : "no")
        << ", gap=" << pre_event.uc_schedule.mip_gap
        << " & Establishes a model-reachable day-ahead schedule; controller trim and fuel logistics are not represented. \\\\\n+        ";
    tex << "Hazard & Holland field and segment fragility & " << hazard.faults.size()
        << " faults, " << hazard.generated_segments.size()
        << " segments & Conditional synthetic event with fixed seed; not an annual-frequency calibration. \\\\\n+        ";
    tex << "Restoration & Strict hybrid AC/DC MIP & ENS=" << strict_mip.total_shed_mwh
        << " MWh, $R$=" << strict_mip.resilience_index
        << " & Active-power LinDistFlow/transport model; losses, reactive power, protection, and transients are excluded. \\\\\n+        ";
    tex << "Dynamic gate & L3 DAE threshold oracle & " << dae.transitions.size()
        << " transitions, proof=" << (dae.proof_valid ? "yes" : "no")
        << " & Scenario/horizon/model-specific threshold certificate, not a global stability proof. \\\\\n+        ";
    tex << "Information failure & Cyber executability gate and Level-1 FMEA & $\\Delta$EENS="
        << fmea.cyber_physical.delta_eens_mwh_yr
        << " MWh/yr & No communication topology, packet traffic, relay pickup, or FRT trajectory. \\\\\n+        ";
    tex << "Held-out risk & 12 matched stochastic seeds & mean=" << baseline_mean
        << ", CVaR$_{0.95}$=" << cvar(baseline_samples, baseline_var)
        << " weighted MWh & Empirical event-conditioned risk; sample size is illustrative and not a converged tail estimate. \\\\\n+        ";
    tex << "\\bottomrule\n\\end{tabular}\n\\end{table*}\n\n";

    tex << "\\begin{table}[t]\n\\centering\n"
           "\\caption{Weak-link rankings from non-equivalent evidence.}\n"
           "\\label{tab:review-weak-links}\n"
           "\\begin{tabular}{rrrrrr}\n\\toprule\n"
           "Branch & Fragility & Bridge & FMEA & Matched ENS & Severity \\\\\n+           "\\midrule\n";
    for (const auto& row : weak_rows) {
      tex << row["branch_index"].get<int>() << " & "
          << row["fragility_probability"].get<double>() << " & "
          << (row["graph_bridge"].get<bool>() ? "yes" : "no") << " & "
          << row["fmea_eens_contribution_mwh_yr"].get<double>() << " & "
          << row["matched_weighted_unserved_mwh"].get<double>() << " & "
          << row["multidimensional_severity"].get<double>() << " \\\\\n+          ";
    }
    tex << "\\bottomrule\n\\end{tabular}\n\\end{table}\n\n";

    tex << "\\begin{table}[t]\n\\centering\n"
           "\\caption{Held-out intervention check using matched hazard seeds.}\n"
           "\\label{tab:review-heldout-risk}\n"
           "\\begin{tabular}{lrr}\n\\toprule\nMetric & Baseline & Intervention \\\\\n+           "\\midrule\n"
        << "Mean weighted ENS (MWh) & " << baseline_mean << " & "
        << improved_mean << " \\\\\n+        " << "VaR$_{0.95}$ (MWh) & " << baseline_var << " & "
        << improved_var << " \\\\\n+        " << "CVaR$_{0.95}$ (MWh) & " << cvar(baseline_samples, baseline_var)
        << " & " << cvar(improved_samples, improved_var) << " \\\\\n+        " << "\\bottomrule\n\\end{tabular}\n\\end{table}\n";
#endif
    std::ostringstream generated_tex;
    generated_tex << std::fixed << std::setprecision(3);
    generated_tex << R"LATEX(% Generated by distribution_resilience_review_case; do not edit by hand.
\begin{table*}[t]
\centering
\caption{End-to-end review benchmark: computed results and declared scope.}
\label{tab:review-simulation-summary}
\begin{tabularx}{\textwidth}{@{}Y{0.18\textwidth}Y{0.18\textwidth}Y{0.22\textwidth}Z@{}}
\toprule
Layer & Method & Computed result & Interpretation boundary \\
\midrule
)LATEX";
    generated_tex << "Pre-event & 24-h UC with cyclic SOC & feasible="
                  << (pre_event.uc_schedule.feasible ? "yes" : "no")
                  << ", gap=" << pre_event.uc_schedule.mip_gap
                  << " & Model-reachable schedule; controller trim and fuel logistics are excluded. \\\\"
                  << '\n';
    generated_tex << "Hazard & Holland field and segment fragility & "
                  << hazard.faults.size() << " faults, "
                  << hazard.generated_segments.size()
                  << " segments & Fixed-seed conditional event, not annual-frequency calibration. \\\\"
                  << '\n';
    generated_tex << "Restoration & Strict hybrid AC/DC MIP & ENS="
                  << strict_mip.total_shed_mwh << " MWh, $R$="
                  << strict_mip.resilience_index
                  << " & Active-power model; losses, reactive power, protection, and transients are excluded. \\\\"
                  << '\n';
    generated_tex << "Dynamic gate & L3 DAE threshold oracle & "
                  << dae.transitions.size() << " transitions, proof="
                  << (dae.proof_valid ? "yes" : "no")
                  << " & Scenario/horizon/model-specific threshold certificate, not global stability. \\\\"
                  << '\n';
    generated_tex << "Information failure & Executability gate and Level-1 FMEA & $\\Delta$EENS="
                  << (fmea.cyber_physical.eens_adjusted_mwh_yr -
                      fmea.cyber_physical.eens_perfect_cyber_mwh_yr)
                  << " MWh/yr & No packet network, relay pickup, or FRT trajectory. \\\\"
                  << '\n';
    generated_tex << "Held-out risk & " << requested_sample_count
                  << " matched stochastic seeds & mean="
                  << baseline_mean << ", CVaR$_{0.95}$="
                  << baseline_risk.cvar95
                  << " MWh ENS & Event-conditioned empirical risk; convergence="
                  << (risk_converged ? "yes" : "no")
                  << ", not an annual-risk estimate. \\\\"
                  << '\n';
    generated_tex << R"LATEX(\bottomrule
\end{tabularx}
\end{table*}

\begin{table}[t]
\centering
\caption{Weak-link rankings from non-equivalent evidence.}
\label{tab:review-weak-links}
\begin{tabular}{rrrrrr}
\toprule
Branch & Fragility & Bridge & FMEA & Matched ENS & Severity \\
\midrule
)LATEX";
    for (const auto& row : weak_rows) {
      generated_tex << row["branch_index"].get<int>() << " & "
                    << row["fragility_probability"].get<double>() << " & "
                    << (row["graph_bridge"].get<bool>() ? "yes" : "no")
                    << " & "
                    << row["fmea_eens_contribution_mwh_yr"].get<double>()
                    << " & "
                    << row["matched_unserved_mwh"].get<double>()
                    << " & "
                    << row["multidimensional_severity"].get<double>()
                    << " \\\\" << '\n';
    }
    generated_tex << R"LATEX(\bottomrule
\end{tabular}
\end{table}

\begin{table}[t]
\centering
\caption{Held-out intervention check using matched hazard seeds.}
\label{tab:review-heldout-risk}
\begin{tabular}{lrr}
\toprule
Metric & Baseline & Intervention \\
\midrule
)LATEX";
    generated_tex << "Mean ENS (MWh) & " << baseline_mean << " & "
                  << improved_mean << " \\\\" << '\n'
                  << "VaR$_{0.95}$ (MWh) & " << baseline_var << " & "
                  << improved_var << " \\\\" << '\n'
                  << "CVaR$_{0.95}$ (MWh) & "
                  << baseline_risk.cvar95 << " & "
                  << improved_risk.cvar95 << " \\\\" << '\n'
                  << "Paired feasible/requested & " << baseline_samples.size()
                  << "/" << requested_sample_count << " & "
                  << baseline_samples.size() << "/" << requested_sample_count
                  << " \\\\" << '\n'
                  << "Convergence rule met & "
                  << (risk_converged ? "yes" : "no") << " & "
                  << (risk_converged ? "yes" : "no") << " \\\\" << '\n';
    generated_tex << R"LATEX(\bottomrule
\end{tabular}
\end{table}
)LATEX";
    write_text(output_dir.parent_path() / "generated_simulation_results.tex",
               generated_tex.str());

    write_text(output_dir / "review_case_results.json", report.dump(2) + "\n");
    std::cout << report.dump(2) << '\n';
    return strict_mip.feasible && pre_event.uc_schedule.feasible ? 0 : 2;
  } catch (const std::exception& error) {
    std::cerr << "distribution resilience review case failed: " << error.what()
              << '\n';
    return 1;
  }
}
