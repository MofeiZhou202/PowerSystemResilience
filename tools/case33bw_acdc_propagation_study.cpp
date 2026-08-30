#include <algorithm>
#include <cmath>
#include <complex>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "hacdcpf/dynamics/dynamics.hpp"
#include "hacdcpf/io/case_builders.hpp"

using json = nlohmann::json;
using namespace hacdcpf;
using namespace hacdcpf::dynamics;

namespace {

constexpr double kFaultTimeS = 0.20;
constexpr double kClearTimeS = 0.40;
constexpr double kEndTimeS = 0.80;
constexpr int kAcFaultBus = 18;
constexpr int kDcFaultBus = 2;
constexpr int kObservedVsc = 2;
// Detection gates fixed before the runs and documented in
// docs/modules/reliability/chapters/theory_reliability_protection_frt.tex,
// Sec. "case33bw AC/DC recovery-entry capability audit". Both correspond to
// 1e-4 pu on this 10 MVA case (1e-3 MW / 10 MVA = 1e-4 pu), well above the
// required 1e-9 algebraic current-balance tolerance.
constexpr double kVoltagePropagationThresholdPu = 1e-4;
constexpr double kPowerPropagationThresholdMw = 1e-3;
constexpr double kEquilibriumDriftThresholdPu = 1e-4;
constexpr double kEnhancedPowerDeviationMinimumMw = 0.05;
constexpr double kLegacyPowerDeviationMaximumMw = 1e-6;
constexpr double kStepPowerRelativeTolerance = 0.05;
constexpr double kBlockTimeToleranceS = 0.003;

enum class ScenarioKind {
  Baseline,
  AcFault,
  DcFaultLegacy,
  DcFaultEnhanced
};

struct Sample {
  double time_s{0.0};
  double ac18_voltage_pu{0.0};
  double dc1_voltage_pu{0.0};
  double dc2_voltage_pu{0.0};
  double vsc1_p_mw{0.0};
  double vsc1_in_service{0.0};
  double vsc1_protection_tripped{0.0};
  double vsc1_restore_scale{0.0};
  double vsc1_vdc_link_pu{0.0};
  double vsc2_p_mw{0.0};
  double vsc2_in_service{0.0};
  double vsc2_protection_tripped{0.0};
  double vsc2_restore_scale{0.0};
  double vsc2_vdc_link_pu{0.0};
  double vsc2_dc_active_power_scale{1.0};
  double vsc2_dc_undervoltage_timer_s{0.0};
  double vsc2_dc_undervoltage_blocked{0.0};
};

struct ScenarioResult {
  ScenarioKind kind{ScenarioKind::Baseline};
  double dt_s{0.0};
  bool success{false};
  std::string message;
  bool power_flow_converged{false};
  double power_flow_residual{std::numeric_limits<double>::quiet_NaN()};
  double max_post_event_algebraic_residual{0.0};
  std::vector<Sample> samples;
  std::vector<DynamicAppliedEventRecord> events;
};

std::string scenario_name(ScenarioKind kind) {
  switch (kind) {
    case ScenarioKind::Baseline: return "no_fault_baseline";
    case ScenarioKind::AcFault: return "ac_bus_18_fault";
    case ScenarioKind::DcFaultLegacy: return "dc_bus_2_fault_legacy";
    case ScenarioKind::DcFaultEnhanced: return "dc_bus_2_fault_enhanced";
  }
  return "unknown";
}

double output_value(const DynamicDeviceOutput& output,
                    const std::string& key,
                    double fallback = 0.0) {
  const auto found = output.values.find(key);
  return found == output.values.end() ? fallback : found->second;
}

const DynamicDeviceOutput* vsc_output(const DynamicSnapshot& snapshot,
                                      int component_index) {
  const auto found = std::find_if(
      snapshot.device_outputs.begin(), snapshot.device_outputs.end(),
      [component_index](const DynamicDeviceOutput& output) {
        return output.component_index == component_index &&
               (output.source_type == "vsc_grid_following" ||
                output.source_type == "vsc_grid_forming");
      });
  return found == snapshot.device_outputs.end() ? nullptr : &*found;
}

double positive_sequence_magnitude(const DynamicSnapshot& snapshot,
                                   int ac_bus_pos) {
  if (ac_bus_pos < 0 || 3 * ac_bus_pos + 2 >= snapshot.vac_abc.size()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  // Fortescue positive-sequence transform. See Kundur, Power System Stability
  // and Control (1994), Sec. 2.3; voltages are phase-node phasors in pu.
  const std::complex<double> a = std::polar(1.0, 2.0 * std::acos(-1.0) / 3.0);
  const std::complex<double> v1 =
      (snapshot.vac_abc[3 * ac_bus_pos] +
       a * snapshot.vac_abc[3 * ac_bus_pos + 1] +
       a * a * snapshot.vac_abc[3 * ac_bus_pos + 2]) /
      3.0;
  return std::abs(v1);
}

HybridPowerSystem make_study_system(bool enable_dc_fault_control = false) {
  HybridPowerSystem system = io::build_case33bw_acdc();
  if (system.vsc_converters.size() != 2 || system.dc.buses.size() != 2 ||
      system.dc.branches.size() != 1) {
    throw std::runtime_error(
        "build_case33bw_acdc no longer has the expected 2-VSC, 2-DC-bus overlay");
  }

  DCLoad load;
  load.index = 1;
  load.bus = kDcFaultBus;
  load.name = "Propagation-study DC load";
  load.p_mw = 1.0;
  load.in_service = true;
  system.dc.loads = {load};

  auto& station_1 = system.vsc_converters[0];
  station_1.control_mode = ConverterMode::VDC_Q;
  station_1.k_vdc = 25.0;
  station_1.p_set_mw = 0.0;
  station_1.p_schedule_mw = 0.0;
  station_1.q_set_mvar = 0.0;
  station_1.p_rated_mw = 3.0;
  station_1.pmax_mw = 3.0;
  station_1.pmin_mw = -3.0;
  station_1.i_ac_max_pu = 0.30;
  station_1.dynamic_model.standard = "IEEE/NERC";
  station_1.dynamic_model.model_name = "REGC_REEC_GFL_Subset";

  auto& station_2 = system.vsc_converters[1];
  station_2.control_mode = ConverterMode::PQ_MODE;
  station_2.p_set_mw = -1.05;
  station_2.p_schedule_mw = -1.05;
  station_2.q_set_mvar = 0.0;
  station_2.p_rated_mw = 3.0;
  station_2.pmax_mw = 3.0;
  station_2.pmin_mw = -3.0;
  station_2.i_ac_max_pu = 0.30;
  station_2.dynamic_model.standard = "IEEE/NERC";
  station_2.dynamic_model.model_name = "REGC_REEC_GFL_Subset";
  station_2.dynamic_model.parameters = {
      {"ieee1547_enabled", 1.0},
      {"ieee1547_category", 2.0},
      {"ieee1547_allow_reconnect", 0.0},
      {"current_limit_pu", station_2.i_ac_max_pu},
  };
  if (enable_dc_fault_control) {
    station_2.dynamic_model.parameters["dc_fault_control_enabled"] = 1.0;
    station_2.dynamic_model.parameters["dc_active_power_derate_start_pu"] = 0.95;
    station_2.dynamic_model.parameters["dc_undervoltage_block_pu"] = 0.75;
    station_2.dynamic_model.parameters["dc_undervoltage_block_delay_s"] = 0.02;
  }
  return system;
}

DynamicEvent fault_event(ScenarioKind kind) {
  DynamicEvent event;
  event.time_s = kFaultTimeS;
  event.type = DynamicEventType::FaultShunt;
  event.bus = kind == ScenarioKind::AcFault ? kAcFaultBus : kDcFaultBus;
  event.component_type = kind == ScenarioKind::AcFault ? "AC" : "DC";
  event.label = scenario_name(kind);
  event.params["r_pu"] = 0.02;
  event.params["x_pu"] = 0.0;
  return event;
}

DynamicEvent clear_event(ScenarioKind kind) {
  DynamicEvent event;
  event.time_s = kClearTimeS;
  event.type = DynamicEventType::ClearFault;
  event.bus = kind == ScenarioKind::AcFault ? kAcFaultBus : kDcFaultBus;
  event.component_type = kind == ScenarioKind::AcFault ? "AC" : "DC";
  event.label = scenario_name(kind) + " clearance";
  return event;
}

ScenarioResult run_scenario(ScenarioKind kind, double dt_s) {
  ScenarioResult out;
  out.kind = kind;
  out.dt_s = dt_s;

  DynamicSolverOptions options;
  options.solver_type = DynamicSolverType::MassMatrixDae;
  options.dae_step_method = DynamicDaeStepMethod::BackwardEuler;
  options.t_end_s = kEndTimeS;
  options.dt_s = dt_s;
  options.run_power_flow_initialization = true;
  options.use_consistent_dynamic_initialization = true;
  options.dynamic_dc_link = true;
  options.dc_link_capacitance_s = 0.10;
  options.dc_link_coupling_conductance_pu = 20.0;
  options.source_stiffness_pu = 100.0;
  options.enable_der_protection = true;
  options.localize_der_protection_events = true;
  options.protection_event_time_tol_s = 1e-6;
  options.post_event_algebraic_residual_tol = 1e-7;
  options.algebraic_network_tol = 1e-9;
  options.algebraic_network_max_iters = 25;
  options.record_every_step = true;
  options.record_device_outputs = true;
  options.enforce_voltage_health_check = false;

  DynamicModelBuilder builder;
  DynamicSystem dynamic = builder.build(
      make_study_system(kind == ScenarioKind::DcFaultEnhanced), options);
  const int ac18_pos = dynamic.network.acBusPosition(kAcFaultBus);
  const int dc1_pos = dynamic.network.dcBusPosition(1);
  const int dc2_pos = dynamic.network.dcBusPosition(2);
  if (ac18_pos < 0 || dc1_pos < 0 || dc2_pos < 0) {
    throw std::runtime_error("study observation buses were lost during canonical projection");
  }
  if (kind != ScenarioKind::Baseline) {
    dynamic.events.push_back(fault_event(kind));
    dynamic.events.push_back(clear_event(kind));
  }

  const DynamicResults result = DynamicSolver{}.solve(dynamic);
  out.success = result.success;
  out.message = result.message;
  out.power_flow_converged = result.initialization.power_flow_converged;
  out.power_flow_residual = result.initialization.residual;
  out.max_post_event_algebraic_residual =
      result.max_post_event_algebraic_residual;
  out.events = result.applied_event_records;
  for (const auto& snapshot : result.snapshots) {
    const DynamicDeviceOutput* station_1 = vsc_output(snapshot, 1);
    const DynamicDeviceOutput* station_2 = vsc_output(snapshot, kObservedVsc);
    if (station_1 == nullptr || station_2 == nullptr) continue;
    Sample sample;
    sample.time_s = snapshot.time_s;
    sample.ac18_voltage_pu = positive_sequence_magnitude(snapshot, ac18_pos);
    sample.dc1_voltage_pu = snapshot.vdc[dc1_pos];
    sample.dc2_voltage_pu = snapshot.vdc[dc2_pos];
    sample.vsc1_p_mw = output_value(*station_1, "p_mw");
    sample.vsc1_in_service = output_value(*station_1, "in_service");
    sample.vsc1_protection_tripped =
        output_value(*station_1, "protection_tripped");
    sample.vsc1_restore_scale =
        output_value(*station_1, "protection_restore_scale", 1.0);
    sample.vsc1_vdc_link_pu =
        output_value(*station_1, "vdc_link_pu", snapshot.vdc[dc1_pos]);
    sample.vsc2_p_mw = output_value(*station_2, "p_mw");
    sample.vsc2_in_service = output_value(*station_2, "in_service");
    sample.vsc2_protection_tripped =
        output_value(*station_2, "protection_tripped");
    sample.vsc2_restore_scale =
        output_value(*station_2, "protection_restore_scale", 1.0);
    sample.vsc2_vdc_link_pu =
        output_value(*station_2, "vdc_link_pu", snapshot.vdc[dc2_pos]);
    sample.vsc2_dc_active_power_scale =
        output_value(*station_2, "dc_active_power_scale", 1.0);
    sample.vsc2_dc_undervoltage_timer_s =
        output_value(*station_2, "dc_undervoltage_timer_s");
    sample.vsc2_dc_undervoltage_blocked =
        output_value(*station_2, "dc_undervoltage_blocked");
    out.samples.push_back(sample);
  }
  if (out.success && out.samples.empty()) {
    out.success = false;
    out.message = "VSC2 dynamic output is absent from all snapshots";
  }
  return out;
}

double interpolate(const std::vector<Sample>& samples,
                   double time_s,
                   double Sample::*member) {
  if (samples.empty()) return std::numeric_limits<double>::quiet_NaN();
  const auto upper = std::lower_bound(
      samples.begin(), samples.end(), time_s,
      [](const Sample& sample, double time) { return sample.time_s < time; });
  if (upper == samples.begin()) return (*upper).*member;
  if (upper == samples.end()) return samples.back().*member;
  const auto lower = std::prev(upper);
  const double width = upper->time_s - lower->time_s;
  if (width <= 0.0) return (*lower).*member;
  const double alpha = (time_s - lower->time_s) / width;
  return (1.0 - alpha) * ((*lower).*member) + alpha * ((*upper).*member);
}

double max_deviation(const ScenarioResult& disturbed,
                     const ScenarioResult& baseline,
                     double Sample::*member) {
  double maximum = 0.0;
  for (const auto& sample : disturbed.samples) {
    if (sample.time_s + 1e-12 < kFaultTimeS) continue;
    const double reference = interpolate(baseline.samples, sample.time_s, member);
    maximum = std::max(maximum, std::abs(sample.*member - reference));
  }
  return maximum;
}

double max_baseline_drift(const ScenarioResult& baseline,
                          double Sample::*member) {
  if (baseline.samples.empty()) return std::numeric_limits<double>::infinity();
  const double initial = baseline.samples.front().*member;
  double maximum = 0.0;
  for (const auto& sample : baseline.samples) {
    maximum = std::max(maximum, std::abs(sample.*member - initial));
  }
  return maximum;
}

double dc_undervoltage_block_time(const ScenarioResult& scenario) {
  const auto found = std::find_if(
      scenario.events.begin(), scenario.events.end(),
      [](const DynamicAppliedEventRecord& event) {
        return event.label.find("DC-link undervoltage block") !=
               std::string::npos;
      });
  return found == scenario.events.end()
             ? std::numeric_limits<double>::quiet_NaN()
             : found->time_s;
}

double relative_difference(double lhs, double rhs) {
  return std::abs(lhs - rhs) /
         std::max({1e-12, std::abs(lhs), std::abs(rhs)});
}

json nullable_number(double value) {
  return std::isfinite(value) ? json(value) : json(nullptr);
}

json scenario_json(const ScenarioResult& scenario) {
  json events = json::array();
  for (const auto& event : scenario.events) {
    events.push_back({
        {"time_s", event.time_s},
        {"type", event.type},
        {"component_type", event.component_type},
        {"component_index", event.component_index},
        {"label", event.label},
    });
  }
  const Sample* terminal = scenario.samples.empty() ? nullptr : &scenario.samples.back();
  return {
      {"name", scenario_name(scenario.kind)},
      {"dt_s", scenario.dt_s},
      {"success", scenario.success},
      {"message", scenario.message},
      {"power_flow_converged", scenario.power_flow_converged},
      {"power_flow_residual", nullable_number(scenario.power_flow_residual)},
      {"max_post_event_algebraic_residual",
       scenario.max_post_event_algebraic_residual},
      {"snapshot_count", scenario.samples.size()},
      {"terminal_vsc2_in_service",
       terminal == nullptr ? json(nullptr) : json(terminal->vsc2_in_service > 0.5)},
      {"terminal_vsc2_protection_tripped",
       terminal == nullptr ? json(nullptr)
                           : json(terminal->vsc2_protection_tripped > 0.5)},
      {"terminal_vsc2_p_mw",
       terminal == nullptr ? json(nullptr) : json(terminal->vsc2_p_mw)},
      {"terminal_vsc2_vdc_link_pu",
       terminal == nullptr ? json(nullptr) : json(terminal->vsc2_vdc_link_pu)},
      {"terminal_vsc2_dc_active_power_scale",
       terminal == nullptr ? json(nullptr)
                           : json(terminal->vsc2_dc_active_power_scale)},
      {"terminal_vsc2_dc_undervoltage_blocked",
       terminal == nullptr
           ? json(nullptr)
           : json(terminal->vsc2_dc_undervoltage_blocked > 0.5)},
      {"dc_undervoltage_block_time_s",
       nullable_number(dc_undervoltage_block_time(scenario))},
      {"applied_events", std::move(events)},
  };
}

void write_csv(const std::filesystem::path& path,
               const std::vector<ScenarioResult>& scenarios) {
  std::ofstream file(path);
  if (!file) throw std::runtime_error("cannot open trajectory CSV: " + path.string());
  file << "scenario,dt_s,time_s,ac18_voltage_pu,dc1_voltage_pu,dc2_voltage_pu,"
          "vsc1_p_mw,vsc1_in_service,vsc1_protection_tripped,"
          "vsc1_restore_scale,vsc1_vdc_link_pu,"
          "vsc2_p_mw,vsc2_in_service,vsc2_protection_tripped,"
          "vsc2_restore_scale,vsc2_vdc_link_pu,"
          "vsc2_dc_active_power_scale,vsc2_dc_undervoltage_timer_s,"
          "vsc2_dc_undervoltage_blocked\n";
  file << std::setprecision(17);
  for (const auto& scenario : scenarios) {
    for (const auto& sample : scenario.samples) {
      file << scenario_name(scenario.kind) << ',' << scenario.dt_s << ','
           << sample.time_s << ','
           << sample.ac18_voltage_pu << ',' << sample.dc1_voltage_pu << ','
           << sample.dc2_voltage_pu << ',' << sample.vsc1_p_mw << ','
           << sample.vsc1_in_service << ',' << sample.vsc1_protection_tripped
           << ',' << sample.vsc1_restore_scale << ','
           << sample.vsc1_vdc_link_pu << ',' << sample.vsc2_p_mw << ','
           << sample.vsc2_in_service << ',' << sample.vsc2_protection_tripped
           << ',' << sample.vsc2_restore_scale << ','
           << sample.vsc2_vdc_link_pu << ','
           << sample.vsc2_dc_active_power_scale << ','
           << sample.vsc2_dc_undervoltage_timer_s << ','
           << sample.vsc2_dc_undervoltage_blocked << '\n';
    }
  }
}

}  // namespace

int main(int argc, char** argv) {
  try {
    bool smoke = false;
    std::filesystem::path output_path;
    std::filesystem::path csv_path;
    for (int i = 1; i < argc; ++i) {
      const std::string arg = argv[i];
      if (arg == "--smoke") {
        smoke = true;
      } else if (arg == "--output" && i + 1 < argc) {
        output_path = argv[++i];
      } else if (arg == "--csv" && i + 1 < argc) {
        csv_path = argv[++i];
      } else {
        throw std::invalid_argument(
            "usage: case33bw_acdc_propagation_study [--smoke] "
            "[--output report.json] [--csv trajectories.csv]");
      }
    }

    const double dt_s = 0.002;
    std::vector<ScenarioResult> scenarios;
    scenarios.push_back(run_scenario(ScenarioKind::Baseline, dt_s));
    scenarios.push_back(run_scenario(ScenarioKind::AcFault, dt_s));
    scenarios.push_back(run_scenario(ScenarioKind::DcFaultLegacy, dt_s));
    scenarios.push_back(run_scenario(ScenarioKind::DcFaultEnhanced, dt_s));
    scenarios.push_back(run_scenario(ScenarioKind::DcFaultEnhanced, 0.001));
    scenarios.push_back(run_scenario(ScenarioKind::DcFaultEnhanced, 0.005));
    const auto& baseline = scenarios[0];
    const auto& ac_fault = scenarios[1];
    const auto& dc_fault_legacy = scenarios[2];
    const auto& dc_fault_enhanced_2ms = scenarios[3];
    const auto& dc_fault_enhanced_1ms = scenarios[4];
    const auto& dc_fault_enhanced_5ms = scenarios[5];

    const bool simulations_valid = std::all_of(
        scenarios.begin(), scenarios.end(),
        [](const ScenarioResult& result) { return result.success; });
    const double baseline_ac_drift =
        max_baseline_drift(baseline, &Sample::ac18_voltage_pu);
    const double baseline_dc_drift =
        max_baseline_drift(baseline, &Sample::dc2_voltage_pu);
    const bool equilibrium_valid =
        baseline_ac_drift <= kEquilibriumDriftThresholdPu &&
        baseline_dc_drift <= kEquilibriumDriftThresholdPu;

    const double ac_to_dc_v1 =
        max_deviation(ac_fault, baseline, &Sample::dc1_voltage_pu);
    const double ac_to_dc_v2 =
        max_deviation(ac_fault, baseline, &Sample::dc2_voltage_pu);
    const double ac_to_dc_max = std::max(ac_to_dc_v1, ac_to_dc_v2);
    const bool ac_to_dc = simulations_valid && equilibrium_valid &&
                          ac_to_dc_max >= kVoltagePropagationThresholdPu;

    const double legacy_dc_to_ac_voltage =
        max_deviation(dc_fault_legacy, baseline, &Sample::ac18_voltage_pu);
    const double legacy_dc_to_ac_power =
        max_deviation(dc_fault_legacy, baseline, &Sample::vsc2_p_mw);
    const bool legacy_identity =
        legacy_dc_to_ac_power <= kLegacyPowerDeviationMaximumMw;

    const auto enhanced_voltage = [&](const ScenarioResult& scenario) {
      return max_deviation(scenario, baseline, &Sample::ac18_voltage_pu);
    };
    const auto enhanced_power = [&](const ScenarioResult& scenario) {
      return max_deviation(scenario, baseline, &Sample::vsc2_p_mw);
    };
    const double enhanced_voltage_1ms =
        enhanced_voltage(dc_fault_enhanced_1ms);
    const double enhanced_voltage_2ms =
        enhanced_voltage(dc_fault_enhanced_2ms);
    const double enhanced_voltage_5ms =
        enhanced_voltage(dc_fault_enhanced_5ms);
    const double enhanced_power_1ms =
        enhanced_power(dc_fault_enhanced_1ms);
    const double enhanced_power_2ms =
        enhanced_power(dc_fault_enhanced_2ms);
    const double enhanced_power_5ms =
        enhanced_power(dc_fault_enhanced_5ms);
    const auto propagated = [](double voltage, double power) {
      return voltage >= kVoltagePropagationThresholdPu ||
             power >= kPowerPropagationThresholdMw;
    };
    const bool dc_to_ac = simulations_valid && equilibrium_valid &&
        enhanced_voltage_2ms >= kVoltagePropagationThresholdPu &&
        enhanced_power_2ms >= kEnhancedPowerDeviationMinimumMw;
    const bool step_classification_consistent =
        propagated(enhanced_voltage_1ms, enhanced_power_1ms) &&
        propagated(enhanced_voltage_2ms, enhanced_power_2ms) &&
        propagated(enhanced_voltage_5ms, enhanced_power_5ms);
    const double power_relative_difference_1ms_2ms =
        relative_difference(enhanced_power_1ms, enhanced_power_2ms);
    const bool power_step_converged =
        power_relative_difference_1ms_2ms <= kStepPowerRelativeTolerance;

    const double block_time_1ms =
        dc_undervoltage_block_time(dc_fault_enhanced_1ms);
    const double block_time_2ms =
        dc_undervoltage_block_time(dc_fault_enhanced_2ms);
    const double block_time_5ms =
        dc_undervoltage_block_time(dc_fault_enhanced_5ms);
    const bool all_block_times_resolved =
        std::isfinite(block_time_1ms) && std::isfinite(block_time_2ms) &&
        std::isfinite(block_time_5ms);
    const double block_time_spread_s = all_block_times_resolved
        ? std::max({block_time_1ms, block_time_2ms, block_time_5ms}) -
              std::min({block_time_1ms, block_time_2ms, block_time_5ms})
        : std::numeric_limits<double>::infinity();
    const bool block_time_converged =
        all_block_times_resolved && block_time_spread_s <= kBlockTimeToleranceS;

    const HybridPowerSystem binary_system = make_study_system(false);
    const Sample* dae_terminal =
        ac_fault.samples.empty() ? nullptr : &ac_fault.samples.back();
    const bool availability_comparison_resolved =
        simulations_valid && equilibrium_valid && dae_terminal != nullptr;
    const bool binary_vsc1_available = binary_system.vsc_converters.at(0).in_service;
    const bool binary_vsc2_available = binary_system.vsc_converters.at(1).in_service;
    const bool dae_vsc1_available = availability_comparison_resolved &&
        dae_terminal->vsc1_in_service > 0.5 &&
        dae_terminal->vsc1_protection_tripped < 0.5;
    const bool dae_vsc2_available = availability_comparison_resolved &&
        dae_terminal->vsc2_in_service > 0.5 &&
        dae_terminal->vsc2_protection_tripped < 0.5;
    const int availability_mismatch_count = availability_comparison_resolved
        ? static_cast<int>(binary_vsc1_available != dae_vsc1_available) +
              static_cast<int>(binary_vsc2_available != dae_vsc2_available)
        : 0;
    const bool validation_passed =
        simulations_valid && equilibrium_valid && ac_to_dc && legacy_identity &&
        dc_to_ac && step_classification_consistent && power_step_converged &&
        block_time_converged;

    json report;
    report["study"] = "modified case33bw_acdc bidirectional propagation capability audit";
    report["case_provenance"] = {
        {"builder", "hacdcpf::io::build_case33bw_acdc"},
        {"ac_buses", 33},
        {"dc_buses", 2},
        {"ac_branches", 37},
        {"dc_branches", 1},
        {"vsc_converters", 2},
        {"study_modifications",
         {"1 MW DC load at DC bus 2",
          "explicit VSC schedules, ratings, current limits and dynamic profiles",
          "IEEE 1547 Category II AC-terminal protection on VSC2",
          "opt-in DC-link active-power derating and undervoltage block on VSC2"}},
    };
    report["settings"] = {
        {"dt_s", dt_s},
        {"cross_validation_dt_s", {0.001, 0.002, 0.005}},
        {"smoke_requested", smoke},
        {"fault_time_s", kFaultTimeS},
        {"clear_time_s", kClearTimeS},
        {"terminal_time_s", kEndTimeS},
        {"fault_resistance_pu", 0.02},
        {"voltage_propagation_threshold_pu", kVoltagePropagationThresholdPu},
        {"power_propagation_threshold_mw", kPowerPropagationThresholdMw},
        {"equilibrium_drift_threshold_pu", kEquilibriumDriftThresholdPu},
        {"dc_active_power_derate_start_pu", 0.95},
        {"dc_undervoltage_block_pu", 0.75},
        {"dc_undervoltage_block_delay_s", 0.02},
    };
    report["baseline_validation"] = {
        {"all_simulations_successful", simulations_valid},
        {"equilibrium_valid", equilibrium_valid},
        {"maximum_ac18_voltage_drift_pu", baseline_ac_drift},
        {"maximum_dc2_voltage_drift_pu", baseline_dc_drift},
    };
    report["propagation_tests"] = {
        {"ac_fault_to_dc_side",
         {{"passed", ac_to_dc},
          {"maximum_dc1_voltage_deviation_pu", ac_to_dc_v1},
          {"maximum_dc2_voltage_deviation_pu", ac_to_dc_v2},
          {"maximum_dc_voltage_deviation_pu", ac_to_dc_max}}},
        {"dc_fault_to_ac_side_legacy",
         {{"passed", propagated(legacy_dc_to_ac_voltage,
                                  legacy_dc_to_ac_power)},
          {"legacy_identity_passed", legacy_identity},
          {"maximum_ac18_voltage_deviation_pu", legacy_dc_to_ac_voltage},
          {"maximum_vsc2_ac_power_deviation_mw", legacy_dc_to_ac_power}}},
        {"dc_fault_to_ac_side_enhanced",
         {{"passed", dc_to_ac},
          {"maximum_ac18_voltage_deviation_pu", enhanced_voltage_2ms},
          {"maximum_vsc2_ac_power_deviation_mw", enhanced_power_2ms}}},
    };
    report["step_size_cross_validation"] = {
        {"classification_consistent", step_classification_consistent},
        {"power_step_converged", power_step_converged},
        {"block_time_converged", block_time_converged},
        {"power_relative_difference_1ms_2ms",
         power_relative_difference_1ms_2ms},
        {"power_relative_tolerance", kStepPowerRelativeTolerance},
        {"block_time_spread_s", nullable_number(block_time_spread_s)},
        {"block_time_tolerance_s", kBlockTimeToleranceS},
        {"runs",
         {{{"dt_s", 0.001},
           {"maximum_ac18_voltage_deviation_pu", enhanced_voltage_1ms},
           {"maximum_vsc2_ac_power_deviation_mw", enhanced_power_1ms},
           {"dc_undervoltage_block_time_s", nullable_number(block_time_1ms)}},
          {{"dt_s", 0.002},
           {"maximum_ac18_voltage_deviation_pu", enhanced_voltage_2ms},
           {"maximum_vsc2_ac_power_deviation_mw", enhanced_power_2ms},
           {"dc_undervoltage_block_time_s", nullable_number(block_time_2ms)}},
          {{"dt_s", 0.005},
           {"maximum_ac18_voltage_deviation_pu", enhanced_voltage_5ms},
           {"maximum_vsc2_ac_power_deviation_mw", enhanced_power_5ms},
           {"dc_undervoltage_block_time_s", nullable_number(block_time_5ms)}}}}};
    report["binary_vs_dae_recovery_entry"] = {
        {"resolved", availability_comparison_resolved},
        {"availability_mismatch_count",
         availability_comparison_resolved ? json(availability_mismatch_count)
                                          : json(nullptr)},
        {"comparison_demonstrated",
         availability_comparison_resolved && availability_mismatch_count > 0},
    };
    report["binary_vs_dae_recovery_entry"]["converter_states"] = json::array({
        {{"vsc_index", 1},
         {"binary_available", binary_vsc1_available},
         {"dae_available", availability_comparison_resolved
                               ? json(dae_vsc1_available) : json(nullptr)},
         {"dae_in_service", availability_comparison_resolved
                                ? json(dae_terminal->vsc1_in_service > 0.5)
                                : json(nullptr)},
         {"dae_protection_tripped", availability_comparison_resolved
                                        ? json(dae_terminal->vsc1_protection_tripped > 0.5)
                                        : json(nullptr)},
         {"dae_terminal_p_mw", availability_comparison_resolved
                                   ? json(dae_terminal->vsc1_p_mw) : json(nullptr)},
         {"dae_terminal_vdc_link_pu", availability_comparison_resolved
                                          ? json(dae_terminal->vsc1_vdc_link_pu)
                                          : json(nullptr)}},
        {{"vsc_index", 2},
         {"binary_available", binary_vsc2_available},
         {"dae_available", availability_comparison_resolved
                               ? json(dae_vsc2_available) : json(nullptr)},
         {"dae_in_service", availability_comparison_resolved
                                ? json(dae_terminal->vsc2_in_service > 0.5)
                                : json(nullptr)},
         {"dae_protection_tripped", availability_comparison_resolved
                                        ? json(dae_terminal->vsc2_protection_tripped > 0.5)
                                        : json(nullptr)},
         {"dae_terminal_p_mw", availability_comparison_resolved
                                   ? json(dae_terminal->vsc2_p_mw) : json(nullptr)},
         {"dae_terminal_vdc_link_pu", availability_comparison_resolved
                                          ? json(dae_terminal->vsc2_vdc_link_pu)
                                          : json(nullptr)}},
    });
    report["capability"] = {
        {"ac_to_dc_propagation", ac_to_dc},
        {"dc_to_ac_propagation", dc_to_ac},
        {"legacy_model_identity", legacy_identity},
        {"step_size_cross_validation", step_classification_consistent &&
                                                power_step_converged &&
                                                block_time_converged},
        {"binary_vs_dae_entry_comparison",
         availability_comparison_resolved && availability_mismatch_count > 0},
        {"reliability_entry_bidirectional_propagation",
         ac_to_dc && dc_to_ac && availability_comparison_resolved &&
             availability_mismatch_count > 0},
        {"emt_diode_fed_dc_fault_propagation", false},
        {"validation_passed", validation_passed},
    };
    report["model_scope"] =
        "balanced-phasor mass-matrix DAE with AC/DC algebraic networks, "
        "average-value GFL-VSC controls, dynamic DC-link state, opt-in "
        "DC-voltage active-power derating and latched undervoltage blocking, "
        "and IEEE 1547 AC-terminal protection";
    report["model_limitations"] = {
        "The study is not EMT and does not represent DC-breaker arcs or MMC submodules.",
        "The reduced GFL model does not represent antiparallel-diode fault feed, MMC arm saturation, submodule capacitor balancing or DC overcurrent protection.",
        "The 0.95/0.75 pu thresholds and 20 ms delay are authored study parameters, not universal standards or vendor settings.",
        "The DC undervoltage block is latched; vendor-specific deblocking and precharge are outside the admitted scope.",
        "A solved DC fault is not counted as DC-to-AC propagation unless an AC-side signal changes relative to the no-fault trajectory.",
        "Binary availability is a two-state restoration input; DAE availability is read from the post-fault converter protection state.",
    };
    report["literature_basis"] = json::array({
        {{"reference", "Pandey et al., IEEE Transactions on Industrial Informatics, 2025"},
         {"doi", "10.1109/TII.2025.3574424"},
         {"used_for", "RTDS/CHIL evidence that conventional BIC freewheeling diodes can feed a DC pole fault from the AC side"},
         {"represented_by_this_model", false}},
        {{"reference", "Kaler and Yazdani, IEEE Access, 2022"},
         {"doi", "10.1109/ACCESS.2022.3171346"},
         {"used_for", "DC-fault change in AC real power/current and current-controlled fault-blocking converter response"},
         {"represented_by_this_model", true}},
        {{"reference", "Bandaru et al., IEEE Transactions on Industry Applications, 2024"},
         {"doi", "10.1109/TIA.2024.3396790"},
         {"used_for", "DC-fault active-power interruption and AC-voltage/reactive-power interaction"},
         {"represented_by_this_model", "active-power interruption only"}},
        {{"reference", "Yu et al., IEEE Transactions on Power Delivery, 2024"},
         {"doi", "10.1109/TPWRD.2023.3294173"},
         {"used_for", "staged converter control and protection response during DC faults"},
         {"represented_by_this_model", "reduced definite-time guard only"}},
        {{"reference", "Lv et al., IEEE Transactions on Power Delivery, 2025"},
         {"doi", "10.1109/TPWRD.2024.3485910"},
         {"used_for", "necessity of converter saturation limits and explicit exclusion of MMC arm fidelity"},
         {"represented_by_this_model", false}},
    });
    report["scenarios"] = json::array();
    for (const auto& scenario : scenarios) {
      report["scenarios"].push_back(scenario_json(scenario));
    }

    if (!csv_path.empty()) write_csv(csv_path, scenarios);
    const std::string serialized = report.dump(2) + "\n";
    if (output_path.empty()) {
      std::cout << serialized;
    } else {
      std::ofstream file(output_path);
      if (!file) throw std::runtime_error("cannot open report: " + output_path.string());
      file << serialized;
      std::cout << "Wrote " << output_path << '\n';
    }

    return validation_passed ? 0 : 2;
  } catch (const std::exception& error) {
    std::cerr << "case33bw_acdc propagation study failed: " << error.what() << '\n';
    return 1;
  }
}
