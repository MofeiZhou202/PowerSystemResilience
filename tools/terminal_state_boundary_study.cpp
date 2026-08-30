#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "hacdcpf/dynamics/dynamics.hpp"
#include "hacdcpf/io/case_builders.hpp"
#include "hacdcpf/reliability/protection_frt.hpp"

using json = nlohmann::json;
using namespace hacdcpf;
using namespace hacdcpf::dynamics;

namespace {

constexpr double kFaultTimeS = 1.0;
constexpr double kFiniteHorizonS = 3.0;
constexpr double kReconnectDelayS = 5.0;
constexpr double kPowerRampS = 2.0;
constexpr double kAffectedDcLoadMw = 0.4;
constexpr double kTotalAffectedLoadMw = 1.0;
constexpr double kStudySystemBaseMva = 100.0;
constexpr double kStudyConverterRatingMva = 1.0;
constexpr double kStudyConverterEfficiency = 0.98;
constexpr double kConverterReactiveSupportMvar = 0.3;
constexpr double kConverterCurrentHeadroom = 1.2;
constexpr double kAdequacyBiasLimit = 0.05;
constexpr double kEnsEquivalenceToleranceMwh = 1e-12;
constexpr double kEventToleranceS = 0.011;
constexpr int kVscIndex = 1;
constexpr int kFaultBus = 6;

struct ScenarioSpec {
  double fault_r_pu{0.0};
  double clearing_duration_s{0.0};
  VSCCurrentLimitPriority priority{VSCCurrentLimitPriority::Magnitude};
};

struct FiniteHorizonComparison {
  double network_restoration_delay_s{0.0};
  double static_ens_mwh{0.0};
  double transient_ens_mwh{0.0};
  double incremental_ens_mwh{0.0};
  double relative_static_underestimate{0.0};
};

struct ScenarioResult {
  ScenarioSpec spec;
  bool success{false};
  std::string message;
  bool ever_protection_tripped{false};
  bool terminal_blocked{false};
  bool reconnected{false};
  bool full_power_restored{false};
  bool terminal_protection_tripped{false};
  bool momentary_cessation{false};
  bool priority_mapping_honored{false};
  std::string frt_terminal_class;
  std::string trip_reason;
  double trip_time_s{-1.0};
  double reconnect_time_s{-1.0};
  double full_power_time_s{-1.0};
  double min_ac_voltage_pu{std::numeric_limits<double>::infinity()};
  double min_dc_voltage_pu{std::numeric_limits<double>::infinity()};
  double max_vsc_current_pu{0.0};
  double terminal_restore_scale{1.0};
  double terminal_protection_voltage_pu{1.0};
  double terminal_protection_frequency_hz{50.0};
  bool current_limit_active{false};
  int realized_current_limiter_model{-1};
  std::vector<std::pair<double, double>> restore_scale;
  std::vector<FiniteHorizonComparison> finite_horizon_comparisons;
  std::vector<double> relative_static_bias;
};

std::string priority_name(VSCCurrentLimitPriority priority) {
  switch (priority) {
    case VSCCurrentLimitPriority::ActivePower: return "active_power";
    case VSCCurrentLimitPriority::ReactivePower: return "reactive_power";
    case VSCCurrentLimitPriority::Magnitude: return "magnitude";
  }
  return "magnitude";
}

int expected_limiter_model(VSCCurrentLimitPriority priority) {
  switch (priority) {
    case VSCCurrentLimitPriority::Magnitude:
      return static_cast<int>(CurrentLimiterKind::Magnitude);
    case VSCCurrentLimitPriority::ActivePower:
      return static_cast<int>(CurrentLimiterKind::ActivePriority);
    case VSCCurrentLimitPriority::ReactivePower:
      return static_cast<int>(CurrentLimiterKind::ReactivePriority);
  }
  return -1;
}

const DynamicDeviceOutput* vsc_output(const DynamicSnapshot& snapshot) {
  const auto found = std::find_if(
      snapshot.device_outputs.begin(), snapshot.device_outputs.end(),
      [](const DynamicDeviceOutput& output) {
        return output.component_index == kVscIndex &&
               output.source_type == "vsc_grid_following";
      });
  return found == snapshot.device_outputs.end() ? nullptr : &*found;
}

double output_value(const DynamicDeviceOutput& output,
                    const std::string& key,
                    double fallback) {
  const auto found = output.values.find(key);
  return found == output.values.end() ? fallback : found->second;
}

IEEE1547Settings study_ieee1547_settings() {
  auto settings = make_default_ieee1547(IEEE1547Category::CategoryII, 50.0);
  settings.enabled = true;
  settings.allow_reconnect = true;
  settings.reconnect_delay_s = kReconnectDelayS;
  settings.power_ramp_s = kPowerRampS;
  return settings;
}

HybridPowerSystem study_system(const ScenarioSpec& spec) {
  auto system = io::build_networked_microgrids_islanding();
  auto& converter = system.vsc_converters.at(0);
  const double converter_rating_pu = kStudyConverterRatingMva / system.base_mva;
  converter.current_limit_priority = spec.priority;
  // The authored case has 0 MW scheduled through the VSC and 0.5 MW local DC
  // solar, which cannot exercise converter current-control priority. This
  // campaign isolates the 0.4 MW DC-load supply path: local DC solar is removed
  // and the AC terminal imports the load plus converter loss. Sign convention
  // follows converter_coordination.cpp::converter_dc_injection_mw.
  system.dc.dc_static_generators.at(0).in_service = false;
  converter.eta = kStudyConverterEfficiency;
  converter.p_schedule_mw = -kAffectedDcLoadMw / kStudyConverterEfficiency;
  converter.q_set_mvar = kConverterReactiveSupportMvar;
  converter.p_rated_mw = kStudyConverterRatingMva;
  // i_ac_max_pu is on the system base (converter_components.hpp). The study
  // re-rates this 0.4 MW supply path to 1 MVA; 1.2 pu converter current is
  // therefore 0.012 pu on the 100 MVA system base.
  converter.i_ac_max_pu = kConverterCurrentHeadroom * converter_rating_pu;
  converter.dynamic_model.standard = "IEEE/NERC";
  converter.dynamic_model.model_name = "REGC_REEC_GFL_Subset";
  converter.dynamic_model.parameters["ieee1547_enabled"] = 1.0;
  converter.dynamic_model.parameters["ieee1547_category"] = 2.0;
  converter.dynamic_model.parameters["ieee1547_allow_reconnect"] = 1.0;
  converter.dynamic_model.parameters["ieee1547_reconnect_delay_s"] =
      kReconnectDelayS;
  converter.dynamic_model.parameters["ieee1547_power_ramp_s"] = kPowerRampS;
  converter.dynamic_model.parameters["current_limit_pu"] =
      converter.i_ac_max_pu;
  return system;
}

DynamicEvent fault_event(const ScenarioSpec& spec) {
  DynamicEvent event;
  event.time_s = kFaultTimeS;
  event.type = DynamicEventType::FaultShunt;
  event.bus = kFaultBus;
  event.phase = -1;
  event.component_type = "AC";
  event.label = "terminal-state boundary study AC fault";
  event.params["r_pu"] = spec.fault_r_pu;
  event.params["x_pu"] = 0.0;
  return event;
}

DynamicEvent clear_event(const ScenarioSpec& spec) {
  DynamicEvent event;
  event.time_s = kFaultTimeS + spec.clearing_duration_s;
  event.type = DynamicEventType::ClearFault;
  event.bus = kFaultBus;
  event.component_type = "AC";
  event.label = "terminal-state boundary study fault clearance";
  return event;
}

double integrate_unavailable_seconds(
    const std::vector<std::pair<double, double>>& trajectory,
    double begin_s,
    double end_s) {
  if (trajectory.size() < 2) return 0.0;
  double integral = 0.0;
  for (std::size_t i = 1; i < trajectory.size(); ++i) {
    const auto [t0_raw, y0_raw] = trajectory[i - 1];
    const auto [t1_raw, y1_raw] = trajectory[i];
    if (t1_raw <= begin_s || t0_raw >= end_s || t1_raw <= t0_raw) continue;
    const double t0 = std::max(t0_raw, begin_s);
    const double t1 = std::min(t1_raw, end_s);
    const double alpha0 = (t0 - t0_raw) / (t1_raw - t0_raw);
    const double alpha1 = (t1 - t0_raw) / (t1_raw - t0_raw);
    const double y0 = std::clamp(
        y0_raw + alpha0 * (y1_raw - y0_raw), 0.0, 1.0);
    const double y1 = std::clamp(
        y0_raw + alpha1 * (y1_raw - y0_raw), 0.0, 1.0);
    integral += 0.5 * ((1.0 - y0) + (1.0 - y1)) * (t1 - t0);
  }
  return integral;
}

ScenarioResult run_scenario(const ScenarioSpec& spec,
                            double dt_s,
                            double terminal_time_s) {
  ScenarioResult out;
  out.spec = spec;

  DynamicSolverOptions options;
  options.solver_type = DynamicSolverType::MassMatrixDae;
  options.dae_step_method = DynamicDaeStepMethod::BackwardEuler;
  options.t_end_s = terminal_time_s;
  options.dt_s = dt_s;
  options.run_power_flow_initialization = true;
  options.use_consistent_dynamic_initialization = true;
  options.enable_der_protection = true;
  options.localize_der_protection_events = true;
  options.protection_event_time_tol_s = 1e-6;
  options.record_every_step = true;
  options.record_device_outputs = true;

  DynamicModelBuilder builder;
  auto dynamic_system = builder.build(study_system(spec), options);
  dynamic_system.events.push_back(fault_event(spec));
  dynamic_system.events.push_back(clear_event(spec));
  DynamicSolver solver;
  const DynamicResults result = solver.solve(dynamic_system);
  out.success = result.success;
  out.message = result.message;
  if (!result.success) return out;

  for (const auto& event : result.applied_event_records) {
    if (event.component_index != kVscIndex || event.component_type != "VSC")
      continue;
    if (event.type == "VSCTrip") {
      out.ever_protection_tripped = true;
      if (out.trip_time_s < 0.0) {
        out.trip_time_s = event.time_s;
        out.trip_reason = event.label;
      }
    } else if (event.type == "Custom" &&
               event.label.rfind("IEEE1547 reconnect:", 0) == 0) {
      out.reconnected = true;
      if (out.reconnect_time_s < 0.0) out.reconnect_time_s = event.time_s;
    }
  }

  std::vector<analysis::DERTrajectoryPoint> frt_trajectory;
  frt_trajectory.reserve(result.snapshots.size());
  for (const auto& snapshot : result.snapshots) {
    out.min_ac_voltage_pu =
        std::min(out.min_ac_voltage_pu, snapshot.min_ac_voltage_pu);
    out.min_dc_voltage_pu =
        std::min(out.min_dc_voltage_pu, snapshot.min_dc_voltage_pu);
    const auto* device = vsc_output(snapshot);
    if (!device) continue;
    const double restore = output_value(*device, "protection_restore_scale", 1.0);
    const double protection_voltage =
        output_value(*device, "protection_v_meas_pu", 1.0);
    const double protection_frequency =
        output_value(*device, "protection_f_meas_hz", 50.0);
    const double pll_angle = output_value(*device, "pll_angle_rad", 0.0);
    out.restore_scale.emplace_back(snapshot.time_s, restore);
    frt_trajectory.push_back({snapshot.time_s, protection_voltage, pll_angle});
    out.max_vsc_current_pu = std::max(
        out.max_vsc_current_pu,
        std::abs(output_value(*device, "i_mag_pu", 0.0)));
    out.current_limit_active = out.current_limit_active ||
        output_value(*device, "current_limit_active", 0.0) > 0.5;
    out.realized_current_limiter_model = static_cast<int>(std::llround(
        output_value(*device, "current_limiter_model", -1.0)));
    if (out.reconnected && snapshot.time_s + 1e-12 >= out.reconnect_time_s &&
        !out.full_power_restored && restore >= 1.0 - 1e-6) {
      out.full_power_restored = true;
      out.full_power_time_s = snapshot.time_s;
    }
    out.terminal_restore_scale = restore;
    out.terminal_protection_tripped =
        output_value(*device, "protection_tripped", 0.0) > 0.5;
    out.terminal_protection_voltage_pu = protection_voltage;
    out.terminal_protection_frequency_hz = protection_frequency;
  }
  if (frt_trajectory.empty()) {
    out.success = false;
    out.message = "target VSC grid-following output is absent from snapshots";
    return out;
  }
  out.priority_mapping_honored = out.realized_current_limiter_model ==
      expected_limiter_model(spec.priority);
  if (!out.priority_mapping_honored) {
    out.success = false;
    out.message = "authored VSC current-limit priority was not preserved";
    return out;
  }
  out.terminal_blocked = out.terminal_protection_tripped;

  analysis::DERMomentaryCessationSettings cessation;
  cessation.enabled = true;
  cessation.enter_below_voltage_pu = 0.5;
  cessation.exit_above_voltage_pu = 0.9;
  cessation.exit_dwell_s = 0.1;  // five cycles on the authored 50 Hz system
  cessation.maximum_duration_s = 0.0;
  const auto frt = analysis::classify_der_frt_trajectory(
      study_ieee1547_settings(), cessation, frt_trajectory, options.t_end_s);
  out.momentary_cessation = frt.ever_momentary_ceased;
  if (out.terminal_protection_tripped && out.reconnected)
    out.frt_terminal_class = "retripped_at_horizon";
  else if (out.terminal_protection_tripped)
    out.frt_terminal_class = "tripped_at_horizon";
  else if (out.reconnected && out.full_power_restored)
    out.frt_terminal_class = "reconnected_full_power";
  else if (out.reconnected)
    out.frt_terminal_class = "reconnected_ramping";
  else
    out.frt_terminal_class = "ride_through";

  // Finite-window comparison used for the admitted static/transient baseline.
  // Derivation: manuscript README, "Analytical Fast-Assessment Model".
  // The static model assumes immediate converter availability after network
  // restoration. The transient-aware model is its nested extension: it adds
  // only the simulated unavailable area of the converter-fed DC load.
  static constexpr double kFiniteWindowRestorationDelaysS[] = {
      0.25, 0.5, 1.0, 1.5, 2.0};
  for (double restoration_delay_s : kFiniteWindowRestorationDelaysS) {
    const double network_restored_s = kFaultTimeS + restoration_delay_s;
    const double static_outage_s = std::min(
        restoration_delay_s, kFiniteHorizonS - kFaultTimeS);
    const double static_ens_mwh =
        kTotalAffectedLoadMw * static_outage_s / 3600.0;
    const double incremental_ens_mwh = kAffectedDcLoadMw *
        integrate_unavailable_seconds(
            out.restore_scale, network_restored_s, kFiniteHorizonS) /
        3600.0;
    const double transient_ens_mwh = static_ens_mwh + incremental_ens_mwh;
    out.finite_horizon_comparisons.push_back({
        restoration_delay_s,
        static_ens_mwh,
        transient_ens_mwh,
        incremental_ens_mwh,
        transient_ens_mwh > 0.0
            ? incremental_ens_mwh / transient_ens_mwh
            : 0.0});
  }

  static constexpr double kSwitchDelaysS[] = {1.0, 5.0, 15.0, 60.0};
  for (double switch_delay_s : kSwitchDelaysS) {
    const double static_ens_mwh =
        kTotalAffectedLoadMw * switch_delay_s / 3600.0;
    // Reliability interface derivation: after network restoration, only the
    // unavailable fraction of the 0.4 MW DC path is incremental. IEEE 1547-2018
    // enter-service delay and soft-start scale are read from the simulated VSC.
    const double network_restored_s = kFaultTimeS + switch_delay_s;
    const bool consequence_resolved = !out.ever_protection_tripped ||
        (out.full_power_restored && out.full_power_time_s <= terminal_time_s);
    if (!consequence_resolved) {
      out.relative_static_bias.push_back(
          std::numeric_limits<double>::quiet_NaN());
      continue;
    }
    const double extra_ens_mwh = kAffectedDcLoadMw *
        integrate_unavailable_seconds(
            out.restore_scale, network_restored_s, terminal_time_s) /
        3600.0;
    const double dynamic_ens_mwh = static_ens_mwh + extra_ens_mwh;
    out.relative_static_bias.push_back(
        dynamic_ens_mwh > 0.0
            ? (dynamic_ens_mwh - static_ens_mwh) / dynamic_ens_mwh
            : 0.0);
  }
  return out;
}


json finite_or_null(double value) {
  return std::isfinite(value) ? json(value) : json(nullptr);
}

json scenario_json(const ScenarioResult& row) {
  static constexpr double kSwitchDelaysS[] = {1.0, 5.0, 15.0, 60.0};
  json bias = json::object();
  for (std::size_t i = 0; i < row.relative_static_bias.size(); ++i) {
    const double value = row.relative_static_bias[i];
    bias[std::to_string(static_cast<int>(kSwitchDelaysS[i])) + "s"] =
        std::isfinite(value) ? json(value) : json(nullptr);
  }
  json finite_horizon = json::array();
  for (const auto& comparison : row.finite_horizon_comparisons) {
    finite_horizon.push_back({
        {"network_restoration_delay_s",
         comparison.network_restoration_delay_s},
        {"static_ens_mwh", comparison.static_ens_mwh},
        {"transient_ens_mwh", comparison.transient_ens_mwh},
        {"incremental_ens_mwh", comparison.incremental_ens_mwh},
        {"relative_static_underestimate",
         comparison.relative_static_underestimate},
        {"static_and_transient_equivalent",
         std::abs(comparison.incremental_ens_mwh) <=
             kEnsEquivalenceToleranceMwh}});
  }
  return {
      {"fault_r_pu", row.spec.fault_r_pu},
      {"clearing_duration_s", row.spec.clearing_duration_s},
      {"current_limit_priority", priority_name(row.spec.priority)},
      {"success", row.success},
      {"message", row.message},
      {"ever_protection_tripped", row.ever_protection_tripped},
      {"terminal_blocked", row.terminal_blocked},
      {"reconnected", row.reconnected},
      {"full_power_restored", row.full_power_restored},
      {"terminal_protection_tripped", row.terminal_protection_tripped},
      {"momentary_cessation", row.momentary_cessation},
      {"priority_mapping_honored", row.priority_mapping_honored},
      {"frt_terminal_class", row.frt_terminal_class},
      {"trip_reason", row.trip_reason.empty() ? json(nullptr) : json(row.trip_reason)},
      {"trip_time_s", row.trip_time_s >= 0.0 ? json(row.trip_time_s) : json(nullptr)},
      {"reconnect_time_s",
       row.reconnect_time_s >= 0.0 ? json(row.reconnect_time_s) : json(nullptr)},
      {"full_power_time_s",
       row.full_power_time_s >= 0.0 ? json(row.full_power_time_s) : json(nullptr)},
      {"min_ac_voltage_pu", finite_or_null(row.min_ac_voltage_pu)},
      {"min_dc_voltage_pu", finite_or_null(row.min_dc_voltage_pu)},
      {"max_vsc_current_pu", row.max_vsc_current_pu},
      {"terminal_restore_scale", row.terminal_restore_scale},
      {"terminal_protection_voltage_pu", row.terminal_protection_voltage_pu},
      {"terminal_protection_frequency_hz", row.terminal_protection_frequency_hz},
      {"current_limit_active", row.current_limit_active},
      {"realized_current_limiter_model", row.realized_current_limiter_model},
      {"finite_horizon_comparison", std::move(finite_horizon)},
      {"static_eens_relative_underestimate", std::move(bias)}};
}

std::vector<ScenarioSpec> scenario_matrix(bool smoke) {
  const std::vector<double> fault_resistance =
      smoke ? std::vector<double>{0.01, 0.20}
            : std::vector<double>{0.01, 0.03, 0.10, 0.20};
  const std::vector<double> clearing =
      smoke ? std::vector<double>{0.08, 0.30}
            : std::vector<double>{0.08, 0.12, 0.16, 0.20, 0.30};
  const std::vector<VSCCurrentLimitPriority> priorities =
      smoke ? std::vector<VSCCurrentLimitPriority>{VSCCurrentLimitPriority::Magnitude}
            : std::vector<VSCCurrentLimitPriority>{
                  VSCCurrentLimitPriority::Magnitude,
                  VSCCurrentLimitPriority::ActivePower,
                  VSCCurrentLimitPriority::ReactivePower};
  std::vector<ScenarioSpec> result;
  for (double resistance : fault_resistance)
    for (double duration : clearing)
      for (auto priority : priorities)
        result.push_back({resistance, duration, priority});
  return result;
}

void write_csv(const std::filesystem::path& path,
               const std::vector<ScenarioResult>& rows) {
  std::ofstream stream(path);
  if (!stream) throw std::runtime_error("cannot open CSV output: " + path.string());
  stream << "fault_r_pu,clearing_duration_s,current_limit_priority,success,"
            "ever_protection_tripped,terminal_blocked,reconnected,"
            "full_power_restored,terminal_protection_tripped,momentary_cessation,"
            "priority_mapping_honored,realized_current_limiter_model,"
            "frt_terminal_class,trip_reason,trip_time_s,reconnect_time_s,"
            "full_power_time_s,min_ac_voltage_pu,min_dc_voltage_pu,"
            "max_vsc_current_pu,current_limit_active,terminal_restore_scale,"
            "terminal_protection_voltage_pu,terminal_protection_frequency_hz,"
            "finite_0p25s_static_ens_mwh,finite_0p25s_transient_ens_mwh,"
            "finite_0p25s_incremental_ens_mwh,finite_0p25s_relative_bias,"
            "finite_0p5s_static_ens_mwh,finite_0p5s_transient_ens_mwh,"
            "finite_0p5s_incremental_ens_mwh,finite_0p5s_relative_bias,"
            "finite_1s_static_ens_mwh,finite_1s_transient_ens_mwh,"
            "finite_1s_incremental_ens_mwh,finite_1s_relative_bias,"
            "finite_1p5s_static_ens_mwh,finite_1p5s_transient_ens_mwh,"
            "finite_1p5s_incremental_ens_mwh,finite_1p5s_relative_bias,"
            "finite_2s_static_ens_mwh,finite_2s_transient_ens_mwh,"
            "finite_2s_incremental_ens_mwh,finite_2s_relative_bias,"
            "bias_1s,bias_5s,bias_15s,bias_60s\n";
  stream << std::setprecision(12);
  for (const auto& row : rows) {
    stream << row.spec.fault_r_pu << ',' << row.spec.clearing_duration_s << ','
           << priority_name(row.spec.priority) << ',' << row.success << ','
           << row.ever_protection_tripped << ',' << row.terminal_blocked << ','
           << row.reconnected << ',' << row.full_power_restored << ','
           << row.terminal_protection_tripped << ',' << row.momentary_cessation << ','
           << row.priority_mapping_honored << ','
           << row.realized_current_limiter_model << ','
           << row.frt_terminal_class << ',' << row.trip_reason << ',';
    if (row.trip_time_s >= 0.0) stream << row.trip_time_s;
    stream << ',';
    if (row.reconnect_time_s >= 0.0) stream << row.reconnect_time_s;
    stream << ',';
    if (row.full_power_time_s >= 0.0) stream << row.full_power_time_s;
    stream << ',' << row.min_ac_voltage_pu << ',' << row.min_dc_voltage_pu << ','
           << row.max_vsc_current_pu << ',' << row.current_limit_active << ','
           << row.terminal_restore_scale << ','
           << row.terminal_protection_voltage_pu << ','
           << row.terminal_protection_frequency_hz;
    for (const auto& comparison : row.finite_horizon_comparisons) {
      stream << ',' << comparison.static_ens_mwh
             << ',' << comparison.transient_ens_mwh
             << ',' << comparison.incremental_ens_mwh
             << ',' << comparison.relative_static_underestimate;
    }
    for (double value : row.relative_static_bias) {
      stream << ',';
      if (std::isfinite(value)) stream << value;
    }
    stream << '\n';
  }
}

int run(int argc, char** argv) {
  bool smoke = false;
  double dt_s = 0.005;
  double terminal_time_s = 25.0;
  std::filesystem::path json_path;
  std::filesystem::path csv_path;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--smoke") smoke = true;
    else if (arg == "--output" && i + 1 < argc) json_path = argv[++i];
    else if (arg == "--csv" && i + 1 < argc) csv_path = argv[++i];
    else if (arg == "--dt" && i + 1 < argc) dt_s = std::stod(argv[++i]);
    else if (arg == "--t-end" && i + 1 < argc)
      terminal_time_s = std::stod(argv[++i]);
    else throw std::invalid_argument(
        "usage: terminal_state_boundary_study [--smoke] [--dt seconds] "
        "[--t-end seconds] [--output file] [--csv file]");
  }
  if (!std::isfinite(dt_s) || dt_s <= 0.0 ||
      !std::isfinite(terminal_time_s) || terminal_time_s < kFiniteHorizonS)
    throw std::invalid_argument(
        "dt must be positive and t-end must be at least the 3 s comparison horizon");

  const auto specs = scenario_matrix(smoke);
  std::vector<ScenarioResult> rows;
  rows.reserve(specs.size());
  for (const auto& spec : specs)
    rows.push_back(run_scenario(spec, dt_s, terminal_time_s));

  int valid = 0;
  int tripped = 0;
  int terminal_blocked = 0;
  int reconnected = 0;
  int momentary = 0;
  int current_limited = 0;
  int ride_through = 0;
  bool chronology_valid = true;
  bool reconnect_delay_valid = true;
  std::map<std::string, int> trip_reason_counts;
  bool finite_horizon_ordering_valid = true;
  bool ride_through_equivalence_valid = true;
  bool analytical_reconstruction_valid = true;
  for (const auto& row : rows) {
    if (!row.success) continue;
    ++valid;
    tripped += row.ever_protection_tripped ? 1 : 0;
    terminal_blocked += row.terminal_blocked ? 1 : 0;
    reconnected += row.reconnected ? 1 : 0;
    momentary += row.momentary_cessation ? 1 : 0;
    current_limited += row.current_limit_active ? 1 : 0;
    ride_through += !row.ever_protection_tripped ? 1 : 0;
    if (row.ever_protection_tripped) ++trip_reason_counts[row.trip_reason];
    if (row.reconnected) {
      chronology_valid = chronology_valid && row.trip_time_s >= 0.0 &&
          row.reconnect_time_s > row.trip_time_s;
      reconnect_delay_valid = reconnect_delay_valid &&
          row.reconnect_time_s + kEventToleranceS >=
              kFaultTimeS + row.spec.clearing_duration_s + kReconnectDelayS;
    }
    for (const auto& comparison : row.finite_horizon_comparisons) {
      finite_horizon_ordering_valid = finite_horizon_ordering_valid &&
          comparison.transient_ens_mwh + kEnsEquivalenceToleranceMwh >=
              comparison.static_ens_mwh;
      if (!row.ever_protection_tripped) {
        ride_through_equivalence_valid = ride_through_equivalence_valid &&
            std::abs(comparison.incremental_ens_mwh) <=
                kEnsEquivalenceToleranceMwh;
      }
    }
  }

  static constexpr double kFiniteWindowRestorationDelaysS[] = {
      0.25, 0.5, 1.0, 1.5, 2.0};
  json finite_horizon_summary = json::array();
  for (std::size_t i = 0; i < 5; ++i) {
    double static_sum = 0.0;
    double transient_sum = 0.0;
    double incremental_sum = 0.0;
    double tripped_incremental_sum = 0.0;
    double max_incremental = 0.0;
    int compared = 0;
    int tripped_compared = 0;
    int equivalent = 0;
    int ride_through_compared = 0;
    int ride_through_equivalent = 0;
    for (const auto& row : rows) {
      if (!row.success || row.finite_horizon_comparisons.size() <= i) continue;
      const auto& comparison = row.finite_horizon_comparisons[i];
      static_sum += comparison.static_ens_mwh;
      transient_sum += comparison.transient_ens_mwh;
      incremental_sum += comparison.incremental_ens_mwh;
      max_incremental = std::max(
          max_incremental, comparison.incremental_ens_mwh);
      ++compared;
      const bool is_equivalent =
          std::abs(comparison.incremental_ens_mwh) <=
          kEnsEquivalenceToleranceMwh;
      equivalent += is_equivalent ? 1 : 0;
      if (!row.ever_protection_tripped) {
        ++ride_through_compared;
        ride_through_equivalent += is_equivalent ? 1 : 0;
      } else {
        ++tripped_compared;
        tripped_incremental_sum += comparison.incremental_ens_mwh;
      }
    }
    const double mean_static =
        compared > 0 ? static_sum / compared : 0.0;
    const double mean_transient =
        compared > 0 ? transient_sum / compared : 0.0;
    const double mean_incremental =
        compared > 0 ? incremental_sum / compared : 0.0;
    const double relative_underestimate = mean_transient > 0.0
        ? mean_incremental / mean_transient
        : 0.0;
    const double trip_probability = compared > 0
        ? static_cast<double>(tripped_compared) / compared
        : 0.0;
    // Law of total expectation: E[E_dyn] = E[E_stat] +
    // p_trip * L_dc * E[A_trip | trip] / 3600. The ride-through kernel is
    // exactly zero under the validated nested-model contract.
    const double conditional_trip_unavailable_s = tripped_compared > 0
        ? tripped_incremental_sum * 3600.0 /
              (kAffectedDcLoadMw * tripped_compared)
        : 0.0;
    const double analytically_reconstructed_transient_ens = mean_static +
        trip_probability * kAffectedDcLoadMw *
            conditional_trip_unavailable_s / 3600.0;
    const double reconstruction_error = std::abs(
        analytically_reconstructed_transient_ens - mean_transient);
    analytical_reconstruction_valid = analytical_reconstruction_valid &&
        reconstruction_error <= kEnsEquivalenceToleranceMwh;
    finite_horizon_summary.push_back({
        {"network_restoration_delay_s",
         kFiniteWindowRestorationDelaysS[i]},
        {"compared_scenario_count", compared},
        {"mean_static_ens_mwh_per_event", mean_static},
        {"mean_transient_ens_mwh_per_event", mean_transient},
        {"mean_incremental_ens_mwh_per_event", mean_incremental},
        {"maximum_incremental_ens_mwh_per_event", max_incremental},
        {"relative_static_underestimate", relative_underestimate},
        {"equivalent_scenario_count", equivalent},
        {"ride_through_scenario_count", ride_through_compared},
        {"ride_through_equivalent_scenario_count", ride_through_equivalent},
        {"protective_trip_scenario_count", tripped_compared},
        {"protective_trip_probability_uniform_grid", trip_probability},
        {"conditional_tripped_mean_unavailable_s",
         conditional_trip_unavailable_s},
        {"analytically_reconstructed_mean_transient_ens_mwh_per_event",
         analytically_reconstructed_transient_ens},
        {"analytical_reconstruction_error_mwh", reconstruction_error},
        {"static_model_adequate_at_5pct",
         relative_underestimate <= kAdequacyBiasLimit}});
  }

  static constexpr double kSwitchDelaysS[] = {1.0, 5.0, 15.0, 60.0};
  json boundary = json::array();
  for (std::size_t i = 0; i < 4; ++i) {
    double average_dynamic_ens = 0.0;
    int resolved = 0;
    const double static_ens = kTotalAffectedLoadMw * kSwitchDelaysS[i] / 3600.0;
    for (const auto& row : rows) {
      if (!row.success) continue;
      if (!std::isfinite(row.relative_static_bias[i])) continue;
      average_dynamic_ens += static_ens /
          std::max(1e-12, 1.0 - row.relative_static_bias[i]);
      ++resolved;
    }
    if (resolved > 0) average_dynamic_ens /= resolved;
    const double bias = resolved == valid && average_dynamic_ens > 0.0
        ? (average_dynamic_ens - static_ens) / average_dynamic_ens : 0.0;
    const bool boundary_resolved = valid > 0 && resolved == valid;
    boundary.push_back({
        {"network_restoration_delay_s", kSwitchDelaysS[i]},
        {"static_eens_mwh_per_event", static_ens},
        {"resolved_scenario_count", resolved},
        {"trajectory_conditioned_mean_eens_mwh_per_event",
         boundary_resolved ? json(average_dynamic_ens) : json(nullptr)},
        {"relative_static_underestimate",
         boundary_resolved ? json(bias) : json(nullptr)},
        {"static_model_adequate_at_5pct",
         boundary_resolved ? json(bias <= kAdequacyBiasLimit) : json(nullptr)}});
  }

  json output = {
      {"study", "networked hybrid AC/DC microgrid terminal-state boundary pilot"},
      {"case", "build_networked_microgrids_islanding"},
      {"sampling_measure", smoke ? "uniform 2x2x1 smoke grid" : "uniform 4x5x3 deterministic grid"},
      {"probability_interpretation",
       "Empirical fractions are conditional on this uniform parameter grid; they are not field or annual probabilities."},
      {"model_scope",
       "phasor mass-matrix DAE + VSC GFL current limiter + IEEE 1547 protection trip/reconnect"},
      {"model_limitations", json::array({
           "p_protective_trip is a deterministic design-space fraction, not a calibrated stochastic p_block.",
           "VSCTrip represents IEEE 1547 terminal protection; MMC valve blocking, DC overcurrent blocking, precharge, and vendor unlock logic are not modeled.",
           "Momentary cessation is trajectory-classified but is not fed back as a VSC current-suppression state in this campaign.",
           "The reliability bias proxy isolates the 0.4 MW DC path and assumes the full 1.0 MW affected load is unavailable until network restoration.",
           "The admitted static/transient comparison is conditional on a 3 s post-fault observation window and must not be interpreted as annual EENS or an identified reconnect-time distribution.",
           "A reliability consequence is unresolved when the VSC has not returned to full power by the simulation horizon; unresolved values are null, never zero."})},
      {"settings", {
           {"fault_bus_ac", kFaultBus},
           {"vsc_index", kVscIndex},
           {"fault_time_s", kFaultTimeS},
           {"finite_comparison_horizon_s", kFiniteHorizonS},
           {"reconnect_delay_s", kReconnectDelayS},
           {"power_ramp_s", kPowerRampS},
           {"terminal_time_s", terminal_time_s},
           {"dt_s", dt_s},
           {"dc_local_generation_in_service", false},
           {"vsc_ac_power_schedule_mw",
            -kAffectedDcLoadMw / kStudyConverterEfficiency},
           {"vsc_reactive_support_mvar", kConverterReactiveSupportMvar},
           {"vsc_rating_mva", kStudyConverterRatingMva},
           {"vsc_current_limit_pu_converter_base", kConverterCurrentHeadroom},
           {"vsc_current_limit_pu_system_base",
            kConverterCurrentHeadroom * kStudyConverterRatingMva /
                kStudySystemBaseMva},
           {"adequacy_bias_limit", kAdequacyBiasLimit}}},
      {"summary", {
           {"scenario_count", rows.size()},
           {"valid_scenario_count", valid},
           {"failed_scenario_count", static_cast<int>(rows.size()) - valid},
           {"p_protective_trip_uniform_grid", valid > 0 ? static_cast<double>(tripped) / valid : 0.0},
           {"p_terminal_blocked_at_horizon_uniform_grid", valid > 0 ? static_cast<double>(terminal_blocked) / valid : 0.0},
           {"p_reconnected_by_horizon_uniform_grid", valid > 0 ? static_cast<double>(reconnected) / valid : 0.0},
           {"p_momentary_cessation_uniform_grid", valid > 0 ? static_cast<double>(momentary) / valid : 0.0},
           {"current_limit_active_scenario_count", current_limited},
           {"ride_through_count", ride_through},
           {"chronology_valid", chronology_valid},
           {"reconnect_delay_lower_bound_valid", reconnect_delay_valid},
           {"finite_horizon_ordering_valid", finite_horizon_ordering_valid},
           {"ride_through_equivalence_tolerance_mwh",
            kEnsEquivalenceToleranceMwh},
           {"ride_through_equivalence_valid",
            ride_through_equivalence_valid},
           {"analytical_reconstruction_valid",
            analytical_reconstruction_valid}}},
      {"trip_reason_counts", trip_reason_counts},
      {"finite_horizon_static_transient_comparison",
       std::move(finite_horizon_summary)},
      {"static_model_boundary", std::move(boundary)},
      {"scenarios", json::array()}};
  for (const auto& row : rows) output["scenarios"].push_back(scenario_json(row));

  if (!json_path.empty()) {
    std::ofstream stream(json_path);
    if (!stream) throw std::runtime_error("cannot open JSON output: " + json_path.string());
    stream << std::setw(2) << output << '\n';
  } else {
    std::cout << std::setw(2) << output << '\n';
  }
  if (!csv_path.empty()) write_csv(csv_path, rows);

  if (valid != static_cast<int>(rows.size()) || !chronology_valid ||
      !reconnect_delay_valid || !finite_horizon_ordering_valid ||
      !ride_through_equivalence_valid || !analytical_reconstruction_valid ||
      tripped == 0 || ride_through == 0 || current_limited == 0) {
    std::cerr << "study validation failed: valid=" << valid << '/' << rows.size()
              << " tripped=" << tripped << " ride_through=" << ride_through
              << " current_limited=" << current_limited
              << " chronology=" << chronology_valid
              << " reconnect_delay=" << reconnect_delay_valid
              << " finite_ordering=" << finite_horizon_ordering_valid
              << " ride_through_equivalence="
              << ride_through_equivalence_valid
              << " analytical_reconstruction="
              << analytical_reconstruction_valid << '\n';
    return 2;
  }
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    return run(argc, argv);
  } catch (const std::exception& error) {
    std::cerr << "terminal-state boundary study failed: " << error.what() << '\n';
    return 1;
  }
}
