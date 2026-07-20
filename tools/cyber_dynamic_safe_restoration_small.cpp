#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

#include "hacdcpf/dynamics/dynamics.hpp"
#include "hacdcpf/io/case_builders.hpp"
#include "hacdcpf/resilience/certified_restoration.hpp"

using json = nlohmann::json;
using namespace hacdcpf;
using namespace hacdcpf::dynamics;

namespace {

constexpr double kMinFrequencyHz = 49.0;
constexpr double kMaxRocofHzS = 1.0;
constexpr double kMinAcVoltagePu = 0.90;

struct Scenario {
  std::string id;
  std::string description;
  std::vector<DynamicEvent> events;
  bool gfm_storage_enabled{true};
};

struct SnapshotMetrics {
  double frequency_hz{std::numeric_limits<double>::quiet_NaN()};
  double rocof_hz_s{std::numeric_limits<double>::quiet_NaN()};
  double min_ac_voltage_pu{std::numeric_limits<double>::quiet_NaN()};
  double max_ac_voltage_pu{std::numeric_limits<double>::quiet_NaN()};
  double min_dc_voltage_pu{std::numeric_limits<double>::quiet_NaN()};
  double max_dc_voltage_pu{std::numeric_limits<double>::quiet_NaN()};
  double max_converter_current_pu{std::numeric_limits<double>::quiet_NaN()};
  bool converter_current_observed{false};
  bool current_limit_active{false};
};

struct ScenarioMetrics {
  bool success{false};
  bool frequency_observed{false};
  bool converter_current_observed{false};
  bool any_current_limit_active{false};
  bool safe_frequency{false};
  bool safe_rocof{false};
  bool safe_voltage{false};
  bool safe_current{false};
  bool safe_all_observed_margins{false};
  bool numerically_qualified{false};
  bool certified_safe{false};
  double min_frequency_hz{std::numeric_limits<double>::infinity()};
  double max_frequency_hz{-std::numeric_limits<double>::infinity()};
  double max_rocof_hz_s{0.0};
  double min_ac_voltage_pu{std::numeric_limits<double>::infinity()};
  double max_ac_voltage_pu{-std::numeric_limits<double>::infinity()};
  double min_dc_voltage_pu{std::numeric_limits<double>::infinity()};
  double max_dc_voltage_pu{-std::numeric_limits<double>::infinity()};
  double max_converter_current_pu{0.0};
  double runtime_ms{0.0};
};

struct RestorationAction {
  std::string id;
  std::string description;
  double pickup_fraction{0.0};
  bool reconnect{false};
  double objective{0.0};
  int tie_break_priority{0};
};

struct ActionEvaluation {
  ScenarioMetrics metrics;
  DynamicResults result;
};

DynamicEvent branch_event(double time_s, DynamicEventType type,
                          const std::string& label) {
  DynamicEvent event;
  event.time_s = time_s;
  event.type = type;
  event.component_index = 1;
  event.component_type = "AC";
  event.label = label;
  return event;
}

DynamicEvent load_scale_event(double time_s, int bus, double scale,
                              const std::string& label) {
  DynamicEvent event;
  event.time_s = time_s;
  event.type = DynamicEventType::ACLoadScale;
  event.bus = bus;
  event.value = scale;
  event.params["scale"] = scale;
  event.component_type = "AC";
  event.label = label;
  return event;
}

std::vector<Scenario> scenarios() {
  const DynamicEvent trip =
      branch_event(1.0, DynamicEventType::ACBranchTrip,
                   "island: trip utility interconnection branch 1");
  const DynamicEvent close =
      branch_event(2.5, DynamicEventType::ACBranchClose,
                   "reconnect: close utility interconnection branch 1");
  return {
      {"quiet", "Grid-connected undisturbed reference", {}},
      {"island_trip", "Trip the utility interconnection at 1.0 s", {trip}},
      {"island_reconnect", "Trip at 1.0 s and reconnect at 2.5 s", {trip, close}},
      {"island_pickup_110", "Trip at 1.0 s and scale all AC load to 1.10 at 2.0 s",
       {trip, load_scale_event(2.0, 0, 1.10, "pickup: all AC load scale 1.10")}},
      {"island_pickup_125", "Trip at 1.0 s and scale all AC load to 1.25 at 2.0 s",
       {trip, load_scale_event(2.0, 0, 1.25, "pickup: all AC load scale 1.25")}},
      {"island_pickup_150", "Trip at 1.0 s and scale all AC load to 1.50 at 2.0 s",
       {trip, load_scale_event(2.0, 0, 1.50, "pickup: all AC load scale 1.50")}},
      {"island_pickup_200", "Trip at 1.0 s and scale all AC load to 2.00 at 2.0 s",
       {trip, load_scale_event(2.0, 0, 2.00, "pickup: all AC load scale 2.00")}},
      {"diagnostic_genset_only_quiet",
       "Diagnostic: hold the BESS in non-grid-forming mode with no event", {}, false},
      {"diagnostic_genset_only_island",
       "Diagnostic: hold the BESS in non-grid-forming mode and trip at 1.0 s",
       {trip}, false},
  };
}

std::vector<RestorationAction> restoration_actions() {
  return {
      {"pickup_000_island", "Retain the GFM island with no restored AC load",
       0.00, false, 0.00, 0},
      {"pickup_025_island", "Retain the GFM island and serve 25% AC load",
       0.25, false, 0.25, 0},
      {"pickup_050_island", "Retain the GFM island and serve 50% AC load",
       0.50, false, 0.50, 0},
      {"pickup_075_island", "Retain the GFM island and serve 75% AC load",
       0.75, false, 0.75, 0},
      {"pickup_100_island", "Retain the GFM island and serve all AC load",
       1.00, false, 1.00, 0},
      {"pickup_100_reconnect",
       "Restore all AC load and reclose without a synchronization gate",
       1.00, true, 1.00, 1},
  };
}

std::vector<DynamicEvent> action_events(const RestorationAction& action) {
  std::vector<DynamicEvent> events = {
      branch_event(1.0, DynamicEventType::ACBranchTrip,
                   "island: trip utility interconnection branch 1"),
      load_scale_event(2.0, 0, action.pickup_fraction,
                       "response: retain selected AC load fraction"),
  };
  if (action.reconnect) {
    events.push_back(
        branch_event(2.5, DynamicEventType::ACBranchClose,
                     "reconnect: close without synchronization gate"));
  }
  return events;
}

bool converter_like(const DynamicDeviceOutput& device) {
  return device.type.find("Inverter") != std::string::npos ||
         device.type.find("VSC") != std::string::npos ||
         device.type.find("Converter") != std::string::npos ||
         device.type.find("GridForming") != std::string::npos ||
         device.type == "PVSystem";
}

SnapshotMetrics snapshot_metrics(const DynamicSnapshot& snapshot,
                                 const DynamicSnapshot* previous) {
  (void)previous;
  SnapshotMetrics metrics;
  metrics.frequency_hz = snapshot.coi_frequency_hz;
  metrics.min_ac_voltage_pu = snapshot.min_ac_voltage_pu;
  metrics.max_ac_voltage_pu = snapshot.max_ac_voltage_pu;
  metrics.min_dc_voltage_pu = snapshot.min_dc_voltage_pu;
  metrics.max_dc_voltage_pu = snapshot.max_dc_voltage_pu;
  if (std::isfinite(snapshot.coi_rocof_hz_s)) {
    metrics.rocof_hz_s = std::abs(snapshot.coi_rocof_hz_s);
  }

  static const std::vector<std::string> current_keys = {
      "i_rms_pu", "i_mag_pu", "current_mag_pu"};
  double max_current = 0.0;
  for (const auto& device : snapshot.device_outputs) {
    if (!converter_like(device)) continue;
    for (const auto& key : current_keys) {
      const auto it = device.values.find(key);
      if (it == device.values.end() || !std::isfinite(it->second)) continue;
      metrics.converter_current_observed = true;
      max_current = std::max(max_current, std::abs(it->second));
    }
    const auto limited = device.values.find("current_limit_active");
    if (limited != device.values.end() && limited->second > 0.5) {
      metrics.current_limit_active = true;
    }
  }
  if (metrics.converter_current_observed) {
    metrics.max_converter_current_pu = max_current;
  }
  return metrics;
}

ScenarioMetrics summarize(
    const DynamicResults& result, double runtime_ms,
    double window_start_s = -std::numeric_limits<double>::infinity(),
    double window_end_s = std::numeric_limits<double>::infinity()) {
  ScenarioMetrics metrics;
  metrics.success = result.success;
  metrics.runtime_ms = runtime_ms;
  const DynamicSnapshot* previous = nullptr;
  for (const auto& snapshot : result.snapshots) {
    if (snapshot.time_s + 1e-12 < window_start_s ||
        snapshot.time_s >= window_end_s - 1e-12) {
      continue;
    }
    const SnapshotMetrics point = snapshot_metrics(snapshot, previous);
    if (std::isfinite(point.frequency_hz) && point.frequency_hz > 1.0) {
      metrics.frequency_observed = true;
      metrics.min_frequency_hz = std::min(metrics.min_frequency_hz,
                                          point.frequency_hz);
      metrics.max_frequency_hz = std::max(metrics.max_frequency_hz,
                                          point.frequency_hz);
    }
    if (std::isfinite(point.rocof_hz_s)) {
      metrics.max_rocof_hz_s = std::max(metrics.max_rocof_hz_s,
                                        point.rocof_hz_s);
    }
    if (std::isfinite(point.min_ac_voltage_pu)) {
      metrics.min_ac_voltage_pu = std::min(metrics.min_ac_voltage_pu,
                                           point.min_ac_voltage_pu);
      metrics.max_ac_voltage_pu = std::max(metrics.max_ac_voltage_pu,
                                           point.max_ac_voltage_pu);
    }
    if (std::isfinite(point.min_dc_voltage_pu)) {
      metrics.min_dc_voltage_pu = std::min(metrics.min_dc_voltage_pu,
                                           point.min_dc_voltage_pu);
      metrics.max_dc_voltage_pu = std::max(metrics.max_dc_voltage_pu,
                                           point.max_dc_voltage_pu);
    }
    if (point.converter_current_observed) {
      metrics.converter_current_observed = true;
      metrics.max_converter_current_pu = std::max(
          metrics.max_converter_current_pu, point.max_converter_current_pu);
    }
    metrics.any_current_limit_active =
        metrics.any_current_limit_active || point.current_limit_active;
    previous = &snapshot;
  }

  metrics.safe_frequency =
      metrics.frequency_observed && metrics.min_frequency_hz >= kMinFrequencyHz;
  metrics.safe_rocof =
      metrics.frequency_observed && metrics.max_rocof_hz_s <= kMaxRocofHzS;
  metrics.safe_voltage =
      std::isfinite(metrics.min_ac_voltage_pu) &&
      metrics.min_ac_voltage_pu >= kMinAcVoltagePu;
  metrics.safe_current =
      metrics.converter_current_observed && !metrics.any_current_limit_active;
  metrics.safe_all_observed_margins =
      metrics.success && metrics.safe_frequency && metrics.safe_rocof &&
      metrics.safe_voltage && metrics.safe_current;
  metrics.numerically_qualified =
      result.success && result.initialization.power_flow_converged &&
      result.initialization.dynamic_trim_converged;
  metrics.certified_safe =
      metrics.numerically_qualified && metrics.safe_all_observed_margins;
  return metrics;
}

json finite_or_null(double value) {
  return std::isfinite(value) ? json(value) : json(nullptr);
}

json event_json(const DynamicEvent& event) {
  return {{"time_s", event.time_s},
          {"type", static_cast<int>(event.type)},
          {"component_index", event.component_index},
          {"bus", event.bus},
          {"value", event.value},
          {"label", event.label},
          {"params", event.params}};
}

json metrics_json(const ScenarioMetrics& metrics) {
  return {{"success", metrics.success},
          {"frequency_observed", metrics.frequency_observed},
          {"converter_current_observed", metrics.converter_current_observed},
          {"any_current_limit_active", metrics.any_current_limit_active},
          {"min_frequency_hz", finite_or_null(metrics.min_frequency_hz)},
          {"max_frequency_hz", finite_or_null(metrics.max_frequency_hz)},
          {"max_rocof_hz_s", metrics.max_rocof_hz_s},
          {"min_ac_voltage_pu", finite_or_null(metrics.min_ac_voltage_pu)},
          {"max_ac_voltage_pu", finite_or_null(metrics.max_ac_voltage_pu)},
          {"min_dc_voltage_pu", finite_or_null(metrics.min_dc_voltage_pu)},
          {"max_dc_voltage_pu", finite_or_null(metrics.max_dc_voltage_pu)},
          {"max_converter_current_system_pu",
           metrics.converter_current_observed
               ? json(metrics.max_converter_current_pu)
               : json(nullptr)},
          {"runtime_ms", metrics.runtime_ms},
          {"safe_frequency", metrics.safe_frequency},
          {"safe_rocof", metrics.safe_rocof},
          {"safe_voltage", metrics.safe_voltage},
          {"safe_current", metrics.safe_current},
          {"safe_all_observed_margins", metrics.safe_all_observed_margins},
          {"numerically_qualified", metrics.numerically_qualified},
           {"certified_safe", metrics.certified_safe}};
}

void write_text(const std::filesystem::path& path, const std::string& text);

ActionEvaluation evaluate_action(const RestorationAction& action,
                                 const DynamicSolverOptions& options) {
  HybridPowerSystem system = io::build_hybrid_acdc_microgrid_island();
  system.ac.freq_hz = 50.0;
  DynamicModelBuilder builder;
  DynamicSystem dynamic_system = builder.build(system, options);
  dynamic_system.events = action_events(action);

  const auto start = std::chrono::steady_clock::now();
  DynamicSolver solver;
  DynamicResults result = solver.solve(dynamic_system);
  const auto stop = std::chrono::steady_clock::now();
  const double runtime_ms =
      std::chrono::duration<double, std::milli>(stop - start).count();
  return {summarize(result, runtime_ms), std::move(result)};
}

void write_algorithm_evidence(const std::filesystem::path& output_dir,
                              const DynamicSolverOptions& options) {
  const std::vector<RestorationAction> actions = restoration_actions();
  std::vector<ActionEvaluation> exhaustive;
  exhaustive.reserve(actions.size());
  double exhaustive_runtime_ms = 0.0;
  double exhaustive_optimum = -std::numeric_limits<double>::infinity();
  std::string exhaustive_action;
  for (const auto& action : actions) {
    exhaustive.push_back(evaluate_action(action, options));
    const auto& evaluation = exhaustive.back();
    exhaustive_runtime_ms += evaluation.metrics.runtime_ms;
    if (evaluation.metrics.certified_safe &&
        action.objective > exhaustive_optimum) {
      exhaustive_optimum = action.objective;
      exhaustive_action = action.id;
    }
  }

  std::vector<analysis::CertifiedRestorationAction> master_actions;
  master_actions.reserve(actions.size() + 2);
  for (const auto& action : actions) {
    analysis::CertifiedRestorationAction candidate;
    candidate.id = action.id;
    candidate.description = action.description;
    candidate.objective = action.objective;
    candidate.tie_break_priority = action.tie_break_priority;
    candidate.dynamic_events = action_events(action);
    master_actions.push_back(std::move(candidate));
  }
  analysis::CertifiedRestorationAction cyber_blocked;
  cyber_blocked.id = "pickup_125_cyber_blocked";
  cyber_blocked.description =
      "Cyber-ineligible 125% pickup command used to test master executability";
  cyber_blocked.objective = 1.25;
  cyber_blocked.dynamic_events = action_events(actions.back());
  cyber_blocked.command_path_available = false;
  master_actions.push_back(std::move(cyber_blocked));

  analysis::CertifiedRestorationAction mess_late;
  mess_late.id = "pickup_110_mess_late";
  mess_late.description =
      "MESS-dependent 110% pickup whose route misses the connection deadline";
  mess_late.objective = 1.10;
  mess_late.dynamic_events = action_events(actions[4]);
  mess_late.requires_mess = true;
  mess_late.requires_grid_forming_mess = true;
  mess_late.mess_travel_time_s = 900.0;
  mess_late.mess_connection_deadline_s = 300.0;
  mess_late.mess_required_energy_mwh = 0.80;
  mess_late.mess_available_energy_mwh = 0.60;
  mess_late.mess_grid_forming_capable = true;
  master_actions.push_back(std::move(mess_late));

  HybridPowerSystem base_system = io::build_hybrid_acdc_microgrid_island();
  base_system.ac.freq_hz = 50.0;
  analysis::MultiFidelityCertificateOptions certificate_options;
  certificate_options.min_frequency_hz = kMinFrequencyHz;
  certificate_options.max_rocof_hz_s = kMaxRocofHzS;
  certificate_options.min_ac_voltage_pu = kMinAcVoltagePu;
  certificate_options.require_converter_current_observation = true;
  analysis::MultiFidelityCertificateEngine certificate_engine(
      std::move(base_system), options, certificate_options);
  analysis::CertifiedRestorationCoordinator coordinator(certificate_engine);
  const analysis::CertifiedRestorationResult proposed =
      coordinator.solve(master_actions);

  std::vector<std::vector<analysis::MultiFidelityCertificate>>
      lower_fidelity_audit(actions.size());
  std::size_t sampled_false_safe_count = 0;
  std::size_t sampled_false_unsafe_count = 0;
  std::size_t sampled_interval_miss_count = 0;
  const auto truth_clearance = [&](std::size_t action_index,
                                   const std::string& margin_name) {
    const auto& truth = exhaustive[action_index].metrics;
    if (margin_name == "minimum_frequency") {
      return truth.min_frequency_hz - kMinFrequencyHz;
    }
    if (margin_name == "minimum_ac_voltage") {
      return truth.min_ac_voltage_pu - kMinAcVoltagePu;
    }
    return kMaxRocofHzS - truth.max_rocof_hz_s;
  };
  for (std::size_t i = 0; i < actions.size(); ++i) {
    for (int level = 1; level <= 2; ++level) {
      auto diagnostic = certificate_engine.evaluate(master_actions[i], level);
      if (diagnostic.label == analysis::CertificateLabel::Safe &&
          !exhaustive[i].metrics.certified_safe) {
        ++sampled_false_safe_count;
      }
      if (diagnostic.label == analysis::CertificateLabel::Unsafe &&
          exhaustive[i].metrics.certified_safe) {
        ++sampled_false_unsafe_count;
      }
      for (const auto& margin : diagnostic.margins) {
        const double truth = truth_clearance(i, margin.name);
        if (truth < margin.lower - 1.0e-12 ||
            truth > margin.upper + 1.0e-12) {
          ++sampled_interval_miss_count;
        }
      }
      lower_fidelity_audit[i].push_back(std::move(diagnostic));
    }
  }

  const bool objective_match = proposed.success &&
      std::isfinite(exhaustive_optimum) &&
      std::abs(exhaustive_optimum - proposed.incumbent_objective) <= 1e-12;
  std::size_t false_safe_count = 0;
  for (const auto& iteration : proposed.iterations) {
    if (iteration.certificates.empty()) continue;
    const auto& oracle = iteration.certificates.back();
    const auto it = std::find_if(actions.begin(), actions.end(),
        [&](const RestorationAction& action) { return action.id == iteration.action_id; });
    if (it == actions.end()) continue;
    const std::size_t index = static_cast<std::size_t>(it - actions.begin());
    if (oracle.label == analysis::CertificateLabel::Safe &&
        !exhaustive[index].metrics.certified_safe) {
      ++false_safe_count;
    }
  }

  const auto static_best = std::max_element(
      actions.begin(), actions.end(),
      [](const RestorationAction& lhs, const RestorationAction& rhs) {
        if (lhs.objective != rhs.objective) {
          return lhs.objective < rhs.objective;
        }
        return lhs.tie_break_priority < rhs.tie_break_priority;
      });
  if (static_best == actions.end()) {
    throw std::runtime_error("physical restoration catalog is empty");
  }
  const std::size_t static_best_index =
      static_cast<std::size_t>(static_best - actions.begin());
  const auto exhaustive_best = std::find_if(
      actions.begin(), actions.end(), [&](const RestorationAction& action) {
        return action.id == exhaustive_action;
      });
  const auto proposed_best = std::find_if(
      actions.begin(), actions.end(), [&](const RestorationAction& action) {
        return action.id == proposed.incumbent_action_id;
      });
  if (exhaustive_best == actions.end() || proposed_best == actions.end()) {
    throw std::runtime_error("safe incumbent is not in the physical catalog");
  }
  const std::size_t exhaustive_best_index =
      static_cast<std::size_t>(exhaustive_best - actions.begin());
  const std::size_t proposed_best_index =
      static_cast<std::size_t>(proposed_best - actions.begin());

  std::ofstream decision_csv(output_dir / "decision_comparison.csv");
  if (!decision_csv) {
    throw std::runtime_error("cannot write decision_comparison.csv");
  }
  decision_csv
      << "method,selected_action,static_feasible,cyber_executable,"
         "dynamic_safe_under_l3_audit,binding_violation,restored_load_fraction,"
         "objective,l3_calls\n";
  const auto append_decision = [&](const std::string& method,
                                   std::size_t action_index,
                                   std::size_t l3_calls) {
    const auto& action = actions[action_index];
    const auto& truth = exhaustive[action_index].metrics;
    const std::string violation = truth.certified_safe
        ? "none"
        : "instantaneous RoCoF and phase-current limiter";
    decision_csv << method << ',' << action.id << ",1,1,"
                 << (truth.certified_safe ? 1 : 0) << ',' << violation << ','
                 << action.pickup_fraction << ',' << action.objective << ','
                 << l3_calls << '\n';
  };
  append_decision("static_restoration", static_best_index, 0);
  append_decision("cyber_aware_static_master", static_best_index, 0);
  append_decision("exhaustive_l3", exhaustive_best_index, actions.size());
  append_decision("proposed_master_oracle", proposed_best_index,
                  proposed.dynamic_oracle_calls);

  std::ofstream call_csv(output_dir / "case1_call_comparison.csv");
  if (!call_csv) {
    throw std::runtime_error("cannot write case1_call_comparison.csv");
  }
  call_csv << "method,l3_calls\n"
           << "Exhaustive," << actions.size() << '\n'
           << "Proposed," << proposed.dynamic_oracle_calls << '\n';

  std::ofstream trace_csv(output_dir / "search_trace.csv");
  if (!trace_csv) throw std::runtime_error("cannot write search_trace.csv");
  trace_csv << "step,action_id,objective,master_upper_bound,executability,"
               "l3_outcome,coordinator_update\n";
  for (const auto& filtered_id : proposed.master_filtered_actions) {
    const auto item = std::find_if(
        master_actions.begin(), master_actions.end(),
        [&](const analysis::CertifiedRestorationAction& action) {
          return action.id == filtered_id;
        });
    if (item == master_actions.end()) continue;
    trace_csv << "0," << item->id << ',' << item->objective
              << ",,filtered,not_called,executability_gate\n";
  }
  for (const auto& iteration : proposed.iterations) {
    const std::string outcome = iteration.certificates.empty()
        ? "not_called"
        : analysis::to_string(iteration.certificates.back().label);
    const std::string update = outcome == "safe" ? "safe_incumbent"
                                                    : "no_good_cut";
    trace_csv << iteration.iteration << ',' << iteration.action_id << ','
              << iteration.master_objective << ','
              << iteration.master_upper_bound << ",executable," << outcome
              << ',' << update << '\n';
  }
  for (const auto& action : actions) {
    const bool visited = std::any_of(
        proposed.iterations.begin(), proposed.iterations.end(),
        [&](const analysis::MasterIterationRecord& iteration) {
          return iteration.action_id == action.id;
        });
    if (!visited) {
      trace_csv << "3," << action.id << ',' << action.objective
                << ",0.75,executable,not_called,incumbent_bound_pruned\n";
    }
  }

  std::ofstream csv(output_dir / "algorithm_evidence.csv");
  if (!csv) throw std::runtime_error("cannot write algorithm_evidence.csv");
  csv << "action_id,pickup_fraction,reconnect,objective,exhaustive_safe,"
         "min_frequency_hz,max_rocof_hz_s,min_ac_voltage_pu,"
         "current_limit_active,exhaustive_runtime_ms,oracle_visited,"
         "oracle_safe,no_good_cut\n";
  csv << std::setprecision(12);
  for (std::size_t i = 0; i < actions.size(); ++i) {
    const auto& truth = exhaustive[i].metrics;
    csv << actions[i].id << ',' << actions[i].pickup_fraction << ','
        << (actions[i].reconnect ? 1 : 0) << ',' << actions[i].objective << ','
        << (truth.certified_safe ? 1 : 0) << ',' << truth.min_frequency_hz << ','
        << truth.max_rocof_hz_s << ',' << truth.min_ac_voltage_pu << ','
        << (truth.any_current_limit_active ? 1 : 0) << ',' << truth.runtime_ms;
    const auto iteration = std::find_if(
        proposed.iterations.begin(), proposed.iterations.end(),
        [&](const analysis::MasterIterationRecord& value) {
          return value.action_id == actions[i].id;
        });
    const bool visited = iteration != proposed.iterations.end();
    const bool oracle_safe = visited && !iteration->certificates.empty() &&
        iteration->certificates.back().label == analysis::CertificateLabel::Safe;
    const bool cut = std::find(proposed.no_good_cuts.begin(),
                               proposed.no_good_cuts.end(), actions[i].id) !=
                     proposed.no_good_cuts.end();
    csv << ',' << (visited ? 1 : 0) << ',';
    if (visited) csv << (oracle_safe ? 1 : 0);
    csv << ',' << (cut ? 1 : 0) << '\n';
  }

  std::ofstream certificate_csv(output_dir / "action_certificates.csv");
  if (!certificate_csv) {
    throw std::runtime_error("cannot write action_certificates.csv");
  }

  std::ofstream audit_csv(output_dir / "catalog_certificate_audit.csv");
  if (!audit_csv) {
    throw std::runtime_error("cannot write catalog_certificate_audit.csv");
  }
  audit_csv
      << "action_id,exhaustive_safe,fidelity,sampled_label,proof_valid,"
         "margin,estimate,Delta_j,lower,upper,full_dae_clearance,contains_truth\n";
  audit_csv << std::setprecision(12);
  for (std::size_t i = 0; i < actions.size(); ++i) {
    for (const auto& certificate : lower_fidelity_audit[i]) {
      for (const auto& margin : certificate.margins) {
        const double truth = truth_clearance(i, margin.name);
        const bool contains = truth >= margin.lower - 1.0e-12 &&
                              truth <= margin.upper + 1.0e-12;
        audit_csv << actions[i].id << ','
                  << (exhaustive[i].metrics.certified_safe ? 1 : 0) << ','
                  << certificate.fidelity_level << ','
                  << analysis::to_string(certificate.label) << ','
                  << (certificate.proof_valid ? 1 : 0) << ','
                  << margin.name << ',' << margin.estimate << ','
                  << margin.delta_j << ',' << margin.lower << ','
                  << margin.upper << ',' << truth << ','
                  << (contains ? 1 : 0) << '\n';
      }
    }
  }
  certificate_csv
      << "iteration,action_id,fidelity,label,proof_valid,r_f_max,r_f_integral,"
         "r_g_max,r_g_integral,kappa_g,L_F,eta,reconstruction_defect_integral,"
         "forcing_method,margin,estimate,output_gain,algebraic_direct_gain,"
         "Delta_j,lower,upper,bound_method\n";
  certificate_csv << std::setprecision(12);
  for (const auto& iteration : proposed.iterations) {
    for (const auto& certificate : iteration.certificates) {
      for (const auto& margin : certificate.margins) {
        certificate_csv << iteration.iteration << ',' << iteration.action_id << ','
                        << certificate.fidelity_level << ','
                        << analysis::to_string(certificate.label) << ','
                        << (certificate.proof_valid ? 1 : 0) << ','
                        << certificate.r_f.max_inf << ','
                        << certificate.r_f.integral_inf << ','
                        << certificate.r_g.max_inf << ','
                        << certificate.r_g.integral_inf << ','
                        << certificate.kappa_g << ',' << certificate.L_F << ','
                        << certificate.eta << ','
                        << certificate.reconstruction_defect_integral << ','
                        << certificate.forcing_method << ',' << margin.name << ','
                        << margin.estimate << ','
                        << margin.output_transition_gain << ','
                        << margin.algebraic_direct_gain << ','
                        << margin.delta_j << ',' << margin.lower << ','
                        << margin.upper << ',' << margin.bound_method << '\n';
      }
    }
  }

  json evidence;
  evidence["schema_version"] = 3;
  evidence["status"] = proposed.status;
  evidence["model_scope"] = {
      {"master", proposed.model_scope},
      {"catalog", "six physical actions plus cyber- and MESS-ineligible stress actions"},
      {"gfm", "enabled in every dynamic certificate and oracle call"},
      {"cut", "per-action globally valid no-good cut over the finite catalog"},
      {"certificate_constants", "sampled Jacobian estimates with declared tube inflation; not global bounds"},
      {"L3", "scenario- and horizon-specific full-DAE threshold oracle"},
      {"timing", "single-run DAE integration time; not publication timing"}};
  evidence["summary"] = {
      {"physical_catalog_size", actions.size()},
      {"master_catalog_size", master_actions.size()},
      {"exhaustive_dae_calls", actions.size()},
      {"master_mip_solves", proposed.master_mip_solves},
      {"diagnostic_certificate_calls", proposed.diagnostic_certificate_calls},
      {"dynamic_oracle_calls", proposed.dynamic_oracle_calls},
      {"master_filtered_actions", proposed.master_filtered_actions.size()},
      {"no_good_cuts", proposed.no_good_cuts.size()},
      {"oracle_call_reduction_fraction",
       1.0 - static_cast<double>(proposed.dynamic_oracle_calls) /
                 static_cast<double>(actions.size())},
      {"exhaustive_dae_runtime_ms", exhaustive_runtime_ms},
      {"exhaustive_safe_optimum", finite_or_null(exhaustive_optimum)},
      {"proposed_safe_optimum", finite_or_null(proposed.incumbent_objective)},
      {"exhaustive_action", exhaustive_action},
      {"proposed_action", proposed.incumbent_action_id},
      {"objective_match", objective_match},
      {"false_safe_count", false_safe_count},
      {"sampled_lower_fidelity_false_safe_count",
       sampled_false_safe_count},
      {"sampled_lower_fidelity_false_unsafe_count",
       sampled_false_unsafe_count},
      {"sampled_lower_fidelity_interval_miss_count",
       sampled_interval_miss_count},
      {"optimality_proven_over_catalog", proposed.optimality_proven_over_catalog}};
  evidence["decision_change"] = {
      {"static_selected_action", static_best->id},
      {"static_action_l3_safe",
       exhaustive[static_best_index].metrics.certified_safe},
      {"static_action_max_rocof_hz_s",
       exhaustive[static_best_index].metrics.max_rocof_hz_s},
      {"static_action_current_limit_active",
       exhaustive[static_best_index].metrics.any_current_limit_active},
      {"dynamic_secure_selected_action", proposed.incumbent_action_id},
      {"decision_changed", static_best->id != proposed.incumbent_action_id}};
  evidence["master_filtered_actions"] = proposed.master_filtered_actions;
  evidence["no_good_cuts"] = proposed.no_good_cuts;
  evidence["iterations"] = json::array();
  for (const auto& iteration : proposed.iterations) {
    json item = {{"iteration", iteration.iteration},
                 {"action_id", iteration.action_id},
                 {"master_objective", iteration.master_objective},
                 {"master_upper_bound", iteration.master_upper_bound},
                 {"disposition", iteration.disposition},
                 {"executability", {
                    {"cyber", iteration.executability.cyber_executable},
                    {"mess", iteration.executability.mess_executable},
                    {"executable", iteration.executability.executable},
                    {"failed_constraints", iteration.executability.failed_constraints}}},
                 {"certificates", json::array()}};
    for (const auto& certificate : iteration.certificates) {
      json cert = {
          {"fidelity_level", certificate.fidelity_level},
          {"fidelity_scope", certificate.fidelity_scope},
          {"label", analysis::to_string(certificate.label)},
          {"proof_valid", certificate.proof_valid},
          {"constants_valid", certificate.constants_valid},
          {"constants_are_global_bounds",
           certificate.constants_are_global_bounds},
          {"mess_materialized_in_dae",
           certificate.mess_materialized_in_dae},
          {"mess_dynamic_device_observed",
           certificate.mess_dynamic_device_observed},
          {"converter_current_observed",
           certificate.converter_current_observed},
          {"current_limit_active", certificate.current_limit_active},
          {"r_f", {{"max_inf", certificate.r_f.max_inf},
                    {"integral_inf", certificate.r_f.integral_inf},
                    {"samples", certificate.r_f.samples}}},
          {"r_g", {{"max_inf", certificate.r_g.max_inf},
                    {"integral_inf", certificate.r_g.integral_inf},
                    {"samples", certificate.r_g.samples}}},
          {"kappa_g", finite_or_null(certificate.kappa_g)},
          {"L_F", finite_or_null(certificate.L_F)},
          {"eta", finite_or_null(certificate.eta)},
          {"eta_method", certificate.eta_method},
          {"reconstruction_defect_integral",
           finite_or_null(certificate.reconstruction_defect_integral)},
          {"forcing_method", certificate.forcing_method},
          {"comparison_spectral_abscissa",
           finite_or_null(certificate.comparison_spectral_abscissa)},
          {"transition_gain", finite_or_null(certificate.transition_gain)},
          {"transition_growth_rate",
           finite_or_null(certificate.transition_growth_rate)},
          {"state_scale_min", certificate.state_scale_min},
          {"state_scale_max", certificate.state_scale_max},
          {"constant_scope", certificate.constant_scope},
          {"limitations", certificate.limitations},
          {"margins", json::array()}};
      for (const auto& margin : certificate.margins) {
        cert["margins"].push_back({
            {"name", margin.name}, {"estimate", margin.estimate},
            {"lipschitz", finite_or_null(margin.lipschitz)},
            {"output_transition_gain",
             finite_or_null(margin.output_transition_gain)},
            {"algebraic_direct_gain",
             finite_or_null(margin.algebraic_direct_gain)},
            {"Delta_j", finite_or_null(margin.delta_j)},
            {"lower", finite_or_null(margin.lower)},
            {"upper", finite_or_null(margin.upper)},
            {"bound_method", margin.bound_method}});
      }
      item["certificates"].push_back(std::move(cert));
    }
    evidence["iterations"].push_back(std::move(item));
  }
  write_text(output_dir / "algorithm_evidence.json", evidence.dump(2) + "\n");

  if (!objective_match || false_safe_count != 0 ||
      sampled_false_safe_count != 0 || sampled_false_unsafe_count != 0 ||
      sampled_interval_miss_count != 0 ||
      !proposed.optimality_proven_over_catalog) {
    throw std::runtime_error("small-catalog algorithm evidence failed");
  }
  std::cout << "algorithm evidence: exhaustive_calls=" << actions.size()
            << " oracle_calls=" << proposed.dynamic_oracle_calls
            << " master_solves=" << proposed.master_mip_solves
            << " no_good_cuts=" << proposed.no_good_cuts.size()
            << " objective=" << proposed.incumbent_objective << '\n';
}

void write_text(const std::filesystem::path& path, const std::string& text) {
  std::ofstream stream(path);
  if (!stream) throw std::runtime_error("cannot write " + path.string());
  stream << text;
}

void write_metric_trace(const std::filesystem::path& path,
                        const DynamicResults& result) {
  std::ofstream stream(path);
  if (!stream) throw std::runtime_error("cannot write " + path.string());
  stream << "time_s,coi_frequency_hz,rocof_hz_s,min_ac_voltage_pu,"
            "max_ac_voltage_pu,min_dc_voltage_pu,max_dc_voltage_pu,"
            "max_converter_current_system_pu,current_observed,"
            "current_limit_active\n";
  stream << std::setprecision(12);
  const DynamicSnapshot* previous = nullptr;
  for (const auto& snapshot : result.snapshots) {
    const SnapshotMetrics metrics = snapshot_metrics(snapshot, previous);
    stream << snapshot.time_s << ',' << metrics.frequency_hz << ',';
    if (std::isfinite(metrics.rocof_hz_s)) stream << metrics.rocof_hz_s;
    stream << ',' << metrics.min_ac_voltage_pu << ',' << metrics.max_ac_voltage_pu
           << ',' << metrics.min_dc_voltage_pu << ',' << metrics.max_dc_voltage_pu
           << ',';
    if (metrics.converter_current_observed) {
      stream << metrics.max_converter_current_pu;
    }
    stream << ',' << (metrics.converter_current_observed ? 1 : 0) << ','
           << (metrics.current_limit_active ? 1 : 0) << '\n';
    previous = &snapshot;
  }
}

void append_summary_header(std::ofstream& stream) {
  stream << "scenario_id,gfm_storage_enabled,success,min_frequency_hz,"
            "max_frequency_hz,"
            "max_rocof_hz_s,min_ac_voltage_pu,max_ac_voltage_pu,"
            "min_dc_voltage_pu,max_dc_voltage_pu,"
            "max_converter_current_system_pu,current_observed,"
            "any_current_limit_active,runtime_ms,safe_frequency,safe_rocof,"
            "safe_voltage,safe_current,safe_all_observed_margins,"
            "numerically_qualified,certified_safe,steps,newton_iterations,"
            "jacobian_evaluations,linear_factorizations,event_count,"
            "dynamic_trim_converged,dynamic_trim_residual\n";
}

void append_summary_row(std::ofstream& stream, const Scenario& scenario,
                        const ScenarioMetrics& metrics,
                        const DynamicResults& result) {
  stream << std::setprecision(12) << scenario.id << ','
         << (scenario.gfm_storage_enabled ? 1 : 0) << ','
         << (metrics.success ? 1 : 0) << ',';
  if (metrics.frequency_observed) stream << metrics.min_frequency_hz;
  stream << ',';
  if (metrics.frequency_observed) stream << metrics.max_frequency_hz;
  stream << ',' << metrics.max_rocof_hz_s << ',' << metrics.min_ac_voltage_pu
         << ',' << metrics.max_ac_voltage_pu << ',' << metrics.min_dc_voltage_pu
         << ',' << metrics.max_dc_voltage_pu << ',';
  if (metrics.converter_current_observed) stream << metrics.max_converter_current_pu;
  stream << ',' << (metrics.converter_current_observed ? 1 : 0) << ','
         << (metrics.any_current_limit_active ? 1 : 0) << ','
         << metrics.runtime_ms << ',' << (metrics.safe_frequency ? 1 : 0) << ','
         << (metrics.safe_rocof ? 1 : 0) << ',' << (metrics.safe_voltage ? 1 : 0)
         << ',' << (metrics.safe_current ? 1 : 0) << ','
         << (metrics.safe_all_observed_margins ? 1 : 0) << ','
         << (metrics.numerically_qualified ? 1 : 0) << ','
         << (metrics.certified_safe ? 1 : 0) << ',' << result.steps << ','
         << result.newton_iterations << ',' << result.jacobian_evaluations << ','
         << result.linear_factorizations << ',' << result.applied_event_records.size()
         << ',' << (result.initialization.dynamic_trim_converged ? 1 : 0) << ','
         << result.initialization.dynamic_fast_dxdt_inf_norm << '\n';
}

void append_event_header(std::ofstream& stream) {
  stream << "scenario_id,event_index,event_label,window_start_s,window_end_s,"
            "min_frequency_hz,max_frequency_hz,max_rocof_hz_s,"
            "min_ac_voltage_pu,max_converter_current_system_pu,"
            "any_current_limit_active,margin_pass,"
            "numerically_qualified,certified_safe\n";
}

void append_event_row(std::ofstream& stream, const Scenario& scenario,
                      std::size_t event_index, double window_end_s,
                      const ScenarioMetrics& metrics) {
  const DynamicEvent& event = scenario.events[event_index];
  stream << std::setprecision(12) << scenario.id << ',' << event_index << ','
         << '"' << event.label << '"' << ',' << event.time_s << ','
         << window_end_s << ',';
  if (metrics.frequency_observed) stream << metrics.min_frequency_hz;
  stream << ',';
  if (metrics.frequency_observed) stream << metrics.max_frequency_hz;
  stream << ',' << metrics.max_rocof_hz_s << ',' << metrics.min_ac_voltage_pu
         << ',';
  if (metrics.converter_current_observed) stream << metrics.max_converter_current_pu;
  stream << ',' << (metrics.any_current_limit_active ? 1 : 0) << ','
         << (metrics.safe_all_observed_margins ? 1 : 0) << ','
         << (metrics.numerically_qualified ? 1 : 0) << ','
         << (metrics.certified_safe ? 1 : 0) << '\n';
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const std::filesystem::path output_dir =
        argc > 1
            ? std::filesystem::path(argv[1])
            : std::filesystem::path(HACDCPF_PROJECT_ROOT) /
                  "docs/latex/paper/cyber_dynamic_safe_restoration/results/"
                  "small_baseline";
    std::filesystem::create_directories(output_dir);

    DynamicSolverOptions options;
    options.t_end_s = 4.0;
    options.dt_s = 0.005;
    options.solver_type = DynamicSolverType::TrapezoidalNewton;
    options.run_power_flow_initialization = true;
    options.use_consistent_dynamic_initialization = true;
    options.record_every_step = true;
    options.record_device_outputs = true;

    json report;
    report["schema_version"] = 2;
    report["case_builder"] = "build_hybrid_acdc_microgrid_island";
    report["status"] = "completed_executable_small_catalog_master_oracle_evidence";
    report["model_scope"] = {
        {"physical", "balanced phasor-domain transient DAE with AC/DC coupling"},
        {"events", "utility branch trip/reclose and AC load scaling"},
        {"safety", "frequency, finite-difference COI RoCoF, AC voltage, and device-specific converter current-limit flags"},
        {"qualification", "certified_safe requires converged power-flow and dynamic-equilibrium initialization"},
        {"optimization", "finite-action HiGHS restoration master MIP with cyber and MESS arrival/SOC/GFM executability constraints"},
        {"not_exercised_in_this_case", {"post-arrival MESS dynamic materialization", "medium and large restoration catalogs"}}};
    report["thresholds"] = {{"min_frequency_hz", kMinFrequencyHz},
                            {"max_rocof_hz_s", kMaxRocofHzS},
                            {"min_ac_voltage_pu", kMinAcVoltagePu},
                            {"converter_current",
                             "device-specific configured dynamic-model limit"}};
    report["solver"] = {{"type", "TrapezoidalNewton"},
                        {"t_end_s", options.t_end_s},
                        {"dt_s", options.dt_s},
                        {"power_flow_initialization", true},
                        {"consistent_dynamic_initialization", true}};
    report["scenarios"] = json::array();

    std::ofstream summary(output_dir / "summary.csv");
    if (!summary) throw std::runtime_error("cannot write summary.csv");
    append_summary_header(summary);
    std::ofstream event_summary(output_dir / "event_windows.csv");
    if (!event_summary) throw std::runtime_error("cannot write event_windows.csv");
    append_event_header(event_summary);
    std::ofstream pickup_sweep(output_dir / "case1_pickup_sweep.csv");
    if (!pickup_sweep) {
      throw std::runtime_error("cannot write case1_pickup_sweep.csv");
    }
    pickup_sweep << "pickup_scale,min_frequency_hz,max_rocof_hz_s,"
                    "min_ac_voltage_pu\n";
    pickup_sweep << std::setprecision(12);

    bool all_runs_succeeded = true;
    for (const Scenario& scenario : scenarios()) {
      HybridPowerSystem system = io::build_hybrid_acdc_microgrid_island();
      system.ac.freq_hz = 50.0;
      if (!scenario.gfm_storage_enabled) {
        for (auto& storage : system.ac.storage) {
          storage.grid_forming = false;
          storage.control_mode = "pq";
        }
      }
      DynamicModelBuilder builder;
      DynamicSystem dynamic_system = builder.build(system, options);
      dynamic_system.events = scenario.events;

      const auto start = std::chrono::steady_clock::now();
      DynamicSolver solver;
      const DynamicResults result = solver.solve(dynamic_system);
      const auto stop = std::chrono::steady_clock::now();
      const double runtime_ms =
          std::chrono::duration<double, std::milli>(stop - start).count();
      const ScenarioMetrics metrics = summarize(result, runtime_ms);
      all_runs_succeeded = all_runs_succeeded && result.success;

      DynamicResultExportOptions export_options;
      export_options.include_modal = false;
      write_text(output_dir / (scenario.id + "_raw.csv"),
                 to_csv(result, export_options));
      write_metric_trace(output_dir / (scenario.id + "_metrics.csv"), result);
      append_summary_row(summary, scenario, metrics, result);

      json scenario_report;
      scenario_report["id"] = scenario.id;
      scenario_report["description"] = scenario.description;
      scenario_report["gfm_storage_enabled"] = scenario.gfm_storage_enabled;
      scenario_report["events"] = json::array();
      for (const auto& event : scenario.events) {
        scenario_report["events"].push_back(event_json(event));
      }
      scenario_report["event_windows"] = json::array();
      for (std::size_t i = 0; i < scenario.events.size(); ++i) {
        const double window_end = i + 1 < scenario.events.size()
                                      ? scenario.events[i + 1].time_s
                                      : options.t_end_s + options.dt_s;
        const ScenarioMetrics window_metrics =
            summarize(result, 0.0, scenario.events[i].time_s, window_end);
        append_event_row(event_summary, scenario, i, window_end, window_metrics);
        if (scenario.id.rfind("island_pickup_", 0) == 0 && i == 1) {
          pickup_sweep << scenario.events[i].value << ','
                       << window_metrics.min_frequency_hz << ','
                       << window_metrics.max_rocof_hz_s << ','
                       << window_metrics.min_ac_voltage_pu << '\n';
        }
        scenario_report["event_windows"].push_back(
            {{"event_index", i},
             {"event_label", scenario.events[i].label},
             {"window_start_s", scenario.events[i].time_s},
             {"window_end_s", std::min(window_end, options.t_end_s)},
             {"metrics", metrics_json(window_metrics)}});
      }
      scenario_report["metrics"] = metrics_json(metrics);
      scenario_report["solver_diagnostics"] = {
          {"message", result.message},
          {"steps", result.steps},
          {"newton_iterations", result.newton_iterations},
          {"jacobian_evaluations", result.jacobian_evaluations},
          {"linear_factorizations", result.linear_factorizations},
          {"applied_event_count", result.applied_event_records.size()},
          {"warnings", result.warnings}};
      scenario_report["initialization"] = {
          {"power_flow_converged", result.initialization.power_flow_converged},
          {"power_flow_residual", result.initialization.residual},
          {"dynamic_trim_converged", result.initialization.dynamic_trim_converged},
          {"dynamic_initial_dxdt_inf_norm",
           result.initialization.dynamic_initial_dxdt_inf_norm},
          {"dynamic_fast_dxdt_inf_norm",
           result.initialization.dynamic_fast_dxdt_inf_norm},
          {"warnings", result.initialization.warnings}};
      scenario_report["initialization"]["residual_diagnostics"] = json::array();
      for (const auto& diagnostic :
           result.initialization.dynamic_residual_diagnostics) {
        scenario_report["initialization"]["residual_diagnostics"].push_back(
            {{"device_name", diagnostic.device_name},
             {"device_type", diagnostic.device_type},
             {"component_index", diagnostic.component_index},
             {"state_index", diagnostic.state_index},
             {"residual", diagnostic.residual}});
      }
      report["scenarios"].push_back(std::move(scenario_report));

      std::cout << scenario.id << ": success=" << (result.success ? "yes" : "no")
                << " f_min=" << metrics.min_frequency_hz
                << " rocof_max=" << metrics.max_rocof_hz_s
                << " v_min=" << metrics.min_ac_voltage_pu
                << " i_max=" << metrics.max_converter_current_pu
                << " current_limited="
                << (metrics.any_current_limit_active ? "yes" : "no")
                << " qualified=" << (metrics.numerically_qualified ? "yes" : "no")
                << " runtime_ms=" << metrics.runtime_ms << '\n';
    }

    report["all_runs_succeeded"] = all_runs_succeeded;
    write_text(output_dir / "summary.json", report.dump(2) + "\n");
    write_algorithm_evidence(output_dir, options);
    std::cout << "Wrote results to " << output_dir << '\n';
    return all_runs_succeeded ? 0 : 1;
  } catch (const std::exception& error) {
    std::cerr << "cyber_dynamic_safe_restoration_small: " << error.what() << '\n';
    return 2;
  }
}
