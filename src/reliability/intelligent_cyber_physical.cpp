#include "hacdcpf/reliability/intelligent_cyber_physical.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <numeric>
#include <queue>
#include <random>
#include <stdexcept>
#include <unordered_map>

namespace hacdcpf::analysis {
namespace {

constexpr double kProbabilityTolerance = 1e-12;
constexpr double kHoursPerYear = 8760.0;

void require_probability(double value, const char* name) {
  if (!std::isfinite(value) || value < 0.0 || value > 1.0)
    throw std::invalid_argument(std::string(name) + " must be in [0,1]");
}

void require_probability_row(const std::vector<double>& row,
                             size_t required_size, const char* name) {
  if (row.size() != required_size)
    throw std::invalid_argument(std::string(name) + " has an invalid dimension");
  double sum = 0.0;
  for (double value : row) {
    require_probability(value, name);
    sum += value;
  }
  if (std::abs(sum - 1.0) > kProbabilityTolerance)
    throw std::invalid_argument(std::string(name) + " must sum to one");
}

bool all_functions_available(const std::vector<size_t>& required,
                             const std::vector<bool>& available) {
  for (size_t index : required) {
    if (index >= available.size())
      throw std::invalid_argument("required information-function index is out of range");
    if (!available[index]) return false;
  }
  return true;
}

double dot(const std::vector<double>& a, const std::vector<double>& b) {
  if (a.size() != b.size()) throw std::logic_error("dot-product dimension mismatch");
  return std::inner_product(a.begin(), a.end(), b.begin(), 0.0);
}

struct POMDPEvaluator {
  const FinitePOMDPModel& model;
  int nodes{0};

  std::pair<double, int> evaluate(const std::vector<double>& belief, int steps) {
    ++nodes;
    if (steps == 0) return {dot(belief, model.terminal_cost), -1};

    double best = std::numeric_limits<double>::infinity();
    int best_action = -1;
    const size_t n_state = model.state_ids.size();
    const size_t n_observation = model.observation_ids.size();
    for (size_t action = 0; action < model.action_ids.size(); ++action) {
      double cost = 0.0;
      for (size_t state = 0; state < n_state; ++state)
        cost += belief[state] * model.stage_cost[state][action];

      std::vector<double> predicted(n_state, 0.0);
      for (size_t state = 0; state < n_state; ++state)
        for (size_t next = 0; next < n_state; ++next)
          predicted[next] += belief[state] *
              model.transition_probability[action][state][next];

      for (size_t observation = 0; observation < n_observation; ++observation) {
        double p_observation = 0.0;
        std::vector<double> posterior(n_state, 0.0);
        for (size_t next = 0; next < n_state; ++next) {
          posterior[next] = predicted[next] *
              model.observation_probability[action][next][observation];
          p_observation += posterior[next];
        }
        if (p_observation <= kProbabilityTolerance) continue;
        for (double& value : posterior) value /= p_observation;
        cost += p_observation * evaluate(posterior, steps - 1).first;
      }
      if (cost < best) {
        best = cost;
        best_action = static_cast<int>(action);
      }
    }
    return {best, best_action};
  }
};

enum class QueueEventKind { CyberFailure, CyberRepair, CommonCauseStart,
                            CommonCauseEnd, PhysicalFault, PowerRestore };

struct QueueEvent {
  double time_hr{0.0};
  QueueEventKind kind{QueueEventKind::CyberFailure};
  size_t index{0};
  uint64_t order{0};
  bool operator>(const QueueEvent& other) const {
    return time_hr != other.time_hr ? time_hr > other.time_hr
                                    : order > other.order;
  }
};

struct ShedInterval {
  double begin_hr{0.0};
  double end_hr{0.0};
};

double exponential_wait_hr(std::mt19937_64& rng, double rate_per_hour) {
  if (rate_per_hour <= 0.0) return std::numeric_limits<double>::infinity();
  return std::exponential_distribution<double>(rate_per_hour)(rng);
}

ProtectionCyberStateClass deterministic_cyber_state(
    const std::vector<ChronologicalCyberComponent>& components,
    const std::vector<ProtectionCyberFunction>& functions,
    const std::vector<bool>& effective_up) {
  ProtectionCyberStateClass state;
  state.environment_name = "chronological";
  state.probability = 1.0;
  state.component_available.resize(components.size(), false);
  for (size_t i = 0; i < components.size(); ++i)
    state.component_available[i] = effective_up[i];
  state.function_available.assign(functions.size(), false);
  for (size_t f = 0; f < functions.size(); ++f) {
    for (const auto& path : functions[f].alternative_paths) {
      bool path_up = true;
      double latency = 0.0;
      double jitter = 0.0;
      double delivery = 1.0;
      for (size_t component : path.component_indices) {
        if (component >= components.size())
          throw std::invalid_argument("chronological cyber path index is out of range");
        path_up = path_up && state.component_available[component];
        latency += components[component].service.latency_ms;
        jitter += components[component].service.jitter_ms;
        delivery *= components[component].service.packet_delivery_probability;
      }
      path_up = path_up &&
          (functions[f].max_latency_ms <= 0.0 ||
           latency <= functions[f].max_latency_ms) &&
          (functions[f].max_jitter_ms <= 0.0 ||
           jitter <= functions[f].max_jitter_ms) &&
          delivery >= functions[f].min_packet_delivery_probability;
      if (path_up) {
        state.function_available[f] = true;
        break;
      }
    }
  }
  return state;
}

const ProtectionCyberGeneratedClass& sample_protection_class(
    const ProtectionCyberReliabilityScenario& source,
    const ProtectionCyberStateClass& cyber_state,
    std::mt19937_64& rng,
    ProtectionCyberClassGenerationResult& generated) {
  ProtectionCyberEventInput event = source.event;
  const auto binding = [&](int index, const char* name) {
    if (index < -1 ||
        (index >= 0 && static_cast<size_t>(index) >=
                           cyber_state.function_available.size()))
      throw std::invalid_argument(std::string(name) +
                                  " function binding is out of range");
    return index < 0 ||
        cyber_state.function_available[static_cast<size_t>(index)];
  };
  const bool detection = binding(
      source.event.function_bindings.detection_function, "detection");
  const bool primary_channel = binding(
      source.event.function_bindings.primary_trip_function, "primary trip");
  const bool backup_channel = binding(
      source.event.function_bindings.backup_trip_function, "backup trip");
  if (!detection || !primary_channel)
    event.primary.relay_success_probability = 0.0;
  if (!detection || !backup_channel)
    event.backup.relay_success_probability = 0.0;
  event.information_components.clear();
  event.information_functions.clear();
  event.information_environments.clear();
  event.function_bindings = {};
  generated = generate_protection_cyber_classes(event);
  if (generated.classes.empty())
    throw std::runtime_error("protection event tree generated no classes");

  const double draw = std::generate_canonical<double, 53>(rng);
  double cumulative = 0.0;
  for (const auto& item : generated.classes) {
    cumulative += item.protection.conditional_probability;
    if (draw <= cumulative + kProbabilityTolerance) return item;
  }
  throw std::runtime_error("protection class sampling lost probability mass");
}

double sampled_der_curtailment_mw(
    const ProtectionCyberGeneratedClass& generated,
    const ProtectionCyberEventInput& event) {
  if (generated.protection.der_results.size() != event.ders.size())
    throw std::runtime_error("DER FRT result dimension mismatch");
  double shed = 0.0;
  for (size_t i = 0; i < event.ders.size(); ++i) {
    const double capacity = event.ders[i].loss_of_generation_shed_mw;
    if (!std::isfinite(capacity) || capacity < 0.0)
      throw std::invalid_argument(
          "DER loss-of-generation shed contribution must be non-negative");
    shed += capacity * std::clamp(
        1.0 - generated.protection.der_results[i].terminal_restore_scale,
        0.0, 1.0);
  }
  return shed;
}

}  // namespace

IntelligentDetectionResult evaluate_intelligent_detection(
    const IntelligentDetectionModel& model) {
  const size_t n = model.fault_class_ids.size();
  if (n == 0 || model.confusion_matrix.size() != n + 1 ||
      model.latency_by_true_class.size() != n)
    throw std::invalid_argument("intelligent detection dimensions are invalid");
  if (!std::isfinite(model.timely_detection_limit_s) ||
      model.timely_detection_limit_s < 0.0 ||
      !std::isfinite(model.no_fault_decision_windows_per_year) ||
      model.no_fault_decision_windows_per_year < 0.0)
    throw std::invalid_argument("intelligent detection timing is invalid");
  for (const auto& row : model.confusion_matrix)
    require_probability_row(row, n + 1, "detection confusion row");

  IntelligentDetectionResult result;
  result.detection_probability.assign(n, 0.0);
  result.correct_classification_probability.assign(n, 0.0);
  result.expected_detection_latency_s.assign(n, 0.0);
  result.timely_detection_probability.assign(n, 0.0);
  result.false_alarm_frequency_per_year =
      model.no_fault_decision_windows_per_year *
      (1.0 - model.confusion_matrix[0][0]);

  for (size_t truth = 0; truth < n; ++truth) {
    const auto& latency_classes = model.latency_by_true_class[truth];
    if (latency_classes.empty())
      throw std::invalid_argument("each true fault class requires latency classes");
    double latency_sum = 0.0;
    for (const auto& latency : latency_classes) {
      require_probability(latency.probability, "detection latency probability");
      if (!std::isfinite(latency.latency_s) || latency.latency_s < 0.0)
        throw std::invalid_argument("detection latency must be non-negative");
      latency_sum += latency.probability;
    }
    if (std::abs(latency_sum - 1.0) > kProbabilityTolerance)
      throw std::invalid_argument("detection latency probabilities must sum to one");

    const auto& row = model.confusion_matrix[truth + 1];
    result.detection_probability[truth] = 1.0 - row[0];
    result.correct_classification_probability[truth] = row[truth + 1];
    for (size_t reported = 0; reported <= n; ++reported) {
      if (reported == 0) {
        IntelligentDetectionClass item;
        item.true_fault_class = model.fault_class_ids[truth];
        item.reported_fault_class = "no-alarm";
        item.probability = row[reported];
        item.missed = true;
        result.classes.push_back(std::move(item));
        continue;
      }
      for (const auto& latency : latency_classes) {
        IntelligentDetectionClass item;
        item.true_fault_class = model.fault_class_ids[truth];
        item.reported_fault_class = model.fault_class_ids[reported - 1];
        item.probability = row[reported] * latency.probability;
        item.latency_s = latency.latency_s;
        item.correct = reported == truth + 1;
        item.timely = model.timely_detection_limit_s <= 0.0 ||
            latency.latency_s <= model.timely_detection_limit_s;
        result.expected_detection_latency_s[truth] +=
            item.probability * item.latency_s;
        if (item.timely)
          result.timely_detection_probability[truth] += item.probability;
        result.classes.push_back(std::move(item));
      }
    }
    if (result.detection_probability[truth] > 0.0)
      result.expected_detection_latency_s[truth] /=
          result.detection_probability[truth];
  }
  result.probabilities_normalized = true;
  return result;
}

IntelligentIsolationResult select_minimum_risk_isolation(
    const IntelligentIsolationInput& input) {
  if (input.posterior_fault_probability.empty() || input.candidates.empty())
    throw std::invalid_argument("isolation requires a posterior and candidates");
  require_probability_row(input.posterior_fault_probability,
                          input.posterior_fault_probability.size(),
                          "fault posterior");
  const double costs[] = {input.missed_isolation_cost,
                          input.isolated_load_cost,
                          input.operation_time_cost};
  for (double cost : costs)
    if (!std::isfinite(cost) || cost < 0.0)
      throw std::invalid_argument("isolation costs must be non-negative");

  IntelligentIsolationResult result;
  double best = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < input.candidates.size(); ++i) {
    const auto& candidate = input.candidates[i];
    require_probability(candidate.action_success_probability,
                        "isolation action success probability");
    if (!std::isfinite(candidate.isolated_load_mw) ||
        candidate.isolated_load_mw < 0.0 ||
        !std::isfinite(candidate.operation_time_s) ||
        candidate.operation_time_s < 0.0)
      throw std::invalid_argument("isolation candidate values are invalid");
    IsolationCandidateEvaluation evaluation;
    evaluation.id = candidate.id;
    evaluation.executable = all_functions_available(
        candidate.required_information_functions,
        input.information_function_available);
    for (size_t fault : candidate.covered_fault_classes) {
      if (fault >= input.posterior_fault_probability.size())
        throw std::invalid_argument("isolation coverage index is out of range");
      evaluation.covered_probability += input.posterior_fault_probability[fault];
    }
    const double successful_coverage = evaluation.executable
        ? evaluation.covered_probability * candidate.action_success_probability
        : 0.0;
    evaluation.expected_cost = input.missed_isolation_cost *
            (1.0 - successful_coverage) +
        input.isolated_load_cost * candidate.isolated_load_mw +
        input.operation_time_cost * candidate.operation_time_s;
    if (evaluation.executable && evaluation.expected_cost < best) {
      best = evaluation.expected_cost;
      result.selected_candidate = static_cast<int>(i);
      result.selected_candidate_id = candidate.id;
      result.selected_expected_cost = best;
    }
    result.candidates.push_back(std::move(evaluation));
  }
  return result;
}

RiskConstrainedRestorationResult select_risk_constrained_restoration(
    const RiskConstrainedRestorationInput& input) {
  require_probability(input.maximum_unsafe_probability,
                      "maximum unsafe probability");
  if (!std::isfinite(input.unserved_energy_cost_per_mwh) ||
      input.unserved_energy_cost_per_mwh < 0.0 ||
      !std::isfinite(input.decision_horizon_hr) ||
      input.decision_horizon_hr < 0.0)
    throw std::invalid_argument("restoration objective values are invalid");
  RiskConstrainedRestorationResult result;
  result.objective = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < input.actions.size(); ++i) {
    const auto& action = input.actions[i];
    require_probability(action.unsafe_probability,
                        "restoration unsafe probability");
    require_probability(action.execution_success_probability,
                        "restoration execution success probability");
    const double values[] = {action.restored_load_mw, action.switching_cost,
                             action.remote_time_s, action.manual_time_s};
    bool invalid = false;
    for (double value : values) invalid = invalid || !std::isfinite(value) || value < 0.0;
    if (invalid) throw std::invalid_argument("restoration action values are invalid");
    if (action.unsafe_probability > input.maximum_unsafe_probability +
        kProbabilityTolerance) {
      result.rejected_actions.push_back(action.id + ": risk limit");
      continue;
    }
    const bool remote = all_functions_available(
        action.required_information_functions,
        input.information_function_available);
    if (!remote && action.manual_time_s <= 0.0) {
      result.rejected_actions.push_back(action.id + ": no executable path");
      continue;
    }
    const double duration_s = remote ? action.remote_time_s : action.manual_time_s;
    const double success = action.execution_success_probability;
    const double expected_restored = success * action.restored_load_mw;
    const double objective = action.switching_cost +
        input.unserved_energy_cost_per_mwh * input.decision_horizon_hr *
            (action.restored_load_mw - expected_restored) +
        duration_s / 3600.0 * input.unserved_energy_cost_per_mwh *
            action.restored_load_mw;
    if (objective < result.objective) {
      result.selected_action = static_cast<int>(i);
      result.selected_action_id = action.id;
      result.used_manual_fallback = !remote;
      result.selected_duration_s = duration_s;
      result.expected_restored_load_mw = expected_restored;
      result.objective = objective;
    }
  }
  return result;
}

FinitePOMDPResult solve_finite_pomdp(const FinitePOMDPModel& model) {
  const size_t n_state = model.state_ids.size();
  const size_t n_action = model.action_ids.size();
  const size_t n_observation = model.observation_ids.size();
  if (n_state == 0 || n_action == 0 || n_observation == 0 ||
      n_state > 12 || n_action > 8 || n_observation > 8 ||
      model.horizon_steps < 1 || model.horizon_steps > 8)
    throw std::invalid_argument("POMDP dimensions exceed exact-solver bounds");
  require_probability_row(model.initial_belief, n_state, "initial belief");
  if (model.transition_probability.size() != n_action ||
      model.observation_probability.size() != n_action ||
      model.stage_cost.size() != n_state ||
      model.terminal_cost.size() != n_state)
    throw std::invalid_argument("POMDP dimensions are inconsistent");
  for (size_t action = 0; action < n_action; ++action) {
    if (model.transition_probability[action].size() != n_state ||
        model.observation_probability[action].size() != n_state)
      throw std::invalid_argument("POMDP action matrix dimensions are invalid");
    for (size_t state = 0; state < n_state; ++state) {
      require_probability_row(model.transition_probability[action][state],
                              n_state, "POMDP transition row");
      require_probability_row(model.observation_probability[action][state],
                              n_observation, "POMDP observation row");
    }
  }
  for (const auto& row : model.stage_cost) {
    if (row.size() != n_action)
      throw std::invalid_argument("POMDP stage-cost dimensions are invalid");
    for (double value : row)
      if (!std::isfinite(value))
        throw std::invalid_argument("POMDP costs must be finite");
  }
  for (double value : model.terminal_cost)
    if (!std::isfinite(value))
      throw std::invalid_argument("POMDP terminal costs must be finite");

  POMDPEvaluator evaluator{model};
  const auto [cost, action] = evaluator.evaluate(model.initial_belief,
                                                  model.horizon_steps);
  FinitePOMDPResult result;
  result.first_action = action;
  result.first_action_id = action >= 0 ? model.action_ids[action] : "";
  result.expected_total_cost = cost;
  result.belief_nodes_evaluated = evaluator.nodes;
  result.exact_for_initial_belief = true;
  return result;
}

ChronologicalCyberPhysicalResult run_chronological_cyber_physical_reliability(
    const std::vector<ChronologicalCyberComponent>& cyber_components,
    const std::vector<ProtectionCyberFunction>& functions,
    const std::vector<ChronologicalCommonCause>& common_causes,
    const std::vector<ChronologicalPhysicalFault>& physical_faults,
    const ChronologicalCyberPhysicalOptions& options) {
  if (options.simulated_years <= 0 || options.maximum_events <= 0 ||
      !std::isfinite(options.hours_per_year) || options.hours_per_year <= 0.0 ||
      !std::isfinite(options.curtailment_threshold_mw) ||
      options.curtailment_threshold_mw < 0.0)
    throw std::invalid_argument("chronological reliability options are invalid");
  std::mt19937_64 rng(options.random_seed);
  const double horizon_hr = options.simulated_years * options.hours_per_year;
  std::priority_queue<QueueEvent, std::vector<QueueEvent>, std::greater<>> queue;
  uint64_t event_order = 0;
  const auto push_event = [&](double time_hr, QueueEventKind kind, size_t index) {
    queue.push({time_hr, kind, index, event_order++});
  };

  std::vector<bool> component_up(cyber_components.size(), true);
  std::vector<double> battery_energy_wh(cyber_components.size(), 0.0);
  bool has_power_dependency = false;
  for (size_t i = 0; i < cyber_components.size(); ++i) {
    const auto& component = cyber_components[i];
    const auto& service = component.service;
    if (!std::isfinite(component.failure_rate_per_year) ||
        component.failure_rate_per_year < 0.0 ||
        !std::isfinite(component.mean_repair_time_hr) ||
        component.mean_repair_time_hr < 0.0 ||
        !std::isfinite(component.backup_recharge_power_w) ||
        component.backup_recharge_power_w < 0.0 ||
        !std::isfinite(service.backup_energy_wh) ||
        service.backup_energy_wh < 0.0 ||
        !std::isfinite(service.power_draw_w) || service.power_draw_w < 0.0 ||
        !std::isfinite(service.packet_delivery_probability) ||
        service.packet_delivery_probability < 0.0 ||
        service.packet_delivery_probability > 1.0)
      throw std::invalid_argument(
          "chronological cyber component parameters are invalid");
    if (!service.supplied_by_bus_id.empty() && service.power_draw_w <= 0.0)
      throw std::invalid_argument(
          "bus-supplied cyber component requires positive power draw");
    has_power_dependency =
        has_power_dependency || !service.supplied_by_bus_id.empty();
    component_up[i] = component.initially_available;
    battery_energy_wh[i] = service.backup_energy_wh;
    if (component.initially_available && component.failure_rate_per_year > 0.0)
      push_event(exponential_wait_hr(
                     rng, component.failure_rate_per_year / options.hours_per_year),
                 QueueEventKind::CyberFailure, i);
    else if (!component.initially_available && component.mean_repair_time_hr > 0.0)
      push_event(exponential_wait_hr(rng, 1.0 / component.mean_repair_time_hr),
                 QueueEventKind::CyberRepair, i);
  }

  std::vector<bool> common_cause_active(common_causes.size(), false);
  for (size_t i = 0; i < common_causes.size(); ++i) {
    if (common_causes[i].group.empty() ||
        !std::isfinite(common_causes[i].occurrence_rate_per_year) ||
        common_causes[i].occurrence_rate_per_year < 0.0 ||
        !std::isfinite(common_causes[i].mean_duration_hr) ||
        common_causes[i].mean_duration_hr <= 0.0)
      throw std::invalid_argument("common-cause process parameters are invalid");
    if (common_causes[i].occurrence_rate_per_year > 0.0)
      push_event(exponential_wait_hr(
                     rng, common_causes[i].occurrence_rate_per_year /
                              options.hours_per_year),
                 QueueEventKind::CommonCauseStart, i);
  }
  for (size_t i = 0; i < physical_faults.size(); ++i) {
    const auto& fault = physical_faults[i];
    const double rate = fault.scenario.initiating_frequency_per_year;
    if (!std::isfinite(rate) || rate < 0.0 ||
        !std::isfinite(fault.information_power_outage_duration_hr) ||
        fault.information_power_outage_duration_hr < 0.0 ||
        (!fault.deenergized_bus_ids.empty() &&
         fault.information_power_outage_duration_hr <= 0.0))
      throw std::invalid_argument("chronological physical fault parameters are invalid");
    if (!fault.scheduled_occurrence_hours.empty()) {
      for (double time_hr : fault.scheduled_occurrence_hours) {
        if (!std::isfinite(time_hr) || time_hr < 0.0 || time_hr > horizon_hr)
          throw std::invalid_argument(
              "scheduled physical fault time is outside the simulation horizon");
        push_event(time_hr, QueueEventKind::PhysicalFault, i);
      }
    } else if (rate > 0.0) {
      push_event(exponential_wait_hr(rng, rate / options.hours_per_year),
                 QueueEventKind::PhysicalFault, i);
    }
  }

  ChronologicalCyberPhysicalResult result;
  result.minimum_battery_energy_wh = battery_energy_wh.empty()
      ? 0.0
      : *std::min_element(battery_energy_wh.begin(), battery_energy_wh.end());
  std::unordered_map<std::string, int> bus_outage_count;
  std::vector<ShedInterval> interruption_intervals;
  double last_event_time_hr = 0.0;
  double sampled_eens_mwh = 0.0;

  const auto advance_batteries = [&](double time_hr) {
    const double duration_hr = time_hr - last_event_time_hr;
    if (duration_hr < -kProbabilityTolerance)
      throw std::logic_error("chronological event queue moved backward in time");
    for (size_t i = 0; i < cyber_components.size(); ++i) {
      const auto& component = cyber_components[i];
      const auto& service = component.service;
      if (service.supplied_by_bus_id.empty()) continue;
      const auto it = bus_outage_count.find(service.supplied_by_bus_id);
      const bool energized = it == bus_outage_count.end() || it->second == 0;
      if (energized) {
        battery_energy_wh[i] = std::min(
            service.backup_energy_wh,
            battery_energy_wh[i] +
                component.backup_recharge_power_w * duration_hr);
      } else {
        battery_energy_wh[i] = std::max(
            0.0, battery_energy_wh[i] - service.power_draw_w * duration_hr);
      }
      result.minimum_battery_energy_wh =
          std::min(result.minimum_battery_energy_wh, battery_energy_wh[i]);
    }
    last_event_time_hr = time_hr;
  };

  const auto add_shed_stage = [&](double begin_hr, double duration_hr,
                                  double shed_mw) {
    if (!std::isfinite(duration_hr) || duration_hr < 0.0 ||
        !std::isfinite(shed_mw) || shed_mw < 0.0)
      throw std::invalid_argument("sampled reliability consequence is invalid");
    const double clipped_begin = std::clamp(begin_hr, 0.0, horizon_hr);
    const double clipped_end = std::clamp(begin_hr + duration_hr, 0.0, horizon_hr);
    if (clipped_end <= clipped_begin) return;
    sampled_eens_mwh += (clipped_end - clipped_begin) * shed_mw;
    if (shed_mw > options.curtailment_threshold_mw)
      interruption_intervals.push_back({clipped_begin, clipped_end});
  };

  int processed = 0;
  while (!queue.empty() && queue.top().time_hr <= horizon_hr) {
    if (++processed > options.maximum_events)
      throw std::runtime_error("chronological reliability event limit exceeded");
    const QueueEvent event = queue.top();
    queue.pop();
    advance_batteries(event.time_hr);

    if (event.kind == QueueEventKind::CyberFailure) {
      component_up[event.index] = false;
      ++result.cyber_transitions_sampled;
      const double repair = cyber_components[event.index].mean_repair_time_hr;
      if (repair > 0.0)
        push_event(event.time_hr + exponential_wait_hr(rng, 1.0 / repair),
                   QueueEventKind::CyberRepair, event.index);
      continue;
    }
    if (event.kind == QueueEventKind::CyberRepair) {
      component_up[event.index] = true;
      ++result.cyber_transitions_sampled;
      const double rate = cyber_components[event.index].failure_rate_per_year /
          options.hours_per_year;
      if (rate > 0.0)
        push_event(event.time_hr + exponential_wait_hr(rng, rate),
                   QueueEventKind::CyberFailure, event.index);
      continue;
    }
    if (event.kind == QueueEventKind::CommonCauseStart) {
      common_cause_active[event.index] = true;
      ++result.common_cause_events_sampled;
      push_event(event.time_hr + exponential_wait_hr(
                     rng, 1.0 / common_causes[event.index].mean_duration_hr),
                 QueueEventKind::CommonCauseEnd, event.index);
      continue;
    }
    if (event.kind == QueueEventKind::CommonCauseEnd) {
      common_cause_active[event.index] = false;
      const double rate = common_causes[event.index].occurrence_rate_per_year /
          options.hours_per_year;
      if (rate > 0.0)
        push_event(event.time_hr + exponential_wait_hr(rng, rate),
                   QueueEventKind::CommonCauseStart, event.index);
      continue;
    }
    if (event.kind == QueueEventKind::PowerRestore) {
      for (const auto& bus : physical_faults[event.index].deenergized_bus_ids) {
        auto it = bus_outage_count.find(bus);
        if (it == bus_outage_count.end() || it->second <= 0)
          throw std::logic_error("power-restoration event has no active outage");
        --it->second;
      }
      continue;
    }

    ++result.physical_faults_sampled;
    const auto& fault = physical_faults[event.index];
    for (const auto& bus : fault.deenergized_bus_ids)
      ++bus_outage_count[bus];
    if (!fault.deenergized_bus_ids.empty())
      push_event(event.time_hr + fault.information_power_outage_duration_hr,
                 QueueEventKind::PowerRestore, event.index);

    std::vector<bool> effective_up = component_up;
    for (size_t component = 0; component < cyber_components.size(); ++component) {
      const auto& service = cyber_components[component].service;
      for (size_t group = 0; group < common_causes.size(); ++group) {
        if (common_cause_active[group] &&
            service.common_cause_group == common_causes[group].group)
          effective_up[component] = false;
      }
      if (!service.supplied_by_bus_id.empty()) {
        const bool bus_energized =
            bus_outage_count[service.supplied_by_bus_id] == 0;
        const bool battery_alive = battery_energy_wh[component] >
            kProbabilityTolerance;
        effective_up[component] =
            effective_up[component] && (bus_energized || battery_alive);
      }
      const bool delivered = std::generate_canonical<double, 53>(rng) <
          service.packet_delivery_probability;
      effective_up[component] = effective_up[component] && delivered;
    }
    const auto cyber_state = deterministic_cyber_state(
        cyber_components, functions, effective_up);
    ProtectionCyberClassGenerationResult generated;
    ProtectionCyberGeneratedClass selected = sample_protection_class(
        fault.scenario, cyber_state, rng, generated);
    const auto binding = [&](int index, const char* name) {
      if (index < -1 ||
          (index >= 0 && static_cast<size_t>(index) >=
                             cyber_state.function_available.size()))
        throw std::invalid_argument(std::string(name) +
                                    " function binding is out of range");
      return index < 0 ||
          cyber_state.function_available[static_cast<size_t>(index)];
    };
    selected.detection_success = binding(
        fault.scenario.event.function_bindings.detection_function,
        "detection");
    selected.primary_trip_channel_available = binding(
        fault.scenario.event.function_bindings.primary_trip_function,
        "primary trip");
    selected.backup_trip_channel_available = binding(
        fault.scenario.event.function_bindings.backup_trip_function,
        "backup trip");
    selected.isolation_success = selected.detection_success && binding(
        fault.scenario.event.function_bindings.isolation_function,
        "isolation");
    selected.restoration_success = selected.isolation_success && binding(
        fault.scenario.event.function_bindings.restoration_function,
        "restoration");

    const auto& consequence = fault.scenario.consequence;
    const double clearing_hr = selected.protection.terminal_time_s / 3600.0;
    const double switching_hr = selected.restoration_success
        ? consequence.automatic_restoration_hr
        : consequence.manual_restoration_hr;
    const double residual_hr =
        std::max(0.0, consequence.repair_hr - clearing_hr - switching_hr);
    double clearing_shed = consequence.uncleared_shed_mw;
    if (selected.protection.protection_outcome ==
        ProtectionClearingOutcome::PrimaryCleared)
      clearing_shed = consequence.primary_clearing_shed_mw;
    else if (selected.protection.protection_outcome ==
             ProtectionClearingOutcome::BackupCleared)
      clearing_shed = consequence.backup_clearing_shed_mw;
    const double der_shed = sampled_der_curtailment_mw(
        selected, fault.scenario.event);
    const double switching_shed =
        (selected.isolation_success ? consequence.isolated_shed_mw
                                    : consequence.isolation_failed_shed_mw) +
        der_shed;
    const double residual_shed =
        (selected.restoration_success ? consequence.restored_shed_mw
                                      : consequence.restoration_failed_shed_mw) +
        der_shed;
    add_shed_stage(event.time_hr, clearing_hr, clearing_shed);
    add_shed_stage(event.time_hr + clearing_hr, switching_hr, switching_shed);
    add_shed_stage(event.time_hr + clearing_hr + switching_hr, residual_hr,
                   residual_shed);

    if (fault.scheduled_occurrence_hours.empty()) {
      const double rate = fault.scenario.initiating_frequency_per_year /
          options.hours_per_year;
      if (rate > 0.0)
        push_event(event.time_hr + exponential_wait_hr(rng, rate),
                   QueueEventKind::PhysicalFault, event.index);
    }
  }

  std::sort(interruption_intervals.begin(), interruption_intervals.end(),
            [](const auto& lhs, const auto& rhs) {
              return lhs.begin_hr != rhs.begin_hr
                  ? lhs.begin_hr < rhs.begin_hr
                  : lhs.end_hr < rhs.end_hr;
            });
  double union_duration_hr = 0.0;
  if (!interruption_intervals.empty()) {
    double begin = interruption_intervals.front().begin_hr;
    double end = interruption_intervals.front().end_hr;
    for (size_t i = 1; i < interruption_intervals.size(); ++i) {
      if (interruption_intervals[i].begin_hr <= end + kProbabilityTolerance) {
        end = std::max(end, interruption_intervals[i].end_hr);
      } else {
        union_duration_hr += end - begin;
        ++result.merged_interruption_intervals;
        begin = interruption_intervals[i].begin_hr;
        end = interruption_intervals[i].end_hr;
      }
    }
    union_duration_hr += end - begin;
    ++result.merged_interruption_intervals;
  }
  result.eens_mwh_yr = sampled_eens_mwh / options.simulated_years;
  result.lole_hr_yr = union_duration_hr / options.simulated_years;
  result.lolf_occ_yr = static_cast<double>(result.merged_interruption_intervals) /
      options.simulated_years;
  result.subhour_event_timing = true;
  result.cyber_power_coupling_modelled = has_power_dependency;
  result.battery_energy_trajectory_modelled = has_power_dependency;
  result.common_cause_modelled = !common_causes.empty();
  result.probability_weighted_protection_modelled = !physical_faults.empty();
  result.packet_delivery_sampled =
      !cyber_components.empty() && result.physical_faults_sampled > 0;
  result.outage_interval_union_modelled = true;
  return result;
}

}  // namespace hacdcpf::analysis
