#include "hacdcpf/analysis/counterfactual_planning.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <numeric>
#include <sstream>
#include <unordered_map>

#include "hacdcpf/api/hacdcpf.hpp"
#include "hacdcpf/carbon_analysis/carbon_analysis.hpp"
#include "hacdcpf/reliability/reliability_assessment.hpp"
#include "hacdcpf/resilience/resilience_assessment.hpp"

namespace hacdcpf::analysis {
namespace {

constexpr int kMetricCount = 4;

template <typename Component>
int next_index(const std::vector<Component>& components) {
  int value = 0;
  for (const auto& component : components) value = std::max(value, component.index);
  return value + 1;
}

double total_ac_load_mw(const HybridPowerSystem& system) {
  double total = 0.0;
  for (const auto& bus : system.ac.buses)
    if (bus.in_service) total += std::max(0.0, bus.pd_mw);
  for (const auto& load : system.ac.loads)
    if (load.in_service) total += std::max(0.0, load.p_mw * load.scaling);
  return total;
}

std::vector<std::pair<int, double>> load_by_bus(const HybridPowerSystem& system) {
  std::unordered_map<int, double> totals;
  for (const auto& bus : system.ac.buses)
    if (bus.in_service) totals[bus.index] += std::max(0.0, bus.pd_mw);
  for (const auto& load : system.ac.loads)
    if (load.in_service) totals[load.bus] += std::max(0.0, load.p_mw * load.scaling);
  std::vector<std::pair<int, double>> result(totals.begin(), totals.end());
  std::sort(result.begin(), result.end(), [](const auto& a, const auto& b) {
    if (std::abs(a.second - b.second) > 1e-12) return a.second > b.second;
    return a.first < b.first;
  });
  return result;
}

int source_bus(const HybridPowerSystem& system) {
  for (const auto& grid : system.ac.external_grids)
    if (grid.in_service) return grid.bus;
  for (const auto& bus : system.ac.buses)
    if (bus.in_service && bus.bus_type == BusType::SLACK) return bus.index;
  return system.ac.buses.empty() ? -1 : system.ac.buses.front().index;
}

bool directly_connected(const HybridPowerSystem& system, int a, int b) {
  return std::any_of(system.ac.branches.begin(), system.ac.branches.end(),
                     [&](const auto& branch) {
    return (branch.from_bus == a && branch.to_bus == b) ||
           (branch.from_bus == b && branch.to_bus == a);
  });
}

const ACBranch* line_target(const HybridPowerSystem& system) {
  const ACBranch* best = nullptr;
  double best_score = -1.0;
  for (const auto& branch : system.ac.branches) {
    if (!branch.in_service) continue;
    const double reliability = std::max(0.0, branch.failure_rate) *
                               std::max(0.0, branch.mttr_hr);
    const double electrical = std::abs(branch.r_pu) + 0.25 * std::abs(branch.x_pu);
    const double capacity = branch.rate_a_mva > 0.0 ? 1.0 / branch.rate_a_mva : 0.0;
    const double score = reliability + electrical + capacity;
    if (score > best_score) {
      best_score = score;
      best = &branch;
    }
  }
  return best;
}

std::vector<int> select_resilience_faults(const HybridPowerSystem& system,
                                          int count) {
  std::vector<std::pair<double, int>> ranked;
  for (const auto& branch : system.ac.branches) {
    if (!branch.in_service) continue;
    const double score = std::max(branch.failure_rate, 1e-9) *
                         std::max(branch.mttr_hr, 1.0) +
                         std::abs(branch.r_pu);
    ranked.emplace_back(score, branch.index);
  }
  std::stable_sort(ranked.begin(), ranked.end(), [](const auto& a, const auto& b) {
    if (std::abs(a.first - b.first) > 1e-12) return a.first > b.first;
    return a.second < b.second;
  });
  std::vector<int> result;
  for (const auto& [score, index] : ranked) {
    (void)score;
    if (static_cast<int>(result.size()) >= std::max(0, count)) break;
    result.push_back(index);
  }
  return result;
}

double automation_availability(const HybridPowerSystem& system) {
  if (system.ac.switches.empty()) return 0.0;
  int available = 0;
  int automated = 0;
  for (const auto& sw : system.ac.switches) {
    if (!sw.in_service) continue;
    ++available;
    if (sw.is_automated || sw.is_remote || sw.t_operation_s > 0.0) ++automated;
  }
  return available > 0 ? static_cast<double>(automated) / available : 0.0;
}

CounterfactualMetricSet evaluate_system(
    const HybridPowerSystem& system,
    const CounterfactualPlanningOptions& options,
    const std::vector<int>& resilience_faults) {
  CounterfactualMetricSet metrics;
  std::optional<PowerFlowResult> pf;
  if (options.include_economic || options.include_carbon) {
    try {
      PowerFlowOptions pf_options;
      pf_options.max_iter = 100;
      pf_options.tol = 1e-7;
      auto result = hacdcpf::solve_power_flow(system, pf_options);
      if (result.converged) pf = std::move(result);
      else metrics.diagnostics[1] = "power flow did not converge";
    } catch (const std::exception& error) {
      metrics.diagnostics[1] = error.what();
    }
  }

  if (options.include_economic) {
    try {
      opf::ACOPFOptions opf_options;
      opf_options.max_inner_iterations = 120;
      opf_options.allow_fallback = true;
      const auto opf_result = hacdcpf::solve_ac_opf(system, opf_options);
      if (opf_result.converged && std::isfinite(opf_result.objective) &&
          std::abs(opf_result.objective) > 1e-9) {
        metrics.values[0] = opf_result.objective * options.annual_hours;
        metrics.diagnostics[0] = "annualized AC OPF objective";
      } else if (pf) {
        double loss_mw = 0.0;
        for (const auto& flow : pf->branch_flows)
          loss_mw += std::max(0.0, flow.pf_mw + flow.pt_mw);
        for (const auto& transfer : pf->vsc_transfers)
          loss_mw += std::max(0.0, transfer.loss_mw);
        for (const auto& transfer : pf->dcdc_transfers)
          loss_mw += std::max(0.0, transfer.loss_mw);
        metrics.values[0] = loss_mw * options.energy_price_per_mwh *
                            options.annual_hours;
        metrics.diagnostics[0] = "annualized monetized physical-loss fallback";
      } else {
        metrics.diagnostics[0] = opf_result.status;
      }
    } catch (const std::exception& error) {
      metrics.diagnostics[0] = error.what();
    }
  }

  if (options.include_carbon) {
    if (pf) {
      try {
        const auto carbon = compute_carbon_analysis(system, *pf);
        const auto& summary = carbon.matrix_solved ? carbon.matrix_summary
                                                   : carbon.tracing_summary;
        const double emissions = summary.total_generation_emissions_tco2;
        if (std::isfinite(emissions)) {
          metrics.values[1] = emissions * options.annual_hours;
          metrics.diagnostics[1] = carbon.matrix_solved
              ? "annualized matrix carbon flow" : "annualized proportional carbon tracing";
        }
      } catch (const std::exception& error) {
        metrics.diagnostics[1] = error.what();
      }
    } else if (metrics.diagnostics[1].empty()) {
      metrics.diagnostics[1] = "carbon requires a converged power flow";
    }
  }

  if (options.include_reliability) {
    try {
      FMEAOptions fmea;
      fmea.enable_parallel = false;
      fmea.enable_switch_reconfiguration = true;
      fmea.enable_repair_reconfiguration = true;
      fmea.max_repair_switch_actions = 2;
      fmea.max_repair_opf_calls = std::max(1, options.reliability_physical_budget);
      const double availability = automation_availability(system);
      fmea.cyber_physical.enabled = !system.ac.switches.empty();
      fmea.cyber_physical.automation_availability = availability;
      fmea.cyber_physical.automatic_switching_time_hr = 0.05;
      fmea.cyber_physical.manual_switching_time_hr = 1.0;
      fmea.cyber_physical.freeze_der_on_automation_loss = true;
      const auto reliability = run_distribution_fmea(system, fmea);
      if (std::isfinite(reliability.eens_mwh_yr)) {
        metrics.values[2] = reliability.eens_mwh_yr;
        metrics.diagnostics[2] = "deterministic N-1 FMEA";
      }
    } catch (const std::exception& error) {
      metrics.diagnostics[2] = error.what();
    }
  }

  if (options.include_resilience) {
    try {
      DistributionResilienceOptions resilience;
      resilience.model = DistributionResilienceModel::HeuristicSequential;
      resilience.horizon_hours = std::max(1, options.resilience_horizon_hours);
      resilience.time_step_hr = 1.0;
      resilience.allow_reconfiguration = true;
      resilience.allow_mess_dispatch = false;
      resilience.enable_storage_dispatch = true;
      resilience.default_fault_count = 0;
      for (int index : resilience_faults) {
        resilience.faults.emplace_back(index, 0.0,
            std::max(1.0, options.resilience_repair_time_hr),
            "Counterfactual common fault " + std::to_string(index));
      }
      const auto result = hacdcpf::analysis::run_distribution_resilience_assessment(
          system, resilience);
      if (result.feasible && std::isfinite(result.weighted_unserved_mwh)) {
        metrics.values[3] = result.weighted_unserved_mwh;
        metrics.diagnostics[3] = "common-fault heuristic sequential resilience";
      } else {
        metrics.diagnostics[3] = result.status;
      }
    } catch (const std::exception& error) {
      metrics.diagnostics[3] = error.what();
    }
  }
  return metrics;
}

bool metrics_complete(const CounterfactualMetricSet& metrics,
                      const CounterfactualPlanningOptions& options) {
  const std::array<bool, kMetricCount> enabled = {
      options.include_economic, options.include_carbon,
      options.include_reliability, options.include_resilience};
  for (int d = 0; d < kMetricCount; ++d) {
    if (enabled[static_cast<size_t>(d)] &&
        !metrics.values[static_cast<size_t>(d)]) return false;
  }
  return true;
}

CounterfactualBenefitVector benefit_between(const CounterfactualMetricSet& baseline,
                                            const CounterfactualMetricSet& outcome) {
  CounterfactualBenefitVector result;
  constexpr double tolerance = 1e-9;
  for (int d = 0; d < kMetricCount; ++d) {
    const auto& base = baseline.values[static_cast<size_t>(d)];
    const auto& value = outcome.values[static_cast<size_t>(d)];
    if (!base || !value || !std::isfinite(*base) || !std::isfinite(*value)) continue;
    const double benefit = *base - *value;
    const double scale = std::max({std::abs(*base), std::abs(*value), tolerance});
    result.absolute[static_cast<size_t>(d)] = benefit;
    result.relative[static_cast<size_t>(d)] = benefit / scale;
    if (benefit > tolerance * scale) ++result.improved_dimensions;
    else if (benefit < -tolerance * scale) ++result.worsened_dimensions;
  }
  if (result.improved_dimensions > 0 && result.worsened_dimensions > 0)
    result.relation = "conflict";
  else if (result.improved_dimensions >= 2)
    result.relation = "compatible";
  else if (result.improved_dimensions == 1)
    result.relation = "single_benefit";
  else if (result.worsened_dimensions > 0)
    result.relation = "adverse";
  else
    result.relation = "neutral";
  return result;
}

CounterfactualBenefitVector synergy_between(
    const CounterfactualBenefitVector& combined,
    const CounterfactualBenefitVector& a,
    const CounterfactualBenefitVector& b) {
  CounterfactualBenefitVector result;
  constexpr double tolerance = 1e-9;
  for (int d = 0; d < kMetricCount; ++d) {
    const auto& combined_value = combined.absolute[static_cast<size_t>(d)];
    const auto& a_value = a.absolute[static_cast<size_t>(d)];
    const auto& b_value = b.absolute[static_cast<size_t>(d)];
    if (!combined_value || !a_value || !b_value) continue;
    const double value = *combined_value - *a_value - *b_value;
    const double scale = std::max({std::abs(*combined_value), std::abs(*a_value) +
                                  std::abs(*b_value), tolerance});
    result.absolute[static_cast<size_t>(d)] = value;
    result.relative[static_cast<size_t>(d)] = value / scale;
    if (value > tolerance * scale) ++result.improved_dimensions;
    else if (value < -tolerance * scale) ++result.worsened_dimensions;
  }
  if (result.improved_dimensions > 0 && result.worsened_dimensions > 0)
    result.relation = "mixed_interaction";
  else if (result.improved_dimensions > 0)
    result.relation = "synergy";
  else if (result.worsened_dimensions > 0)
    result.relation = "redundancy_or_conflict";
  else
    result.relation = "additive";
  return result;
}

}  // namespace

const char* counterfactual_measure_type_name(CounterfactualMeasureType type) noexcept {
  switch (type) {
    case CounterfactualMeasureType::LineCapacity: return "line_capacity";
    case CounterfactualMeasureType::Storage: return "storage";
    case CounterfactualMeasureType::TieSwitch: return "tie_switch";
    case CounterfactualMeasureType::Automation: return "automation";
    case CounterfactualMeasureType::DistributedEnergyResource: return "der";
  }
  return "unknown";
}

const char* counterfactual_metric_name(int dimension) noexcept {
  static constexpr const char* names[] = {
      "economic", "carbon", "reliability", "resilience"};
  return dimension >= 0 && dimension < kMetricCount ? names[dimension] : "unknown";
}

const char* counterfactual_metric_unit(int dimension) noexcept {
  static constexpr const char* units[] = {
      "USD/yr", "tCO2/yr", "MWh/yr", "weighted MWh/event"};
  return dimension >= 0 && dimension < kMetricCount ? units[dimension] : "";
}

std::vector<CounterfactualMeasure> generate_counterfactual_measures(
    const HybridPowerSystem& system,
    const CounterfactualPlanningOptions& options) {
  std::vector<CounterfactualMeasure> result;
  const auto loads = load_by_bus(system);
  const int load_bus = loads.empty() ? source_bus(system) : loads.front().first;
  const double peak_load = std::max(total_ac_load_mw(system), 0.1);
  const auto* branch = line_target(system);

  if (options.enable_line_capacity && branch) {
    CounterfactualMeasure measure;
    measure.id = "line_capacity:AC:" + std::to_string(branch->index);
    measure.name = "Line capacity " + (branch->name.empty()
        ? std::to_string(branch->index) : branch->name);
    measure.type = CounterfactualMeasureType::LineCapacity;
    measure.target_index = branch->index;
    measure.from_bus = branch->from_bus;
    measure.to_bus = branch->to_bus;
    measure.expansion_factor = std::max(1.01, options.line_expansion_factor);
    measure.capex = std::max(1.0, branch->length_km) * 300000.0 *
                    (measure.expansion_factor - 1.0);
    result.push_back(std::move(measure));
  }

  if (options.enable_storage && load_bus >= 0) {
    CounterfactualMeasure measure;
    measure.id = "storage:AC:" + std::to_string(load_bus);
    measure.name = "Storage at AC Bus " + std::to_string(load_bus);
    measure.type = CounterfactualMeasureType::Storage;
    measure.bus = load_bus;
    measure.capacity_mw = options.storage_power_mw > 0.0
        ? std::min(options.storage_power_mw, peak_load * 0.75)
        : peak_load * 0.25;
    measure.energy_mwh = measure.capacity_mw *
                         std::max(1.0, options.storage_duration_hr);
    measure.dispatch_fraction = std::clamp(options.storage_dispatch_fraction, 0.0, 1.0);
    measure.capex = measure.capacity_mw * 250000.0 +
                    measure.energy_mwh * 400000.0;
    result.push_back(std::move(measure));
  }

  if (options.enable_tie_switch && load_bus >= 0) {
    const int source = source_bus(system);
    int tie_bus = load_bus;
    for (const auto& [bus, demand] : loads) {
      (void)demand;
      if (bus != source && !directly_connected(system, source, bus)) {
        tie_bus = bus;
        break;
      }
    }
    if (source >= 0 && tie_bus >= 0 && source != tie_bus &&
        !directly_connected(system, source, tie_bus)) {
      CounterfactualMeasure measure;
      measure.id = "tie_switch:AC:" + std::to_string(source) + ":" +
                   std::to_string(tie_bus);
      measure.name = "Normally-open tie " + std::to_string(source) + "-" +
                     std::to_string(tie_bus);
      measure.type = CounterfactualMeasureType::TieSwitch;
      measure.from_bus = source;
      measure.to_bus = tie_bus;
      measure.capacity_mw = options.tie_capacity_mw > 0.0
          ? options.tie_capacity_mw : peak_load * 0.75;
      measure.capex = 180000.0 + measure.capacity_mw * 30000.0;
      result.push_back(std::move(measure));
    }
  }

  if (options.enable_automation) {
    CounterfactualMeasure measure;
    measure.type = CounterfactualMeasureType::Automation;
    const auto switch_it = std::find_if(system.ac.switches.begin(),
                                        system.ac.switches.end(),
                                        [](const auto& sw) {
      return sw.in_service && (!sw.is_automated || !sw.is_remote);
    });
    if (switch_it != system.ac.switches.end()) {
      measure.target_index = switch_it->index;
      measure.from_bus = switch_it->bus_from;
      measure.to_bus = switch_it->bus_to;
    } else if (branch) {
      measure.target_index = branch->index;
      measure.from_bus = branch->from_bus;
      measure.to_bus = branch->to_bus;
    }
    if (measure.target_index >= 0) {
      measure.id = "automation:AC:" + std::to_string(measure.target_index);
      measure.name = "Feeder automation " + std::to_string(measure.target_index);
      measure.capex = 40000.0;
      result.push_back(std::move(measure));
    }
  }

  if (options.enable_der && load_bus >= 0) {
    CounterfactualMeasure measure;
    measure.id = "der:AC:" + std::to_string(load_bus);
    measure.name = "Low-carbon DER at AC Bus " + std::to_string(load_bus);
    measure.type = CounterfactualMeasureType::DistributedEnergyResource;
    measure.bus = load_bus;
    measure.capacity_mw = options.der_capacity_mw > 0.0
        ? std::min(options.der_capacity_mw, peak_load * 0.75)
        : peak_load * 0.25;
    measure.capacity_factor = std::clamp(options.der_capacity_factor, 0.0, 1.0);
    measure.capex = measure.capacity_mw * 900000.0;
    result.push_back(std::move(measure));
  }

  if (options.max_measures > 0 &&
      result.size() > static_cast<size_t>(options.max_measures))
    result.resize(static_cast<size_t>(options.max_measures));
  return result;
}

CounterfactualApplyResult apply_counterfactual_measure(
    HybridPowerSystem& system, const CounterfactualMeasure& measure) {
  CounterfactualApplyResult result;
  const double factor = std::max(1.01, measure.expansion_factor);
  switch (measure.type) {
    case CounterfactualMeasureType::LineCapacity: {
      auto branch = std::find_if(system.ac.branches.begin(), system.ac.branches.end(),
                                 [&](const auto& row) {
        return row.index == measure.target_index;
      });
      if (branch == system.ac.branches.end()) {
        result.status = "target AC branch not found";
        return result;
      }
      const double fallback_rate = std::max(system.base_mva, system.ac.base_mva);
      branch->rate_a_mva = (branch->rate_a_mva > 0.0 ? branch->rate_a_mva
                                                      : fallback_rate) * factor;
      if (branch->rate_b_mva > 0.0) branch->rate_b_mva *= factor;
      if (branch->rate_c_mva > 0.0) branch->rate_c_mva *= factor;
      branch->r_pu /= factor;
      branch->x_pu /= factor;
      if (branch->r_ohm_per_km > 0.0) branch->r_ohm_per_km /= factor;
      branch->n_parallel = std::max(branch->n_parallel,
                                    static_cast<int>(std::ceil(factor)));
      result.applied = true;
      result.status = "AC line rating increased and parallel impedance reduced";
      return result;
    }
    case CounterfactualMeasureType::Storage: {
      const bool bus_exists = std::any_of(system.ac.buses.begin(), system.ac.buses.end(),
                                          [&](const auto& bus) {
        return bus.index == measure.bus;
      });
      if (!bus_exists || measure.capacity_mw <= 0.0 || measure.energy_mwh <= 0.0) {
        result.status = "invalid storage bus or capacity";
        return result;
      }
      Storage storage;
      storage.index = next_index(system.ac.storage);
      storage.bus = measure.bus;
      storage.name = measure.name;
      storage.p_rated_mw = measure.capacity_mw;
      storage.pmax_mw = measure.capacity_mw;
      storage.pmin_mw = -measure.capacity_mw;
      storage.p_mw = measure.capacity_mw *
                     std::clamp(measure.dispatch_fraction, 0.0, 1.0);
      storage.e_rated_mwh = measure.energy_mwh;
      storage.e_mwh = measure.energy_mwh * 0.8;
      storage.soc_init = 0.8;
      storage.soc_min = 0.1;
      storage.soc_max = 0.9;
      storage.controllable = true;
      storage.grid_forming = true;
      storage.forced_outage_rate = 0.02;
      storage.mttr_hr = 8.0;
      system.ac.storage.push_back(std::move(storage));
      result.applied = true;
      result.status = "grid-forming AC storage added";
      return result;
    }
    case CounterfactualMeasureType::TieSwitch: {
      if (measure.from_bus < 0 || measure.to_bus < 0 ||
          directly_connected(system, measure.from_bus, measure.to_bus)) {
        result.status = "invalid or duplicate tie endpoints";
        return result;
      }
      ACBranch tie;
      tie.index = next_index(system.ac.branches);
      tie.from_bus = measure.from_bus;
      tie.to_bus = measure.to_bus;
      tie.name = measure.name;
      tie.r_pu = 0.01;
      tie.x_pu = 0.05;
      tie.rate_a_mva = std::max(0.1, measure.capacity_mw);
      tie.in_service = false;
      tie.failure_rate = 0.0;
      tie.mttr_hr = 4.0;
      system.ac.branches.push_back(tie);

      Switch sw;
      sw.index = next_index(system.ac.switches);
      sw.name = measure.name + " switch";
      sw.bus_from = measure.from_bus;
      sw.bus_to = measure.to_bus;
      sw.in_service = true;
      sw.closed = false;
      sw.is_remote = true;
      sw.is_automated = false;
      sw.element_id = tie.index;
      system.ac.switches.push_back(std::move(sw));
      result.applied = true;
      result.status = "normally-open AC tie branch and switch added";
      return result;
    }
    case CounterfactualMeasureType::Automation: {
      auto sw = std::find_if(system.ac.switches.begin(), system.ac.switches.end(),
                             [&](const auto& row) {
        return row.index == measure.target_index;
      });
      if (sw == system.ac.switches.end()) {
        const auto branch = std::find_if(system.ac.branches.begin(), system.ac.branches.end(),
                                         [&](const auto& row) {
          return row.index == measure.target_index;
        });
        if (branch == system.ac.branches.end()) {
          result.status = "automation target not found";
          return result;
        }
        Switch added;
        added.index = next_index(system.ac.switches);
        added.name = measure.name;
        added.bus_from = branch->from_bus;
        added.bus_to = branch->to_bus;
        added.in_service = true;
        added.closed = branch->in_service;
        added.element_id = branch->index;
        system.ac.switches.push_back(std::move(added));
        sw = std::prev(system.ac.switches.end());
      }
      sw->is_remote = true;
      sw->is_automated = true;
      sw->t_operation_s = 3.0;
      result.applied = true;
      result.status = "switch upgraded to remote automatic operation";
      return result;
    }
    case CounterfactualMeasureType::DistributedEnergyResource: {
      const bool bus_exists = std::any_of(system.ac.buses.begin(), system.ac.buses.end(),
                                          [&](const auto& bus) {
        return bus.index == measure.bus;
      });
      if (!bus_exists || measure.capacity_mw <= 0.0) {
        result.status = "invalid DER bus or capacity";
        return result;
      }
      StaticGenerator der;
      der.index = next_index(system.ac.static_generators);
      der.bus = measure.bus;
      der.name = measure.name;
      der.p_rated_mw = measure.capacity_mw;
      der.pmax_mw = measure.capacity_mw;
      der.pmin_mw = 0.0;
      der.p_mw = measure.capacity_mw *
                 std::clamp(measure.capacity_factor, 0.0, 1.0);
      der.sn_mva = measure.capacity_mw;
      der.qmax_mvar = measure.capacity_mw * 0.33;
      der.qmin_mvar = -der.qmax_mvar;
      der.controllable = true;
      der.cost_c1 = 0.0;
      der.co2_emission_rate = 0.0;
      der.mtbf_hours = 2000.0;
      der.mttr_hours = 8.0;
      system.ac.static_generators.push_back(std::move(der));
      result.applied = true;
      result.status = "controllable low-carbon AC DER added";
      return result;
    }
  }
  result.status = "unknown measure type";
  return result;
}

CounterfactualPlanningResult run_counterfactual_planning_assessment(
    const HybridPowerSystem& system,
    const std::vector<CounterfactualMeasure>& input_measures,
    const CounterfactualPlanningOptions& options) {
  CounterfactualPlanningResult result;
  auto measures = input_measures.empty()
      ? generate_counterfactual_measures(system, options) : input_measures;
  if (options.max_measures > 0 &&
      measures.size() > static_cast<size_t>(options.max_measures))
    measures.resize(static_cast<size_t>(options.max_measures));
  result.resilience_fault_branch_indices = options.resilience_fault_branch_indices.empty()
      ? select_resilience_faults(system, options.resilience_fault_count)
      : options.resilience_fault_branch_indices;
  result.baseline = evaluate_system(system, options,
                                    result.resilience_fault_branch_indices);
  ++result.evaluated_systems;
  if (!metrics_complete(result.baseline, options)) ++result.failed_systems;

  result.measures.reserve(measures.size());
  for (const auto& measure : measures) {
    CounterfactualMeasureResult row;
    row.measure = measure;
    auto modified = system;
    const auto applied = apply_counterfactual_measure(modified, measure);
    row.applied = applied.applied;
    row.apply_status = applied.status;
    if (applied.applied) {
      row.metrics = evaluate_system(modified, options,
                                    result.resilience_fault_branch_indices);
      row.benefit = benefit_between(result.baseline, row.metrics);
      ++result.evaluated_systems;
      if (!metrics_complete(row.metrics, options)) ++result.failed_systems;
    } else {
      ++result.failed_systems;
    }
    result.measures.push_back(std::move(row));
  }

  if (options.include_pairs && options.max_pairs != 0) {
    int pair_count = 0;
    for (size_t i = 0; i < result.measures.size(); ++i) {
      for (size_t j = i + 1; j < result.measures.size(); ++j) {
        if (options.max_pairs > 0 && pair_count >= options.max_pairs) break;
        CounterfactualInteractionResult row;
        row.measure_a = result.measures[i].measure.id;
        row.measure_b = result.measures[j].measure.id;
        row.combined_capex = result.measures[i].measure.capex +
                             result.measures[j].measure.capex;
        auto modified = system;
        const auto first = apply_counterfactual_measure(
            modified, result.measures[i].measure);
        const auto second = first.applied
            ? apply_counterfactual_measure(modified, result.measures[j].measure)
            : CounterfactualApplyResult{};
        row.applied = first.applied && second.applied;
        row.apply_status = first.status + (first.applied ? "; " + second.status : "");
        if (row.applied) {
          row.metrics = evaluate_system(modified, options,
                                        result.resilience_fault_branch_indices);
          row.combined_benefit = benefit_between(result.baseline, row.metrics);
          row.synergy = synergy_between(row.combined_benefit,
                                        result.measures[i].benefit,
                                        result.measures[j].benefit);
          ++result.evaluated_systems;
          if (!metrics_complete(row.metrics, options)) ++result.failed_systems;
        } else {
          ++result.failed_systems;
        }
        result.interactions.push_back(std::move(row));
        ++pair_count;
      }
      if (options.max_pairs > 0 && pair_count >= options.max_pairs) break;
    }
  }
  result.methodology =
      "Each measure and pair is applied to an independent clone of the same baseline. "
      "Benefits are baseline loss minus counterfactual loss. Pair synergy is "
      "B(a+b)-B(a)-B(b). Economic cost uses annualized AC-OPF objective, with a "
      "monetized physical-loss fallback; carbon is annualized static carbon flow; "
      "reliability is deterministic N-1 FMEA; resilience uses an identical explicit "
      "fault set for every clone.";
  return result;
}

}  // namespace hacdcpf::analysis
