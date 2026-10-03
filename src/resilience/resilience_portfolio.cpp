#include "hacdcpf/resilience/resilience_portfolio.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <numeric>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>

#include "hacdcpf/resilience/resilience_assessment.hpp"

namespace hacdcpf::analysis {
namespace {

bool finite_nonnegative(double value) {
  return std::isfinite(value) && value >= 0.0;
}

std::unordered_map<int, std::vector<double>> read_profiles(const nlohmann::json& ts) {
  if (!ts.is_object() || !ts.value("num_steps", 0))
    throw std::invalid_argument("representative has no valid standard_time_series");
  const int steps = ts.at("num_steps").get<int>();
  if (steps < 1 || steps > 168 || !ts.contains("profiles") || !ts.at("profiles").is_array())
    throw std::invalid_argument("representative profile count is invalid");
  std::unordered_map<int, std::vector<double>> profiles;
  for (const auto& row : ts.at("profiles")) {
    if (!row.is_object() || !row.contains("id") || !row.contains("values") ||
        !row.at("values").is_array())
      throw std::invalid_argument("representative contains malformed profile");
    const int id = row.at("id").get<int>();
    const auto values = row.at("values").get<std::vector<double>>();
    if (static_cast<int>(values.size()) != steps ||
        std::any_of(values.begin(), values.end(), [](double x) { return !finite_nonnegative(x); }) ||
        !profiles.emplace(id, values).second)
      throw std::invalid_argument("representative profile length, value or ID is invalid");
  }
  return profiles;
}

DistributionResilienceOptions options_for(const ScenarioCandidate& representative) {
  if (!representative.resilience_event)
    throw std::invalid_argument("cluster representative has no resilience event");
  const auto& ts = representative.standard_time_series;
  const auto profiles = read_profiles(ts);
  const double dt = ts.at("step_duration_hr").get<double>();
  const int steps = ts.at("num_steps").get<int>();
  if (!std::isfinite(dt) || dt <= 0.0 || dt > 24.0 ||
      !std::isfinite(dt * steps) || dt * steps > 168.0)
    throw std::invalid_argument("representative time axis is invalid");
  DistributionResilienceOptions opts;
  opts.model = DistributionResilienceModel::RAStyleStageMILP;
  opts.horizon_hours = static_cast<int>(std::ceil(dt * steps));
  opts.time_step_hr = dt;
  opts.allow_reconfiguration = true;
  opts.allow_mess_dispatch = true;
  opts.use_ra_style_stage_milp = true;
  opts.enable_disaster_stages = false;
  opts.allow_stage1_open_switches = false;
  opts.allow_stage2_close_ties = false;
  opts.require_switch_for_nonfault_branch_operation = false;
  opts.allow_branch_operation_without_switch = true;
  opts.use_strict_mip_for_mess = false;
  opts.mip.solver = DistributionResilienceMIPSolver::HiGHS;
  opts.mip.mip_gap = 0.03;
  opts.mip.max_time_s = 10;
  opts.respect_fault_windows = representative.resilience_event->hazard_type == "rainstorm" ||
                               representative.resilience_event->hazard_type == "lightning";
  opts.faults = representative.resilience_event->faults;
  if (auto it = profiles.find(0); it != profiles.end()) opts.load_profile = it->second;
  if (auto it = profiles.find(1); it != profiles.end()) opts.wind_profile = it->second;
  if (auto it = profiles.find(2); it != profiles.end()) {
    opts.pv_profile = it->second;
    opts.renewable_profile = it->second;
  }
  const auto binding = ts.value("binding", nlohmann::json::object());
  const auto load_map = binding.value("load_profile_map", nlohmann::json::array());
  if (!load_map.is_array()) throw std::invalid_argument("load_profile_map must be an array");
  for (const auto& row : load_map) {
    if (!row.is_object() || !row.contains("profile_id"))
      throw std::invalid_argument("load profile binding is malformed");
    const int id = row.at("profile_id").get<int>();
    const auto it = profiles.find(id);
    if (it == profiles.end()) throw std::invalid_argument("load profile binding has unknown profile ID");
    const auto kind = row.value("kind", std::string{"AC_LOAD"});
    const int position = row.value("load_position", row.value("position", -1));
    const int index = row.value("load_index", -1);
    const int bus = row.value("bus", -1);
    if (kind == "DC_LOAD") {
      if (position >= 0) opts.dc_load_profiles_by_position[position] = it->second;
      if (index >= 0) opts.dc_load_profiles_by_index[index] = it->second;
      if (bus >= 0) opts.dc_load_profiles_by_bus[bus] = it->second;
    } else if (kind == "AC_BUS") {
      if (bus >= 0) {
        opts.ac_bus_load_profiles_by_bus[bus] = it->second;
        opts.ac_load_profiles_by_bus[bus] = it->second;
      }
    } else if (kind == "DC_BUS") {
      if (bus >= 0) {
        opts.dc_bus_load_profiles_by_bus[bus] = it->second;
        opts.dc_load_profiles_by_bus[bus] = it->second;
      }
    } else if (kind == "AC_LOAD") {
      if (position >= 0) opts.ac_load_profiles_by_position[position] = it->second;
      if (index >= 0) opts.ac_load_profiles_by_index[index] = it->second;
      if (bus >= 0) opts.ac_load_profiles_by_bus[bus] = it->second;
    } else {
      throw std::invalid_argument("unsupported load profile binding kind");
    }
  }
  return opts;
}

struct WeightedCluster {
  const ScenarioCluster* cluster{nullptr};
  std::string group;
  double weight{0.0};
};

std::vector<WeightedCluster> weighted_clusters(const ResilienceScenarioResult& result) {
  std::vector<WeightedCluster> out;
  std::unordered_set<std::string> representative_ids;
  int nonempty_groups = 0;
  for (const auto& group : result.intensities) if (!group.clusters.empty()) ++nonempty_groups;
  if (!nonempty_groups) throw std::invalid_argument("no generated resilience clusters are available");
  for (const auto& group : result.intensities) {
    if (group.clusters.empty()) continue;
    double mass = 0.0;
    for (const auto& cluster : group.clusters) {
      if (!finite_nonnegative(cluster.probability))
        throw std::invalid_argument("cluster weight must be finite and nonnegative");
      mass += cluster.probability;
    }
    if (mass <= 0.0) throw std::invalid_argument("cluster group has zero design weight");
    const std::string label = group.hazard_type + "/" + to_string(group.intensity);
    for (const auto& cluster : group.clusters) {
      if (cluster.representative_id.empty() ||
          cluster.representative.id != cluster.representative_id ||
          !representative_ids.insert(cluster.representative_id).second)
        throw std::invalid_argument("cluster representative identity is missing or ambiguous");
      out.push_back({&cluster, label,
                     cluster.probability / mass / static_cast<double>(nonempty_groups)});
    }
  }
  return out;
}

int next_generator_index(const HybridPowerSystem& system) {
  int index = 0;
  for (const auto& row : system.ac.generators) index = std::max(index, row.index);
  return index + 1;
}

int next_mobile_index(const HybridPowerSystem& system) {
  int index = 0;
  for (const auto& row : system.mobile_storage) index = std::max(index, row.index);
  return index + 1;
}

}  // namespace

void apply_resilience_portfolio_plan(HybridPowerSystem& system,
                                     const ResiliencePortfolioPlan& plan) {
  const auto has_ac_bus = [&](int index) {
    return std::count_if(system.ac.buses.begin(), system.ac.buses.end(),
                         [&](const auto& row) { return row.index == index && row.in_service; }) == 1;
  };
  if (!finite_nonnegative(plan.ac_generator_mw) ||
      !finite_nonnegative(plan.mobile_storage_mw) ||
      !finite_nonnegative(plan.mobile_storage_mwh))
    throw std::invalid_argument("portfolio resource capacity must be finite and nonnegative");
  if (plan.ac_generator_mw > 0.0) {
    if (!has_ac_bus(plan.ac_generator_bus) ||
        plan.ac_generator_index <= 0 ||
        std::any_of(system.ac.generators.begin(), system.ac.generators.end(),
                    [&](const auto& row) { return row.index == plan.ac_generator_index; }))
      throw std::invalid_argument("portfolio generator identity or capacity is invalid");
  }
  if (plan.mobile_storage_mw > 0.0) {
    if (!has_ac_bus(plan.mobile_storage_bus) || plan.mobile_storage_mwh <= 0.0 ||
        plan.mobile_storage_index <= 0 ||
        std::any_of(system.mobile_storage.begin(), system.mobile_storage.end(),
                    [&](const auto& row) { return row.index == plan.mobile_storage_index; }))
      throw std::invalid_argument("portfolio mobile storage identity or capacity is invalid");
  }
  if (plan.ac_generator_mw > 0.0) {
    Generator generator;
    generator.index = plan.ac_generator_index;
    generator.bus = plan.ac_generator_bus;
    generator.name = "Portfolio backup source";
    generator.pmax_mw = plan.ac_generator_mw;
    generator.pmin_mw = 0.0;
    generator.pg_mw = 0.0;
    generator.qmax_mvar = plan.ac_generator_mw * 0.5;
    generator.qmin_mvar = -generator.qmax_mvar;
    system.ac.generators.push_back(std::move(generator));
  }
  if (plan.mobile_storage_mw > 0.0) {
    MobileStorage storage;
    storage.index = plan.mobile_storage_index;
    storage.bus = plan.mobile_storage_bus;
    storage.target_bus = plan.mobile_storage_bus;
    storage.name = "Portfolio mobile storage";
    storage.p_rated_mw = plan.mobile_storage_mw;
    storage.pmax_mw = plan.mobile_storage_mw;
    storage.e_rated_mwh = plan.mobile_storage_mwh;
    storage.soc_init = 0.9;
    storage.soc_min = 0.1;
    storage.soc_max = 0.9;
    storage.e_mwh = storage.soc_init * storage.e_rated_mwh;
    storage.grid_forming = true;
    system.mobile_storage.push_back(std::move(storage));
  }
}

ResiliencePortfolioResult plan_resilience_portfolio(
    const HybridPowerSystem& system,
    const ResilienceScenarioResult& generated,
    bool add_generator,
    bool add_mobile_storage,
    const std::function<bool()>& should_cancel) {
  if (!add_generator && !add_mobile_storage)
    throw std::invalid_argument("select at least one portfolio resource");
  const auto clusters = weighted_clusters(generated);
  if (system.ac.buses.empty())
    throw std::invalid_argument("portfolio siting requires an AC network");

  // Shared first-stage siting score: authored load importance and the weighted
  // fault-window exposure of adjacent branches over *all* representative
  // clusters. This is a screening heuristic, not a globally optimal plan.
  std::map<int, double> site_score;
  double total_ac_load = 0.0;
  for (const auto& bus : system.ac.buses) {
    if (!bus.in_service) continue;
    site_score[bus.index] = std::max(0.0, bus.pd_mw) * std::max(1.0, bus.importance);
    total_ac_load += std::max(0.0, bus.pd_mw);
  }
  for (const auto& load : system.ac.loads) {
    if (!load.in_service || !site_score.contains(load.bus)) continue;
    site_score[load.bus] += std::max(0.0, load.p_mw * std::max(1.0, load.scaling));
    total_ac_load += std::max(0.0, load.p_mw * std::max(1.0, load.scaling));
  }
  for (const auto& item : clusters) {
    if (!item.cluster->representative.resilience_event)
      throw std::invalid_argument("cluster representative has no resilience event");
    const auto& event = *item.cluster->representative.resilience_event;
    for (const auto& fault : event.faults) {
      if (fault.branch_kind != ResilienceBranchKind::AC) continue;
      const auto branch = std::find_if(system.ac.branches.begin(), system.ac.branches.end(),
                                      [&](const auto& row) { return row.index == fault.branch_index; });
      if (branch == system.ac.branches.end())
        throw std::invalid_argument("generated fault references an unknown AC branch");
      const double exposure = item.weight * std::max(0.0, fault.repair_duration_hr);
      if (site_score.contains(branch->from_bus)) site_score[branch->from_bus] += exposure;
      if (site_score.contains(branch->to_bus)) site_score[branch->to_bus] += exposure;
    }
  }
  if (site_score.empty()) throw std::invalid_argument("no eligible AC siting bus");
  std::vector<std::pair<int, double>> ranked(site_score.begin(), site_score.end());
  std::sort(ranked.begin(), ranked.end(), [](const auto& a, const auto& b) {
    return a.second != b.second ? a.second > b.second : a.first < b.first;
  });

  ResiliencePortfolioResult output;
  auto& plan = output.plan;
  const double default_power = std::clamp(total_ac_load * 0.15, 0.2, 1.0);
  if (add_generator) {
    plan.ac_generator_bus = ranked.front().first;
    plan.ac_generator_index = next_generator_index(system);
    plan.ac_generator_mw = default_power;
  }
  if (add_mobile_storage) {
    plan.mobile_storage_bus = ranked.size() > 1 ? ranked[1].first : ranked.front().first;
    plan.mobile_storage_index = next_mobile_index(system);
    plan.mobile_storage_mw = default_power;
    plan.mobile_storage_mwh = default_power * 4.0;
  }
  HybridPowerSystem planned_system = system;
  apply_resilience_portfolio_plan(planned_system, plan);
  output.scenarios.reserve(clusters.size());
  for (const auto& item : clusters) {
    if (should_cancel && should_cancel()) throw std::runtime_error("portfolio planning cancelled");
    auto opts = options_for(item.cluster->representative);
    const auto baseline = run_distribution_resilience_assessment(system, opts);
    const auto planned = run_distribution_resilience_assessment(planned_system, opts);
    if (!baseline.feasible || !planned.feasible || !baseline.completed || !planned.completed)
      throw std::runtime_error("portfolio assessment did not complete for " +
                               item.cluster->representative_id);
    ResiliencePortfolioScenarioScore row;
    row.scenario_id = item.cluster->representative_id;
    row.group = item.group;
    row.design_weight = item.weight;
    row.baseline_shed_mwh = baseline.total_shed_mwh;
    row.planned_shed_mwh = planned.total_shed_mwh;
    row.baseline_weighted_unserved_mwh = baseline.weighted_unserved_mwh;
    row.planned_weighted_unserved_mwh = planned.weighted_unserved_mwh;
    output.baseline_design_weighted_shed_mwh += item.weight * baseline.total_shed_mwh;
    output.planned_design_weighted_shed_mwh += item.weight * planned.total_shed_mwh;
    output.baseline_worst_shed_mwh = std::max(output.baseline_worst_shed_mwh, baseline.total_shed_mwh);
    output.planned_worst_shed_mwh = std::max(output.planned_worst_shed_mwh, planned.total_shed_mwh);
    output.scenarios.push_back(std::move(row));
  }
  output.limitations = {
      "The shared siting decision is a weighted exposure heuristic, not a proven global optimum.",
      "Equal design weight is assigned to each requested intensity group; cluster weights are conditional sample weights, not hazard occurrence probabilities.",
      "Backup generator and mobile storage sizes are synthetic demo assumptions. No investment cost, fuel logistics, V2G, or fixed-storage expansion is optimized.",
      "RA-style staged restoration is a linearized/topological operational assessment; dynamic safety is not certified.",
      "Planning scores enable mobile-storage dispatch; the rapid-recovery toggle may disable it, so a single-scenario result can differ from its planning score.",
      "The same generated fault schedules are evaluated before and after resource configuration; planning does not prevent or delete faults."};
  return output;
}

}  // namespace hacdcpf::analysis
