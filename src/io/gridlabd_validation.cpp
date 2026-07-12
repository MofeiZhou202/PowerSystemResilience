#include "hacdcpf/io/gridlabd_validation.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <complex>
#include <limits>
#include <map>
#include <optional>
#include <queue>
#include <set>
#include <sstream>
#include <unordered_map>
#include <unordered_set>

#include <nlohmann/json.hpp>

#include "hacdcpf/dynamics/DynamicEvent.hpp"
#include "hacdcpf/dynamics/DynamicModelBuilder.hpp"
#include "hacdcpf/dynamics/DynamicSolver.hpp"
#include "hacdcpf/io/case_builders.hpp"

namespace hacdcpf::io {

namespace {

constexpr double kTiny = 1e-9;

std::string sanitize_model_name(std::string text) {
  for (char& ch : text) {
    const unsigned char uch = static_cast<unsigned char>(ch);
    if (!std::isalnum(uch) && ch != '_') ch = '_';
  }
  while (!text.empty() && text.front() == '_') text.erase(text.begin());
  if (text.empty()) text = "gridlabd_validation";
  if (std::isdigit(static_cast<unsigned char>(text.front()))) {
    text.insert(text.begin(), 'm');
  }
  return text;
}

ACBus make_bus(int index,
               BusType type = BusType::PQ,
               double base_kv = 12.47,
               double pd_mw = 0.0,
               double qd_mvar = 0.0) {
  ACBus b;
  b.index = index;
  b.bus_type = type;
  b.base_kv = base_kv;
  b.pd_mw = pd_mw;
  b.qd_mvar = qd_mvar;
  b.vm_pu = 1.0;
  b.va_deg = 0.0;
  b.in_service = true;
  b.name = "Bus" + std::to_string(index);
  return b;
}

ACBranch make_branch(int index,
                     int from,
                     int to,
                     double r_pu,
                     double x_pu,
                     bool in_service = true) {
  ACBranch br;
  br.index = index;
  br.from_bus = from;
  br.to_bus = to;
  br.r_pu = r_pu;
  br.x_pu = x_pu;
  br.b_pu = 0.0;
  br.tap = 1.0;
  br.shift_deg = 0.0;
  br.in_service = in_service;
  br.name = "Line" + std::to_string(index);
  return br;
}

HybridPowerSystem build_two_bus_component_case() {
  HybridPowerSystem sys;
  sys.name = "gridlabd_two_bus_component";
  sys.base_mva = 10.0;
  sys.ac.base_mva = 10.0;
  sys.ac.freq_hz = 50.0;
  sys.ac.buses = {
      make_bus(1, BusType::SLACK, 12.47),
      make_bus(2, BusType::PQ, 12.47, 1.2, 0.45),
  };
  sys.ac.branches = {make_branch(1, 1, 2, 0.012, 0.032)};
  Generator g;
  g.index = 1;
  g.bus = 1;
  g.is_slack = true;
  g.in_service = true;
  g.vg_pu = 1.0;
  sys.ac.generators.push_back(g);
  return sys;
}

HybridPowerSystem build_three_bus_meshed_case() {
  auto sys = build_two_bus_component_case();
  sys.name = "gridlabd_three_bus_meshed";
  sys.ac.buses.push_back(make_bus(3, BusType::PQ, 12.47, 0.55, 0.18));
  sys.ac.branches = {
      make_branch(1, 1, 2, 0.012, 0.032),
      make_branch(2, 2, 3, 0.010, 0.025),
      make_branch(3, 1, 3, 0.018, 0.040),
  };
  return sys;
}

HybridPowerSystem build_three_bus_open_tie_case() {
  auto sys = build_three_bus_meshed_case();
  sys.name = "gridlabd_three_bus_open_tie";
  for (auto& br : sys.ac.branches) {
    if (br.index == 3) br.in_service = false;
  }
  return sys;
}

HybridPowerSystem build_static_der_component_case() {
  auto sys = build_three_bus_meshed_case();
  sys.name = "gridlabd_static_der_component";
  StaticGenerator sg;
  sg.index = 1;
  sg.bus = 3;
  sg.in_service = true;
  sg.p_mw = 0.20;
  sg.q_mvar = 0.04;
  sg.scaling = 1.0;
  sg.name = "StaticDER1";
  sys.ac.static_generators.push_back(sg);
  return sys;
}

double system_base_mva(const HybridPowerSystem& sys) {
  if (sys.base_mva > 0.0) return sys.base_mva;
  if (sys.ac.base_mva > 0.0) return sys.ac.base_mva;
  return 100.0;
}

bool is_slack_bus(const HybridPowerSystem& sys, int bus) {
  for (const auto& b : sys.ac.buses) {
    if (b.index == bus && b.in_service && b.bus_type == BusType::SLACK) return true;
  }
  for (const auto& g : sys.ac.generators) {
    if (g.in_service && g.is_slack && g.bus == bus) return true;
  }
  for (const auto& eg : sys.ac.external_grids) {
    if (eg.in_service && eg.bus == bus) return true;
  }
  return false;
}

std::vector<int> slack_buses(const HybridPowerSystem& sys) {
  std::vector<int> out;
  for (const auto& b : sys.ac.buses) {
    if (b.in_service && b.bus_type == BusType::SLACK) out.push_back(b.index);
  }
  for (const auto& g : sys.ac.generators) {
    if (g.in_service && g.is_slack) out.push_back(g.bus);
  }
  for (const auto& eg : sys.ac.external_grids) {
    if (eg.in_service) out.push_back(eg.bus);
  }
  std::sort(out.begin(), out.end());
  out.erase(std::unique(out.begin(), out.end()), out.end());
  if (out.empty() && !sys.ac.buses.empty()) out.push_back(sys.ac.buses.front().index);
  return out;
}

bool bus_has_power_or_source(const HybridPowerSystem& sys, int bus) {
  for (const auto& b : sys.ac.buses) {
    if (b.index == bus && b.in_service &&
        (std::abs(b.pd_mw) > kTiny || std::abs(b.qd_mvar) > kTiny ||
         std::abs(b.gs_mw) > kTiny || std::abs(b.bs_mvar) > kTiny ||
         b.bus_type == BusType::SLACK || b.bus_type == BusType::PV)) {
      return true;
    }
  }
  for (const auto& load : sys.ac.loads) {
    if (load.in_service && load.bus == bus &&
        (std::abs(load.p_mw * load.scaling) > kTiny ||
         std::abs(load.q_mvar * load.scaling) > kTiny)) {
      return true;
    }
  }
  for (const auto& sh : sys.ac.shunts) {
    if (sh.in_service && sh.bus == bus &&
        (std::abs(sh.gs_mw) > kTiny || std::abs(sh.bs_mvar) > kTiny)) {
      return true;
    }
  }
  for (const auto& g : sys.ac.generators) {
    if (g.in_service && g.bus == bus) return true;
  }
  for (const auto& g : sys.ac.static_generators) {
    if (g.in_service && g.bus == bus) return true;
  }
  for (const auto& g : sys.ac.renewable_gens) {
    if (g.in_service && g.bus == bus) return true;
  }
  for (const auto& pv : sys.ac.pv_systems) {
    if (pv.in_service && pv.bus == bus) return true;
  }
  for (const auto& st : sys.ac.storage) {
    if (st.in_service && st.bus == bus) return true;
  }
  return false;
}

bool preserves_slack_connectivity(const HybridPowerSystem& sys,
                                  std::optional<int> excluded_branch_index = {}) {
  const auto slacks = slack_buses(sys);
  if (slacks.empty()) return false;

  std::unordered_map<int, std::vector<int>> adj;
  std::unordered_set<int> in_service_buses;
  for (const auto& bus : sys.ac.buses) {
    if (bus.in_service) in_service_buses.insert(bus.index);
  }
  for (const auto& br : sys.ac.branches) {
    if (!br.in_service) continue;
    if (excluded_branch_index.has_value() && br.index == *excluded_branch_index) {
      continue;
    }
    if (in_service_buses.count(br.from_bus) == 0U ||
        in_service_buses.count(br.to_bus) == 0U) {
      continue;
    }
    adj[br.from_bus].push_back(br.to_bus);
    adj[br.to_bus].push_back(br.from_bus);
  }

  std::queue<int> q;
  std::unordered_set<int> reached;
  for (const int slack : slacks) {
    if (in_service_buses.count(slack) == 0U) continue;
    reached.insert(slack);
    q.push(slack);
  }
  while (!q.empty()) {
    const int u = q.front();
    q.pop();
    for (const int v : adj[u]) {
      if (reached.insert(v).second) q.push(v);
    }
  }

  for (const int bus : in_service_buses) {
    if (!bus_has_power_or_source(sys, bus)) continue;
    if (reached.count(bus) == 0U) return false;
  }
  return true;
}

std::vector<int> ranked_load_buses(const HybridPowerSystem& sys) {
  std::map<int, double> load_by_bus;
  for (const auto& bus : sys.ac.buses) {
    if (!bus.in_service) continue;
    const double s = std::hypot(bus.pd_mw, bus.qd_mvar);
    if (s > kTiny) load_by_bus[bus.index] += s;
  }
  for (const auto& load : sys.ac.loads) {
    if (!load.in_service) continue;
    const double scale = load.scaling > 0.0 ? load.scaling : 1.0;
    const double s = std::hypot(scale * load.p_mw, scale * load.q_mvar);
    if (s > kTiny) load_by_bus[load.bus] += s;
  }
  std::vector<std::pair<int, double>> rows(load_by_bus.begin(), load_by_bus.end());
  std::sort(rows.begin(), rows.end(), [](const auto& a, const auto& b) {
    if (a.second == b.second) return a.first < b.first;
    return a.second > b.second;
  });
  std::vector<int> out;
  out.reserve(rows.size());
  for (const auto& [bus, ignored] : rows) {
    (void)ignored;
    if (!is_slack_bus(sys, bus)) out.push_back(bus);
  }
  return out;
}

std::optional<int> first_non_slack_bus(const HybridPowerSystem& sys) {
  for (const auto& bus : sys.ac.buses) {
    if (!bus.in_service || is_slack_bus(sys, bus.index)) continue;
    return bus.index;
  }
  return std::nullopt;
}

bool has_ac_bus(const HybridPowerSystem& sys, int bus_index) {
  return std::any_of(sys.ac.buses.begin(), sys.ac.buses.end(),
                     [&](const ACBus& bus) {
                       return bus.in_service && bus.index == bus_index;
                     });
}

void push_note(GridLABDFailureScenario& scenario, std::string note) {
  scenario.notes.push_back(std::move(note));
}

nlohmann::json scenario_to_json(const GridLABDFailureScenario& s) {
  nlohmann::json j;
  j["name"] = s.name;
  j["kind"] = gridlabd_failure_kind_name(s.kind);
  j["component_index"] = s.component_index;
  j["bus"] = s.bus;
  j["value"] = s.value;
  j["r_pu"] = s.r_pu;
  j["x_pu"] = s.x_pu;
  j["scale"] = s.scale;
  j["expected_exact_ac_snapshot"] = s.expected_exact_ac_snapshot;
  j["scope"] = s.scope;
  j["notes"] = s.notes;
  return j;
}

nlohmann::json comparison_summary_to_json(const GridLABDComparisonReport& c) {
  nlohmann::json j;
  j["gridlabd_available"] = c.gridlabd_available;
  j["gridlabd_run_attempted"] = c.gridlabd_run_attempted;
  j["gridlabd_run_success"] = c.gridlabd_run_success;
  j["hacdcpf_power_flow_converged"] = c.hacdcpf_power_flow_converged;
  j["numerical_comparison_passed"] = c.numerical_comparison_passed;
  j["equivalence_passed"] = c.equivalence_passed;
  j["passed"] = c.passed;
  j["equivalence_scope"] = c.equivalence_scope;
  j["equivalence_claim"] = c.equivalence_claim;
  j["unsupported_features"] = c.unsupported_features;
  j["diagnostic_only_reasons"] = c.diagnostic_only_reasons;
  j["warnings"] = c.warnings;
  j["skipped"] = c.skipped;

  int failed = 0;
  nlohmann::json failed_items = nlohmann::json::array();
  for (const auto& item : c.items) {
    if (item.passed) continue;
    ++failed;
    if (failed_items.size() < 20U) {
      failed_items.push_back({
          {"kind", item.kind},
          {"key", item.key},
          {"hacdcpf_value", item.hacdcpf_value},
          {"gridlabd_value", item.gridlabd_value},
          {"difference", item.difference},
          {"tolerance", item.tolerance},
          {"detail", item.detail},
      });
    }
  }
  j["comparison_item_count"] = c.items.size();
  j["failed_item_count"] = failed;
  j["failed_items"] = failed_items;
  return j;
}

GridLABDComparisonItem make_comparison_item(std::string kind,
                                            std::string key,
                                            double hacdcpf_value,
                                            double gridlabd_value,
                                            double tolerance,
                                            std::string detail = {}) {
  GridLABDComparisonItem item;
  item.kind = std::move(kind);
  item.key = std::move(key);
  item.hacdcpf_value = hacdcpf_value;
  item.gridlabd_value = gridlabd_value;
  item.difference = hacdcpf_value - gridlabd_value;
  item.tolerance = tolerance;
  item.passed = std::abs(item.difference) <= tolerance;
  item.detail = std::move(detail);
  return item;
}

bool same_time(double a, double b, double tol = 1e-8) {
  return std::abs(a - b) <= tol * std::max({1.0, std::abs(a), std::abs(b)});
}

bool is_active_at(const GridLABDLongDurationEvent& event, double t) {
  if (t + 1e-10 < event.time_s) return false;
  if (event.duration_s <= 0.0) return true;
  return t < event.time_s + event.duration_s - 1e-10;
}

std::string long_event_label(const GridLABDLongDurationEvent& event) {
  if (!event.label.empty()) return event.label;
  if (!event.scenario.name.empty()) return event.scenario.name;
  return gridlabd_failure_kind_name(event.scenario.kind) + "@" +
         std::to_string(event.time_s);
}

bool event_is_active_fault(const GridLABDLongDurationEvent& event, double t) {
  return event.scenario.kind == GridLABDFailureKind::FaultShunt &&
         is_active_at(event, t);
}

GridLABDScenarioApplication materialize_long_duration_state(
    const HybridPowerSystem& base,
    const std::vector<GridLABDLongDurationEvent>& events,
    double t,
    std::vector<std::string>& active_labels,
    bool& active_fault) {
  GridLABDScenarioApplication out;
  out.system = base;
  out.applied = true;
  out.comparable_with_gridlabd = true;
  out.exact_ac_snapshot = true;
  active_fault = false;

  for (const auto& event : events) {
    if (!is_active_at(event, t)) continue;
    active_labels.push_back(long_event_label(event));
    active_fault = active_fault || event.scenario.kind == GridLABDFailureKind::FaultShunt;
    GridLABDScenarioApplication step =
        apply_gridlabd_failure_scenario(out.system, event.scenario);
    out.system = std::move(step.system);
    out.applied = out.applied && step.applied;
    out.comparable_with_gridlabd =
        out.comparable_with_gridlabd && step.comparable_with_gridlabd;
    out.exact_ac_snapshot = out.exact_ac_snapshot && step.exact_ac_snapshot;
    out.warnings.insert(out.warnings.end(),
                        step.warnings.begin(),
                        step.warnings.end());
  }

  return out;
}

dynamics::DynamicEventType dynamic_event_type_for(
    GridLABDFailureKind kind,
    bool restore) {
  using dynamics::DynamicEventType;
  switch (kind) {
    case GridLABDFailureKind::ACBranchTrip:
      return restore ? DynamicEventType::ACBranchClose
                     : DynamicEventType::ACBranchTrip;
    case GridLABDFailureKind::ACBranchClose:
      return restore ? DynamicEventType::ACBranchTrip
                     : DynamicEventType::ACBranchClose;
    case GridLABDFailureKind::ACLoadScale:
      return DynamicEventType::ACLoadScale;
    case GridLABDFailureKind::GeneratorTrip:
    case GridLABDFailureKind::StaticGeneratorTrip:
    case GridLABDFailureKind::RenewableTrip:
    case GridLABDFailureKind::PVTrip:
      return DynamicEventType::GeneratorTrip;
    case GridLABDFailureKind::StorageTrip:
      return DynamicEventType::StoragePowerStep;
    case GridLABDFailureKind::FaultShunt:
      return DynamicEventType::FaultShunt;
    case GridLABDFailureKind::DCBranchTrip:
      return restore ? DynamicEventType::DCBranchClose
                     : DynamicEventType::DCBranchTrip;
    case GridLABDFailureKind::DCBranchClose:
      return restore ? DynamicEventType::DCBranchTrip
                     : DynamicEventType::DCBranchClose;
    case GridLABDFailureKind::DCLoadScale:
      return DynamicEventType::DCLoadScale;
    case GridLABDFailureKind::DCDCTrip:
      return DynamicEventType::DCDCTrip;
    case GridLABDFailureKind::VSCTrip:
      return DynamicEventType::VSCTrip;
    case GridLABDFailureKind::DCStorageTrip:
      return DynamicEventType::DCStoragePowerStep;
    case GridLABDFailureKind::BaseCase:
      return DynamicEventType::Custom;
  }
  return DynamicEventType::Custom;
}

std::optional<dynamics::DynamicEvent> dynamic_event_from_long_event(
    const GridLABDLongDurationEvent& event,
    bool restore) {
  if (event.scenario.kind == GridLABDFailureKind::BaseCase) return std::nullopt;
  dynamics::DynamicEvent out;
  out.time_s = restore ? event.time_s + event.duration_s : event.time_s;
  out.type = dynamic_event_type_for(event.scenario.kind, restore);
  out.component_index = event.scenario.component_index;
  out.bus = event.scenario.bus;
  out.target_id = event.scenario.bus;
  out.component_type =
      event.scenario.kind == GridLABDFailureKind::DCBranchTrip ||
              event.scenario.kind == GridLABDFailureKind::DCBranchClose ||
              event.scenario.kind == GridLABDFailureKind::DCLoadScale ||
              event.scenario.kind == GridLABDFailureKind::DCDCTrip ||
              event.scenario.kind == GridLABDFailureKind::DCStorageTrip
          ? "DC"
          : "AC";
  out.label = (restore ? "restore " : "") + long_event_label(event);

  if (event.scenario.kind == GridLABDFailureKind::FaultShunt) {
    out.duration_s = restore ? 0.0 : event.duration_s;
    out.params["r_pu"] = event.scenario.r_pu;
    out.params["x_pu"] = event.scenario.x_pu;
    if (event.duration_s > 0.0) out.params["duration_s"] = event.duration_s;
    return out;
  }

  if (event.scenario.kind == GridLABDFailureKind::ACLoadScale ||
      event.scenario.kind == GridLABDFailureKind::DCLoadScale) {
    const double scale = restore ? 1.0
                                 : (event.scenario.scale > 0.0
                                        ? event.scenario.scale
                                        : event.scenario.value);
    out.value = scale;
    out.params["scale"] = scale;
    return out;
  }

  out.value = event.scenario.value;
  return out;
}

std::vector<double> long_duration_sample_times(
    const std::vector<GridLABDLongDurationEvent>& events,
    const GridLABDLongDurationOptions& options) {
  std::vector<double> times;
  auto add = [&](double t) {
    if (!std::isfinite(t)) return;
    t = std::clamp(t, 0.0, options.t_end_s);
    for (const double existing : times) {
      if (same_time(existing, t, 1e-7)) return;
    }
    times.push_back(t);
  };

  add(0.0);
  add(options.t_end_s);
  if (options.sample_interval_s > 0.0) {
    for (double t = options.sample_interval_s;
         t < options.t_end_s - 1e-10;
         t += options.sample_interval_s) {
      add(t);
    }
  }
  if (options.include_event_probe_samples) {
    const double eps = std::max(1e-7, options.event_probe_offset_s);
    for (const auto& event : events) {
      add(event.time_s - eps);
      add(event.time_s);
      if (event.duration_s > 0.0) {
        add(event.time_s + 0.5 * event.duration_s);
        add(event.time_s + event.duration_s);
        add(event.time_s + event.duration_s + eps);
      }
    }
  }
  std::sort(times.begin(), times.end());
  return times;
}

const dynamics::DynamicSnapshot* dynamic_snapshot_not_after(
    const dynamics::DynamicResults& result,
    double t) {
  const dynamics::DynamicSnapshot* latest = nullptr;
  const dynamics::DynamicSnapshot* nearest = nullptr;
  double nearest_error = std::numeric_limits<double>::infinity();
  for (const auto& snapshot : result.snapshots) {
    if (snapshot.time_s <= t + 1e-9 &&
        (latest == nullptr || snapshot.time_s > latest->time_s)) {
      latest = &snapshot;
    }
    const double error = std::abs(snapshot.time_s - t);
    if (error < nearest_error) {
      nearest_error = error;
      nearest = &snapshot;
    }
  }
  return latest != nullptr ? latest : nearest;
}

std::string long_duration_stage(bool active_fault,
                                const std::vector<std::string>& active_events) {
  if (active_fault) return "active_fault";
  if (!active_events.empty()) return "active_contingency";
  return "prefault_or_restored";
}

int ac_bus_position(const std::vector<int>& bus_ids, int bus_index) {
  const auto it = std::find(bus_ids.begin(), bus_ids.end(), bus_index);
  if (it == bus_ids.end()) return -1;
  return static_cast<int>(std::distance(bus_ids.begin(), it));
}

}  // namespace

std::string gridlabd_failure_kind_name(GridLABDFailureKind kind) {
  switch (kind) {
    case GridLABDFailureKind::BaseCase:
      return "BaseCase";
    case GridLABDFailureKind::ACBranchTrip:
      return "ACBranchTrip";
    case GridLABDFailureKind::ACBranchClose:
      return "ACBranchClose";
    case GridLABDFailureKind::ACLoadScale:
      return "ACLoadScale";
    case GridLABDFailureKind::GeneratorTrip:
      return "GeneratorTrip";
    case GridLABDFailureKind::StaticGeneratorTrip:
      return "StaticGeneratorTrip";
    case GridLABDFailureKind::RenewableTrip:
      return "RenewableTrip";
    case GridLABDFailureKind::PVTrip:
      return "PVTrip";
    case GridLABDFailureKind::StorageTrip:
      return "StorageTrip";
    case GridLABDFailureKind::FaultShunt:
      return "FaultShunt";
    case GridLABDFailureKind::DCBranchTrip:
      return "DCBranchTrip";
    case GridLABDFailureKind::DCBranchClose:
      return "DCBranchClose";
    case GridLABDFailureKind::DCLoadScale:
      return "DCLoadScale";
    case GridLABDFailureKind::DCDCTrip:
      return "DCDCTrip";
    case GridLABDFailureKind::VSCTrip:
      return "VSCTrip";
    case GridLABDFailureKind::DCStorageTrip:
      return "DCStorageTrip";
  }
  return "Unknown";
}

std::vector<GridLABDStandardCase> build_gridlabd_standard_cases(
    const GridLABDValidationMatrixOptions& options) {
  std::vector<GridLABDStandardCase> cases;

  auto add = [&](std::string name,
                 std::string category,
                 std::string source,
                 HybridPowerSystem system,
                 bool large = false) {
    if (large && !options.include_large_cases) return;
    cases.push_back(GridLABDStandardCase{
        .name = std::move(name),
        .category = std::move(category),
        .source = std::move(source),
        .system = std::move(system),
        .large = large,
    });
  };

  if (options.include_component_cases) {
    add("two_bus_component", "component", "internal balanced AC component case",
        build_two_bus_component_case());
    add("three_bus_meshed", "component", "internal meshed AC component case",
        build_three_bus_meshed_case());
    add("three_bus_open_tie", "component", "internal AC open-tie component case",
        build_three_bus_open_tie_case());
    add("static_der_component", "component", "internal static-DER component case",
        build_static_der_component_case());
  }

  if (options.include_distribution_cases) {
    add("case33bw_ac", "distribution", "MATPOWER case33bw projected to AC",
        build_ac_only_version(build_case33bw_acdc()));
    add("case69_ac", "distribution", "MATPOWER case69 projected to AC",
        build_ac_only_version(build_case69_acdc()));
  }

  if (options.include_transmission_cases) {
    add("ieee14_ac", "transmission", "IEEE 14 AC benchmark",
        build_ac_only_version(build_ieee14_acdc()));
    add("case300_ac", "transmission", "MATPOWER case300 projected to AC",
        build_ac_only_version(build_case300_acdc()), true);
  }

  return cases;
}

std::vector<GridLABDFailureScenario> build_gridlabd_failure_scenarios(
    const HybridPowerSystem& sys,
    const GridLABDValidationMatrixOptions& options) {
  std::vector<GridLABDFailureScenario> scenarios;

  if (options.include_base_case) {
    scenarios.push_back(GridLABDFailureScenario{
        .name = "base_case",
        .kind = GridLABDFailureKind::BaseCase,
        .expected_exact_ac_snapshot = true,
        .scope = "Balanced AC algebraic base snapshot",
    });
  }

  int branch_trip_count = 0;
  int diagnostic_branch_trip_count = 0;
  for (const auto& branch : sys.ac.branches) {
    if (branch_trip_count >= options.max_branch_trip_scenarios_per_case) break;
    if (!branch.in_service) continue;
    GridLABDFailureScenario scenario;
    scenario.kind = GridLABDFailureKind::ACBranchTrip;
    scenario.component_index = branch.index;
    scenario.name = "trip_ac_branch_" + std::to_string(branch.index);
    scenario.scope = "Post-contingency balanced AC branch-outage snapshot";
    if (!preserves_slack_connectivity(sys, branch.index)) {
      if (!options.include_diagnostic_islanding_branch_trips ||
          diagnostic_branch_trip_count > 0) {
        continue;
      }
      scenario.expected_exact_ac_snapshot = false;
      push_note(scenario,
                "Branch trip disconnects energized AC load/source islands; "
                "GridLAB-D comparison is diagnostic until island restoration "
                "and load-shed semantics are part of the harness.");
      ++diagnostic_branch_trip_count;
    }
    scenarios.push_back(std::move(scenario));
    ++branch_trip_count;
  }

  int branch_close_count = 0;
  for (const auto& branch : sys.ac.branches) {
    if (branch_close_count >= options.max_branch_close_scenarios_per_case) break;
    if (branch.in_service) continue;
    GridLABDFailureScenario scenario;
    scenario.kind = GridLABDFailureKind::ACBranchClose;
    scenario.component_index = branch.index;
    scenario.name = "close_ac_branch_" + std::to_string(branch.index);
    scenario.expected_exact_ac_snapshot = true;
    scenario.scope = "Post-contingency balanced AC branch-close snapshot";
    scenarios.push_back(std::move(scenario));
    ++branch_close_count;
  }

  const auto load_buses = ranked_load_buses(sys);
  int load_scale_count = 0;
  for (const int bus : load_buses) {
    if (load_scale_count >= options.max_load_scale_scenarios_per_case) break;
    GridLABDFailureScenario up;
    up.kind = GridLABDFailureKind::ACLoadScale;
    up.bus = bus;
    up.component_index = bus;
    up.scale = load_scale_count == 0 ? options.load_step_up_scale
                                     : options.load_step_down_scale;
    up.value = up.scale;
    up.name = "scale_ac_load_bus_" + std::to_string(bus) + "_" +
              std::to_string(static_cast<int>(std::round(up.scale * 100.0))) +
              "pct";
    up.expected_exact_ac_snapshot = true;
    up.scope = "Post-contingency balanced AC constant-PQ load disturbance";
    scenarios.push_back(std::move(up));
    ++load_scale_count;
  }

  int gen_trip_count = 0;
  for (const auto& gen : sys.ac.generators) {
    if (gen_trip_count >= options.max_generator_trip_scenarios_per_case) break;
    if (!gen.in_service || gen.is_slack) continue;
    GridLABDFailureScenario scenario;
    scenario.kind = GridLABDFailureKind::GeneratorTrip;
    scenario.component_index = gen.index;
    scenario.bus = gen.bus;
    scenario.name = "trip_generator_" + std::to_string(gen.index);
    scenario.expected_exact_ac_snapshot = false;
    scenario.scope = "Generator trip as post-contingency AC snapshot";
    push_note(scenario,
              "Non-slack synchronous generator voltage regulation is not an "
              "exact GridLAB-D snapshot object yet; this row is diagnostic.");
    scenarios.push_back(std::move(scenario));
    ++gen_trip_count;
  }

  int static_trip_count = 0;
  for (const auto& gen : sys.ac.static_generators) {
    if (static_trip_count >= options.max_generator_trip_scenarios_per_case) break;
    if (!gen.in_service) continue;
    GridLABDFailureScenario scenario;
    scenario.kind = GridLABDFailureKind::StaticGeneratorTrip;
    scenario.component_index = gen.index;
    scenario.bus = gen.bus;
    scenario.name = "trip_static_generator_" + std::to_string(gen.index);
    scenario.expected_exact_ac_snapshot = true;
    scenario.scope = "Static DER trip as fixed-PQ AC injection removal";
    scenarios.push_back(std::move(scenario));
    ++static_trip_count;
  }

  if (options.include_fault_shunts) {
    const auto bus = !load_buses.empty() ? std::optional<int>(load_buses.front())
                                         : first_non_slack_bus(sys);
    if (bus.has_value() &&
        options.max_fault_shunt_scenarios_per_case > 0) {
      GridLABDFailureScenario scenario;
      scenario.kind = GridLABDFailureKind::FaultShunt;
      scenario.bus = *bus;
      scenario.component_index = *bus;
      scenario.r_pu = options.fault_r_pu;
      scenario.x_pu = options.fault_x_pu;
      scenario.name = "fault_shunt_bus_" + std::to_string(*bus);
      scenario.expected_exact_ac_snapshot =
          options.comparison_options.export_options
              .include_shunt_admittance_as_impedance;
      scenario.scope =
          "Post-contingency balanced AC shunt-fault admittance snapshot";
      if (!scenario.expected_exact_ac_snapshot) {
        push_note(scenario,
                  "Shunt admittance impedance export is disabled, so this "
                  "fault becomes diagnostic-only.");
      }
      scenarios.push_back(std::move(scenario));
    }
  }

  if (options.include_diagnostic_native_failures) {
    if (!sys.vsc_converters.empty()) {
      const auto& vsc = sys.vsc_converters.front();
      if (vsc.in_service) {
        GridLABDFailureScenario scenario;
        scenario.kind = GridLABDFailureKind::VSCTrip;
        scenario.component_index = vsc.index;
        scenario.bus = vsc.bus_ac;
        scenario.name = "trip_vsc_" + std::to_string(vsc.index);
        scenario.expected_exact_ac_snapshot = false;
        scenario.scope = "Native hybrid AC/DC converter failure";
        push_note(scenario,
                  "VSC internal controls and the DC network are outside the "
                  "current GridLAB-D exact AC snapshot harness.");
        scenarios.push_back(std::move(scenario));
      }
    }
    if (!sys.dc.branches.empty()) {
      const auto& branch = sys.dc.branches.front();
      if (branch.in_service) {
        GridLABDFailureScenario scenario;
        scenario.kind = GridLABDFailureKind::DCBranchTrip;
        scenario.component_index = branch.index;
        scenario.name = "trip_dc_branch_" + std::to_string(branch.index);
        scenario.expected_exact_ac_snapshot = false;
        scenario.scope = "Native DC branch failure";
        push_note(scenario,
                  "DC branch topology is not represented in the GridLAB-D AC "
                  "snapshot; this row records taxonomy coverage only.");
        scenarios.push_back(std::move(scenario));
      }
    }
  }

  return scenarios;
}

GridLABDScenarioApplication apply_gridlabd_failure_scenario(
    const HybridPowerSystem& sys,
    const GridLABDFailureScenario& scenario) {
  GridLABDScenarioApplication out;
  out.system = sys;
  out.comparable_with_gridlabd = true;
  out.exact_ac_snapshot = scenario.expected_exact_ac_snapshot;

  auto mark_missing = [&](const std::string& what) {
    out.applied = false;
    out.warnings.push_back("Scenario target not found: " + what + ".");
  };

  switch (scenario.kind) {
    case GridLABDFailureKind::BaseCase:
      out.applied = true;
      break;

    case GridLABDFailureKind::ACBranchTrip: {
      for (auto& branch : out.system.ac.branches) {
        if (branch.index == scenario.component_index) {
          branch.in_service = false;
          out.applied = true;
          break;
        }
      }
      if (!out.applied) mark_missing("AC branch " + std::to_string(scenario.component_index));
      if (!preserves_slack_connectivity(out.system)) {
        out.exact_ac_snapshot = false;
        out.warnings.push_back(
            "AC branch trip creates an island without an explicit restoration/"
            "load-shed policy; comparison is diagnostic-only.");
      }
      break;
    }

    case GridLABDFailureKind::ACBranchClose: {
      for (auto& branch : out.system.ac.branches) {
        if (branch.index == scenario.component_index) {
          branch.in_service = true;
          out.applied = true;
          break;
        }
      }
      if (!out.applied) mark_missing("AC branch " + std::to_string(scenario.component_index));
      break;
    }

    case GridLABDFailureKind::ACLoadScale: {
      const int target_bus = scenario.bus != 0 ? scenario.bus
                                               : scenario.component_index;
      const double scale =
          scenario.scale > 0.0 ? scenario.scale
                               : (scenario.value > 0.0 ? scenario.value : 1.0);
      for (auto& bus : out.system.ac.buses) {
        if (!bus.in_service) continue;
        if (target_bus != 0 && bus.index != target_bus) continue;
        bus.pd_mw *= scale;
        bus.qd_mvar *= scale;
        out.applied = true;
      }
      for (auto& load : out.system.ac.loads) {
        if (!load.in_service) continue;
        if (target_bus != 0 && load.bus != target_bus) continue;
        load.scaling *= scale;
        out.applied = true;
      }
      for (auto& load : out.system.ac.flexible_loads) {
        if (!load.in_service) continue;
        if (target_bus != 0 && load.bus != target_bus) continue;
        load.p_mw *= scale;
        load.q_mvar *= scale;
        out.applied = true;
      }
      for (auto& load : out.system.ac.asymmetric_loads) {
        if (!load.in_service) continue;
        if (target_bus != 0 && load.bus != target_bus) continue;
        load.scaling *= scale;
        out.applied = true;
      }
      for (auto& station : out.system.ac.charging_stations) {
        if (!station.in_service) continue;
        if (target_bus != 0 && station.bus != target_bus) continue;
        station.p_total_kw *= scale;
        station.q_total_kvar *= scale;
        out.applied = true;
      }
      if (!out.applied) mark_missing("AC load at bus " + std::to_string(target_bus));
      break;
    }

    case GridLABDFailureKind::GeneratorTrip: {
      for (auto& gen : out.system.ac.generators) {
        if (gen.index == scenario.component_index) {
          if (gen.is_slack) {
            out.exact_ac_snapshot = false;
            out.warnings.push_back(
                "Slack generator trip requires source reassignment before an "
                "exact GridLAB-D AC snapshot comparison.");
          }
          gen.in_service = false;
          out.applied = true;
          break;
        }
      }
      if (!out.applied) mark_missing("generator " + std::to_string(scenario.component_index));
      break;
    }

    case GridLABDFailureKind::StaticGeneratorTrip: {
      for (auto& gen : out.system.ac.static_generators) {
        if (gen.index == scenario.component_index) {
          gen.in_service = false;
          out.applied = true;
          break;
        }
      }
      if (!out.applied) {
        mark_missing("static generator " + std::to_string(scenario.component_index));
      }
      break;
    }

    case GridLABDFailureKind::RenewableTrip: {
      for (auto& gen : out.system.ac.renewable_gens) {
        if (gen.index == scenario.component_index) {
          gen.in_service = false;
          out.applied = true;
          break;
        }
      }
      if (!out.applied) mark_missing("renewable generator " + std::to_string(scenario.component_index));
      break;
    }

    case GridLABDFailureKind::PVTrip: {
      for (auto& pv : out.system.ac.pv_systems) {
        if (pv.index == scenario.component_index) {
          pv.in_service = false;
          out.applied = true;
          break;
        }
      }
      if (!out.applied) mark_missing("PV system " + std::to_string(scenario.component_index));
      break;
    }

    case GridLABDFailureKind::StorageTrip: {
      for (auto& storage : out.system.ac.storage) {
        if (storage.index == scenario.component_index) {
          storage.in_service = false;
          out.applied = true;
          break;
        }
      }
      if (!out.applied) mark_missing("AC storage " + std::to_string(scenario.component_index));
      break;
    }

    case GridLABDFailureKind::FaultShunt: {
      const int bus_index = scenario.bus != 0 ? scenario.bus
                                               : scenario.component_index;
      if (!has_ac_bus(out.system, bus_index)) {
        mark_missing("AC fault bus " + std::to_string(bus_index));
        break;
      }
      const double r = scenario.r_pu;
      const double x = scenario.x_pu;
      std::complex<double> y_pu{0.0, 0.0};
      if (std::abs(r) > kTiny || std::abs(x) > kTiny) {
        const std::complex<double> z(std::max(0.0, r), x);
        y_pu = std::complex<double>(1.0, 0.0) / z;
      } else {
        y_pu = {scenario.value > 0.0 ? scenario.value : 1.0, 0.0};
      }
      const double base = system_base_mva(out.system);
      for (auto& bus : out.system.ac.buses) {
        if (bus.index != bus_index) continue;
        bus.gs_mw += y_pu.real() * base;
        bus.bs_mvar += y_pu.imag() * base;
        out.applied = true;
        break;
      }
      if (!out.applied) mark_missing("AC fault bus " + std::to_string(bus_index));
      break;
    }

    case GridLABDFailureKind::DCBranchTrip:
    case GridLABDFailureKind::DCBranchClose:
      out.comparable_with_gridlabd = false;
      out.exact_ac_snapshot = false;
      for (auto& branch : out.system.dc.branches) {
        if (branch.index == scenario.component_index) {
          branch.in_service = scenario.kind == GridLABDFailureKind::DCBranchClose;
          out.applied = true;
          break;
        }
      }
      if (!out.applied) mark_missing("DC branch " + std::to_string(scenario.component_index));
      break;

    case GridLABDFailureKind::DCLoadScale:
      out.comparable_with_gridlabd = false;
      out.exact_ac_snapshot = false;
      for (auto& load : out.system.dc.loads) {
        if (scenario.bus != 0 && load.bus != scenario.bus) continue;
        if (scenario.bus == 0 && scenario.component_index != 0 &&
            load.index != scenario.component_index) {
          continue;
        }
        load.scaling *= scenario.scale > 0.0 ? scenario.scale : scenario.value;
        out.applied = true;
      }
      if (!out.applied) mark_missing("DC load");
      break;

    case GridLABDFailureKind::DCDCTrip:
      out.comparable_with_gridlabd = false;
      out.exact_ac_snapshot = false;
      for (auto& converter : out.system.dc.dcdc_converters) {
        if (converter.index == scenario.component_index) {
          converter.in_service = false;
          out.applied = true;
          break;
        }
      }
      if (!out.applied) mark_missing("DCDC converter " + std::to_string(scenario.component_index));
      break;

    case GridLABDFailureKind::VSCTrip:
      out.comparable_with_gridlabd = false;
      out.exact_ac_snapshot = false;
      for (auto& converter : out.system.vsc_converters) {
        if (converter.index == scenario.component_index) {
          converter.in_service = false;
          out.applied = true;
          break;
        }
      }
      if (!out.applied) mark_missing("VSC converter " + std::to_string(scenario.component_index));
      break;

    case GridLABDFailureKind::DCStorageTrip:
      out.comparable_with_gridlabd = false;
      out.exact_ac_snapshot = false;
      for (auto& storage : out.system.dc.storage) {
        if (storage.index == scenario.component_index) {
          storage.in_service = false;
          out.applied = true;
          break;
        }
      }
      if (!out.applied) mark_missing("DC storage " + std::to_string(scenario.component_index));
      break;
  }

  return out;
}

GridLABDValidationMatrixReport run_gridlabd_validation_matrix(
    const std::vector<GridLABDStandardCase>& cases,
    const GridLABDValidationMatrixOptions& options) {
  GridLABDValidationMatrixReport report;
  report.case_count = static_cast<int>(cases.size());
  const auto discovered = discover_gridlabd();
  report.gridlabd_available = discovered.available;
  if (!discovered.available) {
    for (const auto& diag : discovered.diagnostics) report.warnings.push_back(diag);
  }

  for (const auto& test_case : cases) {
    const auto scenarios =
        build_gridlabd_failure_scenarios(test_case.system, options);
    for (const auto& scenario : scenarios) {
      GridLABDValidationScenarioReport row;
      row.case_name = test_case.name;
      row.case_category = test_case.category;
      row.scenario = scenario;
      row.application =
          apply_gridlabd_failure_scenario(test_case.system, scenario);
      row.warnings = row.application.warnings;

      if (!row.application.applied) {
        row.passed = false;
        ++report.skipped_count;
        report.scenarios.push_back(std::move(row));
        continue;
      }
      if (!row.application.comparable_with_gridlabd) {
        row.passed = false;
        ++report.diagnostic_count;
        row.warnings.push_back(
            "Scenario is native-only for now and was not sent to GridLAB-D.");
        report.scenarios.push_back(std::move(row));
        continue;
      }

      auto comparison_options = options.comparison_options;
      comparison_options.require_gridlabd = options.require_gridlabd;
      comparison_options.export_options.model_name = sanitize_model_name(
          "gld_val_" + test_case.name + "_" + scenario.name);

      row.comparison =
          compare_gridlabd_snapshot(row.application.system, comparison_options);
      row.gridlabd_attempted = row.comparison.gridlabd_run_attempted;
      row.gridlabd_solved = row.comparison.gridlabd_run_success;
      row.exact_gate_item =
          row.application.exact_ac_snapshot &&
          row.comparison.unsupported_features.empty() &&
          row.comparison.diagnostic_only_reasons.empty();
      row.passed = row.exact_gate_item ? row.comparison.passed
                                       : row.comparison.gridlabd_run_success;
      if (row.exact_gate_item) {
        ++report.exact_gate_count;
        if (row.passed) ++report.exact_gate_passed_count;
      } else {
        ++report.diagnostic_count;
      }
      if (!row.gridlabd_attempted) ++report.skipped_count;

      report.scenarios.push_back(std::move(row));
    }
  }

  report.scenario_count = static_cast<int>(report.scenarios.size());
  report.exact_gate_passed =
      report.exact_gate_count > 0 &&
      report.exact_gate_passed_count == report.exact_gate_count;
  if (!report.gridlabd_available && options.require_gridlabd) {
    report.exact_gate_passed = false;
  }
  return report;
}

GridLABDValidationMatrixReport run_gridlabd_standard_validation_matrix(
    const GridLABDValidationMatrixOptions& options) {
  return run_gridlabd_validation_matrix(build_gridlabd_standard_cases(options),
                                        options);
}

std::string gridlabd_validation_matrix_to_json(
    const GridLABDValidationMatrixReport& report,
    int indent) {
  nlohmann::json root;
  root["gridlabd_available"] = report.gridlabd_available;
  root["exact_gate_passed"] = report.exact_gate_passed;
  root["case_count"] = report.case_count;
  root["scenario_count"] = report.scenario_count;
  root["exact_gate_count"] = report.exact_gate_count;
  root["exact_gate_passed_count"] = report.exact_gate_passed_count;
  root["diagnostic_count"] = report.diagnostic_count;
  root["skipped_count"] = report.skipped_count;
  root["warnings"] = report.warnings;
  root["scenarios"] = nlohmann::json::array();
  for (const auto& row : report.scenarios) {
    nlohmann::json j;
    j["case_name"] = row.case_name;
    j["case_category"] = row.case_category;
    j["scenario"] = scenario_to_json(row.scenario);
    j["applied"] = row.application.applied;
    j["comparable_with_gridlabd"] = row.application.comparable_with_gridlabd;
    j["exact_ac_snapshot"] = row.application.exact_ac_snapshot;
    j["gridlabd_attempted"] = row.gridlabd_attempted;
    j["gridlabd_solved"] = row.gridlabd_solved;
    j["exact_gate_item"] = row.exact_gate_item;
    j["passed"] = row.passed;
    j["warnings"] = row.warnings;
    j["comparison"] = comparison_summary_to_json(row.comparison);
    root["scenarios"].push_back(std::move(j));
  }
  return indent >= 0 ? root.dump(indent) : root.dump();
}

std::vector<GridLABDLongDurationEvent> build_gridlabd_long_duration_sequence(
    const HybridPowerSystem& sys,
    const GridLABDLongDurationOptions& options) {
  (void)options;
  std::vector<GridLABDLongDurationEvent> events;
  const auto load_buses = ranked_load_buses(sys);
  const std::optional<int> target_bus =
      !load_buses.empty() ? std::optional<int>(load_buses.front())
                          : first_non_slack_bus(sys);

  if (target_bus.has_value()) {
    GridLABDFailureScenario load;
    load.kind = GridLABDFailureKind::ACLoadScale;
    load.bus = *target_bus;
    load.component_index = *target_bus;
    load.scale = 1.15;
    load.value = 1.15;
    load.name = "long_scale_ac_load_bus_" + std::to_string(*target_bus);
    load.expected_exact_ac_snapshot = true;
    load.scope = "Long-duration temporary AC load disturbance";
    events.push_back(GridLABDLongDurationEvent{
        .time_s = 1.0,
        .duration_s = 2.0,
        .scenario = std::move(load),
        .label = "temporary AC load increase",
    });

    GridLABDFailureScenario fault;
    fault.kind = GridLABDFailureKind::FaultShunt;
    fault.bus = *target_bus;
    fault.component_index = *target_bus;
    fault.r_pu = 0.03;
    fault.x_pu = 0.03;
    fault.name = "long_fault_shunt_bus_" + std::to_string(*target_bus);
    fault.expected_exact_ac_snapshot = true;
    fault.scope = "Long-duration sequence temporary AC shunt fault";
    events.push_back(GridLABDLongDurationEvent{
        .time_s = 4.0,
        .duration_s = 0.08,
        .scenario = std::move(fault),
        .label = "temporary AC shunt fault",
    });
  }

  for (const auto& branch : sys.ac.branches) {
    if (!branch.in_service) continue;
    if (!preserves_slack_connectivity(sys, branch.index)) continue;
    GridLABDFailureScenario trip;
    trip.kind = GridLABDFailureKind::ACBranchTrip;
    trip.component_index = branch.index;
    trip.name = "long_trip_ac_branch_" + std::to_string(branch.index);
    trip.expected_exact_ac_snapshot = true;
    trip.scope = "Long-duration temporary AC branch outage";
    events.push_back(GridLABDLongDurationEvent{
        .time_s = 6.0,
        .duration_s = 1.0,
        .scenario = std::move(trip),
        .label = "temporary AC branch outage",
    });
    break;
  }

  return events;
}

GridLABDLongDurationReport run_gridlabd_long_duration_validation(
    const GridLABDStandardCase& test_case,
    const std::vector<GridLABDLongDurationEvent>& events,
    const GridLABDLongDurationOptions& options) {
  GridLABDLongDurationReport report;
  const auto discovered = discover_gridlabd();
  report.gridlabd_available = discovered.available;
  if (!discovered.available) {
    for (const auto& diag : discovered.diagnostics) report.warnings.push_back(diag);
  }

  auto dyn_options = options.dynamic_options;
  dyn_options.t_start_s = 0.0;
  dyn_options.t_end_s = options.t_end_s;
  dyn_options.use_adaptive_step = true;
  dyn_options.record_every_step = true;
  dyn_options.record_initial_state = true;
  dyn_options.max_recorded_snapshots = 0;
  dyn_options.run_power_flow_initialization = true;
  dyn_options.trim_dynamic_initial_conditions = true;

  dynamics::DynamicModelBuilder builder;
  dynamics::DynamicSystem dynamic_system =
      builder.build(test_case.system, dyn_options);
  for (const auto& event : events) {
    if (const auto de = dynamic_event_from_long_event(event, false)) {
      dynamic_system.events.push_back(*de);
    }
    const bool restore =
        event.duration_s > 0.0 &&
        (event.scenario.kind == GridLABDFailureKind::ACBranchTrip ||
         event.scenario.kind == GridLABDFailureKind::ACBranchClose ||
         event.scenario.kind == GridLABDFailureKind::ACLoadScale ||
         event.scenario.kind == GridLABDFailureKind::DCBranchTrip ||
         event.scenario.kind == GridLABDFailureKind::DCBranchClose ||
         event.scenario.kind == GridLABDFailureKind::DCLoadScale);
    if (restore) {
      if (const auto de = dynamic_event_from_long_event(event, true)) {
        dynamic_system.events.push_back(*de);
      }
    }
  }

  dynamics::DynamicSolver solver;
  const auto dynamic_start = std::chrono::steady_clock::now();
  const dynamics::DynamicResults dynamic_results = solver.solve(dynamic_system);
  report.dynamic_elapsed_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - dynamic_start).count();
  report.dynamic_success = dynamic_results.success;
  report.dynamic_message = dynamic_results.message;
  report.dynamic_steps = dynamic_results.steps;
  report.dynamic_rejected_steps = dynamic_results.rejected_steps;
  report.dynamic_max_local_error_norm = dynamic_results.max_local_error_norm;
  report.dynamic_snapshot_count = static_cast<int>(dynamic_results.snapshots.size());
  report.dynamic_applied_event_count =
      static_cast<int>(dynamic_results.applied_event_records.size());
  report.dynamic_min_ac_voltage_pu = std::numeric_limits<double>::infinity();
  report.dynamic_max_ac_voltage_pu = -std::numeric_limits<double>::infinity();
  report.dynamic_min_dc_voltage_pu = std::numeric_limits<double>::infinity();
  report.dynamic_max_dc_voltage_pu = -std::numeric_limits<double>::infinity();
  report.dynamic_frequency_nadir_hz = std::numeric_limits<double>::infinity();
  report.dynamic_frequency_zenith_hz = -std::numeric_limits<double>::infinity();
  report.dynamic_bus_frequency_nadir_hz = std::numeric_limits<double>::infinity();
  report.dynamic_bus_frequency_zenith_hz = -std::numeric_limits<double>::infinity();
  const dynamics::DynamicSnapshot* previous_snapshot = nullptr;
  for (const auto& snapshot : dynamic_results.snapshots) {
    report.dynamic_min_ac_voltage_pu =
        std::min(report.dynamic_min_ac_voltage_pu, snapshot.min_ac_voltage_pu);
    report.dynamic_max_ac_voltage_pu =
        std::max(report.dynamic_max_ac_voltage_pu, snapshot.max_ac_voltage_pu);
    if (snapshot.vdc.size() > 0) {
      report.dynamic_min_dc_voltage_pu =
          std::min(report.dynamic_min_dc_voltage_pu, snapshot.min_dc_voltage_pu);
      report.dynamic_max_dc_voltage_pu =
          std::max(report.dynamic_max_dc_voltage_pu, snapshot.max_dc_voltage_pu);
    }
    report.dynamic_frequency_nadir_hz =
        std::min(report.dynamic_frequency_nadir_hz, snapshot.frequency_hz);
    report.dynamic_frequency_zenith_hz =
        std::max(report.dynamic_frequency_zenith_hz, snapshot.frequency_hz);
    if (!snapshot.bus_frequency_hz.empty()) {
      const auto [fmin, fmax] = std::minmax_element(
          snapshot.bus_frequency_hz.begin(), snapshot.bus_frequency_hz.end());
      report.dynamic_max_bus_frequency_spread_hz = std::max(
          report.dynamic_max_bus_frequency_spread_hz, *fmax - *fmin);
      report.dynamic_bus_frequency_nadir_hz =
          std::min(report.dynamic_bus_frequency_nadir_hz, *fmin);
      report.dynamic_bus_frequency_zenith_hz =
          std::max(report.dynamic_bus_frequency_zenith_hz, *fmax);
    }
    if (previous_snapshot != nullptr) {
      const double dt = snapshot.time_s - previous_snapshot->time_s;
      if (dt > 1e-12) {
        report.dynamic_max_abs_rocof_hz_per_s = std::max(
            report.dynamic_max_abs_rocof_hz_per_s,
            std::abs(snapshot.frequency_hz - previous_snapshot->frequency_hz) / dt);
      }
    }
    previous_snapshot = &snapshot;
  }
  if (!std::isfinite(report.dynamic_min_ac_voltage_pu)) report.dynamic_min_ac_voltage_pu = 0.0;
  if (!std::isfinite(report.dynamic_max_ac_voltage_pu)) report.dynamic_max_ac_voltage_pu = 0.0;
  if (!std::isfinite(report.dynamic_min_dc_voltage_pu)) report.dynamic_min_dc_voltage_pu = 0.0;
  if (!std::isfinite(report.dynamic_max_dc_voltage_pu)) report.dynamic_max_dc_voltage_pu = 0.0;
  if (!std::isfinite(report.dynamic_frequency_nadir_hz)) report.dynamic_frequency_nadir_hz = 0.0;
  if (!std::isfinite(report.dynamic_frequency_zenith_hz)) report.dynamic_frequency_zenith_hz = 0.0;
  if (!std::isfinite(report.dynamic_bus_frequency_nadir_hz))
    report.dynamic_bus_frequency_nadir_hz = 0.0;
  if (!std::isfinite(report.dynamic_bus_frequency_zenith_hz))
    report.dynamic_bus_frequency_zenith_hz = 0.0;
  if (const auto* final = dynamic_results.final_snapshot()) {
    report.dynamic_final_frequency_hz = final->frequency_hz;
    report.dynamic_final_min_ac_voltage_pu = final->min_ac_voltage_pu;
    report.dynamic_final_min_dc_voltage_pu =
        final->vdc.size() > 0 ? final->min_dc_voltage_pu : 0.0;
  }
  report.warnings.insert(report.warnings.end(),
                         dynamic_results.warnings.begin(),
                         dynamic_results.warnings.end());
  report.warnings.insert(report.warnings.end(),
                         dynamic_results.initialization.warnings.begin(),
                         dynamic_results.initialization.warnings.end());

  std::map<std::string, GridLABDComparisonReport> stage_cache;
  const auto sample_times = long_duration_sample_times(events, options);
  for (const double t : sample_times) {
    GridLABDLongDurationSampleReport sample;
    sample.time_s = t;
    bool active_fault = false;
    sample.application = materialize_long_duration_state(
        test_case.system, events, t, sample.active_events, active_fault);
    sample.stage = long_duration_stage(active_fault, sample.active_events);
    sample.warnings = sample.application.warnings;

    const dynamics::DynamicSnapshot* snapshot =
        dynamic_snapshot_not_after(dynamic_results, t);
    sample.dynamic_snapshot_found = snapshot != nullptr;
    if (snapshot != nullptr) {
      sample.dynamic_min_ac_voltage_pu = snapshot->min_ac_voltage_pu;
      sample.dynamic_max_ac_voltage_pu = snapshot->max_ac_voltage_pu;
      sample.dynamic_min_dc_voltage_pu =
          snapshot->vdc.size() > 0 ? snapshot->min_dc_voltage_pu : 0.0;
      sample.dynamic_max_dc_voltage_pu =
          snapshot->vdc.size() > 0 ? snapshot->max_dc_voltage_pu : 0.0;
      sample.dynamic_frequency_hz = snapshot->frequency_hz;
      sample.dynamic_coi_frequency_hz = snapshot->coi_frequency_hz;
      if (!snapshot->bus_frequency_hz.empty()) {
        const auto [fmin, fmax] = std::minmax_element(
            snapshot->bus_frequency_hz.begin(), snapshot->bus_frequency_hz.end());
        sample.dynamic_min_bus_frequency_hz = *fmin;
        sample.dynamic_max_bus_frequency_hz = *fmax;
      }
    } else {
      sample.warnings.push_back("No dynamic snapshot was recorded near sample time.");
    }

    const bool skip_active_fault =
        active_fault && !options.compare_active_fault_samples;
    if (!sample.application.applied) {
      sample.passed = false;
      ++report.skipped_count;
      report.samples.push_back(std::move(sample));
      continue;
    }
    if (skip_active_fault) {
      sample.passed = sample.dynamic_snapshot_found;
      sample.warnings.push_back(
          "Active fault sample recorded as dynamic diagnostic only; "
          "GridLAB-D exact snapshot comparison is disabled for fault-on time.");
      ++report.diagnostic_count;
      report.samples.push_back(std::move(sample));
      continue;
    }
    if (!sample.application.comparable_with_gridlabd) {
      sample.passed = sample.dynamic_snapshot_found;
      sample.warnings.push_back(
          "Sample contains native AC/DC state outside the current GridLAB-D AC scope.");
      ++report.diagnostic_count;
      report.samples.push_back(std::move(sample));
      continue;
    }

    auto comparison_options = options.comparison_options;
    comparison_options.require_gridlabd = options.require_gridlabd;
    comparison_options.export_options.model_name = sanitize_model_name(
        "gld_long_" + test_case.name + "_" + std::to_string(report.samples.size()));
    std::string cache_key = sample.active_events.empty() ? "base" : "active";
    for (const auto& label : sample.active_events) cache_key += "|" + label;
    const auto cached = stage_cache.find(cache_key);
    if (cached != stage_cache.end()) {
      sample.comparison = cached->second;
      ++report.gridlabd_stage_cache_hits;
    } else {
      sample.comparison =
          compare_gridlabd_snapshot(sample.application.system, comparison_options);
      stage_cache.emplace(cache_key, sample.comparison);
      ++report.gridlabd_unique_stage_runs;
    }
    sample.gridlabd_attempted = sample.comparison.gridlabd_run_attempted;
    sample.gridlabd_solved = sample.comparison.gridlabd_run_success;

    bool dynamic_voltage_passed = sample.dynamic_snapshot_found;
    if (snapshot != nullptr && sample.comparison.gridlabd_run_success) {
      for (const auto& gld_v : sample.comparison.gridlabd_result.bus_voltages) {
        const int pos =
            ac_bus_position(dynamic_system.network.ac_bus_ids,
                            gld_v.canonical_bus_index);
        if (pos < 0 || 3 * pos >= snapshot->vac_abc.size()) {
          sample.warnings.push_back(
              "Unable to map dynamic bus voltage for bus " +
              std::to_string(gld_v.canonical_bus_index));
          dynamic_voltage_passed = false;
          continue;
        }
        const double dyn_vm = std::abs(snapshot->vac_abc[3 * pos]);
        auto item = make_comparison_item(
            "dynamic_bus_vm_pu",
            gld_v.gridlabd_name,
            dyn_vm,
            gld_v.vm_a_pu,
            options.dynamic_vm_tolerance_pu,
            "HACDCPF transient A-phase voltage vs GridLAB-D sampled AC snapshot");
        dynamic_voltage_passed = dynamic_voltage_passed && item.passed;
        report.max_dynamic_voltage_error_pu = std::max(
            report.max_dynamic_voltage_error_pu, std::abs(item.difference));
        sample.dynamic_voltage_items.push_back(std::move(item));
      }
    }
    for (const auto& item : sample.comparison.items) {
      const double error = std::abs(item.difference);
      if (item.kind == "bus_vm_pu")
        report.max_snapshot_vm_error_pu = std::max(report.max_snapshot_vm_error_pu, error);
      else if (item.kind == "bus_va_deg")
        report.max_snapshot_va_error_deg = std::max(report.max_snapshot_va_error_deg, error);
      else if (item.kind == "branch_pf_mw" || item.kind == "branch_loss_p_mw")
        report.max_snapshot_branch_p_error_mw = std::max(report.max_snapshot_branch_p_error_mw, error);
      else if (item.kind == "branch_qf_mvar" || item.kind == "branch_loss_q_mvar")
        report.max_snapshot_branch_q_error_mvar = std::max(report.max_snapshot_branch_q_error_mvar, error);
    }

    sample.exact_gate_item =
        report.dynamic_success &&
        sample.application.exact_ac_snapshot &&
        sample.comparison.unsupported_features.empty() &&
        sample.comparison.diagnostic_only_reasons.empty();
    sample.passed =
        sample.exact_gate_item
            ? (sample.comparison.passed && dynamic_voltage_passed)
            : (sample.comparison.gridlabd_run_success && sample.dynamic_snapshot_found);
    if (sample.exact_gate_item) {
      ++report.exact_gate_count;
      if (sample.passed) ++report.exact_gate_passed_count;
    } else {
      ++report.diagnostic_count;
    }
    if (!sample.gridlabd_attempted) ++report.skipped_count;

    report.samples.push_back(std::move(sample));
  }

  report.sample_count = static_cast<int>(report.samples.size());
  report.exact_gate_passed =
      report.dynamic_success &&
      report.exact_gate_count > 0 &&
      report.exact_gate_passed_count == report.exact_gate_count;
  if (!report.gridlabd_available && options.require_gridlabd) {
    report.exact_gate_passed = false;
  }
  return report;
}

std::string gridlabd_long_duration_report_to_json(
    const GridLABDLongDurationReport& report,
    int indent) {
  nlohmann::json root;
  root["gridlabd_available"] = report.gridlabd_available;
  root["dynamic_success"] = report.dynamic_success;
  root["dynamic_message"] = report.dynamic_message;
  root["dynamic_steps"] = report.dynamic_steps;
  root["dynamic_rejected_steps"] = report.dynamic_rejected_steps;
  root["dynamic_max_local_error_norm"] = report.dynamic_max_local_error_norm;
  root["dynamic_elapsed_ms"] = report.dynamic_elapsed_ms;
  root["dynamic_snapshot_count"] = report.dynamic_snapshot_count;
  root["dynamic_applied_event_count"] = report.dynamic_applied_event_count;
  root["dynamic_min_ac_voltage_pu"] = report.dynamic_min_ac_voltage_pu;
  root["dynamic_max_ac_voltage_pu"] = report.dynamic_max_ac_voltage_pu;
  root["dynamic_min_dc_voltage_pu"] = report.dynamic_min_dc_voltage_pu;
  root["dynamic_max_dc_voltage_pu"] = report.dynamic_max_dc_voltage_pu;
  root["dynamic_frequency_nadir_hz"] = report.dynamic_frequency_nadir_hz;
  root["dynamic_frequency_zenith_hz"] = report.dynamic_frequency_zenith_hz;
  root["dynamic_bus_frequency_nadir_hz"] = report.dynamic_bus_frequency_nadir_hz;
  root["dynamic_bus_frequency_zenith_hz"] = report.dynamic_bus_frequency_zenith_hz;
  root["dynamic_max_abs_rocof_hz_per_s"] = report.dynamic_max_abs_rocof_hz_per_s;
  root["dynamic_max_bus_frequency_spread_hz"] = report.dynamic_max_bus_frequency_spread_hz;
  root["dynamic_final_frequency_hz"] = report.dynamic_final_frequency_hz;
  root["dynamic_final_min_ac_voltage_pu"] = report.dynamic_final_min_ac_voltage_pu;
  root["dynamic_final_min_dc_voltage_pu"] = report.dynamic_final_min_dc_voltage_pu;
  root["gridlabd_unique_stage_runs"] = report.gridlabd_unique_stage_runs;
  root["gridlabd_stage_cache_hits"] = report.gridlabd_stage_cache_hits;
  root["max_dynamic_voltage_error_pu"] = report.max_dynamic_voltage_error_pu;
  root["max_snapshot_vm_error_pu"] = report.max_snapshot_vm_error_pu;
  root["max_snapshot_va_error_deg"] = report.max_snapshot_va_error_deg;
  root["max_snapshot_branch_p_error_mw"] = report.max_snapshot_branch_p_error_mw;
  root["max_snapshot_branch_q_error_mvar"] = report.max_snapshot_branch_q_error_mvar;
  root["exact_gate_passed"] = report.exact_gate_passed;
  root["sample_count"] = report.sample_count;
  root["exact_gate_count"] = report.exact_gate_count;
  root["exact_gate_passed_count"] = report.exact_gate_passed_count;
  root["diagnostic_count"] = report.diagnostic_count;
  root["skipped_count"] = report.skipped_count;
  root["warnings"] = report.warnings;
  root["samples"] = nlohmann::json::array();
  for (const auto& sample : report.samples) {
    nlohmann::json items = nlohmann::json::array();
    for (const auto& item : sample.dynamic_voltage_items) {
      items.push_back({
          {"kind", item.kind},
          {"key", item.key},
          {"hacdcpf_value", item.hacdcpf_value},
          {"gridlabd_value", item.gridlabd_value},
          {"difference", item.difference},
          {"tolerance", item.tolerance},
          {"passed", item.passed},
          {"detail", item.detail},
      });
    }
    nlohmann::json j;
    j["time_s"] = sample.time_s;
    j["stage"] = sample.stage;
    j["active_events"] = sample.active_events;
    j["dynamic_snapshot_found"] = sample.dynamic_snapshot_found;
    j["dynamic_min_ac_voltage_pu"] = sample.dynamic_min_ac_voltage_pu;
    j["dynamic_max_ac_voltage_pu"] = sample.dynamic_max_ac_voltage_pu;
    j["dynamic_min_dc_voltage_pu"] = sample.dynamic_min_dc_voltage_pu;
    j["dynamic_max_dc_voltage_pu"] = sample.dynamic_max_dc_voltage_pu;
    j["dynamic_frequency_hz"] = sample.dynamic_frequency_hz;
    j["dynamic_coi_frequency_hz"] = sample.dynamic_coi_frequency_hz;
    j["dynamic_min_bus_frequency_hz"] = sample.dynamic_min_bus_frequency_hz;
    j["dynamic_max_bus_frequency_hz"] = sample.dynamic_max_bus_frequency_hz;
    j["applied"] = sample.application.applied;
    j["comparable_with_gridlabd"] = sample.application.comparable_with_gridlabd;
    j["exact_ac_snapshot"] = sample.application.exact_ac_snapshot;
    j["gridlabd_attempted"] = sample.gridlabd_attempted;
    j["gridlabd_solved"] = sample.gridlabd_solved;
    j["exact_gate_item"] = sample.exact_gate_item;
    j["passed"] = sample.passed;
    j["warnings"] = sample.warnings;
    j["comparison"] = comparison_summary_to_json(sample.comparison);
    j["dynamic_voltage_items"] = std::move(items);
    root["samples"].push_back(std::move(j));
  }
  return indent >= 0 ? root.dump(indent) : root.dump();
}

}  // namespace hacdcpf::io
