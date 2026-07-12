#include <algorithm>
#include <chrono>
#include <cmath>
#include <complex>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "hacdcpf/dynamics/dynamics.hpp"
#include "hacdcpf/model/hybrid_power_system.hpp"

using json = nlohmann::json;
using namespace hacdcpf;
using namespace hacdcpf::dynamics;

namespace {

ACBus ac_bus(int id, BusType type) {
  ACBus b;
  b.index = id;
  b.bus_type = type;
  b.base_kv = 13.8;
  b.vm_pu = 1.0;
  b.in_service = true;
  return b;
}

ACBranch ac_branch(int id, int from, int to, double r, double x) {
  ACBranch b;
  b.index = id;
  b.from_bus = from;
  b.to_bus = to;
  b.r_pu = r;
  b.x_pu = x;
  b.tap = 1.0;
  b.in_service = true;
  return b;
}

DCBus dc_bus(int id, DCBusType type) {
  DCBus b;
  b.index = id;
  b.bus_type = type;
  b.base_kv = 1.0;
  b.vm_pu = 1.0;
  b.in_service = true;
  return b;
}

DCBranch dc_branch(int id, int from, int to, double r) {
  DCBranch b;
  b.index = id;
  b.from_bus = from;
  b.to_bus = to;
  b.r_pu = r;
  b.in_service = true;
  return b;
}

HybridPowerSystem benchmark_system(bool unbalanced = false) {
  HybridPowerSystem sys;
  sys.name = unbalanced ? "transient_matrix_unbalanced" : "transient_matrix_hybrid";
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;
  sys.ac.freq_hz = 50.0;
  sys.dc.base_mva = 100.0;
  sys.ac.buses = {ac_bus(1, BusType::SLACK), ac_bus(2, BusType::PQ),
                  ac_bus(3, BusType::PV)};
  sys.ac.branches = {ac_branch(1, 1, 2, 0.01, 0.08),
                     ac_branch(2, 2, 3, 0.012, 0.09),
                     ac_branch(3, 1, 3, 0.015, 0.10)};

  Generator slack;
  slack.index = 1;
  slack.bus = 1;
  slack.is_slack = true;
  slack.in_service = true;
  slack.vg_pu = 1.0;
  slack.pg_mw = 30.0;
  slack.xdpp_pu = 0.2;
  slack.inertia_h = 5.0;
  Generator gen = slack;
  gen.index = 2;
  gen.bus = 3;
  gen.is_slack = false;
  gen.pg_mw = 10.0;
  gen.inertia_h = 3.5;
  sys.ac.generators = {slack, gen};

  Load load;
  load.index = 1;
  load.bus = 2;
  load.p_mw = 25.0;
  load.q_mvar = 6.0;
  load.scaling = 1.0;
  load.in_service = true;
  sys.ac.loads = {load};
  if (unbalanced) {
    sys.ac.loads.clear();
    AsymmetricLoad asym;
    asym.index = 1;
    asym.bus = 2;
    asym.pa_mw = 14.0;
    asym.qa_mvar = 3.0;
    asym.pb_mw = 7.0;
    asym.qb_mvar = 2.0;
    asym.pc_mw = 4.0;
    asym.qc_mvar = 1.0;
    asym.scaling = 1.0;
    asym.in_service = true;
    sys.ac.asymmetric_loads = {asym};
  }

  Storage storage;
  storage.index = 1;
  storage.bus = 3;
  storage.p_mw = 0.5;
  storage.p_rated_mw = 5.0;
  storage.e_rated_mwh = 10.0;
  storage.e_mwh = 5.0;
  storage.in_service = true;
  sys.ac.storage = {storage};

  sys.dc.buses = {dc_bus(10, DCBusType::DC_V), dc_bus(11, DCBusType::DC_P),
                  dc_bus(12, DCBusType::DC_P)};
  sys.dc.branches = {dc_branch(1, 10, 11, 0.02),
                     dc_branch(2, 11, 12, 0.025),
                     dc_branch(3, 10, 12, 0.04)};
  DCLoad dc_load;
  dc_load.index = 1;
  dc_load.bus = 12;
  dc_load.p_mw = 3.0;
  dc_load.scaling = 1.0;
  dc_load.in_service = true;
  sys.dc.loads = {dc_load};

  VSCConverter vsc;
  vsc.index = 1;
  vsc.bus_ac = 2;
  vsc.bus_dc = 10;
  vsc.control_mode = ConverterMode::PQ_MODE;
  vsc.p_set_mw = 5.0;
  vsc.p_schedule_mw = 5.0;
  vsc.q_set_mvar = 0.5;
  vsc.eta = 0.98;
  vsc.in_service = true;
  sys.vsc_converters = {vsc};

  DCDCConverter dcdc;
  dcdc.index = 1;
  dcdc.bus_in = 10;
  dcdc.bus_out = 11;
  dcdc.p_ref_mw = 1.5;
  dcdc.eta = 0.97;
  dcdc.in_service = true;
  sys.dc.dcdc_converters = {dcdc};

  DCStorage dc_storage;
  dc_storage.index = 1;
  dc_storage.bus = 11;
  dc_storage.p_mw = 0.3;
  dc_storage.p_rated_mw = 3.0;
  dc_storage.e_rated_mwh = 6.0;
  dc_storage.e_mwh = 3.0;
  dc_storage.in_service = true;
  sys.dc.dc_storage = {dc_storage};
  return sys;
}

DynamicEvent event(double time, DynamicEventType type, int component,
                   int bus, std::string label) {
  DynamicEvent e;
  e.time_s = time;
  e.type = type;
  e.component_index = component;
  e.bus = bus;
  e.target_id = bus;
  e.label = std::move(label);
  return e;
}

std::string type_name(DynamicEventType type) {
  switch (type) {
    case DynamicEventType::ACBranchTrip: return "ACBranchTrip";
    case DynamicEventType::ACBranchClose: return "ACBranchClose";
    case DynamicEventType::ACBranchImpedanceScale: return "ACBranchImpedanceScale";
    case DynamicEventType::DCBranchTrip: return "DCBranchTrip";
    case DynamicEventType::DCBranchClose: return "DCBranchClose";
    case DynamicEventType::ACLoadScale: return "ACLoadScale";
    case DynamicEventType::DCLoadScale: return "DCLoadScale";
    case DynamicEventType::GeneratorTrip: return "GeneratorTrip";
    case DynamicEventType::VSCTrip: return "VSCTrip";
    case DynamicEventType::DCDCTrip: return "DCDCTrip";
    case DynamicEventType::StoragePowerStep: return "StoragePowerStep";
    case DynamicEventType::DCStoragePowerStep: return "DCStoragePowerStep";
    case DynamicEventType::FaultShunt: return "FaultShunt";
    case DynamicEventType::ClearFault: return "ClearFault";
    case DynamicEventType::Custom: return "Custom";
  }
  return "Unknown";
}

struct Case {
  std::string id;
  std::string domain;
  std::string balance{"balanced"};
  std::vector<DynamicEvent> events;
};

std::vector<Case> cases() {
  auto load = event(0.10, DynamicEventType::ACLoadScale, 1, 2, "AC load +20%");
  load.value = 1.2;
  load.params["scale"] = 1.2;
  auto dc_load = event(0.10, DynamicEventType::DCLoadScale, 1, 12, "DC load +20%");
  dc_load.value = 1.2;
  dc_load.params["scale"] = 1.2;
  auto imp = event(0.10, DynamicEventType::ACBranchImpedanceScale, 1, 0,
                   "AC branch impedance x1.5");
  imp.value = 1.5;
  imp.params["scale"] = 1.5;
  auto ac_fault = event(0.10, DynamicEventType::FaultShunt, 2, 2, "three-phase AC fault");
  ac_fault.component_type = "AC";
  ac_fault.duration_s = 0.04;
  ac_fault.params = {{"r_pu", 0.03}, {"x_pu", 0.03}, {"duration_s", 0.04}};
  auto slg = ac_fault;
  slg.label = "A-phase-to-ground fault";
  slg.phase = 0;
  auto dc_fault = event(0.10, DynamicEventType::FaultShunt, 12, 12, "DC pole fault");
  dc_fault.component_type = "DC";
  dc_fault.duration_s = 0.04;
  dc_fault.params = {{"r_pu", 0.05}, {"duration_s", 0.04}};
  auto storage = event(0.10, DynamicEventType::StoragePowerStep, 1, 3, "AC storage step");
  storage.value = -1.0;
  storage.params["p_ref_mw"] = -1.0;
  auto dc_storage = event(0.10, DynamicEventType::DCStoragePowerStep, 1, 11, "DC storage step");
  dc_storage.value = -0.5;
  dc_storage.params["p_ref_mw"] = -0.5;
  auto ac_trip = event(0.10, DynamicEventType::ACBranchTrip, 2, 0, "AC branch trip");
  auto ac_close = event(0.16, DynamicEventType::ACBranchClose, 2, 0, "AC branch close");
  auto dc_trip = event(0.10, DynamicEventType::DCBranchTrip, 2, 0, "DC branch trip");
  auto dc_close = event(0.16, DynamicEventType::DCBranchClose, 2, 0, "DC branch close");
  auto gen_trip = event(0.10, DynamicEventType::GeneratorTrip, 2, 3, "generator trip");
  auto vsc_trip = event(0.10, DynamicEventType::VSCTrip, 1, 2, "VSC trip");
  auto dcdc_trip = event(0.10, DynamicEventType::DCDCTrip, 1, 11, "DC/DC trip");
  auto clear = event(0.14, DynamicEventType::ClearFault, 2, 2, "explicit fault clear");
  auto indefinite_fault = ac_fault;
  indefinite_fault.duration_s = 0.0;
  indefinite_fault.params.erase("duration_s");

  std::vector<Case> out = {
      {"no_event_hybrid", "hybrid_acdc", "balanced", {}},
      {"ac_load_step", "ac", "balanced", {load}},
      {"ac_branch_trip_close", "ac", "balanced", {ac_trip, ac_close}},
      {"ac_branch_impedance_step", "ac", "balanced", {imp}},
      {"ac_three_phase_fault", "ac", "balanced", {ac_fault}},
      {"ac_slg_fault_unbalanced", "ac", "unbalanced", {slg}},
      {"ac_explicit_fault_clear", "ac", "balanced", {indefinite_fault, clear}},
      {"generator_trip", "ac", "balanced", {gen_trip}},
      {"dc_load_step", "dc", "n/a", {dc_load}},
      {"dc_branch_trip_close", "dc", "n/a", {dc_trip, dc_close}},
      {"dc_pole_fault", "dc", "n/a", {dc_fault}},
      {"vsc_trip", "hybrid_acdc", "balanced", {vsc_trip}},
      {"dcdc_trip", "hybrid_acdc", "balanced", {dcdc_trip}},
      {"ac_storage_power_step", "ac", "balanced", {storage}},
      {"dc_storage_power_step", "dc", "n/a", {dc_storage}},
      {"sequential_ac_combination", "ac", "balanced", {}},
      {"sequential_hybrid_combination", "hybrid_acdc", "balanced", {}},
      {"unbalanced_sequential_combination", "hybrid_acdc", "unbalanced", {}},
  };
  out[15].events = {load, ac_fault, ac_trip, ac_close, storage};
  for (std::size_t i = 0; i < out[15].events.size(); ++i)
    out[15].events[i].time_s = 0.04 + 0.04 * static_cast<double>(i);
  out[16].events = {load, dc_load, ac_fault, dc_trip, dc_close, vsc_trip,
                    dcdc_trip, storage, dc_storage};
  for (std::size_t i = 0; i < out[16].events.size(); ++i)
    out[16].events[i].time_s = 0.025 + 0.025 * static_cast<double>(i);
  out[17].events = {load, slg, dc_load, storage, dc_storage};
  for (std::size_t i = 0; i < out[17].events.size(); ++i)
    out[17].events[i].time_s = 0.04 + 0.04 * static_cast<double>(i);
  return out;
}

json run_case(const Case& c) {
  DynamicSolverOptions opt;
  opt.t_end_s = 0.30;
  opt.dt_s = 0.002;
  opt.use_adaptive_step = true;
  opt.abs_tol = 1e-7;
  opt.rel_tol = 1e-5;
  opt.max_step_halving = 12;
  opt.run_power_flow_initialization = false;
  opt.trim_dynamic_initial_conditions = false;
  opt.record_every_step = true;
  opt.record_initial_state = true;
  opt.enforce_voltage_health_check = false;

  DynamicModelBuilder builder;
  DynamicSystem dynamic_system = builder.build(
      benchmark_system(c.balance == "unbalanced"), opt);
  dynamic_system.events = c.events;
  const auto start = std::chrono::steady_clock::now();
  DynamicSolver solver;
  const DynamicResults result = solver.solve(dynamic_system);
  const double elapsed_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - start).count();

  double min_ac = std::numeric_limits<double>::infinity();
  double max_ac = 0.0;
  double min_dc = std::numeric_limits<double>::infinity();
  double max_dc = 0.0;
  double f_nadir = std::numeric_limits<double>::infinity();
  double f_zenith = -std::numeric_limits<double>::infinity();
  double max_rocof = 0.0;
  double max_bus_spread = 0.0;
  double bus_f_nadir = std::numeric_limits<double>::infinity();
  double bus_f_zenith = -std::numeric_limits<double>::infinity();
  double max_phase_unbalance = 0.0;
  double ac_voltage_deviation_integral_pu_s = 0.0;
  double dc_voltage_deviation_integral_pu_s = 0.0;
  double frequency_deviation_integral_hz_s = 0.0;
  double ac_time_below_0_9_s = 0.0;
  double dc_time_below_0_9_s = 0.0;
  const DynamicSnapshot* previous = nullptr;
  for (const auto& s : result.snapshots) {
    min_ac = std::min(min_ac, s.min_ac_voltage_pu);
    max_ac = std::max(max_ac, s.max_ac_voltage_pu);
    if (s.vdc.size() > 0) {
      min_dc = std::min(min_dc, s.min_dc_voltage_pu);
      max_dc = std::max(max_dc, s.max_dc_voltage_pu);
    }
    f_nadir = std::min(f_nadir, s.frequency_hz);
    f_zenith = std::max(f_zenith, s.frequency_hz);
    if (!s.bus_frequency_hz.empty()) {
      const auto [lo, hi] = std::minmax_element(s.bus_frequency_hz.begin(),
                                                s.bus_frequency_hz.end());
      max_bus_spread = std::max(max_bus_spread, *hi - *lo);
      bus_f_nadir = std::min(bus_f_nadir, *lo);
      bus_f_zenith = std::max(bus_f_zenith, *hi);
    }
    for (Eigen::Index i = 0; i + 2 < s.vac_abc.size(); i += 3) {
      const double va = std::abs(s.vac_abc[i]);
      const double vb = std::abs(s.vac_abc[i + 1]);
      const double vc = std::abs(s.vac_abc[i + 2]);
      max_phase_unbalance = std::max(
          max_phase_unbalance, std::max({va, vb, vc}) - std::min({va, vb, vc}));
    }
    if (previous != nullptr && s.time_s > previous->time_s) {
      const double dt = s.time_s - previous->time_s;
      max_rocof = std::max(max_rocof,
          std::abs(s.frequency_hz - previous->frequency_hz) /
              dt);
      ac_voltage_deviation_integral_pu_s +=
          0.5 * (std::abs(1.0 - previous->min_ac_voltage_pu) +
                 std::abs(1.0 - s.min_ac_voltage_pu)) * dt;
      frequency_deviation_integral_hz_s +=
          0.5 * (std::abs(previous->frequency_hz - 50.0) +
                 std::abs(s.frequency_hz - 50.0)) * dt;
      if (previous->min_ac_voltage_pu < 0.9 || s.min_ac_voltage_pu < 0.9)
        ac_time_below_0_9_s += dt;
      if (previous->vdc.size() > 0 && s.vdc.size() > 0) {
        dc_voltage_deviation_integral_pu_s +=
            0.5 * (std::abs(1.0 - previous->min_dc_voltage_pu) +
                   std::abs(1.0 - s.min_dc_voltage_pu)) * dt;
        if (previous->min_dc_voltage_pu < 0.9 || s.min_dc_voltage_pu < 0.9)
          dc_time_below_0_9_s += dt;
      }
    }
    previous = &s;
  }
  if (!std::isfinite(min_ac)) min_ac = 0.0;
  if (!std::isfinite(min_dc)) min_dc = 0.0;
  if (!std::isfinite(f_nadir)) f_nadir = 0.0;
  if (!std::isfinite(f_zenith)) f_zenith = 0.0;
  if (!std::isfinite(bus_f_nadir)) bus_f_nadir = 0.0;
  if (!std::isfinite(bus_f_zenith)) bus_f_zenith = 0.0;

  json events_json = json::array();
  for (const auto& e : c.events) {
    events_json.push_back({{"time_s", e.time_s}, {"type", type_name(e.type)},
                           {"label", e.label}, {"component_index", e.component_index},
                           {"bus", e.bus}, {"phase", e.phase}, {"value", e.value},
                           {"duration_s", e.duration_s}, {"params", e.params}});
  }
  const auto* final = result.final_snapshot();
  double last_disturbance_time = 0.0;
  for (const auto& e : c.events)
    last_disturbance_time = std::max(last_disturbance_time, e.time_s + e.duration_s);
  double settling_time_s = -1.0;
  if (final != nullptr) {
    for (std::size_t i = 0; i < result.snapshots.size(); ++i) {
      if (result.snapshots[i].time_s + 1e-12 < last_disturbance_time) continue;
      if (final->time_s - result.snapshots[i].time_s < 0.02 - 1e-12) continue;
      bool settled = true;
      for (std::size_t j = i; j < result.snapshots.size(); ++j) {
        const auto& s = result.snapshots[j];
        if (std::abs(s.min_ac_voltage_pu - final->min_ac_voltage_pu) > 0.01 ||
            std::abs(s.frequency_hz - final->frequency_hz) > 0.01 ||
            (s.vdc.size() > 0 &&
             std::abs(s.min_dc_voltage_pu - final->min_dc_voltage_pu) > 0.01)) {
          settled = false;
          break;
        }
      }
      if (settled) {
        settling_time_s = result.snapshots[i].time_s - last_disturbance_time;
        break;
      }
    }
  }
  return {{"id", c.id}, {"domain", c.domain}, {"balance", c.balance},
          {"events", events_json}, {"success", result.success},
          {"message", result.message}, {"elapsed_ms", elapsed_ms},
          {"steps", result.steps}, {"rejected_steps", result.rejected_steps},
          {"newton_iterations", result.newton_iterations},
          {"linear_factorizations", result.linear_factorizations},
          {"snapshot_count", result.snapshots.size()},
          {"expected_event_count", c.events.size()},
          {"applied_event_count", result.applied_event_records.size()},
          {"min_ac_voltage_pu", min_ac}, {"max_ac_voltage_pu", max_ac},
          {"min_dc_voltage_pu", min_dc}, {"max_dc_voltage_pu", max_dc},
          {"frequency_nadir_hz", f_nadir}, {"frequency_zenith_hz", f_zenith},
          {"max_abs_rocof_hz_per_s", max_rocof},
          {"max_bus_frequency_spread_hz", max_bus_spread},
          {"bus_frequency_nadir_hz", bus_f_nadir},
          {"bus_frequency_zenith_hz", bus_f_zenith},
          {"max_phase_voltage_unbalance_pu", max_phase_unbalance},
          {"max_frequency_deviation_hz",
           std::max(std::abs(f_nadir - 50.0), std::abs(f_zenith - 50.0))},
          {"ac_voltage_deviation_integral_pu_s", ac_voltage_deviation_integral_pu_s},
          {"dc_voltage_deviation_integral_pu_s", dc_voltage_deviation_integral_pu_s},
          {"frequency_deviation_integral_hz_s", frequency_deviation_integral_hz_s},
          {"ac_time_below_0_9_s", ac_time_below_0_9_s},
          {"dc_time_below_0_9_s", dc_time_below_0_9_s},
          {"settling_time_s", settling_time_s},
          {"settled_by_end", settling_time_s >= 0.0},
          {"final_frequency_hz", final ? final->frequency_hz : 0.0},
          {"final_min_ac_voltage_pu", final ? final->min_ac_voltage_pu : 0.0},
          {"final_min_dc_voltage_pu", final && final->vdc.size() > 0
                                           ? final->min_dc_voltage_pu : 0.0},
          {"warnings", result.warnings}};
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "usage: " << argv[0] << " <output.json>\n";
    return 2;
  }
  try {
    json rows = json::array();
    for (const auto& c : cases()) rows.push_back(run_case(c));
    const int passed = static_cast<int>(std::count_if(
        rows.begin(), rows.end(), [](const json& row) {
          return row.value("success", false) &&
                 row.value("applied_event_count", 0) >=
                     row.value("expected_event_count", 0);
        }));
    json report = {{"schema", "hacdcpf.transient.native_disturbance_matrix.v1"},
                   {"cases", rows},
                   {"summary", {{"case_count", rows.size()}, {"passed", passed},
                                {"failed", static_cast<int>(rows.size()) - passed}}}};
    std::ofstream out(argv[1]);
    if (!out) throw std::runtime_error("failed to open output path");
    out << report.dump(2) << '\n';
    std::cout << report["summary"].dump() << '\n';
    return passed == static_cast<int>(rows.size()) ? 0 : 1;
  } catch (const std::exception& e) {
    std::cerr << "transient_validation_matrix: " << e.what() << '\n';
    return 2;
  }
}
