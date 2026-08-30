#include <algorithm>
#include <chrono>
#include <cmath>
#include <complex>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <numeric>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

#include "hacdcpf/dynamics/dynamics.hpp"
#include "hacdcpf/io/case_builders.hpp"
#include "hacdcpf/io/external_grid_io.hpp"
#include "hacdcpf/network_reconfiguration/topology_reconfiguration.hpp"

using json = nlohmann::json;
using namespace hacdcpf;
using namespace hacdcpf::dynamics;

namespace {

constexpr double kFaultTimeS = 0.20;
constexpr double kFaultDurationS = 0.12;
constexpr double kAcFaultResistancePu = 2.00;
constexpr double kDcFaultResistancePu = 0.05;
constexpr double kEndTimeS = 0.50;
constexpr double kRepairDurationH = 4.0;
constexpr double kMaximumPostEventResidual = 1e-7;
constexpr double kAggregationToleranceMwhYr = 1e-10;
constexpr double kNecessityRelativeGate = 0.005;
constexpr double kCompressionRatioGate = 0.50;
constexpr double kLeaveOneOutRelativeGate = 0.02;
constexpr double kLeaveOneOutClassAccuracyGate = 0.98;
constexpr double kSpeedupGate = 20.0;
constexpr std::size_t kMcBenchmarkSamples = 100'000;

enum class AssetKind { ACBranch, DCBranch, VSC };

struct Asset {
  AssetKind kind{AssetKind::ACBranch};
  int index{0};
  int fault_bus{0};
  double severity_weight{1.0};
  std::string label;
};

struct OperatingState {
  double ac_load_scale{1.0};
  double dc_load_scale{1.0};
  double probability{1.0};
  std::string label;
};

struct StudyCase {
  std::string key;
  std::string source;
  std::string model_scope;
  HybridPowerSystem system;
  std::vector<Asset> assets;
  std::vector<std::string> import_warnings;
  std::vector<std::string> import_skipped;
  int converted_closed_switches{0};
  int radialized_ac_ties{0};
};

struct EventSet {
  int order{0};
  std::vector<std::size_t> asset_positions;
  double frequency_per_year{0.0};
  std::string key;
};

struct TrajectoryResult {
  bool resolved{false};
  std::string message;
  double runtime_s{0.0};
  double max_post_event_residual{0.0};
  std::set<int> unavailable_vsc;
  int emitted_vsc_trip_count{0};
  std::string voltage_margin_band{"unknown"};
};

struct Sample {
  std::size_t event_index{0};
  std::size_t operating_index{0};
  double scenario_probability{0.0};
  bool resolved{false};
  std::string message;
  std::string class_label;
  double static_reward_mwh{0.0};
  double dynamic_reward_mwh{0.0};
  double trajectory_runtime_s{0.0};
  double direct_runtime_s{0.0};
  double static_restoration_runtime_s{0.0};
  double dynamic_restoration_runtime_s{0.0};
  int additional_vsc_outages{0};
  std::set<int> unavailable_vsc;
  bool loo_has_neighbor{false};
  bool loo_restoration_resolved{false};
  std::size_t loo_neighbor_operating_index{0};
  std::string loo_predicted_class_label;
  double loo_predicted_reward_mwh{0.0};
};

struct Options {
  std::filesystem::path output{"nk_acdc_reliability_report.json"};
  std::string selected_case{"all"};
  int max_order{3};
  double dt_s{0.005};
  bool smoke{false};
  bool audit_only{false};
};

std::string asset_prefix(AssetKind kind) {
  switch (kind) {
    case AssetKind::ACBranch: return "ac";
    case AssetKind::DCBranch: return "dc";
    case AssetKind::VSC: return "vsc";
  }
  throw std::logic_error("unsupported N-k asset kind");
}

graph::EdgeCategory edge_category(AssetKind kind) {
  switch (kind) {
    case AssetKind::ACBranch: return graph::EdgeCategory::AC_Line;
    case AssetKind::DCBranch: return graph::EdgeCategory::DC_Line;
    case AssetKind::VSC: return graph::EdgeCategory::VSC_Coupling;
  }
  throw std::logic_error("unsupported N-k asset kind");
}

void validate_probability_mass(const std::vector<OperatingState>& states) {
  const double mass = std::accumulate(
      states.begin(), states.end(), 0.0,
      [](double value, const OperatingState& state) {
        return value + state.probability;
      });
  if (states.empty() || std::abs(mass - 1.0) > 1e-12) {
    throw std::runtime_error("operating-state probabilities must sum to one");
  }
}

std::vector<OperatingState> operating_states(bool smoke) {
  if (smoke) {
    return {{0.85, 0.85, 0.5, "low"},
            {1.15, 1.15, 0.5, "high"}};
  }
  // Authored discrete design measure. These are not chronological or field
  // probabilities; they test conditional-state compression over declared
  // loading states, following Omri et al., IEEE TPWRS 2024,
  // DOI 10.1109/TPWRS.2024.3354299.
  return {{0.75, 0.80, 0.15, "L075-D080"},
          {0.90, 1.10, 0.20, "L090-D110"},
          {1.00, 0.90, 0.25, "L100-D090"},
          {1.00, 1.20, 0.15, "L100-D120"},
          {1.10, 1.00, 0.15, "L110-D100"},
          {1.25, 1.15, 0.10, "L125-D115"}};
}

void configure_vsc(VSCConverter& converter, int index, int ac_bus, int dc_bus,
                   double p_schedule_mw, bool vdc_control) {
  converter.index = index;
  converter.name = "N-k study VSC " + std::to_string(index);
  converter.bus_ac = ac_bus;
  converter.bus_dc = dc_bus;
  converter.in_service = true;
  converter.control_mode = vdc_control ? ConverterMode::VDC_Q
                                       : ConverterMode::PQ_MODE;
  converter.p_set_mw = p_schedule_mw;
  converter.p_schedule_mw = p_schedule_mw;
  converter.p_initial_mw = p_schedule_mw;
  converter.p_is_hard_constraint = false;
  converter.q_set_mvar = 0.0;
  converter.v_dc_set_pu = 1.0;
  converter.v_ac_set_pu = 1.0;
  converter.eta = 0.98;
  converter.k_vdc = 25.0;
  converter.pmax_mw = 2.0;
  converter.pmin_mw = -2.0;
  converter.qmax_mvar = 1.0;
  converter.qmin_mvar = -1.0;
  converter.p_rated_mw = 2.0;
  converter.i_ac_max_pu = 0.30;
  converter.dynamic_model.standard = "IEEE/NERC";
  converter.dynamic_model.model_name = "REGC_REEC_GFL_Subset";
  converter.dynamic_model.parameters = {
      {"ieee1547_enabled", 1.0},
      {"ieee1547_category", 2.0},
      {"ieee1547_allow_reconnect", 0.0},
      {"dc_fault_control_enabled", 1.0},
      {"dc_active_power_derate_start_pu", 0.95},
      {"dc_undervoltage_block_pu", 0.75},
      {"dc_undervoltage_block_delay_s", 0.02},
  };
}

void add_dc_bus(HybridPowerSystem& system, int index, bool reference) {
  DCBus bus;
  bus.index = index;
  bus.name = "N-k DC bus " + std::to_string(index);
  bus.bus_type = reference ? DCBusType::DC_V : DCBusType::DC_P;
  bus.vm_pu = 1.0;
  bus.vmin_pu = 0.90;
  bus.vmax_pu = 1.10;
  bus.base_kv = 1.5;
  bus.in_service = true;
  system.dc.buses.push_back(bus);
}

void add_dc_branch(HybridPowerSystem& system, int index, int from, int to,
                   bool in_service) {
  DCBranch branch;
  branch.index = index;
  branch.name = "N-k DC branch " + std::to_string(index);
  branch.from_bus = from;
  branch.to_bus = to;
  branch.r_pu = 0.02;
  branch.rate_a_mva = 2.0;
  branch.s_max_mva = 2.0;
  branch.in_service = in_service;
  system.dc.branches.push_back(branch);
}

void add_dc_load(HybridPowerSystem& system, int index, int bus, double p_mw,
                 int customers) {
  DCLoad load;
  load.index = index;
  load.bus = bus;
  load.name = "N-k DC load " + std::to_string(index);
  load.p_mw = p_mw;
  load.p_rated_mw = p_mw;
  load.n_customers = customers;
  load.in_service = true;
  system.dc.loads.push_back(load);
}

int bus_named(const HybridPowerSystem& system, const std::string& name) {
  const auto found = std::find_if(
      system.ac.buses.begin(), system.ac.buses.end(),
      [&](const ACBus& bus) { return bus.name == name; });
  if (found == system.ac.buses.end()) {
    throw std::runtime_error("IEEE 123 positive-sequence bus is absent: " + name);
  }
  return found->index;
}

std::vector<const ACBranch*> in_service_ac_branches(
    const HybridPowerSystem& system) {
  std::vector<const ACBranch*> branches;
  for (const auto& branch : system.ac.branches) {
    if (branch.in_service) branches.push_back(&branch);
  }
  return branches;
}

StudyCase make_case33() {
  StudyCase data;
  data.key = "case33_acdc";
  data.source = "hacdcpf::io::build_case33bw_acdc with a study-specific DC overlay";
  data.model_scope =
      "balanced IEEE 33-bus AC feeder; five-bus radial DC feeder with one "
      "normally-open DC tie; three average-value GFL VSCs";
  data.system = io::build_case33bw_acdc();
  data.system.name = "N-k modified case33 AC/DC";
  data.system.dc = DCSystem{};
  data.system.dc.base_mva = data.system.base_mva;
  data.system.vsc_converters.clear();
  for (int i = 0; i < 5; ++i) add_dc_bus(data.system, 101 + i, i == 0);
  add_dc_branch(data.system, 1001, 101, 102, true);
  add_dc_branch(data.system, 1002, 102, 103, true);
  add_dc_branch(data.system, 1003, 103, 104, true);
  add_dc_branch(data.system, 1004, 104, 105, true);
  add_dc_branch(data.system, 1005, 102, 105, false);
  add_dc_load(data.system, 1101, 102, 0.30, 45);
  add_dc_load(data.system, 1102, 104, 0.35, 55);
  add_dc_load(data.system, 1103, 105, 0.25, 35);
  data.system.vsc_converters.resize(3);
  configure_vsc(data.system.vsc_converters[0], 2001, 6, 101, 0.0, true);
  configure_vsc(data.system.vsc_converters[1], 2002, 18, 103, -0.45, false);
  configure_vsc(data.system.vsc_converters[2], 2003, 30, 105, -0.48, false);
  for (auto& branch : data.system.ac.branches) {
    if (branch.rate_a_mva <= 0.0) branch.rate_a_mva = 5.0;
  }
  const std::vector<int> ac_ids{5, 15, 28};
  for (int id : ac_ids) {
    const auto found = std::find_if(
        data.system.ac.branches.begin(), data.system.ac.branches.end(),
        [id](const ACBranch& branch) {
          return branch.index == id && branch.in_service;
        });
    if (found == data.system.ac.branches.end()) {
      throw std::runtime_error("case33 N-k AC branch selection is unavailable");
    }
    data.assets.push_back(
        {AssetKind::ACBranch, id, found->to_bus, 1.0,
         "AC feeder section " + std::to_string(id)});
  }
  data.assets.push_back({AssetKind::DCBranch, 1001, 102, 0.9,
                         "DC source section 1001"});
  data.assets.push_back({AssetKind::DCBranch, 1002, 103, 0.8,
                         "DC section 1002"});
  data.assets.push_back({AssetKind::DCBranch, 1004, 105, 0.7,
                         "DC section 1004"});
  data.assets.push_back({AssetKind::VSC, 2002, 18, 0.6, "VSC 2002"});
  data.assets.push_back({AssetKind::VSC, 2003, 30, 0.6, "VSC 2003"});
  return data;
}

StudyCase make_case123() {
  StudyCase data;
  data.key = "case123_acdc";
  const std::filesystem::path master =
      std::filesystem::path(HACDCPF_PROJECT_ROOT) / "external_data" /
      "opendss_ieee_pes" / "opendss_reference" / "123_node" /
      "IEEE123Master.dss";
  io::OpenDSSImportOptions import;
  import.mode = io::ImportMode::Permissive;
  import.default_base_mva = 10.0;
  import.default_base_kv_ll = 4.16;
  import.default_frequency_hz = 60.0;
  import.convert_actual_to_per_unit = true;
  import.create_slack_generator = true;
  auto loaded = io::load_opendss_with_report(master, import);
  data.import_warnings = loaded.warnings;
  data.import_skipped = loaded.skipped;
  data.system = std::move(loaded.system);
  data.system.name = "N-k redesigned case123 AC/DC";
  data.system.base_mva = 10.0;
  data.system.ac.base_mva = 10.0;
  data.system.dc = DCSystem{};
  data.system.dc.base_mva = 10.0;
  data.system.vsc_converters.clear();
  data.source = master.string();
  data.model_scope =
      "balanced positive-sequence projection of the public OpenDSS IEEE 123 "
      "feeder; seven-bus radial DC feeder with one normally-open tie; four "
      "average-value GFL VSCs";
  int next_branch = 1;
  for (const auto& branch : data.system.ac.branches) {
    next_branch = std::max(next_branch, branch.index + 1);
  }
  // Keep the positive-sequence screening topology electrically connected by
  // applying the same closed-switch projection used by
  // src/io/sioux_falls_ieee123_case.cpp::prepare_power_system.
  for (const auto& sw : data.system.ac.switches) {
    if (!sw.in_service || !sw.closed) continue;
    ACBranch branch;
    branch.index = next_branch++;
    branch.name = "closed-switch-" + sw.name;
    branch.from_bus = sw.bus_from;
    branch.to_bus = sw.bus_to;
    branch.r_pu = 0.0;
    branch.x_pu = 1e-4;
    branch.rate_a_mva = 5.0;
    branch.in_service = true;
    data.system.ac.branches.push_back(branch);
    ++data.converted_closed_switches;
  }
  data.system.ac.switches.clear();
  for (auto& branch : data.system.ac.branches) {
    if (branch.rate_a_mva <= 0.0) branch.rate_a_mva = 5.0;
    if (std::abs(branch.x_pu) < 1e-6) branch.x_pu = 1e-4;
  }
  std::map<int, std::size_t> bus_position;
  for (std::size_t i = 0; i < data.system.ac.buses.size(); ++i) {
    bus_position.emplace(data.system.ac.buses[i].index, i);
  }
  std::vector<std::size_t> parent(data.system.ac.buses.size());
  std::iota(parent.begin(), parent.end(), 0);
  const auto find_root = [&](const auto& self, std::size_t node) -> std::size_t {
    if (parent[node] != node) parent[node] = self(self, parent[node]);
    return parent[node];
  };
  for (auto& branch : data.system.ac.branches) {
    if (!branch.in_service) continue;
    const auto from = bus_position.find(branch.from_bus);
    const auto to = bus_position.find(branch.to_bus);
    if (from == bus_position.end() || to == bus_position.end()) {
      throw std::runtime_error(
          "IEEE 123 branch endpoint is absent from the AC bus map");
    }
    const std::size_t from_root = find_root(find_root, from->second);
    const std::size_t to_root = find_root(find_root, to->second);
    if (from_root == to_root) {
      branch.in_service = false;
      ++data.radialized_ac_ties;
    } else {
      parent[to_root] = from_root;
    }
  }
  const std::size_t connected_root = find_root(find_root, 0);
  if (std::any_of(parent.begin(), parent.end(), [&](std::size_t node) {
        return find_root(find_root, node) != connected_root;
      })) {
    throw std::runtime_error(
        "IEEE 123 positive-sequence projection is disconnected before outages");
  }
  for (int i = 0; i < 7; ++i) add_dc_bus(data.system, 501 + i, i == 0);
  for (int i = 0; i < 6; ++i) {
    add_dc_branch(data.system, 3001 + i, 501 + i, 502 + i, true);
  }
  add_dc_branch(data.system, 3007, 502, 507, false);
  add_dc_load(data.system, 3101, 502, 0.25, 40);
  add_dc_load(data.system, 3102, 504, 0.30, 50);
  add_dc_load(data.system, 3103, 506, 0.35, 60);
  add_dc_load(data.system, 3104, 507, 0.20, 30);
  const std::vector<int> ac_buses{
      bus_named(data.system, "35"), bus_named(data.system, "60"),
      bus_named(data.system, "97"), bus_named(data.system, "114")};
  data.system.vsc_converters.resize(4);
  configure_vsc(data.system.vsc_converters[0], 4001, ac_buses[0], 501, 0.0,
                true);
  configure_vsc(data.system.vsc_converters[1], 4002, ac_buses[1], 503, -0.38,
                false);
  configure_vsc(data.system.vsc_converters[2], 4003, ac_buses[2], 505, -0.38,
                false);
  configure_vsc(data.system.vsc_converters[3], 4004, ac_buses[3], 507, -0.38,
                false);

  const auto ac = in_service_ac_branches(data.system);
  if (ac.size() < 20) {
    throw std::runtime_error("IEEE 123 import has too few in-service AC branches");
  }
  const std::vector<std::size_t> quantiles{
      ac.size() / 5, 2 * ac.size() / 5, 3 * ac.size() / 5,
      4 * ac.size() / 5};
  for (std::size_t position : quantiles) {
    const ACBranch& branch = *ac.at(std::min(position, ac.size() - 1));
    data.assets.push_back(
        {AssetKind::ACBranch, branch.index, branch.to_bus, 1.0,
         "IEEE123 AC section " + std::to_string(branch.index)});
  }
  data.assets.push_back({AssetKind::DCBranch, 3001, 502, 0.9,
                         "DC source section 3001"});
  data.assets.push_back({AssetKind::DCBranch, 3003, 504, 0.8,
                         "DC section 3003"});
  data.assets.push_back({AssetKind::DCBranch, 3005, 506, 0.7,
                         "DC section 3005"});
  data.assets.push_back({AssetKind::VSC, 4002, ac_buses[1], 0.6, "VSC 4002"});
  data.assets.push_back({AssetKind::VSC, 4003, ac_buses[2], 0.6, "VSC 4003"});
  data.assets.push_back({AssetKind::VSC, 4004, ac_buses[3], 0.6, "VSC 4004"});
  return data;
}

void scale_operating_state(HybridPowerSystem& system,
                           const OperatingState& operating) {
  for (auto& bus : system.ac.buses) {
    bus.pd_mw *= operating.ac_load_scale;
    bus.qd_mvar *= operating.ac_load_scale;
  }
  for (auto& load : system.ac.loads) load.scaling *= operating.ac_load_scale;
  for (auto& load : system.dc.loads) load.scaling *= operating.dc_load_scale;
}

std::string event_key(const StudyCase& data,
                      const std::vector<std::size_t>& positions) {
  std::vector<std::string> labels;
  for (std::size_t position : positions) {
    const Asset& asset = data.assets.at(position);
    labels.push_back(asset_prefix(asset.kind) + ":" +
                     std::to_string(asset.index));
  }
  std::sort(labels.begin(), labels.end());
  std::ostringstream out;
  for (std::size_t i = 0; i < labels.size(); ++i) {
    if (i != 0) out << '+';
    out << labels[i];
  }
  return out.str();
}

std::vector<EventSet> enumerate_events(const StudyCase& data, int max_order) {
  std::vector<EventSet> events;
  const int admitted_order = std::min<int>(max_order, data.assets.size());
  for (int order = 1; order <= admitted_order; ++order) {
    std::vector<std::vector<std::size_t>> combinations;
    std::vector<std::size_t> current;
    const auto recurse = [&](const auto& self, std::size_t begin) -> void {
      if (static_cast<int>(current.size()) == order) {
        combinations.push_back(current);
        return;
      }
      const std::size_t remaining =
          static_cast<std::size_t>(order) - current.size();
      for (std::size_t i = begin; i + remaining <= data.assets.size(); ++i) {
        current.push_back(i);
        self(self, i + 1);
        current.pop_back();
      }
    };
    recurse(recurse, 0);
    double normalizer = 0.0;
    for (const auto& positions : combinations) {
      double weight = 1.0;
      for (std::size_t position : positions) {
        weight *= data.assets[position].severity_weight;
      }
      normalizer += weight;
    }
    // Explicit joint-entry frequency by outage order. This benchmark measure
    // follows Billinton-Allan multi-state event accounting; it deliberately
    // does not multiply marginal component failure frequencies.
    const double order_frequency = 0.30 * std::pow(0.10, order - 1);
    for (const auto& positions : combinations) {
      double weight = 1.0;
      for (std::size_t position : positions) {
        weight *= data.assets[position].severity_weight;
      }
      events.push_back({order, positions, order_frequency * weight / normalizer,
                        event_key(data, positions)});
    }
  }
  return events;
}

DynamicEvent entry_event(const Asset& asset) {
  DynamicEvent event;
  event.time_s = kFaultTimeS;
  event.component_index = asset.index;
  event.label = "N-k entry " + asset.label;
  switch (asset.kind) {
    case AssetKind::ACBranch:
    case AssetKind::DCBranch:
      // Kundur, Power System Stability and Control (1994), Sec. 12.2:
      // represent the pre-isolation fault by a shunt admittance. The permanent
      // equipment outage is imposed separately in the restoration MILP.
      event.type = DynamicEventType::FaultShunt;
      event.bus = asset.fault_bus;
      event.component_type = asset.kind == AssetKind::ACBranch ? "AC" : "DC";
      event.params["r_pu"] = asset.kind == AssetKind::ACBranch
          ? kAcFaultResistancePu
          : kDcFaultResistancePu;
      event.params["x_pu"] = 0.0;
      event.params["duration_s"] = kFaultDurationS;
      break;
    case AssetKind::VSC:
      event.type = DynamicEventType::VSCTrip;
      event.component_type = "VSC";
      event.bus = asset.fault_bus;
      break;
  }
  return event;
}

std::string final_voltage_margin_band(const DynamicSnapshot& snapshot) {
  double margin = std::numeric_limits<double>::infinity();
  for (Eigen::Index bus = 0; 3 * bus + 2 < snapshot.vac_abc.size(); ++bus) {
    const std::complex<double> a =
        std::polar(1.0, 2.0 * std::acos(-1.0) / 3.0);
    const std::complex<double> v1 =
        (snapshot.vac_abc[3 * bus] + a * snapshot.vac_abc[3 * bus + 1] +
         a * a * snapshot.vac_abc[3 * bus + 2]) /
        3.0;
    const double magnitude = std::abs(v1);
    margin = std::min(margin, std::min(magnitude - 0.90, 1.10 - magnitude));
  }
  for (double voltage : snapshot.vdc) {
    margin = std::min(margin, std::min(voltage - 0.90, 1.10 - voltage));
  }
  if (!std::isfinite(margin)) return "unknown";
  if (margin < 0.0) return "violated";
  if (margin < 0.05) return "alert";
  return "secure";
}

TrajectoryResult run_trajectory(const HybridPowerSystem& system,
                                const StudyCase& data,
                                const EventSet& event_set, double dt_s) {
  TrajectoryResult out;
  const auto start = std::chrono::steady_clock::now();
  try {
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
    options.post_event_algebraic_residual_tol = kMaximumPostEventResidual;
    options.algebraic_network_tol =
        data.key == "case123_acdc" ? kMaximumPostEventResidual : 1e-9;
    options.algebraic_network_max_iters = 25;
    options.record_every_step = false;
    options.record_device_outputs = true;
    options.enforce_voltage_health_check = false;
    DynamicSystem dynamic = DynamicModelBuilder{}.build(system, options);
    for (std::size_t position : event_set.asset_positions) {
      dynamic.events.push_back(entry_event(data.assets.at(position)));
    }
    const DynamicResults result = DynamicSolver{}.solve(dynamic);
    out.message = result.message;
    out.max_post_event_residual = result.max_post_event_algebraic_residual;
    if (!result.success || result.final_snapshot() == nullptr) {
      out.resolved = false;
    } else {
      out.resolved = true;
      const DynamicSnapshot& final = *result.final_snapshot();
      out.voltage_margin_band = final_voltage_margin_band(final);
      for (const auto& device : final.device_outputs) {
        if (device.source_type != "vsc_grid_following" &&
            device.source_type != "vsc_grid_forming") {
          continue;
        }
        const auto available = device.values.find("in_service");
        if (available != device.values.end() && available->second < 0.5) {
          out.unavailable_vsc.insert(device.component_index);
        }
      }
      for (const auto& applied : result.applied_event_records) {
        if (applied.type == "VSCTrip" &&
            applied.time_s >= kFaultTimeS - 1e-9) {
          ++out.emitted_vsc_trip_count;
        }
      }
    }
  } catch (const std::exception& error) {
    out.resolved = false;
    out.message = error.what();
  }
  out.runtime_s = std::chrono::duration<double>(
                      std::chrono::steady_clock::now() - start)
                      .count();
  return out;
}

analysis::TopoReconfResult run_restoration(
    const HybridPowerSystem& system, const StudyCase& data,
    const EventSet& event_set, const std::set<int>& additional_vsc) {
  analysis::TopoReconfOptions options;
  options.enable_pf = true;
  options.enable_voltage = true;
  options.enable_thermal = true;
  options.split_domain_trees = true;
  options.lambda_shed = 1e5;
  options.lambda_island = 1e6;
  options.max_time_s = 30;
  options.mip_gap = 1e-4;
  options.verbose = false;
  for (std::size_t position : event_set.asset_positions) {
    const Asset& asset = data.assets.at(position);
    options.faulted_branches.push_back(
        {edge_category(asset.kind), asset.index});
  }
  for (int vsc : additional_vsc) {
    const bool initiated = std::any_of(
        event_set.asset_positions.begin(), event_set.asset_positions.end(),
        [&](std::size_t position) {
          const Asset& asset = data.assets.at(position);
          return asset.kind == AssetKind::VSC && asset.index == vsc;
        });
    if (!initiated) {
      options.faulted_branches.push_back(
          {graph::EdgeCategory::VSC_Coupling, vsc});
    }
  }
  return analysis::run_topology_reconfiguration(system, options);
}

std::string class_label(const std::set<int>& unavailable_vsc) {
  // The equivalence relation is defined only on the DAE output supplied to
  // the restoration MILP. Event identity is carried separately by the group
  // key; load and generation remain state-specific MILP inputs. See
  // docs/theory/reliability_assessment_models.md, "N-k trajectory classes".
  std::ostringstream label;
  label << "unavailable-vsc=";
  for (int index : unavailable_vsc) label << index << ',';
  return label.str();
}

json run_case(const StudyCase& data, const Options& options) {
  const auto operating = operating_states(options.smoke);
  validate_probability_mass(operating);
  StudyCase admitted = data;
  if (options.smoke && admitted.assets.size() > 3) {
    std::vector<Asset> representative;
    for (AssetKind kind : {AssetKind::ACBranch, AssetKind::DCBranch,
                           AssetKind::VSC}) {
      const auto found = std::find_if(
          admitted.assets.begin(), admitted.assets.end(),
          [kind](const Asset& asset) { return asset.kind == kind; });
      if (found == admitted.assets.end()) {
        throw std::runtime_error(
            "smoke design requires one AC branch, DC branch, and VSC entry");
      }
      representative.push_back(*found);
    }
    admitted.assets = std::move(representative);
  }
  const auto events = enumerate_events(admitted, options.max_order);
  std::vector<Sample> samples;
  samples.reserve(events.size() * operating.size());
  double maximum_residual = 0.0;
  double measured_direct_runtime_s = 0.0;
  double attempted_dae_library_runtime_s = 0.0;
  double resolved_dae_runtime_s = 0.0;
  double measured_static_restoration_runtime_s = 0.0;
  double measured_dynamic_restoration_runtime_s = 0.0;
  const auto library_start = std::chrono::steady_clock::now();

  for (std::size_t event_index = 0; event_index < events.size(); ++event_index) {
    const EventSet& event = events[event_index];
    for (std::size_t operating_index = 0; operating_index < operating.size();
         ++operating_index) {
      HybridPowerSystem system = admitted.system;
      scale_operating_state(system, operating[operating_index]);
      const TrajectoryResult trajectory =
          run_trajectory(system, admitted, event, options.dt_s);
      Sample sample;
      sample.event_index = event_index;
      sample.operating_index = operating_index;
      sample.scenario_probability = operating[operating_index].probability;
      sample.trajectory_runtime_s = trajectory.runtime_s;
      sample.direct_runtime_s = trajectory.runtime_s;
      attempted_dae_library_runtime_s += trajectory.runtime_s;
      sample.message = trajectory.message;
      if (!trajectory.resolved) {
        samples.push_back(std::move(sample));
        continue;
      }
      maximum_residual = std::max(maximum_residual,
                                  trajectory.max_post_event_residual);
      const auto static_start = std::chrono::steady_clock::now();
      const auto static_restoration =
          run_restoration(system, admitted, event, {});
      sample.static_restoration_runtime_s = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - static_start).count();
      const auto dynamic_start = std::chrono::steady_clock::now();
      const auto dynamic_restoration =
          run_restoration(system, admitted, event, trajectory.unavailable_vsc);
      sample.dynamic_restoration_runtime_s = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - dynamic_start).count();
      sample.direct_runtime_s += sample.dynamic_restoration_runtime_s;
      if (!static_restoration.feasible) {
        sample.message = "static restoration unresolved: " +
                         static_restoration.solver_status;
        samples.push_back(std::move(sample));
        continue;
      }
      if (!dynamic_restoration.feasible) {
        sample.message = "DAE-entry restoration unresolved: " +
                         dynamic_restoration.solver_status;
        samples.push_back(std::move(sample));
        continue;
      }
      sample.resolved = true;
      measured_direct_runtime_s += sample.direct_runtime_s;
      resolved_dae_runtime_s += sample.trajectory_runtime_s;
      measured_static_restoration_runtime_s +=
          sample.static_restoration_runtime_s;
      measured_dynamic_restoration_runtime_s +=
          sample.dynamic_restoration_runtime_s;
      sample.static_reward_mwh =
          std::max(0.0, static_restoration.total_shed_mw) * kRepairDurationH;
      sample.dynamic_reward_mwh =
          std::max(0.0, dynamic_restoration.total_shed_mw) * kRepairDurationH;
      sample.unavailable_vsc = trajectory.unavailable_vsc;
      sample.additional_vsc_outages = 0;
      for (int vsc : trajectory.unavailable_vsc) {
        const bool initiated = std::any_of(
            event.asset_positions.begin(), event.asset_positions.end(),
            [&](std::size_t position) {
              const Asset& asset = admitted.assets[position];
              return asset.kind == AssetKind::VSC && asset.index == vsc;
            });
        if (!initiated) ++sample.additional_vsc_outages;
      }
      sample.class_label = class_label(sample.unavailable_vsc);
      samples.push_back(std::move(sample));
    }
  }
  const double library_wall_s = std::chrono::duration<double>(
                                    std::chrono::steady_clock::now() -
                                    library_start)
                                    .count();

  struct Group {
    double probability{0.0};
    double reward_probability{0.0};
    std::vector<std::size_t> sample_indices;
  };
  std::map<std::pair<std::size_t, std::string>, Group> groups;
  double direct_static = 0.0;
  double direct_dynamic = 0.0;
  double unresolved_frequency = 0.0;
  double resolved_frequency = 0.0;
  int higher_order_necessity_samples = 0;
  std::map<std::string, std::pair<std::size_t, double>> unresolved_reasons;
  for (std::size_t i = 0; i < samples.size(); ++i) {
    const Sample& sample = samples[i];
    const EventSet& event = events[sample.event_index];
    const double exposure = event.frequency_per_year * sample.scenario_probability;
    if (!sample.resolved) {
      unresolved_frequency += exposure;
      auto& reason = unresolved_reasons[sample.message.empty()
                                              ? "unspecified failure"
                                              : sample.message];
      ++reason.first;
      reason.second += exposure;
      continue;
    }
    resolved_frequency += exposure;
    direct_static += exposure * sample.static_reward_mwh;
    direct_dynamic += exposure * sample.dynamic_reward_mwh;
    Group& group = groups[{sample.event_index, sample.class_label}];
    group.probability += sample.scenario_probability;
    group.reward_probability +=
        sample.scenario_probability * sample.dynamic_reward_mwh;
    group.sample_indices.push_back(i);
    const double denominator = std::max(1e-12, sample.dynamic_reward_mwh);
    const double relative =
        std::abs(sample.dynamic_reward_mwh - sample.static_reward_mwh) /
        denominator;
    if (event.order >= 2 &&
        (sample.additional_vsc_outages > 0 ||
         relative > kNecessityRelativeGate)) {
      ++higher_order_necessity_samples;
    }
  }

  double class_dynamic = 0.0;
  for (const auto& [key, group] : groups) {
    const EventSet& event = events[key.first];
    if (group.probability <= 0.0) continue;
    const double conditional_mean =
        group.reward_probability / group.probability;
    class_dynamic += event.frequency_per_year * group.probability *
                     conditional_mean;
  }

  // Leave one operating state out for each event. The nearest remaining
  // pre-fault state predicts only the DAE-derived restoration-entry class;
  // the held-out load state still receives its own restoration MILP solve.
  // This is a non-circular predictive test, unlike class-mean consequence
  // reconstruction, which is only a law-of-total-expectation identity.
  const auto [ac_min_it, ac_max_it] = std::minmax_element(
      operating.begin(), operating.end(),
      [](const OperatingState& lhs, const OperatingState& rhs) {
        return lhs.ac_load_scale < rhs.ac_load_scale;
      });
  const auto [dc_min_it, dc_max_it] = std::minmax_element(
      operating.begin(), operating.end(),
      [](const OperatingState& lhs, const OperatingState& rhs) {
        return lhs.dc_load_scale < rhs.dc_load_scale;
      });
  const double ac_span = std::max(1e-12,
      ac_max_it->ac_load_scale - ac_min_it->ac_load_scale);
  const double dc_span = std::max(1e-12,
      dc_max_it->dc_load_scale - dc_min_it->dc_load_scale);
  double loo_estimate = 0.0;
  double loo_reference = 0.0;
  double loo_weighted_absolute_error = 0.0;
  double loo_reference_absolute = 0.0;
  double loo_unresolved_frequency = 0.0;
  double loo_misclassified_frequency = 0.0;
  double loo_audited_frequency = 0.0;
  for (std::size_t i = 0; i < samples.size(); ++i) {
    Sample& sample = samples[i];
    if (!sample.resolved) continue;
    const EventSet& event = events[sample.event_index];
    const double exposure = event.frequency_per_year * sample.scenario_probability;
    double best_distance = std::numeric_limits<double>::infinity();
    std::size_t best_index = samples.size();
    for (std::size_t j = 0; j < samples.size(); ++j) {
      const Sample& candidate = samples[j];
      if (j == i || !candidate.resolved ||
          candidate.event_index != sample.event_index) {
        continue;
      }
      const OperatingState& target_state = operating[sample.operating_index];
      const OperatingState& candidate_state =
          operating[candidate.operating_index];
      const double dac = (target_state.ac_load_scale -
                          candidate_state.ac_load_scale) / ac_span;
      const double ddc = (target_state.dc_load_scale -
                          candidate_state.dc_load_scale) / dc_span;
      const double distance = dac * dac + ddc * ddc;
      if (distance < best_distance - 1e-15 ||
          (std::abs(distance - best_distance) <= 1e-15 && j < best_index)) {
        best_distance = distance;
        best_index = j;
      }
    }
    if (best_index == samples.size()) {
      loo_unresolved_frequency += exposure;
      continue;
    }
    sample.loo_has_neighbor = true;
    sample.loo_neighbor_operating_index = samples[best_index].operating_index;
    sample.loo_predicted_class_label = samples[best_index].class_label;
    loo_audited_frequency += exposure;
    if (sample.loo_predicted_class_label != sample.class_label) {
      loo_misclassified_frequency += exposure;
    }

    HybridPowerSystem held_out_system = admitted.system;
    scale_operating_state(held_out_system, operating[sample.operating_index]);
    const auto predicted_restoration = run_restoration(
        held_out_system, admitted, event, samples[best_index].unavailable_vsc);
    if (!predicted_restoration.feasible) {
      loo_unresolved_frequency += exposure;
      continue;
    }
    sample.loo_restoration_resolved = true;
    sample.loo_predicted_reward_mwh =
        std::max(0.0, predicted_restoration.total_shed_mw) * kRepairDurationH;
    loo_estimate += exposure * sample.loo_predicted_reward_mwh;
    loo_reference += exposure * sample.dynamic_reward_mwh;
    loo_weighted_absolute_error += exposure * std::abs(
        sample.loo_predicted_reward_mwh - sample.dynamic_reward_mwh);
    loo_reference_absolute += exposure * std::abs(sample.dynamic_reward_mwh);
  }
  const bool loo_has_coverage = loo_reference > 1e-12;
  const double loo_relative_error = loo_has_coverage
      ? std::abs(loo_estimate - loo_reference) / std::abs(loo_reference)
      : std::numeric_limits<double>::infinity();
  const double loo_weighted_absolute_relative_error = loo_reference_absolute > 1e-12
      ? loo_weighted_absolute_error / loo_reference_absolute
      : std::numeric_limits<double>::infinity();
  const double loo_class_accuracy = loo_audited_frequency > 1e-12
      ? 1.0 - loo_misclassified_frequency / loo_audited_frequency
      : 0.0;

  const std::size_t resolved_samples = static_cast<std::size_t>(std::count_if(
      samples.begin(), samples.end(), [](const Sample& sample) {
        return sample.resolved;
      }));
  const double compression_ratio = resolved_samples == 0
      ? std::numeric_limits<double>::infinity()
      : static_cast<double>(groups.size()) /
            static_cast<double>(resolved_samples);
  const double mean_direct_s = resolved_samples == 0
      ? std::numeric_limits<double>::infinity()
      : measured_direct_runtime_s / static_cast<double>(resolved_samples);
  const double mean_static_restoration_s = resolved_samples == 0
      ? std::numeric_limits<double>::infinity()
      : measured_static_restoration_runtime_s /
            static_cast<double>(resolved_samples);
  const double mean_dae_s = resolved_samples == 0
      ? std::numeric_limits<double>::infinity()
      : resolved_dae_runtime_s / static_cast<double>(resolved_samples);
  const double mean_dynamic_restoration_s = resolved_samples == 0
      ? std::numeric_limits<double>::infinity()
      : measured_dynamic_restoration_runtime_s /
            static_cast<double>(resolved_samples);

  volatile double lookup_sink = 0.0;
  std::vector<double> class_rewards;
  class_rewards.reserve(groups.size());
  for (const auto& [key, group] : groups) {
    (void)key;
    class_rewards.push_back(group.reward_probability /
                            std::max(1e-12, group.probability));
  }
  const auto lookup_start = std::chrono::steady_clock::now();
  if (!class_rewards.empty()) {
    for (std::size_t i = 0; i < kMcBenchmarkSamples; ++i) {
      lookup_sink = lookup_sink + class_rewards[i % class_rewards.size()];
    }
  }
  const double lookup_wall_s = std::chrono::duration<double>(
                                   std::chrono::steady_clock::now() -
                                   lookup_start)
                                   .count();
  if (lookup_sink < 0.0) std::cerr << "unreachable lookup sink\n";
  const double projected_direct_s =
      static_cast<double>(kMcBenchmarkSamples) * mean_direct_s;
  const double projected_dae_only_s =
      attempted_dae_library_runtime_s + lookup_wall_s;
  const double projected_tcerm_s =
      projected_dae_only_s + static_cast<double>(kMcBenchmarkSamples) *
                                 mean_dynamic_restoration_s;
  const double projected_speedup =
      projected_direct_s / std::max(1e-12, projected_tcerm_s);
  const bool timing_projection_admissible = unresolved_frequency <= 1e-12;
  const double break_even_samples =
      attempted_dae_library_runtime_s /
      std::max(1e-12, mean_dae_s -
                         lookup_wall_s /
                             static_cast<double>(kMcBenchmarkSamples));

  std::map<int, std::size_t> events_by_order;
  std::map<int, double> frequency_by_order;
  for (const EventSet& event : events) {
    ++events_by_order[event.order];
    frequency_by_order[event.order] += event.frequency_per_year;
  }
  json order_catalog = json::array();
  for (const auto& [order, count] : events_by_order) {
    order_catalog.push_back({{"order", order},
                             {"joint_events", count},
                             {"authored_total_frequency_per_year",
                              frequency_by_order[order]}});
  }

  const double aggregation_error = std::abs(class_dynamic - direct_dynamic);
  const bool necessity_passed = higher_order_necessity_samples > 0;
  const bool accuracy_passed =
      aggregation_error <= kAggregationToleranceMwhYr &&
      loo_has_coverage && loo_unresolved_frequency <= 1e-12 &&
      loo_weighted_absolute_relative_error <= kLeaveOneOutRelativeGate &&
      loo_class_accuracy >= kLeaveOneOutClassAccuracyGate;
  const bool compression_passed =
      compression_ratio <= kCompressionRatioGate &&
      projected_speedup >= kSpeedupGate;
  const bool validation_passed =
      necessity_passed && accuracy_passed && compression_passed &&
      unresolved_frequency <= 1e-12 &&
      maximum_residual <= kMaximumPostEventResidual;

  json result;
  result["case"] = admitted.key;
  result["source"] = admitted.source;
  result["model_scope"] = admitted.model_scope;
  result["import_audit"] = {
      {"warnings", admitted.import_warnings},
      {"skipped", admitted.import_skipped},
      {"converted_closed_switches", admitted.converted_closed_switches},
      {"radialized_ac_ties", admitted.radialized_ac_ties},
  };
  result["network"] = {
      {"ac_buses", admitted.system.ac.buses.size()},
      {"ac_branches", admitted.system.ac.branches.size()},
      {"dc_buses", admitted.system.dc.buses.size()},
      {"dc_branches", admitted.system.dc.branches.size()},
      {"vsc_converters", admitted.system.vsc_converters.size()},
      {"initiating_assets", admitted.assets.size()},
  };
  result["catalog"] = {
      {"maximum_validated_order", options.max_order},
      {"entry_model", "explicit simultaneous joint-entry frequency"},
      {"marginal_frequency_products_used", false},
      {"dynamic_entry_model",
       "simultaneous finite-duration AC/DC shunt faults and VSC trips; "
       "faulted equipment is unavailable to restoration"},
      {"fault_time_s", kFaultTimeS},
      {"fault_duration_s", kFaultDurationS},
      {"ac_fault_resistance_pu", kAcFaultResistancePu},
      {"dc_fault_resistance_pu", kDcFaultResistancePu},
      {"events_by_order", order_catalog},
      {"operating_states", operating.size()},
      {"conditional_states", samples.size()},
      {"resolved_conditional_states", resolved_samples},
      {"unresolved_frequency_per_year", unresolved_frequency},
      {"resolved_frequency_per_year", resolved_frequency},
  };
  result["catalog"]["unresolved_reasons"] = json::array();
  for (const auto& [message, count_and_frequency] : unresolved_reasons) {
    result["catalog"]["unresolved_reasons"].push_back({
        {"message", message},
        {"conditional_states", count_and_frequency.first},
        {"frequency_per_year", count_and_frequency.second},
    });
  }
  result["event_catalog"] = json::array();
  for (const auto& event : events) {
    json assets = json::array();
    for (std::size_t position : event.asset_positions) {
      const Asset& asset = admitted.assets.at(position);
      assets.push_back({
          {"kind", asset_prefix(asset.kind)},
          {"component_index", asset.index},
          {"fault_bus", asset.fault_bus},
      });
    }
    result["event_catalog"].push_back({
        {"key", event.key},
        {"order", event.order},
        {"frequency_per_year", event.frequency_per_year},
        {"assets", std::move(assets)},
    });
  }
  result["operating_state_design"] = json::array();
  for (const auto& state : operating) {
    result["operating_state_design"].push_back({
        {"label", state.label},
        {"ac_load_scale", state.ac_load_scale},
        {"dc_load_scale", state.dc_load_scale},
        {"probability", state.probability},
    });
  }
  result["conditional_results"] = json::array();
  for (const auto& sample : samples) {
    const EventSet& event = events.at(sample.event_index);
    const OperatingState& state = operating.at(sample.operating_index);
    result["conditional_results"].push_back({
        {"event_key", event.key},
        {"event_order", event.order},
        {"event_frequency_per_year", event.frequency_per_year},
        {"operating_state", state.label},
        {"operating_probability", sample.scenario_probability},
        {"exposure_frequency_per_year",
         event.frequency_per_year * sample.scenario_probability},
        {"resolved", sample.resolved},
        {"message", sample.message},
        {"class_label", sample.class_label},
        {"static_consequence_mwh", sample.static_reward_mwh},
        {"dynamic_consequence_mwh", sample.dynamic_reward_mwh},
        {"additional_vsc_outages", sample.additional_vsc_outages},
        {"unavailable_vsc", sample.unavailable_vsc},
        {"trajectory_runtime_s", sample.trajectory_runtime_s},
        {"direct_condition_runtime_s", sample.direct_runtime_s},
        {"static_restoration_runtime_s",
         sample.static_restoration_runtime_s},
        {"dynamic_restoration_runtime_s",
         sample.dynamic_restoration_runtime_s},
        {"loo_has_neighbor", sample.loo_has_neighbor},
        {"loo_neighbor_operating_state",
         sample.loo_has_neighbor
             ? operating[sample.loo_neighbor_operating_index].label
             : ""},
        {"loo_predicted_class_label", sample.loo_predicted_class_label},
        {"loo_restoration_resolved", sample.loo_restoration_resolved},
        {"loo_predicted_dynamic_consequence_mwh",
         sample.loo_predicted_reward_mwh},
    });
  }
  result["transient_necessity"] = {
      {"higher_order_samples_with_entry_or_consequence_difference",
       higher_order_necessity_samples},
      {"static_eens_mwh_per_year", direct_static},
      {"transition_aware_eens_mwh_per_year", direct_dynamic},
      {"relative_difference",
       std::abs(direct_dynamic - direct_static) /
           std::max(1e-12, std::abs(direct_dynamic))},
      {"passed", necessity_passed},
  };
  result["class_accuracy"] = {
      {"class_records", groups.size()},
      {"direct_eens_mwh_per_year", direct_dynamic},
      {"class_eens_mwh_per_year", class_dynamic},
      {"aggregation_error_mwh_per_year", aggregation_error},
      {"aggregation_tolerance_mwh_per_year", kAggregationToleranceMwhYr},
      {"leave_one_out_reference_mwh_per_year", loo_reference},
      {"leave_one_out_estimate_mwh_per_year", loo_estimate},
      {"leave_one_out_relative_error", loo_relative_error},
      {"leave_one_out_weighted_absolute_relative_error",
       loo_weighted_absolute_relative_error},
      {"leave_one_out_relative_tolerance", kLeaveOneOutRelativeGate},
      {"leave_one_out_entry_state_accuracy", loo_class_accuracy},
      {"leave_one_out_entry_state_accuracy_gate",
       kLeaveOneOutClassAccuracyGate},
      {"leave_one_out_misclassified_frequency_per_year",
       loo_misclassified_frequency},
      {"leave_one_out_unresolved_frequency_per_year",
       loo_unresolved_frequency},
      {"leave_one_out_has_coverage", loo_has_coverage},
      {"passed", accuracy_passed},
  };
  result["compression"] = {
      {"class_records", groups.size()},
      {"resolved_conditional_states", resolved_samples},
      {"class_to_state_ratio", compression_ratio},
      {"ratio_gate", kCompressionRatioGate},
      {"library_wall_time_s", library_wall_s},
      {"dae_library_attempt_wall_time_s",
       attempted_dae_library_runtime_s},
      {"measured_mean_direct_condition_time_s", mean_direct_s},
      {"measured_mean_dae_time_s", mean_dae_s},
      {"measured_mean_static_restoration_time_s",
       mean_static_restoration_s},
      {"measured_mean_dynamic_restoration_time_s",
       mean_dynamic_restoration_s},
      {"mc_samples", kMcBenchmarkSamples},
      {"lookup_wall_time_s", lookup_wall_s},
      {"projected_direct_mc_wall_time_s", projected_direct_s},
      {"projected_dae_library_plus_lookup_time_s",
       projected_dae_only_s},
      {"projected_class_mc_wall_time_s", projected_tcerm_s},
      {"projected_end_to_end_speedup", projected_speedup},
      {"timing_projection_admissible", timing_projection_admissible},
      {"net_computational_saving_demonstrated",
       timing_projection_admissible && projected_speedup > 1.0},
      {"speedup_gate", kSpeedupGate},
      {"break_even_samples", break_even_samples},
      {"dae_solve_reduction_fraction_at_mc_samples",
       1.0 - static_cast<double>(samples.size()) /
                 static_cast<double>(kMcBenchmarkSamples)},
      {"passed", compression_passed},
      {"timing_interpretation",
       "direct MC cost is DAE plus DAE-entry restoration. Class compression "
       "removes repeated DAE solves but does not remove state-dependent "
       "restoration solves; the class projection therefore includes 100000 "
       "measured-mean restoration costs, exhaustive DAE-library construction, "
       "and measured lookup. It is not a claim that 100000 direct DAE runs "
       "were executed. A nonzero unresolved frequency makes this timing "
       "projection diagnostic rather than admissible"},
  };
  result["numerics"] = {
      {"dt_s", options.dt_s},
      {"maximum_post_event_algebraic_residual", maximum_residual},
      {"maximum_post_event_algebraic_residual_gate",
       kMaximumPostEventResidual},
      {"passed", maximum_residual <= kMaximumPostEventResidual},
  };
  result["validation_passed"] = validation_passed;
  result["model_limitations"] = {
      "N-k events are authored simultaneous joint entries; sequential overlaps and common-cause calibration require separate data.",
      "AC/DC branch entries use finite-duration bus-shunt faults before isolation; they do not resolve travelling waves, breaker arcs, or branch-internal fault location.",
      "IEEE 123 dynamics use the balanced positive-sequence OpenDSS projection, not native three-phase DAE.",
      "Restoration uses the existing hybrid AC/DC LinDistFlow model; post-action nonlinear AC/DC feasibility is not certified here.",
      "The restoration MILP retains its native active/reactive load-shedding variables for every operating state; trajectory classes replace neither those variables nor the state-specific MILP solve.",
      "The class timing comparison includes measured library construction and measured lookup but projects the direct 100000-sample MC cost from measured per-condition times.",
      "Class accuracy is admitted only on the declared finite operating-state design measure; continuous-state generalization is not claimed.",
  };
  return result;
}

Options parse_options(int argc, char** argv) {
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string argument = argv[i];
    auto value = [&](const std::string& option) {
      if (++i >= argc) throw std::invalid_argument("missing value for " + option);
      return std::string(argv[i]);
    };
    if (argument == "--output") {
      options.output = value(argument);
    } else if (argument == "--case") {
      options.selected_case = value(argument);
    } else if (argument == "--max-order") {
      options.max_order = std::stoi(value(argument));
    } else if (argument == "--dt") {
      options.dt_s = std::stod(value(argument));
    } else if (argument == "--smoke") {
      options.smoke = true;
      options.max_order = std::min(options.max_order, 2);
      options.dt_s = 0.01;
    } else if (argument == "--audit-only") {
      options.audit_only = true;
    } else if (argument == "--help" || argument == "-h") {
      std::cout << "usage: nk_acdc_reliability_study [--case all|case33_acdc|case123_acdc] "
                   "[--max-order K] [--dt SECONDS] [--smoke] [--audit-only] "
                   "[--output FILE]\n";
      std::exit(0);
    } else {
      throw std::invalid_argument("unknown option: " + argument);
    }
  }
  if (options.max_order < 1 || options.dt_s <= 0.0) {
    throw std::invalid_argument("max-order and dt must be positive");
  }
  if (options.selected_case != "all" &&
      options.selected_case != "case33_acdc" &&
      options.selected_case != "case123_acdc") {
    throw std::invalid_argument("unsupported case selection");
  }
  return options;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const Options options = parse_options(argc, argv);
    json report;
    report["study"] = "N-k transition-aware reliability case analysis";
    report["pre_registered_gates"] = {
        {"maximum_aggregation_error_mwh_per_year",
         kAggregationToleranceMwhYr},
        {"maximum_leave_one_out_relative_error",
         kLeaveOneOutRelativeGate},
        {"minimum_leave_one_out_entry_state_accuracy",
         kLeaveOneOutClassAccuracyGate},
        {"maximum_class_to_state_ratio", kCompressionRatioGate},
        {"minimum_projected_end_to_end_speedup", kSpeedupGate},
        {"minimum_dynamic_static_relative_difference",
         kNecessityRelativeGate},
        {"maximum_post_event_algebraic_residual",
         kMaximumPostEventResidual},
    };
    report["cases"] = json::array();
    if (options.selected_case == "all" ||
        options.selected_case == "case33_acdc") {
      report["cases"].push_back(run_case(make_case33(), options));
    }
    if (options.selected_case == "all" ||
        options.selected_case == "case123_acdc") {
      report["cases"].push_back(run_case(make_case123(), options));
    }
    report["validation_passed"] = std::all_of(
        report["cases"].begin(), report["cases"].end(),
        [](const json& result) {
          return result.value("validation_passed", false);
        });
    std::ofstream output(options.output);
    if (!output) {
      throw std::runtime_error("cannot open output report: " +
                               options.output.string());
    }
    output << std::setw(2) << report << '\n';
    std::cout << "Wrote " << options.output << '\n';
    return options.audit_only || report["validation_passed"].get<bool>() ? 0 : 2;
  } catch (const std::exception& error) {
    std::cerr << "N-k AC/DC reliability study failed: " << error.what() << '\n';
    return 1;
  }
}
