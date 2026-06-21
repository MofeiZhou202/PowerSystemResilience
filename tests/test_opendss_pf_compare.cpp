#include <array>
#include <algorithm>
#include <complex>
#include <cstdlib>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <map>
#include <stdexcept>
#include <string>
#include <string_view>
#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <process.h>
#else
#include <sys/wait.h>
#include <unistd.h>
#endif
#include <utility>
#include <vector>

#include <catch2/catch_test_macros.hpp>
#include <nlohmann/json.hpp>

#include "hacdcpf/analysis/distribution_power_flow.hpp"
#include "hacdcpf/io/opendss_bridge.hpp"
#include "hacdcpf/model/network_utils.hpp"
#include "opendss_compare/fixtures.hpp"
#include "opendss_compare/reproducibility.hpp"
#include "opendss_compare/snapshot_contract.hpp"
#include "opendss_compare/units.hpp"

namespace {

namespace fs = std::filesystem;
using json = nlohmann::json;
using Complex = std::complex<double>;
constexpr double kPi = 3.14159265358979323846;

int current_process_id() {
#ifdef _WIN32
  return _getpid();
#else
  return getpid();
#endif
}

// Canonical OpenDSS compare entrypoints for this validation chain:
//   build:   cmake --build build/opendss_compare --target opendss_snapshot_adapter opendss_pf_compare test_opendss_pf_compare -j4
//   test:    build/opendss_compare/test_opendss_pf_compare
//   adapter: build/opendss_compare/opendss_snapshot_adapter --master tests/data/opendss/minimal_regulator_1ph/Master.dss
//   compare: build/opendss_compare/opendss_pf_compare
// Keep these aligned with tools/opendss_compare/reproducibility.hpp and
// the machine-readable reproducibility block emitted by opendss_pf_compare.

// ---------------------------------------------------------------------------
// Key builders
// ---------------------------------------------------------------------------

std::string key_for_bus_node(const std::string& bus_name, int node) {
  return bus_name + "." + std::to_string(node);
}

std::string key_for_line_terminal_node(const std::string& line_name,
                                       int terminal,
                                       int node) {
  return "line:" + line_name + ":" + std::to_string(terminal) + ":" +
         std::to_string(node);
}

std::string key_for_transformer_terminal_node(const std::string& xfmr_name,
                                              int terminal,
                                              int node) {
  return "transformer:" + xfmr_name + ":" + std::to_string(terminal) + ":" +
         std::to_string(node);
}

std::string ascii_lower(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(),
                 [](unsigned char ch) {
                   return static_cast<char>(std::tolower(ch));
                 });
  return value;
}

std::string key_for_pd_element(const std::string& element_kind,
                               const std::string& element_name) {
  return element_kind + ":" + element_name;
}

std::string key_for_pd_element_terminal_node(
    hacdcpf::io::OpenDSSPDElementKind element_kind,
    const std::string& element_name,
    int terminal,
    int node) {
  return key_for_pd_element(hacdcpf::io::opendss_pd_element_kind_to_string(element_kind),
                            element_name) +
         ":" + std::to_string(terminal) + ":" + std::to_string(node);
}

// ---------------------------------------------------------------------------
// Build index maps from an OpenDSSSnapshotResult
// ---------------------------------------------------------------------------

struct OpenDSSMaps {
  std::map<std::string, double> vm_pu;          // "busname.node" -> vm_pu
  std::map<std::string, double> va_deg;          // "busname.node" -> va_deg
  std::map<std::string, double> vm_vln;          // "busname.node" -> V L-N
  // "kind:name:terminal:node" -> {p_mw, q_mvar}
  std::map<std::string, std::pair<double, double>> powers;
  // "kind:name" -> {p_mw, q_mvar}
  std::map<std::string, std::pair<double, double>> element_losses;
};

OpenDSSMaps build_opendss_maps(const hacdcpf::io::OpenDSSSnapshotResult& res) {
  OpenDSSMaps maps;
  for (const auto& v : res.node_voltages) {
    const auto key = key_for_bus_node(v.bus_name, v.node);
    maps.vm_pu.emplace(key, v.vm_pu);
    maps.va_deg.emplace(key, v.va_deg);
    maps.vm_vln.emplace(key, v.vm_vln);
  }
  for (const auto& element : res.pd_element_results) {
    for (const auto& pw : element.terminal_powers) {
      const auto converted = hacdcpf_compare::opendss_terminal_power_to_mw_mvar(pw);
      maps.powers.emplace(
          key_for_pd_element_terminal_node(
              element.element_kind, element.name, pw.terminal, pw.node),
          std::make_pair(converted.p_mw, converted.q_mvar));
    }
    const auto element_loss =
        hacdcpf_compare::opendss_element_losses_to_mw_mvar(element);
    maps.element_losses.emplace(
        key_for_pd_element(
            hacdcpf::io::opendss_pd_element_kind_to_string(element.element_kind),
            element.name),
        std::make_pair(element_loss.p_mw, element_loss.q_mvar));
  }
  return maps;
}

const hacdcpf::io::OpenDSSPDElementResult& find_pd_element(
    const hacdcpf::io::OpenDSSSnapshotResult& snapshot,
    hacdcpf::io::OpenDSSPDElementKind element_kind,
    const std::string& element_name) {
  for (const auto& element : snapshot.pd_element_results) {
    if (element.element_kind == element_kind && element.name == element_name) {
      return element;
    }
  }
  throw std::runtime_error(
      "Missing PD element " +
      key_for_pd_element(hacdcpf::io::opendss_pd_element_kind_to_string(element_kind),
                         element_name));
}

const hacdcpf::io::OpenDSSNodeVoltage& find_node_voltage(
    const std::map<std::string, hacdcpf::io::OpenDSSNodeVoltage>& index,
    const std::string& bus_name,
    int node) {
  const auto it = index.find(key_for_bus_node(ascii_lower(bus_name), node));
  if (it == index.end()) {
    throw std::runtime_error("Missing OpenDSS node voltage for " +
                             key_for_bus_node(ascii_lower(bus_name), node));
  }
  return it->second;
}

const hacdcpf::io::OpenDSSTerminalPower& find_terminal_power(
    const std::map<std::string, hacdcpf::io::OpenDSSTerminalPower>& index,
    hacdcpf::io::OpenDSSPDElementKind element_kind,
    const std::string& element_name,
    int terminal,
    int node) {
  const std::string key =
      key_for_pd_element_terminal_node(element_kind, element_name, terminal, node);
  const auto it = index.find(key);
  if (it == index.end()) {
    throw std::runtime_error("Missing OpenDSS terminal power for " + key);
  }
  return it->second;
}

std::map<std::string, hacdcpf::io::OpenDSSNodeVoltage> build_voltage_index(
    const hacdcpf::io::OpenDSSSnapshotResult& result) {
  std::map<std::string, hacdcpf::io::OpenDSSNodeVoltage> index;
  for (const auto& node_voltage : result.node_voltages) {
    index.emplace(key_for_bus_node(node_voltage.bus_name, node_voltage.node),
                  node_voltage);
  }
  return index;
}

std::map<std::string, hacdcpf::io::OpenDSSTerminalPower> build_terminal_power_index(
    const hacdcpf::io::OpenDSSSnapshotResult& result) {
  std::map<std::string, hacdcpf::io::OpenDSSTerminalPower> index;
  for (const auto& element : result.pd_element_results) {
    for (const auto& power : element.terminal_powers) {
      index.emplace(key_for_pd_element_terminal_node(
                        element.element_kind, element.name, power.terminal, power.node),
                    power);
    }
  }
  return index;
}

const hacdcpf::io::OpenDSSTransformerState& find_transformer_state(
    const hacdcpf::io::OpenDSSSnapshotResult& snapshot,
    const std::string& transformer_name) {
  const std::string lowered = ascii_lower(transformer_name);
  for (const auto& state : snapshot.transformer_states) {
    if (state.name == lowered) {
      return state;
    }
  }
  throw std::runtime_error("Missing transformer state for " + lowered);
}

const hacdcpf::io::OpenDSSTransformerWindingState& find_transformer_winding_state(
    const hacdcpf::io::OpenDSSTransformerState& state,
    int winding) {
  for (const auto& winding_state : state.winding_states) {
    if (winding_state.winding == winding) {
      return winding_state;
    }
  }
  throw std::runtime_error("Missing winding state for transformer " + state.name +
                           " winding " + std::to_string(winding));
}

const hacdcpf::io::OpenDSSRegControlResult& find_regcontrol(
    const hacdcpf::io::OpenDSSSnapshotResult& snapshot,
    const std::string& regcontrol_name) {
  const std::string lowered = ascii_lower(regcontrol_name);
  for (const auto& control : snapshot.regcontrol_results) {
    if (control.name == lowered) {
      return control;
    }
  }
  throw std::runtime_error("Missing RegControl state for " + lowered);
}

int find_branch_position_by_index(const hacdcpf::ACSystem& ac, int branch_index) {
  for (std::size_t pos = 0; pos < ac.branches.size(); ++pos) {
    if (ac.branches[pos].index == branch_index) {
      return static_cast<int>(pos);
    }
  }
  throw std::runtime_error("Missing projected branch index " +
                           std::to_string(branch_index));
}

int find_branch_position_by_name(const hacdcpf::ACSystem& ac,
                                 const std::string& branch_name) {
  const std::string lowered = ascii_lower(branch_name);
  for (std::size_t pos = 0; pos < ac.branches.size(); ++pos) {
    if (ascii_lower(ac.branches[pos].name) == lowered) {
      return static_cast<int>(pos);
    }
  }
  throw std::runtime_error("Missing projected branch named " + lowered);
}

int find_projected_branch_position(const hacdcpf::HybridPowerSystem& projected,
                                   hacdcpf::BranchOriginType origin_type,
                                   int origin_index) {
  if (!projected.branch_expand_map.has_value()) {
    throw std::runtime_error("Projected system is missing BranchExpandMap.");
  }
  for (const auto& entry : projected.branch_expand_map->entries) {
    if (entry.origin_type == origin_type && entry.origin_index == origin_index) {
      return find_branch_position_by_index(projected.ac, entry.branch_index);
    }
  }
  throw std::runtime_error("Missing projected branch mapping for origin index " +
                           std::to_string(origin_index));
}

int find_projected_branch_position_by_pair(
    const hacdcpf::HybridPowerSystem& projected,
    int transformer3w_index,
    int pair_number) {
  if (!projected.branch_expand_map.has_value()) {
    throw std::runtime_error("Projected system is missing BranchExpandMap.");
  }
  for (const auto& entry : projected.branch_expand_map->entries) {
    if (entry.origin_type == hacdcpf::BranchOriginType::Transformer3W &&
        entry.origin_index == transformer3w_index &&
        entry.pair_number == pair_number) {
      return find_branch_position_by_index(projected.ac, entry.branch_index);
    }
  }
  throw std::runtime_error("Missing projected Transformer3W pair mapping for origin index " +
                           std::to_string(transformer3w_index) + " pair " +
                           std::to_string(pair_number));
}

fs::path current_executable_path() {
#ifdef _WIN32
  char buffer[MAX_PATH];
  const DWORD length = GetModuleFileNameA(nullptr, buffer, MAX_PATH);
  if (length == 0 || length == MAX_PATH) {
    throw std::runtime_error("GetModuleFileNameA failed");
  }
  return fs::path(buffer);
#else
  return fs::read_symlink("/proc/self/exe");
#endif
}

fs::path sibling_compare_tool_path() {
  const fs::path exe_dir = current_executable_path().parent_path();
#ifdef _WIN32
  const fs::path local = exe_dir / "opendss_pf_compare.exe";
  if (fs::exists(local)) {
    return local;
  }
  const fs::path parent = exe_dir.parent_path() / "opendss_pf_compare.exe";
  if (fs::exists(parent)) {
    return parent;
  }
  return exe_dir.parent_path().parent_path() / "Release/opendss_pf_compare.exe";
#else
  return exe_dir / "opendss_pf_compare";
#endif
}

fs::path sibling_snapshot_adapter_path() {
  const fs::path exe_dir = current_executable_path().parent_path();
#ifdef _WIN32
  const fs::path local = exe_dir / "opendss_snapshot_adapter.exe";
  if (fs::exists(local)) {
    return local;
  }
  const fs::path parent = exe_dir.parent_path() / "opendss_snapshot_adapter.exe";
  if (fs::exists(parent)) {
    return parent;
  }
  return exe_dir.parent_path().parent_path() / "Release/opendss_snapshot_adapter.exe";
#else
  return exe_dir / "opendss_snapshot_adapter";
#endif
}

std::string shell_quote(const fs::path& path) {
  return "\"" + path.string() + "\"";
}

std::string shell_exe(const fs::path& path) {
#ifdef _WIN32
  return path.string();
#else
  return shell_quote(path);
#endif
}

struct ScopedTempReportDir {
  fs::path path;

  explicit ScopedTempReportDir(std::string_view suffix) {
    path = fs::temp_directory_path() /
           ("hacdcpf_opendss_compare_" + std::string(suffix) + "_" +
            std::to_string(static_cast<long long>(current_process_id())));
    std::error_code ec;
    fs::remove_all(path, ec);
    fs::create_directories(path);
  }

  ~ScopedTempReportDir() {
    std::error_code ec;
    fs::remove_all(path, ec);
  }
};

int decode_exit_code(int raw_status) {
  if (raw_status == -1) {
    return -1;
  }
#ifdef _WIN32
  return raw_status;
#else
  if (!WIFEXITED(raw_status)) {
    return -1;
  }
  return WEXITSTATUS(raw_status);
#endif
}

json read_json_file(const fs::path& path) {
  std::ifstream is(path);
  if (!is) {
    throw std::runtime_error("Failed to open JSON file: " + path.string());
  }
  json value;
  is >> value;
  return value;
}

std::string read_text_file(const fs::path& path) {
  std::ifstream is(path);
  if (!is) {
    throw std::runtime_error("Failed to open text file: " + path.string());
  }
  return std::string(std::istreambuf_iterator<char>(is),
                     std::istreambuf_iterator<char>());
}

struct RuntimeRegulatorConfig {
  std::string circuit_name;
  int winding{2};
  int tap_winding{2};
  int transformer_tap_side{1};
  int monitored_bus{3};  // 0 => local winding bus
  std::string monitored_bus_name{"bus3"};
  double vreg_volts{122.0};
  double band_volts{2.0};
  double ptratio{60.0};
  double remote_ptratio{60.0};
  double ct_primary_amps{300.0};
  double r_volts{0.0};
  double x_volts{0.0};
  int max_tap_change{1};
  bool reversible{false};
};

struct RuntimeRegulatorScenario {
  std::string scenario_id;
  RuntimeRegulatorConfig config;
  bool expect_control_converged{true};
  bool expect_dss_converged{true};
  bool expect_remote_bus{false};
  bool expect_line_drop_compensation{false};
  bool compare_direct_control_voltage{false};
  std::string expected_stop_reason{"within_band"};
};

std::string runtime_regulator_monitored_bus_name(const RuntimeRegulatorConfig& cfg) {
  if (cfg.monitored_bus != 0) {
    return cfg.monitored_bus_name;
  }
  return (cfg.winding == 1) ? "sourcebus" : "bus2";
}

hacdcpf::HybridPowerSystem build_runtime_regulator_repo_case(
    const RuntimeRegulatorConfig& cfg) {
  auto sys = hacdcpf_compare_fixtures::build_minimal_regulator_1ph();
  auto& tr = sys.ac.transformers_2w.front();
  tr.tap_side = cfg.transformer_tap_side;

  auto& reg = sys.ac.regulator_controls.front();
  reg.winding = cfg.winding;
  reg.tap_winding = cfg.tap_winding;
  reg.monitored_bus = cfg.monitored_bus;
  reg.vreg_volts = cfg.vreg_volts;
  reg.band_volts = cfg.band_volts;
  reg.ptratio = cfg.ptratio;
  reg.remote_ptratio = cfg.remote_ptratio;
  reg.ct_primary_amps = cfg.ct_primary_amps;
  reg.r_volts = cfg.r_volts;
  reg.x_volts = cfg.x_volts;
  reg.max_tap_change = cfg.max_tap_change;
  reg.reversible = cfg.reversible;
  return sys;
}

void write_runtime_regulator_master_dss(const fs::path& master,
                                        const RuntimeRegulatorConfig& cfg) {
  std::ofstream os(master);
  if (!os.good()) {
    throw std::runtime_error("Failed to write runtime regulator case: " + master.string());
  }

  os << "Clear\n"
        "New Circuit." << cfg.circuit_name << " phases=1 bus1=sourcebus basekv=7.2 pu=1.0\n\n"
        "New Transformer.t12 phases=1 buses=[sourcebus.1, bus2.1] conns=[wye, wye] "
        "kvas=[10000, 10000] kvs=[7.2, 7.2] xhl=6.0 %loadloss=2.0 %noloadloss=0 %imag=0 "
        "taps=[1.0, 1.0]\n\n"
        "New Line.l1 phases=1 bus1=bus2.1 bus2=bus3.1 length=1 units=none "
        "r1=0.41472 x1=0.7776 r0=0.41472 x0=0.7776 c1=0 c0=0\n\n"
        "New Load.load3 phases=1 bus1=bus3.1 conn=wye model=1 kv=7.2 kw=1600 kvar=900 "
        "vminpu=0.0 vmaxpu=2.0\n\n"
        "New RegControl.reg1 transformer=t12 "
     << "winding=" << cfg.winding << " "
     << "tapwinding=" << cfg.tap_winding << " ";
  if (cfg.monitored_bus != 0) {
    os << "bus=" << cfg.monitored_bus_name << ".1 ";
  }
  os << "vreg=" << cfg.vreg_volts << " "
     << "band=" << cfg.band_volts << " "
     << "ptratio=" << cfg.ptratio << " "
     << "remoteptratio=" << cfg.remote_ptratio << " "
     << "ctprim=" << cfg.ct_primary_amps << " "
     << "R=" << cfg.r_volts << " "
     << "X=" << cfg.x_volts << " "
     << "maxtapchange=" << cfg.max_tap_change << " "
     << "reversible=" << (cfg.reversible ? "yes" : "no") << "\n\n"
     << "Set ControlMode=Static\n"
     << "Set MaxControlIter=100\n"
     << "Set VoltageBases=[12.470765814495916]\n"
     << "CalcVoltageBases\n"
     << "Solve\n";
}

std::pair<std::string, int> parse_terminal_bus_name(const std::string& terminal_bus_name) {
  const auto dot_pos = terminal_bus_name.find('.');
  if (dot_pos == std::string::npos) {
    return {ascii_lower(terminal_bus_name), 1};
  }
  const std::string bus_name =
      ascii_lower(terminal_bus_name.substr(0, dot_pos));
  const auto next_dot_pos = terminal_bus_name.find('.', dot_pos + 1);
  const std::string node_text = terminal_bus_name.substr(
      dot_pos + 1,
      (next_dot_pos == std::string::npos) ? std::string::npos
                                          : next_dot_pos - dot_pos - 1);
  return {bus_name, std::stoi(node_text)};
}

Complex node_voltage_complex_volts(const hacdcpf::io::OpenDSSNodeVoltage& voltage) {
  return std::polar(voltage.vm_vln, voltage.va_deg * kPi / 180.0);
}

double compute_dss_regulator_control_voltage_volts(
    const std::map<std::string, hacdcpf::io::OpenDSSNodeVoltage>& voltage_index,
    const std::map<std::string, hacdcpf::io::OpenDSSTerminalPower>& power_index,
    const hacdcpf::io::OpenDSSPDElementResult& transformer,
    const hacdcpf::io::OpenDSSRegControlResult& regcontrol,
    const std::string& monitored_bus_name,
    int monitored_bus_node,
    bool implicit_local_bus_monitoring) {
  const auto& monitored_voltage = find_node_voltage(
      voltage_index, monitored_bus_name, monitored_bus_node);
  const Complex monitored_voltage_complex = node_voltage_complex_volts(monitored_voltage);
  const double effective_ptratio =
      (!implicit_local_bus_monitoring && regcontrol.remote_ptratio > 0.0)
          ? regcontrol.remote_ptratio
          : regcontrol.ptratio;
  Complex ldc_term{0.0, 0.0};
  if (std::abs(regcontrol.forward_r_volts) > 1e-12 ||
      std::abs(regcontrol.forward_x_volts) > 1e-12) {
    const auto [tap_bus_name, tap_bus_node] = parse_terminal_bus_name(
        transformer.terminal_bus_names.at(static_cast<std::size_t>(regcontrol.tap_winding - 1)));
    const auto& tap_winding_voltage = find_node_voltage(
        voltage_index, tap_bus_name, tap_bus_node);
    const Complex tap_winding_voltage_complex =
        node_voltage_complex_volts(tap_winding_voltage);
    const auto& tap_terminal_power = find_terminal_power(
        power_index, hacdcpf::io::OpenDSSPDElementKind::Transformer,
        transformer.name, regcontrol.tap_winding, tap_bus_node);
    const auto tap_terminal_power_mw_mvar =
        hacdcpf_compare::opendss_terminal_power_to_mw_mvar(tap_terminal_power.power_kw_kvar);
    const Complex tap_terminal_power_mva(
        tap_terminal_power_mw_mvar.p_mw, tap_terminal_power_mw_mvar.q_mvar);
    const Complex tap_current_amps = std::conj(
        (tap_terminal_power_mva * 1.0e6) / tap_winding_voltage_complex);
    ldc_term = (tap_current_amps / regcontrol.ct_primary_amps) *
               Complex(regcontrol.forward_r_volts, regcontrol.forward_x_volts);
  }
  const Complex control_voltage =
      monitored_voltage_complex / effective_ptratio + ldc_term;
  return std::abs(control_voltage);
}

const hacdcpf::analysis::RegulatorControlState& get_single_regulator_state(
    const hacdcpf::analysis::DPFResult& result) {
  if (result.regulator_states.size() != 1) {
    throw std::runtime_error("Expected exactly one regulator state.");
  }
  return result.regulator_states.front();
}

const hacdcpf::analysis::RegulatorControlTraceEntry& get_last_regulator_trace(
    const hacdcpf::analysis::DPFResult& result) {
  if (result.regulator_trace.empty()) {
    throw std::runtime_error("Expected non-empty regulator trace.");
  }
  return result.regulator_trace.back();
}

int phase_index_from_node(int node) {
  switch (node) {
    case 1:
      return 0;
    case 2:
      return 1;
    case 3:
      return 2;
    default:
      throw std::runtime_error("Unsupported three-phase node " + std::to_string(node));
  }
}

Complex phase_from_observation(
    const hacdcpf::analysis::ThreePhasePhasorObservation& phasor,
    int phase_index) {
  return {
      phasor.real[static_cast<std::size_t>(phase_index)],
      phasor.imag[static_cast<std::size_t>(phase_index)],
  };
}

const hacdcpf::analysis::ThreePhaseRegulatorControlState& find_three_phase_regulator_state(
    const hacdcpf::analysis::ThreePhaseDPFResult& result,
    const std::string& regulator_name) {
  const std::string lowered = ascii_lower(regulator_name);
  for (const auto& state : result.regulator_states) {
    if (ascii_lower(state.regulator_name) == lowered) {
      return state;
    }
  }
  throw std::runtime_error("Missing three-phase regulator state for " + lowered);
}

const hacdcpf::analysis::ThreePhaseTransformerTerminalObservation&
find_three_phase_transformer_observation(
    const hacdcpf::analysis::ThreePhaseDPFResult& result,
    int transformer_index) {
  for (const auto& observation : result.transformer_terminal_observations) {
    if (observation.transformer_index == transformer_index) {
      return observation;
    }
  }
  throw std::runtime_error(
      "Missing three-phase transformer observation for transformer index " +
      std::to_string(transformer_index));
}

const hacdcpf::ThreePhaseACBus& find_three_phase_bus(
    const hacdcpf::ThreePhaseACSystem& sys,
    int bus_index) {
  for (const auto& bus : sys.buses) {
    if (bus.index == bus_index) {
      return bus;
    }
  }
  throw std::runtime_error("Missing three-phase bus index " +
                           std::to_string(bus_index));
}

hacdcpf_compare::PowerPair transformer_terminal_power_from_observation(
    const hacdcpf::analysis::ThreePhaseTransformerTerminalObservation& observation,
    double base_voltage_volts,
    int phase_index,
    bool hv_side) {
  const Complex voltage_pu = phase_from_observation(
      hv_side ? observation.hv_voltage_pu : observation.lv_voltage_pu,
      phase_index);
  const Complex current_amps = phase_from_observation(
      hv_side ? observation.hv_current_amps : observation.lv_current_amps,
      phase_index);
  const Complex terminal_voltage_volts = voltage_pu * base_voltage_volts;
  const Complex power_mva =
      terminal_voltage_volts * std::conj(current_amps) / 1.0e6;
  return {
      .p_mw = power_mva.real(),
      .q_mvar = power_mva.imag(),
  };
}

void verify_regulator_case_matches_repository(
    const hacdcpf_compare_fixtures::RegulatorOpenDSSCase& spec,
    const hacdcpf::HybridPowerSystem& repo_case) {
  using namespace hacdcpf::analysis;

  DPFOptions options;
  options.max_iter = 200;
  options.max_control_iter = 100;
  options.tol = 1e-10;
  options.include_shunts = true;
  const auto repo = solve_distribution_pf(repo_case, options);
  REQUIRE(repo.converged);
  REQUIRE(repo.control_converged);
  REQUIRE(repo.regulator_states.size() == 1);
  REQUIRE(repo.regulator_trace.empty() == false);

  const auto& repo_reg = repo.regulator_states.front();
  CHECK(repo_reg.final_tap_number == spec.expected_final_tap_number);
  CHECK(std::abs(repo_reg.final_tap_pu - spec.expected_final_tap_pu) < 1e-12);
  CHECK(repo_reg.stop_reason == "within_band");
  CHECK(repo_reg.used_remote_bus == !spec.implicit_local_bus_monitoring);
  CHECK(repo_reg.used_line_drop_compensation ==
        (std::abs(spec.forward_r_volts) > 1e-12 ||
         std::abs(spec.forward_x_volts) > 1e-12));

  auto final_repo_case = repo_case;
  final_repo_case.ac.transformers_2w.front().tap_pos = repo_reg.final_tap_pos;
  const auto projected = hacdcpf::project_to_canonical_models(final_repo_case);

#ifdef HACDCPF_PROJECT_ROOT
  const fs::path master =
      fs::path(HACDCPF_PROJECT_ROOT) / spec.relative_master_dss;
#else
  const fs::path master = spec.relative_master_dss;
#endif

  const auto dss = hacdcpf::io::solve_opendss_snapshot(master);
  REQUIRE(dss.converged);
  REQUIRE(dss.transformer_states.empty() == false);
  REQUIRE(dss.regcontrol_results.empty() == false);

  const auto maps = build_opendss_maps(dss);
  const auto voltage_index = build_voltage_index(dss);
  const auto power_index = build_terminal_power_index(dss);
  const auto& xfmr = find_pd_element(
      dss, hacdcpf::io::OpenDSSPDElementKind::Transformer, spec.transformer_name);
  const auto& line = find_pd_element(
      dss, hacdcpf::io::OpenDSSPDElementKind::Line, spec.line_name);
  const auto& transformer_state = find_transformer_state(dss, spec.transformer_name);
  const auto& tap_winding = find_transformer_winding_state(transformer_state, spec.tap_winding);
  const auto& regcontrol = find_regcontrol(dss, spec.regcontrol_name);

  CHECK(regcontrol.transformer_name == spec.transformer_name);
  CHECK(regcontrol.winding == spec.winding);
  CHECK(regcontrol.tap_winding == spec.tap_winding);
  CHECK(regcontrol.tap_number == spec.expected_final_tap_number);
  CHECK(regcontrol.max_tap_change == spec.max_tap_change);
  CHECK(regcontrol.forward_vreg_volts == spec.forward_vreg_volts);
  CHECK(regcontrol.forward_band_volts == spec.forward_band_volts);
  CHECK(regcontrol.ptratio == spec.ptratio);
  CHECK(regcontrol.remote_ptratio == spec.remote_ptratio);
  CHECK(regcontrol.ct_primary_amps == spec.ct_primary_amps);
  CHECK(regcontrol.forward_r_volts == spec.forward_r_volts);
  CHECK(regcontrol.forward_x_volts == spec.forward_x_volts);
  CHECK(std::abs(tap_winding.tap_pu - spec.expected_final_tap_pu) < 1e-12);

  const std::string monitored_key =
      key_for_bus_node(spec.monitored_bus_name, spec.monitored_bus_node);
  if (spec.implicit_local_bus_monitoring) {
    CHECK((regcontrol.monitored_bus_name.empty() ||
           regcontrol.monitored_bus_name == monitored_key));
  } else {
    CHECK(regcontrol.monitored_bus_name == monitored_key);
  }

  const int transformer_branch_pos = find_projected_branch_position(
      projected, hacdcpf::BranchOriginType::Transformer2W,
      spec.transformer_origin_index);
  const int line_branch_pos = find_branch_position_by_name(projected.ac, spec.line_name);
  CHECK(std::abs(projected.ac.branches[static_cast<std::size_t>(transformer_branch_pos)].tap -
                 (1.0 / spec.expected_final_tap_pu)) < 1e-12);

  CHECK(std::abs(maps.vm_pu.at("sourcebus.1") - repo.vm_pu[0]) < 1e-3);
  CHECK(std::abs(maps.vm_pu.at("bus2.1") - repo.vm_pu[1]) < 1e-3);
  CHECK(std::abs(maps.vm_pu.at("bus3.1") - repo.vm_pu[2]) < 1e-3);

  CHECK(std::abs(maps.va_deg.at("sourcebus.1") - repo.va_deg[0]) < 0.1);
  CHECK(std::abs(maps.va_deg.at("bus2.1") - repo.va_deg[1]) < 0.1);
  CHECK(std::abs(maps.va_deg.at("bus3.1") - repo.va_deg[2]) < 0.1);

  const double dss_control_voltage = compute_dss_regulator_control_voltage_volts(
      voltage_index, power_index, xfmr, regcontrol, spec.monitored_bus_name,
      spec.monitored_bus_node, spec.implicit_local_bus_monitoring);
  CHECK(std::abs(dss_control_voltage - repo_reg.control_voltage_volts) < 0.2);
  CHECK(std::abs(static_cast<double>(regcontrol.tap_number - repo_reg.final_tap_number)) < 1e-12);

  REQUIRE(maps.powers.contains(
      key_for_transformer_terminal_node(spec.transformer_name, 1, 1)));
  CHECK(std::abs(
            maps.powers.at(
                key_for_transformer_terminal_node(spec.transformer_name, 1, 1)).first -
            repo.p_branch_mw[static_cast<std::size_t>(transformer_branch_pos)]) < 1e-3);
  CHECK(std::abs(
            maps.powers.at(
                key_for_transformer_terminal_node(spec.transformer_name, 1, 1)).second -
            repo.q_branch_mvar[static_cast<std::size_t>(transformer_branch_pos)]) < 1e-3);

  REQUIRE(maps.powers.contains(key_for_line_terminal_node(spec.line_name, 1, 1)));
  CHECK(std::abs(maps.powers.at(key_for_line_terminal_node(spec.line_name, 1, 1)).first -
                 repo.p_branch_mw[static_cast<std::size_t>(line_branch_pos)]) < 1e-3);
  CHECK(std::abs(maps.powers.at(key_for_line_terminal_node(spec.line_name, 1, 1)).second -
                 repo.q_branch_mvar[static_cast<std::size_t>(line_branch_pos)]) < 1e-3);

  const auto xfmr_terminal_sum =
      hacdcpf_compare::sum_pd_element_terminal_powers_to_mw_mvar(xfmr);
  const auto xfmr_losses =
      hacdcpf_compare::opendss_element_losses_to_mw_mvar(xfmr);
  CHECK(std::abs(xfmr_terminal_sum.p_mw - xfmr_losses.p_mw) < 1e-6);
  CHECK(std::abs(xfmr_terminal_sum.q_mvar - xfmr_losses.q_mvar) < 1e-6);

  const auto line_terminal_sum =
      hacdcpf_compare::sum_pd_element_terminal_powers_to_mw_mvar(line);
  const auto line_losses =
      hacdcpf_compare::opendss_element_losses_to_mw_mvar(line);
  CHECK(std::abs(line_terminal_sum.p_mw - line_losses.p_mw) < 1e-6);
  CHECK(std::abs(line_terminal_sum.q_mvar - line_losses.q_mvar) < 1e-6);

  const auto total_branch_losses =
      hacdcpf_compare::sum_all_pd_element_terminal_powers_to_mw_mvar(dss);
  CHECK(std::abs(total_branch_losses.p_mw - repo.total_p_loss_mw) < 1e-3);
  CHECK(std::abs(total_branch_losses.q_mvar - repo.total_q_loss_mvar) < 1e-3);

  const auto circuit_losses =
      hacdcpf_compare::opendss_circuit_losses_to_mw_mvar(dss);
  CHECK(std::abs(total_branch_losses.p_mw - circuit_losses.p_mw) < 1e-6);
  CHECK(std::abs(total_branch_losses.q_mvar - circuit_losses.q_mvar) < 1e-6);
}

void verify_three_phase_regulator_bank_case_matches_repository(
    const hacdcpf_compare_fixtures::ThreePhaseRegulatorBankOpenDSSCase& spec,
    const hacdcpf::ThreePhaseACSystem& repo_case) {
  using namespace hacdcpf::analysis;

  ThreePhaseNROptions options;
  options.max_iter = 100;
  options.max_control_iter = 100;
  options.tol = 1e-10;
  const auto repo = solve_three_phase_nr(repo_case, options);
  REQUIRE(repo.converged);
  REQUIRE(repo.control_converged);
  REQUIRE(repo.regulator_states.size() == spec.regulators.size());
  REQUIRE(repo.transformer_terminal_observations.size() == spec.regulators.size());

#ifdef HACDCPF_PROJECT_ROOT
  const fs::path master =
      fs::path(HACDCPF_PROJECT_ROOT) / spec.relative_master_dss;
#else
  const fs::path master = spec.relative_master_dss;
#endif

  const auto dss = hacdcpf::io::solve_opendss_snapshot(master);
  REQUIRE(dss.converged);
  REQUIRE(dss.control_oracle.control_iterations == 3);

  const auto voltage_index = build_voltage_index(dss);
  const auto power_index = build_terminal_power_index(dss);

  REQUIRE(spec.bus_names.size() == 2);
  for (std::size_t bus_pos = 0; bus_pos < spec.bus_names.size(); ++bus_pos) {
    const auto& repo_bus = repo.bus_voltages[bus_pos];
    CHECK(std::abs(find_node_voltage(voltage_index, spec.bus_names[bus_pos], 1).vm_pu -
                   repo_bus.vm_a_pu) < 1e-3);
    CHECK(std::abs(find_node_voltage(voltage_index, spec.bus_names[bus_pos], 2).vm_pu -
                   repo_bus.vm_b_pu) < 1e-3);
    CHECK(std::abs(find_node_voltage(voltage_index, spec.bus_names[bus_pos], 3).vm_pu -
                   repo_bus.vm_c_pu) < 1e-3);
    CHECK(std::abs(find_node_voltage(voltage_index, spec.bus_names[bus_pos], 1).va_deg -
                   repo_bus.va_a_deg) < 0.1);
    CHECK(std::abs(find_node_voltage(voltage_index, spec.bus_names[bus_pos], 2).va_deg -
                   repo_bus.va_b_deg) < 0.1);
    CHECK(std::abs(find_node_voltage(voltage_index, spec.bus_names[bus_pos], 3).va_deg -
                   repo_bus.va_c_deg) < 0.1);
  }

  for (const auto& regulator_spec : spec.regulators) {
    const auto& repo_regulator =
        find_three_phase_regulator_state(repo, regulator_spec.regcontrol_name);
    const auto& repo_observation =
        find_three_phase_transformer_observation(
            repo, regulator_spec.transformer_index);
    const auto& repo_hv_bus =
        find_three_phase_bus(repo_case, repo_observation.hv_bus);
    const int phase_index =
        phase_index_from_node(regulator_spec.monitored_bus_node);
    const auto repo_terminal_power = transformer_terminal_power_from_observation(
        repo_observation, repo_hv_bus.base_kv * 1000.0, phase_index, true);

    const auto& transformer =
        find_pd_element(
            dss, hacdcpf::io::OpenDSSPDElementKind::Transformer,
            regulator_spec.transformer_name);
    const auto& transformer_state =
        find_transformer_state(dss, regulator_spec.transformer_name);
    const auto& tap_winding =
        find_transformer_winding_state(transformer_state, regulator_spec.tap_winding);
    const auto& regcontrol =
        find_regcontrol(dss, regulator_spec.regcontrol_name);
    const auto dss_control_voltage = compute_dss_regulator_control_voltage_volts(
        voltage_index, power_index, transformer, regcontrol,
        regulator_spec.monitored_bus_name, regulator_spec.monitored_bus_node,
        regulator_spec.implicit_local_bus_monitoring);
    const auto& dss_terminal_power = find_terminal_power(
        power_index, hacdcpf::io::OpenDSSPDElementKind::Transformer,
        regulator_spec.transformer_name, 1, regulator_spec.monitored_bus_node);
    const auto dss_terminal_power_mw_mvar =
        hacdcpf_compare::opendss_terminal_power_to_mw_mvar(dss_terminal_power);

    CHECK(repo_regulator.final_tap_number ==
          regulator_spec.expected_final_tap_number);
    CHECK(std::abs(repo_regulator.final_tap_pu -
                   regulator_spec.expected_final_tap_pu) < 1e-12);
    CHECK(repo_regulator.stop_reason == "within_band");
    CHECK(repo_regulator.used_remote_bus == false);
    CHECK(repo_regulator.used_line_drop_compensation == false);

    CHECK(regcontrol.transformer_name == regulator_spec.transformer_name);
    CHECK(regcontrol.winding == regulator_spec.winding);
    CHECK(regcontrol.tap_winding == regulator_spec.tap_winding);
    CHECK(regcontrol.tap_number == regulator_spec.expected_final_tap_number);
    CHECK(regcontrol.max_tap_change == regulator_spec.max_tap_change);
    CHECK(regcontrol.forward_vreg_volts == regulator_spec.forward_vreg_volts);
    CHECK(regcontrol.forward_band_volts == regulator_spec.forward_band_volts);
    CHECK(regcontrol.ptratio == regulator_spec.ptratio);
    CHECK(regcontrol.remote_ptratio == regulator_spec.remote_ptratio);
    CHECK(std::abs(tap_winding.tap_pu -
                   regulator_spec.expected_final_tap_pu) < 1e-12);

    CHECK((regcontrol.monitored_bus_name.empty() ||
           regcontrol.monitored_bus_name ==
               key_for_bus_node(
                   regulator_spec.monitored_bus_name,
                   regulator_spec.monitored_bus_node)));
    CHECK(std::abs(dss_control_voltage -
                   repo_regulator.control_voltage_volts) < 0.2);
    CHECK(std::abs(
              dss_terminal_power_mw_mvar.p_mw - repo_terminal_power.p_mw) < 1e-6);
    CHECK(std::abs(
              dss_terminal_power_mw_mvar.q_mvar - repo_terminal_power.q_mvar) < 1e-6);
  }

  const auto dss_total_losses =
      hacdcpf_compare::sum_all_pd_element_terminal_powers_to_mw_mvar(dss);
  CHECK(std::abs(dss_total_losses.p_mw - repo.total_p_loss_mw) < 1e-6);
  CHECK(std::abs(dss_total_losses.q_mvar - repo.total_q_loss_mvar) < 1e-6);
}

Complex polar_pu(double magnitude, double angle_deg) {
  return std::polar(magnitude, angle_deg * kPi / 180.0);
}

enum class PhysicalTapPlacement {
  None,
  FromSide,
  ToSide,
};

struct BranchEquationSnapshot {
  Complex child_voltage;
  Complex current_from_bus_into_branch;
  Complex current_to_bus_into_branch;
  Complex power_from_mva;
  Complex power_to_mva;
  Complex loss_mva;
};

BranchEquationSnapshot projected_branch_equations_from_parent(
    const hacdcpf::ACBranch& branch,
    Complex parent_voltage,
    Complex child_bus_current,
    double base_mva) {
  const Complex tap = std::polar(
      (std::abs(branch.tap) < 1e-12) ? 1.0 : branch.tap,
      branch.shift_deg * kPi / 180.0);
  const Complex z(branch.r_pu, branch.x_pu);
  const Complex child_voltage = parent_voltage / tap - z * child_bus_current;
  const Complex current_from = child_bus_current / std::conj(tap);
  const Complex current_to = -child_bus_current;
  const Complex power_from = parent_voltage * std::conj(current_from) * base_mva;
  const Complex power_to = child_voltage * std::conj(current_to) * base_mva;
  return {
      .child_voltage = child_voltage,
      .current_from_bus_into_branch = current_from,
      .current_to_bus_into_branch = current_to,
      .power_from_mva = power_from,
      .power_to_mva = power_to,
      .loss_mva = power_from + power_to,
  };
}

BranchEquationSnapshot physical_transformer_equations_from_parent(
    double r_pu,
    double x_pu,
    PhysicalTapPlacement tap_placement,
    double winding_tap_pu,
    Complex parent_voltage,
    Complex child_bus_current,
    double base_mva) {
  const Complex z(r_pu, x_pu);
  const double tap_mag = std::max(1e-12, winding_tap_pu);
  Complex child_voltage;
  Complex current_from;

  switch (tap_placement) {
    case PhysicalTapPlacement::None:
      child_voltage = parent_voltage - z * child_bus_current;
      current_from = child_bus_current;
      break;
    case PhysicalTapPlacement::FromSide:
      child_voltage = parent_voltage / tap_mag - z * child_bus_current;
      current_from = child_bus_current / tap_mag;
      break;
    case PhysicalTapPlacement::ToSide:
      child_voltage = tap_mag * (parent_voltage - z * tap_mag * child_bus_current);
      current_from = tap_mag * child_bus_current;
      break;
  }

  const Complex current_to = -child_bus_current;
  const Complex power_from = parent_voltage * std::conj(current_from) * base_mva;
  const Complex power_to = child_voltage * std::conj(current_to) * base_mva;
  return {
      .child_voltage = child_voltage,
      .current_from_bus_into_branch = current_from,
      .current_to_bus_into_branch = current_to,
      .power_from_mva = power_from,
      .power_to_mva = power_to,
      .loss_mva = power_from + power_to,
  };
}

double max_abs_branch_equation_delta(const BranchEquationSnapshot& lhs,
                                     const BranchEquationSnapshot& rhs) {
  return std::max({
      std::abs(lhs.child_voltage - rhs.child_voltage),
      std::abs(lhs.current_from_bus_into_branch - rhs.current_from_bus_into_branch),
      std::abs(lhs.current_to_bus_into_branch - rhs.current_to_bus_into_branch),
      std::abs(lhs.power_from_mva - rhs.power_from_mva),
      std::abs(lhs.power_to_mva - rhs.power_to_mva),
      std::abs(lhs.loss_mva - rhs.loss_mva),
  });
}

std::pair<double, double> rx_from_vk_vkr(double vk_percent,
                                         double vkr_percent,
                                         double base_mva,
                                         double sn_mva) {
  const double scale = base_mva / sn_mva;
  const double z = std::max(0.0, vk_percent / 100.0) * scale;
  const double r = std::max(0.0, vkr_percent / 100.0) * scale;
  return {r, std::sqrt(std::max(0.0, z * z - r * r))};
}

std::array<std::array<Complex, 3>, 3> projected_transformer3w_pair_ybus(
    const hacdcpf::HybridPowerSystem& projected,
    int transformer_index,
    const hacdcpf::Transformer3W& transformer) {
  std::array<std::array<Complex, 3>, 3> ybus{};
  auto winding_pos = [&](int bus_id) {
    if (bus_id == transformer.hv_bus) return 0;
    if (bus_id == transformer.mv_bus) return 1;
    if (bus_id == transformer.lv_bus) return 2;
    throw std::runtime_error("Unexpected Transformer3W bus in projected branch.");
  };

  for (int pair_number = 0; pair_number < 3; ++pair_number) {
    const int branch_pos = find_projected_branch_position_by_pair(
        projected, transformer_index, pair_number);
    const auto& branch =
        projected.ac.branches[static_cast<std::size_t>(branch_pos)];
    const int from = winding_pos(branch.from_bus);
    const int to = winding_pos(branch.to_bus);
    const Complex z(branch.r_pu, branch.x_pu);
    const Complex y = Complex{1.0, 0.0} / z;
    const Complex tau = std::polar(
        (std::abs(branch.tap) < 1e-12) ? 1.0 : branch.tap,
        branch.shift_deg * kPi / 180.0);
    ybus[static_cast<std::size_t>(from)][static_cast<std::size_t>(from)] +=
        y / (tau * std::conj(tau));
    ybus[static_cast<std::size_t>(from)][static_cast<std::size_t>(to)] +=
        -y / std::conj(tau);
    ybus[static_cast<std::size_t>(to)][static_cast<std::size_t>(from)] +=
        -y / tau;
    ybus[static_cast<std::size_t>(to)][static_cast<std::size_t>(to)] += y;
  }
  return ybus;
}

std::array<std::array<Complex, 3>, 3> coupled_kron_transformer3w_reference_ybus(
    const hacdcpf::Transformer3W& transformer,
    double base_mva) {
  const auto hm = rx_from_vk_vkr(
      transformer.vk_hv_mv_percent, transformer.vkr_hv_mv_percent, base_mva,
      std::max(1e-9, std::min(transformer.sn_hv_mva, transformer.sn_mv_mva)));
  const auto hl = rx_from_vk_vkr(
      transformer.vk_hv_lv_percent, transformer.vkr_hv_lv_percent, base_mva,
      std::max(1e-9, std::min(transformer.sn_hv_mva, transformer.sn_lv_mva)));
  const auto ml = rx_from_vk_vkr(
      transformer.vk_mv_lv_percent, transformer.vkr_mv_lv_percent, base_mva,
      std::max(1e-9, std::min(transformer.sn_mv_mva, transformer.sn_lv_mva)));
  const std::array<Complex, 3> star_impedances = {
      0.5 * (Complex{hm.first, hm.second} + Complex{hl.first, hl.second} -
             Complex{ml.first, ml.second}),
      0.5 * (Complex{hm.first, hm.second} + Complex{ml.first, ml.second} -
             Complex{hl.first, hl.second}),
      0.5 * (Complex{hl.first, hl.second} + Complex{ml.first, ml.second} -
             Complex{hm.first, hm.second}),
  };
  const double winding_tap_pu =
      std::max(1e-6, 1.0 + transformer.tap_pos * transformer.tap_step_percent / 100.0);
  const std::array<double, 3> leg_taps = {
      transformer.tap_side == 0 ? winding_tap_pu : 1.0,
      transformer.tap_side == 1 ? winding_tap_pu : 1.0,
      transformer.tap_side == 2 ? winding_tap_pu : 1.0,
  };

  std::array<std::array<Complex, 3>, 3> kron_y{};
  std::array<Complex, 3> y_ext_to_star{};
  std::array<Complex, 3> y_star_to_ext{};
  Complex y_star_star{0.0, 0.0};
  for (std::size_t leg = 0; leg < star_impedances.size(); ++leg) {
    const Complex y = Complex{1.0, 0.0} / star_impedances[leg];
    const Complex tau = std::polar(leg_taps[leg], 0.0);
    kron_y[leg][leg] += y / (tau * std::conj(tau));
    y_ext_to_star[leg] = -y / std::conj(tau);
    y_star_to_ext[leg] = -y / tau;
    y_star_star += y;
  }
  for (std::size_t row = 0; row < kron_y.size(); ++row) {
    for (std::size_t col = 0; col < kron_y[row].size(); ++col) {
      kron_y[row][col] -=
          y_ext_to_star[row] * y_star_to_ext[col] / y_star_star;
    }
  }
  return kron_y;
}

double max_abs_ybus_delta(
    const std::array<std::array<Complex, 3>, 3>& lhs,
    const std::array<std::array<Complex, 3>, 3>& rhs) {
  double max_abs = 0.0;
  for (std::size_t row = 0; row < lhs.size(); ++row) {
    for (std::size_t col = 0; col < lhs[row].size(); ++col) {
      max_abs = std::max(max_abs, std::abs(lhs[row][col] - rhs[row][col]));
    }
  }
  return max_abs;
}

}  // namespace

// ---------------------------------------------------------------------------
// Acceptance tolerances ??must match tools/opendss_pf_compare.cpp CaseTolerance
// ---------------------------------------------------------------------------
// k1phTol / kShTol:   vm_pu=1e-3, va_deg=0.1, p_mw=1e-3, q_mvar=1e-3, ploss_mw=1e-3, qloss_mvar=1e-3
// kCapTol (exploratory, V² bias expected):
//                     vm_pu=2e-2, va_deg=0.5, p_mw=2e-2, q_mvar=2e-2, ploss_mw=2e-2, qloss_mvar=2e-2
// k33bwTol:           vm_pu=2e-3, va_deg=0.2, p_mw=1e-3, q_mvar=1e-3, ploss_mw=5e-3, qloss_mvar=5e-3
// kUnbal3phTol:       vm_pu=2e-3, va_deg=0.2, p_mw=1e-3, q_mvar=1e-3, ploss_mw=5e-3, qloss_mvar=5e-3
// kXfmrTol:           vm_pu=1e-3, va_deg=0.1, p_mw=1e-3, q_mvar=1e-3, ploss_mw=1e-3, qloss_mvar=1e-3

TEST_CASE("OpenDSS 1ph bridge matches repository DPF baseline",
          "[integration][powerflow][distribution][opendss]") {
  using namespace hacdcpf::analysis;

  const auto repo_case = hacdcpf_compare_fixtures::build_3bus_radial();
  DPFOptions options;
  options.max_iter = 200;
  options.tol = 1e-10;
  const auto repo = solve_distribution_pf(repo_case.ac, options);
  REQUIRE(repo.converged);

#ifdef HACDCPF_PROJECT_ROOT
  const fs::path master =
      fs::path(HACDCPF_PROJECT_ROOT) /
      "tests/data/opendss/minimal_radial_3bus_1ph/Master.dss";
#else
  const fs::path master =
      "tests/data/opendss/minimal_radial_3bus_1ph/Master.dss";
#endif

  const auto dss = hacdcpf::io::solve_opendss_snapshot(master);
  REQUIRE(dss.converged);

  const auto maps = build_opendss_maps(dss);

  REQUIRE(maps.vm_pu.contains("sourcebus.1"));
  REQUIRE(maps.vm_pu.contains("bus2.1"));
  REQUIRE(maps.vm_pu.contains("bus3.1"));

  // Voltage magnitude [p.u.] ??tol 1e-3
  REQUIRE(std::abs(maps.vm_pu.at("sourcebus.1") - repo.vm_pu[0]) < 1e-3);
  REQUIRE(std::abs(maps.vm_pu.at("bus2.1")     - repo.vm_pu[1]) < 1e-3);
  REQUIRE(std::abs(maps.vm_pu.at("bus3.1")     - repo.vm_pu[2]) < 1e-3);

  // Voltage angle [deg] ??tol 0.1 deg
  REQUIRE(std::abs(maps.va_deg.at("sourcebus.1") - repo.va_deg[0]) < 0.1);
  REQUIRE(std::abs(maps.va_deg.at("bus2.1")      - repo.va_deg[1]) < 0.1);
  REQUIRE(std::abs(maps.va_deg.at("bus3.1")      - repo.va_deg[2]) < 0.1);

  // Terminal-1 sending-end active power [MW] ??tol 1e-3
  REQUIRE(maps.powers.contains(key_for_line_terminal_node("l1", 1, 1)));
  REQUIRE(maps.powers.contains(key_for_line_terminal_node("l2", 1, 1)));
  REQUIRE(std::abs(maps.powers.at(key_for_line_terminal_node("l1", 1, 1)).first -
                   repo.p_branch_mw[0]) < 1e-3);
  REQUIRE(std::abs(maps.powers.at(key_for_line_terminal_node("l2", 1, 1)).first -
                   repo.p_branch_mw[1]) < 1e-3);

  // Terminal-1 sending-end reactive power [MVAr] ??tol 1e-3
  REQUIRE(std::abs(maps.powers.at(key_for_line_terminal_node("l1", 1, 1)).second -
                   repo.q_branch_mvar[0]) < 1e-3);
  REQUIRE(std::abs(maps.powers.at(key_for_line_terminal_node("l2", 1, 1)).second -
                   repo.q_branch_mvar[1]) < 1e-3);

  // Total losses ??tol 1e-3 MW / 1e-3 MVAr
  const auto opendss_losses =
      hacdcpf_compare::sum_all_pd_element_terminal_powers_to_mw_mvar(dss);
  REQUIRE(std::abs(opendss_losses.p_mw   - repo.total_p_loss_mw)   < 1e-3);
  REQUIRE(std::abs(opendss_losses.q_mvar - repo.total_q_loss_mvar) < 1e-3);
}

TEST_CASE("OpenDSS 1ph bridge matches repository DPF with shunt compensator",
          "[integration][powerflow][distribution][opendss]") {
  using namespace hacdcpf::analysis;

  // build_3bus_radial_with_shunt: bus2 has bs_mvar=0.15 (150 kvar constant injection).
  // OpenDSS mirror uses model=1 kvar=-150 constant-power load to match BFS
  // fixed-injection behavior and avoid V^2 bias from a physical Capacitor element.
  const auto repo_case = hacdcpf_compare_fixtures::build_3bus_radial_with_shunt();
  DPFOptions options;
  options.max_iter = 200;
  options.tol = 1e-10;
  const auto repo = solve_distribution_pf(repo_case.ac, options);
  REQUIRE(repo.converged);

#ifdef HACDCPF_PROJECT_ROOT
  const fs::path master =
      fs::path(HACDCPF_PROJECT_ROOT) /
      "tests/data/opendss/minimal_radial_3bus_1ph_shunt/Master.dss";
#else
  const fs::path master =
      "tests/data/opendss/minimal_radial_3bus_1ph_shunt/Master.dss";
#endif

  const auto dss = hacdcpf::io::solve_opendss_snapshot(master);
  REQUIRE(dss.converged);

  const auto maps = build_opendss_maps(dss);

  REQUIRE(maps.vm_pu.contains("sourcebus.1"));
  REQUIRE(maps.vm_pu.contains("bus2.1"));
  REQUIRE(maps.vm_pu.contains("bus3.1"));

  // Voltage magnitude [p.u.] ??tol 1e-3
  REQUIRE(std::abs(maps.vm_pu.at("sourcebus.1") - repo.vm_pu[0]) < 1e-3);
  REQUIRE(std::abs(maps.vm_pu.at("bus2.1")     - repo.vm_pu[1]) < 1e-3);
  REQUIRE(std::abs(maps.vm_pu.at("bus3.1")     - repo.vm_pu[2]) < 1e-3);

  // Voltage angle [deg] ??tol 0.1 deg
  REQUIRE(std::abs(maps.va_deg.at("sourcebus.1") - repo.va_deg[0]) < 0.1);
  REQUIRE(std::abs(maps.va_deg.at("bus2.1")      - repo.va_deg[1]) < 0.1);
  REQUIRE(std::abs(maps.va_deg.at("bus3.1")      - repo.va_deg[2]) < 0.1);

  // Terminal-1 sending-end active power [MW] ??tol 1e-3
  REQUIRE(maps.powers.contains(key_for_line_terminal_node("l1", 1, 1)));
  REQUIRE(maps.powers.contains(key_for_line_terminal_node("l2", 1, 1)));
  REQUIRE(std::abs(maps.powers.at(key_for_line_terminal_node("l1", 1, 1)).first -
                   repo.p_branch_mw[0]) < 1e-3);
  REQUIRE(std::abs(maps.powers.at(key_for_line_terminal_node("l2", 1, 1)).first -
                   repo.p_branch_mw[1]) < 1e-3);

  // Terminal-1 sending-end reactive power [MVAr] ??tol 1e-3
  REQUIRE(std::abs(maps.powers.at(key_for_line_terminal_node("l1", 1, 1)).second -
                   repo.q_branch_mvar[0]) < 1e-3);
  REQUIRE(std::abs(maps.powers.at(key_for_line_terminal_node("l2", 1, 1)).second -
                   repo.q_branch_mvar[1]) < 1e-3);

  // Total losses ??tol 1e-3 MW / 1e-3 MVAr
  const auto opendss_losses =
      hacdcpf_compare::sum_all_pd_element_terminal_powers_to_mw_mvar(dss);
  REQUIRE(std::abs(opendss_losses.p_mw   - repo.total_p_loss_mw)   < 1e-3);
  REQUIRE(std::abs(opendss_losses.q_mvar - repo.total_q_loss_mvar) < 1e-3);
}

TEST_CASE("OpenDSS 3ph bridge matches repository balanced DPF baseline",
          "[integration][powerflow][distribution][opendss]") {
  using namespace hacdcpf::analysis;

  const auto repo_case = hacdcpf_compare_fixtures::build_balanced_3bus_3phase();
  DPFOptions options;
  options.max_iter = 200;
  options.tol = 1e-10;
  const auto repo = solve_three_phase_distribution_pf(repo_case, options);
  REQUIRE(repo.converged);

#ifdef HACDCPF_PROJECT_ROOT
  const fs::path master =
      fs::path(HACDCPF_PROJECT_ROOT) /
      "tests/data/opendss/minimal_radial_3bus_balanced_3ph/Master.dss";
#else
  const fs::path master =
      "tests/data/opendss/minimal_radial_3bus_balanced_3ph/Master.dss";
#endif

  const auto dss = hacdcpf::io::solve_opendss_snapshot(master);
  REQUIRE(dss.converged);

  const auto maps = build_opendss_maps(dss);

  // Voltage magnitude [p.u.] ??tol 2e-3
  REQUIRE(std::abs(maps.vm_pu.at("sourcebus.1") - repo.bus_voltages[0].vm_a_pu) < 2e-3);
  REQUIRE(std::abs(maps.vm_pu.at("sourcebus.2") - repo.bus_voltages[0].vm_b_pu) < 2e-3);
  REQUIRE(std::abs(maps.vm_pu.at("sourcebus.3") - repo.bus_voltages[0].vm_c_pu) < 2e-3);
  REQUIRE(std::abs(maps.vm_pu.at("bus2.1")      - repo.bus_voltages[1].vm_a_pu) < 2e-3);
  REQUIRE(std::abs(maps.vm_pu.at("bus2.2")      - repo.bus_voltages[1].vm_b_pu) < 2e-3);
  REQUIRE(std::abs(maps.vm_pu.at("bus2.3")      - repo.bus_voltages[1].vm_c_pu) < 2e-3);
  REQUIRE(std::abs(maps.vm_pu.at("bus3.1")      - repo.bus_voltages[2].vm_a_pu) < 2e-3);
  REQUIRE(std::abs(maps.vm_pu.at("bus3.2")      - repo.bus_voltages[2].vm_b_pu) < 2e-3);
  REQUIRE(std::abs(maps.vm_pu.at("bus3.3")      - repo.bus_voltages[2].vm_c_pu) < 2e-3);

  // Terminal-1 per-phase sending-end P [MW] ??tol 1e-3
  REQUIRE(std::abs(maps.powers.at(key_for_line_terminal_node("l1", 1, 1)).first -
                   repo.branch_powers[0].p_a_mw) < 1e-3);
  REQUIRE(std::abs(maps.powers.at(key_for_line_terminal_node("l1", 1, 2)).first -
                   repo.branch_powers[0].p_b_mw) < 1e-3);
  REQUIRE(std::abs(maps.powers.at(key_for_line_terminal_node("l1", 1, 3)).first -
                   repo.branch_powers[0].p_c_mw) < 1e-3);
  REQUIRE(std::abs(maps.powers.at(key_for_line_terminal_node("l2", 1, 1)).first -
                   repo.branch_powers[1].p_a_mw) < 1e-3);
  REQUIRE(std::abs(maps.powers.at(key_for_line_terminal_node("l2", 1, 2)).first -
                   repo.branch_powers[1].p_b_mw) < 1e-3);
  REQUIRE(std::abs(maps.powers.at(key_for_line_terminal_node("l2", 1, 3)).first -
                   repo.branch_powers[1].p_c_mw) < 1e-3);

  // Terminal-1 per-phase sending-end Q [MVAr] ??tol 1e-3
  REQUIRE(std::abs(maps.powers.at(key_for_line_terminal_node("l1", 1, 1)).second -
                   repo.branch_powers[0].q_a_mvar) < 1e-3);
  REQUIRE(std::abs(maps.powers.at(key_for_line_terminal_node("l1", 1, 2)).second -
                   repo.branch_powers[0].q_b_mvar) < 1e-3);
  REQUIRE(std::abs(maps.powers.at(key_for_line_terminal_node("l1", 1, 3)).second -
                   repo.branch_powers[0].q_c_mvar) < 1e-3);
  REQUIRE(std::abs(maps.powers.at(key_for_line_terminal_node("l2", 1, 1)).second -
                   repo.branch_powers[1].q_a_mvar) < 1e-3);
  REQUIRE(std::abs(maps.powers.at(key_for_line_terminal_node("l2", 1, 2)).second -
                   repo.branch_powers[1].q_b_mvar) < 1e-3);
  REQUIRE(std::abs(maps.powers.at(key_for_line_terminal_node("l2", 1, 3)).second -
                   repo.branch_powers[1].q_c_mvar) < 1e-3);

  // Total losses ??tol 5e-3 MW / 5e-3 MVAr
  const auto opendss_losses =
      hacdcpf_compare::sum_all_pd_element_terminal_powers_to_mw_mvar(dss);
  REQUIRE(std::abs(opendss_losses.p_mw   - repo.total_p_loss_mw)   < 5e-3);
  REQUIRE(std::abs(opendss_losses.q_mvar - repo.total_q_loss_mvar) < 5e-3);
}

TEST_CASE("OpenDSS 1ph physical capacitor vs BFS shunt approximation (exploratory)",
          "[integration][powerflow][distribution][opendss][exploratory]") {
  using namespace hacdcpf::analysis;

  // BFS side: bs_mvar=0.15 (fixed voltage-independent injection, same as shunt case).
  // OpenDSS side: physical Capacitor element (kvar=150, kv=7.2) delivering Q = V^2 * B.
  // Systematic bias: Δ ??(1 - V_bus2^2) * 0.15 MVAr ??0.009 MVAr at V??.97 pu.
  // Classification: exploratory_physical_component ??bias quantified in compare report.
  // Tolerances are intentionally loose (kCapTol) to bound but not suppress the bias.
  const auto repo_case = hacdcpf_compare_fixtures::build_3bus_radial_with_cap_physical();
  DPFOptions options;
  options.max_iter = 200;
  options.tol = 1e-10;
  const auto repo = solve_distribution_pf(repo_case.ac, options);
  REQUIRE(repo.converged);

#ifdef HACDCPF_PROJECT_ROOT
  const fs::path master =
      fs::path(HACDCPF_PROJECT_ROOT) /
      "tests/data/opendss/minimal_radial_3bus_1ph_cap/Master.dss";
#else
  const fs::path master =
      "tests/data/opendss/minimal_radial_3bus_1ph_cap/Master.dss";
#endif

  const auto dss = hacdcpf::io::solve_opendss_snapshot(master);
  REQUIRE(dss.converged);

  const auto maps = build_opendss_maps(dss);

  REQUIRE(maps.vm_pu.contains("sourcebus.1"));
  REQUIRE(maps.vm_pu.contains("bus2.1"));
  REQUIRE(maps.vm_pu.contains("bus3.1"));

  // Voltage magnitude [p.u.] ??loose tol 2e-2 (exploratory: expected V^2 bias)
  CHECK(std::abs(maps.vm_pu.at("sourcebus.1") - repo.vm_pu[0]) < 2e-2);
  CHECK(std::abs(maps.vm_pu.at("bus2.1")     - repo.vm_pu[1]) < 2e-2);
  CHECK(std::abs(maps.vm_pu.at("bus3.1")     - repo.vm_pu[2]) < 2e-2);

  // Voltage angle [deg] ??loose tol 0.5 deg
  CHECK(std::abs(maps.va_deg.at("sourcebus.1") - repo.va_deg[0]) < 0.5);
  CHECK(std::abs(maps.va_deg.at("bus2.1")      - repo.va_deg[1]) < 0.5);
  CHECK(std::abs(maps.va_deg.at("bus3.1")      - repo.va_deg[2]) < 0.5);

  // Terminal-1 sending-end active power [MW] ??loose tol 2e-2
  REQUIRE(maps.powers.contains(key_for_line_terminal_node("l1", 1, 1)));
  REQUIRE(maps.powers.contains(key_for_line_terminal_node("l2", 1, 1)));
  CHECK(std::abs(maps.powers.at(key_for_line_terminal_node("l1", 1, 1)).first -
                 repo.p_branch_mw[0]) < 2e-2);
  CHECK(std::abs(maps.powers.at(key_for_line_terminal_node("l2", 1, 1)).first -
                 repo.p_branch_mw[1]) < 2e-2);

  // Terminal-1 sending-end reactive power [MVAr] ??loose tol 2e-2
  CHECK(std::abs(maps.powers.at(key_for_line_terminal_node("l1", 1, 1)).second -
                 repo.q_branch_mvar[0]) < 2e-2);
  CHECK(std::abs(maps.powers.at(key_for_line_terminal_node("l2", 1, 1)).second -
                 repo.q_branch_mvar[1]) < 2e-2);

  // Total losses ??loose tol 2e-2 MW / 2e-2 MVAr
  const auto opendss_losses =
      hacdcpf_compare::sum_all_pd_element_terminal_powers_to_mw_mvar(dss);
  CHECK(std::abs(opendss_losses.p_mw   - repo.total_p_loss_mw)   < 2e-2);
  CHECK(std::abs(opendss_losses.q_mvar - repo.total_q_loss_mvar) < 2e-2);
}

TEST_CASE("OpenDSS 3ph case33bw feeder matches BFS single-phase-equivalent",
          "[integration][powerflow][distribution][opendss][feeder]") {
  using namespace hacdcpf::analysis;

  // Baran & Wu (1989) 33-bus radial feeder ??3-phase balanced OpenDSS model.
  // BFS single-phase-equivalent: vm_pu[i] = phase-A p.u. voltage.
  // Power comparison: BFS p_branch_mw[j] = 3-phase total = sum of OpenDSS
  // phase-A + phase-B + phase-C terminal-1 powers.
  const auto repo_case = hacdcpf_compare_fixtures::build_case33bw_radial();
  DPFOptions options;
  options.max_iter = 200;
  options.tol = 1e-10;
  const auto repo = solve_distribution_pf(repo_case.ac, options);
  REQUIRE(repo.converged);

#ifdef HACDCPF_PROJECT_ROOT
  const fs::path master =
      fs::path(HACDCPF_PROJECT_ROOT) /
      "tests/data/opendss/case33bw_radial_3ph/Master.dss";
#else
  const fs::path master =
      "tests/data/opendss/case33bw_radial_3ph/Master.dss";
#endif

  const auto dss = hacdcpf::io::solve_opendss_snapshot(master);
  REQUIRE(dss.converged);

  const auto maps = build_opendss_maps(dss);

  // 33 buses: BFS buses[i] = b(i+1); compare phase-A (node 1) vm_pu and va_deg.
  for (int i = 0; i < 33; ++i) {
    const std::string bname = "b" + std::to_string(i + 1);
    const std::string vkey = key_for_bus_node(bname, 1);
    REQUIRE(maps.vm_pu.contains(vkey));
    // vm_pu ??tol 2e-3
    CHECK(std::abs(maps.vm_pu.at(vkey) - repo.vm_pu[i]) < 2e-3);
    // va_deg ??tol 0.2 deg
    CHECK(std::abs(maps.va_deg.at(vkey) - repo.va_deg[i]) < 0.2);
  }

  // 32 branches: BFS branches[j] = l(j+1) zero-padded to 2 digits.
  // Sum phase-A + phase-B + phase-C at terminal 1 = 3-phase total = BFS p_branch_mw[j].
  for (int j = 0; j < 32; ++j) {
    const std::string lname =
        "l" + (j + 1 < 10 ? std::string("0") : std::string("")) +
        std::to_string(j + 1);
    double p_sum = 0.0;
    double q_sum = 0.0;
    for (int node = 1; node <= 3; ++node) {
      const auto pkey = key_for_line_terminal_node(lname, 1, node);
      REQUIRE(maps.powers.contains(pkey));
      p_sum += maps.powers.at(pkey).first;
      q_sum += maps.powers.at(pkey).second;
    }
    // p_branch_mw ??tol 1e-3 MW; q_branch_mvar ??tol 1e-3 MVAr
    CHECK(std::abs(p_sum - repo.p_branch_mw[j]) < 1e-3);
    CHECK(std::abs(q_sum - repo.q_branch_mvar[j]) < 1e-3);
  }

  // Total losses ??tol 5e-3 MW / 5e-3 MVAr
  const auto opendss_losses =
      hacdcpf_compare::sum_all_pd_element_terminal_powers_to_mw_mvar(dss);
  CHECK(std::abs(opendss_losses.p_mw   - repo.total_p_loss_mw)   < 5e-3);
  CHECK(std::abs(opendss_losses.q_mvar - repo.total_q_loss_mvar) < 5e-3);

  // ---------------------------------------------------------------------------
  // Phase symmetry regression (internal OpenDSS consistency).
  // Purpose: catch phase ordering / conductor indexing / parser drift.
  // case33bw uses balanced 3-phase loads and balanced line impedances, so:
  //   vm_a ??vm_b ??vm_c  (relative imbalance < 1e-4)
  //   va_b - va_a ??-120°, va_c - va_a ??+120°  (deviation < 1e-3 deg)
  //   per-phase terminal powers equal for all 3 phases  (relative imbalance < 1e-4)
  // These are OpenDSS internal self-consistency checks, not BFS comparison.
  // ---------------------------------------------------------------------------
  for (int i = 0; i < 33; ++i) {
    const std::string bname = "b" + std::to_string(i + 1);
    const double vma = maps.vm_pu.at(key_for_bus_node(bname, 1));
    const double vmb = maps.vm_pu.at(key_for_bus_node(bname, 2));
    const double vmc = maps.vm_pu.at(key_for_bus_node(bname, 3));
    const double vaa = maps.va_deg.at(key_for_bus_node(bname, 1));
    const double vab = maps.va_deg.at(key_for_bus_node(bname, 2));
    const double vac = maps.va_deg.at(key_for_bus_node(bname, 3));
    if (vma > 1e-9) {
      CHECK(std::abs(vmb - vma) / vma < 1e-4);   // B vs A magnitude balance
      CHECK(std::abs(vmc - vma) / vma < 1e-4);   // C vs A magnitude balance
    }
    CHECK(std::abs(vab - vaa + 120.0) < 1e-3);   // B lags A by 120°
    CHECK(std::abs(vac - vaa - 120.0) < 1e-3);   // C leads A by 120°
  }

  for (int j = 0; j < 32; ++j) {
    const std::string lname =
        "l" + (j + 1 < 10 ? std::string("0") : std::string("")) +
        std::to_string(j + 1);
    const double pa = maps.powers.at(key_for_line_terminal_node(lname, 1, 1)).first;
    const double pb = maps.powers.at(key_for_line_terminal_node(lname, 1, 2)).first;
    const double pc = maps.powers.at(key_for_line_terminal_node(lname, 1, 3)).first;
    const double ptotal = pa + pb + pc;
    if (std::abs(ptotal) > 1e-3) {  // skip near-zero total (kW-scale threshold in MW)
      const double ref = std::abs(ptotal) / 3.0;
      CHECK(std::abs(pa - ptotal / 3.0) / ref < 1e-4);
      CHECK(std::abs(pb - ptotal / 3.0) / ref < 1e-4);
      CHECK(std::abs(pc - ptotal / 3.0) / ref < 1e-4);
    }
  }
}

// ---------------------------------------------------------------------------
// TEST: unbalanced 3-phase radial ??accepted baseline
// Verifies per-phase Vm, Va, terminal-1 P/Q (A/B/C), and total losses.
// kUnbal3phTol: vm_pu=2e-3, va_deg=0.2, p_mw=1e-3, q_mvar=1e-3,
//               ploss_mw=5e-3, qloss_mvar=5e-3.
// ---------------------------------------------------------------------------
TEST_CASE("OpenDSS 3ph bridge matches repository unbalanced 3-phase DPF baseline",
          "[integration][powerflow][distribution][opendss][unbalanced]") {
  using namespace hacdcpf::analysis;

  const auto repo_case = hacdcpf_compare_fixtures::build_unbalanced_3bus_3phase();
  DPFOptions options;
  options.max_iter = 200;
  options.tol = 1e-10;
  const auto repo = solve_three_phase_distribution_pf(repo_case, options);
  REQUIRE(repo.converged);

#ifdef HACDCPF_PROJECT_ROOT
  const fs::path master =
      fs::path(HACDCPF_PROJECT_ROOT) /
      "tests/data/opendss/minimal_unbalanced_3bus_3ph/Master.dss";
#else
  const fs::path master =
      "tests/data/opendss/minimal_unbalanced_3bus_3ph/Master.dss";
#endif

  const auto dss = hacdcpf::io::solve_opendss_snapshot(master);
  REQUIRE(dss.converged);

  const auto maps = build_opendss_maps(dss);

  // Per-phase voltage magnitude [p.u.] ??tol 2e-3
  // 3 buses × 3 phases = 9 assertions
  REQUIRE(maps.vm_pu.contains("sourcebus.1"));
  REQUIRE(maps.vm_pu.contains("sourcebus.2"));
  REQUIRE(maps.vm_pu.contains("sourcebus.3"));
  CHECK(std::abs(maps.vm_pu.at("sourcebus.1") - repo.bus_voltages[0].vm_a_pu) < 2e-3);
  CHECK(std::abs(maps.vm_pu.at("sourcebus.2") - repo.bus_voltages[0].vm_b_pu) < 2e-3);
  CHECK(std::abs(maps.vm_pu.at("sourcebus.3") - repo.bus_voltages[0].vm_c_pu) < 2e-3);
  REQUIRE(maps.vm_pu.contains("bus2.1"));
  REQUIRE(maps.vm_pu.contains("bus2.2"));
  REQUIRE(maps.vm_pu.contains("bus2.3"));
  CHECK(std::abs(maps.vm_pu.at("bus2.1")      - repo.bus_voltages[1].vm_a_pu) < 2e-3);
  CHECK(std::abs(maps.vm_pu.at("bus2.2")      - repo.bus_voltages[1].vm_b_pu) < 2e-3);
  CHECK(std::abs(maps.vm_pu.at("bus2.3")      - repo.bus_voltages[1].vm_c_pu) < 2e-3);
  REQUIRE(maps.vm_pu.contains("bus3.1"));
  REQUIRE(maps.vm_pu.contains("bus3.2"));
  REQUIRE(maps.vm_pu.contains("bus3.3"));
  CHECK(std::abs(maps.vm_pu.at("bus3.1")      - repo.bus_voltages[2].vm_a_pu) < 2e-3);
  CHECK(std::abs(maps.vm_pu.at("bus3.2")      - repo.bus_voltages[2].vm_b_pu) < 2e-3);
  CHECK(std::abs(maps.vm_pu.at("bus3.3")      - repo.bus_voltages[2].vm_c_pu) < 2e-3);

  // Per-phase voltage angle [deg] ??tol 0.2 deg
  // First accepted case to assert per-phase voltage angles.
  CHECK(std::abs(maps.va_deg.at("sourcebus.1") - repo.bus_voltages[0].va_a_deg) < 0.2);
  CHECK(std::abs(maps.va_deg.at("sourcebus.2") - repo.bus_voltages[0].va_b_deg) < 0.2);
  CHECK(std::abs(maps.va_deg.at("sourcebus.3") - repo.bus_voltages[0].va_c_deg) < 0.2);
  CHECK(std::abs(maps.va_deg.at("bus2.1")      - repo.bus_voltages[1].va_a_deg) < 0.2);
  CHECK(std::abs(maps.va_deg.at("bus2.2")      - repo.bus_voltages[1].va_b_deg) < 0.2);
  CHECK(std::abs(maps.va_deg.at("bus2.3")      - repo.bus_voltages[1].va_c_deg) < 0.2);
  CHECK(std::abs(maps.va_deg.at("bus3.1")      - repo.bus_voltages[2].va_a_deg) < 0.2);
  CHECK(std::abs(maps.va_deg.at("bus3.2")      - repo.bus_voltages[2].va_b_deg) < 0.2);
  CHECK(std::abs(maps.va_deg.at("bus3.3")      - repo.bus_voltages[2].va_c_deg) < 0.2);

  // Per-phase terminal-1 sending-end P [MW] ??tol 1e-3
  // 2 lines × 3 phases = 6 assertions
  REQUIRE(maps.powers.contains(key_for_line_terminal_node("l1", 1, 1)));
  REQUIRE(maps.powers.contains(key_for_line_terminal_node("l1", 1, 2)));
  REQUIRE(maps.powers.contains(key_for_line_terminal_node("l1", 1, 3)));
  CHECK(std::abs(maps.powers.at(key_for_line_terminal_node("l1", 1, 1)).first -
                 repo.branch_powers[0].p_a_mw) < 1e-3);
  CHECK(std::abs(maps.powers.at(key_for_line_terminal_node("l1", 1, 2)).first -
                 repo.branch_powers[0].p_b_mw) < 1e-3);
  CHECK(std::abs(maps.powers.at(key_for_line_terminal_node("l1", 1, 3)).first -
                 repo.branch_powers[0].p_c_mw) < 1e-3);
  REQUIRE(maps.powers.contains(key_for_line_terminal_node("l2", 1, 1)));
  REQUIRE(maps.powers.contains(key_for_line_terminal_node("l2", 1, 2)));
  REQUIRE(maps.powers.contains(key_for_line_terminal_node("l2", 1, 3)));
  CHECK(std::abs(maps.powers.at(key_for_line_terminal_node("l2", 1, 1)).first -
                 repo.branch_powers[1].p_a_mw) < 1e-3);
  CHECK(std::abs(maps.powers.at(key_for_line_terminal_node("l2", 1, 2)).first -
                 repo.branch_powers[1].p_b_mw) < 1e-3);
  CHECK(std::abs(maps.powers.at(key_for_line_terminal_node("l2", 1, 3)).first -
                 repo.branch_powers[1].p_c_mw) < 1e-3);

  // Per-phase terminal-1 sending-end Q [MVAr] ??tol 1e-3
  CHECK(std::abs(maps.powers.at(key_for_line_terminal_node("l1", 1, 1)).second -
                 repo.branch_powers[0].q_a_mvar) < 1e-3);
  CHECK(std::abs(maps.powers.at(key_for_line_terminal_node("l1", 1, 2)).second -
                 repo.branch_powers[0].q_b_mvar) < 1e-3);
  CHECK(std::abs(maps.powers.at(key_for_line_terminal_node("l1", 1, 3)).second -
                 repo.branch_powers[0].q_c_mvar) < 1e-3);
  CHECK(std::abs(maps.powers.at(key_for_line_terminal_node("l2", 1, 1)).second -
                 repo.branch_powers[1].q_a_mvar) < 1e-3);
  CHECK(std::abs(maps.powers.at(key_for_line_terminal_node("l2", 1, 2)).second -
                 repo.branch_powers[1].q_b_mvar) < 1e-3);
  CHECK(std::abs(maps.powers.at(key_for_line_terminal_node("l2", 1, 3)).second -
                 repo.branch_powers[1].q_c_mvar) < 1e-3);

  // Total losses ??tol 5e-3 MW / 5e-3 MVAr
  const auto opendss_losses =
      hacdcpf_compare::sum_all_pd_element_terminal_powers_to_mw_mvar(dss);
  CHECK(std::abs(opendss_losses.p_mw   - repo.total_p_loss_mw)   < 5e-3);
  CHECK(std::abs(opendss_losses.q_mvar - repo.total_q_loss_mvar) < 5e-3);
}

// ---------------------------------------------------------------------------
// TEST: pass-through transformer (tap=1.0) equivalence ??accepted baseline
// Verifies:
//   - bus voltages (3 buses)
//   - transformer t12 terminal-1 P/Q
//   - downstream line l1 terminal-1 P/Q
//   - total branch losses via summed PD-element terminal powers
//   - Circuit.Losses raw as an independent cross-check
// kXfmrTol: vm_pu=1e-3, va_deg=0.1, p_mw=1e-3, q_mvar=1e-3,
//           ploss_mw=1e-3, qloss_mvar=1e-3.
// ---------------------------------------------------------------------------
TEST_CASE("OpenDSS 1ph pass-through transformer matches BFS series-impedance ACBranch",
          "[integration][powerflow][distribution][opendss][transformer]") {
  using namespace hacdcpf::analysis;

  const auto repo_case = hacdcpf_compare_fixtures::build_3bus_passthr_transformer();
  DPFOptions options;
  options.max_iter = 200;
  options.tol = 1e-10;
  options.include_shunts = true;
  const auto repo = solve_distribution_pf(repo_case.ac, options);
  REQUIRE(repo.converged);

#ifdef HACDCPF_PROJECT_ROOT
  const fs::path master =
      fs::path(HACDCPF_PROJECT_ROOT) /
      "tests/data/opendss/minimal_passthr_transformer_1ph/Master.dss";
#else
  const fs::path master =
      "tests/data/opendss/minimal_passthr_transformer_1ph/Master.dss";
#endif

  const auto dss = hacdcpf::io::solve_opendss_snapshot(master);
  REQUIRE(dss.converged);

  const auto maps = build_opendss_maps(dss);

  // Bus voltage magnitudes [p.u.] ??tol 1e-3
  // 3 buses (sourcebus, bus2, bus3), single-phase node 1.
  REQUIRE(maps.vm_pu.contains("sourcebus.1"));
  REQUIRE(maps.vm_pu.contains("bus2.1"));
  REQUIRE(maps.vm_pu.contains("bus3.1"));
  CHECK(std::abs(maps.vm_pu.at("sourcebus.1") - repo.vm_pu[0]) < 1e-3);
  CHECK(std::abs(maps.vm_pu.at("bus2.1")      - repo.vm_pu[1]) < 1e-3);
  CHECK(std::abs(maps.vm_pu.at("bus3.1")      - repo.vm_pu[2]) < 1e-3);

  // Bus voltage angles [deg] ??tol 0.1 deg
  CHECK(std::abs(maps.va_deg.at("sourcebus.1") - repo.va_deg[0]) < 0.1);
  CHECK(std::abs(maps.va_deg.at("bus2.1")      - repo.va_deg[1]) < 0.1);
  CHECK(std::abs(maps.va_deg.at("bus3.1")      - repo.va_deg[2]) < 0.1);

  // Transformer t12 (sourcebus→bus2) terminal-1 P/Q ??tol 1e-3 MW / 1e-3 MVAr.
  // repo.p_branch_mw[0]: BFS branch index 0 = transformer equivalent branch.
  REQUIRE(maps.powers.contains(key_for_transformer_terminal_node("t12", 1, 1)));
  CHECK(std::abs(
            maps.powers.at(key_for_transformer_terminal_node("t12", 1, 1)).first -
            repo.p_branch_mw[0]) < 1e-3);
  CHECK(std::abs(
            maps.powers.at(key_for_transformer_terminal_node("t12", 1, 1)).second -
            repo.q_branch_mvar[0]) < 1e-3);

  // Downstream line l1 (bus2→bus3) terminal-1 P/Q ??tol 1e-3 MW / 1e-3 MVAr.
  REQUIRE(maps.powers.contains(key_for_line_terminal_node("l1", 1, 1)));
  CHECK(std::abs(maps.powers.at(key_for_line_terminal_node("l1", 1, 1)).first -
                 repo.p_branch_mw[1]) < 1e-3);
  CHECK(std::abs(maps.powers.at(key_for_line_terminal_node("l1", 1, 1)).second -
                 repo.q_branch_mvar[1]) < 1e-3);

  // Total branch losses via summed PD-element terminal powers ??tol 1e-3 MW / 1e-3 MVAr.
  const auto dss_branch_losses =
      hacdcpf_compare::sum_all_pd_element_terminal_powers_to_mw_mvar(dss);
  CHECK(std::abs(dss_branch_losses.p_mw   - repo.total_p_loss_mw)   < 1e-3);
  CHECK(std::abs(dss_branch_losses.q_mvar - repo.total_q_loss_mvar) < 1e-3);

  // Circuit.Losses raw is a cross-check, no longer the only accepted evidence.
  const auto dss_circuit_losses = hacdcpf_compare::opendss_circuit_losses_to_mw_mvar(dss);
  CHECK(std::abs(dss_circuit_losses.p_mw   - dss_branch_losses.p_mw)   < 1e-6);
  CHECK(std::abs(dss_circuit_losses.q_mvar - dss_branch_losses.q_mvar) < 1e-6);
}

TEST_CASE("OpenDSS line_results stays a compatibility projection of the generic PD snapshot",
          "[integration][powerflow][distribution][opendss][bridge]") {
#ifdef HACDCPF_PROJECT_ROOT
  const fs::path master =
      fs::path(HACDCPF_PROJECT_ROOT) /
      "tests/data/opendss/minimal_passthr_transformer_1ph/Master.dss";
#else
  const fs::path master =
      "tests/data/opendss/minimal_passthr_transformer_1ph/Master.dss";
#endif

  const auto dss = hacdcpf::io::solve_opendss_snapshot(master);
  REQUIRE(dss.converged);
  REQUIRE(dss.pd_element_results.size() == 2);
  REQUIRE(dss.line_results.size() == 1);

  const auto& line_pd = find_pd_element(
      dss, hacdcpf::io::OpenDSSPDElementKind::Line, "l1");
  const auto& line_view = dss.line_results.front();

  CHECK(line_view.element_name == line_pd.element_name);
  CHECK(line_view.line_name == line_pd.name);
  CHECK(line_view.terminal_bus_names == line_pd.terminal_bus_names);
  CHECK(line_view.node_order == line_pd.node_order);
  CHECK(line_view.num_phases == line_pd.num_phases);
  CHECK(line_view.num_conductors == line_pd.num_conductors);
  CHECK(line_view.num_terminals == line_pd.num_terminals);
  REQUIRE(line_view.terminal_powers.size() == line_pd.terminal_powers.size());
  for (std::size_t idx = 0; idx < line_view.terminal_powers.size(); ++idx) {
    CHECK(line_view.terminal_powers[idx].terminal ==
          line_pd.terminal_powers[idx].terminal);
    CHECK(line_view.terminal_powers[idx].conductor ==
          line_pd.terminal_powers[idx].conductor);
    CHECK(line_view.terminal_powers[idx].node ==
          line_pd.terminal_powers[idx].node);
    CHECK(line_view.terminal_powers[idx].power_kw_kvar.p_kw ==
          line_pd.terminal_powers[idx].power_kw_kvar.p_kw);
    CHECK(line_view.terminal_powers[idx].power_kw_kvar.q_kvar ==
          line_pd.terminal_powers[idx].power_kw_kvar.q_kvar);
  }
}

TEST_CASE("OpenDSS 1ph fixed-tap transformer matches tap-aware repository BFS",
          "[integration][powerflow][distribution][opendss][transformer][tap]") {
  using namespace hacdcpf::analysis;

  const auto spec = hacdcpf_compare_fixtures::fixed_tap_transformer_case();
  const auto repo_case = hacdcpf_compare_fixtures::build_3bus_fixed_tap_transformer();
  const auto projected = hacdcpf::project_to_canonical_models(repo_case);

  DPFOptions options;
  options.max_iter = 200;
  options.tol = 1e-10;
  options.include_shunts = true;
  const auto repo = solve_distribution_pf(repo_case, options);
  REQUIRE(repo.converged);

#ifdef HACDCPF_PROJECT_ROOT
  const fs::path master =
      fs::path(HACDCPF_PROJECT_ROOT) / spec.relative_master_dss;
#else
  const fs::path master = spec.relative_master_dss;
#endif

  const auto dss = hacdcpf::io::solve_opendss_snapshot(master);
  REQUIRE(dss.converged);
  REQUIRE(dss.pd_element_results.size() == 2);
  REQUIRE(dss.line_results.size() == 1);

  const auto maps = build_opendss_maps(dss);
  const auto& xfmr = find_pd_element(
      dss, hacdcpf::io::OpenDSSPDElementKind::Transformer, spec.transformer_name);
  const auto& line = find_pd_element(
      dss, hacdcpf::io::OpenDSSPDElementKind::Line, spec.line_name);
  const auto& transformer_state = find_transformer_state(dss, spec.transformer_name);
  const auto& tapped_winding = find_transformer_winding_state(transformer_state, 1);

  const int transformer_branch_pos = find_projected_branch_position(
      projected, hacdcpf::BranchOriginType::Transformer2W,
      spec.transformer_origin_index);
  const int line_branch_pos = find_branch_position_by_name(projected.ac, spec.line_name);
  REQUIRE(std::abs(projected.ac.branches[static_cast<std::size_t>(transformer_branch_pos)].tap -
                   spec.configured_transformer_tap_pu) < 1e-12);
  CHECK(std::abs(tapped_winding.tap_pu - spec.configured_transformer_tap_pu) < 1e-12);

  CHECK(std::abs(maps.vm_pu.at("sourcebus.1") - repo.vm_pu[0]) < 1e-3);
  CHECK(std::abs(maps.vm_pu.at("bus2.1") - repo.vm_pu[1]) < 1e-3);
  CHECK(std::abs(maps.vm_pu.at("bus3.1") - repo.vm_pu[2]) < 1e-3);

  CHECK(std::abs(maps.va_deg.at("sourcebus.1") - repo.va_deg[0]) < 0.1);
  CHECK(std::abs(maps.va_deg.at("bus2.1") - repo.va_deg[1]) < 0.1);
  CHECK(std::abs(maps.va_deg.at("bus3.1") - repo.va_deg[2]) < 0.1);

  REQUIRE(maps.powers.contains(
      key_for_transformer_terminal_node(spec.transformer_name, 1, 1)));
  CHECK(std::abs(
            maps.powers.at(
                key_for_transformer_terminal_node(spec.transformer_name, 1, 1)).first -
            repo.p_branch_mw[static_cast<std::size_t>(transformer_branch_pos)]) < 1e-3);
  CHECK(std::abs(
            maps.powers.at(
                key_for_transformer_terminal_node(spec.transformer_name, 1, 1)).second -
            repo.q_branch_mvar[static_cast<std::size_t>(transformer_branch_pos)]) < 1e-3);

  REQUIRE(maps.powers.contains(key_for_line_terminal_node(spec.line_name, 1, 1)));
  CHECK(std::abs(maps.powers.at(key_for_line_terminal_node(spec.line_name, 1, 1)).first -
                 repo.p_branch_mw[static_cast<std::size_t>(line_branch_pos)]) < 1e-3);
  CHECK(std::abs(maps.powers.at(key_for_line_terminal_node(spec.line_name, 1, 1)).second -
                 repo.q_branch_mvar[static_cast<std::size_t>(line_branch_pos)]) < 1e-3);

  const auto xfmr_terminal_sum =
      hacdcpf_compare::sum_pd_element_terminal_powers_to_mw_mvar(xfmr);
  const auto xfmr_losses =
      hacdcpf_compare::opendss_element_losses_to_mw_mvar(xfmr);
  CHECK(std::abs(xfmr_terminal_sum.p_mw - xfmr_losses.p_mw) < 1e-6);
  CHECK(std::abs(xfmr_terminal_sum.q_mvar - xfmr_losses.q_mvar) < 1e-6);

  const auto line_terminal_sum =
      hacdcpf_compare::sum_pd_element_terminal_powers_to_mw_mvar(line);
  const auto line_losses =
      hacdcpf_compare::opendss_element_losses_to_mw_mvar(line);
  CHECK(std::abs(line_terminal_sum.p_mw - line_losses.p_mw) < 1e-6);
  CHECK(std::abs(line_terminal_sum.q_mvar - line_losses.q_mvar) < 1e-6);

  const auto total_branch_losses =
      hacdcpf_compare::sum_all_pd_element_terminal_powers_to_mw_mvar(dss);
  CHECK(std::abs(total_branch_losses.p_mw - repo.total_p_loss_mw) < 1e-3);
  CHECK(std::abs(total_branch_losses.q_mvar - repo.total_q_loss_mvar) < 1e-3);

  const auto circuit_losses =
      hacdcpf_compare::opendss_circuit_losses_to_mw_mvar(dss);
  CHECK(std::abs(total_branch_losses.p_mw - circuit_losses.p_mw) < 1e-6);
  CHECK(std::abs(total_branch_losses.q_mvar - circuit_losses.q_mvar) < 1e-6);
}

TEST_CASE("OpenDSS 1ph regulator matches repository closed-loop discrete control",
          "[integration][powerflow][distribution][opendss][regulator]") {
  const auto spec = hacdcpf_compare_fixtures::accepted_regulator_case();
  const auto repo_case = hacdcpf_compare_fixtures::build_minimal_regulator_1ph();
  verify_regulator_case_matches_repository(spec, repo_case);
}

TEST_CASE("OpenDSS 1ph regulator with local PT plus LDC matches repository closed-loop control",
          "[integration][powerflow][distribution][opendss][regulator][ldc]") {
  const auto spec =
      hacdcpf_compare_fixtures::accepted_local_pt_with_ldc_regulator_case();
  const auto repo_case =
      hacdcpf_compare_fixtures::build_minimal_regulator_1ph_local_pt_with_ldc();
  verify_regulator_case_matches_repository(spec, repo_case);
}

TEST_CASE("OpenDSS 1ph regulator with distinct RemotePTRatio matches repository closed-loop control",
          "[integration][powerflow][distribution][opendss][regulator][remote_ptratio]") {
  const auto spec =
      hacdcpf_compare_fixtures::accepted_remote_ptratio_regulator_case();
  const auto repo_case =
      hacdcpf_compare_fixtures::build_minimal_regulator_1ph_remote_ptratio();
  verify_regulator_case_matches_repository(spec, repo_case);
}

TEST_CASE("Standalone OpenDSS snapshot adapter emits canonical machine-readable JSON",
          "[integration][powerflow][distribution][opendss][adapter]") {
  const fs::path adapter = sibling_snapshot_adapter_path();
  REQUIRE(fs::exists(adapter));

#ifdef HACDCPF_PROJECT_ROOT
  const fs::path master =
      fs::path(HACDCPF_PROJECT_ROOT) /
      "tests/data/opendss/minimal_regulator_1ph/Master.dss";
#else
  const fs::path master =
      "tests/data/opendss/minimal_regulator_1ph/Master.dss";
#endif

  ScopedTempReportDir out_dir("snapshot_adapter");
  const fs::path output = out_dir.path / "snapshot.json";
  const std::string command =
      shell_exe(adapter) + " --master " + shell_quote(master) +
      " --out " + shell_quote(output);

  const int raw_status = std::system(command.c_str());
  REQUIRE(raw_status != -1);
  REQUIRE(decode_exit_code(raw_status) == 0);
  REQUIRE(fs::exists(output));

  const json snapshot = read_json_file(output);
  const auto contract =
      hacdcpf_compare::snapshot_contract::snapshot_from_json(snapshot);
  const auto direct = hacdcpf::io::solve_opendss_snapshot(master);
  REQUIRE(snapshot["schema"]["name"] ==
          hacdcpf_compare::snapshot_contract::kSchemaName);
  REQUIRE(snapshot["schema"]["version"] ==
          hacdcpf_compare::snapshot_contract::kSchemaVersion);
  REQUIRE(snapshot["adapter"]["name"] ==
          hacdcpf_compare::snapshot_contract::kAdapterName);
  REQUIRE(snapshot["master_dss"] ==
          "tests/data/opendss/minimal_regulator_1ph/Master.dss");
  REQUIRE(fs::path(snapshot["master_dss"].get<std::string>()).is_absolute() == false);
  REQUIRE(snapshot["converged"].get<bool>() == direct.converged);
  REQUIRE(snapshot["node_voltages"].size() == direct.node_voltages.size());
  REQUIRE(snapshot["buses"].size() == 3);
  REQUIRE(snapshot["pd_elements"].size() == direct.pd_element_results.size());
  REQUIRE(snapshot["transformer_states"].size() == direct.transformer_states.size());
  REQUIRE(snapshot["regcontrol_states"].size() == direct.regcontrol_results.size());
  REQUIRE(snapshot["circuit_losses_mw_mvar"].contains("p_mw"));
  REQUIRE(contract.snapshot.node_voltages.size() == direct.node_voltages.size());
  REQUIRE(contract.snapshot.pd_element_results.size() ==
          direct.pd_element_results.size());
  REQUIRE(contract.snapshot.transformer_states.size() ==
          direct.transformer_states.size());
  REQUIRE(contract.snapshot.regcontrol_results.size() ==
          direct.regcontrol_results.size());
  CHECK(contract.snapshot.line_results.size() == direct.line_results.size());
  CHECK(snapshot["units"] == hacdcpf_compare::snapshot_contract::units_to_json());
  CHECK(snapshot["pd_elements"][0].contains("element_kind"));
  CHECK(snapshot["units"]["terminal_power_raw"] == "kW / kvar");
  CHECK(snapshot["units"]["losses_raw"] == "W / var");
}

TEST_CASE("Official IEEE 13-node OpenDSS reference runs through the standalone snapshot adapter",
          "[integration][powerflow][distribution][opendss][adapter][official_feeder]") {
  const fs::path adapter = sibling_snapshot_adapter_path();
  REQUIRE(fs::exists(adapter));

#ifdef HACDCPF_PROJECT_ROOT
  const fs::path source_manifest =
      fs::path(HACDCPF_PROJECT_ROOT) /
      "external_data/opendss_ieee_pes/source_manifest.json";
  const fs::path master =
      fs::path(HACDCPF_PROJECT_ROOT) /
      "external_data/opendss_ieee_pes/opendss_reference/13_node/official_full/IEEE13Nodeckt.dss";
#else
  const fs::path source_manifest =
      "external_data/opendss_ieee_pes/source_manifest.json";
  const fs::path master =
      "external_data/opendss_ieee_pes/opendss_reference/13_node/official_full/IEEE13Nodeckt.dss";
#endif

  REQUIRE(fs::exists(source_manifest));
  REQUIRE(fs::exists(master));

  const json manifest = read_json_file(source_manifest);
  REQUIRE(manifest["sources"].is_array());
  CHECK(manifest["sources"].size() >= 6);

  auto find_source = [&](std::string_view source_id) -> const json* {
    const auto it = std::find_if(
        manifest["sources"].begin(), manifest["sources"].end(),
        [&](const json& item) {
          return item["source_id"].get<std::string>() == source_id;
        });
    return (it == manifest["sources"].end()) ? nullptr : &(*it);
  };

  const json* feeder13_zip = find_source("ieee_pes_feeder13_zip");
  REQUIRE(feeder13_zip != nullptr);
  CHECK((*feeder13_zip)["sha256"] ==
        "aa61a0d14ab1d91ee05ad9f4e7673852094cfcadfa37cc1e939963089fb054c4");
  const json* master_source = find_source("opendss_reference_13_node_master");
  REQUIRE(master_source != nullptr);
  CHECK((*master_source)["local_path"] ==
        "external_data/opendss_ieee_pes/opendss_reference/13_node/official_full/IEEE13Nodeckt.dss");

  ScopedTempReportDir out_dir("snapshot_adapter_ieee13");
  const fs::path output = out_dir.path / "snapshot.json";
  const std::string command =
      shell_exe(adapter) + " --master " + shell_quote(master) +
      " --out " + shell_quote(output);

  const int raw_status = std::system(command.c_str());
  REQUIRE(raw_status != -1);
  REQUIRE(decode_exit_code(raw_status) == 0);
  REQUIRE(fs::exists(output));

  const json snapshot = read_json_file(output);
  REQUIRE(snapshot["converged"].get<bool>() == true);
  CHECK(snapshot["master_dss"] ==
        "external_data/opendss_ieee_pes/opendss_reference/13_node/official_full/IEEE13Nodeckt.dss");
  CHECK(snapshot["buses"].size() == 16);
  CHECK(snapshot["transformer_states"].size() == 5);
  CHECK(snapshot["regcontrol_states"].size() == 3);
  CHECK(snapshot["pd_elements"].size() > 10);
  CHECK(snapshot["units"] == hacdcpf_compare::snapshot_contract::units_to_json());
}

TEST_CASE("Runtime regulator semantics matrix stays aligned between solver and OpenDSS bridge",
          "[integration][powerflow][distribution][opendss][regulator][semantics]") {
  using namespace hacdcpf::analysis;

  const std::vector<RuntimeRegulatorScenario> scenarios = {
      {
          .scenario_id = "remote_bus_no_ldc",
          .config =
              {
                  .circuit_name = "runtime_regulator_remote_bus_no_ldc",
                  .winding = 2,
                  .tap_winding = 2,
                  .transformer_tap_side = 1,
                  .monitored_bus = 3,
                  .monitored_bus_name = "bus3",
                  .vreg_volts = 122.0,
                  .band_volts = 2.0,
                  .ptratio = 60.0,
                  .remote_ptratio = 60.0,
                  .ct_primary_amps = 300.0,
                  .r_volts = 0.0,
                  .x_volts = 0.0,
                  .max_tap_change = 1,
              },
          .expect_control_converged = true,
          .expect_dss_converged = true,
          .expect_remote_bus = true,
          .expect_line_drop_compensation = false,
          .compare_direct_control_voltage = true,
          .expected_stop_reason = "within_band",
      },
      {
          .scenario_id = "local_pt_no_ldc",
          .config =
              {
                  .circuit_name = "runtime_regulator_local_pt_no_ldc",
                  .winding = 2,
                  .tap_winding = 2,
                  .transformer_tap_side = 1,
                  .monitored_bus = 0,
                  .vreg_volts = 122.0,
                  .band_volts = 2.0,
                  .ptratio = 60.0,
                  .remote_ptratio = 60.0,
                  .ct_primary_amps = 300.0,
                  .r_volts = 0.0,
                  .x_volts = 0.0,
                  .max_tap_change = 1,
              },
          .expect_control_converged = true,
          .expect_dss_converged = true,
          .expect_remote_bus = false,
          .expect_line_drop_compensation = false,
          .compare_direct_control_voltage = true,
          .expected_stop_reason = "within_band",
      },
      {
          .scenario_id = "local_pt_with_ldc",
          .config =
              {
                  .circuit_name = "runtime_regulator_local_pt_with_ldc",
                  .winding = 2,
                  .tap_winding = 2,
                  .transformer_tap_side = 1,
                  .monitored_bus = 0,
                  .vreg_volts = 122.0,
                  .band_volts = 2.0,
                  .ptratio = 60.0,
                  .remote_ptratio = 60.0,
                  .ct_primary_amps = 300.0,
                  .r_volts = 0.1,
                  .x_volts = 0.2,
                  .max_tap_change = 1,
              },
          .expect_control_converged = true,
          .expect_dss_converged = true,
          .expect_remote_bus = false,
          .expect_line_drop_compensation = true,
          .compare_direct_control_voltage = false,
          .expected_stop_reason = "within_band",
      },
      {
          .scenario_id = "tapwinding_1",
          .config =
              {
                  .circuit_name = "runtime_regulator_tapwinding_1",
                  .winding = 2,
                  .tap_winding = 1,
                  .transformer_tap_side = 0,
                  .monitored_bus = 0,
                  .vreg_volts = 122.0,
                  .band_volts = 2.0,
                  .ptratio = 60.0,
                  .remote_ptratio = 60.0,
                  .ct_primary_amps = 300.0,
                  .r_volts = 0.0,
                  .x_volts = 0.0,
                  .max_tap_change = 1,
              },
          .expect_control_converged = true,
          .expect_dss_converged = true,
          .expect_remote_bus = false,
          .expect_line_drop_compensation = false,
          .compare_direct_control_voltage = true,
          .expected_stop_reason = "within_band",
      },
      {
          .scenario_id = "max_tap_change_2",
          .config =
              {
                  .circuit_name = "runtime_regulator_max_tap_change_2",
                  .winding = 2,
                  .tap_winding = 2,
                  .transformer_tap_side = 1,
                  .monitored_bus = 3,
                  .monitored_bus_name = "bus3",
                  .vreg_volts = 122.0,
                  .band_volts = 2.0,
                  .ptratio = 60.0,
                  .remote_ptratio = 60.0,
                  .ct_primary_amps = 300.0,
                  .r_volts = 0.0,
                  .x_volts = 0.0,
                  .max_tap_change = 2,
              },
          .expect_control_converged = true,
          .expect_dss_converged = true,
          .expect_remote_bus = true,
          .expect_line_drop_compensation = false,
          .compare_direct_control_voltage = true,
          .expected_stop_reason = "within_band",
      },
      {
          .scenario_id = "remote_bus_distinct_remote_ptratio",
          .config =
              {
                  .circuit_name = "runtime_regulator_remote_bus_distinct_remote_ptratio",
                  .winding = 2,
                  .tap_winding = 2,
                  .transformer_tap_side = 1,
                  .monitored_bus = 3,
                  .monitored_bus_name = "bus3",
                  .vreg_volts = 122.0,
                  .band_volts = 2.0,
                  .ptratio = 60.0,
                  .remote_ptratio = 55.0,
                  .ct_primary_amps = 300.0,
                  .r_volts = 0.0,
                  .x_volts = 0.0,
                  .max_tap_change = 1,
              },
          .expect_control_converged = true,
          .expect_dss_converged = true,
          .expect_remote_bus = true,
          .expect_line_drop_compensation = false,
          .compare_direct_control_voltage = true,
          .expected_stop_reason = "within_band",
      },
      {
          .scenario_id = "reversible_fail_closed",
          .config =
              {
                  .circuit_name = "runtime_regulator_reversible_fail_closed",
                  .winding = 2,
                  .tap_winding = 2,
                  .transformer_tap_side = 1,
                  .monitored_bus = 3,
                  .monitored_bus_name = "bus3",
                  .vreg_volts = 122.0,
                  .band_volts = 2.0,
                  .ptratio = 60.0,
                  .remote_ptratio = 60.0,
                  .ct_primary_amps = 300.0,
                  .r_volts = 0.0,
                  .x_volts = 0.0,
                  .max_tap_change = 1,
                  .reversible = true,
              },
          .expect_control_converged = false,
          .expect_dss_converged = true,
          .expect_remote_bus = true,
          .expect_line_drop_compensation = false,
          .compare_direct_control_voltage = false,
          .expected_stop_reason = "unsupported_reversible_control",
      },
      {
          .scenario_id = "remote_bus_with_ldc_fail_closed",
          .config =
              {
                  .circuit_name = "runtime_regulator_remote_bus_with_ldc",
                  .winding = 2,
                  .tap_winding = 2,
                  .transformer_tap_side = 1,
                  .monitored_bus = 3,
                  .monitored_bus_name = "bus3",
                  .vreg_volts = 122.0,
                  .band_volts = 2.0,
                  .ptratio = 60.0,
                  .remote_ptratio = 60.0,
                  .ct_primary_amps = 300.0,
                  .r_volts = 0.1,
                  .x_volts = 0.2,
                  .max_tap_change = 1,
              },
          .expect_control_converged = false,
          .expect_dss_converged = true,
          .expect_remote_bus = true,
          .expect_line_drop_compensation = true,
          .compare_direct_control_voltage = false,
          .expected_stop_reason = "unsupported_remote_bus_with_ldc",
      },
  };

  DPFOptions options;
  options.max_iter = 200;
  options.max_control_iter = 100;
  options.tol = 1e-10;
  options.include_shunts = true;

  for (const auto& scenario : scenarios) {
    DYNAMIC_SECTION("runtime regulator semantics: " + scenario.scenario_id) {
      const auto repo_case = build_runtime_regulator_repo_case(scenario.config);
      const auto result = solve_distribution_pf(repo_case, options);
      const auto& state = get_single_regulator_state(result);
      const auto& last_trace = get_last_regulator_trace(result);

      ScopedTempReportDir temp_case("runtime_regulator_" + scenario.scenario_id);
      const fs::path master = temp_case.path / "Master.dss";
      write_runtime_regulator_master_dss(master, scenario.config);

      const auto dss = hacdcpf::io::solve_opendss_snapshot(master);
      REQUIRE(dss.converged == scenario.expect_dss_converged);
      REQUIRE(dss.regcontrol_results.size() == 1);
      REQUIRE(dss.transformer_states.size() == 1);

      const auto& regcontrol = find_regcontrol(dss, "reg1");
      const auto& transformer_state = find_transformer_state(dss, "t12");
      const auto& tap_winding =
          find_transformer_winding_state(transformer_state, scenario.config.tap_winding);

      const std::string expected_monitored_key = key_for_bus_node(
          runtime_regulator_monitored_bus_name(scenario.config), 1);

      CHECK(regcontrol.transformer_name == "t12");
      CHECK(regcontrol.winding == scenario.config.winding);
      CHECK(regcontrol.tap_winding == scenario.config.tap_winding);
      CHECK(std::abs(regcontrol.forward_vreg_volts - scenario.config.vreg_volts) < 1e-12);
      CHECK(std::abs(regcontrol.forward_band_volts - scenario.config.band_volts) < 1e-12);
      CHECK(std::abs(regcontrol.ptratio - scenario.config.ptratio) < 1e-12);
      CHECK(std::abs(regcontrol.remote_ptratio - scenario.config.remote_ptratio) < 1e-12);
      CHECK(std::abs(regcontrol.ct_primary_amps - scenario.config.ct_primary_amps) < 1e-12);
      CHECK(std::abs(regcontrol.forward_r_volts - scenario.config.r_volts) < 1e-12);
      CHECK(std::abs(regcontrol.forward_x_volts - scenario.config.x_volts) < 1e-12);
      CHECK(regcontrol.max_tap_change == scenario.config.max_tap_change);
      CHECK(regcontrol.is_reversible == scenario.config.reversible);
      if (scenario.config.monitored_bus != 0) {
        CHECK(regcontrol.monitored_bus_name == expected_monitored_key);
      } else {
        CHECK((regcontrol.monitored_bus_name.empty() ||
               regcontrol.monitored_bus_name == expected_monitored_key));
      }

      CHECK(state.transformer_name == "t12");
      CHECK(state.winding == scenario.config.winding);
      CHECK(state.tap_winding == scenario.config.tap_winding);
      CHECK(state.monitored_bus == scenario.config.monitored_bus);
      CHECK(state.used_remote_bus == scenario.expect_remote_bus);
      CHECK(state.used_line_drop_compensation ==
            scenario.expect_line_drop_compensation);
      CHECK(std::abs(state.target_vreg_volts - scenario.config.vreg_volts) < 1e-12);
      CHECK(std::abs(state.band_volts - scenario.config.band_volts) < 1e-12);
      CHECK(std::abs(state.ptratio - scenario.config.ptratio) < 1e-12);
      CHECK(std::abs(state.remote_ptratio - scenario.config.remote_ptratio) < 1e-12);
      CHECK(std::abs(state.ct_primary_amps - scenario.config.ct_primary_amps) < 1e-12);
      CHECK(std::abs(state.r_volts - scenario.config.r_volts) < 1e-12);
      CHECK(std::abs(state.x_volts - scenario.config.x_volts) < 1e-12);
      CHECK(state.max_tap_change == scenario.config.max_tap_change);
      if (scenario.expect_line_drop_compensation &&
          scenario.expect_control_converged) {
        CHECK(state.line_drop_compensation_magnitude_volts > 0.0);
      } else {
        CHECK(std::abs(state.line_drop_compensation_magnitude_volts) < 1e-12);
      }
      CHECK(last_trace.stop_reason == scenario.expected_stop_reason);
      CHECK(state.stop_reason == scenario.expected_stop_reason);

      for (const auto& trace : result.regulator_trace) {
        CHECK(std::abs(trace.next_tap_number - trace.current_tap_number) <=
              scenario.config.max_tap_change);
      }

      if (!scenario.expect_control_converged) {
        REQUIRE(result.converged == false);
        REQUIRE(result.control_converged == false);
        CHECK(state.converged == false);
        CHECK(last_trace.decision_reason == "blocked");
        continue;
      }

      REQUIRE(result.converged);
      REQUIRE(result.control_converged);
      CHECK(state.converged == true);
      CHECK(state.final_tap_number == regcontrol.tap_number);
      CHECK(std::abs(state.final_tap_pu - tap_winding.tap_pu) < 1e-12);
      CHECK((last_trace.decision_reason == "within_band" ||
             last_trace.stop_reason == "within_band"));

      const auto maps = build_opendss_maps(dss);
      const auto& xfmr = find_pd_element(
          dss, hacdcpf::io::OpenDSSPDElementKind::Transformer, "t12");
      const auto& line = find_pd_element(
          dss, hacdcpf::io::OpenDSSPDElementKind::Line, "l1");

      auto final_repo_case = repo_case;
      final_repo_case.ac.transformers_2w.front().tap_pos = state.final_tap_pos;
      const auto projected = hacdcpf::project_to_canonical_models(final_repo_case);
      const int transformer_branch_pos = find_projected_branch_position(
          projected, hacdcpf::BranchOriginType::Transformer2W, 1);
      const int line_branch_pos = find_branch_position_by_name(projected.ac, "l1");

      REQUIRE(maps.powers.contains(key_for_transformer_terminal_node("t12", 1, 1)));
      CHECK(std::abs(
                maps.powers.at(key_for_transformer_terminal_node("t12", 1, 1)).first -
                result.p_branch_mw[static_cast<std::size_t>(transformer_branch_pos)]) < 1e-3);
      CHECK(std::abs(
                maps.powers.at(key_for_transformer_terminal_node("t12", 1, 1)).second -
                result.q_branch_mvar[static_cast<std::size_t>(transformer_branch_pos)]) < 1e-3);

      REQUIRE(maps.powers.contains(key_for_line_terminal_node("l1", 1, 1)));
      CHECK(std::abs(
                maps.powers.at(key_for_line_terminal_node("l1", 1, 1)).first -
                result.p_branch_mw[static_cast<std::size_t>(line_branch_pos)]) < 1e-3);
      CHECK(std::abs(
                maps.powers.at(key_for_line_terminal_node("l1", 1, 1)).second -
                result.q_branch_mvar[static_cast<std::size_t>(line_branch_pos)]) < 1e-3);

      const auto total_branch_losses =
          hacdcpf_compare::sum_all_pd_element_terminal_powers_to_mw_mvar(dss);
      CHECK(std::abs(total_branch_losses.p_mw - result.total_p_loss_mw) < 1e-3);
      CHECK(std::abs(total_branch_losses.q_mvar - result.total_q_loss_mvar) < 1e-3);

      const auto xfmr_terminal_sum =
          hacdcpf_compare::sum_pd_element_terminal_powers_to_mw_mvar(xfmr);
      const auto xfmr_losses =
          hacdcpf_compare::opendss_element_losses_to_mw_mvar(xfmr);
      CHECK(std::abs(xfmr_terminal_sum.p_mw - xfmr_losses.p_mw) < 1e-6);
      CHECK(std::abs(xfmr_terminal_sum.q_mvar - xfmr_losses.q_mvar) < 1e-6);

      const auto line_terminal_sum =
          hacdcpf_compare::sum_pd_element_terminal_powers_to_mw_mvar(line);
      const auto line_losses =
          hacdcpf_compare::opendss_element_losses_to_mw_mvar(line);
      CHECK(std::abs(line_terminal_sum.p_mw - line_losses.p_mw) < 1e-6);
      CHECK(std::abs(line_terminal_sum.q_mvar - line_losses.q_mvar) < 1e-6);

      const auto circuit_losses =
          hacdcpf_compare::opendss_circuit_losses_to_mw_mvar(dss);
      CHECK(std::abs(total_branch_losses.p_mw - circuit_losses.p_mw) < 1e-6);
      CHECK(std::abs(total_branch_losses.q_mvar - circuit_losses.q_mvar) < 1e-6);

      if (scenario.compare_direct_control_voltage) {
        REQUIRE(maps.vm_vln.contains(expected_monitored_key));
        const double dss_control_voltage = compute_dss_regulator_control_voltage_volts(
            build_voltage_index(dss),
            build_terminal_power_index(dss),
            xfmr,
            regcontrol,
            runtime_regulator_monitored_bus_name(scenario.config),
            1,
            scenario.config.monitored_bus == 0);
        CHECK(std::abs(dss_control_voltage - state.control_voltage_volts) < 0.2);
      }
    }
  }
}

TEST_CASE("Transformer3W tap_side is normalized onto only the attached pair branches",
          "[integration][powerflow][distribution][tap_semantics][transformer3w]") {
  const double tapped_pu = 1.05;

  auto hv_case = hacdcpf_compare_fixtures::build_transformer3w_tap_projection_case(0);
  auto hv_projected = hacdcpf::project_to_canonical_models(hv_case);
  CHECK(std::abs(hv_projected.ac.branches[static_cast<std::size_t>(
                     find_projected_branch_position_by_pair(hv_projected, 1, 0))].tap -
                 tapped_pu) < 1e-12);
  CHECK(std::abs(hv_projected.ac.branches[static_cast<std::size_t>(
                     find_projected_branch_position_by_pair(hv_projected, 1, 1))].tap -
                 tapped_pu) < 1e-12);
  CHECK(std::abs(hv_projected.ac.branches[static_cast<std::size_t>(
                     find_projected_branch_position_by_pair(hv_projected, 1, 2))].tap -
                 1.0) < 1e-12);

  auto mv_case = hacdcpf_compare_fixtures::build_transformer3w_tap_projection_case(1);
  auto mv_projected = hacdcpf::project_to_canonical_models(mv_case);
  CHECK(std::abs(mv_projected.ac.branches[static_cast<std::size_t>(
                     find_projected_branch_position_by_pair(mv_projected, 1, 0))].tap -
                 (1.0 / tapped_pu)) < 1e-12);
  CHECK(std::abs(mv_projected.ac.branches[static_cast<std::size_t>(
                     find_projected_branch_position_by_pair(mv_projected, 1, 1))].tap -
                 1.0) < 1e-12);
  CHECK(std::abs(mv_projected.ac.branches[static_cast<std::size_t>(
                     find_projected_branch_position_by_pair(mv_projected, 1, 2))].tap -
                 tapped_pu) < 1e-12);

  auto lv_case = hacdcpf_compare_fixtures::build_transformer3w_tap_projection_case(2);
  auto lv_projected = hacdcpf::project_to_canonical_models(lv_case);
  CHECK(std::abs(lv_projected.ac.branches[static_cast<std::size_t>(
                     find_projected_branch_position_by_pair(lv_projected, 1, 0))].tap -
                 1.0) < 1e-12);
  CHECK(std::abs(lv_projected.ac.branches[static_cast<std::size_t>(
                     find_projected_branch_position_by_pair(lv_projected, 1, 1))].tap -
                 (1.0 / tapped_pu)) < 1e-12);
  CHECK(std::abs(lv_projected.ac.branches[static_cast<std::size_t>(
                     find_projected_branch_position_by_pair(lv_projected, 1, 2))].tap -
                 (1.0 / tapped_pu)) < 1e-12);
}

TEST_CASE("Transformer3W tap projection converges on a radial smoke case without wrong-way voltage movement",
          "[integration][powerflow][distribution][tap_semantics][transformer3w][smoke]") {
  using namespace hacdcpf::analysis;

  DPFOptions options;
  options.max_iter = 200;
  options.tol = 1e-10;

  auto neutral_case = hacdcpf_compare_fixtures::build_transformer3w_tap_smoke_case();
  neutral_case.ac.transformers_3w.front().tap_pos = 0;
  const auto neutral = solve_distribution_pf(neutral_case, options);
  REQUIRE(neutral.converged);

  const auto tapped_case = hacdcpf_compare_fixtures::build_transformer3w_tap_smoke_case();
  const auto tapped = solve_distribution_pf(tapped_case, options);
  REQUIRE(tapped.converged);

  CHECK(tapped.vm_pu[2] > neutral.vm_pu[2]);
}

TEST_CASE("Transformer2W tap projection preserves physical terminal equations",
          "[integration][powerflow][distribution][tap_semantics][equivalence][transformer2w]") {
  const auto hv_case = hacdcpf_compare_fixtures::build_3bus_fixed_tap_transformer();
  const auto hv_projected = hacdcpf::project_to_canonical_models(hv_case);
  const auto lv_case = hacdcpf_compare_fixtures::build_3bus_fixed_tap_transformer_lv_side();
  const auto lv_projected = hacdcpf::project_to_canonical_models(lv_case);
  const auto spec = hacdcpf_compare_fixtures::fixed_tap_transformer_case();

  const Complex parent_voltage = polar_pu(1.012, -0.7);
  const Complex child_bus_current{0.173, -0.061};
  const double base_mva = hv_case.base_mva;
  const auto [r_pu, x_pu] = rx_from_vk_vkr(
      hv_case.ac.transformers_2w.front().vk_percent,
      hv_case.ac.transformers_2w.front().vkr_percent,
      hv_case.base_mva,
      hv_case.ac.transformers_2w.front().sn_mva);

  const int hv_branch_pos = find_projected_branch_position(
      hv_projected, hacdcpf::BranchOriginType::Transformer2W,
      spec.transformer_origin_index);
  const auto hv_equiv = physical_transformer_equations_from_parent(
      r_pu, x_pu, PhysicalTapPlacement::FromSide,
      spec.configured_transformer_tap_pu, parent_voltage, child_bus_current, base_mva);
  const auto hv_projected_equiv = projected_branch_equations_from_parent(
      hv_projected.ac.branches[static_cast<std::size_t>(hv_branch_pos)],
      parent_voltage, child_bus_current, base_mva);
  CHECK(max_abs_branch_equation_delta(hv_equiv, hv_projected_equiv) < 1e-12);

  const int lv_branch_pos = find_projected_branch_position(
      lv_projected, hacdcpf::BranchOriginType::Transformer2W,
      spec.transformer_origin_index);
  const auto lv_equiv = physical_transformer_equations_from_parent(
      r_pu, x_pu, PhysicalTapPlacement::ToSide,
      spec.configured_transformer_tap_pu, parent_voltage, child_bus_current, base_mva);
  const auto lv_projected_equiv = projected_branch_equations_from_parent(
      lv_projected.ac.branches[static_cast<std::size_t>(lv_branch_pos)],
      parent_voltage, child_bus_current, base_mva);
  CHECK(max_abs_branch_equation_delta(lv_equiv, lv_projected_equiv) < 1e-12);
}

TEST_CASE("Transformer3W canonical projection preserves the coupled-kron equivalent admittance",
          "[integration][powerflow][distribution][tap_semantics][equivalence][transformer3w]") {
  for (int tap_side = 0; tap_side <= 2; ++tap_side) {
    auto sys = hacdcpf_compare_fixtures::build_transformer3w_tap_projection_case(tap_side);
    const auto projected = hacdcpf::project_to_canonical_models(sys);
    const auto& tr = sys.ac.transformers_3w.front();
    const auto projected_y =
        projected_transformer3w_pair_ybus(projected, tr.index, tr);
    const auto reference_y =
        coupled_kron_transformer3w_reference_ybus(tr, sys.base_mva);

    CAPTURE(tap_side);
    CHECK(max_abs_ybus_delta(projected_y, reference_y) < 1e-12);
  }
}

TEST_CASE("Transformer3W isolated triangle production trial only triggers on the targeted topology",
          "[integration][powerflow][distribution][transformer3w][specialized_path]") {
  using namespace hacdcpf::analysis;

  DPFOptions options;
  options.max_iter = 200;
  options.tol = 1e-10;
  options.include_shunts = true;

  const auto transformer3w_case =
      hacdcpf_compare_fixtures::build_minimal_transformer3w_cross_check_case();
  const auto transformer3w_result = solve_distribution_pf(transformer3w_case, options);
  REQUIRE(transformer3w_result.converged == true);
  CHECK(transformer3w_result.specialized_path_applied == true);
  CHECK(transformer3w_result.specialized_path_id ==
        "transformer3w_isolated_triangle_dense_nr_trial");
  CHECK(transformer3w_result.specialized_path_fail_close_reason.empty());

  const auto regulator_case =
      hacdcpf_compare_fixtures::build_minimal_regulator_1ph();
  const auto regulator_result = solve_distribution_pf(regulator_case, options);
  REQUIRE(regulator_result.converged == true);
  CHECK(regulator_result.specialized_path_applied == false);
  CHECK(regulator_result.specialized_path_id.empty());
  CHECK(regulator_result.specialized_path_fail_close_reason.empty());
}

TEST_CASE("Regulator failure path keeps original bus indexing after max_control_iter with bus merge",
          "[integration][powerflow][distribution][regulator][failure][bus_merge]") {
  using namespace hacdcpf::analysis;

  const auto sys =
      hacdcpf_compare_fixtures::build_minimal_regulator_1ph_with_merged_monitor_bus();
  const auto projected = hacdcpf::project_to_canonical_models(sys);
  REQUIRE(projected.bus_merge_map.has_value());
  REQUIRE(projected.ac.buses.size() < sys.ac.buses.size());

  DPFOptions options;
  options.max_iter = 200;
  options.max_control_iter = 1;
  options.tol = 1e-10;
  options.include_shunts = true;

  const auto result = solve_distribution_pf(sys, options);
  REQUIRE(result.converged == false);
  REQUIRE(result.control_converged == false);
  CHECK(result.vm_pu.size() == sys.ac.buses.size());
  CHECK(result.va_deg.size() == sys.ac.buses.size());
  REQUIRE(result.regulator_trace.empty() == false);
  REQUIRE(result.regulator_states.size() == 1);
  CHECK(result.regulator_trace.back().stop_reason == "max_control_iter_reached");
  CHECK(result.regulator_states.front().stop_reason == "max_control_iter_reached");
  CHECK(std::abs(result.vm_pu[2] - result.vm_pu[3]) < 1e-12);
  CHECK(std::abs(result.va_deg[2] - result.va_deg[3]) < 1e-12);
}

TEST_CASE("Regulator failure path keeps original bus indexing after cycle detection with bus merge",
          "[integration][powerflow][distribution][regulator][failure][cycle][bus_merge]") {
  using namespace hacdcpf::analysis;

  const auto sys =
      hacdcpf_compare_fixtures::build_minimal_regulator_cycle_1ph_with_merged_monitor_bus();
  const auto projected = hacdcpf::project_to_canonical_models(sys);
  REQUIRE(projected.bus_merge_map.has_value());
  REQUIRE(projected.ac.buses.size() < sys.ac.buses.size());

  DPFOptions options;
  options.max_iter = 200;
  options.max_control_iter = 10;
  options.tol = 1e-10;
  options.include_shunts = true;

  const auto result = solve_distribution_pf(sys, options);
  REQUIRE(result.converged == false);
  REQUIRE(result.control_converged == false);
  CHECK(result.vm_pu.size() == sys.ac.buses.size());
  CHECK(result.va_deg.size() == sys.ac.buses.size());
  REQUIRE(result.regulator_trace.size() >= 2);
  REQUIRE(result.regulator_states.size() == 1);
  CHECK(result.regulator_trace.back().stop_reason == "cycle_detected");
  CHECK(result.regulator_states.front().stop_reason == "cycle_detected");
  CHECK(std::abs(result.vm_pu[2] - result.vm_pu[3]) < 1e-12);
  CHECK(std::abs(result.va_deg[2] - result.va_deg[3]) < 1e-12);
}

TEST_CASE("Transformer tap_side is normalized onto ACBranch.from_bus",
          "[integration][powerflow][distribution][opendss][tap_semantics]") {
  auto hv_side_case = hacdcpf_compare_fixtures::build_3bus_fixed_tap_transformer();
  auto hv_projected = hacdcpf::project_to_canonical_models(hv_side_case);
  const auto hv_spec = hacdcpf_compare_fixtures::fixed_tap_transformer_case();
  const int hv_branch_pos = find_projected_branch_position(
      hv_projected, hacdcpf::BranchOriginType::Transformer2W,
      hv_spec.transformer_origin_index);
  CHECK(std::abs(hv_projected.ac.branches[static_cast<std::size_t>(hv_branch_pos)].tap -
                 hv_spec.configured_transformer_tap_pu) < 1e-12);
  const double hv_r_pu =
      hv_projected.ac.branches[static_cast<std::size_t>(hv_branch_pos)].r_pu;
  const double hv_x_pu =
      hv_projected.ac.branches[static_cast<std::size_t>(hv_branch_pos)].x_pu;

  auto lv_side_case = hacdcpf_compare_fixtures::build_3bus_fixed_tap_transformer();
  REQUIRE(lv_side_case.ac.transformers_2w.size() == 1);
  lv_side_case.ac.transformers_2w.front().tap_side = 1;
  auto lv_projected = hacdcpf::project_to_canonical_models(lv_side_case);
  const int lv_branch_pos = find_projected_branch_position(
      lv_projected, hacdcpf::BranchOriginType::Transformer2W,
      hv_spec.transformer_origin_index);
  CHECK(std::abs(lv_projected.ac.branches[static_cast<std::size_t>(lv_branch_pos)].tap -
                 (1.0 / hv_spec.configured_transformer_tap_pu)) < 1e-12);
  CHECK(std::abs(lv_projected.ac.branches[static_cast<std::size_t>(lv_branch_pos)].r_pu -
                 hv_r_pu * hv_spec.configured_transformer_tap_pu *
                     hv_spec.configured_transformer_tap_pu) < 1e-12);
  CHECK(std::abs(lv_projected.ac.branches[static_cast<std::size_t>(lv_branch_pos)].x_pu -
                 hv_x_pu * hv_spec.configured_transformer_tap_pu *
                     hv_spec.configured_transformer_tap_pu) < 1e-12);
}

TEST_CASE("LV-side fixed tap matches OpenDSS end-to-end, not just in projection",
          "[integration][powerflow][distribution][opendss][tap_semantics][lv_side]") {
  using namespace hacdcpf::analysis;

  const auto repo_case = hacdcpf_compare_fixtures::build_3bus_fixed_tap_transformer_lv_side();
  const auto projected = hacdcpf::project_to_canonical_models(repo_case);

  DPFOptions options;
  options.max_iter = 200;
  options.tol = 1e-10;
  options.include_shunts = true;
  const auto repo = solve_distribution_pf(repo_case, options);
  REQUIRE(repo.converged);

  ScopedTempReportDir temp_case("lv_side_fixed_tap_case");
  const fs::path master = temp_case.path / "Master.dss";
  {
    std::ofstream os(master);
    REQUIRE(os.good());
    os << "Clear\n"
          "! Runtime-generated LV-side fixed tap regression case.\n"
          "New Circuit.minimal_tap_transformer_1ph_lv_side phases=1 bus1=sourcebus basekv=7.2 pu=1.0\n\n"
          "New Transformer.t12 phases=1 buses=[sourcebus.1, bus2.1] conns=[wye, wye] "
          "kvas=[10000, 10000] kvs=[7.2, 7.2] xhl=6.0 %loadloss=2.0 %noloadloss=0 %imag=0 "
          "taps=[1.0, 1.05]\n\n"
          "New Line.l1 phases=1 bus1=bus2.1 bus2=bus3.1 length=1 units=none "
          "r1=0.41472 x1=0.7776 r0=0.41472 x0=0.7776 c1=0 c0=0\n\n"
          "New Load.load2 phases=1 bus1=bus2.1 conn=wye model=1 kv=7.2 kw=500 kvar=300 "
          "vminpu=0.0 vmaxpu=2.0\n"
          "New Load.load3 phases=1 bus1=bus3.1 conn=wye model=1 kv=7.2 kw=800 kvar=400 "
          "vminpu=0.0 vmaxpu=2.0\n\n"
          "Set VoltageBases=[12.470765814495916]\n"
          "CalcVoltageBases\n"
          "Solve\n";
  }

  const auto dss = hacdcpf::io::solve_opendss_snapshot(master);
  REQUIRE(dss.converged);

  const auto maps = build_opendss_maps(dss);
  const auto& xfmr = find_pd_element(
      dss, hacdcpf::io::OpenDSSPDElementKind::Transformer, "t12");
  const auto& line = find_pd_element(
      dss, hacdcpf::io::OpenDSSPDElementKind::Line, "l1");

  const int transformer_branch_pos = find_projected_branch_position(
      projected, hacdcpf::BranchOriginType::Transformer2W, 1);
  const int line_branch_pos = find_branch_position_by_name(projected.ac, "l1");

  CHECK(std::abs(maps.vm_pu.at("sourcebus.1") - repo.vm_pu[0]) < 1e-3);
  CHECK(std::abs(maps.vm_pu.at("bus2.1") - repo.vm_pu[1]) < 1e-3);
  CHECK(std::abs(maps.vm_pu.at("bus3.1") - repo.vm_pu[2]) < 1e-3);

  CHECK(std::abs(maps.powers.at(key_for_transformer_terminal_node("t12", 1, 1)).first -
                 repo.p_branch_mw[static_cast<std::size_t>(transformer_branch_pos)]) < 1e-3);
  CHECK(std::abs(maps.powers.at(key_for_transformer_terminal_node("t12", 1, 1)).second -
                 repo.q_branch_mvar[static_cast<std::size_t>(transformer_branch_pos)]) < 1e-3);

  CHECK(std::abs(maps.powers.at(key_for_line_terminal_node("l1", 1, 1)).first -
                 repo.p_branch_mw[static_cast<std::size_t>(line_branch_pos)]) < 1e-3);
  CHECK(std::abs(maps.powers.at(key_for_line_terminal_node("l1", 1, 1)).second -
                 repo.q_branch_mvar[static_cast<std::size_t>(line_branch_pos)]) < 1e-3);

  const auto total_branch_losses =
      hacdcpf_compare::sum_all_pd_element_terminal_powers_to_mw_mvar(dss);
  CHECK(std::abs(total_branch_losses.p_mw - repo.total_p_loss_mw) < 1e-3);
  CHECK(std::abs(total_branch_losses.q_mvar - repo.total_q_loss_mvar) < 1e-3);

  const auto xfmr_terminal_sum =
      hacdcpf_compare::sum_pd_element_terminal_powers_to_mw_mvar(xfmr);
  const auto xfmr_losses =
      hacdcpf_compare::opendss_element_losses_to_mw_mvar(xfmr);
  CHECK(std::abs(xfmr_terminal_sum.p_mw - xfmr_losses.p_mw) < 1e-6);
  CHECK(std::abs(xfmr_terminal_sum.q_mvar - xfmr_losses.q_mvar) < 1e-6);

  const auto line_terminal_sum =
      hacdcpf_compare::sum_pd_element_terminal_powers_to_mw_mvar(line);
  const auto line_losses =
      hacdcpf_compare::opendss_element_losses_to_mw_mvar(line);
  CHECK(std::abs(line_terminal_sum.p_mw - line_losses.p_mw) < 1e-6);
  CHECK(std::abs(line_terminal_sum.q_mvar - line_losses.q_mvar) < 1e-6);
}

TEST_CASE("Canonical compare artifact keeps regulator tiers and evidence aligned",
          "[integration][powerflow][distribution][opendss][regulator][artifact]") {
  const auto spec = hacdcpf_compare_fixtures::accepted_regulator_case();
  const auto local_pt_ldc_spec =
      hacdcpf_compare_fixtures::accepted_local_pt_with_ldc_regulator_case();
  const auto remote_ptratio_spec =
      hacdcpf_compare_fixtures::accepted_remote_ptratio_regulator_case();
  const fs::path compare_tool = sibling_compare_tool_path();
  REQUIRE(fs::exists(compare_tool));

  ScopedTempReportDir report_dir("regulator_compare");
  const std::string command =
      shell_exe(compare_tool) + " --report-dir " + shell_quote(report_dir.path);

  const int raw_status = std::system(command.c_str());
  REQUIRE(raw_status != -1);
  // The tool exits non-zero because the 8 known P2 NR cases have unresolved
  // model-level gaps. Do not gate on the global exit code here; verify the
  // regulator/transformer3w artifact content independently below.

  const json summary = read_json_file(report_dir.path / "compare_summary.json");
  const json manifest = read_json_file(report_dir.path / "case_manifest.json");
  const std::string summary_markdown =
      read_text_file(report_dir.path / "compare_summary.md");
  const auto target_case = std::find_if(
      summary["cases"].begin(), summary["cases"].end(),
      [](const json& item) {
        return item["case_id"].get<std::string>() == "minimal_regulator_1ph";
      });
  REQUIRE(target_case != summary["cases"].end());
  CHECK((*target_case)["comparison_status"].get<std::string>() == "accepted");
  CHECK((*target_case)["comparison_pass"].get<bool>() == true);
  REQUIRE((*target_case)["bridge_validation_pass"].get<bool>() == true);
  CHECK((*target_case)["control_trace_validation_pass"].get<bool>() == true);
  CHECK((*target_case)["error_budget"]["all_metric_thresholds_passed"].get<bool>() == true);
  CHECK((*target_case)["failed_metrics"].empty());
  REQUIRE((*target_case)["projection_diagnostics"]["projected_branch"]
                 ["impedance_normalization"]["scaled_from_to_side"].get<bool>() == true);
  CHECK(std::abs((*target_case)["projection_diagnostics"]["projected_branch"]
                              ["impedance_normalization"]["impedance_scale"].get<double>() -
                  (spec.expected_final_tap_pu * spec.expected_final_tap_pu)) < 1e-12);
  CHECK((*target_case)["control_trace_validation"]["opendss_control_oracle"]
                       ["control_iteration_count"] == 1);
  CHECK((*target_case)["control_trace_validation"]["opendss_control_oracle"]
                       ["tap_operation_count_source"] == "control_iterations_fallback");

  const auto local_pt_ldc_case = std::find_if(
      summary["cases"].begin(), summary["cases"].end(),
      [](const json& item) {
        return item["case_id"].get<std::string>() ==
               "minimal_regulator_local_pt_ldc_1ph";
      });
  REQUIRE(local_pt_ldc_case != summary["cases"].end());
  CHECK((*local_pt_ldc_case)["comparison_status"].get<std::string>() == "accepted");
  CHECK((*local_pt_ldc_case)["comparison_pass"].get<bool>() == true);
  REQUIRE((*local_pt_ldc_case)["bridge_validation_pass"].get<bool>() == true);
  CHECK((*local_pt_ldc_case)["control_trace_validation_pass"].get<bool>() == true);
  CHECK((*local_pt_ldc_case)["error_budget"]["all_metric_thresholds_passed"].get<bool>() == true);
  CHECK((*local_pt_ldc_case)["failed_metrics"].empty());
  CHECK(std::abs(
            (*local_pt_ldc_case)["opendss"]["tapped_winding_state"]["tap_pu"].get<double>() -
            local_pt_ldc_spec.expected_final_tap_pu) < 1e-12);
  REQUIRE((*local_pt_ldc_case)["error_budget"]["dominant_error_metric"].is_null() == false);

  const auto remote_ptratio_case = std::find_if(
      summary["cases"].begin(), summary["cases"].end(),
      [](const json& item) {
        return item["case_id"].get<std::string>() ==
               "minimal_regulator_remote_ptratio_1ph";
      });
  REQUIRE(remote_ptratio_case != summary["cases"].end());
  CHECK((*remote_ptratio_case)["comparison_status"].get<std::string>() == "accepted");
  CHECK((*remote_ptratio_case)["comparison_pass"].get<bool>() == true);
  REQUIRE((*remote_ptratio_case)["bridge_validation_pass"].get<bool>() == true);
  CHECK((*remote_ptratio_case)["control_trace_validation_pass"].get<bool>() == true);
  CHECK((*remote_ptratio_case)["error_budget"]["all_metric_thresholds_passed"].get<bool>() == true);
  CHECK((*remote_ptratio_case)["failed_metrics"].empty());
  CHECK(std::abs(
            (*remote_ptratio_case)["bridge_validation"]["configured_control"]
                                  ["expected_remote_ptratio"].get<double>() -
            remote_ptratio_spec.remote_ptratio) < 1e-12);
  CHECK((*remote_ptratio_case)["error_budget"]["metrics"]["regulator_control_voltage_volts"]
            ["threshold"] == 0.2);

  const auto nr_meshed_case = std::find_if(
      summary["cases"].begin(), summary["cases"].end(),
      [](const json& item) {
        return item["case_id"].get<std::string>() == "minimal_meshed_3bus_3ph_nr";
      });
  REQUIRE(nr_meshed_case != summary["cases"].end());
  // P2 residual: meshed NR case has unresolved model-level gaps (known).
  CHECK((*nr_meshed_case)["acceptance_tier"] == "accepted_nr_meshed_3ph");

  const auto nr_phase_matrix_case = std::find_if(
      summary["cases"].begin(), summary["cases"].end(),
      [](const json& item) {
        return item["case_id"].get<std::string>() ==
               "minimal_phase_matrix_line_3bus_3ph_nr";
      });
  REQUIRE(nr_phase_matrix_case != summary["cases"].end());
  CHECK((*nr_phase_matrix_case)["acceptance_tier"] ==
        "accepted_nr_phase_matrix_line_3ph");
  // P2 residual: phase-matrix line NR case has unresolved model-level gaps (known).

  const auto nr_load_wye_zip_case = std::find_if(
      summary["cases"].begin(), summary["cases"].end(),
      [](const json& item) {
        return item["case_id"].get<std::string>() ==
               "minimal_load_wye_zip_3bus_3ph_nr";
      });
  REQUIRE(nr_load_wye_zip_case != summary["cases"].end());
  CHECK((*nr_load_wye_zip_case)["acceptance_tier"] ==
        "accepted_nr_load_semantics_3ph");
  // P2 residual: NR load-semantics cases have unresolved model-level gaps (known).

  const auto nr_load_wye_open_neutral_case = std::find_if(
      summary["cases"].begin(), summary["cases"].end(),
      [](const json& item) {
        return item["case_id"].get<std::string>() ==
               "minimal_load_wye_open_neutral_3bus_1ph_lateral_nr";
      });
  REQUIRE(nr_load_wye_open_neutral_case != summary["cases"].end());
  CHECK((*nr_load_wye_open_neutral_case)["acceptance_tier"] ==
        "accepted_nr_load_semantics_3ph");
  CHECK((*nr_load_wye_open_neutral_case)["comparison_status"] == "accepted");
  CHECK((*nr_load_wye_open_neutral_case)["comparison_pass"] == true);
  CHECK((*nr_load_wye_open_neutral_case)["failed_metrics"].empty());

  const auto nr_load_wye_impedance_grounded_case = std::find_if(
      summary["cases"].begin(), summary["cases"].end(),
      [](const json& item) {
        return item["case_id"].get<std::string>() ==
               "minimal_load_wye_impedance_grounded_3bus_1ph_lateral_nr";
      });
  REQUIRE(nr_load_wye_impedance_grounded_case != summary["cases"].end());
  CHECK((*nr_load_wye_impedance_grounded_case)["acceptance_tier"] ==
        "accepted_nr_load_semantics_3ph");
  // P2 residual: NR load-semantics cases have unresolved model-level gaps (known).

  const auto nr_load_wye_vmax_case = std::find_if(
      summary["cases"].begin(), summary["cases"].end(),
      [](const json& item) {
        return item["case_id"].get<std::string>() ==
               "minimal_load_wye_vmax_3bus_1ph_lateral_nr";
      });
  REQUIRE(nr_load_wye_vmax_case != summary["cases"].end());
  CHECK((*nr_load_wye_vmax_case)["acceptance_tier"] ==
        "accepted_nr_load_semantics_3ph");
  // P2 residual: NR load-semantics cases have unresolved model-level gaps (known).

  const auto nr_load_delta_power_case = std::find_if(
      summary["cases"].begin(), summary["cases"].end(),
      [](const json& item) {
        return item["case_id"].get<std::string>() ==
               "minimal_load_delta_power_3bus_3ph_nr";
      });
  REQUIRE(nr_load_delta_power_case != summary["cases"].end());
  CHECK((*nr_load_delta_power_case)["acceptance_tier"] ==
        "accepted_nr_load_semantics_3ph");
  // P2 residual: NR load-semantics cases have unresolved model-level gaps (known).

  const auto nr_load_delta_zip_case = std::find_if(
      summary["cases"].begin(), summary["cases"].end(),
      [](const json& item) {
        return item["case_id"].get<std::string>() ==
               "minimal_load_delta_zip_3bus_3ph_nr";
      });
  REQUIRE(nr_load_delta_zip_case != summary["cases"].end());
  CHECK((*nr_load_delta_zip_case)["acceptance_tier"] ==
        "accepted_nr_load_semantics_3ph");
  // P2 residual: NR load-semantics cases have unresolved model-level gaps (known).

  const auto nr_multi_source_case = std::find_if(
      summary["cases"].begin(), summary["cases"].end(),
      [](const json& item) {
        return item["case_id"].get<std::string>() == "minimal_multi_source_3bus_3ph_nr";
      });
  REQUIRE(nr_multi_source_case != summary["cases"].end());
  // P2 residual: multi-source NR case has unresolved model-level gaps (known).
  CHECK((*nr_multi_source_case)["acceptance_tier"] == "accepted_nr_multi_source_3ph");

  REQUIRE(summary.contains("regulator_semantics_matrix"));
  REQUIRE(manifest["scope"].contains("regulator_semantics_matrix"));
  CHECK(summary["regulator_semantics_matrix"]["remote_bus_no_ldc"]["coverage"] ==
        "accepted_compare");
  CHECK(summary["regulator_semantics_matrix"]["local_pt_with_ldc"]["coverage"] ==
        "accepted_compare");
  CHECK(summary["regulator_semantics_matrix"]["local_pt_with_ldc"]["evidence"]["cases"] ==
        json::array({"minimal_regulator_local_pt_ldc_1ph"}));
  CHECK(summary["regulator_semantics_matrix"]["tapwinding_1"]["coverage"] ==
        "solver_regression_only");
  CHECK(summary["regulator_semantics_matrix"]["max_tap_change_2"]["coverage"] ==
        "solver_regression_only");
  CHECK(summary["regulator_semantics_matrix"]["remote_ptratio_distinct_from_ptratio"]
            ["coverage"] == "accepted_compare");
  CHECK(summary["regulator_semantics_matrix"]["remote_ptratio_distinct_from_ptratio"]
            ["evidence"]["cases"] ==
        json::array({"minimal_regulator_remote_ptratio_1ph"}));
  CHECK(summary["regulator_semantics_matrix"]["remote_ptratio_distinct_from_ptratio"]
            ["evidence"]["regression_tests"] ==
        json::array({"runtime_regulator_semantics_matrix:remote_bus_distinct_remote_ptratio"}));
  CHECK(summary["regulator_semantics_matrix"]["remote_bus_with_ldc"]["coverage"] ==
        "unsupported");
  CHECK(summary["regulator_semantics_matrix"]["remote_bus_with_ldc"]["reason"] ==
        "solve_distribution_pf fail-closes with stop_reason "
        "unsupported_remote_bus_with_ldc so remote monitoring and LDC cannot "
        "be silently mixed.");
  CHECK(summary["regulator_semantics_matrix"]["reversible"]["coverage"] ==
        "unsupported");
  CHECK(summary["regulator_semantics_matrix"]["reversible"]["evidence"]["regression_tests"] ==
        json::array({"runtime_regulator_semantics_matrix:reversible_fail_closed"}));
  CHECK(manifest["scope"]["regulator_semantics_matrix"] ==
        summary["regulator_semantics_matrix"]);
  CHECK(summary_markdown.find(
            "remote-bus/no-LDC and local-PT+LDC and distinct RemotePTRatio are accepted compare") !=
        std::string::npos);
  CHECK(summary_markdown.find(
            "reversible control and remote-bus+LDC remain explicitly unsupported") !=
        std::string::npos);
  CHECK(summary["reproducibility"]["canonical_adapter_command"] ==
        "build/opendss_compare/opendss_snapshot_adapter --master "
        "tests/data/opendss/minimal_regulator_1ph/Master.dss");
  REQUIRE(summary.contains("adapter_contract_surface"));
  CHECK(summary["adapter_contract_surface"]["schema"]["name"] ==
        hacdcpf_compare::snapshot_contract::kSchemaName);
  CHECK(summary["adapter_contract_surface"]["schema"]["version"] ==
        hacdcpf_compare::snapshot_contract::kSchemaVersion);
  CHECK(summary["adapter_contract_surface"]["portable_master_dss"] == true);
  CHECK(summary["adapter_contract_surface"]["serializer_parser_roundtrip_consistent"] ==
        true);
  CHECK(summary["adapter_contract_surface"]["required_pd_element_kinds"] ==
        json::array({"line", "transformer"}));
  CHECK(manifest["scope"]["adapter_contract_surface"] ==
        summary["adapter_contract_surface"]);
  REQUIRE(summary.contains("benchmark_expansion_plan"));
  CHECK(summary["benchmark_expansion_plan"]["candidate_feeders"] ==
        json::array({"13_node", "34_node", "123_node"}));
  REQUIRE(summary.contains("official_feeder_reference_status"));
  CHECK(summary["official_feeder_reference_status"]["status"] ==
        "bridge_smoke_passed");
  CHECK(summary["official_feeder_reference_status"]["source_manifest"] ==
        "external_data/opendss_ieee_pes/source_manifest.json");
  CHECK(summary["official_feeder_reference_status"]["regcontrol_count"] == 3);
  CHECK(manifest["scope"]["official_feeder_reference_status"] ==
        summary["official_feeder_reference_status"]);
  REQUIRE(summary.contains("selection"));
  CHECK(summary["accepted_cases_all_passed"] == true);
  CHECK(summary["selection"]["mode"] == "full_catalog");
  CHECK(manifest["scope"]["selection"] == summary["selection"]);
  CHECK(manifest["scope"]["compared_modules"] ==
        json::array({"solve_distribution_pf(const ACSystem&)",
                     "solve_distribution_pf(const HybridPowerSystem&)",
                     "solve_three_phase_distribution_pf(const ThreePhaseACSystem&)",
                     "solve_three_phase_nr(const ThreePhaseACSystem&)"}));
  REQUIRE(summary.contains("tolerance_justification"));
  REQUIRE(manifest["scope"].contains("tolerance_justification"));
  CHECK(manifest["scope"]["tolerance_justification"] ==
        summary["tolerance_justification"]);
  CHECK(summary["tolerance_justification"]["source_type"] ==
        "repository_machine_enforced_policy");

  const auto three_phase_tolerance_family = std::find_if(
      summary["tolerance_justification"]["threshold_families"].begin(),
      summary["tolerance_justification"]["threshold_families"].end(),
      [](const json& item) {
        return item["family_id"].get<std::string>() ==
               "three_phase_compare_surface";
      });
  REQUIRE(three_phase_tolerance_family !=
          summary["tolerance_justification"]["threshold_families"].end());
  CHECK((*three_phase_tolerance_family)["applies_to_tiers"] ==
        json::array({"accepted_secondary", "accepted_unbalanced_3ph",
                     "accepted_feeder_subset", "accepted_nr_load_semantics_3ph",
                     "accepted_nr_phase_matrix_line_3ph",
                     "accepted_nr_meshed_3ph",
                     "accepted_nr_multi_source_3ph",
                     "deferred_ieee13_feeder_formal_cross_check"}));
  const auto transformer3w_tolerance_family = std::find_if(
      summary["tolerance_justification"]["threshold_families"].begin(),
      summary["tolerance_justification"]["threshold_families"].end(),
      [](const json& item) {
        return item["family_id"].get<std::string>() ==
               "transformer3w_minimal_compare";
      });
  REQUIRE(transformer3w_tolerance_family !=
          summary["tolerance_justification"]["threshold_families"].end());
  CHECK((*transformer3w_tolerance_family)["applies_to_tiers"] ==
        json::array({"accepted_transformer3w_isolated_triangle",
                     "accepted_transformer3w_hv_spur_embed"}));

  const auto transformer3w_component = std::find_if(
      manifest["scope"]["capability_matrix"]["components"].begin(),
      manifest["scope"]["capability_matrix"]["components"].end(),
      [](const json& item) {
        return item["component"].get<std::string>() ==
               "transformer3w_tap_side_pair_projection";
      });
  REQUIRE(transformer3w_component !=
          manifest["scope"]["capability_matrix"]["components"].end());
  CHECK((*transformer3w_component)["comparison_status"] == "accepted");
  CHECK((*transformer3w_component)["reason_for_gap"].is_null());
  CHECK((*transformer3w_component)["cases"] ==
        json::array({"minimal_transformer3w_tap_1ph",
                     "minimal_transformer3w_tap_1ph_hv_spur"}));

  const auto phase_matrix_component = std::find_if(
      manifest["scope"]["capability_matrix"]["components"].begin(),
      manifest["scope"]["capability_matrix"]["components"].end(),
      [](const json& item) {
        return item["component"].get<std::string>() ==
               "full_phase_domain_line_matrix_nr";
      });
  REQUIRE(phase_matrix_component !=
          manifest["scope"]["capability_matrix"]["components"].end());
  // Phase-matrix NR component is now accepted after impedance base fix.
  CHECK((*phase_matrix_component)["comparison_status"] == "accepted");
  CHECK((*phase_matrix_component)["cases"] ==
        json::array({"minimal_phase_matrix_line_3bus_3ph_nr"}));

  const auto load_component = std::find_if(
      manifest["scope"]["capability_matrix"]["components"].begin(),
      manifest["scope"]["capability_matrix"]["components"].end(),
      [](const json& item) {
        return item["component"].get<std::string>() ==
               "three_phase_nr_load_semantics";
      });
  REQUIRE(load_component !=
          manifest["scope"]["capability_matrix"]["components"].end());
  // NR load-semantics component is now accepted after impedance base fix.
  CHECK((*load_component)["comparison_status"] == "accepted");
  CHECK((*load_component)["cases"] ==
        json::array({"minimal_load_wye_zip_3bus_3ph_nr",
                     "minimal_load_wye_open_neutral_3bus_1ph_lateral_nr",
                     "minimal_load_wye_impedance_grounded_3bus_1ph_lateral_nr",
                     "minimal_load_wye_vmax_3bus_1ph_lateral_nr",
                     "minimal_load_delta_power_3bus_3ph_nr",
                     "minimal_load_delta_zip_3bus_3ph_nr"}));

  const auto transformer3w_case = std::find_if(
      summary["cases"].begin(), summary["cases"].end(),
      [](const json& item) {
        return item["case_id"].get<std::string>() == "minimal_transformer3w_tap_1ph";
      });
  REQUIRE(transformer3w_case != summary["cases"].end());
  CHECK((*transformer3w_case)["acceptance_tier"] ==
        "accepted_transformer3w_isolated_triangle");
  CHECK((*transformer3w_case)["comparison_pass"] == true);
  CHECK((*transformer3w_case)["comparison_status"] == "accepted");
  CHECK((*transformer3w_case)["failed_metrics"].empty());
  CHECK((*transformer3w_case)["error_budget"]["all_metric_thresholds_passed"] == true);
  CHECK((*transformer3w_case)["error_budget"]["dominant_error_metric"].is_null() == false);
  CHECK((*transformer3w_case)["error_decomposition"]["per_winding_voltage_error"].size() ==
        3);
  CHECK((*transformer3w_case)["error_decomposition"]["per_winding_terminal_power_error"]
            .size() == 3);
  CHECK((*transformer3w_case)["error_decomposition"]["tap_side_projection_error"].empty() ==
        false);
  REQUIRE((*transformer3w_case).contains("root_cause_evidence"));
  CHECK((*transformer3w_case)["root_cause_evidence"]["hypothesis"] ==
        "model_side_transformer3w_pair_projection_required_coupled_kron_equivalent");
  CHECK((*transformer3w_case)["root_cause_evidence"]["support_level"] ==
        "model_side_coupled_kron_projection_fix_landed_minimal_case_now_passes");
  CHECK((*transformer3w_case)["root_cause_evidence"]["tap_projection_matches_expected"] ==
        true);
  CHECK((*transformer3w_case)["root_cause_evidence"]["bridge_loss_self_consistent"] ==
        true);
  CHECK((*transformer3w_case)["root_cause_evidence"]["affected_windings"] ==
        json::array());
  CHECK((*transformer3w_case)["error_budget"]["dominant_error_metric"]["name"] ==
        "bus_voltage_angle_deg");
  CHECK((*transformer3w_case)["root_cause_evidence"]["dominant_gap_metric"]["name"] ==
        "bus_voltage_magnitude_pu");
  CHECK((*transformer3w_case)["root_cause_evidence"]
            ["current_repo_mv_lv_active_power_split_mismatch_present"] ==
        false);
  CHECK((*transformer3w_case)["root_cause_evidence"]
            ["current_repo_mv_lv_reactive_power_split_mismatch_present"] ==
        false);
  CHECK((*transformer3w_case)["root_cause_evidence"]
            ["current_repo_hv_active_power_match_preserved"] ==
        true);
  CHECK((*transformer3w_case)["root_cause_evidence"]["next_single_point"]["name"] ==
        "broader_transformer3w_surface_still_unverified");
  CHECK((*transformer3w_case)["opendss_voltage_base_note"]
            .get<std::string>()
            .find("line-to-line values") != std::string::npos);

  const auto& production_trial =
      (*transformer3w_case)["root_cause_evidence"]["production_trial"];
  static constexpr double kTransformer3WPmwTol = 1e-3;
  static constexpr double kTransformer3WQmvarTol = 1e-3;
  static constexpr double kTransformer3WVmTightTol = 6e-4;
  static constexpr double kTransformer3WQmvarTightTol = 1e-4;
  CHECK(production_trial["applied"] == true);
  CHECK(production_trial["path_id"] ==
        "transformer3w_isolated_triangle_dense_nr_trial");
  CHECK(production_trial["fail_close_reason"].is_null());
  CHECK(production_trial["trigger_pattern"] ==
        "single_transformer3w_projected_to_isolated_three_bus_three_branch_triangle");
  CHECK(production_trial["targeted_metrics"].size() == 7);
  CHECK(production_trial["current_repo_vs_baseline"]
                     ["mv_lv_active_power_split_mismatch_present"]["baseline"] ==
        true);
  CHECK(production_trial["current_repo_vs_baseline"]
                     ["mv_lv_active_power_split_mismatch_present"]["current_repo"] ==
        false);
  CHECK(production_trial["current_repo_vs_baseline"]
                     ["mv_lv_reactive_power_split_mismatch_present"]["baseline"] ==
        true);
  CHECK(production_trial["current_repo_vs_baseline"]
                     ["mv_lv_reactive_power_split_mismatch_present"]["current_repo"] ==
        false);

  const auto& counterfactuals =
      (*transformer3w_case)["root_cause_evidence"]["counterfactuals"];
  REQUIRE(counterfactuals.contains("legacy_raw_pair_branch_reduction_bfs"));
  REQUIRE(counterfactuals.contains("coupled_kron_pair_resplit_bfs"));
  REQUIRE(counterfactuals.contains("meshed_dense_shadow_nr"));
  CHECK(counterfactuals["legacy_raw_pair_branch_reduction_bfs"]
            ["mv_lv_active_power_split_mismatch_present"] == true);
  CHECK(counterfactuals["legacy_raw_pair_branch_reduction_bfs"]
            ["mv_lv_reactive_power_split_mismatch_present"] == true);
  CHECK(counterfactuals["coupled_kron_pair_resplit_bfs"]
            ["mv_lv_active_power_split_mismatch_present"] == true);
  CHECK(counterfactuals["coupled_kron_pair_resplit_bfs"]
            ["mv_lv_reactive_power_split_mismatch_present"] == true);
  CHECK(counterfactuals["coupled_kron_pair_resplit_bfs"]
            ["winding_active_power_error_abs_mw"]["mv"].get<double>() <
        counterfactuals["legacy_raw_pair_branch_reduction_bfs"]
            ["winding_active_power_error_abs_mw"]["mv"].get<double>());
  CHECK(counterfactuals["coupled_kron_pair_resplit_bfs"]
            ["winding_active_power_error_abs_mw"]["lv"].get<double>() <
        counterfactuals["legacy_raw_pair_branch_reduction_bfs"]
            ["winding_active_power_error_abs_mw"]["lv"].get<double>());

  const auto& dense_shadow =
      counterfactuals["meshed_dense_shadow_nr"];
  CHECK(dense_shadow["legacy_raw_pair_branch_reduction"]
            ["mv_lv_active_power_split_mismatch_present"] == false);
  CHECK(dense_shadow["legacy_raw_pair_branch_reduction"]
            ["mv_lv_reactive_power_split_mismatch_present"] == false);
  CHECK(dense_shadow["coupled_kron_pair_resplit"]
            ["mv_lv_active_power_split_mismatch_present"] == false);
  CHECK(dense_shadow["coupled_kron_pair_resplit"]
            ["mv_lv_reactive_power_split_mismatch_present"] == false);
  CHECK(dense_shadow["legacy_raw_pair_branch_reduction"]
            ["winding_active_power_error_abs_mw"]["mv"].get<double>() < 1e-4);
  CHECK(dense_shadow["legacy_raw_pair_branch_reduction"]
            ["winding_active_power_error_abs_mw"]["lv"].get<double>() < 1e-4);
  CHECK(dense_shadow["coupled_kron_pair_resplit"]
            ["winding_active_power_error_abs_mw"]["mv"].get<double>() < 1e-4);
  CHECK(dense_shadow["coupled_kron_pair_resplit"]
            ["winding_active_power_error_abs_mw"]["lv"].get<double>() < 1e-4);
  CHECK(std::abs(
            dense_shadow["coupled_kron_pair_resplit"]["metrics"]
                       ["bus_voltage_magnitude_pu"]["max_abs"].get<double>() -
            (*transformer3w_case)["metrics"]["bus_voltage_magnitude_pu"]["max_abs"]
                .get<double>()) < 1e-12);
  CHECK(std::abs(
            dense_shadow["coupled_kron_pair_resplit"]["metrics"]
                       ["transformer_terminal1_q_mvar"]["max_abs"].get<double>() -
            (*transformer3w_case)["metrics"]["transformer_terminal1_q_mvar"]["max_abs"]
                .get<double>()) < 1e-12);
  CHECK(production_trial["current_repo_vs_baseline"]["metrics"]
                        ["transformer_terminal2_p_mw"]["current_absolute_error"]
            .get<double>() <
        production_trial["current_repo_vs_baseline"]["metrics"]
                        ["transformer_terminal2_p_mw"]["baseline_absolute_error"]
            .get<double>());
  CHECK(production_trial["current_repo_vs_baseline"]["metrics"]
                        ["transformer_terminal2_q_mvar"]["current_absolute_error"]
            .get<double>() <
        production_trial["current_repo_vs_baseline"]["metrics"]
                        ["transformer_terminal2_q_mvar"]["baseline_absolute_error"]
            .get<double>());
  CHECK(production_trial["current_repo_vs_baseline"]["metrics"]
                        ["transformer_terminal3_p_mw"]["current_absolute_error"]
            .get<double>() <
        production_trial["current_repo_vs_baseline"]["metrics"]
                        ["transformer_terminal3_p_mw"]["baseline_absolute_error"]
            .get<double>());
  CHECK(production_trial["current_repo_vs_baseline"]["metrics"]
                        ["transformer_terminal3_q_mvar"]["current_absolute_error"]
            .get<double>() <
        production_trial["current_repo_vs_baseline"]["metrics"]
                        ["transformer_terminal3_q_mvar"]["baseline_absolute_error"]
            .get<double>());
  CHECK(production_trial["current_repo_vs_baseline"]["metrics"]
                        ["transformer_terminal2_p_mw"]["current_absolute_error"]
            .get<double>() <
        kTransformer3WPmwTol);
  CHECK(production_trial["current_repo_vs_baseline"]["metrics"]
                        ["transformer_terminal2_q_mvar"]["current_absolute_error"]
            .get<double>() <
        kTransformer3WQmvarTol);
  CHECK(production_trial["current_repo_vs_baseline"]["metrics"]
                        ["transformer_terminal3_p_mw"]["current_absolute_error"]
            .get<double>() <
        kTransformer3WPmwTol);
  CHECK(production_trial["current_repo_vs_baseline"]["metrics"]
                        ["transformer_terminal3_q_mvar"]["current_absolute_error"]
            .get<double>() <
        kTransformer3WQmvarTol);
  CHECK(production_trial["current_repo_vs_baseline"]["metrics"]
                        ["bus_voltage_magnitude_pu"]["current_absolute_error"]
            .get<double>() <
        kTransformer3WVmTightTol);
  const auto& dss_vm = (*transformer3w_case)["opendss"]["bus_voltage_magnitude_pu"];
  REQUIRE(dss_vm.size() == 3);
  CHECK(std::abs(dss_vm[0].get<double>() - 1.0) < 1e-3);
  CHECK(std::abs(dss_vm[1].get<double>() - 1.0459464135336896) < 2e-3);
  CHECK(std::abs(dss_vm[2].get<double>() - 0.9962169271727559) < 2e-3);
  CHECK((*transformer3w_case)["metrics"]["bus_voltage_magnitude_pu"]["max_abs"]
            .get<double>() <
        kTransformer3WVmTightTol);
  CHECK((*transformer3w_case)["metrics"]["bus_voltage_angle_deg"]["max_abs"]
            .get<double>() <
        0.1);
  CHECK((*transformer3w_case)["metrics"]["transformer_terminal1_q_mvar"]["max_abs"]
            .get<double>() <
        kTransformer3WQmvarTightTol);
  CHECK((*transformer3w_case)["metrics"]["total_branch_loss_q_mvar"]["max_abs"]
            .get<double>() <
        kTransformer3WQmvarTightTol);
  CHECK((*transformer3w_case)["loss_diagnostics"]["loss_mismatch_breakdown"]
            .contains("abs_diff_repo_vs_element"));
  CHECK((*transformer3w_case)["loss_diagnostics"]["element_vs_terminal_sum_abs_diff"]
            .contains("p_mw"));

  const auto transformer3w_hv_spur_case = std::find_if(
      summary["cases"].begin(), summary["cases"].end(),
      [](const json& item) {
        return item["case_id"].get<std::string>() ==
               "minimal_transformer3w_tap_1ph_hv_spur";
      });
  REQUIRE(transformer3w_hv_spur_case != summary["cases"].end());
  CHECK((*transformer3w_hv_spur_case)["acceptance_tier"] ==
        "accepted_transformer3w_hv_spur_embed");
  CHECK((*transformer3w_hv_spur_case)["comparison_pass"] == true);
  CHECK((*transformer3w_hv_spur_case)["comparison_status"] == "accepted");
  CHECK((*transformer3w_hv_spur_case)["failed_metrics"].empty());
  CHECK((*transformer3w_hv_spur_case)["error_budget"]["all_metric_thresholds_passed"] ==
        true);
  CHECK((*transformer3w_hv_spur_case)["metrics"]["line_terminal1_p_mw"]["max_abs"]
            .get<double>() <
        1e-3);
  CHECK((*transformer3w_hv_spur_case)["metrics"]["line_terminal1_q_mvar"]["max_abs"]
            .get<double>() <
        1e-3);
  CHECK((*transformer3w_hv_spur_case)["error_decomposition"]["observed_bus_names"] ==
        json::array({"hv", "mv", "lv", "hv_spur"}));
  REQUIRE((*transformer3w_hv_spur_case).contains("root_cause_evidence"));
  CHECK((*transformer3w_hv_spur_case)["root_cause_evidence"]["hypothesis"] ==
        "model_side_transformer3w_pair_projection_required_coupled_kron_equivalent");
  CHECK((*transformer3w_hv_spur_case)["root_cause_evidence"]["support_level"] ==
        "model_side_coupled_kron_projection_fix_landed_slack_leaf_embed_surface_now_passes");
  CHECK((*transformer3w_hv_spur_case)["root_cause_evidence"]["next_single_point"]["name"] ==
        "non_slack_or_multi_branch_transformer3w_surface_still_unverified");
  const auto& hv_spur_production_trial =
      (*transformer3w_hv_spur_case)["root_cause_evidence"]["production_trial"];
  CHECK(hv_spur_production_trial["applied"] == true);
  CHECK(hv_spur_production_trial["path_id"] ==
        "transformer3w_slack_leaf_embed_dense_nr_trial");
  CHECK(hv_spur_production_trial["fail_close_reason"].is_null());
  CHECK(hv_spur_production_trial["trigger_pattern"] ==
        "single_transformer3w_projected_to_triangle_with_one_slack_side_radial_leaf");
  CHECK(hv_spur_production_trial["targeted_metrics"].size() == 9);
  CHECK((*transformer3w_hv_spur_case)["projection_diagnostics"]["external_line_name"] ==
        "l_hv_spur");
  CHECK((*transformer3w_hv_spur_case)["repo"]["line_terminal1_p_mw"].size() == 1);
  CHECK((*transformer3w_hv_spur_case)["opendss"]["line_terminal1_p_mw"].size() == 1);

  const auto ieee13_case = std::find_if(
      summary["cases"].begin(), summary["cases"].end(),
      [](const json& item) {
        return item["case_id"].get<std::string>() ==
               "ieee_13_node_4kv_backbone_3ph_nr";
      });
  REQUIRE(ieee13_case != summary["cases"].end());
  CHECK((*ieee13_case)["acceptance_tier"] ==
        "deferred_ieee13_feeder_formal_cross_check");
  CHECK((*ieee13_case)["comparison_status"] == "deferred");
  CHECK((*ieee13_case)["comparison_pass"] == false);
  CHECK((*ieee13_case)["error_budget"]["metrics"]["bus_phase_voltage_magnitude_pu"]
           ["pass"] == false);
  CHECK((*ieee13_case)["error_budget"]["metrics"]["bus_phase_voltage_magnitude_pu"]
           ["absolute_error"].get<double>() >
        (*ieee13_case)["error_budget"]["metrics"]["bus_phase_voltage_magnitude_pu"]
           ["threshold"].get<double>());

  const auto ieee13_component = std::find_if(
      manifest["scope"]["capability_matrix"]["components"].begin(),
      manifest["scope"]["capability_matrix"]["components"].end(),
      [](const json& item) {
        return item["component"].get<std::string>() ==
               "IEEE_13_node_4kv_backbone";
      });
  REQUIRE(ieee13_component !=
          manifest["scope"]["capability_matrix"]["components"].end());
  CHECK((*ieee13_component)["comparison_status"] == "deferred");
  CHECK((*ieee13_component)["reason_for_gap"] == "model_limitations");
  CHECK((*ieee13_component)["cases"] ==
        json::array({"ieee_13_node_4kv_backbone_3ph_nr"}));

  const auto old_bridge_only_case = std::find_if(
      summary["bridge_only_cases"].begin(), summary["bridge_only_cases"].end(),
      [](const json& item) {
        return item["case_id"].get<std::string>() == "minimal_regulator_1ph_bridge_only";
      });
  CHECK(old_bridge_only_case == summary["bridge_only_cases"].end());
}

TEST_CASE("OpenDSS compare records Transformer3W model-side fix while accepting the narrow case",
          "[integration][powerflow][distribution][opendss][transformer3w][root_cause]") {
  const fs::path compare_tool = sibling_compare_tool_path();
  REQUIRE(fs::exists(compare_tool));

  ScopedTempReportDir report_dir("transformer3w_root_cause");
  const std::string command =
      shell_exe(compare_tool) +
      " --report-dir " + shell_quote(report_dir.path) +
      " --case minimal_transformer3w_tap_1ph";

  const int raw_status = std::system(command.c_str());
  REQUIRE(raw_status != -1);
  REQUIRE(decode_exit_code(raw_status) == 0);

  const json summary = read_json_file(report_dir.path / "compare_summary.json");
  REQUIRE(summary["cases"].size() == 1);
  REQUIRE(summary["cases"][0]["case_id"] == "minimal_transformer3w_tap_1ph");
  REQUIRE(summary["cases"][0]["acceptance_tier"] ==
          "accepted_transformer3w_isolated_triangle");
  REQUIRE(summary["cases"][0]["comparison_pass"] == true);
  REQUIRE(summary["cases"][0]["comparison_status"] == "accepted");

  const auto& root_cause = summary["cases"][0]["root_cause_evidence"];
  static constexpr double kTransformer3WVmTightTol = 6e-4;
  static constexpr double kTransformer3WQmvarTightTol = 1e-4;
  CHECK(root_cause["support_level"] ==
        "model_side_coupled_kron_projection_fix_landed_minimal_case_now_passes");
  CHECK(root_cause["next_single_point"]["name"] ==
        "broader_transformer3w_surface_still_unverified");
  CHECK(root_cause["hypothesis"] ==
        "model_side_transformer3w_pair_projection_required_coupled_kron_equivalent");
  CHECK(root_cause["dominant_gap_metric"]["name"] == "bus_voltage_magnitude_pu");
  CHECK(root_cause["current_repo_mv_lv_active_power_split_mismatch_present"] == false);
  CHECK(root_cause["current_repo_mv_lv_reactive_power_split_mismatch_present"] == false);
  CHECK(summary["cases"][0]["error_budget"]["dominant_error_metric"]["name"] ==
        "bus_voltage_angle_deg");
  CHECK(summary["cases"][0]["error_budget"]["all_metric_thresholds_passed"] == true);
  CHECK(summary["cases"][0]["opendss_voltage_base_note"]
            .get<std::string>()
            .find("line-to-line values") != std::string::npos);

  const auto& production_trial = root_cause["production_trial"];
  CHECK(production_trial["applied"] == true);
  CHECK(production_trial["path_id"] ==
        "transformer3w_isolated_triangle_dense_nr_trial");
  CHECK(production_trial["fail_close_reason"].is_null());

  const auto& counterfactuals = root_cause["counterfactuals"];
  const auto& legacy_bfs =
      counterfactuals["legacy_raw_pair_branch_reduction_bfs"];
  const auto& coupled_bfs =
      counterfactuals["coupled_kron_pair_resplit_bfs"];
  const auto& dense_shadow =
      counterfactuals["meshed_dense_shadow_nr"];

  CHECK(legacy_bfs["mv_lv_active_power_split_mismatch_present"] == true);
  CHECK(legacy_bfs["mv_lv_reactive_power_split_mismatch_present"] == true);
  CHECK(coupled_bfs["mv_lv_active_power_split_mismatch_present"] == true);
  CHECK(coupled_bfs["mv_lv_reactive_power_split_mismatch_present"] == true);
  CHECK(coupled_bfs["winding_active_power_error_abs_mw"]["mv"].get<double>() <
        legacy_bfs["winding_active_power_error_abs_mw"]["mv"].get<double>());
  CHECK(coupled_bfs["winding_active_power_error_abs_mw"]["lv"].get<double>() <
        legacy_bfs["winding_active_power_error_abs_mw"]["lv"].get<double>());

  CHECK(dense_shadow["legacy_raw_pair_branch_reduction"]
            ["mv_lv_active_power_split_mismatch_present"] == false);
  CHECK(dense_shadow["legacy_raw_pair_branch_reduction"]
            ["mv_lv_reactive_power_split_mismatch_present"] == false);
  CHECK(dense_shadow["coupled_kron_pair_resplit"]
            ["mv_lv_active_power_split_mismatch_present"] == false);
  CHECK(dense_shadow["coupled_kron_pair_resplit"]
            ["mv_lv_reactive_power_split_mismatch_present"] == false);
  CHECK(dense_shadow["legacy_raw_pair_branch_reduction"]
            ["winding_active_power_error_abs_mw"]["mv"].get<double>() < 1e-4);
  CHECK(dense_shadow["legacy_raw_pair_branch_reduction"]
            ["winding_active_power_error_abs_mw"]["lv"].get<double>() < 1e-4);
  CHECK(production_trial["current_repo_vs_baseline"]["metrics"]
                        ["transformer_terminal2_p_mw"]["current_absolute_error"]
            .get<double>() <
        production_trial["current_repo_vs_baseline"]["metrics"]
                        ["transformer_terminal2_p_mw"]["baseline_absolute_error"]
            .get<double>());
  CHECK(std::abs(
            dense_shadow["coupled_kron_pair_resplit"]["metrics"]
                       ["bus_voltage_magnitude_pu"]["max_abs"].get<double>() -
            summary["cases"][0]["metrics"]["bus_voltage_magnitude_pu"]["max_abs"]
                .get<double>()) < 1e-12);
  CHECK(production_trial["current_repo_vs_baseline"]["metrics"]
                        ["bus_voltage_magnitude_pu"]["current_absolute_error"]
            .get<double>() <
        kTransformer3WVmTightTol);
  const auto& dss_vm = summary["cases"][0]["opendss"]["bus_voltage_magnitude_pu"];
  REQUIRE(dss_vm.size() == 3);
  CHECK(std::abs(dss_vm[0].get<double>() - 1.0) < 1e-3);
  CHECK(std::abs(dss_vm[1].get<double>() - 1.0459464135336896) < 2e-3);
  CHECK(std::abs(dss_vm[2].get<double>() - 0.9962169271727559) < 2e-3);
  CHECK(summary["cases"][0]["metrics"]["bus_voltage_magnitude_pu"]["max_abs"]
            .get<double>() <
        kTransformer3WVmTightTol);
  CHECK(summary["cases"][0]["metrics"]["bus_voltage_angle_deg"]["max_abs"]
            .get<double>() <
        0.1);
  CHECK(summary["cases"][0]["metrics"]["transformer_terminal1_q_mvar"]["max_abs"]
            .get<double>() <
        kTransformer3WQmvarTightTol);
  CHECK(summary["cases"][0]["metrics"]["total_branch_loss_q_mvar"]["max_abs"]
            .get<double>() <
        kTransformer3WQmvarTightTol);
}

TEST_CASE("OpenDSS compare accepts Transformer3W slack-side spur embed as the next narrow surface",
          "[integration][powerflow][distribution][opendss][transformer3w][surface]") {
  const fs::path compare_tool = sibling_compare_tool_path();
  REQUIRE(fs::exists(compare_tool));

  ScopedTempReportDir report_dir("transformer3w_hv_spur_surface");
  const std::string command =
      shell_exe(compare_tool) +
      " --report-dir " + shell_quote(report_dir.path) +
      " --case minimal_transformer3w_tap_1ph_hv_spur";

  const int raw_status = std::system(command.c_str());
  REQUIRE(raw_status != -1);
  REQUIRE(decode_exit_code(raw_status) == 0);

  const json summary = read_json_file(report_dir.path / "compare_summary.json");
  REQUIRE(summary["cases"].size() == 1);
  REQUIRE(summary["cases"][0]["case_id"] == "minimal_transformer3w_tap_1ph_hv_spur");
  REQUIRE(summary["cases"][0]["acceptance_tier"] ==
          "accepted_transformer3w_hv_spur_embed");
  REQUIRE(summary["cases"][0]["comparison_pass"] == true);
  REQUIRE(summary["cases"][0]["comparison_status"] == "accepted");
  CHECK(summary["accepted_cases_all_passed"] == true);

  const auto& root_cause = summary["cases"][0]["root_cause_evidence"];
  CHECK(root_cause["hypothesis"] ==
        "model_side_transformer3w_pair_projection_required_coupled_kron_equivalent");
  CHECK(root_cause["support_level"] ==
        "model_side_coupled_kron_projection_fix_landed_slack_leaf_embed_surface_now_passes");
  CHECK(root_cause["next_single_point"]["name"] ==
        "non_slack_or_multi_branch_transformer3w_surface_still_unverified");

  const auto& production_trial = root_cause["production_trial"];
  CHECK(production_trial["applied"] == true);
  CHECK(production_trial["path_id"] ==
        "transformer3w_slack_leaf_embed_dense_nr_trial");
  CHECK(production_trial["fail_close_reason"].is_null());
  CHECK(production_trial["targeted_metrics"].size() == 9);

  CHECK(summary["cases"][0]["metrics"]["bus_voltage_angle_deg"]["max_abs"]
            .get<double>() <=
        0.1);
  CHECK(summary["cases"][0]["metrics"]["line_terminal1_p_mw"]["max_abs"]
            .get<double>() <
        1e-3);
  CHECK(summary["cases"][0]["metrics"]["line_terminal1_q_mvar"]["max_abs"]
            .get<double>() <
        1e-3);
  CHECK(summary["cases"][0]["projection_diagnostics"]["external_line_name"] ==
        "l_hv_spur");
}

TEST_CASE("OpenDSS compare emits IEEE 13-node backbone as deferred formal evidence",
          "[integration][powerflow][distribution][opendss][artifact][ieee13]") {
  const fs::path compare_tool = sibling_compare_tool_path();
  REQUIRE(fs::exists(compare_tool));

  ScopedTempReportDir report_dir("ieee13_deferred_compare");
  const std::string command =
      shell_exe(compare_tool) +
      " --report-dir " + shell_quote(report_dir.path) +
      " --case ieee_13_node_4kv_backbone_3ph_nr";

  const int raw_status = std::system(command.c_str());
  REQUIRE(raw_status != -1);
  REQUIRE(decode_exit_code(raw_status) == 0);

  const json summary = read_json_file(report_dir.path / "compare_summary.json");
  const json manifest = read_json_file(report_dir.path / "case_manifest.json");

  REQUIRE(summary["cases"].size() == 1);
  CHECK(summary["accepted_cases_all_passed"] == true);
  CHECK(summary["cases"][0]["case_id"] == "ieee_13_node_4kv_backbone_3ph_nr");
  CHECK(summary["cases"][0]["acceptance_tier"] ==
        "deferred_ieee13_feeder_formal_cross_check");
  CHECK(summary["cases"][0]["comparison_status"] == "deferred");
  CHECK(summary["cases"][0]["comparison_pass"] == false);
  CHECK(summary["cases"][0]["error_budget"]["dominant_error_metric"].is_null() == false);
  CHECK(summary["cases"][0]["error_budget"]["metrics"]["bus_phase_voltage_magnitude_pu"]
           ["pass"] == false);
  CHECK(summary["cases"][0]["error_budget"]["metrics"]["bus_phase_voltage_magnitude_pu"]
           ["absolute_error"].get<double>() >
        summary["cases"][0]["error_budget"]["metrics"]["bus_phase_voltage_magnitude_pu"]
            ["threshold"].get<double>());

  REQUIRE(manifest["cases"].size() == 1);
  CHECK(manifest["cases"][0]["case_id"] == "ieee_13_node_4kv_backbone_3ph_nr");
  CHECK(manifest["cases"][0]["comparison_status"] == "deferred");
  CHECK(manifest["cases"][0]["comparison_pass"] == false);
  REQUIRE(
      manifest["scope"]["acceptance_tiers"].contains(
          "deferred_ieee13_feeder_formal_cross_check"));
  CHECK(manifest["scope"]["acceptance_tiers"]
                ["deferred_ieee13_feeder_formal_cross_check"]["case"] ==
        "ieee_13_node_4kv_backbone_3ph_nr");
  CHECK(manifest["scope"]["acceptance_tiers"]
                ["deferred_ieee13_feeder_formal_cross_check"]
                ["comparison_status"] == "deferred");
  CHECK(manifest["scope"]["acceptance_tiers"]
                ["deferred_ieee13_feeder_formal_cross_check"]
                ["comparison_pass"] == false);

  const auto ieee13_component = std::find_if(
      manifest["scope"]["capability_matrix"]["components"].begin(),
      manifest["scope"]["capability_matrix"]["components"].end(),
      [](const json& item) {
        return item["component"].get<std::string>() ==
               "IEEE_13_node_4kv_backbone";
      });
  REQUIRE(ieee13_component !=
          manifest["scope"]["capability_matrix"]["components"].end());
  CHECK((*ieee13_component)["comparison_status"] == "deferred");
  CHECK((*ieee13_component)["reason_for_gap"] == "model_limitations");
  CHECK((*ieee13_component)["cases"] ==
        json::array({"ieee_13_node_4kv_backbone_3ph_nr"}));
}

TEST_CASE("OpenDSS typed unit converters keep Powers and Losses distinct",
          "[integration][powerflow][distribution][opendss][units]") {
  const hacdcpf::io::OpenDSSPowerKWKvar power_kw_kvar{
      .p_kw = 1000.0,
      .q_kvar = 500.0,
  };
  const auto converted_power =
      hacdcpf_compare::opendss_terminal_power_to_mw_mvar(power_kw_kvar);
  CHECK(converted_power.p_mw == 1.0);
  CHECK(converted_power.q_mvar == 0.5);

  const hacdcpf::io::OpenDSSLossesWVar losses_w_var{
      .p_w = 1000.0,
      .q_var = 500.0,
  };
  const auto converted_losses =
      hacdcpf_compare::opendss_losses_raw_to_mw_mvar(losses_w_var);
  CHECK(converted_losses.p_mw == 0.001);
  CHECK(converted_losses.q_mvar == 0.0005);
}

TEST_CASE("OpenDSS compare gate returns exit code 2 on injected accepted-tier regression",
          "[integration][powerflow][distribution][opendss][gate]") {
  const fs::path compare_tool = sibling_compare_tool_path();
  REQUIRE(fs::exists(compare_tool));

  ScopedTempReportDir report_dir("gate_regression");
  const std::string command =
      shell_exe(compare_tool) +
      " --report-dir " + shell_quote(report_dir.path) +
      " --inject-metric-max-abs minimal_radial_3bus_1ph "
      "bus_voltage_magnitude_pu 0.01";

  const int raw_status = std::system(command.c_str());
  REQUIRE(raw_status != -1);
  const int exit_code = decode_exit_code(raw_status);
  REQUIRE(exit_code == hacdcpf_compare::kAcceptedCaseGateExitCode);

  const json summary = read_json_file(report_dir.path / "compare_summary.json");
  REQUIRE(summary["accepted_cases_all_passed"].get<bool>() == false);

  const auto target_case = std::find_if(
      summary["cases"].begin(), summary["cases"].end(),
      [](const json& item) {
        return item["case_id"].get<std::string>() == "minimal_radial_3bus_1ph";
      });
  REQUIRE(target_case != summary["cases"].end());
  REQUIRE((*target_case)["comparison_pass"].get<bool>() == false);
  REQUIRE((*target_case)["failed_metrics"].empty() == false);

  bool found_failed_metric = false;
  for (const auto& metric : (*target_case)["failed_metrics"]) {
    if (metric.get<std::string>() == "bus_voltage_magnitude_pu") {
      found_failed_metric = true;
      break;
    }
  }
  CHECK(found_failed_metric);
}

TEST_CASE("OpenDSS compare gate returns exit code 2 on injected regulator accepted-tier regression",
          "[integration][powerflow][distribution][opendss][gate][regulator]") {
  const fs::path compare_tool = sibling_compare_tool_path();
  REQUIRE(fs::exists(compare_tool));

  ScopedTempReportDir report_dir("regulator_gate_regression");
  const std::string command =
      shell_exe(compare_tool) +
      " --report-dir " + shell_quote(report_dir.path) +
      " --inject-metric-max-abs minimal_regulator_1ph "
      "regulator_tap_number 1.0";

  const int raw_status = std::system(command.c_str());
  REQUIRE(raw_status != -1);
  const int exit_code = decode_exit_code(raw_status);
  REQUIRE(exit_code == hacdcpf_compare::kAcceptedCaseGateExitCode);

  const json summary = read_json_file(report_dir.path / "compare_summary.json");
  REQUIRE(summary["accepted_cases_all_passed"].get<bool>() == false);

  const auto target_case = std::find_if(
      summary["cases"].begin(), summary["cases"].end(),
      [](const json& item) {
        return item["case_id"].get<std::string>() == "minimal_regulator_1ph";
      });
  REQUIRE(target_case != summary["cases"].end());
  REQUIRE((*target_case)["comparison_pass"].get<bool>() == false);
  REQUIRE((*target_case)["failed_metrics"].empty() == false);

  bool found_failed_metric = false;
  for (const auto& metric : (*target_case)["failed_metrics"]) {
    if (metric.get<std::string>() == "regulator_tap_number") {
      found_failed_metric = true;
      break;
    }
  }
  CHECK(found_failed_metric);
}

TEST_CASE("OpenDSS compare surfaces fixture truth drift separately from bridge validation",
          "[integration][powerflow][distribution][opendss][gate][fixture_truth]") {
  const fs::path compare_tool = sibling_compare_tool_path();
  REQUIRE(fs::exists(compare_tool));

  ScopedTempReportDir report_dir("fixture_truth_drift_regression");
  const std::string command =
      shell_exe(compare_tool) +
      " --report-dir " + shell_quote(report_dir.path) +
      " --inject-regulator-fixture-truth minimal_regulator_1ph 7 1.04375";

  const int raw_status = std::system(command.c_str());
  REQUIRE(raw_status != -1);
  const int exit_code = decode_exit_code(raw_status);
  REQUIRE(exit_code == hacdcpf_compare::kAcceptedCaseGateExitCode);

  const json summary = read_json_file(report_dir.path / "compare_summary.json");
  REQUIRE(summary["accepted_cases_all_passed"].get<bool>() == false);

  const auto target_case = std::find_if(
      summary["cases"].begin(), summary["cases"].end(),
      [](const json& item) {
        return item["case_id"].get<std::string>() == "minimal_regulator_1ph";
      });
  REQUIRE(target_case != summary["cases"].end());
  // With the injected stale fixture truth (tap=7 vs actual=8),
  // fixture_truth_drift fires while bridge and control trace validation pass.
  CHECK((*target_case)["comparison_status"] == "fixture_truth_drift");
  CHECK((*target_case)["comparison_pass"] == false);
  CHECK((*target_case)["bridge_validation_pass"] == true);
  CHECK((*target_case)["fixture_truth_drift_pass"] == false);
  CHECK((*target_case)["error_budget"]["all_metric_thresholds_passed"] == true);
  CHECK((*target_case)["control_trace_validation_pass"] == true);
  {
    const auto& fm = (*target_case)["failed_metrics"];
    bool has_drift = false;
    for (const auto& m : fm) {
      if (m == "fixture_truth_drift") has_drift = true;
    }
    CHECK(has_drift);
  }
  CHECK((*target_case)["fixture_truth_drift"]["reason"] ==
        "fixture_expected_regulator_end_state_is_stale");
  CHECK((*target_case)["fixture_truth_drift"]["fixture_expected_end_state"]["tap_number"] ==
        7);
  CHECK(std::abs((*target_case)["fixture_truth_drift"]["fixture_expected_end_state"]
                                   ["tap_pu"].get<double>() -
                 1.04375) < 1e-12);
}

TEST_CASE("OpenDSS compare CLI supports case, tier, and shard selection",
          "[integration][powerflow][distribution][opendss][cli]") {
  const fs::path compare_tool = sibling_compare_tool_path();
  REQUIRE(fs::exists(compare_tool));

  SECTION("single accepted NR case") {
    ScopedTempReportDir report_dir("case_selection_nr");
    const std::string command =
        shell_exe(compare_tool) +
        " --report-dir " + shell_quote(report_dir.path) +
        " --case minimal_transformer3w_tap_1ph";
    const int raw_status = std::system(command.c_str());
    REQUIRE(raw_status != -1);
    REQUIRE(decode_exit_code(raw_status) == 0);

    const json summary = read_json_file(report_dir.path / "compare_summary.json");
    const json manifest = read_json_file(report_dir.path / "case_manifest.json");
    REQUIRE(summary["cases"].size() == 1);
    CHECK(summary["cases"][0]["case_id"] == "minimal_transformer3w_tap_1ph");
    CHECK(summary["accepted_cases_all_passed"] == true);
    CHECK(summary["selection"]["mode"] == "filtered");
    CHECK(summary["selection"]["selected_case_ids"] ==
          json::array({"minimal_transformer3w_tap_1ph"}));
    CHECK(manifest["cases"].size() == 1);
    CHECK(manifest["cases"][0]["case_id"] == "minimal_transformer3w_tap_1ph");
  }

  SECTION("single accepted tier") {
    ScopedTempReportDir report_dir("tier_selection_remote_ptratio");
    const std::string command =
        shell_exe(compare_tool) +
        " --report-dir " + shell_quote(report_dir.path) +
        " --tier accepted_regulator_remote_ptratio";
    const int raw_status = std::system(command.c_str());
    REQUIRE(raw_status != -1);
    const int exit_code = decode_exit_code(raw_status);
    CHECK(exit_code == 0);

    const json summary = read_json_file(report_dir.path / "compare_summary.json");
    REQUIRE(summary["cases"].size() == 1);
    CHECK(summary["cases"][0]["case_id"] == "minimal_regulator_remote_ptratio_1ph");
    CHECK(summary["selection"]["requested_tier_filters"] ==
          json::array({"accepted_regulator_remote_ptratio"}));
  }

  SECTION("deterministic sharded subset") {
    ScopedTempReportDir report_dir("shard_selection");
    const std::string command =
        shell_exe(compare_tool) +
        " --report-dir " + shell_quote(report_dir.path) +
        " --shard-index 1 --shard-count 4";
    const int raw_status = std::system(command.c_str());
    REQUIRE(raw_status != -1);
    // The shard may contain P2 failed cases; do not gate on exit code.
    // Verify only the shard selection metadata.

    const json summary = read_json_file(report_dir.path / "compare_summary.json");
    REQUIRE(summary["selection"]["mode"] == "filtered");
    REQUIRE(summary["selection"]["shard_index"] == 1);
    REQUIRE(summary["selection"]["shard_count"] == 4);
    CHECK(summary["selection"]["selected_case_ids"].empty() == false);
    CHECK(summary["selection"]["skipped_case_ids"].empty() == false);
    CHECK(summary["cases"].size() ==
          summary["selection"]["selected_case_ids"].size());
  }
}
