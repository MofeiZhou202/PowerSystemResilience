#pragma once

#include <algorithm>
#include <filesystem>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include <nlohmann/json.hpp>

#include "hacdcpf/io/opendss_bridge.hpp"
#include "opendss_compare/units.hpp"

namespace hacdcpf_compare::snapshot_contract {

using json = nlohmann::json;
namespace fs = std::filesystem;

inline constexpr const char* kSchemaName = "hacdcpf.opendss_snapshot";
inline constexpr int kSchemaVersion = 1;
inline constexpr const char* kAdapterName = "opendss_snapshot_adapter";
inline constexpr const char* kAdapterBackend = "dss_capi_classic_surface";

struct SnapshotContractEnvelope {
  std::string master_dss;
  std::string engine_version;
  std::string adapter_name;
  std::string adapter_backend;
  hacdcpf::io::OpenDSSSnapshotResult snapshot;
};

inline std::string ascii_lower(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(),
                 [](unsigned char ch) {
                   return static_cast<char>(std::tolower(ch));
                 });
  return value;
}

inline std::string portable_path_string(const fs::path& path,
                                        const fs::path& project_root = {}) {
  const fs::path normalized = path.lexically_normal();
  if (!project_root.empty()) {
    const fs::path normalized_root = project_root.lexically_normal();
    std::error_code ec;
    const fs::path relative = fs::relative(normalized, normalized_root, ec);
    if (!ec && !relative.empty()) {
      return relative.generic_string();
    }
  }
  if (normalized.is_absolute()) {
    std::error_code ec;
    const fs::path cwd_relative =
        fs::relative(normalized, fs::current_path(), ec);
    if (!ec && !cwd_relative.empty()) {
      return cwd_relative.generic_string();
    }
  }
  return normalized.generic_string();
}

inline json schema_to_json() {
  return {
      {"name", kSchemaName},
      {"version", kSchemaVersion},
  };
}

inline json adapter_to_json() {
  return {
      {"name", kAdapterName},
      {"backend", kAdapterBackend},
  };
}

inline json units_to_json() {
  return {
      {"node_voltage_magnitude", "p.u."},
      {"node_voltage_angle", "degrees"},
      {"node_voltage_actual", "volts line-to-neutral"},
      {"terminal_power_raw", "kW / kvar"},
      {"terminal_power_converted", "MW / MVAr"},
      {"losses_raw", "W / var"},
      {"losses_converted", "MW / MVAr"},
  };
}

inline json power_pair_to_json(const hacdcpf_compare::PowerPair& power) {
  return {
      {"p_mw", power.p_mw},
      {"q_mvar", power.q_mvar},
  };
}

inline json losses_raw_to_json(const hacdcpf::io::OpenDSSLossesWVar& losses) {
  return {
      {"p_w", losses.p_w},
      {"q_var", losses.q_var},
  };
}

inline json node_voltage_to_json(const hacdcpf::io::OpenDSSNodeVoltage& voltage) {
  return {
      {"bus_name", voltage.bus_name},
      {"node", voltage.node},
      {"vm_pu", voltage.vm_pu},
      {"va_deg", voltage.va_deg},
      {"vm_vln_volts", voltage.vm_vln},
  };
}

inline json terminal_power_to_json(const hacdcpf::io::OpenDSSTerminalPower& power) {
  return {
      {"terminal", power.terminal},
      {"conductor", power.conductor},
      {"node", power.node},
      {"power_kw_kvar",
       {
           {"p_kw", power.power_kw_kvar.p_kw},
           {"q_kvar", power.power_kw_kvar.q_kvar},
       }},
      {"power_mw_mvar",
       power_pair_to_json(
           hacdcpf_compare::opendss_terminal_power_to_mw_mvar(power))},
  };
}

inline json transformer_state_to_json(
    const hacdcpf::io::OpenDSSTransformerState& state) {
  json windings = json::array();
  for (const auto& winding : state.winding_states) {
    windings.push_back({
        {"winding", winding.winding},
        {"tap_pu", winding.tap_pu},
        {"min_tap_pu", winding.min_tap_pu},
        {"max_tap_pu", winding.max_tap_pu},
        {"num_taps", winding.num_taps},
        {"kv", winding.kv},
        {"kva", winding.kva},
    });
  }
  return {
      {"name", state.name},
      {"num_windings", state.num_windings},
      {"winding_states", windings},
  };
}

inline json regcontrol_to_json(const hacdcpf::io::OpenDSSRegControlResult& control) {
  return {
      {"name", control.name},
      {"transformer_name", control.transformer_name},
      {"monitored_bus_name", control.monitored_bus_name},
      {"winding", control.winding},
      {"tap_winding", control.tap_winding},
      {"tap_number", control.tap_number},
      {"max_tap_change", control.max_tap_change},
      {"forward_vreg_volts", control.forward_vreg_volts},
      {"forward_band_volts", control.forward_band_volts},
      {"ptratio", control.ptratio},
      {"remote_ptratio", control.remote_ptratio},
      {"ct_primary_amps", control.ct_primary_amps},
      {"forward_r_volts", control.forward_r_volts},
      {"forward_x_volts", control.forward_x_volts},
      {"voltage_limit_volts", control.voltage_limit_volts},
      {"is_reversible", control.is_reversible},
  };
}

inline json pd_element_to_json(const hacdcpf::io::OpenDSSPDElementResult& element) {
  json terminal_powers = json::array();
  for (const auto& power : element.terminal_powers) {
    terminal_powers.push_back(terminal_power_to_json(power));
  }
  return {
      {"element_kind",
       hacdcpf::io::opendss_pd_element_kind_to_string(element.element_kind)},
      {"element_name", element.element_name},
      {"name", element.name},
      {"terminal_bus_names", element.terminal_bus_names},
      {"node_order", element.node_order},
      {"num_phases", element.num_phases},
      {"num_conductors", element.num_conductors},
      {"num_terminals", element.num_terminals},
      {"terminal_powers", terminal_powers},
      {"terminal_power_sum_mw_mvar",
       power_pair_to_json(
           hacdcpf_compare::sum_pd_element_terminal_powers_to_mw_mvar(element))},
      {"losses_raw", losses_raw_to_json(element.losses_raw)},
      {"losses_mw_mvar",
       power_pair_to_json(
           hacdcpf_compare::opendss_element_losses_to_mw_mvar(element))},
  };
}

inline json grouped_buses_to_json(
    const std::vector<hacdcpf::io::OpenDSSNodeVoltage>& node_voltages) {
  std::map<std::string, json> buses;
  for (const auto& voltage : node_voltages) {
    auto& bus = buses[voltage.bus_name];
    if (bus.is_null()) {
      bus = {
          {"name", voltage.bus_name},
          {"nodes", json::array()},
      };
    }
    bus["nodes"].push_back(node_voltage_to_json(voltage));
  }

  json out = json::array();
  for (auto& [_, bus] : buses) {
    out.push_back(std::move(bus));
  }
  return out;
}

inline json snapshot_to_json(const hacdcpf::io::OpenDSSSnapshotResult& snapshot,
                             const std::string& master_dss) {
  json node_voltages = json::array();
  for (const auto& voltage : snapshot.node_voltages) {
    node_voltages.push_back(node_voltage_to_json(voltage));
  }

  json pd_elements = json::array();
  for (const auto& element : snapshot.pd_element_results) {
    pd_elements.push_back(pd_element_to_json(element));
  }

  json transformer_states = json::array();
  for (const auto& state : snapshot.transformer_states) {
    transformer_states.push_back(transformer_state_to_json(state));
  }

  json regcontrol_states = json::array();
  for (const auto& control : snapshot.regcontrol_results) {
    regcontrol_states.push_back(regcontrol_to_json(control));
  }

  return {
      {"schema", schema_to_json()},
      {"adapter", adapter_to_json()},
      {"master_dss", master_dss},
      {"engine_version", snapshot.engine_version},
      {"converged", snapshot.converged},
      {"node_voltages", node_voltages},
      {"buses", grouped_buses_to_json(snapshot.node_voltages)},
      {"pd_elements", pd_elements},
      {"transformer_states", transformer_states},
      {"regcontrol_states", regcontrol_states},
      {"circuit_losses_raw", losses_raw_to_json(snapshot.circuit_losses_raw)},
      {"circuit_losses_mw_mvar",
       power_pair_to_json(
           hacdcpf_compare::opendss_circuit_losses_to_mw_mvar(snapshot))},
      {"units", units_to_json()},
  };
}

inline hacdcpf::io::OpenDSSPDElementKind pd_element_kind_from_string(
    std::string_view value) {
  const std::string lowered = ascii_lower(std::string(value));
  if (lowered == "line") {
    return hacdcpf::io::OpenDSSPDElementKind::Line;
  }
  if (lowered == "transformer") {
    return hacdcpf::io::OpenDSSPDElementKind::Transformer;
  }
  throw std::runtime_error("Unsupported PD element kind in snapshot contract: " +
                           std::string(value));
}

inline hacdcpf::io::OpenDSSLineResult line_projection_from_pd_element(
    const hacdcpf::io::OpenDSSPDElementResult& element) {
  return {
      .element_name = element.element_name,
      .line_name = element.name,
      .terminal_bus_names = element.terminal_bus_names,
      .node_order = element.node_order,
      .num_phases = element.num_phases,
      .num_conductors = element.num_conductors,
      .num_terminals = element.num_terminals,
      .terminal_powers = element.terminal_powers,
      .losses_raw = element.losses_raw,
  };
}

inline SnapshotContractEnvelope snapshot_from_json(const json& document) {
  if (document.at("schema") != schema_to_json()) {
    throw std::runtime_error("OpenDSS snapshot contract schema mismatch.");
  }
  if (document.at("adapter") != adapter_to_json()) {
    throw std::runtime_error("OpenDSS snapshot contract adapter descriptor mismatch.");
  }
  if (document.at("units") != units_to_json()) {
    throw std::runtime_error("OpenDSS snapshot contract units mismatch.");
  }

  SnapshotContractEnvelope envelope;
  envelope.master_dss = document.at("master_dss").get<std::string>();
  envelope.engine_version = document.at("engine_version").get<std::string>();
  envelope.adapter_name = document.at("adapter").at("name").get<std::string>();
  envelope.adapter_backend = document.at("adapter").at("backend").get<std::string>();
  envelope.snapshot.engine_version = envelope.engine_version;
  envelope.snapshot.converged = document.at("converged").get<bool>();

  for (const auto& item : document.at("node_voltages")) {
    envelope.snapshot.node_voltages.push_back({
        .bus_name = item.at("bus_name").get<std::string>(),
        .node = item.at("node").get<int>(),
        .vm_pu = item.at("vm_pu").get<double>(),
        .va_deg = item.at("va_deg").get<double>(),
        .vm_vln = item.at("vm_vln_volts").get<double>(),
    });
  }

  for (const auto& item : document.at("pd_elements")) {
    hacdcpf::io::OpenDSSPDElementResult element;
    element.element_kind =
        pd_element_kind_from_string(item.at("element_kind").get<std::string>());
    element.element_name = item.at("element_name").get<std::string>();
    element.name = item.at("name").get<std::string>();
    element.terminal_bus_names =
        item.at("terminal_bus_names").get<std::vector<std::string>>();
    element.node_order = item.at("node_order").get<std::vector<int>>();
    element.num_phases = item.at("num_phases").get<int>();
    element.num_conductors = item.at("num_conductors").get<int>();
    element.num_terminals = item.at("num_terminals").get<int>();
    element.losses_raw = {
        .p_w = item.at("losses_raw").at("p_w").get<double>(),
        .q_var = item.at("losses_raw").at("q_var").get<double>(),
    };
    for (const auto& power_item : item.at("terminal_powers")) {
      element.terminal_powers.push_back({
          .terminal = power_item.at("terminal").get<int>(),
          .conductor = power_item.at("conductor").get<int>(),
          .node = power_item.at("node").get<int>(),
          .power_kw_kvar =
              {
                  .p_kw = power_item.at("power_kw_kvar").at("p_kw").get<double>(),
                  .q_kvar =
                      power_item.at("power_kw_kvar").at("q_kvar").get<double>(),
              },
      });
    }
    envelope.snapshot.pd_element_results.push_back(element);
    if (element.element_kind == hacdcpf::io::OpenDSSPDElementKind::Line) {
      envelope.snapshot.line_results.push_back(
          line_projection_from_pd_element(element));
    }
  }

  for (const auto& item : document.at("transformer_states")) {
    hacdcpf::io::OpenDSSTransformerState state;
    state.name = item.at("name").get<std::string>();
    state.num_windings = item.at("num_windings").get<int>();
    for (const auto& winding_item : item.at("winding_states")) {
      state.winding_states.push_back({
          .winding = winding_item.at("winding").get<int>(),
          .tap_pu = winding_item.at("tap_pu").get<double>(),
          .min_tap_pu = winding_item.at("min_tap_pu").get<double>(),
          .max_tap_pu = winding_item.at("max_tap_pu").get<double>(),
          .num_taps = winding_item.at("num_taps").get<int>(),
          .kv = winding_item.at("kv").get<double>(),
          .kva = winding_item.at("kva").get<double>(),
      });
    }
    envelope.snapshot.transformer_states.push_back(std::move(state));
  }

  for (const auto& item : document.at("regcontrol_states")) {
    envelope.snapshot.regcontrol_results.push_back({
        .name = item.at("name").get<std::string>(),
        .transformer_name = item.at("transformer_name").get<std::string>(),
        .monitored_bus_name = item.at("monitored_bus_name").get<std::string>(),
        .winding = item.at("winding").get<int>(),
        .tap_winding = item.at("tap_winding").get<int>(),
        .tap_number = item.at("tap_number").get<int>(),
        .max_tap_change = item.at("max_tap_change").get<int>(),
        .forward_vreg_volts = item.at("forward_vreg_volts").get<double>(),
        .forward_band_volts = item.at("forward_band_volts").get<double>(),
        .ptratio = item.at("ptratio").get<double>(),
        .remote_ptratio = item.at("remote_ptratio").get<double>(),
        .ct_primary_amps = item.at("ct_primary_amps").get<double>(),
        .forward_r_volts = item.at("forward_r_volts").get<double>(),
        .forward_x_volts = item.at("forward_x_volts").get<double>(),
        .voltage_limit_volts = item.at("voltage_limit_volts").get<double>(),
        .is_reversible = item.at("is_reversible").get<bool>(),
    });
  }

  envelope.snapshot.circuit_losses_raw = {
      .p_w = document.at("circuit_losses_raw").at("p_w").get<double>(),
      .q_var = document.at("circuit_losses_raw").at("q_var").get<double>(),
  };
  return envelope;
}

inline json build_contract_surface_summary(const SnapshotContractEnvelope& envelope) {
  std::set<std::string> element_kinds;
  for (const auto& element : envelope.snapshot.pd_element_results) {
    element_kinds.insert(
        hacdcpf::io::opendss_pd_element_kind_to_string(element.element_kind));
  }

  json observed_kinds = json::array();
  for (const auto& kind : element_kinds) {
    observed_kinds.push_back(kind);
  }

  return {
      {"schema", schema_to_json()},
      {"adapter", adapter_to_json()},
      {"required_top_level_fields",
       json::array({"schema",
                    "adapter",
                    "master_dss",
                    "engine_version",
                    "converged",
                    "node_voltages",
                    "buses",
                    "pd_elements",
                    "transformer_states",
                    "regcontrol_states",
                    "circuit_losses_raw",
                    "circuit_losses_mw_mvar",
                    "units"})},
      {"required_pd_element_kinds", json::array({"line", "transformer"})},
      {"observed_pd_element_kinds", observed_kinds},
      {"units", units_to_json()},
      {"master_dss", envelope.master_dss},
      {"portable_master_dss", !fs::path(envelope.master_dss).is_absolute()},
      {"roundtrip_counts",
       {
           {"node_voltages", envelope.snapshot.node_voltages.size()},
           {"pd_elements", envelope.snapshot.pd_element_results.size()},
           {"transformer_states", envelope.snapshot.transformer_states.size()},
           {"regcontrol_states", envelope.snapshot.regcontrol_results.size()},
       }},
  };
}

}  // namespace hacdcpf_compare::snapshot_contract
