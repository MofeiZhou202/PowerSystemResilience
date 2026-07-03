#include "hacdcpf/io/opendss_bridge.hpp"

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include "hacdcpf/io/dss_capi_adapter.hpp"

namespace hacdcpf::io {

namespace {

class DSSContext {
 public:
  DSSContext() {
    ScopedDSSFloatingPointEnv fp_env;
    ctx_ = ctx_New();
    if (ctx_ == nullptr) {
      throw std::runtime_error("Failed to create DSS C-API context.");
    }

    error_ptr_ = ctx_Error_Get_NumberPtr(ctx_);
    ctx_DSS_Start(ctx_, 0);
    check("ctx_DSS_Start");
  }

  ~DSSContext() {
    if (ctx_ != nullptr) {
      ScopedDSSFloatingPointEnv fp_env;
      ctx_Dispose(ctx_);
    }
  }

  DSSContext(const DSSContext&) = delete;
  DSSContext& operator=(const DSSContext&) = delete;

  const void* get() const { return ctx_; }

  void check(const char* what = nullptr) const {
    if (error_ptr_ == nullptr || *error_ptr_ == 0) {
      return;
    }

    const int32_t code = *error_ptr_;
    const char* description = ctx_Error_Get_Description(ctx_);
    *error_ptr_ = 0;

    std::string message =
        (description != nullptr) ? std::string(description) : "Unknown DSS error";
    if (what != nullptr && *what != '\0') {
      throw std::runtime_error(std::string(what) + " failed: " + message +
                               " (code " + std::to_string(code) + ")");
    }
    throw std::runtime_error(message + " (code " + std::to_string(code) + ")");
  }

 private:
  const void* ctx_{nullptr};
  int32_t* error_ptr_{nullptr};
};

std::string ascii_lower(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(),
                 [](unsigned char ch) {
                   return static_cast<char>(std::tolower(ch));
                 });
  return value;
}

std::string quote_path_for_dss(const std::filesystem::path& path) {
  return "\"" + path.string() + "\"";
}

std::string short_element_name(const std::string& full_name) {
  const std::size_t dot_pos = full_name.find('.');
  if (dot_pos == std::string::npos || dot_pos + 1 >= full_name.size()) {
    return full_name;
  }
  return full_name.substr(dot_pos + 1);
}

std::optional<OpenDSSPDElementKind> supported_pd_element_kind(
    const std::string& full_name) {
  const std::string lowered = ascii_lower(full_name);
  if (lowered.rfind("line.", 0) == 0) {
    return OpenDSSPDElementKind::Line;
  }
  if (lowered.rfind("transformer.", 0) == 0) {
    return OpenDSSPDElementKind::Transformer;
  }
  return std::nullopt;
}

template <typename Fn, typename... Args>
std::vector<double> get_double_array(const DSSContext& api, Fn fn, Args... args) {
  double* values = nullptr;
  int32_t dims[4] = {0, 0, 0, 0};
  if constexpr (std::is_invocable_v<Fn, const void*, double**, int32_t*, Args...>) {
    fn(api.get(), &values, dims, args...);
  } else {
    fn(api.get(), &values, dims);
  }
  api.check();

  if (values == nullptr || dims[0] <= 0) {
    return {};
  }

  return {values, values + dims[0]};
}

template <typename Fn, typename... Args>
std::vector<int32_t> get_int_array(const DSSContext& api, Fn fn, Args... args) {
  int32_t* values = nullptr;
  int32_t dims[4] = {0, 0, 0, 0};
  if constexpr (std::is_invocable_v<Fn, const void*, int32_t**, int32_t*, Args...>) {
    fn(api.get(), &values, dims, args...);
  } else {
    fn(api.get(), &values, dims);
  }
  api.check();

  if (values == nullptr || dims[0] <= 0) {
    return {};
  }

  return {values, values + dims[0]};
}

template <typename Fn, typename... Args>
std::vector<std::string> get_string_array(const DSSContext& api,
                                          Fn fn,
                                          Args... args) {
  char** values = nullptr;
  int32_t dims[4] = {0, 0, 0, 0};
  if constexpr (std::is_invocable_v<Fn, const void*, char***, int32_t*, Args...>) {
    fn(api.get(), &values, dims, args...);
  } else {
    fn(api.get(), &values, dims);
  }
  api.check();

  std::vector<std::string> out;
  if (values == nullptr || dims[0] <= 0) {
    return out;
  }

  out.reserve(static_cast<std::size_t>(dims[0]));
  for (int32_t i = 0; i < dims[0]; ++i) {
    out.emplace_back(values[i] != nullptr ? values[i] : "");
  }
  return out;
}

std::string get_string_value(const DSSContext& api,
                             const char* (*fn)(const void*)) {
  const char* value = fn(api.get());
  api.check();
  return (value != nullptr) ? std::string(value) : std::string();
}

template <typename Fn>
int get_int_value(const DSSContext& api, Fn fn, const char* what) {
  const int value = static_cast<int>(fn(api.get()));
  api.check(what);
  return value;
}

template <typename Fn>
double get_double_value(const DSSContext& api, Fn fn, const char* what) {
  const double value = fn(api.get());
  api.check(what);
  return value;
}

std::string get_active_dss_property_value(const DSSContext& api,
                                          const char* property_name,
                                          const char* what) {
  ctx_DSSProperty_Set_Name(api.get(), property_name);
  api.check("ctx_DSSProperty_Set_Name");
  const char* value = ctx_DSSProperty_Get_Val(api.get());
  api.check(what);
  return (value != nullptr) ? std::string(value) : std::string();
}

double get_active_dss_property_double(const DSSContext& api,
                                      const char* property_name,
                                      const char* what,
                                      double default_value = 0.0) {
  const std::string raw = get_active_dss_property_value(api, property_name, what);
  if (raw.empty()) {
    return default_value;
  }
  try {
    return std::stod(raw);
  } catch (const std::exception&) {
    throw std::runtime_error(std::string(what) + " returned non-numeric value '" +
                             raw + "'");
  }
}

template <typename ElementResult>
void populate_pd_element_snapshot(const DSSContext& api,
                                  ElementResult& element_result) {
  element_result.terminal_bus_names =
      get_string_array(api, ctx_CktElement_Get_BusNames, uint16_t{0});
  for (auto& name : element_result.terminal_bus_names) {
    name = ascii_lower(name);
  }

  const auto node_order = get_int_array(api, ctx_CktElement_Get_NodeOrder);
  element_result.node_order.assign(node_order.begin(), node_order.end());

  element_result.num_phases = ctx_CktElement_Get_NumPhases(api.get());
  api.check("ctx_CktElement_Get_NumPhases");
  element_result.num_conductors = ctx_CktElement_Get_NumConductors(api.get());
  api.check("ctx_CktElement_Get_NumConductors");
  element_result.num_terminals = ctx_CktElement_Get_NumTerminals(api.get());
  api.check("ctx_CktElement_Get_NumTerminals");

  const auto powers = get_double_array(api, ctx_CktElement_Get_Powers);
  const std::size_t pair_count = powers.size() / 2;
  const std::size_t expected_count = static_cast<std::size_t>(
      std::max(0, element_result.num_conductors * element_result.num_terminals));
  const std::size_t value_count = std::min(pair_count, expected_count);

  element_result.terminal_powers.reserve(value_count);
  for (std::size_t linear_idx = 0; linear_idx < value_count; ++linear_idx) {
    const int terminal =
        static_cast<int>(linear_idx / element_result.num_conductors) + 1;
    const int conductor =
        static_cast<int>(linear_idx % element_result.num_conductors) + 1;
    const int node =
        (linear_idx < node_order.size()) ? node_order[linear_idx] : conductor;

    element_result.terminal_powers.push_back(OpenDSSTerminalPower{
        .terminal = terminal,
        .conductor = conductor,
        .node = node,
        .power_kw_kvar =
            {
                .p_kw = powers[2 * linear_idx],
                .q_kvar = powers[2 * linear_idx + 1],
            },
    });
  }

  const auto element_losses = get_double_array(api, ctx_CktElement_Get_Losses);
  if (element_losses.size() >= 2) {
    element_result.losses_raw = {
        .p_w = element_losses[0],
        .q_var = element_losses[1],
    };
  }
}

OpenDSSLineResult project_line_result(const OpenDSSPDElementResult& element) {
  OpenDSSLineResult line_result;
  line_result.element_name = element.element_name;
  line_result.line_name = element.name;
  line_result.terminal_bus_names = element.terminal_bus_names;
  line_result.node_order = element.node_order;
  line_result.num_phases = element.num_phases;
  line_result.num_conductors = element.num_conductors;
  line_result.num_terminals = element.num_terminals;
  line_result.terminal_powers = element.terminal_powers;
  line_result.losses_raw = element.losses_raw;
  return line_result;
}

}  // namespace

const char* opendss_pd_element_kind_to_string(OpenDSSPDElementKind kind) {
  switch (kind) {
    case OpenDSSPDElementKind::Line:
      return "line";
    case OpenDSSPDElementKind::Transformer:
      return "transformer";
  }
  return "unknown";
}

OpenDSSSnapshotResult solve_opendss_snapshot(
    const std::filesystem::path& master_dss) {
  ScopedDSSFloatingPointEnv fp_env;
  const std::filesystem::path absolute_master =
      std::filesystem::absolute(master_dss);
  if (!std::filesystem::exists(absolute_master)) {
    throw std::runtime_error(
        "solve_opendss_snapshot: Master.dss not found at " +
        absolute_master.string());
  }

  DSSContext api;
  ctx_DSS_ClearAll(api.get());
  api.check("ctx_DSS_ClearAll");

  ctx_Error_Set_EarlyAbort(api.get(), 1);
  api.check("ctx_Error_Set_EarlyAbort");
  ctx_Error_Set_ExtendedErrors(api.get(), 1);
  api.check("ctx_Error_Set_ExtendedErrors");
  ctx_DSS_Set_COMErrorResults(api.get(), 0);
  api.check("ctx_DSS_Set_COMErrorResults");
  ctx_DSS_Set_AllowForms(api.get(), 0);
  api.check("ctx_DSS_Set_AllowForms");
  ctx_DSS_Set_AllowEditor(api.get(), 0);
  api.check("ctx_DSS_Set_AllowEditor");
  ctx_DSS_Set_AllowDOScmd(api.get(), 0);
  api.check("ctx_DSS_Set_AllowDOScmd");
  ctx_DSS_Set_AllowChangeDir(api.get(), 0);
  api.check("ctx_DSS_Set_AllowChangeDir");

  const std::string data_path = absolute_master.parent_path().string();
  ctx_DSS_Set_DataPath(api.get(), data_path.c_str());
  api.check("ctx_DSS_Set_DataPath");

  const std::string compile_command =
      "compile " + quote_path_for_dss(absolute_master);
  ctx_Text_Set_Command(api.get(), compile_command.c_str());
  api.check("ctx_Text_Set_Command");
  // Always run a final Solve after Compile.
  // Some feeders call CalcVoltageBases during Compile, which performs a
  // zero-load solve and leaves Solution.Converged=true even though the final
  // loaded operating point has not been solved yet.
  ctx_Solution_Solve(api.get());
  api.check("ctx_Solution_Solve");

  OpenDSSSnapshotResult result;
  result.engine_version = get_string_value(api, ctx_DSS_Get_Version);
  result.converged = (ctx_Solution_Get_Converged(api.get()) != 0);
  api.check("ctx_Solution_Get_Converged");

  const auto bus_names = get_string_array(api, ctx_Circuit_Get_AllBusNames);
  for (const auto& bus_name : bus_names) {
    ctx_Circuit_SetActiveBus(api.get(), bus_name.c_str());
    api.check("ctx_Circuit_SetActiveBus");

    const auto pu_mag_angle = get_double_array(api, ctx_Bus_Get_puVmagAngle);
    const auto mag_angle = get_double_array(api, ctx_Bus_Get_VMagAngle);
    // Actual DSS node numbers for this bus, ordered to match the voltage arrays.
    // ctx_Bus_Get_Nodes: "Integer Array of Node Numbers defined at the bus
    // in same order as the voltages."  Using idx+1 as a fallback was incorrect
    // for partial-phase buses (e.g. a 2-phase bus on nodes 1&3 reported [1,2]).
    const auto bus_node_ids = get_int_array(api, ctx_Bus_Get_Nodes);
    api.check("ctx_Bus_Get_Nodes");
    const std::size_t node_count =
        std::min(pu_mag_angle.size(), mag_angle.size()) / 2;

    for (std::size_t idx = 0; idx < node_count; ++idx) {
      const int node_id = (idx < bus_node_ids.size())
                              ? bus_node_ids[static_cast<std::size_t>(idx)]
                              : static_cast<int>(idx + 1);
      result.node_voltages.push_back(OpenDSSNodeVoltage{
          .bus_name = ascii_lower(bus_name),
          .node = node_id,
          .vm_pu = pu_mag_angle[2 * idx],
          .va_deg = pu_mag_angle[2 * idx + 1],
          .vm_vln = mag_angle[2 * idx],
      });
    }
  }

  const auto element_names = get_string_array(api, ctx_Circuit_Get_AllElementNames);
  for (const auto& raw_element_name : element_names) {
    const std::string element_name = ascii_lower(raw_element_name);
    const auto element_kind = supported_pd_element_kind(element_name);
    if (!element_kind.has_value()) {
      continue;
    }

    ctx_Circuit_SetActiveElement(api.get(), raw_element_name.c_str());
    api.check("ctx_Circuit_SetActiveElement");
    if (ctx_CktElement_Get_Enabled(api.get()) == 0) {
      api.check("ctx_CktElement_Get_Enabled");
      continue;
    }
    api.check("ctx_CktElement_Get_Enabled");

    OpenDSSPDElementResult element_result;
    element_result.element_kind = *element_kind;
    element_result.element_name =
        ascii_lower(get_string_value(api, ctx_CktElement_Get_Name));
    element_result.name = ascii_lower(short_element_name(element_result.element_name));
    populate_pd_element_snapshot(api, element_result);
    result.pd_element_results.push_back(std::move(element_result));
  }

  result.line_results.reserve(result.pd_element_results.size());
  for (const auto& element : result.pd_element_results) {
    if (element.element_kind == OpenDSSPDElementKind::Line) {
      result.line_results.push_back(project_line_result(element));
    }
  }

  for (int has_transformer = ctx_Transformers_Get_First(api.get());
       has_transformer != 0;
       has_transformer = ctx_Transformers_Get_Next(api.get())) {
    api.check("ctx_Transformers_Get_First/Next");

    OpenDSSTransformerState transformer_state;
    transformer_state.name =
        ascii_lower(get_string_value(api, ctx_Transformers_Get_Name));
    transformer_state.num_windings = get_int_value(
        api, ctx_Transformers_Get_NumWindings, "ctx_Transformers_Get_NumWindings");
    transformer_state.winding_states.reserve(
        static_cast<std::size_t>(std::max(0, transformer_state.num_windings)));

    for (int winding = 1; winding <= transformer_state.num_windings; ++winding) {
      ctx_Transformers_Set_Wdg(api.get(), winding);
      api.check("ctx_Transformers_Set_Wdg");

      transformer_state.winding_states.push_back(OpenDSSTransformerWindingState{
          .winding = winding,
          .tap_pu = get_double_value(
              api, ctx_Transformers_Get_Tap, "ctx_Transformers_Get_Tap"),
          .min_tap_pu = get_double_value(
              api, ctx_Transformers_Get_MinTap, "ctx_Transformers_Get_MinTap"),
          .max_tap_pu = get_double_value(
              api, ctx_Transformers_Get_MaxTap, "ctx_Transformers_Get_MaxTap"),
          .num_taps = get_int_value(
              api, ctx_Transformers_Get_NumTaps, "ctx_Transformers_Get_NumTaps"),
          .kv = get_double_value(api, ctx_Transformers_Get_kV, "ctx_Transformers_Get_kV"),
          .kva = get_double_value(
              api, ctx_Transformers_Get_kVA, "ctx_Transformers_Get_kVA"),
      });
    }

    result.transformer_states.push_back(std::move(transformer_state));
  }

  for (int has_regcontrol = ctx_RegControls_Get_First(api.get());
       has_regcontrol != 0;
       has_regcontrol = ctx_RegControls_Get_Next(api.get())) {
    api.check("ctx_RegControls_Get_First/Next");

    result.regcontrol_results.push_back(OpenDSSRegControlResult{
        .name = ascii_lower(get_string_value(api, ctx_RegControls_Get_Name)),
        .transformer_name =
            ascii_lower(get_string_value(api, ctx_RegControls_Get_Transformer)),
        .monitored_bus_name =
            ascii_lower(get_string_value(api, ctx_RegControls_Get_MonitoredBus)),
        .winding = get_int_value(
            api, ctx_RegControls_Get_Winding, "ctx_RegControls_Get_Winding"),
        .tap_winding = get_int_value(
            api, ctx_RegControls_Get_TapWinding, "ctx_RegControls_Get_TapWinding"),
        .tap_number = get_int_value(
            api, ctx_RegControls_Get_TapNumber, "ctx_RegControls_Get_TapNumber"),
        .max_tap_change = get_int_value(
            api, ctx_RegControls_Get_MaxTapChange,
            "ctx_RegControls_Get_MaxTapChange"),
        .forward_vreg_volts = get_double_value(
            api, ctx_RegControls_Get_ForwardVreg,
            "ctx_RegControls_Get_ForwardVreg"),
        .forward_band_volts = get_double_value(
            api, ctx_RegControls_Get_ForwardBand,
            "ctx_RegControls_Get_ForwardBand"),
        .ptratio = get_double_value(
            api, ctx_RegControls_Get_PTratio, "ctx_RegControls_Get_PTratio"),
        .remote_ptratio = get_active_dss_property_double(
            api, "remoteptratio", "ctx_DSSProperty_Get_Val(remoteptratio)"),
        .ct_primary_amps = get_double_value(
            api, ctx_RegControls_Get_CTPrimary,
            "ctx_RegControls_Get_CTPrimary"),
        .forward_r_volts = get_double_value(
            api, ctx_RegControls_Get_ForwardR, "ctx_RegControls_Get_ForwardR"),
        .forward_x_volts = get_double_value(
            api, ctx_RegControls_Get_ForwardX, "ctx_RegControls_Get_ForwardX"),
        .voltage_limit_volts = get_double_value(
            api, ctx_RegControls_Get_VoltageLimit,
            "ctx_RegControls_Get_VoltageLimit"),
        .is_reversible = (get_int_value(
                              api, ctx_RegControls_Get_IsReversible,
                              "ctx_RegControls_Get_IsReversible") != 0),
    });
  }

  result.control_oracle.control_iterations = get_int_value(
      api, ctx_Solution_Get_ControlIterations, "ctx_Solution_Get_ControlIterations");
  result.control_oracle.max_control_iterations = get_int_value(
      api, ctx_Solution_Get_MaxControlIterations,
      "ctx_Solution_Get_MaxControlIterations");
  const auto event_log = get_string_array(api, ctx_Solution_Get_EventLog);
  result.control_oracle.event_log_entries.reserve(event_log.size());
  for (std::size_t idx = 0; idx < event_log.size(); ++idx) {
    result.control_oracle.event_log_entries.push_back(OpenDSSEventLogEntry{
        .event_index = static_cast<int>(idx + 1),
        .message = event_log[idx],
    });
  }

  const auto losses = get_double_array(api, ctx_Circuit_Get_Losses);
  if (losses.size() >= 2) {
    result.circuit_losses_raw = {
        .p_w = losses[0],
        .q_var = losses[1],
    };
  }

  return result;
}

}  // namespace hacdcpf::io
