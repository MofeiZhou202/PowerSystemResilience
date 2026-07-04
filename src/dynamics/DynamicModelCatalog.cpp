#include "hacdcpf/dynamics/DynamicModelCatalog.hpp"

#include <algorithm>

namespace hacdcpf::dynamics {
namespace {

using D = DynamicParamDescriptor;

// Compact descriptor constructor for a real-valued parameter.
D real(std::string key, std::vector<std::string> aliases, std::string label,
       std::string unit, double def, std::optional<double> lo, std::optional<double> hi,
       std::string group, bool advanced = false) {
  D d;
  d.key = std::move(key);
  d.aliases = std::move(aliases);
  d.label = std::move(label);
  d.unit = std::move(unit);
  d.kind = DynamicParamKind::Real;
  d.default_value = def;
  d.min_value = lo;
  d.max_value = hi;
  d.group = std::move(group);
  d.advanced = advanced;
  return d;
}

DynamicModelDescriptor model(std::string name, std::string standard, std::string display,
                             std::string role, std::vector<D> params,
                             std::string loc = "root") {
  DynamicModelDescriptor m;
  m.model_name = std::move(name);
  m.standard = std::move(standard);
  m.display_name = std::move(display);
  m.block_role = std::move(role);
  m.parameters_location = std::move(loc);
  m.parameters = std::move(params);
  return m;
}

std::vector<DynamicModelDescriptor> build_catalog() {
  std::vector<DynamicModelDescriptor> c;

  // ── Synchronous machine models ──
  c.push_back(model(
      "GENROU", "IEEE", "GENROU round-rotor machine", "machine",
      {real("H", {"h"}, "Inertia constant", "s", 3.5, 0.1, 30.0, "Rotor"),
       real("D", {"damping_d", "damping"}, "Damping", "pu", 0.0, 0.0, 10.0, "Rotor"),
       real("R", {"Ra", "r"}, "Stator resistance", "pu", 0.0, 0.0, 1.0, "Reactances", true),
       real("Xd", {"xd"}, "d-axis synchronous reactance", "pu", 1.8, 0.1, 3.0, "Reactances"),
       real("Xq", {"xq"}, "q-axis synchronous reactance", "pu", 1.7, 0.1, 3.0, "Reactances"),
       real("Xd_p", {"Xdp", "xd_p", "xdp"}, "d-axis transient reactance", "pu", 0.30, 0.0, 2.0, "Reactances"),
       real("Xq_p", {"Xqp", "xq_p", "xqp"}, "q-axis transient reactance", "pu", 0.55, 0.0, 2.0, "Reactances"),
       real("Xd_pp", {"Xdpp", "xd_pp", "xdpp"}, "Subtransient reactance", "pu", 0.25, 0.0, 2.0, "Reactances"),
       real("Xl", {"xl"}, "Leakage reactance", "pu", 0.20, 0.0, 1.0, "Reactances"),
       real("Td0_p", {"Td0p", "td0_p", "td0p"}, "d-axis transient time const", "s", 8.0, 0.1, 20.0, "Time constants"),
       real("Td0_pp", {"Td0pp", "td0_pp", "td0pp"}, "d-axis subtransient time const", "s", 0.03, 0.001, 1.0, "Time constants"),
       real("Tq0_p", {"Tq0p", "tq0_p", "tq0p"}, "q-axis transient time const", "s", 0.4, 0.01, 10.0, "Time constants"),
       real("Tq0_pp", {"Tq0pp", "tq0_pp", "tq0pp"}, "q-axis subtransient time const", "s", 0.05, 0.001, 1.0, "Time constants"),
       real("Sat_A", {"saturation_a", "Se_A"}, "Saturation coeff A", "", 0.0, 0.0, 5.0, "Saturation", true),
       real("Sat_B", {"saturation_b", "Se_B"}, "Saturation coeff B", "", 0.0, 0.0, 5.0, "Saturation", true)}));
  c.push_back(model(
      "OneDOneQMachine", "PowerSystems", "One d-axis / one q-axis machine", "machine",
      {real("H", {"h"}, "Inertia constant", "s", 3.5, 0.1, 30.0, "Rotor"),
       real("D", {"damping_d", "damping"}, "Damping", "pu", 0.0, 0.0, 10.0, "Rotor"),
       real("R", {"Ra", "r"}, "Stator resistance", "pu", 0.0, 0.0, 1.0, "Reactances", true),
       real("Xd", {"xd"}, "d-axis synchronous reactance", "pu", 1.3125, 0.1, 3.0, "Reactances"),
       real("Xq", {"xq"}, "q-axis synchronous reactance", "pu", 1.2578, 0.1, 3.0, "Reactances"),
       real("Xd_p", {"Xdp", "xd_p", "xdp"}, "d-axis transient reactance", "pu", 0.1813, 0.0, 2.0, "Reactances"),
       real("Xq_p", {"Xqp", "xq_p", "xqp"}, "q-axis transient reactance", "pu", 0.25, 0.0, 2.0, "Reactances"),
       real("Td0_p", {"Td0p", "td0_p", "td0p"}, "d-axis transient time const", "s", 5.89, 0.1, 20.0, "Time constants"),
       real("Tq0_p", {"Tq0p", "tq0_p", "tq0p"}, "q-axis transient time const", "s", 0.6, 0.01, 10.0, "Time constants")}));
  c.push_back(model(
      "ClassicalMachine", "IEEE", "Classical (E' behind X') machine", "machine", {}));

  // ── Governors ──
  c.push_back(model(
      "TGOV1", "IEEE", "TGOV1 steam governor", "governor",
      {real("R", {"droop_r", "droop"}, "Droop", "pu", 0.05, 0.0, 0.2, "Droop"),
       real("T1", {"Tg", "valve_t_s"}, "Governor/valve time const", "s", 0.5, 0.0, 10.0, "Time constants"),
       real("T3", {"Tt", "turbine_t_s"}, "Turbine time const", "s", 0.5, 0.0, 20.0, "Time constants"),
       real("Vmax", {"pmax_pu"}, "Max valve/power", "pu", 0.0, 0.0, 2.0, "Limits", true),
       real("Vmin", {"pmin_pu"}, "Min valve/power", "pu", 0.0, -2.0, 0.0, "Limits", true)}));
  c.push_back(model(
      "IEEEG1", "IEEE", "IEEEG1 reheat governor", "governor",
      {real("R", {"droop_r", "droop"}, "Droop", "pu", 0.05, 0.0, 0.2, "Droop"),
       real("T1", {"Tg", "valve_t_s"}, "Governor/valve time const", "s", 0.5, 0.0, 10.0, "Time constants"),
       real("T3", {"Tt", "turbine_t_s"}, "Turbine time const", "s", 0.5, 0.0, 20.0, "Time constants"),
       real("Tr", {"T5", "reheat_t_s"}, "Reheat time const", "s", 6.0, 0.0, 30.0, "Time constants"),
       real("K1", {"Khp", "reheat_k"}, "HP fraction", "", 0.30, 0.0, 1.0, "Reheat"),
       real("Vmax", {"pmax_pu"}, "Max valve/power", "pu", 0.0, 0.0, 2.0, "Limits", true),
       real("Vmin", {"pmin_pu"}, "Min valve/power", "pu", 0.0, -2.0, 0.0, "Limits", true)}));

  // ── Exciters / AVRs ──
  c.push_back(model(
      "SEXS", "IEEE4215", "SEXS simplified excitation", "exciter",
      {real("Ka", {"K", "ka"}, "Regulator gain", "pu", 20.0, 1.0, 400.0, "Gain"),
       real("Ta", {"ta", "Tb"}, "Regulator time const", "s", 0.05, 0.0, 5.0, "Time constants"),
       real("Emax", {"Vrmax", "efd_max_pu"}, "Max field voltage", "pu", 5.0, 0.0, 10.0, "Limits"),
       real("Emin", {"Vrmin", "efd_min_pu"}, "Min field voltage", "pu", 0.0, -10.0, 5.0, "Limits"),
       real("Vref", {"v_ref_pu"}, "Voltage setpoint", "pu", 1.0, 0.5, 1.5, "Setpoint", true)}));
  c.push_back(model(
      "IEEET1", "IEEE4215", "IEEET1 DC exciter", "exciter",
      {real("Ka", {"K", "ka"}, "Regulator gain", "pu", 20.0, 1.0, 400.0, "Gain"),
       real("Ta", {"ta", "Tb"}, "Regulator time const", "s", 0.05, 0.0, 5.0, "Time constants"),
       real("Te", {"te"}, "Exciter time const", "s", 0.4, 0.0, 5.0, "Time constants"),
       real("Emax", {"Vrmax", "efd_max_pu"}, "Max field voltage", "pu", 5.0, 0.0, 10.0, "Limits"),
       real("Emin", {"Vrmin", "efd_min_pu"}, "Min field voltage", "pu", 0.0, -10.0, 5.0, "Limits"),
       real("Vref", {"v_ref_pu"}, "Voltage setpoint", "pu", 1.0, 0.5, 1.5, "Setpoint", true)}));

  // ── Power system stabilizer ──
  c.push_back(model(
      "PSS1A", "IEEE", "PSS1A single-input speed stabilizer", "pss",
      {real("Ks", {"Ks1", "ks"}, "Stabilizer gain", "pu", 5.0, 0.0, 50.0, "Gain"),
       real("Tw", {"tw"}, "Washout time const", "s", 10.0, 0.1, 30.0, "Washout"),
       real("T1", {"t1"}, "Lead-lag 1 numerator", "s", 0.15, 0.0, 2.0, "Lead-lag"),
       real("T2", {"t2"}, "Lead-lag 1 denominator", "s", 0.03, 0.001, 2.0, "Lead-lag"),
       real("T3", {"t3"}, "Lead-lag 2 numerator", "s", 0.15, 0.0, 2.0, "Lead-lag"),
       real("T4", {"t4"}, "Lead-lag 2 denominator", "s", 0.03, 0.001, 2.0, "Lead-lag"),
       real("Vsmax", {"vs_max_pu", "Vstmax"}, "Max output", "pu", 0.10, 0.0, 0.5, "Limits"),
       real("Vsmin", {"vs_min_pu", "Vstmin"}, "Min output", "pu", -0.10, -0.5, 0.0, "Limits")}));

  // ── Grid-following inverter (also used for PV / renewable / static gen) ──
  c.push_back(model(
      "REGC_REEC_GFL_Subset", "NERC", "Grid-following converter (REGC/REEC subset)", "gfl",
      {real("pll_kp", {"kp_pll"}, "PLL proportional gain", "", 0.01, 0.0, 5.0, "PLL"),
       real("pll_ki", {"ki_pll"}, "PLL integral gain", "", 1.0, 0.0, 100.0, "PLL"),
       real("pll_lpf_t_s", {}, "PLL low-pass time const", "s", 0.005, 0.0, 0.5, "PLL"),
       real("response_t_s", {"Tg", "Trv"}, "Current response time const", "s", 0.02, 0.0, 1.0, "Response"),
       real("power_filter_t_s", {"Tp", "Tpf"}, "Power filter time const", "s", 0.02, 0.0, 1.0, "Response"),
       real("current_limit_pu", {"Imax", "imax_pu"}, "Current limit (0=off)", "pu", 0.0, 0.0, 3.0, "Limits"),
       real("frequency_watt_droop_pu", {"Ddn", "kf"}, "Frequency-watt droop", "pu", 0.0, 0.0, 50.0, "Droop", true),
       real("volt_var_droop_pu", {"Dvv", "kq"}, "Volt-var droop", "pu", 0.0, 0.0, 50.0, "Droop", true)}));

  // ── Grid-forming inverter ──
  c.push_back(model(
      "GridFormingNortonDroop", "NERC", "Grid-forming droop converter", "gfm",
      {real("virtual_x_pu", {"Xv", "xv"}, "Virtual reactance", "pu", 0.10, 0.0, 1.0, "Virtual impedance"),
       real("virtual_r_pu", {"Rv", "rv"}, "Virtual resistance", "pu", 0.0, 0.0, 1.0, "Virtual impedance"),
       real("p_droop_pu", {"mp", "Dp"}, "P-f droop", "pu", 0.01, 0.0, 1.0, "Droop"),
       real("q_droop_pu", {"mq", "Dq"}, "Q-V droop", "pu", 0.05, 0.0, 1.0, "Droop"),
       real("power_filter_t_s", {"Tf", "Tpf"}, "Power filter time const", "s", 0.05, 0.0, 1.0, "Response"),
       real("voltage_control_t_s", {"Tv"}, "Voltage control time const", "s", 0.02, 0.0, 1.0, "Response", true),
       real("voltage_kp", {"Kpv"}, "Voltage proportional gain", "", 0.1, 0.0, 10.0, "Voltage control", true),
       real("voltage_ki", {"Kiv"}, "Voltage integral gain", "", 10.0, 0.0, 100.0, "Voltage control", true),
       real("current_limit_pu", {"Imax", "imax_pu"}, "Current limit (0=off)", "pu", 0.0, 0.0, 3.0, "Limits")}));

  // ── PLL variants (component-level parameters under a "pll" child) ──
  for (const char* name : {"ReducedOrderPLL", "KauraPLL", "FixedFrequency"}) {
    c.push_back(model(
        name, "NERC", std::string(name), "pll",
        {real("kp_pll", {"pll_kp"}, "PLL proportional gain", "", 0.01, 0.0, 5.0, "PLL"),
         real("ki_pll", {"pll_ki"}, "PLL integral gain", "", 1.0, 0.0, 100.0, "PLL"),
         real("pll_lpf_t_s", {}, "PLL low-pass time const", "s", 0.005, 0.0, 0.5, "PLL")},
        "component:pll"));
  }

  // ── DC/DC converter ──
  c.push_back(model(
      "FirstOrderDCDCConverter", "HACDCPF", "First-order DC/DC converter", "dcdc",
      {real("eta", {"efficiency"}, "Efficiency", "", 0.98, 0.5, 1.0, "Losses"),
       real("response_t_s", {"Tp", "tp"}, "Response time const", "s", 0.02, 0.0, 1.0, "Response")}));

  // ── Battery / storage ──
  c.push_back(model(
      "BatterySOCFirstOrder", "IEEE1547", "Battery SOC first-order model", "storage",
      {real("response_t_s", {"Tp", "tp"}, "Power response time const", "s", 0.05, 0.0, 1.0, "Response"),
       real("e_rated_mwh", {"Erated"}, "Rated energy", "MWh", 0.0, 0.0, 1e4, "Capacity"),
       real("soc_init", {"SOC0", "soc0"}, "Initial SOC", "", 0.5, 0.0, 1.0, "SOC"),
       real("soc_min", {"SOCmin"}, "Minimum SOC", "", 0.0, 0.0, 1.0, "SOC"),
       real("soc_max", {"SOCmax"}, "Maximum SOC", "", 1.0, 0.0, 1.0, "SOC"),
       real("eta_charge", {"eta_c"}, "Charge efficiency", "", 0.95, 0.5, 1.0, "Efficiency"),
       real("eta_discharge", {"eta_d"}, "Discharge efficiency", "", 0.95, 0.5, 1.0, "Efficiency"),
       real("self_discharge_pct_per_h", {}, "Self-discharge", "%/h", 0.0, 0.0, 100.0, "SOC", true)}));

  // ── Load models (model_name selects behaviour; no numeric dynamic_model params) ──
  for (const char* nm : {"ConstantPower", "ConstantCurrent", "ConstantImpedance", "ZIP"}) {
    c.push_back(model(nm, "HACDCPF", std::string(nm) + " load", "load", {}));
  }
  return c;
}

DynamicControlBlockSlot slot(std::string s, std::string label, bool optional,
                             std::vector<std::string> models, std::string def,
                             std::string visible_when = "") {
  DynamicControlBlockSlot b;
  b.slot = std::move(s);
  b.label = std::move(label);
  b.optional = optional;
  b.model_names = std::move(models);
  b.default_model = std::move(def);
  b.visible_when_model = std::move(visible_when);
  return b;
}

std::vector<DynamicComponentComposition> build_composition() {
  std::vector<DynamicComponentComposition> comps;

  comps.push_back(
      {"gen", "Synchronous generator", "AC",
       {slot("machine", "Machine model", false,
             {"GENROU", "OneDOneQMachine", "ClassicalMachine"}, "ClassicalMachine"),
        slot("governor", "Governor", true, {"TGOV1", "IEEEG1"}, "None"),
        slot("exciter", "Exciter / AVR", true, {"SEXS", "IEEET1"}, "None"),
        slot("pss", "Power system stabilizer", true, {"PSS1A"}, "None")}});

  comps.push_back(
      {"vsc", "VSC converter", "AC",
       {slot("converter", "Converter control", false,
             {"REGC_REEC_GFL_Subset", "GridFormingNortonDroop"}, "REGC_REEC_GFL_Subset"),
        slot("pll", "PLL", false, {"ReducedOrderPLL", "KauraPLL", "FixedFrequency"},
             "ReducedOrderPLL", "REGC_REEC_GFL_Subset")}});

  const auto gfl_component = [](std::string canvas, std::string name) {
    return DynamicComponentComposition{
        std::move(canvas), std::move(name), "AC",
        {slot("converter", "Converter control", false, {"REGC_REEC_GFL_Subset"},
              "REGC_REEC_GFL_Subset"),
         slot("pll", "PLL", false, {"ReducedOrderPLL", "KauraPLL", "FixedFrequency"},
              "ReducedOrderPLL")}};
  };
  comps.push_back(gfl_component("pv", "PV system"));
  comps.push_back(gfl_component("sgen", "Static generator"));
  comps.push_back(gfl_component("renGen", "Renewable generator"));

  comps.push_back(
      {"storage", "Battery storage", "AC",
       {slot("storage", "Storage model", false, {"BatterySOCFirstOrder"}, "BatterySOCFirstOrder"),
        slot("converter", "Grid-forming control", true, {"GridFormingNortonDroop"}, "None")}});

  comps.push_back(
      {"dcdcConverter", "DC/DC converter", "DC",
       {slot("converter", "Converter model", false, {"FirstOrderDCDCConverter"},
             "FirstOrderDCDCConverter")}});

  comps.push_back(
      {"load", "Load", "AC",
       {slot("load", "Load model", false,
             {"ConstantPower", "ConstantCurrent", "ConstantImpedance", "ZIP"}, "ZIP")}});

  return comps;
}

}  // namespace

const std::vector<DynamicModelDescriptor>& dynamic_model_catalog() {
  static const std::vector<DynamicModelDescriptor> catalog = build_catalog();
  return catalog;
}

const std::vector<DynamicComponentComposition>& dynamic_block_composition() {
  static const std::vector<DynamicComponentComposition> comps = build_composition();
  return comps;
}

std::optional<DynamicModelDescriptor> find_dynamic_model(std::string_view model_name) {
  for (const auto& m : dynamic_model_catalog()) {
    if (m.model_name == model_name) return m;
  }
  return std::nullopt;
}

std::optional<DynamicComponentComposition> find_block_composition(
    std::string_view canvas_type) {
  for (const auto& c : dynamic_block_composition()) {
    if (c.canvas_type == canvas_type) return c;
  }
  return std::nullopt;
}

std::string to_string(DynamicParamKind kind) {
  switch (kind) {
    case DynamicParamKind::Real: return "Real";
    case DynamicParamKind::Integer: return "Integer";
    case DynamicParamKind::Boolean: return "Boolean";
    case DynamicParamKind::Enum: return "Enum";
  }
  return "Real";
}

}  // namespace hacdcpf::dynamics
