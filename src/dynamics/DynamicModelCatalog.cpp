#include "hacdcpf/dynamics/DynamicModelCatalog.hpp"

#include <algorithm>
#include <utility>

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
      "GENROE", "IEEE", "GENROE round-rotor machine with exponential saturation", "machine",
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
       real("Sat_A", {"saturation_a", "Se_A"}, "Exponential saturation coeff A", "", 0.0, 0.0, 30.0, "Saturation", true),
       real("Sat_B", {"saturation_b", "Se_B"}, "Exponential saturation coeff B", "", 0.0, 0.0, 5.0, "Saturation", true)}));
  c.push_back(model(
      "GENSAL", "IEEE", "GENSAL salient-pole machine", "machine",
      {real("H", {"h"}, "Inertia constant", "s", 5.0, 0.1, 30.0, "Rotor"),
       real("D", {"damping_d", "damping"}, "Damping", "pu", 0.0, 0.0, 10.0, "Rotor"),
       real("R", {"Ra", "r"}, "Stator resistance", "pu", 0.0, 0.0, 1.0, "Reactances", true),
       real("Xd", {"xd"}, "d-axis synchronous reactance", "pu", 1.0, 0.1, 3.0, "Reactances"),
       real("Xq", {"xq"}, "q-axis synchronous reactance", "pu", 0.75, 0.1, 3.0, "Reactances"),
       real("Xd_p", {"Xdp", "xd_p", "xdp"}, "d-axis transient reactance", "pu", 0.40, 0.0, 2.0, "Reactances"),
       real("Xd_pp", {"Xdpp", "xd_pp", "xdpp"}, "Subtransient reactance", "pu", 0.25, 0.0, 2.0, "Reactances"),
       real("Xl", {"xl"}, "Leakage reactance", "pu", 0.10, 0.0, 1.0, "Reactances"),
       real("Td0_p", {"Td0p", "td0_p", "td0p"}, "d-axis transient time const", "s", 5.0, 0.1, 20.0, "Time constants"),
       real("Td0_pp", {"Td0pp", "td0_pp", "td0pp"}, "d-axis subtransient time const", "s", 0.05, 0.001, 1.0, "Time constants"),
       real("Tq0_pp", {"Tq0pp", "tq0_pp", "tq0pp"}, "q-axis subtransient time const", "s", 0.20, 0.001, 10.0, "Time constants"),
       real("Sat_A", {"saturation_a", "Se_A"}, "Saturation coeff A", "", 0.0, 0.0, 5.0, "Saturation", true),
       real("Sat_B", {"saturation_b", "Se_B"}, "Saturation coeff B", "", 0.0, 0.0, 20.0, "Saturation", true)}));
  c.push_back(model(
      "GENSAE", "IEEE", "GENSAE salient-pole machine with exponential saturation", "machine",
      {real("H", {"h"}, "Inertia constant", "s", 5.0, 0.1, 30.0, "Rotor"),
       real("D", {"damping_d", "damping"}, "Damping", "pu", 0.0, 0.0, 10.0, "Rotor"),
       real("R", {"Ra", "r"}, "Stator resistance", "pu", 0.0, 0.0, 1.0, "Reactances", true),
       real("Xd", {"xd"}, "d-axis synchronous reactance", "pu", 1.0, 0.1, 3.0, "Reactances"),
       real("Xq", {"xq"}, "q-axis synchronous reactance", "pu", 0.75, 0.1, 3.0, "Reactances"),
       real("Xd_p", {"Xdp", "xd_p", "xdp"}, "d-axis transient reactance", "pu", 0.40, 0.0, 2.0, "Reactances"),
       real("Xd_pp", {"Xdpp", "xd_pp", "xdpp"}, "Subtransient reactance", "pu", 0.25, 0.0, 2.0, "Reactances"),
       real("Xl", {"xl"}, "Leakage reactance", "pu", 0.10, 0.0, 1.0, "Reactances"),
       real("Td0_p", {"Td0p", "td0_p", "td0p"}, "d-axis transient time const", "s", 5.0, 0.1, 20.0, "Time constants"),
       real("Td0_pp", {"Td0pp", "td0_pp", "td0pp"}, "d-axis subtransient time const", "s", 0.05, 0.001, 1.0, "Time constants"),
       real("Tq0_pp", {"Tq0pp", "tq0_pp", "tq0pp"}, "q-axis subtransient time const", "s", 0.20, 0.001, 10.0, "Time constants"),
       real("Sat_A", {"saturation_a", "Se_A"}, "Exponential saturation coeff A", "", 0.0, 0.0, 30.0, "Saturation", true),
       real("Sat_B", {"saturation_b", "Se_B"}, "Exponential saturation coeff B", "", 0.0, 0.0, 5.0, "Saturation", true)}));
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
      "SimpleMarconatoMachine", "PowerSystems", "Simple Marconato machine", "machine",
      {real("H", {"h"}, "Inertia constant", "s", 3.01, 0.1, 30.0, "Rotor"),
       real("D", {"damping_d", "damping"}, "Damping", "pu", 0.0, 0.0, 10.0, "Rotor"),
       real("R", {"Ra", "r"}, "Stator resistance", "pu", 0.0, 0.0, 1.0, "Reactances", true),
       real("Xd", {"xd"}, "d-axis synchronous reactance", "pu", 1.3125, 0.1, 3.0, "Reactances"),
       real("Xq", {"xq"}, "q-axis synchronous reactance", "pu", 1.2578, 0.1, 3.0, "Reactances"),
       real("Xd_p", {"Xdp", "xd_p", "xdp"}, "d-axis transient reactance", "pu", 0.1813, 0.0, 2.0, "Reactances"),
       real("Xq_p", {"Xqp", "xq_p", "xqp"}, "q-axis transient reactance", "pu", 0.25, 0.0, 2.0, "Reactances"),
       real("Xd_pp", {"Xdpp", "xd_pp", "xdpp"}, "d-axis subtransient reactance", "pu", 0.14, 0.0, 2.0, "Reactances"),
       real("Xq_pp", {"Xqpp", "xq_pp", "xqpp"}, "q-axis subtransient reactance", "pu", 0.18, 0.0, 2.0, "Reactances"),
       real("Td0_p", {"Td0p", "td0_p", "td0p"}, "d-axis transient time const", "s", 5.89, 0.1, 20.0, "Time constants"),
       real("Tq0_p", {"Tq0p", "tq0_p", "tq0p"}, "q-axis transient time const", "s", 0.6, 0.01, 10.0, "Time constants"),
       real("Td0_pp", {"Td0pp", "td0_pp", "td0pp"}, "d-axis subtransient time const", "s", 0.5, 0.001, 2.0, "Time constants"),
       real("Tq0_pp", {"Tq0pp", "tq0_pp", "tq0pp"}, "q-axis subtransient time const", "s", 0.023, 0.001, 1.0, "Time constants"),
       real("T_AA", {"TAA", "t_aa", "taa"}, "d-axis additional leakage time const", "s", 0.0, 0.0, 2.0, "Time constants", true)}));
  c.push_back(model(
      "MarconatoMachine", "PowerSystems", "Full Marconato machine", "machine",
      {real("H", {"h"}, "Inertia constant", "s", 3.01, 0.1, 30.0, "Rotor"),
       real("D", {"damping_d", "damping"}, "Damping", "pu", 0.0, 0.0, 10.0, "Rotor"),
       real("R", {"Ra", "r"}, "Stator resistance", "pu", 0.0, 0.0, 1.0, "Reactances", true),
       real("Xd", {"xd"}, "d-axis synchronous reactance", "pu", 1.3125, 0.1, 3.0, "Reactances"),
       real("Xq", {"xq"}, "q-axis synchronous reactance", "pu", 1.2578, 0.1, 3.0, "Reactances"),
       real("Xd_p", {"Xdp", "xd_p", "xdp"}, "d-axis transient reactance", "pu", 0.1813, 0.0, 2.0, "Reactances"),
       real("Xq_p", {"Xqp", "xq_p", "xqp"}, "q-axis transient reactance", "pu", 0.25, 0.0, 2.0, "Reactances"),
       real("Xd_pp", {"Xdpp", "xd_pp", "xdpp"}, "d-axis subtransient reactance", "pu", 0.14, 0.0, 2.0, "Reactances"),
       real("Xq_pp", {"Xqpp", "xq_pp", "xqpp"}, "q-axis subtransient reactance", "pu", 0.18, 0.0, 2.0, "Reactances"),
       real("Td0_p", {"Td0p", "td0_p", "td0p"}, "d-axis transient time const", "s", 5.89, 0.1, 20.0, "Time constants"),
       real("Tq0_p", {"Tq0p", "tq0_p", "tq0p"}, "q-axis transient time const", "s", 0.6, 0.01, 10.0, "Time constants"),
       real("Td0_pp", {"Td0pp", "td0_pp", "td0pp"}, "d-axis subtransient time const", "s", 0.5, 0.001, 2.0, "Time constants"),
       real("Tq0_pp", {"Tq0pp", "tq0_pp", "tq0pp"}, "q-axis subtransient time const", "s", 0.023, 0.001, 1.0, "Time constants"),
       real("T_AA", {"TAA", "t_aa", "taa"}, "d-axis additional leakage time const", "s", 0.0, 0.0, 2.0, "Time constants", true)}));
  c.push_back(model(
      "SauerPaiMachine", "PowerSystems", "Sauer-Pai sixth-order machine", "machine",
      {real("H", {"h"}, "Inertia constant", "s", 3.01, 0.1, 30.0, "Rotor"),
       real("D", {"damping_d", "damping"}, "Damping", "pu", 0.0, 0.0, 10.0, "Rotor"),
       real("R", {"Ra", "r"}, "Stator resistance", "pu", 0.002, 0.0, 1.0, "Reactances", true),
       real("Xd", {"xd"}, "d-axis synchronous reactance", "pu", 1.79, 0.1, 3.0, "Reactances"),
       real("Xq", {"xq"}, "q-axis synchronous reactance", "pu", 1.71, 0.1, 3.0, "Reactances"),
       real("Xd_p", {"Xdp", "xd_p", "xdp"}, "d-axis transient reactance", "pu", 0.169, 0.0, 2.0, "Reactances"),
       real("Xq_p", {"Xqp", "xq_p", "xqp"}, "q-axis transient reactance", "pu", 0.228, 0.0, 2.0, "Reactances"),
       real("Xd_pp", {"Xdpp", "xd_pp", "xdpp"}, "d-axis subtransient reactance", "pu", 0.135, 0.0, 2.0, "Reactances"),
       real("Xq_pp", {"Xqpp", "xq_pp", "xqpp"}, "q-axis subtransient reactance", "pu", 0.2, 0.0, 2.0, "Reactances"),
       real("Xl", {"xl"}, "Leakage reactance", "pu", 0.13, 0.0, 1.0, "Reactances"),
       real("Td0_p", {"Td0p", "td0_p", "td0p"}, "d-axis transient time const", "s", 4.3, 0.1, 20.0, "Time constants"),
       real("Tq0_p", {"Tq0p", "tq0_p", "tq0p"}, "q-axis transient time const", "s", 0.85, 0.01, 10.0, "Time constants"),
       real("Td0_pp", {"Td0pp", "td0_pp", "td0pp"}, "d-axis subtransient time const", "s", 0.032, 0.001, 2.0, "Time constants"),
       real("Tq0_pp", {"Tq0pp", "tq0_pp", "tq0pp"}, "q-axis subtransient time const", "s", 0.05, 0.001, 1.0, "Time constants")}));
  c.push_back(model(
      "SimpleAFMachine", "PowerSystems", "Simple Anderson-Fouad machine", "machine",
      {real("H", {"h"}, "Inertia constant", "s", 3.01, 0.1, 30.0, "Rotor"),
       real("D", {"damping_d", "damping"}, "Damping", "pu", 0.0, 0.0, 10.0, "Rotor"),
       real("R", {"Ra", "r"}, "Stator resistance", "pu", 0.0, 0.0, 1.0, "Reactances", true),
       real("Xd", {"xd"}, "d-axis synchronous reactance", "pu", 0.8979, 0.1, 3.0, "Reactances"),
       real("Xq", {"xq"}, "q-axis synchronous reactance", "pu", 0.646, 0.1, 3.0, "Reactances"),
       real("Xd_p", {"Xdp", "xd_p", "xdp"}, "d-axis transient reactance", "pu", 0.2995, 0.0, 2.0, "Reactances"),
       real("Xq_p", {"Xqp", "xq_p", "xqp"}, "q-axis transient reactance", "pu", 0.646, 0.0, 2.0, "Reactances"),
       real("Xd_pp", {"Xdpp", "xd_pp", "xdpp"}, "d-axis subtransient reactance", "pu", 0.23, 0.0, 2.0, "Reactances"),
       real("Xq_pp", {"Xqpp", "xq_pp", "xqpp"}, "q-axis subtransient reactance", "pu", 0.4, 0.0, 2.0, "Reactances"),
       real("Td0_p", {"Td0p", "td0_p", "td0p"}, "d-axis transient time const", "s", 3.0, 0.1, 20.0, "Time constants"),
       real("Tq0_p", {"Tq0p", "tq0_p", "tq0p"}, "q-axis transient time const", "s", 0.1, 0.001, 10.0, "Time constants"),
       real("Td0_pp", {"Td0pp", "td0_pp", "td0pp"}, "d-axis subtransient time const", "s", 0.01, 0.001, 2.0, "Time constants"),
       real("Tq0_pp", {"Tq0pp", "tq0_pp", "tq0pp"}, "q-axis subtransient time const", "s", 0.033, 0.001, 1.0, "Time constants")}));
  c.push_back(model(
      "AndersonFouadMachine", "PowerSystems", "Anderson-Fouad machine", "machine",
      {real("H", {"h"}, "Inertia constant", "s", 3.01, 0.1, 30.0, "Rotor"),
       real("D", {"damping_d", "damping"}, "Damping", "pu", 0.0, 0.0, 10.0, "Rotor"),
       real("R", {"Ra", "r"}, "Stator resistance", "pu", 0.0, 0.0, 1.0, "Reactances", true),
       real("Xd", {"xd"}, "d-axis synchronous reactance", "pu", 0.8979, 0.1, 3.0, "Reactances"),
       real("Xq", {"xq"}, "q-axis synchronous reactance", "pu", 0.646, 0.1, 3.0, "Reactances"),
       real("Xd_p", {"Xdp", "xd_p", "xdp"}, "d-axis transient reactance", "pu", 0.2995, 0.0, 2.0, "Reactances"),
       real("Xq_p", {"Xqp", "xq_p", "xqp"}, "q-axis transient reactance", "pu", 0.646, 0.0, 2.0, "Reactances"),
       real("Xd_pp", {"Xdpp", "xd_pp", "xdpp"}, "d-axis subtransient reactance", "pu", 0.23, 0.0, 2.0, "Reactances"),
       real("Xq_pp", {"Xqpp", "xq_pp", "xqpp"}, "q-axis subtransient reactance", "pu", 0.4, 0.0, 2.0, "Reactances"),
       real("Td0_p", {"Td0p", "td0_p", "td0p"}, "d-axis transient time const", "s", 3.0, 0.1, 20.0, "Time constants"),
       real("Tq0_p", {"Tq0p", "tq0_p", "tq0p"}, "q-axis transient time const", "s", 0.1, 0.001, 10.0, "Time constants"),
       real("Td0_pp", {"Td0pp", "td0_pp", "td0pp"}, "d-axis subtransient time const", "s", 0.01, 0.001, 2.0, "Time constants"),
       real("Tq0_pp", {"Tq0pp", "tq0_pp", "tq0pp"}, "q-axis subtransient time const", "s", 0.033, 0.001, 1.0, "Time constants")}));
  c.push_back(model(
      "ClassicalMachine", "IEEE", "Classical (E' behind X') machine", "machine", {}));
  c.push_back(model(
      "SingleMass", "PowerSimulationsDynamics", "Single-mass shaft", "shaft", {}));
  c.push_back(model(
      "FiveMassShaft", "PowerSimulationsDynamics", "Five-mass turbine-generator shaft", "shaft",
      {real("H1", {"H_hp", "h_hp"}, "HP turbine inertia", "s", 0.30, 0.01, 30.0, "Inertia"),
       real("H2", {"H_ip", "h_ip"}, "IP turbine inertia", "s", 0.30, 0.01, 30.0, "Inertia"),
       real("H3", {"H_lpa", "h_lpa"}, "LPA turbine inertia", "s", 0.30, 0.01, 30.0, "Inertia"),
       real("H4", {"H_lpb", "h_lpb"}, "LPB turbine inertia", "s", 0.30, 0.01, 30.0, "Inertia"),
       real("H5", {"H_gen", "h_gen"}, "Generator inertia", "s", 1.80, 0.01, 30.0, "Inertia"),
       real("K12", {"K_hp_ip", "k12"}, "HP-IP shaft stiffness", "pu/rad", 25.0, 0.001, 1e4, "Shaft"),
       real("K23", {"K_ip_lpa", "k23"}, "IP-LPA shaft stiffness", "pu/rad", 25.0, 0.001, 1e4, "Shaft"),
       real("K34", {"K_lpa_lpb", "k34"}, "LPA-LPB shaft stiffness", "pu/rad", 25.0, 0.001, 1e4, "Shaft"),
       real("K45", {"K_lpb_gen", "k45"}, "LPB-generator shaft stiffness", "pu/rad", 25.0, 0.001, 1e4, "Shaft"),
       real("D12", {"D_hp_ip", "d12"}, "HP-IP damping", "pu", 0.02, 0.0, 100.0, "Shaft", true),
       real("D23", {"D_ip_lpa", "d23"}, "IP-LPA damping", "pu", 0.02, 0.0, 100.0, "Shaft", true),
       real("D34", {"D_lpa_lpb", "d34"}, "LPA-LPB damping", "pu", 0.02, 0.0, 100.0, "Shaft", true),
       real("D45", {"D_lpb_gen", "d45"}, "LPB-generator damping", "pu", 0.02, 0.0, 100.0, "Shaft", true)}));

  // ── Governors ──
  c.push_back(model(
      "TGOV1", "IEEE", "TGOV1 steam governor", "governor",
      {real("R", {"droop_r", "droop"}, "Droop", "pu", 0.05, 0.0, 0.2, "Droop"),
       real("T1", {"Tg", "valve_t_s"}, "Governor/valve time const", "s", 0.5, 0.0, 10.0, "Time constants"),
       real("T2", {"t2"}, "Lead-lag numerator", "s", 0.0, 0.0, 20.0, "Time constants"),
       real("T3", {"Tt", "turbine_t_s"}, "Turbine time const", "s", 0.5, 0.0, 20.0, "Time constants"),
       real("D_T", {"Dt", "D_t", "d_t"}, "Turbine damping", "pu", 0.0, 0.0, 10.0, "Damping"),
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
  c.push_back(model(
      "TGTypeI", "PowerSystems", "Type I turbine governor", "governor",
      {real("R", {"droop_r", "droop"}, "Droop", "pu", 0.05, 0.0, 0.2, "Droop"),
       real("Ts", {"T_s", "T1"}, "Governor time const", "s", 0.2, 0.001, 20.0, "Time constants"),
       real("Tc", {"T_c", "servo_t_s"}, "Servo time const", "s", 0.5, 0.001, 20.0, "Time constants"),
       real("T3", {"t3"}, "Transient gain time const", "s", 0.1, 0.0, 20.0, "Time constants"),
       real("T4", {"t4"}, "Power fraction time const", "s", 0.3, 0.0, 20.0, "Time constants"),
       real("T5", {"t5"}, "Reheat time const", "s", 5.0, 0.001, 40.0, "Time constants"),
       real("Vmax", {"pmax_pu", "tau_max"}, "Max valve/power", "pu", 0.0, 0.0, 2.0, "Limits", true),
       real("Vmin", {"pmin_pu", "tau_min"}, "Min valve/power", "pu", 0.0, -2.0, 0.0, "Limits", true)}));
  c.push_back(model(
      "TGTypeII", "PowerSystems", "Type II turbine governor", "governor",
      {real("R", {"droop_r", "droop"}, "Droop", "pu", 0.05, 0.0, 0.2, "Droop"),
       real("T1", {"t1"}, "Lead-lag numerator", "s", 0.3, 0.0, 20.0, "Time constants"),
       real("T2", {"t2"}, "Lead-lag denominator", "s", 0.5, 0.001, 20.0, "Time constants"),
       real("Vmax", {"pmax_pu", "tau_max"}, "Max governor output", "pu", 0.0, 0.0, 2.0, "Limits", true),
       real("Vmin", {"pmin_pu", "tau_min"}, "Min governor output", "pu", 0.0, -2.0, 0.0, "Limits", true)}));
  for (const auto& entry : {
           std::pair{"GAST", "GAST gas turbine governor"},
           std::pair{"HYGOV", "HYGOV hydro turbine governor"},
           std::pair{"DEGOV", "DEGOV diesel governor"},
           std::pair{"DEGOV1", "DEGOV1 diesel governor"},
           std::pair{"PIDGOV", "PID hydro governor"},
           std::pair{"WPIDHY", "WPIDHY hydro governor"},
           std::pair{"TGSimple", "Simple turbine governor"},
       }) {
    c.push_back(model(
        entry.first, "PowerSystems", entry.second, "governor",
        {real("R", {"droop_r", "droop", "Rperm", "reg"}, "Droop / regulation", "pu", 0.05, 0.0, 1.0, "Droop"),
         real("T1", {"t1", "Tg", "T_reg"}, "Primary governor time const", "s", 0.5, 0.001, 40.0, "Time constants"),
         real("Ta", {"ta"}, "Actuator numerator / derivative time const", "s", 0.10, 0.0, 40.0, "Time constants"),
         real("Tb", {"tb"}, "Actuator denominator time const", "s", 0.30, 0.001, 40.0, "Time constants"),
         real("Tf", {"fuel_t_s", "T_fuel"}, "Fuel/actuator lag", "s", 0.40, 0.001, 40.0, "Time constants"),
         real("Tw", {"water_t_s"}, "Water inertia / hydro lag", "s", 1.0, 0.001, 40.0, "Time constants"),
         real("Ki", {"ki"}, "Integral gain", "", 1.0, 0.0, 100.0, "PID", true),
         real("Kd", {"kd"}, "Derivative gain", "", 0.0, 0.0, 100.0, "PID", true),
         real("Gmax", {"gate_max_pu"}, "Gate upper limit", "pu", 1.0, 0.0, 2.0, "Limits", true),
         real("Gmin", {"gate_min_pu"}, "Gate lower limit", "pu", 0.0, -1.0, 1.0, "Limits", true),
         real("Vmax", {"pmax_pu", "Pmax"}, "Max mechanical power", "pu", 0.0, 0.0, 2.0, "Limits", true),
         real("Vmin", {"pmin_pu", "Pmin"}, "Min mechanical power", "pu", 0.0, -2.0, 0.0, "Limits", true)}));
  }

  // ── Exciters / AVRs ──
  c.push_back(model(
      "AVRSimple", "PowerSystems", "Simple integrator AVR", "exciter",
      {real("Kv", {"kv"}, "Voltage-error integrator gain", "1/s", 20.0, 0.0, 500.0, "Gain"),
       real("V_ref", {"Vref", "v_ref_pu"}, "Voltage setpoint", "pu", 1.0, 0.5, 1.5, "Setpoint", true)}));
  c.push_back(model(
      "SEXS", "IEEE4215", "SEXS simplified excitation", "exciter",
      {real("Ka", {"K", "ka"}, "Regulator gain", "pu", 20.0, 1.0, 400.0, "Gain"),
       real("Ta_Tb", {"TaTb", "Ta_over_Tb", "TaOverTb", "ta_tb"}, "Lead-lag numerator/denominator ratio", "pu", 1.0, 0.0, 10.0, "Time constants"),
       real("Tb", {"tb"}, "Lead-lag denominator time constant", "s", 0.05, 0.001, 20.0, "Time constants"),
       real("Te", {"te"}, "Field-voltage time constant", "s", 0.4, 0.001, 20.0, "Time constants"),
       real("Ta", {"ta"}, "Legacy compact regulator time const", "s", 0.05, 0.0, 5.0, "Time constants"),
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
  c.push_back(model(
      "AVRTypeI", "PowerSystems", "Type I AVR", "exciter",
      {real("Ka", {"ka"}, "Amplifier gain", "pu", 20.0, 0.0, 500.0, "Gain"),
       real("Ke", {"ke"}, "Exciter gain", "pu", 1.0, 0.0, 20.0, "Gain"),
       real("Kf", {"kf"}, "Stabilizing feedback gain", "pu", 0.0, 0.0, 20.0, "Gain"),
       real("Ta", {"ta"}, "Amplifier time const", "s", 0.05, 0.001, 10.0, "Time constants"),
       real("Te", {"te"}, "Exciter time const", "s", 0.4, 0.001, 20.0, "Time constants"),
       real("Tf", {"tf"}, "Feedback time const", "s", 1.0, 0.001, 20.0, "Time constants"),
       real("Tr", {"tr"}, "Measurement time const", "s", 0.01, 0.001, 10.0, "Time constants"),
       real("Ae", {"ae"}, "Saturation coeff A", "", 0.0, 0.0, 20.0, "Saturation", true),
       real("Be", {"be"}, "Saturation coeff B", "", 0.0, 0.0, 20.0, "Saturation", true),
       real("V_ref", {"Vref", "v_ref_pu"}, "Voltage setpoint", "pu", 1.0, 0.5, 1.5, "Setpoint", true)}));
  c.push_back(model(
      "AVRTypeII", "PowerSystems", "Type II AVR", "exciter",
      {real("K0", {"k0"}, "Regulator gain", "pu", 20.0, 0.0, 500.0, "Gain"),
       real("T1", {"t1"}, "Lead-lag 1 denominator", "s", 0.05, 0.001, 20.0, "Time constants"),
       real("T2", {"t2"}, "Lead-lag 1 numerator", "s", 0.01, 0.0, 20.0, "Time constants"),
       real("T3", {"t3"}, "Lead-lag 2 denominator", "s", 0.05, 0.001, 20.0, "Time constants"),
       real("T4", {"t4"}, "Lead-lag 2 numerator", "s", 0.01, 0.0, 20.0, "Time constants"),
       real("Te", {"te"}, "Exciter time const", "s", 0.4, 0.001, 20.0, "Time constants"),
       real("Tr", {"tr"}, "Measurement time const", "s", 0.01, 0.001, 10.0, "Time constants"),
       real("Va_max", {"Vrmax", "Emax"}, "Max regulator output", "pu", 5.0, 0.0, 20.0, "Limits"),
       real("Va_min", {"Vrmin", "Emin"}, "Min regulator output", "pu", -5.0, -20.0, 0.0, "Limits"),
       real("Ae", {"ae"}, "Saturation coeff A", "", 0.0, 0.0, 20.0, "Saturation", true),
       real("Be", {"be"}, "Saturation coeff B", "", 0.0, 0.0, 20.0, "Saturation", true),
       real("V_ref", {"Vref", "v_ref_pu"}, "Voltage setpoint", "pu", 1.0, 0.5, 1.5, "Setpoint", true)}));
  for (const auto& entry : {
           std::pair{"ESAC1A", "IEEE AC1A excitation system"},
           std::pair{"EXAC1", "PSS/E EXAC1 excitation system"},
           std::pair{"EXAC1A", "PSS/E EXAC1A excitation system"},
           std::pair{"EXST1", "PSS/E EXST1 static excitation system"},
           std::pair{"SCRX", "SCRX solid-state excitation system"},
           std::pair{"ESST1A", "IEEE ST1A excitation system"},
           std::pair{"ST6B", "IEEE ST6B excitation system"},
           std::pair{"ST8C", "IEEE ST8C excitation system"},
       }) {
    c.push_back(model(
        entry.first, "IEEE4215", entry.second, "exciter",
        {real("Ka", {"K", "ka"}, "Regulator gain", "pu", 20.0, 0.0, 500.0, "Gain"),
         real("Ta", {"ta"}, "Regulator denominator time const", "s", 0.05, 0.001, 20.0, "Time constants"),
         real("Tb", {"tb"}, "Lead-lag denominator", "s", 0.05, 0.001, 20.0, "Time constants"),
         real("Tc", {"tc"}, "Lead-lag numerator", "s", 0.05, 0.0, 20.0, "Time constants"),
         real("Te", {"te"}, "Exciter time const", "s", 0.40, 0.001, 20.0, "Time constants"),
         real("Tr", {"tr"}, "Voltage transducer time const", "s", 0.01, 0.001, 20.0, "Time constants"),
         real("Kf", {"kf"}, "Stabilizing feedback gain", "pu", 0.0, 0.0, 100.0, "Feedback", true),
         real("Tf", {"tf"}, "Stabilizing feedback time const", "s", 1.0, 0.001, 40.0, "Feedback", true),
         real("Kc", {"kc"}, "Rectifier/loading coefficient", "", 0.0, 0.0, 100.0, "Compounding", true),
         real("Kd", {"kd"}, "Demagnetizing coefficient", "", 0.0, 0.0, 100.0, "Compounding", true),
         real("Kg", {"kg"}, "Terminal-voltage compounding gain", "", 1.0, 0.0, 100.0, "Compounding", true),
         real("Emax", {"Vrmax", "efd_max_pu", "Va_max"}, "Max field voltage", "pu", 5.0, 0.0, 20.0, "Limits"),
         real("Emin", {"Vrmin", "efd_min_pu", "Va_min"}, "Min field voltage", "pu", 0.0, -20.0, 5.0, "Limits"),
         real("V_ref", {"Vref", "v_ref_pu"}, "Voltage setpoint", "pu", 1.0, 0.5, 1.5, "Setpoint", true)}));
  }

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
  c.push_back(model(
      "IEEEST", "IEEE", "IEEEST stabilizer", "pss",
      {real("A1", {"a1"}, "Second-order filter A1", "", 0.0, 0.0, 20.0, "Filter"),
       real("A2", {"a2"}, "Second-order filter A2", "", 1.0, 0.001, 20.0, "Filter"),
       real("A3", {"a3"}, "Input filter A3", "", 1.0, 0.0, 20.0, "Filter"),
       real("A4", {"a4"}, "Input filter A4", "", 1.0, 0.001, 20.0, "Filter"),
       real("A5", {"a5"}, "Second-order numerator A5", "", 1.0, 0.0, 20.0, "Filter"),
       real("A6", {"a6"}, "Second-order numerator A6", "", 0.0, 0.0, 20.0, "Filter"),
       real("T1", {"t1"}, "Lead-lag 1 numerator", "s", 0.15, 0.0, 20.0, "Lead-lag"),
       real("T2", {"t2"}, "Lead-lag 1 denominator", "s", 0.03, 0.001, 20.0, "Lead-lag"),
       real("T3", {"t3"}, "Lead-lag 2 numerator", "s", 0.15, 0.0, 20.0, "Lead-lag"),
       real("T4", {"t4"}, "Lead-lag 2 denominator", "s", 0.03, 0.001, 20.0, "Lead-lag"),
       real("T5", {"t5"}, "Washout numerator", "s", 0.10, 0.0, 20.0, "Washout"),
       real("T6", {"t6"}, "Washout denominator", "s", 0.05, 0.001, 20.0, "Washout"),
       real("Ks", {"ks"}, "Stabilizer gain", "pu", 5.0, 0.0, 100.0, "Gain"),
       real("Lsmax", {"Vsmax", "vs_max_pu"}, "Max output", "pu", 0.10, 0.0, 0.5, "Limits"),
       real("Lsmin", {"Vsmin", "vs_min_pu"}, "Min output", "pu", -0.10, -0.5, 0.0, "Limits"),
       real("Vcu", {"vcu"}, "Compensated voltage upper cutout", "pu", 0.0, 0.0, 2.0, "Limits", true),
       real("Vcl", {"vcl"}, "Compensated voltage lower cutout", "pu", 0.0, 0.0, 2.0, "Limits", true)}));
  c.push_back(model(
      "STAB1", "IEEE", "STAB1 speed-sensitive stabilizer", "pss",
      {real("KT", {"Kt", "kt"}, "K/T washout gain", "1/s", 5.0, 0.0, 100.0, "Gain"),
       real("T", {"Tw", "tw"}, "Washout time const", "s", 10.0, 0.01, 40.0, "Washout"),
       real("T1T3", {"t1_over_t3"}, "Lead-lag 1 ratio", "", 1.0, 0.0, 20.0, "Lead-lag"),
       real("T3", {"t3"}, "Lead-lag 1 denominator", "s", 0.15, 0.01, 20.0, "Lead-lag"),
       real("T2T4", {"t2_over_t4"}, "Lead-lag 2 ratio", "", 1.0, 0.0, 20.0, "Lead-lag"),
       real("T4", {"t4"}, "Lead-lag 2 denominator", "s", 0.03, 0.01, 20.0, "Lead-lag"),
       real("H_lim", {"Hlim", "h_lim"}, "Output limit", "pu", 0.10, 0.0, 0.5, "Limits")}));
  for (const auto& entry : {
           std::pair{"PSS2A", "IEEE PSS2A dual-input stabilizer"},
           std::pair{"PSS2B", "IEEE PSS2B dual-input stabilizer"},
           std::pair{"PSS2C", "IEEE PSS2C dual-input stabilizer"},
       }) {
    c.push_back(model(
        entry.first, "IEEE4215", entry.second, "pss",
        {real("Ks1", {"ks1"}, "Stabilizer output gain", "pu", 10.0, 0.0, 200.0, "Gain"),
         real("Ks2", {"ks2"}, "First input gain", "pu", 1.0, -100.0, 100.0, "Gain"),
         real("Ks3", {"ks3"}, "Second input gain", "pu", 1.0, -100.0, 100.0, "Gain"),
         real("M", {"M_rtf", "m_rtf"}, "Ramp-tracking numerator", "", 5.0, 0.0, 100.0, "Ramp tracking"),
         real("N", {"N_rtf", "n_rtf"}, "Ramp-tracking denominator", "", 1.0, 0.001, 100.0, "Ramp tracking"),
         real("Tw1", {"tw1"}, "Washout 1", "s", 2.0, 0.001, 40.0, "Washout"),
         real("Tw2", {"tw2"}, "Washout 2", "s", 2.0, 0.001, 40.0, "Washout"),
         real("Tw3", {"tw3"}, "Washout 3", "s", 2.0, 0.001, 40.0, "Washout"),
         real("T6", {"t6"}, "Signal filter", "s", 0.05, 0.001, 40.0, "Filters"),
         real("T7", {"t7"}, "Signal filter", "s", 2.0, 0.001, 40.0, "Filters"),
         real("T8", {"t8"}, "Signal filter", "s", 0.20, 0.001, 40.0, "Filters"),
         real("T9", {"t9"}, "Signal filter", "s", 0.10, 0.001, 40.0, "Filters"),
         real("T1", {"t1"}, "Lead-lag numerator 1", "s", 0.15, 0.0, 20.0, "Lead-lag"),
         real("T2", {"t2"}, "Lead-lag denominator 1", "s", 0.03, 0.001, 20.0, "Lead-lag"),
         real("T3", {"t3"}, "Lead-lag numerator 2", "s", 0.15, 0.0, 20.0, "Lead-lag"),
         real("T4", {"t4"}, "Lead-lag denominator 2", "s", 0.03, 0.001, 20.0, "Lead-lag"),
         real("Vstmax", {"Vsmax", "vs_max_pu"}, "Max stabilizer output", "pu", 0.10, 0.0, 1.0, "Limits"),
         real("Vstmin", {"Vsmin", "vs_min_pu"}, "Min stabilizer output", "pu", -0.10, -1.0, 0.0, "Limits")}));
  }

  // ── Dynamic branch models ──
  c.push_back(model(
      "DynamicRLLine", "PowerSimulationsDynamics", "Dynamic RL branch current", "branch",
      {real("R", {"r_pu"}, "Series resistance", "pu", 0.01, 0.0, 2.0, "Series"),
       real("X", {"x_pu"}, "Series reactance", "pu", 0.05, 0.001, 5.0, "Series")}));

  // ── Grid-following inverter (also used for PV / renewable / static gen) ──
  c.push_back(model(
      "REGC_REEC_GFL_Subset", "NERC", "Grid-following converter (REGC/REEC subset)", "gfl",
      {real("pll_kp", {"kp_pll"}, "PLL proportional gain", "", 0.01, 0.0, 5.0, "PLL"),
       real("pll_ki", {"ki_pll"}, "PLL integral gain", "", 1.0, 0.0, 100.0, "PLL"),
       real("pll_lpf_t_s", {}, "PLL low-pass time const", "s", 0.005, 0.0, 0.5, "PLL"),
       real("response_t_s", {"Tg", "Trv"}, "Current response time const", "s", 0.02, 0.0, 1.0, "Response"),
       real("power_filter_t_s", {"Tp", "Tpf"}, "Power filter time const", "s", 0.02, 0.0, 1.0, "Response"),
       real("current_limit_pu", {"Imax", "imax_pu"}, "Current limit (0=off)", "pu", 0.0, 0.0, 3.0, "Limits"),
       real("filter_c_pu", {"Cf"}, "LCL filter capacitance / voltage-state time", "pu", 0.0, 0.0, 10.0, "Filter", true),
       real("filter_grid_r_pu", {"Rg", "Rgrid"}, "LCL grid-side resistance", "pu", 0.0, 0.0, 1.0, "Filter", true),
       real("filter_grid_x_pu", {"Xg", "Xgrid"}, "LCL grid-side reactance", "pu", 0.05, 0.0, 2.0, "Filter", true),
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
       real("current_limit_pu", {"Imax", "imax_pu"}, "Current limit (0=off)", "pu", 0.0, 0.0, 3.0, "Limits"),
       real("filter_c_pu", {"Cf"}, "LCL filter capacitance / voltage-state time", "pu", 0.0, 0.0, 10.0, "Filter", true),
       real("filter_grid_r_pu", {"Rg", "Rgrid"}, "LCL grid-side resistance", "pu", 0.0, 0.0, 1.0, "Filter", true),
       real("filter_grid_x_pu", {"Xg", "Xgrid"}, "LCL grid-side reactance", "pu", 0.05, 0.0, 2.0, "Filter", true)}));
  c.push_back(model(
      "VSMGridForming", "PowerSimulationsDynamics", "Virtual synchronous machine grid-forming converter", "gfm",
      {real("virtual_x_pu", {"Xv", "xv"}, "Virtual reactance", "pu", 0.10, 0.0, 1.0, "Virtual impedance"),
       real("virtual_r_pu", {"Rv", "rv"}, "Virtual resistance", "pu", 0.0, 0.0, 1.0, "Virtual impedance"),
       real("Ta", {"vsm_ta_s"}, "Virtual inertia time constant", "s", 2.0, 0.001, 20.0, "VSM"),
       real("kd", {"vsm_damping_kd"}, "VSM damping gain", "", 400.0, 0.0, 1000.0, "VSM"),
       real("komega", {"kw", "vsm_frequency_droop_kw"}, "VSM frequency droop gain", "", 20.0, 0.0, 200.0, "VSM"),
       real("q_droop_pu", {"kq", "Dq"}, "Q-V droop", "pu", 0.20, 0.0, 10.0, "Droop"),
       real("reactive_power_filter_t_s", {"Tq"}, "Reactive-power filter time const", "s", 0.001, 0.0, 1.0, "Response"),
       real("voltage_control_t_s", {"Tv"}, "Voltage control time const", "s", 0.02, 0.0, 1.0, "Response", true),
       real("current_limit_pu", {"Imax", "imax_pu"}, "Current limit (0=off)", "pu", 0.0, 0.0, 3.0, "Limits"),
       real("filter_c_pu", {"Cf"}, "LCL filter capacitance / voltage-state time", "pu", 0.0, 0.0, 10.0, "Filter", true),
       real("filter_grid_r_pu", {"Rg", "Rgrid"}, "LCL grid-side resistance", "pu", 0.0, 0.0, 1.0, "Filter", true),
       real("filter_grid_x_pu", {"Xg", "Xgrid"}, "LCL grid-side reactance", "pu", 0.05, 0.0, 2.0, "Filter", true)}));
  c.push_back(model(
      "VOCGridForming", "PowerSimulationsDynamics", "Virtual oscillator grid-forming converter", "gfm",
      {real("virtual_x_pu", {"Xv", "xv"}, "Virtual reactance", "pu", 0.10, 0.0, 1.0, "Virtual impedance"),
       real("virtual_r_pu", {"Rv", "rv"}, "Virtual resistance", "pu", 0.0, 0.0, 1.0, "Virtual impedance"),
       real("k1", {"voc_k1"}, "Active oscillator gain", "", 0.0033, 0.0, 1.0, "VOC"),
       real("psi", {"voc_psi_rad"}, "VOC phase angle", "rad", 0.7853981633974483, -3.1416, 3.1416, "VOC"),
       real("k2", {"voc_k2"}, "Reactive oscillator gain", "", 0.0796, 0.0, 10.0, "VOC"),
       real("power_filter_t_s", {"Tp"}, "Telemetry active-power filter time const", "s", 0.05, 0.0, 1.0, "Response", true),
       real("reactive_power_filter_t_s", {"Tq"}, "Telemetry reactive-power filter time const", "s", 0.001, 0.0, 1.0, "Response", true),
       real("current_limit_pu", {"Imax", "imax_pu"}, "Current limit (0=off)", "pu", 0.0, 0.0, 3.0, "Limits"),
       real("filter_c_pu", {"Cf"}, "LCL filter capacitance / voltage-state time", "pu", 0.0, 0.0, 10.0, "Filter", true),
       real("filter_grid_r_pu", {"Rg", "Rgrid"}, "LCL grid-side resistance", "pu", 0.0, 0.0, 1.0, "Filter", true),
       real("filter_grid_x_pu", {"Xg", "Xgrid"}, "LCL grid-side reactance", "pu", 0.05, 0.0, 2.0, "Filter", true)}));

  // ── Phase-3 inverter composition blocks ──
  c.push_back(model("AverageConverter", "PSID", "Average-value converter bridge",
                    "converter", {}));
  c.push_back(model("ConstantDCSource", "PSID", "Constant DC source",
                    "dc_source", {}));
  c.push_back(model(
      "DynamicDCLink", "HACDCPF", "Dynamic DC-link source", "dc_source",
      {real("dc_link_capacitance_s", {"Cdc", "Tdc"}, "DC-link capacitance",
            "s", 0.10, 0.0, 10.0, "DC link"),
       real("vdc_ref_pu", {"Vdc_ref"}, "DC voltage reference", "pu", 1.0,
            0.1, 2.5, "DC link")}));
  c.push_back(model(
      "RLFilter", "PSID", "RL output filter", "filter",
      {real("virtual_r_pu", {"Rf", "Rv", "rv"}, "Filter resistance", "pu",
            0.0, 0.0, 1.0, "Filter"),
       real("virtual_x_pu", {"Xf", "Xv", "xv"}, "Filter reactance", "pu",
            0.10, 0.0, 1.0, "Filter")}));
  c.push_back(model(
      "LCLFilter", "PSID", "LCL output filter", "filter",
      {real("virtual_r_pu", {"Rf", "Rv", "rv"}, "Series resistance", "pu",
            0.0, 0.0, 1.0, "Filter"),
       real("virtual_x_pu", {"Xf", "Xv", "xv"}, "Series reactance", "pu",
            0.10, 0.0, 1.0, "Filter"),
       real("Cf", {"filter_c_pu"}, "Filter capacitance", "pu", 0.0, 0.0,
            10.0, "Filter", true),
       real("filter_grid_r_pu", {"Rg", "Rgrid"}, "Grid-side resistance", "pu",
            0.0, 0.0, 1.0, "Filter", true),
       real("filter_grid_x_pu", {"Xg", "Xgrid"}, "Grid-side reactance", "pu",
            0.05, 0.0, 2.0, "Filter", true)}));
  c.push_back(model(
      "GFLPQOuterControl", "NERC", "Grid-following P/Q outer control",
      "outer_control",
      {real("power_filter_t_s", {"Tp", "Tpf"}, "Power filter time const",
            "s", 0.02, 0.0, 1.0, "Response"),
       real("frequency_watt_droop_pu", {"Ddn", "kf"}, "Frequency-watt droop",
            "pu", 0.0, 0.0, 50.0, "Droop", true),
       real("volt_var_droop_pu", {"Dvv", "kq"}, "Volt-var droop", "pu",
            0.0, 0.0, 50.0, "Droop", true)}));
  c.push_back(model(
      "GFMDroopOuterControl", "NERC", "Grid-forming droop outer control",
      "outer_control",
      {real("p_droop_pu", {"mp", "Dp"}, "P-f droop", "pu", 0.01, 0.0,
            1.0, "Droop"),
       real("q_droop_pu", {"mq", "Dq"}, "Q-V droop", "pu", 0.05, 0.0,
            1.0, "Droop"),
       real("power_filter_t_s", {"Tf", "Tpf"}, "Power filter time const",
            "s", 0.05, 0.0, 1.0, "Response")}));
  c.push_back(model(
      "VSMOuterControl", "PowerSimulationsDynamics", "Virtual inertia outer control",
      "outer_control",
      {real("Ta", {"vsm_ta_s"}, "Virtual inertia time constant", "s", 2.0, 0.001, 20.0, "VSM"),
       real("kd", {"vsm_damping_kd"}, "VSM damping gain", "", 400.0, 0.0, 1000.0, "VSM"),
       real("komega", {"kw", "vsm_frequency_droop_kw"}, "VSM frequency droop gain", "", 20.0, 0.0, 200.0, "VSM"),
       real("q_droop_pu", {"kq", "Dq"}, "Q-V droop", "pu", 0.20, 0.0, 10.0, "Droop"),
       real("reactive_power_filter_t_s", {"Tq"}, "Reactive-power filter time const", "s", 0.001, 0.0, 1.0, "Response")}));
  c.push_back(model(
      "VOCOuterControl", "PowerSimulationsDynamics", "Virtual oscillator outer control",
      "outer_control",
      {real("k1", {"voc_k1"}, "Active oscillator gain", "", 0.0033, 0.0, 1.0, "VOC"),
       real("psi", {"voc_psi_rad"}, "VOC phase angle", "rad", 0.7853981633974483, -3.1416, 3.1416, "VOC"),
       real("k2", {"voc_k2"}, "Reactive oscillator gain", "", 0.0796, 0.0, 10.0, "VOC")}));
  c.push_back(model(
      "PIInnerCurrentControl", "PSID", "PI inner current control",
      "inner_control",
      {real("response_t_s", {"Tg", "Trv"}, "Current response time const",
            "s", 0.02, 0.0, 1.0, "Response"),
       real("current_limit_pu", {"Imax", "imax_pu"}, "Current limit (0=off)",
            "pu", 0.0, 0.0, 3.0, "Limits")}));
  for (const auto& entry : {
           std::pair{"MagnitudeOutputCurrentLimiter", "Magnitude current limiter"},
           std::pair{"InstantaneousOutputCurrentLimiter", "Instantaneous current limiter"},
           std::pair{"SaturationOutputCurrentLimiter", "Component saturation current limiter"},
           std::pair{"PriorityOutputCurrentLimiter", "Reactive-priority current limiter"},
           std::pair{"ActivePriorityCurrentLimiter", "Active-priority current limiter"},
           std::pair{"HybridOutputCurrentLimiter", "Hybrid current limiter"},
       }) {
    c.push_back(model(
        entry.first, "PSID", entry.second, "limiter",
        {real("current_limit_pu", {"Imax", "imax_pu"}, "Current limit", "pu",
              1.2, 0.0, 3.0, "Limits"),
         real("reactive_current_priority", {"iq_priority"}, "Reactive current priority",
              "", 0.0, 0.0, 1.0, "Limits", true)}));
  }
  c.push_back(model(
      "VirtualImpedanceInnerControl", "HACDCPF",
      "Virtual-impedance inner control", "inner_control",
      {real("virtual_r_pu", {"Rv", "rv"}, "Virtual resistance", "pu", 0.0,
            0.0, 1.0, "Virtual impedance"),
       real("virtual_x_pu", {"Xv", "xv"}, "Virtual reactance", "pu", 0.10,
            0.0, 1.0, "Virtual impedance")}));

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
  c.push_back(model(
      "PeriodicVariableSource", "PowerSimulationsDynamics", "Periodic variable source", "source",
      {real("R_th", {"r_th_pu", "R"}, "Thevenin resistance", "pu", 0.0, 0.0, 1.0, "Thevenin"),
       real("X_th", {"x_th_pu", "X"}, "Thevenin reactance", "pu", 0.05, 0.0, 5.0, "Thevenin"),
       real("internal_voltage_bias", {"voltage_bias_pu"}, "Internal voltage bias", "pu", 1.0, 0.0, 2.0, "Voltage"),
       real("internal_voltage_frequency_rad_s", {"omega_v"}, "Voltage harmonic frequency", "rad/s", 6.283185307179586, 0.0, 1000.0, "Voltage"),
       real("internal_voltage_sin_coeff", {"V_sin"}, "Voltage sine coefficient", "pu", 1.0, -2.0, 2.0, "Voltage"),
       real("internal_voltage_cos_coeff", {"V_cos"}, "Voltage cosine coefficient", "pu", 0.0, -2.0, 2.0, "Voltage"),
       real("internal_angle_bias", {"angle_bias_rad"}, "Internal angle bias", "rad", 0.0, -6.2832, 6.2832, "Angle"),
       real("internal_angle_frequency_rad_s", {"omega_theta"}, "Angle harmonic frequency", "rad/s", 6.283185307179586, 0.0, 1000.0, "Angle"),
       real("internal_angle_sin_coeff", {"theta_sin"}, "Angle sine coefficient", "rad", 0.0, -6.2832, 6.2832, "Angle"),
       real("internal_angle_cos_coeff", {"theta_cos"}, "Angle cosine coefficient", "rad", 1.0, -6.2832, 6.2832, "Angle")}));
  c.push_back(model(
      "CSVGN1", "PSS/E", "Static shunt compensator dynamic injector", "dynamic_injection",
      {real("K", {}, "Voltage regulator gain", "", 20.0, 0.0, 1000.0, "Regulator"),
       real("T1", {}, "Lead-lag time constant", "s", 0.0, 0.0, 20.0, "Regulator"),
       real("T2", {}, "Lead-lag time constant", "s", 1.0, 0.0, 20.0, "Regulator"),
       real("T3", {}, "Lead-lag time constant", "s", 0.154833, 0.0, 20.0, "Regulator"),
       real("T4", {}, "Lead-lag time constant", "s", 1.0, 0.0, 20.0, "Regulator"),
       real("T5", {}, "Thyristor time constant", "s", 0.005167, 0.0, 20.0, "Regulator"),
       real("Rmin", {}, "Minimum reactor admittance", "pu", 0.0, 0.0, 10.0, "Limits"),
       real("Vmax", {}, "Regulator max", "pu", 1.0, 0.0, 10.0, "Limits"),
       real("Vmin", {}, "Regulator min", "pu", 0.0, -10.0, 10.0, "Limits"),
       real("CBase", {"Cbase"}, "Capacitor base", "Mvar", 60.0, 0.0, 10000.0, "Base"),
       real("base_power", {"Mbase", "model_base_mva"}, "Model base", "MVA", 500.0, 0.001, 10000.0, "Base")}));
  c.push_back(model(
      "AggregateDistributedGenerationA", "PSS/E", "DERA aggregate distributed generation", "dynamic_injection",
      {real("base_power", {"Mbase", "model_base_mva"}, "Model base", "MVA", 100.0, 0.001, 10000.0, "Base"),
       real("Freq_Flag", {}, "Frequency-control flag", "", 0.0, 0.0, 1.0, "Flags"),
       real("Pf_Flag", {}, "Power-factor control flag", "", 1.0, 0.0, 1.0, "Flags"),
       real("T_rv", {"Trv"}, "Voltage measurement time constant", "s", 0.02, 0.0, 10.0, "Response"),
       real("Trf", {}, "Frequency measurement time constant", "s", 0.02, 0.0, 10.0, "Response"),
       real("K_qv", {"Kqv"}, "Voltage-reactive current gain", "", 5.0, 0.0, 100.0, "Voltage control"),
       real("Tp", {}, "Active-power measurement time constant", "s", 0.02, 0.0, 10.0, "Response"),
       real("T_iq", {"Tiq"}, "Reactive current time constant", "s", 0.02, 0.0, 10.0, "Response"),
       real("Tg", {}, "Current transducer time constant", "s", 0.02, 0.0, 10.0, "Response"),
       real("Tv", {}, "Voltage/frequency multiplier time constant", "s", 0.02, 0.0, 10.0, "Response"),
       real("Tpord", {}, "Power-order time constant", "s", 0.02, 0.0, 10.0, "Frequency control"),
       real("Kpg", {}, "Frequency-control proportional gain", "", 0.1, 0.0, 100.0, "Frequency control"),
       real("Kig", {}, "Frequency-control integral gain", "", 10.0, 0.0, 1000.0, "Frequency control"),
       real("I_max", {"Imax"}, "Current limit", "pu", 1.2, 0.0, 10.0, "Limits"),
       real("Iq_min", {}, "Minimum reactive current", "pu", -1.0, -10.0, 10.0, "Limits"),
       real("Iq_max", {}, "Maximum reactive current", "pu", 1.0, -10.0, 10.0, "Limits"),
       real("Vtrip_L", {"v_trip_low_pu"}, "Voltage trip low threshold", "pu", 0.0, 0.0, 2.0, "Protection", true),
       real("Vtrip_H", {"v_trip_high_pu"}, "Voltage trip high threshold", "pu", 0.0, 0.0, 2.0, "Protection", true),
       real("Ftrip_L", {"f_trip_low_pu"}, "Frequency trip low threshold", "pu", 0.0, 0.0, 2.0, "Protection", true),
       real("Ftrip_H", {"f_trip_high_pu"}, "Frequency trip high threshold", "pu", 0.0, 0.0, 2.0, "Protection", true),
       real("Ttrip", {"trip_delay_s"}, "Trip delay", "s", 0.0, 0.0, 10.0, "Protection", true)}));
  c.push_back(model(
      "SimplifiedSingleCageInductionMachine", "PowerSystems",
      "Simplified single-cage induction machine", "motor",
      {real("P_mech", {"p_mech_mw", "P_load"}, "Mechanical load power", "MW",
            0.0, 0.0, 1e5, "Load"),
       real("Q_load", {"q_nom_mvar"}, "Nominal reactive load", "Mvar",
            0.0, -1e5, 1e5, "Load"),
       real("Rs", {"r_s_pu", "R_stator"}, "Stator resistance", "pu",
            0.02, 0.0, 2.0, "Electrical"),
       real("Xs", {"x_s_pu", "X_stator"}, "Stator reactance", "pu",
            0.10, 0.0, 5.0, "Electrical"),
       real("Rr", {"r_r_pu", "R_rotor"}, "Rotor resistance", "pu",
            0.02, 0.0, 2.0, "Electrical"),
       real("Xr", {"x_r_pu", "X_rotor"}, "Rotor reactance", "pu",
            0.08, 0.0, 5.0, "Electrical"),
       real("Xm", {"x_m_pu", "X_magnetizing"}, "Magnetizing reactance", "pu",
            3.0, 0.0, 20.0, "Electrical"),
       real("H", {"inertia_h"}, "Rotor inertia", "s", 0.5, 0.01, 20.0, "Mechanical"),
       real("D", {"damping_d"}, "Damping", "pu", 0.0, 0.0, 20.0, "Mechanical"),
       real("torque_exponent", {"N_exp"}, "Mechanical torque exponent", "",
            2.0, 0.0, 5.0, "Mechanical", true)}));
  c.push_back(model(
      "SingleCageInductionMachine", "PowerSystems",
      "Single-cage induction machine", "motor",
      {real("P_mech", {"p_mech_mw", "P_load"}, "Mechanical load power", "MW",
            0.0, 0.0, 1e5, "Load"),
       real("Q_load", {"q_nom_mvar"}, "Nominal reactive load", "Mvar",
            0.0, -1e5, 1e5, "Load"),
       real("Rs", {"r_s_pu", "R_stator"}, "Stator resistance", "pu",
            0.02, 0.0, 2.0, "Electrical"),
       real("Xs", {"x_s_pu", "X_stator"}, "Stator reactance", "pu",
            0.10, 0.0, 5.0, "Electrical"),
       real("Rr", {"r_r_pu", "R_rotor"}, "Rotor resistance", "pu",
            0.02, 0.0, 2.0, "Electrical"),
       real("Xr", {"x_r_pu", "X_rotor"}, "Rotor reactance", "pu",
            0.08, 0.0, 5.0, "Electrical"),
       real("Xm", {"x_m_pu", "X_magnetizing"}, "Magnetizing reactance", "pu",
            3.0, 0.0, 20.0, "Electrical"),
       real("H", {"inertia_h"}, "Rotor inertia", "s", 0.5, 0.01, 20.0, "Mechanical"),
       real("D", {"damping_d"}, "Damping", "pu", 0.0, 0.0, 20.0, "Mechanical"),
       real("torque_exponent", {"N_exp"}, "Mechanical torque exponent", "",
            2.0, 0.0, 5.0, "Mechanical", true)}));
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
	     {"GENROU", "GENROE", "GENSAL", "GENSAE", "OneDOneQMachine",
	      "SimpleMarconatoMachine", "MarconatoMachine", "SimpleAFMachine",
	      "AndersonFouadMachine", "SauerPaiMachine", "ClassicalMachine"},
	     "ClassicalMachine"),
        slot("shaft", "Shaft", true, {"SingleMass", "FiveMassShaft"}, "SingleMass"),
	    slot("governor", "Governor", true,
	         {"TGOV1", "IEEEG1", "TGTypeI", "TGTypeII", "GAST", "HYGOV",
	          "DEGOV", "DEGOV1", "PIDGOV", "WPIDHY", "TGSimple"}, "None"),
        slot("exciter", "Exciter / AVR", true,
             {"SEXS", "IEEET1", "AVRSimple", "AVRTypeI", "AVRTypeII",
              "ESAC1A", "EXAC1", "EXAC1A", "EXST1", "SCRX", "ESST1A",
              "ST6B", "ST8C"}, "None"),
        slot("pss", "Power system stabilizer", true,
             {"PSS1A", "IEEEST", "STAB1", "PSS2A", "PSS2B", "PSS2C"}, "None")}});

  comps.push_back(
      {"vsc", "VSC converter", "AC",
       {slot("converter", "Converter control", false,
             {"REGC_REEC_GFL_Subset", "GridFormingNortonDroop",
              "VSMGridForming", "VOCGridForming"}, "REGC_REEC_GFL_Subset"),
        slot("converter_block", "Converter bridge", false,
             {"AverageConverter"}, "AverageConverter"),
        slot("dc_source", "DC source", true,
             {"ConstantDCSource", "DynamicDCLink"}, "ConstantDCSource"),
        slot("filter", "Output filter", false, {"RLFilter", "LCLFilter"}, "RLFilter"),
        slot("limiter", "Current limiter", true,
             {"MagnitudeOutputCurrentLimiter", "InstantaneousOutputCurrentLimiter",
              "SaturationOutputCurrentLimiter", "PriorityOutputCurrentLimiter",
              "ActivePriorityCurrentLimiter", "HybridOutputCurrentLimiter"},
             "MagnitudeOutputCurrentLimiter"),
        slot("pll", "PLL", false, {"ReducedOrderPLL", "KauraPLL", "FixedFrequency"},
             "ReducedOrderPLL", "REGC_REEC_GFL_Subset"),
        slot("outer_control", "Outer control", false,
             {"GFLPQOuterControl", "GFMDroopOuterControl",
              "VSMOuterControl", "VOCOuterControl"}, "GFLPQOuterControl"),
        slot("inner_control", "Inner control", false,
             {"PIInnerCurrentControl", "VirtualImpedanceInnerControl"},
             "PIInnerCurrentControl")}});

  const auto gfl_component = [](std::string canvas, std::string name) {
    return DynamicComponentComposition{
        std::move(canvas), std::move(name), "AC",
        {slot("converter", "Converter control", false, {"REGC_REEC_GFL_Subset"},
              "REGC_REEC_GFL_Subset"),
         slot("converter_block", "Converter bridge", false,
              {"AverageConverter"}, "AverageConverter"),
         slot("dc_source", "DC source", true,
              {"ConstantDCSource", "DynamicDCLink"}, "ConstantDCSource"),
         slot("filter", "Output filter", false, {"RLFilter", "LCLFilter"}, "RLFilter"),
         slot("limiter", "Current limiter", true,
              {"MagnitudeOutputCurrentLimiter", "InstantaneousOutputCurrentLimiter",
               "SaturationOutputCurrentLimiter", "PriorityOutputCurrentLimiter",
               "ActivePriorityCurrentLimiter", "HybridOutputCurrentLimiter"},
              "MagnitudeOutputCurrentLimiter"),
         slot("pll", "PLL", false, {"ReducedOrderPLL", "KauraPLL", "FixedFrequency"},
              "ReducedOrderPLL"),
         slot("outer_control", "Outer control", false,
              {"GFLPQOuterControl"}, "GFLPQOuterControl"),
         slot("inner_control", "Inner control", false,
              {"PIInnerCurrentControl"}, "PIInnerCurrentControl")}};
  };
  comps.push_back(gfl_component("pv", "PV system"));
  comps.push_back(gfl_component("sgen", "Static generator"));
  comps.push_back(gfl_component("renGen", "Renewable generator"));

  comps.push_back(
      {"storage", "Battery storage", "AC",
       {slot("storage", "Storage model", false, {"BatterySOCFirstOrder"}, "BatterySOCFirstOrder"),
        slot("converter", "Grid-forming control", true,
             {"GridFormingNortonDroop", "VSMGridForming", "VOCGridForming"}, "None"),
        slot("converter_block", "Converter bridge", true,
             {"AverageConverter"}, "None"),
        slot("filter", "Output filter", true, {"RLFilter", "LCLFilter"}, "None"),
        slot("limiter", "Current limiter", true,
             {"MagnitudeOutputCurrentLimiter", "InstantaneousOutputCurrentLimiter",
              "SaturationOutputCurrentLimiter", "PriorityOutputCurrentLimiter",
              "ActivePriorityCurrentLimiter", "HybridOutputCurrentLimiter"},
             "None"),
        slot("outer_control", "Outer control", true,
             {"GFMDroopOuterControl", "VSMOuterControl", "VOCOuterControl"}, "None"),
        slot("inner_control", "Inner control", true,
             {"VirtualImpedanceInnerControl"}, "None")}});

  comps.push_back(
      {"dcdcConverter", "DC/DC converter", "DC",
       {slot("converter", "Converter model", false, {"FirstOrderDCDCConverter"},
             "FirstOrderDCDCConverter")}});

  comps.push_back(
      {"load", "Load", "AC",
       {slot("load", "Load model", false,
             {"ConstantPower", "ConstantCurrent", "ConstantImpedance", "ZIP"}, "ZIP")}});

  comps.push_back(
      {"motors", "Asynchronous motor", "AC",
       {slot("motor", "Induction machine", false,
             {"SimplifiedSingleCageInductionMachine", "SingleCageInductionMachine"},
             "SimplifiedSingleCageInductionMachine")}});

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
