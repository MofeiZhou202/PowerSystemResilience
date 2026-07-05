#pragma once

#include <array>
#include <cstddef>
#include <string_view>

#include "hacdcpf/dynamics/DynamicState.hpp"

namespace hacdcpf::dynamics {

enum class GeneratorInnerVar : int {
  ElectricalTorque = 0,
  MechanicalTorque,
  FieldVoltage,
  StabilizerVoltage,
  TerminalVoltageReal,
  TerminalVoltageImag,
  PsiD,
  PsiQ,
  XadIfd,
  Count
};

constexpr int kGeneratorInnerVarCount =
    static_cast<int>(GeneratorInnerVar::Count);

inline std::string_view to_string(GeneratorInnerVar slot) {
  switch (slot) {
    case GeneratorInnerVar::ElectricalTorque: return "tau_e";
    case GeneratorInnerVar::MechanicalTorque: return "tau_m";
    case GeneratorInnerVar::FieldVoltage: return "vf";
    case GeneratorInnerVar::StabilizerVoltage: return "v_pss";
    case GeneratorInnerVar::TerminalVoltageReal: return "vr";
    case GeneratorInnerVar::TerminalVoltageImag: return "vi";
    case GeneratorInnerVar::PsiD: return "psi_d";
    case GeneratorInnerVar::PsiQ: return "psi_q";
    case GeneratorInnerVar::XadIfd: return "xad_ifd";
    case GeneratorInnerVar::Count: break;
  }
  return "unknown";
}

struct GeneratorInnerVariableSnapshot {
  std::array<double, kGeneratorInnerVarCount> values{};
  std::array<bool, kGeneratorInnerVarCount> present{};

  void set(GeneratorInnerVar slot, double value) {
    const int idx = static_cast<int>(slot);
    values[static_cast<std::size_t>(idx)] = value;
    present[static_cast<std::size_t>(idx)] = true;
  }
};

struct GeneratorInnerVariableBus {
  const StateIndexRange* machine_range{nullptr};
  bool valid{false};
  int bus_pos{-1};
  double base_mva{100.0};
  double frequency_hz{50.0};
  double inertia_h{1.0};
  bool has_machine{true};
  bool has_shaft{true};
  bool has_avr{false};
  bool has_turbine_governor{false};
  bool has_pss{false};
  int omega_local{1};
  std::array<int, kGeneratorInnerVarCount> state_local{};

  GeneratorInnerVariableBus() { state_local.fill(-1); }

  [[nodiscard]] int omegaIndex() const {
    return machine_range ? machine_range->offset + omega_local : -1;
  }

  [[nodiscard]] int stateIndex(GeneratorInnerVar slot) const {
    const int local = state_local[static_cast<std::size_t>(static_cast<int>(slot))];
    return machine_range && local >= 0 ? machine_range->offset + local : -1;
  }

  [[nodiscard]] int mechanicalTorqueIndex() const {
    return stateIndex(GeneratorInnerVar::MechanicalTorque);
  }

  [[nodiscard]] int fieldVoltageIndex() const {
    return stateIndex(GeneratorInnerVar::FieldVoltage);
  }

  [[nodiscard]] bool hasState(GeneratorInnerVar slot) const {
    return stateIndex(slot) >= 0;
  }
};

enum class InverterInnerVar : int {
  ModulationD = 0,
  ModulationQ,
  DCVoltage,
  FilterVoltageReal,
  FilterVoltageImag,
  PllOmega,
  PllAngle,
  OuterCurrentD,
  OuterCurrentQ,
  InnerCurrentD,
  InnerCurrentQ,
  ConverterCurrentReal,
  ConverterCurrentImag,
  FilterCurrentReal,
  FilterCurrentImag,
  ConverterVoltageReal,
  ConverterVoltageImag,
  FilteredActivePower,
  FilteredReactivePower,
  VoltageReference,
  FrequencyReference,
  CurrentReferenceD,
  CurrentReferenceQ,
  PllVoltageD,
  PllVoltageQ,
  Count
};

constexpr int kInverterInnerVarCount =
    static_cast<int>(InverterInnerVar::Count);

inline std::string_view to_string(InverterInnerVar slot) {
  switch (slot) {
    case InverterInnerVar::ModulationD: return "md";
    case InverterInnerVar::ModulationQ: return "mq";
    case InverterInnerVar::DCVoltage: return "vdc";
    case InverterInnerVar::FilterVoltageReal: return "vr_filter";
    case InverterInnerVar::FilterVoltageImag: return "vi_filter";
    case InverterInnerVar::PllOmega: return "omega_pll";
    case InverterInnerVar::PllAngle: return "theta_pll";
    case InverterInnerVar::OuterCurrentD: return "id_oc";
    case InverterInnerVar::OuterCurrentQ: return "iq_oc";
    case InverterInnerVar::InnerCurrentD: return "id_ic";
    case InverterInnerVar::InnerCurrentQ: return "iq_ic";
    case InverterInnerVar::ConverterCurrentReal: return "ir_cnv";
    case InverterInnerVar::ConverterCurrentImag: return "ii_cnv";
    case InverterInnerVar::FilterCurrentReal: return "ir_filter";
    case InverterInnerVar::FilterCurrentImag: return "ii_filter";
    case InverterInnerVar::ConverterVoltageReal: return "vr_cnv";
    case InverterInnerVar::ConverterVoltageImag: return "vi_cnv";
    case InverterInnerVar::FilteredActivePower: return "p_filter";
    case InverterInnerVar::FilteredReactivePower: return "q_filter";
    case InverterInnerVar::VoltageReference: return "v_ref";
    case InverterInnerVar::FrequencyReference: return "f_ref";
    case InverterInnerVar::CurrentReferenceD: return "id_ref";
    case InverterInnerVar::CurrentReferenceQ: return "iq_ref";
    case InverterInnerVar::PllVoltageD: return "vd_pll";
    case InverterInnerVar::PllVoltageQ: return "vq_pll";
    case InverterInnerVar::Count: break;
  }
  return "unknown";
}

struct InverterInnerVariableSnapshot {
  std::array<double, kInverterInnerVarCount> values{};
  std::array<bool, kInverterInnerVarCount> present{};

  void set(InverterInnerVar slot, double value) {
    const int idx = static_cast<int>(slot);
    values[static_cast<std::size_t>(idx)] = value;
    present[static_cast<std::size_t>(idx)] = true;
  }
};

struct InverterInnerVariableBus {
  const StateIndexRange* range{nullptr};
  bool valid{false};
  int bus_pos{-1};
  int dc_bus_pos{-1};
  bool grid_following{true};
  bool has_dc_source{false};
  bool has_filter{true};
  bool has_frequency_estimator{true};
  bool has_outer_control{true};
  bool has_inner_control{true};
  bool has_converter{true};
  std::array<int, kInverterInnerVarCount> state_local{};

  InverterInnerVariableBus() { state_local.fill(-1); }

  [[nodiscard]] int stateIndex(InverterInnerVar slot) const {
    const int local = state_local[static_cast<std::size_t>(static_cast<int>(slot))];
    return range && local >= 0 ? range->offset + local : -1;
  }

  [[nodiscard]] bool hasState(InverterInnerVar slot) const {
    return stateIndex(slot) >= 0;
  }
};

}  // namespace hacdcpf::dynamics
