#pragma once

#include "hacdcpf/io/opendss_bridge.hpp"

namespace hacdcpf_compare {

struct PowerPair {
  double p_mw{0.0};
  double q_mvar{0.0};
};

// ActiveCktElement.Powers is reported in kW / kvar.
inline PowerPair opendss_terminal_power_to_mw_mvar(
    const hacdcpf::io::OpenDSSPowerKWKvar& power) {
  return {
      power.p_kw / 1000.0,
      power.q_kvar / 1000.0,
  };
}

inline PowerPair opendss_terminal_power_to_mw_mvar(
    const hacdcpf::io::OpenDSSTerminalPower& power) {
  return opendss_terminal_power_to_mw_mvar(power.power_kw_kvar);
}

template <typename TerminalPowerContainer>
inline PowerPair sum_terminal_power_range_to_mw_mvar(
    const TerminalPowerContainer& terminal_powers) {
  PowerPair total;
  for (const auto& power : terminal_powers) {
    const auto converted = opendss_terminal_power_to_mw_mvar(power);
    total.p_mw += converted.p_mw;
    total.q_mvar += converted.q_mvar;
  }
  return total;
}

inline PowerPair sum_pd_element_terminal_powers_to_mw_mvar(
    const hacdcpf::io::OpenDSSPDElementResult& element) {
  return sum_terminal_power_range_to_mw_mvar(element.terminal_powers);
}

inline PowerPair sum_line_terminal_powers_to_mw_mvar(
    const hacdcpf::io::OpenDSSLineResult& line) {
  return sum_terminal_power_range_to_mw_mvar(line.terminal_powers);
}

inline PowerPair sum_all_pd_element_terminal_powers_to_mw_mvar(
    const hacdcpf::io::OpenDSSSnapshotResult& result) {
  PowerPair total;
  for (const auto& element : result.pd_element_results) {
    const auto element_total = sum_pd_element_terminal_powers_to_mw_mvar(element);
    total.p_mw += element_total.p_mw;
    total.q_mvar += element_total.q_mvar;
  }
  return total;
}

inline PowerPair sum_all_line_terminal_powers_to_mw_mvar(
    const hacdcpf::io::OpenDSSSnapshotResult& result) {
  PowerPair total;
  for (const auto& line : result.line_results) {
    const auto line_total = sum_line_terminal_powers_to_mw_mvar(line);
    total.p_mw += line_total.p_mw;
    total.q_mvar += line_total.q_mvar;
  }
  return total;
}

// Circuit/Element Losses raw arrays are reported in W / var.
inline PowerPair opendss_losses_raw_to_mw_mvar(
    const hacdcpf::io::OpenDSSLossesWVar& losses) {
  return {
      losses.p_w / 1'000'000.0,
      losses.q_var / 1'000'000.0,
  };
}

inline PowerPair opendss_element_losses_to_mw_mvar(
    const hacdcpf::io::OpenDSSPDElementResult& element) {
  return opendss_losses_raw_to_mw_mvar(element.losses_raw);
}

inline PowerPair opendss_line_losses_to_mw_mvar(
    const hacdcpf::io::OpenDSSLineResult& line) {
  return opendss_losses_raw_to_mw_mvar(line.losses_raw);
}

inline PowerPair opendss_circuit_losses_to_mw_mvar(
    const hacdcpf::io::OpenDSSSnapshotResult& result) {
  return opendss_losses_raw_to_mw_mvar(result.circuit_losses_raw);
}

}  // namespace hacdcpf_compare
