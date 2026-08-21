#include <cmath>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

#include <nlohmann/json.hpp>

#include "hacdcpf/analysis/short_circuit.hpp"
#include "hacdcpf/model/system.hpp"

namespace {

using hacdcpf::ACBus;
using hacdcpf::BusType;
using hacdcpf::ExternalGrid;
using hacdcpf::HybridPowerSystem;
using hacdcpf::analysis::FaultType;
using hacdcpf::analysis::SCDetailedOptions;
using nlohmann::json;

const hacdcpf::analysis::SCDetailedBusResult& fault_row(
    const hacdcpf::analysis::SCDetailedResult& result) {
  for (const auto& row : result.bus_results) {
    if (row.bus_id == result.fault_bus_id) return row;
  }
  throw std::runtime_error("fault-bus result row is missing");
}

const char* fault_name(FaultType type) {
  switch (type) {
    case FaultType::ThreePhase: return "three_phase";
    case FaultType::SinglePhaseGround: return "single_phase_ground";
    case FaultType::TwoPhase: return "two_phase";
    case FaultType::TwoPhaseGround: return "two_phase_ground";
  }
  throw std::runtime_error("unknown short-circuit fault type");
}

json solve_case(const json& input) {
  const double base_kv = input.at("base_kv").get<double>();
  const double base_mva = input.value("base_mva", 100.0);
  const auto z1 = input.at("z1_ohm");
  const auto z0 = input.at("z0_ohm");
  const double z_base_ohm = base_kv * base_kv / base_mva;

  HybridPowerSystem system;
  system.base_mva = system.ac.base_mva = base_mva;
  ACBus bus;
  bus.index = 1;
  bus.bus_type = BusType::SLACK;
  bus.base_kv = base_kv;
  bus.in_service = true;
  system.ac.buses = {bus};

  ExternalGrid source;
  source.index = 1;
  source.bus = 1;
  source.r_pu = z1.at(0).get<double>() / z_base_ohm;
  source.x_pu = z1.at(1).get<double>() / z_base_ohm;
  source.r0_pu = z0.at(0).get<double>() / z_base_ohm;
  source.x0_pu = z0.at(1).get<double>() / z_base_ohm;
  system.ac.external_grids = {source};

  json output = {{"name", input.at("name")}, {"base_kv", base_kv},
                 {"faults", json::object()}};
  const FaultType fault_types[] = {
      FaultType::ThreePhase, FaultType::SinglePhaseGround,
      FaultType::TwoPhase, FaultType::TwoPhaseGround};
  for (FaultType type : fault_types) {
    SCDetailedOptions options;
    options.fault_type = type;
    options.c_factor = 1.0;
    options.apply_iec_transformer_correction = false;
    options.compute_branch_flows = false;
    options.compute_voltage_drops = false;
    options.compute_nonfault_currents = false;
    options.compute_ith = false;
    const auto result =
        hacdcpf::analysis::run_short_circuit_detailed(system, 1, options);
    if (!result.solved) {
      throw std::runtime_error(input.at("name").get<std::string>() +
                               ": " + result.status + ": " + result.message);
    }
    const auto& row = fault_row(result);
    output["faults"][fault_name(type)] = {
        {"ikss_ka", row.ikss_ka},
        {"i_phase_a_ka", row.i_phase_a_ka},
        {"i_phase_b_ka", row.i_phase_b_ka},
        {"i_phase_c_ka", row.i_phase_c_ka},
        {"i_ground_ka", row.i_ground_ka}};
  }
  return output;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    if (argc != 3) {
      throw std::invalid_argument(
          "usage: short_circuit_thevenin_batch INPUT.json OUTPUT.json");
    }
    std::ifstream stream(argv[1]);
    if (!stream) throw std::runtime_error("cannot open input JSON");
    json input;
    stream >> input;
    json output = {{"schema", "hacdcpf-short-circuit-thevenin-v1"},
                   {"cases", json::array()}};
    for (const auto& item : input.at("cases")) {
      output["cases"].push_back(solve_case(item));
    }
    std::ofstream result_stream(argv[2]);
    if (!result_stream) throw std::runtime_error("cannot open output JSON");
    result_stream << output.dump(2) << '\n';
    std::cout << argv[2] << '\n';
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "short_circuit_thevenin_batch: " << error.what() << '\n';
    return 1;
  }
}
