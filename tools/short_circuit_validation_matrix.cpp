#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

#include "hacdcpf/analysis/short_circuit.hpp"
#include "hacdcpf/model/components.hpp"
#include "hacdcpf/model/system.hpp"

namespace {

using hacdcpf::ACBranch;
using hacdcpf::ACBus;
using hacdcpf::BusType;
using hacdcpf::ExternalGrid;
using hacdcpf::HybridPowerSystem;
using hacdcpf::Transformer2W;
using hacdcpf::VSCConverter;
using hacdcpf::analysis::FaultType;
using hacdcpf::analysis::SCCalcType;
using hacdcpf::analysis::SCDetailedOptions;
using hacdcpf::analysis::SCKappaMethod;
using hacdcpf::analysis::SCTopology;
using nlohmann::json;

struct BenchmarkCase {
  std::string name;
  std::string category;
  HybridPowerSystem system;
  int fault_bus{1};
  SCDetailedOptions options;
  json model;
};

ACBus bus(int index, double base_kv, BusType type = BusType::PQ) {
  ACBus value;
  value.index = index;
  value.base_kv = base_kv;
  value.bus_type = type;
  value.in_service = true;
  value.name = "bus" + std::to_string(index);
  return value;
}

ExternalGrid source(int bus_id, double r1_pu, double x1_pu,
                    double r0_pu, double x0_pu) {
  ExternalGrid value;
  value.index = 1;
  value.bus = bus_id;
  value.in_service = true;
  value.r_pu = r1_pu;
  value.x_pu = x1_pu;
  value.r0_pu = r0_pu;
  value.x0_pu = x0_pu;
  return value;
}

SCDetailedOptions benchmark_options(FaultType fault_type) {
  SCDetailedOptions options;
  options.fault_type = fault_type;
  options.calc_type = SCCalcType::Max;
  options.c_factor = 1.0;
  options.kappa_method = SCKappaMethod::A;
  options.topology = SCTopology::Radial;
  options.apply_iec_transformer_correction = false;
  options.compute_branch_flows = false;
  options.compute_voltage_drops = false;
  options.compute_ith = false;
  return options;
}

std::string fault_name(FaultType type) {
  switch (type) {
    case FaultType::ThreePhase: return "three_phase";
    case FaultType::SinglePhaseGround: return "single_phase_ground";
    case FaultType::TwoPhase: return "two_phase";
    case FaultType::TwoPhaseGround: return "two_phase_ground";
  }
  return "unknown";
}

BenchmarkCase line_case(std::string name, std::string category,
                        double base_kv, double source_r1, double source_x1,
                        std::vector<std::pair<double, double>> line_z1,
                        int fault_bus, FaultType fault_type,
                        double zero_sequence_multiplier = 1.0,
                        bool meshed = false) {
  BenchmarkCase value;
  value.name = std::move(name);
  value.category = std::move(category);
  value.fault_bus = fault_bus;
  value.options = benchmark_options(fault_type);
  value.options.topology = meshed ? SCTopology::Meshed : SCTopology::Radial;
  value.system.name = value.name;
  value.system.base_mva = 100.0;
  value.system.ac.base_mva = 100.0;
  value.system.ac.freq_hz = 50.0;

  int bus_count = static_cast<int>(line_z1.size()) + 1;
  if (meshed) bus_count = 3;
  for (int i = 1; i <= bus_count; ++i) {
    value.system.ac.buses.push_back(bus(i, base_kv, i == 1 ? BusType::SLACK : BusType::PQ));
  }
  value.system.ac.external_grids.push_back(source(
      1, source_r1, source_x1,
      source_r1 * zero_sequence_multiplier,
      source_x1 * zero_sequence_multiplier));

  json branches = json::array();
  int branch_index = 1;
  for (std::size_t i = 0; i < line_z1.size(); ++i) {
    ACBranch branch;
    branch.index = branch_index++;
    if (meshed && i == 2) {
      branch.from_bus = 1;
      branch.to_bus = 3;
    } else {
      branch.from_bus = static_cast<int>(i) + 1;
      branch.to_bus = static_cast<int>(i) + 2;
    }
    branch.r_pu = line_z1[i].first;
    branch.x_pu = line_z1[i].second;
    branch.r0_pu = branch.r_pu * zero_sequence_multiplier;
    branch.x0_pu = branch.x_pu * zero_sequence_multiplier;
    branch.tap = 1.0;
    branch.in_service = true;
    value.system.ac.branches.push_back(branch);
    branches.push_back({
        {"from_bus", branch.from_bus}, {"to_bus", branch.to_bus},
        {"r1_pu", branch.r_pu}, {"x1_pu", branch.x_pu},
        {"r0_pu", branch.r0_pu}, {"x0_pu", branch.x0_pu}});
  }

  value.model = {
      {"kind", "line_network"}, {"base_mva", 100.0},
      {"base_kv", base_kv}, {"bus_count", bus_count},
      {"source", {{"r1_pu", source_r1}, {"x1_pu", source_x1},
                    {"r0_pu", source_r1 * zero_sequence_multiplier},
                    {"x0_pu", source_x1 * zero_sequence_multiplier}}},
      {"branches", branches}};
  return value;
}

BenchmarkCase transformer_case(std::string name, std::string category,
                               int tap_side, double tap_pu,
                               double vk_percent, double vkr_percent,
                               FaultType fault_type = FaultType::ThreePhase,
                               double z0_multiplier = 1.0) {
  BenchmarkCase value;
  value.name = std::move(name);
  value.category = std::move(category);
  value.fault_bus = 2;
  value.options = benchmark_options(fault_type);
  value.system.name = value.name;
  value.system.base_mva = 100.0;
  value.system.ac.base_mva = 100.0;
  value.system.ac.freq_hz = 50.0;
  value.system.ac.buses = {bus(1, 110.0, BusType::SLACK), bus(2, 20.0)};
  constexpr double source_z_mag = 0.1;
  constexpr double source_xr = 10.0;
  const double source_r = source_z_mag / std::sqrt(1.0 + source_xr * source_xr);
  const double source_x = source_r * source_xr;
  value.system.ac.external_grids.push_back(source(
      1, source_r, source_x, source_r * z0_multiplier, source_x * z0_multiplier));

  Transformer2W transformer;
  transformer.index = 1;
  transformer.name = "transformer";
  transformer.hv_bus = 1;
  transformer.lv_bus = 2;
  transformer.in_service = true;
  transformer.sn_mva = 100.0;
  transformer.vn_hv_kv = 110.0;
  transformer.vn_lv_kv = 20.0;
  transformer.vk_percent = vk_percent;
  transformer.vkr_percent = vkr_percent;
  transformer.z0_percent = vk_percent * z0_multiplier;
  const double x_percent = std::sqrt(std::max(
      0.0, vk_percent * vk_percent - vkr_percent * vkr_percent));
  transformer.x0_r0 = vkr_percent > 0.0 ? x_percent / vkr_percent : 1e9;
  transformer.tap_side = tap_side;
  transformer.tap_neutral = 0;
  transformer.tap_step_percent = 1.0;
  transformer.tap_pos = static_cast<int>(std::lround((tap_pu - 1.0) * 100.0));
  value.system.ac.transformers_2w.push_back(transformer);

  value.model = {
      {"kind", "transformer_network"}, {"base_mva", 100.0},
      {"hv_kv", 110.0}, {"lv_kv", 20.0},
      {"source", {{"r1_pu", source_r}, {"x1_pu", source_x},
                    {"r0_pu", source_r * z0_multiplier},
                    {"x0_pu", source_x * z0_multiplier}}},
      {"transformer", {{"sn_mva", 100.0}, {"vk_percent", vk_percent},
                        {"vkr_percent", vkr_percent},
                        {"z0_multiplier", z0_multiplier},
                        {"tap_side", tap_side == 0 ? "hv" : "lv"},
                        {"tap_pu", tap_pu}}}};
  return value;
}

struct IBRSpec {
  std::string mode;
  int bus{2};
  double rating_mva{20.0};
  double current_limit_pu{1.2};
  double r_sc_pu{0.01};
  double x_sc_pu{0.15};
};

BenchmarkCase ibr_case(std::string name, std::string category,
                       std::vector<std::pair<double, double>> line_z1,
                       int fault_bus, const std::vector<IBRSpec>& specs) {
  BenchmarkCase value = line_case(
      std::move(name), std::move(category), 20.0,
      0.00995037, 0.09950372, std::move(line_z1), fault_bus,
      FaultType::ThreePhase);
  json devices = json::array();
  int index = 1;
  for (const auto& spec : specs) {
    hacdcpf::DCBus dc_bus;
    dc_bus.index = index;
    dc_bus.base_kv = 20.0;
    dc_bus.in_service = true;
    value.system.dc.buses.push_back(dc_bus);

    VSCConverter converter;
    converter.index = index;
    converter.name = "ibr" + std::to_string(index);
    converter.bus_ac = spec.bus;
    converter.bus_dc = index;
    converter.in_service = true;
    converter.p_rated_mw = spec.rating_mva;
    converter.vn_ac_kv = 20.0;
    converter.i_max_pu = spec.current_limit_pu;
    converter.r_sc_pu = spec.r_sc_pu;
    converter.x_sc_pu = spec.x_sc_pu;
    converter.ac_grid_forming = spec.mode == "grid_forming";
    value.system.vsc_converters.push_back(converter);

    devices.push_back({
        {"name", converter.name}, {"mode", spec.mode}, {"bus", spec.bus},
        {"rating_mva", spec.rating_mva},
        {"current_limit_pu", spec.current_limit_pu},
        {"r_sc_pu", spec.r_sc_pu}, {"x_sc_pu", spec.x_sc_pu}});
    ++index;
  }
  value.model["ibrs"] = std::move(devices);
  return value;
}

std::vector<BenchmarkCase> build_cases() {
  std::vector<BenchmarkCase> cases;
  const std::vector<std::pair<std::string, std::pair<double, double>>> sources = {
      {"strong_xr20", {0.00249688, 0.04993762}},
      {"medium_xr10", {0.00995037, 0.09950372}},
      {"weak_xr5", {0.03922323, 0.19611614}},
      {"resistive_xr2", {0.08944272, 0.17888544}},
  };
  for (const auto& [label, z] : sources) {
    cases.push_back(line_case("source_" + label, "source_sweep", 20.0,
                              z.first, z.second, {{0.005, 0.05}}, 2,
                              FaultType::ThreePhase));
  }

  const std::vector<std::pair<std::string, std::pair<double, double>>> lines = {
      {"short", {0.001, 0.01}}, {"nominal", {0.01, 0.08}},
      {"long", {0.04, 0.24}}, {"high_r", {0.12, 0.08}},
  };
  for (const auto& [label, z] : lines) {
    cases.push_back(line_case("line_" + label, "line_sweep", 20.0,
                              0.00995037, 0.09950372, {z}, 2,
                              FaultType::ThreePhase));
  }

  cases.push_back(line_case("radial_3bus_near", "topology", 20.0,
                            0.00995037, 0.09950372,
                            {{0.01, 0.06}, {0.015, 0.09}}, 2,
                            FaultType::ThreePhase));
  cases.push_back(line_case("radial_3bus_far", "topology", 20.0,
                            0.00995037, 0.09950372,
                            {{0.01, 0.06}, {0.015, 0.09}}, 3,
                            FaultType::ThreePhase));
  cases.push_back(line_case("meshed_3bus", "topology", 20.0,
                            0.00995037, 0.09950372,
                            {{0.01, 0.06}, {0.015, 0.09}, {0.02, 0.10}}, 3,
                            FaultType::ThreePhase, 1.0, true));
  const std::vector<std::pair<double, double>> feeder_10bus = {
      {0.004, 0.012}, {0.005, 0.014}, {0.004, 0.013},
      {0.006, 0.016}, {0.005, 0.015}, {0.007, 0.018},
      {0.006, 0.017}, {0.008, 0.020}, {0.007, 0.019},
  };
  for (const auto& [label, fault_bus] :
       std::vector<std::pair<std::string, int>>{{"near", 3}, {"mid", 6}, {"far", 10}}) {
    cases.push_back(line_case("feeder_10bus_" + label, "distribution_feeder",
                              12.47, 0.00995037, 0.09950372,
                              feeder_10bus, fault_bus, FaultType::ThreePhase,
                              2.5));
  }

  for (const int tap_side : {0, 1}) {
    for (const double tap : {0.90, 0.95, 1.00, 1.05, 1.10}) {
      const std::string side = tap_side == 0 ? "hv" : "lv";
      cases.push_back(transformer_case(
          "transformer_" + side + "_tap_" + std::to_string(static_cast<int>(tap * 100)),
          "transformer_tap", tap_side, tap, 10.0, 1.0));
    }
  }
  for (const auto& [vk, vkr] : std::vector<std::pair<double, double>>{
           {4.0, 0.5}, {6.0, 0.8}, {10.0, 1.0}, {16.0, 1.5}}) {
    cases.push_back(transformer_case(
        "transformer_vk_" + std::to_string(static_cast<int>(vk)),
        "transformer_impedance", 0, 1.0, vk, vkr));
  }

  for (const double z0_multiplier : {0.5, 1.0, 2.0, 3.0}) {
    cases.push_back(line_case(
        "slg_z0_" + std::to_string(static_cast<int>(z0_multiplier * 10)),
        "zero_sequence", 20.0, 0.00995037, 0.09950372,
        {{0.01, 0.08}}, 2, FaultType::SinglePhaseGround,
        z0_multiplier));
  }
  cases.push_back(line_case("two_phase_radial", "fault_type", 20.0,
                            0.00995037, 0.09950372, {{0.01, 0.08}}, 2,
                            FaultType::TwoPhase));
  cases.push_back(transformer_case("transformer_slg_tap_lv_95", "zero_sequence",
                                   1, 0.95, 10.0, 1.0,
                                   FaultType::SinglePhaseGround));

  for (const double limit : {1.05, 1.20, 1.50}) {
    cases.push_back(ibr_case(
        "gfl_limit_" + std::to_string(static_cast<int>(limit * 100)),
        "ibr_grid_following", {{0.01, 0.08}}, 2,
        {{"grid_following", 2, 20.0, limit, 0.01, 0.15}}));
  }
  for (const double rating : {5.0, 20.0, 50.0}) {
    cases.push_back(ibr_case(
        "gfl_rating_" + std::to_string(static_cast<int>(rating)),
        "ibr_grid_following", {{0.01, 0.08}}, 2,
        {{"grid_following", 2, rating, 1.20, 0.01, 0.15}}));
  }
  cases.push_back(ibr_case(
      "gfl_remote_bus", "ibr_grid_following",
      {{0.008, 0.05}, {0.012, 0.07}}, 3,
      {{"grid_following", 2, 20.0, 1.20, 0.01, 0.15}}));
  cases.push_back(ibr_case(
      "gfl_dual_local", "ibr_grid_following", {{0.01, 0.08}}, 2,
      {{"grid_following", 2, 10.0, 1.20, 0.01, 0.15},
       {"grid_following", 2, 15.0, 1.10, 0.01, 0.15}}));

  for (const double rating : {10.0, 25.0, 50.0}) {
    cases.push_back(ibr_case(
        "gfm_rating_" + std::to_string(static_cast<int>(rating)),
        "ibr_grid_forming", {{0.01, 0.08}}, 2,
        {{"grid_forming", 2, rating, 2.0, 0.01, 0.15}}));
  }
  for (const double x_sc : {0.10, 0.20, 0.30}) {
    cases.push_back(ibr_case(
        "gfm_x_" + std::to_string(static_cast<int>(x_sc * 100)),
        "ibr_grid_forming", {{0.01, 0.08}}, 2,
        {{"grid_forming", 2, 25.0, 2.0, 0.01, x_sc}}));
  }
  cases.push_back(ibr_case(
      "gfm_remote_bus", "ibr_grid_forming",
      {{0.008, 0.05}, {0.012, 0.07}}, 3,
      {{"grid_forming", 2, 25.0, 2.0, 0.01, 0.15}}));
  cases.push_back(ibr_case(
      "ibr_mixed_gfm_gfl", "ibr_mixed", {{0.01, 0.08}}, 2,
      {{"grid_forming", 2, 25.0, 2.0, 0.01, 0.15},
       {"grid_following", 2, 20.0, 1.20, 0.01, 0.15}}));
  return cases;
}

const hacdcpf::analysis::SCDetailedBusResult& fault_row(
    const hacdcpf::analysis::SCDetailedResult& result) {
  for (const auto& row : result.bus_results) {
    if (row.bus_id == result.fault_bus_id) return row;
  }
  throw std::runtime_error("fault result row is missing");
}

}  // namespace

int main(int argc, char** argv) {
  std::filesystem::path output_path;
  if (argc == 3 && std::string(argv[1]) == "--out") {
    output_path = argv[2];
  } else if (argc != 1) {
    std::cerr << "Usage: " << argv[0] << " [--out PATH]\n";
    return 2;
  }

  try {
    json output;
    output["schema"] = "hacdcpf-short-circuit-validation-v1";
    output["method"] = "HACDCPF IEC 60909 detailed solver; transformer correction disabled for engine parity";
    output["cases"] = json::array();
    for (auto& test_case : build_cases()) {
      const auto result = hacdcpf::analysis::run_short_circuit_detailed(
          test_case.system, test_case.fault_bus, test_case.options);
      if (!result.solved) throw std::runtime_error(test_case.name + " did not solve");
      const auto& row = fault_row(result);
      output["cases"].push_back({
          {"name", test_case.name}, {"category", test_case.category},
          {"fault_bus", test_case.fault_bus},
          {"fault_type", fault_name(test_case.options.fault_type)},
          {"model", test_case.model},
          {"hacdcpf", {{"ikss_ka", row.ikss_ka}, {"ip_ka", row.ip_ka},
                        {"ibr_contribution_ka", row.ikss_converter_contrib_ka}}}});
    }
    output["case_count"] = output["cases"].size();

    const std::string text = output.dump(2) + "\n";
    if (output_path.empty()) {
      std::cout << text;
    } else {
      std::ofstream stream(output_path);
      if (!stream) throw std::runtime_error("cannot open " + output_path.string());
      stream << text;
    }
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "short_circuit_validation_matrix failed: " << error.what() << "\n";
    return 1;
  }
}
