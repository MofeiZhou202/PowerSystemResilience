#pragma once

/// power_models/dc_opf_model_builder.hpp
/// =========================================
/// AML-based DC OPF model builder (LP formulation).
/// Replaces: power_models/dc_opf_builder.hpp.
/// NOTE: Result type here (power_models::DCOPFResult) differs from
/// hacdcpf::opf::DCOPFResult — this one is the raw AML builder output.

#include <map>
#include <string>
#include <tuple>
#include <vector>

#include "hacdcpf/aml/aml.hpp"

namespace hacdcpf::power_models {

struct DCOPFData {
  std::vector<std::string> bus_ids;
  std::string slack_bus;

  /// Generator tuples: {id, bus_id, pmin_mw, pmax_mw, cost_per_mwh}
  struct GenTuple {
    std::string id;
    std::string bus;
    double pmin_mw;
    double pmax_mw;
    double cost_per_mwh;
  };
  std::vector<GenTuple> generators;

  /// Branch tuples: {from_bus, to_bus, susceptance_pu, max_flow_mw}
  struct BranchTuple {
    std::string from_bus;
    std::string to_bus;
    double susceptance_pu;
    double max_flow_mw;
  };
  std::vector<BranchTuple> branches;

  /// Nodal demands, keyed by bus id string.
  std::map<std::string, double> demands_MW;
};

/// DC OPF result (AML model builder output; distinct from hacdcpf::opf::DCOPFResult).
struct DCOPFResult {
  aml::SolveResult solve_result;
  std::map<std::string, double> gen_dispatch_MW;
  std::map<std::string, double> lmp_per_MWh;
  std::map<std::string, double> voltage_angle_rad;
  std::map<std::string, double> branch_flow_MW;
};

DCOPFResult solve_dc_opf(const DCOPFData& data, const aml::SolveOptions& opts = {});

}  // namespace hacdcpf::power_models

