// Carbon-analysis independent cross-validation: evidence emitter.
//
// Runs the production carbon-flow analysis (compute_carbon_analysis) on three
// deterministic, hand-crafted AC cases with analytically known nodal carbon
// intensities, and dumps the inputs (sources, loads, directed branch flows) plus
// the solver's nodal-intensity solution as JSON.  The companion oracle
// tools/carbon_analysis_validation/run_cross_validation.py does NOT link
// hacdcpf: it re-derives the Kang carbon-emission-flow linear system A w = b from
// these inputs, re-solves it independently, and checks the solver output against
// analytic closed forms, nodal conservation and the loss-allocation rule.
//
// The power flow is supplied directly (a converged PowerFlowResult with branch
// flows) so the cases are exactly reproducible and independent of any PF solver.
//
// Usage: validate_carbon_xref <output.json>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

#include <nlohmann/json.hpp>

#include "hacdcpf/carbon_analysis/carbon_analysis.hpp"
#include "hacdcpf/model/hybrid_power_system.hpp"
#include "hacdcpf/power_flow/power_flow_result.hpp"

namespace fs = std::filesystem;
using nlohmann::json;
using namespace hacdcpf;
using namespace hacdcpf::analysis;

namespace {

ACBus make_bus(int index, BusType type) {
  ACBus bus;
  bus.index = index;
  bus.bus_type = type;
  bus.in_service = true;
  return bus;
}

Generator make_generator(int index, int bus, double p_mw, double emission_factor) {
  Generator gen;
  gen.index = index;
  gen.bus = bus;
  gen.in_service = true;
  gen.pg_mw = p_mw;
  gen.pmin_mw = 0.0;
  gen.pmax_mw = 500.0;
  gen.emission_factor_tco2_mwh = emission_factor;
  return gen;
}

Load make_load(int index, int bus, double p_mw) {
  Load load;
  load.index = index;
  load.bus = bus;
  load.in_service = true;
  load.p_mw = p_mw;
  return load;
}

ACBranch make_branch(int index, int from_bus, int to_bus) {
  ACBranch branch;
  branch.index = index;
  branch.from_bus = from_bus;
  branch.to_bus = to_bus;
  branch.in_service = true;
  return branch;
}

// Serialize one case: the exact matrix inputs (sources, loads, directed edges)
// and the production nodal-intensity solution.
json emit_case(const std::string& name, const HybridPowerSystem& sys,
               const PowerFlowResult& pf) {
  CarbonAnalysisOptions opt;  // default loss_allocation_alpha = 0.5
  const CarbonAnalysisResult ca = compute_carbon_analysis(sys, pf, opt);

  const int n_ac = static_cast<int>(sys.ac.buses.size());
  const int n_dc = static_cast<int>(sys.dc.buses.size());
  const int node_count = n_ac + n_dc;

  std::unordered_map<int, int> ac_loc;
  for (int i = 0; i < n_ac; ++i) ac_loc[sys.ac.buses[i].index] = i;
  std::unordered_map<int, int> dc_loc;
  for (int j = 0; j < n_dc; ++j) dc_loc[sys.dc.buses[j].index] = n_ac + j;

  const auto loc_of = [&](int bus, bool is_dc) -> int {
    if (is_dc) {
      auto it = dc_loc.find(bus);
      return it == dc_loc.end() ? -1 : it->second;
    }
    auto it = ac_loc.find(bus);
    return it == ac_loc.end() ? -1 : it->second;
  };

  // Sources: b(node_loc) += emission_factor * power_mw. These echo generator
  // emission factors and dispatch (inputs), not the carbon solution.
  json sources = json::array();
  for (const auto& cs : ca.carbon_sources) {
    sources.push_back({{"node_loc", loc_of(cs.bus, cs.is_dc)},
                       {"ef", cs.emission_factor_tco2_mwh},
                       {"power_mw", cs.power_mw}});
  }

  // Loads: through-power sinks at each node (real load demand).
  json loads = json::array();
  for (const auto& lc : ca.load_carbon) {
    loads.push_back({{"node_loc", loc_of(lc.bus, false)},
                     {"demand_mw", lc.demand_mw}});
  }
  for (const auto& lc : ca.dc_load_carbon) {
    loads.push_back({{"node_loc", loc_of(lc.bus, true)},
                     {"demand_mw", lc.demand_mw}});
  }

  // Directed branch flows from the supplied power flow (AC branches only in
  // these cases): orient so send >= 0, recv = arriving power, loss = send-recv.
  json edges = json::array();
  for (std::size_t i = 0; i < sys.ac.branches.size(); ++i) {
    const auto& br = sys.ac.branches[i];
    if (i >= pf.branch_flows.size()) continue;
    const BranchFlow& bf = pf.branch_flows[i];
    const int fl = loc_of(br.from_bus, false);
    const int tl = loc_of(br.to_bus, false);
    int from_loc, to_loc;
    double send, recv;
    if (bf.pf_mw >= 0.0) {
      from_loc = fl;
      to_loc = tl;
      send = bf.pf_mw;
      recv = std::max(-bf.pt_mw, 0.0);
    } else {
      from_loc = tl;
      to_loc = fl;
      send = bf.pt_mw;
      recv = std::max(-bf.pf_mw, 0.0);
    }
    const double loss = std::clamp(std::max(send - recv, 0.0), 0.0, send);
    edges.push_back({{"from_node_loc", from_loc},
                     {"to_node_loc", to_loc},
                     {"send_mw", send},
                     {"recv_mw", recv},
                     {"loss_mw", loss}});
  }

  // Production nodal-intensity solution w, in node order (AC buses then DC).
  json bus_intensity = json::array();
  for (const auto& bc : ca.bus_carbon)
    bus_intensity.push_back(bc.carbon_intensity_tco2_mwh);
  for (const auto& bc : ca.dc_bus_carbon)
    bus_intensity.push_back(bc.carbon_intensity_tco2_mwh);

  json result_loads = json::array();
  for (const auto& lc : ca.load_carbon) {
    result_loads.push_back({{"node_loc", loc_of(lc.bus, false)},
                            {"demand_mw", lc.demand_mw},
                            {"intensity", lc.carbon_intensity_tco2_mwh},
                            {"emissions_tco2", lc.total_emissions_tco2}});
  }

  json result_branches = json::array();
  for (const auto& bc : ca.branch_carbon) {
    result_branches.push_back({{"from_node_loc", loc_of(bc.from_bus, false)},
                               {"to_node_loc", loc_of(bc.to_bus, false)},
                               {"loss_mw", bc.loss_mw},
                               {"intensity", bc.carbon_intensity_tco2_mwh},
                               {"emissions_tco2", bc.total_emissions_tco2}});
  }

  return json{
      {"name", name},
      {"alpha", opt.loss_allocation_alpha},
      {"node_count", node_count},
      {"n_ac", n_ac},
      {"n_dc", n_dc},
      {"sources", sources},
      {"loads", loads},
      {"edges", edges},
      {"bus_intensity", bus_intensity},
      {"result_loads", result_loads},
      {"result_branches", result_branches},
      {"matrix_solved", ca.matrix_solved},
      {"matrix_rank", ca.matrix_rank},
      {"matrix_relative_residual", ca.matrix_relative_residual},
      {"matrix_summary",
       {{"total_generation_emissions_tco2",
         ca.matrix_summary.total_generation_emissions_tco2},
        {"total_load_emissions_tco2",
         ca.matrix_summary.total_load_emissions_tco2},
        {"total_loss_emissions_tco2",
         ca.matrix_summary.total_loss_emissions_tco2},
        {"balance_error_tco2", ca.matrix_summary.balance_error_tco2}}}};
}

// Case 1 — single source, series feed: one clean generator supplies two
// downstream loads through a lossless radial path. Every reachable bus inherits
// the source emission factor exactly (w = 0.5 everywhere).
json single_source_series() {
  HybridPowerSystem sys;
  sys.base_mva = sys.ac.base_mva = 100.0;
  sys.ac.buses = {make_bus(1, BusType::PV), make_bus(2, BusType::PQ),
                  make_bus(3, BusType::PQ)};
  sys.ac.generators = {make_generator(1, 1, 100.0, 0.5)};
  sys.ac.loads = {make_load(1, 2, 40.0), make_load(2, 3, 60.0)};
  sys.ac.branches = {make_branch(1, 1, 2), make_branch(2, 2, 3)};

  PowerFlowResult pf;
  pf.converged = true;
  pf.vm = {1.0, 1.0, 1.0};
  pf.va = {0.0, 0.0, 0.0};
  pf.branch_flows = {BranchFlow{100.0, 0.0, -100.0, 0.0},
                     BranchFlow{60.0, 0.0, -60.0, 0.0}};
  return emit_case("single_source_series", sys, pf);
}

// Case 2 — two sources mixing at a common load, lossless: the load intensity is
// the dispatch-weighted average (30*1.0 + 70*0.0)/100 = 0.3.
json two_source_mixing() {
  HybridPowerSystem sys;
  sys.base_mva = sys.ac.base_mva = 100.0;
  sys.ac.buses = {make_bus(1, BusType::PV), make_bus(2, BusType::PV),
                  make_bus(3, BusType::PQ)};
  sys.ac.generators = {make_generator(1, 1, 30.0, 1.0),
                       make_generator(2, 2, 70.0, 0.0)};
  sys.ac.loads = {make_load(1, 3, 100.0)};
  sys.ac.branches = {make_branch(1, 1, 3), make_branch(2, 2, 3)};

  PowerFlowResult pf;
  pf.converged = true;
  pf.vm = {1.0, 1.0, 1.0};
  pf.va = {0.0, 0.0, 0.0};
  pf.branch_flows = {BranchFlow{30.0, 0.0, -30.0, 0.0},
                     BranchFlow{70.0, 0.0, -70.0, 0.0}};
  return emit_case("two_source_mixing", sys, pf);
}

// Case 3 — two sources mixing with line losses: exercises the loss-allocation
// terms. With alpha = 0.5 the load-bus intensity is 57.5/95 and each branch loss
// intensity is 0.5 w_from + 0.5 w_to; the emission balance closes to 60 tCO2.
json two_source_lossy() {
  HybridPowerSystem sys;
  sys.base_mva = sys.ac.base_mva = 100.0;
  sys.ac.buses = {make_bus(1, BusType::PV), make_bus(2, BusType::PV),
                  make_bus(3, BusType::PQ)};
  sys.ac.generators = {make_generator(1, 1, 60.0, 1.0),
                       make_generator(2, 2, 40.0, 0.0)};
  sys.ac.loads = {make_load(1, 3, 90.0)};
  sys.ac.branches = {make_branch(1, 1, 3), make_branch(2, 2, 3)};

  PowerFlowResult pf;
  pf.converged = true;
  pf.vm = {1.0, 1.0, 1.0};
  pf.va = {0.0, 0.0, 0.0};
  pf.branch_flows = {BranchFlow{60.0, 0.0, -55.0, 0.0},
                     BranchFlow{40.0, 0.0, -35.0, 0.0}};
  return emit_case("two_source_lossy", sys, pf);
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "usage: validate_carbon_xref <output.json>\n";
    return 2;
  }
  json out;
  out["schema"] = "hysim-carbon-analysis-evidence-v1";
  out["cases"] = json::array({single_source_series(), two_source_mixing(),
                              two_source_lossy()});

  const fs::path path(argv[1]);
  if (path.has_parent_path()) fs::create_directories(path.parent_path());
  std::ofstream os(path);
  if (!os) {
    std::cerr << "cannot open output: " << argv[1] << "\n";
    return 1;
  }
  os << out.dump(2) << "\n";
  return 0;
}
