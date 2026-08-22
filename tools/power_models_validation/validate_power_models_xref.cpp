#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

#include <nlohmann/json.hpp>

#include "hacdcpf/power_models/dc_opf_model_builder.hpp"
#include "hacdcpf/power_models/lindistflow_builder.hpp"
#include "hacdcpf/power_models/scuc_builder.hpp"

namespace fs = std::filesystem;
using nlohmann::json;
using namespace hacdcpf::power_models;

namespace {

json solve_metadata(const hacdcpf::aml::SolveResult& result) {
  return {{"has_primal", result.has_primal()},
          {"optimal", result.is_optimal()},
          {"has_duals", result.has_duals()},
          {"objective", result.objective_value},
          {"objective_bound", result.objective_bound},
          {"optimality_gap", result.optimality_gap},
          {"solver", result.solver_used}};
}

// The explicit backend and fail-closed fallback contract implement the
// cross-backend protocol in numerical_cross_validation.tex.
hacdcpf::aml::SolveOptions exact_solver(const std::string& solver_name) {
  hacdcpf::aml::SolveOptions options;
  options.solver_name = solver_name;
  options.allow_fallback = false;
  return options;
}

json dcopf_case(const std::string& lp_solver) {
  // Analytic congested two-bus case from
  // docs/modules/power_models/chapters/theory_modeling_foundations.tex.
  DCOPFData data;
  data.bus_ids = {"b1", "b2"};
  data.slack_bus = "b1";
  data.generators = {{"cheap", "b1", 0.0, 100.0, 10.0},
                     {"local", "b2", 0.0, 100.0, 50.0}};
  data.branches = {{"b1", "b2", 10.0, 40.0}};
  data.demands_MW = {{"b1", 0.0}, {"b2", 80.0}};

  const auto result = solve_dc_opf(data, exact_solver(lp_solver));
  if (!result.solve_result.has_primal()) {
    throw std::runtime_error("DCOPF validation case has no primal solution");
  }
  return {{"input",
           {{"demand_b2_mw", 80.0}, {"susceptance", 10.0},
            {"line_limit_mw", 40.0}, {"cheap_cost", 10.0},
            {"local_cost", 50.0}}},
          {"solve", solve_metadata(result.solve_result)},
          {"dispatch_mw", result.gen_dispatch_MW},
          {"angles_rad", result.voltage_angle_rad},
          {"flows_mw", result.branch_flow_MW},
          {"lmp_per_mwh", result.lmp_per_MWh}};
}

json lindistflow_case(const std::string& lp_solver) {
  // Baran-Wu lossless branch-flow recursion; derivation and units are recorded
  // in docs/modules/power_models/chapters/theory_modeling_foundations.tex.
  LinDistFlowData data;
  data.bus_ids = {"b1", "b2", "b3"};
  data.root_bus = "b1";
  data.root_v_sq_pu = 1.0;
  data.branches = {{"b1", "b2", 0.01, 0.02, 10.0},
                   {"b2", "b3", 0.015, 0.025, 10.0}};
  data.p_load_mw = {{"b2", 1.0}, {"b3", 0.5}};
  data.q_load_mvar = {{"b2", 0.2}, {"b3", 0.1}};

  const auto result = solve_lindistflow(data, exact_solver(lp_solver));
  if (!result.solve_result.has_primal()) {
    throw std::runtime_error("LinDistFlow validation case has no primal solution");
  }
  return {{"input",
           {{"root_v_sq_pu", 1.0},
            {"branches", {{{"from", "b1"}, {"to", "b2"}, {"r", 0.01}, {"x", 0.02}},
                           {{"from", "b2"}, {"to", "b3"}, {"r", 0.015}, {"x", 0.025}}}},
            {"p_load_mw", data.p_load_mw}, {"q_load_mvar", data.q_load_mvar}}},
          {"solve", solve_metadata(result.solve_result)},
          {"voltage_sq_pu", result.voltage_sq_pu},
          {"branch_p_mw", result.branch_p_mw},
          {"branch_q_mvar", result.branch_q_mvar}};
}

json scuc_case() {
  // Two-period merit-order UC benchmark from
  // docs/modules/power_models/chapters/theory_modeling_foundations.tex.
  SCUCData data;
  data.generator_ids = {"cheap", "peaker"};
  data.period_ids = {"t1", "t2"};
  data.demand_MW = {{"t1", 30.0}, {"t2", 70.0}};
  data.cost_per_MWh = {{"cheap", 10.0}, {"peaker", 30.0}};
  data.pmin_MW = {{"cheap", 0.0}, {"peaker", 0.0}};
  data.pmax_MW = {{"cheap", 50.0}, {"peaker", 50.0}};
  data.ramp_up_MW = {{"cheap", 100.0}, {"peaker", 100.0}};
  data.ramp_dn_MW = {{"cheap", 100.0}, {"peaker", 100.0}};
  data.startup_cost = {{"cheap", 0.0}, {"peaker", 0.0}};
  data.commit_cost = {{"cheap", 0.0}, {"peaker", 0.0}};

  const auto result = solve_scuc(data, exact_solver("StrictHiGHS"));
  if (!result.solve_result.has_primal()) {
    throw std::runtime_error("SCUC validation case has no primal solution");
  }
  json dispatch;
  json commitment;
  for (const auto& generator : data.generator_ids) {
    for (const auto& period : data.period_ids) {
      const auto key = std::make_pair(generator, period);
      dispatch[generator][period] = result.dispatch_MW.at(key);
      commitment[generator][period] = result.commitment.at(key);
    }
  }
  return {{"input",
           {{"periods", data.period_ids}, {"demand_mw", data.demand_MW},
            {"cost_per_mwh", data.cost_per_MWh}, {"pmax_mw", data.pmax_MW}}},
          {"solve", solve_metadata(result.solve_result)},
          {"dispatch_mw", dispatch}, {"commitment", commitment},
          {"model_scope", result.model_scope},
          {"price_valid", result.price_valid},
          {"price_validity_reason", result.price_validity_reason}};
}

}  // namespace

int main(int argc, char** argv) {
  try {
    if (argc != 4) {
      throw std::runtime_error(
          "usage: validate_power_models_xref OUTPUT.json DCOPF_SOLVER "
          "LINDISTFLOW_SOLVER");
    }
    const std::string dcopf_solver = argv[2];
    const std::string lindistflow_solver = argv[3];
    const json output = {{"schema", "hysim-power-models-xref-v1"},
                         {"requested_dcopf_solver", dcopf_solver},
                         {"requested_lindistflow_solver", lindistflow_solver},
                         {"dcopf", dcopf_case(dcopf_solver)},
                         {"lindistflow", lindistflow_case(lindistflow_solver)},
                         {"scuc", scuc_case()}};
    const fs::path path = argv[1];
    if (!path.parent_path().empty()) fs::create_directories(path.parent_path());
    std::ofstream stream(path);
    if (!stream) throw std::runtime_error("cannot open output file: " + path.string());
    stream << output.dump(2) << '\n';
    std::cout << path.string() << '\n';
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "power-model evidence generation failed: " << error.what() << '\n';
    return 2;
  }
}
