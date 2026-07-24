#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "hacdcpf/io/json_io.hpp"
#include "hacdcpf/resilience/resilience_assessment.hpp"

namespace {

using hacdcpf::RenewableGen;
using hacdcpf::RenewableType;
using hacdcpf::analysis::DistributionResilienceFault;
using hacdcpf::analysis::DistributionResilienceModel;
using hacdcpf::analysis::DistributionResilienceOptions;
using nlohmann::json;

json read_json(const std::filesystem::path& path) {
  std::ifstream input(path);
  if (!input) throw std::runtime_error("cannot open input bundle: " + path.string());
  return json::parse(input);
}

void write_json(const std::filesystem::path& path, const json& value) {
  std::filesystem::create_directories(path.parent_path());
  const auto temporary = path.string() + ".part";
  {
    std::ofstream output(temporary, std::ios::trunc);
    if (!output) throw std::runtime_error("cannot open output: " + temporary);
    output << value.dump(2) << '\n';
  }
  std::filesystem::rename(temporary, path);
}

std::filesystem::path resolve_path(const std::filesystem::path& bundle,
                                   const std::string& value) {
  std::filesystem::path path(value);
  if (path.is_absolute()) return path;
#ifdef HACDCPF_PROJECT_ROOT
  const auto project_path = std::filesystem::path(HACDCPF_PROJECT_ROOT) / path;
  if (std::filesystem::exists(project_path)) return project_path;
#endif
  const auto bundle_path = bundle.parent_path() / path;
  if (std::filesystem::exists(bundle_path)) return bundle_path;
  return path;
}

void add_fixed_renewables(hacdcpf::HybridPowerSystem& system,
                          const json& case_definition) {
  int next_index = 1;
  for (const auto& renewable : system.ac.renewable_gens) {
    next_index = std::max(next_index, renewable.index + 1);
  }
  const auto add = [&](const json& spec, RenewableType type,
                       const std::string& name) {
    RenewableGen renewable;
    renewable.index = next_index++;
    renewable.bus = spec.at("bus").get<int>();
    renewable.name = name;
    renewable.type = type;
    renewable.p_rated_mw = spec.at("rated_MW").get<double>();
    renewable.p_mw = renewable.p_rated_mw;
    renewable.capacity_factor = 1.0;
    renewable.curtailable = true;
    renewable.anti_islanding = false;
    system.ac.renewable_gens.push_back(renewable);
  };
  add(case_definition.at("added_PV"), RenewableType::SolarPV,
      "FixedCasePV");
  add(case_definition.at("added_wind"), RenewableType::Wind,
      "FixedCaseWind");
}

std::vector<double> profile(const json& item, const char* key,
                            std::size_t expected) {
  auto values = item.at(key).get<std::vector<double>>();
  if (values.size() != expected) {
    throw std::runtime_error(std::string(key) + " length differs from horizon");
  }
  if (!std::all_of(values.begin(), values.end(),
                   [](double value) { return std::isfinite(value); })) {
    throw std::runtime_error(std::string(key) + " contains non-finite values");
  }
  return values;
}

json step_json(const hacdcpf::analysis::DistributionResilienceStepResult& step) {
  return {{"step", step.step_index},
          {"hour", step.hour},
          {"load_multiplier", step.load_multiplier},
          {"pv_multiplier", step.pv_multiplier},
          {"wind_multiplier", step.wind_multiplier},
          {"demand_mw", step.total_demand_mw},
          {"served_mw", step.served_mw},
          {"shed_mw", step.shed_mw},
          {"restoration_ratio", step.restoration_ratio},
          {"active_faults", step.active_faults},
          {"repaired_faults", step.repaired_faults},
          {"switch_actions", step.switch_actions}};
}

json run_case(const hacdcpf::HybridPowerSystem& base_system,
              const json& case_definition, const json& item) {
  const int horizon = case_definition.at("horizon_hours").get<int>();
  const double time_step = case_definition.at("time_step_hours").get<double>();
  const auto expected = static_cast<std::size_t>(
      std::ceil(static_cast<double>(horizon) / time_step));
  DistributionResilienceOptions options;
  options.model = DistributionResilienceModel::HeuristicSequential;
  options.horizon_hours = horizon;
  options.time_step_hr = time_step;
  options.allow_reconfiguration =
      case_definition.at("allow_reconfiguration").get<bool>();
  options.allow_mess_dispatch =
      case_definition.at("allow_MESS_dispatch").get<bool>();
  options.run_power_flow = case_definition.at("run_power_flow").get<bool>();
  options.default_fault_count = 0;
  options.load_profile = profile(item, "load_profile", expected);
  options.pv_profile = profile(item, "pv_profile", expected);
  options.wind_profile = profile(item, "wind_profile", expected);
  options.renewable_profile.assign(expected, 0.0);
  for (const auto& fault_json : item.at("faults")) {
    DistributionResilienceFault fault;
    fault.ac_branch_index = fault_json.at("ac_branch_index").get<int>();
    fault.outage_start_hr = fault_json.at("outage_start_hr").get<double>();
    fault.repair_duration_hr = fault_json.at("repair_duration_hr").get<double>();
    fault.name = fault_json.at("name").get<std::string>();
    options.faults.push_back(fault);
  }
  const auto result =
      hacdcpf::analysis::run_distribution_resilience_assessment(base_system, options);
  json steps = json::array();
  for (const auto& step : result.steps) steps.push_back(step_json(step));
  return {{"id", item.at("id")},
          {"model", item.at("model")},
          {"experiment", item.at("experiment")},
          {"site", item.at("site")},
          {"method", item.at("method")},
          {"member", item.at("member")},
          {"event_start_date", item.at("event_start_date")},
          {"event_end_date", item.at("event_end_date")},
          {"event_compound_stress", item.at("event_compound_stress")},
          {"weather_diagnostics", item.at("weather_diagnostics")},
          {"feasible", result.feasible},
          {"status", result.status},
          {"model_scope", "AC-only heuristic sequential resilience assessment"},
          {"total_demand_mwh", result.total_demand_mwh},
          {"total_served_mwh", result.total_served_mwh},
          {"total_shed_mwh", result.total_shed_mwh},
          {"weighted_unserved_mwh", result.weighted_unserved_mwh},
          {"resilience_index", result.resilience_index},
          {"peak_shed_mw", result.peak_shed_mw},
          {"total_switch_actions", result.total_switch_actions},
          {"total_repaired_faults", result.total_repaired_faults},
          {"steps", std::move(steps)}};
}

}  // namespace

int main(int argc, char** argv) {
  try {
    if (argc != 3) {
      std::cerr << "usage: weather_resilience_case_study INPUT_BUNDLE OUTPUT_JSON\n";
      return 2;
    }
    const std::filesystem::path input_path(argv[1]);
    const std::filesystem::path output_path(argv[2]);
    const auto bundle = read_json(input_path);
    const auto network_path =
        resolve_path(input_path, bundle.at("network").get<std::string>());
    auto system = hacdcpf::io::load_json(network_path.string());
    system.dc = {};
    system.vsc_converters.clear();
    const auto& case_definition = bundle.at("resilience_case");
    add_fixed_renewables(system, case_definition);
    hacdcpf::analysis::apply_distribution_resilience_demo_data(system);
    json results = json::array();
    int failed = 0;
    for (const auto& item : bundle.at("cases")) {
      auto result = run_case(system, case_definition, item);
      if (!result.at("feasible").get<bool>()) ++failed;
      results.push_back(std::move(result));
    }
    const json output = {
        {"schema_version", "1.0"},
        {"operation", "fixed_algorithm_weather_resilience_case_study"},
        {"input_bundle", input_path.string()},
        {"definition_sha256", bundle.at("definition_sha256")},
        {"network", bundle.at("network")},
        {"network_sha256", bundle.at("network_sha256")},
        {"algorithm", "DistributionResilienceModel::HeuristicSequential"},
        {"model_scope", "AC-only heuristic sequential resilience assessment"},
        {"algorithm_tuned_from_weather_results", false},
        {"case_count", results.size()},
        {"failed_case_count", failed},
        {"results", std::move(results)}};
    write_json(output_path, output);
    std::cout << json{{"case_count", output.at("case_count")},
                      {"failed_case_count", failed},
                      {"output", output_path.string()}}
                     .dump(2)
              << '\n';
    return failed == 0 ? 0 : 2;
  } catch (const std::exception& error) {
    std::cerr << "ERROR: " << error.what() << '\n';
    return 2;
  }
}
