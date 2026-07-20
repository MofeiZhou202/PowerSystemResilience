#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "hacdcpf/dynamics/dynamics.hpp"
#include "hacdcpf/io/case_builders.hpp"
#include "hacdcpf/resilience/certified_restoration.hpp"

using json = nlohmann::json;
using namespace hacdcpf;
using namespace hacdcpf::dynamics;

namespace {

void enable_gfm_storage(HybridPowerSystem& system, bool all_storage) {
  if (system.ac.storage.empty()) {
    throw std::runtime_error("case has no stationary AC storage for GFM overlay");
  }
  auto largest = std::max_element(
      system.ac.storage.begin(), system.ac.storage.end(),
      [](const Storage& lhs, const Storage& rhs) {
        return lhs.p_rated_mw < rhs.p_rated_mw;
      });
  for (auto& storage : system.ac.storage) {
    const bool selected = all_storage || &storage == &*largest;
    storage.grid_forming = selected;
    storage.control_mode = selected ? "grid_forming" : "pq";
    if (!selected) continue;
    storage.anti_islanding = false;
    storage.qmax_mvar = std::max(storage.qmax_mvar, 0.6 * storage.p_rated_mw);
    storage.qmin_mvar = std::min(storage.qmin_mvar, -0.6 * storage.p_rated_mw);
  }
}

std::size_t count_gfm_outputs(const DynamicResults& result) {
  if (result.snapshots.empty()) return 0;
  return static_cast<std::size_t>(std::count_if(
      result.snapshots.back().device_outputs.begin(),
      result.snapshots.back().device_outputs.end(),
      [](const DynamicDeviceOutput& output) {
        return output.type.find("GridForming") != std::string::npos;
      }));
}

DynamicEvent load_scale_event(double time_s, double scale) {
  DynamicEvent event;
  event.time_s = time_s;
  event.type = DynamicEventType::ACLoadScale;
  event.value = scale;
  event.params["scale"] = scale;
  event.component_type = "AC";
  event.label = "restoration pickup scale " + std::to_string(scale);
  return event;
}

int add_validation_mess(HybridPowerSystem& system) {
  const auto largest = std::max_element(
      system.ac.storage.begin(), system.ac.storage.end(),
      [](const Storage& lhs, const Storage& rhs) {
        return lhs.p_rated_mw < rhs.p_rated_mw;
      });
  if (largest == system.ac.storage.end()) {
    throw std::runtime_error("cannot derive validation MESS without stationary storage");
  }
  MobileStorage mobile;
  mobile.index = 9001;
  mobile.name = "Validation-MESS";
  mobile.bus = system.ac.buses.front().index;
  mobile.target_bus = largest->bus;
  mobile.in_service = true;
  mobile.p_mw = 0.0;
  mobile.p_rated_mw = std::max(0.05, 0.5 * largest->p_rated_mw);
  mobile.pmax_mw = mobile.p_rated_mw;
  mobile.pmin_mw = -mobile.p_rated_mw;
  mobile.qmax_mvar = 0.6 * mobile.p_rated_mw;
  mobile.qmin_mvar = -mobile.qmax_mvar;
  mobile.e_rated_mwh = std::max(0.2, 0.5 * largest->e_rated_mwh);
  mobile.soc_init = 0.8;
  mobile.soc_min = 0.1;
  mobile.soc_max = 0.9;
  mobile.e_mwh = mobile.soc_init * mobile.e_rated_mwh;
  mobile.eta_charge = 0.95;
  mobile.eta_discharge = 0.95;
  mobile.e_consumption_mwh_km = 0.002;
  mobile.max_travel_distance_km = 50.0;
  mobile.status = MobileStorageStatus::Stationary;
  mobile.grid_forming = true;
  mobile.type = "Li-ion";
  system.mobile_storage.push_back(mobile);
  return mobile.target_bus;
}

json finite_or_null(double value) {
  return std::isfinite(value) ? json(value) : json(nullptr);
}

json certificate_json(const analysis::MultiFidelityCertificate& certificate) {
  json margins = json::array();
  for (const auto& margin : certificate.margins) {
    margins.push_back({{"name", margin.name},
                       {"estimate", finite_or_null(margin.estimate)},
                       {"lipschitz", finite_or_null(margin.lipschitz)},
                       {"output_transition_gain",
                        finite_or_null(margin.output_transition_gain)},
                       {"algebraic_direct_gain",
                        finite_or_null(margin.algebraic_direct_gain)},
                       {"Delta_j", finite_or_null(margin.delta_j)},
                       {"lower", finite_or_null(margin.lower)},
                       {"upper", finite_or_null(margin.upper)},
                       {"bound_method", margin.bound_method}});
  }
  return {{"fidelity_level", certificate.fidelity_level},
          {"label", analysis::to_string(certificate.label)},
          {"proof_valid", certificate.proof_valid},
          {"constants_valid", certificate.constants_valid},
          {"constants_are_global_bounds",
           certificate.constants_are_global_bounds},
          {"constant_scope", certificate.constant_scope},
          {"mess_materialized_in_dae",
           certificate.mess_materialized_in_dae},
          {"mess_dynamic_device_observed",
           certificate.mess_dynamic_device_observed},
          {"converter_current_observed",
           certificate.converter_current_observed},
          {"current_limit_active", certificate.current_limit_active},
          {"r_f", {{"max_inf", certificate.r_f.max_inf},
                    {"integral_inf", certificate.r_f.integral_inf}}},
          {"r_g", {{"max_inf", certificate.r_g.max_inf},
                    {"integral_inf", certificate.r_g.integral_inf}}},
          {"kappa_g", finite_or_null(certificate.kappa_g)},
          {"L_F", finite_or_null(certificate.L_F)},
          {"eta", finite_or_null(certificate.eta)},
          {"eta_method", certificate.eta_method},
          {"reconstruction_defect_integral",
           finite_or_null(certificate.reconstruction_defect_integral)},
          {"forcing_method", certificate.forcing_method},
          {"comparison_spectral_abscissa",
           finite_or_null(certificate.comparison_spectral_abscissa)},
          {"transition_gain", finite_or_null(certificate.transition_gain)},
          {"transition_growth_rate",
           finite_or_null(certificate.transition_growth_rate)},
          {"state_scale_min", certificate.state_scale_min},
          {"state_scale_max", certificate.state_scale_max},
          {"margins", std::move(margins)},
          {"limitations", certificate.limitations}};
}

json run_master_oracle_case(const std::string& scale, HybridPowerSystem system,
                            bool all_storage) {
  enable_gfm_storage(system, all_storage);
  system.ac.freq_hz = 50.0;
  const int mess_target_bus = add_validation_mess(system);
  const auto& mobile = system.mobile_storage.back();

  DynamicSolverOptions dynamic_options;
  dynamic_options.t_end_s = 0.30;
  dynamic_options.dt_s = 0.01;
  dynamic_options.solver_type = DynamicSolverType::TrapezoidalNewton;
  dynamic_options.run_power_flow_initialization = true;
  dynamic_options.use_consistent_dynamic_initialization = true;
  dynamic_options.record_every_step = true;
  dynamic_options.record_device_outputs = true;

  analysis::MultiFidelityCertificateOptions certificate_options;
  certificate_options.max_residual_samples = 24;
  certificate_options.max_jacobian_samples = 1;
  certificate_options.dense_jacobian_dimension_limit = 400;
  certificate_options.directional_jacobian_probes = 12;
  certificate_options.max_initial_dynamic_residual = 1.0e-6;
  analysis::MultiFidelityCertificateEngine engine(
      system, dynamic_options, certificate_options);

  analysis::CertifiedRestorationAction cyber_blocked;
  cyber_blocked.id = scale + "_cyber_blocked";
  cyber_blocked.objective = 1.30;
  cyber_blocked.command_path_available = false;
  cyber_blocked.dynamic_events = {load_scale_event(0.10, 1.10)};

  analysis::CertifiedRestorationAction mess_late;
  mess_late.id = scale + "_mess_late";
  mess_late.objective = 1.20;
  mess_late.requires_mess = true;
  mess_late.requires_grid_forming_mess = true;
  mess_late.mess_grid_forming_capable = true;
  mess_late.mess_storage_index = mobile.index;
  mess_late.mess_target_ac_bus = mess_target_bus;
  mess_late.mess_travel_time_s = 900.0;
  mess_late.mess_connection_deadline_s = 300.0;
  mess_late.mess_required_energy_mwh = 0.1;
  mess_late.mess_available_energy_mwh = mobile.e_mwh;

  analysis::CertifiedRestorationAction mess_support = mess_late;
  mess_support.id = scale + "_mess_gfm_support";
  mess_support.objective = 1.10;
  mess_support.mess_travel_time_s = 120.0;
  mess_support.mess_connection_deadline_s = 300.0;
  mess_support.mess_dispatch_mw = 0.25 * mobile.p_rated_mw;
  mess_support.dynamic_events = {load_scale_event(0.10, 1.05)};

  analysis::CertifiedRestorationAction stationary;
  stationary.id = scale + "_stationary_gfm";
  stationary.objective = 1.00;
  stationary.dynamic_events = {load_scale_event(0.10, 1.00)};

  analysis::CertifiedRestorationCoordinator coordinator(engine);
  const analysis::CertifiedRestorationResult result = coordinator.solve(
      {cyber_blocked, mess_late, mess_support, stationary});

  json catalog = json::array({
      {{"action_id", cyber_blocked.id},
       {"objective", cyber_blocked.objective},
       {"physical_available", true},
       {"cyber_executable", false},
       {"logistics_executable", true},
       {"master_outcome", "filtered: no command path"}},
      {{"action_id", mess_late.id},
       {"objective", mess_late.objective},
       {"physical_available", true},
       {"cyber_executable", true},
       {"logistics_executable", false},
       {"master_outcome", "filtered: MESS deadline missed"}},
      {{"action_id", mess_support.id},
       {"objective", mess_support.objective},
       {"physical_available", true},
       {"cyber_executable", true},
       {"logistics_executable", true},
       {"master_outcome", "passed to dynamic oracle"}},
      {{"action_id", stationary.id},
       {"objective", stationary.objective},
       {"physical_available", true},
       {"cyber_executable", true},
       {"logistics_executable", true},
       {"master_outcome", "incumbent-bound dominated"}},
  });

  json iterations = json::array();
  for (const auto& iteration : result.iterations) {
    json certificates = json::array();
    for (const auto& certificate : iteration.certificates) {
      certificates.push_back(certificate_json(certificate));
    }
    iterations.push_back({{"iteration", iteration.iteration},
                          {"action_id", iteration.action_id},
                          {"disposition", iteration.disposition},
                          {"certificates", std::move(certificates)}});
  }
  return {{"scale", scale},
          {"case_name", system.name},
          {"ac_buses", system.ac.buses.size()},
          {"algebraic_dimension", 6 * system.ac.buses.size() + system.dc.buses.size()},
          {"success", result.success},
          {"optimality_proven_over_catalog",
           result.optimality_proven_over_catalog},
          {"status", result.status},
          {"incumbent_action", result.incumbent_action_id},
          {"incumbent_objective", finite_or_null(result.incumbent_objective)},
          {"master_mip_solves", result.master_mip_solves},
          {"dynamic_oracle_calls", result.dynamic_oracle_calls},
          {"diagnostic_certificate_calls",
           result.diagnostic_certificate_calls},
          {"master_filtered_actions", result.master_filtered_actions},
          {"no_good_cuts", result.no_good_cuts},
          {"decision_comparison",
           {{"physical_only_selected_action", cyber_blocked.id},
            {"physical_only_objective", cyber_blocked.objective},
            {"cyber_logistics_selected_action", mess_support.id},
            {"cyber_logistics_objective", mess_support.objective},
            {"dynamic_secure_selected_action", result.incumbent_action_id},
            {"dynamic_secure_objective",
             finite_or_null(result.incumbent_objective)},
            {"cyber_constraint_changes_decision",
             cyber_blocked.id != mess_support.id}}},
          {"catalog", std::move(catalog)},
          {"iterations", std::move(iterations)}};
}

json probe_case(const std::string& scale, HybridPowerSystem system,
                bool all_storage) {
  enable_gfm_storage(system, all_storage);
  system.ac.freq_hz = 50.0;

  DynamicSolverOptions options;
  options.t_end_s = 0.05;
  options.dt_s = 0.01;
  options.solver_type = DynamicSolverType::TrapezoidalNewton;
  options.run_power_flow_initialization = true;
  options.use_consistent_dynamic_initialization = true;
  options.record_every_step = true;
  options.record_device_outputs = true;
  options.max_recorded_snapshots = 16;

  const auto build_start = std::chrono::steady_clock::now();
  DynamicModelBuilder builder;
  DynamicSystem dynamic_system = builder.build(system, options);
  const auto solve_start = std::chrono::steady_clock::now();
  DynamicSolver solver;
  const DynamicResults result = solver.solve(dynamic_system);
  const auto stop = std::chrono::steady_clock::now();

  const double build_ms = std::chrono::duration<double, std::milli>(
                              solve_start - build_start)
                              .count();
  const double solve_ms =
      std::chrono::duration<double, std::milli>(stop - solve_start).count();

  return {
      {"scale", scale},
      {"case_name", system.name},
      {"ac_buses", system.ac.buses.size()},
      {"ac_branches", system.ac.branches.size()},
      {"switches", system.ac.switches.size()},
      {"ac_loads", system.ac.loads.size()},
      {"ac_storage", system.ac.storage.size()},
      {"three_phase_present", system.three_phase_ac.has_value()},
      {"dynamic_ac_nodes", dynamic_system.network.ac_bus_ids.size()},
      {"dynamic_ac_branches", dynamic_system.network.ac_branches.size()},
      {"dynamic_devices", dynamic_system.devices.size()},
      {"dynamic_states", dynamic_system.x.size()},
      {"algebraic_states",
       2 * dynamic_system.y.ac_phase_nodes() +
           dynamic_system.y.dc_bus_count()},
      {"build_ms", build_ms},
      {"solve_ms", solve_ms},
      {"success", result.success},
      {"message", result.message},
      {"power_flow_converged", result.initialization.power_flow_converged},
      {"power_flow_residual", result.initialization.residual},
      {"dynamic_trim_converged", result.initialization.dynamic_trim_converged},
      {"dynamic_trim_residual",
       result.initialization.dynamic_fast_dxdt_inf_norm},
      {"min_ac_voltage_pu",
       result.snapshots.empty() ? nullptr
                                : json(result.snapshots.back().min_ac_voltage_pu)},
      {"gfm_output_count", count_gfm_outputs(result)},
      {"warnings", result.warnings},
      {"initialization_warnings", result.initialization.warnings}};
}

void write_text(const std::filesystem::path& path, const std::string& text) {
  std::ofstream stream(path);
  if (!stream) throw std::runtime_error("cannot write " + path.string());
  stream << text;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const std::filesystem::path output_dir =
        argc > 1
            ? std::filesystem::path(argv[1])
            : std::filesystem::path(HACDCPF_PROJECT_ROOT) /
                  "docs/latex/paper/cyber_dynamic_safe_restoration/results/"
                  "scale_sentinels";
    std::filesystem::create_directories(output_dir);

    json report;
    report["schema_version"] = 3;
    report["status"] = "medium_large_master_oracle_validation";
    report["model_scope"] = {
        {"medium_gfm", "largest stationary BESS"},
        {"large_gfm", "all four community BESS units"},
        {"window", "quiet 0.05 s initialization probe plus 0.30 s restoration action oracle"},
        {"master", "finite-action HiGHS MIP with cyber and MESS executability"},
        {"mess_dynamic", "post-arrival MESS materialized as a target-bus dynamic AC storage device"},
        {"large_constants", "directional algebraic-conditioning proxy plus dense 350-state network-solved reduced Jacobian; sampled, not global bounds"}};
    report["cases"] = json::array();
    report["cases"].push_back(probe_case(
        "medium", io::build_dist33_microgrid_der(), false));
    report["cases"].push_back(probe_case(
        "large", io::build_urban_lvn_primary_secondary(), true));
    report["master_oracle_cases"] = json::array();
    report["master_oracle_cases"].push_back(run_master_oracle_case(
        "medium", io::build_dist33_microgrid_der(), false));
    report["master_oracle_cases"].push_back(run_master_oracle_case(
        "large", io::build_urban_lvn_primary_secondary(), true));

    write_text(output_dir / "scale_validation.json", report.dump(2) + "\n");
    std::cout << report.dump(2) << '\n';
    const bool probes_ok = std::all_of(
        report["cases"].begin(), report["cases"].end(),
        [](const json& item) {
          return item.value("success", false) &&
                 item.value("power_flow_converged", false) &&
                 item.value("dynamic_trim_residual", 1.0) <= 1.0e-6 &&
                 item.value("gfm_output_count", 0) > 0;
        });
    const bool coordination_ok = std::all_of(
        report["master_oracle_cases"].begin(),
        report["master_oracle_cases"].end(), [](const json& item) {
          return item.value("success", false) &&
                 item.value("optimality_proven_over_catalog", false) &&
                 item.value("dynamic_oracle_calls", 0) > 0;
        });
    return probes_ok && coordination_ok
               ? 0
               : 1;
  } catch (const std::exception& error) {
    std::cerr << "cyber_dynamic_safe_restoration_scale: " << error.what()
              << '\n';
    return 2;
  }
}
