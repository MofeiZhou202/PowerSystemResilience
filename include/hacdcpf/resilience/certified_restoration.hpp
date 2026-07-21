#pragma once

#include <limits>
#include <string>
#include <vector>

#include "hacdcpf/dynamics/DynamicEvent.hpp"
#include "hacdcpf/dynamics/DynamicResults.hpp"
#include "hacdcpf/dynamics/DynamicSolverOptions.hpp"
#include "hacdcpf/model/hybrid_power_system.hpp"

namespace hacdcpf::analysis {

enum class CertificateLabel { Safe, Unsafe, Unresolved, Failed };

const char* to_string(CertificateLabel label) noexcept;

struct CertifiedRestorationAction {
  std::string id;
  std::string description;
  double objective{0.0};
  int tie_break_priority{0};
  std::vector<dynamics::DynamicEvent> dynamic_events;

  // Cyber command-chain contract used by the master executability constraints.
  bool command_path_available{true};
  bool authorization_valid{true};
  bool acknowledgement_available{true};

  // MESS contract. Times are relative to the restoration decision instant.
  bool requires_mess{false};
  bool requires_grid_forming_mess{false};
  double mess_travel_time_s{0.0};
  double mess_connection_deadline_s{std::numeric_limits<double>::infinity()};
  double mess_required_energy_mwh{0.0};
  double mess_available_energy_mwh{0.0};
  bool mess_grid_forming_capable{false};
  int mess_storage_index{0};
  int mess_target_ac_bus{0};
  double mess_dispatch_mw{0.0};
  double mess_dispatch_mvar{0.0};
};

struct ExecutabilityReport {
  bool cyber_executable{false};
  bool mess_executable{false};
  bool executable{false};
  std::vector<std::string> failed_constraints;
};

struct ResidualRecord {
  double max_inf{0.0};
  double integral_inf{0.0};
  int samples{0};
};

struct MarginCertificate {
  std::string name;
  double estimate{0.0};
  double lipschitz{0.0};
  double output_transition_gain{std::numeric_limits<double>::infinity()};
  double algebraic_direct_gain{0.0};
  double delta_j{0.0};
  double lower{0.0};
  double upper{0.0};
  std::string bound_method;
};

struct MultiFidelityCertificate {
  std::string action_id;
  int fidelity_level{0};
  std::string fidelity_scope;
  CertificateLabel label{CertificateLabel::Failed};
  bool simulation_success{false};
  bool proof_valid{false};
  bool constants_valid{false};
  bool constants_are_global_bounds{false};
  bool mess_materialized_in_dae{false};
  bool mess_dynamic_device_observed{false};
  bool converter_current_observed{false};
  bool current_limit_active{false};
  ResidualRecord r_f;
  ResidualRecord r_g;
  double kappa_g{std::numeric_limits<double>::infinity()};
  double L_F{std::numeric_limits<double>::infinity()};
  double initial_error{0.0};
  double numerical_error{0.0};
  double reconstruction_defect_integral{
      std::numeric_limits<double>::infinity()};
  std::string forcing_method;
  double eta{std::numeric_limits<double>::infinity()};
  std::string eta_method;
  double comparison_spectral_abscissa{
      std::numeric_limits<double>::quiet_NaN()};
  double transition_gain{std::numeric_limits<double>::quiet_NaN()};
  double transition_growth_rate{
      std::numeric_limits<double>::quiet_NaN()};
  double state_scale_min{1.0};
  double state_scale_max{1.0};
  std::vector<MarginCertificate> margins;
  std::string constant_scope;
  std::vector<std::string> limitations;
  dynamics::DynamicResults trajectory;
};

struct MultiFidelityCertificateOptions {
  double min_frequency_hz{49.0};
  double max_rocof_hz_s{1.0};
  double min_ac_voltage_pu{0.90};
  bool require_converter_current_observation{true};
  double jacobian_tube_factor{1.25};
  double finite_difference_relative_step{1.0e-6};
  int max_residual_samples{80};
  int max_jacobian_samples{3};
  int dense_jacobian_dimension_limit{400};
  int directional_jacobian_probes{12};
  int transition_time_probes{8};
  double state_scale_floor{1.0e-3};
  double level1_step_multiplier{4.0};
  double level2_step_multiplier{2.0};
  double max_initial_dynamic_residual{1.0e-6};
};

class MultiFidelityCertificateEngine {
 public:
  MultiFidelityCertificateEngine(
      HybridPowerSystem system, dynamics::DynamicSolverOptions oracle_options,
      MultiFidelityCertificateOptions certificate_options = {});

  [[nodiscard]] ExecutabilityReport checkExecutability(
      const CertifiedRestorationAction& action) const;

  /// Levels 1 and 2 are coarse/fine sampled-Jacobian diagnostics until their
  /// tube constants are replaced by verified enclosures. Level 3 is the
  /// configured full-DAE threshold oracle. Its proof_valid scope is the
  /// simulated scenario, thresholds, models, and horizon, not global stability.
  [[nodiscard]] MultiFidelityCertificate evaluate(
      const CertifiedRestorationAction& action, int fidelity_level) const;

 private:
  HybridPowerSystem system_;
  dynamics::DynamicSolverOptions oracle_options_;
  MultiFidelityCertificateOptions certificate_options_;
};

struct MasterIterationRecord {
  int iteration{0};
  std::string action_id;
  double master_objective{0.0};
  double master_upper_bound{0.0};
  ExecutabilityReport executability;
  std::vector<MultiFidelityCertificate> certificates;
  std::string disposition;
};

struct CertifiedRestorationResult {
  bool success{false};
  bool optimality_proven_over_catalog{false};
  std::string status;
  std::string model_scope{"finite-action restoration master MIP + cyber/MESS executability + dynamic DAE oracle"};
  std::string incumbent_action_id;
  double incumbent_objective{-std::numeric_limits<double>::infinity()};
  int master_mip_solves{0};
  int dynamic_oracle_calls{0};
  int diagnostic_certificate_calls{0};
  std::vector<std::string> master_filtered_actions;
  std::vector<std::string> no_good_cuts;
  std::vector<MasterIterationRecord> iterations;
};

class CertifiedRestorationCoordinator {
 public:
  explicit CertifiedRestorationCoordinator(
      const MultiFidelityCertificateEngine& certificate_engine)
      : certificate_engine_(certificate_engine) {}

  [[nodiscard]] CertifiedRestorationResult solve(
      const std::vector<CertifiedRestorationAction>& actions) const;

 private:
  const MultiFidelityCertificateEngine& certificate_engine_;
};

}  // namespace hacdcpf::analysis
