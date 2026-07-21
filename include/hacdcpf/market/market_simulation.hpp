#pragma once

#include <string>
#include <vector>

#include "hacdcpf/model/hybrid_power_system.hpp"
#include "hacdcpf/power_flow/power_flow_options.hpp"
#include "hacdcpf/power_flow/power_flow_result.hpp"
#include "hacdcpf/time_series/time_series_pf.hpp"

namespace hacdcpf::market {

enum class BehaviorPolicyType {
  CostBased,
  FixedMarkup,
  CapacityWithholding,
  MarkupAndWithholding
};

/// Parametric participant policy for the first behavioural market layer.
/// Fractions use engineering convention: 0.20 means +20% price markup or 20%
/// of flexible capacity withheld.  Commitment markup applies to no-load and
/// startup/shutdown offers separately from incremental energy markup.
struct ParticipantBehavior {
  BehaviorPolicyType type{BehaviorPolicyType::CostBased};
  double energy_markup_fraction{0.0};
  double capacity_withholding_fraction{0.0};
  double commitment_markup_fraction{0.0};
  double upward_reserve_price_per_mwh{0.0};
};

/// A market participant may own multiple physical generators.  Positions are
/// resolved against HybridPowerSystem::ac.generators and audited on submission.
struct MarketParticipant {
  std::string participant_id;
  std::string participant_name;
  std::vector<int> generator_positions;
  ParticipantBehavior behavior;
};

/// A price/quantity block submitted for one market product.
struct OfferSegment {
  double quantity_mw{0.0};
  double price_per_mwh{0.0};
};

/// Cost-based offer for one in-service AC generator.
///
/// `generator_position` is the position in HybridPowerSystem::ac.generators;
/// `generator_index` is the authored component identity.  Keeping both avoids
/// treating a vector position as a durable market identifier.
struct GeneratorOffer {
  std::string participant_id;
  int generator_position{-1};
  int generator_index{0};
  std::string generator_name;
  double minimum_output_mw{0.0};
  double physical_maximum_output_mw{0.0};
  double offered_maximum_output_mw{0.0};
  double minimum_output_cost_per_hour{0.0};
  double no_load_price_per_hour{0.0};
  double startup_price{0.0};
  double shutdown_price{0.0};
  std::vector<OfferSegment> energy_segments;
  double upward_reserve_price_per_mwh{0.0};
};

struct BehaviorAction {
  std::string participant_id;
  int generator_position{-1};
  int generator_index{0};
  double energy_markup_fraction{0.0};
  double capacity_withholding_fraction{0.0};
  double physical_capacity_mw{0.0};
  double offered_capacity_mw{0.0};
  double withheld_capacity_mw{0.0};
};

struct OfferSubmission {
  std::vector<MarketParticipant> participants;
  std::vector<GeneratorOffer> offers;
  std::vector<BehaviorAction> actions;
  std::vector<std::string> warnings;
};

/// Fixed-commitment SCED and price result for one interval.
struct PricingPeriod {
  bool converged{false};
  std::string status;
  std::vector<double> generator_dispatch_mw;  ///< full authored generator order
  std::vector<double> upward_reserve_mw;      ///< full authored generator order
  std::vector<double> lmp_per_mwh;            ///< authored AC bus order
  std::vector<double> branch_flow_mw;         ///< authored AC branch order
  std::vector<double> load_shedding_mw;       ///< authored AC bus order
  std::vector<double> exogenous_curtailment_mw; ///< authored AC bus order
  std::vector<double> dc_lmp_per_mwh;          ///< authored DC bus order
  std::vector<double> dc_bus_voltage_pu;       ///< authored DC bus order
  std::vector<double> dc_branch_flow_mw;       ///< authored DC branch order
  std::vector<double> dc_load_shedding_mw;     ///< authored DC bus order
  std::vector<double> dc_exogenous_curtailment_mw; ///< authored DC bus order
  std::vector<double> vsc_ac_injection_mw;     ///< authored VSC order; + into AC
  std::vector<double> vsc_dc_injection_mw;     ///< authored VSC order; + into DC
  std::vector<double> vsc_loss_mw;             ///< authored VSC order
  std::vector<double> dcdc_input_withdrawal_mw; ///< authored DC/DC order
  std::vector<double> dcdc_output_injection_mw; ///< authored DC/DC order
  std::vector<double> dcdc_loss_mw;             ///< authored DC/DC order
  std::vector<double> legacy_dc_storage_dispatch_mw; ///< dc.storage order; + discharge
  std::vector<double> legacy_dc_storage_soc_mwh; ///< dc.storage order; end-period MWh
  std::vector<double> dc_storage_dispatch_mw;   ///< dc.dc_storage order; + discharge
  std::vector<double> dc_storage_soc_mwh;       ///< dc.dc_storage order; end-period MWh
  double reserve_requirement_mw{0.0};
  double upward_reserve_price_per_mwh{0.0};
  double gross_demand_mw{0.0};
  double gross_dc_demand_mw{0.0};
  double objective{0.0};
};

/// Machine-readable fidelity declaration for the commercial market model.
/// The nonlinear certification layer reports its own ConverterModelScope.
struct MarketModelScope {
  std::string model_scope{"ac-dc-linear-v1"};
  bool ac_network_modelled{true};
  bool dc_network_modelled{false};
  bool dc_voltage_linearized{false};
  bool dc_branch_losses_modelled{false};
  bool vsc_bidirectional_efficiency_modelled{false};
  bool vsc_fixed_and_quadratic_losses_modelled{false};
  bool dcdc_bidirectional_efficiency_modelled{false};
  bool dc_storage_optimized{false};
  bool dc_storage_intertemporal_modelled{false};
  bool hybrid_n1_modelled{false};
  bool full_component_n1_modelled{false};
  std::string n1_recourse_policy{"none"};
  bool energy_prices_valid{false};
  std::vector<std::string> limitations;
};

/// Model-size estimates and wall-clock stage timings for scalability audits.
struct MarketPerformanceProfile {
  int periods{0};
  int active_generators{0};
  int active_ac_branches{0};
  int active_dc_branches{0};
  int active_vsc_converters{0};
  int active_dcdc_converters{0};
  int optimized_dc_storages{0};
  long long estimated_scuc_variables{0};
  long long estimated_scuc_binary_variables{0};
  long long estimated_sced_variables{0};
  long long estimated_lodf_dense_bytes{0};
  long long estimated_lodf_sparse_bytes{0};
  int lodf_computed_columns{0};
  int component_contingency_solves{0};
  bool component_n1_parallel_requested{false};
  bool component_n1_parallel_effective{false};
  int component_n1_parallel_workers{1};
  std::string scuc_solver_name;
  std::string pricing_solver_name;
  bool pricing_solver_fallback_used{false};
  bool pricing_large_model_direct_highs_used{false};
  bool scuc_mip_start_provided{false};
  bool scuc_structure_hint_provided{false};
  bool scuc_branching_priorities_provided{false};
  bool scuc_structured_branching_used{false};
  bool scuc_mip_gap_target_met{false};
  bool scuc_optimality_proven{false};
  double scuc_mip_gap{0.0};
  std::string scuc_solver_status;
  double scuc_warm_start_generation_sec{0.0};
  bool scuc_cross_round_solver_state_reuse_enabled{false};
  bool scuc_cross_round_solver_state_reuse_used{false};
  int scuc_cross_round_solver_state_reuse_rounds{0};
  bool scuc_root_cuts_reused{false};
  int scuc_root_cuts_reused_count{0};
  bool scuc_root_basis_reused{false};
  bool scuc_pseudocosts_reused{false};
  bool scuc_search_tree_rebuilt{false};
  bool scuc_network_constraint_generation_run{false};
  bool scuc_network_constraint_generation_converged{false};
  int scuc_network_constraint_generation_iterations{0};
  int scuc_network_constraint_candidates{0};
  int scuc_network_constraints_activated{0};
  int scuc_network_constraint_remaining_violations{0};
  double scuc_network_constraint_worst_violation_mw{0.0};
  double offer_submission_sec{0.0};
  double scuc_sec{0.0};
  double base_sced_sec{0.0};
  double lodf_build_sec{0.0};
  double n1_cut_loop_sec{0.0};
  double component_n1_sec{0.0};
  double nonlinear_validation_sec{0.0};
  double settlement_sec{0.0};
  double total_sec{0.0};
};

/// One physical limit that failed during post-clearing nonlinear validation.
/// Positions use authored vector order while indices are stable model IDs.
struct MarketConstraintViolation {
  std::string category;       ///< bus_voltage / branch_thermal / generator_active
  std::string component_type;
  int component_position{-1};
  int component_index{0};
  std::string component_name;
  int bus{0};
  int from_bus{0};
  int to_bus{0};
  double actual{0.0};
  double lower_limit{0.0};
  double upper_limit{0.0};
  double violation{0.0};
  std::string unit;
};

/// AC power-flow certification of the commercial SCED dispatch.
struct ACValidationPeriod {
  bool converged{false};
  bool secure{false};
  double residual{0.0};
  double total_branch_loss_mw{0.0};
  double maximum_branch_loading_percent{0.0};
  double maximum_voltage_violation_pu{0.0};
  double maximum_branch_overload_mva{0.0};
  double maximum_generator_active_violation_mw{0.0};
  double maximum_dc_voltage_violation_pu{0.0};
  double maximum_dc_branch_overload_mw{0.0};
  double maximum_vsc_schedule_deviation_mw{0.0};
  double maximum_dcdc_schedule_deviation_mw{0.0};
  double slack_adjustment_mw{0.0};
  int slack_generator_position{-1};
  double maximum_p_mismatch_pu{0.0};
  double maximum_q_mismatch_pu{0.0};
  std::vector<std::string> solver_warnings;
  std::vector<MarketConstraintViolation> violations;
  ConverterModelScope converter_model_scope;
  std::string status;
};

/// Per-generator day-ahead settlement.  True/as-bid cost and submitted offer
/// are deliberately separate so later strategic behaviour does not corrupt
/// profit and markup accounting.
struct GeneratorSettlement {
  int generator_position{-1};
  int generator_index{0};
  std::string generator_name;
  double energy_mwh{0.0};
  double reserve_mwh{0.0};
  double energy_revenue{0.0};
  double reserve_revenue{0.0};
  double true_cost{0.0};
  double as_bid_cost{0.0};
  double offered_cost_markup{0.0};
  double uplift{0.0};
  double profit_after_uplift{0.0};
};

struct DCStorageSettlement {
  std::string component_type;  ///< legacy_dc_storage / dc_storage
  int component_position{-1};
  int component_index{0};
  std::string component_name;
  int dc_bus{0};
  double charge_mwh{0.0};
  double discharge_mwh{0.0};
  double net_injection_mwh{0.0};
  double energy_revenue{0.0};
  double as_bid_cost{0.0};
  double profit{0.0};
  double initial_soc_mwh{0.0};
  double terminal_soc_mwh{0.0};
};

struct ParticipantSettlement {
  std::string participant_id;
  std::string participant_name;
  std::vector<int> generator_positions;
  double physical_capacity_mw{0.0};
  double offered_capacity_mw{0.0};
  double withheld_capacity_mw{0.0};
  double energy_mwh{0.0};
  double reserve_mwh{0.0};
  double market_revenue{0.0};
  double true_cost{0.0};
  double as_bid_cost{0.0};
  double uplift{0.0};
  double profit_after_uplift{0.0};
};

struct MarketPowerMetrics {
  double output_hhi{0.0};                 ///< 0..10000 using energy shares
  double revenue_hhi{0.0};                ///< 0..10000 using revenue shares
  double top3_output_share_percent{0.0};
  double maximum_output_share_percent{0.0};
  std::string maximum_output_participant;
  double maximum_profit{0.0};
  std::string maximum_profit_participant;
  double total_withheld_capacity_mw{0.0};
  double average_offer_markup_fraction{0.0};
};

struct N1Violation {
  int period{0};
  int outage_branch_position{-1};
  int outage_branch_index{0};
  int monitored_branch_position{-1};
  int monitored_branch_index{0};
  double base_flow_mw{0.0};
  double post_contingency_flow_mw{0.0};
  double emergency_rating_mw{0.0};
  double overload_mw{0.0};
};

struct N1Iteration {
  int iteration{0};
  int violations{0};
  int cuts_added{0};
  int cumulative_cuts{0};
  double worst_overload_mw{0.0};
};

struct ACContingencyCheck {
  int period{0};
  int outage_branch_position{-1};
  int outage_branch_index{0};
  std::string outage_branch_name;
  int outage_from_bus{0};
  int outage_to_bus{0};
  bool converged{false};
  bool secure{false};
  double maximum_voltage_violation_pu{0.0};
  double maximum_branch_overload_mva{0.0};
  std::vector<MarketConstraintViolation> violations;
  std::string status;
};

/// Corrective fixed-commitment SCED check for one authored N-1 component.
/// AC branches additionally retain the preventive LODF cut workflow above.
struct ComponentContingencyCheck {
  std::string component_type;
  int component_position{-1};
  int component_index{0};
  std::string component_name;
  int bus{0};
  int from_bus{0};
  int to_bus{0};
  bool sced_converged{false};
  bool secure{false};
  double incremental_load_shedding_mwh{0.0};
  double total_load_shedding_mwh{0.0};
  double total_exogenous_curtailment_mwh{0.0};
  double objective{0.0};
  std::string status;
};

/// In-service authored assets which the current market formulation cannot
/// represent.  Returning them explicitly avoids a generic scope failure.
struct UnsupportedMarketAsset {
  std::string component_type;
  int component_position{-1};
  int component_index{0};
  std::string component_name;
  int bus{0};
  int from_bus{0};
  int to_bus{0};
  std::string reason;
};

struct SecurityResult {
  bool enabled{false};
  bool lodf_available{false};
  bool dc_n1_secured{false};  ///< legacy name: preventive AC-branch LODF secure
  bool ac_contingency_validation_run{false};
  bool ac_contingencies_secure{false};
  bool full_component_validation_run{false};
  bool full_component_contingencies_secure{false};
  int candidate_contingencies{0};
  int full_component_candidate_contingencies{0};
  int iterations{0};
  int cuts_added{0};
  int initial_violations{0};
  int final_violations{0};
  double initial_worst_overload_mw{0.0};
  double final_worst_overload_mw{0.0};
  double baseline_pricing_objective{0.0};
  double secured_pricing_objective{0.0};
  double preventive_redispatch_cost{0.0};
  double incremental_security_uplift{0.0};
  double customer_payment_impact{0.0};
  std::vector<int> skipped_islanding_branch_positions;
  std::vector<N1Iteration> trajectory;
  std::vector<N1Violation> remaining_violations;
  std::vector<ACContingencyCheck> ac_checks;
  std::vector<ComponentContingencyCheck> component_checks;
  std::vector<std::string> warnings;
};

/// Auditable day-ahead cash-flow ledger.
struct SettlementLedger {
  double customer_energy_payment{0.0};
  double customer_ac_energy_payment{0.0};
  double customer_dc_energy_payment{0.0};
  double customer_reserve_charge{0.0};
  double customer_uplift_charge{0.0};
  double customer_total_payment{0.0};
  double resource_energy_revenue{0.0};
  double resource_ac_energy_revenue{0.0};
  double resource_dc_energy_revenue{0.0};
  double resource_reserve_revenue{0.0};
  double resource_uplift_revenue{0.0};
  double resource_total_revenue{0.0};
  double congestion_rent{0.0};
  double cashflow_residual{0.0};
};

struct MarketOptions {
  /// Options for the commitment stage.  The market runner forces fixed
  /// commitment pricing to use the schedule returned by this stage.
  TimeSeriesPFOptions uc_options;
  PowerFlowOptions ac_validation_options;

  int energy_offer_segments{8};
  double upward_reserve_fraction{0.05};
  double value_of_lost_load_per_mwh{10000.0};
  double exogenous_curtailment_penalty_per_mwh{0.0};
  int pricing_native_max_variables{5000}; ///< 0 = route every pricing LP to HiGHS
  bool structured_scuc_branching{true};
  int structured_scuc_min_binary_variables{1000};
  double scuc_mip_relative_gap{1e-2};
  double scuc_time_limit_sec{120.0};
  int scuc_max_nodes{50000};
  bool enable_scuc_cross_round_solver_state_reuse{true};
  bool enable_scuc_network_constraint_generation{true};
  int scuc_network_constraint_generation_min_candidates{10000};
  int scuc_network_constraint_generation_max_iterations{8};
  int scuc_network_constraint_generation_max_new_per_iteration{0};
  double scuc_network_constraint_generation_tolerance_mw{1e-5};
  bool optimize_dc_storage{true};
  bool enforce_terminal_dc_storage_soc{true};
  bool enable_network_constraints{true};
  bool run_ac_validation{true};
  double ac_validation_voltage_tolerance_pu{1e-5};
  double ac_validation_thermal_tolerance_mva{1e-4};
  double ac_validation_generator_tolerance_mw{1e-5};
  bool verbose{false};

  // Optional real-time dispatch envelopes in full authored generator order.
  // Non-finite entries mean "not constrained". These non-owning schedules are
  // consumed only by the fixed-commitment SCED pricing layer.
  const std::vector<std::vector<double>>*
      minimum_dispatch_schedule_mw{nullptr};
  const std::vector<std::vector<double>>*
      fixed_dispatch_schedule_mw{nullptr};

  // Preventive branch N-1 security for the fixed-commitment SCED.  Violated
  // LODF rows are added iteratively and therefore become price-eligible.
  bool enable_n1_security{false};
  int n1_max_iterations{8};
  int n1_max_cuts_per_iteration{200};
  int n1_max_contingencies{0};       ///< 0 = every active covered component
  double n1_emergency_rating_multiplier{1.0};
  double n1_violation_tolerance_mw{1e-5};
  bool parallel_component_n1{true};
  int component_n1_parallel_workers{4}; ///< 0 = hardware concurrency

  // Nonlinear certification of the final preventive dispatch.  This does not
  // silently repair a failed contingency: failures remain explicit results.
  bool run_ac_contingency_validation{false};
  int max_ac_contingencies{3};       ///< 0 = every valid N-1 branch
  double ac_contingency_voltage_tolerance_pu{1e-5};
  double ac_contingency_thermal_tolerance_mva{1e-4};

  /// Empty means one independent cost-based participant per active generator.
  /// Explicit ownership may group several generators under one participant.
  std::vector<MarketParticipant> participants;
};

struct MarketResult {
  bool feasible{false};
  std::string status;
  std::vector<std::string> warnings;
  std::vector<UnsupportedMarketAsset> unsupported_assets;

  std::vector<MarketParticipant> participants;
  std::vector<BehaviorAction> behavior_actions;
  std::vector<GeneratorOffer> offers;
  UCSchedule commitment;
  std::vector<PricingPeriod> pricing;
  std::vector<ACValidationPeriod> ac_validation;
  std::vector<PowerFlowResult> ac_power_flow_results;
  std::vector<GeneratorSettlement> generator_settlement;
  std::vector<DCStorageSettlement> dc_storage_settlement;
  std::vector<ParticipantSettlement> participant_settlement;
  MarketPowerMetrics market_power;
  SecurityResult security;
  SettlementLedger settlement;
  MarketModelScope model_scope;
  MarketPerformanceProfile performance;

  double commitment_cost{0.0};
  double pricing_objective{0.0};
  int num_pricing_converged{0};
  int num_ac_converged{0};
  int num_ac_secure{0};
};

/// Ex-post reserve activation, performance, and imbalance settlement.  All
/// prices are non-negative currency/MWh and tolerance bands are fractions of
/// the corresponding day-ahead schedule.  Performance factors use full
/// authored generator order; an omitted entry is treated as perfect delivery.
struct AncillaryServiceOptions {
  bool enabled{false};
  double generator_imbalance_tolerance_fraction{0.02};
  double load_imbalance_tolerance_fraction{0.02};
  double generator_imbalance_penalty_per_mwh{50.0};
  double load_imbalance_penalty_per_mwh{50.0};
  double reserve_performance_payment_per_mwh{10.0};
  double reserve_nonperformance_penalty_per_mwh{100.0};
  std::vector<double> reserve_performance_factor_by_generator;
};

/// Real-time fixed-commitment dispatch options.  `market_options` controls the
/// same network, N-1 and AC certification layers as the day-ahead market; the
/// runner pins its commitment to the accepted day-ahead schedule.
struct RealTimeMarketOptions {
  MarketOptions market_options;
  AncillaryServiceOptions ancillary_services;
};

struct RealTimePeriod {
  int period{0};
  double day_ahead_demand_mw{0.0};
  double realized_demand_mw{0.0};
  double demand_deviation_mw{0.0};
  double absolute_generator_deviation_mw{0.0};
  double average_day_ahead_lmp_per_mwh{0.0};
  double average_real_time_lmp_per_mwh{0.0};
  double customer_deviation_payment{0.0};
  double resource_deviation_revenue{0.0};
  double dc_storage_deviation_revenue{0.0};
  double deviation_congestion_rent{0.0};
  double reserve_activation_requirement_mw{0.0};
  double reserve_instruction_mw{0.0};
  double reserve_delivered_mw{0.0};
  double reserve_shortfall_mw{0.0};
  double load_penalized_imbalance_mwh{0.0};
  double load_imbalance_penalty{0.0};
};

struct GeneratorDeviationSettlement {
  std::string participant_id;
  int generator_position{-1};
  int generator_index{0};
  std::string generator_name;
  double day_ahead_energy_mwh{0.0};
  double real_time_energy_mwh{0.0};
  double real_time_dispatch_instruction_mwh{0.0};
  double actual_output_deviation_mwh{0.0};
  double deviation_mwh{0.0};
  double day_ahead_energy_revenue{0.0};
  double real_time_deviation_revenue{0.0};
  double day_ahead_reserve_revenue{0.0};
  double day_ahead_uplift{0.0};
  double instructed_reserve_mwh{0.0};
  double delivered_reserve_mwh{0.0};
  double reserve_shortfall_mwh{0.0};
  double reserve_performance_ratio{1.0};
  double reserve_performance_payment{0.0};
  double reserve_nonperformance_charge{0.0};
  double penalized_imbalance_mwh{0.0};
  double imbalance_charge{0.0};
  double net_ancillary_adjustment{0.0};
  double two_settlement_revenue{0.0};
  double actual_true_cost{0.0};
  double profit_after_two_settlement{0.0};
};

struct ParticipantDeviationSettlement {
  std::string participant_id;
  std::string participant_name;
  std::vector<int> generator_positions;
  double day_ahead_energy_mwh{0.0};
  double real_time_energy_mwh{0.0};
  double real_time_dispatch_instruction_mwh{0.0};
  double actual_output_deviation_mwh{0.0};
  double deviation_mwh{0.0};
  double day_ahead_market_revenue{0.0};
  double real_time_deviation_revenue{0.0};
  double instructed_reserve_mwh{0.0};
  double delivered_reserve_mwh{0.0};
  double reserve_shortfall_mwh{0.0};
  double reserve_performance_ratio{1.0};
  double reserve_performance_payment{0.0};
  double reserve_nonperformance_charge{0.0};
  double penalized_imbalance_mwh{0.0};
  double imbalance_charge{0.0};
  double net_ancillary_adjustment{0.0};
  double two_settlement_revenue{0.0};
  double actual_true_cost{0.0};
  double profit_after_two_settlement{0.0};
};

/// Incremental real-time leg and combined day-ahead/real-time cash ledger.
struct DeviationSettlementLedger {
  double customer_day_ahead_payment{0.0};
  double customer_real_time_deviation_payment{0.0};
  double customer_imbalance_penalty{0.0};
  double customer_two_settlement_payment{0.0};
  double resource_day_ahead_revenue{0.0};
  double resource_real_time_deviation_revenue{0.0};
  double resource_reserve_performance_payment{0.0};
  double resource_generator_imbalance_charge{0.0};
  double resource_reserve_nonperformance_charge{0.0};
  double resource_two_settlement_revenue{0.0};
  double day_ahead_congestion_rent{0.0};
  double real_time_deviation_congestion_rent{0.0};
  double total_congestion_rent{0.0};
  double system_operator_ancillary_balance{0.0};
  double cashflow_residual{0.0};
};

struct RealTimeMarketResult {
  bool feasible{false};
  bool ancillary_services_enabled{false};
  std::string status;
  std::vector<std::string> warnings;
  MarketResult dispatch_instruction_market;
  MarketResult real_time_market;
  std::vector<RealTimePeriod> periods;
  std::vector<GeneratorDeviationSettlement> generator_deviation_settlement;
  std::vector<ParticipantDeviationSettlement> participant_deviation_settlement;
  DeviationSettlementLedger settlement;
  double total_absolute_generator_deviation_mwh{0.0};
};

struct GameParticipantRound {
  std::string participant_id;
  ParticipantBehavior behavior;
  ParticipantBehavior next_behavior;
  double profit{0.0};
  double net_ancillary_adjustment{0.0};
  double best_response_profit{0.0};
  double best_response_improvement{0.0};
  bool strategy_changed{false};
};

struct RepeatedGameRound {
  int round{0};
  bool day_ahead_feasible{false};
  bool real_time_feasible{false};
  double day_ahead_average_lmp_per_mwh{0.0};
  double real_time_average_lmp_per_mwh{0.0};
  double total_absolute_generator_deviation_mwh{0.0};
  double output_hhi{0.0};
  std::vector<GameParticipantRound> participants;
};

/// Deterministic synchronous local-best-response game.  Each learning player
/// evaluates its current action plus +/- one markup and withholding step while
/// opponents remain fixed.  Updates are bounded and accepted only when the
/// player's two-settlement profit improves by more than the tolerance.
struct RepeatedGameOptions {
  int max_rounds{8};
  double markup_step_fraction{0.05};
  double withholding_step_fraction{0.05};
  double maximum_markup_fraction{1.0};
  double maximum_withholding_fraction{0.50};
  double profit_improvement_tolerance{1e-4};
  bool include_cost_based_participants{false};
  bool run_real_time{true};
  MarketOptions day_ahead_options;
  RealTimeMarketOptions real_time_options;
};

struct RepeatedGameResult {
  bool feasible{false};
  bool converged{false};
  std::string status;
  std::vector<std::string> warnings;
  std::vector<RepeatedGameRound> rounds;
  std::vector<MarketParticipant> final_participants;
  MarketResult final_day_ahead;
  RealTimeMarketResult final_real_time;
};

/// Construct truthful, cost-based piecewise-linear offers from the physical
/// generator cost curves.  This is the competitive benchmark policy used by
/// the first market vertical slice.
std::vector<GeneratorOffer> make_cost_based_offers(
    const HybridPowerSystem& system,
    int segment_count = 8);

/// Resolve ownership and submit participant offers.  Unowned active generators
/// are assigned an explicit cost-based participant; duplicate ownership or an
/// invalid generator position is rejected.
OfferSubmission submit_participant_offers(
    const HybridPowerSystem& system,
    const std::vector<MarketParticipant>& participants,
    int segment_count = 8);

/// Run the native day-ahead market workflow:
///
///   participant offers -> SCUC -> fixed-commitment multi-period SCED/pricing
///   -> optional iterative LODF N-1 cuts -> AC base/contingency certification
///   -> day-ahead settlement, uplift, and preventive-security cost attribution.
///
/// The input system is never mutated.  The current implementation intentionally
/// prices the AC network subset; hybrid AC/DC market co-optimisation is a later
/// extension and is rejected explicitly rather than silently simplified.
MarketResult run_day_ahead_market(
    const HybridPowerSystem& system,
    const TimeSeriesData& time_series,
    const MarketOptions& options = {});

/// Re-dispatch the accepted day-ahead commitment against realized injections
/// and settle deviations at real-time LMPs using the standard two-settlement
/// identity: day-ahead schedule at DA price plus deviation at RT price.
RealTimeMarketResult run_real_time_market(
    const HybridPowerSystem& system,
    const TimeSeriesData& day_ahead_time_series,
    const TimeSeriesData& realized_time_series,
    const MarketResult& day_ahead_result,
    const RealTimeMarketOptions& options = {});

/// Run repeated day-ahead/real-time clearing and synchronous local best
/// responses for the configured participants.
RepeatedGameResult run_repeated_market_game(
    const HybridPowerSystem& system,
    const TimeSeriesData& day_ahead_time_series,
    const TimeSeriesData& realized_time_series,
    const RepeatedGameOptions& options = {});

}  // namespace hacdcpf::market
