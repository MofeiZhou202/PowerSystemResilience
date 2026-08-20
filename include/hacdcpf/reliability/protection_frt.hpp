#pragma once

#include <cstddef>
#include <complex>
#include <string>
#include <vector>

#include "hacdcpf/dynamics/devices/IEEE1547Protection.hpp"
#include "hacdcpf/model/hybrid_power_system.hpp"

namespace hacdcpf::analysis {

struct RelayCurrentPoint {
  double time_s{0.0};
  double current{0.0};
};

/// IEC 60255-style inverse-time curve
/// T(M)=TMS*(A/(M^p-1)+B)+T_add, M=|I|/I_pickup.
struct InverseTimeRelaySettings {
  double pickup_current{1.0};
  double time_multiplier{1.0};
  double curve_a{0.14};
  double curve_p{0.02};
  double curve_b{0.0};
  double additional_delay_s{0.0};
  double reset_time_s{1.0};
};

struct InverseTimeRelayResult {
  bool operated{false};
  double command_time_s{0.0};
  double terminal_action_integral{0.0};
  double peak_multiple_of_pickup{0.0};
};

InverseTimeRelayResult evaluate_inverse_time_relay(
    const std::vector<RelayCurrentPoint>& trajectory,
    const InverseTimeRelaySettings& settings);

struct BreakerClearingChain {
  double channel_delay_s{0.0};
  double trip_coil_delay_s{0.0};
  double mechanical_delay_s{0.0};
  double arc_delay_s{0.0};
};

double clearing_time_s(const InverseTimeRelayResult& relay,
                       const BreakerClearingChain& chain);

enum class ProtectionRelayCharacteristic {
  DefiniteTimeOvercurrent,
  InverseTimeOvercurrent,
  Distance,
  Differential
};

struct ProtectionMeasurementPoint {
  double time_s{0.0};
  double current{0.0};
  /// Positive means forward for a directional element.
  double directional_current{0.0};
  double apparent_impedance_ohm{0.0};
  double differential_current{0.0};
  double restraint_current{0.0};
  bool phasor_measurement_valid{false};
  std::complex<double> voltage_phasor_v{0.0, 0.0};
  std::complex<double> current_phasor_a{0.0, 0.0};
};

struct PrimaryProtectionPhasorPoint {
  double time_s{0.0};
  std::complex<double> voltage_phasor_v{0.0, 0.0};
  std::complex<double> current_phasor_a{0.0, 0.0};
};

/// First-order CT/PT measurement dynamics. Ratios are primary/secondary.
/// Saturation limits apply to secondary RMS phasor magnitudes; zero disables
/// the corresponding limit.
struct InstrumentTransformerSettings {
  double ct_ratio{1.0};
  double pt_ratio{1.0};
  double ct_time_constant_s{0.0};
  double pt_time_constant_s{0.0};
  double ct_saturation_secondary_a{0.0};
  double pt_saturation_secondary_v{0.0};
  std::complex<double> initial_ct_secondary_a{0.0, 0.0};
  std::complex<double> initial_pt_secondary_v{0.0, 0.0};
};

std::vector<ProtectionMeasurementPoint> simulate_instrument_transformers(
    const std::vector<PrimaryProtectionPhasorPoint>& trajectory,
    const InstrumentTransformerSettings& settings);

enum class DistanceZoneShape {
  MagnitudeCircle,
  Mho,
  Quadrilateral
};

struct DistanceProtectionZone {
  std::string name;
  double reach_ohm{0.0};
  double delay_s{0.0};
  DistanceZoneShape shape{DistanceZoneShape::MagnitudeCircle};
  double line_angle_rad{0.0};
  double forward_resistance_ohm{0.0};
  double reverse_resistance_ohm{0.0};
  double forward_reactance_ohm{0.0};
  double reverse_reactance_ohm{0.0};
};

struct ProtectionRelayModel {
  std::string relay_id;
  ProtectionRelayCharacteristic characteristic{
      ProtectionRelayCharacteristic::InverseTimeOvercurrent};
  bool directional{false};
  double pickup_current{1.0};
  double definite_time_delay_s{0.0};
  InverseTimeRelaySettings inverse_time{};
  std::vector<DistanceProtectionZone> distance_zones;
  double differential_pickup{0.0};
  double differential_slope{0.0};
  double differential_high_set{0.0};
  double reset_time_s{1.0};
};

struct ProtectionRelayEvaluation {
  std::string relay_id;
  bool operated{false};
  double command_time_s{0.0};
  double terminal_action_integral{0.0};
  int operated_zone{-1};
  std::string operated_zone_name;
  std::string criterion;
};

ProtectionRelayEvaluation evaluate_protection_relay(
    const std::vector<ProtectionMeasurementPoint>& trajectory,
    const ProtectionRelayModel& relay);

struct AdaptiveProtectionContext {
  double pickup_scale{1.0};
  double distance_reach_scale{1.0};
  double time_delay_scale{1.0};
  bool communication_available{true};
};

ProtectionRelayModel adapt_protection_settings(
    const ProtectionRelayModel& base,
    const AdaptiveProtectionContext& context);

enum class ProtectionSequenceAction {
  TripOpen,
  Reclose,
  FuseOpen,
  SectionalizerOpen,
  Lockout
};

struct ProtectionSequenceEvent {
  double time_s{0.0};
  ProtectionSequenceAction action{ProtectionSequenceAction::TripOpen};
  int shot{0};
};

struct RecloserFuseSectionalizerInput {
  /// Energized fault duration before each recloser trip command.
  std::vector<double> shot_trip_times_s;
  std::vector<double> reclose_intervals_s;
  int maximum_shots{0};
  /// Fault becomes clear after this trip count; -1 means permanent.
  int fault_clears_after_shot{-1};
  bool instantaneous_lockout{false};
  double fuse_total_clearing_time_s{0.0};
  int sectionalizer_count_to_open{0};
};

struct RecloserFuseSectionalizerResult {
  std::vector<ProtectionSequenceEvent> events;
  int shots_completed{0};
  bool fault_cleared{false};
  bool recloser_closed{true};
  bool recloser_locked_out{false};
  bool fuse_open{false};
  bool sectionalizer_open{false};
  double fuse_melting_fraction{0.0};
  double terminal_time_s{0.0};
};

RecloserFuseSectionalizerResult simulate_recloser_fuse_sectionalizer(
    const RecloserFuseSectionalizerInput& input);

struct AutomaticProtectionTopologyInput {
  std::vector<int> opened_switch_indices;
  std::vector<int> opened_ac_branch_indices;
  std::vector<int> opened_dc_branch_indices;
};

struct AutomaticProtectionTopologyResult {
  std::vector<int> deenergized_ac_bus_ids;
  std::vector<int> deenergized_dc_bus_ids;
  double shed_mw{0.0};
  int interrupted_customers{0};
  int islands_without_source{0};
  bool stable_ids_resolved{false};
  bool domain_qualified_topology_used{false};
  std::string model_scope{
      "automatic-protection-topology-source-reachability"};
  std::string model_limitations{
      "拓扑供电可达性后果；不包含潮流、电压、无功和热稳定可行性"};
};

AutomaticProtectionTopologyResult evaluate_automatic_protection_topology(
    const HybridPowerSystem& system,
    const AutomaticProtectionTopologyInput& input);

struct ProtectionMisoperationInput {
  double no_fault_decision_windows_per_year{0.0};
  double false_trip_probability_per_window{0.0};
  double trip_channel_success_probability{1.0};
  double breaker_success_probability{1.0};
  double disconnected_load_mw{0.0};
  double restoration_duration_hr{0.0};
};

struct ProtectionMisoperationResult {
  double false_trip_frequency_per_year{0.0};
  double eens_mwh_yr{0.0};
  double lole_hr_yr{0.0};
  double lolf_occ_yr{0.0};
};

ProtectionMisoperationResult evaluate_protection_misoperation(
    const ProtectionMisoperationInput& input,
    double curtailment_threshold_mw = 0.01);

struct ProtectionCoordinationPair {
  size_t primary_relay{0};
  size_t backup_relay{0};
  double required_margin_s{0.0};
};

struct ProtectionCoordinationCheck {
  std::string primary_relay_id;
  std::string backup_relay_id;
  bool primary_operated{false};
  bool backup_operated{false};
  double primary_clear_time_s{0.0};
  double backup_clear_time_s{0.0};
  double actual_margin_s{0.0};
  double required_margin_s{0.0};
  bool selective{false};
  std::string message;
};

struct ProtectionCoordinationReport {
  std::vector<ProtectionRelayEvaluation> relay_evaluations;
  std::vector<double> clearing_times_s;
  std::vector<ProtectionCoordinationCheck> checks;
  bool all_selective{false};
};

ProtectionCoordinationReport evaluate_protection_coordination(
    const std::vector<ProtectionRelayModel>& relays,
    const std::vector<std::vector<ProtectionMeasurementPoint>>& trajectories,
    const std::vector<BreakerClearingChain>& breaker_chains,
    const std::vector<ProtectionCoordinationPair>& pairs);

struct DERTrajectoryPoint {
  double time_s{0.0};
  double voltage_pu{1.0};
  double angle_rad{0.0};
};

struct DERMomentaryCessationSettings {
  bool enabled{false};
  double enter_below_voltage_pu{0.5};
  double exit_above_voltage_pu{0.9};
  double exit_dwell_s{0.0};
  /// Zero disables escalation. Otherwise continuous cessation beyond this
  /// duration is classified as a trip even if the IEEE-1547 trip table has not
  /// yet operated.
  double maximum_duration_s{0.0};
};

enum class DERFRTTerminalClass {
  NotApplicable,
  RideThrough,
  MomentaryCessation,
  Tripped,
  Reconnected
};

struct DERFRTTrajectoryResult {
  DERFRTTerminalClass terminal_class{DERFRTTerminalClass::NotApplicable};
  bool terminal_connected{true};
  bool ever_tripped{false};
  bool ever_reconnected{false};
  bool ever_momentary_ceased{false};
  bool terminal_momentary_ceased{false};
  double first_trip_time_s{-1.0};
  double first_reconnect_time_s{-1.0};
  double first_momentary_cessation_time_s{-1.0};
  double first_momentary_recovery_time_s{-1.0};
  double terminal_restore_scale{1.0};
  std::string trip_reason;
};

DERFRTTrajectoryResult classify_der_frt_trajectory(
    const dynamics::IEEE1547Settings& settings,
    const std::vector<DERTrajectoryPoint>& trajectory,
    double terminal_time_s);

DERFRTTrajectoryResult classify_der_frt_trajectory(
    const dynamics::IEEE1547Settings& settings,
    const DERMomentaryCessationSettings& momentary_cessation,
    const std::vector<DERTrajectoryPoint>& trajectory,
    double terminal_time_s);

struct MicrogridSynchronizationInput {
  double voltage_difference_pu{0.0};
  double frequency_difference_hz{0.0};
  double angle_difference_rad{0.0};
  double maximum_voltage_difference_pu{0.1};
  double maximum_frequency_difference_hz{0.2};
  double maximum_angle_difference_rad{0.17453292519943295};
  bool synchronization_measurement_available{true};
  bool close_command_channel_available{true};
};

struct MicrogridSynchronizationResult {
  bool voltage_within_window{false};
  bool frequency_within_window{false};
  bool angle_within_window{false};
  bool close_permitted{false};
  std::string reason;
};

MicrogridSynchronizationResult evaluate_microgrid_synchronization(
    const MicrogridSynchronizationInput& input);

enum class ProtectionClearingOutcome {
  PrimaryCleared,
  BackupCleared,
  Uncleared
};

enum class ProtectionFailureCause {
  None,
  PrimaryRelayFailed,
  PrimaryBreakerFailed,
  PrimaryRelayAndBackupFailed,
  PrimaryBreakerAndBackupFailed,
  DetectionUnavailable,
  PrimaryTripChannelUnavailable,
  BackupTripChannelUnavailable
};

struct ProtectionRelayChainInput {
  std::vector<RelayCurrentPoint> current_trajectory;
  InverseTimeRelaySettings relay;
  BreakerClearingChain breaker;
  double relay_success_probability{1.0};
  double breaker_success_probability{1.0};
};

struct ProtectionFRTDERInput {
  std::string stable_id;
  dynamics::IEEE1547Settings settings;
  DERMomentaryCessationSettings momentary_cessation;
  std::vector<DERTrajectoryPoint> trajectory;
  /// Additional load-equivalent curtailment when the DER restore scale is 0.
  double loss_of_generation_shed_mw{0.0};
};

struct ProtectionFRTEventInput {
  ProtectionRelayChainInput primary;
  ProtectionRelayChainInput backup;
  double coordination_margin_s{0.0};
  double uncleared_terminal_time_s{0.0};
  std::vector<ProtectionFRTDERInput> ders;
};

struct ProtectionFRTGeneratedClass {
  ProtectionClearingOutcome protection_outcome{
      ProtectionClearingOutcome::Uncleared};
  ProtectionFailureCause failure_cause{ProtectionFailureCause::None};
  double conditional_probability{0.0};
  double terminal_time_s{0.0};
  bool coordination_satisfied{false};
  std::vector<std::string> der_stable_ids;
  std::vector<DERFRTTrajectoryResult> der_results;
};

struct ProtectionFRTClassGenerationResult {
  std::vector<ProtectionFRTGeneratedClass> classes;
  InverseTimeRelayResult primary_relay;
  InverseTimeRelayResult backup_relay;
  double primary_clear_time_s{0.0};
  double backup_clear_time_s{0.0};
  bool class_probabilities_normalized{false};
};

ProtectionFRTClassGenerationResult generate_protection_frt_classes(
    const ProtectionFRTEventInput& input);

/// Information component used by the protection/cyber event tree.  Conditional
/// on one environment state, intrinsic availability and packet delivery are
/// independent Bernoulli factors.  Shared paths retain the same component bit.
struct ProtectionCyberComponent {
  std::string id;
  double intrinsic_availability{1.0};
  double packet_delivery_probability{1.0};
  double latency_ms{0.0};
  double jitter_ms{0.0};
  std::string supplied_by_bus_id;
  double backup_energy_wh{0.0};
  double power_draw_w{0.0};
  std::string common_cause_group;
};

struct ProtectionCyberPath {
  std::vector<size_t> component_indices;
};

struct ProtectionCyberFunction {
  std::string name;
  std::vector<ProtectionCyberPath> alternative_paths;
  /// A zero maximum disables that deterministic QoS gate. Packet delivery is
  /// both an end-to-end eligibility threshold here and a Bernoulli component
  /// success factor in the enumerated state probability.
  double max_latency_ms{0.0};
  double max_jitter_ms{0.0};
  double min_packet_delivery_probability{0.0};
};

/// Mutually exclusive outer environment state.  It carries common-cause and
/// physical-to-information power dependencies; independent component states
/// are enumerated conditionally inside each environment.
struct ProtectionCyberEnvironment {
  std::string name;
  double probability{1.0};
  std::vector<std::string> failed_common_cause_groups;
  std::vector<std::string> deenergized_bus_ids;
  double information_outage_duration_hr{0.0};
};

struct ProtectionCyberStateClass {
  std::string environment_name;
  double probability{0.0};
  std::vector<bool> component_available;
  std::vector<bool> function_available;
};

struct ProtectionCyberStateResult {
  std::vector<ProtectionCyberStateClass> classes;
  std::vector<double> function_availability;
  bool probabilities_normalized{false};
  bool shared_dependencies_modelled{false};
  bool common_cause_conditioned{false};
  bool power_dependency_modelled{false};
};

ProtectionCyberStateResult enumerate_protection_cyber_states(
    const std::vector<ProtectionCyberComponent>& components,
    const std::vector<ProtectionCyberFunction>& functions,
    const std::vector<ProtectionCyberEnvironment>& environments = {});

struct CoordinatedProtectionChainInput {
  ProtectionRelayModel relay;
  std::vector<ProtectionMeasurementPoint> trajectory;
  BreakerClearingChain breaker;
  double relay_success_probability{1.0};
  double breaker_success_probability{1.0};
};

struct ProtectionCyberFunctionBindings {
  int detection_function{-1};
  int primary_trip_function{-1};
  int backup_trip_function{-1};
  int isolation_function{-1};
  int restoration_function{-1};
};

struct ProtectionCyberEventInput {
  CoordinatedProtectionChainInput primary;
  CoordinatedProtectionChainInput backup;
  double coordination_margin_s{0.0};
  double uncleared_terminal_time_s{0.0};
  std::vector<ProtectionFRTDERInput> ders;
  /// Optional outcome-specific DER trajectories. Empty vectors inherit `ders`.
  /// Capacities and stable IDs must match `ders`; only trajectories may differ.
  std::vector<ProtectionFRTDERInput> backup_ders;
  std::vector<ProtectionFRTDERInput> uncleared_ders;
  std::vector<ProtectionCyberComponent> information_components;
  std::vector<ProtectionCyberFunction> information_functions;
  std::vector<ProtectionCyberEnvironment> information_environments;
  ProtectionCyberFunctionBindings function_bindings;
};

struct ProtectionCyberGeneratedClass {
  ProtectionFRTGeneratedClass protection;
  std::string information_environment;
  std::vector<bool> information_component_available;
  bool detection_success{true};
  bool primary_trip_channel_available{true};
  bool backup_trip_channel_available{true};
  bool isolation_success{true};
  bool restoration_success{true};
};

struct ProtectionCyberClassGenerationResult {
  ProtectionCoordinationReport coordination;
  ProtectionCyberStateResult information;
  std::vector<ProtectionCyberGeneratedClass> classes;
  bool class_probabilities_normalized{false};
};

ProtectionCyberClassGenerationResult generate_protection_cyber_classes(
    const ProtectionCyberEventInput& input);

/// Automatic three-window consequence map for each generated class.
/// Clearing shed lasts until the electrical terminal time.  Switching shed
/// lasts for automatic/manual restoration time.  Residual shed then lasts
/// until physical repair.
struct ProtectionCyberConsequenceModel {
  double primary_clearing_shed_mw{0.0};
  double backup_clearing_shed_mw{0.0};
  double uncleared_shed_mw{0.0};
  double isolated_shed_mw{0.0};
  double isolation_failed_shed_mw{0.0};
  double restored_shed_mw{0.0};
  double restoration_failed_shed_mw{0.0};
  double automatic_restoration_hr{0.0};
  double manual_restoration_hr{0.0};
  double repair_hr{0.0};
};

struct ProtectionCyberReliabilityScenario {
  std::string scenario_id;
  double initiating_frequency_per_year{0.0};
  ProtectionCyberEventInput event;
  ProtectionCyberConsequenceModel consequence;
};

struct ProtectionCyberReliabilityComparison {
  double static_fmea_eens_mwh_yr{0.0};
  double protection_only_eens_mwh_yr{0.0};
  double cyber_conditioned_eens_mwh_yr{0.0};
  double cyber_increment_mwh_yr{0.0};
  double protection_benefit_mwh_yr{0.0};
  double static_fmea_lole_hr_yr{0.0};
  double protection_only_lole_hr_yr{0.0};
  double cyber_conditioned_lole_hr_yr{0.0};
  double static_fmea_lolf_occ_yr{0.0};
  double protection_only_lolf_occ_yr{0.0};
  double cyber_conditioned_lolf_occ_yr{0.0};
  int scenarios_evaluated{0};
  int joint_classes_evaluated{0};
  std::string model_scope{
      "protection-cyber-l2-discrete-event-tree+three-window-consequence"};
  struct ValidityFlags {
    bool relay_logic_modelled{false};
    bool protection_coordination_modelled{false};
    bool breaker_failure_modelled{false};
    bool information_topology_modelled{false};
    bool information_qos_modelled{false};
    bool common_cause_conditioned{false};
    bool cyber_power_dependency_modelled{false};
    bool der_ride_through_modelled{false};
    bool online_network_dae_coupled{false};
  } validity;
};

ProtectionCyberReliabilityComparison compare_protection_cyber_reliability(
    const std::vector<ProtectionCyberReliabilityScenario>& scenarios,
    double curtailment_threshold_mw = 0.01);

struct ProtectionFRTShedStage {
  double duration_hr{0.0};
  double shed_mw{0.0};
};

struct ProtectionFRTReliabilityClass {
  ProtectionFRTGeneratedClass generated_class;
  std::vector<ProtectionFRTShedStage> shed_stages;
};

struct ProtectionFRTReliabilityScenario {
  std::string scenario_id;
  double initiating_frequency_per_year{0.0};
  /// Must be true only when classes came from generate_protection_frt_classes
  /// using trace inputs representative of this scenario.
  bool trace_classes_validated{false};
  std::vector<ProtectionFRTReliabilityClass> classes;
};

struct ProtectionFRTReliabilityResult {
  double eens_mwh_yr{0.0};
  double lole_hr_yr{0.0};
  double lolf_occ_yr{0.0};
  int scenarios_evaluated{0};
  int classes_evaluated{0};
  std::string model_scope{"protection-frt-l1-trace-classes"};
  struct ValidityFlags {
    bool protection_frt_reliability_coupled{false};
    bool der_ride_through_modelled{false};
    bool protection_coordination_modelled{false};
    bool breaker_failure_modelled{false};
    bool online_dae_coupled{false};
  } validity;
};

ProtectionFRTReliabilityResult aggregate_protection_frt_reliability(
    const std::vector<ProtectionFRTReliabilityScenario>& scenarios,
    double curtailment_threshold_mw = 0.01);

}  // namespace hacdcpf::analysis
