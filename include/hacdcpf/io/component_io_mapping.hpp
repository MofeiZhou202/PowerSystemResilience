#pragma once

#include <cstddef>
#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "hacdcpf/model/hybrid_power_system.hpp"
#include "hacdcpf/io/roundtrip.hpp"

namespace hacdcpf::io {

/// Electrical domain owned by a rich-model component collection.
enum class ComponentIODomain {
  AC,
  DC,
  Hybrid,
  ThreePhaseAC,
};

/// Exchange target used by the component compatibility registry.
enum class ComponentIOFormat {
  InternalJSON,
  CanonicalModel,
  GridLABD,
  OpenDSS,
  PowerSimulationsDynamicsJulia,
};

/// Semantic preservation policy for one component type and one target format.
enum class ComponentIOPolicy {
  Exact,
  Equivalent,
  Aggregated,
  BoundaryInjection,
  Projected,
  InternalOnly,
  Unsupported,
  DiagnosticOnly,
};

/// Numerical evidence that should be used before claiming equivalence.
enum class NumericalVerificationScope {
  ExactRoundTrip,
  EquivalentRoundTrip,
  NativeSolver,
  ExternalPowerFlow,
  BoundaryInjectionSnapshot,
  StructuralOnly,
  NotApplicable,
};

/// Standards/profile family attached to a mapping, rule, or readiness finding.
enum class ComponentStandardFamily {
  HACDCPF,
  IEC61970CIM,
  IEC61850,
  IEC60909,
  IEEE,
  IEEE1547,
  IEEE4215,
  NERC,
  GridLABD,
  OpenDSS,
  PowerSimulationsDynamics,
};

/// Parameter domain used by the standard-aware model-quality audit.
enum class ComponentParameterCategory {
  Static,
  Dynamic,
  Transient,
  Failure,
  Reliability,
};

/// Finding severity shared by parameter audits and digital-twin readiness.
enum class ComponentParameterSeverity {
  Info,
  Warning,
  Error,
};

/// Readiness dimensions used to judge whether IO data can support a twin.
enum class DigitalTwinDimension {
  AssetIdentity,
  TopologyConnectivity,
  ElectricalParameters,
  DynamicBehavior,
  TelemetryObservability,
  StateSynchronization,
  ScenarioEvents,
  ReliabilityLifecycle,
  StandardsInteroperability,
  NumericalValidation,
  ProvenanceGovernance,
};

/// Industry-standard profile that explains how a component maps externally.
struct ComponentStandardProfile {
  ComponentStandardFamily family{ComponentStandardFamily::HACDCPF};
  std::string profile;
  std::string model_name;
  ComponentIOPolicy policy{ComponentIOPolicy::Exact};
  NumericalVerificationScope verification_scope{
      NumericalVerificationScope::StructuralOnly};
  std::string notes;
};

/// Range/presence rule for one component parameter under a standards profile.
///
/// Rules are deliberately data-like so they can be served to the GUI, used by
/// importers, and reused later by parameter-estimation workflows.  Numeric
/// bounds are optional; presence-only rules set @c required without a bound.
struct ComponentParameterRule {
  std::string component_type;
  std::string collection_path;
  std::string parameter_path;
  ComponentParameterCategory category{ComponentParameterCategory::Static};
  std::optional<double> min_value;
  std::optional<double> max_value;
  bool min_inclusive{true};
  bool max_inclusive{true};
  bool required{false};
  ComponentStandardFamily standard_family{ComponentStandardFamily::HACDCPF};
  std::string standard_profile;
  std::string units;
  ComponentParameterSeverity missing_severity{
      ComponentParameterSeverity::Info};
  ComponentParameterSeverity range_severity{
      ComponentParameterSeverity::Warning};
  std::string notes;
};

/// One parameter audit finding emitted for a concrete component instance.
struct ComponentParameterFinding {
  std::string component_type;
  std::string collection_path;
  std::size_t component_position{0};
  int component_index{0};
  std::string component_name;
  std::string parameter_path;
  ComponentParameterCategory category{ComponentParameterCategory::Static};
  ComponentParameterSeverity severity{ComponentParameterSeverity::Info};
  std::optional<double> value;
  std::optional<double> expected_min;
  std::optional<double> expected_max;
  bool min_inclusive{true};
  bool max_inclusive{true};
  bool required{false};
  ComponentStandardFamily standard_family{ComponentStandardFamily::HACDCPF};
  std::string standard_profile;
  std::string units;
  std::string message;
};

/// Aggregate result from applying the parameter-rule catalog to a system.
struct ComponentParameterAuditReport {
  std::size_t component_instances_checked{0};
  std::size_t checked_parameters{0};
  std::vector<ComponentParameterFinding> findings;

  [[nodiscard]] std::size_t count(
      ComponentParameterSeverity severity) const;
  [[nodiscard]] std::size_t count(
      ComponentParameterCategory category) const;
};

/// Weighted criterion used by the digital-twin readiness score.
struct DigitalTwinReadinessCriterion {
  std::string criterion_id;
  DigitalTwinDimension dimension{DigitalTwinDimension::AssetIdentity};
  std::string title;
  std::string description;
  double weight{1.0};
  ComponentParameterSeverity severity_if_failed{
      ComponentParameterSeverity::Warning};
  ComponentStandardFamily standard_family{ComponentStandardFamily::HACDCPF};
  std::string standard_profile;
};

/// Diagnostic result for one digital-twin readiness criterion.
struct DigitalTwinReadinessFinding {
  std::string criterion_id;
  DigitalTwinDimension dimension{DigitalTwinDimension::AssetIdentity};
  ComponentParameterSeverity severity{ComponentParameterSeverity::Info};
  double score{0.0};
  double max_score{1.0};
  std::string title;
  std::string message;
  std::string evidence;
  ComponentStandardFamily standard_family{ComponentStandardFamily::HACDCPF};
  std::string standard_profile;
};

/// Accumulated readiness score for one dimension.
struct DigitalTwinDimensionScore {
  DigitalTwinDimension dimension{DigitalTwinDimension::AssetIdentity};
  double score{0.0};
  double max_score{0.0};
  std::size_t findings{0};
};

/// One weakest-link maturity gate on the fidelity or integration axis.
///
/// See docs/digital_twin_data_io_architecture.md §4: maturity gates rather than
/// averages.  A level is achieved only when every gate at that level and below
/// passes.  @c evidence explains why a gate passed or failed.
struct DigitalTwinMaturityGate {
  std::string gate_id;   ///< "F1".."F3" or "I1".."I2".
  std::string axis;      ///< "fidelity" or "integration".
  int level{0};          ///< The level this gate guards.
  bool passed{false};
  std::string title;
  std::string evidence;
};

/// Digital-twin maturity report for one loaded system.
///
/// Two orthogonal, weakest-link gated axes (§4):
///   - @c fidelity_level     F0..F3 (file-parsed → static → executable → validated)
///   - @c integration_level  I0..I2 (offline → synchronized → closed-loop)
/// @c maturity_level is the coarse legacy L0..L5 *projection* of (F,I), retained
/// for display.  @c readiness_ratio is the weighted within-level completeness
/// indicator (secondary, never the level selector).
struct DigitalTwinReadinessReport {
  double score{0.0};
  double max_score{0.0};
  double readiness_ratio{0.0};
  int fidelity_level{0};
  std::string fidelity_label;
  int integration_level{0};
  std::string integration_label;
  int maturity_level{0};
  std::string maturity_label;
  std::vector<DigitalTwinMaturityGate> gates;
  std::vector<RoundTripEvidence> round_trip_evidence;
  std::vector<DigitalTwinDimensionScore> dimension_scores;
  std::vector<DigitalTwinReadinessFinding> findings;

  [[nodiscard]] std::size_t count(
      ComponentParameterSeverity severity) const;
};

/// Digital-twin suitability of one conversion target for the current system.
///
/// This turns the component-level IO registry into an operational decision:
/// which format can be the authoritative twin contract, which one is useful for
/// solver validation, and which one is only a diagnostic or projected view.
struct DigitalTwinConversionCapability {
  ComponentIOFormat format{ComponentIOFormat::InternalJSON};
  std::string role;
  std::string binding_level;
  std::string recommended_use;
  std::size_t represented_instances{0};
  std::size_t unrepresented_instances{0};
  std::size_t exact_or_equivalent_instances{0};
  std::size_t projected_instances{0};
  std::size_t diagnostic_only_instances{0};
  std::size_t unsupported_instances{0};
  double coverage_ratio{0.0};
  double fidelity_score{0.0};
  double validation_score{0.0};
  double risk_score{1.0};
  std::string risk_level;
  bool round_trip_available{false};
  bool twin_path_safe{false};
  std::vector<std::string> blocking_collections;
  std::vector<std::string> risks;
  std::vector<std::string> next_actions;
};

/// Stored validation evidence for one model/adapter twin pathway.
struct DigitalTwinEvidenceLedgerEntry {
  std::string model_name;
  ComponentIOFormat conversion_target{ComponentIOFormat::InternalJSON};
  std::string adapter;
  std::string fidelity_level;
  std::string validation_method;
  bool passed{false};
  double residual{1.0};
  double tolerance{0.0};
  std::string timestamp_utc;
  std::vector<std::string> blocking_collections;
  double confidence_score{0.0};
  double risk_score{1.0};
  std::string risk_level;
  std::string evidence_summary;
};

/// Timestamped digital-twin evidence ledger for one loaded model.
struct DigitalTwinEvidenceLedger {
  std::string model_name;
  std::string generated_at_utc;
  int fidelity_level{0};
  std::string fidelity_label;
  int integration_level{0};
  std::string integration_label;
  int maturity_level{0};
  std::string maturity_label;
  std::vector<DigitalTwinEvidenceLedgerEntry> entries;
};

/// Compatibility row for one rich component collection.
struct ComponentIOMapping {
  std::string component_type;
  std::string collection_path;
  ComponentIODomain domain{ComponentIODomain::AC};
  ComponentIOPolicy json_policy{ComponentIOPolicy::Exact};
  ComponentIOPolicy canonical_policy{ComponentIOPolicy::Exact};
  ComponentIOPolicy gridlabd_policy{ComponentIOPolicy::Unsupported};
  ComponentIOPolicy opendss_policy{ComponentIOPolicy::Unsupported};
  ComponentIOPolicy psd_policy{ComponentIOPolicy::Unsupported};
  NumericalVerificationScope verification_scope{
      NumericalVerificationScope::StructuralOnly};
  std::string canonical_target;
  std::string gridlabd_target;
  std::string opendss_target;
  std::string psd_target;
  std::vector<ComponentStandardProfile> standard_profiles;
  std::string notes;
};

/// Coverage row pairing a registry mapping with current system population.
struct ComponentIOCoverageItem {
  ComponentIOMapping mapping;
  std::size_t count{0};
};

/// Component IO coverage summary for a concrete system.
struct ComponentIOCoverageReport {
  std::vector<ComponentIOCoverageItem> items;
  std::vector<std::string> diagnostics;

  [[nodiscard]] std::size_t total_instances() const;
  [[nodiscard]] std::size_t represented_instances(
      ComponentIOFormat format) const;
  [[nodiscard]] std::size_t unrepresented_instances(
      ComponentIOFormat format) const;
};

/// Returns the immutable component-format compatibility registry.
const std::vector<ComponentIOMapping>& component_io_mappings();

/// Returns the immutable standards-aware parameter-rule catalog.
const std::vector<ComponentParameterRule>& component_parameter_rules();

/// Returns the immutable digital-twin readiness criterion catalog.
const std::vector<DigitalTwinReadinessCriterion>&
digital_twin_readiness_criteria();

/// Finds the first registry row for a component type name.
std::optional<ComponentIOMapping> find_component_io_mapping(
    std::string_view component_type);

/// Counts current system instances and external-format representation coverage.
ComponentIOCoverageReport analyze_component_io_coverage(
    const HybridPowerSystem& sys);

/// Applies parameter rules and consistency checks to the current rich model.
ComponentParameterAuditReport analyze_component_parameter_quality(
    const HybridPowerSystem& sys);

/// Computes the L0--L5 digital-twin readiness report for the current model.
DigitalTwinReadinessReport analyze_digital_twin_readiness(
    const HybridPowerSystem& sys);

/// Computes per-format conversion suitability for digital-twin workflows.
std::vector<DigitalTwinConversionCapability>
analyze_digital_twin_conversion_capabilities(const HybridPowerSystem& sys);

/// Builds timestamped model/adapter evidence entries for the GUI and API.
DigitalTwinEvidenceLedger analyze_digital_twin_evidence_ledger(
    const HybridPowerSystem& sys);

/// True when a policy preserves some executable/diagnostic target semantics.
bool is_represented_policy(ComponentIOPolicy policy);
/// Returns the policy stored in @p mapping for @p format.
ComponentIOPolicy policy_for_format(const ComponentIOMapping& mapping,
                                    ComponentIOFormat format);

std::string to_string(ComponentIODomain domain);
std::string to_string(ComponentIOFormat format);
std::string to_string(ComponentIOPolicy policy);
std::string to_string(NumericalVerificationScope scope);
std::string to_string(ComponentStandardFamily family);
std::string to_string(ComponentParameterCategory category);
std::string to_string(ComponentParameterSeverity severity);
std::string to_string(DigitalTwinDimension dimension);

/// Builds user-facing diagnostics for populated collections not represented in a target format.
std::vector<std::string> external_io_diagnostics(
    const ComponentIOCoverageReport& report,
    ComponentIOFormat format);

}  // namespace hacdcpf::io
