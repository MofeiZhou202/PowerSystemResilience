#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "hacdcpf/model/hybrid_power_system.hpp"

namespace hacdcpf::io {

enum class ComponentIODomain {
  AC,
  DC,
  Hybrid,
  ThreePhaseAC,
};

enum class ComponentIOFormat {
  InternalJSON,
  CanonicalModel,
  GridLABD,
  OpenDSS,
};

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

enum class NumericalVerificationScope {
  ExactRoundTrip,
  EquivalentRoundTrip,
  NativeSolver,
  ExternalPowerFlow,
  BoundaryInjectionSnapshot,
  StructuralOnly,
  NotApplicable,
};

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
};

struct ComponentStandardProfile {
  ComponentStandardFamily family{ComponentStandardFamily::HACDCPF};
  std::string profile;
  std::string model_name;
  ComponentIOPolicy policy{ComponentIOPolicy::Exact};
  NumericalVerificationScope verification_scope{
      NumericalVerificationScope::StructuralOnly};
  std::string notes;
};

struct ComponentIOMapping {
  std::string component_type;
  std::string collection_path;
  ComponentIODomain domain{ComponentIODomain::AC};
  ComponentIOPolicy json_policy{ComponentIOPolicy::Exact};
  ComponentIOPolicy canonical_policy{ComponentIOPolicy::Exact};
  ComponentIOPolicy gridlabd_policy{ComponentIOPolicy::Unsupported};
  ComponentIOPolicy opendss_policy{ComponentIOPolicy::Unsupported};
  NumericalVerificationScope verification_scope{
      NumericalVerificationScope::StructuralOnly};
  std::string canonical_target;
  std::string gridlabd_target;
  std::string opendss_target;
  std::vector<ComponentStandardProfile> standard_profiles;
  std::string notes;
};

struct ComponentIOCoverageItem {
  ComponentIOMapping mapping;
  std::size_t count{0};
};

struct ComponentIOCoverageReport {
  std::vector<ComponentIOCoverageItem> items;
  std::vector<std::string> diagnostics;

  [[nodiscard]] std::size_t total_instances() const;
  [[nodiscard]] std::size_t represented_instances(
      ComponentIOFormat format) const;
  [[nodiscard]] std::size_t unrepresented_instances(
      ComponentIOFormat format) const;
};

const std::vector<ComponentIOMapping>& component_io_mappings();

std::optional<ComponentIOMapping> find_component_io_mapping(
    std::string_view component_type);

ComponentIOCoverageReport analyze_component_io_coverage(
    const HybridPowerSystem& sys);

bool is_represented_policy(ComponentIOPolicy policy);
ComponentIOPolicy policy_for_format(const ComponentIOMapping& mapping,
                                    ComponentIOFormat format);

std::string to_string(ComponentIODomain domain);
std::string to_string(ComponentIOFormat format);
std::string to_string(ComponentIOPolicy policy);
std::string to_string(NumericalVerificationScope scope);
std::string to_string(ComponentStandardFamily family);

std::vector<std::string> external_io_diagnostics(
    const ComponentIOCoverageReport& report,
    ComponentIOFormat format);

}  // namespace hacdcpf::io
