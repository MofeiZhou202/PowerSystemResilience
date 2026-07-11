#pragma once

#include <string>
#include <vector>

#include "hacdcpf/model/hybrid_power_system.hpp"
#include "hacdcpf/optimal_power_flow/opf_result.hpp"
#include "hacdcpf/power_flow/power_flow_result.hpp"
#include "hacdcpf/projection/project_to_canonical.hpp"

namespace hacdcpf::projection {

/// Result of the SPPT forward operator Pi: Rich -> Canonical. The canonical
/// model and its provenance must travel together; separating them makes a
/// type-correct canonical-to-rich attribution impossible.
struct ProjectionBundle {
  HybridPowerSystem canonical;
  ProjectionOptions options;
};

class RichToCanonicalOperator {
 public:
  static ProjectionBundle apply(
      const HybridPowerSystem& rich,
      const ProjectionOptions& options = ProjectionOptions{});
  static ProjectionBundle apply(
      HybridPowerSystem&& rich,
      const ProjectionOptions& options = ProjectionOptions{});
};

struct AttributedValue {
  std::string name;
  double value{0.0};
  std::string unit;
};

struct AttributedTerminal {
  std::string name;
  int bus{0};
  bool is_dc{false};
  double p_mw{0.0};
  double q_mvar{0.0};
  double v_pu{0.0};
  bool has_p{false};
  bool has_q{false};
  bool has_v{false};
};

struct CanonicalSourceRef {
  std::string component_type;
  int component_index{0};
  double participation_factor{1.0};
};

/// One row in A_S: Obs(Pi(S)) -> Obs(S), keyed only by rich identity.
/// `position` is display metadata and must never be used as identity.
struct RichComponentResult {
  std::string component_type;
  std::string domain;
  int component_index{0};
  int position{0};
  std::string name;
  bool in_service{true};
  RecoveryClass recovery{RecoveryClass::Unsupported};
  std::string recovery_reason;
  std::vector<CanonicalSourceRef> canonical_sources;
  std::vector<AttributedTerminal> terminals;
  std::vector<AttributedValue> values;
};

struct AttributionCoverage {
  int rich_components{0};
  int attributed_components{0};
  int strong_components{0};
  int approximate_components{0};
  int audit_only_components{0};
  int unsupported_components{0};

  [[nodiscard]] bool total() const noexcept {
    return unsupported_components == 0 &&
           rich_components == attributed_components;
  }
};

struct RichResultAttribution {
  std::vector<RichComponentResult> components;
  AttributionCoverage coverage;
  std::vector<std::string> diagnostics;
};

struct AttributionOptions {
  // A time-series operating point may deliberately replay the inter-temporal
  // storage schedule instead of the single-period OPF's band-level adjustment.
  // In that case storage P/Q comes from the rich snapshot while all other OPF
  // observables remain authoritative.
  bool prefer_rich_storage_dispatch{false};
};

/// SPPT provenance reconstruction operator A_S. `canonical_pf` is the result in
/// canonical branch space; `rich_pf` is the same solved point after bus/original
/// branch broadcast. Either PF pointer may be null for direct OPF observables.
class CanonicalToRichOperator {
 public:
  /// Bus-observable part of R_S. The returned vector is aligned with
  /// rich.ac.buses; each entry is its canonical bus position, or -1 when the
  /// bus was removed from the energized canonical support.
  static std::vector<int> ac_bus_reprojection_positions(
      const HybridPowerSystem& rich,
      const ProjectionBundle& projection);

  static RichResultAttribution apply(
      const HybridPowerSystem& rich,
      const ProjectionBundle& projection,
      const opf::ACOPFResult* opf_result,
      const PowerFlowResult* canonical_pf,
      const PowerFlowResult* rich_pf,
      const AttributionOptions& options = {});
};

const char* recovery_class_name(RecoveryClass recovery) noexcept;

}  // namespace hacdcpf::projection
