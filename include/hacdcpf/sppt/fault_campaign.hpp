#pragma once

/// Statistical fault-injection campaign for the SPPT manuscript.
/// The design follows docs/latex/sppt_theory.tex, Secs. 5--8: structural
/// validation, converter-role well-posedness, total attribution, and an
/// independently evaluated authored-equation residual are tested separately.

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "hacdcpf/model/hybrid_power_system.hpp"

namespace hacdcpf::sppt {

enum class CampaignFaultClass {
  ValidControl,
  MissingVscAcTerminal,
  MissingVscDcTerminal,
  MissingAcReference,
  MissingDcSupport,
  InvalidConverterEfficiency,
  DuplicateAcBusIdentity,
  PlausibleLoadError,
  PlausibleConverterSetpointError,
};

struct FaultCampaignOptions {
  std::uint64_t seed{20260818};
  int repetitions{25};
  double residual_tolerance{1e-6};
  bool solve_structural_faults{true};
  int max_structural_solver_ac_buses{330};
  int max_impact_solver_ac_buses{823};
};

struct FaultCampaignRow {
  std::string case_name;
  CampaignFaultClass fault{CampaignFaultClass::ValidControl};
  int repetition{0};
  std::string target;
  double magnitude{0.0};
  bool expected_admissible{true};

  bool validation_rejects{false};
  bool solver_rejects{false};
  bool validation_solver_rejects{false};
  bool sppt_rejects{false};
  bool sppt_independent_rejects{false};
  bool localized{false};

  bool solver_attempted{false};
  bool solver_converged{false};
  double solver_residual{0.0};
  bool independent_supported{false};
  double independent_residual{0.0};

  double validation_ms{0.0};
  double projection_ms{0.0};
  double guard_ms{0.0};
  double solve_ms{0.0};

  double max_ac_voltage_error_pu{0.0};
  double max_dc_voltage_error_pu{0.0};
  double max_branch_active_error_mw{0.0};
  double max_converter_active_error_mw{0.0};
  std::string note;
};

struct FaultCampaignSummaryRow {
  std::string detector;
  CampaignFaultClass fault{CampaignFaultClass::ValidControl};
  int samples{0};
  int detected{0};
  int localized{0};
  double detection_rate{0.0};
  double wilson_low{0.0};
  double wilson_high{0.0};
};

struct FaultCampaign {
  FaultCampaignOptions options;
  std::vector<FaultCampaignRow> rows;
  std::vector<FaultCampaignSummaryRow> summaries;

  [[nodiscard]] std::string samples_csv() const;
  [[nodiscard]] std::string summary_csv() const;
  [[nodiscard]] std::string to_latex() const;
};

[[nodiscard]] const char* to_string(CampaignFaultClass fault) noexcept;

FaultCampaign run_fault_campaign(
    const std::vector<std::pair<std::string, HybridPowerSystem>>& systems,
    const FaultCampaignOptions& options = {});

}  // namespace hacdcpf::sppt
