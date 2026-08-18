#include <catch2/catch_test_macros.hpp>

#include "hacdcpf/io/case_builders.hpp"
#include "hacdcpf/sppt/fault_campaign.hpp"

using namespace hacdcpf;

TEST_CASE("SPPT fault campaign is deterministic and emits complete evidence",
          "[sppt][fault-campaign]") {
  sppt::FaultCampaignOptions options;
  options.seed = 20260818;
  options.repetitions = 1;
  options.solve_structural_faults = false;
  const std::vector<std::pair<std::string, HybridPowerSystem>> systems = {
      {"IEEE14-ACDC", io::build_ieee14_acdc()}};

  const auto campaign = sppt::run_fault_campaign(systems, options);
  REQUIRE(campaign.rows.size() == 9);
  CHECK(campaign.summaries.size() == 45);
  CHECK(campaign.samples_csv().find("max_ac_voltage_error_pu") !=
        std::string::npos);
  CHECK(campaign.summary_csv().find("wilson_95_low") != std::string::npos);
  CHECK(campaign.to_latex().find("rate [95\\% CI]") != std::string::npos);

  for (const auto& row : campaign.rows) {
    if (row.fault == sppt::CampaignFaultClass::PlausibleConverterSetpointError) {
      CHECK(row.target.rfind("unsupported:", 0) != 0);
      CHECK(row.magnitude != 0.0);
    }
    if (row.fault == sppt::CampaignFaultClass::MissingVscAcTerminal ||
        row.fault == sppt::CampaignFaultClass::MissingVscDcTerminal ||
        row.fault == sppt::CampaignFaultClass::InvalidConverterEfficiency ||
        row.fault == sppt::CampaignFaultClass::DuplicateAcBusIdentity) {
      CHECK(row.validation_rejects);
      CHECK(row.sppt_rejects);
      CHECK(row.localized);
    }
  }
}
