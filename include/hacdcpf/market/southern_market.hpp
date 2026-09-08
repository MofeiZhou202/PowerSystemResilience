#pragma once

#include <nlohmann/json.hpp>
#include <vector>
#include "hacdcpf/model/hybrid_power_system.hpp"

namespace hacdcpf::market {

// Standalone, versioned boundary snapshots. Public references are authored IDs,
// never positions in HybridPowerSystem vectors. Schema validation is mandatory.
nlohmann::json southern_market_schema();
nlohmann::json southern_market_boundary_catalog();
nlohmann::json southern_market_solver_capabilities();
nlohmann::json preview_market_operation_boundary(const nlohmann::json& boundary,
                                                const nlohmann::json& config, int day);
nlohmann::json validate_southern_market(const nlohmann::json& boundary);
nlohmann::json make_southern_market_example();
nlohmann::json southern_market_from_system(const HybridPowerSystem& system, bool augment_research_resources = false);
nlohmann::json southern_market_from_system(const HybridPowerSystem& system, bool augment_research_resources, int thermal_limit);
// PTDF rows in authored AC bus/branch IDs; period is 0..97. Independent island slacks.
nlohmann::json southern_market_ptdf(const nlohmann::json& boundary, int period, const std::vector<int>& branch_ids);
nlohmann::json make_southern_market_demo();
// IEEE 118 AC network plus explicitly synthetic market/resource declarations.
nlohmann::json make_southern_market_ieee118(const HybridPowerSystem& system);
// Mixed-fleet research case; the legacy IEEE118 benchmark above is retained.
nlohmann::json make_southern_market_ieee118_mixed(const HybridPowerSystem& system);
// Engineering snapshot of that case; reservoir/98-point contracts stay in the
// companion Southern boundary, not in the generic static engineering model.
HybridPowerSystem make_ieee118_market_system(const HybridPowerSystem& system);
nlohmann::json make_market_operation(const nlohmann::json& boundary, const nlohmann::json& config);
nlohmann::json step_market_operation(const nlohmann::json& job);
nlohmann::json explain_market_operation_day(const nlohmann::json& job, int day,
                                           const std::string& pricing = "dispatch_only");
nlohmann::json run_southern_dispatch_recovery(const nlohmann::json& boundary);
nlohmann::json market_forecast_defaults();
nlohmann::json make_market_study(const nlohmann::json& boundary, const nlohmann::json& config);
// Post-dispatch audit and conditional energy ledger; never a settlement certificate.
nlohmann::json analyze_southern_market_result(const nlohmann::json& result, bool audit_ac);
nlohmann::json make_market_forecast(const nlohmann::json& boundary, const nlohmann::json& config);
nlohmann::json step_market_forecast(const nlohmann::json& job);
nlohmann::json explain_market_forecast_day(const nlohmann::json& job, int scenario, int day,
                                          const std::string& pricing = "dispatch_only");
nlohmann::json market_forecast_statistics(const nlohmann::json& job);
nlohmann::json run_southern_day_ahead_market(const nlohmann::json& boundary);
// Southern chapter 3: 24 x 5min dispatch, separate 8 x 15min pricing.
// Jobs retain sealed inputs and carry only the first three executed points.
nlohmann::json southern_realtime_defaults(const nlohmann::json& boundary);
nlohmann::json southern_realtime_catalog();
nlohmann::json make_southern_realtime(const nlohmann::json& boundary, const nlohmann::json& config);
nlohmann::json step_southern_realtime(const nlohmann::json& job);
// Yunnan 2025 hourly AGC prearrangement, followed by fixed-UC/primary SCED.
// Synthetic example declarations are explicitly labelled; no AGC mileage is inferred.
nlohmann::json yunnan_ancillary_defaults(const nlohmann::json& boundary);
void validate_yunnan_ancillary(const nlohmann::json& boundary,const nlohmann::json& config);
nlohmann::json run_yunnan_ancillary_market(const nlohmann::json& boundary,
                                        const nlohmann::json& config);
nlohmann::json run_yunnan_ancillary_intraday(const nlohmann::json& boundary,
                                          const nlohmann::json& config,
                                          const nlohmann::json& day_ahead);
nlohmann::json settle_yunnan_ancillary(const nlohmann::json& result,
                                     const nlohmann::json& request);
nlohmann::json post_yunnan_ancillary_statement(const nlohmann::json& journal,
                                              const nlohmann::json& result,
                                              const nlohmann::json& entry);
nlohmann::json settle_yunnan_ancillary_month(const nlohmann::json& journal,
                                           const nlohmann::json& request);
// Assemble and count SCUC only; no solve, feasibility or price certification.
nlohmann::json inspect_southern_market_model(const nlohmann::json& boundary);
nlohmann::json compare_southern_market_results(const nlohmann::json& baseline,
                                              const nlohmann::json& scenario);

}  // namespace hacdcpf::market
