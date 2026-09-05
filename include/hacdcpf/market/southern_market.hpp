#pragma once

#include <nlohmann/json.hpp>
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
nlohmann::json make_southern_market_demo();
nlohmann::json make_market_operation(const nlohmann::json& boundary, const nlohmann::json& config);
nlohmann::json step_market_operation(const nlohmann::json& job);
nlohmann::json market_forecast_defaults();
nlohmann::json make_market_forecast(const nlohmann::json& boundary, const nlohmann::json& config);
nlohmann::json step_market_forecast(const nlohmann::json& job);
nlohmann::json market_forecast_statistics(const nlohmann::json& job);
nlohmann::json run_southern_day_ahead_market(const nlohmann::json& boundary);
nlohmann::json compare_southern_market_results(const nlohmann::json& baseline,
                                              const nlohmann::json& scenario);

}  // namespace hacdcpf::market
