#include "edition_profile.hpp"

#include "hacdcpf/resilience/resilience_metrics.hpp"
#include "hacdcpf/analysis/weather_hazards.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <set>
#include <stdexcept>
#include <utility>
#include <vector>

namespace hacdcpf::server {
namespace {

using json = nlohmann::json;

constexpr auto kRouteManifest = std::to_array<EditionRouteRule>({
    // Profile and unversioned utility routes.
    {"GET", "/api/edition", "edition_profile", EditionRouteShape::Literal,
     EditionRouteAccess::Retained, EditionRouteAccess::Retained},
    {"POST", "/api/edition/analysis_plan", "edition_profile",
     EditionRouteShape::Literal, EditionRouteAccess::Retained,
     EditionRouteAccess::Retained},
    {"GET", "/api/cases", "model_io", EditionRouteShape::Literal,
     EditionRouteAccess::Retained, EditionRouteAccess::Retained},
    {"GET", "/api/matpower_files", "model_io", EditionRouteShape::Literal,
     EditionRouteAccess::Retained, EditionRouteAccess::Retained},
    {"GET", "/api/io/model_compatibility", "model_io",
     EditionRouteShape::Literal, EditionRouteAccess::Retained,
     EditionRouteAccess::Retained},
    {"GET", "/api/opf/parameter_contract", "optimal_power_flow",
     EditionRouteShape::Literal, EditionRouteAccess::Retained,
     EditionRouteAccess::Retained},

    // Parameter and reliability configuration.
    {"GET", "/api/session/parameter_library", "parameter_validation",
     EditionRouteShape::Literal, EditionRouteAccess::Retained,
     EditionRouteAccess::Retained},
    {"POST", "/api/session/parameter_library/select", "parameter_validation",
     EditionRouteShape::Literal, EditionRouteAccess::Retained,
     EditionRouteAccess::Retained},
    {"POST", "/api/session/parameter_library/update", "parameter_validation",
     EditionRouteShape::Literal, EditionRouteAccess::Retained,
     EditionRouteAccess::Retained},
    {"POST", "/api/session/parameter_library/validate", "parameter_validation",
     EditionRouteShape::Literal, EditionRouteAccess::Retained,
     EditionRouteAccess::Retained},
    {"POST", "/api/session/parameter_library/apply", "parameter_validation",
     EditionRouteShape::Literal, EditionRouteAccess::Retained,
     EditionRouteAccess::Retained},
    {"POST", "/api/session/design_handbook_parameters/preview",
     "parameter_validation", EditionRouteShape::Literal,
     EditionRouteAccess::Retained, EditionRouteAccess::Retained},
    {"POST", "/api/session/design_handbook_parameters/apply",
     "parameter_validation", EditionRouteShape::Literal,
     EditionRouteAccess::Retained, EditionRouteAccess::Retained},
    {"GET", "/api/session/reliability/data_quality", "reliability",
     EditionRouteShape::Literal, EditionRouteAccess::Retained,
     EditionRouteAccess::Retained},
    {"GET", "/api/session/reliability/configuration", "reliability",
     EditionRouteShape::Literal, EditionRouteAccess::Retained,
     EditionRouteAccess::Retained},
    {"POST", "/api/session/reliability/configuration/validate", "reliability",
     EditionRouteShape::Literal, EditionRouteAccess::Retained,
     EditionRouteAccess::Retained},
    {"POST", "/api/session/reliability/configuration", "reliability",
     EditionRouteShape::Literal, EditionRouteAccess::Retained,
     EditionRouteAccess::Retained},
    {"POST", "/api/session/reliability/compare_results", "reliability",
     EditionRouteShape::Literal, EditionRouteAccess::Retained,
     EditionRouteAccess::Retained},

    // Canonical legacy/session analysis and model-I/O routes. analysis_id values
    // deliberately match python/src/hysim/analyses.py AnalysisSpec.name.
    {"POST", "/api/session/pf", "power_flow", EditionRouteShape::Literal,
     EditionRouteAccess::Retained, EditionRouteAccess::Retained, "power_flow"},
    {"POST", "/api/session/opf", "optimal_power_flow",
     EditionRouteShape::Literal, EditionRouteAccess::Retained,
     EditionRouteAccess::Retained, "optimal_power_flow"},
    {"POST", "/api/session/opf_ac", "optimal_power_flow",
     EditionRouteShape::Literal, EditionRouteAccess::Retained,
     EditionRouteAccess::Retained, "opf_ac"},
    {"POST", "/api/session/opf_parity", "optimal_power_flow",
     EditionRouteShape::Literal, EditionRouteAccess::Retained,
     EditionRouteAccess::Retained, "opf_parity"},
    {"POST", "/api/session/opf_dc", "optimal_power_flow",
     EditionRouteShape::Literal, EditionRouteAccess::Retained,
     EditionRouteAccess::Retained, "opf_dc"},
    {"POST", "/api/session/rpo_inputs", "reactive_power_optimization",
     EditionRouteShape::Literal, EditionRouteAccess::Disabled,
     EditionRouteAccess::Disabled, "rpo_inputs"},
    {"POST", "/api/session/run_rpo", "reactive_power_optimization",
     EditionRouteShape::Literal, EditionRouteAccess::Disabled,
     EditionRouteAccess::Disabled, "reactive_power_optimization"},
    {"POST", "/api/session/sc", "short_circuit", EditionRouteShape::Literal,
     EditionRouteAccess::Retained, EditionRouteAccess::Retained, "short_circuit"},
    {"POST", "/api/session/sc_detailed", "short_circuit",
     EditionRouteShape::Literal, EditionRouteAccess::Retained,
     EditionRouteAccess::Retained, "detailed_short_circuit"},
    {"POST", "/api/session/dc_sc", "short_circuit",
     EditionRouteShape::Literal, EditionRouteAccess::Retained,
     EditionRouteAccess::Retained, "dc_short_circuit"},
    {"POST", "/api/session/harmonics", "harmonics", EditionRouteShape::Literal,
     EditionRouteAccess::Disabled, EditionRouteAccess::Disabled, "harmonics"},
    {"POST", "/api/session/harmonics_3ph", "harmonics",
     EditionRouteShape::Literal, EditionRouteAccess::Disabled,
     EditionRouteAccess::Disabled, "three_phase_harmonics"},
    {"POST", "/api/session/harmonics_freqscan", "harmonics",
     EditionRouteShape::Literal, EditionRouteAccess::Disabled,
     EditionRouteAccess::Disabled, "harmonics_frequency_scan"},
    {"POST", "/api/session/harmonics_hss", "harmonics",
     EditionRouteShape::Literal, EditionRouteAccess::Disabled,
     EditionRouteAccess::Disabled, "harmonics_hss"},
    {"POST", "/api/session/harmonics_newton", "harmonics",
     EditionRouteShape::Literal, EditionRouteAccess::Disabled,
     EditionRouteAccess::Disabled, "harmonics_newton"},
    {"POST", "/api/session/harmonics_metrics", "harmonics",
     EditionRouteShape::Literal, EditionRouteAccess::Disabled,
     EditionRouteAccess::Disabled, "harmonics_metrics"},
    {"POST", "/api/session/run_transient", "dynamics",
     EditionRouteShape::Literal, EditionRouteAccess::Disabled,
     EditionRouteAccess::Disabled, "transient"},
    {"POST", "/api/session/small_signal", "dynamics",
     EditionRouteShape::Literal, EditionRouteAccess::Disabled,
     EditionRouteAccess::Disabled, "small_signal"},
    {"POST", "/api/session/run_uc", "market", EditionRouteShape::Literal,
     EditionRouteAccess::Disabled, EditionRouteAccess::Disabled, "unit_commitment"},
    {"POST", "/api/session/set_ts_config", "time_series",
     EditionRouteShape::Literal, EditionRouteAccess::Disabled,
     EditionRouteAccess::Disabled, "set_ts_config"},
    {"POST", "/api/session/run_ts_pf", "time_series",
     EditionRouteShape::Literal, EditionRouteAccess::Disabled,
     EditionRouteAccess::Disabled, "time_series_power_flow"},
    {"POST", "/api/session/run_annual_sim", "time_series",
     EditionRouteShape::Literal, EditionRouteAccess::Disabled,
     EditionRouteAccess::Disabled, "annual_production"},
    {"POST", "/api/session/run_lifecycle_sim", "time_series",
     EditionRouteShape::Literal, EditionRouteAccess::Disabled,
     EditionRouteAccess::Disabled, "lifecycle_simulation"},
    {"POST", "/api/session/run_lifecycle_compare", "time_series",
     EditionRouteShape::Literal, EditionRouteAccess::Disabled,
     EditionRouteAccess::Disabled, "lifecycle_compare"},
    {"POST", "/api/session/run_carbon", "carbon", EditionRouteShape::Literal,
     EditionRouteAccess::Retained, EditionRouteAccess::Disabled, "carbon_flow"},
    {"POST", "/api/session/run_dynamic_carbon", "time_series",
     EditionRouteShape::Literal, EditionRouteAccess::Disabled,
     EditionRouteAccess::Disabled, "dynamic_carbon_flow"},
    {"POST", "/api/session/run_reliability_nsq", "reliability",
     EditionRouteShape::Literal, EditionRouteAccess::Retained,
     EditionRouteAccess::Retained, "reliability_nonsequential"},
    {"POST", "/api/session/run_reliability_seq", "reliability",
     EditionRouteShape::Literal, EditionRouteAccess::Retained,
     EditionRouteAccess::Retained, "reliability_sequential"},
    {"POST", "/api/session/run_reliability_fmea", "reliability",
     EditionRouteShape::Literal, EditionRouteAccess::Retained,
     EditionRouteAccess::Retained, "reliability_fmea"},
    {"POST", "/api/session/run_reliability_fd", "reliability",
     EditionRouteShape::Literal, EditionRouteAccess::Retained,
     EditionRouteAccess::Retained, "reliability_fd"},
    {"POST", "/api/session/run_reliability_three_stage", "reliability",
     EditionRouteShape::Literal, EditionRouteAccess::Retained,
     EditionRouteAccess::Retained, "reliability_three_stage"},
    {"POST", "/api/session/run_reliability", "reliability",
     EditionRouteShape::Literal, EditionRouteAccess::Retained,
     EditionRouteAccess::Retained, "reliability"},
    {"POST", "/api/session/run_distribution_resilience", "resilience",
     EditionRouteShape::Literal, EditionRouteAccess::Retained,
     EditionRouteAccess::Retained, "distribution_resilience"},
    {"GET", "/api/session/resilience/metric_catalog", "resilience_metrics",
     EditionRouteShape::Literal, EditionRouteAccess::Disabled,
     EditionRouteAccess::Retained},
    {"POST", "/api/session/resilience/metrics", "resilience_metrics",
     EditionRouteShape::Literal, EditionRouteAccess::Disabled,
     EditionRouteAccess::Retained},
    {"POST", "/api/session/run_market_clearing", "market",
     EditionRouteShape::Literal, EditionRouteAccess::Disabled,
     EditionRouteAccess::Disabled, "market_clearing"},
    {"POST", "/api/session/run_real_time_market", "market",
     EditionRouteShape::Literal, EditionRouteAccess::Disabled,
     EditionRouteAccess::Disabled, "real_time_market"},
    {"POST", "/api/session/run_repeated_market_game", "market",
     EditionRouteShape::Literal, EditionRouteAccess::Disabled,
     EditionRouteAccess::Disabled, "repeated_market_game"},
    {"POST", "/api/session/run_southern_market", "market",
     EditionRouteShape::Literal, EditionRouteAccess::Disabled,
     EditionRouteAccess::Disabled, "southern_market"},
    {"POST", "/api/session/market_ptdf", "market",
     EditionRouteShape::Literal, EditionRouteAccess::Disabled,
     EditionRouteAccess::Disabled, "market_ptdf"},
    {"POST", "/api/session/run_campus_ies", "integrated_energy",
     EditionRouteShape::Literal, EditionRouteAccess::Disabled,
     EditionRouteAccess::Disabled, "campus_ies"},
    {"POST", "/api/session/run_ev_traffic", "ev_traffic",
     EditionRouteShape::Literal, EditionRouteAccess::Disabled,
     EditionRouteAccess::Disabled, "ev_power_traffic"},
    {"POST", "/api/session/run_reconfig", "network_reconfiguration",
     EditionRouteShape::Literal, EditionRouteAccess::Disabled,
     EditionRouteAccess::Retained, "reconfiguration"},
    {"POST", "/api/session/run_hosting_capacity", "hosting_capacity",
     EditionRouteShape::Literal, EditionRouteAccess::Retained,
     EditionRouteAccess::Disabled, "hosting_capacity"},
    {"POST", "/api/session/run_counterfactual_planning",
     "counterfactual_planning", EditionRouteShape::Literal,
     EditionRouteAccess::Disabled, EditionRouteAccess::Disabled,
     "counterfactual_planning"},
    {"POST", "/api/session/run_multidimensional_weak_links", "weak_links",
     EditionRouteShape::Literal, EditionRouteAccess::Retained,
     EditionRouteAccess::Disabled, "multidimensional_weak_links"},
    {"POST", "/api/session/generate_scenarios", "scenario_generation",
     EditionRouteShape::Literal, EditionRouteAccess::Retained,
     EditionRouteAccess::Retained, "scenario_generation"},
    {"POST", "/api/session/generate_typhoon_faults", "typhoon_faults",
     EditionRouteShape::Literal, EditionRouteAccess::Retained,
     EditionRouteAccess::Retained, "typhoon_faults"},
    {"POST", "/api/session/sppt_guard", "sppt_agent",
     EditionRouteShape::Literal, EditionRouteAccess::Disabled,
     EditionRouteAccess::Disabled, "sppt_guard"},
    {"POST", "/api/session/topology", "topology", EditionRouteShape::Literal,
     EditionRouteAccess::Retained, EditionRouteAccess::Retained, "topology"},
    {"POST", "/api/session/network_reduction", "network_reconfiguration",
     EditionRouteShape::Literal, EditionRouteAccess::Disabled,
     EditionRouteAccess::Retained, "network_reduction"},
    {"POST", "/api/session/load_builtin", "model_io", EditionRouteShape::Literal,
     EditionRouteAccess::Retained, EditionRouteAccess::Retained, "load_builtin"},
    {"POST", "/api/session/load_matpower", "model_io", EditionRouteShape::Literal,
     EditionRouteAccess::Retained, EditionRouteAccess::Retained, "load_matpower"},
    {"POST", "/api/session/load_json_string", "model_io",
     EditionRouteShape::Literal, EditionRouteAccess::Retained,
     EditionRouteAccess::Retained, "load_json_string"},
    {"POST", "/api/session/new_empty", "model_io", EditionRouteShape::Literal,
     EditionRouteAccess::Retained, EditionRouteAccess::Retained, "new_empty"},
    {"POST", "/api/session/update_components", "model",
     EditionRouteShape::Literal, EditionRouteAccess::Retained,
     EditionRouteAccess::Retained, "update_components"},
    {"POST", "/api/session/load_bpa_dat", "advanced_io",
     EditionRouteShape::Literal, EditionRouteAccess::Disabled,
     EditionRouteAccess::Disabled, "load_bpa_dat"},
    {"POST", "/api/session/load_cim_dist", "advanced_io",
     EditionRouteShape::Literal, EditionRouteAccess::Disabled,
     EditionRouteAccess::Disabled, "load_cim_dist"},
    {"POST", "/api/session/load_gridlabd", "advanced_io",
     EditionRouteShape::Literal, EditionRouteAccess::Retained,
     EditionRouteAccess::Disabled, "load_gridlabd"},
    {"POST", "/api/session/load_opendss", "advanced_io",
     EditionRouteShape::Literal, EditionRouteAccess::Retained,
     EditionRouteAccess::Disabled, "load_opendss"},
    {"POST", "/api/session/load_etap_xml", "advanced_io",
     EditionRouteShape::Literal, EditionRouteAccess::Disabled,
     EditionRouteAccess::Disabled, "load_etap_xml"},
    {"POST", "/api/session/load_svg_distribution", "advanced_io",
     EditionRouteShape::Literal, EditionRouteAccess::Disabled,
     EditionRouteAccess::Disabled, "load_svg_distribution"},
    {"POST", "/api/session/export_json", "model_io", EditionRouteShape::Literal,
     EditionRouteAccess::Retained, EditionRouteAccess::Retained, "export_json"},
    {"POST", "/api/session/export_matpower", "model_io",
     EditionRouteShape::Literal, EditionRouteAccess::Retained,
     EditionRouteAccess::Retained, "export_matpower"},
    {"POST", "/api/session/export_bpa_dat", "advanced_io",
     EditionRouteShape::Literal, EditionRouteAccess::Disabled,
     EditionRouteAccess::Disabled, "export_bpa_dat"},
    {"POST", "/api/session/export_etap", "advanced_io",
     EditionRouteShape::Literal, EditionRouteAccess::Disabled,
     EditionRouteAccess::Disabled, "export_etap"},
    {"POST", "/api/session/export_etap_xml", "advanced_io",
     EditionRouteShape::Literal, EditionRouteAccess::Disabled,
     EditionRouteAccess::Disabled, "export_etap_xml"},
    {"POST", "/api/session/export_cim_dist", "advanced_io",
     EditionRouteShape::Literal, EditionRouteAccess::Disabled,
     EditionRouteAccess::Disabled, "export_cim_dist"},
    {"POST", "/api/session/export_gridlabd", "advanced_io",
     EditionRouteShape::Literal, EditionRouteAccess::Retained,
     EditionRouteAccess::Disabled, "export_gridlabd"},
    {"POST", "/api/session/export_opendss", "advanced_io",
     EditionRouteShape::Literal, EditionRouteAccess::Retained,
     EditionRouteAccess::Disabled, "export_opendss"},
    {"POST", "/api/session/export_svg_distribution", "advanced_io",
     EditionRouteShape::Literal, EditionRouteAccess::Disabled,
     EditionRouteAccess::Disabled, "export_svg_distribution"},

    // Other registered legacy/session routes without AnalysisSpec entries.
    {"GET", "/api/dynamics/model_schema", "dynamics",
     EditionRouteShape::Literal, EditionRouteAccess::Disabled,
     EditionRouteAccess::Disabled},
    {"POST", "/api/dynamics/validate_profile", "dynamics",
     EditionRouteShape::Literal, EditionRouteAccess::Disabled,
     EditionRouteAccess::Disabled},
    {"GET", "/api/session/transient/frame", "dynamics",
     EditionRouteShape::Literal, EditionRouteAccess::Disabled,
     EditionRouteAccess::Disabled},
    {"GET", "/api/session/tspf/frame", "time_series",
     EditionRouteShape::Literal, EditionRouteAccess::Disabled,
     EditionRouteAccess::Disabled},
    {"POST", "/api/session/topology_window", "topology",
     EditionRouteShape::Literal, EditionRouteAccess::Retained,
     EditionRouteAccess::Retained},
    {"POST", "/api/session/result_window", "power_flow",
     EditionRouteShape::Literal, EditionRouteAccess::Retained,
     EditionRouteAccess::Retained},
    {"POST", "/api/session/export_powersimulationsdynamics", "advanced_io",
     EditionRouteShape::Literal, EditionRouteAccess::Disabled,
     EditionRouteAccess::Disabled},
    {"POST", "/api/session/export_powersimulationsdynamics_julia",
     "advanced_io", EditionRouteShape::Literal, EditionRouteAccess::Disabled,
     EditionRouteAccess::Disabled},
    {"POST", "/api/session/load_powersimulationsdynamics_julia", "advanced_io",
     EditionRouteShape::Literal, EditionRouteAccess::Disabled,
     EditionRouteAccess::Disabled},
    {"POST", "/api/session/load_etap_xlsx", "advanced_io",
     EditionRouteShape::Literal, EditionRouteAccess::Disabled,
     EditionRouteAccess::Disabled},
    {"POST", "/api/session/validate_scenario_bundle", "advanced_io",
     EditionRouteShape::Literal, EditionRouteAccess::Disabled,
     EditionRouteAccess::Disabled},
    {"POST", "/api/session/export_scenario_workbook", "advanced_io",
     EditionRouteShape::Literal, EditionRouteAccess::Disabled,
     EditionRouteAccess::Disabled},
    {"POST", "/api/session/import_scenario_workbook", "advanced_io",
     EditionRouteShape::Literal, EditionRouteAccess::Disabled,
     EditionRouteAccess::Disabled},
    {"POST", "/api/session/update_carbon_factors", "carbon",
     EditionRouteShape::Literal, EditionRouteAccess::Retained,
     EditionRouteAccess::Disabled},
    {"POST", "/api/session/cancel", "session", EditionRouteShape::Literal,
     EditionRouteAccess::Retained, EditionRouteAccess::Retained},
    {"GET", "/api/session/status", "session", EditionRouteShape::Literal,
     EditionRouteAccess::Retained, EditionRouteAccess::Retained},
    {"POST", "/api/session/sppt_agent", "sppt_agent",
     EditionRouteShape::Literal, EditionRouteAccess::Disabled,
     EditionRouteAccess::Disabled},
    {"GET", "/api/session/southern_market", "market",
     EditionRouteShape::Literal, EditionRouteAccess::Disabled,
     EditionRouteAccess::Disabled},
    {"POST", "/api/session/southern_market", "market",
     EditionRouteShape::Literal, EditionRouteAccess::Disabled,
     EditionRouteAccess::Disabled},
    {"GET", "/api/session/southern_realtime", "market",
     EditionRouteShape::Literal, EditionRouteAccess::Disabled,
     EditionRouteAccess::Disabled},
    {"POST", "/api/session/southern_realtime", "market",
     EditionRouteShape::Literal, EditionRouteAccess::Disabled,
     EditionRouteAccess::Disabled},
    {"GET", "/api/session/yunnan_ancillary", "market",
     EditionRouteShape::Literal, EditionRouteAccess::Disabled,
     EditionRouteAccess::Disabled},
    {"POST", "/api/session/yunnan_ancillary", "market",
     EditionRouteShape::Literal, EditionRouteAccess::Disabled,
     EditionRouteAccess::Disabled},
    {"GET", "/api/session/market_operation", "market",
     EditionRouteShape::Literal, EditionRouteAccess::Disabled,
     EditionRouteAccess::Disabled},
    {"POST", "/api/session/market_operation", "market",
     EditionRouteShape::Literal, EditionRouteAccess::Disabled,
     EditionRouteAccess::Disabled},
    {"GET", "/api/session/market_forecast", "market",
     EditionRouteShape::Literal, EditionRouteAccess::Disabled,
     EditionRouteAccess::Disabled},
    {"POST", "/api/session/market_forecast", "market",
     EditionRouteShape::Literal, EditionRouteAccess::Disabled,
     EditionRouteAccess::Disabled},
    {"GET", "/api/session/market_study", "market",
     EditionRouteShape::Literal, EditionRouteAccess::Disabled,
     EditionRouteAccess::Disabled},
    {"POST", "/api/session/market_study", "market",
     EditionRouteShape::Literal, EditionRouteAccess::Disabled,
     EditionRouteAccess::Disabled},

    // Compatibility analysis routes.
    {"POST", "/api/load_case", "model_io", EditionRouteShape::Literal,
     EditionRouteAccess::Retained, EditionRouteAccess::Retained},
    {"POST", "/api/pf", "power_flow", EditionRouteShape::Literal,
     EditionRouteAccess::Retained, EditionRouteAccess::Retained},
    {"POST", "/api/opf/ac", "optimal_power_flow", EditionRouteShape::Literal,
     EditionRouteAccess::Retained, EditionRouteAccess::Retained},
    {"POST", "/api/opf/dc", "optimal_power_flow", EditionRouteShape::Literal,
     EditionRouteAccess::Retained, EditionRouteAccess::Retained},
    {"POST", "/api/sc", "short_circuit", EditionRouteShape::Literal,
     EditionRouteAccess::Retained, EditionRouteAccess::Retained},
    {"POST", "/api/dc_sc", "short_circuit", EditionRouteShape::Literal,
     EditionRouteAccess::Retained, EditionRouteAccess::Retained},

    // Strictly parsed versioned resources. Braced components are templates, not
    // regular expressions; matching below enforces identifier/decimal domains.
    {"GET", "/api/v1", "runtime_api_v1", EditionRouteShape::StructuredV1,
     EditionRouteAccess::Retained, EditionRouteAccess::Retained},
    {"GET", "/api/v1/sessions", "runtime_api_v1",
     EditionRouteShape::StructuredV1, EditionRouteAccess::Retained,
     EditionRouteAccess::Retained},
    {"POST", "/api/v1/sessions", "runtime_api_v1",
     EditionRouteShape::StructuredV1, EditionRouteAccess::Retained,
     EditionRouteAccess::Retained},
    {"GET", "/api/v1/sessions/{session_id}", "runtime_api_v1",
     EditionRouteShape::StructuredV1, EditionRouteAccess::Retained,
     EditionRouteAccess::Retained},
    {"DELETE", "/api/v1/sessions/{session_id}", "runtime_api_v1",
     EditionRouteShape::StructuredV1, EditionRouteAccess::Retained,
     EditionRouteAccess::Retained},
    {"GET", "/api/v1/sessions/{session_id}/model", "runtime_api_v1",
     EditionRouteShape::StructuredV1, EditionRouteAccess::Retained,
     EditionRouteAccess::Retained},
    {"PUT", "/api/v1/sessions/{session_id}/model", "runtime_api_v1",
     EditionRouteShape::StructuredV1, EditionRouteAccess::Retained,
     EditionRouteAccess::Retained},
    {"GET", "/api/v1/sessions/{session_id}/topology", "runtime_api_v1",
     EditionRouteShape::StructuredV1, EditionRouteAccess::Retained,
     EditionRouteAccess::Retained},
    {"GET", "/api/v1/sessions/{session_id}/subgraph", "runtime_api_v1",
     EditionRouteShape::StructuredV1, EditionRouteAccess::Retained,
     EditionRouteAccess::Retained},
    {"GET", "/api/v1/sessions/{session_id}/jobs", "runtime_api_v1",
     EditionRouteShape::StructuredV1, EditionRouteAccess::Retained,
     EditionRouteAccess::Retained},
    {"POST", "/api/v1/sessions/{session_id}/jobs", "runtime_api_v1",
     EditionRouteShape::StructuredV1, EditionRouteAccess::Retained,
     EditionRouteAccess::Retained},
    {"GET", "/api/v1/jobs/{job_id}", "runtime_api_v1",
     EditionRouteShape::StructuredV1, EditionRouteAccess::Retained,
     EditionRouteAccess::Retained},
    {"DELETE", "/api/v1/jobs/{job_id}", "runtime_api_v1",
     EditionRouteShape::StructuredV1, EditionRouteAccess::Retained,
     EditionRouteAccess::Retained},
    {"GET", "/api/v1/jobs/{job_id}/violations", "runtime_api_v1",
     EditionRouteShape::StructuredV1, EditionRouteAccess::Retained,
     EditionRouteAccess::Retained},
    {"POST", "/api/v1/jobs/{job_id}/cancel", "runtime_api_v1",
     EditionRouteShape::StructuredV1, EditionRouteAccess::Retained,
     EditionRouteAccess::Retained},
    {"GET", "/api/v1/jobs/{job_id}/frames/{step}", "runtime_api_v1",
     EditionRouteShape::StructuredV1, EditionRouteAccess::Retained,
     EditionRouteAccess::Retained},
});

constexpr auto kV1Analyses =
    std::to_array<std::string_view>({"power_flow", "optimal_power_flow"});

bool is_identifier(std::string_view value) {
  return !value.empty() &&
         std::all_of(value.begin(), value.end(), [](unsigned char c) {
           return std::isalnum(c) || c == '_' || c == '-';
         });
}

bool is_decimal(std::string_view value) {
  return !value.empty() &&
         std::all_of(value.begin(), value.end(), [](unsigned char c) {
           return std::isdigit(c);
         });
}

std::vector<std::string_view> path_segments(std::string_view path) {
  std::vector<std::string_view> segments;
  if (path.empty() || path.front() != '/' || path.back() == '/') return segments;
  std::size_t begin = 1;
  while (begin < path.size()) {
    const auto end = path.find('/', begin);
    const auto segment = path.substr(begin, end - begin);
    if (segment.empty()) return {};
    segments.push_back(segment);
    if (end == std::string_view::npos) break;
    begin = end + 1;
  }
  return segments;
}

bool structured_v1_match(std::string_view path_template,
                         std::string_view path) {
  const auto expected = path_segments(path_template);
  const auto actual = path_segments(path);
  if (expected.size() != actual.size()) return false;
  for (std::size_t i = 0; i < expected.size(); ++i) {
    if (expected[i] == "{session_id}" || expected[i] == "{job_id}") {
      if (!is_identifier(actual[i])) return false;
    } else if (expected[i] == "{step}") {
      if (!is_decimal(actual[i])) return false;
    } else if (expected[i] != actual[i]) {
      return false;
    }
  }
  return true;
}

bool route_matches(const EditionRouteRule& rule, std::string_view method,
                   std::string_view path) {
  if (method != rule.method) return false;
  if (rule.shape == EditionRouteShape::Literal) {
    return path == rule.path_template;
  }
  return structured_v1_match(rule.path_template, path);
}

EditionRouteDecision classify_route_for(Edition edition, std::string_view method,
                                        std::string_view path) {
  if (method == "OPTIONS") return {EditionRouteAccess::Retained, "cors"};
  if (path != "/api" && !path.starts_with("/api/")) {
    return {EditionRouteAccess::Retained, "static_content"};
  }
  if (edition == Edition::Full) {
    return {EditionRouteAccess::Retained, "full_edition"};
  }
  for (const auto& rule : kRouteManifest) {
    if (!route_matches(rule, method, path)) continue;
    const auto access = edition == Edition::Trial ? rule.trial_access
                                                   : rule.resilience_access;
    return {access, std::string(rule.feature)};
  }
  return {EditionRouteAccess::Unclassified, "unclassified_api"};
}

json workflow_json(Edition edition) {
  if (edition == Edition::Resilience) {
    return json::array({
        {{"id", "metric_selection"}, {"label", "指标选择"}},
        {{"id", "scenario_selection"}, {"label", "场景生成与选择"}},
        {{"id", "proactive_defense"}, {"label", "主动防御"}},
        {{"id", "rapid_recovery"}, {"label", "快速恢复"}},
        {{"id", "metric_output"}, {"label", "指标输出"}},
    });
  }
  return json::array({
      {{"id", "modeling"}, {"label", "模型建立"}},
      {{"id", "parameter_validation"}, {"label", "参数校核"}},
      {{"id", "indicator_design"}, {"label", "指标设计"}},
      {{"id", "panoramic_simulation"}, {"label", "全景仿真"}},
      {{"id", "weak_link_identification"}, {"label", "薄弱辨识"}},
  });
}

json resilience_metric_catalog_json() {
  json entries = json::array();
  for (const auto& definition : hacdcpf::analysis::resilience_metric_catalog()) {
    entries.push_back({
        {"id", definition.id},
        {"name_zh", definition.name_zh},
        {"name_en", definition.name_en},
        {"symbol", definition.symbol},
        {"phase", definition.phase},
        {"topic", definition.topic},
        {"formula_ref", definition.formula_ref},
        {"unit", definition.unit},
        {"direction", definition.direction},
        {"calculation_scope", definition.calculation_scope},
        {"availability", definition.availability},
        {"required_inputs", definition.required_inputs},
        {"source_notes", definition.source_notes},
        {"limitations", definition.limitations},
    });
  }
  return {{"schema", "resilience_metric_catalog_v1"},
          {"definition_version", hacdcpf::analysis::resilience_metric_definition_version},
          {"entries", std::move(entries)}};
}
json indicators_json(Edition edition) {
  const json indicators = json::array({
      {{"id", "system_economic"}, {"level", "system"},
       {"dimension", "economic"}, {"label", "系统经济性"}},
      {{"id", "user_economic"}, {"level", "user"},
       {"dimension", "economic"}, {"label", "用户经济性"}},
      {{"id", "system_carbon"}, {"level", "system"},
       {"dimension", "carbon"}, {"label", "系统碳指标"}},
      {{"id", "user_carbon"}, {"level", "user"},
       {"dimension", "carbon"}, {"label", "用户碳指标"}},
      {{"id", "system_reliability"}, {"level", "system"},
       {"dimension", "reliability"}, {"label", "系统可靠性"}},
      {{"id", "user_reliability"}, {"level", "user"},
       {"dimension", "reliability"}, {"label", "用户可靠性"}},
      {{"id", "system_resilience"}, {"level", "system"},
       {"dimension", "resilience"}, {"label", "系统弹性"}},
      {{"id", "user_resilience"}, {"level", "user"},
       {"dimension", "resilience"}, {"label", "用户弹性"}},
  });
  if (edition != Edition::Resilience) return indicators;

  json enabled = json::array();
  for (const auto& indicator : indicators) {
    if (indicator.at("dimension") != "carbon") enabled.push_back(indicator);
  }
  return enabled;
}

}  // namespace

#if defined(HACDCPF_TRIAL_EDITION) && defined(HACDCPF_RESILIENCE_EDITION)
#error "Trial and Resilience edition macros are mutually exclusive"
#endif

Edition current_edition() {
#if defined(HACDCPF_TRIAL_EDITION)
  return Edition::Trial;
#elif defined(HACDCPF_RESILIENCE_EDITION)
  return Edition::Resilience;
#else
  return Edition::Full;
#endif
}

std::string_view edition_name(Edition edition) {
  switch (edition) {
    case Edition::Trial:
      return "trial";
    case Edition::Resilience:
      return "resilience";
    case Edition::Full:
      return "full";
  }
  return "full";
}

bool edition_gating_enabled() { return current_edition() != Edition::Full; }

bool trial_edition_enabled() { return current_edition() == Edition::Trial; }

bool resilience_edition_enabled() {
  return current_edition() == Edition::Resilience;
}

std::span<const EditionRouteRule> edition_route_manifest() {
  return kRouteManifest;
}

V1AnalysisDecision classify_v1_analysis(std::string_view analysis) {
  if (std::find(kV1Analyses.begin(), kV1Analyses.end(), analysis) !=
      kV1Analyses.end()) {
    return {V1AnalysisAccess::Enabled, std::string(analysis)};
  }
  for (const auto& rule : kRouteManifest) {
    if (rule.analysis_id == analysis) {
      return {V1AnalysisAccess::KnownDisabled, std::string(rule.analysis_id)};
    }
  }
  return {};
}

bool v1_executor_supports_analysis(std::string_view analysis) {
  return classify_v1_analysis(analysis).access == V1AnalysisAccess::Enabled;
}

json edition_analyses_json() {
  json analyses = json::array();
  for (const auto analysis : kV1Analyses) {
    if (classify_v1_analysis(analysis).access == V1AnalysisAccess::Enabled) {
      analyses.push_back(analysis);
    }
  }
  return analyses;
}

json edition_known_disabled_analyses_json() {
  json analyses = json::array();
  for (const auto& rule : kRouteManifest) {
    if (!rule.analysis_id.empty() &&
        classify_v1_analysis(rule.analysis_id).access ==
            V1AnalysisAccess::KnownDisabled) {
      analyses.push_back(rule.analysis_id);
    }
  }
  return analyses;
}

json edition_analysis_catalog_json() {
  const Edition edition = current_edition();
  json entries = json::array();
  for (const auto& rule : kRouteManifest) {
    if (rule.analysis_id.empty()) continue;
    EditionRouteAccess access = EditionRouteAccess::Retained;
    if (edition == Edition::Trial) {
      access = rule.trial_access;
    } else if (edition == Edition::Resilience) {
      access = rule.resilience_access;
    }
    entries.push_back({{"name", rule.analysis_id},
                       {"method", rule.method},
                       {"route", rule.path_template},
                       {"enabled", access == EditionRouteAccess::Retained}});
  }
  return {{"schema", "hacdcpf.edition-analysis-catalog.v1"},
          {"entries", std::move(entries)}};
}

json edition_profile_json() {
  const Edition edition = current_edition();
  json profile{{"schema", "hacdcpf.edition-profile.v1"},
               {"edition", edition_name(edition)},
               {"product_name", edition == Edition::Trial
                                    ? "HySim-XJTU-HRPES Trial"
                                    : edition == Edition::Resilience
                                          ? "PowerSystemResilience"
                                          : "HySim-XJTU-HRPES"},
               {"analyses", edition_analyses_json()},
               {"analysis_catalog", edition_analysis_catalog_json()},
               {"workflow", workflow_json(edition)},
               {"indicators", indicators_json(edition)}};
  if (edition == Edition::Resilience) {
    profile["scenario_hazards"] = hacdcpf::analysis::weather_hazard_schema();
    profile["resilience_metric_catalog"] = resilience_metric_catalog_json();
  }
  if (edition == Edition::Full) {
    profile["enabled_modules"] = json::array({"*"});
    profile["frontend_modules"] = json::array({"*"});
    profile["enabled_io_formats"] = json::array({"*"});
    profile["disabled_features"] = json::array();
    return profile;
  }
  if (edition == Edition::Trial) {
    profile["enabled_modules"] = json::array(
        {"model_io", "parameter_validation", "topology_analysis",
         "indicator_design", "scenario_generation", "power_flow", "opf",
         "line_loss", "carbon_flow", "voltage_compliance", "hosting_capacity",
         "reliability", "resilience", "short_circuit",
         "multidimensional_weak_links"});
    profile["frontend_modules"] = json::array(
        {"modelIO", "parameterLibrary", "topologyAnalysis", "indicatorDesign",
         "scenarioGeneration", "powerFlow", "opf", "shortCircuit", "hosting",
         "reliability", "resilience", "carbonFlow", "weakLinks"});
    profile["enabled_io_formats"] =
        json::array({"json", "matpower", "gridlabd", "opendss"});
    profile["disabled_features"] = json::array(
        {"reactive_power_optimization", "harmonics", "dynamics", "market",
         "integrated_energy", "ev_traffic", "time_series",
         "network_reconfiguration", "counterfactual_planning", "sppt_agent",
         "advanced_io"});
  } else {
    profile["enabled_modules"] = json::array(
        {"model", "model_io", "parameter_validation", "projection",
         "attribution", "graph", "topology", "power_flow",
         "optimal_power_flow", "reliability", "resilience",
         "network_reconfiguration", "scenario_generation", "typhoon_faults",
         "short_circuit", "resilience_profiles", "mipsolvers"});
    profile["frontend_modules"] = json::array(
        {"modelIO", "parameterLibrary", "topologyAnalysis",
         "scenarioGeneration", "powerFlow", "opf", "resilience",
         "proactiveDefense", "rapidRecovery", "resilienceMetrics"});
    profile["enabled_io_formats"] = json::array({"json", "matpower"});
    profile["disabled_features"] = json::array(
        {"reactive_power_optimization", "harmonics", "dynamics", "market",
         "carbon", "integrated_energy", "ev_traffic", "time_series",
         "hosting_capacity", "weak_links", "counterfactual_planning",
         "sppt_agent", "advanced_io"});
    profile["solver_capabilities"] = json::array(
        {"ac_power_flow", "dc_optimal_power_flow", "ac_optimal_power_flow",
         "highs", "native_branch_and_cut", "aml"});
    profile["model_scope"] =
        "AC/DC hybrid resilience assessment and restoration; capability-specific limitations remain part of each result.";
    profile["limitations"] = json::array(
        {"Standalone transient and small-signal research endpoints are disabled in this first release.",
         "Restoration feasibility is not a certified dynamic-safety result unless an explicit certification result says so.",
         "Fallbacks, approximations, time limits, model coverage, and skipped physical validation remain explicit result limitations."});
    profile["restoration_certification"] =
        {{"ordinary_feasibility_is_certified_safe", false},
         {"dynamic_certification", "not_exposed_in_first_release"}};
  }
  profile["route_policy"] =
      {{"mode", "fail_closed"}, {"unknown_api_routes", "disabled"}};
  return profile;
}

json edition_analysis_plan_json(const json& request) {
  if (!request.is_object()) {
    throw std::invalid_argument("analysis plan request must be an object");
  }
  const json selected = request.value("indicators", json::array());
  if (!selected.is_array()) {
    throw std::invalid_argument("indicators must be an array");
  }
  std::set<std::string> dimensions;
  for (const auto& value : selected) {
    if (!value.is_string()) {
      throw std::invalid_argument("indicator entries must be strings");
    }
    const std::string indicator_id = value.get<std::string>();
    const auto separator = indicator_id.find('_');
    if (separator == std::string::npos ||
        (indicator_id.substr(0, separator) != "system" &&
         indicator_id.substr(0, separator) != "user")) {
      throw std::invalid_argument("unsupported indicator: " + indicator_id);
    }
    std::string indicator = indicator_id.substr(separator + 1);
    if (indicator != "economic" && indicator != "carbon" &&
        indicator != "reliability" && indicator != "resilience") {
      throw std::invalid_argument("unsupported indicator: " + indicator_id);
    }
    if (resilience_edition_enabled() && indicator == "carbon") {
      throw std::invalid_argument(
          "indicator is not enabled in the resilience edition: " + indicator_id);
    }
    dimensions.insert(std::move(indicator));
  }

  if (resilience_edition_enabled()) {
    return {{"schema", "hacdcpf.edition-analysis-plan.v1"},
            {"indicators", selected},
            {"steps", workflow_json(Edition::Resilience)},
            {"automatic_execution", false},
            {"model_scope",
             "Resilience workflow: metric selection, scenario selection, proactive defense skipped when unavailable, rapid recovery, and metric output."}};
  }

  json steps = json::array();
  auto add = [&steps](const char* module, const char* label,
                      const char* scenario) {
    steps.push_back({{"module", module}, {"label", label},
                     {"scenario_family", scenario},
                     {"execution", "user_confirmed"}});
  };
  if (dimensions.contains("economic") || dimensions.contains("carbon")) {
    add("powerFlow", "潮流/线损/电压", "normal");
  }
  if (dimensions.contains("economic")) {
    add("opf", "最优潮流", "normal");
    add("hosting", "承载力", "normal");
  }
  if (dimensions.contains("carbon")) {
    add("carbonFlow", "碳流", "normal");
  }
  if (dimensions.contains("reliability") || dimensions.contains("resilience")) {
    add("scenarioGeneration", "场景生成", "fault");
  }
  if (dimensions.contains("reliability")) {
    add("reliability", "可靠性", "expected_fault");
  }
  if (dimensions.contains("resilience")) {
    add("resilience", "弹性", "unexpected_fault");
  }
  if (!dimensions.empty()) add("shortCircuit", "短路", "fault_evidence");
  if (dimensions.size() > 1) {
    add("weakLinks", "多维薄弱环节", "cross_scenario");
  }
  return {{"schema", "hacdcpf.edition-analysis-plan.v1"},
          {"indicators", selected},
          {"steps", std::move(steps)},
          {"automatic_execution", false},
          {"model_scope",
           "The plan orders enabled analyses; each solver run requires explicit user confirmation."}};
}

EditionRouteDecision classify_edition_route(std::string_view method,
                                            std::string_view path) {
  return classify_route_for(current_edition(), method, path);
}

EditionRouteDecision classify_trial_route(std::string_view method,
                                          std::string_view path) {
  return classify_route_for(Edition::Trial, method, path);
}

}  // namespace hacdcpf::server
