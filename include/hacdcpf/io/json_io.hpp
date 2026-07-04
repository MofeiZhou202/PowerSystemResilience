#pragma once
#include <string>

#include "hacdcpf/carbon_analysis/carbon_analysis.hpp"
#include "hacdcpf/time_series/time_series_pf.hpp"
#include "hacdcpf/model/defaults.hpp"
#include "hacdcpf/model/error.hpp"
#include "hacdcpf/power_flow/power_flow_result.hpp"
#include "hacdcpf/model/hybrid_power_system.hpp"
#include "hacdcpf/optimal_power_flow/opf_result.hpp"
#include "hacdcpf/io/import_report.hpp"  // ImportMode + uniform ImportReport contract

namespace hacdcpf::io {
// ── Import mode ──────────────────────────────────────────────────────────────
// ImportMode is defined canonically in import_report.hpp (it is part of the
// import contract). It remains in namespace hacdcpf::io, so existing users are
// unaffected.

// ── Schema versioning ─────────────────────────────────────────────────────────

/// Current on-disk JSON schema version for HybridPowerSystem.
/// Increment the minor version when adding optional fields; increment the
/// major version (and bump kSchemaVersion in defaults.hpp) when making
/// backward-incompatible changes.
inline constexpr const char* kCurrentSchemaVersion  = Defaults::kSchemaVersion;
inline constexpr const char* kCurrentPackageVersion = Defaults::kPackageVersion;

/// Check whether a JSON string's "schema_version" field is compatible with
/// this build.  Returns an Error if incompatible, empty Result<void> otherwise.
/// Compatibility: same major version is required; minor version may differ.
Result<bool> check_schema_version(const std::string& json_str);

// ── HybridPowerSystem serialisation ──────────────────────────────────────────

std::string to_json(const HybridPowerSystem& sys, int indent = 2);

HybridPowerSystem from_json(const std::string& json_str);

void save_json(const HybridPowerSystem& sys, const std::string& path,
               int indent = 2);

HybridPowerSystem load_json(const std::string& path);
/// Exception-free variants — return Result<HybridPowerSystem> instead of
/// throwing.  On failure the Result holds an Error with an appropriate
/// ErrorCode (ParseError, FileNotFound, SchemaVersionMismatch).
Result<HybridPowerSystem> try_from_json(const std::string& json_str,
                                        ImportMode mode = ImportMode::Strict);

Result<HybridPowerSystem> try_load_json(const std::string& path,
                                        ImportMode mode = ImportMode::Strict);
// ── JPC matrix-case format ───────────────────────────────────────────────────

std::string to_jpc_json(const HybridPowerSystem& sys, int indent = 2);

HybridPowerSystem from_jpc_json(const std::string& json_str);

void save_jpc_json(const HybridPowerSystem& sys, const std::string& path,
                   int indent = 2);

HybridPowerSystem load_jpc_json(const std::string& path);

// ── Power flow result serialisation ──────────────────────────────────────────

std::string power_flow_result_to_json(const HybridPowerSystem& sys,
                                      const PowerFlowResult& result,
                                      int indent = 2);

void save_power_flow_result_json(const HybridPowerSystem& sys,
                                 const PowerFlowResult& result,
                                 const std::string& path);

// ── OPF result serialisation ─────────────────────────────────────────────────

std::string opf_result_to_json(const opf::ACOPFResult& result, int indent = 2);

void save_opf_result_json(const opf::ACOPFResult& result,
                          const std::string& path);

opf::ACOPFResult opf_result_from_json(const std::string& json_str);

/// Serialise a DC OPF result including solver_chain and objective_model fields.
std::string dc_opf_result_to_json(const opf::DCOPFResult& result, int indent = 2);

opf::DCOPFResult dc_opf_result_from_json(const std::string& json_str);

void save_dc_opf_result_json(const opf::DCOPFResult& result,
                             const std::string& path, int indent = 2);

opf::DCOPFResult load_dc_opf_result_json(const std::string& path);

opf::ACOPFResult load_opf_result_json(const std::string& path);

// ── Carbon analysis result serialisation ─────────────────────────────────────

std::string carbon_result_to_json(const analysis::CarbonAnalysisResult& result,
                                  int indent = 2);

void save_carbon_result_json(const analysis::CarbonAnalysisResult& result,
                             const std::string& path, int indent = 2);

analysis::CarbonAnalysisResult carbon_result_from_json(
    const std::string& json_str);

analysis::CarbonAnalysisResult load_carbon_result_json(
    const std::string& path);

// ── Time-series data serialisation ───────────────────────────────────────────

std::string time_series_data_to_json(const TimeSeriesData& ts, int indent = 2);

TimeSeriesData time_series_data_from_json(const std::string& json_str);

}  // namespace hacdcpf::io
