#pragma once

#include <string>

#include "hacdcpf/model/system.hpp"
#include "hacdcpf/model/results.hpp"
#include "hacdcpf/optimal_power_flow/ac_opf.hpp"
#include "hacdcpf/analysis/carbon_analysis.hpp"
#include "hacdcpf/analysis/time_series_pf.hpp"

namespace hacdcpf::io {

// ═══════════════════════════════════════════════════════════════════════
// JSON serialization / deserialization
// ═══════════════════════════════════════════════════════════════════════

// Serialize a HybridPowerSystem to a JSON string (pretty-printed with indent).
std::string to_json(const HybridPowerSystem& sys, int indent = 2);

// Write a HybridPowerSystem to a JSON file.
void save_json(const HybridPowerSystem& sys, const std::string& path,
               int indent = 2);

// Parse a JSON string into a HybridPowerSystem.
HybridPowerSystem from_json(const std::string& json_str);

// Load a HybridPowerSystem from a JSON file.
HybridPowerSystem load_json(const std::string& path);

// Serialize a power-flow result with stable component ids and derived metrics.
std::string power_flow_result_to_json(const HybridPowerSystem& sys,
                                      const PowerFlowResult& result,
                                      int indent = 2);

// Write a power-flow result JSON contract to disk.
void save_power_flow_result_json(const HybridPowerSystem& sys,
                                 const PowerFlowResult& result,
                                 const std::string& path,
                                 int indent = 2);

// Serialize an ACOPFResult to a JSON string.
std::string opf_result_to_json(const opf::ACOPFResult& result, int indent = 2);

// Write an ACOPFResult to a JSON file.
void save_opf_result_json(const opf::ACOPFResult& result,
                          const std::string& path, int indent = 2);

// Parse a JSON string into an ACOPFResult.
opf::ACOPFResult opf_result_from_json(const std::string& json_str);

// Load an ACOPFResult from a JSON file.
opf::ACOPFResult load_opf_result_json(const std::string& path);

// Serialize a CarbonAnalysisResult to a JSON string.
std::string carbon_result_to_json(const analysis::CarbonAnalysisResult& result,
                                  int indent = 2);

// Write a CarbonAnalysisResult to a JSON file.
void save_carbon_result_json(const analysis::CarbonAnalysisResult& result,
                             const std::string& path, int indent = 2);

// Parse a JSON string into a CarbonAnalysisResult.
analysis::CarbonAnalysisResult carbon_result_from_json(const std::string& json_str);

// Load a CarbonAnalysisResult from a JSON file.
analysis::CarbonAnalysisResult load_carbon_result_json(const std::string& path);

// Serialize TimeSeriesData to a JSON string.
std::string time_series_data_to_json(const TimeSeriesData& ts, int indent = 2);

// Parse a JSON string into TimeSeriesData.
TimeSeriesData time_series_data_from_json(const std::string& json_str);

// ═══════════════════════════════════════════════════════════════════════
// JPC (Julia Power Case) format serialization / deserialization
// Enables bidirectional interoperability with Julia's DistributionPowerFlow module
// ═══════════════════════════════════════════════════════════════════════

// Serialize a HybridPowerSystem to JPC-compatible JSON string (matrix format).
std::string to_jpc_json(const HybridPowerSystem& sys, int indent = 2);

// Write a HybridPowerSystem to a JPC-compatible JSON file.
void save_jpc_json(const HybridPowerSystem& sys, const std::string& path,
                   int indent = 2);

// Parse a JPC-compatible JSON string into a HybridPowerSystem.
HybridPowerSystem from_jpc_json(const std::string& json_str);

// Load a HybridPowerSystem from a JPC-compatible JSON file.
HybridPowerSystem load_jpc_json(const std::string& path);

}  // namespace hacdcpf::io
