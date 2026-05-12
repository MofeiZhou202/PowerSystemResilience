#pragma once
#include <string>

#include "hacdcpf/analysis/carbon_analysis.hpp"
#include "hacdcpf/analysis/time_series_pf.hpp"
#include "hacdcpf/model/results.hpp"
#include "hacdcpf/model/system.hpp"
#include "hacdcpf/optimal_power_flow/ac_opf.hpp"

namespace hacdcpf::io {

// ── HybridPowerSystem serialisation ──────────────────────────────────────────

std::string to_json(const HybridPowerSystem& sys, int indent = 2);

HybridPowerSystem from_json(const std::string& json_str);

void save_json(const HybridPowerSystem& sys, const std::string& path,
               int indent = 2);

HybridPowerSystem load_json(const std::string& path);

// ── JPC (Julia Power Case) format ────────────────────────────────────────────

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
