#pragma once

#include <string>

#include "hacdcpf/model/system.hpp"
#include "hacdcpf/model/results.hpp"
#include "hacdcpf/optimal_power_flow/ac_opf.hpp"
#include "hacdcpf/analysis/carbon_analysis.hpp"

namespace hacdcpf::io {

// ═══════════════════════════════════════════════════════════════════════
// Excel (.xlsx) serialization / deserialization
// Uses OpenXLSX library for round-trip Excel I/O
// ═══════════════════════════════════════════════════════════════════════

/// Serialize a HybridPowerSystem to an Excel (.xlsx) file.
/// Creates one sheet per component type, plus an "Info" sheet for system-level parameters.
void save_xlsx(const HybridPowerSystem& sys, const std::string& path);

/// Load a HybridPowerSystem from an Excel (.xlsx) file.
/// Missing sheets are silently skipped (component vectors remain empty).
HybridPowerSystem load_xlsx(const std::string& path);

/// Export power flow results to an Excel file.
/// Creates sheets for bus voltages, branch flows, and summary metrics.
void save_results_xlsx(const HybridPowerSystem& sys,
                       const PowerFlowResult& result,
                       const std::string& path);

/// Export power flow results in a compact, input-like workbook.
/// Keeps the same element sheets as the input export and appends a small set
/// of result_* columns to those existing tables.
void save_results_compact_xlsx(const HybridPowerSystem& sys,
                               const PowerFlowResult& result,
                               const std::string& path);

/// Export AC OPF results to an Excel file.
/// Creates sheets for optimal dispatch, bus voltages, load shedding,
/// converter operating points, and a summary sheet.
void save_opf_results_xlsx(const HybridPowerSystem& sys,
                           const opf::ACOPFResult& result,
                           const std::string& path);

/// Export AC OPF results in a compact workbook that also includes the
/// input data sheets alongside the results.
void save_opf_results_compact_xlsx(const HybridPowerSystem& sys,
                                   const opf::ACOPFResult& result,
                                   const std::string& path);

/// Export carbon analysis results to an Excel file.
/// Creates sheets for load carbon intensity, bus carbon intensity,
/// branch loss emissions, generator loss allocation, and a summary sheet.
void save_carbon_results_xlsx(const analysis::CarbonAnalysisResult& result,
                              const std::string& path);

}  // namespace hacdcpf::io
