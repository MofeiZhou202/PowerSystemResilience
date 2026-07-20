#pragma once

/// io/bpa_io.hpp
/// =============
/// PSD-BPA / DSP card-file (.dat) importer.
///
/// Parses the fixed-column power-flow data cards used by the Chinese DSP
/// (CSGStudio) and PSD-BPA programs into a HybridPowerSystem:
///   * AC bus cards        B / BS / BE / BQ
///   * AC branch cards     L (line), T (two-winding transformer)
///   * Two-terminal HVDC   BD (converter node), LD (DC line)
/// Control cards ((...), /..., >...<, comments) are consumed for MVA base and
/// case id; unsupported cards are skipped with a report record.
///
/// LCC converter stations are approximated by VSC converters (the model
/// library has no LCC element): the rectifier runs PQ_MODE drawing the
/// scheduled power, the inverter runs VDC_Q forming the DC voltage so the
/// delivered power emerges from the island balance.  The approximation and
/// every skipped record are documented in the uniform ImportReport (docs §8).

#include <string>

#include "hacdcpf/io/import_report.hpp"
#include "hacdcpf/model/hybrid_power_system.hpp"

namespace hacdcpf::io {

/// Options controlling BPA/DSP import.
struct BpaImportOptions {
  /// Strict rejects the import when any record is coerced / skipped;
  /// Permissive records diagnostics and proceeds.
  ImportMode mode{ImportMode::Permissive};
  /// Reactive-power absorption estimate of an LCC converter station as a
  /// fraction of its active-power transfer (Q = ratio * P).  LCC stations
  /// typically consume 50-60% of the transmitted active power.
  double lcc_q_ratio{0.5};
};

/// BPA/DSP import result carrying the reconstructed system and a uniform
/// import report (docs §8).
struct BpaImportResult {
  HybridPowerSystem system;
  ImportReport report;
};

/// Parse a BPA/DSP .dat card file and return the system plus import report.
/// GBK-encoded files (Chinese bus names) are handled: card columns are read
/// in the raw byte domain and name fields are converted to UTF-8.
BpaImportResult parse_bpa_dat(const std::string& filepath,
                              const BpaImportOptions& options = {});

/// Parse BPA/DSP card text held in memory (used by the REST import path).
BpaImportResult parse_bpa_dat_string(const std::string& content,
                                     const BpaImportOptions& options = {});

}  // namespace hacdcpf::io
