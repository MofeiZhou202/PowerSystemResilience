#pragma once

/// io/bpa_io.hpp
/// =============
/// PSD-BPA / DSP card-file (.dat) importer and exporter.
///
/// Parses the fixed-column power-flow data cards used by the Chinese DSP
/// (CSGStudio) and PSD-BPA programs into a HybridPowerSystem:
///   * AC bus cards        B / BS / BE / BQ
///   * AC branch cards     L (line), T (two-winding transformer)
///   * Two-terminal HVDC   BD (converter node), LD (DC line)
///   * VSC-HVDC            BZ / BZ+ (converter station), LZ (DC line)
/// Control cards ((...), /..., >...<, comments) are consumed for MVA base and
/// case id; unsupported cards are skipped with a report record.
///
/// LCC converter stations can be imported two ways (see BpaImportOptions::
/// lcc_model):
///   * VscApprox (legacy opt-in) — each station is approximated by a VSC
///     converter:
///     the rectifier runs PQ_MODE drawing the scheduled power, the inverter
///     runs VDC_Q forming the DC voltage so the delivered power emerges from
///     the island balance.
///   * LccQuasiSteady (default) — each LD link produces a DCBranch plus two
///     native
///     LCCConverter elements carrying the full BD/LD quasi-steady parameters
///     (bridge count, alpha/gamma limits, valve drop, commutation reactance
///     from the converter transformer T card, control setpoints).  The unified
///     Newton power flow consumes the native AC-P/AC-Q/DC coupling directly;
///     the shared Parity/Ipopt OPF formulation consumes the same quasi-steady
///     characteristic.  Fixed taps and other declared limitations remain on
///     each element's model_scope / model_limitations.
/// The approximation and every skipped record are documented in the uniform
/// ImportReport (docs §8).

#include <string>

#include "hacdcpf/io/import_report.hpp"
#include "hacdcpf/model/hybrid_power_system.hpp"

namespace hacdcpf::io {

/// How two-terminal LCC HVDC links (BD/LD cards) are represented.
enum class BpaLccModel {
  /// Approximate each station by a VSC converter (legacy behavior).
  VscApprox = 0,
  /// Import native LCCConverter quasi-steady elements; consumed by the
  /// unified Newton power flow (lcc-quasi-steady model, fixed taps).
  LccQuasiSteady = 1,
};

/// Options controlling BPA/DSP import.
struct BpaImportOptions {
  /// Strict rejects the import when any record is coerced / skipped;
  /// Permissive records diagnostics and proceeds.
  ImportMode mode{ImportMode::Permissive};
  /// Reactive-power absorption estimate of an LCC converter station as a
  /// fraction of its active-power transfer (Q = ratio * P).  LCC stations
  /// typically consume 50-60% of the transmitted active power.  Only used by
  /// the VscApprox path.
  double lcc_q_ratio{0.5};
  /// LCC station representation for BD/LD links; default imports the native
  /// quasi-steady LCC model (the legacy VSC approximation remains available
  /// via VscApprox).
  BpaLccModel lcc_model{BpaLccModel::LccQuasiSteady};
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

/// Export the BPA-representable steady-state subset as fixed-column card text.
///
/// BPA bus identifiers are limited to eight bytes.  The exporter therefore
/// assigns stable ASCII identifiers (A0000001, A0000002, ...) so UTF-8 model
/// names cannot corrupt fixed columns.  AC buses, aggregated bus load/shunt/
/// generator data, canonical AC branches, and two-terminal DC links backed by
/// one VSC at each terminal are represented.  Rich assets outside that subset
/// must be reported by the caller as model limitations.
std::string to_bpa_dat(const HybridPowerSystem& system);

/// Export to a BPA/DSP .dat file.
void save_bpa_dat(const HybridPowerSystem& system,
                  const std::string& filepath);

}  // namespace hacdcpf::io
