#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

/// @file import_report.hpp
/// Uniform, machine-readable import-diagnostic contract shared by every
/// importer (JSON, MATPOWER, GridLAB-D, OpenDSS, ETAP, CIM).
///
/// See docs/digital_twin_data_io_architecture.md:
///   - §8  Import Diagnostic Contract  (this vocabulary)
///   - §3.2 rich vs. canonical binding level
///   - §6.3 unit discipline (asserted / inferred / best-effort)
///
/// The types here are deliberately dependency-free so the header can be
/// included by any adapter without pulling in the model or nlohmann::json.

namespace hacdcpf::io {

// ── Import mode ──────────────────────────────────────────────────────────────

/// Controls strictness of any import (JSON / Excel / external formats).
///
/// Canonically defined here because it is part of the import contract; other
/// headers (e.g. json_io.hpp) include this header for it.
enum class ImportMode {
  /// Unknown fields, invalid enum strings, missing required fields, and any
  /// coerced/inferred value cause the import to be rejected.
  Strict,
  /// Unknown fields / invalid enums / coercions are recorded as diagnostics and
  /// the import proceeds; hard errors still fail.
  Permissive,
};

// ── Disposition, severity, and provenance of a single imported record ─────────

/// What happened to one source record during import.
enum class ImportDisposition {
  Accepted,  ///< Imported verbatim.
  Coerced,   ///< Imported after defaulting / clamping / unit inference.
  Rejected,  ///< Not imported (hard error for this record).
  Skipped,   ///< Intentionally ignored (unsupported / out-of-scope feature).
};

/// Severity of a single import record, independent of parameter-audit severity.
enum class ImportSeverity { Info, Warning, Error };

/// Whether an importer reconstructs rich components or lands at canonical level.
/// See §3.2: canonical-binding importers must not pretend to recover rich
/// structure; the structural loss is recorded and downgrades readiness.
enum class ImportBindingLevel { Rich, Canonical };

/// How units were established for the imported model (see §6.3).
enum class UnitAssertion {
  Asserted,    ///< Units declared explicitly in the source (twin-path safe).
  Inferred,    ///< Units inferred from the source's own declared conventions.
  BestEffort,  ///< Units guessed heuristically; provenance downgraded.
};

/// Stable, enumerated reason codes so reports are machine-readable.
enum class ImportReasonCode {
  Ok,
  UnknownField,
  UnresolvedBusRef,
  UnitInferred,
  UnsupportedControl,
  RangeCoerced,
  DuplicateId,
  StructuralLoss,
  MissingRequired,
  InvalidEnum,
  XmlEntityRejected,
  SizeCapExceeded,
  SchemaMigrated,
  ParseError,
  Other,
};

/// One diagnostic record for one source object.
struct ImportRecord {
  std::string source_locator;             ///< file line / sheet+row / xpath.
  std::optional<std::string> target_ref;  ///< component ref; empty if rejected.
  ImportDisposition disposition{ImportDisposition::Accepted};
  ImportReasonCode reason_code{ImportReasonCode::Ok};
  ImportSeverity severity{ImportSeverity::Info};
  std::string message;
};

/// Disposition tally for a whole import.
struct ImportSummary {
  std::size_t accepted{0};
  std::size_t coerced{0};
  std::size_t rejected{0};
  std::size_t skipped{0};
};

/// Uniform result emitted by every importer (§8).
struct ImportReport {
  std::vector<ImportRecord> records;
  ImportSummary summary;
  ImportBindingLevel binding_level{ImportBindingLevel::Rich};
  UnitAssertion unit_assertion{UnitAssertion::Asserted};

  /// Appends @p rec and updates @c summary from its disposition.
  void add(ImportRecord rec);

  /// Convenience appender.
  void add(ImportDisposition disposition,
           ImportReasonCode reason_code,
           ImportSeverity severity,
           std::string source_locator,
           std::string message,
           std::optional<std::string> target_ref = std::nullopt);

  [[nodiscard]] std::size_t count(ImportSeverity severity) const;
  [[nodiscard]] bool has_errors() const;
  /// True if any record was coerced or units were inferred / best-effort.
  [[nodiscard]] bool has_coerced_or_inferred() const;
};

/// Whether an import result is acceptable under @p mode (§8).
///
/// - Strict:     no Error-severity records AND nothing coerced/inferred.
/// - Permissive: no Error-severity records (coercions/inferences are allowed
///               and are recorded in the report).
[[nodiscard]] bool passes_mode(const ImportReport& report, ImportMode mode);

std::string to_string(ImportMode mode);
std::string to_string(ImportDisposition disposition);
std::string to_string(ImportSeverity severity);
std::string to_string(ImportBindingLevel level);
std::string to_string(UnitAssertion assertion);
std::string to_string(ImportReasonCode code);

}  // namespace hacdcpf::io
