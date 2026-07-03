#include "hacdcpf/io/import_report.hpp"

#include <utility>

namespace hacdcpf::io {

void ImportReport::add(ImportRecord rec) {
  switch (rec.disposition) {
    case ImportDisposition::Accepted: ++summary.accepted; break;
    case ImportDisposition::Coerced:  ++summary.coerced;  break;
    case ImportDisposition::Rejected: ++summary.rejected; break;
    case ImportDisposition::Skipped:  ++summary.skipped;  break;
  }
  records.push_back(std::move(rec));
}

void ImportReport::add(ImportDisposition disposition,
                       ImportReasonCode reason_code,
                       ImportSeverity severity,
                       std::string source_locator,
                       std::string message,
                       std::optional<std::string> target_ref) {
  add(ImportRecord{std::move(source_locator),
                   std::move(target_ref),
                   disposition,
                   reason_code,
                   severity,
                   std::move(message)});
}

std::size_t ImportReport::count(ImportSeverity severity) const {
  std::size_t n = 0;
  for (const auto& r : records)
    if (r.severity == severity) ++n;
  return n;
}

bool ImportReport::has_errors() const {
  return count(ImportSeverity::Error) > 0;
}

bool ImportReport::has_coerced_or_inferred() const {
  if (unit_assertion != UnitAssertion::Asserted) return true;
  for (const auto& r : records)
    if (r.disposition == ImportDisposition::Coerced) return true;
  return false;
}

ImportReport uniform_report_from_messages(
    const std::vector<std::string>& warnings,
    const std::vector<std::string>& skipped, ImportBindingLevel level,
    UnitAssertion unit_assertion) {
  ImportReport report;
  report.binding_level = level;
  report.unit_assertion = unit_assertion;
  for (const auto& w : warnings) {
    report.add(ImportDisposition::Coerced, ImportReasonCode::Other,
               ImportSeverity::Warning, std::string(), w);
  }
  for (const auto& s : skipped) {
    report.add(ImportDisposition::Skipped, ImportReasonCode::StructuralLoss,
               ImportSeverity::Warning, std::string(), s);
  }
  return report;
}

bool passes_mode(const ImportReport& report, ImportMode mode) {
  if (report.has_errors()) return false;
  if (mode == ImportMode::Strict) return !report.has_coerced_or_inferred();
  return true;
}

std::string to_string(ImportMode mode) {
  switch (mode) {
    case ImportMode::Strict:     return "strict";
    case ImportMode::Permissive: return "permissive";
  }
  return "unknown";
}

std::string to_string(ImportDisposition disposition) {
  switch (disposition) {
    case ImportDisposition::Accepted: return "accepted";
    case ImportDisposition::Coerced:  return "coerced";
    case ImportDisposition::Rejected: return "rejected";
    case ImportDisposition::Skipped:  return "skipped";
  }
  return "unknown";
}

std::string to_string(ImportSeverity severity) {
  switch (severity) {
    case ImportSeverity::Info:    return "info";
    case ImportSeverity::Warning: return "warning";
    case ImportSeverity::Error:   return "error";
  }
  return "unknown";
}

std::string to_string(ImportBindingLevel level) {
  switch (level) {
    case ImportBindingLevel::Rich:      return "rich";
    case ImportBindingLevel::Canonical: return "canonical";
  }
  return "unknown";
}

std::string to_string(UnitAssertion assertion) {
  switch (assertion) {
    case UnitAssertion::Asserted:   return "asserted";
    case UnitAssertion::Inferred:   return "inferred";
    case UnitAssertion::BestEffort: return "best_effort";
  }
  return "unknown";
}

std::string to_string(ImportReasonCode code) {
  switch (code) {
    case ImportReasonCode::Ok:                 return "ok";
    case ImportReasonCode::UnknownField:       return "unknown_field";
    case ImportReasonCode::UnresolvedBusRef:   return "unresolved_bus_ref";
    case ImportReasonCode::UnitInferred:       return "unit_inferred";
    case ImportReasonCode::UnsupportedControl: return "unsupported_control";
    case ImportReasonCode::RangeCoerced:       return "range_coerced";
    case ImportReasonCode::DuplicateId:        return "duplicate_id";
    case ImportReasonCode::StructuralLoss:     return "structural_loss";
    case ImportReasonCode::MissingRequired:    return "missing_required";
    case ImportReasonCode::InvalidEnum:        return "invalid_enum";
    case ImportReasonCode::XmlEntityRejected:  return "xml_entity_rejected";
    case ImportReasonCode::SizeCapExceeded:    return "size_cap_exceeded";
    case ImportReasonCode::SchemaMigrated:     return "schema_migrated";
    case ImportReasonCode::ParseError:         return "parse_error";
    case ImportReasonCode::Other:              return "other";
  }
  return "unknown";
}

}  // namespace hacdcpf::io
