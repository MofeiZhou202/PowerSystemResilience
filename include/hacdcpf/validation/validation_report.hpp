#pragma once

#include <string>
#include <vector>

namespace hacdcpf::validation {

// ── Severity ─────────────────────────────────────────────────────────────────

enum class Severity {
    Info,     ///< Informational note; does not prevent solving.
    Warning,  ///< Degraded results may occur; solving proceeds.
    Error     ///< Fatal; the system cannot be solved as-is.
};

// ── Single diagnostic issue ───────────────────────────────────────────────────

struct ValidationIssue {
    Severity    severity{Severity::Error};
    std::string component_type; ///< e.g. "ACBus", "ACBranch", "VSCConverter"
    std::string component_id;   ///< external ID / name of the offending component
    std::string field;          ///< field name that is problematic (may be empty)
    std::string message;        ///< human-readable description
};

// ── Aggregate report ──────────────────────────────────────────────────────────

struct ValidationReport {
    std::vector<ValidationIssue> issues;

    /// Returns true if there are no Error-severity issues.
    [[nodiscard]] bool is_valid() const noexcept {
        for (const auto& i : issues)
            if (i.severity == Severity::Error) return false;
        return true;
    }

    /// Shorthand: any fatal errors present?
    [[nodiscard]] bool has_errors() const noexcept   { return !is_valid(); }

    /// Any warnings (excluding errors)?
    [[nodiscard]] bool has_warnings() const noexcept {
        for (const auto& i : issues)
            if (i.severity == Severity::Warning) return true;
        return false;
    }

    /// Number of issues at or above the given severity.
    [[nodiscard]] int count(Severity s) const noexcept {
        int n = 0;
        for (const auto& i : issues) if (i.severity == s) ++n;
        return n;
    }

    void add(Severity s, std::string type, std::string id,
             std::string field, std::string msg) {
        issues.push_back({s, std::move(type), std::move(id),
                          std::move(field), std::move(msg)});
    }

    /// Alias for is_valid() — no Error-severity issues present.
    [[nodiscard]] bool ok() const noexcept { return is_valid(); }

    /// Returns a single human-readable summary string.
    /// Example: "OK" / "2 error(s), 1 warning(s): [ACBus/1/vmin_pu] ..."
    [[nodiscard]] std::string summary() const {
        int nerr  = count(Severity::Error);
        int nwarn = count(Severity::Warning);
        if (nerr == 0 && nwarn == 0) return "OK";
        std::string s;
        if (nerr  > 0) s += std::to_string(nerr)  + " error(s)";
        if (nwarn > 0) {
            if (!s.empty()) s += ", ";
            s += std::to_string(nwarn) + " warning(s)";
        }
        s += ":";
        for (const auto& i : issues) {
            s += " [" + i.component_type;
            if (!i.component_id.empty()) s += "/" + i.component_id;
            if (!i.field.empty())        s += "/" + i.field;
            s += "] " + i.message + ";";
        }
        return s;
    }
};

}  // namespace hacdcpf::validation
