#pragma once

/// Structured error and result types
/// ===================================
/// Provides a lightweight Result<T> monad and Error type so batch
/// simulation workflows can propagate failures without exceptions.
///
/// Usage (caller):
///
///   auto res = safe_solve_power_flow(sys, opt);
///   if (!res) {
///       std::cerr << res.error().message << "\n";
///       return;
///   }
///   use(res.value().vm);

#include <stdexcept>
#include <string>
#include <variant>
#include <vector>

#include "hacdcpf/validation/validation_report.hpp"

namespace hacdcpf {

// ── Error codes ───────────────────────────────────────────────────────────────

enum class ErrorCode {
    // Structural / input errors
    InvalidInput         = 1000,
    ValidationFailed     = 1001,
    MissingSlackBus      = 1002,
    DisconnectedNetwork  = 1003,
    SingularAdmittance   = 1004,

    // Solver errors
    NotConverged         = 2000,
    MaxIterationsReached = 2001,
    NumericalFailure     = 2002,
    JacobianSingular     = 2003,
    HomotopyFailed       = 2004,

    // I/O errors
    FileNotFound         = 3000,
    ParseError           = 3001,
    SchemaVersionMismatch = 3002,

    // General
    NotImplemented       = 9000,
    Unknown              = 9999,
};

// ── Error descriptor ──────────────────────────────────────────────────────────

struct Error {
    ErrorCode   code{ErrorCode::Unknown};
    std::string message;

    /// Optional validation report when code == ValidationFailed.
    std::vector<validation::ValidationIssue> details;

    // ── Convenience factories ─────────────────────────────────────────────
    static Error validation_failed(validation::ValidationReport report) {
        Error e;
        e.code    = ErrorCode::ValidationFailed;
        e.message = "System validation failed with " +
                    std::to_string(report.count(validation::Severity::Error)) +
                    " error(s)";
        e.details = std::move(report.issues);
        return e;
    }

    static Error not_converged(const std::string& reason) {
        return {ErrorCode::NotConverged, "Solver did not converge: " + reason, {}};
    }

    static Error parse_error(const std::string& path, const std::string& msg) {
        return {ErrorCode::ParseError, "Parse error in '" + path + "': " + msg, {}};
    }
};

// ── Result<T> ─────────────────────────────────────────────────────────────────

/// Lightweight either-type: holds a T on success or an Error on failure.
/// Intentionally header-only with zero dependencies beyond the STL.
template <typename T>
class Result {
public:
    // ── Constructors ─────────────────────────────────────────────────────────
    // NOLINTNEXTLINE(google-explicit-constructor)
    Result(T value) : storage_(std::move(value)) {}
    // NOLINTNEXTLINE(google-explicit-constructor)
    Result(Error err)  : storage_(std::move(err))  {}

    // ── Queries ───────────────────────────────────────────────────────────────
    [[nodiscard]] bool has_value() const noexcept {
        return std::holds_alternative<T>(storage_);
    }
    explicit operator bool() const noexcept { return has_value(); }

    // ── Access ────────────────────────────────────────────────────────────────
    [[nodiscard]] T& value()             { return std::get<T>(storage_); }
    [[nodiscard]] const T& value() const { return std::get<T>(storage_); }
    [[nodiscard]] T& operator*()         { return value(); }
    [[nodiscard]] const T& operator*() const { return value(); }
    [[nodiscard]] T* operator->()        { return &value(); }
    [[nodiscard]] const T* operator->() const { return &value(); }

    [[nodiscard]] Error& error()             { return std::get<Error>(storage_); }
    [[nodiscard]] const Error& error() const { return std::get<Error>(storage_); }

    // ── Convenience ───────────────────────────────────────────────────────────
    /// Return value or throw std::runtime_error with the error message.
    [[nodiscard]] T& value_or_throw() {
        if (!has_value())
            throw std::runtime_error(std::get<Error>(storage_).message);
        return value();
    }

private:
    std::variant<T, Error> storage_;
};

}  // namespace hacdcpf
