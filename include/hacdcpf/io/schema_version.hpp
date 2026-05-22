#pragma once

/// io/schema_version.hpp
/// ======================
/// Schema and package version constants, plus system-level numeric defaults
/// for JSON/Excel import and MATPOWER parsing.
/// Replaces: model/defaults.hpp (promoted to public API).

namespace hacdcpf {
namespace io {

/// JSON schema version written to all serialised documents.
static constexpr const char* kSchemaVersion  = "1.0";
/// Library package version.
static constexpr const char* kPackageVersion = "0.5.0";

}  // namespace io
}  // namespace hacdcpf

// Also expose the Defaults struct at the hacdcpf namespace level (unchanged).
#include "hacdcpf/model/defaults.hpp"
