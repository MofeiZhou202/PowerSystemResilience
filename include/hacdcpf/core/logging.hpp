#pragma once

// Lightweight logging facade wrapping spdlog.
// All library code should use HACDCPF_LOG_* macros instead of raw
// std::cout / std::cerr / fprintf(stderr, ...).
//
// Compile-time control:
//   - Define HACDCPF_LOG_LEVEL=<n> to set minimum compiled-in level
//     (0=trace, 1=debug, 2=info, 3=warn, 4=error, 5=critical, 6=off).
//   - Default is debug (1) in Debug builds, info (2) otherwise.
//
// Runtime control:
//   hacdcpf::logging::set_level(spdlog::level::debug);

#include <spdlog/spdlog.h>
#include <spdlog/fmt/fmt.h>

namespace hacdcpf::logging {

/// Return the shared library logger (creates on first call).
inline spdlog::logger& logger() {
  static auto inst = spdlog::default_logger();
  return *inst;
}

/// Change the runtime log level.
inline void set_level(spdlog::level::level_enum lvl) {
  spdlog::set_level(lvl);
}

}  // namespace hacdcpf::logging

// ── Convenience macros ──────────────────────────────────────────────────
// These forward to spdlog macros, which respect SPDLOG_ACTIVE_LEVEL for
// compile-time stripping.  The default SPDLOG_ACTIVE_LEVEL is
// SPDLOG_LEVEL_DEBUG when NDEBUG is not defined, SPDLOG_LEVEL_INFO otherwise.

#define HACDCPF_LOG_TRACE(...)    SPDLOG_TRACE(__VA_ARGS__)
#define HACDCPF_LOG_DEBUG(...)    SPDLOG_DEBUG(__VA_ARGS__)
#define HACDCPF_LOG_INFO(...)     SPDLOG_INFO(__VA_ARGS__)
#define HACDCPF_LOG_WARN(...)     SPDLOG_WARN(__VA_ARGS__)
#define HACDCPF_LOG_ERROR(...)    SPDLOG_ERROR(__VA_ARGS__)
#define HACDCPF_LOG_CRITICAL(...) SPDLOG_CRITICAL(__VA_ARGS__)
