#pragma once

/// Lightweight logging facade for mipsolvers.
/// Uses fmt::print to stderr. No external logging library required.
///
/// Compile-time control: define MIPSOLVERS_LOG_LEVEL=<n>
///   0=trace, 1=debug, 2=info, 3=warn, 4=error, 5=critical, 6=off
/// Default: 2 (info) in Release, 1 (debug) in Debug.

#include <fmt/format.h>
#include <fmt/ostream.h>
#include <cstdio>
#include <string_view>

#ifndef MIPSOLVERS_LOG_LEVEL
#  ifdef NDEBUG
#    define MIPSOLVERS_LOG_LEVEL 2  // info
#  else
#    define MIPSOLVERS_LOG_LEVEL 1  // debug
#  endif
#endif

// Keep HACDCPF_LOG_LEVEL in sync for backwards-compatible code.
#ifndef HACDCPF_LOG_LEVEL
#  define HACDCPF_LOG_LEVEL MIPSOLVERS_LOG_LEVEL
#endif

namespace mipsolvers::logging {

enum class Level { trace = 0, debug = 1, info = 2, warn = 3, error = 4, critical = 5, off = 6 };

inline void set_level(Level /*lvl*/) {
  // Runtime level filtering is a no-op in this lightweight backend.
}

template <typename... Args>
inline void log(Level lvl, fmt::format_string<Args...> fmt_str, Args&&... args) {
  if (static_cast<int>(lvl) < MIPSOLVERS_LOG_LEVEL) return;
  const char* prefix = "";
  switch (lvl) {
    case Level::trace:    prefix = "[TRACE] "; break;
    case Level::debug:    prefix = "[DEBUG] "; break;
    case Level::info:     prefix = "[INFO]  "; break;
    case Level::warn:     prefix = "[WARN]  "; break;
    case Level::error:    prefix = "[ERROR] "; break;
    case Level::critical: prefix = "[CRIT]  "; break;
    default: break;
  }
  fmt::print(stderr, "{}{}\n", prefix,
             fmt::format(fmt_str, std::forward<Args>(args)...));
}

}  // namespace mipsolvers::logging

// ── Convenience macros (mirrors spdlog interface for drop-in compatibility) ──
#define MIPSOLVERS_LOG_TRACE(...)    \
  do { if (MIPSOLVERS_LOG_LEVEL <= 0) mipsolvers::logging::log(mipsolvers::logging::Level::trace, __VA_ARGS__); } while(0)
#define MIPSOLVERS_LOG_DEBUG(...)    \
  do { if (MIPSOLVERS_LOG_LEVEL <= 1) mipsolvers::logging::log(mipsolvers::logging::Level::debug, __VA_ARGS__); } while(0)
#define MIPSOLVERS_LOG_INFO(...)     \
  do { if (MIPSOLVERS_LOG_LEVEL <= 2) mipsolvers::logging::log(mipsolvers::logging::Level::info, __VA_ARGS__); } while(0)
#define MIPSOLVERS_LOG_WARN(...)     \
  do { if (MIPSOLVERS_LOG_LEVEL <= 3) mipsolvers::logging::log(mipsolvers::logging::Level::warn, __VA_ARGS__); } while(0)
#define MIPSOLVERS_LOG_ERROR(...)    \
  do { if (MIPSOLVERS_LOG_LEVEL <= 4) mipsolvers::logging::log(mipsolvers::logging::Level::error, __VA_ARGS__); } while(0)
#define MIPSOLVERS_LOG_CRITICAL(...) \
  do { if (MIPSOLVERS_LOG_LEVEL <= 5) mipsolvers::logging::log(mipsolvers::logging::Level::critical, __VA_ARGS__); } while(0)

// Backwards-compatible HACDCPF_ macros (aliased to MIPSOLVERS_).
#define HACDCPF_LOG_TRACE(...)    MIPSOLVERS_LOG_TRACE(__VA_ARGS__)
#define HACDCPF_LOG_DEBUG(...)    MIPSOLVERS_LOG_DEBUG(__VA_ARGS__)
#define HACDCPF_LOG_INFO(...)     MIPSOLVERS_LOG_INFO(__VA_ARGS__)
#define HACDCPF_LOG_WARN(...)     MIPSOLVERS_LOG_WARN(__VA_ARGS__)
#define HACDCPF_LOG_ERROR(...)    MIPSOLVERS_LOG_ERROR(__VA_ARGS__)
#define HACDCPF_LOG_CRITICAL(...) MIPSOLVERS_LOG_CRITICAL(__VA_ARGS__)
