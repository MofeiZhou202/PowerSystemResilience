// Compatibility shim: provides the spdlog::info/warn/error/debug interface
// using fmt as the backend.  This header is used when spdlog is not available.
#pragma once

#include <fmt/core.h>
#include <fmt/format.h>

#include <memory>
#include <utility>

namespace spdlog {

namespace detail {

template <typename... Args>
inline void print_log(const char* prefix,
                      fmt::format_string<Args...> fmt_str,
                      Args&&... args) {
  // Avoid fmt::format() here. On Windows, a Debug consumer may link against a
  // Release shared fmt library, and returning std::string across that boundary
  // is not ABI-safe when iterator debugging is enabled.
  fmt::print("{}", prefix);
  fmt::print(fmt_str, std::forward<Args>(args)...);
  fmt::print("\n");
}

inline void print_log(const char* prefix, const char* message) {
  fmt::print("{}{}\n", prefix, message);
}

}  // namespace detail

namespace level {
enum level_enum : int {
  trace    = 0,
  debug    = 1,
  info     = 2,
  warn     = 3,
  err      = 4,
  critical = 5,
  off      = 6,
};
}  // namespace level

inline void set_level(level::level_enum /*lvl*/) {
  // No-op: runtime log level control is not supported by this shim.
}

// Logger stub returned by default_logger()
struct logger {
  template <typename... Args>
  void trace(fmt::format_string<Args...> f, Args&&... a)    { detail::print_log("[trace]    ", f, std::forward<Args>(a)...); }
  template <typename... Args>
  void debug(fmt::format_string<Args...> f, Args&&... a)    { detail::print_log("[debug]    ", f, std::forward<Args>(a)...); }
  template <typename... Args>
  void info(fmt::format_string<Args...> f, Args&&... a)     { detail::print_log("[info]     ", f, std::forward<Args>(a)...); }
  template <typename... Args>
  void warn(fmt::format_string<Args...> f, Args&&... a)     { detail::print_log("[warn]     ", f, std::forward<Args>(a)...); }
  template <typename... Args>
  void error(fmt::format_string<Args...> f, Args&&... a)    { detail::print_log("[error]    ", f, std::forward<Args>(a)...); }
  template <typename... Args>
  void critical(fmt::format_string<Args...> f, Args&&... a) { detail::print_log("[critical] ", f, std::forward<Args>(a)...); }
};

inline std::shared_ptr<spdlog::logger> default_logger() {
  static auto inst = std::make_shared<spdlog::logger>();
  return inst;
}

template <typename... Args>
inline void trace(fmt::format_string<Args...> fmt_str, Args&&... args) {
  detail::print_log("[trace]    ", fmt_str, std::forward<Args>(args)...);
}

template <typename... Args>
inline void debug(fmt::format_string<Args...> fmt_str, Args&&... args) {
  detail::print_log("[debug]    ", fmt_str, std::forward<Args>(args)...);
}

template <typename... Args>
inline void info(fmt::format_string<Args...> fmt_str, Args&&... args) {
  detail::print_log("[info]     ", fmt_str, std::forward<Args>(args)...);
}

template <typename... Args>
inline void warn(fmt::format_string<Args...> fmt_str, Args&&... args) {
  detail::print_log("[warn]     ", fmt_str, std::forward<Args>(args)...);
}

template <typename... Args>
inline void error(fmt::format_string<Args...> fmt_str, Args&&... args) {
  detail::print_log("[error]    ", fmt_str, std::forward<Args>(args)...);
}

template <typename... Args>
inline void critical(fmt::format_string<Args...> fmt_str, Args&&... args) {
  detail::print_log("[critical] ", fmt_str, std::forward<Args>(args)...);
}

// Single-argument (message-only) overloads
inline void trace(const char* msg)    { detail::print_log("[trace]    ", msg); }
inline void debug(const char* msg)    { detail::print_log("[debug]    ", msg); }
inline void info(const char* msg)     { detail::print_log("[info]     ", msg); }
inline void warn(const char* msg)     { detail::print_log("[warn]     ", msg); }
inline void error(const char* msg)    { detail::print_log("[error]    ", msg); }
inline void critical(const char* msg) { detail::print_log("[critical] ", msg); }

}  // namespace spdlog

// ── SPDLOG_* macros (no-op or passthrough) ───────────────────────────────────
#ifndef SPDLOG_ACTIVE_LEVEL
#  define SPDLOG_ACTIVE_LEVEL 2  // info
#endif

#define SPDLOG_TRACE(...)    do { if (0 <= SPDLOG_ACTIVE_LEVEL) spdlog::trace(__VA_ARGS__);    } while(0)
#define SPDLOG_DEBUG(...)    do { if (1 <= SPDLOG_ACTIVE_LEVEL) spdlog::debug(__VA_ARGS__);    } while(0)
#define SPDLOG_INFO(...)     do { if (2 <= SPDLOG_ACTIVE_LEVEL) spdlog::info(__VA_ARGS__);     } while(0)
#define SPDLOG_WARN(...)     do { if (3 <= SPDLOG_ACTIVE_LEVEL) spdlog::warn(__VA_ARGS__);     } while(0)
#define SPDLOG_ERROR(...)    do { if (4 <= SPDLOG_ACTIVE_LEVEL) spdlog::error(__VA_ARGS__);    } while(0)
#define SPDLOG_CRITICAL(...) do { if (5 <= SPDLOG_ACTIVE_LEVEL) spdlog::critical(__VA_ARGS__); } while(0)
