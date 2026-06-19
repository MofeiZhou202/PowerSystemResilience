// Compatibility shim: provides the spdlog::info/warn/error/debug interface
// using fmt as the backend.  This header is used when spdlog is not available.
#pragma once

#include <fmt/core.h>
#include <fmt/format.h>

#include <memory>
#include <utility>

namespace spdlog {

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
  void trace(fmt::format_string<Args...> f, Args&&... a)    { fmt::print("[trace]    {}\n", fmt::format(f, std::forward<Args>(a)...)); }
  template <typename... Args>
  void debug(fmt::format_string<Args...> f, Args&&... a)    { fmt::print("[debug]    {}\n", fmt::format(f, std::forward<Args>(a)...)); }
  template <typename... Args>
  void info(fmt::format_string<Args...> f, Args&&... a)     { fmt::print("[info]     {}\n", fmt::format(f, std::forward<Args>(a)...)); }
  template <typename... Args>
  void warn(fmt::format_string<Args...> f, Args&&... a)     { fmt::print("[warn]     {}\n", fmt::format(f, std::forward<Args>(a)...)); }
  template <typename... Args>
  void error(fmt::format_string<Args...> f, Args&&... a)    { fmt::print("[error]    {}\n", fmt::format(f, std::forward<Args>(a)...)); }
  template <typename... Args>
  void critical(fmt::format_string<Args...> f, Args&&... a) { fmt::print("[critical] {}\n", fmt::format(f, std::forward<Args>(a)...)); }
};

inline std::shared_ptr<spdlog::logger> default_logger() {
  static auto inst = std::make_shared<spdlog::logger>();
  return inst;
}

template <typename... Args>
inline void trace(fmt::format_string<Args...> fmt_str, Args&&... args) {
  fmt::print("[trace]    {}\n", fmt::format(fmt_str, std::forward<Args>(args)...));
}

template <typename... Args>
inline void debug(fmt::format_string<Args...> fmt_str, Args&&... args) {
  fmt::print("[debug]    {}\n", fmt::format(fmt_str, std::forward<Args>(args)...));
}

template <typename... Args>
inline void info(fmt::format_string<Args...> fmt_str, Args&&... args) {
  fmt::print("[info]     {}\n", fmt::format(fmt_str, std::forward<Args>(args)...));
}

template <typename... Args>
inline void warn(fmt::format_string<Args...> fmt_str, Args&&... args) {
  fmt::print("[warn]     {}\n", fmt::format(fmt_str, std::forward<Args>(args)...));
}

template <typename... Args>
inline void error(fmt::format_string<Args...> fmt_str, Args&&... args) {
  fmt::print("[error]    {}\n", fmt::format(fmt_str, std::forward<Args>(args)...));
}

template <typename... Args>
inline void critical(fmt::format_string<Args...> fmt_str, Args&&... args) {
  fmt::print("[critical] {}\n", fmt::format(fmt_str, std::forward<Args>(args)...));
}

// Single-argument (message-only) overloads
inline void trace(const char* msg)    { fmt::print("[trace]    {}\n", msg); }
inline void debug(const char* msg)    { fmt::print("[debug]    {}\n", msg); }
inline void info(const char* msg)     { fmt::print("[info]     {}\n", msg); }
inline void warn(const char* msg)     { fmt::print("[warn]     {}\n", msg); }
inline void error(const char* msg)    { fmt::print("[error]    {}\n", msg); }
inline void critical(const char* msg) { fmt::print("[critical] {}\n", msg); }

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

