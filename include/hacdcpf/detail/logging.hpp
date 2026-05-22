#pragma once

/// detail/logging.hpp — Lightweight logging macros for internal use.
/// Uses spdlog when available, falls back to stderr.

#if __has_include("spdlog/spdlog.h")
#  include "spdlog/spdlog.h"
#  define HACDCPF_LOG_TRACE(...)  SPDLOG_TRACE(__VA_ARGS__)
#  define HACDCPF_LOG_DEBUG(...)  SPDLOG_DEBUG(__VA_ARGS__)
#  define HACDCPF_LOG_INFO(...)   SPDLOG_INFO(__VA_ARGS__)
#  define HACDCPF_LOG_WARN(...)   SPDLOG_WARN(__VA_ARGS__)
#  define HACDCPF_LOG_ERROR(...)  SPDLOG_ERROR(__VA_ARGS__)
#else
#  include <cstdio>
#  define HACDCPF_LOG_TRACE(...)  ((void)0)
#  define HACDCPF_LOG_DEBUG(...)  ((void)0)
#  define HACDCPF_LOG_INFO(fmt,  ...) (void)std::fprintf(stderr, "[INFO]  " fmt "\n", ##__VA_ARGS__)
#  define HACDCPF_LOG_WARN(fmt,  ...) (void)std::fprintf(stderr, "[WARN]  " fmt "\n", ##__VA_ARGS__)
#  define HACDCPF_LOG_ERROR(fmt, ...) (void)std::fprintf(stderr, "[ERROR] " fmt "\n", ##__VA_ARGS__)
#endif
