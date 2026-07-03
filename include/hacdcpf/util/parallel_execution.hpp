#pragma once

#include <algorithm>
#include <string>
#include <thread>

namespace hacdcpf::util {

/// User-facing diagnostics for outer-loop parallel execution.  The existing
/// scalar result fields (effective/workers/mode) remain the compatibility
/// shorthand; this struct explains why a run did or did not use multiple cores.
struct ParallelExecutionInfo {
  bool requested{false};
  int requested_threads{0};
  int hardware_threads{1};
  int work_items{0};
  int resolved_workers{1};
  bool effective{false};
  std::string mode{"serial"};
  std::string backend;
  std::string guard_reason;

  // Optional workload counters.  They are intentionally generic so methods can
  // report their dominant source of serial/parallel work without new structs.
  long long actual_parallel_evaluations{0};
  long long serial_evaluations{0};
  int batch_size{0};
  long long cache_hits{0};
  long long cache_misses{0};
  long long n0_evaluations{0};
};

inline int detect_hardware_threads() {
  int n = static_cast<int>(std::thread::hardware_concurrency());
  return n > 0 ? n : 1;
}

inline int resolve_worker_count(int requested_threads, int work_items) {
  if (work_items <= 0) return 1;
  int nthreads = requested_threads > 0 ? requested_threads
                                       : detect_hardware_threads();
  if (nthreads <= 0) nthreads = 1;
  return std::max(1, std::min(nthreads, work_items));
}

inline ParallelExecutionInfo make_parallel_execution_info(
    bool requested,
    int requested_threads,
    int work_items,
    const std::string& mode,
    const std::string& backend = {},
    const std::string& guard_reason = {}) {
  ParallelExecutionInfo info;
  info.requested = requested;
  info.requested_threads = requested_threads;
  info.hardware_threads = detect_hardware_threads();
  info.work_items = std::max(0, work_items);
  info.backend = backend;
  info.guard_reason = guard_reason;
  info.resolved_workers =
      requested && guard_reason.empty()
          ? resolve_worker_count(requested_threads, info.work_items)
          : 1;
  info.effective =
      requested && guard_reason.empty() && info.resolved_workers > 1 &&
      info.work_items > 1;
  if (!requested) {
    info.mode = "serial/disabled";
  } else if (!guard_reason.empty()) {
    info.mode = mode.empty() ? "serial/guarded" : mode;
  } else if (info.effective) {
    info.mode = mode;
  } else {
    info.mode = "serial/insufficient-work";
  }
  return info;
}

inline std::string insufficient_work_reason(const ParallelExecutionInfo& info) {
  if (!info.requested) return "parallel execution was not requested";
  if (!info.guard_reason.empty()) return info.guard_reason;
  if (info.work_items <= 1) return "only one independent work item is available";
  if (info.resolved_workers <= 1) return "resolved worker count is one";
  return {};
}

}  // namespace hacdcpf::util
