#pragma once

#include <array>
#include <chrono>
#include <cstdio>
#include <cstdlib>

namespace mipsolvers::engine::lp_timing {

// Interval accounting and measurement contract: Windows remediation R6.
// Categories are exclusive; retry is an overlapping diagnostic subset.
enum class Kind { Assembly, Symbolic, Numeric };
struct Record {
  std::array<double, 3> ms{};
  std::array<unsigned, 3> calls{};
  double retry_ms{0.0};
  unsigned retries{0};
  std::array<char, 16> adaptive_backend_sequence{};
  unsigned adaptive_backend_selections{0};
  bool adaptive_backend_overflow{false};
};
inline thread_local Record* active = nullptr;
inline void note_adaptive_backend(bool ldlt) {
  if (!active) return;
  if (active->adaptive_backend_selections >=
      active->adaptive_backend_sequence.size()) {
    active->adaptive_backend_overflow = true;
    return;
  }
  // Joint R3 state model: observe the choice without changing its certificate.
  active->adaptive_backend_sequence[active->adaptive_backend_selections++] =
      ldlt ? 'L' : 'U';
}
class Scope;
inline thread_local Scope* parent = nullptr;
using Clock = std::chrono::steady_clock;

class Scope {
 public:
  explicit Scope(Kind kind) : record_(active), kind_(static_cast<unsigned>(kind)) {
    if (!record_) return;
    previous_ = parent;
    parent = this;
    start_ = Clock::now();
    ++record_->calls[kind_];
  }
  Scope(const Scope&) = delete;
  Scope& operator=(const Scope&) = delete;
  ~Scope() { finish(); }
  void finish() {
    if (!record_) return;
    const double elapsed = std::chrono::duration<double, std::milli>(Clock::now() - start_).count();
    record_->ms[kind_] += elapsed - children_ms_;
    if (previous_) previous_->children_ms_ += elapsed;
    parent = previous_;
    record_ = nullptr;
  }
 private:
  Record* record_;
  unsigned kind_;
  Scope* previous_{nullptr};
  Clock::time_point start_{};
  double children_ms_{0.0};
};

class Retry {
 public:
  Retry() : record_(active) {
    if (record_) { ++record_->retries; start_ = Clock::now(); }
  }
  ~Retry() {
    if (record_) record_->retry_ms +=
        std::chrono::duration<double, std::milli>(Clock::now() - start_).count();
  }
 private:
  Record* record_;
  Clock::time_point start_{};
};

class Run {
 public:
  Run(int rows, int cols, int rounds, const char* entry_reason = "primary",
      int source_iterations = 0)
      : rows_(rows), cols_(cols), rounds_(rounds),
        entry_reason_(entry_reason), source_iterations_(source_iterations),
        previous_(active), previous_parent_(parent) {
    const char* flag = std::getenv("MIPSOLVERS_LP_FACTOR_TIMING");
    enabled_ = flag && *flag && *flag != '0';
    active = enabled_ ? &record_ : nullptr;
    parent = nullptr;
    if (enabled_) start_ = Clock::now();
  }
  void set_formulation(const char* formulation) {
    formulation_ = formulation;
  }
  ~Run() {
    active = previous_;
    parent = previous_parent_;
    if (!enabled_) return;
    const double elapsed = std::chrono::duration<double, std::milli>(Clock::now() - start_).count();
    std::fprintf(stderr,
        "LP-FACTOR {\"schema_version\":2,\"rows\":%d,\"cols\":%d,\"ruiz_rounds\":%d,"
        "\"entry_reason\":\"%s\",\"source_iterations\":%d,"
        "\"formulation\":\"%s\","
        "\"adaptive_backend_sequence\":\"%.*s\","
        "\"adaptive_backend_overflow\":%s,"
        "\"variant_ms\":%.6f,\"assembly_ms\":%.6f,\"symbolic_ms\":%.6f,"
        "\"numeric_ms\":%.6f,\"other_ms\":%.6f,\"retry_ms\":%.6f,"
        "\"assembly_calls\":%u,\"symbolic_calls\":%u,\"numeric_calls\":%u,\"retries\":%u}\n",
        rows_, cols_, rounds_, entry_reason_, source_iterations_, formulation_,
        static_cast<int>(record_.adaptive_backend_selections),
        record_.adaptive_backend_sequence.data(),
        record_.adaptive_backend_overflow ? "true" : "false",
        elapsed, record_.ms[0], record_.ms[1], record_.ms[2],
        elapsed - record_.ms[0] - record_.ms[1] - record_.ms[2], record_.retry_ms,
        record_.calls[0], record_.calls[1], record_.calls[2], record_.retries);
  }
 private:
  int rows_, cols_, rounds_;
  const char* entry_reason_;
  int source_iterations_;
  const char* formulation_{"unselected"};
  bool enabled_{false};
  Record record_;
  Record* previous_;
  Scope* previous_parent_;
  Clock::time_point start_{};
};

}  // namespace mipsolvers::engine::lp_timing
