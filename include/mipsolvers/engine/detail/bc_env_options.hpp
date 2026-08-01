#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace mipsolvers::engine {

/// Immutable environment snapshot used by one top-level B&C solve.
///
/// Only solver-owned MIPSOLVERS_* and HACDCPF_* variables are captured. The
/// generic value/flag accessors serve diagnostics; algorithm-affecting values
/// are parsed into named fields below. Nested solves and worker threads bind
/// the same shared snapshot, so the process environment cannot change a run
/// after it starts.
struct BcEnvOptions {
  bool require_strict_tree_exhaustion{false};
  bool root_separation_only{false};
  bool strict_full_retain_root_lp{false};
  bool strict_root_disable_highs_evaluate_root{false};
  bool strict_root_stop_after_round{false};
  bool root_oracle_stop_after_root_round{false};

  bool scuc_dynamic_cuts_enabled{false};
  bool scuc_dynamic_cuts_disabled{false};
  bool scuc_dynamic_unit_ramp_cuts{false};

  /// Return the captured value, or nullptr when the variable was absent.
  const char* value(const char* name) const;
  bool flag(const char* name) const;

  /// Sorted NAME=VALUE records suitable for BCResult provenance.
  std::vector<std::string> active_settings() const;

 private:
  friend std::shared_ptr<const BcEnvOptions> capture_bc_env_options();
  std::unordered_map<std::string, std::string> settings_;
};

/// Capture the current process environment for a new top-level solve.
std::shared_ptr<const BcEnvOptions> capture_bc_env_options();

/// Snapshot currently bound to this thread, or nullptr outside a solve.
std::shared_ptr<const BcEnvOptions> current_bc_env_options();

/// Bind a solve snapshot to the current thread. Nested scopes restore the
/// previous binding; parallel workers receive the same shared snapshot.
class ScopedBcEnvOptions {
 public:
  explicit ScopedBcEnvOptions(std::shared_ptr<const BcEnvOptions> options);
  ~ScopedBcEnvOptions();

  ScopedBcEnvOptions(const ScopedBcEnvOptions&) = delete;
  ScopedBcEnvOptions& operator=(const ScopedBcEnvOptions&) = delete;

 private:
  std::shared_ptr<const BcEnvOptions> previous_;
};

/// Access the bound snapshot. Outside a solve, the first access on a thread
/// captures a fallback snapshot for direct helper tests.
const BcEnvOptions& bc_env_options();

}  // namespace mipsolvers::engine
