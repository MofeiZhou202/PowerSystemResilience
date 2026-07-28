#pragma once

namespace mipsolvers::engine {

/// Algorithm-affecting environment toggles for the legacy B&C stack.
///
/// The legacy branch-and-cut historically reads 200+ environment variables
/// scattered through a 33k-line function, making runs environment-dependent
/// and un-CI-able.  This struct is the first governance step: the genuinely
/// algorithm-affecting toggles are parsed ONCE here, and call sites migrate
/// from ad-hoc `std::getenv`/`bc_env_flag_enabled(name)` to fields.
///
/// Debug/trace flags (MIPSOLVERS_*_DIAG, *_TIMELINE, *_CONFORM, *_TRACE,
/// *_STATS, HACDCPF_*) are intentionally NOT captured — they are diagnostics,
/// not algorithm choices, and stay env-driven until the tree split lands.
struct BcEnvOptions {
  // StrictHiGHS diagnostics and root-pipeline experiments.
  bool require_strict_tree_exhaustion{false};     ///< MIPSOLVERS_REQUIRE_STRICT_TREE_EXHAUSTION
  bool root_separation_only{false};               ///< MIPSOLVERS_ROOT_SEPARATION_ONLY
  bool strict_full_retain_root_lp{false};         ///< MIPSOLVERS_STRICT_FULL_RETAIN_ROOT_LP
  bool strict_root_disable_highs_evaluate_root{false};  ///< MIPSOLVERS_STRICT_ROOT_DISABLE_HIGHS_EVALUATE_ROOT
  bool strict_root_stop_after_round{false};       ///< MIPSOLVERS_STRICT_ROOT_STOP_AFTER_ROUND
  bool root_oracle_stop_after_root_round{false};  ///< MIPSOLVERS_ROOT_ORACLE_STOP_AFTER_ROOT_ROUND

  // SCUC dynamic-cut feature toggles.
  bool scuc_dynamic_cuts_enabled{false};          ///< MIPSOLVERS_ENABLE_SCUC_DYNAMIC_CUTS
  bool scuc_dynamic_cuts_disabled{false};         ///< MIPSOLVERS_DISABLE_SCUC_DYNAMIC_CUTS
  bool scuc_dynamic_unit_ramp_cuts{false};        ///< MIPSOLVERS_SCUC_DYNAMIC_UNIT_RAMP_CUTS
};

/// Parsed-once accessor.  The environment is read at first call and frozen;
/// getenv is not re-read, so behavior is reproducible for the process
/// lifetime (and unit tests get a single consistent snapshot).
const BcEnvOptions& bc_env_options();

}  // namespace mipsolvers::engine
