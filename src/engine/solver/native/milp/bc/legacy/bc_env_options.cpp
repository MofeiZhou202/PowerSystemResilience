#include "mipsolvers/engine/detail/bc_env_options.hpp"

#include <cstdlib>

namespace mipsolvers::engine {

namespace {

bool env_flag(const char* name) {
  const char* v = std::getenv(name);
  return v != nullptr && v[0] != '\0' && v[0] != '0';
}

BcEnvOptions parse_env_options() {
  BcEnvOptions o;
  o.require_strict_tree_exhaustion =
      env_flag("MIPSOLVERS_REQUIRE_STRICT_TREE_EXHAUSTION");
  o.root_separation_only = env_flag("MIPSOLVERS_ROOT_SEPARATION_ONLY");
  o.strict_full_retain_root_lp = env_flag("MIPSOLVERS_STRICT_FULL_RETAIN_ROOT_LP");
  o.strict_root_disable_highs_evaluate_root =
      env_flag("MIPSOLVERS_STRICT_ROOT_DISABLE_HIGHS_EVALUATE_ROOT");
  o.strict_root_stop_after_round = env_flag("MIPSOLVERS_STRICT_ROOT_STOP_AFTER_ROUND");
  o.root_oracle_stop_after_root_round =
      env_flag("MIPSOLVERS_ROOT_ORACLE_STOP_AFTER_ROOT_ROUND");
  o.scuc_dynamic_cuts_enabled = env_flag("MIPSOLVERS_ENABLE_SCUC_DYNAMIC_CUTS");
  o.scuc_dynamic_cuts_disabled = env_flag("MIPSOLVERS_DISABLE_SCUC_DYNAMIC_CUTS");
  o.scuc_dynamic_unit_ramp_cuts = env_flag("MIPSOLVERS_SCUC_DYNAMIC_UNIT_RAMP_CUTS");
  return o;
}

}  // namespace

const BcEnvOptions& bc_env_options() {
  static const BcEnvOptions opts = parse_env_options();
  return opts;
}

}  // namespace mipsolvers::engine
