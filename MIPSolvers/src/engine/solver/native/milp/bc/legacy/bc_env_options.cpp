#include "mipsolvers/engine/detail/bc_env_options.hpp"

#include <algorithm>
#include <cstring>
#include <utility>

#if defined(_WIN32)
#include <stdlib.h>
#else
extern char** environ;
#endif

namespace mipsolvers::engine {

namespace {

thread_local std::shared_ptr<const BcEnvOptions> active_options;
thread_local std::shared_ptr<const BcEnvOptions> fallback_options;

bool solver_owned_name(const char* name, std::size_t length) {
  constexpr char kMipsolversPrefix[] = "MIPSOLVERS_";
  constexpr char kHacdcpfPrefix[] = "HACDCPF_";
  return (length >= sizeof(kMipsolversPrefix) - 1 &&
          std::memcmp(name, kMipsolversPrefix,
                      sizeof(kMipsolversPrefix) - 1) == 0) ||
         (length >= sizeof(kHacdcpfPrefix) - 1 &&
          std::memcmp(name, kHacdcpfPrefix,
                      sizeof(kHacdcpfPrefix) - 1) == 0);
}

void parse_typed_fields(BcEnvOptions& options) {
  options.require_strict_tree_exhaustion =
      options.flag("MIPSOLVERS_REQUIRE_STRICT_TREE_EXHAUSTION");
  options.root_separation_only =
      options.flag("MIPSOLVERS_ROOT_SEPARATION_ONLY");
  options.strict_full_retain_root_lp =
      options.flag("MIPSOLVERS_STRICT_FULL_RETAIN_ROOT_LP");
  options.strict_root_disable_highs_evaluate_root =
      options.flag("MIPSOLVERS_STRICT_ROOT_DISABLE_HIGHS_EVALUATE_ROOT");
  options.strict_root_stop_after_round =
      options.flag("MIPSOLVERS_STRICT_ROOT_STOP_AFTER_ROUND");
  options.root_oracle_stop_after_root_round =
      options.flag("MIPSOLVERS_ROOT_ORACLE_STOP_AFTER_ROOT_ROUND");
  options.scuc_dynamic_cuts_enabled =
      options.flag("MIPSOLVERS_ENABLE_SCUC_DYNAMIC_CUTS");
  options.scuc_dynamic_cuts_disabled =
      options.flag("MIPSOLVERS_DISABLE_SCUC_DYNAMIC_CUTS");
  options.scuc_dynamic_unit_ramp_cuts =
      options.flag("MIPSOLVERS_SCUC_DYNAMIC_UNIT_RAMP_CUTS");
}

}  // namespace

const char* BcEnvOptions::value(const char* name) const {
  if (name == nullptr) return nullptr;
  const auto it = settings_.find(name);
  return it == settings_.end() ? nullptr : it->second.c_str();
}

bool BcEnvOptions::flag(const char* name) const {
  const char* raw = value(name);
  return raw != nullptr && raw[0] != '\0' && raw[0] != '0';
}

std::vector<std::string> BcEnvOptions::active_settings() const {
  std::vector<std::string> result;
  result.reserve(settings_.size());
  for (const auto& [name, value] : settings_) {
    result.push_back(name + "=" + value);
  }
  std::sort(result.begin(), result.end());
  return result;
}

std::shared_ptr<const BcEnvOptions> capture_bc_env_options() {
  auto options = std::make_shared<BcEnvOptions>();
#if defined(_WIN32)
  char** environment = _environ;
#else
  char** environment = ::environ;
#endif
  if (environment != nullptr) {
    for (char** item = environment; *item != nullptr; ++item) {
      const char* entry = *item;
      const char* separator = std::strchr(entry, '=');
      if (separator == nullptr) continue;
      const std::size_t name_length =
          static_cast<std::size_t>(separator - entry);
      if (!solver_owned_name(entry, name_length)) continue;
      options->settings_.emplace(std::string(entry, name_length),
                                 std::string(separator + 1));
    }
  }
  parse_typed_fields(*options);
  return options;
}

std::shared_ptr<const BcEnvOptions> current_bc_env_options() {
  return active_options;
}

ScopedBcEnvOptions::ScopedBcEnvOptions(
    std::shared_ptr<const BcEnvOptions> options)
    : previous_(std::move(active_options)) {
  active_options = options ? std::move(options) : capture_bc_env_options();
}

ScopedBcEnvOptions::~ScopedBcEnvOptions() {
  active_options = std::move(previous_);
}

const BcEnvOptions& bc_env_options() {
  if (active_options) return *active_options;
  if (!fallback_options) fallback_options = capture_bc_env_options();
  return *fallback_options;
}

}  // namespace mipsolvers::engine
