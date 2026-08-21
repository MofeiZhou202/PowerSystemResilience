#pragma once

#include <cstdint>
#include <string>

#include <nlohmann/json.hpp>

#include "mipsolvers/engine/problem_types.hpp"

namespace mipsolvers::l2o {

struct FingerprintOptions {
  bool include_objective_sense{true};
  bool include_variable_names{false};
  bool include_bounds{false};
  bool include_matrix_values{false};
  bool include_rhs_values{false};
  bool include_objective_values{false};
  bool include_uc_hint{true};
};

struct ModelFingerprint {
  std::string algorithm{"mipsolvers-l2o-fnv1a-v1"};
  std::string value;
  std::int64_t num_variables{0};
  std::int64_t num_ineq_rows{0};
  std::int64_t num_eq_rows{0};
  std::int64_t num_ineq_nonzeros{0};
  std::int64_t num_eq_nonzeros{0};
  std::int64_t num_integer_vars{0};
  std::int64_t num_binary_vars{0};
  bool has_uc_hint{false};
  nlohmann::json metadata = nlohmann::json::object();
};

void to_json(nlohmann::json& j, const FingerprintOptions& options);
void from_json(const nlohmann::json& j, FingerprintOptions& options);
void to_json(nlohmann::json& j, const ModelFingerprint& fingerprint);
void from_json(const nlohmann::json& j, ModelFingerprint& fingerprint);

ModelFingerprint fingerprint_lp(
    const engine::LPModel& lp,
    const FingerprintOptions& options = FingerprintOptions{});

ModelFingerprint fingerprint_mip(
    const engine::MIPModel& mip,
    const FingerprintOptions& options = FingerprintOptions{});

bool compatible_fingerprint(const ModelFingerprint& lhs,
                            const ModelFingerprint& rhs);

}  // namespace mipsolvers::l2o