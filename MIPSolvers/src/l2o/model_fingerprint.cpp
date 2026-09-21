#include "mipsolvers/l2o/model_fingerprint.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string_view>
#include <vector>

namespace mipsolvers::l2o {
namespace {

constexpr std::uint64_t kFnvOffset = 14695981039346656037ull;
constexpr std::uint64_t kFnvPrime = 1099511628211ull;

class HashBuilder {
 public:
  void add(std::string_view value) {
    for (const char c : value) add_byte(static_cast<unsigned char>(c));
    add_byte(0xffu);
  }

  void add_bool(bool value) { add(value ? "true" : "false"); }

  void add_int64(std::int64_t value) { add(std::to_string(value)); }

  void add_double(double value) {
    if (std::isnan(value)) {
      add("nan");
      return;
    }
    if (std::isinf(value)) {
      add(value > 0.0 ? "inf" : "-inf");
      return;
    }
    std::ostringstream os;
    os << std::setprecision(std::numeric_limits<double>::max_digits10) << value;
    add(os.str());
  }

  std::string hex() const {
    std::ostringstream os;
    os << std::hex << std::setw(16) << std::setfill('0') << state_;
    return os.str();
  }

 private:
  void add_byte(unsigned char byte) {
    state_ ^= byte;
    state_ *= kFnvPrime;
  }

  std::uint64_t state_{kFnvOffset};
};

std::string sense_name(engine::Sense sense) {
  return sense == engine::Sense::Minimize ? "minimize" : "maximize";
}

std::string var_type_name(engine::VarType type) {
  switch (type) {
    case engine::VarType::Continuous: return "continuous";
    case engine::VarType::Integer: return "integer";
    case engine::VarType::Binary: return "binary";
  }
  return "continuous";
}

template <typename T>
void add_integral_vector(HashBuilder& hash, const std::vector<T>& values) {
  hash.add_int64(static_cast<std::int64_t>(values.size()));
  for (const auto value : values) hash.add_int64(static_cast<std::int64_t>(value));
}

void add_double_vector_shape(HashBuilder& hash, const std::vector<double>& values) {
  hash.add_int64(static_cast<std::int64_t>(values.size()));
}

void add_double_vector_values(HashBuilder& hash, const std::vector<double>& values) {
  hash.add_int64(static_cast<std::int64_t>(values.size()));
  for (const double value : values) hash.add_double(value);
}

template <typename SparseMatrix>
void add_sparse_matrix(HashBuilder& hash,
                       const SparseMatrix& matrix,
                       bool include_values) {
  hash.add_int64(static_cast<std::int64_t>(matrix.rows()));
  hash.add_int64(static_cast<std::int64_t>(matrix.cols()));
  hash.add_int64(static_cast<std::int64_t>(matrix.nonZeros()));
  for (int outer = 0; outer < matrix.outerSize(); ++outer) {
    for (typename SparseMatrix::InnerIterator it(matrix, outer); it; ++it) {
      hash.add_int64(static_cast<std::int64_t>(it.row()));
      hash.add_int64(static_cast<std::int64_t>(it.col()));
      if (include_values) hash.add_double(it.value());
    }
  }
}

void add_lp_to_hash(HashBuilder& hash,
                    const engine::LPModel& lp,
                    const FingerprintOptions& options) {
  hash.add("lp");
  hash.add_bool(options.include_objective_sense);
  hash.add_bool(options.include_variable_names);
  hash.add_bool(options.include_bounds);
  hash.add_bool(options.include_matrix_values);
  hash.add_bool(options.include_rhs_values);
  hash.add_bool(options.include_objective_values);

  if (options.include_objective_sense) hash.add(sense_name(lp.sense));
  hash.add_int64(static_cast<std::int64_t>(lp.c.size()));
  hash.add_int64(static_cast<std::int64_t>(lp.vars.size()));

  for (const auto& var : lp.vars) {
    hash.add(var_type_name(var.type));
    if (options.include_variable_names) hash.add(var.name);
    if (options.include_bounds) {
      hash.add_double(var.lb);
      hash.add_double(var.ub);
    }
  }

  if (options.include_objective_values) {
    for (int i = 0; i < lp.c.size(); ++i) hash.add_double(lp.c[i]);
  }

  add_sparse_matrix(hash, lp.A, options.include_matrix_values);
  hash.add_bool(engine::lp_has_row_lhs(lp));
  if (engine::lp_has_row_lhs(lp)) {
    for (int i = 0; i < lp.row_lhs.size(); ++i) {
      hash.add_bool(std::isfinite(lp.row_lhs[i]));
      if (options.include_rhs_values) hash.add_double(lp.row_lhs[i]);
    }
  }
  hash.add_int64(static_cast<std::int64_t>(lp.b.size()));
  if (options.include_rhs_values) {
    for (int i = 0; i < lp.b.size(); ++i) hash.add_double(lp.b[i]);
  }

  add_sparse_matrix(hash, lp.Aeq, options.include_matrix_values);
  hash.add_int64(static_cast<std::int64_t>(lp.beq.size()));
  if (options.include_rhs_values) {
    for (int i = 0; i < lp.beq.size(); ++i) hash.add_double(lp.beq[i]);
  }

  add_integral_vector(hash, lp.highs_row_to_native_row);
  add_integral_vector(hash, lp.highs_row_start);
  add_integral_vector(hash, lp.highs_row_index);
  if (options.include_matrix_values) add_double_vector_values(hash, lp.highs_row_value);
  else add_double_vector_shape(hash, lp.highs_row_value);
}

void add_uc_hint_to_hash(HashBuilder& hash,
                         const engine::MIPModel::UCGenHint& hint,
                         const FingerprintOptions& options) {
  hash.add("uc_hint");
  hash.add_int64(hint.ng);
  hash.add_int64(hint.T);
  hash.add_double(hint.period_hours);
  hash.add_int64(hint.ig_start);
  hash.add_int64(hint.su_start);
  hash.add_int64(hint.sd_start);
  hash.add_int64(hint.pg_start);
  add_integral_vector(hash, hint.min_up);
  add_integral_vector(hash, hint.min_down);
  add_integral_vector(hash, hint.ig_cols);
  add_integral_vector(hash, hint.su_cols);
  add_integral_vector(hash, hint.sd_cols);
  add_integral_vector(hash, hint.pg_cols);
  hash.add_bool(hint.certifies_power_balance_rows);
  hash.add_bool(hint.certifies_generation_capacity_rows);
  hash.add_bool(hint.certifies_ramping_rows);
  hash.add_bool(hint.certifies_min_up_down_rows);
  hash.add_bool(hint.certifies_segment_bound_rows);
  hash.add_bool(hint.certifies_system_reserve_rows);
  hash.add_bool(hint.certifies_area_import_rows);
  hash.add_bool(hint.certifies_hard_network_flow_rows);
  hash.add_bool(hint.certifies_hard_section_flow_rows);
  hash.add_bool(hint.certifies_storage_cycle_rows);
  add_integral_vector(hash, hint.gen_bus);
  add_double_vector_shape(hash, hint.demand);
  add_double_vector_shape(hash, hint.reserve_requirement);
  add_double_vector_shape(hash, hint.spinning_requirement);
  add_double_vector_shape(hash, hint.regulation_up_requirement);
  add_double_vector_shape(hash, hint.regulation_down_requirement);
  hash.add_int64(hint.n_areas);
  add_integral_vector(hash, hint.gen_area);
  add_double_vector_shape(hash, hint.area_demand);
  add_double_vector_shape(hash, hint.area_reserve_requirement);
  add_double_vector_shape(hash, hint.area_import_capacity);
  hash.add_int64(hint.network_line_count);
  add_double_vector_shape(hash, hint.line_gsf);
  add_double_vector_shape(hash, hint.line_fwd_rhs);
  add_double_vector_shape(hash, hint.line_rev_rhs);
  hash.add_int64(hint.section_count);
  add_double_vector_shape(hash, hint.section_gsf);
  add_double_vector_shape(hash, hint.section_fwd_rhs);
  add_double_vector_shape(hash, hint.section_rev_rhs);
  hash.add_int64(hint.n_segments);
  add_integral_vector(hash, hint.segment_cols);
  add_double_vector_shape(hash, hint.segment_cap);
  hash.add_int64(hint.n_storage);
  add_integral_vector(hash, hint.storage_charge_cols);
  add_integral_vector(hash, hint.storage_discharge_cols);
  add_double_vector_shape(hash, hint.storage_energy_capacity);
  add_double_vector_shape(hash, hint.storage_efficiency);
  add_double_vector_shape(hash, hint.storage_initial_energy);
  add_double_vector_shape(hash, hint.storage_cycle_limit);

  if (options.include_rhs_values) {
    add_double_vector_values(hash, hint.demand);
    add_double_vector_values(hash, hint.reserve_requirement);
    add_double_vector_values(hash, hint.line_fwd_rhs);
    add_double_vector_values(hash, hint.line_rev_rhs);
    add_double_vector_values(hash, hint.section_fwd_rhs);
    add_double_vector_values(hash, hint.section_rev_rhs);
  }
}

nlohmann::json uc_hint_metadata(const engine::MIPModel::UCGenHint& hint) {
  return nlohmann::json{
      {"ng", hint.ng},
      {"T", hint.T},
      {"period_hours", hint.period_hours},
      {"ig_start", hint.ig_start},
      {"su_start", hint.su_start},
      {"sd_start", hint.sd_start},
      {"pg_start", hint.pg_start},
      {"n_areas", hint.n_areas},
      {"network_line_count", hint.network_line_count},
      {"section_count", hint.section_count},
      {"n_segments", hint.n_segments},
      {"n_storage", hint.n_storage},
      {"certifies_power_balance_rows", hint.certifies_power_balance_rows},
      {"certifies_generation_capacity_rows", hint.certifies_generation_capacity_rows},
      {"certifies_ramping_rows", hint.certifies_ramping_rows},
      {"certifies_min_up_down_rows", hint.certifies_min_up_down_rows},
      {"certifies_segment_bound_rows", hint.certifies_segment_bound_rows},
      {"certifies_system_reserve_rows", hint.certifies_system_reserve_rows},
      {"certifies_area_import_rows", hint.certifies_area_import_rows},
      {"certifies_hard_network_flow_rows", hint.certifies_hard_network_flow_rows},
      {"certifies_hard_section_flow_rows", hint.certifies_hard_section_flow_rows},
      {"certifies_storage_cycle_rows", hint.certifies_storage_cycle_rows}};
}

ModelFingerprint base_lp_fingerprint(const engine::LPModel& lp) {
  ModelFingerprint fingerprint;
  fingerprint.num_variables = std::max<std::int64_t>(
      static_cast<std::int64_t>(lp.c.size()),
      static_cast<std::int64_t>(lp.vars.size()));
  fingerprint.num_ineq_rows = static_cast<std::int64_t>(lp.A.rows());
  fingerprint.num_eq_rows = static_cast<std::int64_t>(lp.Aeq.rows());
  fingerprint.num_ineq_nonzeros = static_cast<std::int64_t>(lp.A.nonZeros());
  fingerprint.num_eq_nonzeros = static_cast<std::int64_t>(lp.Aeq.nonZeros());
  fingerprint.metadata["sense"] = sense_name(lp.sense);
  fingerprint.metadata["has_row_lhs"] = engine::lp_has_row_lhs(lp);
  return fingerprint;
}

}  // namespace

void to_json(nlohmann::json& j, const FingerprintOptions& options) {
  j = nlohmann::json{
      {"include_objective_sense", options.include_objective_sense},
      {"include_variable_names", options.include_variable_names},
      {"include_bounds", options.include_bounds},
      {"include_matrix_values", options.include_matrix_values},
      {"include_rhs_values", options.include_rhs_values},
      {"include_objective_values", options.include_objective_values},
      {"include_uc_hint", options.include_uc_hint}};
}

void from_json(const nlohmann::json& j, FingerprintOptions& options) {
  options.include_objective_sense = j.value("include_objective_sense", true);
  options.include_variable_names = j.value("include_variable_names", false);
  options.include_bounds = j.value("include_bounds", false);
  options.include_matrix_values = j.value("include_matrix_values", false);
  options.include_rhs_values = j.value("include_rhs_values", false);
  options.include_objective_values = j.value("include_objective_values", false);
  options.include_uc_hint = j.value("include_uc_hint", true);
}

void to_json(nlohmann::json& j, const ModelFingerprint& fingerprint) {
  j = nlohmann::json{
      {"algorithm", fingerprint.algorithm},
      {"value", fingerprint.value},
      {"num_variables", fingerprint.num_variables},
      {"num_ineq_rows", fingerprint.num_ineq_rows},
      {"num_eq_rows", fingerprint.num_eq_rows},
      {"num_ineq_nonzeros", fingerprint.num_ineq_nonzeros},
      {"num_eq_nonzeros", fingerprint.num_eq_nonzeros},
      {"num_integer_vars", fingerprint.num_integer_vars},
      {"num_binary_vars", fingerprint.num_binary_vars},
      {"has_uc_hint", fingerprint.has_uc_hint},
      {"metadata", fingerprint.metadata}};
}

void from_json(const nlohmann::json& j, ModelFingerprint& fingerprint) {
  fingerprint.algorithm = j.value("algorithm", "mipsolvers-l2o-fnv1a-v1");
  fingerprint.value = j.value("value", "");
  fingerprint.num_variables = j.value("num_variables", std::int64_t{0});
  fingerprint.num_ineq_rows = j.value("num_ineq_rows", std::int64_t{0});
  fingerprint.num_eq_rows = j.value("num_eq_rows", std::int64_t{0});
  fingerprint.num_ineq_nonzeros = j.value("num_ineq_nonzeros", std::int64_t{0});
  fingerprint.num_eq_nonzeros = j.value("num_eq_nonzeros", std::int64_t{0});
  fingerprint.num_integer_vars = j.value("num_integer_vars", std::int64_t{0});
  fingerprint.num_binary_vars = j.value("num_binary_vars", std::int64_t{0});
  fingerprint.has_uc_hint = j.value("has_uc_hint", false);
  fingerprint.metadata = j.value("metadata", nlohmann::json::object());
}

ModelFingerprint fingerprint_lp(const engine::LPModel& lp,
                                const FingerprintOptions& options) {
  HashBuilder hash;
  add_lp_to_hash(hash, lp, options);

  ModelFingerprint fingerprint = base_lp_fingerprint(lp);
  fingerprint.value = hash.hex();
  fingerprint.metadata["fingerprint_options"] = options;
  return fingerprint;
}

ModelFingerprint fingerprint_mip(const engine::MIPModel& mip,
                                 const FingerprintOptions& options) {
  HashBuilder hash;
  add_lp_to_hash(hash, mip.linear_part, options);
  hash.add("mip");
  add_integral_vector(hash, mip.integer_idx);
  add_integral_vector(hash, mip.binary_idx);
  add_integral_vector(hash, mip.branching_priority);
  hash.add_bool(options.include_uc_hint);
  hash.add_bool(mip.uc_hint.has_value());
  if (options.include_uc_hint && mip.uc_hint) {
    add_uc_hint_to_hash(hash, *mip.uc_hint, options);
  }

  ModelFingerprint fingerprint = base_lp_fingerprint(mip.linear_part);
  fingerprint.value = hash.hex();
  fingerprint.num_integer_vars = static_cast<std::int64_t>(mip.integer_idx.size());
  fingerprint.num_binary_vars = static_cast<std::int64_t>(mip.binary_idx.size());
  fingerprint.has_uc_hint = mip.uc_hint.has_value();
  fingerprint.metadata["fingerprint_options"] = options;
  if (mip.uc_hint) fingerprint.metadata["uc_hint"] = uc_hint_metadata(*mip.uc_hint);
  return fingerprint;
}

bool compatible_fingerprint(const ModelFingerprint& lhs,
                            const ModelFingerprint& rhs) {
  return lhs.algorithm == rhs.algorithm &&
         lhs.value == rhs.value &&
         lhs.num_variables == rhs.num_variables &&
         lhs.num_ineq_rows == rhs.num_ineq_rows &&
         lhs.num_eq_rows == rhs.num_eq_rows &&
         lhs.num_ineq_nonzeros == rhs.num_ineq_nonzeros &&
         lhs.num_eq_nonzeros == rhs.num_eq_nonzeros &&
         lhs.num_integer_vars == rhs.num_integer_vars &&
         lhs.num_binary_vars == rhs.num_binary_vars &&
         lhs.has_uc_hint == rhs.has_uc_hint;
}

}  // namespace mipsolvers::l2o