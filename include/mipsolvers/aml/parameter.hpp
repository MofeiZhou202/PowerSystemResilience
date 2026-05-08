#pragma once

#include <stdexcept>
#include <string>
#include <unordered_map>

#include "mipsolvers/aml/key.hpp"

namespace mipsolvers::aml {

/// What to do when get() is called for a key that has no value.
enum class MissingPolicy {
  Error,         ///< (default) throw std::out_of_range
  ReturnZero,    ///< silently return 0.0
};

/// A named, multi-dimensional table of double-precision parameter values.
/// Indexed by Key; default MissingPolicy is Error to catch mis-keyed data.
class Parameter {
 public:
  Parameter(std::string name, int dim, std::string unit,
            MissingPolicy policy = MissingPolicy::Error)
      : name_(std::move(name)),
        dim_(dim),
        unit_(std::move(unit)),
        policy_(policy) {}

  // ---- Metadata ----------------------------------------------------------
  [[nodiscard]] const std::string& name()   const noexcept { return name_; }
  [[nodiscard]] int                dimension() const noexcept { return dim_; }
  [[nodiscard]] const std::string& unit()   const noexcept { return unit_; }
  [[nodiscard]] MissingPolicy      policy() const noexcept { return policy_; }

  void set_policy(MissingPolicy p) noexcept { policy_ = p; }

  // ---- Assignment --------------------------------------------------------

  /// Set value for key k.  Throws if dimension mismatch.
  void set(const Key& k, double v) {
    check_dim(k);
    data_[k] = v;
  }

  /// Convenience: scalar parameter (dim == 0) stored at empty key.
  void set_scalar(double v) {
    set(Key{{}}, v);
  }

  /// Bulk load from map.
  void load(const std::unordered_map<Key, double, KeyHash>& m) {
    for (const auto& [k, v] : m) set(k, v);
  }

  // ---- Retrieval ---------------------------------------------------------

  /// Get value; throws std::out_of_range if key missing and policy==Error.
  [[nodiscard]] double get(const Key& k) const {
    check_dim(k);
    auto it = data_.find(k);
    if (it == data_.end()) {
      if (policy_ == MissingPolicy::Error) {
        throw std::out_of_range(
            "Parameter '" + name_ + "': key not set");
      }
      return 0.0;
    }
    return it->second;
  }

  /// Get scalar (dim-0).
  [[nodiscard]] double get_scalar() const {
    return get(Key{{}});
  }

  /// Get with explicit fallback; never throws regardless of policy.
  [[nodiscard]] double get_or(const Key& k, double default_val) const {
    check_dim(k);
    auto it = data_.find(k);
    return (it != data_.end()) ? it->second : default_val;
  }

  /// True if a value exists for k.
  [[nodiscard]] bool contains(const Key& k) const {
    return data_.count(k) > 0;
  }

  /// Copy the internal map.
  [[nodiscard]] std::unordered_map<Key, double, KeyHash> to_map() const {
    return data_;
  }

  /// Number of set values.
  [[nodiscard]] int size() const { return static_cast<int>(data_.size()); }

 private:
  void check_dim(const Key& k) const {
    if (k.dimension() != dim_) {
      throw std::invalid_argument(
          "Parameter '" + name_ + "': key dimension " +
          std::to_string(k.dimension()) + " != declared dimension " +
          std::to_string(dim_));
    }
  }

  std::string  name_;
  int          dim_    = 1;
  std::string  unit_;
  MissingPolicy policy_;
  std::unordered_map<Key, double, KeyHash> data_;
};

}  // namespace mipsolvers::aml
