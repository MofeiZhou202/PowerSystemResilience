#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include "mipsolvers/aml/expression.hpp"
#include "mipsolvers/aml/ids.hpp"
#include "mipsolvers/aml/key.hpp"

namespace mipsolvers::aml {

// ════════════════════════════════════════════════════════════════════════════
// ConMeta — per-constraint metadata
// ════════════════════════════════════════════════════════════════════════════
struct ConMeta {
  std::string family;  ///< Name of the owning ConstraintArray (or "" for anonymous)
  Key         key;     ///< Index key within the family
  CompareOp   sense  = CompareOp::Equal;
};

// ════════════════════════════════════════════════════════════════════════════
// ConstraintRef — reference to a single scalar constraint row.
// ════════════════════════════════════════════════════════════════════════════
class ConstraintRef {
 public:
  ConstraintRef() = default;
  ConstraintRef(ConId id, std::string name) : id_(id), name_(std::move(name)) {}

  [[nodiscard]] ConId             id()   const noexcept { return id_; }
  [[nodiscard]] const std::string& name() const noexcept { return name_; }

 private:
  ConId       id_   = kInvalidCon;
  std::string name_;
};

// ════════════════════════════════════════════════════════════════════════════
// ConstraintArray — indexed family of constraints.
// ════════════════════════════════════════════════════════════════════════════
class ConstraintArray {
 public:
  explicit ConstraintArray(std::string name) : name_(std::move(name)) {}

  // ---- Metadata ----------------------------------------------------------
  [[nodiscard]] const std::string& name()        const noexcept { return name_; }
  [[nodiscard]] int                total_count() const noexcept {
    return static_cast<int>(key_to_id_.size());
  }

  // ---- Registration (called by ModelImpl during add_constraints) ---------
  void register_con(const Key& k, ConId id) {
    key_to_id_[k] = id;
  }

  // ---- Access ------------------------------------------------------------
  [[nodiscard]] ConstraintRef operator()(const Key& k) const {
    auto it = key_to_id_.find(k);
    if (it == key_to_id_.end()) {
      throw std::out_of_range(
          "ConstraintArray '" + name_ + "': key not found");
    }
    return ConstraintRef(it->second, name_ + "[...]");
  }
  [[nodiscard]] ConstraintRef operator()(const Atom& a) const {
    return (*this)(Key::scalar(a));
  }
  [[nodiscard]] ConstraintRef operator()(const Atom& a, const Atom& b) const {
    return (*this)(Key::pair(a, b));
  }

  [[nodiscard]] const std::unordered_map<Key, ConId, KeyHash>& key_to_id() const {
    return key_to_id_;
  }

 private:
  std::string name_;
  std::unordered_map<Key, ConId, KeyHash> key_to_id_;
};

}  // namespace mipsolvers::aml
