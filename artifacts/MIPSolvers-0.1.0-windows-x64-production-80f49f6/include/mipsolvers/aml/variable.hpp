#pragma once

#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "mipsolvers/aml/expression.hpp"
#include "mipsolvers/aml/ids.hpp"
#include "mipsolvers/aml/key.hpp"

namespace mipsolvers::aml {

// ── Forward declaration ────────────────────────────────────────────────────
class VarArray;

// ════════════════════════════════════════════════════════════════════════════
// VarType
// ════════════════════════════════════════════════════════════════════════════
enum class VarType { Continuous, Integer, Binary };

// ════════════════════════════════════════════════════════════════════════════
// VarRef — reference to a single scalar variable.
// Implicitly convertible to LinearExpr (coefficient 1.0).
// ════════════════════════════════════════════════════════════════════════════
class VarRef {
 public:
  VarRef() = default;
  explicit VarRef(VarId id) : id_(id) {}

  [[nodiscard]] VarId id() const noexcept { return id_; }

  // Implicit conversion: x → 1.0*x
  operator LinearExpr() const {  // NOLINT(google-explicit-constructor)
    return LinearExpr::from_var(id_);
  }

  // Arithmetic with scalars
  friend LinearExpr operator*(double c, const VarRef& v) {
    return LinearExpr::from_var(v.id_, c);
  }
  friend LinearExpr operator*(const VarRef& v, double c) {
    return LinearExpr::from_var(v.id_, c);
  }
  friend LinearExpr operator+(const VarRef& a, const VarRef& b) {
    LinearExpr e = LinearExpr::from_var(a.id_);
    e += LinearExpr::from_var(b.id_);
    return e;
  }
  friend LinearExpr operator+(const VarRef& v, const LinearExpr& e) {
    LinearExpr r = LinearExpr::from_var(v.id_);
    r += e;
    return r;
  }
  friend LinearExpr operator+(const LinearExpr& e, const VarRef& v) {
    LinearExpr r = e;
    r += LinearExpr::from_var(v.id_);
    return r;
  }
  friend LinearExpr operator-(const VarRef& v, const LinearExpr& e) {
    LinearExpr r = LinearExpr::from_var(v.id_);
    r -= e;
    return r;
  }
  friend LinearExpr operator-(const LinearExpr& e, const VarRef& v) {
    LinearExpr r = e;
    r -= LinearExpr::from_var(v.id_);
    return r;
  }
  friend LinearExpr operator-(const VarRef& v) {
    return LinearExpr::from_var(v.id_, -1.0);
  }

  // ---- Quadratic: VarRef * VarRef → QuadExpr ----------------------------
  friend QuadExpr operator*(const VarRef& a, const VarRef& b) {
    return QuadExpr::bilinear(a.id_, b.id_);
  }

  // ---- Comparison operators (forward to LinearExpr) ---------------------
  [[nodiscard]] TempConstr operator<=(double rhs) const {
    return static_cast<LinearExpr>(*this) <= rhs;
  }
  [[nodiscard]] TempConstr operator>=(double rhs) const {
    return static_cast<LinearExpr>(*this) >= rhs;
  }
  // NOLINTNEXTLINE(misc-unconventional-assign-operator)
  [[nodiscard]] TempConstr operator==(double rhs) const {
    return static_cast<LinearExpr>(*this) == rhs;
  }
  [[nodiscard]] TempConstr operator<=(const LinearExpr& rhs) const {
    return static_cast<LinearExpr>(*this) <= rhs;
  }
  [[nodiscard]] TempConstr operator>=(const LinearExpr& rhs) const {
    return static_cast<LinearExpr>(*this) >= rhs;
  }
  // NOLINTNEXTLINE(misc-unconventional-assign-operator)
  [[nodiscard]] TempConstr operator==(const LinearExpr& rhs) const {
    return static_cast<LinearExpr>(*this) == rhs;
  }

 private:
  VarId id_ = kInvalidVar;
};

// ════════════════════════════════════════════════════════════════════════════
// VarMeta — per-variable metadata stored in ModelImpl
// ════════════════════════════════════════════════════════════════════════════
struct VarMeta {
  std::string family;   ///< Name of the owning VarArray
  Key         key;      ///< Index key within that family
  VarType     type  = VarType::Continuous;
  double      lb    = -1e20;
  double      ub    =  1e20;
};

// ════════════════════════════════════════════════════════════════════════════
// VarArray — indexed family of variables over a domain set.
// Stores only metadata; actual VarId assignment is done by ModelImpl.
// ════════════════════════════════════════════════════════════════════════════
class VarArray {
 public:
  VarArray(std::string name, VarType type, double lb, double ub)
      : name_(std::move(name)), type_(type), lb_(lb), ub_(ub) {}

  // ---- Metadata ----------------------------------------------------------
  [[nodiscard]] const std::string& name()        const noexcept { return name_; }
  [[nodiscard]] VarType            type()        const noexcept { return type_; }
  [[nodiscard]] int                total_count() const noexcept {
    return static_cast<int>(key_to_id_.size());
  }

  // ---- Registration (called by ModelImpl during add_var) -----------------
  void register_var(const Key& k, VarId id) {
    key_to_id_[k] = id;
  }

  // ---- Bound overrides ---------------------------------------------------
  void set_lb(const Key& k, double lb) { lb_overrides_[k] = lb; }
  void set_ub(const Key& k, double ub) { ub_overrides_[k] = ub; }
  void fix   (const Key& k, double v)  { lb_overrides_[k] = v; ub_overrides_[k] = v; }

  [[nodiscard]] double lb(const Key& k) const {
    auto it = lb_overrides_.find(k);
    return (it != lb_overrides_.end()) ? it->second : lb_;
  }
  [[nodiscard]] double ub(const Key& k) const {
    auto it = ub_overrides_.find(k);
    return (it != ub_overrides_.end()) ? it->second : ub_;
  }

  // ---- Access ------------------------------------------------------------
  [[nodiscard]] VarRef operator()(const Key& k) const {
    auto it = key_to_id_.find(k);
    if (it == key_to_id_.end()) {
      throw std::out_of_range(
          "VarArray '" + name_ + "': key not found");
    }
    return VarRef(it->second);
  }
  [[nodiscard]] VarRef operator()(const Atom& a) const {
    return (*this)(Key::scalar(a));
  }
  [[nodiscard]] VarRef operator()(const Atom& a, const Atom& b) const {
    return (*this)(Key::pair(a, b));
  }

  [[nodiscard]] const std::unordered_map<Key, VarId, KeyHash>& key_to_id() const {
    return key_to_id_;
  }

  // Default bounds (before per-key overrides)
  [[nodiscard]] double default_lb() const noexcept { return lb_; }
  [[nodiscard]] double default_ub() const noexcept { return ub_; }

 private:
  std::string  name_;
  VarType      type_;
  double       lb_  = -1e20;
  double       ub_  =  1e20;
  std::unordered_map<Key, VarId, KeyHash> key_to_id_;
  std::unordered_map<Key, double, KeyHash> lb_overrides_;
  std::unordered_map<Key, double, KeyHash> ub_overrides_;
};

// ── VarRef quadratic helpers (defined here so VarRef is fully complete) ─────

/// Convenience: QuadExpr::sq(v) for a VarRef.
inline QuadExpr quad_sq(const VarRef& v, double coef = 1.0) {
  return QuadExpr::sq(v.id(), coef);
}

/// Convenience: QuadExpr::bilinear(a, b) for two VarRefs.
inline QuadExpr quad_bilinear(const VarRef& a, const VarRef& b, double coef = 1.0) {
  return QuadExpr::bilinear(a.id(), b.id(), coef);
}

}  // namespace mipsolvers::aml
