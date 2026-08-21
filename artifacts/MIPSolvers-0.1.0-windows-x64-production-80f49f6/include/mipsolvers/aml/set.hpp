#pragma once

#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "mipsolvers/aml/key.hpp"

namespace mipsolvers::aml {

// ────────────────────────────────────────────────────────────────────────────
// Forward declarations
// ────────────────────────────────────────────────────────────────────────────
class OrderedSet;

// ────────────────────────────────────────────────────────────────────────────
// Set — base class for all set types
// ────────────────────────────────────────────────────────────────────────────

/// Abstract base: a finite, indexable collection of Keys of fixed dimension.
class Set {
 public:
  virtual ~Set() = default;

  [[nodiscard]] const std::string& name()       const noexcept { return name_; }
  [[nodiscard]] int                dimension()  const noexcept { return dim_; }
  [[nodiscard]] int                cardinality() const         { return static_cast<int>(elements().size()); }

  /// O(1) average membership test.
  [[nodiscard]] virtual bool contains(const Key& k) const = 0;

  /// Ordered element list (stable iteration order).
  [[nodiscard]] virtual const std::vector<Key>& elements() const = 0;

  /// Add a new element.  Returns false if the key was already present.
  virtual bool add_element(const Key& k);

  // ---- Set algebra -------------------------------------------------------

  /// Lazy Cartesian product.  Result dim = this->dimension() + other->dimension().
  [[nodiscard]] std::shared_ptr<Set> cross(std::shared_ptr<Set> other) const;

  /// Eagerly materialized filtered subset.
  [[nodiscard]] std::shared_ptr<Set> filter(
      std::function<bool(const Key&)> pred,
      const std::string& new_name = "") const;

  // ---- 1-D convenience helpers ------------------------------------------

  /// Add a scalar (1-D) element.  Throws if dim != 1.
  void add_element(const Atom& a);

 protected:
  explicit Set(std::string name, int dim)
      : name_(std::move(name)), dim_(dim) {}

  void check_dim(const Key& k, const char* context) const;

  std::string name_;
  int         dim_  = 1;
};

// ────────────────────────────────────────────────────────────────────────────
// ExplicitSet — user-managed, insertion-ordered
// ────────────────────────────────────────────────────────────────────────────
class ExplicitSet : public Set {
 public:
  /// Construct empty set with given dimension.
  explicit ExplicitSet(const std::string& name, int dim = 1)
      : Set(name, dim) {}

  /// Construct with initial elements.
  ExplicitSet(const std::string& name,
              const std::vector<Atom>& atoms);  // dim-1 convenience

  ExplicitSet(const std::string& name, int dim,
              const std::vector<Key>& elems);

  [[nodiscard]] bool contains(const Key& k) const override {
    return lookup_.count(k) > 0;
  }
  [[nodiscard]] const std::vector<Key>& elements() const override {
    return elems_;
  }
  bool add_element(const Key& k) override;

 private:
  std::vector<Key>                             elems_;
  std::unordered_set<Key, KeyHash>             lookup_;
};

// ────────────────────────────────────────────────────────────────────────────
// OrderedSet — ExplicitSet with prev / next navigation
// ────────────────────────────────────────────────────────────────────────────
class OrderedSet : public ExplicitSet {
 public:
  explicit OrderedSet(const std::string& name, int dim = 1)
      : ExplicitSet(name, dim) {}

  OrderedSet(const std::string& name, const std::vector<Atom>& atoms)
      : ExplicitSet(name, atoms) { rebuild_index(); }

  bool add_element(const Key& k) override {
    bool ok = ExplicitSet::add_element(k);
    if (ok) rebuild_index();
    return ok;
  }

  void add_element(const Atom& a) {
    ExplicitSet::add_element(Key::scalar(a));
    rebuild_index();
  }

  /// 0-based positional access.
  [[nodiscard]] const Key& at(int pos) const {
    return elements().at(static_cast<std::size_t>(pos));
  }

  /// Position of k; throws std::out_of_range if absent.
  [[nodiscard]] int position(const Key& k) const;

  /// Previous element; nullopt if k is first.
  [[nodiscard]] std::optional<Key> prev(const Key& k) const;

  /// Next element; nullopt if k is last.
  [[nodiscard]] std::optional<Key> next(const Key& k) const;

 private:
  void rebuild_index();
  std::unordered_map<Key, int, KeyHash> pos_map_;
};

// ────────────────────────────────────────────────────────────────────────────
// CartesianProductSet — lazy wrapper
// ────────────────────────────────────────────────────────────────────────────
class CartesianProductSet : public Set {
 public:
  CartesianProductSet(std::shared_ptr<Set> a, std::shared_ptr<Set> b)
      : Set(a->name() + "x" + b->name(), a->dimension() + b->dimension()),
        a_(std::move(a)), b_(std::move(b)) {}

  [[nodiscard]] bool contains(const Key& k) const override;
  [[nodiscard]] const std::vector<Key>& elements() const override;
  bool add_element(const Key& k) override;

 private:
  std::shared_ptr<Set> a_;
  std::shared_ptr<Set> b_;
  mutable std::vector<Key> cache_;
  mutable bool cached_ = false;
  void materialize() const;
};

// ────────────────────────────────────────────────────────────────────────────
// FilteredSet — eagerly materialized subset
// ────────────────────────────────────────────────────────────────────────────
class FilteredSet : public ExplicitSet {
 public:
  FilteredSet(const Set& parent,
              const std::function<bool(const Key&)>& pred,
              const std::string& name);
};

}  // namespace mipsolvers::aml
