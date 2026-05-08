#include "mipsolvers/aml/set.hpp"

#include <stdexcept>
#include <string>

namespace mipsolvers::aml {

// ────────────────────────────────────────────────────────────────────────────
// Set helpers
// ────────────────────────────────────────────────────────────────────────────
void Set::check_dim(const Key& k, const char* context) const {
  if (k.dimension() != dim_) {
    throw std::invalid_argument(
        std::string(context) + ": key dimension " +
        std::to_string(k.dimension()) + " != set '" + name_ +
        "' dimension " + std::to_string(dim_));
  }
}

bool Set::add_element(const Key& k) {
  (void)k;
  throw std::logic_error("Set::add_element not supported on this set type");
}

void Set::add_element(const Atom& a) {
  if (dim_ != 1) {
    throw std::invalid_argument(
        "Set '" + name_ + "' has dimension " + std::to_string(dim_) +
        "; use add_element(Key) for multi-dimensional sets");
  }
  add_element(Key::scalar(a));
}

std::shared_ptr<Set> Set::cross(std::shared_ptr<Set> other) const {
  // Return a new CartesianProductSet wrapping shared_ptrs.
  // We cannot take shared_ptr<this> here without a shared_from_this base,
  // so the caller normally uses the shared_ptr it already owns.
  // This overload is mainly used inside Model::add_var / add_constraints.
  (void)other;
  throw std::logic_error(
      "Set::cross: call via shared_ptr<Set>, not raw reference");
}

std::shared_ptr<Set> Set::filter(
    std::function<bool(const Key&)> pred,
    const std::string& new_name) const {
  std::string nm = new_name.empty() ? (name_ + "_filtered") : new_name;
  return std::make_shared<FilteredSet>(*this, pred, nm);
}

// ────────────────────────────────────────────────────────────────────────────
// ExplicitSet
// ────────────────────────────────────────────────────────────────────────────
ExplicitSet::ExplicitSet(const std::string& name,
                         const std::vector<Atom>& atoms)
    : Set(name, 1) {
  for (const auto& a : atoms) {
    Key k = Key::scalar(a);
    if (lookup_.insert(k).second) elems_.push_back(k);
  }
}

ExplicitSet::ExplicitSet(const std::string& name, int dim,
                         const std::vector<Key>& elems)
    : Set(name, dim) {
  for (const auto& k : elems) {
    check_dim(k, "ExplicitSet::ExplicitSet");
    if (lookup_.insert(k).second) elems_.push_back(k);
  }
}

bool ExplicitSet::add_element(const Key& k) {
  check_dim(k, "ExplicitSet::add_element");
  auto [it, inserted] = lookup_.insert(k);
  if (inserted) elems_.push_back(k);
  return inserted;
}

// ────────────────────────────────────────────────────────────────────────────
// OrderedSet
// ────────────────────────────────────────────────────────────────────────────
void OrderedSet::rebuild_index() {
  pos_map_.clear();
  const auto& elems = elements();
  for (int i = 0; i < static_cast<int>(elems.size()); ++i) {
    pos_map_[elems[i]] = i;
  }
}

int OrderedSet::position(const Key& k) const {
  auto it = pos_map_.find(k);
  if (it == pos_map_.end()) {
    throw std::out_of_range(
        "OrderedSet '" + name() + "': key not found");
  }
  return it->second;
}

std::optional<Key> OrderedSet::prev(const Key& k) const {
  int pos = position(k);  // throws if absent
  if (pos == 0) return std::nullopt;
  return elements()[static_cast<std::size_t>(pos - 1)];
}

std::optional<Key> OrderedSet::next(const Key& k) const {
  int pos = position(k);
  int last = static_cast<int>(elements().size()) - 1;
  if (pos >= last) return std::nullopt;
  return elements()[static_cast<std::size_t>(pos + 1)];
}

// ────────────────────────────────────────────────────────────────────────────
// CartesianProductSet
// ────────────────────────────────────────────────────────────────────────────
bool CartesianProductSet::contains(const Key& k) const {
  check_dim(k, "CartesianProductSet::contains");
  int da = a_->dimension();
  Key ka, kb;
  ka.values.assign(k.values.begin(), k.values.begin() + da);
  kb.values.assign(k.values.begin() + da, k.values.end());
  return a_->contains(ka) && b_->contains(kb);
}

void CartesianProductSet::materialize() const {
  if (cached_) return;
  cache_.clear();
  for (const auto& ka : a_->elements()) {
    for (const auto& kb : b_->elements()) {
      cache_.push_back(Key::concat(ka, kb));
    }
  }
  cached_ = true;
}

const std::vector<Key>& CartesianProductSet::elements() const {
  materialize();
  return cache_;
}

bool CartesianProductSet::add_element(const Key& k) {
  (void)k;
  throw std::logic_error(
      "CartesianProductSet '" + name() +
      "': cannot add elements to a derived set");
}

// ────────────────────────────────────────────────────────────────────────────
// FilteredSet
// ────────────────────────────────────────────────────────────────────────────
FilteredSet::FilteredSet(const Set& parent,
                         const std::function<bool(const Key&)>& pred,
                         const std::string& name)
    : ExplicitSet(name, parent.dimension()) {
  for (const auto& k : parent.elements()) {
    if (pred(k)) {
      ExplicitSet::add_element(k);  // already dimension-checked by parent
    }
  }
}

// ────────────────────────────────────────────────────────────────────────────
// Set::cross — shared_ptr version (free function)
// ────────────────────────────────────────────────────────────────────────────
std::shared_ptr<Set> cross(std::shared_ptr<Set> a, std::shared_ptr<Set> b) {
  return std::make_shared<CartesianProductSet>(std::move(a), std::move(b));
}

}  // namespace mipsolvers::aml
