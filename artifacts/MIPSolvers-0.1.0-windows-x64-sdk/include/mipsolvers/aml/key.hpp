#pragma once

#include <functional>
#include <initializer_list>
#include <string>
#include <vector>

namespace mipsolvers::aml {

/// An atom is a string label used as a set index element.
/// Phase 2 may extend to variant<int64_t, string> for numeric bus indices.
using Atom = std::string;

/// A multi-dimensional index key: a flat vector of atoms.
/// Dimension-1 keys (scalars) have values.size() == 1.
/// CartesianProductSet keys have values.size() == dimA + dimB.
struct Key {
  std::vector<Atom> values;

  /// Number of dimensions.
  [[nodiscard]] int dimension() const noexcept {
    return static_cast<int>(values.size());
  }

  bool operator==(const Key&) const noexcept = default;
  bool operator< (const Key& o) const noexcept { return values < o.values; }
  bool operator!=(const Key& o) const noexcept { return !(*this == o); }

  // ---- Convenience factories ------------------------------------------

  /// Build a 1-D key from a single atom.
  static Key scalar(const Atom& a) { return Key{{a}}; }

  /// Build a 2-D key from two atoms.
  static Key pair(const Atom& a, const Atom& b) { return Key{{a, b}}; }

  /// Build an N-D key from an initializer list.
  static Key make(std::initializer_list<Atom> il) {
    return Key{std::vector<Atom>{il}};
  }

  /// Concatenate two keys (used by CartesianProductSet).
  static Key concat(const Key& a, const Key& b) {
    Key out;
    out.values.reserve(a.values.size() + b.values.size());
    out.values.insert(out.values.end(), a.values.begin(), a.values.end());
    out.values.insert(out.values.end(), b.values.begin(), b.values.end());
    return out;
  }
};

/// FNV-1a-style hash for Key.
struct KeyHash {
  std::size_t operator()(const Key& k) const noexcept {
    std::size_t h = 0xcbf29ce484222325ULL;
    for (const auto& a : k.values) {
      for (unsigned char c : a) {
        h ^= static_cast<std::size_t>(c);
        h *= 0x100000001b3ULL;
      }
      // separator
      h ^= 0x3dULL;
      h *= 0x100000001b3ULL;
    }
    return h;
  }
};

}  // namespace mipsolvers::aml
