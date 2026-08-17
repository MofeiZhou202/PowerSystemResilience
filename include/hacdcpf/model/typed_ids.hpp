#pragma once

/// model/typed_ids.hpp
/// ===================
/// Opt-in strong ID types for NEW public boundaries. They compile-time separate
/// the three integer index spaces that AGENTS.md iron rule #2 keeps distinct:
///
///   - StableBusId : authored component `.index` (stable, external, persistable)
///   - VectorPos   : 0-based solver/vector position (internal, transient)
///   - NodeIdx     : PowerSystemGraph node index (transient)
///
/// Each wraps an `int` with an explicit constructor and explicit `.value()`
/// access, and admits no implicit conversion to another space — so a
/// StableBusId cannot be passed where a VectorPos is expected. Existing code
/// keeps using `int`; adopt these at new interfaces to prevent cross-space
/// mixups. Governance contract: docs/data_structure_api_contract.md §4 and
/// docs/data_structure_design_review.md §4.

#include <cstddef>
#include <functional>

namespace hacdcpf {

namespace detail {

/// Tag-parameterised strong integer index. Distinct tags are distinct types.
template <typename Tag>
class TypedIndex {
 public:
  constexpr TypedIndex() = default;
  constexpr explicit TypedIndex(int value) noexcept : value_(value) {}

  [[nodiscard]] constexpr int value() const noexcept { return value_; }
  /// Convention: a negative value is the "unset/invalid" sentinel.
  [[nodiscard]] constexpr bool valid() const noexcept { return value_ >= 0; }

  friend constexpr bool operator==(TypedIndex a, TypedIndex b) noexcept {
    return a.value_ == b.value_;
  }
  friend constexpr bool operator!=(TypedIndex a, TypedIndex b) noexcept {
    return a.value_ != b.value_;
  }
  friend constexpr bool operator<(TypedIndex a, TypedIndex b) noexcept {
    return a.value_ < b.value_;
  }

 private:
  int value_{-1};
};

}  // namespace detail

struct StableBusIdTag {};
struct VectorPosTag {};
struct NodeIdxTag {};

/// Authored, stable, external bus/component identity (`.index`). Persistable and
/// the only space that may appear in results, JSON, or GUI payloads.
using StableBusId = detail::TypedIndex<StableBusIdTag>;

/// 0-based internal solver/vector position. Never persisted or reported.
using VectorPos = detail::TypedIndex<VectorPosTag>;

/// PowerSystemGraph node index. Transient; never persisted or reported.
using NodeIdx = detail::TypedIndex<NodeIdxTag>;

}  // namespace hacdcpf

// Hash support so typed IDs can key unordered containers. Partial specialisation
// of std::hash for a user-defined type is permitted by [namespace.std].
template <typename Tag>
struct std::hash<hacdcpf::detail::TypedIndex<Tag>> {
  std::size_t operator()(
      hacdcpf::detail::TypedIndex<Tag> id) const noexcept {
    return std::hash<int>{}(id.value());
  }
};
