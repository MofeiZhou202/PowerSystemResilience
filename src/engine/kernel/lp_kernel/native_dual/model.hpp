#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <iterator>
#include <memory>
#include <optional>
#include <string>
#include <tuple>
#include <type_traits>
#include <vector>

#include <Eigen/Core>

#include "../native_dual_core.hpp"

namespace mipsolvers::engine::native_dual::detail {

enum class Phase { One, DualOne, Two };

enum class EdgeWeightMode { SteepestEdge, Devex };

enum class RebuildReason {
  Initial,
  UpdateLimit,
  SyntheticWork,
  NumericalTrouble,
  PossiblyOptimal,
  PossiblyPrimalInfeasible,
  Cleanup,
};

enum class Move : std::int8_t {
  Down = -1,
  Fixed = 0,
  Up = 1,
};

inline int sign(Move move) { return static_cast<int>(move); }

using CycleKey = std::pair<std::uint64_t, std::uint64_t>;
using TabooChangeKey =
    std::tuple<std::uint64_t, std::uint64_t, int, int>;
using TabooRowKey = std::tuple<std::uint64_t, std::uint64_t, int>;

inline std::size_t cycle_hash_word(std::uint64_t value) noexcept {
  value ^= value >> 30;
  value *= 0xbf58476d1ce4e5b9ULL;
  value ^= value >> 27;
  value *= 0x94d049bb133111ebULL;
  return static_cast<std::size_t>(value ^ (value >> 31));
}

struct CycleKeyHash {
  std::size_t operator()(const CycleKey& key) const noexcept {
    return cycle_hash_word(key.first) ^
           (cycle_hash_word(key.second) + 0x9e3779b97f4a7c15ULL);
  }
};

struct TabooChangeKeyHash {
  std::size_t operator()(const TabooChangeKey& key) const noexcept {
    const auto [a, b, leaving, entering] = key;
    std::uint64_t word = a ^ (b + 0x9e3779b97f4a7c15ULL);
    word ^= static_cast<std::uint64_t>(static_cast<std::uint32_t>(leaving))
            << 32;
    word ^= static_cast<std::uint32_t>(entering);
    return cycle_hash_word(word);
  }
};

struct TabooRowKeyHash {
  std::size_t operator()(const TabooRowKey& key) const noexcept {
    const auto [a, b, leaving] = key;
    return cycle_hash_word(
        a ^ (b + 0x9e3779b97f4a7c15ULL) ^
        static_cast<std::uint32_t>(leaving));
  }
};

inline constexpr std::size_t kCycleHistoryCapacity = 65536;
inline constexpr std::size_t kTabooCapacity = 65536;
inline constexpr int kMaxTabooLifetime = 4096;

// Compact linear-probing table for the bounded anti-cycling dictionaries.
// Keys and values live inline in one contiguous allocation; there is no
// per-entry node allocation or pointer chasing as with std::unordered_map.
template <typename Key, typename Value, typename Hash>
class FlatHashMap {
 public:
  using value_type = std::pair<Key, Value>;

 private:
  struct Slot {
    std::optional<value_type> value;
    bool tombstone{false};
  };

  template <bool IsConst>
  class Iterator {
   public:
    using iterator_category = std::forward_iterator_tag;
    using difference_type = std::ptrdiff_t;
    using exposed_value =
        std::conditional_t<IsConst, const value_type, value_type>;
    using pointer = exposed_value*;
    using reference = exposed_value&;

    Iterator() = default;
    reference operator*() const {
      return map_->slots_[index_].value.value();
    }
    pointer operator->() const {
      return &map_->slots_[index_].value.value();
    }
    Iterator& operator++() {
      ++index_;
      skip_empty();
      return *this;
    }
    bool operator==(const Iterator& other) const {
      return map_ == other.map_ && index_ == other.index_;
    }
    bool operator!=(const Iterator& other) const { return !(*this == other); }

   private:
    using Map = std::conditional_t<IsConst, const FlatHashMap, FlatHashMap>;
    friend class FlatHashMap;
    Iterator(Map* map, std::size_t index) : map_(map), index_(index) {
      skip_empty();
    }
    void skip_empty() {
      if (map_ == nullptr) return;
      while (index_ < map_->slots_.size() &&
             !map_->slots_[index_].value.has_value()) {
        ++index_;
      }
    }

    Map* map_{nullptr};
    std::size_t index_{0};
  };

 public:
  using iterator = Iterator<false>;
  using const_iterator = Iterator<true>;

  bool empty() const noexcept { return size_ == 0; }
  std::size_t size() const noexcept { return size_; }
  std::size_t storage_capacity() const noexcept { return slots_.size(); }
  std::size_t storage_bytes() const noexcept {
    return slots_.capacity() * sizeof(Slot);
  }

  iterator begin() { return iterator(this, 0); }
  iterator end() { return iterator(this, slots_.size()); }
  const_iterator begin() const { return const_iterator(this, 0); }
  const_iterator end() const { return const_iterator(this, slots_.size()); }

  void clear() noexcept {
    for (Slot& slot : slots_) {
      slot.value.reset();
      slot.tombstone = false;
    }
    size_ = 0;
    tombstones_ = 0;
  }

  void reserve(std::size_t expected_size) {
    std::size_t capacity = 8;
    while (capacity < 2 * std::max<std::size_t>(expected_size, 1)) {
      capacity <<= 1;
    }
    if (capacity > slots_.size()) rehash(capacity);
  }

  iterator find(const Key& key) {
    return iterator(this, find_index(key));
  }
  const_iterator find(const Key& key) const {
    return const_iterator(this, find_index(key));
  }

  template <typename... Args>
  std::pair<iterator, bool> try_emplace(const Key& key, Args&&... args) {
    ensure_insert_capacity();
    const auto [index, found] = insertion_index(key);
    if (found) return {iterator(this, index), false};
    Slot& slot = slots_[index];
    if (slot.tombstone) {
      slot.tombstone = false;
      --tombstones_;
    }
    slot.value.emplace(key, Value(std::forward<Args>(args)...));
    ++size_;
    return {iterator(this, index), true};
  }

  std::pair<iterator, bool> emplace(const Key& key, const Value& value) {
    return try_emplace(key, value);
  }

  void erase(iterator position) {
    if (position.map_ != this || position.index_ >= slots_.size()) return;
    Slot& slot = slots_[position.index_];
    if (!slot.value.has_value()) return;
    slot.value.reset();
    slot.tombstone = true;
    --size_;
    ++tombstones_;
  }

  std::size_t erase(const Key& key) {
    iterator position = find(key);
    if (position == end()) return 0;
    erase(position);
    return 1;
  }

 private:
  std::size_t find_index(const Key& key) const {
    if (slots_.empty()) return slots_.size();
    const std::size_t mask = slots_.size() - 1;
    std::size_t index = hasher_(key) & mask;
    for (std::size_t probes = 0; probes < slots_.size(); ++probes) {
      const Slot& slot = slots_[index];
      if (!slot.value.has_value()) {
        if (!slot.tombstone) return slots_.size();
      } else if (slot.value->first == key) {
        return index;
      }
      index = (index + 1) & mask;
    }
    return slots_.size();
  }

  std::pair<std::size_t, bool> insertion_index(const Key& key) const {
    const std::size_t mask = slots_.size() - 1;
    std::size_t index = hasher_(key) & mask;
    std::size_t first_tombstone = slots_.size();
    for (;;) {
      const Slot& slot = slots_[index];
      if (!slot.value.has_value()) {
        if (!slot.tombstone) {
          return {first_tombstone == slots_.size() ? index : first_tombstone,
                  false};
        }
        if (first_tombstone == slots_.size()) first_tombstone = index;
      } else if (slot.value->first == key) {
        return {index, true};
      }
      index = (index + 1) & mask;
    }
  }

  void ensure_insert_capacity() {
    if (slots_.empty()) {
      rehash(8);
      return;
    }
    if (4 * (size_ + tombstones_ + 1) > 3 * slots_.size()) {
      const std::size_t next =
          4 * (size_ + 1) > 3 * slots_.size() ? 2 * slots_.size()
                                               : slots_.size();
      rehash(next);
    }
  }

  void rehash(std::size_t capacity) {
    std::vector<Slot> old = std::move(slots_);
    slots_.assign(capacity, Slot{});
    size_ = 0;
    tombstones_ = 0;
    for (Slot& slot : old) {
      if (!slot.value.has_value()) continue;
      const auto [index, found] = insertion_index(slot.value->first);
      (void)found;
      slots_[index].value.emplace(std::move(*slot.value));
      ++size_;
    }
  }

  std::vector<Slot> slots_;
  std::size_t size_{0};
  std::size_t tombstones_{0};
  Hash hasher_;
};

struct IntHash {
  std::size_t operator()(int value) const noexcept {
    return cycle_hash_word(
        static_cast<std::uint32_t>(value) + 0x9e3779b97f4a7c15ULL);
  }
};

// Sparse-first journal for additive objective changes. The ordinary dual
// simplex path shifts only a small subset of columns, so allocating a full
// n-vector for each journal wastes memory. Once at least about n/8 columns
// have been touched, a dense vector is cheaper and faster than hash lookups.
class SparseCostJournal {
 public:
  void reset(int dimension) {
    dimension_ = std::max(0, dimension);
    dense_ = false;
    nonzero_count_ = 0;
    slot_by_column_ = {};
    std::vector<int>().swap(columns_);
    std::vector<double>().swap(sparse_values_);
    std::vector<double>().swap(dense_values_);
  }

  void clear() {
    nonzero_count_ = 0;
    if (dense_) {
      std::fill(dense_values_.begin(), dense_values_.end(), 0.0);
      return;
    }
    slot_by_column_.clear();
    columns_.clear();
    sparse_values_.clear();
  }

  int dimension() const noexcept { return dimension_; }
  std::size_t size() const noexcept { return nonzero_count_; }
  bool empty() const noexcept { return nonzero_count_ == 0; }
  bool is_dense() const noexcept { return dense_; }

  double at(int column) const noexcept {
    if (column < 0 || column >= dimension_) return 0.0;
    if (dense_) return dense_values_[static_cast<std::size_t>(column)];
    const auto found = slot_by_column_.find(column);
    return found == slot_by_column_.end()
               ? 0.0
               : sparse_values_[found->second];
  }

  void set(int column, double value) {
    if (column < 0 || column >= dimension_) return;
    if (dense_) {
      double& current = dense_values_[static_cast<std::size_t>(column)];
      if (current == 0.0 && value != 0.0) {
        ++nonzero_count_;
      } else if (current != 0.0 && value == 0.0) {
        --nonzero_count_;
      }
      current = value;
      return;
    }
    auto found = slot_by_column_.find(column);
    if (found != slot_by_column_.end()) {
      double& current = sparse_values_[found->second];
      if (current == 0.0 && value != 0.0) {
        ++nonzero_count_;
      } else if (current != 0.0 && value == 0.0) {
        --nonzero_count_;
      }
      current = value;
      return;
    }
    if (value == 0.0) return;
    if (should_densify(columns_.size() + 1)) {
      densify();
      dense_values_[static_cast<std::size_t>(column)] = value;
      ++nonzero_count_;
      return;
    }
    const std::size_t slot = columns_.size();
    slot_by_column_.try_emplace(column, slot);
    columns_.push_back(column);
    sparse_values_.push_back(value);
    ++nonzero_count_;
  }

  void add(int column, double delta) {
    if (delta == 0.0) return;
    if (dense_) {
      if (column < 0 || column >= dimension_) return;
      double& current = dense_values_[static_cast<std::size_t>(column)];
      const double updated = current + delta;
      if (current == 0.0 && updated != 0.0) {
        ++nonzero_count_;
      } else if (current != 0.0 && updated == 0.0) {
        --nonzero_count_;
      }
      current = updated;
      return;
    }
    const auto found = slot_by_column_.find(column);
    if (found != slot_by_column_.end()) {
      double& current = sparse_values_[found->second];
      const double updated = current + delta;
      if (current == 0.0 && updated != 0.0) {
        ++nonzero_count_;
      } else if (current != 0.0 && updated == 0.0) {
        --nonzero_count_;
      }
      current = updated;
      return;
    }
    set(column, delta);
  }

  double max_abs() const noexcept {
    double result = 0.0;
    const auto& values = dense_ ? dense_values_ : sparse_values_;
    for (double value : values) result = std::max(result, std::abs(value));
    return result;
  }

  template <typename Function>
  void for_each(Function&& function) const {
    if (dense_) {
      for (int column = 0; column < dimension_; ++column) {
        const double value = dense_values_[static_cast<std::size_t>(column)];
        if (value != 0.0) function(column, value);
      }
      return;
    }
    for (std::size_t slot = 0; slot < columns_.size(); ++slot) {
      if (sparse_values_[slot] != 0.0)
        function(columns_[slot], sparse_values_[slot]);
    }
  }

  std::size_t memory_bytes() const noexcept {
    return slot_by_column_.storage_bytes() +
           columns_.capacity() * sizeof(int) +
           sparse_values_.capacity() * sizeof(double) +
           dense_values_.capacity() * sizeof(double);
  }

 private:
  bool should_densify(std::size_t count) const noexcept {
    if (dimension_ <= 0) return false;
    const std::size_t threshold =
        std::max<std::size_t>(8, (static_cast<std::size_t>(dimension_) + 7) / 8);
    return count >= threshold;
  }

  void densify() {
    dense_values_.assign(static_cast<std::size_t>(dimension_), 0.0);
    for (std::size_t slot = 0; slot < columns_.size(); ++slot) {
      dense_values_[static_cast<std::size_t>(columns_[slot])] =
          sparse_values_[slot];
    }
    dense_ = true;
    slot_by_column_ = {};
    std::vector<int>().swap(columns_);
    std::vector<double>().swap(sparse_values_);
  }

  int dimension_{0};
  bool dense_{false};
  std::size_t nonzero_count_{0};
  FlatHashMap<int, std::size_t, IntHash> slot_by_column_;
  std::vector<int> columns_;
  std::vector<double> sparse_values_;
  std::vector<double> dense_values_;
};

struct Bounds {
  Eigen::VectorXd lower;
  Eigen::VectorXd upper;
  std::vector<char> enterable;
};

class BasisFactor;

struct State {
  struct LeavingHeapEntry {
    double merit{0.0};
    int row{-1};
    std::uint64_t version{0};
  };

  const StandardFormLP* sf{nullptr};
  const SimplexOptions* options{nullptr};
  Phase phase{Phase::Two};
  int m{0};
  int n{0};
  int kernel_threads{1};
  Bounds bounds;
  std::vector<int> basis;
  std::vector<char> basic;
  std::vector<Move> move;
  Eigen::VectorXd cost;
  Eigen::VectorXd original_cost;
  SparseCostJournal cost_perturbation;
  SparseCostJournal cost_shift;
  // Feasible anchor Ax0=b used to express dual Phase I in the homogeneous
  // coordinates z=x-x0. Empty outside dual Phase I.
  Eigen::VectorXd dual_phase_one_anchor;
  Eigen::VectorXd x_basic;
  Eigen::VectorXd reduced_costs;
  std::vector<double> edge_weight;
  EdgeWeightMode edge_weight_mode{EdgeWeightMode::SteepestEdge};
  bool certified_exact_dse_pricing{false};
  int certified_dse_btrans{0};
  int certified_dse_candidates{0};
  int certified_dse_rejections{0};
  double certified_dse_time_sec{0.0};
  std::vector<char> devex_reference;
  int devex_iterations{0};
  double objective{0.0};
  std::shared_ptr<BasisFactor> factor;
  bool fresh_rebuild{false};
  bool costs_perturbed{false};
  bool costs_shifted{false};
  bool perturbation_disabled{false};
  int updates_since_rebuild{0};
  int canonical_primal_corrections{0};
  int canonical_residual_audits{0};
  int last_canonical_residual_audit_rebuild{-1};
  bool reinvert_after_pivot{false};
  std::uint64_t cycle_signature_a{0};
  std::uint64_t cycle_signature_b{0};
  // Incrementally maintained signature of the CURRENT (basis, nonbasic-side)
  // state.  Updated by O(1) token XORs at each pivot commit and fully resynced
  // by resync_cycle_signature whenever basis/move change outside the minor
  // iteration (major rebuilds reclassify every nonbasic side).
  // record_cycle_arrival publishes it into cycle_signature_a/b.
  std::uint64_t cycle_signature_live_a{0};
  std::uint64_t cycle_signature_live_b{0};
  int pivot_sequence{0};
  int pricing_epoch{0};
  // Lazy CHUZR heap. Each touched row gets a new version and, when infeasible,
  // a new heap entry. Stale entries are discarded when they reach the top.
  // Dense primal changes and reconstructions invalidate the whole structure.
  std::vector<LeavingHeapEntry> leaving_heap;
  std::vector<std::uint64_t> leaving_row_version;
  bool leaving_heap_valid{false};
  // Per-column dominating coefficient for the BFRT dot-product error bound:
  // coefficient * max|row_ep| is provably >= dot_error_bound for that column
  // on every pivotal row (see choose_entering_bfrt). Derived from A alone;
  // filled once by initialize.
  std::vector<double> bfrt_error_coef;

  struct CycleRecord {
    std::uint64_t signature_a{0};
    std::uint64_t signature_b{0};
    int leaving_col{-1};
    int entering_col{-1};
    int last_seen{0};
  };
  struct TimedTaboo {
    int expiry{-1};
    std::uint64_t ticket{0};
  };
  FlatHashMap<CycleKey, CycleRecord, CycleKeyHash> cycle_history;
  FlatHashMap<TabooChangeKey, TimedTaboo, TabooChangeKeyHash> taboo_changes;
  FlatHashMap<TabooRowKey, TimedTaboo, TabooRowKeyHash> taboo_rows;
  std::deque<CycleKey> cycle_history_fifo;
  std::deque<std::pair<TabooChangeKey, std::uint64_t>> taboo_changes_fifo;
  std::deque<std::pair<TabooRowKey, std::uint64_t>> taboo_rows_fifo;
  std::uint64_t taboo_ticket{0};
};

struct Audit {
  bool ok{false};
  std::string failure;
  double equation_residual{0.0};
  double max_primal_infeasibility{0.0};
  double max_dual_infeasibility{0.0};
  double max_basic_reduced_cost{0.0};
  int worst_primal_row{-1};
  int worst_dual_col{-1};
};

struct IndexedVector {
  int dimension{0};
  std::vector<int> index;
  std::vector<double> value;
  // Coordinate-lookup table, built lazily by at(): most solve results are
  // only iterated over their support, while the few queried vectors amortize
  // one build across their queries. Entries need not be sorted.
  mutable std::vector<int> lookup_slot;

  void clear(int size) {
    dimension = size;
    index.clear();
    value.clear();
    lookup_slot.clear();
  }
  double at(int target) const {
    if (lookup_slot.empty()) {
      if (index.size() < 8) {
        for (std::size_t k = 0; k < index.size(); ++k) {
          if (index[k] == target) return value[k];
        }
        return 0.0;
      }
      build_lookup();
    }
    const std::size_t mask = lookup_slot.size() - 1;
    std::size_t slot =
        (static_cast<std::uint32_t>(target) * 0x9e3779b1u) & mask;
    for (;;) {
      const int position = lookup_slot[slot];
      if (position < 0) return 0.0;
      if (index[static_cast<std::size_t>(position)] == target)
        return value[static_cast<std::size_t>(position)];
      slot = (slot + 1) & mask;
    }
  }
  void build_lookup() const {
    std::size_t slot_count = 4;
    while (slot_count < index.size() * 2) slot_count *= 2;
    lookup_slot.assign(slot_count, -1);
    const std::size_t mask = slot_count - 1;
    for (std::size_t k = 0; k < index.size(); ++k) {
      std::size_t slot =
          (static_cast<std::uint32_t>(index[k]) * 0x9e3779b1u) & mask;
      while (lookup_slot[slot] >= 0) slot = (slot + 1) & mask;
      lookup_slot[slot] = static_cast<int>(k);
    }
  }
  bool finite() const {
    if (index.size() != value.size()) return false;
    for (double entry : value) {
      if (!std::isfinite(entry)) return false;
    }
    return true;
  }
};

struct Leaving {
  int row{-1};
  int side{0};
  double delta{0.0};
  double violation{0.0};
  IndexedVector row_ep;
  bool released_taboo_row{false};
  int taboo_row_rejections{0};
  bool row_ep_refined{false};
};

struct Entering {
  int col{-1};
  double pivot{0.0};
  double alpha{0.0};
  double theta{0.0};
};

struct BoundFlip {
  int col{-1};
  Move old_move{Move::Fixed};
  double range{0.0};
};

struct WorkingCostShift {
  int col{-1};
  double delta{0.0};
};

struct PivotTransaction {
  Entering entering;
  std::vector<BoundFlip> flips;
  std::vector<WorkingCostShift> cost_shifts;
  IndexedVector bfrt_rhs;
  double covered_violation{0.0};
  double dual_step_lower{0.0};
  double dual_step_upper{0.0};
  int taboo_rejections{0};
  bool all_harris_candidates_taboo{false};
  bool stability_blocked{false};
  int unstable_pivot_rejections{0};
  int harris_second_pass_candidates{0};
  // CHUZC terminal evidence. The three nested candidate domains distinguish
  // the mathematical sign condition from the dot-product certification and
  // the stronger pivot-stability threshold used for an actual basis update.
  int positive_candidate_count{0};
  int certified_candidate_count{0};
  int stable_candidate_count{0};
  double positive_capacity{0.0};
  double certified_capacity{0.0};
  double stable_capacity{0.0};
  double stable_capacity_error{0.0};
  // BFRT profiling evidence. These counters describe the exact candidate
  // ordering used by this transaction and do not participate in the pivot.
  int bfrt_candidate_count{0};
  int bfrt_group_count{0};
  int bfrt_selected_group_size{0};
  int bfrt_stability_prefiltered{0};
  double bfrt_sort_time_sec{0.0};
  double bfrt_order_time_sec{0.0};
};

struct Certificate {
  bool valid{false};
  Eigen::VectorXd multiplier;
  double attainable_lower{0.0};
  double attainable_upper{0.0};
  double rhs{0.0};
  double error_bound{0.0};
  double margin{0.0};
};

}  // namespace mipsolvers::engine::native_dual::detail
