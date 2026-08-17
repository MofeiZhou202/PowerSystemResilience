/// @file bc_threading.hpp
/// @brief Thread-safe data structures for heterogeneous parallel B&C.
///
/// Provides SharedCutPool (reader-writer lock), SharedConflictPool (reader-writer lock), SharedSolutionPool (mutex),
/// SharedIncumbent (lock-free CAS + mutex), ThreadSafeNodeQueue, CutRequestQueue,
/// WorkStealingDeque, and AtomicBCStats for the parallel B&C architecture.

#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <condition_variable>
#include <chrono>
#include <cstdint>
#include <deque>
#include <limits>
#include <mutex>
#include <optional>
#include <random>
#include <set>
#include <shared_mutex>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "mipsolvers/engine/bc/enums.hpp"
#include "mipsolvers/engine/detail/bc_types.hpp"
#include "mipsolvers/engine/detail/bc_pools.hpp"

namespace mipsolvers::engine::detail {

/// Wakes the parallel-tree monitor when search state changes.
///
/// The generation counter makes notifications durable: a state change that
/// happens just before the monitor starts waiting is still observed.
class ParallelProgressEvent {
 public:
  std::uint64_t generation() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return generation_;
  }

  void notify() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      ++generation_;
    }
    condition_.notify_one();
  }

  bool wait_until_change(
      std::uint64_t& observed_generation,
      std::chrono::steady_clock::time_point deadline) {
    std::unique_lock<std::mutex> lock(mutex_);
    const bool changed = condition_.wait_until(lock, deadline, [&] {
      return generation_ != observed_generation;
    });
    observed_generation = generation_;
    return changed;
  }

 private:
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  std::uint64_t generation_{0};
};

/// @brief Work-stealing deque for per-worker local node storage.
/// Owner thread pushes/pops from the top (LIFO), thieves steal from the bottom.
/// Uses a simple mutex-based implementation (Chase-Lev deque is overkill for B&C
/// where node processing time >> deque operation time).
class WorkStealingDeque {
  mutable std::mutex mtx_;
  std::deque<Node> deque_;
  int max_size_;
public:
  explicit WorkStealingDeque(int max_size = 256) : max_size_(max_size) {}

  /// @brief Whether owner can push without spilling to the shared queue.
  bool can_push() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return static_cast<int>(deque_.size()) < max_size_;
  }

  /// @brief Owner: push to top (LIFO). Returns false if full.
  ///
  /// The node is accepted by rvalue reference so a failed push does not consume
  /// the caller's node. This matters for spill-to-global-queue fallback.
  bool push(Node&& node) {
    std::lock_guard<std::mutex> lk(mtx_);
    if (static_cast<int>(deque_.size()) >= max_size_) return false;
    deque_.push_back(std::move(node));
    return true;
  }

  /// @brief Owner: pop from top (LIFO). Returns false if empty.
  bool pop(Node& out) {
    std::lock_guard<std::mutex> lk(mtx_);
    if (deque_.empty()) return false;
    out = std::move(deque_.back());
    deque_.pop_back();
    return true;
  }

  /// @brief Thief: steal from bottom (oldest node = highest subtree).
  /// Returns false if empty.
  bool steal(Node& out) {
    std::lock_guard<std::mutex> lk(mtx_);
    if (deque_.empty()) return false;
    out = std::move(deque_.front());
    deque_.pop_front();
    return true;
  }

  /// @brief Owner/proof scheduler: drain local DFS nodes for global requeue.
  ///
  /// Once a finite incumbent is available, the proof frontier must be ordered
  /// by certified lower bound rather than by each worker's speculative DFS
  /// stack. Draining preserves every node while letting the shared priority
  /// queue become the single certificate frontier.
  int drain(std::vector<Node>& out, int max_nodes = std::numeric_limits<int>::max()) {
    std::lock_guard<std::mutex> lk(mtx_);
    int moved = 0;
    while (!deque_.empty() && moved < max_nodes) {
      out.push_back(std::move(deque_.back()));
      deque_.pop_back();
      ++moved;
    }
    return moved;
  }

  /// @brief Check size (for debugging/stats).
  int size() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return static_cast<int>(deque_.size());
  }

  /// @brief Check if empty.
  bool empty() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return deque_.empty();
  }

  /// @brief Minimum lower bound across all nodes in the deque.
  double lower_bound() const {
    std::lock_guard<std::mutex> lk(mtx_);
    double lb = kInf;
    for (const auto& node : deque_) lb = std::min(lb, node.bound);
    return lb;
  }
};

/// @brief Pool of work-stealing deques for all worker threads.
/// Provides centralized steal arbitration with random victim selection.
class WorkStealingPool {
  std::vector<std::unique_ptr<WorkStealingDeque>> deques_;
  mutable std::mutex rng_mtx_;
  std::mt19937 rng_;
  std::atomic<int> total_steals_{0};
  std::atomic<int> failed_steals_{0};
public:
  /// @param seed Deterministic seed for victim selection, including zero.
  explicit WorkStealingPool(int n_workers,
                             int max_deque_size = 256,
                             std::uint64_t seed = 0x9E3779B97F4A7C15ULL)
      : rng_(static_cast<std::mt19937::result_type>(seed)) {
    deques_.reserve(static_cast<size_t>(n_workers));
    for (int i = 0; i < n_workers; ++i) {
      deques_.push_back(std::make_unique<WorkStealingDeque>(max_deque_size));
    }
  }

  /// @brief Get the deque for a specific worker.
  WorkStealingDeque& get(int worker_id) {
    return *deques_.at(static_cast<size_t>(worker_id));
  }

  /// @brief Attempt to steal from a random victim (not self).
  /// Uses random victim selection for load balancing.
  /// @param thief_id The stealing worker's ID.
  /// @param out Output node if successful.
  /// @return true if a node was stolen.
  bool try_steal(int thief_id, Node& out) {
    const int n = static_cast<int>(deques_.size());
    if (n <= 1) return false;

    // Random starting point to avoid convoy effects.
    int start;
    {
      std::lock_guard<std::mutex> lk(rng_mtx_);
      std::uniform_int_distribution<int> dist(0, n - 2);
      start = dist(rng_);
    }

    // Try all other workers in random order.
    // Map [0, n-2] to [0, n-1] excluding thief_id.
    for (int i = 0; i < n - 1; ++i) {
      int victim = (start + i) % (n - 1);  // Range [0, n-2]
      if (victim >= thief_id) ++victim;     // Now [0, n-1] excluding thief_id
      if (deques_[static_cast<size_t>(victim)]->steal(out)) {
        total_steals_.fetch_add(1, std::memory_order_relaxed);
        return true;
      }
    }
    failed_steals_.fetch_add(1, std::memory_order_relaxed);
    return false;
  }

  /// @brief Total number of live nodes across all deques.
  int total_size() const {
    int s = 0;
    for (const auto& d : deques_) s += d->size();
    return s;
  }

  /// @brief Statistics: successful steals.
  int get_total_steals() const { return total_steals_.load(std::memory_order_relaxed); }
  /// @brief Statistics: failed steal attempts.
  int get_failed_steals() const { return failed_steals_.load(std::memory_order_relaxed); }

  /// @brief Minimum lower bound across all deques.
  double lower_bound() const {
    double lb = kInf;
    for (const auto& d : deques_) lb = std::min(lb, d->lower_bound());
    return lb;
  }
};

/// @brief Lower bounds of nodes currently owned by parallel workers.
///
/// Queue/deque lower bounds only cover idle live nodes. During a solve, the
/// worker's node is temporarily outside every queue but must still be included
/// in the global proof lower bound until it is pruned or publishes children.
class ActiveNodeBounds {
 public:
  explicit ActiveNodeBounds(int n = 0) { resize(n); }

  void resize(int n) {
    std::lock_guard<std::mutex> lk(mtx_);
    bounds_.assign(static_cast<std::size_t>(std::max(0, n)), kInf);
  }

  void enter(int worker, double bound) {
    std::lock_guard<std::mutex> lk(mtx_);
    if (worker >= 0 && worker < static_cast<int>(bounds_.size())) {
      bounds_[static_cast<std::size_t>(worker)] = bound;
    }
  }

  void leave(int worker) {
    std::lock_guard<std::mutex> lk(mtx_);
    if (worker >= 0 && worker < static_cast<int>(bounds_.size())) {
      bounds_[static_cast<std::size_t>(worker)] = kInf;
    }
  }

  double lower_bound() const {
    std::lock_guard<std::mutex> lk(mtx_);
    double lb = kInf;
    for (double b : bounds_) lb = std::min(lb, b);
    return lb;
  }

 private:
  mutable std::mutex mtx_;
  std::vector<double> bounds_;
};

/// @brief Worker role for multi-agent parallel B&C.
enum class WorkerRole {
  Diver,     ///< DFS-focused, fast incumbent finding, all heuristics
  Prover,    ///< Best-bound focused, reliability branching, proving optimality
  IPMDiver,  ///< IPM-based DFS, fast node processing without simplex basis
};

/// @brief Thread-safe queue for cut requests (explorer -> cut worker).
class CutRequestQueue {
  std::mutex mtx_;
  std::condition_variable cv_;
  std::deque<CutRequest> queue_;
  int max_size_;
  std::atomic<bool> shutdown_{false};
public:
  /// @param max_size Maximum pending requests before push is rejected.
  explicit CutRequestQueue(int max_size = 64) : max_size_(max_size) {}

  /// @brief Non-blocking push. Returns false if full or shut down.
  bool try_push(CutRequest&& req) {
    if (shutdown_.load(std::memory_order_relaxed)) return false;
    std::lock_guard<std::mutex> lk(mtx_);
    if (static_cast<int>(queue_.size()) >= max_size_) return false;
    queue_.push_back(std::move(req));
    cv_.notify_one();
    return true;
  }

  /// @brief Blocking pop with timeout. Returns nullopt on timeout or shutdown.
  std::optional<CutRequest> pop(std::chrono::milliseconds timeout = std::chrono::milliseconds(50)) {
    std::unique_lock<std::mutex> lk(mtx_);
    if (!cv_.wait_for(lk, timeout, [&] { return !queue_.empty() || shutdown_.load(std::memory_order_relaxed); })) {
      return std::nullopt;
    }
    if (queue_.empty()) return std::nullopt;
    CutRequest req = std::move(queue_.front());
    queue_.pop_front();
    return req;
  }

  /// @brief Signal shutdown to all waiting threads.
  void request_shutdown() {
    shutdown_.store(true, std::memory_order_release);
    cv_.notify_all();
  }

  /// @brief Check if shutdown has been requested.
  bool is_shutdown() const { return shutdown_.load(std::memory_order_acquire); }
};

/// @brief Thread-safe wrapper around CutPool using reader-writer lock.
class SharedCutPool {
  mutable std::shared_mutex mtx_;
  CutPool pool_;
public:
  SharedCutPool(int max_size = 2000, int max_age = 50,
                int expected_size = -1)
      : pool_(max_size, max_age, expected_size) {}

  void set_expected_size(int expected_size) {
    std::unique_lock<std::shared_mutex> lk(mtx_);
    pool_.set_expected_size(expected_size);
  }

  /// @brief Add a sparse cut (exclusive lock).
  bool add(Eigen::SparseVector<double> coeff, double rhs,
           bool domain_only = false,
           ValidityScope validity_scope = ValidityScope::GlobalCut) {
    std::unique_lock<std::shared_mutex> lk(mtx_);
    return pool_.add(std::move(coeff), rhs, domain_only, nullptr,
                     validity_scope);
  }

  bool add(PoolCut cut) {
    std::unique_lock<std::shared_mutex> lk(mtx_);
    return pool_.add(std::move(cut));
  }

  /// @brief Add a dense cut (exclusive lock, converts to sparse).
  bool add(const Eigen::VectorXd& coeff, double rhs,
           bool domain_only = false,
           ValidityScope validity_scope = ValidityScope::GlobalCut) {
    std::unique_lock<std::shared_mutex> lk(mtx_);
    return pool_.add(coeff, rhs, domain_only, nullptr, validity_scope);
  }

  /// Select, copy, age, and purge under one lock so returned indices cannot be
  /// invalidated by another worker.
  std::vector<PoolCut> select_violated_and_age(
      const Eigen::VectorXd& x, double min_viol, int max_selected) {
    std::unique_lock<std::shared_mutex> lk(mtx_);
    return pool_.select_violated_and_age(x, min_viol, max_selected);
  }

  /// @brief Current pool size (shared lock).
  int size() const {
    std::shared_lock<std::shared_mutex> lk(mtx_);
    return pool_.size();
  }

  /// @brief Seed pool from root cuts (called before threads start, no lock needed).
  void seed(Eigen::SparseVector<double> coeff, double rhs) {
    pool_.add(std::move(coeff), rhs);
  }
  void seed(const Eigen::VectorXd& coeff, double rhs) {
    pool_.add(coeff, rhs);
  }
};

/// @brief Thread-safe wrapper around ConflictPool using reader-writer lock.
class SharedConflictPool {
  mutable std::shared_mutex mtx_;
  ConflictPool pool_;

 public:
  explicit SharedConflictPool(int max_size = 1024, int max_literals = 64,
                              bool minimize = false)
      : pool_(max_size, max_literals, minimize) {}

  /// @brief Add a clause. Exact deduplication happens inside ConflictPool.
  bool add(std::vector<BranchDomainLiteral> literals) {
    std::unique_lock<std::shared_mutex> lk(mtx_);
    return pool_.add(std::move(literals));
  }

  bool has_conflict(const Eigen::VectorXd& node_lb,
                    const Eigen::VectorXd& node_ub,
                    double tol = 1e-9) const {
    std::shared_lock<std::shared_mutex> lk(mtx_);
    return pool_.has_conflict(node_lb, node_ub, tol);
  }

  bool propagate(const std::vector<VariableMeta>& vars,
                 Eigen::VectorXd& node_lb,
                 Eigen::VectorXd& node_ub,
                 int* tightened = nullptr,
                 std::vector<BoundChangeInfo>* changes_out = nullptr,
                 double tol = 1e-9) const {
    std::shared_lock<std::shared_mutex> lk(mtx_);
    return pool_.propagate(vars, node_lb, node_ub, tightened, changes_out, tol);
  }

  bool propagate_with_reasons(
      const std::vector<VariableMeta>& vars,
      Eigen::VectorXd& node_lb,
      Eigen::VectorXd& node_ub,
      int* tightened,
      std::vector<BoundChangeInfo>* changes_out,
      std::vector<DomainReasonBound>* reason_bounds_out,
      double tol = 1e-9) const {
    std::shared_lock<std::shared_mutex> lk(mtx_);
    return pool_.propagate_with_reasons(vars, node_lb, node_ub, tightened,
                                        changes_out, reason_bounds_out, tol);
  }

  int size() const {
    std::shared_lock<std::shared_mutex> lk(mtx_);
    return pool_.size();
  }

};

/// @brief Thread-safe wrapper around SolutionPool using mutex.
class SharedSolutionPool {
  mutable std::mutex mtx_;
  SolutionPool pool_;
public:
  explicit SharedSolutionPool(int max_size = 10) : pool_(max_size) {}

  /// @brief Add a solution (mutex lock).
  bool add(const Eigen::VectorXd& x, double obj) {
    std::lock_guard<std::mutex> lk(mtx_);
    return pool_.add(x, obj);
  }

  /// @brief Guided rounding from pool solutions (mutex lock).
  Eigen::VectorXd guided_rounding(const Eigen::VectorXd& x_relax,
                                    const std::vector<VariableMeta>& vars) const {
    std::lock_guard<std::mutex> lk(mtx_);
    return pool_.guided_rounding(x_relax, vars);
  }

  /// @brief Get the top 2 solutions (for crossover).
  std::pair<PoolSolution, PoolSolution> top2() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return {pool_[0], pool_[1]};
  }

  /// @brief Current pool size (mutex lock).
  int size() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return pool_.size();
  }

  /// @brief Seed pool (called before threads start, no lock needed).
  void seed(const Eigen::VectorXd& x, double obj) {
    pool_.add(x, obj);
  }
};

/// @brief Incumbent sharing with atomic rejection and transactional publication.
struct SharedIncumbent {
  std::atomic<double> obj{kInf};
  std::atomic<bool> has_incumbent{false};
  mutable std::mutex x_mtx;
  Eigen::VectorXd x;

  /// @brief Try to update incumbent.
  ///
  /// The unlocked objective read is only a rejection fast path. The candidate
  /// is rechecked while holding `x_mtx`, then the solution is installed before
  /// the objective is release-published. Readers can therefore never prune on
  /// a newly published objective while the matching solution is still absent.
  /// @return true if incumbent was updated.
  bool try_update(const Eigen::VectorXd& new_x, double new_obj) {
    if (!(new_obj < obj.load(std::memory_order_acquire))) return false;
    std::lock_guard<std::mutex> lk(x_mtx);
    if (!(new_obj < obj.load(std::memory_order_relaxed))) return false;
    x = new_x;
    obj.store(new_obj, std::memory_order_release);
    has_incumbent.store(true, std::memory_order_release);
    return true;
  }

  /// @brief Get current incumbent objective (atomic load).
  double get_obj() const { return obj.load(std::memory_order_acquire); }
  /// @brief Check if any incumbent exists (atomic load).
  bool has() const { return has_incumbent.load(std::memory_order_acquire); }
  /// @brief Get incumbent solution vector (mutex lock + copy).
  Eigen::VectorXd get_x() const {
    std::lock_guard<std::mutex> lk(x_mtx);
    return x;
  }

  /// @brief Atomic snapshot of (has_incumbent, obj) for a single iteration.
  ///
  /// `try_update` installs the solution before release-publishing `obj` and
  /// `has_incumbent`, so an acquired finite objective always has a matching
  /// solution available through `get_x()`.
  /// Callers should use this snapshot throughout one outer iteration
  /// instead of calling `has()` and `get_obj()` at separate sites, which
  /// can witness inconsistent views and make two explorer threads reach
  /// different prune/keep decisions on the same node.
  struct Snapshot {
    bool   has{false};
    double obj{kInf};
  };
  Snapshot snapshot() const {
    const double o = obj.load(std::memory_order_acquire);
    const bool   h = has_incumbent.load(std::memory_order_acquire);
    Snapshot s;
    s.has = h;
    s.obj = h ? o : kInf;
    return s;
  }
};

/// @brief Round-robin turn token used by `deterministic_parallel` mode.
///
/// When enabled, every explorer thread calls `wait_turn(tid)` before the
/// node-queue pop (and again before the child push), and `advance()` when
/// its turn's work is done.  The token's value modulo the active worker
/// count selects the next worker to act, with done workers (marked via
/// `mark_done`) automatically skipped.  Parallelism is preserved over the
/// LP-solve body between each `advance()` and the next `wait_turn`.
class DeterministicTurnToken {
  mutable std::mutex mtx_;
  std::condition_variable cv_;
  std::uint64_t token_{0};
  std::vector<char> done_;
  int n_workers_;

 public:
  explicit DeterministicTurnToken(int n_workers)
      : done_(static_cast<std::size_t>(std::max(1, n_workers)), 0),
        n_workers_(std::max(1, n_workers)) {}

  /// Block until it is worker `tid`'s turn.
  void wait_turn(int tid) {
    std::unique_lock<std::mutex> lk(mtx_);
    cv_.wait(lk, [&] {
      // Skip workers that have exited.  All-done is treated as "my turn"
      // so every caller can unblock and proceed to its shutdown path.
      int skipped = 0;
      while (n_workers_ > 0 && skipped < n_workers_ &&
             done_[static_cast<std::size_t>(token_ % n_workers_)]) {
        ++token_;
        ++skipped;
      }
      if (skipped >= n_workers_) return true;
      return static_cast<int>(token_ % n_workers_) == tid;
    });
  }

  /// Advance the token by one slot.  Callers must pair every successful
  /// `wait_turn` with exactly one `advance`.
  void advance() {
    {
      std::lock_guard<std::mutex> lk(mtx_);
      ++token_;
    }
    cv_.notify_all();
  }

  /// Mark worker `tid` as exited so it is skipped in the rotation.
  void mark_done(int tid) {
    {
      std::lock_guard<std::mutex> lk(mtx_);
      if (tid >= 0 && tid < n_workers_) {
        done_[static_cast<std::size_t>(tid)] = 1;
      }
    }
    cv_.notify_all();
  }
};

/// @brief Thread-safe node queue combining priority queue (best-first) and DFS stack.
class ThreadSafeNodeQueue {
  mutable std::mutex mtx_;
  std::condition_variable cv_;
  using QueueKey = std::pair<double, std::int64_t>;

  std::int64_t next_id_{0};
  std::unordered_map<std::int64_t, Node> nodes_;
  std::multiset<QueueKey> priority_;
  std::multiset<QueueKey> bounds_;
  std::vector<std::int64_t> dfs_;
  std::multiset<QueueKey> dfs_bounds_;
  NodeSelection mode_;
  std::atomic<bool> shutdown_{false};
  bool domain_signature_enabled_{false};
  std::vector<int> domain_signature_cols_;
  Eigen::Index domain_signature_size_{0};
  Eigen::VectorXd root_domain_lb_;
  Eigen::VectorXd root_domain_ub_;
  bool domain_storage_enabled_{false};
  NodeQueueDomainStorageStats domain_storage_stats_;
  std::unordered_map<std::size_t, std::vector<std::int64_t>>
      signature_hash_to_ids_;
  std::unordered_map<std::int64_t, std::size_t> id_to_signature_hash_;

  static double priority_key(const Node& node) {
    if (!std::isfinite(node.bound)) {
      return kInf;
    }
    if (!std::isfinite(node.estimate)) {
      return node.bound;
    }
    return 0.5 * (node.bound + std::max(node.bound, node.estimate));
  }

  double signature_bound_tol(double bound) const {
    return std::max(1e-9, 1e-12 * std::max(1.0, std::abs(bound)));
  }

  Eigen::Index queued_domain_size(const Node& node) const {
    if (node_domain_is_compact(node)) return node.compact_domain_size;
    return node.lb.size() == node.ub.size() ? node.lb.size() : 0;
  }

  double queued_domain_lb(const Node& node, Eigen::Index col) const {
    if (!node_domain_is_compact(node)) return node.lb[col];
    const auto it = std::lower_bound(
        node.compact_domain.begin(), node.compact_domain.end(), col,
        [](const SparseNodeBound& bound, Eigen::Index index) {
          return bound.var_idx < index;
        });
    return it != node.compact_domain.end() && it->var_idx == col
               ? it->lb
               : root_domain_lb_[col];
  }

  double queued_domain_ub(const Node& node, Eigen::Index col) const {
    if (!node_domain_is_compact(node)) return node.ub[col];
    const auto it = std::lower_bound(
        node.compact_domain.begin(), node.compact_domain.end(), col,
        [](const SparseNodeBound& bound, Eigen::Index index) {
          return bound.var_idx < index;
        });
    return it != node.compact_domain.end() && it->var_idx == col
               ? it->ub
               : root_domain_ub_[col];
  }

  bool compact_node_locked(Node& node) noexcept {
    if (!domain_storage_enabled_ || node_domain_is_compact(node)) return true;
    const auto released =
        static_cast<std::uint64_t>(node.lb.size() + node.ub.size());
    std::size_t entries = 0;
    if (!compact_node_domain(node, root_domain_lb_, root_domain_ub_,
                             &entries)) {
      ++domain_storage_stats_.compaction_failures;
      return false;
    }
    ++domain_storage_stats_.compactions;
    domain_storage_stats_.dense_bound_values_released += released;
    domain_storage_stats_.compact_entries_created += entries;
    ++domain_storage_stats_.current_compact_nodes;
    domain_storage_stats_.current_compact_entries += entries;
    domain_storage_stats_.peak_compact_nodes = std::max(
        domain_storage_stats_.peak_compact_nodes,
        domain_storage_stats_.current_compact_nodes);
    domain_storage_stats_.peak_compact_entries = std::max(
        domain_storage_stats_.peak_compact_entries,
        domain_storage_stats_.current_compact_entries);
    return true;
  }

  void release_compact_node_locked(const Node& node) noexcept {
    if (!node_domain_is_compact(node)) return;
    const auto entries =
        static_cast<std::uint64_t>(node.compact_domain.size());
    if (domain_storage_stats_.current_compact_nodes > 0) {
      --domain_storage_stats_.current_compact_nodes;
    }
    domain_storage_stats_.current_compact_entries =
        entries <= domain_storage_stats_.current_compact_entries
            ? domain_storage_stats_.current_compact_entries - entries
            : 0;
  }

  void materialize_node_locked(Node& node) {
    if (!node_domain_is_compact(node)) return;
    const auto entries =
        static_cast<std::uint64_t>(node.compact_domain.size());
    if (!materialize_node_domain(node, root_domain_lb_, root_domain_ub_)) {
      ++domain_storage_stats_.materialization_failures;
      throw std::logic_error(
          "parallel queued node domain cannot be materialized from the root domain");
    }
    ++domain_storage_stats_.materializations;
    if (domain_storage_stats_.current_compact_nodes > 0) {
      --domain_storage_stats_.current_compact_nodes;
    }
    domain_storage_stats_.current_compact_entries =
        entries <= domain_storage_stats_.current_compact_entries
            ? domain_storage_stats_.current_compact_entries - entries
            : 0;
  }

  void materialize_all_locked() {
    for (auto& [id, node] : nodes_) {
      (void)id;
      materialize_node_locked(node);
    }
  }

  void compact_all_locked() {
    if (!domain_storage_enabled_) return;
    for (auto& [id, node] : nodes_) {
      (void)id;
      if (!compact_node_locked(node)) {
        domain_storage_enabled_ = false;
        for (auto& [restore_id, restore_node] : nodes_) {
          (void)restore_id;
          if (node_domain_is_compact(restore_node)) {
            materialize_node_locked(restore_node);
          }
        }
        throw std::invalid_argument(
            "parallel queued node domain does not match the configured root domain");
      }
    }
  }

  std::optional<std::size_t> make_domain_signature_hash(
      const Node& node) const {
    if (!domain_signature_enabled_ ||
        queued_domain_size(node) != domain_signature_size_) {
      return std::nullopt;
    }
    std::size_t hash = 0xcbf29ce484222325ULL;
    auto combine = [&](std::size_t value) {
      hash ^= value + 0x9e3779b97f4a7c15ULL + (hash << 6) + (hash >> 2);
    };
    for (int j : domain_signature_cols_) {
      if (j < 0 || j >= queued_domain_size(node)) continue;
      const double raw_lb = queued_domain_lb(node, j);
      const double raw_ub = queued_domain_ub(node, j);
      const double lb = raw_lb == 0.0 ? 0.0 : raw_lb;
      const double ub = raw_ub == 0.0 ? 0.0 : raw_ub;
      combine(std::hash<int>{}(j));
      combine(std::hash<double>{}(lb));
      combine(std::hash<double>{}(ub));
    }
    return hash;
  }

  bool same_domain(const Node& lhs, const Node& rhs) const {
    if (queued_domain_size(lhs) != queued_domain_size(rhs)) {
      return false;
    }
    for (int j : domain_signature_cols_) {
      if (j < 0 || j >= queued_domain_size(lhs)) continue;
      const double raw_lhs_lb = queued_domain_lb(lhs, j);
      const double raw_rhs_lb = queued_domain_lb(rhs, j);
      const double raw_lhs_ub = queued_domain_ub(lhs, j);
      const double raw_rhs_ub = queued_domain_ub(rhs, j);
      const double lhs_lb = raw_lhs_lb == 0.0 ? 0.0 : raw_lhs_lb;
      const double rhs_lb = raw_rhs_lb == 0.0 ? 0.0 : raw_rhs_lb;
      const double lhs_ub = raw_lhs_ub == 0.0 ? 0.0 : raw_lhs_ub;
      const double rhs_ub = raw_rhs_ub == 0.0 ? 0.0 : raw_rhs_ub;
      if (lhs_lb != rhs_lb || lhs_ub != rhs_ub) return false;
    }
    return true;
  }

  void forget_domain_signature_locked(std::int64_t id) {
    auto hash_it = id_to_signature_hash_.find(id);
    if (hash_it == id_to_signature_hash_.end()) return;
    auto bucket_it = signature_hash_to_ids_.find(hash_it->second);
    if (bucket_it != signature_hash_to_ids_.end()) {
      auto& ids = bucket_it->second;
      ids.erase(std::remove(ids.begin(), ids.end(), id), ids.end());
      if (ids.empty()) signature_hash_to_ids_.erase(bucket_it);
    }
    id_to_signature_hash_.erase(hash_it);
  }

  void erase_id_locked(std::int64_t id) {
    auto node_it = nodes_.find(id);
    if (node_it == nodes_.end()) return;
    const Node& node = node_it->second;
    auto prio_it = priority_.find({priority_key(node), id});
    if (prio_it != priority_.end()) priority_.erase(prio_it);
    auto bound_it = bounds_.find({node.bound, id});
    if (bound_it != bounds_.end()) bounds_.erase(bound_it);
    auto dfs_bound_it = dfs_bounds_.find({node.bound, id});
    if (dfs_bound_it != dfs_bounds_.end()) dfs_bounds_.erase(dfs_bound_it);
    dfs_.erase(std::remove(dfs_.begin(), dfs_.end(), id), dfs_.end());
    forget_domain_signature_locked(id);
    release_compact_node_locked(node_it->second);
    nodes_.erase(node_it);
  }

  std::int64_t store_locked(Node node) {
    std::optional<std::size_t> signature_hash;
    if (domain_signature_enabled_) {
      signature_hash = make_domain_signature_hash(node);
      if (signature_hash.has_value()) {
        const auto bucket_it = signature_hash_to_ids_.find(*signature_hash);
        if (bucket_it != signature_hash_to_ids_.end()) {
          const std::vector<std::int64_t> candidates = bucket_it->second;
          for (std::int64_t existing_id : candidates) {
            auto existing_node = nodes_.find(existing_id);
            if (existing_node == nodes_.end() ||
                !same_domain(existing_node->second, node)) {
              continue;
            }
            if (existing_node->second.bound <=
                node.bound + signature_bound_tol(node.bound)) {
              return -1;
            }
            erase_id_locked(existing_id);
            break;
          }
        }
      }
    }
    const auto id = next_id_++;
    if (domain_storage_enabled_ && !compact_node_locked(node)) {
      throw std::invalid_argument(
          "parallel queued node domain does not match the configured root domain");
    }
    nodes_.emplace(id, std::move(node));
    if (signature_hash.has_value()) {
      signature_hash_to_ids_[*signature_hash].push_back(id);
      id_to_signature_hash_[id] = *signature_hash;
    }
    return id;
  }

  bool take_id_locked(std::int64_t id, Node& out) {
    auto node_it = nodes_.find(id);
    if (node_it == nodes_.end()) return false;
    materialize_node_locked(node_it->second);
    const Node& node = node_it->second;
    auto priority_it = priority_.find({priority_key(node), id});
    if (priority_it != priority_.end()) priority_.erase(priority_it);
    auto bound_it = bounds_.find({node.bound, id});
    if (bound_it != bounds_.end()) bounds_.erase(bound_it);
    auto dfs_bound_it = dfs_bounds_.find({node.bound, id});
    if (dfs_bound_it != dfs_bounds_.end()) dfs_bounds_.erase(dfs_bound_it);
    dfs_.erase(std::remove(dfs_.begin(), dfs_.end(), id), dfs_.end());
    out = std::move(node_it->second);
    forget_domain_signature_locked(id);
    nodes_.erase(node_it);
    return true;
  }

  bool pop_dfs_locked(Node& out) {
    return !dfs_.empty() && take_id_locked(dfs_.back(), out);
  }

  bool pop_priority_locked(Node& out) {
    return !priority_.empty() && take_id_locked(priority_.begin()->second, out);
  }

  bool pop_bestbound_locked(Node& out) {
    const bool have_priority_bound = !bounds_.empty();
    const bool have_dfs_bound = !dfs_bounds_.empty();
    if (!have_priority_bound && !have_dfs_bound) {
      return false;
    }

    if (have_dfs_bound &&
        (!have_priority_bound || dfs_bounds_.begin()->first <= bounds_.begin()->first)) {
      return take_id_locked(dfs_bounds_.begin()->second, out);
    }
    return take_id_locked(bounds_.begin()->second, out);
  }

public:
  explicit ThreadSafeNodeQueue(NodeSelection mode) : mode_(mode) {}

  void configure_domain_signature(const std::vector<char>& branchable_cols,
                                  const Eigen::VectorXd& root_lb,
                                  const Eigen::VectorXd& root_ub,
                                  double tol = 1e-9) {
    std::lock_guard<std::mutex> lk(mtx_);
    materialize_all_locked();
    root_domain_lb_ = root_lb;
    root_domain_ub_ = root_ub;
    domain_storage_enabled_ =
        root_lb.size() > 0 && root_lb.size() == root_ub.size();
    domain_signature_cols_.clear();
    domain_signature_cols_.reserve(branchable_cols.size());
    for (std::size_t j = 0; j < branchable_cols.size(); ++j) {
      if (branchable_cols[j] != 0) {
        domain_signature_cols_.push_back(static_cast<int>(j));
      }
    }
    domain_signature_size_ = root_lb.size();
    (void)tol;
    domain_signature_enabled_ =
        !domain_signature_cols_.empty() &&
        root_lb.size() == root_ub.size();
    signature_hash_to_ids_.clear();
    id_to_signature_hash_.clear();
    std::vector<std::int64_t> doomed;
    std::unordered_set<std::int64_t> doomed_set;
    std::vector<std::int64_t> ids;
    ids.reserve(nodes_.size());
    for (const auto& [id, node] : nodes_) {
      (void)node;
      ids.push_back(id);
    }
    std::sort(ids.begin(), ids.end());
    for (std::int64_t id : ids) {
      if (doomed_set.count(id) != 0) continue;
      const auto node_it = nodes_.find(id);
      if (node_it == nodes_.end()) continue;
      const auto signature_hash = make_domain_signature_hash(node_it->second);
      if (!signature_hash.has_value()) continue;
      auto& bucket = signature_hash_to_ids_[*signature_hash];
      auto existing = std::find_if(
          bucket.begin(), bucket.end(), [&](std::int64_t candidate_id) {
            const auto candidate = nodes_.find(candidate_id);
            return candidate != nodes_.end() &&
                   same_domain(candidate->second, node_it->second);
          });
      if (existing == bucket.end()) {
        bucket.push_back(id);
        id_to_signature_hash_[id] = *signature_hash;
        continue;
      }
      const auto keep_it = nodes_.find(*existing);
      if (keep_it != nodes_.end() &&
          keep_it->second.bound <=
              node_it->second.bound +
                  signature_bound_tol(node_it->second.bound)) {
        doomed.push_back(id);
        doomed_set.insert(id);
      } else {
        doomed.push_back(*existing);
        doomed_set.insert(*existing);
        id_to_signature_hash_.erase(*existing);
        *existing = id;
        id_to_signature_hash_[id] = *signature_hash;
      }
    }
    for (std::int64_t id : doomed) erase_id_locked(id);
    compact_all_locked();
  }

  NodeQueueDomainStorageStats domain_storage_stats() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return domain_storage_stats_;
  }

  /// @brief Pop a node (blocks briefly if empty).
  /// @return false on shutdown/timeout.
  bool pop(Node& out, bool has_incumbent,
           std::chrono::milliseconds timeout = std::chrono::milliseconds(10)) {
    std::unique_lock<std::mutex> lk(mtx_);
    if (!cv_.wait_for(lk, timeout, [&] {
      return !dfs_.empty() || !priority_.empty() || shutdown_.load(std::memory_order_relaxed);
    })) {
      return false;
    }
    if (shutdown_.load(std::memory_order_relaxed) && dfs_.empty() && priority_.empty()) {
      return false;
    }

    if (mode_ == NodeSelection::DepthFirst) {
      return pop_dfs_locked(out);
    }

    if (mode_ == NodeSelection::Hybrid && !has_incumbent && !dfs_.empty()) {
      return pop_dfs_locked(out);
    }

    if (mode_ == NodeSelection::Hybrid && has_incumbent) {
      return pop_bestbound_locked(out);
    }

    if (!dfs_.empty()) {
      return pop_dfs_locked(out);
    }

    return pop_priority_locked(out);
  }

  /// @brief Push a node to PQ or DFS based on mode.
  void push(Node node, bool has_incumbent) {
    std::lock_guard<std::mutex> lk(mtx_);
    if (mode_ == NodeSelection::DepthFirst) {
      const auto id = store_locked(std::move(node));
      if (id < 0) return;
      dfs_.push_back(id);
      dfs_bounds_.emplace(nodes_.at(id).bound, id);
    } else if (mode_ == NodeSelection::Hybrid && !has_incumbent) {
      const auto id = store_locked(std::move(node));
      if (id < 0) return;
      dfs_.push_back(id);
      dfs_bounds_.emplace(nodes_.at(id).bound, id);
    } else {
      const auto id = store_locked(std::move(node));
      if (id < 0) return;
      const Node& stored = nodes_.at(id);
      priority_.emplace(priority_key(stored), id);
      bounds_.emplace(stored.bound, id);
    }
    cv_.notify_one();
  }

  /// @brief Push directly to DFS stack (for plunging children).
  void push_dfs(Node node) {
    std::lock_guard<std::mutex> lk(mtx_);
    const auto id = store_locked(std::move(node));
    if (id < 0) return;
    dfs_.push_back(id);
    dfs_bounds_.emplace(nodes_.at(id).bound, id);
    cv_.notify_one();
  }

  /// @brief Push directly to PQ (for best-bound workers).
  void push_pq(Node node) {
    std::lock_guard<std::mutex> lk(mtx_);
    const auto id = store_locked(std::move(node));
    if (id < 0) return;
    const Node& stored = nodes_.at(id);
    priority_.emplace(priority_key(stored), id);
    bounds_.emplace(stored.bound, id);
    cv_.notify_one();
  }

  /// @brief Remove queued nodes already dominated by the incumbent bound.
  int prune_by_incumbent(bool has_incumbent,
                         double incumbent_obj,
                         double prune_tol = 1e-12) {
    if (!has_incumbent || !std::isfinite(incumbent_obj)) {
      return 0;
    }
    std::lock_guard<std::mutex> lk(mtx_);
    if (nodes_.empty()) return 0;

    std::vector<std::int64_t> doomed;
    doomed.reserve(nodes_.size());
    for (const auto& [id, node] : nodes_) {
      if (std::isfinite(node.bound) &&
          node.bound >= incumbent_obj - prune_tol) {
        doomed.push_back(id);
      }
    }
    for (const auto id : doomed) erase_id_locked(id);
    return static_cast<int>(doomed.size());
  }

  /// @brief Pop best-bound node from PQ only (for Prover workers).
  bool pop_bestbound(Node& out,
                     std::chrono::milliseconds timeout = std::chrono::milliseconds(10)) {
    std::unique_lock<std::mutex> lk(mtx_);
    if (!cv_.wait_for(lk, timeout, [&] {
      return !bounds_.empty() || !dfs_.empty() || shutdown_.load(std::memory_order_relaxed);
    })) {
      return false;
    }
    if (shutdown_.load(std::memory_order_relaxed) && bounds_.empty() && dfs_.empty()) {
      return false;
    }
    return pop_bestbound_locked(out);
  }

  /// @brief Minimum lower bound across all live nodes.
  double lower_bound() const {
    std::lock_guard<std::mutex> lk(mtx_);
    double lb = kInf;
    if (!bounds_.empty()) lb = bounds_.begin()->first;
    if (!dfs_bounds_.empty()) lb = std::min(lb, dfs_bounds_.begin()->first);
    return lb;
  }

  /// @brief Check if both PQ and DFS are empty.
  bool empty() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return priority_.empty() && dfs_.empty();
  }

  /// @brief Total number of live nodes.
  int size() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return static_cast<int>(priority_.size()) + static_cast<int>(dfs_.size());
  }

  int priority_size() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return static_cast<int>(priority_.size());
  }

  int dfs_size() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return static_cast<int>(dfs_.size());
  }

  int count_clause_hits(const std::vector<BranchDomainLiteral>& literals,
                        bool require_unit_or_conflict = true,
                        int max_hits = std::numeric_limits<int>::max(),
                        double tol = 1e-9) const {
    if (literals.empty()) return 0;
    std::lock_guard<std::mutex> lk(mtx_);
    int hits = 0;
    for (const auto& [id, node] : nodes_) {
      (void)id;
      int unsatisfied = 0;
      for (const auto& literal : literals) {
        if (literal.var_idx < 0 ||
            literal.var_idx >= queued_domain_size(node)) {
          unsatisfied = 2;
          break;
        }
        const bool active = literal.is_lb
            ? (queued_domain_lb(node, literal.var_idx) >= literal.value - tol)
            : (queued_domain_ub(node, literal.var_idx) <= literal.value + tol);
        if (!active && ++unsatisfied > 1) break;
      }
      if (require_unit_or_conflict ? (unsatisfied <= 1)
                                   : (unsatisfied == 0)) {
        ++hits;
        if (hits >= max_hits) return hits;
      }
    }
    return hits;
  }

  /// @brief Signal shutdown to all waiting threads.
  void request_shutdown() {
    shutdown_.store(true, std::memory_order_release);
    cv_.notify_all();
  }
};

/// @brief Atomic statistics counters for the parallel B&C.
/// Each high-frequency counter occupies its own cache line (64 bytes)
/// to eliminate false sharing between 8+ parallel workers.  The two hot
/// counters (`nodes_explored`, `lp_solves`) get individual lines; the
/// remaining counters share lines since they are updated infrequently.
struct AtomicBCStats {
  // Hot path — one cache line each to avoid MESI write-invalidate between workers.
  alignas(64) std::atomic<int> nodes_explored{0};
  alignas(64) std::atomic<int> lp_solves{0};
  // Less-hot counters — share lines; updated at most once per cut round or
  // on worker exit, so false sharing cost is negligible.
  alignas(64) std::atomic<int> cuts_added{0};
               std::atomic<int> pool_cuts_separated{0};
               std::atomic<int> cut_requests_submitted{0};
               std::atomic<int> cut_requests_dropped{0};
               std::atomic<int> cut_worker_cuts{0};
               std::atomic<int> cut_worker_incumbents{0};
               std::atomic<int> crossover_incumbents{0};
  // Parallel sharing telemetry (2026-Q2 improvements).
  std::atomic<std::uint64_t> shared_implications_published{0};
  std::atomic<std::uint64_t> shared_implications_consumed{0};
  std::atomic<std::uint64_t> shared_implication_pulls{0};
  std::atomic<std::uint64_t> rc_fixings{0};
  std::atomic<std::uint64_t> rc_forbidden_literals{0};
  std::atomic<std::uint64_t> rc_conflict_clauses_learned{0};
  std::atomic<std::uint64_t> rc_binary_implications_learned{0};
  std::atomic<std::uint64_t> rc_conflict_cuts_added{0};
  std::atomic<std::uint64_t> rc_proof_conflict_clauses_learned{0};
  std::atomic<std::uint64_t> rc_proof_conflict_literals_before{0};
  std::atomic<std::uint64_t> rc_proof_conflict_literals_after{0};
  std::atomic<std::uint64_t> rc_conflict_verification_lps{0};
  std::atomic<std::uint64_t> rc_verified_conflict_clauses_learned{0};
  std::atomic<std::uint64_t> gap_suboptimal_queue_prune_passes{0};
  std::atomic<std::uint64_t> gap_suboptimal_queue_prunes{0};
  std::atomic<std::uint64_t> branch_direction_preferred_down{0};
  std::atomic<std::uint64_t> branch_direction_preferred_up{0};
  std::atomic<std::uint64_t> branch_direction_first_down{0};
  std::atomic<std::uint64_t> branch_direction_first_up{0};
  std::atomic<std::uint64_t> branch_first_child_incumbent_updates{0};
  std::atomic<std::uint64_t> branch_first_child_cutoffs{0};
  std::atomic<std::uint64_t> branch_second_child_cutoffs{0};
  std::atomic<std::uint64_t> branch_payload_child_creations{0};
  std::atomic<std::uint64_t> branch_payload_shared_vectors{0};
  std::atomic<std::uint64_t> branch_payload_shared_elements{0};
  std::atomic<std::uint64_t> branch_domain_dense_copies{0};
  std::atomic<std::uint64_t> branch_domain_dense_values_copied{0};
  std::atomic<std::uint64_t> branch_domain_moves{0};
  std::atomic<std::uint64_t> strong_probe_base_sf_materializations{0};
  std::atomic<std::uint64_t> strong_probe_bound_transactions{0};
  std::atomic<std::uint64_t> strong_probe_transaction_snapshot_values{0};
  std::atomic<std::uint64_t> strong_probe_transaction_rollbacks{0};
  std::atomic<std::uint64_t> strong_probe_transaction_failures{0};
  std::atomic<std::uint64_t> strong_probe_backend_cold_solves{0};
  std::atomic<std::uint64_t> strong_probe_backend_persistent_resolves{0};
  std::atomic<std::uint64_t> separator_sparse_candidates_created{0};
  std::atomic<std::uint64_t> separator_sparse_candidate_entries_created{0};
  std::atomic<std::uint64_t> separator_peak_live_sparse_candidates{0};
  std::atomic<std::uint64_t> separator_peak_live_sparse_entries{0};
  std::atomic<std::uint64_t> separator_sparse_aggregation_snapshots{0};
  std::atomic<std::uint64_t> separator_sparse_aggregation_entries{0};
  std::atomic<std::uint64_t> separator_dense_workspace_materializations{0};
  std::atomic<std::uint64_t> separator_dense_workspace_values{0};
  std::atomic<std::uint64_t> separator_matrix_append_calls{0};
  std::atomic<std::uint64_t> separator_matrix_appended_rows{0};
  std::atomic<std::uint64_t> separator_matrix_appended_entries{0};
  std::atomic<std::uint64_t>
      separator_matrix_prior_entries_bypassing_triplet_rebuild{0};
  std::atomic<std::uint64_t> separator_matrix_storage_reallocations{0};
  std::atomic<std::uint64_t> separator_matrix_peak_spare_entries{0};
  std::atomic<std::uint64_t> node_estimate_calibration_samples{0};
  std::atomic<double> node_estimate_predicted_lift_sum{0.0};
  std::atomic<double> node_estimate_realized_lift_sum{0.0};
  std::atomic<double> node_estimate_abs_error_sum{0.0};
  std::atomic<double> node_estimate_squared_error_sum{0.0};
  std::atomic<double> node_estimate_predicted_sq_sum{0.0};
  std::atomic<double> node_estimate_realized_sq_sum{0.0};
  std::atomic<double> node_estimate_cross_sum{0.0};
  std::atomic<std::uint64_t> directional_calibration_samples{0};
  std::atomic<double> directional_predicted_gain_sum{0.0};
  std::atomic<double> directional_realized_gain_sum{0.0};
  std::atomic<double> directional_abs_error_sum{0.0};
  std::atomic<double> directional_squared_error_sum{0.0};
  std::atomic<std::uint64_t> directional_rank_samples{0};
  std::atomic<std::uint64_t> directional_rank_concordant{0};
};

/// @brief Append-only thread-safe store of binary implications learned across
/// all parallel explorers. Supports a generation-counter fast-path so readers
/// can detect "no new arcs" without acquiring any lock.
///
/// Arcs are globally valid iff derived from constraints visible to every
/// explorer (root rows + shared conflict clauses). All learned 2-literal
/// binary conflicts satisfy this; see
/// `docs/archive/solvers.md`, section 8.
class SharedImplicationGraph {
 public:
  struct Arc {
    int trigger_var{-1};
    bool trigger_value_one{false};
    int implied_var{-1};
    bool implied_is_lb{false};
    double implied_value{0.0};
  };

  SharedImplicationGraph() = default;

  /// @brief Append a batch of arcs (unique-write lock; increments generation).
  /// @return number of arcs actually appended (deduplication is done by
  ///         caller's local graph; we store as-is, readers dedup on insert).
  std::size_t publish(const std::vector<Arc>& batch) {
    if (batch.empty()) return 0;
    std::unique_lock<std::shared_mutex> lk(mtx_);
    arcs_.insert(arcs_.end(), batch.begin(), batch.end());
    gen_.fetch_add(static_cast<std::uint64_t>(batch.size()),
                   std::memory_order_release);
    return batch.size();
  }

  /// @brief Current generation counter (lock-free).
  std::uint64_t generation() const {
    return gen_.load(std::memory_order_acquire);
  }

  /// @brief Copy arcs in generation range [from, to) into `out`.
  ///
  /// `to` is interpreted as the generation returned by a previous
  /// `generation()` call. `from` is the last generation the caller consumed.
  /// Returns the new "last consumed" value (== snapshot size at call time).
  std::uint64_t pull_since(std::uint64_t from, std::vector<Arc>& out) const {
    std::shared_lock<std::shared_mutex> lk(mtx_);
    const std::uint64_t to = gen_.load(std::memory_order_acquire);
    if (from >= to) return to;
    const std::size_t start = static_cast<std::size_t>(from);
    const std::size_t end = static_cast<std::size_t>(to);
    if (start < arcs_.size() && end <= arcs_.size()) {
      out.insert(out.end(), arcs_.begin() + start, arcs_.begin() + end);
    }
    return to;
  }

  std::size_t size() const {
    std::shared_lock<std::shared_mutex> lk(mtx_);
    return arcs_.size();
  }

 private:
  mutable std::shared_mutex mtx_;
  std::vector<Arc> arcs_;
  std::atomic<std::uint64_t> gen_{0};
};

}  // namespace mipsolvers::engine::detail
