/// @file bc_threading.hpp
/// @brief Thread-safe data structures for heterogeneous parallel B&C.
///
/// Provides SharedCutPool (reader-writer lock), SharedConflictPool (reader-writer lock), SharedSolutionPool (mutex),
/// SharedIncumbent (lock-free CAS + mutex), ThreadSafeNodeQueue, CutRequestQueue,
/// WorkStealingDeque (P5.1), and AtomicBCStats for the parallel B&C architecture.

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
#include <unordered_map>
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

/// @brief P5.1 Work-Stealing Deque for per-worker local node storage.
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
  /// @param seed Deterministic seed for victim selection; pass 0 to request
  ///             a non-deterministic (random_device-based) seed.
  explicit WorkStealingPool(int n_workers,
                             int max_deque_size = 256,
                             std::uint64_t seed = 0x9E3779B97F4A7C15ULL)
      : rng_(seed != 0
                 ? static_cast<std::mt19937::result_type>(seed)
                 : static_cast<std::mt19937::result_type>(std::random_device{}())) {
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
  Eigen::VectorXd domain_signature_root_lb_;
  Eigen::VectorXd domain_signature_root_ub_;
  std::unordered_map<std::string, std::int64_t> signature_to_id_;
  std::unordered_map<std::int64_t, std::string> id_to_signature_;

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

  template <typename T>
  static void append_signature_bytes(std::string& sig, const T& value) {
    sig.append(reinterpret_cast<const char*>(&value), sizeof(T));
  }

  std::optional<std::string> make_domain_signature(const Node& node) const {
    if (!domain_signature_enabled_ ||
        node.lb.size() != node.ub.size() ||
        node.lb.size() != domain_signature_root_lb_.size() ||
        node.ub.size() != domain_signature_root_ub_.size()) {
      return std::nullopt;
    }
    std::string sig;
    sig.reserve(domain_signature_cols_.size() * 26);
    constexpr char lower_tag = 'L';
    constexpr char upper_tag = 'U';
    for (int j : domain_signature_cols_) {
      if (j < 0 || j >= node.lb.size()) continue;
      const std::int32_t col = static_cast<std::int32_t>(j);
      const double lb = node.lb[j] == 0.0 ? 0.0 : node.lb[j];
      const double ub = node.ub[j] == 0.0 ? 0.0 : node.ub[j];
      append_signature_bytes(sig, col);
      sig.push_back(lower_tag);
      append_signature_bytes(sig, lb);
      append_signature_bytes(sig, col);
      sig.push_back(upper_tag);
      append_signature_bytes(sig, ub);
    }
    return sig;
  }

  void forget_domain_signature_locked(std::int64_t id) {
    auto sig_it = id_to_signature_.find(id);
    if (sig_it == id_to_signature_.end()) return;
    auto owner_it = signature_to_id_.find(sig_it->second);
    if (owner_it != signature_to_id_.end() && owner_it->second == id) {
      signature_to_id_.erase(owner_it);
    }
    id_to_signature_.erase(sig_it);
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
    nodes_.erase(node_it);
  }

  std::int64_t store_locked(Node node) {
    std::optional<std::string> sig;
    if (domain_signature_enabled_) {
      sig = make_domain_signature(node);
      if (sig.has_value()) {
        auto existing = signature_to_id_.find(*sig);
        if (existing != signature_to_id_.end()) {
          auto existing_node = nodes_.find(existing->second);
          if (existing_node != nodes_.end()) {
            if (existing_node->second.bound <=
                node.bound + signature_bound_tol(node.bound)) {
              return -1;
            }
            erase_id_locked(existing->second);
          } else {
            signature_to_id_.erase(existing);
          }
        }
      }
    }
    const auto id = next_id_++;
    nodes_.emplace(id, std::move(node));
    if (sig.has_value()) {
      signature_to_id_[*sig] = id;
      id_to_signature_[id] = std::move(*sig);
    }
    return id;
  }

  bool pop_dfs_locked(Node& out) {
    if (dfs_.empty()) {
      return false;
    }

    const auto id = dfs_.back();
    dfs_.pop_back();

    auto node_it = nodes_.find(id);
    if (node_it == nodes_.end()) {
      return false;
    }

    const double bound = node_it->second.bound;
    auto dfs_bound_it = dfs_bounds_.find({bound, id});
    if (dfs_bound_it != dfs_bounds_.end()) {
      dfs_bounds_.erase(dfs_bound_it);
    }

    out = std::move(node_it->second);
    forget_domain_signature_locked(id);
    nodes_.erase(node_it);
    return true;
  }

  bool pop_priority_locked(Node& out) {
    if (priority_.empty()) {
      return false;
    }

    auto it = priority_.begin();
    const auto [priority, id] = *it;
    priority_.erase(it);

    auto node_it = nodes_.find(id);
    if (node_it == nodes_.end()) {
      return false;
    }

    const double bound = node_it->second.bound;
    auto bound_it = bounds_.find({bound, id});
    if (bound_it != bounds_.end()) {
      bounds_.erase(bound_it);
    }

    out = std::move(node_it->second);
    forget_domain_signature_locked(id);
    nodes_.erase(node_it);
    (void)priority;
    return true;
  }

  bool pop_bestbound_locked(Node& out) {
    const bool have_priority_bound = !bounds_.empty();
    const bool have_dfs_bound = !dfs_bounds_.empty();
    if (!have_priority_bound && !have_dfs_bound) {
      return false;
    }

    if (have_dfs_bound &&
        (!have_priority_bound || dfs_bounds_.begin()->first <= bounds_.begin()->first)) {
      auto it = dfs_bounds_.begin();
      const auto [bound, id] = *it;
      dfs_bounds_.erase(it);

      auto node_it = nodes_.find(id);
      if (node_it == nodes_.end()) {
        return false;
      }

      auto prio_it = priority_.find({priority_key(node_it->second), id});
      if (prio_it != priority_.end()) {
        priority_.erase(prio_it);
      }
      auto bound_it = bounds_.find({bound, id});
      if (bound_it != bounds_.end()) {
        bounds_.erase(bound_it);
      }
      dfs_.erase(std::remove(dfs_.begin(), dfs_.end(), id), dfs_.end());

      out = std::move(node_it->second);
      forget_domain_signature_locked(id);
      nodes_.erase(node_it);
      return true;
    }

    auto it = bounds_.begin();
    const auto [bound, id] = *it;
    bounds_.erase(it);

    auto node_it = nodes_.find(id);
    if (node_it == nodes_.end()) {
      return false;
    }

    auto prio_it = priority_.find({priority_key(node_it->second), id});
    if (prio_it != priority_.end()) {
      priority_.erase(prio_it);
    }

    out = std::move(node_it->second);
    forget_domain_signature_locked(id);
    nodes_.erase(node_it);
    (void)bound;
    return true;
  }

public:
  explicit ThreadSafeNodeQueue(NodeSelection mode) : mode_(mode) {}

  void configure_domain_signature(const std::vector<char>& branchable_cols,
                                  const Eigen::VectorXd& root_lb,
                                  const Eigen::VectorXd& root_ub,
                                  double tol = 1e-9) {
    std::lock_guard<std::mutex> lk(mtx_);
    domain_signature_cols_.clear();
    domain_signature_cols_.reserve(branchable_cols.size());
    for (std::size_t j = 0; j < branchable_cols.size(); ++j) {
      if (branchable_cols[j] != 0) {
        domain_signature_cols_.push_back(static_cast<int>(j));
      }
    }
    domain_signature_root_lb_ = root_lb;
    domain_signature_root_ub_ = root_ub;
    (void)tol;
    domain_signature_enabled_ =
        !domain_signature_cols_.empty() &&
        root_lb.size() == root_ub.size();
    signature_to_id_.clear();
    id_to_signature_.clear();
    if (!domain_signature_enabled_) return;
    std::vector<std::int64_t> doomed;
    for (const auto& [id, node] : nodes_) {
      auto sig = make_domain_signature(node);
      if (!sig.has_value()) continue;
      const auto existing = signature_to_id_.find(*sig);
      if (existing == signature_to_id_.end()) {
        signature_to_id_.emplace(*sig, id);
        id_to_signature_.emplace(id, *sig);
        continue;
      }
      const auto keep_it = nodes_.find(existing->second);
      if (keep_it != nodes_.end() &&
          keep_it->second.bound <= node.bound + signature_bound_tol(node.bound)) {
        doomed.push_back(id);
      } else {
        doomed.push_back(existing->second);
        signature_to_id_[*sig] = id;
        id_to_signature_[id] = *sig;
      }
    }
    for (std::int64_t id : doomed) erase_id_locked(id);
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
            literal.var_idx >= static_cast<int>(node.lb.size()) ||
            literal.var_idx >= static_cast<int>(node.ub.size())) {
          unsatisfied = 2;
          break;
        }
        const bool active = literal.is_lb
            ? (node.lb[literal.var_idx] >= literal.value - tol)
            : (node.ub[literal.var_idx] <= literal.value + tol);
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
/// P7.5: Each high-frequency counter occupies its own cache line (64 bytes)
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
/// `docs/parallel_bc_sharing_improvements_2026q2.md`.
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
