/// @file thread_pool.hpp
/// @brief Lightweight persistent thread pool for parallel power-flow kernels.
///
/// Provides a fixed-size worker pool with submit() and parallel_for() primitives.
/// A process-wide singleton is available via ThreadPool::global().

#pragma once

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <functional>
#include <future>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

namespace mipsolvers::util {

class ThreadPool {
 public:
  /// Construct a pool with \p num_threads workers.
  /// If num_threads <= 0, uses hardware_concurrency().
  explicit ThreadPool(int num_threads = 0) {
    if (num_threads <= 0) {
      num_threads = static_cast<int>(std::thread::hardware_concurrency());
      if (num_threads <= 0) num_threads = 1;
    }
    workers_.reserve(static_cast<size_t>(num_threads));
    for (int i = 0; i < num_threads; ++i) {
      workers_.emplace_back([this] { worker_loop(); });
    }
  }

  ~ThreadPool() {
    {
      std::lock_guard<std::mutex> lk(mtx_);
      shutdown_ = true;
    }
    cv_.notify_all();
    for (auto& w : workers_) {
      if (w.joinable()) w.join();
    }
  }

  ThreadPool(const ThreadPool&) = delete;
  ThreadPool& operator=(const ThreadPool&) = delete;

  /// Number of worker threads.
  int size() const { return static_cast<int>(workers_.size()); }

  /// Submit a callable and obtain a future for its result.
  template <typename F>
  auto submit(F&& f) -> std::future<decltype(f())> {
    using R = decltype(f());
    auto task = std::make_shared<std::packaged_task<R()>>(std::forward<F>(f));
    std::future<R> result = task->get_future();
    {
      std::lock_guard<std::mutex> lk(mtx_);
      tasks_.emplace([task]() { (*task)(); });
    }
    cv_.notify_one();
    return result;
  }

  /// Divide [0, count) into chunks and execute \p body(begin, end) on each.
  /// Chunk 0 runs on the calling thread; the rest are dispatched to the pool.
  /// Blocks until all chunks complete.
  void parallel_for(size_t count,
                    const std::function<void(size_t, size_t)>& body,
                    int num_tasks = 0) {
    if (count == 0) return;
    if (num_tasks <= 0) num_tasks = size();
    num_tasks = std::min(num_tasks, static_cast<int>(count));
    if (num_tasks <= 1) {
      body(0, count);
      return;
    }

    std::vector<std::future<void>> futures;
    futures.reserve(static_cast<size_t>(num_tasks - 1));

    for (int t = 1; t < num_tasks; ++t) {
      const size_t begin =
          (count * static_cast<size_t>(t)) / static_cast<size_t>(num_tasks);
      const size_t end =
          (count * static_cast<size_t>(t + 1)) / static_cast<size_t>(num_tasks);
      futures.push_back(submit([&body, begin, end]() { body(begin, end); }));
    }

    // Chunk 0 runs on the caller.
    const size_t end0 = count / static_cast<size_t>(num_tasks);
    body(0, end0);

    for (auto& f : futures) {
      f.get();
    }
  }

  /// Process-wide singleton (lazy, thread-safe).
  static ThreadPool& global() {
    static ThreadPool pool;
    return pool;
  }

 private:
  void worker_loop() {
    while (true) {
      std::function<void()> task;
      {
        std::unique_lock<std::mutex> lk(mtx_);
        cv_.wait(lk, [this] { return shutdown_ || !tasks_.empty(); });
        if (shutdown_ && tasks_.empty()) return;
        task = std::move(tasks_.front());
        tasks_.pop();
      }
      task();
    }
  }

  std::vector<std::thread> workers_;
  std::queue<std::function<void()>> tasks_;
  std::mutex mtx_;
  std::condition_variable cv_;
  bool shutdown_{false};
};

}  // namespace mipsolvers::util
