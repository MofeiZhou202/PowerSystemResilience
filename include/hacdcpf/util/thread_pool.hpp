/// @file thread_pool.hpp
/// @brief Lightweight persistent thread pool for parallel power-flow kernels.
///
/// Provides a fixed-size worker pool with submit() and parallel_for() primitives.
/// A process-wide singleton is available via ThreadPool::global().

#pragma once

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <exception>
#include <functional>
#include <future>
#include <mutex>
#include <queue>
#include <system_error>
#include <thread>
#include <vector>

#if !defined(_WIN32)
#include <pthread.h>
#endif

namespace hacdcpf::util {

class ThreadPool {
 public:
  /// Construct a pool with \p num_threads workers.
  /// If num_threads <= 0, uses hardware_concurrency().
  explicit ThreadPool(int num_threads = 0,
                      size_t minimum_worker_stack_size = 0) {
    if (num_threads <= 0) {
      num_threads = static_cast<int>(std::thread::hardware_concurrency());
      if (num_threads <= 0) num_threads = 1;
    }
    workers_.reserve(static_cast<size_t>(num_threads));
#if !defined(_WIN32)
    if (minimum_worker_stack_size != 0) {
      start_pthread_workers(num_threads, minimum_worker_stack_size);
      return;
    }
#else
    (void)minimum_worker_stack_size;
#endif
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
#if !defined(_WIN32)
    for (pthread_t worker : pthread_workers_) pthread_join(worker, nullptr);
#endif
  }

  ThreadPool(const ThreadPool&) = delete;
  ThreadPool& operator=(const ThreadPool&) = delete;

  /// Number of worker threads.
  int size() const {
#if !defined(_WIN32)
    return static_cast<int>(workers_.size() + pthread_workers_.size());
#else
    return static_cast<int>(workers_.size());
#endif
  }

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

    std::exception_ptr first_error;
    // Chunk 0 runs on the caller.  Preserve its exception but still join every
    // submitted task before local captures (`body`, chunk state) are destroyed.
    const size_t end0 = count / static_cast<size_t>(num_tasks);
    try {
      body(0, end0);
    } catch (...) {
      first_error = std::current_exception();
    }

    for (auto& f : futures) {
      try {
        f.get();
      } catch (...) {
        if (!first_error) first_error = std::current_exception();
      }
    }
    if (first_error) std::rethrow_exception(first_error);
  }

  /// Execute one work item at a time using an atomic cursor.  This is better
  /// than static chunking when item costs vary substantially, e.g. contingency
  /// searches or Monte Carlo cache-miss states.
  void parallel_for_dynamic(size_t count,
                            const std::function<void(size_t)>& body,
                            int num_tasks = 0) {
    if (count == 0) return;
    if (num_tasks <= 0) num_tasks = size();
    num_tasks = std::min(num_tasks, static_cast<int>(count));
    if (num_tasks <= 1) {
      for (size_t i = 0; i < count; ++i) body(i);
      return;
    }

    std::atomic<size_t> next{0};
    std::atomic<bool> cancel{false};
    auto run_worker = [&]() {
      while (!cancel.load(std::memory_order_relaxed)) {
        const size_t i = next.fetch_add(1, std::memory_order_relaxed);
        if (i >= count) break;
        try {
          body(i);
        } catch (...) {
          cancel.store(true, std::memory_order_relaxed);
          throw;
        }
      }
    };

    std::vector<std::future<void>> futures;
    futures.reserve(static_cast<size_t>(num_tasks - 1));
    for (int t = 1; t < num_tasks; ++t) {
      futures.push_back(submit(run_worker));
    }
    std::exception_ptr first_error;
    try {
      run_worker();
    } catch (...) {
      first_error = std::current_exception();
    }
    for (auto& f : futures) {
      try {
        f.get();
      } catch (...) {
        if (!first_error) first_error = std::current_exception();
      }
    }
    if (first_error) std::rethrow_exception(first_error);
  }

  /// Process-wide singleton (lazy, thread-safe).
  static ThreadPool& global() {
#if defined(_WIN32)
    // Keep Windows workers alive until process termination. Joining them from
    // static teardown can run after a linked runtime has destroyed its TLS
    // state (MinGW manifests this in _tinycthread_tss_callback).
    static ThreadPool* const pool = new ThreadPool();
    return *pool;
#else
    static ThreadPool pool;
    return pool;
#endif
  }

 private:
#if !defined(_WIN32)
  static void* pthread_worker_entry(void* pool) {
    static_cast<ThreadPool*>(pool)->worker_loop();
    return nullptr;
  }

  void start_pthread_workers(int num_threads, size_t minimum_stack_size) {
    pthread_attr_t attr;
    int error = pthread_attr_init(&attr);
    if (error != 0)
      throw std::system_error(error, std::generic_category(),
                              "pthread_attr_init");

    size_t default_stack_size = 0;
    error = pthread_attr_getstacksize(&attr, &default_stack_size);
    if (error == 0 && minimum_stack_size > default_stack_size)
      error = pthread_attr_setstacksize(&attr, minimum_stack_size);
    if (error != 0) {
      pthread_attr_destroy(&attr);
      throw std::system_error(error, std::generic_category(),
                              "pthread worker stack size");
    }

    pthread_workers_.reserve(static_cast<size_t>(num_threads));
    for (int i = 0; i < num_threads; ++i) {
      pthread_t worker;
      error = pthread_create(&worker, &attr, &ThreadPool::pthread_worker_entry,
                             this);
      if (error != 0) {
        {
          std::lock_guard<std::mutex> lk(mtx_);
          shutdown_ = true;
        }
        cv_.notify_all();
        for (pthread_t started : pthread_workers_)
          pthread_join(started, nullptr);
        pthread_workers_.clear();
        pthread_attr_destroy(&attr);
        throw std::system_error(error, std::generic_category(),
                                "pthread_create");
      }
      pthread_workers_.push_back(worker);
    }
    pthread_attr_destroy(&attr);
  }
#endif

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
#if !defined(_WIN32)
  std::vector<pthread_t> pthread_workers_;
#endif
  std::queue<std::function<void()>> tasks_;
  std::mutex mtx_;
  std::condition_variable cv_;
  bool shutdown_{false};
};

}  // namespace hacdcpf::util
