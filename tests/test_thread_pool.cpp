#include <atomic>
#include <chrono>
#include <stdexcept>
#include <thread>

#include <catch2/catch_test_macros.hpp>

#include "hacdcpf/util/thread_pool.hpp"

using namespace std::chrono_literals;

TEST_CASE("ThreadPool static parallel_for joins every worker before rethrow",
          "[thread-pool][exception-safety]") {
  hacdcpf::util::ThreadPool pool(4);
  std::atomic<int> active{0};

  CHECK_THROWS_AS(
      pool.parallel_for(
          40,
          [&](size_t begin, size_t) {
            active.fetch_add(1, std::memory_order_relaxed);
            struct Guard {
              std::atomic<int>& active;
              ~Guard() { active.fetch_sub(1, std::memory_order_relaxed); }
            } guard{active};
            if (begin == 10) throw std::runtime_error("worker failure");
            std::this_thread::sleep_for(25ms);
          },
          4),
      std::runtime_error);
  CHECK(active.load(std::memory_order_relaxed) == 0);
}

TEST_CASE("ThreadPool dynamic parallel_for cancels, joins, and rethrows",
          "[thread-pool][exception-safety]") {
  hacdcpf::util::ThreadPool pool(4);
  std::atomic<int> active{0};

  CHECK_THROWS_AS(
      pool.parallel_for_dynamic(
          100,
          [&](size_t i) {
            active.fetch_add(1, std::memory_order_relaxed);
            struct Guard {
              std::atomic<int>& active;
              ~Guard() { active.fetch_sub(1, std::memory_order_relaxed); }
            } guard{active};
            if (i == 0) throw std::runtime_error("dynamic worker failure");
            std::this_thread::sleep_for(2ms);
          },
          4),
      std::runtime_error);
  CHECK(active.load(std::memory_order_relaxed) == 0);
}

TEST_CASE("ThreadPool global instance completes work before process exit",
          "[thread-pool][global]") {
  std::atomic<int> completed{0};
  auto& pool = hacdcpf::util::ThreadPool::global();

  pool.parallel_for(
      64,
      [&](size_t begin, size_t end) {
        completed.fetch_add(static_cast<int>(end - begin),
                            std::memory_order_relaxed);
      },
      4);

  CHECK(completed.load(std::memory_order_relaxed) == 64);
}
