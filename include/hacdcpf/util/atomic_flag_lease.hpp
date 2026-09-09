#pragma once

#include <atomic>

namespace hacdcpf::util {

// A request may release only the flag it successfully acquired.
class AtomicFlagLease {
 public:
  explicit AtomicFlagLease(std::atomic<bool>& flag) noexcept : flag_(flag) {}
  ~AtomicFlagLease() { release(); }
  AtomicFlagLease(const AtomicFlagLease&) = delete;
  AtomicFlagLease& operator=(const AtomicFlagLease&) = delete;

  bool try_acquire() noexcept {
    if (owned_) return true;
    bool expected = false;
    owned_ = flag_.compare_exchange_strong(expected, true);
    return owned_;
  }

  void release() noexcept {
    if (owned_) {
      owned_ = false;
      flag_.store(false);
    }
  }

 private:
  std::atomic<bool>& flag_;
  bool owned_{false};
};

}  // namespace hacdcpf::util
