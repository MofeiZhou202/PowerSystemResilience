#pragma once

#include <cstdint>

// Project-level adapter for the vendored DSS C-API package.
// The current release bundle exposes the ctx_* API as altdss/capi/dss_ctx.h.
// Older dss_capi packages exposed the same API as dss_capi_ctx.h.
#if __has_include(<dss_capi_ctx.h>)
#include <dss_capi_ctx.h>
#elif __has_include(<altdss/capi/dss_ctx.h>)
#include <altdss/capi/dss_ctx.h>
#else
#error "DSS C-API header not found. Expected vendored third_party/dss_capi/dss_capi."
#endif

namespace hacdcpf::io {

class ScopedDSSFloatingPointEnv {
 public:
  ScopedDSSFloatingPointEnv() {
#if defined(__aarch64__) || defined(__arm64__)
    asm volatile("mrs %0, fpcr" : "=r"(fpcr_));
    asm volatile("mrs %0, fpsr" : "=r"(fpsr_));

    // DSS C-API expects IEEE floating-point exceptions to be masked.  Some GUI
    // builds arrive here with ARM64 FP traps enabled, which makes OpenDSS abort
    // on benign internal operations such as temporary division by zero during
    // circuit creation.
    const std::uint64_t masked_fpcr = fpcr_ & ~kArm64ExceptionEnableMask;
    asm volatile("msr fpcr, %0" : : "r"(masked_fpcr));
    const std::uint64_t clear_fpsr = 0;
    asm volatile("msr fpsr, %0" : : "r"(clear_fpsr));
    active_ = true;
#endif
  }

  ~ScopedDSSFloatingPointEnv() {
#if defined(__aarch64__) || defined(__arm64__)
    if (active_) {
      asm volatile("msr fpsr, %0" : : "r"(fpsr_));
      asm volatile("msr fpcr, %0" : : "r"(fpcr_));
    }
#endif
  }

  ScopedDSSFloatingPointEnv(const ScopedDSSFloatingPointEnv&) = delete;
  ScopedDSSFloatingPointEnv& operator=(const ScopedDSSFloatingPointEnv&) = delete;

 private:
#if defined(__aarch64__) || defined(__arm64__)
  static constexpr std::uint64_t kArm64ExceptionEnableMask = 0x1f00;
  std::uint64_t fpcr_{0};
  std::uint64_t fpsr_{0};
  bool active_{false};
#endif
};

}  // namespace hacdcpf::io
