#pragma once

#include <bit>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string_view>

namespace mipsolvers::engine::native_dual {

// Observational committed-transaction projection, never used in pricing.
// See general_solver_performance_program_2026-09-13.md, R3 release isolation.
// Binary64 bits avoid locale, decimal rounding, and signed-zero ambiguity.
inline void emit_pivot_trace(int phase, int row, int entering, int leaving,
                             double pivot, double primal, double dual) {
  const char* flag = std::getenv("MIPSOLVERS_LP_PIVOT_TRACE");
  if (!flag || std::string_view(flag) != "1") return;
  if (std::fprintf(stderr,
      "LP-PIVOT 2 %d %d %d %d %016llx %016llx %016llx\n",
      phase, row, entering, leaving,
      static_cast<unsigned long long>(std::bit_cast<std::uint64_t>(pivot)),
      static_cast<unsigned long long>(std::bit_cast<std::uint64_t>(primal)),
      static_cast<unsigned long long>(std::bit_cast<std::uint64_t>(dual))) < 0) {
    std::abort();
  }
}

}  // namespace mipsolvers::engine::native_dual
