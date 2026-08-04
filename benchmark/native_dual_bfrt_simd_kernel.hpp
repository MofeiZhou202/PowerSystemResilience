#pragma once

#include <cstddef>
#include <cstdint>

namespace mipsolvers::benchmark::bfrt_simd {

struct InputView {
  const int* column{nullptr};
  const double* pivot{nullptr};
  const std::uint8_t* basic{nullptr};
  const std::int8_t* move{nullptr};
  const double* lower{nullptr};
  const double* upper{nullptr};
  const double* error_coefficient{nullptr};
  std::size_t count{0};
  int leaving_side{1};
  double row_ep_max_abs{1.0};
  double stable_pivot_tolerance{1e-9};
};

struct ReferenceEvaluation {
  int column{-1};
  int direction{0};
  double pivot{0.0};
  double signed_alpha{0.0};
  double range{0.0};
  double cheap_error{0.0};
  bool active{false};
  bool positive{false};
  bool stability_blocked{false};
  bool prefiltered{false};
  bool cheap_rejected{false};
  bool cheap_certified{false};
  bool needs_exact{false};
};

struct CompactOutputView {
  double* signed_alpha{nullptr};
  double* range{nullptr};
  std::uint8_t* flags{nullptr};
};

enum ClassificationFlag : std::uint8_t {
  kActive = 1U << 0,
  kPositive = 1U << 1,
  kStabilityBlocked = 1U << 2,
  kPrefiltered = 1U << 3,
  kCheapRejected = 1U << 4,
  kCheapCertified = 1U << 5,
  kNeedsExact = 1U << 6,
};

struct WorkAudit {
  std::uint64_t input_bytes{0};
  std::uint64_t output_bytes{0};
  std::uint64_t modeled_machine_instructions{0};
  std::uint64_t data_dependent_branches{0};
  std::uint64_t loop_branches{0};
  bool machine_instruction_model_valid{false};
};

struct ClassificationCounts {
  std::uint64_t basic_inactive{0};
  std::uint64_t move_zero_inactive{0};
  std::uint64_t positive_prefiltered{0};
  std::uint64_t nonpositive_cheap_rejected{0};
  std::uint64_t nonpositive_needs_exact{0};
  std::uint64_t positive_cheap_certified{0};
  std::uint64_t positive_needs_exact{0};

  std::uint64_t total() const;
  std::uint64_t needs_exact() const;
};

std::size_t block_count(std::size_t count);
ClassificationCounts classification_counts(const InputView& input);
WorkAudit reference_work_audit(const InputView& input);
WorkAudit neon_work_audit(const InputView& input);

extern "C" std::uint64_t bfrt_prefilter_reference(
    const InputView* input, ReferenceEvaluation* output);
extern "C" std::uint64_t bfrt_prefilter_neon(
    const InputView* input, CompactOutputView output);

}  // namespace mipsolvers::benchmark::bfrt_simd
