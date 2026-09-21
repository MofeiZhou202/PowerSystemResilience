#include "native_dual_bfrt_simd_kernel.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

#if defined(__aarch64__)
#include <arm_neon.h>
#endif

namespace mipsolvers::benchmark::bfrt_simd {
namespace {

#if defined(__clang__) || defined(__GNUC__)
#define MIPSOLVERS_NOINLINE __attribute__((noinline))
#else
#define MIPSOLVERS_NOINLINE
#endif

#if defined(__aarch64__) && defined(__APPLE__) && defined(__clang__) && \
    __clang_major__ == 21
#define MIPSOLVERS_AUDITED_BFRT_ASSEMBLY 1
#else
#define MIPSOLVERS_AUDITED_BFRT_ASSEMBLY 0
#endif

#if MIPSOLVERS_AUDITED_BFRT_ASSEMBLY
static_assert(sizeof(ReferenceEvaluation) == 48,
              "BFRT instruction model requires a new disassembly audit");
#endif

std::uint8_t reference_flags(const ReferenceEvaluation& evaluation) {
  return static_cast<std::uint8_t>(
      (evaluation.active ? kActive : 0) |
      (evaluation.positive ? kPositive : 0) |
      (evaluation.stability_blocked ? kStabilityBlocked : 0) |
      (evaluation.prefiltered ? kPrefiltered : 0) |
      (evaluation.cheap_rejected ? kCheapRejected : 0) |
      (evaluation.cheap_certified ? kCheapCertified : 0) |
      (evaluation.needs_exact ? kNeedsExact : 0));
}

ReferenceEvaluation classify_one(const InputView& input,
                                 std::size_t position) {
  ReferenceEvaluation evaluation;
  const int column = input.column[position];
  evaluation.column = column;
  if (input.basic[static_cast<std::size_t>(column)] != 0) return evaluation;
  const int direction = input.move[static_cast<std::size_t>(column)];
  evaluation.direction = direction;
  if (direction == 0) return evaluation;
  evaluation.active = true;
  evaluation.pivot = input.pivot[position];
  evaluation.signed_alpha =
      input.leaving_side * direction * evaluation.pivot;
  evaluation.range = input.upper[static_cast<std::size_t>(column)] -
                     input.lower[static_cast<std::size_t>(column)];
  evaluation.positive = evaluation.signed_alpha > 0.0;
  evaluation.cheap_error =
      input.error_coefficient[static_cast<std::size_t>(column)] *
      input.row_ep_max_abs;
  if (evaluation.positive &&
      !(evaluation.signed_alpha > input.stable_pivot_tolerance)) {
    evaluation.stability_blocked = true;
    evaluation.prefiltered = true;
    return evaluation;
  }
  if (!evaluation.positive) {
    if (!(evaluation.signed_alpha + evaluation.cheap_error > 0.0)) {
      evaluation.cheap_rejected = true;
    } else {
      evaluation.needs_exact = true;
    }
    return evaluation;
  }
  if (evaluation.signed_alpha > evaluation.cheap_error) {
    evaluation.cheap_certified = true;
  } else {
    evaluation.needs_exact = true;
  }
  return evaluation;
}

}  // namespace

std::size_t block_count(std::size_t count) { return (count + 1) / 2; }

std::uint64_t ClassificationCounts::total() const {
  return basic_inactive + move_zero_inactive + positive_prefiltered +
         nonpositive_cheap_rejected + nonpositive_needs_exact +
         positive_cheap_certified + positive_needs_exact;
}

std::uint64_t ClassificationCounts::needs_exact() const {
  return nonpositive_needs_exact + positive_needs_exact;
}

ClassificationCounts classification_counts(const InputView& input) {
  ClassificationCounts counts;
  for (std::size_t position = 0; position < input.count; ++position) {
    const int column = input.column[position];
    if (input.basic[static_cast<std::size_t>(column)] != 0) {
      ++counts.basic_inactive;
      continue;
    }
    if (input.move[static_cast<std::size_t>(column)] == 0) {
      ++counts.move_zero_inactive;
      continue;
    }
    const ReferenceEvaluation evaluation = classify_one(input, position);
    if (evaluation.prefiltered) {
      ++counts.positive_prefiltered;
    } else if (!evaluation.positive && evaluation.cheap_rejected) {
      ++counts.nonpositive_cheap_rejected;
    } else if (!evaluation.positive) {
      ++counts.nonpositive_needs_exact;
    } else if (evaluation.cheap_certified) {
      ++counts.positive_cheap_certified;
    } else {
      ++counts.positive_needs_exact;
    }
  }
  return counts;
}

WorkAudit reference_work_audit(const InputView& input) {
  WorkAudit audit;
  const ClassificationCounts counts = classification_counts(input);
  audit.output_bytes = input.count * sizeof(ReferenceEvaluation);
  audit.loop_branches = input.count;
  const std::uint64_t active =
      input.count - counts.basic_inactive - counts.move_zero_inactive;
  audit.input_bytes = counts.basic_inactive *
                          (sizeof(int) + sizeof(std::uint8_t)) +
                      counts.move_zero_inactive *
                          (sizeof(int) + sizeof(std::uint8_t) +
                           sizeof(std::int8_t)) +
                      active * (sizeof(int) + sizeof(std::uint8_t) +
                                sizeof(std::int8_t) + 4 * sizeof(double));
  const std::uint64_t nonpositive = counts.nonpositive_cheap_rejected +
                                    counts.nonpositive_needs_exact;
  audit.data_dependent_branches = counts.basic_inactive +
                                  2 * counts.move_zero_inactive +
                                  4 * nonpositive +
                                  4 * counts.positive_prefiltered +
                                  5 * (counts.positive_cheap_certified +
                                       counts.positive_needs_exact);
#if MIPSOLVERS_AUDITED_BFRT_ASSEMBLY
  // Apple clang 21 -O3 disassembly audit (2026-08-04). Counts include every
  // instruction from the loop header through the loop back/exit branch for
  // each mutually exclusive path. The 16-instruction prologue and two-
  // instruction return block are charged once. Re-audit these constants when
  // the compiler or source loop changes.
  audit.modeled_machine_instructions =
      18 + 37 * counts.basic_inactive + 40 * counts.move_zero_inactive +
      56 * counts.positive_prefiltered +
      56 * counts.nonpositive_cheap_rejected +
      57 * counts.nonpositive_needs_exact +
      58 * counts.positive_cheap_certified +
      58 * counts.positive_needs_exact;
  audit.machine_instruction_model_valid = true;
#endif
  return audit;
}

WorkAudit neon_work_audit(const InputView& input) {
  WorkAudit audit;
  // The vector loop gathers the same seven semantic inputs for both lanes.
  audit.input_bytes = input.count *
                      (sizeof(int) + sizeof(std::uint8_t) +
                       sizeof(std::int8_t) + 4 * sizeof(double));
  audit.output_bytes = input.count * (2 * sizeof(double) + sizeof(std::uint8_t));
  audit.data_dependent_branches = input.count & 1U ? 4 : 0;
  audit.loop_branches = input.count / 2 + (input.count & 1U);
#if MIPSOLVERS_AUDITED_BFRT_ASSEMBLY
  // The audited vector body is 80 AArch64 instructions for two lanes, with
  // one loop branch and no candidate-dependent branch. Restrict the machine-
  // instruction comparison to even counts so the scalar tail cannot be
  // mistaken for part of the fixed SIMD body.
  if ((input.count & 1U) == 0) {
    audit.modeled_machine_instructions = 24 + 80 * (input.count / 2);
    audit.machine_instruction_model_valid = true;
  }
#endif
  return audit;
}

extern "C" MIPSOLVERS_NOINLINE std::uint64_t bfrt_prefilter_reference(
    const InputView* input, ReferenceEvaluation* output) {
  std::uint64_t checksum = 0;
  for (std::size_t position = 0; position < input->count; ++position) {
    output[position] = classify_one(*input, position);
    checksum += reference_flags(output[position]);
  }
  return checksum;
}

extern "C" MIPSOLVERS_NOINLINE std::uint64_t bfrt_prefilter_neon(
    const InputView* input, CompactOutputView output) {
  std::uint64_t checksum = 0;
  std::size_t position = 0;
#if defined(__aarch64__)
  const float64x2_t zero = vdupq_n_f64(0.0);
  const float64x2_t side =
      vdupq_n_f64(static_cast<double>(input->leaving_side));
  const float64x2_t stable =
      vdupq_n_f64(input->stable_pivot_tolerance);
  const uint64x2_t all_ones = vdupq_n_u64(~std::uint64_t{0});
  for (; position + 1 < input->count; position += 2) {
    const int column0 = input->column[position];
    const int column1 = input->column[position + 1];
    const std::size_t index0 = static_cast<std::size_t>(column0);
    const std::size_t index1 = static_cast<std::size_t>(column1);
    const uint64x2_t basic = {
        static_cast<std::uint64_t>(input->basic[index0]),
        static_cast<std::uint64_t>(input->basic[index1])};
    const int64x2_t direction_integer = {
        static_cast<std::int64_t>(input->move[index0]),
        static_cast<std::int64_t>(input->move[index1])};
    const float64x2_t direction = vcvtq_f64_s64(direction_integer);
    const float64x2_t pivot = vld1q_f64(input->pivot + position);
    const float64x2_t lower = {input->lower[index0], input->lower[index1]};
    const float64x2_t upper = {input->upper[index0], input->upper[index1]};
    const float64x2_t coefficient = {
        input->error_coefficient[index0],
        input->error_coefficient[index1]};
    const float64x2_t alpha = vmulq_f64(vmulq_f64(side, direction), pivot);
    const float64x2_t range = vsubq_f64(upper, lower);
    const float64x2_t cheap_error =
        vmulq_n_f64(coefficient, input->row_ep_max_abs);

    const uint64x2_t active = vandq_u64(
        vceqq_u64(basic, vdupq_n_u64(0)),
        veorq_u64(vceqq_f64(direction, zero), all_ones));
    const uint64x2_t positive = vandq_u64(active, vcgtq_f64(alpha, zero));
    const uint64x2_t above_stable = vcgtq_f64(alpha, stable);
    const uint64x2_t prefiltered =
        vandq_u64(positive, veorq_u64(above_stable, all_ones));
    const uint64x2_t nonpositive =
        vandq_u64(active,
                  veorq_u64(vcgtq_f64(alpha, zero), all_ones));
    const uint64x2_t nonpositive_ambiguous = vandq_u64(
        nonpositive, vcgtq_f64(vaddq_f64(alpha, cheap_error), zero));
    const uint64x2_t cheap_rejected =
        vandq_u64(nonpositive,
                  veorq_u64(nonpositive_ambiguous, all_ones));
    const uint64x2_t positive_eligible =
        vandq_u64(positive, above_stable);
    const uint64x2_t cheap_certified =
        vandq_u64(positive_eligible, vcgtq_f64(alpha, cheap_error));
    const uint64x2_t needs_exact = vorrq_u64(
        nonpositive_ambiguous,
        vandq_u64(positive_eligible,
                  veorq_u64(cheap_certified, all_ones)));

    vst1q_f64(output.signed_alpha + position, alpha);
    vst1q_f64(output.range + position, range);
    uint64x2_t flags = vshrq_n_u64(active, 63);
    flags = vorrq_u64(flags, vshlq_n_u64(vshrq_n_u64(positive, 63), 1));
    const uint64x2_t prefiltered_bit = vshrq_n_u64(prefiltered, 63);
    flags = vorrq_u64(flags, vshlq_n_u64(prefiltered_bit, 2));
    flags = vorrq_u64(flags, vshlq_n_u64(prefiltered_bit, 3));
    flags = vorrq_u64(
        flags, vshlq_n_u64(vshrq_n_u64(cheap_rejected, 63), 4));
    flags = vorrq_u64(
        flags, vshlq_n_u64(vshrq_n_u64(cheap_certified, 63), 5));
    flags = vorrq_u64(flags,
                      vshlq_n_u64(vshrq_n_u64(needs_exact, 63), 6));
    const std::uint8_t flags0 =
        static_cast<std::uint8_t>(vgetq_lane_u64(flags, 0));
    const std::uint8_t flags1 =
        static_cast<std::uint8_t>(vgetq_lane_u64(flags, 1));
    const std::uint16_t packed_flags = static_cast<std::uint16_t>(
        static_cast<std::uint16_t>(flags0) |
        (static_cast<std::uint16_t>(flags1) << 8));
    std::memcpy(output.flags + position, &packed_flags,
                sizeof(packed_flags));
    checksum += flags0 + flags1;
  }
#endif
  while (position < input->count) {
    const ReferenceEvaluation evaluation = classify_one(*input, position);
    output.signed_alpha[position] = evaluation.signed_alpha;
    output.range[position] = evaluation.range;
    output.flags[position] = reference_flags(evaluation);
    checksum += output.flags[position];
    ++position;
  }
  return checksum;
}

}  // namespace mipsolvers::benchmark::bfrt_simd
