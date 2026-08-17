#include "native_dual_bfrt_simd_kernel.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <random>
#include <vector>

namespace simd = mipsolvers::benchmark::bfrt_simd;

namespace {

struct Fixture {
  std::vector<int> column;
  std::vector<double> pivot;
  std::vector<std::uint8_t> basic;
  std::vector<std::int8_t> move;
  std::vector<double> lower;
  std::vector<double> upper;
  std::vector<double> error_coefficient;
  std::vector<simd::ReferenceEvaluation> reference;
  std::vector<double> alpha;
  std::vector<double> range;
  std::vector<std::uint8_t> flags;
  int leaving_side{1};
  double row_ep_max_abs{1.25};
  double stable_pivot_tolerance{3e-8};

  simd::InputView input() const {
    return {column.data(), pivot.data(), basic.data(), move.data(),
            lower.data(), upper.data(), error_coefficient.data(),
            column.size(), leaving_side, row_ep_max_abs,
            stable_pivot_tolerance};
  }
  simd::CompactOutputView output() {
    return {alpha.data(), range.data(), flags.data()};
  }
};

Fixture make_storage(std::size_t count) {
  Fixture fixture;
  fixture.column.resize(count);
  fixture.pivot.assign(count, 1.0);
  fixture.basic.assign(count, 0);
  fixture.move.assign(count, 1);
  fixture.lower.assign(count, 0.0);
  fixture.upper.assign(count, 1.0);
  fixture.error_coefficient.assign(count, 0.0);
  fixture.reference.resize(count);
  fixture.alpha.resize(count);
  fixture.range.resize(count);
  fixture.flags.resize(count);
  for (std::size_t i = 0; i < count; ++i) {
    fixture.column[i] = static_cast<int>(i);
  }
  return fixture;
}

Fixture make_fixture(std::size_t count) {
  Fixture fixture = make_storage(count);

  std::mt19937_64 random(0x42565254ULL);
  std::vector<int> permutation(count);
  for (std::size_t i = 0; i < count; ++i) permutation[i] = static_cast<int>(i);
  std::shuffle(permutation.begin(), permutation.end(), random);
  for (std::size_t position = 0; position < count; ++position) {
    const int column = permutation[position];
    fixture.column[position] = column;
    const std::size_t index = static_cast<std::size_t>(column);
    fixture.basic[index] = position % 19 == 0 ? 1 : 0;
    fixture.move[index] = position % 23 == 0
                              ? 0
                              : (position % 3 == 0 ? -1 : 1);
    const double magnitude =
        position % 17 == 0
            ? 1e-9
            : 1e-5 + static_cast<double>(position % 4093) / 4093.0;
    fixture.pivot[position] = position % 5 == 0 ? -magnitude : magnitude;
    fixture.lower[index] = -static_cast<double>(position % 7);
    fixture.upper[index] = fixture.lower[index] + 1.0 + position % 31;
    fixture.error_coefficient[index] =
        position % 13 == 0 ? 2.0 * magnitude : 0.25 * magnitude;
  }
  return fixture;
}

Fixture make_boundary_fixture() {
  Fixture fixture = make_storage(18);
  const double stable = fixture.stable_pivot_tolerance;
  fixture.basic[0] = 1;
  fixture.move[1] = 0;
  fixture.move[2] = -1;
  fixture.pivot[2] = 0.0;
  fixture.pivot[3] = stable;
  fixture.pivot[4] = std::nextafter(stable, 1.0);
  fixture.error_coefficient[4] = 1.0;
  fixture.pivot[5] = 1.0;
  fixture.error_coefficient[5] = 0.8;
  fixture.pivot[6] = 1.0;
  fixture.error_coefficient[6] = std::nextafter(0.8, 0.0);
  fixture.pivot[7] = -1.0;
  fixture.error_coefficient[7] = 0.8;
  fixture.pivot[8] = -1.0;
  fixture.error_coefficient[8] = std::nextafter(0.8, 1.0);
  fixture.pivot[9] = std::numeric_limits<double>::quiet_NaN();
  fixture.pivot[10] = std::numeric_limits<double>::infinity();
  fixture.pivot[11] = -std::numeric_limits<double>::infinity();
  fixture.upper[12] = std::numeric_limits<double>::infinity();
  fixture.lower[13] = std::numeric_limits<double>::infinity();
  fixture.upper[13] = std::numeric_limits<double>::infinity();
  fixture.error_coefficient[14] = std::numeric_limits<double>::infinity();
  fixture.error_coefficient[15] =
      std::numeric_limits<double>::quiet_NaN();
  fixture.pivot[16] = -1.0;
  fixture.error_coefficient[16] = std::numeric_limits<double>::infinity();
  fixture.pivot[17] = -1.0;
  fixture.error_coefficient[17] =
      std::numeric_limits<double>::quiet_NaN();
  return fixture;
}

bool same_fp(double lhs, double rhs) {
  if (std::isnan(lhs) && std::isnan(rhs)) return true;
  return std::memcmp(&lhs, &rhs, sizeof(double)) == 0;
}

bool validate(Fixture& fixture) {
  const simd::InputView input = fixture.input();
  simd::bfrt_prefilter_reference(&input, fixture.reference.data());
  simd::bfrt_prefilter_neon(&input, fixture.output());
  for (std::size_t position = 0; position < input.count; ++position) {
    const simd::ReferenceEvaluation& expected = fixture.reference[position];
    const std::uint8_t expected_flags = static_cast<std::uint8_t>(
        (expected.active ? simd::kActive : 0) |
        (expected.positive ? simd::kPositive : 0) |
        (expected.stability_blocked ? simd::kStabilityBlocked : 0) |
        (expected.prefiltered ? simd::kPrefiltered : 0) |
        (expected.cheap_rejected ? simd::kCheapRejected : 0) |
        (expected.cheap_certified ? simd::kCheapCertified : 0) |
        (expected.needs_exact ? simd::kNeedsExact : 0));
    if (fixture.flags[position] != expected_flags ||
        (expected.active &&
         (!same_fp(fixture.alpha[position], expected.signed_alpha) ||
          !same_fp(fixture.range[position], expected.range)))) {
      std::fprintf(stderr, "classification mismatch at position %zu\n",
                   position);
      return false;
    }
  }
  return true;
}

bool validate_deterministic_fixtures() {
  for (std::size_t count = 0; count <= 9; ++count) {
    Fixture fixture = make_fixture(count);
    if (!validate(fixture)) return false;
  }
  for (std::size_t count : {std::size_t{2}, std::size_t{7}}) {
    Fixture all_basic = make_storage(count);
    std::fill(all_basic.basic.begin(), all_basic.basic.end(), 1);
    if (!validate(all_basic)) return false;
    Fixture all_fixed = make_storage(count);
    std::fill(all_fixed.move.begin(), all_fixed.move.end(), 0);
    if (!validate(all_fixed)) return false;
  }
  Fixture boundary = make_boundary_fixture();
  if (!validate(boundary)) return false;
  const std::array<std::uint8_t, 18> expected_flags = {
      0,
      0,
      simd::kActive | simd::kCheapRejected,
      simd::kActive | simd::kPositive | simd::kStabilityBlocked |
          simd::kPrefiltered,
      simd::kActive | simd::kPositive | simd::kNeedsExact,
      simd::kActive | simd::kPositive | simd::kNeedsExact,
      simd::kActive | simd::kPositive | simd::kCheapCertified,
      simd::kActive | simd::kCheapRejected,
      simd::kActive | simd::kNeedsExact,
      simd::kActive | simd::kCheapRejected,
      simd::kActive | simd::kPositive | simd::kCheapCertified,
      simd::kActive | simd::kCheapRejected,
      simd::kActive | simd::kPositive | simd::kCheapCertified,
      simd::kActive | simd::kPositive | simd::kCheapCertified,
      simd::kActive | simd::kPositive | simd::kNeedsExact,
      simd::kActive | simd::kPositive | simd::kNeedsExact,
      simd::kActive | simd::kNeedsExact,
      simd::kActive | simd::kCheapRejected,
  };
  if (!std::equal(boundary.flags.begin(), boundary.flags.end(),
                  expected_flags.begin())) {
    std::fprintf(stderr, "boundary fixture semantic expectation failed\n");
    return false;
  }
  boundary.leaving_side = -1;
  if (!validate(boundary)) return false;
  boundary.leaving_side = 1;
  boundary.row_ep_max_abs = std::numeric_limits<double>::infinity();
  if (!validate(boundary)) return false;
  boundary.row_ep_max_abs = std::numeric_limits<double>::quiet_NaN();
  if (!validate(boundary)) return false;
  boundary.row_ep_max_abs = 1.25;
  boundary.stable_pivot_tolerance =
      std::numeric_limits<double>::quiet_NaN();
  return validate(boundary);
}

template <typename Function>
double measure(Function&& function, int repeats, std::uint64_t& checksum) {
  const auto start = std::chrono::steady_clock::now();
  for (int repeat = 0; repeat < repeats; ++repeat) checksum += function();
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - start)
      .count();
}

double median(std::vector<double> samples) {
  std::sort(samples.begin(), samples.end());
  return samples[samples.size() / 2];
}

}  // namespace

int main(int argc, char** argv) {
  std::size_t count = 1U << 20;
  int repeats = 20;
  int sample_count = 7;
  if (argc > 1) count = std::max<std::size_t>(3, std::strtoull(argv[1], nullptr, 10));
  if (argc > 2) repeats = std::max(1, std::atoi(argv[2]));
  if (argc > 3) sample_count = std::max(3, std::atoi(argv[3]) | 1);
  if (!validate_deterministic_fixtures()) return 2;
  Fixture fixture = make_fixture(count);
  if (!validate(fixture)) return 2;

  const simd::InputView input = fixture.input();
  std::uint64_t checksum = 0;
  const auto run_reference = [&] {
    return simd::bfrt_prefilter_reference(&input, fixture.reference.data());
  };
  const auto run_neon = [&] {
    return simd::bfrt_prefilter_neon(&input, fixture.output());
  };
  checksum += run_reference();
  checksum += run_neon();
  std::vector<double> reference_samples;
  std::vector<double> neon_samples;
  reference_samples.reserve(static_cast<std::size_t>(sample_count));
  neon_samples.reserve(static_cast<std::size_t>(sample_count));
  for (int sample = 0; sample < sample_count; ++sample) {
    if ((sample & 1) == 0) {
      reference_samples.push_back(measure(run_reference, repeats, checksum));
      neon_samples.push_back(measure(run_neon, repeats, checksum));
    } else {
      neon_samples.push_back(measure(run_neon, repeats, checksum));
      reference_samples.push_back(measure(run_reference, repeats, checksum));
    }
  }
  const double reference_sec = median(reference_samples);
  const double neon_sec = median(neon_samples);
  const simd::WorkAudit reference = simd::reference_work_audit(input);
  const simd::WorkAudit neon = simd::neon_work_audit(input);
  const simd::ClassificationCounts counts = simd::classification_counts(input);
  const std::uint64_t reference_bytes =
      reference.input_bytes + reference.output_bytes;
  const std::uint64_t neon_bytes = neon.input_bytes + neon.output_bytes;

  std::printf(
      "BFRT SIMD prefilter benchmark: count=%zu repeats_per_sample=%d "
      "samples=%d\n",
      count, repeats, sample_count);
#if defined(__aarch64__)
  constexpr const char* kArchitecture = "aarch64";
#else
  constexpr const char* kArchitecture = "non-aarch64";
#endif
  // __VERSION__ is GCC/Clang-only; MSVC reports the compiler version through
  // _MSC_FULL_VER instead.
#if defined(_MSC_VER)
  std::printf("architecture=%s compiler=MSVC %d\n", kArchitecture,
              _MSC_FULL_VER);
#elif defined(__VERSION__)
  std::printf("architecture=%s compiler=%s\n", kArchitecture, __VERSION__);
#else
  std::printf("architecture=%s compiler=unknown\n", kArchitecture);
#endif
  std::printf("correctness=exact checksum=%llu reference_record_bytes=%zu\n",
              static_cast<unsigned long long>(checksum),
              sizeof(simd::ReferenceEvaluation));
  std::printf("reference_sec=%.9f neon_sec=%.9f speedup=%.6f\n",
              reference_sec, neon_sec, reference_sec / neon_sec);
  std::printf(
      "modeled_array_bytes: reference=%llu neon=%llu reduction=%.6f "
      "input=%llu/%llu output=%llu/%llu\n",
              static_cast<unsigned long long>(reference_bytes),
              static_cast<unsigned long long>(neon_bytes),
              1.0 - static_cast<double>(neon_bytes) / reference_bytes,
              static_cast<unsigned long long>(reference.input_bytes),
              static_cast<unsigned long long>(neon.input_bytes),
              static_cast<unsigned long long>(reference.output_bytes),
              static_cast<unsigned long long>(neon.output_bytes));
  std::printf(
      "categories: basic=%llu move_zero=%llu positive_prefiltered=%llu "
      "nonpositive_rejected=%llu nonpositive_exact=%llu "
      "positive_certified=%llu positive_exact=%llu total=%llu\n",
      static_cast<unsigned long long>(counts.basic_inactive),
      static_cast<unsigned long long>(counts.move_zero_inactive),
      static_cast<unsigned long long>(counts.positive_prefiltered),
      static_cast<unsigned long long>(counts.nonpositive_cheap_rejected),
      static_cast<unsigned long long>(counts.nonpositive_needs_exact),
      static_cast<unsigned long long>(counts.positive_cheap_certified),
      static_cast<unsigned long long>(counts.positive_needs_exact),
      static_cast<unsigned long long>(counts.total()));
  if (reference.machine_instruction_model_valid &&
      neon.machine_instruction_model_valid) {
    std::printf(
        "modeled_aarch64_machine_instructions: reference=%llu neon=%llu "
        "reduction=%.6f\n",
        static_cast<unsigned long long>(
            reference.modeled_machine_instructions),
        static_cast<unsigned long long>(neon.modeled_machine_instructions),
        1.0 - static_cast<double>(neon.modeled_machine_instructions) /
                  reference.modeled_machine_instructions);
  } else {
    std::printf("modeled_machine_instructions=UNAVAILABLE\n");
  }
  std::printf("data_branches: reference=%llu neon=%llu loop_branches=%llu/%llu\n",
              static_cast<unsigned long long>(reference.data_dependent_branches),
              static_cast<unsigned long long>(neon.data_dependent_branches),
              static_cast<unsigned long long>(reference.loop_branches),
              static_cast<unsigned long long>(neon.loop_branches));

  const bool instruction_model_available =
      reference.machine_instruction_model_valid &&
      neon.machine_instruction_model_valid;
  const bool accepted = instruction_model_available &&
                        counts.total() == count &&
                        reference_sec > neon_sec && neon_bytes < reference_bytes &&
                        neon.modeled_machine_instructions <
                            reference.modeled_machine_instructions &&
                        neon.data_dependent_branches <
                            reference.data_dependent_branches;
  if (!instruction_model_available) {
    std::printf("gate=SKIP (requires audited Apple-Clang-21 AArch64 even-count body)\n");
    return 0;
  }
  std::printf("gate=%s\n", accepted ? "PASS" : "FAIL");
  return accepted ? 0 : 3;
}
