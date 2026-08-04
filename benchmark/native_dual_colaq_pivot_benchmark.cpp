/// native_dual_colaq_pivot_benchmark.cpp
///
/// S2 independent-kernel harness (roadmap §6/§13.3) for the col_aq consumer.
///
/// The entering-column FTRAN result `direction` (col_aq = B^-1 a_q) is produced
/// densely by the factor backend (HFactor update_vec_aq.array).  native_dual
/// reads it for exactly one scalar per pivot — the pivotal element
/// `column_pivot = direction.at(leaving.row)` (solver.cpp:651).  Because
/// ftran_indexed leaves the IndexedVector lookup table empty, and the FTRAN
/// support is far above the 8-element linear-scan threshold, `.at()` builds an
/// O(support) hash table *for that single lookup, every pivot*.  All other
/// col_aq consumers iterate the packed support once (no dense read needed).
///
/// The S2 resident view reads the factor's dense update_vec_aq backing at the
/// leaving row directly — O(1), no hash build.  Unlike the row_ep dot-scan this
/// cost is NOT amortized over candidates, so it is a per-pivot fixed saving.
///
/// This harness replicates IndexedVector::at()/build_lookup() exactly and
/// measures the current hash-build-then-lookup against the resident O(1) read,
/// over production-distribution FTRAN supports.  Bit-identical value first.
///
/// Usage: native_dual_colaq_pivot_benchmark [m] [support] [pivots] [samples]

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <vector>

namespace {

// Exact replica of IndexedVector::build_lookup / at from
// native_dual/model.hpp, so the measured hash-build cost is the production one.
struct IndexedVector {
  std::vector<int> index;
  std::vector<double> value;
  mutable std::vector<int> lookup_slot;

  void reset() {
    index.clear();
    value.clear();
    lookup_slot.clear();
  }
  void build_lookup() const {
    std::size_t slot_count = 4;
    while (slot_count < index.size() * 2) slot_count *= 2;
    lookup_slot.assign(slot_count, -1);
    const std::size_t mask = slot_count - 1;
    for (std::size_t k = 0; k < index.size(); ++k) {
      std::size_t slot =
          (static_cast<std::uint32_t>(index[k]) * 0x9e3779b1u) & mask;
      while (lookup_slot[slot] >= 0) slot = (slot + 1) & mask;
      lookup_slot[slot] = static_cast<int>(k);
    }
  }
  double at(int target) const {
    if (lookup_slot.empty()) {
      if (index.size() < 8) {
        for (std::size_t k = 0; k < index.size(); ++k) {
          if (index[k] == target) return value[k];
        }
        return 0.0;
      }
      build_lookup();
    }
    const std::size_t mask = lookup_slot.size() - 1;
    std::size_t slot =
        (static_cast<std::uint32_t>(target) * 0x9e3779b1u) & mask;
    for (;;) {
      const int position = lookup_slot[slot];
      if (position < 0) return 0.0;
      if (index[static_cast<std::size_t>(position)] == target)
        return value[static_cast<std::size_t>(position)];
      slot = (slot + 1) & mask;
    }
  }
};

struct Pivot {
  IndexedVector direction;     // packed FTRAN support
  std::vector<double> dense;   // factor-resident dense image (length m)
  int leaving_row{0};
};

// A cohort of independent pivots at a fixed FTRAN support size.
std::vector<Pivot> make_pivots(int m, int support, int pivots) {
  std::vector<Pivot> cohort(static_cast<std::size_t>(pivots));
  std::mt19937_64 random(0x434F4C41'51000000ULL);  // "COLAQ"
  std::uniform_real_distribution<double> unit(-1.0, 1.0);
  for (Pivot& p : cohort) {
    p.dense.assign(static_cast<std::size_t>(m), 0.0);
    std::vector<int> rows(static_cast<std::size_t>(m));
    for (int i = 0; i < m; ++i) rows[static_cast<std::size_t>(i)] = i;
    std::shuffle(rows.begin(), rows.end(), random);
    const int s = std::min(support, m);
    p.direction.index.resize(static_cast<std::size_t>(s));
    p.direction.value.resize(static_cast<std::size_t>(s));
    for (int k = 0; k < s; ++k) {
      const int row = rows[static_cast<std::size_t>(k)];
      const double v = unit(random);
      p.direction.index[static_cast<std::size_t>(k)] = row;
      p.direction.value[static_cast<std::size_t>(k)] = v;
      p.dense[static_cast<std::size_t>(row)] = v;
    }
    p.leaving_row = p.direction.index[static_cast<std::size_t>(s / 2)];
  }
  return cohort;
}

// Current path: fresh direction each pivot (empty lookup) -> at() builds the
// hash for the single column_pivot lookup.
double run_current(std::vector<Pivot>& cohort) {
  double checksum = 0.0;
  for (Pivot& p : cohort) {
    p.direction.lookup_slot.clear();  // fresh from ftran_indexed (empty)
    checksum += p.direction.at(p.leaving_row);
  }
  return checksum;
}

// S2 resident path: O(1) dense read from the factor-resident image.
double run_resident(const std::vector<Pivot>& cohort) {
  double checksum = 0.0;
  for (const Pivot& p : cohort) {
    checksum += p.dense[static_cast<std::size_t>(p.leaving_row)];
  }
  return checksum;
}

bool bit_identical(double a, double b) {
  return std::memcmp(&a, &b, sizeof(double)) == 0;
}

template <typename F>
double measure(F&& f, int repeats, double& sink) {
  const auto start = std::chrono::steady_clock::now();
  for (int r = 0; r < repeats; ++r) sink += f();
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - start)
      .count();
}

double median(std::vector<double> s) {
  std::sort(s.begin(), s.end());
  return s[s.size() / 2];
}

}  // namespace

int main(int argc, char** argv) {
  int m = 1759;         // d2q06c m
  int support = 300;    // representative FTRAN support (>> 8 threshold)
  int pivots = 5609;    // d2q06c pivot count
  int samples = 7;
  if (argc > 1) m = std::max(8, std::atoi(argv[1]));
  if (argc > 2) support = std::max(1, std::atoi(argv[2]));
  if (argc > 3) pivots = std::max(1, std::atoi(argv[3]));
  if (argc > 4) samples = std::max(3, std::atoi(argv[4]) | 1);

  std::vector<Pivot> cohort = make_pivots(m, support, pivots);

  const double current_checksum = run_current(cohort);
  const double resident_checksum = run_resident(cohort);
  if (!bit_identical(current_checksum, resident_checksum)) {
    std::fprintf(stderr,
                 "FAIL: paths disagree (current=%.17g resident=%.17g)\n",
                 current_checksum, resident_checksum);
    return 2;
  }

  std::vector<double> current_samples;
  std::vector<double> resident_samples;
  double sink = 0.0;
  for (int s = 0; s < samples; ++s) {
    if ((s & 1) == 0) {
      current_samples.push_back(measure([&] { return run_current(cohort); }, 1, sink));
      resident_samples.push_back(measure([&] { return run_resident(cohort); }, 1, sink));
    } else {
      resident_samples.push_back(measure([&] { return run_resident(cohort); }, 1, sink));
      current_samples.push_back(measure([&] { return run_current(cohort); }, 1, sink));
    }
  }

  const double current_ms = median(current_samples) * 1e3;
  const double resident_ms = median(resident_samples) * 1e3;
  const double per_pivot_us_saved =
      (current_ms - resident_ms) / pivots * 1e3;

  std::printf("S2 col_aq column_pivot resident-view independent kernel\n");
  std::printf("  cohort: m=%d support=%d pivots=%d\n", m, support, pivots);
  std::printf("  correctness: bit-identical column_pivot (%.17g)\n",
              current_checksum);
  std::printf("  cohort time: current(at+hash)=%.4f ms  resident(dense)=%.4f ms  ratio=%.2fx\n",
              current_ms, resident_ms,
              resident_ms > 0.0 ? current_ms / resident_ms : 0.0);
  std::printf("  per-pivot hash-build eliminated: %.4f us/pivot\n",
              per_pivot_us_saved);
  std::printf("  sink=%.3e\n", sink);
  return 0;
}
