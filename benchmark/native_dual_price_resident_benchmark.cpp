/// native_dual_price_resident_benchmark.cpp
///
/// S2 independent-kernel harness (roadmap native_dual_simplex_system_roadmap
/// 2026-08-04, §6/§13.3) for the row_ep PRICE/dot-error-bound consumer.
///
/// The dual pivot's leaving-row BTRAN result `row_ep` is produced densely by
/// the factor backend (HFactor solve_vec_btran.array).  The current PRICE
/// certification path (`dot_error_bound` in native_dual/pricing.cpp) re-
/// materializes it into a thread-local `dense_row_ep` — an O(m) clear plus an
/// O(support) scatter — once per pivot, then reads that dense image while it
/// scans candidate columns.  S2's factor-resident view lets `dot_error_bound`
/// read the factor's dense BTRAN backing directly, so the per-pivot clear and
/// scatter disappear while every candidate dot is bit-identical.
///
/// This harness replays exactly that comparison on production-distribution
/// fixtures (support and column densities from the S1 phase baseline:
/// d2q06c row_ep ~40% of m, pivot-row ~40% of n).  It proves bit-identical
/// dot values first, then reports the analytic byte model and a wall-time
/// ratio.  It is self-contained (no solver linkage) so it isolates the data
/// motion S2 removes.
///
/// Usage:
///   native_dual_price_resident_benchmark [m] [n] [repeats] [samples]

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <random>
#include <vector>

namespace {

// Compressed-sparse-column matrix, the layout native_dual reads in
// dot_error_bound via StandardColumnMatrix::InnerIterator.
struct Csc {
  int rows{0};
  int cols{0};
  std::vector<int> col_start;       // size cols+1
  std::vector<int> row_index;       // size nnz
  std::vector<double> value;        // size nnz
};

struct Fixture {
  Csc a;
  std::vector<int> row_ep_index;    // packed support of row_ep
  std::vector<double> row_ep_value; // parallel values
  std::vector<double> resident;     // factor-resident dense row_ep (length m)
  int candidate_count{0};           // columns scanned per "pivot"
};

// Production-distribution fixture: a fraction `col_density` of each column's
// entries are nonzero, and row_ep occupies `rowep_density` of the m rows.
Fixture make_fixture(int m, int n, double col_density, double rowep_density) {
  Fixture fixture;
  fixture.a.rows = m;
  fixture.a.cols = n;
  fixture.a.col_start.reserve(static_cast<std::size_t>(n) + 1);
  std::mt19937_64 random(0x50524943'45524553ULL);  // "PRICERES"
  std::uniform_real_distribution<double> unit(-1.0, 1.0);
  std::uniform_real_distribution<double> pick(0.0, 1.0);

  const int per_col = std::max(1, static_cast<int>(col_density * m));
  fixture.a.col_start.push_back(0);
  for (int col = 0; col < n; ++col) {
    // Deterministic strided support so columns overlap the row_ep support.
    int placed = 0;
    const int stride = std::max(1, m / per_col);
    const int offset = (col * 7919) % stride;
    for (int row = offset; row < m && placed < per_col; row += stride) {
      fixture.a.row_index.push_back(row);
      fixture.a.value.push_back(unit(random));
      ++placed;
    }
    fixture.a.col_start.push_back(static_cast<int>(fixture.a.row_index.size()));
  }

  fixture.resident.assign(static_cast<std::size_t>(m), 0.0);
  for (int row = 0; row < m; ++row) {
    if (pick(random) < rowep_density) {
      const double v = unit(random);
      fixture.row_ep_index.push_back(row);
      fixture.row_ep_value.push_back(v);
      fixture.resident[static_cast<std::size_t>(row)] = v;
    }
  }
  fixture.candidate_count = n;
  return fixture;
}

// The exact dot_error_bound arithmetic from native_dual/pricing.cpp, reading a
// dense row_ep image. Identical in both paths — only the image's provenance
// differs — so the produced value is bit-identical.
inline double dot_error_bound(const Csc& a, int col,
                              const std::vector<double>& dense_row_ep) {
  double absolute_dot = 0.0;
  int terms = 0;
  for (int k = a.col_start[static_cast<std::size_t>(col)];
       k < a.col_start[static_cast<std::size_t>(col) + 1]; ++k) {
    const int row = a.row_index[static_cast<std::size_t>(k)];
    const double multiplier = dense_row_ep[static_cast<std::size_t>(row)];
    absolute_dot += std::abs(multiplier * a.value[static_cast<std::size_t>(k)]);
    ++terms;
  }
  const double eps = std::numeric_limits<double>::epsilon();
  const double product = terms * eps;
  const double gamma = product < 0.5 ? product / (1.0 - product) : 1.0;
  if (absolute_dot == 0.0) return 0.0;
  return gamma * absolute_dot + 256.0 * eps * absolute_dot;
}

// Current path: per pivot, clear a thread-local dense image over m, scatter the
// row_ep support into it, then scan candidates.
double run_materialize(const Fixture& fixture, std::vector<double>& scratch) {
  const int m = fixture.a.rows;
  std::fill(scratch.begin(), scratch.begin() + m, 0.0);  // O(m) clear
  for (std::size_t k = 0; k < fixture.row_ep_index.size(); ++k) {  // O(support)
    scratch[static_cast<std::size_t>(fixture.row_ep_index[k])] =
        fixture.row_ep_value[k];
  }
  double checksum = 0.0;
  for (int col = 0; col < fixture.candidate_count; ++col) {
    checksum += dot_error_bound(fixture.a, col, scratch);
  }
  return checksum;
}

// S2 resident path: read the factor-resident dense image directly; no per-pivot
// clear and no scatter.
double run_resident(const Fixture& fixture) {
  double checksum = 0.0;
  for (int col = 0; col < fixture.candidate_count; ++col) {
    checksum += dot_error_bound(fixture.a, col, fixture.resident);
  }
  return checksum;
}

bool bit_identical(double a, double b) {
  return std::memcmp(&a, &b, sizeof(double)) == 0;
}

template <typename F>
double measure(F&& f, int repeats, double& checksum) {
  const auto start = std::chrono::steady_clock::now();
  for (int r = 0; r < repeats; ++r) checksum += f();
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - start)
      .count();
}

double median(std::vector<double> s) {
  std::sort(s.begin(), s.end());
  return s[s.size() / 2];
}

}  // namespace

int main(int argc, char** argv) {
  int m = 1759;          // d2q06c m
  int n = 6423;          // d2q06c n
  int repeats = 200;
  int samples = 7;
  if (argc > 1) m = std::max(8, std::atoi(argv[1]));
  if (argc > 2) n = std::max(8, std::atoi(argv[2]));
  if (argc > 3) repeats = std::max(1, std::atoi(argv[3]));
  if (argc > 4) samples = std::max(3, std::atoi(argv[4]) | 1);

  const Fixture fixture = make_fixture(m, n, 0.40, 0.40);
  std::vector<double> scratch(static_cast<std::size_t>(m), 0.0);

  // Correctness gate: both paths must be bit-identical.
  const double materialize_checksum = run_materialize(fixture, scratch);
  const double resident_checksum = run_resident(fixture);
  if (!bit_identical(materialize_checksum, resident_checksum)) {
    std::fprintf(stderr,
                 "FAIL: paths disagree (materialize=%.17g resident=%.17g)\n",
                 materialize_checksum, resident_checksum);
    return 2;
  }

  std::vector<double> materialize_samples;
  std::vector<double> resident_samples;
  double sink = 0.0;
  for (int s = 0; s < samples; ++s) {
    if ((s & 1) == 0) {
      materialize_samples.push_back(
          measure([&] { return run_materialize(fixture, scratch); }, repeats,
                  sink));
      resident_samples.push_back(
          measure([&] { return run_resident(fixture); }, repeats, sink));
    } else {
      resident_samples.push_back(
          measure([&] { return run_resident(fixture); }, repeats, sink));
      materialize_samples.push_back(
          measure([&] { return run_materialize(fixture, scratch); }, repeats,
                  sink));
    }
  }

  const double materialize_ms = median(materialize_samples) / repeats * 1e3;
  const double resident_ms = median(resident_samples) / repeats * 1e3;

  // Analytic byte model per pivot: the materialize path additionally writes m
  // zeros (clear) and |support| scattered values (16 B: row read + value
  // write) that the resident path never touches. Both scan the same candidate
  // columns, so the dot traffic is identical and cancels.
  const long clear_bytes = static_cast<long>(m) * 8;
  const long scatter_bytes =
      static_cast<long>(fixture.row_ep_index.size()) * 16;
  const long removed_bytes = clear_bytes + scatter_bytes;

  std::printf("S2 PRICE row_ep resident-view independent kernel\n");
  std::printf("  fixture: m=%d n=%d rowep_support=%zu candidates=%d\n", m, n,
              fixture.row_ep_index.size(), fixture.candidate_count);
  std::printf("  correctness: bit-identical checksums (%.17g)\n",
              materialize_checksum);
  std::printf("  median/pivot: materialize=%.4f ms  resident=%.4f ms  ratio=%.3fx\n",
              materialize_ms, resident_ms,
              resident_ms > 0.0 ? materialize_ms / resident_ms : 0.0);
  std::printf("  removed per pivot: clear=%ld B + scatter=%ld B = %ld B "
              "(instructions: m zero-writes + support scatter eliminated)\n",
              clear_bytes, scatter_bytes, removed_bytes);
  std::printf("  checksum sink=%.3e\n", sink);
  return 0;
}
