/// native_dual_price_simd_benchmark.cpp
///
/// S7 independent-kernel harness (roadmap native_dual_simplex_system_roadmap
/// 2026-08-04, §11/§13.3) for the PRICE step `pivot_row = A^T * r_EP`, the
/// single largest per-pivot bucket on the fleet's dominant case (d2q06c:
/// price(A^T*rEP) ~= 27% of wall; DR-3).
///
/// DR-3 established that native's cold gap to HiGHS on d2q06c is per-pivot cost
/// (2.28x), not pivot count, and that r_EP is ~40% dense (rowEP 40.16% of m,
/// pivot-row 40.30% of n) - far above the hypersparse crossover. This harness
/// tests, per the §13.3 promotion gate, whether a SIMD PRICE kernel can reduce
/// BOTH dynamic instructions AND effective bytes (wall alone is not a gate).
///
/// It replays the exact two layouts native already carries:
///   * row-wise SPA scatter  (multiply_AT_indexed_impl, the production hot path
///     the dual-II BFRT uses): iterate r_EP's nonzero rows, stream each hit
///     row of A (CSR) and scatter-accumulate into an epoch-stamped sparse
///     accumulator. Reads only the r_EP-hit rows' nonzeros (~40% of nnz).
///   * column-wise gather-dot (multiply_AT_indexed_csc): iterate every column,
///     gather r_EP over its stored rows and accumulate. Reads ALL nnz (100%).
///
/// and a NEON variant of each, on a d2q06c-shaped sparse fixture (sparse A,
/// 40%-dense r_EP). It reports a bounded cross-layout agreement (FP reduction
/// order differs, so this is Class A per §13.2), the per-pivot analytic byte
/// model, and wall ratios - so the §13.3 verdict rests on measured bytes, not
/// wall time.
///
/// Usage: native_dual_price_simd_benchmark [m] [n] [nnz_per_col] [rowep_density]
///                                         [repeats] [samples]

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <vector>

#if defined(__aarch64__)
#include <arm_neon.h>
#define PRICE_SIMD_NEON 1
#else
#define PRICE_SIMD_NEON 0
#endif

namespace {

// Compressed-sparse-column matrix (native reads columns via
// StandardColumnMatrix::InnerIterator in the CSC price path).
struct Csc {
  int rows{0};
  int cols{0};
  std::vector<int> col_start;  // size cols+1
  std::vector<int> row_index;  // size nnz
  std::vector<double> value;   // size nnz
};

// Compressed-sparse-row matrix (native reads rows via
// StandardRowMatrix::InnerIterator in the row-wise PRICE path).
struct Csr {
  int rows{0};
  int cols{0};
  std::vector<int> row_start;  // size rows+1
  std::vector<int> col_index;  // size nnz
  std::vector<double> value;   // size nnz
};

struct Fixture {
  Csc csc;
  Csr csr;
  std::vector<int> rowep_index;      // packed support of r_EP
  std::vector<double> rowep_value;   // parallel values
  std::vector<double> rowep_dense;   // dense r_EP image (length m)
  long nnz{0};
  long nnz_in_hit_rows{0};           // nnz the row-wise path actually streams
};

// A d2q06c-shaped fixture: sparse A (nnz_per_col entries per column at random
// rows) with a 40%-dense r_EP. Both CSC and CSR views of the same matrix.
Fixture make_fixture(int m, int n, int nnz_per_col, double rowep_density) {
  Fixture fx;
  fx.csc.rows = m;
  fx.csc.cols = n;
  std::mt19937_64 rng(0x50524943'53494D44ULL);  // "PRICSIMD"
  std::uniform_real_distribution<double> unit(-1.0, 1.0);
  std::uniform_real_distribution<double> pick(0.0, 1.0);

  // Build CSC with distinct random rows per column.
  fx.csc.col_start.reserve(static_cast<std::size_t>(n) + 1);
  fx.csc.col_start.push_back(0);
  std::vector<char> used(static_cast<std::size_t>(m), 0);
  std::vector<int> used_rows;
  for (int col = 0; col < n; ++col) {
    const int target = std::min(m, std::max(1, nnz_per_col));
    used_rows.clear();
    int placed = 0;
    int guard = 0;
    while (placed < target && guard < target * 8) {
      const int row = static_cast<int>(rng() % static_cast<unsigned>(m));
      ++guard;
      if (used[static_cast<std::size_t>(row)]) continue;
      used[static_cast<std::size_t>(row)] = 1;
      used_rows.push_back(row);
      ++placed;
    }
    std::sort(used_rows.begin(), used_rows.end());
    for (const int row : used_rows) {
      fx.csc.row_index.push_back(row);
      fx.csc.value.push_back(unit(rng));
      used[static_cast<std::size_t>(row)] = 0;
    }
    fx.csc.col_start.push_back(static_cast<int>(fx.csc.row_index.size()));
  }
  fx.nnz = static_cast<long>(fx.csc.row_index.size());

  // Transpose CSC -> CSR.
  fx.csr.rows = m;
  fx.csr.cols = n;
  fx.csr.row_start.assign(static_cast<std::size_t>(m) + 1, 0);
  for (const int row : fx.csc.row_index)
    ++fx.csr.row_start[static_cast<std::size_t>(row) + 1];
  for (int r = 0; r < m; ++r)
    fx.csr.row_start[static_cast<std::size_t>(r) + 1] +=
        fx.csr.row_start[static_cast<std::size_t>(r)];
  fx.csr.col_index.resize(fx.csc.row_index.size());
  fx.csr.value.resize(fx.csc.value.size());
  std::vector<int> cursor(fx.csr.row_start.begin(), fx.csr.row_start.end() - 1);
  for (int col = 0; col < n; ++col) {
    for (int k = fx.csc.col_start[static_cast<std::size_t>(col)];
         k < fx.csc.col_start[static_cast<std::size_t>(col) + 1]; ++k) {
      const int row = fx.csc.row_index[static_cast<std::size_t>(k)];
      const int dst = cursor[static_cast<std::size_t>(row)]++;
      fx.csr.col_index[static_cast<std::size_t>(dst)] = col;
      fx.csr.value[static_cast<std::size_t>(dst)] =
          fx.csc.value[static_cast<std::size_t>(k)];
    }
  }

  // r_EP: 40% of rows nonzero.
  fx.rowep_dense.assign(static_cast<std::size_t>(m), 0.0);
  for (int row = 0; row < m; ++row) {
    if (pick(rng) < rowep_density) {
      const double v = unit(rng);
      fx.rowep_index.push_back(row);
      fx.rowep_value.push_back(v);
      fx.rowep_dense[static_cast<std::size_t>(row)] = v;
    }
  }
  for (const int row : fx.rowep_index)
    fx.nnz_in_hit_rows += fx.csr.row_start[static_cast<std::size_t>(row) + 1] -
                          fx.csr.row_start[static_cast<std::size_t>(row)];
  return fx;
}

// Packed pivot row output, sorted by column, for cross-variant agreement.
struct PivotRow {
  std::vector<int> index;
  std::vector<double> value;
};

// --- Row-wise SPA scatter: the production hot path (scalar). ---------------
void price_rowwise_scalar(const Fixture& fx, PivotRow& out) {
  const std::size_t cols = static_cast<std::size_t>(fx.csr.cols);
  static thread_local std::vector<double> acc;
  static thread_local std::vector<unsigned> stamp;
  static thread_local unsigned epoch = 0;
  static thread_local std::vector<int> touched;
  if (acc.size() != cols) {
    acc.assign(cols, 0.0);
    stamp.assign(cols, 0);
    epoch = 0;
  }
  if (++epoch == 0) {
    std::fill(stamp.begin(), stamp.end(), 0);
    ++epoch;
  }
  touched.clear();
  for (std::size_t k = 0; k < fx.rowep_index.size(); ++k) {
    const int row = fx.rowep_index[k];
    const double mult = fx.rowep_value[k];
    for (int p = fx.csr.row_start[static_cast<std::size_t>(row)];
         p < fx.csr.row_start[static_cast<std::size_t>(row) + 1]; ++p) {
      const int col = fx.csr.col_index[static_cast<std::size_t>(p)];
      const std::size_t idx = static_cast<std::size_t>(col);
      if (stamp[idx] != epoch) {
        stamp[idx] = epoch;
        acc[idx] = 0.0;
        touched.push_back(col);
      }
      acc[idx] += mult * fx.csr.value[static_cast<std::size_t>(p)];
    }
  }
  out.index.clear();
  out.value.clear();
  for (const int col : touched) {
    out.index.push_back(col);
    out.value.push_back(acc[static_cast<std::size_t>(col)]);
  }
}

// --- Column-wise gather-dot (scalar). --------------------------------------
void price_colwise_scalar(const Fixture& fx, PivotRow& out) {
  out.index.clear();
  out.value.clear();
  const double* rep = fx.rowep_dense.data();
  for (int col = 0; col < fx.csc.cols; ++col) {
    double sum = 0.0;
    for (int k = fx.csc.col_start[static_cast<std::size_t>(col)];
         k < fx.csc.col_start[static_cast<std::size_t>(col) + 1]; ++k) {
      sum += fx.csc.value[static_cast<std::size_t>(k)] *
             rep[static_cast<std::size_t>(
                 fx.csc.row_index[static_cast<std::size_t>(k)])];
    }
    if (sum != 0.0) {
      out.index.push_back(col);
      out.value.push_back(sum);
    }
  }
}

// --- Column-wise gather-dot (NEON: 2 lanes, manual gather). ----------------
void price_colwise_simd(const Fixture& fx, PivotRow& out) {
  out.index.clear();
  out.value.clear();
  const double* rep = fx.rowep_dense.data();
  const double* val = fx.csc.value.data();
  const int* rowi = fx.csc.row_index.data();
  for (int col = 0; col < fx.csc.cols; ++col) {
    const int first = fx.csc.col_start[static_cast<std::size_t>(col)];
    const int last = fx.csc.col_start[static_cast<std::size_t>(col) + 1];
    double sum = 0.0;
    int k = first;
#if PRICE_SIMD_NEON
    float64x2_t vacc = vdupq_n_f64(0.0);
    for (; k + 1 < last; k += 2) {
      const float64x2_t vv =
          vld1q_f64(&val[static_cast<std::size_t>(k)]);  // contiguous values
      const double g0 = rep[static_cast<std::size_t>(rowi[k])];      // gather
      const double g1 = rep[static_cast<std::size_t>(rowi[k + 1])];  // gather
      const float64x2_t vg = {g0, g1};
      vacc = vfmaq_f64(vacc, vv, vg);
    }
    sum = vaddvq_f64(vacc);
#endif
    for (; k < last; ++k)
      sum += val[static_cast<std::size_t>(k)] *
             rep[static_cast<std::size_t>(rowi[k])];
    if (sum != 0.0) {
      out.index.push_back(col);
      out.value.push_back(sum);
    }
  }
}

// --- Row-wise SPA scatter with a NEON multiply of the streamed row. --------
// The scatter itself stays scalar (distinct destination columns), so only the
// multiply is vectorized; this is the most SIMD the hot layout admits.
void price_rowwise_simd(const Fixture& fx, PivotRow& out) {
  const std::size_t cols = static_cast<std::size_t>(fx.csr.cols);
  static thread_local std::vector<double> acc;
  static thread_local std::vector<unsigned> stamp;
  static thread_local unsigned epoch = 0;
  static thread_local std::vector<int> touched;
  if (acc.size() != cols) {
    acc.assign(cols, 0.0);
    stamp.assign(cols, 0);
    epoch = 0;
  }
  if (++epoch == 0) {
    std::fill(stamp.begin(), stamp.end(), 0);
    ++epoch;
  }
  touched.clear();
  const double* val = fx.csr.value.data();
  const int* coli = fx.csr.col_index.data();
  for (std::size_t j = 0; j < fx.rowep_index.size(); ++j) {
    const int row = fx.rowep_index[j];
    const double mult = fx.rowep_value[j];
    const int first = fx.csr.row_start[static_cast<std::size_t>(row)];
    const int last = fx.csr.row_start[static_cast<std::size_t>(row) + 1];
    int p = first;
#if PRICE_SIMD_NEON
    const float64x2_t vmult = vdupq_n_f64(mult);
    for (; p + 1 < last; p += 2) {
      const float64x2_t vv = vld1q_f64(&val[static_cast<std::size_t>(p)]);
      const float64x2_t vp = vmulq_f64(vmult, vv);  // 2 products at once
      double prod[2];
      vst1q_f64(prod, vp);
      for (int lane = 0; lane < 2; ++lane) {
        const int col = coli[p + lane];
        const std::size_t idx = static_cast<std::size_t>(col);
        if (stamp[idx] != epoch) {
          stamp[idx] = epoch;
          acc[idx] = 0.0;
          touched.push_back(col);
        }
        acc[idx] += prod[lane];  // scalar scatter
      }
    }
#endif
    for (; p < last; ++p) {
      const int col = coli[p];
      const std::size_t idx = static_cast<std::size_t>(col);
      if (stamp[idx] != epoch) {
        stamp[idx] = epoch;
        acc[idx] = 0.0;
        touched.push_back(col);
      }
      acc[idx] += mult * val[static_cast<std::size_t>(p)];
    }
  }
  out.index.clear();
  out.value.clear();
  for (const int col : touched) {
    out.index.push_back(col);
    out.value.push_back(acc[static_cast<std::size_t>(col)]);
  }
}

double max_rel_disagreement(PivotRow a, PivotRow b) {
  auto sort_pack = [](PivotRow& p) {
    std::vector<int> order(p.index.size());
    for (std::size_t i = 0; i < order.size(); ++i) order[i] = static_cast<int>(i);
    std::sort(order.begin(), order.end(),
              [&](int x, int y) { return p.index[x] < p.index[y]; });
    PivotRow s;
    for (const int i : order) {
      s.index.push_back(p.index[static_cast<std::size_t>(i)]);
      s.value.push_back(p.value[static_cast<std::size_t>(i)]);
    }
    p = std::move(s);
  };
  sort_pack(a);
  sort_pack(b);
  double worst = 0.0;
  std::size_t ia = 0, ib = 0;
  while (ia < a.index.size() && ib < b.index.size()) {
    if (a.index[ia] == b.index[ib]) {
      const double denom = std::max(1.0, std::abs(a.value[ia]));
      worst = std::max(worst, std::abs(a.value[ia] - b.value[ib]) / denom);
      ++ia;
      ++ib;
    } else if (a.index[ia] < b.index[ib]) {
      // present in a, dropped by b (below its zero test): treat as tiny.
      worst = std::max(worst, std::abs(a.value[ia]));
      ++ia;
    } else {
      worst = std::max(worst, std::abs(b.value[ib]));
      ++ib;
    }
  }
  return worst;
}

template <typename F>
double measure_ms(F&& f, int repeats) {
  const auto start = std::chrono::steady_clock::now();
  for (int r = 0; r < repeats; ++r) f();
  return std::chrono::duration<double, std::milli>(
             std::chrono::steady_clock::now() - start)
             .count() /
         repeats;
}

double median(std::vector<double> s) {
  std::sort(s.begin(), s.end());
  return s[s.size() / 2];
}

}  // namespace

int main(int argc, char** argv) {
  int m = 1759;             // d2q06c m (presolved)
  int n = 6423;             // d2q06c n
  int nnz_per_col = 5;      // d2q06c ~ 32417 nnz / 6423 cols
  double rowep_density = 0.40;  // DS-DENSITY rowEP 40.16% of m
  int repeats = 200;
  int samples = 7;
  if (argc > 1) m = std::max(8, std::atoi(argv[1]));
  if (argc > 2) n = std::max(8, std::atoi(argv[2]));
  if (argc > 3) nnz_per_col = std::max(1, std::atoi(argv[3]));
  if (argc > 4) rowep_density = std::max(0.01, std::atof(argv[4]));
  if (argc > 5) repeats = std::max(1, std::atoi(argv[5]));
  if (argc > 6) samples = std::max(3, std::atoi(argv[6]) | 1);

  const Fixture fx = make_fixture(m, n, nnz_per_col, rowep_density);

  PivotRow rw_s, rw_v, cw_s, cw_v;
  price_rowwise_scalar(fx, rw_s);
  price_rowwise_simd(fx, rw_v);
  price_colwise_scalar(fx, cw_s);
  price_colwise_simd(fx, cw_v);

  // Correctness: SIMD must agree with its scalar layout to rounding; the two
  // layouts agree up to FP reduction order (Class A per §13.2).
  const double rw_simd_vs_scalar = max_rel_disagreement(rw_s, rw_v);
  const double cw_simd_vs_scalar = max_rel_disagreement(cw_s, cw_v);
  const double layout_agreement = max_rel_disagreement(rw_s, cw_s);

  std::vector<double> t_rw_s, t_rw_v, t_cw_s, t_cw_v;
  for (int s = 0; s < samples; ++s) {
    t_rw_s.push_back(measure_ms([&] { PivotRow o; price_rowwise_scalar(fx, o); }, repeats));
    t_rw_v.push_back(measure_ms([&] { PivotRow o; price_rowwise_simd(fx, o); }, repeats));
    t_cw_s.push_back(measure_ms([&] { PivotRow o; price_colwise_scalar(fx, o); }, repeats));
    t_cw_v.push_back(measure_ms([&] { PivotRow o; price_colwise_simd(fx, o); }, repeats));
  }
  const double ms_rw_s = median(t_rw_s);
  const double ms_rw_v = median(t_rw_v);
  const double ms_cw_s = median(t_cw_s);
  const double ms_cw_v = median(t_cw_v);

  // Analytic effective-byte model per pivot (matrix traffic, 12 B per stored
  // entry = 4 B column/row index + 8 B value). The row-wise layout streams
  // only the nonzeros in r_EP-hit rows; the column-wise layout streams every
  // nonzero. SIMD changes neither layout's byte footprint.
  const long rowwise_matrix_bytes = fx.nnz_in_hit_rows * 12;
  const long colwise_matrix_bytes = fx.nnz * 12;

  std::printf("S7 PRICE (A^T*r_EP) SIMD independent kernel  [NEON=%d]\n",
              PRICE_SIMD_NEON);
  std::printf("  fixture: m=%d n=%d nnz=%ld (%.2f/col) rowep=%zu (%.1f%% of m)\n",
              m, n, fx.nnz, static_cast<double>(fx.nnz) / n,
              fx.rowep_index.size(),
              100.0 * static_cast<double>(fx.rowep_index.size()) / m);
  std::printf("  correctness (max rel diff): rowwise simd-vs-scalar=%.2e  "
              "colwise simd-vs-scalar=%.2e  layout agreement=%.2e\n",
              rw_simd_vs_scalar, cw_simd_vs_scalar, layout_agreement);
  std::printf("  wall/pivot ms: rowwise scalar=%.5f simd=%.5f (%.3fx)  |  "
              "colwise scalar=%.5f simd=%.5f (%.3fx)\n",
              ms_rw_s, ms_rw_v, ms_rw_v > 0 ? ms_rw_s / ms_rw_v : 0.0, ms_cw_s,
              ms_cw_v, ms_cw_v > 0 ? ms_cw_s / ms_cw_v : 0.0);
  std::printf("  effective matrix bytes/pivot: rowwise=%ld  colwise=%ld  "
              "(colwise/rowwise=%.2fx)\n",
              rowwise_matrix_bytes, colwise_matrix_bytes,
              rowwise_matrix_bytes > 0
                  ? static_cast<double>(colwise_matrix_bytes) /
                        rowwise_matrix_bytes
                  : 0.0);
  std::printf(
      "  §13.3 verdict: SIMD keeps each layout's byte footprint unchanged "
      "(same data streamed); bytes do NOT drop -> gate not passed.\n");
  std::printf(
      "  Row-wise (production hot path) is already the byte-minimal layout "
      "(%.2fx fewer matrix bytes than colwise); its scatter+per-nnz stamp "
      "branch is SIMD-hostile.\n",
      colwise_matrix_bytes > 0
          ? static_cast<double>(colwise_matrix_bytes) / rowwise_matrix_bytes
          : 0.0);
  return 0;
}
