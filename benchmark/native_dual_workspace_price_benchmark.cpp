/// native_dual_workspace_price_benchmark.cpp
///
/// P1+P2 independent-kernel harness; see docs/solvers.md section 5.2.
///
/// The analysis argues the native cold per-pivot gap is six distributed taxes
/// (T1-T6) that only clear wall-clock resolution when rebuilt TOGETHER, so a
/// single-consumer diff always measures "below resolution" (§4.1). This harness
/// therefore compares the WHOLE PRICE data path both ways on one d2q06c-shaped
/// fixture, and reports the §13.3 gate metrics (dynamic instructions AND
/// effective bytes) analytically, plus wall as a secondary signal:
///
///   * "current" = the production `multiply_AT_indexed_impl` shape: an
///     epoch-stamped sparse accumulator (three memory streams: value + stamp +
///     touched) scattered over EVERY column of each r_EP-hit row (basic columns
///     included, ~m/n of them discarded), followed by a SECOND pass over
///     `touched` to build the packed row and a THIRD logical pass for
///     active_position. Models T1 (no dense view -> stamp compensation), T3.1
///     (basic-column scan), T3.3 (3 streams), T3.4 (multi-pass).
///   * "resident" = the P1/P2 target: a factor-resident dense `row_ap.array`
///     (one stream, HVector-style, cleared only over the prior support) scanned
///     over the NONBASIC PREFIX of each hit row (partitioned CSR), with in-flight
///     index and active_position collection in a SINGLE pass.
///
/// Both compute the identical value on every nonbasic column (same terms, same
/// r_EP-row order), so the packed nonbasic result is bit-identical; the harness
/// checks that and reports the byte/instruction reductions that the §13.3 gate
/// requires before the live rebuild.
///
/// Usage: native_dual_workspace_price_benchmark [m] [n] [nnz_per_col]
///                                              [rowep_density] [repeats]

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <vector>

namespace {

// Compressed-sparse-row matrix, partitioned per row so every row's nonbasic
// column entries form a contiguous prefix [row_start, nonbasic_end) and its
// basic entries the suffix [nonbasic_end, row_start+1). The production row
// matrix is NOT partitioned; "resident" reads only the prefix.
struct Csr {
  int rows{0};
  int cols{0};
  std::vector<int> row_start;      // size rows+1
  std::vector<int> nonbasic_end;   // size rows: prefix boundary per row
  std::vector<int> col_index;      // size nnz
  std::vector<double> value;       // size nnz
};

struct Fixture {
  Csr csr;
  std::vector<char> basic;         // size n: 1 if column is basic
  std::vector<char> movable;       // size n: 1 if nonbasic and not fixed
  std::vector<int> rowep_index;    // packed support of r_EP
  std::vector<double> rowep_value; // parallel values
  long nnz{0};
  long nnz_in_hit_rows{0};         // nnz "current" streams (all columns)
  long nnz_nonbasic_in_hit_rows{0};// nnz "resident" streams (nonbasic prefix)
};

// d2q06c-shaped fixture: sparse A (nnz_per_col per column), a size-m basis
// (m of n columns basic, ~27% at the fleet's shape), r_EP ~40% dense, and a
// per-row nonbasic/basic partition of the CSR.
Fixture make_fixture(int m, int n, int nnz_per_col, double rowep_density) {
  Fixture fx;
  fx.csr.rows = m;
  fx.csr.cols = n;
  std::mt19937_64 rng(0x574B535041524353ULL);  // "WKSPARCS"
  std::uniform_real_distribution<double> unit(-1.0, 1.0);
  std::uniform_real_distribution<double> pick(0.0, 1.0);

  // A size-m basis: mark m distinct columns basic. ~10% of nonbasic are fixed.
  fx.basic.assign(static_cast<std::size_t>(n), 0);
  fx.movable.assign(static_cast<std::size_t>(n), 0);
  {
    std::vector<int> cols(static_cast<std::size_t>(n));
    for (int j = 0; j < n; ++j) cols[static_cast<std::size_t>(j)] = j;
    for (int i = 0; i < m && i < n; ++i) {
      const int r = i + static_cast<int>(rng() % static_cast<unsigned>(n - i));
      std::swap(cols[static_cast<std::size_t>(i)], cols[static_cast<std::size_t>(r)]);
      fx.basic[static_cast<std::size_t>(cols[static_cast<std::size_t>(i)])] = 1;
    }
    for (int j = 0; j < n; ++j) {
      if (!fx.basic[static_cast<std::size_t>(j)])
        fx.movable[static_cast<std::size_t>(j)] = pick(rng) < 0.9 ? 1 : 0;
    }
  }

  // Build CSC-style column lists, then transpose into partitioned CSR.
  std::vector<std::vector<int>> col_rows(static_cast<std::size_t>(n));
  std::vector<std::vector<double>> col_vals(static_cast<std::size_t>(n));
  std::vector<char> used(static_cast<std::size_t>(m), 0);
  for (int col = 0; col < n; ++col) {
    const int target = std::min(m, std::max(1, nnz_per_col));
    std::vector<int>& rows = col_rows[static_cast<std::size_t>(col)];
    int placed = 0, guard = 0;
    while (placed < target && guard < target * 8) {
      const int row = static_cast<int>(rng() % static_cast<unsigned>(m));
      ++guard;
      if (used[static_cast<std::size_t>(row)]) continue;
      used[static_cast<std::size_t>(row)] = 1;
      rows.push_back(row);
      ++placed;
    }
    for (const int row : rows) used[static_cast<std::size_t>(row)] = 0;
    std::sort(rows.begin(), rows.end());
    for (const int row : rows) {
      (void)row;
      col_vals[static_cast<std::size_t>(col)].push_back(unit(rng));
    }
  }

  // Row degree, then place nonbasic entries first, basic entries last.
  fx.csr.row_start.assign(static_cast<std::size_t>(m) + 1, 0);
  for (int col = 0; col < n; ++col)
    for (const int row : col_rows[static_cast<std::size_t>(col)])
      ++fx.csr.row_start[static_cast<std::size_t>(row) + 1];
  for (int r = 0; r < m; ++r)
    fx.csr.row_start[static_cast<std::size_t>(r) + 1] +=
        fx.csr.row_start[static_cast<std::size_t>(r)];
  const long nnz = fx.csr.row_start[static_cast<std::size_t>(m)];
  fx.nnz = nnz;
  fx.csr.col_index.resize(static_cast<std::size_t>(nnz));
  fx.csr.value.resize(static_cast<std::size_t>(nnz));
  fx.csr.nonbasic_end.assign(static_cast<std::size_t>(m), 0);
  // Two-cursor fill: nonbasic entries grow from row_start up, basic from
  // row_end down, so each row is [nonbasic | basic].
  std::vector<int> lo(fx.csr.row_start.begin(), fx.csr.row_start.end() - 1);
  std::vector<int> hi(static_cast<std::size_t>(m));
  for (int r = 0; r < m; ++r)
    hi[static_cast<std::size_t>(r)] =
        fx.csr.row_start[static_cast<std::size_t>(r) + 1] - 1;
  for (int col = 0; col < n; ++col) {
    const auto& rows = col_rows[static_cast<std::size_t>(col)];
    const auto& vals = col_vals[static_cast<std::size_t>(col)];
    for (std::size_t k = 0; k < rows.size(); ++k) {
      const int row = rows[k];
      int dst;
      if (fx.basic[static_cast<std::size_t>(col)]) {
        dst = hi[static_cast<std::size_t>(row)]--;
      } else {
        dst = lo[static_cast<std::size_t>(row)]++;
      }
      fx.csr.col_index[static_cast<std::size_t>(dst)] = col;
      fx.csr.value[static_cast<std::size_t>(dst)] = vals[k];
    }
  }
  for (int r = 0; r < m; ++r)
    fx.csr.nonbasic_end[static_cast<std::size_t>(r)] =
        lo[static_cast<std::size_t>(r)];

  // r_EP: rowep_density of rows nonzero.
  for (int row = 0; row < m; ++row) {
    if (pick(rng) < rowep_density) {
      fx.rowep_index.push_back(row);
      fx.rowep_value.push_back(unit(rng));
    }
  }
  for (const int row : fx.rowep_index) {
    fx.nnz_in_hit_rows += fx.csr.row_start[static_cast<std::size_t>(row) + 1] -
                          fx.csr.row_start[static_cast<std::size_t>(row)];
    fx.nnz_nonbasic_in_hit_rows +=
        fx.csr.nonbasic_end[static_cast<std::size_t>(row)] -
        fx.csr.row_start[static_cast<std::size_t>(row)];
  }
  return fx;
}

struct PivotRow {
  std::vector<int> index;
  std::vector<double> value;
  std::vector<int> active_position;
};

constexpr double kTiny = 1e-14;

// --- current: IndexedVector + epoch-stamp accumulator over ALL columns, then
// a second pass to pack + a third to derive active_position. ---------------
void price_current(const Fixture& fx, PivotRow& out) {
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
    const int end = fx.csr.row_start[static_cast<std::size_t>(row) + 1];
    for (int p = fx.csr.row_start[static_cast<std::size_t>(row)]; p < end; ++p) {
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
  out.active_position.clear();
  for (const int col : touched) {  // second pass: filter + pack
    const double v = acc[static_cast<std::size_t>(col)];
    if (std::abs(v) <= kTiny) continue;
    const int pos = static_cast<int>(out.index.size());
    out.index.push_back(col);
    out.value.push_back(v);
    if (fx.movable[static_cast<std::size_t>(col)])  // third logical pass
      out.active_position.push_back(pos);
  }
}

// --- resident: factor-resident dense row_ap.array over the NONBASIC PREFIX,
// single pass with in-flight index + active_position. ----------------------
void price_resident(const Fixture& fx, PivotRow& out) {
  const std::size_t cols = static_cast<std::size_t>(fx.csr.cols);
  static thread_local std::vector<double> array;  // persistent dense image
  static thread_local std::vector<int> support;   // this pivot's support
  if (array.size() != cols) {
    array.assign(cols, 0.0);
    support.clear();
  }
  for (const int col : support) array[static_cast<std::size_t>(col)] = 0.0;
  support.clear();
  out.index.clear();
  out.value.clear();
  out.active_position.clear();
  for (std::size_t k = 0; k < fx.rowep_index.size(); ++k) {
    const int row = fx.rowep_index[k];
    const double mult = fx.rowep_value[k];
    const int end = fx.csr.nonbasic_end[static_cast<std::size_t>(row)];
    for (int p = fx.csr.row_start[static_cast<std::size_t>(row)]; p < end; ++p) {
      const int col = fx.csr.col_index[static_cast<std::size_t>(p)];
      const std::size_t idx = static_cast<std::size_t>(col);
      if (array[idx] == 0.0) support.push_back(col);  // in-flight index
      array[idx] += mult * fx.csr.value[static_cast<std::size_t>(p)];
    }
  }
  for (const int col : support) {  // single tight() pass over own support
    const double v = array[static_cast<std::size_t>(col)];
    if (std::abs(v) <= kTiny) continue;
    const int pos = static_cast<int>(out.index.size());
    out.index.push_back(col);
    out.value.push_back(v);
    if (fx.movable[static_cast<std::size_t>(col)])
      out.active_position.push_back(pos);
  }
}

template <typename Fn>
double time_pivots(Fn&& fn, PivotRow& out, int repeats) {
  auto t0 = std::chrono::steady_clock::now();
  for (int i = 0; i < repeats; ++i) fn(out);
  auto t1 = std::chrono::steady_clock::now();
  return std::chrono::duration<double, std::milli>(t1 - t0).count() / repeats;
}

// Reduce a packed row to nonbasic-only, sorted, for cross-variant agreement.
void nonbasic_sorted(const Fixture& fx, const PivotRow& in,
                     std::vector<std::pair<int, double>>& out) {
  out.clear();
  for (std::size_t k = 0; k < in.index.size(); ++k) {
    const int col = in.index[k];
    if (fx.basic[static_cast<std::size_t>(col)]) continue;
    out.emplace_back(col, in.value[k]);
  }
  std::sort(out.begin(), out.end());
}

}  // namespace

int main(int argc, char** argv) {
  const int m = argc > 1 ? std::atoi(argv[1]) : 1759;
  const int n = argc > 2 ? std::atoi(argv[2]) : 6423;
  const int nnz_per_col = argc > 3 ? std::atoi(argv[3]) : 5;
  const double rowep_density = argc > 4 ? std::atof(argv[4]) : 0.40;
  const int repeats = argc > 5 ? std::atoi(argv[5]) : 20000;

  const Fixture fx = make_fixture(m, n, nnz_per_col, rowep_density);

  PivotRow cur, res;
  price_current(fx, cur);
  price_resident(fx, res);

  // Correctness: identical value on every nonbasic column (Class A vs the full
  // row because basic columns are dropped, but bit-identical on the consumed
  // nonbasic set).
  std::vector<std::pair<int, double>> a, b;
  nonbasic_sorted(fx, cur, a);
  nonbasic_sorted(fx, res, b);
  double max_rel = 0.0;
  bool same_support = a.size() == b.size();
  if (same_support) {
    for (std::size_t k = 0; k < a.size(); ++k) {
      if (a[k].first != b[k].first) { same_support = false; break; }
      const double d = std::abs(a[k].second - b[k].second);
      const double s = std::max(1.0, std::abs(a[k].second));
      max_rel = std::max(max_rel, d / s);
    }
  }

  // Warm, then time.
  for (int i = 0; i < 200; ++i) { price_current(fx, cur); price_resident(fx, res); }
  const double ms_cur = time_pivots([&](PivotRow& o) { price_current(fx, o); }, cur, repeats);
  const double ms_res = time_pivots([&](PivotRow& o) { price_resident(fx, o); }, res, repeats);

  // Analytic effective-byte model per pivot (12 B per stored matrix nonzero:
  // 4 col index + 8 value; 8 B per dense-array touch; 4 B per index/stamp int).
  const long uniq = static_cast<long>(cur.index.size() + cur.active_position.size());
  const long cur_matrix = fx.nnz_in_hit_rows * 12;
  const long cur_stamp = fx.nnz_in_hit_rows * 4;    // per-nnz stamp read
  const long cur_acc = fx.nnz_in_hit_rows * 8;      // per-nnz acc RMW
  const long cur_touch = static_cast<long>(cur.index.size()) * 4;
  const long cur_second = static_cast<long>(cur.index.size()) * (8 + 12);  // reread acc + emit
  const long cur_bytes = cur_matrix + cur_stamp + cur_acc + cur_touch + cur_second;

  const long res_matrix = fx.nnz_nonbasic_in_hit_rows * 12;
  const long res_arr = fx.nnz_nonbasic_in_hit_rows * 8;  // per-nnz array RMW
  const long res_index = static_cast<long>(res.index.size()) * 4;
  const long res_tight = static_cast<long>(res.index.size()) * (8 + 12);
  const long res_bytes = res_matrix + res_arr + res_index + res_tight;

  // Instruction proxy: total per-nnz scatter ops + per-support second-pass ops.
  const long cur_instr = fx.nnz_in_hit_rows * 5    // load col, load val, fma, stamp cmp, acc add
                       + static_cast<long>(cur.index.size()) * 4;  // second/third pass
  const long res_instr = fx.nnz_nonbasic_in_hit_rows * 4  // load col, load val, fma, zero-test
                       + static_cast<long>(res.index.size()) * 3;  // single tight pass

  const long gather_saved = static_cast<long>(m) * 16;  // T2.1 O(m) gather per solve (read+write)

  std::printf("P1+P2 workspace/PRICE independent kernel\n");
  std::printf("  fixture: m=%d n=%d nnz=%ld (%.2f/col) basis=%d/%d(%.1f%%) "
              "rEP=%zu(%.1f%% of m)\n",
              m, n, fx.nnz, static_cast<double>(fx.nnz) / n, m, n,
              100.0 * m / n, fx.rowep_index.size(),
              100.0 * fx.rowep_index.size() / m);
  std::printf("  nnz streamed/pivot: current(all cols)=%ld  resident(nonbasic "
              "prefix)=%ld  (%.2fx fewer)\n",
              fx.nnz_in_hit_rows, fx.nnz_nonbasic_in_hit_rows,
              fx.nnz_nonbasic_in_hit_rows > 0
                  ? static_cast<double>(fx.nnz_in_hit_rows) /
                        fx.nnz_nonbasic_in_hit_rows
                  : 0.0);
  std::printf("  nonbasic result: cols=%zu  active=%zu  support-agree=%s "
              "max-rel=%.2e\n",
              res.index.size(), res.active_position.size(),
              same_support ? "yes" : "NO", max_rel);
  (void)uniq;
  std::printf("  effective bytes/pivot:  current=%ld  resident=%ld  (%.3fx)\n",
              cur_bytes, res_bytes,
              res_bytes > 0 ? static_cast<double>(cur_bytes) / res_bytes : 0.0);
  std::printf("    + resident also removes the T2.1 O(m) permute gather: "
              "%ld B/solve saved\n", gather_saved);
  std::printf("  instruction proxy/pivot: current=%ld  resident=%ld  (%.3fx)\n",
              cur_instr, res_instr,
              res_instr > 0 ? static_cast<double>(cur_instr) / res_instr : 0.0);
  std::printf("  wall ms/pivot: current=%.6f  resident=%.6f  (%.3fx)\n",
              ms_cur, ms_res, ms_res > 0 ? ms_cur / ms_res : 0.0);
  const bool bytes_drop = res_bytes < cur_bytes;
  const bool instr_drop = res_instr < cur_instr;
  std::printf("  §13.3 verdict: bytes %s AND instructions %s -> gate %s\n",
              bytes_drop ? "DROP" : "flat/up", instr_drop ? "DROP" : "flat/up",
              (bytes_drop && instr_drop) ? "PASSED" : "NOT passed");
  if (!same_support) {
    std::printf("  ERROR: nonbasic support disagreement -> incorrect kernel\n");
    return 1;
  }
  return 0;
}
