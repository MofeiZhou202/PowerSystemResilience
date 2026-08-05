#pragma once

#include "state.hpp"

namespace mipsolvers::engine::native_dual::detail {

struct EdgeWeightUpdate {
  // Aligned with direction.index after skipping the leaving row. Reusing that
  // owner avoids duplicating row indices in the transactional value stream.
  std::vector<double> nonpivotal_value;
  double pivotal_value{0.0};
};

bool choose_leaving(State& state, Leaving& leaving, std::string& failure);
void refresh_leaving_heap(State& state, const std::vector<int>* changed_rows);
IndexedVector multiply_AT_indexed(const StandardRowMatrix& A_row,
                                  const IndexedVector& y);
// Same product written into a caller-owned vector so hot loops can reuse
// its backing storage across pivots.
void multiply_AT_indexed(const StandardRowMatrix& A_row, const IndexedVector& y,
                         IndexedVector& result);
// Dual PRICE keeps the complete pivotal row for reduced-cost and Devex
// updates, while exporting positions of nonbasic movable entries in the same
// producer pass. BFRT consumes this active-only view without rescanning state.
void multiply_AT_indexed_bfrt(const StandardRowMatrix& A_row,
                              const IndexedVector& y,
                              const std::vector<char>& basic,
                              const std::vector<Move>& move,
                              IndexedVector& result,
                              std::vector<int>& active_position);
// Partitioned dual PRICE: scans only the nonbasic prefix of each row_ep-hit
// row (skipping basic columns), then injects the leaving column's pivotal
// entry. Produces the same nonbasic pivotal-row values and active positions
// as multiply_AT_indexed_bfrt, in a nonbasic-first packing order.
void multiply_AT_partitioned_bfrt(const PartitionedRowMatrix& partition,
                                  const StandardColumnMatrix& columns,
                                  const IndexedVector& y,
                                  const std::vector<Move>& move, int leaving_col,
                                  IndexedVector& result,
                                  std::vector<int>& active_position);
// Deterministic experimental CSC kernel exposed within the native-dual module
// for rounding-envelope checks and controlled kernel benchmarks. Production
// PRICE remains on the CSR path until a suite-positive selector is proved.
void multiply_AT_indexed_csc(const StandardColumnMatrix& A,
                             const IndexedVector& y, IndexedVector& result,
                             int thread_count);
bool choose_entering_bfrt(const State& state, const Leaving& leaving,
                          const IndexedVector& pivot_row,
                          PivotTransaction& transaction, std::string& failure);
bool choose_entering_bfrt(const State& state, const Leaving& leaving,
                          const IndexedVector& pivot_row,
                          const std::vector<int>& active_position,
                          PivotTransaction& transaction, std::string& failure);
bool compute_dse_weights(const State& state, const Leaving& leaving,
                         const IndexedVector& pivot_row,
                         const IndexedVector& direction, double pivot,
                         EdgeWeightUpdate& update,
                         bool& restart_devex, std::string& failure);

}  // namespace mipsolvers::engine::native_dual::detail
