#pragma once

#include "state.hpp"

namespace mipsolvers::engine::native_dual::detail {

struct EdgeWeightChange {
  int row{-1};
  double value{0.0};
};

bool choose_leaving(State& state, Leaving& leaving, std::string& failure);
void refresh_leaving_heap(State& state, const std::vector<int>* changed_rows,
                          int extra_row);
IndexedVector multiply_AT_indexed(
    const StandardRowMatrix& A_row,
    const IndexedVector& y);
// Same product written into a caller-owned vector so hot loops can reuse
// its backing storage across pivots.
void multiply_AT_indexed(
    const StandardRowMatrix& A_row,
    const IndexedVector& y, IndexedVector& result);
// Adaptive PRICE: sparse pivotal rows use CSR scatter; sufficiently dense,
// large products use deterministic CSC column partitions on a persistent pool.
void multiply_AT_indexed(
    const StandardColumnMatrix& A, const StandardRowMatrix& A_row,
    const IndexedVector& y, IndexedVector& result, int thread_count);
bool choose_entering_bfrt(const State& state, const Leaving& leaving,
                          const IndexedVector& pivot_row,
                          PivotTransaction& transaction,
                          std::string& failure);
bool compute_dse_weights(const State& state, const Leaving& leaving,
                         const IndexedVector& pivot_row,
                         const IndexedVector& direction, double pivot,
                         std::vector<EdgeWeightChange>& changes,
                         bool& restart_devex, std::string& failure);

}  // namespace mipsolvers::engine::native_dual::detail
