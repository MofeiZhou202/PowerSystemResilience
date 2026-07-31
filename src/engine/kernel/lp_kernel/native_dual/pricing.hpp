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
    const Eigen::SparseMatrix<double, Eigen::RowMajor>& A_row,
    const IndexedVector& y);
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
