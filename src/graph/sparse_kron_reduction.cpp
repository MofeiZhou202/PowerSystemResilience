#include "hacdcpf/graph/sparse_kron_reduction.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <queue>
#include <stdexcept>
#include <unordered_map>

namespace hacdcpf::graph {
namespace {

using Complex = std::complex<double>;

class MutableSparseMatrix {
 public:
  MutableSparseMatrix(const SparseComplexMatrix& matrix,
                      double drop_tolerance)
      : rows_(static_cast<std::size_t>(matrix.rows())),
        cols_(static_cast<std::size_t>(matrix.cols())),
        active_(static_cast<std::size_t>(matrix.rows()), true),
        drop_tolerance_(drop_tolerance) {
    for (int col = 0; col < matrix.outerSize(); ++col) {
      for (SparseComplexMatrix::InnerIterator it(matrix, col); it; ++it) {
        set(it.row(), it.col(), it.value());
      }
    }
    initial_nnz_ = nnz_;
  }

  [[nodiscard]] int size() const { return static_cast<int>(rows_.size()); }
  [[nodiscard]] bool active(int node) const {
    return active_.at(static_cast<std::size_t>(node));
  }
  [[nodiscard]] long long initial_nonzeros() const { return initial_nnz_; }

  [[nodiscard]] Complex value(int row, int col) const {
    const auto& entries = rows_.at(static_cast<std::size_t>(row));
    const auto it = entries.find(col);
    return it == entries.end() ? Complex{} : it->second;
  }

  [[nodiscard]] std::vector<int> row_neighbors(int node) const {
    std::vector<int> result;
    for (const auto& [col, entry] : rows_.at(static_cast<std::size_t>(node))) {
      if (col != node && active(col) && std::abs(entry) > drop_tolerance_) {
        result.push_back(col);
      }
    }
    std::sort(result.begin(), result.end());
    return result;
  }

  [[nodiscard]] std::vector<int> col_neighbors(int node) const {
    std::vector<int> result;
    for (const auto& [row, entry] : cols_.at(static_cast<std::size_t>(node))) {
      if (row != node && active(row) && std::abs(entry) > drop_tolerance_) {
        result.push_back(row);
      }
    }
    std::sort(result.begin(), result.end());
    return result;
  }

  [[nodiscard]] int front_size(int node) const {
    return std::max(static_cast<int>(row_neighbors(node).size()),
                    static_cast<int>(col_neighbors(node).size()));
  }

  [[nodiscard]] long long predicted_nonzeros_after_elimination(int node) const {
    const auto row_nodes = row_neighbors(node);
    const auto col_nodes = col_neighbors(node);
    long long additions = 0;
    for (int row : col_nodes) {
      for (int col : row_nodes) {
        if (std::abs(value(row, col)) <= drop_tolerance_) ++additions;
      }
    }
    long long removals = static_cast<long long>(
        rows_.at(static_cast<std::size_t>(node)).size() +
        cols_.at(static_cast<std::size_t>(node)).size());
    if (std::abs(value(node, node)) > drop_tolerance_) --removals;
    return nnz_ - removals + additions;
  }

  SparseKronRecoveryStep eliminate(int node) {
    const Complex pivot = value(node, node);
    const auto row_nodes = row_neighbors(node);
    const auto col_nodes = col_neighbors(node);

    SparseKronRecoveryStep recovery;
    recovery.node = node;
    recovery.coefficients.reserve(row_nodes.size());
    for (int col : row_nodes) {
      recovery.coefficients.emplace_back(col, -value(node, col) / pivot);
    }

    for (int row : col_nodes) {
      const Complex left = value(row, node);
      for (int col : row_nodes) {
        set(row, col, value(row, col) - left * value(node, col) / pivot);
      }
    }

    std::vector<int> row_keys;
    row_keys.reserve(rows_.at(static_cast<std::size_t>(node)).size());
    for (const auto& [col, unused] : rows_.at(static_cast<std::size_t>(node))) {
      (void)unused;
      row_keys.push_back(col);
    }
    for (int col : row_keys) erase(node, col);

    std::vector<int> col_keys;
    col_keys.reserve(cols_.at(static_cast<std::size_t>(node)).size());
    for (const auto& [row, unused] : cols_.at(static_cast<std::size_t>(node))) {
      (void)unused;
      col_keys.push_back(row);
    }
    for (int row : col_keys) erase(row, node);

    active_.at(static_cast<std::size_t>(node)) = false;
    return recovery;
  }

  SparseComplexMatrix retained_matrix(std::vector<int>& retained) const {
    retained.clear();
    std::vector<int> position(rows_.size(), -1);
    for (int node = 0; node < size(); ++node) {
      if (!active(node)) continue;
      position[static_cast<std::size_t>(node)] = static_cast<int>(retained.size());
      retained.push_back(node);
    }

    std::vector<Eigen::Triplet<Complex>> triplets;
    triplets.reserve(static_cast<std::size_t>(std::max<long long>(0, nnz_)));
    for (int row : retained) {
      for (const auto& [col, entry] : rows_.at(static_cast<std::size_t>(row))) {
        const int reduced_col = position.at(static_cast<std::size_t>(col));
        if (reduced_col < 0 || std::abs(entry) <= drop_tolerance_) continue;
        triplets.emplace_back(position.at(static_cast<std::size_t>(row)),
                              reduced_col, entry);
      }
    }
    SparseComplexMatrix reduced(static_cast<int>(retained.size()),
                                static_cast<int>(retained.size()));
    reduced.setFromTriplets(triplets.begin(), triplets.end());
    reduced.makeCompressed();
    return reduced;
  }

 private:
  void set(int row, int col, Complex entry) {
    auto& row_map = rows_.at(static_cast<std::size_t>(row));
    auto& col_map = cols_.at(static_cast<std::size_t>(col));
    const auto existing = row_map.find(col);
    const bool had_value = existing != row_map.end();
    if (std::abs(entry) <= drop_tolerance_) {
      if (had_value) {
        row_map.erase(existing);
        col_map.erase(row);
        --nnz_;
      }
      return;
    }
    row_map[col] = entry;
    col_map[row] = entry;
    if (!had_value) ++nnz_;
  }

  void erase(int row, int col) {
    auto& row_map = rows_.at(static_cast<std::size_t>(row));
    const auto it = row_map.find(col);
    if (it == row_map.end()) return;
    row_map.erase(it);
    cols_.at(static_cast<std::size_t>(col)).erase(row);
    --nnz_;
  }

  std::vector<std::unordered_map<int, Complex>> rows_;
  std::vector<std::unordered_map<int, Complex>> cols_;
  std::vector<bool> active_;
  double drop_tolerance_{1e-13};
  long long nnz_{0};
  long long initial_nnz_{0};
};

}  // namespace

SparseKronResult reduce_sparse_kron(const SparseComplexMatrix& ybus,
                                    const std::vector<bool>& eligible,
                                    const SparseKronOptions& options) {
  SparseKronResult result;
  result.original_size = static_cast<int>(ybus.rows());
  result.original_nonzeros = static_cast<int>(ybus.nonZeros());
  if (ybus.rows() != ybus.cols()) {
    result.error = "sparse Kron matrix must be square";
    return result;
  }
  if (eligible.size() != static_cast<std::size_t>(ybus.rows())) {
    result.error = "sparse Kron eligibility mask dimension mismatch";
    return result;
  }
  if (options.max_front < 0 || options.max_nnz_ratio < 1.0 ||
      options.pivot_tolerance < 0.0 || options.drop_tolerance < 0.0) {
    result.error = "invalid sparse Kron options";
    return result;
  }

  const auto start = std::chrono::steady_clock::now();
  MutableSparseMatrix matrix(ybus, options.drop_tolerance);
  using QueueEntry = std::pair<int, int>;
  std::priority_queue<QueueEntry, std::vector<QueueEntry>,
                      std::greater<QueueEntry>> queue;
  for (int node = 0; node < matrix.size(); ++node) {
    if (eligible[static_cast<std::size_t>(node)]) {
      queue.emplace(matrix.front_size(node), node);
    }
  }

  while (!queue.empty()) {
    const auto [queued_front, node] = queue.top();
    queue.pop();
    if (!matrix.active(node) || !eligible[static_cast<std::size_t>(node)]) {
      continue;
    }
    const int actual_front = matrix.front_size(node);
    if (actual_front != queued_front) {
      queue.emplace(actual_front, node);
      continue;
    }
    if (actual_front > options.max_front ||
        std::abs(matrix.value(node, node)) <= options.pivot_tolerance) {
      continue;
    }
    const long long predicted = matrix.predicted_nonzeros_after_elimination(node);
    const long long fill_cap = static_cast<long long>(std::ceil(
        options.max_nnz_ratio * static_cast<double>(matrix.initial_nonzeros())));
    if (predicted > fill_cap) continue;

    const auto affected_rows = matrix.row_neighbors(node);
    const auto affected_cols = matrix.col_neighbors(node);
    result.max_front = std::max(result.max_front, actual_front);
    result.recovery_steps.push_back(matrix.eliminate(node));
    for (int neighbor : affected_rows) {
      if (matrix.active(neighbor) && eligible[static_cast<std::size_t>(neighbor)]) {
        queue.emplace(matrix.front_size(neighbor), neighbor);
      }
    }
    for (int neighbor : affected_cols) {
      if (matrix.active(neighbor) && eligible[static_cast<std::size_t>(neighbor)]) {
        queue.emplace(matrix.front_size(neighbor), neighbor);
      }
    }
  }

  result.reduced = matrix.retained_matrix(result.retained);
  result.elapsed_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - start).count();
  return result;
}

Eigen::VectorXcd recover_sparse_kron_state(
    const SparseKronResult& reduction,
    const Eigen::VectorXcd& retained_state) {
  if (!reduction.valid()) {
    throw std::invalid_argument("cannot recover from an invalid sparse Kron result");
  }
  if (retained_state.size() != static_cast<int>(reduction.retained.size())) {
    throw std::invalid_argument("sparse Kron retained-state dimension mismatch");
  }
  Eigen::VectorXcd full = Eigen::VectorXcd::Zero(reduction.original_size);
  for (int pos = 0; pos < static_cast<int>(reduction.retained.size()); ++pos) {
    full[reduction.retained[static_cast<std::size_t>(pos)]] = retained_state[pos];
  }
  for (auto it = reduction.recovery_steps.rbegin();
       it != reduction.recovery_steps.rend(); ++it) {
    Complex value{};
    for (const auto& [neighbor, coefficient] : it->coefficients) {
      value += coefficient * full[neighbor];
    }
    full[it->node] = value;
  }
  return full;
}

Eigen::SparseMatrix<Complex> sparse_kron_recovery_operator(
    const SparseKronResult& reduction,
    double drop_tolerance) {
  if (!reduction.valid()) {
    throw std::invalid_argument(
        "cannot materialize recovery for an invalid sparse Kron result");
  }
  const int nr = static_cast<int>(reduction.retained.size());
  std::vector<Eigen::Triplet<Complex>> triplets;
  for (int col = 0; col < nr; ++col) {
    Eigen::VectorXcd basis = Eigen::VectorXcd::Zero(nr);
    basis[col] = Complex{1.0, 0.0};
    const Eigen::VectorXcd full = recover_sparse_kron_state(reduction, basis);
    for (int row = 0; row < full.size(); ++row) {
      if (std::abs(full[row]) > drop_tolerance) {
        triplets.emplace_back(row, col, full[row]);
      }
    }
  }
  Eigen::SparseMatrix<Complex> recovery(reduction.original_size, nr);
  recovery.setFromTriplets(triplets.begin(), triplets.end());
  recovery.makeCompressed();
  return recovery;
}

}  // namespace hacdcpf::graph
