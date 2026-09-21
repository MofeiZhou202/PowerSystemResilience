#include "mipsolvers/l2o/scuc_warm_start.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace mipsolvers::l2o {
namespace {

bool has_shape(const scuc::Matrix2D& matrix, int rows, int cols) {
  if (static_cast<int>(matrix.size()) != rows) return false;
  for (const auto& row : matrix) {
    if (static_cast<int>(row.size()) != cols) return false;
  }
  return true;
}

void require_shape(const scuc::Matrix2D& matrix,
                   int rows,
                   int cols,
                   const char* name) {
  if (!has_shape(matrix, rows, cols)) {
    throw std::invalid_argument(std::string(name) + " must have shape [ng][T]");
  }
}

int rounded_binary(double value,
                   double threshold,
                   int& out_of_range_predictions) {
  if (value < -1e-9 || value > 1.0 + 1e-9) ++out_of_range_predictions;
  return value >= threshold ? 1 : 0;
}

bool set_col(Eigen::VectorXd& x, int col, double value, int& missing_cols) {
  if (col < 0 || col >= x.size()) {
    ++missing_cols;
    return false;
  }
  x[col] = value;
  return true;
}

}  // namespace

SCUCWarmStartResult make_scuc_commitment_warm_start(
    const engine::MIPModel& mip,
    const scuc::Matrix2D& commitment,
    const scuc::Matrix2D& startup,
    const scuc::Matrix2D& shutdown,
    const SCUCWarmStartOptions& options) {
  if (!mip.uc_hint) {
    throw std::invalid_argument("SCUC warm start requires MIPModel::UCGenHint");
  }

  const auto& hint = *mip.uc_hint;
  const int ng = hint.ng;
  const int T = hint.T;
  const int n = static_cast<int>(mip.linear_part.c.size());
  if (ng <= 0 || T <= 0 || n <= 0) {
    throw std::invalid_argument("SCUC warm start requires positive ng, T, and model dimension");
  }
  require_shape(commitment, ng, T, "commitment");
  if (!startup.empty()) require_shape(startup, ng, T, "startup");
  if (!shutdown.empty()) require_shape(shutdown, ng, T, "shutdown");

  const double threshold = std::clamp(options.binary_threshold, 0.0, 1.0);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(n);
  bool used_existing_seed = false;
  if (options.initialize_from_existing_seed && mip.initial_solution.size() == n) {
    x = mip.initial_solution;
    used_existing_seed = true;
  }

  SCUCWarmStartReport report;
  report.success = true;
  report.message = "ok";
  report.ng = ng;
  report.T = T;
  report.used_existing_seed = used_existing_seed;
  report.inferred_startup = startup.empty() && options.infer_missing_transitions;
  report.inferred_shutdown = shutdown.empty() && options.infer_missing_transitions;

  for (int g = 0; g < ng; ++g) {
    int previous_commitment = 0;
    if (g < static_cast<int>(hint.ig0.size())) previous_commitment = hint.ig0[static_cast<size_t>(g)] != 0 ? 1 : 0;
    for (int t = 0; t < T; ++t) {
      const size_t matrix_pos = static_cast<size_t>(g);
      const size_t time_pos = static_cast<size_t>(t);
      const size_t col_pos = static_cast<size_t>(t * ng + g);

      const int ig = rounded_binary(commitment[matrix_pos][time_pos], threshold,
                                    report.num_out_of_range_predictions);
      if (col_pos < hint.ig_cols.size() && set_col(x, hint.ig_cols[col_pos], ig, report.num_missing_cols)) {
        ++report.num_ig_set;
      } else if (col_pos >= hint.ig_cols.size()) {
        ++report.num_missing_cols;
      }

      int su = 0;
      if (!startup.empty()) {
        su = rounded_binary(startup[matrix_pos][time_pos], threshold,
                            report.num_out_of_range_predictions);
      } else if (options.infer_missing_transitions) {
        su = std::max(0, ig - previous_commitment);
      }
      if (col_pos < hint.su_cols.size() && set_col(x, hint.su_cols[col_pos], su, report.num_missing_cols)) {
        ++report.num_su_set;
      } else if (col_pos >= hint.su_cols.size()) {
        ++report.num_missing_cols;
      }

      int sd = 0;
      if (!shutdown.empty()) {
        sd = rounded_binary(shutdown[matrix_pos][time_pos], threshold,
                            report.num_out_of_range_predictions);
      } else if (options.infer_missing_transitions) {
        sd = std::max(0, previous_commitment - ig);
      }
      if (col_pos < hint.sd_cols.size() && set_col(x, hint.sd_cols[col_pos], sd, report.num_missing_cols)) {
        ++report.num_sd_set;
      } else if (col_pos >= hint.sd_cols.size()) {
        ++report.num_missing_cols;
      }

      previous_commitment = ig;
    }
  }

  return {std::move(x), report};
}

void apply_scuc_commitment_warm_start(
    engine::MIPModel& mip,
    const scuc::Matrix2D& commitment,
    const scuc::Matrix2D& startup,
    const scuc::Matrix2D& shutdown,
    const SCUCWarmStartOptions& options) {
  auto result = make_scuc_commitment_warm_start(mip, commitment, startup, shutdown, options);
  mip.initial_solution = std::move(result.x);
}

}  // namespace mipsolvers::l2o