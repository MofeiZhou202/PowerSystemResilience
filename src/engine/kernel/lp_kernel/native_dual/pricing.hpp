#pragma once

#include "state.hpp"

namespace mipsolvers::engine::native_dual::detail {

bool choose_leaving(State& state, Leaving& leaving, std::string& failure);
bool choose_entering_bfrt(const State& state, const Leaving& leaving,
                          const Eigen::VectorXd& pivot_row,
                          PivotTransaction& transaction,
                          std::string& failure);
bool compute_dse_weights(const State& state, const Leaving& leaving,
                         const Eigen::VectorXd& pivot_row,
                         const Eigen::VectorXd& direction, double pivot,
                         std::vector<double>& updated,
                         bool& restart_devex,
                         std::string& failure);

}  // namespace mipsolvers::engine::native_dual::detail
