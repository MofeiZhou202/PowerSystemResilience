#pragma once

#include <memory>

#include "mipsolvers/power_flow/jacobian_builder.hpp"
#include "mipsolvers/engine/kernel/linear_algebra/linear_solver.hpp"
#include "mipsolvers/power_flow/solver_data.hpp"
#include "mipsolvers/model/options.hpp"
#include "mipsolvers/model/results.hpp"

namespace mipsolvers::engine {

using mipsolvers::powerflow::SolverData;
using mipsolvers::powerflow::JacobianContext;
using mipsolvers::powerflow::JacobianPattern;

class NewtonSolver {
 public:
  PowerFlowResult solve(const SolverData& data,
                        const PowerFlowOptions& opt,
                        const InitialState* init = nullptr) const;

 private:
  struct PatternCache {
    bool valid{false};
    const SolverData* data_ptr{nullptr};
    std::uint64_t data_build_id{0};
    const void* ybus_value_ptr{nullptr};
    const void* ybus_outer_ptr{nullptr};
    const void* ybus_inner_ptr{nullptr};
    const void* gdc_value_ptr{nullptr};
    const void* gdc_outer_ptr{nullptr};
    const void* gdc_inner_ptr{nullptr};
    int ybus_rows{0};
    int ybus_cols{0};
    int ybus_nnz{0};
    int gdc_rows{0};
    int gdc_cols{0};
    int gdc_nnz{0};
    JacobianContext ctx;
    JacobianPattern pattern;
    std::unique_ptr<SparseLinearSolver> solver;
  };

  mutable PatternCache cache_;
};

}  // namespace mipsolvers::engine
