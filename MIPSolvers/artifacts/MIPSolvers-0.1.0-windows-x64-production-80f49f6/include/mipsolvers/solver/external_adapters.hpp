#pragma once
#include "mipsolvers/solver/solver_adapter.hpp"
#include "mipsolvers/engine/solver/external/adapters.hpp"
namespace mipsolvers::solver {
using mipsolvers::engine::HighsAdapter;
using mipsolvers::engine::IpoptAdapter;
using mipsolvers::engine::ScipAdapter;
using mipsolvers::engine::GurobiAdapter;
}  // namespace mipsolvers::solver
