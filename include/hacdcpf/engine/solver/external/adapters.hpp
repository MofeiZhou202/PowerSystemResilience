#pragma once
// Forwarding header: hacdcpf::engine external solver adapters → mipsolvers::engine
#include <mipsolvers/engine/solver/external/adapters.hpp>

namespace hacdcpf::engine {
  using mipsolvers::engine::HighsAdapter;
  using mipsolvers::engine::IpoptAdapter;
  using mipsolvers::engine::ScipAdapter;
  using mipsolvers::engine::GurobiAdapter;
  using mipsolvers::engine::GurobiOptions;
  using mipsolvers::engine::CplexAdapter;
  using mipsolvers::engine::CplexOptions;
  using mipsolvers::engine::CplexSolveInfo;
  using mipsolvers::engine::last_cplex_solve_info;
}
