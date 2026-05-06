#pragma once
#include "hacdcpf/solver/solver_adapter.hpp"
#include "hacdcpf/engine/solver/external/adapters.hpp"
namespace hacdcpf::solver {
using hacdcpf::engine::HighsAdapter;
using hacdcpf::engine::IpoptAdapter;
using hacdcpf::engine::ScipAdapter;
using hacdcpf::engine::GurobiAdapter;
}  // namespace hacdcpf::solver
