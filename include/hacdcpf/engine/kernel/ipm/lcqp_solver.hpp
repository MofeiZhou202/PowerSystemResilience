#pragma once
// Forwarding header: hacdcpf::engine LCQP solver → mipsolvers::engine
#include <mipsolvers/engine/kernel/ipm/lcqp_solver.hpp>

namespace hacdcpf::engine {
  using mipsolvers::engine::LCQPOptions;
  using mipsolvers::engine::NativeLCQPAdapter;
}
