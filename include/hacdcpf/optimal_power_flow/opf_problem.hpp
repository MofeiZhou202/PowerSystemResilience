#pragma once

/// optimal_power_flow/opf_problem.hpp
/// ====================================
/// Self-contained OPF problem descriptor, decoupling "what to optimize"
/// from "how to solve it".  Produced by the projection layer and consumed
/// by any IOPFSolver implementation.

#include "hacdcpf/model/hybrid_power_system.hpp"
#include "hacdcpf/optimal_power_flow/opf_options.hpp"

namespace hacdcpf::opf {

/// A discrete, self-contained description of one OPF solve task.
struct OPFProblem {
  const HybridPowerSystem* sys{nullptr};  ///< Non-owning pointer to the system.
  ACOPFOptions ac_opts;
  DCOPFOptions dc_opts;

  bool is_ac{true};   ///< True = AC OPF; false = DC OPF.
  bool is_dc{false};
};

}  // namespace hacdcpf::opf
