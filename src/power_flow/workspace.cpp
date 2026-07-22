#include "hacdcpf/power_flow/workspace.hpp"

#include <algorithm>

namespace hacdcpf::powerflow {

void SolverWorkspace::prepare_state(int ac_bus_count, int dc_bus_count) {
  nac = std::max(0, ac_bus_count);
  ndc = std::max(0, dc_bus_count);
  vm.setOnes(nac);
  va.setZero(nac);
  vdc.setOnes(ndc);
}

void SolverWorkspace::prepare_equations(int p_equations, int q_equations,
                                        int dc_equations, int extra_equations) {
  np = std::max(0, p_equations);
  nq = std::max(0, q_equations);
  ndc_eq = std::max(0, dc_equations);
  nf = std::max(0, extra_equations);
  residual.setZero(equation_count());
  dx.setZero(equation_count());
}

void SolverWorkspace::reset_iteration() noexcept {
  residual.setZero();
  dx.setZero();
  if (jacobian.nonZeros() > 0) {
    std::fill(jacobian.valuePtr(),
              jacobian.valuePtr() + jacobian.nonZeros(), 0.0);
  }
}

}  // namespace hacdcpf::powerflow
