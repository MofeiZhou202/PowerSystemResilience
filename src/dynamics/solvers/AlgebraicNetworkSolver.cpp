#include "hacdcpf/dynamics/solvers/AlgebraicNetworkSolver.hpp"

#include <cmath>

namespace hacdcpf::dynamics {

AlgebraicNetworkSolveResult AlgebraicNetworkSolver::solve(DynamicSystem& system,
                                                          double t) const {
  DynamicStamp stamp(system.network.acPhaseNodeCount(), system.network.dcBusCount());
  for (const auto& device : system.devices) {
    device->stamp(t, system.x, system.y, stamp);
  }
  return solve(system, stamp);
}

AlgebraicNetworkSolveResult AlgebraicNetworkSolver::solve(DynamicSystem& system,
                                                          const DynamicStamp& stamp) const {
  using Complex = std::complex<double>;
  Eigen::SparseMatrix<Complex> yac_eff;
  Eigen::SparseMatrix<double> gdc_eff;
  Eigen::VectorXcd iac_eff;
  Eigen::VectorXd idc_eff;
  system.network.assembleEffectiveMatrices(stamp,
                                           system.options.singular_regularization_pu,
                                           yac_eff,
                                           iac_eff,
                                           gdc_eff,
                                           idc_eff);
  if (system.network.acPhaseNodeCount() > 0) {
    Eigen::VectorXcd v;
    const auto solve_result = linear_solver_.solve(yac_eff, iac_eff, v);
    if (!solve_result.success) return {false, solve_result.message};
    system.y.Vac_abc = v;
    system.y.Iac_abc = iac_eff;
  }
  if (system.network.dcBusCount() > 0) {
    Eigen::VectorXd v;
    const auto solve_result = linear_solver_.solve(gdc_eff, idc_eff, v);
    if (!solve_result.success) return {false, solve_result.message};
    for (Eigen::Index i = 0; i < v.size(); ++i) {
      system.y.Vdc[i] = std::isfinite(v[i]) ? v[i] : 1.0;
    }
    system.y.Idc = idc_eff;
  }
  for (auto& device : system.devices) {
    device->updateAlgebraicOutputs(system.x, system.y);
  }
  return {true, {}};
}

}  // namespace hacdcpf::dynamics

