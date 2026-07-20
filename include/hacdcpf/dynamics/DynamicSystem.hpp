#pragma once

#include <complex>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Sparse>
#include <Eigen/SparseLU>

#include "hacdcpf/dynamics/DynamicEvent.hpp"
#include "hacdcpf/dynamics/DynamicSolverOptions.hpp"
#include "hacdcpf/dynamics/DynamicState.hpp"
#include "hacdcpf/dynamics/DynamicStamp.hpp"
#include "hacdcpf/dynamics/NetworkState.hpp"
#include "hacdcpf/dynamics/devices/DynamicDevice.hpp"
#include "hacdcpf/model/hybrid_power_system.hpp"
#include "hacdcpf/power_flow/power_flow_result.hpp"

namespace hacdcpf::dynamics {

struct DynamicACBranch {
  int index{0};
  int from_bus{0};
  int to_bus{0};
  int from_pos{-1};
  int to_pos{-1};
  bool in_service{true};
  Eigen::Matrix3cd y_ff{Eigen::Matrix3cd::Zero()};
  Eigen::Matrix3cd y_ft{Eigen::Matrix3cd::Zero()};
  Eigen::Matrix3cd y_tf{Eigen::Matrix3cd::Zero()};
  Eigen::Matrix3cd y_tt{Eigen::Matrix3cd::Zero()};
};

struct DynamicDCBranch {
  int index{0};
  int from_bus{0};
  int to_bus{0};
  int from_pos{-1};
  int to_pos{-1};
  bool in_service{true};
  double conductance_pu{0.0};
};

struct DynamicFaultShunt {
  int bus{0};
  int bus_pos{-1};
  bool is_ac{true};
  bool active{true};
  // Faulted phase for unbalanced (SLG/LL) faults: -1 applies the shunt to all
  // three phases (balanced), 0/1/2 applies it to a single phase (design doc
  // §17 unbalanced fault stamps).
  int phase{-1};
  double g_pu{0.0};
  double b_pu{0.0};
  double clear_time_s{0.0};
};

struct DynamicACBusLoad {
  int bus{0};
  int bus_pos{-1};
  double p_mw{0.0};
  double q_mvar{0.0};
  double nominal_voltage_pu{1.0};
  double scale{1.0};
};

struct DynamicDCBusLoad {
  int bus{0};
  int bus_pos{-1};
  double p_mw{0.0};
  double scale{1.0};
};

class DynamicNetwork {
 public:
  using Complex = std::complex<double>;

  double base_mva{100.0};
  double frequency_hz{50.0};

  std::vector<int> ac_bus_ids;
  std::vector<int> dc_bus_ids;
  std::vector<DynamicACBranch> ac_branches;
  std::vector<DynamicDCBranch> dc_branches;
  std::vector<DynamicFaultShunt> fault_shunts;
  std::vector<DynamicACBusLoad> ac_bus_loads;
  std::vector<DynamicDCBusLoad> dc_bus_loads;

  std::unordered_map<int, int> ac_bus_pos_by_id;
  std::unordered_map<int, int> dc_bus_pos_by_id;

  Eigen::SparseMatrix<Complex> Yac_base;
  Eigen::SparseMatrix<double> Gdc_base;

  [[nodiscard]] int acPhaseNodeCount() const noexcept {
    return static_cast<int>(ac_bus_ids.size()) * 3;
  }

  [[nodiscard]] int dcBusCount() const noexcept {
    return static_cast<int>(dc_bus_ids.size());
  }

  [[nodiscard]] int acPhaseNode(int bus_pos, int phase) const noexcept {
    return 3 * bus_pos + phase;
  }

  [[nodiscard]] int acBusPosition(int bus_id) const;
  [[nodiscard]] int dcBusPosition(int bus_id) const;

  void rebuildBaseMatrices(double singular_regularization_pu);
  void assembleEffectiveMatrices(
      const DynamicStamp& stamp,
      double singular_regularization_pu,
      Eigen::SparseMatrix<Complex>& Yac_eff,
      Eigen::VectorXcd& Iac_eff,
      Eigen::SparseMatrix<double>& Gdc_eff,
      Eigen::VectorXd& Idc_eff) const;
};

// Cached sparse factorizations of the effective network matrices. For the
// current device set the effective admittance is state-independent (device
// Norton stamps contribute constant admittance and only voltage-dependent
// *current* injections), so the matrix is unchanged between events and can be
// factored once and reused across every algebraic iteration, RHS evaluation, and
// step. The cached matrix is compared to the freshly assembled one each call, so
// correctness is preserved even if a device ever stamps a varying admittance
// (the factorization is simply refreshed when the matrix actually changes).
struct NetworkSolveCache {
  Eigen::SparseLU<Eigen::SparseMatrix<std::complex<double>>> ac_lu;
  Eigen::SparseLU<Eigen::SparseMatrix<double>> dc_lu;
  Eigen::SparseMatrix<std::complex<double>> ac_matrix;
  Eigen::SparseMatrix<double> dc_matrix;
  std::vector<Eigen::Triplet<std::complex<double>>> ac_stamp_triplets;
  std::vector<Eigen::Triplet<double>> dc_stamp_triplets;
  bool ac_valid{false};
  bool dc_valid{false};
};

struct DynamicSystem {
  HybridPowerSystem canonical_system;
  PowerFlowResult initial_power_flow;
  DynamicInitializationSummary initialization;
  DynamicNetwork network;
  std::vector<std::unique_ptr<DynamicDevice>> devices;
  std::vector<DynamicEvent> events;
  DynamicState x;
  NetworkState y;
  DynamicSolverOptions options;
  std::vector<std::string> warnings;
  std::unique_ptr<NetworkSolveCache> network_cache;

  // MassMatrixDae effective-admittance cache. The effective Yac/Gdc are
  // state-independent between topology events, so they are assembled once and
  // reused across the many residual evaluations of each Newton/adaptive step
  // instead of rebuilding the sparse matrices (setFromTriplets) on every call.
  // Invalidated whenever an event is applied.
  bool dae_admittance_valid{false};
  Eigen::SparseMatrix<std::complex<double>> dae_yac_cache;
  Eigen::SparseMatrix<double> dae_gdc_cache;

  void assignStateIndices();
  void initializeStatesFromPowerFlow();
  [[nodiscard]] bool solveNetwork(double t, std::string& error);
  [[nodiscard]] bool evaluateDerivatives(double t,
                                         const Eigen::VectorXd& state,
                                         Eigen::VectorXd& dxdt,
                                         std::string& error);
  /// Evaluate the semi-explicit DAE residual at an arbitrary snapshot without
  /// solving the algebraic equations:
  ///   r_f = xdot - f(t, x, y),
  ///   r_g = [Re(Iac-Yac*Vac), Im(Iac-Yac*Vac), Idc-Gdc*Vdc].
  /// The method does not mutate the system state. The caller is responsible for
  /// applying all topology/device events active at `t` before evaluation.
  [[nodiscard]] bool evaluateDaeResidual(
      double t, const Eigen::VectorXd& state, const NetworkState& algebraic,
      const Eigen::VectorXd& state_derivative, Eigen::VectorXd& r_f,
      Eigen::VectorXd& r_g, std::string& error) const;
  [[nodiscard]] double derivativeInfinityNorm(double t, std::string& error);
  [[nodiscard]] int stateCount() const noexcept { return x.size(); }

 private:
  // Solve A x = b reusing a cached factorization when A is unchanged. Falls back
  // to a fresh per-call solve for non-default linear-solver backends.
  bool acSolveCached(const Eigen::SparseMatrix<std::complex<double>>& a,
                     const Eigen::VectorXcd& b, Eigen::VectorXcd& x,
                     std::string& error);
  bool dcSolveCached(const Eigen::SparseMatrix<double>& a, const Eigen::VectorXd& b,
                     Eigen::VectorXd& x, std::string& error);

  // Finite-difference Newton solve of the algebraic network residual
  // g(V) = I_inj(V) - Y_eff*V = 0 with device differential states frozen. Used as
  // a robustness fallback when the Gauss/Picard fixed-point in solveNetwork()
  // stalls short of tolerance.
  bool solveNetworkNewton(double t, std::string& error);
};

}  // namespace hacdcpf::dynamics
