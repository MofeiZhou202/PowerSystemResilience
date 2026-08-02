#pragma once

#include <Eigen/Core>
#include <Eigen/Sparse>

#include "hacdcpf/assembly/solver_data.hpp"

#include <cstdint>
#include <unordered_map>
#include <vector>

namespace hacdcpf::powerflow {

struct JacobianContext {
  int n{0};
  int ndc{0};
  int np{0};
  int nq{0};
  int ndc_eq{0};
  int nvar{0};

  std::vector<int> non_slack;
  std::vector<int> pq;
  std::vector<int> dc_non_slack;

  std::vector<int> p_row;
  std::vector<int> q_row;
  std::vector<int> dc_row;

  std::vector<int> va_col;
  std::vector<int> vm_col;
  std::vector<int> vdc_col;

  // Augmented equation tracking (Direction 2).
  // Q-rows replaced with Vm setpoint equations.
  std::vector<int> augmented_q_buses;
  std::vector<double> augmented_q_targets;
  // DC-rows replaced with Vdc setpoint equations.
  std::vector<int> augmented_dc_buses;
  std::vector<double> augmented_dc_targets;

  // Semi-smooth Newton NCP bus data (Direction 3).
  struct NCPBusData {
    int bus{0};
    double qmin{0.0};
    double qmax{0.0};
    double vm_set{1.0};
  };
  std::vector<NCPBusData> ncp_buses;

  /// Voltage magnitude floor for the Pcalc/V and Qcalc/V diagonal formulas.
  /// Above this floor the analytic Jacobian is exact. Below it the floor is a
  /// deliberate finite-value safeguard and the Jacobian is not the exact
  /// derivative of the unclamped physical residual.
  /// Populated from RobustNonlinearOptions::min_vm_pu by the Newton solver.
  double min_vm_pu{1e-8};
};

struct JacobianPattern {
  struct ACEntry {
    int i{0};
    int j{0};
    int p_va_nz{-1};
    int p_vm_nz{-1};
    int q_va_nz{-1};
    int q_vm_nz{-1};
    double g{0.0};
    double b{0.0};
  };

  struct DCEntry {
    int i{0};
    int j{0};
    int nz{-1};
    double g{0.0};
    bool diagonal{false};
  };

  /// Cross-coupling entry for a VSC converter (AC↔DC Jacobian).
  struct CouplingEntry {
    int conv_index{0};
    int ac_bus{0};
    int dc_bus{0};
    int p_vdc_nz{-1};   // P-row(ac_bus) → Vdc-col(dc_bus)
    int q_vdc_nz{-1};   // Q-row(ac_bus) → Vdc-col(dc_bus)
    int dc_vdc_nz{-1};  // DC-row(dc_bus) → Vdc-col(dc_bus)  (converter contribution)
    int dc_vm_nz{-1};   // DC-row(dc_bus) → Vm-col(ac_bus)   (AC-resistance cross-coupling)
  };

  /// Derivative of an AC grid-forming converter's DC energy-balance row with
  /// respect to a variable on the converter's AC network row. The GFM AC bus
  /// itself is a slack bus, but its network injection depends on every
  /// non-slack voltage adjacent through Ybus.
  struct GFMNetworkCouplingEntry {
    int conv_index{0};
    int ac_bus{0};
    int dc_bus{0};
    int neighbor_bus{0};
    int dc_va_nz{-1};
    int dc_vm_nz{-1};
    double g{0.0};
    double b{0.0};
  };

  /// Cross-coupling entry for a DCDC converter with I²R loss.
  struct DCDCCouplingEntry {
    int dcdc_index{0};
    int bus_in{0};
    int bus_out{0};
    int dc_out_vdc_in_nz{-1};   // DC-row(bus_out) → Vdc-col(bus_in)  [I²R cross-coupling]
  };

  /// Always-on droop entry for a DC-DC converter in Droop mode.
  /// Carries ∂p_out/∂Vdc_out = k_droop (output diagonal) and
  /// ∂(-p_in)/∂Vdc_out = -k_droop/eta (input-bus cross-column).
  struct DCDroopEntry {
    int dcdc_index{0};
    int bus_in{0};
    int bus_out{0};
    double k_droop{0.0};
    double eta{1.0};
    int dc_out_vdc_out_nz{-1};  // DC-row(bus_out) → Vdc-col(bus_out): spec deriv = k_droop
    int dc_in_vdc_out_nz{-1};   // DC-row(bus_in)  → Vdc-col(bus_out): spec deriv = -k_droop/eta
  };

  /// Always-on coupling entry for an LCC station (quasi-steady, dat manual
  /// ch.4).  The station's AC P/Q injections at the valve-side bus and its DC
  /// injection depend on both the valve-side AC voltage (through U_d0) and
  /// the DC terminal voltage; these derivatives are required for the Newton
  /// iteration to converge with stiff CEA characteristics, so the entries
  /// are built regardless of enable_coupled_jacobian.
  struct LCCEntry {
    int lcc_index{0};
    int ac_bus{0};
    int commutation_ac_bus{0};
    int dc_bus{0};
    int p_vm_nz{-1};    // P-row(ac_bus) → Vm-col(ac_bus)
    int q_vm_nz{-1};    // Q-row(ac_bus) → Vm-col(ac_bus)
    int p_vdc_nz{-1};   // P-row(ac_bus) → Vdc-col(dc_bus)
    int q_vdc_nz{-1};   // Q-row(ac_bus) → Vdc-col(dc_bus)
    int dc_vm_nz{-1};   // DC-row(dc_bus) → Vm-col(ac_bus)
    int p_vm_comm_nz{-1};   // P-row(ac_bus) → Vm-col(commutation_ac_bus)
    int q_vm_comm_nz{-1};   // Q-row(ac_bus) → Vm-col(commutation_ac_bus)
    int dc_vm_comm_nz{-1};  // DC-row(dc_bus) → Vm-col(commutation_ac_bus)
  };

  Eigen::SparseMatrix<double> matrix;
  std::unordered_map<std::uint64_t, int> entry_to_nz;
  std::vector<ACEntry> ac_entries;
  std::vector<DCEntry> dc_entries;
  std::vector<CouplingEntry> coupling_entries;
  std::vector<GFMNetworkCouplingEntry> gfm_network_coupling_entries;
  std::vector<DCDCCouplingEntry> dcdc_coupling_entries;
  std::vector<DCDroopEntry> dcdc_droop_entries;  ///< always built; droop diagonal + cross-term
  std::vector<LCCEntry> lcc_entries;  ///< always built; LCC AC↔DC coupling
  std::vector<int> p_va_diag_nz;
  std::vector<int> p_vm_diag_nz;
  std::vector<int> q_va_diag_nz;
  std::vector<int> q_vm_diag_nz;
  /// DC-row(bus) → Vdc-col(bus) nz, indexed by DC bus (0-based).
  /// Always filled. Used to inject ∂pdc_spec/∂vdc for VDC-mode VSC converters
  /// without requiring enable_coupled_jacobian.
  std::vector<int> dc_vdc_diag_nz;
  std::vector<double> g_diag;
  std::vector<double> b_diag;
  bool analyzed{false};
};

JacobianPattern build_jacobian_pattern(const SolverData& data, const JacobianContext& ctx);

double evaluate_residual_and_jacobian(const SolverData& data,
                                      const JacobianContext& ctx,
                                      const std::vector<ACBus>& ac_buses,
                                      const std::vector<VSCConverter>& converters,
                                      const Eigen::VectorXd& pg,
                                      const Eigen::VectorXd& qg,
                                      const Eigen::VectorXd& vm,
                                      const Eigen::VectorXd& va,
                                      const Eigen::VectorXd& vdc,
                                      Eigen::VectorXd& pcalc,
                                      Eigen::VectorXd& qcalc,
                                      Eigen::VectorXd& p_spec,
                                      Eigen::VectorXd& q_spec,
                                      Eigen::VectorXd& pdc_linear,
                                      Eigen::VectorXd& pdc_calc,
                                      Eigen::VectorXd& pdc_spec,
                                      Eigen::VectorXd& mismatch,
                                      JacobianPattern& pattern,
                                      int ac_eval_threads);

double evaluate_residual_only(const SolverData& data,
                              const JacobianContext& ctx,
                              const std::vector<ACBus>& ac_buses,
                              const std::vector<VSCConverter>& converters,
                              const Eigen::VectorXd& pg,
                              const Eigen::VectorXd& qg,
                              const Eigen::VectorXd& vm,
                              const Eigen::VectorXd& va,
                              const Eigen::VectorXd& vdc,
                              Eigen::VectorXd& pcalc,
                              Eigen::VectorXd& qcalc,
                              Eigen::VectorXd& p_spec,
                              Eigen::VectorXd& q_spec,
                              Eigen::VectorXd& pdc_linear,
                              Eigen::VectorXd& pdc_calc,
                              Eigen::VectorXd& pdc_spec,
                              Eigen::VectorXd& mismatch,
                              const JacobianPattern& pattern,
                              int ac_eval_threads);

}  // namespace hacdcpf::powerflow
