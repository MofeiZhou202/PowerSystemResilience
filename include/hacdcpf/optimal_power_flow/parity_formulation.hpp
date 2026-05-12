#pragma once

#include <vector>

#include <Eigen/Core>
#include <Eigen/Sparse>

#include "hacdcpf/core/solver_data.hpp"
#include "hacdcpf/model/system.hpp"

namespace hacdcpf::opf::parity {

struct ParityOptions {
  bool load_shedding{true};
  double voll{0.0};     // 0.0 => auto
  double eps_iac{1e-6}; // smoothing in |Iac|
};

struct VarIndex {
  int n_va{0};
  int n_vm{0};
  int n_pg{0};
  int n_qg{0};
  int n_vdc{0};
  int n_pac{0};
  int n_qac{0};
  int n_pdc{0};
  int n_dpd{0};
  int n_dqd{0};
  // --- Enhanced component variables ---
  int n_pren{0};   // curtailable renewable active power (RenewableGen + PVSystem)
  int n_qren{0};   // curtailable renewable reactive power
  int n_pstor{0};  // storage active power dispatch
  int n_qstor{0};  // storage reactive power dispatch
  int n_pstordc{0}; // DC storage active power dispatch
  int n_pdcdc{0};  // DCDC converter power (input-side)
  int n_pflex{0};  // flexible load adjustment
  int n_total{0};

  int i_va{0};
  int i_vm{0};
  int i_pg{0};
  int i_qg{0};
  int i_vdc{0};
  int i_pac{0};
  int i_qac{0};
  int i_pdc{0};
  int i_dpd{0};
  int i_dqd{0};
  // --- Enhanced component variable offsets ---
  int i_pren{0};
  int i_qren{0};
  int i_pstor{0};
  int i_qstor{0};
  int i_pstordc{0};
  int i_pdcdc{0};
  int i_pflex{0};
};

struct ConstraintIndex {
  int n_pbal_ac{0};
  int n_qbal_ac{0};
  int n_pbal_dc{0};
  int n_conv_bal{0};
  int n_dcdc_bal{0};   // DCDC converter power balance
  int n_er_bal{0};     // EnergyRouter port power balance
  int n_eq_total{0};

  int i_pbal_ac{0};
  int i_qbal_ac{0};
  int i_pbal_dc{0};
  int i_conv_bal{0};
  int i_dcdc_bal{0};
  int i_er_bal{0};

  int n_sf{0};
  int n_st{0};
  int n_sconv{0};
  int n_sdc{0};        // DC branch power flow limit constraints
  int n_ineq_nonlin{0};
};

struct Problem {
  core::SolverData data;
  ParityOptions options;
  VarIndex vidx;
  ConstraintIndex cidx;

  std::vector<int> gen_var_to_data;
  std::vector<int> conv_var_to_data;
  std::vector<int> branch_limited;
  std::vector<int> dc_branch_limited;  // DC branches with rate_a_mva > 0

  std::vector<int> gen_bus;
  std::vector<int> conv_ac_bus;
  std::vector<int> conv_dc_bus;

  // --- Enhanced component mappings ---
  // Curtailable renewables: merged RenewableGen (curtailable=true) + PVSystem (controllable=true)
  // ren_source: 0 = RenewableGen, 1 = PVSystem
  std::vector<int> ren_var_to_data;
  std::vector<int> ren_source;
  std::vector<int> ren_bus;

  // Storage units
  std::vector<int> stor_var_to_data;
  std::vector<int> stor_bus;
  std::vector<int> stor_dc_var_to_data;
  std::vector<int> stor_dc_bus;

  // DCDC converters
  std::vector<int> dcdc_var_to_data;
  std::vector<int> dcdc_bus_in;   // DC bus index (0-based)
  std::vector<int> dcdc_bus_out;  // DC bus index (0-based)

  // Flexible loads
  std::vector<int> flex_var_to_data;
  std::vector<int> flex_bus;

  // EnergyRouter port info (flattened across all routers)
  struct ERPortInfo {
    int router_idx;
    int port_idx;
    int bus;        // 0-based bus index
    bool is_ac;     // true = AC port, false = DC port
  };
  std::vector<ERPortInfo> er_ports;

  double scale_p{1.0};
  double scale_q{1.0};
  double voll_effective{100.0};

  // Per-bus demand (p.u.), aggregated from Load table or bus-level pd_mw.
  // Accounts for project_to_canonical_models() moving bus.pd_mw into the
  // Load table (which zeroes bus.pd_mw).  Pre-computed in build_problem.
  Eigen::VectorXd pd_demand_pu;
  Eigen::VectorXd qd_demand_pu;

  // Per-bus ZIP load coefficients (constant-power / constant-current /
  // constant-impedance weights).  The voltage-dependent demand is:
  //   Pd_eff(V) = pd_demand_pu[i] * (zip_pp[i] + zip_ip[i]*V + zip_zp[i]*V^2)
  //   Qd_eff(V) = qd_demand_pu[i] * (zip_pq[i] + zip_iq[i]*V + zip_zq[i]*V^2)
  // When loads are purely constant-power, zip_pp=1, zip_ip=zip_zp=0 (default).
  Eigen::VectorXd zip_pp;   // constant-power weight, P
  Eigen::VectorXd zip_ip;   // constant-current weight, P
  Eigen::VectorXd zip_zp;   // constant-impedance weight, P
  Eigen::VectorXd zip_pq;   // constant-power weight, Q
  Eigen::VectorXd zip_iq;   // constant-current weight, Q
  Eigen::VectorXd zip_zq;   // constant-impedance weight, Q

  // Fixed DER injections (per AC bus, p.u.) from non-variable components:
  // StaticGenerator, non-curtailable RenewableGen, non-controllable PVSystem,
  // VPP, Microgrid, MobileStorage — pre-computed in build_problem.
  Eigen::VectorXd p_fixed_inj;
  Eigen::VectorXd q_fixed_inj;

  // Ybus diagonal (self-admittance) for fast access — replaces dense g/b matrices
  Eigen::VectorXd g_diag;  // real(diag(Ybus))
  Eigen::VectorXd b_diag;  // imag(diag(Ybus))

  // DC admittance matrix (small, kept dense)
  Eigen::MatrixXd gdc_dense;
};

struct EvalWorkspace {
  Eigen::VectorXd p_calc;
  Eigen::VectorXd q_calc;
  Eigen::VectorXd p_gen;
  Eigen::VectorXd q_gen;
  Eigen::VectorXd p_conv_ac;
  Eigen::VectorXd q_conv_ac;
  Eigen::VectorXd p_conv_dc;
  Eigen::VectorXd p_dc_flow;
  // --- Enhanced component workspace ---
  Eigen::VectorXd p_ren;     // per-bus renewable injection
  Eigen::VectorXd q_ren;     // per-bus renewable Q injection
  Eigen::VectorXd p_stor;    // per-bus storage injection
  Eigen::VectorXd q_stor;    // per-bus storage Q injection
  Eigen::VectorXd p_stor_dc; // per-DC-bus storage injection
  Eigen::VectorXd p_flex;    // per-bus flexible load adjustment
};

Problem build_problem(const HybridPowerSystem& sys, const ParityOptions& opt = {});

void build_variable_bounds(const Problem& prob, Eigen::VectorXd& xmin, Eigen::VectorXd& xmax);

void build_initial_point(const Problem& prob,
                         const Eigen::VectorXd& xmin,
                         const Eigen::VectorXd& xmax,
                         Eigen::VectorXd& x0);

double objective(const Problem& prob, const Eigen::VectorXd& x);

void objective_gradient_hessian_diag(const Problem& prob,
                                     const Eigen::VectorXd& x,
                                     Eigen::VectorXd& grad,
                                     Eigen::VectorXd& hdiag);

void equality_constraints(const Problem& prob,
                          const Eigen::VectorXd& x,
                          EvalWorkspace& ws,
                          Eigen::VectorXd& g);

void equality_jacobian(const Problem& prob,
                       const Eigen::VectorXd& x,
                       EvalWorkspace& ws,
                       Eigen::SparseMatrix<double>& jg);

void nonlinear_inequality_constraints(const Problem& prob,
                                      const Eigen::VectorXd& x,
                                      Eigen::VectorXd& h);

void nonlinear_inequality_jacobian(const Problem& prob,
                                   const Eigen::VectorXd& x,
                                   Eigen::SparseMatrix<double>& jh);

void lagrangian_hessian_dense(const Problem& prob,
                              const Eigen::VectorXd& x,
                              const Eigen::VectorXd& lambda_eq,
                              const Eigen::VectorXd* nu_ineq,
                              Eigen::MatrixXd& hess);

void lagrangian_hessian(const Problem& prob,
                        const Eigen::VectorXd& x,
                        const Eigen::VectorXd& lambda_eq,
                        const Eigen::VectorXd* nu_ineq,
                        Eigen::SparseMatrix<double>& hess,
                        double drop_tol = 0.0);

}  // namespace hacdcpf::opf::parity
