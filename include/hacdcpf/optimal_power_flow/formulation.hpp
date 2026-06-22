#pragma once

/// optimal_power_flow/formulation.hpp
/// =====================================
/// Parity full-space OPF formulation: variable/constraint layout,
/// KKT evaluation functions, and problem assembly.
/// Replaces: optimal_power_flow/parity_formulation.hpp.

#include <vector>

#include <Eigen/Core>
#include <Eigen/Sparse>

#include "hacdcpf/assembly/solver_data.hpp"
#include "hacdcpf/model/hybrid_power_system.hpp"

namespace hacdcpf::opf::parity {

struct ParityOptions {
  bool load_shedding{true};
  double voll{0.0};     ///< 0.0 => auto-compute from max gen cost
  double eps_iac{1e-6}; ///< Smoothing in |Iac|
  // Constraint-family toggles (multi-converter model §3.1).  Default true keeps
  // the always-enforce behavior; the GUI/OPF options can disable a family.
  bool enforce_branch_limits{true};
  bool enforce_converter_capacity{true};
  bool enforce_converter_current_limits{true};
  bool enforce_converter_modulation_limits{true};
};

struct VarIndex {
  int n_va{0}, n_vm{0}, n_pg{0}, n_qg{0};
  int n_vdc{0}, n_pac{0}, n_qac{0}, n_pdc{0};
  int n_dpd{0}, n_dqd{0};
  int n_pren{0}, n_qren{0};
  int n_pstor{0}, n_qstor{0}, n_pstordc{0};
  int n_pdcdc{0}, n_pflex{0};
  int n_total{0};

  int i_va{0}, i_vm{0}, i_pg{0}, i_qg{0};
  int i_vdc{0}, i_pac{0}, i_qac{0}, i_pdc{0};
  int i_dpd{0}, i_dqd{0};
  int i_pren{0}, i_qren{0};
  int i_pstor{0}, i_qstor{0}, i_pstordc{0};
  int i_pdcdc{0}, i_pflex{0};
};

struct ConstraintIndex {
  int n_pbal_ac{0}, n_qbal_ac{0}, n_pbal_dc{0};
  int n_conv_bal{0}, n_dcdc_bal{0}, n_er_bal{0};
  int n_eq_total{0};

  int i_pbal_ac{0}, i_qbal_ac{0}, i_pbal_dc{0};
  int i_conv_bal{0}, i_dcdc_bal{0}, i_er_bal{0};

  int n_sf{0}, n_st{0}, n_sconv{0}, n_sdc{0};
  // Converter physical limits (multi-converter model §3.1.5/3.1.4), appended
  // after the sdc block.  Each counts only the converters that declare the
  // corresponding limit, so unconstrained converters add no rows.
  int n_iac{0};   ///< AC current limits  Iac² ≤ i_ac_max²
  int n_mmax{0};  ///< modulation upper limits  m ≤ m_max
  int n_mmin{0};  ///< modulation lower limits  m ≥ m_min
  int n_dcdc_duty{0};  ///< DC/DC duty-ratio limits (2 linear rows per converter)
  int n_ineq_nonlin{0};
};

struct Problem {
  powerflow::SolverData data;
  ParityOptions options;
  VarIndex vidx;
  ConstraintIndex cidx;

  std::vector<int> gen_var_to_data;
  std::vector<int> conv_var_to_data;
  std::vector<int> branch_limited;
  std::vector<int> dc_branch_limited;

  // Converter var indices (into the pac/qac arrays) that declare each optional
  // physical limit (multi-converter model §3.1.4/3.1.5).
  std::vector<int> conv_iac_limited;
  std::vector<int> conv_mmax_limited;
  std::vector<int> conv_mmin_limited;
  // DC/DC converter var indices (into the dcdc_* arrays) with a non-Generic
  // topology and a valid [d_min, d_max] window → two linear duty-ratio rows each
  // (multi-converter model §3.2).
  std::vector<int> dcdc_duty_limited;

  std::vector<int> gen_bus;
  std::vector<int> conv_ac_bus;
  std::vector<int> conv_dc_bus;

  std::vector<int> ren_var_to_data;
  std::vector<int> ren_source;
  std::vector<int> ren_bus;

  std::vector<int> stor_var_to_data;
  std::vector<int> stor_bus;
  std::vector<int> stor_dc_var_to_data;
  std::vector<int> stor_dc_bus;

  std::vector<int> dcdc_var_to_data;
  std::vector<int> dcdc_bus_in;
  std::vector<int> dcdc_bus_out;

  std::vector<int> flex_var_to_data;
  std::vector<int> flex_bus;

  struct ERPortInfo {
    int router_idx;
    int port_idx;
    int bus;
    bool is_ac;
  };
  std::vector<ERPortInfo> er_ports;

  double scale_p{1.0};
  double scale_q{1.0};
  double voll_effective{100.0};

  Eigen::VectorXd pd_demand_pu;
  Eigen::VectorXd qd_demand_pu;
  Eigen::VectorXd zip_pp, zip_ip, zip_zp;
  Eigen::VectorXd zip_pq, zip_iq, zip_zq;
  Eigen::VectorXd p_fixed_inj;
  Eigen::VectorXd q_fixed_inj;
  Eigen::VectorXd g_diag;
  Eigen::VectorXd b_diag;
  Eigen::MatrixXd gdc_dense;
};

struct EvalWorkspace {
  Eigen::VectorXd p_calc, q_calc;
  Eigen::VectorXd p_gen, q_gen;
  Eigen::VectorXd p_conv_ac, q_conv_ac, p_conv_dc;
  Eigen::VectorXd p_dc_flow;
  Eigen::VectorXd p_ren, q_ren;
  Eigen::VectorXd p_stor, q_stor, p_stor_dc;
  Eigen::VectorXd p_flex;
};

Problem build_problem(const HybridPowerSystem& sys, const ParityOptions& opt = {});

void build_variable_bounds(const Problem& prob,
                           Eigen::VectorXd& xmin,
                           Eigen::VectorXd& xmax);

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
