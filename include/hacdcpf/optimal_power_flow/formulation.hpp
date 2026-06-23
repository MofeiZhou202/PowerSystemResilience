#pragma once

/// optimal_power_flow/formulation.hpp
/// =====================================
/// Parity full-space OPF formulation: variable/constraint layout,
/// KKT evaluation functions, and problem assembly.
///
/// The assembled nonlinear program is
///
/// @htmlonly
/// <div>\[
///   \min_x f(x)\quad
///   \text{s.t.}\quad g(x)=0,\quad h(x)\le 0,\quad x_{min}\le x\le x_{max}.
/// \]</div>
/// @endhtmlonly
///
/// All AC bus angles, voltage magnitudes, generator powers, converter powers,
/// controllable resources, and optional shedding variables are kept as explicit
/// primal variables.  This full-space layout is intentionally review-friendly:
/// `VarIndex` gives each variable block and `ConstraintIndex` gives each
/// equality/inequality row block.
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

/// Contiguous variable-block layout for the parity OPF vector `x`.
///
/// The vector order is
///
/// @htmlonly
/// <div>\[
/// x = [\theta, V, P^g, Q^g, V^{dc}, P^{ac}_c, Q^{ac}_c, P^{dc}_c,
///      \Delta P^d, \Delta Q^d, P^{ren}, Q^{ren}, P^{stor}, Q^{stor},
///      P^{stor,dc}, P^{dcdc}, P^{flex}].
/// \]</div>
/// @endhtmlonly
///
/// `n_*` fields are block lengths and `i_*` fields are the starting offsets
/// into `x`.  Empty component families have length zero but still retain a
/// valid offset equal to the next block start.
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

/// Contiguous equality and nonlinear-inequality row layout.
///
/// Equality rows are ordered as
///
/// @htmlonly
/// <div>\[
///   g(x) = [g^P_{ac}, g^Q_{ac}, g^P_{dc}, g^{vsc}, g^{dcdc}, g^{er}].
/// \]</div>
/// @endhtmlonly
///
/// Nonlinear inequalities use the convention `h(x) <= 0` and are ordered as
/// from-end AC branch limits, to-end AC branch limits, VSC apparent-power
/// limits, DC branch power limits, VSC AC-current limits, VSC modulation upper
/// and lower limits, and DC/DC duty-ratio rows.
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

/// Fully assembled parity OPF problem and all model-to-vector maps.
///
/// `data` is canonical `SolverData`; maps such as `gen_var_to_data` and
/// `conv_var_to_data` translate compact variable positions back to canonical
/// component arrays.  Per-bus vectors store fixed demand/injection terms and
/// ZIP coefficients used by the balance equations.
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

/// Reusable buffers for equality and Jacobian evaluation.
///
/// The workspace avoids repeated allocation while keeping intermediate
/// quantities such as AC network injections and per-bus component sums visible
/// for debugging and finite-difference checks.
struct EvalWorkspace {
  Eigen::VectorXd p_calc, q_calc;
  Eigen::VectorXd p_gen, q_gen;
  Eigen::VectorXd p_conv_ac, q_conv_ac, p_conv_dc;
  Eigen::VectorXd p_dc_flow;
  Eigen::VectorXd p_ren, q_ren;
  Eigen::VectorXd p_stor, q_stor, p_stor_dc;
  Eigen::VectorXd p_flex;
};

/// Canonicalize a rich system and assemble the full parity OPF model.
///
/// The function creates `SolverData`, selects controllable variables, builds
/// component-to-variable maps, precomputes ZIP load coefficients, and counts all
/// equality/inequality row families.
///
/// @param sys Rich hybrid AC/DC system.
/// @param opt Model-family toggles and economic load-shedding options.
/// @return Problem containing canonical data, variable layout, constraint
/// layout, maps, and precomputed coefficients.
Problem build_problem(const HybridPowerSystem& sys, const ParityOptions& opt = {});

/// Build lower and upper bounds for every variable block.
///
/// Typical bounds include voltage limits, generator P/Q limits, converter power
/// limits, load-shedding windows, renewable availability, storage charge/
/// discharge limits, and flexible-load bounds.
///
/// @param prob Assembled parity problem.
/// @param xmin Output lower bounds with length `prob.vidx.n_total`.
/// @param xmax Output upper bounds with length `prob.vidx.n_total`.
void build_variable_bounds(const Problem& prob,
                           Eigen::VectorXd& xmin,
                           Eigen::VectorXd& xmax);

/// Construct a strictly interior warm start.
///
/// Initial values are clipped to
///
/// @htmlonly
/// <div>\[
///   x_{min} + \epsilon < x_0 < x_{max} - \epsilon
/// \]</div>
/// @endhtmlonly
///
/// where possible, so the interior-point solver can start without immediately
/// violating bound slacks.  The warm start uses bus setpoints, generator
/// dispatch estimates, DC voltage guesses, and converter/storage defaults.
///
/// @param prob Assembled parity problem.
/// @param xmin Variable lower bounds.
/// @param xmax Variable upper bounds.
/// @param x0 Output primal initial point.
void build_initial_point(const Problem& prob,
                         const Eigen::VectorXd& xmin,
                         const Eigen::VectorXd& xmax,
                         Eigen::VectorXd& x0);

/// Evaluate the economic objective.
///
/// The main terms are quadratic generator costs and optional value-of-lost-load
/// shedding penalties:
///
/// @htmlonly
/// <div>\[
///   f(x)=\sum_g (c_{2g} P_g^2+c_{1g}P_g+c_{0g})
///       + VOLL\sum_i(\Delta P_i^d+\Delta Q_i^d)
///       + \sum_r c^{curt}_r(P^{avail}_r-P^{ren}_r).
/// \]</div>
/// @endhtmlonly
///
/// MW/MVAr quantities are converted from p.u. using `base_mva` before applying
/// cost coefficients stored in engineering units.
double objective(const Problem& prob, const Eigen::VectorXd& x);

/// Evaluate the objective gradient and diagonal Hessian contribution.
///
/// Only separable objective terms are represented here.  For generator cost
///
/// @htmlonly
/// <div>\[
///   c_2(baseMVA\,p_g)^2 + c_1(baseMVA\,p_g) + c_0,
/// \]</div>
/// @endhtmlonly
///
/// the p.u. derivatives are
///
/// @htmlonly
/// <div>\[
///   \frac{\partial f}{\partial p_g}
///     = 2c_2 baseMVA^2 p_g + c_1 baseMVA,\qquad
///   \frac{\partial^2 f}{\partial p_g^2}=2c_2 baseMVA^2.
/// \]</div>
/// @endhtmlonly
void objective_gradient_hessian_diag(const Problem& prob,
                                     const Eigen::VectorXd& x,
                                     Eigen::VectorXd& grad,
                                     Eigen::VectorXd& hdiag);

/// Evaluate all equality residuals `g(x)`.
///
/// AC active/reactive balances use the sign convention
///
/// @htmlonly
/// <div>\[
/// g^P_i = P^{net}_i(\theta,V)-P^g_i-P^{fixed}_i-P^{ren}_i-P^{stor}_i
///         +P^d_i(V)-\Delta P^d_i+P^{flex}_i-P^{ac}_i,
/// \]</div>
/// <div>\[
/// g^Q_i = Q^{net}_i(\theta,V)-Q^g_i-Q^{fixed}_i-Q^{ren}_i-Q^{stor}_i
///         +Q^d_i(V)-\Delta Q^d_i-Q^{ac}_i.
/// \]</div>
/// @endhtmlonly
///
/// The AC network injections are the polar power-flow equations
///
/// @htmlonly
/// <div>\[
/// P^{net}_i=V_i\sum_j V_j(G_{ij}\cos\theta_{ij}+B_{ij}\sin\theta_{ij}),
/// \]</div>
/// <div>\[
/// Q^{net}_i=V_i\sum_j V_j(G_{ij}\sin\theta_{ij}-B_{ij}\cos\theta_{ij}).
/// \]</div>
/// @endhtmlonly
///
/// DC balances use conductance flow
///
/// @htmlonly
/// <div>\[
/// P^{dc}_k=\sum_m G^{dc}_{km}V^{dc}_k(V^{dc}_k-V^{dc}_m),
/// \]</div>
/// @endhtmlonly
///
/// and VSC coupling rows use
///
/// @htmlonly
/// <div>\[
///   P^{ac}_c + P^{dc}_c + a_c + b_c I^{ac}_c + c_c(I^{ac}_c)^2 = 0.
/// \]</div>
/// @endhtmlonly
void equality_constraints(const Problem& prob,
                          const Eigen::VectorXd& x,
                          EvalWorkspace& ws,
                          Eigen::VectorXd& g);

/// Assemble the sparse Jacobian of `equality_constraints`.
///
/// The resulting matrix has shape
/// `prob.cidx.n_eq_total x prob.vidx.n_total` and follows the row/block order
/// documented by `ConstraintIndex`.
void equality_jacobian(const Problem& prob,
                       const Eigen::VectorXd& x,
                       EvalWorkspace& ws,
                       Eigen::SparseMatrix<double>& jg);

/// Evaluate nonlinear inequalities using the convention `h(x) <= 0`.
///
/// Major row families include:
///
/// @htmlonly
/// <div>\[
///   (P^{f}_{ij})^2+(Q^{f}_{ij})^2-S_{ij,max}^2 \le 0,\qquad
///   (P^{t}_{ij})^2+(Q^{t}_{ij})^2-S_{ij,max}^2 \le 0,
/// \]</div>
/// <div>\[
///   (P^{ac}_c)^2+(Q^{ac}_c)^2-S_{c,max}^2 \le 0,
/// \]</div>
/// <div>\[
///   (P^{dc}_{km})^2-P_{km,max}^2 \le 0,
/// \]</div>
/// <div>\[
///   (P^{ac}_c)^2+(Q^{ac}_c)^2-(I^{ac}_{c,max}V_i)^2 \le 0.
/// \]</div>
/// @endhtmlonly
///
/// Converter modulation and DC/DC duty-ratio bounds are encoded as linear rows
/// in the same nonlinear-inequality vector so the IPM has one inequality API.
void nonlinear_inequality_constraints(const Problem& prob,
                                      const Eigen::VectorXd& x,
                                      Eigen::VectorXd& h);

/// Assemble the sparse Jacobian of `nonlinear_inequality_constraints`.
///
/// The row order is exactly the row order emitted by
/// `nonlinear_inequality_constraints`, which lets IPM duals `mu` be interpreted
/// by the same `ConstraintIndex` counts.
void nonlinear_inequality_jacobian(const Problem& prob,
                                   const Eigen::VectorXd& x,
                                   Eigen::SparseMatrix<double>& jh);

/// Assemble the dense Hessian of the Lagrangian.
///
/// This convenience wrapper returns
///
/// @htmlonly
/// <div>\[
///   \nabla^2_{xx} L
///   = \nabla^2 f(x)
///   + \sum_i \lambda_i\nabla^2 g_i(x)
///   + \sum_j \nu_j\nabla^2 h_j(x).
/// \]</div>
/// @endhtmlonly
///
/// It is useful for diagnostics and dense solver experiments; production IPM
/// paths should prefer the sparse overload.
void lagrangian_hessian_dense(const Problem& prob,
                              const Eigen::VectorXd& x,
                              const Eigen::VectorXd& lambda_eq,
                              const Eigen::VectorXd* nu_ineq,
                              Eigen::MatrixXd& hess);

/// Assemble the sparse Hessian of the Lagrangian.
///
/// `lambda_eq` weights equality rows and `nu_ineq` weights inequality rows.  If
/// `nu_ineq` is null, only objective and equality curvature are included.  The
/// returned matrix is symmetric and has shape
/// `prob.vidx.n_total x prob.vidx.n_total`.
///
/// @param prob Assembled parity problem.
/// @param x Primal point at which curvature is evaluated.
/// @param lambda_eq Equality-row multipliers in `ConstraintIndex` order.
/// @param nu_ineq Optional inequality-row multipliers in `ConstraintIndex`
/// order; pass null to omit inequality curvature.
/// @param hess Output sparse symmetric Hessian matrix.
/// @param drop_tol Reserved for dropping tiny entries; currently kept for API
/// compatibility.
void lagrangian_hessian(const Problem& prob,
                        const Eigen::VectorXd& x,
                        const Eigen::VectorXd& lambda_eq,
                        const Eigen::VectorXd* nu_ineq,
                        Eigen::SparseMatrix<double>& hess,
                        double drop_tol = 0.0);

}  // namespace hacdcpf::opf::parity
