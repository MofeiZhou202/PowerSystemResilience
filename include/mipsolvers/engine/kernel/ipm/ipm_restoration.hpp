#pragma once

#include <Eigen/Core>

#include "mipsolvers/engine/problem_types.hpp"

namespace mipsolvers::engine {

/// Feasibility restoration phase, l1-penalty formulation (Wächter–Biegler
/// §3.3 / IPOPT Implementation Paper Eq. 29):
///
///   min_{x, p, n}   Σ_i (p_i + n_i) + (ζ / 2) ‖D_R (x − x_R)‖²
///   s.t.            g(x) + p − n = 0
///                   h_nonlin(x) ≤ 0          (original nonlinear inequalities)
///                   x_L ≤ x ≤ x_U,   p, n ≥ 0
///
/// where x_R is the iterate at which the main IPM stalled, ζ is a small
/// proximal weight, and D_R = diag(min(1, 1/max(1, |x_R,j|))).
///
/// The decision vector of the returned NLPModel is laid out as
///   [ x  (n vars) | p  (m_eq vars) | n  (m_eq vars) ]
/// with m_eq = number of equality constraints of `prob`. If `prob` has no
/// equalities, the returned model is effectively a zero-penalty problem
/// with just the nonlinear inequalities — callers should not invoke
/// restoration in that case.
///
/// `prob` must provide at least `f`, `grad`, `lagrangian_hess` (or `hess`),
/// `g`, and `jac_g`. If present, `h` and `jac_h` are propagated.
struct RestorationBuild {
  NLPModel model;
  int n_x{0};       // number of original variables
  int n_p{0};       // number of p slacks (== m_eq)
  int n_n{0};       // number of n slacks (== m_eq)
  Eigen::VectorXd x_R;
};

RestorationBuild build_restoration_nlp(const NLPModel& prob,
                                       const Eigen::VectorXd& x_R,
                                       double zeta);

/// Extract the x portion of a restoration solution vector.
Eigen::VectorXd extract_x_from_restoration(const RestorationBuild& build,
                                           const Eigen::VectorXd& w);

}  // namespace mipsolvers::engine
