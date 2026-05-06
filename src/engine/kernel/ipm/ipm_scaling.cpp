#include "hacdcpf/engine/kernel/ipm/ipm_scaling.hpp"

#include <algorithm>
#include <cmath>

namespace hacdcpf::engine {

namespace {

double row_infty_norm(const Eigen::SparseMatrix<double>& M, int i) {
  double nrm = 0.0;
  const Eigen::SparseMatrix<double> R = M.row(i);
  for (Eigen::SparseMatrix<double>::InnerIterator it(R, 0); it; ++it) {
    nrm = std::max(nrm, std::abs(it.value()));
  }
  return nrm;
}

double scale_from_norm(double norm, double g_max) {
  if (!std::isfinite(norm) || norm <= g_max) {
    return 1.0;
  }
  return g_max / norm;
}

}  // namespace

ScalingFactors compute_scaling_factors(const NLPModel& prob,
                                       const Eigen::VectorXd& x0,
                                       double g_max) {
  ScalingFactors out;
  if (g_max <= 0.0 || !std::isfinite(g_max)) {
    return out;
  }

  if (prob.grad) {
    Eigen::VectorXd gf;
    prob.grad(x0, gf);
    if (gf.size() > 0) {
      const double nrm = gf.lpNorm<Eigen::Infinity>();
      out.s_f = scale_from_norm(nrm, g_max);
    }
  }

  if (prob.jac_g) {
    Eigen::SparseMatrix<double> Jg;
    prob.jac_g(x0, Jg);
    if (Jg.rows() > 0) {
      out.s_g.resize(Jg.rows());
      for (int i = 0; i < Jg.rows(); ++i) {
        out.s_g[i] = scale_from_norm(row_infty_norm(Jg, i), g_max);
      }
    }
  }

  if (prob.jac_h) {
    Eigen::SparseMatrix<double> Jh;
    prob.jac_h(x0, Jh);
    if (Jh.rows() > 0) {
      out.s_h.resize(Jh.rows());
      for (int i = 0; i < Jh.rows(); ++i) {
        out.s_h[i] = scale_from_norm(row_infty_norm(Jh, i), g_max);
      }
    }
  }

  return out;
}

NLPModel build_scaled_nlp_model(const NLPModel& prob,
                                const ScalingFactors& factors) {
  NLPModel out;
  out.sense = prob.sense;
  out.vars = prob.vars;
  out.x0 = prob.x0;
  out.symbolic_objective = prob.symbolic_objective;
  out.symbolic_constraints = prob.symbolic_constraints;

  const double s_f = factors.s_f;
  const Eigen::VectorXd s_g = factors.s_g;
  const Eigen::VectorXd s_h = factors.s_h;

  if (prob.f) {
    out.f = [f_orig = prob.f, s_f](const Eigen::VectorXd& x) {
      return s_f * f_orig(x);
    };
  }

  if (prob.grad) {
    out.grad = [grad_orig = prob.grad, s_f](const Eigen::VectorXd& x,
                                            Eigen::VectorXd& gout) {
      grad_orig(x, gout);
      if (s_f != 1.0) gout *= s_f;
    };
  }

  if (prob.hess) {
    out.hess = [hess_orig = prob.hess, s_f](const Eigen::VectorXd& x,
                                            Eigen::SparseMatrix<double>& H) {
      hess_orig(x, H);
      if (s_f != 1.0) H *= s_f;
    };
  }

  if (prob.lagrangian_hess) {
    out.lagrangian_hess =
        [lag_orig = prob.lagrangian_hess, s_f, s_g, s_h](
            const Eigen::VectorXd& x, const Eigen::VectorXd& lambda_s,
            const Eigen::VectorXd* mu_s,
            Eigen::SparseMatrix<double>& H) {
          // Unscale multipliers into the original-Lagrangian coordinates:
          //   λ_orig,i = (s_g,i / s_f) · λ_s,i
          //   μ_orig,i = (s_h,i / s_f) · μ_s,i
          Eigen::VectorXd lambda_orig = lambda_s;
          if (s_g.size() == lambda_s.size() && s_f > 0.0) {
            for (int i = 0; i < lambda_orig.size(); ++i) {
              lambda_orig[i] = lambda_s[i] * s_g[i] / s_f;
            }
          }
          if (mu_s == nullptr) {
            lag_orig(x, lambda_orig, nullptr, H);
          } else {
            Eigen::VectorXd mu_orig = *mu_s;
            // The nonlinear-inequality block is the leading portion of s_h,
            // so we rescale the first mu_s->size() entries (the caller passes
            // only the nonlinear inequality multipliers).
            const int m = static_cast<int>(mu_orig.size());
            if (s_h.size() >= m && s_f > 0.0) {
              for (int i = 0; i < m; ++i) {
                mu_orig[i] = (*mu_s)[i] * s_h[i] / s_f;
              }
            }
            lag_orig(x, lambda_orig, &mu_orig, H);
          }
          if (s_f != 1.0) H *= s_f;
        };
  }

  if (prob.g) {
    out.g = [g_orig = prob.g, s_g](const Eigen::VectorXd& x,
                                   Eigen::VectorXd& v) {
      g_orig(x, v);
      if (s_g.size() == v.size()) {
        for (int i = 0; i < v.size(); ++i) v[i] *= s_g[i];
      }
    };
  }

  if (prob.jac_g) {
    out.jac_g = [jg_orig = prob.jac_g, s_g](const Eigen::VectorXd& x,
                                            Eigen::SparseMatrix<double>& J) {
      jg_orig(x, J);
      if (s_g.size() == J.rows()) {
        for (int k = 0; k < J.outerSize(); ++k) {
          for (Eigen::SparseMatrix<double>::InnerIterator it(J, k); it; ++it) {
            it.valueRef() *= s_g[it.row()];
          }
        }
      }
    };
  }

  if (prob.h) {
    out.h = [h_orig = prob.h, s_h](const Eigen::VectorXd& x,
                                   Eigen::VectorXd& v) {
      h_orig(x, v);
      if (s_h.size() == v.size()) {
        for (int i = 0; i < v.size(); ++i) v[i] *= s_h[i];
      }
    };
  }

  if (prob.jac_h) {
    out.jac_h = [jh_orig = prob.jac_h, s_h](const Eigen::VectorXd& x,
                                            Eigen::SparseMatrix<double>& J) {
      jh_orig(x, J);
      if (s_h.size() == J.rows()) {
        for (int k = 0; k < J.outerSize(); ++k) {
          for (Eigen::SparseMatrix<double>::InnerIterator it(J, k); it; ++it) {
            it.valueRef() *= s_h[it.row()];
          }
        }
      }
    };
  }

  return out;
}

}  // namespace hacdcpf::engine
