// ═══════════════════════════════════════════════════════════════════════════
// Cone algebra for the native conic IPM — implementation.
//
// svec/smat packing, SOC Jordan algebra, per-block Nesterov-Todd scalings,
// interior tests, and maximum step lengths to the cone boundary.
// ═══════════════════════════════════════════════════════════════════════════

#include "mipsolvers/engine/kernel/ipm/cones.hpp"

#include <cmath>
#include <limits>

#include <Eigen/Cholesky>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <Eigen/SVD>

// OpenMP for parallel per-block scaling computations.  Every parallel loop
// writes disjoint per-block outputs, so results are deterministic regardless
// of the thread count.  Mirrors the guard style of ipm_lp_solver.cpp.
#if defined(MIPSOLVERS_USE_OPENMP)
#include <omp.h>
#define MIPSOLVERS_OMP_STR_(x) #x
#define MIPSOLVERS_OMP_STR(x) MIPSOLVERS_OMP_STR_(x)
#define MIPSOLVERS_OMP_PARALLEL_IF(cond) \
  _Pragma(MIPSOLVERS_OMP_STR(omp parallel for if(cond)))
#else
#define MIPSOLVERS_OMP_PARALLEL_IF(cond)
#endif

namespace mipsolvers::engine {
namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kSqrt2 = 1.41421356237309504880168872420969808;

/// Block-parallel work threshold: below this (cubic-scaled) flops estimate,
/// scaling computations stay serial.
constexpr long kParallelWorkThreshold = 4096;

/// Quadratic representation Q(v) x = 2 v (v'x) - J x with J = diag(1, -1..).
[[nodiscard]] Eigen::VectorXd q_qrep(
    const Eigen::Ref<const Eigen::VectorXd>& v,
    const Eigen::Ref<const Eigen::VectorXd>& x) {
  const int k = static_cast<int>(x.size());
  const double vtx = v.dot(x);
  Eigen::VectorXd out(k);
  out[0] = 2.0 * vtx * v[0] - x[0];
  out.tail(k - 1) = 2.0 * vtx * v.tail(k - 1) + x.tail(k - 1);
  return out;
}

/// J x with J = diag(1, -1, ..., -1).
[[nodiscard]] Eigen::VectorXd q_jmul(
    const Eigen::Ref<const Eigen::VectorXd>& x) {
  Eigen::VectorXd out = x;
  out.tail(out.size() - 1) = -out.tail(out.size() - 1);
  return out;
}

}  // namespace

// ═══════════════════════════════════════════════════════════════════════════
// svec packing
// ═══════════════════════════════════════════════════════════════════════════

Eigen::VectorXd svec(const Eigen::Ref<const Eigen::MatrixXd>& x) {
  const int p = static_cast<int>(x.rows());
  Eigen::VectorXd out(svec_size(p));
  int k = 0;
  for (int j = 0; j < p; ++j) {
    for (int i = j; i < p; ++i) {
      out[k++] = (i == j) ? x(i, j) : kSqrt2 * x(i, j);
    }
  }
  return out;
}

Eigen::MatrixXd smat(const Eigen::Ref<const Eigen::VectorXd>& v, int p) {
  Eigen::MatrixXd out(p, p);
  int k = 0;
  for (int j = 0; j < p; ++j) {
    for (int i = j; i < p; ++i) {
      const double val = (i == j) ? v[k] : v[k] / kSqrt2;
      out(i, j) = val;
      out(j, i) = val;
      ++k;
    }
  }
  return out;
}

// ═══════════════════════════════════════════════════════════════════════════
// Cone layout
// ═══════════════════════════════════════════════════════════════════════════

ConeLayout ConeLayout::build(const ConeDims& dims) {
  ConeLayout lay;
  lay.l = dims.l;
  int off = dims.l;
  for (int q : dims.q) {
    lay.q_offsets.push_back(off);
    lay.q_sizes.push_back(q);
    off += q;
  }
  for (int p : dims.s) {
    lay.s_offsets.push_back(off);
    lay.s_orders.push_back(p);
    off += svec_size(p);
  }
  lay.total = off;
  lay.degree = dims.l + static_cast<int>(dims.q.size());
  for (int p : dims.s) {
    lay.degree += p;
  }
  lay.e = Eigen::VectorXd::Zero(lay.total);
  for (int i = 0; i < dims.l; ++i) {
    lay.e[i] = 1.0;
  }
  for (std::size_t b = 0; b < lay.q_sizes.size(); ++b) {
    lay.e[lay.q_offsets[b]] = 1.0;
  }
  for (std::size_t b = 0; b < lay.s_orders.size(); ++b) {
    const int p = lay.s_orders[b];
    const int base = lay.s_offsets[b];
    for (int j = 0; j < p; ++j) {
      lay.e[base + svec_index(j, j, p)] = 1.0;
    }
  }
  return lay;
}

// ═══════════════════════════════════════════════════════════════════════════
// SOC Jordan algebra
// ═══════════════════════════════════════════════════════════════════════════

double q_det(const Eigen::Ref<const Eigen::VectorXd>& u) {
  return u[0] * u[0] - u.tail(u.size() - 1).squaredNorm();
}

Eigen::VectorXd q_jordan_product(
    const Eigen::Ref<const Eigen::VectorXd>& u,
    const Eigen::Ref<const Eigen::VectorXd>& v) {
  const int k = static_cast<int>(u.size());
  Eigen::VectorXd out(k);
  out[0] = u.dot(v);
  out.tail(k - 1) = u[0] * v.tail(k - 1) + v[0] * u.tail(k - 1);
  return out;
}

Eigen::VectorXd q_jordan_inverse(
    const Eigen::Ref<const Eigen::VectorXd>& u) {
  return q_jmul(u) / q_det(u);
}

Eigen::VectorXd q_jordan_solve(
    const Eigen::Ref<const Eigen::VectorXd>& u,
    const Eigen::Ref<const Eigen::VectorXd>& b) {
  // Solve the arrowhead system [[u0, u1'], [u1, u0 I]] y = b:
  //   y0 = (u0 b0 - u1'b1) / det(u)
  //   y1 = b1/u0 + (u1'b1 - u0 b0) / (det(u) u0) * u1
  const int k = static_cast<int>(u.size());
  const double det = q_det(u);
  const double d = u.tail(k - 1).dot(b.tail(k - 1));
  const double y0 = (u[0] * b[0] - d) / det;
  Eigen::VectorXd out(k);
  out[0] = y0;
  out.tail(k - 1) = b.tail(k - 1) / u[0] +
                    ((d - u[0] * b[0]) / (det * u[0])) * u.tail(k - 1);
  return out;
}

Eigen::VectorXd q_jordan_sqrt(
    const Eigen::Ref<const Eigen::VectorXd>& u) {
  const int k = static_cast<int>(u.size());
  const double rho = std::sqrt(q_det(u));
  const double t = std::sqrt((u[0] + rho) / 2.0);
  Eigen::VectorXd out(k);
  out[0] = t;
  out.tail(k - 1) = u.tail(k - 1) / (2.0 * t);
  return out;
}

// ═══════════════════════════════════════════════════════════════════════════
// Interior tests
// ═══════════════════════════════════════════════════════════════════════════

bool l_interior(const double* u, int n) {
  for (int i = 0; i < n; ++i) {
    if (u[i] <= 0.0) return false;
  }
  return true;
}

bool q_interior(const Eigen::Ref<const Eigen::VectorXd>& u) {
  return u[0] > 0.0 && q_det(u) > 0.0;
}

bool s_interior(const Eigen::Ref<const Eigen::VectorXd>& u, int p) {
  const Eigen::MatrixXd x = smat(u, p);
  const Eigen::LLT<Eigen::MatrixXd> llt(x);
  return llt.info() == Eigen::Success;
}

// ═══════════════════════════════════════════════════════════════════════════
// Maximum step to the cone boundary
// ═══════════════════════════════════════════════════════════════════════════

double l_max_step(const double* u, const double* du, int n) {
  double alpha = kInf;
  for (int i = 0; i < n; ++i) {
    if (du[i] < 0.0) {
      alpha = std::min(alpha, -u[i] / du[i]);
    }
  }
  return alpha;
}

double q_max_step(const Eigen::Ref<const Eigen::VectorXd>& u,
                  const Eigen::Ref<const Eigen::VectorXd>& du) {
  const int k = static_cast<int>(u.size());
  const double c = q_det(u);
  if (c <= 0.0 || u[0] <= 0.0) return 0.0;  // u not interior: no valid step
  const double a = q_det(du);
  const double b = u[0] * du[0] - u.tail(k - 1).dot(du.tail(k - 1));

  double alpha = kInf;
  // det(u + alpha*du) = c + 2*b*alpha + a*alpha^2; find its smallest
  // positive root (the det-limit of the ray).
  if (a != 0.0) {
    const double disc = b * b - a * c;
    if (disc >= 0.0) {
      const double sq = std::sqrt(disc);
      // Numerically stable form of the root (-b - sqrt(disc)) / a.
      const double root = (b < 0.0) ? c / (-b + sq) : (-b - sq) / a;
      if (root > 0.0) alpha = std::min(alpha, root);
    }
  } else if (b < 0.0) {
    alpha = std::min(alpha, -c / (2.0 * b));
  }
  // The boundary also requires (u + alpha*du)_0 >= 0.
  if (du[0] < 0.0) {
    alpha = std::min(alpha, -u[0] / du[0]);
  }
  return (alpha < 0.0) ? 0.0 : alpha;
}

double s_max_step(const Eigen::Ref<const Eigen::VectorXd>& u,
                  const Eigen::Ref<const Eigen::VectorXd>& du,
                  int p) {
  const Eigen::MatrixXd x = smat(u, p);
  const Eigen::LLT<Eigen::MatrixXd> llt(x);
  if (llt.info() != Eigen::Success) return 0.0;  // u not interior
  const Eigen::MatrixXd l = llt.matrixL();
  const Eigen::MatrixXd dx = smat(du, p);
  // E = L^{-1} dX L^{-T} (symmetrized against roundoff asymmetry).
  const Eigen::MatrixXd y = l.triangularView<Eigen::Lower>().solve(dx);
  Eigen::MatrixXd e =
      l.triangularView<Eigen::Lower>().solve(y.transpose()).transpose();
  e = 0.5 * (e + e.transpose());
  const Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(
      e, Eigen::EigenvaluesOnly);
  const double lambda_min = es.eigenvalues()[0];
  return (lambda_min >= 0.0) ? kInf : -1.0 / lambda_min;
}

// ═══════════════════════════════════════════════════════════════════════════
// Shifts to the interior
// ═══════════════════════════════════════════════════════════════════════════

double l_shift_to_interior(const double* u, int n) {
  double alpha = 0.0;
  for (int i = 0; i < n; ++i) {
    alpha = std::max(alpha, -u[i]);
  }
  return alpha;
}

double q_shift_to_interior(
    const Eigen::Ref<const Eigen::VectorXd>& u) {
  return std::max(0.0, u.tail(u.size() - 1).norm() - u[0]);
}

double s_shift_to_interior(
    const Eigen::Ref<const Eigen::VectorXd>& u, int p) {
  const Eigen::MatrixXd x = smat(u, p);
  const Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(
      x, Eigen::EigenvaluesOnly);
  return std::max(0.0, -es.eigenvalues()[0]);
}

// ═══════════════════════════════════════════════════════════════════════════
// Nesterov-Todd scaling
// ═══════════════════════════════════════════════════════════════════════════

bool ConeNtScaling::compute(const ConeLayout& layout,
                            const Eigen::VectorXd& s,
                            const Eigen::VectorXd& z) {
  l_extra_.clear();

  // ── Orthant block (serial; O(l)) ───────────────────────────────────────
  l_d_.resize(layout.l);
  lambda_.resize(layout.total);
  for (int i = 0; i < layout.l; ++i) {
    if (s[i] <= 0.0 || z[i] <= 0.0) return false;
    l_d_[i] = std::sqrt(s[i] / z[i]);
    lambda_[i] = std::sqrt(s[i] * z[i]);
  }

  const int nq = static_cast<int>(layout.q_sizes.size());
  const int ns = static_cast<int>(layout.s_orders.size());
  q_.resize(static_cast<std::size_t>(nq));
  s_.resize(static_cast<std::size_t>(ns));
  std::vector<char> keep_q(static_cast<std::size_t>(nq), 1);

  // Cubic-scaled work estimate for the block-parallel region.
  long work = 0;
  for (int q : layout.q_sizes) work += q;
  for (int p : layout.s_orders) work += static_cast<long>(p) * p * p;
  std::vector<char> ok(static_cast<std::size_t>(nq + ns), 1);

  [[maybe_unused]] const bool run_parallel =
      (nq + ns > 1) && (work > kParallelWorkThreshold);
  MIPSOLVERS_OMP_PARALLEL_IF(run_parallel)
  for (int bi = 0; bi < nq + ns; ++bi) {
    if (bi < nq) {
      // ── SOC block ──────────────────────────────────────────────────────
      const int off = layout.q_offsets[static_cast<std::size_t>(bi)];
      const int k = layout.q_sizes[static_cast<std::size_t>(bi)];
      const auto sv = s.segment(off, k);
      const auto zv = z.segment(off, k);
      if (k == 1) {
        // Size-1 SOC blocks are handled exactly like an orthant row.
        if (sv[0] <= 0.0 || zv[0] <= 0.0) {
          ok[static_cast<std::size_t>(bi)] = 0;
          continue;
        }
        lambda_[off] = std::sqrt(sv[0] * zv[0]);
        continue;
      }
      const double det_s = q_det(sv);
      const double det_z = q_det(zv);
      if (det_s <= 0.0 || det_z <= 0.0 || sv[0] <= 0.0 || zv[0] <= 0.0) {
        ok[static_cast<std::size_t>(bi)] = 0;
        continue;
      }
      const double rho_s = std::sqrt(det_s);
      const double rho_z = std::sqrt(det_z);
      const Eigen::VectorXd s_bar = sv / rho_s;
      const Eigen::VectorXd z_bar = zv / rho_z;
      const double gamma = std::sqrt((1.0 + s_bar.dot(z_bar)) / 2.0);

      QScaling qs;
      qs.offset = off;
      qs.size = k;
      qs.beta = std::sqrt(rho_s / rho_z);
      qs.wbar = (s_bar + q_jmul(z_bar)) / (2.0 * gamma);
      qs.v = q_jordan_sqrt(qs.wbar);
      // lambda = W z = beta * (2 v (v'z) - J z)
      lambda_.segment(off, k) = qs.beta * q_qrep(qs.v, zv);
      q_[static_cast<std::size_t>(bi)] = std::move(qs);
    } else {
      // ── SDP block ──────────────────────────────────────────────────────
      const int si = bi - nq;
      const int off = layout.s_offsets[static_cast<std::size_t>(si)];
      const int p = layout.s_orders[static_cast<std::size_t>(si)];
      const int mp = svec_size(p);
      const auto sv = s.segment(off, mp);
      const auto zv = z.segment(off, mp);
      const Eigen::MatrixXd sm = smat(sv, p);
      const Eigen::MatrixXd zm = smat(zv, p);
      const Eigen::LLT<Eigen::MatrixXd> llt_s(sm);
      const Eigen::LLT<Eigen::MatrixXd> llt_z(zm);
      if (llt_s.info() != Eigen::Success || llt_z.info() != Eigen::Success) {
        ok[static_cast<std::size_t>(bi)] = 0;
        continue;
      }
      const Eigen::MatrixXd ls = llt_s.matrixL();
      const Eigen::MatrixXd lz = llt_z.matrixL();
      const Eigen::MatrixXd prod = lz.transpose() * ls;
      const Eigen::JacobiSVD<Eigen::MatrixXd> svd(
          prod, Eigen::ComputeFullU | Eigen::ComputeFullV);
      const Eigen::VectorXd& sigma = svd.singularValues();
      const Eigen::MatrixXd& v = svd.matrixV();

      SScaling ss;
      ss.offset = off;
      ss.order = p;
      ss.sigma = sigma;
      ss.r = ls * v * sigma.cwiseInverse().cwiseSqrt().asDiagonal();
      // Rti = R^{-T} = Ls^{-T} V Sigma^{1/2} via the triangular solve
      // Ls' X = V Sigma^{1/2}.
      ss.rti = ls.transpose().triangularView<Eigen::Upper>().solve(
          v * sigma.cwiseSqrt().asDiagonal());
      ss.h_cong = ss.r * ss.r.transpose();
      ss.h_inv_cong = ss.rti * ss.rti.transpose();
      Eigen::VectorXd lam = Eigen::VectorXd::Zero(mp);
      for (int j = 0; j < p; ++j) {
        lam[svec_index(j, j, p)] = sigma[j];
      }
      lambda_.segment(off, mp) = lam;
      s_[static_cast<std::size_t>(si)] = std::move(ss);
    }
  }

  for (char flag : ok) {
    if (!flag) return false;
  }

  // Collect size-1 SOC rows as extra orthant entries and drop them from the
  // q scaling list (they are scalar-handled everywhere).
  for (int bi = 0; bi < nq; ++bi) {
    if (layout.q_sizes[static_cast<std::size_t>(bi)] == 1) {
      const int off = layout.q_offsets[static_cast<std::size_t>(bi)];
      l_extra_.emplace_back(off, std::sqrt(s[off] / z[off]));
      keep_q[static_cast<std::size_t>(bi)] = 0;
    }
  }
  std::vector<QScaling> q_full;
  for (int bi = 0; bi < nq; ++bi) {
    if (keep_q[static_cast<std::size_t>(bi)]) {
      q_full.push_back(std::move(q_[static_cast<std::size_t>(bi)]));
    }
  }
  q_ = std::move(q_full);
  return true;
}

// ═══════════════════════════════════════════════════════════════════════════
// Whole-cone scaling operators
// ═══════════════════════════════════════════════════════════════════════════

void ConeNtScaling::apply_w(const Eigen::Ref<const Eigen::VectorXd>& x,
                            Eigen::Ref<Eigen::VectorXd> out) const {
  const int l = static_cast<int>(l_d_.size());
  out.head(l) = l_d_.cwiseProduct(x.head(l));
  for (const auto& [row, d] : l_extra_) {
    out[row] = d * x[row];
  }
  for (const QScaling& qs : q_) {
    out.segment(qs.offset, qs.size) =
        qs.beta * q_qrep(qs.v, x.segment(qs.offset, qs.size));
  }
  for (const SScaling& ss : s_) {
    const int mp = svec_size(ss.order);
    const Eigen::MatrixXd xm = smat(x.segment(ss.offset, mp), ss.order);
    out.segment(ss.offset, mp) = svec(ss.r * xm * ss.r.transpose());
  }
}

void ConeNtScaling::apply_w_inv(const Eigen::Ref<const Eigen::VectorXd>& x,
                                Eigen::Ref<Eigen::VectorXd> out) const {
  const int l = static_cast<int>(l_d_.size());
  out.head(l) = x.head(l).cwiseQuotient(l_d_);
  for (const auto& [row, d] : l_extra_) {
    out[row] = x[row] / d;
  }
  for (const QScaling& qs : q_) {
    // W^{-1} x = beta^{-1} (2 (Jv) ((Jv)'x) - J x) = beta^{-1} Q(Jv) x
    out.segment(qs.offset, qs.size) =
        q_qrep(q_jmul(qs.v), x.segment(qs.offset, qs.size)) / qs.beta;
  }
  for (const SScaling& ss : s_) {
    const int mp = svec_size(ss.order);
    const Eigen::MatrixXd xm = smat(x.segment(ss.offset, mp), ss.order);
    out.segment(ss.offset, mp) = svec(ss.rti.transpose() * xm * ss.rti);
  }
}

void ConeNtScaling::apply_wt(const Eigen::Ref<const Eigen::VectorXd>& x,
                             Eigen::Ref<Eigen::VectorXd> out) const {
  const int l = static_cast<int>(l_d_.size());
  out.head(l) = l_d_.cwiseProduct(x.head(l));
  for (const auto& [row, d] : l_extra_) {
    out[row] = d * x[row];
  }
  for (const QScaling& qs : q_) {  // W is symmetric for l/q blocks
    out.segment(qs.offset, qs.size) =
        qs.beta * q_qrep(qs.v, x.segment(qs.offset, qs.size));
  }
  for (const SScaling& ss : s_) {
    const int mp = svec_size(ss.order);
    const Eigen::MatrixXd xm = smat(x.segment(ss.offset, mp), ss.order);
    out.segment(ss.offset, mp) = svec(ss.r.transpose() * xm * ss.r);
  }
}

void ConeNtScaling::apply_wt_inv(const Eigen::Ref<const Eigen::VectorXd>& x,
                                 Eigen::Ref<Eigen::VectorXd> out) const {
  const int l = static_cast<int>(l_d_.size());
  out.head(l) = x.head(l).cwiseQuotient(l_d_);
  for (const auto& [row, d] : l_extra_) {
    out[row] = x[row] / d;
  }
  for (const QScaling& qs : q_) {
    out.segment(qs.offset, qs.size) =
        q_qrep(q_jmul(qs.v), x.segment(qs.offset, qs.size)) / qs.beta;
  }
  for (const SScaling& ss : s_) {
    const int mp = svec_size(ss.order);
    const Eigen::MatrixXd xm = smat(x.segment(ss.offset, mp), ss.order);
    out.segment(ss.offset, mp) =
        svec(ss.rti * xm * ss.rti.transpose());
  }
}

void ConeNtScaling::apply_h(const Eigen::Ref<const Eigen::VectorXd>& x,
                            Eigen::Ref<Eigen::VectorXd> out) const {
  const int l = static_cast<int>(l_d_.size());
  out.head(l) = l_d_.cwiseAbs2().cwiseProduct(x.head(l));
  for (const auto& [row, d] : l_extra_) {
    out[row] = d * d * x[row];
  }
  for (const QScaling& qs : q_) {
    // H x = beta^2 (2 wbar (wbar'x) - J x) = beta^2 Q(wbar) x
    out.segment(qs.offset, qs.size) =
        (qs.beta * qs.beta) * q_qrep(qs.wbar, x.segment(qs.offset, qs.size));
  }
  for (const SScaling& ss : s_) {
    const int mp = svec_size(ss.order);
    const Eigen::MatrixXd xm = smat(x.segment(ss.offset, mp), ss.order);
    out.segment(ss.offset, mp) = svec(ss.h_cong * xm * ss.h_cong);
  }
}

void ConeNtScaling::apply_h_inv(const Eigen::Ref<const Eigen::VectorXd>& x,
                                Eigen::Ref<Eigen::VectorXd> out) const {
  const int l = static_cast<int>(l_d_.size());
  out.head(l) = x.head(l).cwiseQuotient(l_d_.cwiseAbs2());
  for (const auto& [row, d] : l_extra_) {
    out[row] = x[row] / (d * d);
  }
  for (const QScaling& qs : q_) {
    // H^{-1} x = beta^{-2} (2 (Jwbar) ((Jwbar)'x) - J x) = beta^{-2} Q(Jwbar) x
    out.segment(qs.offset, qs.size) =
        q_qrep(q_jmul(qs.wbar), x.segment(qs.offset, qs.size)) /
        (qs.beta * qs.beta);
  }
  for (const SScaling& ss : s_) {
    const int mp = svec_size(ss.order);
    const Eigen::MatrixXd xm = smat(x.segment(ss.offset, mp), ss.order);
    out.segment(ss.offset, mp) =
        svec(ss.h_inv_cong * xm * ss.h_inv_cong);
  }
}

}  // namespace mipsolvers::engine
