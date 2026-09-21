#pragma once

// ═══════════════════════════════════════════════════════════════════════════
// Cone algebra for the native conic IPM (cvxopt conelp formulation).
//
// Cones: K = R^l_+ x Q^{q_0} x ... x S^{s_0} x ...  (nonnegative orthant,
// second-order/Lorentz cones, positive-semidefinite cones).  SDP blocks use
// the svec packing: column-major lower triangle with off-diagonals scaled by
// sqrt(2), so tr(SZ) = svec(S)' * svec(Z) and all inner products are plain
// dot products.
//
// Provides: svec/smat packing, Jordan-algebra primitives for Q^k, per-block
// Nesterov-Todd scalings (class ConeNtScaling), interior tests, and maximum
// step lengths to the cone boundary.
// ═══════════════════════════════════════════════════════════════════════════

#include <vector>

#include <Eigen/Core>

#include "mipsolvers/engine/problem_types.hpp"

namespace mipsolvers::engine {

// ═══════════════════════════════════════════════════════════════════════════
// svec packing utilities
// ═══════════════════════════════════════════════════════════════════════════

/// Number of packed entries for a symmetric matrix of order p.
[[nodiscard]] inline int svec_size(int p) { return p * (p + 1) / 2; }

/// Packed index of entry (i, j) with i >= j (column-major lower triangle).
[[nodiscard]] inline int svec_index(int i, int j, int p) {
  return j * p - j * (j - 1) / 2 + (i - j);
}

/// Pack a symmetric p x p matrix: svec with off-diagonals scaled by sqrt(2).
[[nodiscard]] Eigen::VectorXd svec(const Eigen::Ref<const Eigen::MatrixXd>& x);

/// Unpack a svec-packed vector of length p(p+1)/2 to a symmetric p x p matrix.
[[nodiscard]] Eigen::MatrixXd smat(const Eigen::Ref<const Eigen::VectorXd>& v, int p);

// ═══════════════════════════════════════════════════════════════════════════
// Cone layout: block offsets derived from ConeDims
// ═══════════════════════════════════════════════════════════════════════════

struct ConeLayout {
  int l{0};                       ///< Rows of the nonnegative orthant block.
  std::vector<int> q_offsets;     ///< Row offset of each SOC block.
  std::vector<int> q_sizes;       ///< Size of each SOC block (>= 1).
  std::vector<int> s_offsets;     ///< Row offset of each SDP block (packed).
  std::vector<int> s_orders;      ///< Matrix order p of each SDP block.
  int total{0};                   ///< Total conic rows (== dims.total()).
  int degree{0};                  ///< Cone degree nu = l + #q + sum(p_i).
  Eigen::VectorXd e;              ///< Packed unit vector of the cone.

  /// Number of independently scalable blocks (l counts once when l > 0).
  [[nodiscard]] int num_blocks() const {
    return (l > 0 ? 1 : 0) + static_cast<int>(q_sizes.size()) +
           static_cast<int>(s_orders.size());
  }

  [[nodiscard]] static ConeLayout build(const ConeDims& dims);
};

// ═══════════════════════════════════════════════════════════════════════════
// Second-order cone Jordan algebra (vectors of size k >= 2)
// ═══════════════════════════════════════════════════════════════════════════

/// det(u) = u0^2 - ||u1||^2 (the Lorentz "determinant").
[[nodiscard]] double q_det(const Eigen::Ref<const Eigen::VectorXd>& u);

/// Jordan product u o v = (u'v, u0*v1 + v0*u1).
[[nodiscard]] Eigen::VectorXd q_jordan_product(
    const Eigen::Ref<const Eigen::VectorXd>& u,
    const Eigen::Ref<const Eigen::VectorXd>& v);

/// Jordan inverse u^{-1} = J u / det(u), J = diag(1, -1, ..., -1).
[[nodiscard]] Eigen::VectorXd q_jordan_inverse(
    const Eigen::Ref<const Eigen::VectorXd>& u);

/// Jordan division L(u)^{-1} b: solve u o y = b for y (the "o\" operator).
/// Note L(u)^{-1} != L(u^{-1}) for non-associative Jordan algebras; this is
/// the solve required by the Mehrotra corrector.  u must be interior.
[[nodiscard]] Eigen::VectorXd q_jordan_solve(
    const Eigen::Ref<const Eigen::VectorXd>& u,
    const Eigen::Ref<const Eigen::VectorXd>& b);

/// Jordan square root of an interior vector u.
[[nodiscard]] Eigen::VectorXd q_jordan_sqrt(
    const Eigen::Ref<const Eigen::VectorXd>& u);

// ═══════════════════════════════════════════════════════════════════════════
// Interior tests (strict)
// ═══════════════════════════════════════════════════════════════════════════

[[nodiscard]] bool l_interior(const double* u, int n);
[[nodiscard]] bool q_interior(const Eigen::Ref<const Eigen::VectorXd>& u);
[[nodiscard]] bool s_interior(const Eigen::Ref<const Eigen::VectorXd>& u, int p);

// ═══════════════════════════════════════════════════════════════════════════
// Maximum step to the cone boundary: sup{ alpha >= 0 : u + alpha*du in cl K }.
// Returns +infinity when the ray never leaves the cone.  u must be interior.
// ═══════════════════════════════════════════════════════════════════════════

[[nodiscard]] double l_max_step(const double* u, const double* du, int n);
[[nodiscard]] double q_max_step(const Eigen::Ref<const Eigen::VectorXd>& u,
                                const Eigen::Ref<const Eigen::VectorXd>& du);
[[nodiscard]] double s_max_step(const Eigen::Ref<const Eigen::VectorXd>& u,
                                const Eigen::Ref<const Eigen::VectorXd>& du,
                                int p);

// ═══════════════════════════════════════════════════════════════════════════
// Minimal shift alpha >= 0 such that u + alpha*e is on the boundary closure;
// u + (alpha + eps)*e is strictly interior.  0 when u is already interior.
// ═══════════════════════════════════════════════════════════════════════════

[[nodiscard]] double l_shift_to_interior(const double* u, int n);
[[nodiscard]] double q_shift_to_interior(
    const Eigen::Ref<const Eigen::VectorXd>& u);
[[nodiscard]] double s_shift_to_interior(
    const Eigen::Ref<const Eigen::VectorXd>& u, int p);

// ═══════════════════════════════════════════════════════════════════════════
// Nesterov-Todd scaling for a primal-dual interior pair (s, z).
//
// Per block the scaling provides congruence/Jordan operators:
//   apply_w       primal unscaling   P^{-1} (l: d o x; q: W x; s: R X R')
//   apply_w_inv   primal scaling     P     (l: x/d; q: W^{-1} x; s: R^{-1} X R^{-T})
//   apply_wt      dual scaling       D     (l: d o x; q: W x; s: R' X R)
//   apply_wt_inv  dual unscaling     D^{-1}(l: x/d; q: W^{-1} x; s: Rti X Rti')
//   apply_h       Hessian            H = P^{-1} D  (l: d^2 o x; q: beta^2(2 wbar wbar' - J) x;
//                                                 s: (R R') X (R R'))
//   apply_h_inv   inverse Hessian    H^{-1} = D^{-1} P (l: x/d^2; q: beta^{-2}(2 Jwbar Jwbar' - J) x;
//                                                     s: M X M with M = Rti Rti')
// The scaled point lambda satisfies lambda = P s = D z per block.
// ═══════════════════════════════════════════════════════════════════════════

class ConeNtScaling {
 public:
  /// NT scaling of a q-block of size >= 2.
  struct QScaling {
    int offset{0};
    int size{0};
    double beta{1.0};
    Eigen::VectorXd v;     ///< Jordan sqrt of wbar.
    Eigen::VectorXd wbar;  ///< Normalized scaling point (det(wbar) = 1).
  };

  /// NT scaling of an SDP block of order p.
  struct SScaling {
    int offset{0};
    int order{0};
    Eigen::MatrixXd r;      ///< R = Ls V Sigma^{-1/2} (R' Z R = diag(sigma)).
    Eigen::MatrixXd rti;    ///< R^{-T} = Ls^{-T} V Sigma^{1/2}.
    Eigen::VectorXd sigma;  ///< diag of the scaled point (lambda = svec(diag)).
    Eigen::MatrixXd h_cong;      ///< R R' (congruence factor of H).
    Eigen::MatrixXd h_inv_cong;  ///< Rti Rti' (congruence factor of H^{-1}).
  };

  ConeNtScaling() = default;

  /// Compute the NT scaling of every block.  Returns false when s or z is not
  /// strictly interior (caller should treat as a numerical failure).
  [[nodiscard]] bool compute(const ConeLayout& layout,
                             const Eigen::VectorXd& s,
                             const Eigen::VectorXd& z);

  /// Whole-cone operators (in/out are full conic-row vectors, size total()).
  void apply_w(const Eigen::Ref<const Eigen::VectorXd>& x,
               Eigen::Ref<Eigen::VectorXd> out) const;
  void apply_w_inv(const Eigen::Ref<const Eigen::VectorXd>& x,
                   Eigen::Ref<Eigen::VectorXd> out) const;
  void apply_wt(const Eigen::Ref<const Eigen::VectorXd>& x,
                Eigen::Ref<Eigen::VectorXd> out) const;
  void apply_wt_inv(const Eigen::Ref<const Eigen::VectorXd>& x,
                    Eigen::Ref<Eigen::VectorXd> out) const;
  void apply_h(const Eigen::Ref<const Eigen::VectorXd>& x,
               Eigen::Ref<Eigen::VectorXd> out) const;
  void apply_h_inv(const Eigen::Ref<const Eigen::VectorXd>& x,
                   Eigen::Ref<Eigen::VectorXd> out) const;

  /// Scaled point lambda = P s = D z (size total()).
  [[nodiscard]] const Eigen::VectorXd& lambda() const { return lambda_; }

  /// NT scaling d (size l) of the orthant block; empty when l == 0.
  [[nodiscard]] const Eigen::VectorXd& l_scaling() const { return l_d_; }

  /// q-blocks of size >= 2 (size-1 blocks are folded into l_extra_).
  [[nodiscard]] const std::vector<QScaling>& q_scalings() const { return q_; }
  [[nodiscard]] const std::vector<SScaling>& s_scalings() const { return s_; }

  /// Extra scalar (l-style) rows from q-blocks of size 1: (row, d).
  [[nodiscard]] const std::vector<std::pair<int, double>>& l_extra() const {
    return l_extra_;
  }

 private:
  Eigen::VectorXd l_d_;  ///< Orthant scaling d_i = sqrt(s_i/z_i).
  std::vector<std::pair<int, double>> l_extra_;  ///< (row, d) for size-1 q-blocks.
  std::vector<QScaling> q_;
  std::vector<SScaling> s_;
  Eigen::VectorXd lambda_;
};

}  // namespace mipsolvers::engine
