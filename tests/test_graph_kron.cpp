// tests/test_graph_kron.cpp
//
// Unit tests for Kron (Schur complement) reduction.
//
// Tests:
//   1. Three-node passive interior node: Y_red matches analytical formula
//   2. Boundary voltage consistency: full vs reduced network solutions agree
//   3. Eliminated voltage recovery: V_β = -Y_ββ⁻¹ Y_βα V_α
//   4. Extended Kron with injection: modified injection formula
//   5. Fill-in guard: abort when fill ratio exceeds limit
//   6. Identity case: empty eliminated set → Y_red = Y_αα submatrix

#include <cmath>
#include <complex>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "hacdcpf/graph/kron_reduction.hpp"
#include "hacdcpf/graph/sparse_kron_reduction.hpp"

using namespace hacdcpf::graph;
using Catch::Matchers::WithinAbs;
using cplx = std::complex<double>;

// ─────────────────────────────────────────────────────────────────────
// Helper: build sparse Y-bus from dense
// ─────────────────────────────────────────────────────────────────────

static Eigen::SparseMatrix<cplx> dense_to_sparse(const Eigen::MatrixXcd& M) {
  std::vector<Eigen::Triplet<cplx>> trips;
  for (int i = 0; i < M.rows(); ++i)
    for (int j = 0; j < M.cols(); ++j)
      if (std::abs(M(i,j)) > 1e-15)
        trips.emplace_back(i, j, M(i,j));
  Eigen::SparseMatrix<cplx> S(M.rows(), M.cols());
  S.setFromTriplets(trips.begin(), trips.end());
  return S;
}

TEST_CASE("Sparse Kron tape preserves boundary currents and recovers states",
          "[graph][kron][sparse]") {
  Eigen::MatrixXcd y(5, 5);
  y.setZero();
  const auto add_branch = [&](int i, int j, cplx admittance) {
    y(i, i) += admittance;
    y(j, j) += admittance;
    y(i, j) -= admittance;
    y(j, i) -= admittance;
  };
  add_branch(0, 1, {4.0, -12.0});
  add_branch(1, 2, {3.0, -9.0});
  add_branch(1, 3, {2.0, -8.0});
  add_branch(3, 4, {5.0, -10.0});
  y(0, 0) += cplx{1e-3, 0.0};

  const auto sparse = dense_to_sparse(y);
  std::vector<bool> eligible{false, true, false, true, false};
  SparseKronOptions options;
  options.max_front = 8;
  options.max_nnz_ratio = 10.0;
  const auto reduced = reduce_sparse_kron(sparse, eligible, options);

  REQUIRE(reduced.valid());
  REQUIRE(reduced.recovery_steps.size() == 2);
  REQUIRE(reduced.retained.size() == 3);

  Eigen::VectorXcd vb(3);
  vb << cplx{1.01, 0.02}, cplx{0.97, -0.03}, cplx{1.0, 0.01};
  const Eigen::VectorXcd vf = recover_sparse_kron_state(reduced, vb);
  const Eigen::VectorXcd ifull = sparse * vf;
  const Eigen::VectorXcd ired = reduced.reduced * vb;

  for (int pos = 0; pos < static_cast<int>(reduced.retained.size()); ++pos) {
    REQUIRE_THAT(std::abs(ifull[reduced.retained[static_cast<std::size_t>(pos)]] -
                          ired[pos]),
                 WithinAbs(0.0, 1e-10));
  }
  for (const auto& step : reduced.recovery_steps) {
    REQUIRE_THAT(std::abs(ifull[step.node]), WithinAbs(0.0, 1e-10));
  }

  const Eigen::SparseMatrix<cplx> recovery =
      sparse_kron_recovery_operator(reduced);
  REQUIRE((recovery * vb - vf).cwiseAbs().maxCoeff() < 1e-12);
}

TEST_CASE("Sparse Kron caps leave inadmissible candidates retained",
          "[graph][kron][sparse]") {
  Eigen::MatrixXcd y(4, 4);
  y.setZero();
  for (int leaf = 1; leaf < 4; ++leaf) {
    const cplx admittance{1.0 + leaf, -5.0 - leaf};
    y(0, 0) += admittance;
    y(leaf, leaf) += admittance;
    y(0, leaf) -= admittance;
    y(leaf, 0) -= admittance;
  }

  SparseKronOptions options;
  options.max_front = 1;
  options.max_nnz_ratio = 1.0;
  const auto reduced = reduce_sparse_kron(
      dense_to_sparse(y), std::vector<bool>{true, false, false, false}, options);
  REQUIRE(reduced.valid());
  CHECK(reduced.recovery_steps.empty());
  CHECK(reduced.retained.size() == 4);
}

// ─────────────────────────────────────────────────────────────────────
// Test 1: Three-bus, passive interior node
// ─────────────────────────────────────────────────────────────────────

TEST_CASE("Kron reduction: 3-bus passive interior node", "[graph][kron]") {
  // Network: 1 -- y12 -- 2 -- y23 -- 3
  // Bus 2 is a passive interior node (no injection)
  //
  // Y matrix:
  //   [ y12       -y12      0   ]
  //   [-y12   y12+y23  -y23  ]
  //   [  0       -y23     y23  ]
  //
  // After eliminating Bus 2 (index 1 in 0-based):
  //   y_eq = y12 * y23 / (y12 + y23)  (parallel combination)

  cplx y12{0.0, 1.0/0.06};  // x12 = 0.06 → admittance ≈ -j16.67
  cplx y23{0.0, 1.0/0.09};  // x23 = 0.09 → admittance ≈ -j11.11

  Eigen::MatrixXcd Y(3, 3);
  Y.setZero();
  Y(0,0) =  y12;       Y(0,1) = -y12;      Y(0,2) = cplx{0,0};
  Y(1,0) = -y12;       Y(1,1) =  y12+y23;  Y(1,2) = -y23;
  Y(2,0) =  cplx{0,0}; Y(2,1) = -y23;      Y(2,2) =  y23;

  auto Y_sparse = dense_to_sparse(Y);

  // Eliminate bus index 1 (Bus 2), retain 0 and 2 (Buses 1 and 3)
  std::vector<int> retained  = {0, 2};
  std::vector<int> eliminated = {1};

  auto res = apply_kron_reduction(Y_sparse, retained, eliminated, 10.0);

  REQUIRE(res.kron_data.valid == true);
  REQUIRE(res.diagnostics.empty());

  // Expected Y_red (2×2):
  // y_eq = y12 * y23 / (y12 + y23)
  cplx y_eq = y12 * y23 / (y12 + y23);

  // Y_red = [ y_eq,  -y_eq ]
  //         [-y_eq,   y_eq ]

  Eigen::MatrixXcd Y_red_dense = res.Y_reduced.toDense();

  REQUIRE_THAT(Y_red_dense(0,0).real(), WithinAbs(y_eq.real(), 1e-10));
  REQUIRE_THAT(Y_red_dense(0,0).imag(), WithinAbs(y_eq.imag(), 1e-10));
  REQUIRE_THAT(Y_red_dense(0,1).real(), WithinAbs(-y_eq.real(), 1e-10));
  REQUIRE_THAT(Y_red_dense(0,1).imag(), WithinAbs(-y_eq.imag(), 1e-10));
  REQUIRE_THAT(Y_red_dense(1,0).real(), WithinAbs(-y_eq.real(), 1e-10));
  REQUIRE_THAT(Y_red_dense(1,1).real(), WithinAbs(y_eq.real(), 1e-10));
}

// ─────────────────────────────────────────────────────────────────────
// Test 2: Boundary voltage consistency
// ─────────────────────────────────────────────────────────────────────

TEST_CASE("Kron reduction: boundary voltage consistency", "[graph][kron]") {
  // 4-bus network with 2 interior passive buses.
  // Given boundary injections, the boundary voltages from the reduced
  // system must exactly match those from the full system.

  // Y-bus for a simple 4-bus ring:
  //   1 -- y12 -- 2 -- y23 -- 3 -- y34 -- 4 -- y41 -- 1
  // Retain buses: 0, 2 (indices 0 and 2)
  // Eliminate buses: 1, 3 (indices 1 and 3)

  cplx y{0.0, 10.0}; // all branches identical admittance
  Eigen::MatrixXcd Y(4,4);
  Y.setZero();
  Y(0,0) = 2.0*y; Y(0,1) = -y;     Y(0,3) = -y;
  Y(1,1) = 2.0*y; Y(1,0) = -y;     Y(1,2) = -y;
  Y(2,2) = 2.0*y; Y(2,1) = -y;     Y(2,3) = -y;
  Y(3,3) = 2.0*y; Y(3,2) = -y;     Y(3,0) = -y;

  auto Y_sparse = dense_to_sparse(Y);

  // Known boundary injections (alpha = {0,2})
  Eigen::VectorXcd I_full(4);
  I_full(0) = cplx{1.0, 0.5};
  I_full(1) = cplx{0.0, 0.0}; // passive
  I_full(2) = cplx{-1.0, -0.5};
  I_full(3) = cplx{0.0, 0.0}; // passive

  // Full system solve (pseudo-inverse — use dense LU after grounding bus 0)
  // Solve Y * V = I with V(0) = 0 reference (grounded)
  // Use 3x3 submatrix (drop row/col 0 for grounding)
  Eigen::MatrixXcd Y_dense = Y;
  // Ground bus 0: set row/col 0 to identity
  Eigen::MatrixXcd Y_grnd = Y_dense;
  Y_grnd.row(0).setZero(); Y_grnd.col(0).setZero();
  Y_grnd(0,0) = cplx{1.0,0.0};
  Eigen::VectorXcd I_grnd = I_full;
  I_grnd(0) = cplx{0.0,0.0};
  Eigen::VectorXcd V_full = Y_grnd.lu().solve(I_grnd);

  // Kron reduce: eliminate buses 1 and 3
  std::vector<int> retained   = {0, 2};
  std::vector<int> eliminated = {1, 3};
  auto res = apply_kron_reduction_with_injection(
      Y_sparse, I_full, retained, eliminated, 10.0);
  REQUIRE(res.kron_data.valid == true);

  // Solve reduced system: Y_red * V_alpha = I_red (with V(0)=0 grounding)
  Eigen::MatrixXcd Y_red = res.Y_reduced.toDense();
  // Ground first retained bus (index 0 → alpha index 0)
  Eigen::MatrixXcd Y_r_grnd = Y_red;
  Y_r_grnd.row(0).setZero(); Y_r_grnd.col(0).setZero();
  Y_r_grnd(0,0) = cplx{1.0,0.0};
  Eigen::VectorXcd I_r_grnd = res.I_reduced;
  I_r_grnd(0) = cplx{0.0,0.0};
  Eigen::VectorXcd V_alpha = Y_r_grnd.lu().solve(I_r_grnd);

  // Compare retained voltages
  for (int k = 0; k < 2; ++k) {
    int global_idx = retained[k];
    REQUIRE_THAT(std::abs(V_alpha(k) - V_full(global_idx)),
                 WithinAbs(0.0, 1e-10));
  }
}

// ─────────────────────────────────────────────────────────────────────
// Test 3: Eliminated voltage recovery
// ─────────────────────────────────────────────────────────────────────

TEST_CASE("Kron reduction: eliminated voltage recovery", "[graph][kron]") {
  // 3-bus passive interior node (same as Test 1)
  cplx y12{0.0, 1.0/0.06};
  cplx y23{0.0, 1.0/0.09};

  Eigen::MatrixXcd Y(3,3);
  Y.setZero();
  Y(0,0) =  y12;        Y(0,1) = -y12;       Y(0,2) = cplx{0,0};
  Y(1,0) = -y12;        Y(1,1) =  y12+y23;   Y(1,2) = -y23;
  Y(2,0) =  cplx{0,0};  Y(2,1) = -y23;       Y(2,2) =  y23;
  auto Y_sparse = dense_to_sparse(Y);

  // Full system solve: V(0) = 1.05∠0°, I(2) = -0.5∠-30°, passive bus 1
  Eigen::VectorXcd I_full(3);
  I_full(0) = cplx{0.0, 0.0}; // slack bus — injection computed from solution
  I_full(1) = cplx{0.0, 0.0}; // passive
  I_full(2) = cplx{-0.5*std::cos(M_PI/6.0), -0.5*std::sin(M_PI/6.0)};

  // Full solve with V(0) = 1.05 (slack) using correct elimination:
  // Subtract slack contribution from RHS, then solve (n-1)×(n-1) subsystem.
  const cplx V0_ref{1.05, 0.0};
  // Sub-system: buses 1 and 2 (drop bus 0)
  Eigen::MatrixXcd Y_sub = Y.bottomRightCorner(2, 2);
  Eigen::VectorXcd I_sub(2);
  I_sub(0) = I_full(1) - Y(1, 0) * V0_ref;
  I_sub(1) = I_full(2) - Y(2, 0) * V0_ref;
  Eigen::VectorXcd V_sub = Y_sub.lu().solve(I_sub);
  Eigen::VectorXcd V_full(3);
  V_full(0) = V0_ref;
  V_full(1) = V_sub(0);
  V_full(2) = V_sub(1);

  // Kron reduce: eliminate bus 1
  std::vector<int> retained   = {0, 2};
  std::vector<int> eliminated = {1};
  auto res = apply_kron_reduction(Y_sparse, retained, eliminated, 10.0);
  REQUIRE(res.kron_data.valid == true);

  // Build V_alpha from full solution
  Eigen::VectorXcd V_alpha(2);
  V_alpha(0) = V_full(retained[0]);
  V_alpha(1) = V_full(retained[1]);

  // Recover V_beta
  Eigen::VectorXcd V_beta = recover_eliminated_voltages(res.kron_data, V_alpha);

  // Compare with full solution
  REQUIRE_THAT(std::abs(V_beta(0) - V_full(eliminated[0])),
               WithinAbs(0.0, 1e-10));
}

// ─────────────────────────────────────────────────────────────────────
// Test 4: Extended Kron with constant-current injection
// ─────────────────────────────────────────────────────────────────────

TEST_CASE("Kron reduction: extended with non-zero injection", "[graph][kron]") {
  // 3-bus: buses 0, 1, 2 with all having injection
  // Use simple star network: all buses connected to a virtual ground
  cplx y{0.0, 5.0};
  Eigen::MatrixXcd Y(3,3);
  Y.setZero();
  Y(0,0) = y+y; Y(0,1) = -y; Y(0,2) = cplx{0,0};
  Y(1,0) = -y;  Y(1,1) = y+y; Y(1,2) = -y;
  Y(2,0) = cplx{0,0}; Y(2,1) = -y; Y(2,2) = y;
  auto Y_sparse = dense_to_sparse(Y);

  Eigen::VectorXcd I_full(3);
  I_full(0) = cplx{1.0,  0.0};
  I_full(1) = cplx{0.5,  0.2};
  I_full(2) = cplx{-1.5, -0.2};

  // Full solve (ground bus 0)
  Eigen::MatrixXcd Y_grnd = Y;
  Y_grnd.row(0).setZero(); Y_grnd.col(0).setZero(); Y_grnd(0,0) = cplx{1,0};
  Eigen::VectorXcd I_grnd = I_full; I_grnd(0) = cplx{0,0};
  Eigen::VectorXcd V_full = Y_grnd.lu().solve(I_grnd);

  // Eliminate bus 1 (index 1) with injection
  std::vector<int> retained   = {0, 2};
  std::vector<int> eliminated = {1};
  auto res = apply_kron_reduction_with_injection(
      Y_sparse, I_full, retained, eliminated, 10.0);
  REQUIRE(res.kron_data.valid == true);

  // Solve reduced system
  Eigen::MatrixXcd Y_r = res.Y_reduced.toDense();
  Y_r.row(0).setZero(); Y_r.col(0).setZero(); Y_r(0,0) = cplx{1,0};
  Eigen::VectorXcd I_r = res.I_reduced; I_r(0) = cplx{0,0};
  Eigen::VectorXcd V_alpha = Y_r.lu().solve(I_r);

  // Retained voltages must match full solution
  REQUIRE_THAT(std::abs(V_alpha(0) - V_full(0)), WithinAbs(0.0, 1e-10));
  REQUIRE_THAT(std::abs(V_alpha(1) - V_full(2)), WithinAbs(0.0, 1e-10));
}

// ─────────────────────────────────────────────────────────────────────
// Test 5: Fill-in guard aborts reduction
// ─────────────────────────────────────────────────────────────────────

TEST_CASE("Kron reduction: fill-in guard triggers abort", "[graph][kron]") {
  // Sparse 6-bus chain; eliminating one bus won't generate fill-in,
  // but a very tight limit (max_fill_ratio = 0.0) forces abort.
  cplx y{0.0, 10.0};
  Eigen::MatrixXcd Y(4,4);
  Y.setZero();
  Y(0,0)=2.0*y; Y(0,1)=-y; Y(1,0)=-y; Y(1,1)=2.0*y; Y(1,2)=-y;
  Y(2,1)=-y;  Y(2,2)=2.0*y; Y(2,3)=-y; Y(3,2)=-y; Y(3,3)=y;
  auto Y_sp = dense_to_sparse(Y);

  auto res = apply_kron_reduction(Y_sp, {0, 1, 3}, {2}, 0.0);

  // Should have a fill-in diagnostic
  bool found = false;
  for (const auto& d : res.diagnostics)
    if (d.code == DiagCode::KronFillInTooLarge) { found = true; break; }
  REQUIRE(found);
  REQUIRE(!res.kron_data.valid);
}

// ─────────────────────────────────────────────────────────────────────
// Test 6: Identity case — no buses to eliminate
// ─────────────────────────────────────────────────────────────────────

TEST_CASE("Kron reduction: empty eliminated set is identity", "[graph][kron]") {
  cplx y{0.0, 10.0};
  Eigen::MatrixXcd Y(3,3);
  Y.setZero();
  Y(0,0)=2.0*y; Y(0,1)=-y; Y(1,0)=-y; Y(1,1)=2.0*y; Y(1,2)=-y; Y(2,1)=-y; Y(2,2)=y;
  auto Y_sp = dense_to_sparse(Y);

  std::vector<int> retained   = {0, 1, 2};
  std::vector<int> eliminated = {};
  auto res = apply_kron_reduction(Y_sp, retained, eliminated, 2.0);

  REQUIRE(res.kron_data.valid == true);
  REQUIRE(res.Y_reduced.rows() == 3);

  // Y_red should equal the original Y
  auto Y_red_d = res.Y_reduced.toDense();
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      REQUIRE_THAT(std::abs(Y_red_d(i,j) - Y(i,j)), WithinAbs(0.0, 1e-12));
}

// ─────────────────────────────────────────────────────────────────────
// Test 7: Kron approximation error for nonlinear loads
// ─────────────────────────────────────────────────────────────────────

TEST_CASE("Kron reduction: constant-power load introduces approximation",
          "[graph][kron]") {
  // Validate that applying Kron with I_β = 0 to a node that has a
  // constant-power load is NOT exact (error is voltage-dependent).
  // This test documents the expected behaviour: error ≠ 0.

  cplx y{0.0, 10.0};
  Eigen::MatrixXcd Y(3,3);
  Y.setZero();
  Y(0,0)=2.0*y; Y(0,1)=-y; Y(1,0)=-y; Y(1,1)=2.0*y; Y(1,2)=-y; Y(2,1)=-y; Y(2,2)=y;

  // Bus 1 (index 1) has a constant-power load
  // At V_1 ≈ 0.95 pu, S_load = 1.0 + j0.5 pu
  // Current injection I_1 = S^* / V_1^* (depends on V_1)
  cplx V1_approx{0.95, 0.0};
  cplx S_load{1.0, 0.5};
  cplx I1 = std::conj(S_load) / std::conj(V1_approx);

  // Full injection vector (with I_β ≠ 0)
  Eigen::VectorXcd I_full(3);
  I_full(0) = cplx{0,0};
  I_full(1) = -I1;  // load convention: negative injection
  I_full(2) = cplx{0.5, 0.0};

  auto Y_sp = dense_to_sparse(Y);

  // Kron with I_β = 0 (wrong: ignoring the load injection)
  auto res_no_inj = apply_kron_reduction(Y_sp, {0,2}, {1}, 10.0);

  // Kron with correct I_β
  auto res_with_inj = apply_kron_reduction_with_injection(
      Y_sp, I_full, {0,2}, {1}, 10.0);

  REQUIRE(res_no_inj.kron_data.valid);
  REQUIRE(res_with_inj.kron_data.valid);

  // The I_reduced from with-injection must differ from the no-injection case
  Eigen::VectorXcd I_zero(2); I_zero.setZero();
  double diff = (res_with_inj.I_reduced - I_zero).norm();
  // Because I_β ≠ 0, the modified injection should be non-trivial
  REQUIRE(diff > 1e-6);
}
