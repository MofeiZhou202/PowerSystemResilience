/// test_sppt_dae_wellposedness.cpp
/// ===============================
/// MR8: DAE index-1 well-posedness (Thm. 6.6, the dynamic extension of the
/// steady-state reference well-posedness theorem).  A semi-explicit DAE
///   x' = f(x,y),   0 = g(x,y)
/// is locally index-1 at a consistent operating point when the algebraic
/// sub-Jacobian d g / d y is nonsingular. We assemble the linearization via
/// dynamic_dae_diagnostics and verify full rank on one declared test point.

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Dense>

#include "hacdcpf/dynamics/DynamicDaeDiagnostics.hpp"
#include "hacdcpf/dynamics/dynamics.hpp"
#include "hacdcpf/model/hybrid_power_system.hpp"
#include "hacdcpf/sppt/metamorphic.hpp"

using namespace hacdcpf;
using namespace hacdcpf::dynamics;

namespace {

// Minimal well-posed transient case: a slack machine feeding a load through one
// line.  The synchronous machine contributes the differential states; the two
// buses contribute the (Vr, Vi) algebraic states.
HybridPowerSystem make_transient_case() {
  HybridPowerSystem sys;
  sys.name = "sppt_dae_2bus";
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;
  sys.ac.freq_hz = 50.0;

  ACBus b1;
  b1.index = 1;
  b1.bus_type = BusType::SLACK;
  b1.vm_pu = 1.02;
  b1.in_service = true;
  ACBus b2;
  b2.index = 2;
  b2.bus_type = BusType::PQ;
  b2.vm_pu = 1.0;
  b2.in_service = true;
  sys.ac.buses = {b1, b2};

  ACBranch br;
  br.index = 1;
  br.from_bus = 1;
  br.to_bus = 2;
  br.r_pu = 0.01;
  br.x_pu = 0.05;
  br.tap = 1.0;
  br.in_service = true;
  sys.ac.branches = {br};

  Generator g;
  g.index = 1;
  g.bus = 1;
  g.is_slack = true;
  g.in_service = true;
  g.vg_pu = 1.02;
  g.pg_mw = 30.0;
  g.pmax_mw = 200.0;
  g.pmin_mw = 0.0;
  g.qmax_mvar = 100.0;
  g.qmin_mvar = -100.0;
  g.xdpp_pu = 0.20;
  sys.ac.generators = {g};

  Load ld;
  ld.index = 1;
  ld.bus = 2;
  ld.p_mw = 30.0;
  ld.q_mvar = 10.0;
  ld.in_service = true;
  sys.ac.loads = {ld};
  return sys;
}

}  // namespace

TEST_CASE("MR8: the DAE is index-1 (algebraic sub-Jacobian is nonsingular)",
          "[sppt][dae][mr8][dynamics]") {
  const HybridPowerSystem sys = make_transient_case();

  DynamicSolverOptions opt;
  opt.t_end_s = 0.02;
  opt.dt_s = 0.005;
  opt.run_power_flow_initialization = true;
  opt.use_consistent_dynamic_initialization = true;

  DynamicModelBuilder builder;
  DynamicSystem dyn = builder.build(sys, opt);

  DynamicDaeDiagnosticOptions dopt;
  dopt.positive_sequence_projection = true;
  dopt.build_jacobian = true;

  DynamicDaeDiagnostics diag = dynamic_dae_diagnostics(dyn, dopt);

  INFO(diag.message);
  REQUIRE(diag.success);
  REQUIRE(diag.n_algebraic > 0);
  REQUIRE(diag.n_differential > 0);
  REQUIRE(diag.jacobian.rows() == diag.n_variables);
  REQUIRE(diag.jacobian.cols() == diag.n_variables);

  // Algebraic variables/equations occupy the leading n_algebraic indices
  // (mass_diag == 0), so d g / d y is the leading square block.
  for (int i = 0; i < diag.n_algebraic; ++i)
    CHECK(diag.mass_diag[i] == 0.0);
  for (int k = 0; k < diag.n_differential; ++k)
    CHECK(diag.mass_diag[diag.n_algebraic + k] == 1.0);

  const Eigen::MatrixXd Jgy =
      diag.jacobian.topLeftCorner(diag.n_algebraic, diag.n_algebraic);

  Eigen::FullPivLU<Eigen::MatrixXd> lu(Jgy);
  INFO("algebraic sub-Jacobian rank = " << lu.rank() << " / " << diag.n_algebraic);
  CHECK(lu.rank() == diag.n_algebraic);   // nonsingular  =>  index-1  =>  well-posed
  CHECK(lu.isInvertible());

  // The reduced (algebraic-eliminated) state matrix and its spectrum exist,
  // which is only possible when d g / d y was invertible during elimination.
  CHECK(diag.reduced_jacobian.rows() == diag.n_differential);
  CHECK(diag.eigenvalues.size() == diag.n_differential);
}

TEST_CASE("MR3t: transient trajectories are preserved through projection",
          "[sppt][metamorphic][mr3][dynamics]") {
  const HybridPowerSystem sys = make_transient_case();
  const auto r = hacdcpf::sppt::mr3_semantic_preservation_transient(sys, 1e-6);
  INFO("residual=" << r.residual << " (" << r.detail << ")");
  CHECK(r.passed);
}
