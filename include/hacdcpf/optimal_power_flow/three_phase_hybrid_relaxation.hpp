#pragma once

#include <string>
#include <vector>

#include <Eigen/Core>

#include "hacdcpf/optimal_power_flow/three_phase_hybrid_opf.hpp"

namespace hacdcpf::opf::phase_hybrid {

struct ThreePhaseHybridRelaxationOptions {
  int max_outer_approximation_rounds{12};
  int cost_tangent_points{9};
  double cone_tolerance{1e-6};
  double solver_tolerance{1e-8};
  bool verbose{false};
};

struct ThreePhaseHybridRelaxationResult {
  bool solved{false};
  bool outer_relaxation_valid{false};
  bool dual_certificate_available{false};
  bool soc_outer_approximation_converged{false};
  bool ac_passivity_cut_applied{false};
  bool dc_passivity_cut_applied{false};
  std::string status;
  std::string solver;
  double objective_lower_bound{0.0};
  double lp_primal_objective{0.0};
  double lp_primal_dual_gap{0.0};
  double runtime_ms{0.0};
  double max_soc_violation{0.0};
  double max_ac_lift_violation{0.0};
  double max_ac_psd_violation{0.0};
  double max_dc_lift_violation{0.0};
  double max_converter_apparent_power_violation{0.0};
  double max_converter_current_violation{0.0};
  double primal_residual{0.0};
  double dual_residual{0.0};
  int rounds{0};
  int cuts_added{0};
  int variables{0};
  int equalities{0};
  int inequalities{0};
  Eigen::VectorXd primal;
  std::vector<double> generator_active_power_pu;
  std::vector<double> generator_reactive_power_pu;
  std::vector<double> round_lower_bounds;
  std::vector<std::string> model_limitations;
};

/// Solve a polyhedral outer approximation of the phase-domain AC/DC SOCP
/// relaxation. Every generated cone cut is necessary for the SOCP feasible
/// set, so every solved LP round remains an outer relaxation of the original
/// nonconvex OPF. Convex quadratic generation costs are underestimated by
/// supporting tangents.
ThreePhaseHybridRelaxationResult solve_three_phase_hybrid_opf_relaxation(
    const ThreePhaseHybridOPFCase& problem,
    const ThreePhaseHybridRelaxationOptions& options = {});

std::vector<ThreePhaseHybridRelaxationResult>
solve_three_phase_hybrid_opf_relaxation_sequence(
    const std::vector<ThreePhaseHybridOPFCase>& problems,
    const ThreePhaseHybridRelaxationOptions& options = {});

}  // namespace hacdcpf::opf::phase_hybrid
