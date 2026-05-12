#include "hacdcpf/planning/microgrid_planning_solver.hpp"

namespace hacdcpf::planning {

PlanningResult solve_microgrid_planning(
    const MicrogridPlanningInput& /*input*/,
    const PlanningOptions& /*options*/) {
  // Stub: microgrid planning solver not yet implemented.
  PlanningResult r;
  r.converged = false;
  r.status    = "not_implemented";
  return r;
}

PlanningResult solve_microgrid_planning_baseline(
    const MicrogridPlanningInput& /*input*/,
    const PlanningOptions& /*options*/) {
  PlanningResult r;
  r.converged = false;
  r.status    = "not_implemented";
  return r;
}

PlanningResult solve_microgrid_planning_nested_bc(
    const MicrogridPlanningInput& /*input*/,
    const PlanningOptions& /*options*/) {
  PlanningResult r;
  r.converged = false;
  r.status    = "not_implemented";
  return r;
}

}  // namespace hacdcpf::planning
