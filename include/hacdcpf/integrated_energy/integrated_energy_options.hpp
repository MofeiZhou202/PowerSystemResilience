#pragma once

namespace hacdcpf::integrated_energy {

enum class CampusIESSolver {
  Auto,
  HiGHS,
  SCIP,
  Gurobi,
  Native
};

enum class CampusIESObjective {
  Cost,
  Carbon,
  MinCurtailment,
  Weighted
};

struct CampusIESOptions {
  CampusIESSolver solver{CampusIESSolver::Auto};
  CampusIESObjective objective{CampusIESObjective::Cost};

  bool enable_grid_exchange_exclusivity{false};
  bool enable_storage_exclusivity{false};
  bool enable_carbon_budget{false};
  bool enforce_terminal_storage_cyclic{true};
  bool enable_transport{true};
  bool enable_hydrogen_storage_layers{true};
  bool verbose{false};

  double time_limit_sec{60.0};
  double mip_gap_tol{1e-4};

  double weight_cost{1.0};
  double weight_carbon{0.0};
  double weight_curtailment{0.0};
};

}  // namespace hacdcpf::integrated_energy
