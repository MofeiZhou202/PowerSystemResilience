#pragma once

#include <complex>
#include <cstdint>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Sparse>

#include "hacdcpf/model/aggregation.hpp"
#include "hacdcpf/model/storage.hpp"
#include "hacdcpf/model/system.hpp"

namespace hacdcpf::powerflow {

struct SolverData {
  std::vector<ACBus> ac_buses;
  std::vector<ACBranch> ac_branches;
  std::vector<DCBus> dc_buses;
  std::vector<DCBranch> dc_branches;
  std::vector<VSCConverter> converters;
  std::vector<DCDCConverter> dcdc_converters;
  std::vector<EnergyRouter> energy_routers;
  std::vector<Generator> generators;

  // Component tables (from ACSystem, for component-based aggregation).
  std::vector<Load> loads;
  std::vector<FlexibleLoad> flexible_loads;
  std::vector<StaticGenerator> static_generators;
  std::vector<RenewableGen> renewable_gens;
  std::vector<PVSystem> pv_systems;
  std::vector<Storage> storage_units;

  // DC-side component tables (from DCSystem).
  std::vector<DCLoad> dc_loads;
  std::vector<Storage> dc_storage;
  std::vector<StaticGenerator> dc_static_generators;
  std::vector<PVArrayDC> dc_pv_arrays;

  // Shunt table (from ACSystem, aggregated into Ybus diagonal).
  std::vector<Shunt> shunts;

  // EV charging stations (from ACSystem, aggregated as AC loads).
  std::vector<ChargingStation> charging_stations;
  std::vector<Charger> chargers;

  // External grids (from ACSystem, for slack/voltage setpoint).
  std::vector<ExternalGrid> external_grids;

  // Switches (from ACSystem, for topology control).
  std::vector<Switch> switches;

  // Aggregation components (from HybridPowerSystem).
  std::vector<VirtualPowerPlant> vpps;
  std::vector<Microgrid> microgrids;
  std::vector<MobileStorage> mobile_storage;

  double base_mva{100.0};
  LossModelType loss_model{LossModelType::Linear};

  // Global ZIP load weights (copied from PowerFlowOptions before solve).
  double zip_pw[3]{1.0, 0.0, 0.0};
  double zip_qw[3]{1.0, 0.0, 0.0};

  // Per-bus demand/ZIP from Load table (populated when loads is non-empty).
  bool has_component_loads{false};
  Eigen::VectorXd pd_pu, qd_pu;
  Eigen::VectorXd bus_zip_zp, bus_zip_ip, bus_zip_pp;  // per-bus active ZIP
  Eigen::VectorXd bus_zip_zq, bus_zip_iq, bus_zip_pq;  // per-bus reactive ZIP

  // Exact fully-coupled Newton flags (copied from PowerFlowOptions before solve).
  bool enable_coupled_jacobian{false};
  bool enable_augmented_equations{false};
  bool enable_semi_smooth_newton{false};

  /// Smoothing parameter μ for the smooth Fischer–Burmeister NCP function.
  /// 0 → use the standard near-zero-smoothed FB (original behaviour).
  /// Set from RobustNonlinearOptions::ncp_mu0 and annealed during the solve.
  double ncp_mu{0.0};

  Eigen::VectorXd pg;
  Eigen::VectorXd qg;
  Eigen::SparseMatrix<std::complex<double>> ybus;
  Eigen::SparseMatrix<double> gdc;
  std::uint64_t build_id{0};

  // Bus merge map (propagated from project_to_canonical_models).
  std::optional<BusMergeMap> bus_merge_map;
};

SolverData make_solver_data(const HybridPowerSystem& sys,
                            LossModelType loss_model = LossModelType::Linear);

/// Move-from overload: projects and consumes \p sys, avoiding a deep copy.
SolverData make_solver_data(HybridPowerSystem&& sys,
                            LossModelType loss_model = LossModelType::Linear);

/// Build SolverData from a system that has already been run through
/// project_to_canonical_models().  Skips the (expensive) re-projection.
SolverData make_solver_data_projected(HybridPowerSystem&& projected,
                                      LossModelType loss_model = LossModelType::Linear);

void aggregate_generation(SolverData& data);

void aggregate_load_demand(SolverData& data);

void rebuild_matrices(SolverData& data);

}  // namespace hacdcpf::powerflow
