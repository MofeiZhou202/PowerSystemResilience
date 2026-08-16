#pragma once

/// assembly/solver_data.hpp
/// =========================
/// SolverData: flat, pre-assembled representation of a HybridPowerSystem
/// ready for consumption by power flow and OPF solvers.
/// Replaces: power_flow/solver_data.hpp.

#include <complex>
#include <cstdint>
#include <optional>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Sparse>

#include "hacdcpf/model/hybrid_power_system.hpp"

namespace hacdcpf::powerflow {

struct SolverData {
  // Canonical numerical copies, not a second authored model. Public identity
  // and result recovery remain owned by the rich model and projection maps;
  // solver code must not reinterpret or persist these vector positions as
  // stable component IDs.
  std::vector<ACBus> ac_buses;
  std::vector<ACBranch> ac_branches;
  std::vector<DCBus> dc_buses;
  std::vector<DCBranch> dc_branches;
  std::vector<VSCConverter> converters;
  std::vector<LCCConverter> lcc_converters;
  std::vector<DCDCConverter> dcdc_converters;
  std::vector<EnergyRouter> energy_routers;
  std::vector<Generator> generators;

  std::vector<Load> loads;
  std::vector<FlexibleLoad> flexible_loads;
  std::vector<StaticGenerator> static_generators;
  std::vector<RenewableGen> renewable_gens;
  std::vector<PVSystem> pv_systems;
  std::vector<Storage> storage_units;

  std::vector<DCLoad> dc_loads;
  std::vector<Storage> dc_storage;
  std::vector<StaticGenerator> dc_static_generators;
  std::vector<PVArrayDC> dc_pv_arrays;

  std::vector<Shunt> shunts;

  std::vector<ChargingStation> charging_stations;
  std::vector<Charger> chargers;

  std::vector<ExternalGrid> external_grids;

  std::vector<Switch> switches;

  std::vector<VirtualPowerPlant> vpps;
  std::vector<Microgrid> microgrids;
  std::vector<MobileStorage> mobile_storage;

  double base_mva{100.0};
  LossModelType loss_model{LossModelType::Linear};

  double zip_pw[3]{1.0, 0.0, 0.0};
  double zip_qw[3]{1.0, 0.0, 0.0};

  bool has_component_loads{false};
  Eigen::VectorXd pd_pu, qd_pu;
  Eigen::VectorXd bus_zip_zp, bus_zip_ip, bus_zip_pp;
  Eigen::VectorXd bus_zip_zq, bus_zip_iq, bus_zip_pq;

  bool enable_coupled_jacobian{false};
  bool enable_augmented_equations{false};
  bool enable_semi_smooth_newton{false};

  double ncp_mu{0.0};

  Eigen::VectorXd pg;
  Eigen::VectorXd qg;
  Eigen::SparseMatrix<std::complex<double>> ybus;
  Eigen::SparseMatrix<double> gdc;
  std::uint64_t build_id{0};

  std::optional<BusMergeMap> bus_merge_map;
  std::optional<ProjectionCertificate> projection_certificate;
  std::optional<ProjectionReport> projection_report;
};

SolverData make_solver_data(const HybridPowerSystem& sys,
                            LossModelType loss_model = LossModelType::Linear);

SolverData make_solver_data(HybridPowerSystem&& sys,
                            LossModelType loss_model = LossModelType::Linear);

SolverData make_solver_data_projected(HybridPowerSystem&& projected,
                                      LossModelType loss_model = LossModelType::Linear);

void aggregate_generation(SolverData& data);
void aggregate_load_demand(SolverData& data);
void rebuild_matrices(SolverData& data);

/// Refresh injection/control values without rebuilding network matrices.
/// Returns false unless the rich model maps one-to-one to the existing direct
/// canonical layout and all topology/network parameters are unchanged.
bool refresh_solver_data_values(SolverData& data,
                                const HybridPowerSystem& sys);

}  // namespace hacdcpf::powerflow
