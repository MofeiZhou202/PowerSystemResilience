#pragma once

#include <Eigen/Core>

#include "hacdcpf/power_flow/solver_data.hpp"

namespace hacdcpf::powerflow {

/// Assemble the DC specified-injection vector @p pdc_spec (size ndc).
///
/// The result covers all DC injection sources:
///   - DC loads (or bus pd_mw when the load table is empty),
///   - DC storage, DC static generators, DC PV arrays,
///   - VSC DC-side injections (using the supplied @p vm / @p va for AC context),
///   - DCDC converter transfers,
///   - EnergyRouter DC ports.
///
/// @param data     Solver data (provides component lists, base_mva, loss_model).
/// @param vm       AC voltage magnitudes (p.u.); pass bus defaults when unused.
/// @param va       AC voltage angles (rad); pass zeros when unused.
/// @param vdc      DC voltages (p.u.) used for I²R conduction losses.
/// @param pdc_spec Output vector (must be pre-allocated to ndc); is zeroed on entry.
void assemble_dc_injections(const SolverData& data,
                            const Eigen::VectorXd& vm,
                            const Eigen::VectorXd& va,
                            const Eigen::VectorXd& vdc,
                            Eigen::VectorXd& pdc_spec);

}  // namespace hacdcpf::powerflow
