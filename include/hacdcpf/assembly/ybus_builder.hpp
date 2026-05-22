#pragma once

/// assembly/ybus_builder.hpp
/// ==========================
/// Admittance matrix (Ybus) and DC conductance matrix builders.
/// Replaces: power_flow/assembly/admittance_builder.hpp.

#include <complex>

#include <Eigen/Sparse>

namespace hacdcpf::powerflow {

struct SolverData;

Eigen::SparseMatrix<std::complex<double>> build_admittance_matrix(const SolverData& data);

Eigen::SparseMatrix<double> build_dc_conductance(const SolverData& data);

/// Flags controlling how build_susceptance_matrix assembles B' or B''.
struct BpBuildFlags {
  bool zero_resistance{false};  ///< B' (FDXB): set series R = 0
  bool zero_charging{false};    ///< B' (FDXB): ignore line charging (b_pu)
  bool unit_tap{false};         ///< B' (FDXB): treat tap magnitude as 1
  bool zero_phase_shift{false}; ///< B' and B'': zero all transformer phase shifts
  bool add_bus_shunts{false};   ///< B'' (FDBX): add bus shunt susceptance (-Bs)
  double min_x_pu{0.0};         ///< Skip branches where |x_pu| < min_x_pu
};

inline BpBuildFlags make_bp_flags(double min_x_pu = 1e-6) {
  return {true, true, true, true, false, min_x_pu};
}

inline BpBuildFlags make_bpp_flags() {
  return {false, false, false, true, true, 0.0};
}

Eigen::SparseMatrix<double> build_susceptance_matrix(const SolverData& data,
                                                     const BpBuildFlags& flags);

}  // namespace hacdcpf::powerflow
