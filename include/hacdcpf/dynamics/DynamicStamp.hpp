#pragma once

#include <complex>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Sparse>

namespace hacdcpf::dynamics {

struct DynamicStamp {
  using Complex = std::complex<double>;

  int ac_phase_nodes{0};
  int dc_buses{0};

  std::vector<Eigen::Triplet<Complex>> Yac_triplets;
  Eigen::VectorXcd Iac;

  std::vector<Eigen::Triplet<double>> Gdc_triplets;
  Eigen::VectorXd Idc;

  DynamicStamp() = default;
  DynamicStamp(int ac_nodes, int dc_nodes) { resize(ac_nodes, dc_nodes); }

  void resize(int ac_nodes, int dc_nodes) {
    ac_phase_nodes = ac_nodes;
    dc_buses = dc_nodes;
    Yac_triplets.clear();
    Gdc_triplets.clear();
    Iac = Eigen::VectorXcd::Zero(ac_nodes);
    Idc = Eigen::VectorXd::Zero(dc_nodes);
  }

  void clear() {
    Yac_triplets.clear();
    Gdc_triplets.clear();
    if (Iac.size() > 0) Iac.setZero();
    if (Idc.size() > 0) Idc.setZero();
  }

  void addAcAdmittance(int row, int col, Complex value) {
    if (row < 0 || col < 0 || row >= ac_phase_nodes || col >= ac_phase_nodes) return;
    if (std::abs(value) == 0.0) return;
    Yac_triplets.emplace_back(row, col, value);
  }

  void addAcCurrent(int row, Complex value) {
    if (row < 0 || row >= ac_phase_nodes) return;
    Iac[row] += value;
  }

  void addDcConductance(int row, int col, double value) {
    if (row < 0 || col < 0 || row >= dc_buses || col >= dc_buses) return;
    if (value == 0.0) return;
    Gdc_triplets.emplace_back(row, col, value);
  }

  void addDcCurrent(int row, double value) {
    if (row < 0 || row >= dc_buses) return;
    Idc[row] += value;
  }
};

}  // namespace hacdcpf::dynamics
