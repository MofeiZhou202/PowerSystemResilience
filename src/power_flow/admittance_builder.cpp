#include "hacdcpf/power_flow/admittance_builder.hpp"

#include <cmath>
#include <functional>
#include <vector>

#include "hacdcpf/assembly/solver_data.hpp"

namespace hacdcpf::powerflow {

Eigen::SparseMatrix<std::complex<double>> build_admittance_matrix(const SolverData& data) {
  const int n = static_cast<int>(data.ac_buses.size());
  Eigen::SparseMatrix<std::complex<double>> ybus(n, n);
  if (n == 0) {
    return ybus;
  }

  std::vector<std::complex<double>> diag(static_cast<size_t>(n), {0.0, 0.0});
  std::vector<Eigen::Triplet<std::complex<double>>> triplets;
  triplets.reserve(static_cast<size_t>(n + 2 * data.ac_branches.size()));

  for (const auto& br : data.ac_branches) {
    if (!br.in_service) {
      continue;
    }

    const int i = br.from_bus - 1;
    const int j = br.to_bus - 1;
    if (i < 0 || j < 0 || i >= n || j >= n) {
      continue;
    }

    const std::complex<double> z(br.r_pu, br.x_pu);
    if (std::abs(z) == 0.0) {
      continue;
    }

    const std::complex<double> ys = 1.0 / z;
    const std::complex<double> ytt = ys + std::complex<double>(0.0, br.b_pu / 2.0);
    const double tap_mag = (std::abs(br.tap) < 1e-12) ? 1.0 : br.tap;
    const double shift_rad = br.shift_deg * (std::acos(-1.0) / 180.0);
    const std::complex<double> tap = std::polar(tap_mag, shift_rad);
    const std::complex<double> tap_conj = std::conj(tap);
    const double tap_abs2 = std::norm(tap);
    if (tap_abs2 <= 0.0) {
      continue;
    }

    const std::complex<double> yff = ytt / tap_abs2;
    const std::complex<double> yft = -ys / tap_conj;
    const std::complex<double> ytf = -ys / tap;

    diag[static_cast<size_t>(i)] += yff;
    diag[static_cast<size_t>(j)] += ytt;
    triplets.emplace_back(i, j, yft);
    triplets.emplace_back(j, i, ytf);
  }

  // Add bus shunt admittance to diagonal: Ysh = (Gs + j*Bs) / baseMVA
  // (matches MATPOWER makeYbus.m)
  for (int i = 0; i < n; ++i) {
    const auto& bus = data.ac_buses[static_cast<size_t>(i)];
    diag[static_cast<size_t>(i)] +=
        std::complex<double>(bus.gs_mw, bus.bs_mvar) / data.base_mva;
  }

  // Add Shunt table entries to diagonal (separate from bus-level gs/bs).
  for (const auto& sh : data.shunts) {
    if (!sh.in_service) continue;
    const int i = sh.bus - 1;
    if (i < 0 || i >= n) continue;
    // For switchable shunts, use current_step * bs_per_step; otherwise use gs/bs.
    double gs = sh.gs_mw;
    double bs = sh.bs_mvar;
    if (sh.switchable && sh.n_steps > 0) {
      bs = sh.bs_per_step * sh.current_step;
    }
    diag[static_cast<size_t>(i)] +=
        std::complex<double>(gs, bs) / data.base_mva;
  }

  for (int i = 0; i < n; ++i) {
    triplets.emplace_back(i, i, diag[static_cast<size_t>(i)]);
  }

  ybus.setFromTriplets(triplets.begin(), triplets.end(), std::plus<std::complex<double>>());
  return ybus;
}

Eigen::SparseMatrix<double> build_dc_conductance(const SolverData& data) {
  const int n = static_cast<int>(data.dc_buses.size());
  Eigen::SparseMatrix<double> gdc(n, n);
  if (n == 0) {
    return gdc;
  }

  std::vector<double> diag(static_cast<size_t>(n), 0.0);
  std::vector<Eigen::Triplet<double>> triplets;
  triplets.reserve(static_cast<size_t>(n + 2 * data.dc_branches.size()));

  for (const auto& br : data.dc_branches) {
    if (!br.in_service || br.r_pu == 0.0) {
      continue;
    }

    const int i = br.from_bus - 1;
    const int j = br.to_bus - 1;
    if (i < 0 || j < 0 || i >= n || j >= n) {
      continue;
    }

    const double g = 1.0 / br.r_pu;
    diag[static_cast<size_t>(i)] += g;
    diag[static_cast<size_t>(j)] += g;
    triplets.emplace_back(i, j, -g);
    triplets.emplace_back(j, i, -g);
  }

  for (int i = 0; i < n; ++i) {
    triplets.emplace_back(i, i, diag[static_cast<size_t>(i)]);
  }

  gdc.setFromTriplets(triplets.begin(), triplets.end(), std::plus<double>());
  return gdc;
}

Eigen::SparseMatrix<double> build_susceptance_matrix(const SolverData& data,
                                                     const BpBuildFlags& flags) {
  constexpr double kPi = 3.14159265358979323846;
  const int n = static_cast<int>(data.ac_buses.size());
  std::vector<double> diag(static_cast<size_t>(n), 0.0);
  std::vector<Eigen::Triplet<double>> triplets;
  triplets.reserve(static_cast<size_t>(n + 2 * data.ac_branches.size()));

  for (const auto& br : data.ac_branches) {
    if (!br.in_service) continue;
    const int i = br.from_bus - 1;
    const int j = br.to_bus - 1;
    if (i < 0 || j < 0 || i >= n || j >= n) continue;

    const double r = flags.zero_resistance ? 0.0 : br.r_pu;
    const double x = br.x_pu;
    if (std::abs(x) < flags.min_x_pu) continue;

    const std::complex<double> z(r, x);
    if (std::abs(z) < 1e-20) continue;

    const std::complex<double> ys = 1.0 / z;
    const double b_charging = flags.zero_charging ? 0.0 : br.b_pu;
    const std::complex<double> ytt = ys + std::complex<double>(0.0, b_charging / 2.0);

    const double tap_mag = (flags.unit_tap || std::abs(br.tap) < 1e-12) ? 1.0 : br.tap;
    const double shift_rad = flags.zero_phase_shift ? 0.0 : br.shift_deg * (kPi / 180.0);
    const std::complex<double> tap = std::polar(tap_mag, shift_rad);
    const double tap_abs2 = std::norm(tap);
    if (tap_abs2 <= 0.0) continue;

    const std::complex<double> yff = ytt / tap_abs2;
    const std::complex<double> yft = -ys / std::conj(tap);
    const std::complex<double> ytf = -ys / tap;

    diag[static_cast<size_t>(i)] += -yff.imag();
    diag[static_cast<size_t>(j)] += -ytt.imag();
    triplets.emplace_back(i, j, -yft.imag());
    triplets.emplace_back(j, i, -ytf.imag());
  }

  if (flags.add_bus_shunts) {
    for (int i = 0; i < n; ++i) {
      diag[static_cast<size_t>(i)] +=
          -data.ac_buses[static_cast<size_t>(i)].bs_mvar / data.base_mva;
    }
  }

  for (int i = 0; i < n; ++i) {
    triplets.emplace_back(i, i, diag[static_cast<size_t>(i)]);
  }

  Eigen::SparseMatrix<double> B(n, n);
  B.setFromTriplets(triplets.begin(), triplets.end(), std::plus<double>());
  return B;
}

}  // namespace hacdcpf::powerflow
