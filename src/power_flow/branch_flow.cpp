#include "hacdcpf/power_flow/branch_flow.hpp"

#include <cmath>
#include <complex>

namespace hacdcpf::powerflow {

std::vector<BranchFlow> compute_branch_flows(const SolverData& data,
                                              const std::vector<double>& vm,
                                              const std::vector<double>& va) {
  const int n = static_cast<int>(data.ac_buses.size());
  const int nbr = static_cast<int>(data.ac_branches.size());
  std::vector<BranchFlow> flows(static_cast<size_t>(nbr));

  // Build complex voltage vector.
  std::vector<std::complex<double>> V(static_cast<size_t>(n));
  for (int i = 0; i < n; ++i) {
    V[static_cast<size_t>(i)] = std::polar(vm[static_cast<size_t>(i)],
                                            va[static_cast<size_t>(i)]);
  }

  constexpr double kPi = 3.14159265358979323846;

  for (int k = 0; k < nbr; ++k) {
    const auto& br = data.ac_branches[static_cast<size_t>(k)];
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
    const double shift_rad = br.shift_deg * (kPi / 180.0);
    const std::complex<double> tap = std::polar(tap_mag, shift_rad);
    const double tap_abs2 = std::norm(tap);
    if (tap_abs2 <= 0.0) {
      continue;
    }

    const std::complex<double> yff = ytt / tap_abs2;
    const std::complex<double> yft = -ys / std::conj(tap);
    const std::complex<double> ytf = -ys / tap;

    // Sf = V_from * conj(yff * V_from + yft * V_to) * baseMVA
    const std::complex<double> If = yff * V[static_cast<size_t>(i)] +
                                     yft * V[static_cast<size_t>(j)];
    const std::complex<double> Sf = V[static_cast<size_t>(i)] * std::conj(If) * data.base_mva;

    // St = V_to * conj(ytf * V_from + ytt * V_to) * baseMVA
    const std::complex<double> It = ytf * V[static_cast<size_t>(i)] +
                                     ytt * V[static_cast<size_t>(j)];
    const std::complex<double> St = V[static_cast<size_t>(j)] * std::conj(It) * data.base_mva;

    flows[static_cast<size_t>(k)].pf_mw = Sf.real();
    flows[static_cast<size_t>(k)].qf_mvar = Sf.imag();
    flows[static_cast<size_t>(k)].pt_mw = St.real();
    flows[static_cast<size_t>(k)].qt_mvar = St.imag();
  }

  return flows;
}

}  // namespace hacdcpf::powerflow
