#include "hacdcpf/power_flow/ac_linearized_pf.hpp"

#include <cmath>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/SparseLU>

namespace hacdcpf::powerflow {

ACLinearizedDCResult solve_ac_linearized_dc(const SolverData& data) {
  ACLinearizedDCResult out;
  const bool has_hybrid_assets =
      !data.dc_buses.empty() || !data.dc_branches.empty() ||
      !data.converters.empty() || !data.lcc_converters.empty() ||
      !data.dcdc_converters.empty() || !data.energy_routers.empty();
  if (has_hybrid_assets) {
    out.model_scope = "ac-only-linearized-dc:not-applicable-to-hybrid";
    out.model_limitations =
        "DC networks and VSC/LCC/DC-DC/energy-router injections are not "
        "represented by the B-theta linearized AC model.";
    return out;
  }
  const int n = static_cast<int>(data.ac_buses.size());
  const int nbr = static_cast<int>(data.ac_branches.size());
  if (n == 0) {
    out.model_limitations = "No in-service AC buses were available to solve.";
    return out;
  }

  constexpr double kPi = 3.14159265358979323846;

  // Identify slack bus (first SLACK, or bus 0).
  int slack = 0;
  for (int i = 0; i < n; ++i) {
    if (data.ac_buses[static_cast<size_t>(i)].bus_type == BusType::SLACK) {
      slack = i;
      break;
    }
  }

  // Build B matrix: B(i,j) = -b, B(i,i) += b, B(j,j) += b
  // where b = 1 / (X * tap) for each in-service branch.
  // Also compute phase shift injection: Pshift = b * (-shift_rad).
  std::vector<double> diag(static_cast<size_t>(n), 0.0);
  std::vector<Eigen::Triplet<double>> triplets;
  triplets.reserve(static_cast<size_t>(n + 2 * nbr));
  Eigen::VectorXd p_shift = Eigen::VectorXd::Zero(n);

  std::vector<double> branch_b(static_cast<size_t>(nbr), 0.0);

  for (int k = 0; k < nbr; ++k) {
    const auto& br = data.ac_branches[static_cast<size_t>(k)];
    if (!br.in_service || std::abs(br.x_pu) < 1e-20) {
      continue;
    }

    const int i = br.from_bus - 1;
    const int j = br.to_bus - 1;
    if (i < 0 || j < 0 || i >= n || j >= n) {
      continue;
    }

    const double tap = (std::abs(br.tap) < 1e-12) ? 1.0 : br.tap;
    const double b = 1.0 / (br.x_pu * tap);
    branch_b[static_cast<size_t>(k)] = b;

    diag[static_cast<size_t>(i)] += b;
    diag[static_cast<size_t>(j)] += b;
    triplets.emplace_back(i, j, -b);
    triplets.emplace_back(j, i, -b);

    // Phase shift injection.
    const double shift_rad = br.shift_deg * (kPi / 180.0);
    if (std::abs(shift_rad) > 1e-20) {
      const double p_inj = b * (-shift_rad);
      p_shift[i] += p_inj;
      p_shift[j] -= p_inj;
    }
  }

  for (int i = 0; i < n; ++i) {
    triplets.emplace_back(i, i, diag[static_cast<size_t>(i)]);
  }

  Eigen::SparseMatrix<double> B(n, n);
  B.setFromTriplets(triplets.begin(), triplets.end(), std::plus<double>());
  B.makeCompressed();

  // B*theta + Pshift = Pspec, hence B*theta = Pspec - Pshift.
  Eigen::VectorXd P = Eigen::VectorXd::Zero(n);
  for (int i = 0; i < n; ++i) {
    const double pd = (data.pd_pu.size() == n)
                          ? data.pd_pu[i]
                          : data.ac_buses[static_cast<size_t>(i)].pd_mw / data.base_mva;
    P[i] = data.pg[i] - pd - p_shift[i];
  }

  // Build reduced system (exclude slack).
  std::vector<int> non_slack;
  non_slack.reserve(static_cast<size_t>(n - 1));
  std::vector<int> remap(static_cast<size_t>(n), -1);
  for (int i = 0; i < n; ++i) {
    if (i != slack) {
      remap[static_cast<size_t>(i)] = static_cast<int>(non_slack.size());
      non_slack.push_back(i);
    }
  }
  const int nred = static_cast<int>(non_slack.size());
  if (nred == 0) {
    out.va.assign(static_cast<size_t>(n), 0.0);
    out.pf_mw.assign(static_cast<size_t>(nbr), 0.0);
    out.residual_pu = 0.0;
    out.success = true;
    return out;
  }

  // Extract reduced B and RHS.
  std::vector<Eigen::Triplet<double>> red_triplets;
  for (int col = 0; col < B.outerSize(); ++col) {
    const int rc = remap[static_cast<size_t>(col)];
    if (rc < 0) continue;
    for (Eigen::SparseMatrix<double>::InnerIterator it(B, col); it; ++it) {
      const int rr = remap[static_cast<size_t>(it.row())];
      if (rr < 0) continue;
      red_triplets.emplace_back(rr, rc, it.value());
    }
  }

  Eigen::SparseMatrix<double> B_red(nred, nred);
  B_red.setFromTriplets(red_triplets.begin(), red_triplets.end());
  B_red.makeCompressed();

  // RHS = P_nonslack - B_nonslack_slack * Va_slack
  const double va_slack = data.ac_buses[static_cast<size_t>(slack)].va_deg * (kPi / 180.0);
  Eigen::VectorXd rhs(nred);
  for (int k = 0; k < nred; ++k) {
    const int i = non_slack[static_cast<size_t>(k)];
    rhs[k] = P[i] - B.coeff(i, slack) * va_slack;
  }

  // Solve B_red * Va_red = rhs.
  Eigen::SparseLU<Eigen::SparseMatrix<double>> solver;
  solver.compute(B_red);
  if (solver.info() != Eigen::Success) {
    return out;
  }
  const Eigen::VectorXd va_red = solver.solve(rhs);
  if (solver.info() != Eigen::Success) {
    return out;
  }

  // Assemble full angle vector.
  out.va.assign(static_cast<size_t>(n), 0.0);
  out.va[static_cast<size_t>(slack)] = va_slack;
  for (int k = 0; k < nred; ++k) {
    out.va[static_cast<size_t>(non_slack[static_cast<size_t>(k)])] = va_red[k];
  }

  // Verify the solved non-reference equations after the full state is assembled.
  out.residual_pu = 0.0;
  for (int k = 0; k < nred; ++k) {
    const int bus = non_slack[static_cast<size_t>(k)];
    double balance = -P[bus];
    for (int col = 0; col < n; ++col) {
      balance += B.coeff(bus, col) * out.va[static_cast<size_t>(col)];
    }
    out.residual_pu = std::max(out.residual_pu, std::abs(balance));
  }

  // Compute branch flows: Pf = b * (Va_from - Va_to - shift_rad) * baseMVA
  out.pf_mw.assign(static_cast<size_t>(nbr), 0.0);
  for (int k = 0; k < nbr; ++k) {
    const auto& br = data.ac_branches[static_cast<size_t>(k)];
    if (!br.in_service || std::abs(br.x_pu) < 1e-20) {
      continue;
    }
    const int i = br.from_bus - 1;
    const int j = br.to_bus - 1;
    if (i < 0 || j < 0 || i >= n || j >= n) {
      continue;
    }
    const double shift_rad = br.shift_deg * (kPi / 180.0);
    out.pf_mw[static_cast<size_t>(k)] =
        branch_b[static_cast<size_t>(k)] * (out.va[static_cast<size_t>(i)] -
                                              out.va[static_cast<size_t>(j)] -
                                              shift_rad) * data.base_mva;
  }

  out.success = std::isfinite(out.residual_pu);
  out.model_scope = "ac-only-linearized-dc";
  out.model_limitations =
      "Lossless balanced AC B-theta approximation; voltage magnitudes and "
      "reactive-power balance are not solved.";
  return out;
}

}  // namespace hacdcpf::powerflow
