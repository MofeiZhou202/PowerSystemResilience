#pragma once
// hacdcdss/power_models/admittance.hpp
//
// Module: power_models
//
// Builds the admittance matrices that underpin all power-flow and OPF
// formulations in this library:
//
//   YBus   – complex nodal admittance matrix for the AC grid
//             Y = G + jB  (G = conductance, B = susceptance)
//   GDCBus – real nodal conductance matrix for the DC grid
//
// Both matrices are stored as Eigen sparse real matrices so they can be
// passed directly to MIPSolvers' NLE / LP / NLP problem types.
//
// Construction from NetworkModel follows the π-circuit branch model:
//   y_series = g_s + j*b_s = 1 / (r + j*x)
//   y_shunt  = j*(b_c/2) at each end (half the total line charging)
//
// DC cables are purely resistive: g_dc = 1/r_dc.

#include <hacdcdss/model/network_model.hpp>

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <vector>

namespace hacdcdss::power_models {

// ── AC admittance matrices ────────────────────────────────────────────────────

struct YBus {
    int n_buses = 0;
    Eigen::SparseMatrix<double> G;  // conductance matrix (real part of Y)
    Eigen::SparseMatrix<double> B;  // susceptance matrix (imag part of Y)

    /// Build Y-bus from an in-service NetworkModel.
    static YBus build(const model::NetworkModel& net);

    /// Retrieve G_ij (or 0 if not in the sparsity pattern).
    double g(int i, int j) const;
    /// Retrieve B_ij (or 0 if not in the sparsity pattern).
    double b(int i, int j) const;
};

// ── DC conductance matrix ─────────────────────────────────────────────────────

struct GDCBus {
    int n_buses = 0;
    Eigen::SparseMatrix<double> G;  // nodal conductance matrix

    /// Build DC G-bus from an in-service NetworkModel.
    static GDCBus build(const model::NetworkModel& net);
};

// ── AC power-injection equations ─────────────────────────────────────────────
//
// Compute nodal active and reactive power injections P(V, θ) and Q(V, θ)
// given voltage magnitudes V[n] and angles theta[n].

/// Compute AC active power injections P[k] = V_k * Σ_j V_j*(G_kj*cos(θ_kj) +
/// B_kj*sin(θ_kj)) for every bus.
Eigen::VectorXd compute_P_injections(const YBus&             ybus,
                                     const Eigen::VectorXd&  V,
                                     const Eigen::VectorXd&  theta);

/// Compute AC reactive power injections Q[k] = V_k * Σ_j V_j*(G_kj*sin(θ_kj)
/// − B_kj*cos(θ_kj)) for every bus.
Eigen::VectorXd compute_Q_injections(const YBus&             ybus,
                                     const Eigen::VectorXd&  V,
                                     const Eigen::VectorXd&  theta);

/// Compute branch active and reactive power flows (from-end) for all
/// in-service AC branches.
///   P_from[br] = V_i² * G_ii_series − V_i*V_j*(G_ij*cos(θ_ij) + B_ij*sin(θ_ij))
///   Q_from[br] = −V_i² * (B_ii_series + b_c/2) − V_i*V_j*(G_ij*sin(θ_ij) − B_ij*cos(θ_ij))
void compute_branch_flows(const model::NetworkModel& net,
                          const Eigen::VectorXd&     V,
                          const Eigen::VectorXd&     theta,
                          Eigen::VectorXd&           P_from,
                          Eigen::VectorXd&           Q_from,
                          Eigen::VectorXd&           P_to,
                          Eigen::VectorXd&           Q_to);

/// Compute DC cable power flows (from-end) for all in-service DC cables.
///   P_dc[cable] = (V_from − V_to) / r_pu
void compute_dc_cable_flows(const model::NetworkModel& net,
                             const Eigen::VectorXd&    V_dc,
                             Eigen::VectorXd&          P_cable);

} // namespace hacdcdss::power_models
