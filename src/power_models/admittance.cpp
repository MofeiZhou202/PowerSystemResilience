// src/power_models/admittance.cpp
//
// Module: power_models – Y-bus and G_DC bus construction; power-injection and
// branch-flow equations.
//
// The π-circuit branch model used here:
//
//   y_series = 1/(r + jx)  →  g_s = r/(r²+x²),  b_s = -x/(r²+x²)
//   Half-line-charging shunt at each end: y_shunt = j*(b_c/2)
//
// Resulting Y-bus contributions per branch (from-bus i, to-bus j):
//   Y_ii += g_s + j*(b_s + b_c/2)
//   Y_jj += g_s + j*(b_s + b_c/2)
//   Y_ij -= g_s + j*b_s
//   Y_ji -= g_s + j*b_s
//
// DC cable (purely resistive, from-bus m, to-bus n):
//   G_dc_mm += 1/r_dc
//   G_dc_nn += 1/r_dc
//   G_dc_mn -= 1/r_dc
//   G_dc_nm -= 1/r_dc

#include <hacdcdss/power_models/admittance.hpp>

#include <cmath>
#include <stdexcept>
#include <vector>

namespace hacdcdss::power_models {

// ── YBus::build ───────────────────────────────────────────────────────────────

YBus YBus::build(const model::NetworkModel& net)
{
    const int n = static_cast<int>(net.n_ac_buses());
    if (n == 0)
        throw std::invalid_argument("YBus::build: network has no AC buses.");

    // Accumulate triplets (row, col, value) for G and B separately.
    std::vector<Eigen::Triplet<double>> tG, tB;
    tG.reserve(4 * net.n_ac_branches() + n);
    tB.reserve(4 * net.n_ac_branches() + n);

    for (const auto& br : net.ac_branches) {
        if (!br.in_service) continue;

        const int i = static_cast<int>(br.from_bus);
        const int j = static_cast<int>(br.to_bus);

        // Series admittance: y_s = g_s + j*b_s = 1/(r + jx)
        const double denom = br.r_pu * br.r_pu + br.x_pu * br.x_pu;
        double g_s = 0.0, b_s = 0.0;
        if (denom > 0.0) {
            g_s =  br.r_pu / denom;
            b_s = -br.x_pu / denom;   // negative for inductive lines
        }

        // Half line-charging susceptance at each end.
        const double b_c_half = br.b_pu / 2.0;

        // Self-admittance contributions (diagonal).
        tG.emplace_back(i, i,  g_s);
        tG.emplace_back(j, j,  g_s);
        tB.emplace_back(i, i,  b_s + b_c_half);
        tB.emplace_back(j, j,  b_s + b_c_half);

        // Mutual-admittance contributions (off-diagonal).
        tG.emplace_back(i, j, -g_s);
        tG.emplace_back(j, i, -g_s);
        tB.emplace_back(i, j, -b_s);
        tB.emplace_back(j, i, -b_s);
    }

    YBus ybus;
    ybus.n_buses = n;
    ybus.G.resize(n, n);
    ybus.B.resize(n, n);
    ybus.G.setFromTriplets(tG.begin(), tG.end());
    ybus.B.setFromTriplets(tB.begin(), tB.end());

    return ybus;
}

double YBus::g(int i, int j) const { return G.coeff(i, j); }
double YBus::b(int i, int j) const { return B.coeff(i, j); }

// ── GDCBus::build ─────────────────────────────────────────────────────────────

GDCBus GDCBus::build(const model::NetworkModel& net)
{
    const int n = static_cast<int>(net.n_dc_buses());
    if (n == 0) return {};  // pure AC network – no DC grid

    std::vector<Eigen::Triplet<double>> tG;
    tG.reserve(4 * net.n_dc_cables() + n);

    for (const auto& cable : net.dc_cables) {
        if (!cable.in_service) continue;
        if (cable.r_pu <= 0.0)
            throw std::runtime_error(
                "DcCable " + std::to_string(cable.id) +
                ": r_pu must be > 0.");

        const int m   = static_cast<int>(cable.from_bus);
        const int k   = static_cast<int>(cable.to_bus);
        const double g = 1.0 / cable.r_pu;

        tG.emplace_back(m, m,  g);
        tG.emplace_back(k, k,  g);
        tG.emplace_back(m, k, -g);
        tG.emplace_back(k, m, -g);
    }

    GDCBus gdc;
    gdc.n_buses = n;
    gdc.G.resize(n, n);
    gdc.G.setFromTriplets(tG.begin(), tG.end());

    return gdc;
}

// ── AC power-injection equations ──────────────────────────────────────────────

Eigen::VectorXd compute_P_injections(const YBus&            ybus,
                                     const Eigen::VectorXd& V,
                                     const Eigen::VectorXd& theta)
{
    const int n = ybus.n_buses;
    Eigen::VectorXd P(n);

    for (int k = 0; k < n; ++k) {
        double pk = 0.0;
        // Iterate over non-zero entries of row k in G and B.
        // G and B are stored in column-major (Eigen default for SparseMatrix);
        // we iterate outer (column) and inner (row) for efficiency.
        for (Eigen::SparseMatrix<double>::InnerIterator itG(ybus.G, k);
             itG; ++itG) {
            const int    j    = static_cast<int>(itG.index());
            const double Gkj  = itG.value();
            const double Bkj  = ybus.B.coeff(k, j);
            const double dth  = theta(k) - theta(j);
            pk += V(k) * V(j) * (Gkj * std::cos(dth) + Bkj * std::sin(dth));
        }
        // Pick up any B entries whose corresponding G entry is zero.
        for (Eigen::SparseMatrix<double>::InnerIterator itB(ybus.B, k);
             itB; ++itB) {
            const int    j    = static_cast<int>(itB.index());
            if (ybus.G.coeff(k, j) != 0.0) continue;  // already counted
            const double Bkj  = itB.value();
            const double dth  = theta(k) - theta(j);
            pk += V(k) * V(j) * Bkj * std::sin(dth);
        }
        P(k) = pk;
    }
    return P;
}

Eigen::VectorXd compute_Q_injections(const YBus&            ybus,
                                     const Eigen::VectorXd& V,
                                     const Eigen::VectorXd& theta)
{
    const int n = ybus.n_buses;
    Eigen::VectorXd Q(n);

    for (int k = 0; k < n; ++k) {
        double qk = 0.0;
        for (Eigen::SparseMatrix<double>::InnerIterator itG(ybus.G, k);
             itG; ++itG) {
            const int    j    = static_cast<int>(itG.index());
            const double Gkj  = itG.value();
            const double Bkj  = ybus.B.coeff(k, j);
            const double dth  = theta(k) - theta(j);
            qk += V(k) * V(j) * (Gkj * std::sin(dth) - Bkj * std::cos(dth));
        }
        for (Eigen::SparseMatrix<double>::InnerIterator itB(ybus.B, k);
             itB; ++itB) {
            const int    j    = static_cast<int>(itB.index());
            if (ybus.G.coeff(k, j) != 0.0) continue;
            const double Bkj  = itB.value();
            const double dth  = theta(k) - theta(j);
            qk += V(k) * V(j) * (-Bkj * std::cos(dth));
        }
        Q(k) = qk;
    }
    return Q;
}

// ── Branch power flows ────────────────────────────────────────────────────────

void compute_branch_flows(const model::NetworkModel& net,
                          const Eigen::VectorXd&     V,
                          const Eigen::VectorXd&     theta,
                          Eigen::VectorXd&           P_from,
                          Eigen::VectorXd&           Q_from,
                          Eigen::VectorXd&           P_to,
                          Eigen::VectorXd&           Q_to)
{
    const int nb = static_cast<int>(net.n_ac_branches());
    P_from.resize(nb);  P_from.setZero();
    Q_from.resize(nb);  Q_from.setZero();
    P_to.resize(nb);    P_to.setZero();
    Q_to.resize(nb);    Q_to.setZero();

    for (int idx = 0; idx < nb; ++idx) {
        const auto& br = net.ac_branches[static_cast<std::size_t>(idx)];
        if (!br.in_service) continue;

        const int    i     = static_cast<int>(br.from_bus);
        const int    j     = static_cast<int>(br.to_bus);
        const double denom = br.r_pu * br.r_pu + br.x_pu * br.x_pu;
        if (denom <= 0.0) continue;

        const double g_s      =  br.r_pu / denom;
        const double b_s      = -br.x_pu / denom;
        const double b_c_half =  br.b_pu / 2.0;

        const double Vi  = V(i);
        const double Vj  = V(j);
        const double dth = theta(i) - theta(j);

        // From-end flows.
        P_from(idx) =  Vi * Vi * g_s
                     - Vi * Vj * (g_s * std::cos(dth) + b_s * std::sin(dth));
        Q_from(idx) = -Vi * Vi * (b_s + b_c_half)
                     - Vi * Vj * (g_s * std::sin(dth) - b_s * std::cos(dth));

        // To-end flows (reversed perspective, same branch).
        P_to(idx) =  Vj * Vj * g_s
                   - Vj * Vi * (g_s * std::cos(dth) - b_s * std::sin(dth));
        Q_to(idx) = -Vj * Vj * (b_s + b_c_half)
                   - Vj * Vi * (-g_s * std::sin(dth) - b_s * std::cos(dth));
    }
}

// ── DC cable power flows ──────────────────────────────────────────────────────

void compute_dc_cable_flows(const model::NetworkModel& net,
                             const Eigen::VectorXd&    V_dc,
                             Eigen::VectorXd&          P_cable)
{
    const int nc = static_cast<int>(net.n_dc_cables());
    P_cable.resize(nc);
    P_cable.setZero();

    for (int idx = 0; idx < nc; ++idx) {
        const auto& cable = net.dc_cables[static_cast<std::size_t>(idx)];
        if (!cable.in_service || cable.r_pu <= 0.0) continue;

        const int    m = static_cast<int>(cable.from_bus);
        const int    k = static_cast<int>(cable.to_bus);
        const double g = 1.0 / cable.r_pu;
        P_cable(idx) = g * (V_dc(m) - V_dc(k));
    }
}

} // namespace hacdcdss::power_models
