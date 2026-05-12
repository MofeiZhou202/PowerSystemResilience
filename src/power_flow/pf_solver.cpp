// src/power_flow/pf_solver.cpp
//
// Module: power_flow – Newton-Raphson power-flow solver for hybrid AC/DC
// distribution networks.
//
// ─── AC-ONLY NEWTON-RAPHSON ────────────────────────────────────────────────
//
// State vector   x = [δ_nonslack (n_ns), V_pq (n_pq)]
//   where n_ns = n_ac_buses − 1, n_pq = number of PQ buses.
//
// Residuals  F(x) = [ΔP_nonslack ; ΔQ_pq]
//   ΔP_k = P_sch_k − P_k(V, δ)    for every non-slack bus k
//   ΔQ_k = Q_sch_k − Q_k(V, δ)    for every PQ bus k
//   P_sch_k = Σ_g P_gen_g(at k) − P_load_k
//   Q_sch_k = Σ_g Q_gen_g(at k) − Q_load_k
//
// Jacobian  J = ∂F/∂x  (analytically derived):
//   H block (∂ΔP/∂δ, n_ns × n_ns):
//     H_kk = Q_k + B_kk V_k²
//     H_kj = −V_k V_j (G_kj sin(δ_kj) − B_kj cos(δ_kj))   k ≠ j
//   N block (∂ΔP/∂V, n_ns × n_pq):
//     N_kk = −(P_k/V_k + G_kk V_k)                          k is PQ
//     N_kj = −V_k (G_kj cos(δ_kj) + B_kj sin(δ_kj))        k ≠ j
//   M block (∂ΔQ/∂δ, n_pq × n_ns):
//     M_kk = −P_k + G_kk V_k²                               k is PQ
//     M_kj = V_k V_j (G_kj cos(δ_kj) + B_kj sin(δ_kj))     k ≠ j
//   L block (∂ΔQ/∂V, n_pq × n_pq):
//     L_kk = −(Q_k/V_k − B_kk V_k)                          k is PQ
//     L_kj = −V_k (G_kj sin(δ_kj) − B_kj cos(δ_kj))        k ≠ j
//
// ─── HYBRID AC/DC NEWTON-RAPHSON ─────────────────────────────────────────
//
// State vector   x = [δ_ns, V_pq, V_dc_nonref]
//   DC slack bus voltage is fixed; other DC bus voltages are free.
//   VSC converters are treated as constant PQ injections at their AC buses
//   and constant P injections at their DC buses.  In future iterations, VSC
//   internal states (e.g. DC-voltage-controlled mode) can be added.
//
// Additional residuals (DC power balance, non-reference DC buses):
//   ΔP_dc_m = Σ_n G_mn(V_m − V_n) + P_dc_load_m − P_vsc_dc_m = 0

#include <hacdcdss/power_flow/pf_solver.hpp>
#include <hacdcdss/power_models/admittance.hpp>

#include <mipsolvers/engine/solver/native/nle/nle_solver.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace hacdcdss::power_flow {

namespace {

// ── Bus-index classification helpers ─────────────────────────────────────────

struct AcBusPartition {
    std::size_t         slack_bus;      // index of the AC slack bus
    std::vector<int>    nonslack;       // indices of all non-slack buses (ordered)
    std::vector<int>    pq;             // indices of PQ buses (ordered)
    std::vector<int>    pv;             // indices of PV buses (ordered)

    // Maps global bus index → position in state-vector sub-block, or -1.
    std::vector<int>    ns_pos;         // size = n_buses; ns_pos[k] ∈ [0, n_ns)
    std::vector<int>    pq_pos;         // size = n_buses; pq_pos[k] ∈ [0, n_pq)
};

AcBusPartition partition_ac_buses(const model::NetworkModel& net)
{
    const int n = static_cast<int>(net.n_ac_buses());
    AcBusPartition p;
    p.slack_bus = net.ac_slack_idx();
    if (p.slack_bus == std::size_t(-1))
        throw std::runtime_error("PFSolver: no AC slack bus defined.");

    p.ns_pos.assign(n, -1);
    p.pq_pos.assign(n, -1);

    for (int k = 0; k < n; ++k) {
        if (static_cast<std::size_t>(k) == p.slack_bus) continue;
        p.ns_pos[k] = static_cast<int>(p.nonslack.size());
        p.nonslack.push_back(k);

        if (net.ac_buses[static_cast<std::size_t>(k)].type ==
            model::BusType::PQ) {
            p.pq_pos[k] = static_cast<int>(p.pq.size());
            p.pq.push_back(k);
        } else {
            p.pv.push_back(k);
        }
    }
    return p;
}

// ── Scheduled power injections at each bus ────────────────────────────────────

// Returns (P_sch, Q_sch) vectors of size n_ac_buses.
// Generators and VSC converters are included as injections; loads are subtracted.
std::pair<Eigen::VectorXd, Eigen::VectorXd>
scheduled_injections(const model::NetworkModel& net,
                     const std::vector<double>& load_scale)
{
    const int n = static_cast<int>(net.n_ac_buses());
    Eigen::VectorXd P_sch = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd Q_sch = Eigen::VectorXd::Zero(n);

    // Generators.
    for (const auto& g : net.generators) {
        if (!g.in_service) continue;
        P_sch(static_cast<int>(g.bus)) += g.pg_mw   / net.base_mva;
        Q_sch(static_cast<int>(g.bus)) += g.qg_mvar / net.base_mva;
    }

    // VSC converters — treated as fixed AC injections.
    for (const auto& vsc : net.converters) {
        P_sch(static_cast<int>(vsc.ac_bus)) += vsc.p_set_mw   / net.base_mva;
        Q_sch(static_cast<int>(vsc.ac_bus)) += vsc.q_set_mvar / net.base_mva;
    }

    // Loads (subtract; apply scaling if provided).
    for (int k = 0; k < n; ++k) {
        const double scale =
            (!load_scale.empty() && k < static_cast<int>(load_scale.size()))
            ? load_scale[static_cast<std::size_t>(k)] : 1.0;
        P_sch(k) -= net.ac_buses[static_cast<std::size_t>(k)].pd_mw   / net.base_mva * scale;
        Q_sch(k) -= net.ac_buses[static_cast<std::size_t>(k)].qd_mvar / net.base_mva * scale;
    }

    return {P_sch, Q_sch};
}

// ── Initial voltage state ─────────────────────────────────────────────────────

// Returns x0 = [δ_nonslack = 0, V_pq = 1] (flat start).
// For PV buses, voltage is taken from v_set in the bus data.
Eigen::VectorXd initial_state(const model::NetworkModel& net,
                               const AcBusPartition&      part)
{
    const int n_ns = static_cast<int>(part.nonslack.size());
    const int n_pq = static_cast<int>(part.pq.size());
    Eigen::VectorXd x0(n_ns + n_pq);

    for (int i = 0; i < n_ns; ++i) x0(i) = 0.0;
    for (int j = 0; j < n_pq; ++j) x0(n_ns + j) = 1.0;

    return x0;
}

// ── Extract voltage arrays from state vector ──────────────────────────────────

// Fills V[n] and theta[n] from the current state x.
void state_to_voltage(const Eigen::VectorXd&     x,
                      const model::NetworkModel& net,
                      const AcBusPartition&      part,
                      Eigen::VectorXd&           V,
                      Eigen::VectorXd&           theta)
{
    const int n    = static_cast<int>(net.n_ac_buses());
    const int n_ns = static_cast<int>(part.nonslack.size());

    V.resize(n);
    theta.resize(n);

    // Slack bus: V = v_set, θ = 0.
    V(static_cast<int>(part.slack_bus))     =
        net.ac_buses[part.slack_bus].v_set;
    theta(static_cast<int>(part.slack_bus)) = 0.0;

    // Non-slack buses: θ from state vector.
    for (int i = 0; i < n_ns; ++i) {
        const int k    = part.nonslack[static_cast<std::size_t>(i)];
        theta(k) = x(i);
    }

    // PV buses: V = v_set (fixed during NR; relaxed PV→PQ switching not
    // implemented here); PQ buses: V from state vector.
    for (int k = 0; k < n; ++k) {
        if (static_cast<std::size_t>(k) == part.slack_bus) continue;

        if (net.ac_buses[static_cast<std::size_t>(k)].type ==
            model::BusType::PV) {
            V(k) = net.ac_buses[static_cast<std::size_t>(k)].v_set;
        } else {
            const int j = part.pq_pos[k];
            V(k) = (j >= 0) ? x(n_ns + j) : 1.0;
        }
    }
}

// ── Build NR residual vector F(x) ─────────────────────────────────────────────

Eigen::VectorXd nr_residual(const Eigen::VectorXd&     x,
                             const power_models::YBus&  ybus,
                             const Eigen::VectorXd&     P_sch,
                             const Eigen::VectorXd&     Q_sch,
                             const model::NetworkModel& net,
                             const AcBusPartition&      part)
{
    Eigen::VectorXd V, theta;
    state_to_voltage(x, net, part, V, theta);

    const Eigen::VectorXd P_calc =
        power_models::compute_P_injections(ybus, V, theta);
    const Eigen::VectorXd Q_calc =
        power_models::compute_Q_injections(ybus, V, theta);

    const int n_ns = static_cast<int>(part.nonslack.size());
    const int n_pq = static_cast<int>(part.pq.size());
    Eigen::VectorXd F(n_ns + n_pq);

    for (int i = 0; i < n_ns; ++i) {
        const int k = part.nonslack[static_cast<std::size_t>(i)];
        F(i) = P_sch(k) - P_calc(k);
    }
    for (int j = 0; j < n_pq; ++j) {
        const int k = part.pq[static_cast<std::size_t>(j)];
        F(n_ns + j) = Q_sch(k) - Q_calc(k);
    }

    return F;
}

// ── Build NR Jacobian matrix J(x) ─────────────────────────────────────────────
//
// J = [H  N]
//     [M  L]
//
// Standard power-systems Newton-Raphson Jacobian derived analytically
// from the power-injection equations (see file header for formulae).

Eigen::SparseMatrix<double>
nr_jacobian(const Eigen::VectorXd&     x,
            const power_models::YBus&  ybus,
            const model::NetworkModel& net,
            const AcBusPartition&      part)
{
    Eigen::VectorXd V, theta;
    state_to_voltage(x, net, part, V, theta);

    const Eigen::VectorXd P_calc =
        power_models::compute_P_injections(ybus, V, theta);
    const Eigen::VectorXd Q_calc =
        power_models::compute_Q_injections(ybus, V, theta);

    const int n    = ybus.n_buses;
    const int n_ns = static_cast<int>(part.nonslack.size());
    const int n_pq = static_cast<int>(part.pq.size());
    const int sz   = n_ns + n_pq;

    std::vector<Eigen::Triplet<double>> triplets;
    triplets.reserve(4 * sz);

    // ── H block: row = ns_pos[k], col = ns_pos[j] ─────────────────────────
    for (int k = 0; k < n; ++k) {
        const int row_k = part.ns_pos[k];
        if (row_k < 0) continue;  // skip slack

        // Diagonal: H_kk = Q_k + B_kk * V_k^2
        const double H_kk = Q_calc(k) + ybus.b(k, k) * V(k) * V(k);
        triplets.emplace_back(row_k, row_k, H_kk);

        // Off-diagonal: H_kj = -V_k*V_j*(G_kj*sin(δ_kj) - B_kj*cos(δ_kj))
        for (int j = 0; j < n; ++j) {
            if (j == k) continue;
            const int col_j = part.ns_pos[j];
            if (col_j < 0) continue;  // skip slack columns

            const double Gkj = ybus.g(k, j);
            const double Bkj = ybus.b(k, j);
            if (Gkj == 0.0 && Bkj == 0.0) continue;

            const double dth  = theta(k) - theta(j);
            const double H_kj = -V(k) * V(j) *
                                 (Gkj * std::sin(dth) - Bkj * std::cos(dth));
            triplets.emplace_back(row_k, col_j, H_kj);
        }
    }

    // ── N block: row = ns_pos[k], col = n_ns + pq_pos[j] ─────────────────
    for (int k = 0; k < n; ++k) {
        const int row_k = part.ns_pos[k];
        if (row_k < 0) continue;

        // Diagonal (only if k is PQ): N_kk = -(P_k/V_k + G_kk*V_k)
        const int pq_k = part.pq_pos[k];
        if (pq_k >= 0) {
            const double N_kk = -(P_calc(k) / V(k) + ybus.g(k, k) * V(k));
            triplets.emplace_back(row_k, n_ns + pq_k, N_kk);
        }

        // Off-diagonal: N_kj = -V_k*(G_kj*cos(δ_kj) + B_kj*sin(δ_kj))
        for (int j = 0; j < n; ++j) {
            if (j == k) continue;
            const int pq_j = part.pq_pos[j];
            if (pq_j < 0) continue;

            const double Gkj = ybus.g(k, j);
            const double Bkj = ybus.b(k, j);
            if (Gkj == 0.0 && Bkj == 0.0) continue;

            const double dth  = theta(k) - theta(j);
            const double N_kj = -V(k) *
                                 (Gkj * std::cos(dth) + Bkj * std::sin(dth));
            triplets.emplace_back(row_k, n_ns + pq_j, N_kj);
        }
    }

    // ── M block: row = n_ns + pq_pos[k], col = ns_pos[j] ─────────────────
    for (int k = 0; k < n; ++k) {
        const int pq_k = part.pq_pos[k];
        if (pq_k < 0) continue;  // only PQ rows

        // Diagonal: M_kk = -(P_k - G_kk*V_k^2)  = -P_k + G_kk*V_k^2
        // (∂ΔQ_k/∂δ_k = -∂Q_k/∂δ_k = -P_k)
        // BUT only if k is also non-slack:
        const int ns_k = part.ns_pos[k];
        if (ns_k >= 0) {
            const double M_kk = -P_calc(k);
            triplets.emplace_back(n_ns + pq_k, ns_k, M_kk);
        }

        // Off-diagonal: M_kj = V_k*V_j*(G_kj*cos(δ_kj) + B_kj*sin(δ_kj))
        for (int j = 0; j < n; ++j) {
            if (j == k) continue;
            const int col_j = part.ns_pos[j];
            if (col_j < 0) continue;

            const double Gkj = ybus.g(k, j);
            const double Bkj = ybus.b(k, j);
            if (Gkj == 0.0 && Bkj == 0.0) continue;

            const double dth  = theta(k) - theta(j);
            const double M_kj = V(k) * V(j) *
                                 (Gkj * std::cos(dth) + Bkj * std::sin(dth));
            triplets.emplace_back(n_ns + pq_k, col_j, M_kj);
        }
    }

    // ── L block: row = n_ns + pq_pos[k], col = n_ns + pq_pos[j] ─────────
    for (int k = 0; k < n; ++k) {
        const int pq_k = part.pq_pos[k];
        if (pq_k < 0) continue;

        // Diagonal: L_kk = -(Q_k/V_k - B_kk*V_k)
        const double L_kk = -(Q_calc(k) / V(k) - ybus.b(k, k) * V(k));
        triplets.emplace_back(n_ns + pq_k, n_ns + pq_k, L_kk);

        // Off-diagonal: L_kj = -V_k*(G_kj*sin(δ_kj) - B_kj*cos(δ_kj))
        for (int j = 0; j < n; ++j) {
            if (j == k) continue;
            const int pq_j = part.pq_pos[j];
            if (pq_j < 0) continue;

            const double Gkj = ybus.g(k, j);
            const double Bkj = ybus.b(k, j);
            if (Gkj == 0.0 && Bkj == 0.0) continue;

            const double dth  = theta(k) - theta(j);
            const double L_kj = -V(k) *
                                 (Gkj * std::sin(dth) - Bkj * std::cos(dth));
            triplets.emplace_back(n_ns + pq_k, n_ns + pq_j, L_kj);
        }
    }

    Eigen::SparseMatrix<double> J(sz, sz);
    J.setFromTriplets(triplets.begin(), triplets.end());
    return J;
}

// ── Map NLESolver result back to PFResult ─────────────────────────────────────

PFResult build_pf_result(const mipsolvers::engine::NLEResult& sol,
                          const Eigen::VectorXd&               P_sch,
                          const Eigen::VectorXd&               Q_sch,
                          const power_models::YBus&            ybus,
                          const model::NetworkModel&           net,
                          const AcBusPartition&                part)
{
    PFResult result;
    result.converged   = sol.converged();
    result.iterations  = sol.iterations;
    result.status_msg  = result.converged ? "Converged"
                                           : "Max iterations reached";

    Eigen::VectorXd V, theta;
    state_to_voltage(sol.x, net, part, V, theta);

    const int n = static_cast<int>(net.n_ac_buses());
    result.ac_buses.resize(static_cast<std::size_t>(n));
    for (int k = 0; k < n; ++k) {
        result.ac_buses[static_cast<std::size_t>(k)].v_pu      = V(k);
        result.ac_buses[static_cast<std::size_t>(k)].theta_rad = theta(k);
        result.ac_buses[static_cast<std::size_t>(k)].p_inj_mw  =
            power_models::compute_P_injections(ybus, V, theta)(k) * net.base_mva;
        result.ac_buses[static_cast<std::size_t>(k)].q_inj_mvar =
            power_models::compute_Q_injections(ybus, V, theta)(k) * net.base_mva;
    }

    // Branch flows.
    Eigen::VectorXd Pfrom, Qfrom, Pto, Qto;
    power_models::compute_branch_flows(net, V, theta, Pfrom, Qfrom, Pto, Qto);

    const int nb = static_cast<int>(net.n_ac_branches());
    result.ac_branches.resize(static_cast<std::size_t>(nb));
    for (int i = 0; i < nb; ++i) {
        auto& r           = result.ac_branches[static_cast<std::size_t>(i)];
        r.p_from_mw       = Pfrom(i) * net.base_mva;
        r.q_from_mvar     = Qfrom(i) * net.base_mva;
        r.p_to_mw         = Pto(i)   * net.base_mva;
        r.q_to_mvar       = Qto(i)   * net.base_mva;

        const double rate = net.ac_branches[static_cast<std::size_t>(i)].rate_mva;
        if (rate > 0.0) {
            const double s_from = std::hypot(r.p_from_mw, r.q_from_mvar);
            r.loading_pct = 100.0 * s_from / rate;
        }
    }

    // Max mismatch.
    result.max_mismatch = sol.residual_norm;

    return result;
}

} // anonymous namespace

// ── PFSolver::run_nominal ─────────────────────────────────────────────────────

PFResult PFSolver::run_nominal(const model::NetworkModel& net,
                                const PFOptions&           opts) const
{
    return run(net, model::Scenario{}, opts);
}

// ── PFSolver::run ─────────────────────────────────────────────────────────────

PFResult PFSolver::run(const model::NetworkModel& net,
                       const model::Scenario&     scenario,
                       const PFOptions&           opts) const
{
    if (net.n_dc_buses() > 0 && !net.converters.empty())
        return solve_ac_dc(net, opts);
    return solve_ac(net, opts);
}

// ── AC-only Newton-Raphson ─────────────────────────────────────────────────────

PFResult PFSolver::solve_ac(const model::NetworkModel& net,
                             const PFOptions&           opts) const
{
    if (net.ac_buses.empty())
        throw std::invalid_argument(
            "PFSolver::solve_ac: network has no AC buses.");

    // Build admittance matrix and bus partition.
    const auto ybus = power_models::YBus::build(net);
    const auto part = partition_ac_buses(net);

    // Scheduled injections (no scenario scaling in this overload).
    auto [P_sch, Q_sch] = scheduled_injections(net, {});

    // Formulate NLE problem for MIPSolvers NLESolver.
    mipsolvers::engine::NLEProblem prob;
    prob.x0 = initial_state(net, part);

    prob.residual = [&](const Eigen::VectorXd& x) {
        return nr_residual(x, ybus, P_sch, Q_sch, net, part);
    };

    prob.jacobian = [&](const Eigen::VectorXd& x) {
        return nr_jacobian(x, ybus, net, part);
    };

    mipsolvers::engine::NLEOptions nle_opts;
    nle_opts.tol       = opts.tol;
    nle_opts.max_iters = opts.max_iter;

    mipsolvers::engine::NLESolver solver;
    const auto sol = solver.solve(prob, nle_opts);

    return build_pf_result(sol, P_sch, Q_sch, ybus, net, part);
}

// ── Coupled AC/DC Newton-Raphson ──────────────────────────────────────────────
//
// State vector: x = [δ_ns (n_ns), V_pq (n_pq), V_dc_nonref (n_dc-1)]
//
// VSC converters are treated as fixed PQ injections at their AC buses and
// fixed P injections at their DC buses.  The DC-slack VSC adjusts its
// DC-side power to maintain the reference DC bus voltage.
//
// DC power balance at non-reference DC bus m:
//   ΔP_dc_m = Σ_n G_mn*(V_m - V_n) + P_dc_load_m - P_vsc_dc_m = 0
//   where P_vsc_dc_m = P_vsc_ac_m - loss_a - loss_b*|P_vsc_ac_m|

PFResult PFSolver::solve_ac_dc(const model::NetworkModel& net,
                                const PFOptions&           opts) const
{
    const auto ybus = power_models::YBus::build(net);
    const auto gdc  = power_models::GDCBus::build(net);
    const auto part = partition_ac_buses(net);

    const int n_ns = static_cast<int>(part.nonslack.size());
    const int n_pq = static_cast<int>(part.pq.size());
    const int n_dc = static_cast<int>(net.n_dc_buses());

    // DC reference bus index.
    const int dc_ref = static_cast<int>(net.dc_ref_idx());
    // Indices of non-reference DC buses.
    std::vector<int> dc_nonref;
    for (int m = 0; m < n_dc; ++m)
        if (m != dc_ref) dc_nonref.push_back(m);
    const int n_dcfree = static_cast<int>(dc_nonref.size());

    // Total state size.
    const int sz = n_ns + n_pq + n_dcfree;

    auto [P_sch_ac, Q_sch_ac] = scheduled_injections(net, {});

    // DC scheduled power at each DC bus (load only; VSC handled separately).
    Eigen::VectorXd P_dc_load = Eigen::VectorXd::Zero(n_dc);
    for (int m = 0; m < n_dc; ++m)
        P_dc_load(m) = net.dc_buses[static_cast<std::size_t>(m)].pd_mw /
                       net.base_mva;

    // Build initial state.
    Eigen::VectorXd x0(sz);
    x0.head(n_ns + n_pq) = initial_state(net, part);
    x0.tail(n_dcfree).setOnes();   // V_dc_nonref = 1.0 (flat start)

    // Lambda: extract V_dc from state.
    auto state_to_vdc = [&](const Eigen::VectorXd& x) {
        Eigen::VectorXd V_dc(n_dc);
        // Reference bus at rated voltage.
        V_dc(dc_ref) = (dc_ref < n_dc)
            ? net.dc_buses[static_cast<std::size_t>(dc_ref)].v_set
            : 1.0;
        for (int i = 0; i < n_dcfree; ++i)
            V_dc(dc_nonref[static_cast<std::size_t>(i)]) = x(n_ns + n_pq + i);
        return V_dc;
    };

    // Formulate combined NLE problem.
    mipsolvers::engine::NLEProblem prob;
    prob.x0 = x0;

    prob.residual = [&](const Eigen::VectorXd& x) {
        Eigen::VectorXd F(sz);

        // ── AC part ───────────────────────────────────────────────────────
        Eigen::VectorXd ac_x = x.head(n_ns + n_pq);
        Eigen::VectorXd F_ac = nr_residual(ac_x, ybus, P_sch_ac, Q_sch_ac,
                                            net, part);
        F.head(n_ns + n_pq) = F_ac;

        // ── DC part ───────────────────────────────────────────────────────
        const Eigen::VectorXd V_dc = state_to_vdc(x);

        for (int i = 0; i < n_dcfree; ++i) {
            const int m = dc_nonref[static_cast<std::size_t>(i)];

            // DC power balance.
            double flow = 0.0;
            for (Eigen::SparseMatrix<double>::InnerIterator it(gdc.G, m);
                 it; ++it) {
                const int n_bus = static_cast<int>(it.index());
                flow += it.value() * (V_dc(m) - V_dc(n_bus));
            }

            // VSC injection at this DC bus (AC side → DC side with losses).
            double P_vsc_dc = 0.0;
            for (const auto& vsc : net.converters) {
                if (static_cast<int>(vsc.dc_bus) != m) continue;
                const double P_ac = vsc.p_set_mw / net.base_mva;
                // Loss: P_ac = P_dc + loss_a/base + loss_b*|P_ac|
                // → P_dc = P_ac - loss_a/base - loss_b*|P_ac|
                P_vsc_dc += P_ac
                             - vsc.loss_a_mw / net.base_mva
                             - vsc.loss_b_pu * std::abs(P_ac);
            }

            F(n_ns + n_pq + i) = flow + P_dc_load(m) - P_vsc_dc;
        }

        return F;
    };

    prob.jacobian = [&](const Eigen::VectorXd& x) {
        Eigen::VectorXd ac_x = x.head(n_ns + n_pq);

        // AC Jacobian block.
        Eigen::SparseMatrix<double> J_ac = nr_jacobian(ac_x, ybus, net, part);

        // Assemble full Jacobian as block-diagonal (AC/DC coupling is weak in
        // this formulation since VSC power is treated as fixed).
        // DC-DC block: ∂ΔP_dc_i / ∂V_dc_j = G_mi (for non-reference buses).
        std::vector<Eigen::Triplet<double>> triplets;
        triplets.reserve(J_ac.nonZeros() + n_dcfree * n_dcfree);

        // Copy AC block.
        for (int col = 0; col < J_ac.outerSize(); ++col)
            for (Eigen::SparseMatrix<double>::InnerIterator it(J_ac, col);
                 it; ++it)
                triplets.emplace_back(static_cast<int>(it.row()),
                                      static_cast<int>(it.col()),
                                      it.value());

        // DC block.
        for (int i = 0; i < n_dcfree; ++i) {
            const int m = dc_nonref[static_cast<std::size_t>(i)];

            for (int jj = 0; jj < n_dcfree; ++jj) {
                const int nn = dc_nonref[static_cast<std::size_t>(jj)];
                const double Gmn = gdc.G.coeff(m, nn);
                if (i == jj) {
                    // Diagonal: Σ_n G_mn (sum of row m, i.e., G_mm from Y_dc).
                    triplets.emplace_back(n_ns + n_pq + i,
                                          n_ns + n_pq + jj,
                                          Gmn);  // G_mm already stored as sum
                } else if (Gmn != 0.0) {
                    // Off-diagonal: ∂ΔP_dc_i / ∂V_dc_j = G_mn
                    triplets.emplace_back(n_ns + n_pq + i,
                                          n_ns + n_pq + jj,
                                          Gmn);
                }
            }
        }

        Eigen::SparseMatrix<double> J(sz, sz);
        J.setFromTriplets(triplets.begin(), triplets.end());
        return J;
    };

    mipsolvers::engine::NLEOptions nle_opts;
    nle_opts.tol       = opts.tol;
    nle_opts.max_iters = opts.max_iter;

    mipsolvers::engine::NLESolver solver;
    const auto sol = solver.solve(prob, nle_opts);

    // ── Build result ─────────────────────────────────────────────────────────
    PFResult result = build_pf_result(
        sol, P_sch_ac, Q_sch_ac, ybus, net, part);

    // Fill DC bus results.
    const Eigen::VectorXd V_dc = state_to_vdc(sol.x);
    result.dc_buses.resize(static_cast<std::size_t>(n_dc));
    for (int m = 0; m < n_dc; ++m) {
        result.dc_buses[static_cast<std::size_t>(m)].v_pu = V_dc(m);
    }

    // DC cable flows.
    Eigen::VectorXd P_cable;
    power_models::compute_dc_cable_flows(net, V_dc, P_cable);
    const int nc = static_cast<int>(net.n_dc_cables());
    result.dc_cables.resize(static_cast<std::size_t>(nc));
    for (int i = 0; i < nc; ++i) {
        result.dc_cables[static_cast<std::size_t>(i)].p_from_mw =
            P_cable(i) * net.base_mva;
        // DC cables are lossless in p.u. model; to-end = −from-end power.
        result.dc_cables[static_cast<std::size_t>(i)].p_to_mw =
            -P_cable(i) * net.base_mva;
    }

    return result;
}

} // namespace hacdcdss::power_flow
