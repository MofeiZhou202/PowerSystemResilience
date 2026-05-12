// src/optimal_power_flow/opf_model.cpp
//
// Module: optimal_power_flow
//
// DC OPF (LP) and AC OPF (NLP) formulations for hybrid AC/DC distribution
// networks, solved via MIPSolvers (HiGHS for LP; Ipopt for NLP).
//
// ─── DC OPF FORMULATION ───────────────────────────────────────────────────
//
// The DC (linearised) power-flow approximation assumes:
//   • Voltage magnitudes ≈ 1.0 p.u. everywhere.
//   • Line resistances ≈ 0 (losses ignored).
//   • Angles are small (cos ≈ 1, sin ≈ θ).
//
// Bus susceptance matrix B' (reduced, slack row/column removed):
//   B'_kj = −1/x_ij     for each branch connecting buses k and j
//   B'_kk = Σ_j 1/x_kj  (sum of susceptances of connected branches)
//
// Decision variables:
//   P_g[i]  – active dispatch of generator i   (MW/base_mva, p.u.)
//   θ[k]    – voltage angle of non-slack bus k  (rad)
//
// Objective (linear, cost_b only; quadratic term requires QP):
//   min Σ_i cost_b[i] * P_g[i] * base_mva   ($)
//
// Constraints:
//   (1) Power balance per non-slack bus k:
//       Σ_{g@k} P_g[g] − P_load_k = Σ_{j} (θ_k − θ_j) / x_kj
//       Rearranged: −B'_kj θ_j = P_inj_k − Σ_g P_g
//   (2) Branch thermal limits:
//       −rate_ij ≤ (θ_i − θ_j) / x_ij ≤ rate_ij   [p.u. on base_mva]
//   (3) Generator output bounds:
//       P_g_min[i] ≤ P_g[i] ≤ P_g_max[i]
//   (4) Reference angle: θ_slack = 0 (handled by variable bounds)
//
// Variable layout (LPModel):
//   x = [θ_0, ..., θ_{n_ns-1},   (n_ns angle variables)
//        P_g_0, ..., P_g_{n_g-1}] (n_g generator variables)
//
// ─── AC OPF FORMULATION ───────────────────────────────────────────────────
//
// Full nonlinear AC OPF with exact power balance equations.
// Variables: [V[k], θ[k], P_g[i], Q_g[i]].
// Objective: Σ_i (cost_a*P_g² + cost_b*P_g + cost_c).
// Constraints: nodal P/Q balance, voltage bounds, generator bounds.
// Formulated as NLPModel; solved via Ipopt through SolverEngine::solve_nlp.

#include <hacdcdss/optimal_power_flow/opf_model.hpp>
#include <hacdcdss/power_models/admittance.hpp>

#include <mipsolvers/engine/problem_types.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace hacdcdss::optimal_power_flow {

using namespace mipsolvers::engine;

// ── Constructor ───────────────────────────────────────────────────────────────

OPFModel::OPFModel(mipsolvers::engine::SolverEngine& engine)
    : engine_(engine)
{}

// ── Public run_dc overloads ───────────────────────────────────────────────────

OPFResult OPFModel::run_dc(const model::NetworkModel& net,
                            const OPFOptions&          opts) const
{
    const std::size_t n = net.n_ac_buses();
    return solve_dc_impl(net, std::vector<double>(n, 1.0), opts);
}

OPFResult OPFModel::run_dc(const model::NetworkModel& net,
                            const model::Scenario&     scenario,
                            const OPFOptions&          opts) const
{
    std::vector<double> load_scale = scenario.load_scale;
    if (load_scale.empty()) load_scale.assign(net.n_ac_buses(), 1.0);
    return solve_dc_impl(net, load_scale, opts);
}

// ── Public run_ac overloads ───────────────────────────────────────────────────

OPFResult OPFModel::run_ac(const model::NetworkModel& net,
                            const OPFOptions&          opts) const
{
    const std::size_t n = net.n_ac_buses();
    return solve_ac_impl(net, std::vector<double>(n, 1.0), opts);
}

OPFResult OPFModel::run_ac(const model::NetworkModel& net,
                            const model::Scenario&     scenario,
                            const OPFOptions&          opts) const
{
    std::vector<double> load_scale = scenario.load_scale;
    if (load_scale.empty()) load_scale.assign(net.n_ac_buses(), 1.0);
    return solve_ac_impl(net, load_scale, opts);
}

// ── DC OPF implementation ─────────────────────────────────────────────────────

OPFResult OPFModel::solve_dc_impl(const model::NetworkModel& net,
                                   const std::vector<double>& load_scale,
                                   const OPFOptions&          opts) const
{
    const int n   = static_cast<int>(net.n_ac_buses());
    const int n_g = static_cast<int>(net.n_generators());
    if (n == 0)
        throw std::invalid_argument("OPFModel::run_dc: no AC buses.");

    // Identify slack bus.
    const int slack = static_cast<int>(net.ac_slack_idx());
    if (slack < 0)
        throw std::runtime_error("OPFModel::run_dc: no AC slack bus.");

    // Collect non-slack bus indices.
    std::vector<int> nonslack_idx;
    for (int k = 0; k < n; ++k)
        if (k != slack) nonslack_idx.push_back(k);
    const int n_ns = static_cast<int>(nonslack_idx.size());

    // Map bus index → nonslack position (or -1 for slack).
    std::vector<int> ns_pos(static_cast<std::size_t>(n), -1);
    for (int i = 0; i < n_ns; ++i)
        ns_pos[static_cast<std::size_t>(nonslack_idx[static_cast<std::size_t>(i)])] = i;

    // Variable layout: x = [θ_0..θ_{n_ns-1}, P_g_0..P_g_{n_g-1}]
    const int theta_offset = 0;
    const int pg_offset    = n_ns;
    const int n_vars       = n_ns + n_g;

    // ── Cost vector ───────────────────────────────────────────────────────────
    Eigen::VectorXd c = Eigen::VectorXd::Zero(n_vars);
    for (int i = 0; i < n_g; ++i) {
        const auto& g = net.generators[static_cast<std::size_t>(i)];
        if (!g.in_service) continue;
        // Linear cost term: cost_b [$/MWh] * P_g [MW/base_mva] * base_mva
        //                 = cost_b * P_g [p.u.] * base_mva [$]
        c(pg_offset + i) = g.cost_b * net.base_mva;
    }

    // ── Variable bounds ───────────────────────────────────────────────────────
    std::vector<VariableMeta> vars(static_cast<std::size_t>(n_vars));

    // Angle variables (unconstrained, except reference = 0 handled elsewhere).
    for (int i = 0; i < n_ns; ++i) {
        vars[static_cast<std::size_t>(theta_offset + i)].lb = -M_PI;
        vars[static_cast<std::size_t>(theta_offset + i)].ub =  M_PI;
        vars[static_cast<std::size_t>(theta_offset + i)].name =
            "theta_" + std::to_string(nonslack_idx[static_cast<std::size_t>(i)]);
    }

    // Generator bounds.
    for (int i = 0; i < n_g; ++i) {
        const auto& g = net.generators[static_cast<std::size_t>(i)];
        const double scale = g.in_service ? 1.0 : 0.0;
        vars[static_cast<std::size_t>(pg_offset + i)].lb =
            g.pg_min_mw / net.base_mva * scale;
        vars[static_cast<std::size_t>(pg_offset + i)].ub =
            g.pg_max_mw / net.base_mva * scale;
        vars[static_cast<std::size_t>(pg_offset + i)].name =
            "Pg_" + std::to_string(g.id);
    }

    // ── Net injection at each non-slack bus ───────────────────────────────────
    // P_load(k) − VSC injection included on the RHS of power balance.
    Eigen::VectorXd P_net_load = Eigen::VectorXd::Zero(n);
    for (int k = 0; k < n; ++k) {
        const double sc = (!load_scale.empty() && k < n)
            ? load_scale[static_cast<std::size_t>(k)] : 1.0;
        P_net_load(k) = net.ac_buses[static_cast<std::size_t>(k)].pd_mw
                        / net.base_mva * sc;
    }
    for (const auto& vsc : net.converters)
        P_net_load(static_cast<int>(vsc.ac_bus)) -=
            vsc.p_set_mw / net.base_mva;

    // ── Equality constraints: power balance ───────────────────────────────────
    // For each non-slack bus k:
    //   Σ_{g@k} P_g[g] − Σ_{j} (θ_k − θ_j)/x_kj = P_load_k
    //
    // Rearranged in matrix form Aeq * x = beq:
    //   − B'_bar * θ_ns + C_gen * P_g = P_load_ns
    //
    // where B'_bar is the susceptance sub-matrix for non-slack buses.

    // Build B'_bar: n_ns × n_ns (susceptance matrix for non-slack subgraph).
    // We also need the B'_slack column (contribution from slack bus angle = 0
    // which is added to beq).
    std::vector<Eigen::Triplet<double>> tBp;
    tBp.reserve(4 * static_cast<int>(net.n_ac_branches()) + n_ns);

    // RHS correction from slack-bus branches.
    Eigen::VectorXd beq_rhs = Eigen::VectorXd::Zero(n_ns);

    for (const auto& br : net.ac_branches) {
        if (!br.in_service || br.x_pu == 0.0) continue;

        const int i   = static_cast<int>(br.from_bus);
        const int j   = static_cast<int>(br.to_bus);
        const double b = 1.0 / br.x_pu;  // susceptance [p.u.] (DC approx)

        const int ni = ns_pos[static_cast<std::size_t>(i)];  // -1 if slack
        const int nj = ns_pos[static_cast<std::size_t>(j)];  // -1 if slack

        // Diagonal contributions (always, for non-slack buses).
        if (ni >= 0) tBp.emplace_back(ni, ni,  b);
        if (nj >= 0) tBp.emplace_back(nj, nj,  b);

        // Off-diagonal.
        if (ni >= 0 && nj >= 0) {
            tBp.emplace_back(ni, nj, -b);
            tBp.emplace_back(nj, ni, -b);
        }

        // Slack-bus correction (θ_slack = 0, so no angle correction needed).
    }

    Eigen::SparseMatrix<double> Bprime(n_ns, n_ns);
    Bprime.setFromTriplets(tBp.begin(), tBp.end());

    // Generator incidence: C_gen[k_ns, g] = 1 if generator g is at non-slack
    // bus nonslack_idx[k_ns].
    std::vector<Eigen::Triplet<double>> tCgen;
    for (int i = 0; i < n_g; ++i) {
        const auto& g = net.generators[static_cast<std::size_t>(i)];
        const int   k = ns_pos[static_cast<std::size_t>(g.bus)];
        if (k >= 0) tCgen.emplace_back(k, i, 1.0);
    }
    Eigen::SparseMatrix<double> C_gen(n_ns, n_g);
    C_gen.setFromTriplets(tCgen.begin(), tCgen.end());

    // Aeq: [−Bprime | C_gen]  (n_ns rows, n_vars cols)
    // beq: P_load at non-slack buses.
    Eigen::SparseMatrix<double> Aeq(n_ns, n_vars);
    {
        std::vector<Eigen::Triplet<double>> tAeq;
        tAeq.reserve(Bprime.nonZeros() + C_gen.nonZeros());

        for (int col = 0; col < Bprime.outerSize(); ++col)
            for (Eigen::SparseMatrix<double>::InnerIterator it(Bprime, col);
                 it; ++it)
                tAeq.emplace_back(static_cast<int>(it.row()),
                                  theta_offset + static_cast<int>(it.col()),
                                  -it.value());   // −B' * θ

        for (int col = 0; col < C_gen.outerSize(); ++col)
            for (Eigen::SparseMatrix<double>::InnerIterator it(C_gen, col);
                 it; ++it)
                tAeq.emplace_back(static_cast<int>(it.row()),
                                  pg_offset + static_cast<int>(it.col()),
                                  it.value());    // +C_gen * P_g

        Aeq.setFromTriplets(tAeq.begin(), tAeq.end());
    }

    Eigen::VectorXd beq(n_ns);
    for (int i = 0; i < n_ns; ++i)
        beq(i) = P_net_load(nonslack_idx[static_cast<std::size_t>(i)]);

    // ── Inequality constraints: branch thermal limits ─────────────────────────
    // For each in-service, non-zero-reactance branch (from i, to j):
    //   −rate ≤ (θ_i − θ_j) / x_ij ≤ rate
    //
    // Written as: A_ineq * x ≤ b_ineq (and −A * x ≤ b for lower bound)
    //
    // We use the LPModel's double-sided row format: row_lhs ≤ A*x ≤ b.

    std::vector<Eigen::Triplet<double>> tA;
    std::vector<double> row_lhs_vec, b_ineq_vec;

    for (const auto& br : net.ac_branches) {
        if (!br.in_service || br.x_pu == 0.0) continue;
        const double rate = (br.rate_mva > 0.0)
            ? br.rate_mva / net.base_mva * (1.0 + opts.branch_rate_slack)
            : 1e9;

        const int   i  = static_cast<int>(br.from_bus);
        const int   j  = static_cast<int>(br.to_bus);
        const double b  = 1.0 / br.x_pu;

        const int ni = ns_pos[static_cast<std::size_t>(i)];
        const int nj = ns_pos[static_cast<std::size_t>(j)];

        const int row = static_cast<int>(row_lhs_vec.size());
        row_lhs_vec.push_back(-rate);
        b_ineq_vec.push_back( rate);

        if (ni >= 0) tA.emplace_back(row, theta_offset + ni,  b);
        if (nj >= 0) tA.emplace_back(row, theta_offset + nj, -b);
    }

    const int n_ineq = static_cast<int>(row_lhs_vec.size());
    Eigen::SparseMatrix<double> A_ineq(n_ineq, n_vars);
    A_ineq.setFromTriplets(tA.begin(), tA.end());

    Eigen::VectorXd row_lhs_eig(n_ineq), b_ineq_eig(n_ineq);
    for (int i = 0; i < n_ineq; ++i) {
        row_lhs_eig(i) = row_lhs_vec[static_cast<std::size_t>(i)];
        b_ineq_eig(i)  = b_ineq_vec[static_cast<std::size_t>(i)];
    }

    // ── Assemble and solve LPModel ────────────────────────────────────────────
    LPModel lp;
    lp.sense    = Sense::Minimize;
    lp.c        = c;
    lp.A        = A_ineq;
    lp.row_lhs  = row_lhs_eig;
    lp.b        = b_ineq_eig;
    lp.Aeq      = Aeq;
    lp.beq      = beq;
    lp.vars     = vars;

    const auto res = engine_.solve_lp(lp);

    // ── Build OPFResult ───────────────────────────────────────────────────────
    OPFResult result;
    result.runtime_sec = res.stats.runtime_sec;

    if (!res.stats.success) {
        result.status     = OPFStatus::Infeasible;
        result.status_msg = res.stats.status;
        return result;
    }

    result.status      = OPFStatus::Optimal;
    result.status_msg  = "Optimal";
    result.total_cost  = res.stats.objective;

    // Fill bus angle results.
    result.ac_buses.resize(static_cast<std::size_t>(n));
    for (int k = 0; k < n; ++k)
        result.ac_buses[static_cast<std::size_t>(k)].v_pu = 1.0;  // DC approx
    result.ac_buses[static_cast<std::size_t>(slack)].theta_rad = 0.0;
    for (int i = 0; i < n_ns; ++i) {
        const int k = nonslack_idx[static_cast<std::size_t>(i)];
        result.ac_buses[static_cast<std::size_t>(k)].theta_rad =
            res.x(theta_offset + i);
    }

    // Compute LMPs as the dual of the equality constraints (if available).
    if (!res.constraint_duals.isZero()) {
        for (int i = 0; i < n_ns; ++i) {
            const int k = nonslack_idx[static_cast<std::size_t>(i)];
            result.ac_buses[static_cast<std::size_t>(k)].lmp_mwh =
                -res.constraint_duals(i) / net.base_mva;
        }
    }

    // Fill generator dispatch.
    result.generators.resize(static_cast<std::size_t>(n_g));
    for (int i = 0; i < n_g; ++i) {
        const double pg_pu   = res.x(pg_offset + i);
        const auto&  g       = net.generators[static_cast<std::size_t>(i)];
        result.generators[static_cast<std::size_t>(i)].pg_mw   =
            pg_pu * net.base_mva;
        result.generators[static_cast<std::size_t>(i)].cost_usd =
            g.cost_b * pg_pu * net.base_mva + g.cost_c;
    }

    return result;
}

// ── AC OPF implementation ─────────────────────────────────────────────────────
//
// Full AC OPF using the nonlinear AC power-flow equations.
//
// Variables: x = [V_0..V_{n-1}, θ_0..θ_{n-1}, P_g_0..P_g_{n_g-1},
//                 Q_g_0..Q_g_{n_g-1}]
// Objective: Σ_i (cost_a * P_g_i^2 + cost_b * P_g_i + cost_c)
// Equality constraints:
//   g_k(x): P_k(V,θ) − Σ_{g@k} P_g = P_load_k   for each bus k
//   h_k(x): Q_k(V,θ) − Σ_{g@k} Q_g = Q_load_k   for each bus k
//   θ_slack = 0
// Box bounds on V, P_g, Q_g (voltage limits, generator limits).

OPFResult OPFModel::solve_ac_impl(const model::NetworkModel& net,
                                   const std::vector<double>& load_scale,
                                   const OPFOptions& /*opts*/) const
{
    const int n   = static_cast<int>(net.n_ac_buses());
    const int n_g = static_cast<int>(net.n_generators());
    if (n == 0)
        throw std::invalid_argument("OPFModel::run_ac: no AC buses.");

    const int slack = static_cast<int>(net.ac_slack_idx());

    // Build Y-bus.
    const auto ybus = power_models::YBus::build(net);

    // Variable layout:
    //   [V[0..n-1],  θ[0..n-1],  P_g[0..n_g-1],  Q_g[0..n_g-1]]
    const int V_off  = 0;
    const int th_off = n;
    const int pg_off = 2 * n;
    const int qg_off = 2 * n + n_g;
    const int n_vars = 2 * n + 2 * n_g;

    // Variable metadata.
    std::vector<VariableMeta> vars(static_cast<std::size_t>(n_vars));

    for (int k = 0; k < n; ++k) {
        const auto& bus = net.ac_buses[static_cast<std::size_t>(k)];
        vars[static_cast<std::size_t>(V_off  + k)] = {VarType::Continuous, bus.v_min, bus.v_max, "V_"   + std::to_string(k)};
        vars[static_cast<std::size_t>(th_off + k)] = {VarType::Continuous, -M_PI,    M_PI,      "th_"  + std::to_string(k)};
    }
    for (int i = 0; i < n_g; ++i) {
        const auto& g = net.generators[static_cast<std::size_t>(i)];
        const double s = g.in_service ? 1.0 : 0.0;
        vars[static_cast<std::size_t>(pg_off + i)] = {VarType::Continuous,
            g.pg_min_mw / net.base_mva * s, g.pg_max_mw / net.base_mva * s,
            "Pg_" + std::to_string(g.id)};
        vars[static_cast<std::size_t>(qg_off + i)] = {VarType::Continuous,
            g.qg_min_mvar / net.base_mva * s, g.qg_max_mvar / net.base_mva * s,
            "Qg_" + std::to_string(g.id)};
    }

    // Net load at each bus.
    Eigen::VectorXd P_load(n), Q_load(n);
    for (int k = 0; k < n; ++k) {
        const double sc = (!load_scale.empty() && k < n)
            ? load_scale[static_cast<std::size_t>(k)] : 1.0;
        P_load(k) = net.ac_buses[static_cast<std::size_t>(k)].pd_mw   / net.base_mva * sc;
        Q_load(k) = net.ac_buses[static_cast<std::size_t>(k)].qd_mvar / net.base_mva * sc;
    }

    // Helper: generator incidence per bus.
    // gen_at_bus[k] = list of generator indices at bus k.
    std::vector<std::vector<int>> gen_at_bus(static_cast<std::size_t>(n));
    for (int i = 0; i < n_g; ++i)
        gen_at_bus[net.generators[static_cast<std::size_t>(i)].bus].push_back(i);

    // Objective: Σ_i (cost_a*P_g_i^2 + cost_b*P_g_i + cost_c)
    // scaled from [$/h] to [$/h / base_mva^2] etc.
    NLPModel nlp;
    nlp.sense = Sense::Minimize;
    nlp.vars  = vars;

    nlp.f = [&](const Eigen::VectorXd& x) {
        double obj = 0.0;
        for (int i = 0; i < n_g; ++i) {
            const auto& g = net.generators[static_cast<std::size_t>(i)];
            const double pg = x(pg_off + i) * net.base_mva;  // back to MW
            obj += g.cost_a * pg * pg + g.cost_b * pg + g.cost_c;
        }
        return obj;
    };

    nlp.grad = [&](const Eigen::VectorXd& x, Eigen::VectorXd& grad) {
        grad.resize(n_vars);
        grad.setZero();
        for (int i = 0; i < n_g; ++i) {
            const auto& g = net.generators[static_cast<std::size_t>(i)];
            const double pg = x(pg_off + i) * net.base_mva;
            // ∂/∂P_g_i = (2*cost_a*pg + cost_b) * base_mva
            grad(pg_off + i) = (2.0 * g.cost_a * pg + g.cost_b) * net.base_mva;
        }
    };

    // Equality constraints: [ΔP_0..ΔP_{n-1}, ΔQ_0..ΔQ_{n-1}, θ_slack = 0]
    // Total n_eq = 2*n + 1
    const int n_eq = 2 * n + 1;

    nlp.g = [&](const Eigen::VectorXd& x, Eigen::VectorXd& g_vec) {
        g_vec.resize(n_eq);

        const Eigen::VectorXd V     = x.segment(V_off,  n);
        const Eigen::VectorXd theta = x.segment(th_off, n);

        const Eigen::VectorXd P_calc =
            power_models::compute_P_injections(ybus, V, theta);
        const Eigen::VectorXd Q_calc =
            power_models::compute_Q_injections(ybus, V, theta);

        for (int k = 0; k < n; ++k) {
            double pg_sum = 0.0, qg_sum = 0.0;
            for (int gi : gen_at_bus[static_cast<std::size_t>(k)]) {
                pg_sum += x(pg_off + gi);
                qg_sum += x(qg_off + gi);
            }
            g_vec(k)     = P_calc(k) - pg_sum + P_load(k);  // = 0
            g_vec(n + k) = Q_calc(k) - qg_sum + Q_load(k);  // = 0
        }
        g_vec(2 * n) = x(th_off + slack);  // = 0 (angle reference)
    };

    // Jacobian of equality constraints (numerical sparsity pattern hard to
    // predict; provide dense approximation via finite differences wrapper).
    // For production use, the analytic Jacobian should be provided.
    nlp.jac_g = [&](const Eigen::VectorXd& x,
                     Eigen::SparseMatrix<double>& J) {
        // Finite-difference Jacobian.
        const double eps = 1e-7;
        Eigen::VectorXd g0(n_eq);
        {
            auto& fn = nlp.g;
            fn(x, g0);
        }
        std::vector<Eigen::Triplet<double>> trips;
        trips.reserve(n_eq * n_vars / 4);
        for (int col = 0; col < n_vars; ++col) {
            Eigen::VectorXd xp = x;
            xp(col) += eps;
            Eigen::VectorXd g1(n_eq);
            nlp.g(xp, g1);
            Eigen::VectorXd dg = (g1 - g0) / eps;
            for (int row = 0; row < n_eq; ++row)
                if (std::abs(dg(row)) > 1e-12)
                    trips.emplace_back(row, col, dg(row));
        }
        J.resize(n_eq, n_vars);
        J.setFromTriplets(trips.begin(), trips.end());
    };

    // Initial point: flat start.
    nlp.x0.resize(n_vars);
    nlp.x0.setZero();
    for (int k = 0; k < n; ++k) nlp.x0(V_off + k) = 1.0;
    for (int i = 0; i < n_g; ++i) {
        const auto& g = net.generators[static_cast<std::size_t>(i)];
        nlp.x0(pg_off + i) =
            0.5 * (g.pg_min_mw + g.pg_max_mw) / net.base_mva;
        nlp.x0(qg_off + i) = 0.0;
    }

    // Solve via SolverEngine (dispatches to Ipopt if available).
    const auto res = engine_.solve_nlp(nlp);

    // ── Build result ──────────────────────────────────────────────────────────
    OPFResult result;
    result.runtime_sec = res.stats.runtime_sec;

    if (!res.stats.success) {
        result.status     = OPFStatus::Infeasible;
        result.status_msg = res.stats.status;
        return result;
    }

    result.status     = OPFStatus::Optimal;
    result.status_msg = "Optimal";
    result.total_cost = res.stats.objective;

    result.ac_buses.resize(static_cast<std::size_t>(n));
    for (int k = 0; k < n; ++k) {
        result.ac_buses[static_cast<std::size_t>(k)].v_pu      = res.x(V_off  + k);
        result.ac_buses[static_cast<std::size_t>(k)].theta_rad = res.x(th_off + k);
    }

    result.generators.resize(static_cast<std::size_t>(n_g));
    for (int i = 0; i < n_g; ++i) {
        const auto& g = net.generators[static_cast<std::size_t>(i)];
        result.generators[static_cast<std::size_t>(i)].pg_mw   =
            res.x(pg_off + i) * net.base_mva;
        result.generators[static_cast<std::size_t>(i)].qg_mvar =
            res.x(qg_off + i) * net.base_mva;
        const double pg = res.x(pg_off + i) * net.base_mva;
        result.generators[static_cast<std::size_t>(i)].cost_usd =
            g.cost_a * pg * pg + g.cost_b * pg + g.cost_c;
    }

    return result;
}

} // namespace hacdcdss::optimal_power_flow
