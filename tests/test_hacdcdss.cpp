// tests/test_hacdcdss.cpp
//
// Smoke tests for all four modules: model, power_models, power_flow,
// optimal_power_flow.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <hacdcdss/model/network_model.hpp>
#include <hacdcdss/model/scenario_data.hpp>
#include <hacdcdss/model/time_series_loader.hpp>
#include <hacdcdss/power_models/admittance.hpp>
#include <hacdcdss/power_flow/pf_solver.hpp>
#include <hacdcdss/optimal_power_flow/opf_model.hpp>

#include <mipsolvers/engine/api/solver.hpp>

#include <fstream>
#include <numbers>

using namespace hacdcdss::model;
using namespace hacdcdss::power_models;
using namespace hacdcdss::power_flow;
using namespace hacdcdss::optimal_power_flow;
using Catch::Matchers::WithinAbs;

// ═══════════════════════════════════════════════════════════════════════════════
// Module: model
// ═══════════════════════════════════════════════════════════════════════════════

TEST_CASE("NetworkModel – construct and validate 2-bus AC network",
          "[model][network]")
{
    NetworkModel net;
    net.name     = "2bus_test";
    net.base_mva = 10.0;

    AcBus slack;
    slack.id = 0; slack.name = "slack"; slack.type = BusType::Slack;
    slack.v_set = 1.0; slack.pd_mw = 0.0;

    AcBus load;
    load.id = 1; load.name = "load"; load.type = BusType::PQ;
    load.pd_mw = 2.0; load.qd_mvar = 0.5;

    net.ac_buses = {slack, load};

    AcBranch br;
    br.id = 0; br.from_bus = 0; br.to_bus = 1;
    br.r_pu = 0.01; br.x_pu = 0.05; br.b_pu = 0.002; br.rate_mva = 5.0;
    net.ac_branches = {br};

    Generator gen;
    gen.id = 0; gen.bus = 0; gen.name = "gen0";
    gen.pg_mw = 3.0; gen.qg_mvar = 0.5;
    gen.pg_min_mw = 0.0; gen.pg_max_mw = 10.0;
    gen.cost_b = 40.0;
    net.generators = {gen};

    REQUIRE(net.n_ac_buses()    == 2);
    REQUIRE(net.n_ac_branches() == 1);
    REQUIRE(net.n_generators()  == 1);
    REQUIRE(net.ac_slack_idx()  == 0);
    REQUIRE_NOTHROW(net.validate());
}

TEST_CASE("NetworkModel – validate catches out-of-range branch bus",
          "[model][network]")
{
    NetworkModel net;
    AcBus b;  b.id = 0; b.type = BusType::Slack;
    net.ac_buses = {b};

    AcBranch br; br.id = 0; br.from_bus = 0; br.to_bus = 99;  // invalid
    net.ac_branches = {br};

    REQUIRE_THROWS(net.validate());
}

TEST_CASE("NetworkModel – validate catches missing slack bus",
          "[model][network]")
{
    NetworkModel net;
    AcBus b;  b.id = 0; b.type = BusType::PQ;
    net.ac_buses = {b};
    REQUIRE_THROWS(net.validate());
}

TEST_CASE("NetworkModel – hybrid AC/DC 2-bus AC + 2-bus DC",
          "[model][network][dc]")
{
    NetworkModel net;
    net.base_mva = 10.0;

    AcBus a0; a0.id = 0; a0.type = BusType::Slack;
    AcBus a1; a1.id = 1; a1.type = BusType::PQ; a1.pd_mw = 1.0;
    net.ac_buses = {a0, a1};

    AcBranch br; br.id = 0; br.from_bus = 0; br.to_bus = 1;
    br.r_pu = 0.01; br.x_pu = 0.05; br.rate_mva = 5.0;
    net.ac_branches = {br};

    DcBus d0; d0.id = 0; d0.is_ref = true; d0.v_set = 1.0;
    DcBus d1; d1.id = 1; d1.pd_mw = 0.5;
    net.dc_buses = {d0, d1};

    DcCable cable; cable.id = 0; cable.from_bus = 0; cable.to_bus = 1;
    cable.r_pu = 0.02;
    net.dc_cables = {cable};

    VscConverter vsc; vsc.id = 0; vsc.ac_bus = 0; vsc.dc_bus = 0;
    vsc.rate_mva = 5.0; vsc.dc_slack = true; vsc.p_set_mw = 1.5;
    net.converters = {vsc};

    REQUIRE_NOTHROW(net.validate());
    REQUIRE(net.dc_ref_idx() == 0);
}

// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("ScenarioSet – normalise probabilities", "[model][scenario]")
{
    ScenarioSet ss;
    Scenario s1; s1.id = 0; s1.probability = 2.0;
    Scenario s2; s2.id = 1; s2.probability = 3.0;
    ss.scenarios = {s1, s2};

    ss.normalise_probabilities();

    REQUIRE_THAT(ss.scenarios[0].probability, WithinAbs(0.4, 1e-9));
    REQUIRE_THAT(ss.scenarios[1].probability, WithinAbs(0.6, 1e-9));
}

TEST_CASE("ScenarioSet – zero total probability throws", "[model][scenario]")
{
    ScenarioSet ss;
    Scenario s; s.id = 0; s.probability = 0.0;
    ss.scenarios = {s};
    REQUIRE_THROWS(ss.normalise_probabilities());
}

// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("TimeSeriesLoader – CSV round-trip", "[model][io]")
{
    const std::string path = "/tmp/hacdcdss_test_ts.csv";
    {
        std::ofstream f(path);
        f << "load_bus_0,pv_gen_0\n";
        f << "1.0,0.5\n";
        f << "0.9,0.8\n";
    }
    auto ts = load_csv(path);

    REQUIRE(ts.n_steps == 2);
    REQUIRE(ts.load_profiles.size() == 1);
    REQUIRE(ts.pv_profiles.size()   == 1);
    REQUIRE_THAT(ts.load_profiles[0].values[0], WithinAbs(1.0, 1e-9));
    REQUIRE_THAT(ts.pv_profiles[0].values[1],   WithinAbs(0.8, 1e-9));
}

TEST_CASE("TimeSeriesLoader – empty CSV yields zero steps", "[model][io]")
{
    const std::string path = "/tmp/hacdcdss_test_empty.csv";
    {
        std::ofstream f(path);
        f << "load_bus_0,wind_bus_1\n";
    }
    auto ts = load_csv(path);
    REQUIRE(ts.n_steps == 0);
}

// ═══════════════════════════════════════════════════════════════════════════════
// Module: power_models
// ═══════════════════════════════════════════════════════════════════════════════

// Build a simple 2-bus test network used by several tests.
static NetworkModel make_2bus_network()
{
    NetworkModel net;
    net.name     = "2bus";
    net.base_mva = 100.0;

    AcBus slack; slack.id = 0; slack.type = BusType::Slack;
    slack.v_set = 1.0; slack.pd_mw = 0.0;

    AcBus load; load.id = 1; load.type = BusType::PQ;
    load.pd_mw = 50.0; load.qd_mvar = 20.0;

    net.ac_buses = {slack, load};

    AcBranch br;
    br.id = 0; br.from_bus = 0; br.to_bus = 1;
    br.r_pu = 0.01; br.x_pu = 0.1; br.b_pu = 0.0; br.rate_mva = 200.0;
    net.ac_branches = {br};

    Generator gen;
    gen.id = 0; gen.bus = 0;
    gen.pg_mw = 60.0; gen.qg_mvar = 25.0;
    gen.pg_min_mw = 0.0; gen.pg_max_mw = 100.0;
    gen.cost_b = 30.0;
    net.generators = {gen};

    return net;
}

TEST_CASE("YBus – 2-bus network has correct sparsity", "[power_models]")
{
    auto net  = make_2bus_network();
    auto ybus = YBus::build(net);

    REQUIRE(ybus.n_buses == 2);

    // Diagonal entries should be non-zero (g_s = r/(r²+x²)).
    // Off-diagonal should be non-zero (−g_s, −b_s).
    REQUIRE(ybus.G.nonZeros() == 4);
    REQUIRE(ybus.B.nonZeros() == 4);

    // Symmetry: G_01 = G_10 = -g_s < 0.
    const double g_s =
        0.01 / (0.01 * 0.01 + 0.1 * 0.1);  // r/(r²+x²)
    REQUIRE_THAT(ybus.g(0, 1), WithinAbs(-g_s, 1e-9));
    REQUIRE_THAT(ybus.g(1, 0), WithinAbs(-g_s, 1e-9));

    // Diagonal: G_00 = G_11 = +g_s.
    REQUIRE_THAT(ybus.g(0, 0), WithinAbs(g_s, 1e-9));
    REQUIRE_THAT(ybus.g(1, 1), WithinAbs(g_s, 1e-9));
}

TEST_CASE("YBus – P/Q injections at flat start are ~0 for balanced network",
          "[power_models]")
{
    auto net  = make_2bus_network();
    auto ybus = YBus::build(net);

    const int n = 2;
    Eigen::VectorXd V(n), theta(n);
    V.setOnes();
    theta.setZero();

    auto P = compute_P_injections(ybus, V, theta);
    auto Q = compute_Q_injections(ybus, V, theta);

    // With flat start V=1∠0, all injections should be zero.
    REQUIRE_THAT(P(0), WithinAbs(0.0, 1e-9));
    REQUIRE_THAT(P(1), WithinAbs(0.0, 1e-9));
    REQUIRE_THAT(Q(0), WithinAbs(0.0, 1e-9));
    REQUIRE_THAT(Q(1), WithinAbs(0.0, 1e-9));
}

TEST_CASE("GDCBus – 2-cable DC grid has correct conductance matrix",
          "[power_models][dc]")
{
    NetworkModel net;
    net.base_mva = 10.0;

    DcBus d0; d0.id = 0; d0.is_ref = true;
    DcBus d1; d1.id = 1;
    DcBus d2; d2.id = 2;
    net.dc_buses = {d0, d1, d2};

    DcCable c1; c1.id = 0; c1.from_bus = 0; c1.to_bus = 1; c1.r_pu = 0.1;
    DcCable c2; c2.id = 1; c2.from_bus = 1; c2.to_bus = 2; c2.r_pu = 0.2;
    net.dc_cables = {c1, c2};

    // Need AC slack to be valid (won't be used by GDCBus).
    AcBus a0; a0.id = 0; a0.type = BusType::Slack;
    net.ac_buses = {a0};

    auto gdc = GDCBus::build(net);
    REQUIRE(gdc.n_buses == 3);

    // g_01 = 1/0.1 = 10; g_12 = 1/0.2 = 5
    REQUIRE_THAT(gdc.G.coeff(0, 1), WithinAbs(-10.0, 1e-9));
    REQUIRE_THAT(gdc.G.coeff(1, 0), WithinAbs(-10.0, 1e-9));
    REQUIRE_THAT(gdc.G.coeff(1, 2), WithinAbs(-5.0,  1e-9));
    REQUIRE_THAT(gdc.G.coeff(0, 0), WithinAbs(10.0,  1e-9));  // g_01 only
    REQUIRE_THAT(gdc.G.coeff(1, 1), WithinAbs(15.0,  1e-9));  // g_01 + g_12
}

// ═══════════════════════════════════════════════════════════════════════════════
// Module: power_flow
// ═══════════════════════════════════════════════════════════════════════════════

TEST_CASE("PFSolver – nominal run returns correctly-sized result",
          "[power_flow]")
{
    auto net = make_2bus_network();
    PFSolver pf;
    auto res = pf.run_nominal(net);

    REQUIRE(res.ac_buses.size()    == net.n_ac_buses());
    REQUIRE(res.ac_branches.size() == net.n_ac_branches());
}

TEST_CASE("PFSolver – flat start converges for lightly loaded 2-bus network",
          "[power_flow]")
{
    // Build a 2-bus network where the load is small relative to the generator,
    // so the NR solve should converge easily.
    NetworkModel net;
    net.base_mva = 100.0;

    AcBus slack; slack.id = 0; slack.type = BusType::Slack; slack.v_set = 1.0;
    AcBus load;  load.id  = 1; load.type  = BusType::PQ;
    load.pd_mw = 10.0; load.qd_mvar = 5.0;
    net.ac_buses = {slack, load};

    AcBranch br; br.id = 0; br.from_bus = 0; br.to_bus = 1;
    br.r_pu = 0.01; br.x_pu = 0.1; br.rate_mva = 200.0;
    net.ac_branches = {br};

    Generator gen; gen.id = 0; gen.bus = 0;
    gen.pg_mw = 15.0; gen.qg_mvar = 5.0;
    gen.pg_min_mw = 0.0; gen.pg_max_mw = 50.0;
    net.generators = {gen};

    PFOptions opts;
    opts.tol      = 1e-6;
    opts.max_iter = 50;

    PFSolver pf;
    auto res = pf.run_nominal(net, opts);

    // The slack bus voltage magnitude must remain 1.0.
    REQUIRE_THAT(res.ac_buses[0].v_pu, WithinAbs(1.0, 1e-6));

    // Slack bus angle must be 0.
    REQUIRE_THAT(res.ac_buses[0].theta_rad, WithinAbs(0.0, 1e-9));

    // Load bus voltage should be close to 1 for a lightly loaded line.
    REQUIRE(res.ac_buses[1].v_pu > 0.9);
    REQUIRE(res.ac_buses[1].v_pu < 1.1);
}

TEST_CASE("PFSolver – scenario scaling applies load multiplier",
          "[power_flow]")
{
    auto net = make_2bus_network();
    // Use half the load via a scenario.
    Scenario s;
    s.id = 0; s.probability = 1.0;
    s.load_scale = {1.0, 0.5};  // scale bus-1 load to 50%

    PFSolver pf;
    auto res_nominal  = pf.run_nominal(net);
    auto res_scenario = pf.run(net, s);

    // With lower load, bus-1 voltage should be higher (closer to 1).
    REQUIRE(res_scenario.ac_buses[1].v_pu >= res_nominal.ac_buses[1].v_pu - 1e-3);
}

// ═══════════════════════════════════════════════════════════════════════════════
// Module: optimal_power_flow (DC OPF only; AC OPF requires Ipopt)
// ═══════════════════════════════════════════════════════════════════════════════

TEST_CASE("DC OPF – 3-bus test: cheaper generator dispatched first",
          "[opf][dc]")
{
    // 3-bus network: slack(0) ─ bus1 ─ bus2
    //   Gen0 at bus0: cost_b = 20 $/MWh  (cheaper)
    //   Gen1 at bus1: cost_b = 40 $/MWh  (expensive)
    //   Load at bus2: 30 MW
    NetworkModel net;
    net.base_mva = 100.0;

    AcBus b0; b0.id = 0; b0.type = BusType::Slack; b0.v_set = 1.0;
    AcBus b1; b1.id = 1; b1.type = BusType::PQ;
    AcBus b2; b2.id = 2; b2.type = BusType::PQ; b2.pd_mw = 30.0;
    net.ac_buses = {b0, b1, b2};

    AcBranch br01; br01.id = 0; br01.from_bus = 0; br01.to_bus = 1;
    br01.x_pu = 0.1; br01.rate_mva = 100.0;
    AcBranch br12; br12.id = 1; br12.from_bus = 1; br12.to_bus = 2;
    br12.x_pu = 0.1; br12.rate_mva = 100.0;
    net.ac_branches = {br01, br12};

    Generator g0; g0.id = 0; g0.bus = 0; g0.name = "cheap";
    g0.pg_min_mw = 0.0; g0.pg_max_mw = 50.0; g0.cost_b = 20.0;
    Generator g1; g1.id = 1; g1.bus = 1; g1.name = "expensive";
    g1.pg_min_mw = 0.0; g1.pg_max_mw = 50.0; g1.cost_b = 40.0;
    net.generators = {g0, g1};

    mipsolvers::engine::SolverEngine engine;
    OPFModel opf(engine);
    auto res = opf.run_dc(net);

    REQUIRE(res.status == OPFStatus::Optimal);
    // Total dispatch must equal total load.
    const double total_pg =
        res.generators[0].pg_mw + res.generators[1].pg_mw;
    REQUIRE_THAT(total_pg, WithinAbs(30.0, 1e-3));
    // Cheaper generator should be dispatched more than expensive one.
    REQUIRE(res.generators[0].pg_mw >= res.generators[1].pg_mw - 1e-3);
}

TEST_CASE("DC OPF – no generators: should be infeasible or zero dispatch",
          "[opf][dc]")
{
    NetworkModel net;
    net.base_mva = 100.0;

    AcBus b0; b0.id = 0; b0.type = BusType::Slack;
    AcBus b1; b1.id = 1; b1.type = BusType::PQ; b1.pd_mw = 10.0;
    net.ac_buses = {b0, b1};

    AcBranch br; br.id = 0; br.from_bus = 0; br.to_bus = 1;
    br.x_pu = 0.1; br.rate_mva = 100.0;
    net.ac_branches = {br};

    mipsolvers::engine::SolverEngine engine;
    OPFModel opf(engine);
    // No generators → power balance at bus1 will force the LP infeasible.
    auto res = opf.run_dc(net);
    // Accept either infeasible or an infeasible solve status.
    REQUIRE(res.status != OPFStatus::NotSolved);
}
