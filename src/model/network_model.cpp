// src/model/network_model.cpp
//
// Module: model – NetworkModel I/O and validation.

#include <hacdcdss/model/network_model.hpp>
#include <nlohmann/json.hpp>
#include <fstream>
#include <stdexcept>

namespace hacdcdss::model {

// ── Helpers ───────────────────────────────────────────────────────────────────

namespace {

BusType bus_type_from_int(int t)
{
    switch (t) {
        case 1: return BusType::PQ;
        case 2: return BusType::PV;
        case 3: return BusType::Slack;
        default:
            throw std::runtime_error(
                "NetworkModel: unknown bus type " + std::to_string(t));
    }
}

} // anonymous namespace

// ── NetworkModel::from_json ───────────────────────────────────────────────────

NetworkModel NetworkModel::from_json(const std::string& path)
{
    std::ifstream f(path);
    if (!f.is_open())
        throw std::runtime_error(
            "NetworkModel::from_json: cannot open file '" + path + "'");

    nlohmann::json j;
    f >> j;

    NetworkModel net;
    net.name     = j.value("name",     "unnamed");
    net.base_mva = j.value("base_mva", 100.0);

    // ── AC buses ──────────────────────────────────────────────────────────────
    for (const auto& jb : j.value("ac_buses", nlohmann::json::array())) {
        AcBus bus;
        bus.id      = jb.at("id").get<BusId>();
        bus.name    = jb.value("name", "bus_" + std::to_string(bus.id));
        bus.type    = bus_type_from_int(jb.value("type", 1));
        bus.base_kv = jb.value("base_kv", 10.5);
        bus.v_min   = jb.value("v_min",   0.95);
        bus.v_max   = jb.value("v_max",   1.05);
        bus.v_set   = jb.value("v_set",   1.0);
        bus.pd_mw   = jb.value("pd_mw",   0.0);
        bus.qd_mvar = jb.value("qd_mvar", 0.0);
        net.ac_buses.push_back(std::move(bus));
    }

    // ── DC buses ──────────────────────────────────────────────────────────────
    for (const auto& jb : j.value("dc_buses", nlohmann::json::array())) {
        DcBus bus;
        bus.id      = jb.at("id").get<DCBusId>();
        bus.name    = jb.value("name", "dcbus_" + std::to_string(bus.id));
        bus.base_kv = jb.value("base_kv", 10.0);
        bus.v_min   = jb.value("v_min",   0.95);
        bus.v_max   = jb.value("v_max",   1.05);
        bus.v_set   = jb.value("v_set",   1.0);
        bus.pd_mw   = jb.value("pd_mw",   0.0);
        bus.is_ref  = jb.value("is_ref",  false);
        net.dc_buses.push_back(std::move(bus));
    }

    // ── AC branches ───────────────────────────────────────────────────────────
    for (const auto& jbr : j.value("ac_branches", nlohmann::json::array())) {
        AcBranch br;
        br.id         = jbr.at("id").get<std::uint32_t>();
        br.from_bus   = jbr.at("from_bus").get<BusId>();
        br.to_bus     = jbr.at("to_bus").get<BusId>();
        br.r_pu       = jbr.value("r_pu",      0.0);
        br.x_pu       = jbr.value("x_pu",      0.0);
        br.b_pu       = jbr.value("b_pu",      0.0);
        br.rate_mva   = jbr.value("rate_mva",  0.0);
        br.in_service = jbr.value("in_service", true);
        net.ac_branches.push_back(std::move(br));
    }

    // ── DC cables ─────────────────────────────────────────────────────────────
    for (const auto& jc : j.value("dc_cables", nlohmann::json::array())) {
        DcCable cable;
        cable.id         = jc.at("id").get<std::uint32_t>();
        cable.from_bus   = jc.at("from_bus").get<DCBusId>();
        cable.to_bus     = jc.at("to_bus").get<DCBusId>();
        cable.r_pu       = jc.value("r_pu",      0.0);
        cable.rate_mw    = jc.value("rate_mw",   0.0);
        cable.in_service = jc.value("in_service", true);
        net.dc_cables.push_back(std::move(cable));
    }

    // ── VSC converters ────────────────────────────────────────────────────────
    for (const auto& jv : j.value("converters", nlohmann::json::array())) {
        VscConverter vsc;
        vsc.id         = jv.at("id").get<std::uint32_t>();
        vsc.ac_bus     = jv.at("ac_bus").get<BusId>();
        vsc.dc_bus     = jv.at("dc_bus").get<DCBusId>();
        vsc.rate_mva   = jv.value("rate_mva",   0.0);
        vsc.loss_a_mw  = jv.value("loss_a_mw",  0.0);
        vsc.loss_b_pu  = jv.value("loss_b_pu",  0.0);
        vsc.p_set_mw   = jv.value("p_set_mw",   0.0);
        vsc.q_set_mvar = jv.value("q_set_mvar", 0.0);
        vsc.q_min_mvar = jv.value("q_min_mvar", -1e9);
        vsc.q_max_mvar = jv.value("q_max_mvar",  1e9);
        vsc.dc_slack   = jv.value("dc_slack",   false);
        net.converters.push_back(std::move(vsc));
    }

    // ── Generators ────────────────────────────────────────────────────────────
    for (const auto& jg : j.value("generators", nlohmann::json::array())) {
        Generator gen;
        gen.id           = jg.at("id").get<std::uint32_t>();
        gen.bus          = jg.at("bus").get<BusId>();
        gen.name         = jg.value("name", "gen_" + std::to_string(gen.id));
        gen.pg_mw        = jg.value("pg_mw",       0.0);
        gen.qg_mvar      = jg.value("qg_mvar",     0.0);
        gen.pg_min_mw    = jg.value("pg_min_mw",   0.0);
        gen.pg_max_mw    = jg.value("pg_max_mw",   0.0);
        gen.qg_min_mvar  = jg.value("qg_min_mvar", -1e9);
        gen.qg_max_mvar  = jg.value("qg_max_mvar",  1e9);
        gen.cost_a       = jg.value("cost_a",       0.0);
        gen.cost_b       = jg.value("cost_b",      50.0);
        gen.cost_c       = jg.value("cost_c",       0.0);
        gen.is_renewable = jg.value("is_renewable", false);
        gen.in_service   = jg.value("in_service",   true);
        net.generators.push_back(std::move(gen));
    }

    // ── Storage units ─────────────────────────────────────────────────────────
    for (const auto& js : j.value("storages", nlohmann::json::array())) {
        StorageUnit sto;
        sto.id                   = js.at("id").get<std::uint32_t>();
        sto.bus                  = js.at("bus").get<BusId>();
        sto.name                 = js.value("name", "sto_" + std::to_string(sto.id));
        sto.energy_cap_mwh       = js.value("energy_cap_mwh",    0.0);
        sto.p_charge_max_mw      = js.value("p_charge_max_mw",   0.0);
        sto.p_discharge_max_mw   = js.value("p_discharge_max_mw",0.0);
        sto.eta_charge           = js.value("eta_charge",   0.95);
        sto.eta_discharge        = js.value("eta_discharge",0.95);
        sto.soc_min              = js.value("soc_min", 0.1);
        sto.soc_max              = js.value("soc_max", 0.9);
        sto.soc_init             = js.value("soc_init", 0.5);
        net.storages.push_back(std::move(sto));
    }

    return net;
}

// ── NetworkModel::validate ────────────────────────────────────────────────────

void NetworkModel::validate() const
{
    if (ac_buses.empty())
        throw std::runtime_error("NetworkModel::validate: no AC buses defined.");

    const auto n_ac = ac_buses.size();
    const auto n_dc = dc_buses.size();

    // Check that exactly one AC slack exists.
    int n_slack = 0;
    for (const auto& b : ac_buses)
        if (b.type == BusType::Slack) ++n_slack;
    if (n_slack == 0)
        throw std::runtime_error(
            "NetworkModel::validate: no AC slack bus (BusType::Slack) found.");
    if (n_slack > 1)
        throw std::runtime_error(
            "NetworkModel::validate: more than one AC slack bus found.");

    // Check AC branch bus references.
    for (const auto& br : ac_branches) {
        if (br.from_bus >= n_ac)
            throw std::runtime_error(
                "AcBranch " + std::to_string(br.id) +
                ": from_bus " + std::to_string(br.from_bus) +
                " out of range (n_ac_buses = " + std::to_string(n_ac) + ").");
        if (br.to_bus >= n_ac)
            throw std::runtime_error(
                "AcBranch " + std::to_string(br.id) +
                ": to_bus " + std::to_string(br.to_bus) +
                " out of range (n_ac_buses = " + std::to_string(n_ac) + ").");
        if (br.from_bus == br.to_bus)
            throw std::runtime_error(
                "AcBranch " + std::to_string(br.id) + ": self-loop detected.");
    }

    // Check DC cable bus references (only if DC buses are defined).
    if (!dc_buses.empty()) {
        int n_dc_ref = 0;
        for (const auto& b : dc_buses)
            if (b.is_ref) ++n_dc_ref;
        if (n_dc_ref == 0)
            throw std::runtime_error(
                "NetworkModel::validate: DC buses defined but no DC reference "
                "bus (DcBus::is_ref = true) found.");

        for (const auto& c : dc_cables) {
            if (c.from_bus >= n_dc)
                throw std::runtime_error(
                    "DcCable " + std::to_string(c.id) +
                    ": from_bus out of range.");
            if (c.to_bus >= n_dc)
                throw std::runtime_error(
                    "DcCable " + std::to_string(c.id) +
                    ": to_bus out of range.");
        }
    }

    // Check converter bus references.
    for (const auto& vsc : converters) {
        if (vsc.ac_bus >= n_ac)
            throw std::runtime_error(
                "VscConverter " + std::to_string(vsc.id) +
                ": ac_bus out of range.");
        if (vsc.dc_bus >= n_dc)
            throw std::runtime_error(
                "VscConverter " + std::to_string(vsc.id) +
                ": dc_bus out of range.");
    }

    // Check generator bus references and parameter consistency.
    for (const auto& g : generators) {
        if (g.bus >= n_ac)
            throw std::runtime_error(
                "Generator " + std::to_string(g.id) +
                ": bus out of range.");
        if (g.pg_min_mw > g.pg_max_mw)
            throw std::runtime_error(
                "Generator " + std::to_string(g.id) +
                ": pg_min_mw > pg_max_mw.");
    }

    // Check storage bus references.
    for (const auto& s : storages) {
        if (s.bus >= n_ac)
            throw std::runtime_error(
                "StorageUnit " + std::to_string(s.id) +
                ": bus out of range.");
        if (s.soc_min >= s.soc_max)
            throw std::runtime_error(
                "StorageUnit " + std::to_string(s.id) +
                ": soc_min >= soc_max.");
    }
}

// ── NetworkModel::ac_slack_idx / dc_ref_idx ───────────────────────────────────

std::size_t NetworkModel::ac_slack_idx() const noexcept
{
    for (std::size_t i = 0; i < ac_buses.size(); ++i)
        if (ac_buses[i].type == BusType::Slack) return i;
    return std::size_t(-1);
}

std::size_t NetworkModel::dc_ref_idx() const noexcept
{
    for (std::size_t i = 0; i < dc_buses.size(); ++i)
        if (dc_buses[i].is_ref) return i;
    return std::size_t(-1);
}

} // namespace hacdcdss::model
