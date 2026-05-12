#pragma once
// hacdcdss/model/network_model.hpp
//
// Module: model
//
// NetworkModel describes the static topology of a hybrid AC/DC distribution
// network: AC buses, DC buses, AC branches, DC cables, VSC converters,
// generators (including DERs), and storage units.
//
// Data can be loaded from JSON (Matpower-style or native HACDCDSS format).
// The struct is consumed by all downstream modules (power_models, power_flow,
// optimal_power_flow).

#include <cstdint>
#include <string>
#include <vector>

namespace hacdcdss::model {

// ── Node identifiers ──────────────────────────────────────────────────────────
using BusId   = std::uint32_t;  // AC bus index (0-based)
using DCBusId = std::uint32_t;  // DC bus index (0-based)

// ── AC bus ────────────────────────────────────────────────────────────────────
enum class BusType : std::uint8_t {
    PQ    = 1,  // load bus (P and Q scheduled)
    PV    = 2,  // generator bus (P and |V| scheduled)
    Slack = 3   // reference bus (|V| and angle fixed)
};

struct AcBus {
    BusId       id;
    std::string name;
    BusType     type    = BusType::PQ;
    double      base_kv = 10.5;   // nominal voltage [kV]
    double      v_min   = 0.95;   // lower voltage bound [p.u.]
    double      v_max   = 1.05;   // upper voltage bound [p.u.]
    double      v_set   = 1.0;    // voltage setpoint for PV/Slack buses [p.u.]
    double      pd_mw   = 0.0;    // active load [MW]
    double      qd_mvar = 0.0;    // reactive load [MVAr]
};

// ── DC bus ────────────────────────────────────────────────────────────────────
struct DcBus {
    DCBusId     id;
    std::string name;
    double      base_kv = 10.0;
    double      v_min   = 0.95;
    double      v_max   = 1.05;
    double      v_set   = 1.0;    // voltage setpoint for DC slack bus [p.u.]
    double      pd_mw   = 0.0;    // DC load [MW]
    bool        is_ref  = false;  // reference (slack) DC bus; exactly one per
                                  // connected DC island must be true
};

// ── AC branch (overhead line or cable) ───────────────────────────────────────
struct AcBranch {
    std::uint32_t id;
    BusId         from_bus;
    BusId         to_bus;
    double        r_pu      = 0.0;  // series resistance [p.u.]
    double        x_pu      = 0.0;  // series reactance [p.u.]
    double        b_pu      = 0.0;  // total line-charging susceptance [p.u.]
    double        rate_mva  = 0.0;  // thermal rating [MVA] (0 = unconstrained)
    bool          in_service = true;
};

// ── DC cable ──────────────────────────────────────────────────────────────────
struct DcCable {
    std::uint32_t id;
    DCBusId       from_bus;
    DCBusId       to_bus;
    double        r_pu     = 0.0;   // cable resistance [p.u.]
    double        rate_mw  = 0.0;   // power rating [MW] (0 = unconstrained)
    bool          in_service = true;
};

// ── VSC converter (bidirectional AC ↔ DC) ────────────────────────────────────
//
// Loss model: P_loss = loss_a + loss_b * |P_ac|
// (constant + proportional term; quadratic term omitted for LP compatibility)
//
// Control modes:
//   dc_slack = true   → This VSC controls the DC bus voltage (DC slack).
//                       Its DC-side active power is determined by DC balance.
//   dc_slack = false  → Active power P_set_mw is tracked; DC voltage is free.
struct VscConverter {
    std::uint32_t id;
    BusId         ac_bus;
    DCBusId       dc_bus;
    double        rate_mva    = 0.0;  // apparent power rating [MVA]
    double        loss_a_mw   = 0.0;  // constant loss [MW]
    double        loss_b_pu   = 0.0;  // proportional loss coefficient [-]
    double        p_set_mw    = 0.0;  // AC-side active power setpoint [MW]
                                      // (+ve = inject into AC, −ve = absorb)
    double        q_set_mvar  = 0.0;  // AC-side reactive power setpoint [MVAr]
    double        q_min_mvar  = -1e9;
    double        q_max_mvar  =  1e9;
    bool          dc_slack    = false; // true for the VSC controlling DC voltage
};

// ── Generator (synchronous machine, DER, or renewable) ───────────────────────
struct Generator {
    std::uint32_t id;
    BusId         bus;
    std::string   name;
    double        pg_mw      = 0.0;    // current active power output [MW]
    double        qg_mvar    = 0.0;    // current reactive power output [MVAr]
    double        pg_min_mw  = 0.0;
    double        pg_max_mw  = 0.0;
    double        qg_min_mvar = -1e9;
    double        qg_max_mvar =  1e9;
    double        cost_a      = 0.0;   // quadratic cost coefficient [$/MWh²]
    double        cost_b      = 50.0;  // linear cost coefficient    [$/MWh]
    double        cost_c      = 0.0;   // constant cost              [$/h]
    bool          is_renewable = false;
    bool          in_service   = true;
};

// ── Storage unit ──────────────────────────────────────────────────────────────
struct StorageUnit {
    std::uint32_t id;
    BusId         bus;
    std::string   name;
    double        energy_cap_mwh   = 0.0;
    double        p_charge_max_mw  = 0.0;
    double        p_discharge_max_mw = 0.0;
    double        eta_charge        = 0.95;
    double        eta_discharge     = 0.95;
    double        soc_min           = 0.1;
    double        soc_max           = 0.9;
    double        soc_init          = 0.5;
};

// ── Full network model ────────────────────────────────────────────────────────
struct NetworkModel {
    std::string name;
    double      base_mva = 100.0;

    std::vector<AcBus>        ac_buses;
    std::vector<DcBus>        dc_buses;
    std::vector<AcBranch>     ac_branches;
    std::vector<DcCable>      dc_cables;
    std::vector<VscConverter> converters;
    std::vector<Generator>    generators;
    std::vector<StorageUnit>  storages;

    // ── Factory helpers ───────────────────────────────────────────────────────

    /// Load from a JSON file (Matpower-style or native HACDCDSS format).
    static NetworkModel from_json(const std::string& path);

    /// Validate topology and parameter bounds; throws std::runtime_error.
    void validate() const;

    // ── Convenience accessors ─────────────────────────────────────────────────
    std::size_t n_ac_buses()    const noexcept { return ac_buses.size(); }
    std::size_t n_dc_buses()    const noexcept { return dc_buses.size(); }
    std::size_t n_ac_branches() const noexcept { return ac_branches.size(); }
    std::size_t n_dc_cables()   const noexcept { return dc_cables.size(); }
    std::size_t n_converters()  const noexcept { return converters.size(); }
    std::size_t n_generators()  const noexcept { return generators.size(); }
    std::size_t n_storages()    const noexcept { return storages.size(); }

    /// Return index of the first AC bus with BusType::Slack, or SIZE_MAX.
    std::size_t ac_slack_idx() const noexcept;
    /// Return index of the first DC bus with is_ref == true, or SIZE_MAX.
    std::size_t dc_ref_idx()   const noexcept;
};

} // namespace hacdcdss::model
