// src/validation/validate_system.cpp
//
// Static validation of HybridPowerSystem before projection/solving.

#include "hacdcpf/validation/validate_system.hpp"

#include <cmath>
#include <string>
#include <unordered_set>

#include "hacdcpf/graph/graph.hpp"

namespace hacdcpf::validation {

namespace {

// ── helpers ──────────────────────────────────────────────────────────────────

template <typename Container, typename IdFn>
std::unordered_set<int> collect_ids(const Container& c, IdFn id_fn) {
    std::unordered_set<int> s;
    s.reserve(c.size());
    for (const auto& elem : c) s.insert(id_fn(elem));
    return s;
}

}  // namespace

// ── public API ───────────────────────────────────────────────────────────────

ValidationReport validate(const HybridPowerSystem& sys) {
    ValidationReport r;
    using S = Severity;

    // ── 1. base_mva > 0 ──────────────────────────────────────────────────────
    if (sys.base_mva <= 0.0)
        r.add(S::Error, "HybridPowerSystem", "", "base_mva",
              "base_mva must be positive (got " + std::to_string(sys.base_mva) + ")");

    // ── 2. Duplicate AC bus IDs ───────────────────────────────────────────────
    {
        std::unordered_set<int> seen;
        for (const auto& b : sys.ac.buses) {
            if (!seen.insert(b.index).second)
                r.add(S::Error, "ACBus", std::to_string(b.index),
                      "index", "Duplicate AC bus index " + std::to_string(b.index));
        }
    }

    // ── 3. Duplicate DC bus IDs ───────────────────────────────────────────────
    {
        std::unordered_set<int> seen;
        for (const auto& b : sys.dc.buses) {
            if (!seen.insert(b.index).second)
                r.add(S::Error, "DCBus", std::to_string(b.index),
                      "index", "Duplicate DC bus index " + std::to_string(b.index));
        }
    }

    const auto ac_ids = collect_ids(sys.ac.buses, [](const ACBus& b){ return b.index; });
    const auto dc_ids = collect_ids(sys.dc.buses, [](const DCBus& b){ return b.index; });

    auto require_ac_bus = [&](const std::string& type, int id,
                              const std::string& field, int bus) {
        if (ac_ids.find(bus) == ac_ids.end())
            r.add(S::Error, type, std::to_string(id), field,
                  field + " " + std::to_string(bus) + " not found in AC");
    };
    auto require_dc_bus = [&](const std::string& type, int id,
                              const std::string& field, int bus) {
        if (dc_ids.find(bus) == dc_ids.end())
            r.add(S::Error, type, std::to_string(id), field,
                  field + " " + std::to_string(bus) + " not found in DC");
    };
    auto require_order = [&](const std::string& type, int id,
                             const std::string& lo_field, const std::string& hi_field,
                             double lo, double hi) {
        if (lo > hi)
            r.add(S::Error, type, std::to_string(id), lo_field,
                  lo_field + " (" + std::to_string(lo) + ") > " +
                  hi_field + " (" + std::to_string(hi) + ")");
    };
    auto require_nonnegative = [&](const std::string& type, int id,
                                   const std::string& field, double value) {
        if (value < 0.0)
            r.add(S::Error, type, std::to_string(id), field,
                  field + " must be non-negative (got " + std::to_string(value) + ")");
    };
    auto require_unit_interval = [&](const std::string& type, int id,
                                     const std::string& field, double value) {
        if (value < 0.0 || value > 1.0)
            r.add(S::Error, type, std::to_string(id), field,
                  field + " must be in [0,1] (got " + std::to_string(value) + ")");
    };
    auto require_efficiency = [&](const std::string& type, int id,
                                  const std::string& field, double value) {
        if (value <= 0.0 || value > 1.0)
            r.add(S::Error, type, std::to_string(id), field,
                  field + " must be in (0,1] (got " + std::to_string(value) + ")");
    };

    // ── 4. AC branch bus references ───────────────────────────────────────────
    for (const auto& br : sys.ac.branches) {
        if (ac_ids.find(br.from_bus) == ac_ids.end())
            r.add(S::Error, "ACBranch", std::to_string(br.index),
                  "from_bus", "from_bus " + std::to_string(br.from_bus) + " not found");
        if (ac_ids.find(br.to_bus) == ac_ids.end())
            r.add(S::Error, "ACBranch", std::to_string(br.index),
                  "to_bus", "to_bus " + std::to_string(br.to_bus) + " not found");
    }

    // ── 5. DC branch bus references ───────────────────────────────────────────
    for (const auto& br : sys.dc.branches) {
        if (dc_ids.find(br.from_bus) == dc_ids.end())
            r.add(S::Error, "DCBranch", std::to_string(br.index),
                  "from_bus", "from_bus " + std::to_string(br.from_bus) + " not found in DC");
        if (dc_ids.find(br.to_bus) == dc_ids.end())
            r.add(S::Error, "DCBranch", std::to_string(br.index),
                  "to_bus", "to_bus " + std::to_string(br.to_bus) + " not found in DC");
    }

    // ── 6. AC bus voltage limits ──────────────────────────────────────────────
    for (const auto& b : sys.ac.buses) {
        if (b.vmin_pu <= 0.0)
            r.add(S::Error, "ACBus", std::to_string(b.index),
                  "vmin_pu", "vmin_pu must be positive (got " + std::to_string(b.vmin_pu) + ")");
        if (b.vmax_pu <= b.vmin_pu)
            r.add(S::Error, "ACBus", std::to_string(b.index),
                  "vmax_pu", "vmax_pu (" + std::to_string(b.vmax_pu) +
                  ") must exceed vmin_pu (" + std::to_string(b.vmin_pu) + ")");
    }

    // ── 6b. DC bus voltage limits ─────────────────────────────────────────────
    for (const auto& b : sys.dc.buses) {
        if (b.vmin_pu <= 0.0)
            r.add(S::Error, "DCBus", std::to_string(b.index),
                  "vmin_pu", "vmin_pu must be positive (got " + std::to_string(b.vmin_pu) + ")");
        if (b.vmax_pu <= b.vmin_pu)
            r.add(S::Error, "DCBus", std::to_string(b.index),
                  "vmax_pu", "vmax_pu (" + std::to_string(b.vmax_pu) +
                  ") must exceed vmin_pu (" + std::to_string(b.vmin_pu) + ")");
    }

    // ── 7. Generator limits ───────────────────────────────────────────────────
    for (const auto& g : sys.ac.generators) {
        if (g.pmin_mw > g.pmax_mw)
            r.add(S::Error, "Generator", std::to_string(g.bus),
                  "pmin_mw", "pmin_mw (" + std::to_string(g.pmin_mw) +
                  ") > pmax_mw (" + std::to_string(g.pmax_mw) + ")");
        if (g.qmin_mvar > g.qmax_mvar)
            r.add(S::Error, "Generator", std::to_string(g.bus),
                  "qmin_mvar", "qmin_mvar (" + std::to_string(g.qmin_mvar) +
                  ") > qmax_mvar (" + std::to_string(g.qmax_mvar) + ")");
        if (!g.in_service) continue;
        if (g.pg_mw < g.pmin_mw - 1e-6 || g.pg_mw > g.pmax_mw + 1e-6)
            r.add(S::Warning, "Generator", std::to_string(g.bus),
                  "pg_mw", "Initial dispatch pg_mw=" + std::to_string(g.pg_mw) +
                  " outside [pmin=" + std::to_string(g.pmin_mw) +
                  ", pmax=" + std::to_string(g.pmax_mw) + "]");
        require_ac_bus("Generator", g.index, "bus", g.bus);
    }

    // Rich AC component bus references and basic bounds.
    for (const auto& g : sys.ac.static_generators) {
        require_ac_bus("StaticGenerator", g.index, "bus", g.bus);
        require_order("StaticGenerator", g.index, "pmin_mw", "pmax_mw", g.pmin_mw, g.pmax_mw);
        require_order("StaticGenerator", g.index, "qmin_mvar", "qmax_mvar", g.qmin_mvar, g.qmax_mvar);
        require_nonnegative("StaticGenerator", g.index, "scaling", g.scaling);
    }
    for (const auto& ld : sys.ac.loads) {
        require_ac_bus("Load", ld.index, "bus", ld.bus);
        require_nonnegative("Load", ld.index, "scaling", ld.scaling);
    }
    for (const auto& ld : sys.ac.flexible_loads) {
        require_ac_bus("FlexibleLoad", ld.index, "bus", ld.bus);
        require_nonnegative("FlexibleLoad", ld.index, "flex_up_mw", ld.flex_up_mw);
        require_nonnegative("FlexibleLoad", ld.index, "flex_down_mw", ld.flex_down_mw);
        if (ld.availability_pct < 0.0 || ld.availability_pct > 100.0)
            r.add(S::Error, "FlexibleLoad", std::to_string(ld.index), "availability_pct",
                  "availability_pct must be in [0,100] (got " + std::to_string(ld.availability_pct) + ")");
    }
    for (const auto& ld : sys.ac.asymmetric_loads) {
        require_ac_bus("AsymmetricLoad", ld.index, "bus", ld.bus);
        require_nonnegative("AsymmetricLoad", ld.index, "scaling", ld.scaling);
    }
    for (const auto& sh : sys.ac.shunts) {
        require_ac_bus("Shunt", sh.index, "bus", sh.bus);
        if (sh.n_steps < 0)
            r.add(S::Error, "Shunt", std::to_string(sh.index), "n_steps", "n_steps must be non-negative");
        if (sh.current_step < 0 || (sh.n_steps > 0 && sh.current_step > sh.n_steps))
            r.add(S::Error, "Shunt", std::to_string(sh.index), "current_step",
                  "current_step must be within [0,n_steps]");
    }
    for (const auto& st : sys.ac.storage) {
        require_ac_bus("Storage", st.index, "bus", st.bus);
        require_order("Storage", st.index, "pmin_mw", "pmax_mw", st.pmin_mw, st.pmax_mw);
        require_order("Storage", st.index, "qmin_mvar", "qmax_mvar", st.qmin_mvar, st.qmax_mvar);
        require_nonnegative("Storage", st.index, "e_rated_mwh", st.e_rated_mwh);
        require_unit_interval("Storage", st.index, "soc_min", st.soc_min);
        require_unit_interval("Storage", st.index, "soc_max", st.soc_max);
        require_unit_interval("Storage", st.index, "soc_init", st.soc_init);
        require_order("Storage", st.index, "soc_min", "soc_max", st.soc_min, st.soc_max);
        require_efficiency("Storage", st.index, "eta_charge", st.eta_charge);
        require_efficiency("Storage", st.index, "eta_discharge", st.eta_discharge);
    }
    for (const auto& rg : sys.ac.renewable_gens) {
        require_ac_bus("RenewableGen", rg.index, "bus", rg.bus);
        require_nonnegative("RenewableGen", rg.index, "p_rated_mw", rg.p_rated_mw);
        require_unit_interval("RenewableGen", rg.index, "capacity_factor", rg.capacity_factor);
        require_order("RenewableGen", rg.index, "qmin_mvar", "qmax_mvar", rg.qmin_mvar, rg.qmax_mvar);
    }
    for (const auto& pv : sys.ac.pv_systems) {
        require_ac_bus("PVSystem", pv.index, "bus", pv.bus);
        require_order("PVSystem", pv.index, "pmin_mw", "pmax_mw", pv.pmin_mw, pv.pmax_mw);
        require_order("PVSystem", pv.index, "qmin_mvar", "qmax_mvar", pv.qmin_mvar, pv.qmax_mvar);
        require_efficiency("PVSystem", pv.index, "inverter_eff", pv.inverter_eff);
    }
    for (const auto& eg : sys.ac.external_grids) {
        require_ac_bus("ExternalGrid", eg.index, "bus", eg.bus);
        if (eg.s_sc_max_mva > 0.0 && eg.s_sc_min_mva > eg.s_sc_max_mva)
            r.add(S::Error, "ExternalGrid", std::to_string(eg.index), "s_sc_min_mva",
                  "s_sc_min_mva exceeds s_sc_max_mva");
    }
    for (const auto& tr : sys.ac.transformers_2w) {
        require_ac_bus("Transformer2W", tr.index, "hv_bus", tr.hv_bus);
        require_ac_bus("Transformer2W", tr.index, "lv_bus", tr.lv_bus);
        require_nonnegative("Transformer2W", tr.index, "sn_mva", tr.sn_mva);
    }
    for (const auto& tr : sys.ac.transformers_3w) {
        require_ac_bus("Transformer3W", tr.index, "hv_bus", tr.hv_bus);
        require_ac_bus("Transformer3W", tr.index, "mv_bus", tr.mv_bus);
        require_ac_bus("Transformer3W", tr.index, "lv_bus", tr.lv_bus);
        require_nonnegative("Transformer3W", tr.index, "sn_hv_mva", tr.sn_hv_mva);
        require_nonnegative("Transformer3W", tr.index, "sn_mv_mva", tr.sn_mv_mva);
        require_nonnegative("Transformer3W", tr.index, "sn_lv_mva", tr.sn_lv_mva);
    }
    for (const auto& sw : sys.ac.switches) {
        require_ac_bus("Switch", sw.index, "bus_from", sw.bus_from);
        require_ac_bus("Switch", sw.index, "bus_to", sw.bus_to);
        require_nonnegative("Switch", sw.index, "p_sw_fail", sw.p_sw_fail);
    }
    for (const auto& cb : sys.ac.circuit_breakers) {
        require_ac_bus("CircuitBreaker", cb.index, "bus_from", cb.bus_from);
        require_ac_bus("CircuitBreaker", cb.index, "bus_to", cb.bus_to);
    }
    for (const auto& cs : sys.ac.charging_stations) {
        require_ac_bus("ChargingStation", cs.index, "bus", cs.bus);
        require_nonnegative("ChargingStation", cs.index, "max_power_kw", cs.max_power_kw);
    }
    for (const auto& motor : sys.ac.motors) {
        require_ac_bus("AsynchronousMotor", motor.index, "bus", motor.bus);
        require_nonnegative("AsynchronousMotor", motor.index, "sn_mva", motor.sn_mva);
        require_efficiency("AsynchronousMotor", motor.index, "efficiency", motor.efficiency);
    }

    const auto charging_station_ids = collect_ids(sys.ac.charging_stations,
        [](const ChargingStation& cs){ return cs.index; });
    for (const auto& charger : sys.ac.chargers) {
        if (charging_station_ids.find(charger.station_id) == charging_station_ids.end())
            r.add(S::Error, "Charger", std::to_string(charger.index), "station_id",
                  "station_id " + std::to_string(charger.station_id) + " not found");
        require_nonnegative("Charger", charger.index, "p_rated_kw", charger.p_rated_kw);
        require_efficiency("Charger", charger.index, "eta", charger.eta);
    }

    for (const auto& ld : sys.dc.loads) {
        require_dc_bus("DCLoad", ld.index, "bus", ld.bus);
        require_nonnegative("DCLoad", ld.index, "scaling", ld.scaling);
    }
    for (const auto& st : sys.dc.storage) {
        require_dc_bus("DCStorage", st.index, "bus", st.bus);
        require_order("DCStorage", st.index, "pmin_mw", "pmax_mw", st.pmin_mw, st.pmax_mw);
        require_nonnegative("DCStorage", st.index, "e_rated_mwh", st.e_rated_mwh);
        require_unit_interval("DCStorage", st.index, "soc_min", st.soc_min);
        require_unit_interval("DCStorage", st.index, "soc_max", st.soc_max);
        require_unit_interval("DCStorage", st.index, "soc_init", st.soc_init);
        require_order("DCStorage", st.index, "soc_min", "soc_max", st.soc_min, st.soc_max);
    }
    for (const auto& st : sys.dc.dc_storage) {
        require_dc_bus("DCStorageUnit", st.index, "bus", st.bus);
        require_order("DCStorageUnit", st.index, "pmin_mw", "pmax_mw", st.pmin_mw, st.pmax_mw);
        require_nonnegative("DCStorageUnit", st.index, "e_rated_mwh", st.e_rated_mwh);
        require_unit_interval("DCStorageUnit", st.index, "soc_min", st.soc_min);
        require_unit_interval("DCStorageUnit", st.index, "soc_max", st.soc_max);
        require_unit_interval("DCStorageUnit", st.index, "soc_init", st.soc_init);
        require_order("DCStorageUnit", st.index, "soc_min", "soc_max", st.soc_min, st.soc_max);
    }
    for (const auto& sg : sys.dc.static_generators) {
        require_dc_bus("DCStaticGenerator", sg.index, "bus", sg.bus);
        require_order("DCStaticGenerator", sg.index, "pmin_mw", "pmax_mw", sg.pmin_mw, sg.pmax_mw);
        require_nonnegative("DCStaticGenerator", sg.index, "scaling", sg.scaling);
    }
    for (const auto& sg : sys.dc.dc_static_generators) {
        require_dc_bus("StaticGeneratorDC", sg.index, "bus", sg.bus);
        require_order("StaticGeneratorDC", sg.index, "pmin_mw", "pmax_mw", sg.pmin_mw, sg.pmax_mw);
        require_nonnegative("StaticGeneratorDC", sg.index, "scaling", sg.scaling);
    }
    for (const auto& pv : sys.dc.pv_arrays) {
        require_dc_bus("PVArrayDC", pv.index, "bus", pv.bus);
        require_nonnegative("PVArrayDC", pv.index, "p_set_mw", pv.p_set_mw);
    }
    for (const auto& c : sys.dc.dcdc_converters) {
        require_dc_bus("DCDCConverter", c.index, "bus_in", c.bus_in);
        require_dc_bus("DCDCConverter", c.index, "bus_out", c.bus_out);
        require_order("DCDCConverter", c.index, "pmin_mw", "pmax_mw", c.pmin_mw, c.pmax_mw);
        require_efficiency("DCDCConverter", c.index, "eta", c.eta);
    }
    for (const auto& cb : sys.dc.dc_circuit_breakers) {
        require_dc_bus("DCCircuitBreaker", cb.index, "bus_from", cb.bus_from);
        require_dc_bus("DCCircuitBreaker", cb.index, "bus_to", cb.bus_to);
    }

    // ── 8. Slack bus ──────────────────────────────────────────────────────────
    {
        int slack_count = 0;
        for (const auto& b : sys.ac.buses)
            if (b.bus_type == BusType::SLACK) ++slack_count;
        for (const auto& g : sys.ac.generators)
            if (g.is_slack) ++slack_count;
        if (!sys.ac.buses.empty() && slack_count == 0)
            r.add(S::Error, "ACSystem", "", "",
                  "No slack bus found (BusType::SLACK or Generator::is_slack)");
        if (slack_count > 1)
            r.add(S::Warning, "ACSystem", "", "",
                  std::to_string(slack_count) +
                  " slack buses found; distributed-slack is recommended");
    }

    // ── 9. Negative AC branch resistance ─────────────────────────────────────
    for (const auto& br : sys.ac.branches) {
        if (br.r_pu < 0.0)
            r.add(S::Warning, "ACBranch", std::to_string(br.index),
                  "r_pu", "Negative resistance r_pu=" + std::to_string(br.r_pu) +
                  " (series capacitor?)");
        if (br.x_pu == 0.0 && br.r_pu == 0.0)
            r.add(S::Warning, "ACBranch", std::to_string(br.index),
                  "r_pu/x_pu", "Zero impedance branch may cause numerical issues");
    }

    // ── 10. Transformer tap ratio ─────────────────────────────────────────────
    for (const auto& br : sys.ac.branches) {
        if (br.tap <= 0.0) {
            r.add(S::Error, "ACBranch", std::to_string(br.index),
                  "tap", "tap ratio must be positive (got " + std::to_string(br.tap) + ")");
        } else if (br.tap < 0.5 || br.tap > 2.0) {
            r.add(S::Warning, "ACBranch", std::to_string(br.index),
                  "tap", "tap=" + std::to_string(br.tap) +
                  " is outside the typical (0.5, 2.0) range");
        }
    }

    // ── 11. VSC converter bus references ──────────────────────────────────────
    for (const auto& c : sys.vsc_converters) {
        if (ac_ids.find(c.bus_ac) == ac_ids.end())
            r.add(S::Error, "VSCConverter", std::to_string(c.index),
                  "bus_ac", "bus_ac " + std::to_string(c.bus_ac) + " not found");
        if (dc_ids.find(c.bus_dc) == dc_ids.end())
            r.add(S::Error, "VSCConverter", std::to_string(c.index),
                  "bus_dc", "bus_dc " + std::to_string(c.bus_dc) + " not found");
        require_order("VSCConverter", c.index, "pmin_mw", "pmax_mw", c.pmin_mw, c.pmax_mw);
        require_order("VSCConverter", c.index, "qmin_mvar", "qmax_mvar", c.qmin_mvar, c.qmax_mvar);
        require_efficiency("VSCConverter", c.index, "eta", c.eta);
        if (c.loss_percent < 0.0 || c.loss_percent > 100.0)
            r.add(S::Error, "VSCConverter", std::to_string(c.index), "loss_percent",
                  "loss_percent must be in [0,100]");
    }

    // ── 12. base_mva sub-system consistency ───────────────────────────────────
    if (!sys.ac.buses.empty() && std::fabs(sys.ac.base_mva - sys.base_mva) > 0.1)
        r.add(S::Warning, "ACSystem", "", "base_mva",
              "ACSystem base_mva (" + std::to_string(sys.ac.base_mva) +
              ") differs from HybridPowerSystem base_mva (" +
              std::to_string(sys.base_mva) + ")");
    if (!sys.dc.buses.empty() && std::fabs(sys.dc.base_mva - sys.base_mva) > 0.1)
        r.add(S::Warning, "DCSystem", "", "base_mva",
              "DCSystem base_mva (" + std::to_string(sys.dc.base_mva) +
              ") differs from HybridPowerSystem base_mva (" +
              std::to_string(sys.base_mva) + ")");

    // ── 13. Graph connectivity check ──────────────────────────────────────────
    // Use the graph module to detect isolated load islands and no-slack islands.
    if (!sys.ac.buses.empty()) {
        namespace gr = hacdcpf::graph;
        const auto g    = gr::build_power_system_graph(sys);
        const auto topo = gr::analyze_topology(g);
        for (const auto& isl : topo.islands) {
            if (isl.status == gr::IslandStatus::IsolatedLoad) {
                for (int bid : isl.bus_ids)
                    r.add(S::Warning, "ACBus", std::to_string(bid),
                          "connectivity",
                          "bus " + std::to_string(bid) +
                          " is in an isolated load island (no path to any slack)");
            } else if (isl.status == gr::IslandStatus::NoSlack &&
                       isl.domain == gr::NodeDomain::AC) {
                r.add(S::Warning, "ACSystem", "", "connectivity",
                      "AC island " + std::to_string(isl.island_id) +
                      " (" + std::to_string(isl.bus_ids.size()) +
                      " buses) contains no slack bus");
            }
        }
    }

    return r;
}

ValidationReport validate(const HybridPowerSystem& sys, ValidationLevel level) {
    auto r = validate(sys);

    switch (level) {
    case ValidationLevel::SolverReady:
        return r;  // unchanged

    case ValidationLevel::Strict: {
        // Promote all Warnings to Errors.
        for (auto& issue : r.issues)
            if (issue.severity == Severity::Warning)
                issue.severity = Severity::Error;
        return r;
    }

    case ValidationLevel::Basic: {
        // Suppress all Warnings; keep only hard structural Errors.
        ValidationReport out;
        for (const auto& issue : r.issues)
            if (issue.severity == Severity::Error)
                out.issues.push_back(issue);
        return out;
    }

    case ValidationLevel::Electrical: {
        // Keep Errors + system-level topology / slack Warnings.
        ValidationReport out;
        for (const auto& issue : r.issues) {
            if (issue.severity == Severity::Error) {
                out.issues.push_back(issue);
            } else if (issue.component_type == "ACSystem" ||
                       issue.component_type == "DCSystem") {
                // Slack-bus, multiple-slack, base_mva mismatch — topology-level.
                out.issues.push_back(issue);
            }
        }
        return out;
    }
    }
    return r;  // unreachable, but avoids compiler warning
}

}  // namespace hacdcpf::validation
