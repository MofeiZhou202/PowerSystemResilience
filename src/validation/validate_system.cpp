// src/validation/validate_system.cpp
//
// Static validation of HybridPowerSystem before projection/solving.

#include "hacdcpf/validation/validate_system.hpp"

#include <cmath>
#include <unordered_set>

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
        if (ac_ids.find(g.bus) == ac_ids.end())
            r.add(S::Error, "Generator", std::to_string(g.bus),
                  "bus", "Generator connected to missing AC bus " + std::to_string(g.bus));
    }

    // ── 8. Slack bus ──────────────────────────────────────────────────────────
    {
        int slack_count = 0;
        for (const auto& b : sys.ac.buses)
            if (b.bus_type == BusType::SLACK) ++slack_count;
        for (const auto& g : sys.ac.generators)
            if (g.is_slack) ++slack_count;
        if (slack_count == 0)
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

    return r;
}

}  // namespace hacdcpf::validation
