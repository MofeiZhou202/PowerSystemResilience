#pragma once
// hacdcdss/optimal_power_flow/opf_result.hpp
//
// Module: optimal_power_flow
//
// Result types returned by OPFModel after an OPF solve.

#include <cstdint>
#include <string>
#include <vector>

namespace hacdcdss::optimal_power_flow {

enum class OPFStatus : std::uint8_t {
    Optimal    = 0,
    Infeasible = 1,
    Unbounded  = 2,
    SolverError = 3,
    NotSolved  = 4
};

// ── Per-bus AC result ─────────────────────────────────────────────────────────
struct AcBusOPFResult {
    double v_pu       = 1.0;
    double theta_rad  = 0.0;
    double lmp_mwh    = 0.0;  // locational marginal price [$/MWh] (dual)
};

// ── Per-generator result ──────────────────────────────────────────────────────
struct GeneratorOPFResult {
    double pg_mw   = 0.0;   // dispatched active power [MW]
    double qg_mvar = 0.0;   // dispatched reactive power [MVAr]
    double cost_usd = 0.0;  // operating cost for this generator [$]
};

// ── Full OPF result ───────────────────────────────────────────────────────────
struct OPFResult {
    OPFStatus   status       = OPFStatus::NotSolved;
    double      total_cost   = 0.0;  // total operating cost [$]
    double      runtime_sec  = 0.0;
    std::string status_msg;

    std::vector<AcBusOPFResult>       ac_buses;
    std::vector<GeneratorOPFResult>   generators;
};

} // namespace hacdcdss::optimal_power_flow
