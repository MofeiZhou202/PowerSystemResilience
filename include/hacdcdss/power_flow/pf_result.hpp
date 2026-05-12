#pragma once
// hacdcdss/power_flow/pf_result.hpp
//
// Module: power_flow
//
// Result types returned by PFSolver after a power-flow run.

#include <cstdint>
#include <string>
#include <vector>

namespace hacdcdss::power_flow {

// ── Per-bus AC result ─────────────────────────────────────────────────────────
struct AcBusResult {
    double v_pu    = 1.0;  // voltage magnitude [p.u.]
    double theta_rad = 0.0; // voltage angle [rad]
    double p_inj_mw  = 0.0; // net active power injection [MW]
    double q_inj_mvar = 0.0; // net reactive power injection [MVAr]
};

// ── Per-bus DC result ─────────────────────────────────────────────────────────
struct DcBusResult {
    double v_pu   = 1.0;   // DC bus voltage [p.u.]
    double p_inj_mw = 0.0; // net DC power injection [MW]
};

// ── Per-branch AC result ──────────────────────────────────────────────────────
struct AcBranchResult {
    double p_from_mw   = 0.0;  // active power at from end [MW]
    double q_from_mvar = 0.0;  // reactive power at from end [MVAr]
    double p_to_mw     = 0.0;  // active power at to end [MW]
    double q_to_mvar   = 0.0;  // reactive power at to end [MVAr]
    double loading_pct = 0.0;  // thermal loading [%] (0 if rate_mva == 0)
};

// ── Per-cable DC result ───────────────────────────────────────────────────────
struct DcCableResult {
    double p_from_mw = 0.0;  // power at from end [MW]
    double p_to_mw   = 0.0;  // power at to end (= p_from − losses) [MW]
};

// ── Full power-flow result ────────────────────────────────────────────────────
struct PFResult {
    bool   converged    = false;
    int    iterations   = 0;
    double max_mismatch = 0.0;  // largest bus-power mismatch [p.u. on base_mva]
    std::string status_msg;

    std::vector<AcBusResult>    ac_buses;
    std::vector<DcBusResult>    dc_buses;
    std::vector<AcBranchResult> ac_branches;
    std::vector<DcCableResult>  dc_cables;
};

} // namespace hacdcdss::power_flow
