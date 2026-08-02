#pragma once

// ============================================================================
// hacdcpf/model/effective_capacity.hpp
//
// Single source of truth for "effective" power / capacity / demand values.
//
// Across the codebase, the same component fields (p_mw, pmax_mw, scaling,
// p_rated_mw, in_service, ...) have historically been combined in subtly
// different ways depending on the call site (OPF vs. reliability vs.
// resilience vs. time-series).  That divergence has produced two recurring
// classes of bug:
//
//   1. scaling=0 silently treated as full output  (reliability / resilience)
//   2. pmax_mw==0 interpreted inconsistently instead of using the declared
//      current/nameplate fallback
//
// Centralising the rules here lets every analysis module pull from one
// canonical definition, so the *physical contract* that any one module
// claims to solve actually matches what every other module assumes.
//
// All helpers are header-only inline so they impose zero ABI / link cost
// and can be used freely from constexpr-eligible contexts.
//
// Conventions used below:
//   - "effective_p_mw"        : signed real-power injection at the bus right
//                               now, taking scaling/in_service into account.
//                               Loads return *positive demand* (not injection).
//   - "effective_capacity_mw" : largest |P| the unit can supply (>=0).  For
//                               non-dispatchable / fixed-injection units this
//                               equals |effective_p_mw|.  For dispatchable
//                               units it is the pmax_mw bound multiplied by the
//                               same availability scaling used by dispatch.
//   - For out-of-service units both helpers return 0.
//   - A non-positive scaling collapses the fixed-injection contribution to 0,
//     so callers can use a single helper instead of duplicating the guard.
// ============================================================================

#include <algorithm>
#include <cmath>

#include "hacdcpf/model/ac_components.hpp"
#include "hacdcpf/model/dc_components.hpp"

namespace hacdcpf::model {

// ---------------------------------------------------------------------------
// Scaling sanitisation
// ---------------------------------------------------------------------------
// `scaling` is documented as a fixed-injection multiplier in [0, +inf).
// Negative or NaN values are treated as 0 so that bad data degrades safely
// (the unit acts as out-of-service) rather than corrupting downstream LP / MIP
// inputs with signed or infinite coefficients.
inline double sanitize_scaling(double s) {
  return (std::isfinite(s) && s > 0.0) ? s : 0.0;
}

// ---------------------------------------------------------------------------
// Loads (AC)
// ---------------------------------------------------------------------------
// Returns demand in MW (positive = consumed power).  Out-of-service loads
// return 0.  scaling applied; negative scaling is clamped to 0.
inline double effective_load_p_mw(const Load& ld) {
  if (!ld.in_service) return 0.0;
  return ld.p_mw * sanitize_scaling(ld.scaling);
}

inline double effective_load_p_mw(const DCLoad& ld) {
  if (!ld.in_service) return 0.0;
  return ld.p_mw * sanitize_scaling(ld.scaling);
}

// ---------------------------------------------------------------------------
// Static generators (fixed-injection on AC bus)
// ---------------------------------------------------------------------------
// In OPF static generators contribute (p_mw * scaling) as a *fixed* injection
// (they are not dispatch variables). The reliability / resilience modules
// must use the same value to avoid claiming more capacity than the physical
// model actually delivers.
inline double effective_p_mw(const StaticGenerator& sg) {
  if (!sg.in_service) return 0.0;
  return sg.p_mw * sanitize_scaling(sg.scaling);
}

// For connectivity-only screening we want the upper envelope a fixed-injection
// unit can provide.  The semantics differ by controllability:
//
//   controllable=true  — unit can dispatch up to pmax_mw (or p_rated_mw as
//                        fallback), so the full nameplate (times scaling) is the
//                        recoverable capacity.
//   controllable=false — unit has a fixed operating point; it can only deliver
//                        exactly p_mw * scaling.  Using pmax_mw here would
//                        overstate recoverability: a unit with p_mw=0,
//                        pmax_mw>0, controllable=false cannot be re-dispatched
//                        to provide contingency support.
inline double effective_capacity_mw(const StaticGenerator& sg) {
  if (!sg.in_service) return 0.0;
  const double s = sanitize_scaling(sg.scaling);
  if (s == 0.0) return 0.0;
  if (!sg.controllable) {
    // Fixed-injection: capacity equals the current effective injection.
    return std::max(0.0, sg.p_mw * s);
  }
  // Dispatchable: capacity is the nameplate upper bound.
  double cap = (sg.pmax_mw > 0.0) ? sg.pmax_mw
              : (sg.p_rated_mw > 0.0 ? sg.p_rated_mw : sg.p_mw);
  return std::max(0.0, cap * s);
}

// ---------------------------------------------------------------------------
// DC static generators (fixed-injection on DC bus)
// ---------------------------------------------------------------------------
inline double effective_p_mw(const StaticGeneratorDC& g) {
  if (!g.in_service) return 0.0;
  return g.p_set_mw * sanitize_scaling(g.scaling);
}

inline double effective_capacity_mw(const StaticGeneratorDC& g) {
  if (!g.in_service) return 0.0;
  const double s = sanitize_scaling(g.scaling);
  if (s == 0.0) return 0.0;
  if (!g.controllable) {
    return std::max(0.0, g.p_set_mw * s);
  }
  double cap = (g.pmax_mw > 0.0) ? g.pmax_mw : g.p_set_mw;
  return std::max(0.0, cap * s);
}

// ---------------------------------------------------------------------------
// Dispatchable AC generators
// ---------------------------------------------------------------------------
// Synchronous generators have no `scaling` field — capacity is bounded by
// pmax_mw directly, falling back to the current operating point.
inline double effective_capacity_mw(const Generator& g) {
  if (!g.in_service) return 0.0;
  if (g.pmax_mw > 0.0) return g.pmax_mw;
  return std::max(0.0, g.pg_mw);
}

// ---------------------------------------------------------------------------
// Storage (dispatchable; capacity = max discharge power)
// ---------------------------------------------------------------------------
inline double effective_capacity_mw(const Storage& st) {
  if (!st.in_service) return 0.0;
  if (st.pmax_mw > 0.0) return st.pmax_mw;
  return std::max(0.0, st.p_rated_mw);
}

}  // namespace hacdcpf::model
