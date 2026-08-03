/// @file bc_strict_scuc_cuts.hpp
/// @brief SCUC-specific dynamic node-cut separation for the strict HiGHS path.
///
/// Emits reserve-cover, ramping-perspective and minimum up/down conflict cuts
/// from the UC generation hint at fractional nodes.  Only compiled when the
/// vendored HiGHS MIP library is available, matching the strict-HiGHS node
/// lifecycle that consumes these cuts.

#pragma once

#ifdef MIPSOLVERS_HAVE_HIGHS_LIB

#include <vector>

#include "mipsolvers/engine/bc/ml_hooks.hpp"    // BCCallbacks, BCDynamicNodeCut(Context)
#include "mipsolvers/engine/bc/options.hpp"     // BCOptions
#include "mipsolvers/engine/problem_types.hpp"  // MIPModel, LPModel

namespace mipsolvers::engine::detail {

/// User-data bridge handed to the strict-HiGHS node cut callback.
struct StrictHighsNodeCutCallbackBridge {
  const BCCallbacks* callbacks{nullptr};
  const LPModel* original_lp{nullptr};
};

bool strict_scuc_dynamic_cuts_enabled(const MIPModel::UCGenHint& uc);

void append_strict_scuc_dynamic_cuts(const MIPModel::UCGenHint& uc,
                                     const BCOptions& opt,
                                     const BCDynamicNodeCutContext& ctx,
                                     const LPModel* original_lp,
                                     std::vector<BCDynamicNodeCut>& rows);

}  // namespace mipsolvers::engine::detail

#endif  // MIPSOLVERS_HAVE_HIGHS_LIB
