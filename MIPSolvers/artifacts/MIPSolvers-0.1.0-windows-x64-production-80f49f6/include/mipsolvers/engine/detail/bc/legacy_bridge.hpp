#pragma once

#include "mipsolvers/engine/bc/api.hpp"

namespace mipsolvers::engine::detail {

BCResult solve_milp_bc_legacy_core(const MIPModel& prob, const BCOptions& opt);
BCResult solve_minlp_bc_legacy_core(const MINLPModel& prob, const BCOptions& opt);

BCResult solve_milp_bc_legacy_core(const MIPModel&    prob,
                                   const BCOptions&   opt,
                                   const BCWarmStart& ws,
                                   const BCCallbacks& cbs);

BCResult solve_minlp_bc_legacy_core(const MINLPModel&  prob,
                                    const BCOptions&   opt,
                                    const BCWarmStart& ws,
                                    const BCCallbacks& cbs);

}  // namespace mipsolvers::engine::detail