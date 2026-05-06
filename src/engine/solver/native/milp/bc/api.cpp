#include "hacdcpf/engine/bc/api.hpp"

#include "hacdcpf/engine/detail/bc/legacy_bridge.hpp"

namespace hacdcpf::engine {

BCResult solve_milp_bc(const MIPModel& prob, const BCOptions& opt) {
  return detail::solve_milp_bc_legacy_core(prob, opt);
}

BCResult solve_minlp_bc(const MINLPModel& prob, const BCOptions& opt) {
  return detail::solve_minlp_bc_legacy_core(prob, opt);
}

BCResult solve_milp_bc(const MIPModel&    prob,
                       const BCOptions&   opt,
                       const BCWarmStart& ws,
                       const BCCallbacks& cbs) {
  return detail::solve_milp_bc_legacy_core(prob, opt, ws, cbs);
}

BCResult solve_minlp_bc(const MINLPModel&  prob,
                        const BCOptions&   opt,
                        const BCWarmStart& ws,
                        const BCCallbacks& cbs) {
  return detail::solve_minlp_bc_legacy_core(prob, opt, ws, cbs);
}

}  // namespace hacdcpf::engine