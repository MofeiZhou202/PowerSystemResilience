// Bridge header: re-exports hacdcpf PowerFlowResult into mipsolvers::engine.
#pragma once

#include "hacdcpf/model/results.hpp"

namespace mipsolvers::engine {

using hacdcpf::PowerFlowResult;

}  // namespace mipsolvers::engine
