/// @file bc_minlp_legacy.hpp
/// @brief Legacy MINLP branch-and-cut entry point.

#pragma once

#include "mipsolvers/engine/branch_and_cut.hpp"

namespace mipsolvers::engine::detail {

BCResult branch_and_cut_nlp(const MINLPModel& prob, const BCOptions& opt);

}  // namespace mipsolvers::engine::detail
