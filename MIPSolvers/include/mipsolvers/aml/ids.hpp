#pragma once

#include <cstdint>

namespace mipsolvers::aml {

using SetId   = int32_t;   ///< Set family index
using ParamId = int32_t;   ///< Parameter family index
using VarId   = int32_t;   ///< Column number in compiled matrix
using ConId   = int32_t;   ///< Row number in compiled matrix
using ExprId  = int32_t;   ///< Node index in the nonlinear expression arena

inline constexpr VarId  kInvalidVar  = -1;
inline constexpr ConId  kInvalidCon  = -1;
inline constexpr ExprId kInvalidExpr = -1;
inline constexpr SetId  kInvalidSet  = -1;

}  // namespace mipsolvers::aml
