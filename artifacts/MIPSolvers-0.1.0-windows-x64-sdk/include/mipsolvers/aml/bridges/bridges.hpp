#pragma once

/// AML Bridges — reformulation helpers that introduce auxiliary variables
/// and constraints to make non-smooth or nonlinear expressions LP-compatible.
///
/// All bridges operate on an existing Model reference.  They return handles
/// to the auxiliary variable(s) or constraints for downstream use.

#include "mipsolvers/aml/bridges/abs_value_bridge.hpp"
#include "mipsolvers/aml/bridges/indicator_bridge.hpp"
#include "mipsolvers/aml/bridges/max_epigraph_bridge.hpp"
#include "mipsolvers/aml/bridges/piecewise_linear_bridge.hpp"
