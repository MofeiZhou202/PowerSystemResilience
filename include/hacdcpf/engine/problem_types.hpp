#pragma once
// Forwarding header: hacdcpf::engine types → mipsolvers::engine
#include <mipsolvers/engine/problem_types.hpp>
#include <mipsolvers/engine/solver/solver_adapter.hpp>

namespace hacdcpf::engine {
  using mipsolvers::engine::ProblemClass;
  using mipsolvers::engine::VarType;
  using mipsolvers::engine::Sense;
  using mipsolvers::engine::SymOp;
  using mipsolvers::engine::SymExpr;
  using mipsolvers::engine::VariableMeta;
  using mipsolvers::engine::LPModel;
  using mipsolvers::engine::QPModel;
  using mipsolvers::engine::MIPModel;
  using mipsolvers::engine::NLPModel;
  using mipsolvers::engine::MINLPModel;
  using mipsolvers::engine::SolveStats;
  using mipsolvers::engine::SolveResult;
}
