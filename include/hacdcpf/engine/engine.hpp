#pragma once
/// hacdcpf/engine/engine.hpp
/// ─────────────────────────────────────────────────────────────────────────
/// Single expansion point for the hacdcpf::engine namespace.
///
/// To add new MIPSolvers types to hacdcpf::engine, add their include here.
/// All other engine/*.hpp files simply include this header — no per-type
/// maintenance required in those stubs.
/// ─────────────────────────────────────────────────────────────────────────
#include <mipsolvers/engine/engine.hpp>                             // problem_types, solver_adapter, adapter_registry
#include <mipsolvers/engine/branch_and_cut.hpp>                    // BCOptions, BCStats, BCResult, BCWarmStart, BranchingStrategy, NodeSelection, CutType
#include <mipsolvers/engine/solver/native/native_adapters.hpp>     // NativeBranchAndCutAdapter, NativeLinearAdapter, NativeNewtonAdapter, ...
#include <mipsolvers/engine/solver/external/adapters.hpp>          // HighsAdapter, IpoptAdapter, ScipAdapter, GurobiAdapter

namespace hacdcpf::engine {
  /// Forward the entire mipsolvers::engine namespace.
  /// Adding a type to mipsolvers::engine automatically makes it available
  /// here — no manual using-declaration needed.
  using namespace mipsolvers::engine;
}
