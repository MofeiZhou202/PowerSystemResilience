/// test_adapter_registry.cpp
/// Tests for the AdapterRegistry.
#include <catch2/catch_test_macros.hpp>

#include "mipsolvers/engine/solver/adapter_registry.hpp"
#include "mipsolvers/engine/solver/native/native_adapters.hpp"
#include "mipsolvers/engine/problem_types.hpp"
#include "mipsolvers/engine/api/solver.hpp"

using namespace mipsolvers::engine;

TEST_CASE("AdapterRegistry: empty registry has no adapters", "[registry]") {
  AdapterRegistry reg;
  CHECK(reg.adapters_for(ProblemClass::LP).empty());
  CHECK(reg.adapters_for(ProblemClass::MILP).empty());
  CHECK(reg.first_for(ProblemClass::LP) == nullptr);
}

TEST_CASE("AdapterRegistry: native adapters registered and found", "[registry]") {
  AdapterRegistry reg;

  // Register native adapters individually.
  reg.register_adapter(std::make_shared<NativeBranchAndCutAdapter>());
  reg.register_adapter(std::make_shared<NativeLinearAdapter>());

  // LE solver should be findable.
  auto le = reg.adapters_for(ProblemClass::LE);
  CHECK_FALSE(le.empty());

  // MILP solver (B&C) should be findable.
  auto milp = reg.adapters_for(ProblemClass::MILP);
  CHECK_FALSE(milp.empty());
}

TEST_CASE("AdapterRegistry: find_by_name returns correct adapter", "[registry]") {
  AdapterRegistry reg;
  auto bc = std::make_shared<NativeBranchAndCutAdapter>();
  reg.register_adapter(bc);

  auto found = reg.find_by_name(bc->name());
  REQUIRE(found != nullptr);
  CHECK(found->name() == bc->name());
}

TEST_CASE("AdapterRegistry: unknown name returns nullptr", "[registry]") {
  AdapterRegistry reg;
  auto a = reg.find_by_name("non_existent_adapter_xyz");
  CHECK(a == nullptr);
}

TEST_CASE("SolverEngine: list_solvers returns names after registration", "[registry][engine]") {
  SolverEngine eng(false);
  CHECK(eng.list_solvers(ProblemClass::LP).empty());

  eng.register_default_adapters();
  CHECK_FALSE(eng.list_solvers(ProblemClass::LP).empty());
  CHECK_FALSE(eng.list_solvers(ProblemClass::MILP).empty());
}

