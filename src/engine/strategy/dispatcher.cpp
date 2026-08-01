#include "mipsolvers/engine/strategy/dispatcher.hpp"

#include <array>
#include <algorithm>
#include <unordered_set>

namespace mipsolvers::engine::strategy {
namespace {

SolveResult unsupported(const std::string& reason) {
  SolveResult out;
  out.stats.success = false;
  out.stats.status = reason;
  out.stats.solver_name = "StrategyDispatcher";
  return out;
}

SolveResult solve_with_adapter(const SolverAdapterPtr& adapter,
                               ProblemClass cls,
                               const api::ProblemVariant& problem) {
  switch (cls) {
    case ProblemClass::LE: {
      const auto* p = std::get_if<SparseLinSys>(&problem);
      return p ? adapter->solve_le(*p)
               : unsupported("Problem payload mismatch for LE dispatch");
    }
    case ProblemClass::NLE: {
      const auto* p = std::get_if<NonlinearSystem>(&problem);
      return p ? adapter->solve_nle(*p)
               : unsupported("Problem payload mismatch for NLE dispatch");
    }
    case ProblemClass::LP: {
      const auto* p = std::get_if<LPModel>(&problem);
      return p ? adapter->solve_lp(*p)
               : unsupported("Problem payload mismatch for LP dispatch");
    }
    case ProblemClass::QP: {
      const auto* p = std::get_if<QPModel>(&problem);
      return p ? adapter->solve_qp(*p)
               : unsupported("Problem payload mismatch for QP dispatch");
    }
    case ProblemClass::NLP: {
      const auto* p = std::get_if<NLPModel>(&problem);
      return p ? adapter->solve_nlp(*p)
               : unsupported("Problem payload mismatch for NLP dispatch");
    }
    case ProblemClass::MILP: {
      const auto* p = std::get_if<MIPModel>(&problem);
      return p ? adapter->solve_milp(*p)
               : unsupported("Problem payload mismatch for MILP dispatch");
    }
    case ProblemClass::MINLP: {
      const auto* p = std::get_if<MINLPModel>(&problem);
      return p ? adapter->solve_minlp(*p)
               : unsupported("Problem payload mismatch for MINLP dispatch");
    }
    case ProblemClass::CONIC: {
      const auto* p = std::get_if<ConicModel>(&problem);
      return p ? adapter->solve_conic(*p)
               : unsupported("Problem payload mismatch for CONIC dispatch");
    }
  }
  return unsupported("Unsupported problem class dispatch");
}

std::vector<std::string> default_priority_for(ProblemClass cls) {
  switch (cls) {
    case ProblemClass::LE:
      return {"NativeLinear"};
    case ProblemClass::NLE:
      return {"NativeNewton"};
    case ProblemClass::LP:
      return {"NativeIPMLP", "NativePDLP", "NativeLCQP", "HiGHS"};
    case ProblemClass::QP:
      return {"NativeLCQP"};
    case ProblemClass::NLP:
      return {"Ipopt", "NativeIPM", "NativeNLP"};
    case ProblemClass::MILP:
      return {"StrictHiGHS", "HiGHS", "NativeBranchAndCut"};
    case ProblemClass::MINLP:
      return {"Scip", "NativeBranchAndCut"};
    case ProblemClass::CONIC:
      return {"NativeConicIPM"};
  }
  return {};
}

bool is_external_adapter_name(const std::string& name) {
  return name == "HiGHS" || name == "Ipopt" || name == "Scip" || name == "Gurobi";
}

bool is_native_adapter_name(const std::string& name) {
  return name == "StrictHiGHS" || name.rfind("Native", 0) == 0;
}

StrategyPolicy effective_policy(
    ProblemClass cls,
    StrategyPolicy default_policy,
    const std::map<ProblemClass, StrategyPolicy>& class_policy) {
  auto it = class_policy.find(cls);
  if (it != class_policy.end()) {
    return it->second;
  }
  return default_policy;
}

void apply_policy_ordering(std::vector<std::string>& names, StrategyPolicy policy) {
  if (policy == StrategyPolicy::Auto) {
    return;
  }

  std::stable_sort(names.begin(), names.end(), [&](const std::string& a, const std::string& b) {
    const bool a_external = is_external_adapter_name(a);
    const bool b_external = is_external_adapter_name(b);
    const bool a_native = is_native_adapter_name(a);
    const bool b_native = is_native_adapter_name(b);

    const int a_rank = (policy == StrategyPolicy::ExternalFirst)
                           ? (a_external ? 0 : (a_native ? 1 : 2))
                           : (a_native ? 0 : (a_external ? 1 : 2));
    const int b_rank = (policy == StrategyPolicy::ExternalFirst)
                           ? (b_external ? 0 : (b_native ? 1 : 2))
                           : (b_native ? 0 : (b_external ? 1 : 2));
    return a_rank < b_rank;
  });
}

}  // namespace

void StrategyDispatcher::set_solver_preference(ProblemClass cls,
                                               const std::string& adapter_name) {
  class_preferences_[cls] = adapter_name;
}

std::vector<std::string> StrategyDispatcher::list_solvers(const AdapterRegistry& registry,
                                                          ProblemClass cls) const {
  std::vector<std::string> out;
  for (const auto& adapter : candidate_adapters(registry,
                                                cls,
                                                "",
                                                StrategyPolicy::Auto,
                                                {})) {
    out.push_back(adapter->name());
  }
  return out;
}

std::vector<SolverAdapterPtr> StrategyDispatcher::candidate_adapters(
    const AdapterRegistry& registry,
    ProblemClass cls,
    const std::string& preferred_solver,
    StrategyPolicy default_policy,
    const std::map<ProblemClass, StrategyPolicy>& class_policy) const {
  std::vector<SolverAdapterPtr> ordered;
  std::unordered_set<std::string> seen;

  auto add_if_supported = [&](const std::string& name) {
    if (name.empty()) {
      return;
    }
    auto adapter = registry.find_by_name(name);
    if (!adapter || !adapter->supports(cls)) {
      return;
    }
    if (seen.insert(adapter->name()).second) {
      ordered.push_back(std::move(adapter));
    }
  };

  add_if_supported(preferred_solver);

  auto it = class_preferences_.find(cls);
  if (it != class_preferences_.end()) {
    add_if_supported(it->second);
  }

  auto candidate_names = default_priority_for(cls);
  apply_policy_ordering(candidate_names, effective_policy(cls, default_policy, class_policy));
  for (const auto& name : candidate_names) {
    add_if_supported(name);
  }

  for (const auto& adapter : registry.adapters_for(cls)) {
    if (adapter && seen.insert(adapter->name()).second) {
      ordered.push_back(adapter);
    }
  }

  return ordered;
}

SolveResult StrategyDispatcher::solve(const AdapterRegistry& registry,
                                      const api::ProblemVariant& problem,
                                      const std::string& preferred_solver,
                                      bool allow_fallback,
                                      StrategyPolicy default_policy,
                                      const std::map<ProblemClass, StrategyPolicy>& class_policy) const {
  const ProblemClass cls = api::problem_class(problem);
  const auto candidates = candidate_adapters(
      registry, cls, preferred_solver, default_policy, class_policy);

  if (candidates.empty()) {
    return unsupported("No adapter registered for class " + api::problem_class_name(cls));
  }

  SolveResult last_failure;
  bool have_failure = false;
  for (const auto& adapter : candidates) {
    SolveResult out = solve_with_adapter(adapter, cls, problem);
    if (out.stats.success) {
      return out;
    }
    last_failure = out;
    have_failure = true;
    if (!allow_fallback) {
      return out;
    }
  }

  if (have_failure) {
    return last_failure;
  }
  return unsupported("No viable adapter for class " + api::problem_class_name(cls));
}

}  // namespace mipsolvers::engine::strategy
