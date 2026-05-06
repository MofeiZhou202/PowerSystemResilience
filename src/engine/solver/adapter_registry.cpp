#include "hacdcpf/engine/solver/adapter_registry.hpp"

namespace hacdcpf::engine {

void AdapterRegistry::register_adapter(const SolverAdapterPtr& adapter) {
  if (!adapter) {
    return;
  }
  adapters_.push_back(adapter);
}

std::vector<SolverAdapterPtr> AdapterRegistry::adapters_for(ProblemClass cls) const {
  std::vector<SolverAdapterPtr> out;
  for (const auto& a : adapters_) {
    if (a && a->supports(cls)) {
      out.push_back(a);
    }
  }
  return out;
}

SolverAdapterPtr AdapterRegistry::first_for(ProblemClass cls) const {
  for (const auto& a : adapters_) {
    if (a && a->supports(cls)) {
      return a;
    }
  }
  return nullptr;
}

SolverAdapterPtr AdapterRegistry::find_by_name(const std::string& adapter_name) const {
  for (const auto& a : adapters_) {
    if (a && a->name() == adapter_name) {
      return a;
    }
  }
  return nullptr;
}

}  // namespace hacdcpf::engine
