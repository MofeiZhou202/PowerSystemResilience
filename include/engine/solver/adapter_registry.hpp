#pragma once

#include <memory>
#include <string>
#include <vector>

#include "hacdcpf/engine/solver/solver_adapter.hpp"

namespace hacdcpf::engine {

class AdapterRegistry {
 public:
  void register_adapter(const SolverAdapterPtr& adapter);

  std::vector<SolverAdapterPtr> adapters_for(ProblemClass cls) const;

  SolverAdapterPtr first_for(ProblemClass cls) const;
  SolverAdapterPtr find_by_name(const std::string& adapter_name) const;

 private:
  std::vector<SolverAdapterPtr> adapters_;
};

}  // namespace hacdcpf::engine
