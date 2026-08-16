#pragma once

#include <string>
#include <string_view>
#include <vector>

namespace hacdcpf::model {

enum class RuntimeFidelity {
  Exact,
  Equivalent,
  Aggregated,
  BoundaryInjection,
  Projected,
  Unsupported,
  NotApplicable,
};

struct WorkflowFidelity {
  std::string workflow;
  RuntimeFidelity fidelity{RuntimeFidelity::Unsupported};
  std::string equations_or_role;
};

struct ComponentRuntimeSemantics {
  std::string component_type;
  std::string collection_path;
  std::string identity_contract;
  std::string terminal_contract;
  std::string unit_contract;
  std::string sign_contract;
  std::string parameter_owner;
  std::string result_contract;
  std::vector<WorkflowFidelity> workflows;
  std::string fidelity_boundary;
};

struct ModuleDataContract {
  std::string source_module;
  std::string authoritative_input;
  std::string execution_view;
  std::string public_output;
  std::string identity_space;
  std::string unit_contract;
  std::string fidelity_boundary;
};

const std::vector<ComponentRuntimeSemantics>& component_runtime_semantics();
const std::vector<ModuleDataContract>& module_data_contracts();

const ComponentRuntimeSemantics* find_component_runtime_semantics(
    std::string_view collection_path);
const ModuleDataContract* find_module_data_contract(
    std::string_view source_module);

std::string to_string(RuntimeFidelity fidelity);

}  // namespace hacdcpf::model
