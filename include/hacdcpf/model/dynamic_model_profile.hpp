#pragma once

#include <map>
#include <string>
#include <vector>

namespace hacdcpf {

struct DynamicModelComponentProfile {
  std::string type;
  std::string model;
  std::string standard;
  std::string parameter_set;
  std::map<std::string, double> parameters;
};

struct DynamicModelProfile {
  std::string standard;
  std::string model_name;
  std::string parameter_set;
  std::string source_id;
  std::string notes;
  std::vector<DynamicModelComponentProfile> components;
  std::map<std::string, double> parameters;

  [[nodiscard]] bool empty() const noexcept {
    return standard.empty() && model_name.empty() && parameter_set.empty() &&
           source_id.empty() && notes.empty() && components.empty() &&
           parameters.empty();
  }
};

}  // namespace hacdcpf
