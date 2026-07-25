#pragma once

#include <string>
#include <vector>

#include "mipsolvers/engine/problem_types.hpp"

namespace mipsolvers::engine {

struct ValidationReport {
  bool valid{true};
  std::vector<std::string> errors;
  std::vector<std::string> warnings;
};

ValidationReport validate(const SparseLinSys& model);
ValidationReport validate(const NonlinearSystem& model);
ValidationReport validate(const LPModel& model);
ValidationReport validate(const QPModel& model);
ValidationReport validate(const NLPModel& model);
ValidationReport validate(const MIPModel& model);
ValidationReport validate(const MINLPModel& model);
ValidationReport validate(const ConicModel& model);

}  // namespace mipsolvers::engine
