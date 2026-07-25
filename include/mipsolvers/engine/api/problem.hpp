#pragma once

#include <string>
#include <type_traits>
#include <variant>

#include "mipsolvers/engine/problem_types.hpp"

namespace mipsolvers::engine::api {

using ProblemVariant = std::variant<SparseLinSys,
                                    NonlinearSystem,
                                    LPModel,
                                    QPModel,
                                    NLPModel,
                                    MIPModel,
                                    MINLPModel,
                                    ConicModel>;

inline ProblemClass problem_class(const ProblemVariant& problem) {
  return std::visit(
      [](const auto& p) -> ProblemClass {
        using T = std::decay_t<decltype(p)>;
        if constexpr (std::is_same_v<T, SparseLinSys>) {
          return ProblemClass::LE;
        } else if constexpr (std::is_same_v<T, NonlinearSystem>) {
          return ProblemClass::NLE;
        } else if constexpr (std::is_same_v<T, LPModel>) {
          return ProblemClass::LP;
        } else if constexpr (std::is_same_v<T, QPModel>) {
          return ProblemClass::QP;
        } else if constexpr (std::is_same_v<T, NLPModel>) {
          return ProblemClass::NLP;
        } else if constexpr (std::is_same_v<T, MIPModel>) {
          return ProblemClass::MILP;
        } else if constexpr (std::is_same_v<T, MINLPModel>) {
          return ProblemClass::MINLP;
        }
        return ProblemClass::CONIC;
      },
      problem);
}

inline std::string problem_class_name(ProblemClass cls) {
  switch (cls) {
    case ProblemClass::LE:
      return "LE";
    case ProblemClass::NLE:
      return "NLE";
    case ProblemClass::LP:
      return "LP";
    case ProblemClass::QP:
      return "QP";
    case ProblemClass::NLP:
      return "NLP";
    case ProblemClass::MILP:
      return "MILP";
    case ProblemClass::MINLP:
      return "MINLP";
    case ProblemClass::CONIC:
      return "CONIC";
  }
  return "Unknown";
}

}  // namespace mipsolvers::engine::api
