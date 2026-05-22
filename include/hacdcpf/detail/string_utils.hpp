#pragma once

/// detail/string_utils.hpp — Internal string utility helpers.

#include <algorithm>
#include <cctype>
#include <string>

namespace hacdcpf::detail {

/// Remove leading/trailing whitespace in-place and return a copy.
inline std::string trim(const std::string& s) {
  const auto b = s.find_first_not_of(" \t\r\n");
  if (b == std::string::npos) return {};
  const auto e = s.find_last_not_of(" \t\r\n");
  return s.substr(b, e - b + 1);
}

}  // namespace hacdcpf::detail

// Pull trim into the anonymous / global detail namespace used by io cpp files.
using hacdcpf::detail::trim;
