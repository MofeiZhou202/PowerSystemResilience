#pragma once

#include <algorithm>
#include <cctype>
#include <string>

namespace mipsolvers {

/// Trim leading and trailing whitespace from a string.
inline std::string trim(const std::string& s) {
  size_t b = 0;
  while (b < s.size() && std::isspace(static_cast<unsigned char>(s[b]))) {
    ++b;
  }
  size_t e = s.size();
  while (e > b && std::isspace(static_cast<unsigned char>(s[e - 1]))) {
    --e;
  }
  return s.substr(b, e - b);
}

/// Convert a string to uppercase in-place and return it.
inline std::string to_upper(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char ch) {
    return static_cast<char>(std::toupper(ch));
  });
  return s;
}

}  // namespace mipsolvers
