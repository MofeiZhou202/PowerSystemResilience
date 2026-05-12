#pragma once

// Excel I/O is disabled; this header is a stub.
// Define HACDCPF_ENABLE_EXCEL to enable real Excel support.
#ifndef HACDCPF_NO_EXCEL
#  define HACDCPF_NO_EXCEL
#endif

#include <stdexcept>
#include <string>

#include "hacdcpf/model/system.hpp"

namespace hacdcpf::io {

#ifdef HACDCPF_ENABLE_EXCEL

HybridPowerSystem load_excel(const std::string& path);
void save_excel(const HybridPowerSystem& sys, const std::string& path);

#else

inline HybridPowerSystem load_excel(const std::string&) {
  throw std::runtime_error("Excel I/O not compiled (HACDCPF_ENABLE_EXCEL not set)");
}

inline void save_excel(const HybridPowerSystem&, const std::string&) {
  throw std::runtime_error("Excel I/O not compiled (HACDCPF_ENABLE_EXCEL not set)");
}

#endif  // HACDCPF_ENABLE_EXCEL

}  // namespace hacdcpf::io
