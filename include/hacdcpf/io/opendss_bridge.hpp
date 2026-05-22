#pragma once

// OpenDSS bridge is disabled; this header is a stub.
// Define HACDCPF_ENABLE_OPENDSS to enable real OpenDSS support.
#ifndef HACDCPF_NO_OPENDSS
#  define HACDCPF_NO_OPENDSS
#endif

#include <stdexcept>
#include <string>

#include "hacdcpf/model/hybrid_power_system.hpp"

namespace hacdcpf::io {

#ifdef HACDCPF_ENABLE_OPENDSS

HybridPowerSystem load_opendss(const std::string& dss_path);
void save_opendss(const HybridPowerSystem& sys, const std::string& path);

#else

inline HybridPowerSystem load_opendss(const std::string&) {
  throw std::runtime_error("OpenDSS bridge not compiled (HACDCPF_ENABLE_OPENDSS not set)");
}

inline void save_opendss(const HybridPowerSystem&, const std::string&) {
  throw std::runtime_error("OpenDSS bridge not compiled (HACDCPF_ENABLE_OPENDSS not set)");
}

#endif  // HACDCPF_ENABLE_OPENDSS

}  // namespace hacdcpf::io
