#pragma once
// Compatibility shim: hacdcpf/model/system.hpp
// In this repo, the system types (HybridPowerSystem, ACSystem, DCSystem) live in
// hacdcpf/model/hybrid_power_system.hpp.  Code that includes the planning-repo
// path continues to work unchanged.
#include "hacdcpf/model/hybrid_power_system.hpp"
