// nonmonotone_linesearch.cpp
// The NonmonotoneLineSearch class is header-only (all methods are inline or
// templated).  This translation unit exists so that the class appears in the
// build system and can have explicit instantiations added later if needed.
#include "hacdcpf/power_flow/nonmonotone_linesearch.hpp"

// Explicit template instantiation for the most common evaluator signature
// (a plain function object returning double).  Additional instantiations can
// be added here without changing header files.
namespace hacdcpf::powerflow {
  // No non-inline symbols to define; the header is self-contained.
}  // namespace hacdcpf::powerflow
