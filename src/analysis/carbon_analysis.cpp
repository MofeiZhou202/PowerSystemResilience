#include "hacdcpf/analysis/carbon_analysis.hpp"

namespace hacdcpf::analysis {

CarbonAnalysisResult run_carbon_analysis(const HybridPowerSystem& /*sys*/,
                                          const CarbonAnalysisOptions& /*opt*/) {
  // Stub: carbon tracing not yet implemented.
  CarbonAnalysisResult r;
  r.tracing_verified = false;
  r.matrix_solved    = false;
  r.matrix_residual  = 0.0;
  return r;
}

}  // namespace hacdcpf::analysis
