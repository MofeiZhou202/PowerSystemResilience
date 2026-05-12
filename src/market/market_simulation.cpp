#include "hacdcpf/market/market_simulation.hpp"

namespace hacdcpf::market {

MarketClearingOutput run_market_clearing(
    const hacdcpf::HybridPowerSystem& /*sys*/,
    const MarketConfig& /*cfg*/,
    const MarketProfiles& /*profiles*/) {
  return MarketClearingOutput{};
}

}  // namespace hacdcpf::market
