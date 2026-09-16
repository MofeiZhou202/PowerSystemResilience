"""Comprehensive facade: one client, many typed analysis families.

Start the server first, then run this script:

    ./build/macos-release/tests/run_gui_server --port 8088 --data-dir data
    python3 python/examples/comprehensive_workflow.py
"""

from hysim import (
    HySim,
    MarketClearingRequest,
    ReliabilityRequest,
    RpoRequest,
    TransientRequest,
    UnitCommitmentRequest,
)


sim = HySim("http://127.0.0.1:8088")
sim.load_builtin("ieee14_acdc")

# Each family method resolves its production route through the analysis catalog
# and returns an honest AnalysisResult (inspect scientific_status before use).
results = {
    "power_flow": sim.pf.run(),
    "optimal_power_flow": sim.opf.run(),
    "unit_commitment": sim.time_series.unit_commitment(
        UnitCommitmentRequest(num_steps=24)
    ),
    "market_clearing": sim.market.clearing(
        MarketClearingRequest(num_steps=24, reserve_fraction=0.06)
    ),
    "reliability": sim.reliability.nonsequential(
        ReliabilityRequest(max_iterations=2000, seed=7)
    ),
    "transient": sim.dynamics.transient(TransientRequest(t_end_s=5.0, dt_s=0.005)),
    "reactive_power": sim.reactive_power.optimize(RpoRequest(objective="combined")),
}

for name, result in results.items():
    print(name, result.scientific_status, sorted(result.summary()["result_sizes"]))

# The catalog is the single source of truth for what exists and how risky it is.
print("analyses:", len(sim.catalog))
market_category = sim.catalog.require("market_clearing").category
print("market family:", [s.name for s in sim.catalog.by_category(market_category)])
