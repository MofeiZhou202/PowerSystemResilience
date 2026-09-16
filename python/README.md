# HySim Python SDK

This package is the Python boundary for the HySim-XJTU-HRPES C++ runtime. It
uses the production session routes exposed by `run_gui_server`; it does not
duplicate solver or projection logic in Python.

```bash
python3 -m pip install -e python
./build/macos-release/tests/run_gui_server --port 8088 --data-dir data
```

```python
from hysim import HySimV1Client, PowerFlowOptions, PowerFlowRequest

client = HySimV1Client("http://127.0.0.1:8088")
session = client.create_session(case="ieee14_acdc")
job = session.power_flow(
    PowerFlowRequest(options=PowerFlowOptions(max_iter=100, tol=1e-8))
)
result = job.wait_result().require_usable()
print(result.summary())

# Typed, bounded reads for large models.
for page in session.topology_pages(lod=2, page_size=5000):
    print(page.offset, page.returned_nodes, page.next_offset)

local = session.subgraph_view(page.bus_refs[0], depth=2)
frame = job.frame_chunk(step=0, domain="ac", limit=500)
violations = job.violation_chunk(step=0, limit=50)
```

`HySimV1Client` is the preferred interface for new automation: each session is
isolated, model mutations use ETags, and analyses run as revision-bound jobs.
`TopologyChunk`, `SubgraphView`, `ResultFrameChunk`, and `ViolationChunk` validate
the scalable read contracts. Raw `topology()`, `subgraph()`, `frame()`, and
`violations()` methods remain available for forward-compatible JSON access.
`HySimClient` remains available for the legacy GUI-compatible process-global
routes. See `docs/python_api.md` for architecture and AI tool policy.

## Comprehensive facade

`HySim` composes one typed, namespaced surface over every production analysis
family (18 families, 69 catalogued analyses). Each method resolves its route
through a single `ANALYSIS_CATALOG` and returns an honest `AnalysisResult`.

```python
from hysim import HySim, MarketClearingRequest, ShortCircuitRequest, UnitCommitmentRequest

sim = HySim("http://127.0.0.1:8088")
sim.load_builtin("ieee14_acdc")

pf = sim.pf.run()
uc = sim.time_series.unit_commitment(UnitCommitmentRequest(num_steps=24))
mkt = sim.market.clearing(MarketClearingRequest(num_steps=24, reserve_fraction=0.06))
sc = sim.short_circuit.detailed(ShortCircuitRequest(fault_bus_ids=[3]))
sim.model_io.export("matpower")
mkt.require_usable()  # honest gate: raises on failed/stale/invalid results
```

Families include `pf`, `opf`, `reactive_power`, `short_circuit`, `harmonics`,
`dynamics`, `time_series`, `carbon`, `market`, `reliability`, `resilience`,
`reconfiguration`, `hosting_capacity`, `integrated_energy`, `ev_traffic`,
`planning`, `scenario`, and `model_io`. The catalog carries an `AnalysisEffect`
(`read`/`analyze`/`modify`) plus a narrower `mutates_model` flag per analysis so
tool policies and audit hooks can gate a call without parsing its payload.
`HySimToolRegistry.register_family_tools()` exposes one effect-gated AI tool per
analysis. The design and coverage roadmap are in
`docs/reference/python_comprehensive_api_design.md`.
