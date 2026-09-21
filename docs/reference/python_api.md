# Python API and AI Enhancement Architecture

Updated: 2026-09-19

## Decision

Python is an orchestration and intelligence boundary, not a second simulation
kernel. The C++ rich model, validation, canonical projection, solvers, and
authored-space result attribution remain authoritative. The first implementation
is the dependency-free package under `python/`, which talks to the production
session routes in `tests/run_gui_server.cpp`.

```text
AI / optimization / data science code
        |
        +-- HySimV1ToolRegistry (schemas, effect policy, compact context)
        |
        +-- HySimV1Client (isolated sessions, ETags, async jobs)
        |
        +-- Transport protocol (HTTP now; in-process binding later)
        |
run_gui_server -> hacdcpf C++ facade -> rich/canonical projection -> solvers
```

This boundary avoids duplicating electrical semantics in Python and makes a
future in-process transport possible without changing experiment code.

## Package contract

| Layer | Public types | Responsibility |
|---|---|---|
| Runtime | `LocalHySimServer` | One isolated C++ process per experiment or AI worker. |
| Transport | `Transport`, `UrllibTransport` | Replaceable request execution; no unsafe automatic retry. |
| SDK v1 | `HySimV1Client`, `HySimV1Session`, `HySimJob` | Isolated model lifecycle, optimistic concurrency, asynchronous PF/OPF, typed topology and result chunks. |
| Resources | `TopologyChunk`, `SubgraphView`, `ResultFrameChunk`, `ViolationChunk` | Validate schemas/counts, expose stable bus references, and retain forward-compatible raw payloads. |
| SDK legacy | `HySimClient` | GUI-compatible global lifecycle, registered analyses, lazy frames. |
| Models | `BusRef`, `ComponentRef`, request dataclasses | Domain-qualified bus IDs and stable authored component IDs. |
| Results | `AnalysisResult` | Request/model provenance, scientific status, limitations, raw payload. |
| AI | `HySimV1ToolRegistry`, `ToolPolicy` | Job-oriented tool schemas, effect allowlist, explicit mutation approval. |

The SDK intentionally preserves unknown response fields. C++ modules evolve
quickly, so Python must not discard new diagnostics while waiting for a package
release. Common PF/OPF inputs are typed; specialized analyses accept their
native JSON payload through `run_analysis()`.

## Correctness rules

1. AI-facing bus references are `{domain, index}`. A bare integer is never a
   cross-domain bus identity.
2. `index` means the stable authored component ID. Python does not expose C++
   vector positions or graph indices as external identity.
3. A successful HTTP response is not automatically a valid engineering result.
   Consumers must inspect `scientific_status`, call `require_usable()`, and retain
   `model_scope`, `validity_flags`, fallback state, warnings, and limitations.
4. Each v1 analysis carries its job ID, session ID, immutable model revision,
   and the current revision used to determine staleness.
   The audit hook stores a request hash rather than logging the full system model.
5. Requests are not retried automatically because model changes and analyses are
   not idempotent under the current process-global session.

## AI tool safety

Edition capability admission and `ToolPolicy` are orthogonal gates. A connected
runtime first decides whether an analysis is known and enabled; only then does
`ToolPolicy` decide whether its `read`, `analyze`, `modify_model`, or `control`
effect is allowed and whether explicit approval is required. Expanding
`allowed_effects` cannot enable a route disabled by the edition, and an enabled
route still cannot bypass effect policy. Comprehensive family tools ultimately
call `HySimClient.execute()`, which revalidates the canonical analysis,
route/method, mutation metadata, and connected-edition capability before
transport. V1 job submission uses its own `/api/v1` capability gate.

The default policy permits `read` and `analyze`. `modify_model` and `control`
are disabled. Enabling a mutating effect is a deployment decision, and each call
still requires `approved=True`. This separates model reasoning from execution
and prevents an agent from silently replacing the active case.

Tool results are compact by default: scalar status and diagnostics are retained,
while large vectors are represented by their sizes. Direct SDK calls always
retain the full raw result for NumPy/pandas/PyTorch post-processing. Provider
adapters can consume `function_schemas()` or `openai_tools()`; provider-specific
dependencies do not belong in the core package.

## Edition discovery and availability

Python treats the process-global and isolated discovery domains independently:

- `HySimClient.edition_profile` fetches and caches `GET /api/edition`. The
  profile's `analysis_catalog` has exact schema
  `hacdcpf.edition-analysis-catalog.v1`, exact top-level fields `schema` and
  `entries`, and exact entry fields `name`, `route`, `method`, and `enabled`.
  The parser reconciles all 69 canonical names/routes/methods with the local
  `ANALYSIS_CATALOG`; missing, extra, duplicate, malformed, or contradictory
  data fails closed as `TransportError`.
- `HySimV1Client.capabilities()` fetches and caches the independent
  `GET /api/v1` document. It describes isolated asynchronous job types, not all
  retained legacy routes.

Canonical server IDs are the `AnalysisSpec.name` values. The legacy client
currently accepts only two convenience aliases: `resilience` resolves to
`distribution_resilience`, and `integrated_energy` resolves to `campus_ies`.
Aliases are resolved before capability admission and are not additional server
catalog entries. A name outside the SDK-known universe raises
`UnknownAnalysisError`; a known catalog entry whose `enabled` value is false
raises `AnalysisDisabledError`, both before analysis transport. A direct
low-level call cannot bypass the check because `HySimClient.execute()` performs
the same canonical catalog and edition validation.

## Runtime boundaries

`/api/v1` now owns independent in-memory sessions and a bounded asynchronous
worker pool. The GUI remains on the backward-compatible process-global routes;
the two state stores do not interact. Sessions and job results are not durable,
and v1 currently migrates only balanced aggregate PF/OPF. Deployments exposed
beyond a trusted workstation still require authentication, authorization,
request-size limits, and TLS at a gateway.

Large-model consumers should use `session.topology(lod=..., viewport=...)` and
`session.subgraph(domain, index, depth=...)` instead of reading the complete
model for visualization. After a job succeeds, `job.frame(step, domain=...,
indices=..., viewport=...)` returns a bounded result window and
`job.violations(step)` returns the worst voltage/loading excursions. These
methods preserve domain-qualified stable IDs and pass unknown fields through.

The recommended scalable access pattern is:

```python
for page in session.topology_pages(lod=2, page_size=5000):
    consume(page.nodes)

focus = session.subgraph_view(BusRef(BusDomain.AC, 100), depth=2)
result = job.wait_result().require_usable()
frame = job.frame_chunk(0, domain="ac", limit=500)
worst = job.violation_chunk(0, limit=50)
```

`topology_pages()` and `frame_pages()` reject non-advancing pagination tokens
instead of looping forever. `iter_topology_nodes()` and `iter_frame_nodes()`
flatten node pages for streaming consumers. Direct raw methods remain available
where a caller must consume newly added server fields before the typed SDK is
updated. Client-side validation rejects invalid LOD, domain, viewport, paging,
stable-index, and engineering-limit inputs before any network request.

The v1 AI registry exposes bounded read tools for topology pages, subgraphs,
job frames, violations, and compact job lists. Tool-level limits are deliberately
smaller than direct SDK limits so an agent cannot accidentally pull tens of
thousands of rows into one context window.

`HySimV1Client(audit_hook=...)` now emits the same `ApiCallEvent` contract as
the legacy client. Each event contains a generated request ID, route, HTTP
status, elapsed time, resolved model revision, and SHA-256 of request/query
parameters. It does not retain the full model payload. The request ID is also
sent as `X-HySim-Request-ID` for correlation with gateway or server logs.

## Evolution path

| Phase | Server/API change | AI capability unlocked |
|---|---|---|
| 0 (implemented) | Python SDK over current production routes | Typed experiments, tool calling, audit hooks. |
| 1 (implemented) | `/api/v1`, independent `session_id`, model revision and ETag; v1 implementation isolated under `src/server/` | Safe multi-client model replacement and snapshot consistency. |
| 2 (implemented base) | Bounded asynchronous PF/OPF jobs, polling, cancellation request, retention cleanup, topology LOD and spatial/time result chunks | Concurrent experiment queues, scalable AI context retrieval, and resumable polling. |
| 2 next | Cooperative solver cancellation, progress events, artifact IDs and durable metadata | Long simulation campaigns and restart recovery. |
| 3 | Add immutable experiment manifests, dataset/artifact store, seed and solver-build provenance | Reproducible surrogate training and benchmark evaluation. |
| 4 | Add an optional pybind11 in-process transport only for high-throughput kernels | Batched RL/surrogate loops without HTTP serialization cost. |

Session and job APIs should be implemented before a large FastAPI gateway. A
Python web gateway alone cannot repair the global C++ session semantics and
would create two competing public contracts.

## Quick start

```bash
python3 -m pip install -e python
./build/macos-release/tests/run_gui_server --port 8088 --data-dir data
python3 python/examples/v1_async_workflow.py
```

For tests without a running solver:

```bash
PYTHONPATH=python/src python3 -m unittest discover -s python/tests -v
```
