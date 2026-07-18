# Python API and AI Enhancement Architecture

Updated: 2026-07-19

## Decision

Python is an orchestration and intelligence boundary, not a second simulation
kernel. The C++ rich model, validation, canonical projection, solvers, and
authored-space result attribution remain authoritative. The first implementation
is the dependency-free package under `python/`, which talks to the production
session routes in `tests/run_gui_server.cpp`.

```text
AI / optimization / data science code
        |
        +-- HySimToolRegistry (schemas, effect policy, compact context)
        |
        +-- HySimClient (typed requests, provenance, honest results)
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
| SDK | `HySimClient` | Model lifecycle, PF/OPF, registered analyses, lazy frames. |
| Models | `BusRef`, `ComponentRef`, request dataclasses | Domain-qualified bus IDs and stable authored component IDs. |
| Results | `AnalysisResult` | Request/model provenance, scientific status, limitations, raw payload. |
| AI | `HySimToolRegistry`, `ToolPolicy` | Tool schemas, effect allowlist, explicit mutation approval. |

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
4. Each analysis carries a request UUID and the Python client's model revision.
   The audit hook stores a request hash rather than logging the full system model.
5. Requests are not retried automatically because model changes and analyses are
   not idempotent under the current process-global session.

## AI tool safety

The default policy permits `read` and `analyze`. `modify_model` and `control`
are disabled. Enabling a mutating effect is a deployment decision, and each call
still requires `approved=True`. This separates model reasoning from execution
and prevents an agent from silently replacing the active case.

Tool results are compact by default: scalar status and diagnostics are retained,
while large vectors are represented by their sizes. Direct SDK calls always
retain the full raw result for NumPy/pandas/PyTorch post-processing. Provider
adapters can consume `function_schemas()` or `openai_tools()`; provider-specific
dependencies do not belong in the core package.

## Current limitation

`run_gui_server` owns one process-global session and one busy flag. It is suitable
for a desktop GUI or one AI worker, but not for multiple tenants sharing a
process. The SDK serializes model access within one client, but cannot coordinate
independent clients. Use `LocalHySimServer` to isolate concurrent experiments.

## Evolution path

| Phase | Server/API change | AI capability unlocked |
|---|---|---|
| 0 (implemented) | Python SDK over current production routes | Typed experiments, tool calling, audit hooks. |
| 1 | Move runtime source out of `tests/`; add `/api/v1`, `session_id`, model revision/ETag, OpenAPI schema | Safe multi-client model editing and generated clients. |
| 2 | Add `POST /jobs`, cancellation token, progress events, artifact IDs, bounded worker pool | Long simulation campaigns and resumable agents. |
| 3 | Add immutable experiment manifests, dataset/artifact store, seed and solver-build provenance | Reproducible surrogate training and benchmark evaluation. |
| 4 | Add an optional pybind11 in-process transport only for high-throughput kernels | Batched RL/surrogate loops without HTTP serialization cost. |

Session and job APIs should be implemented before a large FastAPI gateway. A
Python web gateway alone cannot repair the global C++ session semantics and
would create two competing public contracts.

## Quick start

```bash
python3 -m pip install -e python
./build/macos-release/tests/run_gui_server --port 8088 --data-dir data
python3 python/examples/ai_ready_workflow.py
```

For tests without a running solver:

```bash
PYTHONPATH=python/src python3 -m unittest discover -s python/tests -v
```

