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
result = job.wait().result().require_usable()
print(result.summary())
```

`HySimV1Client` is the preferred interface for new automation: each session is
isolated, model mutations use ETags, and analyses run as revision-bound jobs.
`HySimClient` remains available for the legacy GUI-compatible process-global
routes. See `docs/python_api.md` for architecture and AI tool policy.
