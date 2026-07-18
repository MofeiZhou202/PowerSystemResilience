# HySim Python SDK

This package is the Python boundary for the HySim-XJTU-HRPES C++ runtime. It
uses the production session routes exposed by `run_gui_server`; it does not
duplicate solver or projection logic in Python.

```bash
python3 -m pip install -e python
./build/macos-release/tests/run_gui_server --port 8088 --data-dir data
```

```python
from hysim import HySimClient, PowerFlowOptions, PowerFlowRequest

client = HySimClient("http://127.0.0.1:8088")
client.load_builtin("ieee14_acdc")
result = client.power_flow(
    PowerFlowRequest(options=PowerFlowOptions(max_iter=100, tol=1e-8))
)
result.require_usable()
print(result.summary())
```

The runtime currently owns one process-global model session. Use one server
process per independent experiment until the C++ service gains session IDs.
See `docs/python_api.md` for architecture, AI tool policy, and the evolution
path toward multi-session jobs.

