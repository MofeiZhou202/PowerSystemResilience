"""Isolated v1 session with revision-aware asynchronous power flow."""

from hysim import (
    HySimV1Client,
    HySimV1ToolRegistry,
    PowerFlowOptions,
    PowerFlowRequest,
)


client = HySimV1Client("http://127.0.0.1:8088")
session = client.create_session(case="ieee14_acdc")
try:
    job = session.power_flow(
        PowerFlowRequest(options=PowerFlowOptions(max_iter=100, tol=1e-8))
    )
    result = job.wait().result().require_usable()
    print(result.summary())

    tools = HySimV1ToolRegistry(session)
    print(tools.function_schemas())

    job.delete()
finally:
    session.delete()

