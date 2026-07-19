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
    result = job.wait_result().require_usable()
    print(result.summary())

    topology_pages = list(session.topology_pages(lod=2, page_size=8))
    first_bus = topology_pages[0].bus_refs[0]
    print({
        "topology_pages": len(topology_pages),
        "topology_nodes": sum(page.returned_nodes for page in topology_pages),
        "local_nodes": len(session.subgraph_view(first_bus, depth=2).nodes),
        "frame_nodes": job.frame_chunk(0, limit=8).returned_nodes,
        "violations": job.violation_chunk(0, limit=20).returned,
    })

    tools = HySimV1ToolRegistry(session)
    print(tools.function_schemas())

    job.delete()
finally:
    session.delete()
