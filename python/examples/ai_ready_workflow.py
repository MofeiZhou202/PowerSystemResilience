"""Minimal typed workflow and provider-neutral AI tool registration."""

from hysim import (
    HySimClient,
    HySimToolRegistry,
    PowerFlowOptions,
    PowerFlowRequest,
    ToolEffect,
    ToolPolicy,
)


client = HySimClient("http://127.0.0.1:8088")
client.load_builtin("ieee14_acdc")

pf = client.power_flow(
    PowerFlowRequest(options=PowerFlowOptions(max_iter=100, tol=1e-8))
)
pf.require_usable()
print(pf.summary())

# Read/analysis effects are available to an agent. Model mutation remains
# disabled even though its schema is discoverable in the registry.
registry = HySimToolRegistry(
    client,
    policy=ToolPolicy(allowed_effects=frozenset({ToolEffect.READ, ToolEffect.ANALYZE})),
)
print(registry.function_schemas())

