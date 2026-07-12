> Documentation Sync (2026-07-12)
> Status: implementation-backed schema reference.
> Source of truth: `web/schemas/ev_traffic_scenario.schema.json`, import code,
> and registered tests.

# EV Power-Traffic Scenario Format

The GUI imports a versioned JSON document conforming to
`web/schemas/ev_traffic_scenario.schema.json`. A ready-to-edit example is
`web/examples/ev_traffic_scenario_template.json`.

## Top-level structure

```json
{
  "$schema": "/xjtu/schemas/ev_traffic_scenario.schema.json",
  "schema_version": "1.0",
  "name": "Study name",
  "traffic": { "nodes": [], "links": [] },
  "routes": [],
  "demands": [],
  "station_prices": []
}
```

All IDs are user-defined positive integers. IDs need not be contiguous, but
must be unique within their own collection.

## Collections

| Collection | Required fields | Meaning and units |
|---|---|---|
| `traffic.nodes[]` | `index` | Traffic vertex. Optional `name`, `x`, `y` control the GUI layout. |
| `traffic.links[]` | `index`, `from_node`, `to_node`, `length_km`, `free_flow_time_hr`, `capacity_veh_per_hr` | Directed road segment. `jam_vehicles` is the maximum segment occupancy used by CTM. |
| `routes[]` | `index`, `origin_node`, `destination_node`, `link_indices` | Ordered directed path. Consecutive links must form a continuous origin-to-destination chain. |
| `routes[].charging_stops[]` | `station_id`, `requested_energy_kwh_per_vehicle` | Charging action along the route. `station_id` must match an AC charging station in the current power-system model. |
| `demands[]` | `index`, `origin_node`, `destination_node`, `departure_step`, `vehicles` | EV OD demand. `candidate_route_indices` selects eligible routes; an empty list uses every matching OD route. |
| `station_prices[]` | `station_id`, `price_per_kwh` | Per-step electricity prices. A one-value array is a constant price profile. |

## Referential rules

1. Every link endpoint must reference an existing traffic node.
2. Every route link must reference an existing link and the ordered links must
   form a continuous chain from `origin_node` to `destination_node`.
3. Every demand candidate route must exist and must have the same OD pair.
4. Every route `station_id` must exist in the current power-system model's AC
   charging-station collection.
5. `departure_step` and all price/profile indices are zero-based.
6. Power is in kW for charging and MW for the power-system model; energy is in
   kWh per vehicle; traffic capacity is vehicles per hour.

The GUI's **交通网络设计** dialog enforces rules 1-3 before applying a
scenario. The backend performs the final power-system coupling validation.
