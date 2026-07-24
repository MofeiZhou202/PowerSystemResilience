# Sioux Falls TNTP data

Source: Transportation Networks for Research

- Repository: <https://github.com/bstabler/TransportationNetworks>
- Dataset directory: `SiouxFalls/`
- Retrieved: 2026-07-23
- Files: `SiouxFalls_net.tntp`, `SiouxFalls_node.tntp`, and
  `SiouxFalls_trips.tntp`

The source repository states that donated datasets are for academic research
purposes and must be attributed in publications. The files in this directory
are retained without numerical modification. Unit conversion and CTM parameter
completion are performed explicitly by `hacdcpf::io::make_traffic_graph`.

For the coupled EV study, the road topology, link capacity, free-flow time,
node coordinates, and static OD matrix come from these files. Departure-time
profiles, EV penetration, charging stations, and traffic-to-power bus mappings
are study assumptions and are not part of the TNTP dataset.
