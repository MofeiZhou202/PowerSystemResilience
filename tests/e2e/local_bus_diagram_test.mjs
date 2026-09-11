import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import vm from 'node:vm';
import { performance } from 'node:perf_hooks';

const scope = { window: {} };
vm.createContext(scope);
for (const file of ['local_bus_diagram.js', 'layout_graph.js']) vm.runInContext(
  readFileSync(new URL(`../../web/js/core/${file}`, import.meta.url), 'utf8'), scope);
const api = scope.window.HySimCore.LocalBusDiagram;
const bus = index => ({ index, base_kv: 10 });
const edge = (index, from_bus, to_bus) => ({ index, from_bus, to_bus });
const overlap = (a, b) => a.x < b.x + b.width && b.x < a.x + a.width &&
  a.top < b.top + b.height && b.top < a.top + a.height;
function checkGeometry(model) {
  const geometry = api.layout(model), boxes = [...geometry.positions.values()];
  assert.ok(boxes.length <= 80);
  assert.ok(geometry.paths.length <= 160);
  for (let i = 0; i < boxes.length; i++) for (let j = i + 1; j < boxes.length; j++)
    assert.equal(overlap(boxes[i], boxes[j]), false, 'bus rectangles overlap');
  for (const path of geometry.paths) {
    assert.ok(!/NaN|Infinity/.test(path.d));
    for (const [key, x] of [[path.edge.a, path.ax], [path.edge.b, path.bx]]) {
      const box = geometry.positions.get(key);
      assert.ok(x > box.x && x < box.x + box.width);
    }
  }
  return geometry;
}

const system = {
  ac: { buses: [bus(0), bus(1), bus(2), bus(3)],
    branches: [edge(11, 0, 1), edge(12, 0, 1)],
    transformers_2w: [{ index: 77, source_branch_idx: 11, hv_bus: 0, lv_bus: 1 }],
    switches: [{ index: 4, bus_from: 1, bus_to: 2, closed: false }],
    transformers_3w: [{ index: 5, hv_bus: 1, mv_bus: 2, lv_bus: 3 }],
    loads: [{ index: 8, bus: 0 }] },
  dc: { buses: [bus(0), bus(1), bus(2)], branches: [edge(1, 0, 1)],
    dc_circuit_breakers: [{ index: 9, bus_from: 1, bus_to: 2, in_service: false }],
    dcdc_converters: [{ index: 6, bus_in: 0, bus_out: 2 }] },
  vsc_converters: [{ index: 1, bus_ac: 0, bus_dc: 0 }],
  lcc_converters: [{ index: 2, bus_ac: 3, bus_dc: 1 }],
  energy_routers: [{ index: 3, ports: [{ index: 1, bus: 2, port_type: 'AC' },
    { index: 2, bus: 2, port_type: 'DC' }] }],
};
const before = JSON.stringify(system), graph = api.buildGraph(system);
assert.equal(graph.nodes.size, 7);
assert.equal(graph.nodes.get('ac:0').devices.loads, 1);
assert.equal(graph.edges.filter(e => e.a === 'ac:0' && e.b === 'ac:1').length, 2);
assert.equal(graph.edges.find(e => e.key === 'ac.branches:11').aliases[0].index, 77);
assert.equal(graph.edges.find(e => e.kind === 'switch').open, true);
assert.equal(graph.edges.find(e => e.collection === 'dc.dc_circuit_breakers').inService, false);
assert.ok(graph.edges.some(e => e.kind === 'lcc'));
assert.ok(graph.edges.some(e => e.kind === 'dcdc'));
assert.ok(graph.edges.some(e => e.kind === 'router'));
const local = api.extract(graph, { domain: 'ac', index: 0 }, { hops: 4, limit: 80 });
assert.equal(local.nodes.length, 7);
checkGeometry(local);
assert.equal(JSON.stringify(system), before, 'view mutated authored system');
const isolated = api.extract(api.buildGraph({ ac: { buses: [bus(9)] } }), { domain: 'ac', index: 9 });
assert.equal(isolated.nodes.length, 1); checkGeometry(isolated);
assert.throws(() => api.buildGraph({ ac: { buses: [bus(1), bus(1)] } }), /Duplicate bus/);
assert.throws(() => api.buildGraph({ ac: { buses: [{}] } }), /stable index/);
assert.throws(() => api.buildGraph({ ac: { buses: [bus(false)] } }), /stable index/);
assert.throws(() => api.extract(graph, { domain: 'ac', index: 99 }), /not found/);
assert.throws(() => api.buildGraph({ ac: { buses: [bus(1), bus(2)],
  branches: [edge(1, 1, 2), edge(1, 1, 2)] } }), /Duplicate link/);
const invalid = api.buildGraph({ ac: { buses: [bus(1)], branches: [edge(1, 1, 99)] } });
assert.equal(invalid.edges.length, 0); assert.ok(invalid.limitations.length);

const measurements = [];
const dense = { ac: { buses: Array.from({ length: 80 }, (_, i) => bus(i)), branches: [] } };
for (let i = 0; i < 80; i++) for (let j = i + 1; j < 80; j++)
  dense.ac.branches.push(edge(dense.ac.branches.length, i, j));
const denseModel = api.extract(api.buildGraph(dense), { domain: 'ac', index: 0 }, { limit: 80 });
assert.equal(denseModel.rendered.length, 160); checkGeometry(denseModel);
for (const shape of ['hub', 'mesh']) {
  const network = { ac: { buses: Array.from({ length: 5000 }, (_, i) => bus(i + 1)), branches: [] } };
  for (let i = 2; i <= 5000; i++) network.ac.branches.push(edge(i, shape === 'hub' ? 1 : i - 1, i));
  if (shape === 'mesh') for (let i = 1; i <= 5000; i++)
    network.ac.branches.push(edge(10000 + i, i, (i + 36) % 5000 + 1));
  const start = performance.now();
  const g = api.buildGraph(network);
  const model = api.extract(g, { domain: 'ac', index: 1 }, { hops: 4, limit: 80 });
  api.layout(model);
  const ms = performance.now() - start;
  assert.ok(ms < 500, `${shape}: ${ms} ms exceeds predeclared 500 ms ceiling`);
  checkGeometry(model);
  assert.ok(model.boundary.length > 0);
  assert.ok([...model.degree.values()].every(n => n <= 12));
  if (shape === 'hub') {
    assert.equal(model.nodes.length, 80);
    assert.equal(model.boundary.length, 4920);
    assert.equal(model.rendered.length, 12);
    assert.equal(model.hidden.length, 67);
    assert.equal(model.truncated, true);
    const next = g.nodes.get(model.boundary[0].b);
    assert.ok(api.extract(g, next).hopOf.has(next.key));
  }
  measurements.push({ shape, buses: g.nodes.size, local: model.nodes.length,
    paths: model.rendered.length, overlaps: 0, ms });
}
const layoutGraph = scope.window.HySimCore.LayoutGraph;
for (const rotation of [0, 90]) {
  const c = layoutGraph.build([{ id: 1, type: 'ac_bus', rotation, params: {} }], [], { busbarHalfMax: 320 });
  const m = c.metadata['component-1'];
  assert.ok((rotation === 0 ? m.width : m.height) >= 640 + 36);
}
console.log(JSON.stringify({ schema: 'local_bus_diagram_test_v1', measurements }));
