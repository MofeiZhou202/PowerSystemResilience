---
name: develop-gui-backend-contract
description: Design, implement, audit, and test HySim features that cross the C++ backend and the native JavaScript GUI. Use whenever work changes HTTP routes or JSON schemas in tests/run_gui_server.cpp, editable component or analysis parameters, web/index.html, web/js/app.js, web/js/canvas.js, topology/result-to-Canvas navigation, GUI visualization or responsive layout, or browser E2E coverage. Enforce exact backend-to-frontend field mapping, stable component identity, honest resolved/default/override semantics, and end-to-end verification across all affected component families.
---

# Develop GUI Backend Contract

Treat every backend-to-GUI feature as one typed contract. Complete the backend,
serialization, GUI state, Canvas identity, validation, visualization, tests, and
living documentation together.

## Establish Current Behavior

1. Read `AGENTS.md` and `docs/overview/development_status.md`.
2. Read the relevant contract in `docs/README.md`; use
   `docs/reference/runtime_api.md` for session routes and
   `docs/developer/gui_canvas_runtime.md` for Canvas navigation.
3. Trace the feature through its authoritative surfaces:
   - model and resolver: `include/hacdcpf/` plus `src/`;
   - HTTP session and JSON: `tests/run_gui_server.cpp`;
   - structure: `web/index.html`;
   - behavior and rendering: `web/js/app.js`;
   - component identity and selection: `web/js/canvas.js`;
   - presentation: `web/css/style.css`;
   - browser contracts: `tests/e2e/` and `tests/CMakeLists.txt`.
4. Search by route, JSON key, DOM ID, component kind, and stable index. Do not
   infer support from a label, an input control, or a model field alone.
5. Preserve unrelated dirty-worktree changes.

## Define The Mapping Before Editing

Create a task-local ledger for every affected value:

| Contract item | Required evidence |
|---|---|
| Semantic value | Meaning, unit, range, nullable behavior |
| Backend source | C++ field/resolver and precedence |
| API representation | Exact JSON path, type, validation, limitation |
| GUI representation | DOM/control, display format, editable state |
| Identity | Domain, component kind, stable `.index`, aliases |
| Round trip | Load, edit, save, reload, reset/preserve behavior |
| Verification | Unit/API/DOM/interaction assertion |

Stop and resolve any ledger row that lacks an authoritative backend source.
Never fill a GUI gap with a plausible hard-coded value.

## Preserve Identity

- Use `component_kind + component .index` for public component identity. Treat
  vector position as internal and expose it only as explicitly named diagnostic
  metadata such as `component_position`.
- Keep AC and DC bus maps domain-qualified. Never key cross-domain state by a
  bare bus integer.
- Map rich, canonical, and Canvas representations explicitly. If one visual
  object represents multiple source records, register every stable alias. A
  branch-backed two-winding transformer, for example, must be addressable by
  both the AC branch index and rich transformer index.
- Put the stable Canvas target on result/topology rows and verify that clicking
  the row selects and reveals the intended object.
- Reject ambiguous duplicate stable identities instead of selecting the first
  matching vector element.

## Implement The Backend Contract

- Keep physical and reliability defaults in the model/resolver layer, not in
  JavaScript. Resolve precedence once and serialize the effective result.
- Return every editable field in a machine-readable schema and return its
  effective value for every supported component or mode. Keep saved user
  overrides separate from resolved display values.
- State units, bounds, optionality, mutually exclusive fields, data source, and
  unsupported-model reasons in the response or the referenced contract.
- Validate requests in C++ even when the GUI also validates them. Save session
  configuration atomically only after complete validation.
- Define model-change semantics: clear stale configuration on replacement and
  preserve it only for an intentional same-model synchronization followed by
  stable-reference validation.
- Propagate approximation, fallback, unavailable consequence models, and
  ignored configuration through explicit result limitations. Never fabricate a
  zero-impact or successful result.

## Implement The GUI Contract

- Derive controls from the backend schema/effective response when practical.
  Otherwise keep one named field list beside the renderer and test exact key
  equality with the backend.
- Display resolved values for all applicable component families. Visually
  distinguish inherited/effective values from explicit user overrides.
- Save sparse intent: post only values the user changed or previously
  overrode. Do not convert every displayed default into an override.
- Enforce exclusivity groups when a quantity has alternate forms. Editing one
  hazard representation, for example, must remove conflicting hazard fields
  from the outgoing override.
- After save, render the backend response again so the GUI shows canonical
  values and validation outcomes, not optimistic local state.
- Use the existing control and icon conventions. Keep large tables in bounded
  independent scrolling regions, preserve keyboard access, and verify that
  labels and values fit at desktop and mobile widths.
- Visualize the full workflow when the feature is an analysis: show inputs,
  backend-backed stages, results, calculation principles/equations, units, and
  declared limitations. Do not invent progress percentages or unsupported
  stages when the endpoint provides no progress stream.
- Escape all server- or user-provided text before inserting HTML.

## Verify End To End

Scale coverage with the contract surface. For parameter or topology work,
include all of the following:

1. Add C++ resolver/model tests for defaults, custom values, invalid values,
   precedence, stable identity, and every affected component family.
2. Add an API assertion that every returned row contains the complete editable
   schema with valid types, units, finite numeric values where required, and an
   honest support/limitation state.
3. Add browser E2E using a built-in case broad enough to contain all affected
   AC, DC, converter, transformer, source, load, storage, and protection
   families.
4. Assert DOM population, editing a non-generator component, sparse save,
   reload persistence, reset semantics, invalid-input rejection, and backend
   source/override changes.
5. For topology links, cover standalone and extracted/aliased devices; assert
   row identity, map identity, click behavior, selected Canvas object, and
   reveal/scroll behavior.
6. Check desktop and mobile viewports for overlap and page-level horizontal
   overflow. Use bounded table overflow deliberately.
7. Register new E2E scripts in `tests/CMakeLists.txt`; an unregistered script is
   not regression coverage.

Prefer the focused regression targets when relevant:

```bash
node --check web/js/app.js
node --check web/js/canvas.js
cmake --build <build-dir> --target test_reliability_resolver run_gui_server -j4
ctest --test-dir <build-dir> --output-on-failure \
  -R 'reliability_configuration_e2e|topology_transformer_link_e2e|reliability_workflow_e2e'
git diff --check
```

Also run the nearest broader GUI/API suite when blast radius warrants it. If an
unrelated assertion fails, isolate the relevant assertions and report both the
focused pass and the broader non-green result; do not call the whole suite
successful.

## Finish The Change

- Re-read the mapping ledger against source and response payloads.
- Perform real browser inspection after automated E2E; retain an accessible
  local GUI URL when practical.
- Update existing contracts rather than adding dated snapshots. Record volatile
  build/test evidence and unclosed failures in `docs/overview/development_status.md`.
- Use `manage-codebase-context` when changing AI-facing or status documents.
- Do not claim completion while any affected backend field, component family,
  GUI control, stable link, or round-trip path remains unverified.
