# Canvas Runtime and Result Playback

Updated: 2026-07-12

The authored model lives in `Canvas.state`, while analysis results live in SVG
result overlays. These states must remain separate.

## Static results

`Canvas.showPowerFlowResults(result)` is the static PF/OPF presentation path.
It can refresh solved component displays and the visualization overlay.
PF and OPF post-PF payloads share authored-space component keys and terminal
flow fields.

## Time playback

`CanvasPlaybackController` in `web/js/app.js` provides:

- time slider;
- play/pause and previous/next frame;
- 1x, 2x, and 5x speed;
- current frame time and convergence state;
- voltage, flow, frequency, P/Q, and SOC modes;
- lazy frame fetch with stale-request cancellation and a bounded frame cache.

`Canvas.renderFrame(frame)` in `web/js/canvas.js` accepts
`canvas_frame_v1`. It removes and updates only `.frame-overlay` and
`.viz-overlay` elements. It must not:

- write `component.params`;
- rebuild components or connections;
- synchronize the model to the backend;
- reuse static PF equations for transient branch arrows.

TSPF flow/PQ modes may reuse solved static branch geometry because the frame
contains actual per-step PF terminal powers. Transient rendering is limited to
recorded voltage, frequency, device telemetry, SOC/state, and active event
markers. Flow mode is disabled when `capabilities.branch_power` is false.

## Identity and units

Frame component lookup uses `canvas_type`, `canvas_index`, and `position`, with
explicit aliases such as `switch -> switch_comp` and
`renewable_generator -> renewable_gen`. Power values are stored in MW/MVar;
the Canvas display converts both P and Q consistently when MW/kW/W is changed.

## Performance

The backend does not send thousands of complete Canvas frames with the main
analysis response. Playback requests one frame by index and caches a small
window in the browser. This keeps long runs responsive and avoids contaminating
the next authored-model synchronization.

