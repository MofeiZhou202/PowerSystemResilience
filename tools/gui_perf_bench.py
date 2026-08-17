"""GUI backend performance baseline on large cases (run_gui_server).

Measures wall-clock latency and payload sizes for the endpoints that dominate
large-system GUI interactivity:

  load_matpower  (full system load + serialization)
  topology       (full topology response)
  topology_window (bbox-cropped lightweight response)
  pf             (per-request deep copy + solve + result serialization)
  export_json    (full indented dump, now outside the lock)

Usage:
    python3 tools/gui_perf_bench.py [--port 8088] [--case case_ACTIVSg10k.m]
                                    [--pf-repeats 3]

Requires a running server, e.g.:
    ./build/macos-release/run_gui_server --port 8088 --data-dir data \
        --matpower-dir external_data/matpower
"""

from __future__ import annotations

import argparse
import json
import time
import urllib.request
import urllib.error


def post_json(port: int, path: str, payload: dict):
    body = json.dumps(payload).encode()
    req = urllib.request.Request(
        f"http://127.0.0.1:{port}{path}",
        data=body,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    t0 = time.perf_counter()
    try:
        with urllib.request.urlopen(req, timeout=600) as resp:
            raw = resp.read()
            status = resp.status
    except urllib.error.HTTPError as e:
        raw = e.read()
        status = e.code
    dt = time.perf_counter() - t0
    return status, raw, dt


def fmt(raw: bytes, dt: float) -> str:
    return f"{dt * 1000:9.1f} ms  {len(raw) / 1e6:8.2f} MB"


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8088)
    ap.add_argument("--case", default="case_ACTIVSg10k.m")
    ap.add_argument("--pf-repeats", type=int, default=3)
    args = ap.parse_args()
    port = args.port

    print(f"== case: {args.case} ==")

    st, raw, dt = post_json(port, "/api/session/load_matpower",
                            {"filename": args.case})
    body = json.loads(raw) if raw else {}
    counts = body.get("counts") or body.get("summary") or {}
    print(f"load_matpower   {st}  {fmt(raw, dt)}  counts={counts}")
    if st != 200:
        print("load failed; aborting")
        return

    st, raw, dt = post_json(port, "/api/session/topology", {})
    print(f"topology        {st}  {fmt(raw, dt)}")

    # Full-extent window (coordinates are usually absent in MATPOWER cases;
    # the response then honestly declares coordinate_coverage.complete=false).
    win = {"min_x": -180.0, "min_y": -90.0, "max_x": 180.0, "max_y": 90.0,
           "lod": 2}
    st, raw, dt = post_json(port, "/api/session/topology_window", win)
    cov = {}
    if st == 200 and raw:
        cov = (json.loads(raw).get("coordinate_coverage")) or {}
    print(f"topology_window {st}  {fmt(raw, dt)}  coverage={cov}")

    for i in range(args.pf_repeats):
        st, raw, dt = post_json(port, "/api/session/pf", {})
        tag = "pf" if i == 0 else f"pf#{i + 1}"
        conv = ""
        if st == 200 and raw:
            try:
                j = json.loads(raw)
                res = j.get("result") or j
                conv = (f"converged={res.get('converged')} "
                        f"iters={res.get('iterations')}")
            except Exception:
                pass
        print(f"{tag:<16}{st}  {fmt(raw, dt)}  {conv}")

    st, raw, dt = post_json(port, "/api/session/export_json",
                            {"filename": "perf_bench_export.json"})
    print(f"export_json     {st}  {fmt(raw, dt)}")


if __name__ == "__main__":
    main()
