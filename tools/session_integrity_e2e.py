"""Production-route regression for computation ownership and stale publication."""
import argparse
import concurrent.futures
import json
from pathlib import Path
import socket
import subprocess
import tempfile
import time
import urllib.error
import urllib.request


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--server", required=True)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    if not (root / "external_data/matpower/case9241pegase.m").exists():
        print("SKIP: case9241pegase.m is required for the overlapping PF check")
        return 77
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        port = sock.getsockname()[1]
    base = f"http://127.0.0.1:{port}"

    def request(route, body=None, raw=None):
        data = raw if raw is not None else json.dumps(body).encode() if body is not None else None
        req = urllib.request.Request(base + route, data=data,
                                     headers={"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(req, timeout=90) as response:
                return response.status, json.loads(response.read())
        except urllib.error.HTTPError as error:
            return error.code, json.loads(error.read())

    def wait_busy(future):
        for _ in range(500):
            assert not future.done(), "request finished before the overlap was observed"
            if request("/api/session/status")[1]["busy"]:
                return
            time.sleep(0.005)
        raise AssertionError("request never acquired busy")

    with tempfile.TemporaryFile(mode="w+") as log:
        server = subprocess.Popen([str(Path(args.server).resolve()), "--host", "127.0.0.1",
            "--port", str(port), "--data-dir", str(root / "data"), "--matpower-dir",
            str(root / "external_data/matpower")], cwd=root, stdout=log, stderr=subprocess.STDOUT)
        executor = concurrent.futures.ThreadPoolExecutor(max_workers=1)
        try:
            for _ in range(100):
                try:
                    if request("/api/session/status")[0] == 200:
                        break
                except OSError:
                    time.sleep(0.05)
            assert request("/api/session/load_matpower", {"filename": "case14.m"})[0] == 200
            task = executor.submit(request, "/api/session/run_ts_pf", {
                "num_steps": 10000, "skip_uc": True, "run_opf": False})
            wait_busy(task)
            assert request("/api/session/run_ts_pf", raw=b"{")[0] == 400
            assert not task.done(), "ownership check requires an overlapping task"
            assert request("/api/session/status")[1]["busy"] is True
            assert request("/api/session/pf", {"response_detail": "compact"})[0] == 409
            assert request("/api/session/cancel", {})[0] == 200
            state = request("/api/session/status")[1]
            assert state["cancel"] is True
            assert request("/api/session/run_ts_pf", raw=b"{")[0] == 400
            assert request("/api/session/status")[1]["cancel"] is True
            task.result(timeout=90)
            assert request("/api/session/status")[1]["busy"] is False
            print("PASS: invalid/conflicting requests preserve ownership and cancellation")

            assert request("/api/session/load_matpower", {"filename": "case9241pegase.m"})[0] == 200
            task = executor.submit(request, "/api/session/pf", {
                "method": "ac_newton", "response_detail": "compact"})
            wait_busy(task)
            assert request("/api/session/load_matpower", {"filename": "case14.m"})[0] == 200
            assert not task.done(), "replacement must finish before the old PF completes"
            status, result = task.result(timeout=90)
            assert status == 200 and result["converged"] and len(result["vm"]) == 9241
            window_body = {"min_x": -180, "max_x": 180, "min_y": -90, "max_y": 90, "lod": 2}
            status, result = request("/api/session/result_window", window_body)
            assert status == 409 and result["error"] == "no_cached_power_flow", result
            status, result = request("/api/session/pf", {"response_detail": "compact"})
            assert status == 200 and result["converged"] and len(result["vm"]) == 14
            status, result = request("/api/session/result_window", window_body)
            assert status == 200 and result["result_meta"]["result_matches_current_system"] is True
            print("PASS: stale PF cannot repopulate cache; current PF still publishes")
        finally:
            server.terminate()
            try:
                server.wait(timeout=5)
            except subprocess.TimeoutExpired:
                server.kill()
                server.wait(timeout=5)
            executor.shutdown(wait=True, cancel_futures=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
