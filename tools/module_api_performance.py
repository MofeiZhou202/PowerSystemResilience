"""Record exact request/response evidence while running the GUI API regression.

Runs its own server through gui_api_e2e. Expected HTTP errors are retained;
HTTP 200 and regression success do not imply every numerical solve converged.
"""
import argparse
import hashlib
import io
import json
import platform
from pathlib import Path
import sys
import time
import urllib.error

import gui_api_e2e


def capture_http_error_body(error):
    """Read an HTTPError body without consuming it for the calling client."""
    try:
        raw = error.read()
    finally:
        error.close()
    replay = urllib.error.HTTPError(
        error.url, error.code, error.msg, error.hdrs, io.BytesIO(raw)
    )
    return raw, replay


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--server', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--skip-etap', action='store_true')
    args = parser.parse_args()
    output = Path(args.output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    report = {'server_sha256': hashlib.sha256(Path(args.server).read_bytes()).hexdigest(),
              'platform': platform.platform(), 'scope': 'Sequential GUI API regression; HTTP wall time excludes UI rendering.',
              'requests': []}

    class TimedClient(gui_api_e2e.Client):
        def _req(self, path, *, data, ctype, method='POST'):
            index = len(report['requests'])
            row = {'index': index, 'path': path, 'method': method,
                   'request_bytes': len(data or b''),
                   'request_sha256': hashlib.sha256(data or b'').hexdigest()}
            if data and ctype == 'application/json':
                (output / f'{index:04d}-request.json').write_bytes(data)
            started = time.perf_counter()
            try:
                status, headers, raw = super()._req(path, data=data, ctype=ctype, method=method)
                row['http_wall_ms'] = (time.perf_counter() - started) * 1000
                row.update(http_status=status, response_bytes=len(raw))
                (output / f'{index:04d}-response.bin').write_bytes(raw)
                try:
                    body = json.loads(raw)
                    if isinstance(body, dict):
                        row['result'] = {k: body[k] for k in (
                            'converged', 'status', 'objective', 'iterations', 'residual',
                            'solver_backend', 'fallback_used', 'timing', 'model_scope',
                            'model_limitations', 'error', 'success') if k in body}
                except (ValueError, UnicodeError):
                    pass
                return status, headers, raw
            except urllib.error.HTTPError as error:
                raw, replay_error = capture_http_error_body(error)
                row.update(http_status=error.code,
                           response_bytes=len(raw),
                           expected_or_unexpected_error=True)
                (output / f'{index:04d}-response.bin').write_bytes(raw)
                try:
                    body = json.loads(raw)
                    if isinstance(body, dict):
                        row['result'] = {k: body[k] for k in (
                            'error', 'status', 'model_scope', 'model_limitations'
                        ) if k in body}
                except (ValueError, UnicodeError):
                    pass
                raise replay_error from error
            except Exception as error:
                row['exception'] = repr(error)
                raise
            finally:
                if 'http_wall_ms' not in row:
                    row['http_wall_ms'] = (time.perf_counter() - started) * 1000
                report['requests'].append(row)
                (output / 'report.json').write_text(json.dumps(report, indent=2), encoding='utf-8')

    gui_api_e2e.Client = TimedClient
    sys.argv = ['gui_api_e2e', '--server', str(Path(args.server).resolve())]
    if args.skip_etap:
        sys.argv.append('--skip-etap')
    try:
        result = gui_api_e2e.main()
        report['regression_exit_code'] = result
        return result
    finally:
        (output / 'report.json').write_text(json.dumps(report, indent=2), encoding='utf-8')


if __name__ == '__main__':
    raise SystemExit(main())
