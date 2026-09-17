"""Check evidence handling, including timeout cleanup of descendant processes."""
import os
from pathlib import Path
import sys
import tempfile
import time
import unittest
import urllib.error
from io import BytesIO

from module_api_performance import capture_http_error_body
from module_performance_audit import discover_test_suites, execute, test_executable


class AuditRunnerTest(unittest.TestCase):
    def test_http_error_evidence_remains_readable_by_regression_client(self):
        body = b'{"error":"expected"}'
        error = urllib.error.HTTPError(
            'http://localhost/test', 422, 'test', {}, BytesIO(body)
        )
        captured, replay = capture_http_error_body(error)
        self.assertEqual(captured, body)
        self.assertEqual(replay.read(), body)
        error.close()
        replay.close()

    def test_exit_and_log_are_retained(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / 'failure.log'
            row = execute([sys.executable, '-c', 'print("evidence"); raise SystemExit(7)'],
                          directory, output, 10, os.environ.copy())
            self.assertEqual(row['exit_code'], 7)
            self.assertEqual(row['status'], 'failed')
            self.assertIn('evidence', output.read_text(encoding='utf-8'))
            self.assertGreater(row['wall_seconds'], 0)

    def test_timeout_kills_descendants(self):
        with tempfile.TemporaryDirectory() as directory:
            survivor_file = Path(directory) / 'child-survived'
            child = (
                'import pathlib,time; time.sleep(1); '
                f'pathlib.Path({str(survivor_file)!r}).write_text("survived")'
            )
            script = ('import pathlib,subprocess,sys,time; '
                      f'subprocess.Popen([sys.executable,"-c",{child!r}]); '
                      'time.sleep(30)')
            row = execute([sys.executable, '-c', script], directory,
                          Path(directory) / 'timeout.log', 0.5, os.environ.copy())
            self.assertEqual(row['status'], 'timeout')
            self.assertLess(row['wall_seconds'], 5)
            time.sleep(1)
            self.assertFalse(survivor_file.exists())

    def test_test_executable_supports_cmake_layouts(self):
        with tempfile.TemporaryDirectory() as directory:
            build = Path(directory)
            suffix = '.exe' if os.name == 'nt' else ''
            single = build / 'tests' / ('test_alpha' + suffix)
            single.parent.mkdir(parents=True)
            single.touch()
            single.chmod(0o755)
            self.assertEqual(test_executable(build, 'test_alpha'), single)
            self.assertIn('test_alpha', discover_test_suites(build))

            multi = build / 'tests' / 'Release' / ('test_alpha' + suffix)
            multi.parent.mkdir()
            multi.touch()
            multi.chmod(0o755)
            self.assertEqual(test_executable(build, 'test_alpha'), multi)


if __name__ == '__main__':
    unittest.main()
