"""Lifecycle helper for one local C++ runtime process."""

from __future__ import annotations

import socket
import subprocess
import time
from pathlib import Path
from typing import IO, Any

from .client import HySimClient
from .errors import HySimError


class LocalHySimServer:
    """Start an isolated runtime for an experiment or AI worker."""

    def __init__(
        self,
        executable: str | Path,
        *,
        data_dir: str | Path,
        matpower_dir: str | Path | None = None,
        host: str = "127.0.0.1",
        port: int | None = None,
        startup_timeout: float = 20.0,
        log: IO[str] | int | None = subprocess.DEVNULL,
    ) -> None:
        self.executable = Path(executable)
        self.data_dir = Path(data_dir)
        self.matpower_dir = Path(matpower_dir) if matpower_dir else None
        self.host = host
        self.port = port or _free_port(host)
        self.startup_timeout = startup_timeout
        self.log = log
        self.process: subprocess.Popen[str] | None = None

    @property
    def base_url(self) -> str:
        return f"http://{self.host}:{self.port}"

    def start(self) -> "LocalHySimServer":
        if self.process is not None:
            raise RuntimeError("server already started")
        command = [
            str(self.executable),
            "--host",
            self.host,
            "--port",
            str(self.port),
            "--data-dir",
            str(self.data_dir),
        ]
        if self.matpower_dir is not None:
            command.extend(["--matpower-dir", str(self.matpower_dir)])
        self.process = subprocess.Popen(
            command,
            stdout=self.log,
            stderr=self.log,
            text=True,
        )
        client = HySimClient(self.base_url, timeout=2.0)
        deadline = time.monotonic() + self.startup_timeout
        while time.monotonic() < deadline:
            if self.process.poll() is not None:
                raise HySimError(f"run_gui_server exited with code {self.process.returncode}")
            try:
                client.list_cases()
                return self
            except HySimError:
                time.sleep(0.1)
        self.stop()
        raise HySimError(f"run_gui_server did not become ready at {self.base_url}")

    def stop(self) -> None:
        if self.process is None:
            return
        if self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=5.0)
        self.process = None

    def client(self, **kwargs: Any) -> HySimClient:
        return HySimClient(self.base_url, **kwargs)

    def __enter__(self) -> "LocalHySimServer":
        return self.start()

    def __exit__(self, *_: object) -> None:
        self.stop()


def _free_port(host: str) -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.bind((host, 0))
        return int(sock.getsockname()[1])
