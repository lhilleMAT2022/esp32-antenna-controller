"""Reporting controls and sensor-frame vector/UTC presentation."""
from __future__ import annotations

import math
import secrets
import threading
import time
from dataclasses import dataclass
from typing import Callable

MODES = ("continuous", "normal", "quiet")
MODE_TIMEOUTS = {"continuous": 5.0, "normal": 25.0, "quiet": 90.0}


def vector_mae(vector: list[float] | None) -> tuple[float, float | None, float | None] | None:
    if vector is None or len(vector) != 3 or not all(math.isfinite(v) for v in vector):
        return None
    x, y, z = vector
    horizontal = math.hypot(x, y)
    magnitude = math.hypot(horizontal, z)
    if magnitude < 1e-9:
        return magnitude, None, None
    azimuth = math.degrees(math.atan2(y, x)) % 360 if horizontal >= 1e-9 else None
    return magnitude, azimuth, math.degrees(math.atan2(z, horizontal))


def vector_text(vector: list[float] | None, precision: int, unit: str) -> str:
    mae = vector_mae(vector)
    if mae is None:
        return f"XYZ:(—,—,—) / MAE:(—,—,—) {unit}"
    xyz = ",".join(f"{v:.{precision}f}" for v in vector)
    angles = ["—" if v is None else f"{v:.1f}°" for v in mae[1:]]
    return f"XYZ:({xyz}) / MAE:({mae[0]:.{precision}f},{','.join(angles)}) {unit}"


def utc_text(milliseconds: int | None) -> str:
    if not milliseconds:
        return "UNSYNCED"
    return time.strftime("%H:%M:%S", time.gmtime(milliseconds/1000)) + f".{milliseconds % 1000:03d} UTC"


@dataclass
class ModeRequest:
    request: int
    mode: str
    seconds: int
    deadline: float


class ReportingControl:
    """Global requests with independent, explicit node acknowledgments."""
    def __init__(self, send: Callable, report: Callable) -> None:
        self.send, self.report = send, report
        self.pending: dict[int, ModeRequest] = {}
        self.results: dict[int, str] = {}
        self.lock = threading.RLock()

    def command(self, mode: str, seconds: int = 0) -> str:
        if mode not in MODES or isinstance(seconds, bool) or not isinstance(seconds, int):
            raise ValueError("Use report continuous | report normal | report quiet <seconds>")
        if (mode == "quiet" and not 1 <= seconds <= 86400) or (mode != "quiet" and seconds):
            raise ValueError("Quiet needs a duration of 1..86400 seconds; other modes take no duration")
        with self.lock:
            self.expire()
            if self.pending:
                raise ValueError("Waiting for reporting-mode acknowledgments")
            request = ModeRequest(secrets.randbelow(0xffffffff)+1, mode, seconds, time.monotonic()+5)
            self.pending = {node: request for node in (1, 2)}
            self.results = {node: "pending" for node in (1, 2)}
            for node in (1, 2):
                try:
                    self.send(node, {"t": "rm", "n": node, "q": request.request,
                                     "mode": mode, "duration_s": seconds})
                except (OSError, RuntimeError) as exc:
                    self.pending.pop(node, None)
                    self.results[node] = f"send failed: {exc}"
                    self.report(node, self.results[node])
            return f"Requested {mode} for both nodes; " + self.describe()

    def ingest(self, message: dict) -> None:
        if message.get("t") != "rp":
            return
        node = message.get("n")
        with self.lock:
            request = self.pending.get(node)
            if not request or message.get("mr") != request.request:
                return
            error = message.get("me")
            accepted = error == 0 and message.get("mode") == request.mode
            self.results[node] = "confirmed" if accepted else f"rejected / current mode {message.get('mode')}, error {error}"
            self.pending.pop(node)
            self.report(node, f"Reporting {request.mode}: {self.results[node]}")

    def expire(self) -> None:
        with self.lock:
            for node, request in list(self.pending.items()):
                if time.monotonic() > request.deadline:
                    self.pending.pop(node)
                    self.results[node] = "timed out; outcome unknown"
                    self.report(node, self.results[node])

    def confirmed(self) -> bool:
        with self.lock:
            return not self.pending and all(self.results.get(node) == "confirmed" for node in (1, 2))

    def describe(self) -> str:
        with self.lock:
            self.expire()
            return "; ".join(f"N{n} {self.results[n]}" for n in (1, 2) if n in self.results)
