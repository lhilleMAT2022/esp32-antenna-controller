"""Sensor calibration fitting and guided capture; no actuator commands."""

from __future__ import annotations

import json
import math
import secrets
import threading
import time
from collections import deque
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Callable, Any

import numpy as np

FACES = ("+x", "-x", "+y", "-y", "+z", "-z")
AXES = {"+x": 1, "-x": -1, "+y": 2, "-y": -2, "+z": 3, "-z": -3}
ACCEL_LIMITS = {
    "samples_per_face": 12, "max_std_g": 0.025, "min_up_g": 0.7,
    "max_cross_axis_g": 0.25, "max_bias_g": 0.3,
    "min_gain": 0.7, "max_gain": 1.3, "max_vector_error_g": 0.08,
}


@dataclass
class Fit:
    offset: list[float]
    matrix: list[float]
    residual: float

    def values(self) -> list[float]:
        return [round(v, 7) for v in self.offset + self.matrix]


def fit_magnetometer(samples: list[list[float]]) -> Fit:
    """Fit a 3-D ellipsoid, rejecting planar, poorly covered or noisy captures.

    The symmetric correction removes hard/soft iron. Its determinant is one:
    heading is calibrated, but absolute field strength needs a field reference.
    """
    raw = np.asarray(samples, dtype=float)
    if raw.ndim != 2 or raw.shape[1] != 3 or len(raw) < 120:
        raise ValueError("Need at least 120 distinct samples while tumbling all axes")
    if not np.isfinite(raw).all() or np.max(np.abs(raw)) > 2000:
        raise ValueError("Magnetic samples are non-finite or out of range")
    origin = raw.mean(axis=0)
    scale = float(np.std(raw))
    if scale < 5:
        raise ValueError("Insufficient magnetic span; rotate through all orientations")
    xyz = (raw - origin) / scale
    x, y, z = xyz.T
    design = np.column_stack((x*x, y*y, z*z, 2*x*y, 2*x*z, 2*y*z, x, y, z))
    coefficients, _, rank, singular = np.linalg.lstsq(design, np.ones(len(x)), rcond=None)
    if rank < 9 or singular[0] / singular[-1] > 100:
        raise ValueError("Capture is planar or poorly conditioned; tumble in 3-D")
    a, b, c, d, e, f, g, h, i = coefficients
    quad = np.array([[a, d, e], [d, b, f], [e, f, c]])
    try:
        center = -0.5 * np.linalg.solve(quad, [g, h, i])
        norm = 1.0 + center @ quad @ center
        eigenvalues, vectors = np.linalg.eigh(quad / norm)
    except np.linalg.LinAlgError as exc:
        raise ValueError("Cannot fit a valid magnetic ellipsoid") from exc
    if not np.isfinite(eigenvalues).all() or np.min(eigenvalues) <= 0:
        raise ValueError("Capture does not describe a closed magnetic ellipsoid")
    radii = scale / np.sqrt(eigenvalues)
    if min(radii) < 10 or max(radii) > 200 or max(radii)/min(radii) > 4:
        raise ValueError("Implausible magnetic radii or excessive distortion")
    radius = float(np.prod(radii) ** (1/3))
    matrix = vectors @ np.diag(radius / radii) @ vectors.T
    offset = origin + center * scale
    corrected = (raw - offset) @ matrix.T
    lengths = np.linalg.norm(corrected, axis=1)
    residual = float(np.sqrt(np.mean((lengths/radius - 1)**2)))
    directions = corrected / lengths[:, None]
    # Both hemispheres of every axis, all octants, and broadly spread directions.
    octants = {tuple(row > 0) for row in directions}
    coverage = np.linalg.eigvalsh(directions.T @ directions / len(directions))
    if len(octants) < 8 or min(coverage) < 0.15:
        raise ValueError("Incomplete orientation coverage; include all eight octants")
    if residual > 0.05 or np.max(np.abs(lengths/radius - 1)) > 0.15:
        raise ValueError("Magnetic fit residual too large; remove interference and recapture")
    if np.max(np.abs(offset)) > 1000:
        raise ValueError("Magnetic bias exceeds the supported range")
    return Fit(offset.tolist(), matrix.ravel().tolist(), residual)


def accelerometer_diagnostics(faces: dict[str, list[list[float]]]) -> dict[str, Any]:
    """Return the measurements and verdict used by the six-face fit, in g."""
    result: dict[str, Any] = {"limits": dict(ACCEL_LIMITS), "faces": {}, "passed": False}
    means = {}
    problems = []
    for face in FACES:
        raw = np.asarray(faces.get(face, []), dtype=float)
        detail: dict[str, Any] = {"sample_count": len(raw), "problems": []}
        result["faces"][face] = detail
        if raw.shape != (ACCEL_LIMITS["samples_per_face"], 3) or not np.isfinite(raw).all():
            problem = f"Capture 12 stationary samples with {face} pointing up"
            detail["problems"].append(problem)
            problems.append(problem)
            continue
        mean = raw.mean(axis=0)
        std = np.std(raw, axis=0)
        axis = abs(AXES[face]) - 1
        sign = 1 if AXES[face] > 0 else -1
        up, cross = float(sign * mean[axis]), float(max(abs(np.delete(mean, axis))))
        detail.update(mean_g=mean.tolist(), std_g=std.tolist(), up_g=up, cross_axis_g=cross)
        if max(std) > ACCEL_LIMITS["max_std_g"]:
            detail["problems"].append(f"Movement detected on {face}: std {max(std):.4f} g > 0.025 g; repeat that face")
        if up < ACCEL_LIMITS["min_up_g"] or cross > ACCEL_LIMITS["max_cross_axis_g"]:
            detail["problems"].append(f"Wrong orientation for {face}: up {up:.4f} g, cross-axis {cross:.4f} g; point that sensor axis up")
        problems.extend(detail["problems"])
        means[face] = mean
    if problems:
        result["error"] = "; ".join(problems)
        return result
    positive = np.array([means[f"+{axis}"][j] for j, axis in enumerate("xyz")])
    negative = np.array([means[f"-{axis}"][j] for j, axis in enumerate("xyz")])
    offset = (positive + negative)/2
    gains = 2/(positive-negative)
    result.update(offset_g=offset.tolist(), gains=gains.tolist())
    if (max(abs(offset)) > ACCEL_LIMITS["max_bias_g"] or min(gains) < ACCEL_LIMITS["min_gain"]
            or max(gains) > ACCEL_LIMITS["max_gain"]):
        result["error"] = ("Accelerometer bias or scale outside supported limits: "
                           f"bias {offset.round(4).tolist()} g, gains {gains.round(4).tolist()}")
        return result
    errors = {}
    for face, mean in means.items():
        expected = np.zeros(3)
        expected[abs(AXES[face])-1] = 1 if AXES[face] > 0 else -1
        corrected = (mean-offset)*gains
        errors[face] = float(np.linalg.norm(corrected-expected))
        result["faces"][face].update(corrected_g=corrected.tolist(), vector_error_g=errors[face])
    worst = max(errors, key=errors.get)
    failed = sorted((f for f in FACES if errors[f] > ACCEL_LIMITS["max_vector_error_g"]),
                    key=errors.get, reverse=True)
    result.update(worst_face=worst, residual_g=errors[worst], failed_faces=failed)
    if failed:
        measurements = ", ".join(f"{face} {errors[face]:.4f} g" for face in failed)
        result["error"] = (f"Six-face validation failed: {measurements} exceed 0.0800 g "
                           f"(worst {worst}); check alignment and repeat affected faces")
        return result
    result["passed"] = True
    return result


def fit_accelerometer(faces: dict[str, list[list[float]]]) -> Fit:
    """Six stationary faces determine bias and per-axis scale (units: g)."""
    detail = accelerometer_diagnostics(faces)
    if not detail["passed"]:
        raise ValueError(detail["error"])
    return Fit(detail["offset_g"], np.diag(detail["gains"]).ravel().tolist(), detail["residual_g"])


@dataclass
class Capture:
    boot: int | None = None
    last_sequence: int | None = None
    mode: str = ""
    samples: list[list[float]] = field(default_factory=list)
    sample_kind: str = ""
    faces: dict[str, list[list[float]]] = field(default_factory=dict)
    fits: dict[str, Fit] = field(default_factory=dict)
    status: dict[str, Any] = field(default_factory=dict)
    seen: float = 0
    pending: tuple[int, str, float] | None = None
    note: str = "Press c for calibration instructions"
    recent_sequences: deque[int] = field(default_factory=lambda: deque(maxlen=64))
    retired_boots: deque[int] = field(default_factory=lambda: deque(maxlen=8))
    capture_path: Path | None = None
    last_archive: Path | None = None
    archive_error: str = ""
    timeout_s: float = 3.5


class CalibrationManager:
    """One capture and one acknowledged calibration operation per node."""

    def __init__(self, send: Callable, report: Callable,
                 capture_dir: str | Path = "calibration_captures") -> None:
        self.send, self.report = send, report
        self.nodes = {1: Capture(), 2: Capture()}
        self.lock = threading.RLock()
        self.capture_dir = Path(capture_dir).resolve()

    def _archive_path(self, node: int, event: str) -> Path:
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
        return self.capture_dir / f"node{node}-{stamp}-{event}-{secrets.token_hex(3)}.json"

    def _archive(self, node: int, event: str, *, path: Path | None = None,
                 error: str | None = None) -> Path | None:
        """Atomically checkpoint raw captures; disk failures must not drop telemetry."""
        s = self.nodes[node]
        path = path or self._archive_path(node, event)
        payload = {
            "schema_version": 1, "recorded_at_utc": datetime.now(timezone.utc).isoformat(),
            "node": node, "boot": s.boot, "last_sequence": s.last_sequence,
            "event": event, "error": error, "note": s.note,
            "units": {"mag": "uT", "accel": "g"}, "status": s.status,
            "mode": s.mode, "sample_kind": s.sample_kind,
            "samples": s.samples, "faces": s.faces,
            "fits": {kind: asdict(fit) for kind, fit in s.fits.items()},
            "accel_diagnostics": accelerometer_diagnostics(s.faces),
        }
        temporary = path.with_suffix(".tmp")
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            with temporary.open("w", encoding="utf-8") as output:
                json.dump(payload, output, indent=2, allow_nan=False)
                output.write("\n")
            temporary.replace(path)
        except (OSError, ValueError) as exc:
            warning = f"Capture file NOT saved: {exc}"
            if warning != s.archive_error:
                self.report(node, warning)
            s.archive_error = warning
            try:
                temporary.unlink(missing_ok=True)
            except OSError:
                pass
            return None
        s.last_archive, s.archive_error = path, ""
        return path

    def ingest(self, message: dict[str, Any]) -> None:
        node = message.get("n")
        if node not in self.nodes:
            return
        with self.lock:
            state = self.nodes[node]
            if message.get("t") == "cs":
                boot = message.get("boot")
                seq = message.get("q")
                if not all(isinstance(message.get(k), int) and 0 <= message[k] <= 0xffffffff for k in ("boot", "q", "req", "cf", "e")):
                    return
                if boot in state.retired_boots:
                    return
                if state.boot is not None and boot != state.boot:
                    if state.samples or state.faces or state.fits:
                        self._archive(node, "before-node-restart")
                    state.retired_boots.append(state.boot)
                    state.mode = ""
                    state.sample_kind = ""
                    state.capture_path = None
                    state.samples.clear()
                    state.faces.clear()
                    state.fits.clear()
                    state.pending = None
                    state.recent_sequences.clear()
                    state.last_sequence = None
                    state.status.clear()
                    state.note = "Node restarted; capture cleared. Loaded state reported below."
                    self.report(node, state.note)
                state.boot = boot
                previous = state.status.get("q")
                if previous is None or 0 < ((seq-previous) & 0xffffffff) < 0x80000000:
                    state.status = dict(message)
                    state.seen = time.monotonic()
                if state.pending and message.get("req") == state.pending[0]:
                    operation = state.pending[1]
                    error = message.get("e", 0)
                    state.note = f"Node confirmed {operation}" if not error else f"Node rejected {operation}: error {error}"
                    state.pending = None
                    self.report(node, state.note)
                return
            if message.get("t") != "rs" or not state.mode:
                return
            if time.monotonic() - state.seen > state.timeout_s:
                state.note = "Waiting for fresh calibration telemetry"
                return
            seq = message.get("q")
            if not isinstance(seq, int) or seq in state.recent_sequences:
                return
            if state.last_sequence is not None and ((seq-state.last_sequence) & 0xffffffff) >= 0x80000000:
                return
            state.last_sequence = seq
            state.recent_sequences.append(seq)
            key, required_flag = ("m", 8) if state.mode == "mag" else ("a", 4)
            raw = message.get(key)
            if not message.get("sf", 0) & required_flag:
                return
            if not isinstance(raw, list) or len(raw) != 3:
                return
            if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in raw):
                return
            if len(state.samples) < 1800:
                state.samples.append(list(raw))
            else:
                return
            if state.mode in FACES and len(state.samples) == 12:
                face = state.mode
                state.faces[face] = state.samples.copy()
                state.mode = ""
                state.note = f"Captured {face}; faces: {', '.join(state.faces)}. Select the next face."
                self.report(node, state.note)
            elif state.mode == "mag":
                state.note = f"Tumble ALL axes: {len(state.samples)}/120 minimum samples; then cal {node} fit mag"
            self._archive(node, "capture", path=state.capture_path)

    def observe_runtime(self, message: dict[str, Any], timeout_s: float) -> None:
        node = message.get("n")
        if node not in self.nodes or not all(k in message for k in ("boot", "q", "cf")):
            return
        with self.lock:
            state = self.nodes[node]
            state.timeout_s = timeout_s
            # Heartbeats carry current calibration flags even when quiet mode
            # suppresses corrected-vector reports. Do not invent a new heading.
            report = dict(state.status)
            report.update(t="cs", n=node, boot=message["boot"], q=message["q"],
                          cf=message["cf"], req=0, e=0)
            if message["boot"] != state.boot:
                report.pop("ch", None)
            self.ingest(report)

    def describe(self, node: int) -> str:
        with self.lock:
            s = self.nodes[node]
            self._expire(node)
            note = s.note + ("\n" + s.archive_error if s.archive_error else "")
            if not s.seen or time.monotonic()-s.seen > s.timeout_s:
                return "CALIBRATION UNKNOWN / STALE\n" + note
            flags = int(s.status.get("cf", 0))
            parts = [f"{label}: {'yes' if flags & bit else 'no'}" for bit, label in ((1, 'mag'), (2, 'accel'), (4, 'mount'))]
            saved = "SAVED" if flags & 8 else "UNSAVED" if flags & 7 else "UNCALIBRATED"
            if flags & 7:
                saved = ("CALIBRATED / " if flags & 7 == 7 else "PARTIAL / ") + saved
            if flags & 16:
                saved = "NVS INVALID — recalibrate and save"
            heading = s.status.get("ch")
            orientation = f"Corrected heading {heading:.1f}°T" if isinstance(heading, (int, float)) and math.isfinite(heading) else "Corrected heading unavailable"
            return f"{saved} | {' '.join(parts)}\n{orientation}\n{note}"

    def _expire(self, node: int) -> None:
        s = self.nodes[node]
        if s.pending and time.monotonic() > s.pending[2]:
            s.note = f"{s.pending[1]} timed out; outcome unknown. Request status before retrying."
            s.pending = None
            self.report(node, s.note)

    def command(self, fields: list[str]) -> str:
        if len(fields) < 2 or fields[0] not in {"1", "2"}:
            raise ValueError("Use cal <node> start mag | face +x | fit mag/accel | apply mag/accel | align +x +z <declination> | save | clear | status | cancel | export")
        node, action, *args = int(fields[0]), fields[1], *fields[2:]
        with self.lock:
            s = self.nodes[node]
            self._expire(node)
            if action == "export" and not args:
                path = self._archive(node, "export")
                if path is None:
                    raise ValueError(s.archive_error)
                s.note = f"Capture exported to {path}"
                self.report(node, s.note)
                return s.note
            if action == "cancel" and not args:
                s.mode = ""
                s.note = "Capture cancelled; active node calibration unchanged"
                self._archive(node, "cancel")
                return s.note
            if action == "status" and not args:
                return self._request(node, "status", [])
            if not s.seen or time.monotonic()-s.seen > s.timeout_s:
                raise ValueError("No fresh calibration status; update node and gateway firmware, then request status")
            if s.pending:
                raise ValueError("Waiting for node confirmation; do not send another calibration operation")
            if action == "start" and args == ["mag"]:
                if not s.status.get("cf", 0) & 32:
                    raise ValueError("Node has no valid magnetometer readings")
                s.mode, s.samples = "mag", []
                s.sample_kind = "mag"
                s.capture_path = self._archive_path(node, "mag")
                s.fits.pop("mag", None)
                s.note = "Slowly tumble the sensor in ALL orientations for at least 120 seconds; then fit mag"
                self._archive(node, "capture", path=s.capture_path)
            elif action == "face" and len(args) == 1 and args[0] in FACES:
                if not s.status.get("cf", 0) & 64:
                    raise ValueError("Node has no valid accelerometer readings")
                s.mode, s.samples = args[0], []
                s.sample_kind = args[0]
                s.capture_path = self._archive_path(node, "face-" + args[0])
                s.fits.pop("accel", None)
                s.note = f"Hold sensor {args[0]} pointing UP and stationary for 12 samples"
                self._archive(node, "capture", path=s.capture_path)
            elif action == "fit" and args in (["mag"], ["accel"]):
                kind = args[0]
                if kind == "accel" and s.mode:
                    raise ValueError("Wait for the current face capture to finish")
                if kind == "mag" and s.sample_kind != "mag":
                    raise ValueError("Start a magnetometer capture before fitting mag")
                try:
                    fit = fit_magnetometer(s.samples) if kind == "mag" else fit_accelerometer(s.faces)
                except ValueError as exc:
                    s.note = str(exc)
                    path = self._archive(node, f"fit-{kind}-failed", error=str(exc))
                    detail = f"Capture saved to {path}" if path else s.archive_error
                    self.report(node, s.note + "\n" + detail)
                    raise ValueError(s.note + "\n" + detail) from exc
                s.fits[kind] = fit
                s.mode = ""
                s.note = f"{kind} fit passed, residual {fit.residual:.4f}; cal {node} apply {kind}"
                path = self._archive(node, f"fit-{kind}-passed")
                if path:
                    self.report(node, f"Capture saved to {path}")
            elif action == "apply" and len(args) == 1 and args[0] in s.fits:
                return self._request(node, args[0], s.fits[args[0]].values())
            elif action == "align" and len(args) == 3:
                if args[0] not in AXES or args[1] not in AXES or abs(AXES[args[0]]) == abs(AXES[args[1]]):
                    raise ValueError("Forward and up must be distinct signed sensor axes, e.g. +x +z")
                declination = float(args[2])
                if not math.isfinite(declination) or abs(declination) > 180:
                    raise ValueError("Declination must be finite in [-180, 180], east positive")
                return self._request(node, "align", [AXES[args[0]], AXES[args[1]], declination])
            elif action in {"save", "clear"} and not args:
                return self._request(node, action, [])
            else:
                raise ValueError("Invalid calibration command; press c for instructions")
            self.report(node, s.note)
            return s.note

    def _request(self, node: int, operation: str, values: list[float]) -> str:
        s = self.nodes[node]
        if s.pending:
            raise ValueError("Waiting for a calibration response")
        request = secrets.randbelow(0xffffffff) + 1
        s.pending = (request, operation, time.monotonic()+5)
        message = {"t": "cc", "n": node, "q": request, "op": operation, "v": values}
        try:
            route = self.send(node, message)
        except Exception:
            s.pending = None
            raise
        s.note = (f"{operation} sent via {route}; waiting for NODE confirmation" if s.pending else
                  f"{operation} sent via {route}; silent mode suppresses radio ACK; acceptance unconfirmed")
        self.report(node, s.note)
        return s.note
