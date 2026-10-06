"""Antenna Controller (AC) and Raspberry Pi serial-relay application.

The AC role connects the local CYD USB gateway to TCP 31988 and publishes
ICD-compatible ``antenna_state`` datagrams on UDP 31989.  An optional backup
client connects to the Pi relay on TCP 31995.  The relay role exposes one
remote ESP32 USB serial port without interpreting antenna-control policy.
"""

from __future__ import annotations

import argparse
import json
import logging
import math
import os
import platform
from collections import deque
import socket
import socketserver
import threading
import time
import uuid
from dataclasses import dataclass, field
from typing import Any, Callable

from .calibration import CalibrationManager

LOGGER = logging.getLogger("antenna_controller")
SERIAL_BAUD = 115200
AC_COMMAND_PORT = 31988
ANTENNA_STATE_PORT = 31989
PI_RELAY_PORT = 31995
ROUTE_STALE_SECONDS = 3.5
SERIAL_SILENCE_RECONNECT_SECONDS = 8.0


def json_line(message: dict[str, Any]) -> bytes:
    return (
        json.dumps(message, separators=(",", ":"), allow_nan=False) + "\n"
    ).encode("utf-8")


def parse_board_line(line: str | bytes) -> dict[str, Any] | None:
    """Return a recognized compact board message, ignoring human log lines."""
    if isinstance(line, bytes):
        line = line.decode("utf-8", errors="replace")
    line = line.strip()
    if not line.startswith("{"):
        return None
    try:
        message = json.loads(line)
    except json.JSONDecodeError:
        return None
    if not isinstance(message, dict) or message.get("t") not in {
        "rp",
        "rs",
        "ra",
        "rc",
        "cc",
        "cs",
    }:
        return None
    return message


def compact_rotator_command(
    node_id: int,
    sequence: int,
    command: str,
    *,
    azimuth_deg: float | None = None,
    delta_deg: float | None = None,
    execute_at_utc_s: int | None = None,
) -> dict[str, Any]:
    """Create the canonical compact JSON command accepted by CYD and nodes."""
    if node_id not in (1, 2):
        raise ValueError("node_id must be 1 or 2")
    if command not in {"goto", "stop", "step", "queue", "overlap", "time"}:
        raise ValueError(f"unsupported command: {command}")
    message: dict[str, Any] = {
        "t": "rc",
        "n": node_id,
        "q": int(sequence),
        "c": command,
    }
    if command in {"goto", "queue", "overlap"}:
        if azimuth_deg is None:
            raise ValueError(f"{command} requires azimuth_deg")
        if command in {"goto", "queue"} and not 0.0 <= azimuth_deg < 360.0:
            raise ValueError("azimuth must be in [0, 360)")
        message["az"] = round(float(azimuth_deg), 1)
    if command == "step":
        if delta_deg is None:
            raise ValueError("step requires delta_deg")
        message["d"] = round(float(delta_deg), 1)
    if command in {"queue", "time"}:
        if execute_at_utc_s is None or execute_at_utc_s <= 0:
            raise ValueError(f"{command} requires execute_at_utc_s")
        message["at"] = int(execute_at_utc_s)
    return message


@dataclass
class NodeState:
    node_id: int
    heading_deg: float | None = None
    target_deg: float | None = None
    moving: bool = False
    fault_code: str | None = None
    measured_utc_ms: int = 0
    route: str = "unknown"
    magnetic_heading_deg: float | None = None
    magnetic_ut: list[float] | None = None
    acceleration_g: list[float] | None = None
    field_strength_ut: float | None = None
    roll_deg: float | None = None
    pitch_deg: float | None = None
    sensor_flags: int = 0
    rssi_at_gateway_dbm: float | None = None
    rssi_at_node_dbm: float | None = None


@dataclass(frozen=True)
class SerialDeviceIdentity:
    """Stable USB identity used to survive Windows COM-port reassignment."""

    vid: int | None
    pid: int | None
    serial_number: str | None
    location: str | None

    @classmethod
    def from_port_info(cls, port_info: Any) -> "SerialDeviceIdentity":
        return cls(
            getattr(port_info, "vid", None),
            getattr(port_info, "pid", None),
            getattr(port_info, "serial_number", None),
            getattr(port_info, "location", None),
        )

    def matches(self, port_info: Any) -> bool:
        if not self.same_usb_family(port_info):
            return False
        candidate_serial = getattr(port_info, "serial_number", None)
        if self.serial_number:
            return candidate_serial == self.serial_number
        candidate_location = getattr(port_info, "location", None)
        return bool(self.location and candidate_location == self.location)

    def same_usb_family(self, port_info: Any) -> bool:
        return (
            self.vid is not None
            and self.pid is not None
            and self.vid == getattr(port_info, "vid", None)
            and self.pid == getattr(port_info, "pid", None)
        )


@dataclass(frozen=True)
class SerialSettings:
    """Runtime serial framing and flow-control settings."""

    baud: int = SERIAL_BAUD
    data_bits: int = 8
    parity: str = "N"
    stop_bits: float = 1.0
    read_timeout_s: float = 0.5
    write_timeout_s: float = 2.0
    xonxoff: bool = False
    rtscts: bool = False
    dsrdtr: bool = False

    def summary(self) -> str:
        stop_bits = (
            str(int(self.stop_bits))
            if self.stop_bits.is_integer()
            else str(self.stop_bits)
        )
        flow = ",".join(
            name
            for enabled, name in (
                (self.xonxoff, "XON/XOFF"),
                (self.rtscts, "RTS/CTS"),
                (self.dsrdtr, "DSR/DTR"),
            )
            if enabled
        )
        return (
            f"{self.baud} baud, {self.data_bits}{self.parity}{stop_bits}, "
            f"read timeout {self.read_timeout_s:g}s, "
            f"write timeout {self.write_timeout_s:g}s, "
            f"flow control {flow or 'none'}"
        )


@dataclass
class NodeStateStore:
    nodes: dict[int, NodeState] = field(
        default_factory=lambda: {1: NodeState(1), 2: NodeState(2)}
    )
    route_last_seen: dict[tuple[int, str], float] = field(default_factory=dict)
    lock: threading.Lock = field(default_factory=threading.Lock)

    def update(self, message: dict[str, Any], route: str) -> NodeState | None:
        node_id = message.get("n")
        if node_id not in (1, 2):
            return None
        now_ms = int(time.time() * 1000)
        with self.lock:
            state = self.nodes[node_id]
            self.route_last_seen[(node_id, route)] = time.monotonic()
            state.route = route
            state.measured_utc_ms = now_ms
            if message.get("t") == "rp":
                state.heading_deg = _finite_or_none(message.get("h"))
                state.target_deg = _finite_or_none(message.get("tg"))
                state.moving = bool(message.get("mv", 0))
                if "rg" in message:
                    state.rssi_at_gateway_dbm = _finite_or_none(
                        message.get("rg")
                    )
                if "rn" in message:
                    state.rssi_at_node_dbm = _finite_or_none(message.get("rn"))
                error = message.get("e", 0)
                state.fault_code = None if error in (0, None) else str(error)
            elif message.get("t") == "rs":
                state.magnetic_heading_deg = _finite_or_none(message.get("mh"))
                state.magnetic_ut = _vector(message.get("m"))
                state.acceleration_g = _vector(message.get("a"))
                state.field_strength_ut = _finite_or_none(message.get("f"))
                state.roll_deg = _finite_or_none(message.get("r"))
                state.pitch_deg = _finite_or_none(message.get("p"))
                state.sensor_flags = int(message.get("sf", 0))
            return NodeState(**vars(state))

    def route_fresh(
        self, node_id: int, route: str, max_age_s: float = ROUTE_STALE_SECONDS
    ) -> bool:
        with self.lock:
            seen = self.route_last_seen.get((node_id, route))
        return seen is not None and time.monotonic() - seen <= max_age_s

    def route_age(self, node_id: int, route: str) -> float | None:
        with self.lock:
            seen = self.route_last_seen.get((node_id, route))
        return None if seen is None else max(0.0, time.monotonic() - seen)

    def snapshot(self) -> list[NodeState]:
        with self.lock:
            return [NodeState(**vars(self.nodes[node])) for node in (1, 2)]


def _finite_or_none(value: Any) -> float | None:
    if value is None:
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _vector(value: Any) -> list[float] | None:
    if not isinstance(value, list) or len(value) != 3:
        return None
    result = [_finite_or_none(item) for item in value]
    return None if any(item is None for item in result) else result  # type: ignore[return-value]


def _format_vector(value: list[float] | None, precision: int) -> str:
    if value is None:
        return "(---,---,---)"
    return "(" + ",".join(f"{item:.{precision}f}" for item in value) + ")"


class SerialJsonLink:
    """Reconnectable pyserial JSON-line endpoint."""

    def __init__(
        self,
        name: str,
        port: str,
        callback: Callable[[dict[str, Any], str], None],
        settings: SerialSettings | None = None,
        silence_reconnect_s: float = SERIAL_SILENCE_RECONNECT_SECONDS,
    ) -> None:
        self.name = name
        self.requested_port = port
        self.port = port
        self.settings = settings or SerialSettings()
        self.callback = callback
        self.silence_reconnect_s = silence_reconnect_s
        self.connected = threading.Event()
        self._stop = threading.Event()
        self._serial: Any = None
        self._device_identity: SerialDeviceIdentity | None = None
        self._write_lock = threading.Lock()
        self._thread = threading.Thread(
            target=self._run, name=f"serial-{name}", daemon=True
        )

    def start(self) -> None:
        self._thread.start()

    def close(self) -> None:
        self._stop.set()
        serial_port = self._serial
        if serial_port is not None:
            try:
                serial_port.close()
            except Exception:
                pass

    def send(self, message: dict[str, Any]) -> bool:
        payload = json_line(message)
        with self._write_lock:
            if not self.connected.is_set() or self._serial is None:
                return False
            try:
                self._serial.write(payload)
                self._serial.flush()
                return True
            except Exception as exc:
                LOGGER.warning("%s serial write failed: %s", self.name, exc)
                self.connected.clear()
                return False

    def _run(self) -> None:
        try:
            import serial  # type: ignore
            from serial.tools import list_ports  # type: ignore
        except ImportError:
            LOGGER.error(
                "pyserial is required; run: python -m pip install -r requirements.txt"
            )
            return
        down_reported = False
        while not self._stop.is_set():
            try:
                port_info = self._resolve_port(list(list_ports.comports()))
                if port_info is None:
                    self.connected.clear()
                    if not down_reported:
                        LOGGER.warning(
                            "%s serial LOST: device %s is unavailable",
                            self.name,
                            self.port,
                        )
                        down_reported = True
                    self._stop.wait(1.0)
                    continue
                resolved_port = str(getattr(port_info, "device", port_info))
                if resolved_port.casefold() != self.port.casefold():
                    LOGGER.info(
                        "%s serial device moved from %s to %s",
                        self.name,
                        self.port,
                        resolved_port,
                    )
                self.port = resolved_port
                if not isinstance(port_info, str):
                    self._device_identity = SerialDeviceIdentity.from_port_info(
                        port_info
                    )
                LOGGER.info("opening %s serial port %s", self.name, self.port)
                self._serial = serial.Serial(
                    self.port,
                    baudrate=self.settings.baud,
                    bytesize=self.settings.data_bits,
                    parity=self.settings.parity,
                    stopbits=self.settings.stop_bits,
                    timeout=self.settings.read_timeout_s,
                    write_timeout=self.settings.write_timeout_s,
                    xonxoff=self.settings.xonxoff,
                    rtscts=self.settings.rtscts,
                    dsrdtr=self.settings.dsrdtr,
                )
                self.connected.set()
                if down_reported:
                    LOGGER.info(
                        "%s serial RESTORED on %s", self.name, self.port
                    )
                else:
                    LOGGER.info(
                        "%s serial connected on %s (%s)",
                        self.name,
                        self.port,
                        self.settings.summary(),
                    )
                down_reported = False
                last_receive = time.monotonic()
                while not self._stop.is_set():
                    raw = self._serial.readline()
                    if not raw:
                        if not self._port_is_present(
                            list(list_ports.comports())
                        ):
                            raise serial.SerialException(
                                f"{self.name} serial device disappeared"
                            )
                        if (
                            self.silence_reconnect_s > 0
                            and time.monotonic() - last_receive
                            >= self.silence_reconnect_s
                        ):
                            raise serial.SerialException(
                                f"{self.name} serial telemetry stale"
                            )
                        continue
                    last_receive = time.monotonic()
                    message = parse_board_line(raw)
                    if message is not None:
                        self.callback(message, self.name)
                    else:
                        LOGGER.debug(
                            "%s: %s",
                            self.name,
                            raw.decode("utf-8", errors="replace").strip(),
                        )
            except Exception as exc:
                if not self._stop.is_set() and not down_reported:
                    LOGGER.warning("%s serial LOST: %s", self.name, exc)
                    down_reported = True
            finally:
                self.connected.clear()
                if self._serial is not None:
                    try:
                        self._serial.close()
                    except Exception:
                        pass
                self._serial = None
            self._stop.wait(2.0)

    def _resolve_port(self, ports: list[Any]) -> Any | None:
        """Resolve the configured USB device, following COM reassignment."""
        exact = next(
            (
                port_info
                for port_info in ports
                if str(port_info.device).casefold() == self.port.casefold()
            ),
            None,
        )
        if exact is not None and (
            self._device_identity is None
            or self._device_identity.matches(exact)
        ):
            return exact
        if self._device_identity is not None:
            matches = [
                port_info
                for port_info in ports
                if self._device_identity.matches(port_info)
            ]
            if len(matches) == 1:
                return matches[0]
            if not self._device_identity.serial_number:
                same_family = [
                    port_info
                    for port_info in ports
                    if self._device_identity.same_usb_family(port_info)
                ]
                if len(same_family) == 1:
                    LOGGER.info(
                        "%s serial device USB location changed; following "
                        "the only matching VID:PID device",
                        self.name,
                    )
                    return same_family[0]
            return None
        requested = next(
            (
                port_info
                for port_info in ports
                if str(port_info.device).casefold()
                == self.requested_port.casefold()
            ),
            None,
        )
        if requested is not None:
            return requested
        return self.requested_port if os.path.exists(self.requested_port) else None

    def _port_is_present(self, ports: list[Any]) -> bool:
        if os.path.exists(self.port):
            return True
        return self._resolve_port(ports) is not None


class TcpJsonClient:
    """Reconnectable TCP JSON-line client used for the node-2 backup path."""

    def __init__(
        self,
        host: str,
        port: int,
        callback: Callable[[dict[str, Any], str], None],
    ) -> None:
        self.host = host
        self.port = port
        self.callback = callback
        self.connected = threading.Event()
        self._stop = threading.Event()
        self._socket: socket.socket | None = None
        self._write_lock = threading.Lock()
        self._thread = threading.Thread(
            target=self._run, name="backup-client", daemon=True
        )

    def start(self) -> None:
        self._thread.start()

    def close(self) -> None:
        self._stop.set()
        if self._socket is not None:
            try:
                self._socket.close()
            except OSError:
                pass

    def send(self, message: dict[str, Any]) -> bool:
        with self._write_lock:
            if self._socket is None or not self.connected.is_set():
                return False
            try:
                self._socket.sendall(json_line(message))
                return True
            except OSError as exc:
                LOGGER.warning("backup write failed: %s", exc)
                self.connected.clear()
                return False

    def _run(self) -> None:
        down_reported = False
        while not self._stop.is_set():
            try:
                connection = socket.create_connection(
                    (self.host, self.port), timeout=3
                )
                connection.settimeout(1.0)
                self._socket = connection
                self.connected.set()
                if down_reported:
                    LOGGER.info(
                        "Pi relay RESTORED at %s:%d", self.host, self.port
                    )
                else:
                    LOGGER.info(
                        "Pi relay connected at %s:%d", self.host, self.port
                    )
                down_reported = False
                buffer = b""
                while not self._stop.is_set():
                    try:
                        chunk = connection.recv(4096)
                    except socket.timeout:
                        continue
                    if not chunk:
                        raise ConnectionError("relay connection closed")
                    buffer += chunk
                    while b"\n" in buffer:
                        raw, buffer = buffer.split(b"\n", 1)
                        message = parse_board_line(raw)
                        if message is not None:
                            self.callback(message, "backup")
            except OSError as exc:
                if not self._stop.is_set() and not down_reported:
                    LOGGER.warning(
                        "Pi relay LOST at %s:%d: %s",
                        self.host,
                        self.port,
                        exc,
                    )
                    down_reported = True
            finally:
                self.connected.clear()
                if self._socket is not None:
                    try:
                        self._socket.close()
                    except OSError:
                        pass
                self._socket = None
            self._stop.wait(2.0)


class _ThreadingServer(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True


class AntennaController:
    """AC process: command listener, state publisher, and route selector."""

    def __init__(
        self,
        serial_port: str,
        *,
        command_host: str = "127.0.0.1",
        command_port: int = AC_COMMAND_PORT,
        state_host: str = "127.0.0.1",
        state_port: int = ANTENNA_STATE_PORT,
        backup_host: str | None = None,
        backup_port: int = PI_RELAY_PORT,
        prefer_backup_node2: bool = False,
        tui: bool = False,
        serial_settings: SerialSettings | None = None,
    ) -> None:
        self.states = NodeStateStore()
        self.instance_id = str(uuid.uuid4())
        self.sequence = 0
        self.command_sequence = 0
        self.state_destination = (state_host, state_port)
        self.state_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        if state_host == "255.255.255.255":
            self.state_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.primary = SerialJsonLink(
            "gateway",
            serial_port,
            self._handle_board_message,
            settings=serial_settings,
        )
        self.backup = (
            TcpJsonClient(backup_host, backup_port, self._handle_board_message)
            if backup_host
            else None
        )
        self.prefer_backup_node2 = prefer_backup_node2
        self.tui = tui
        self.accept_external_commands = True
        self._event_sequence = 0
        self._events: deque[dict[str, Any]] = deque(maxlen=500)
        self._events_lock = threading.Lock()
        self._client_count = 0
        self._client_lock = threading.Lock()
        self.calibration = CalibrationManager(self._send_calibration, self._report_calibration)
        self._stop = threading.Event()
        self._server = self._make_command_server(command_host, command_port)
        self._server_thread = threading.Thread(
            target=self._server.serve_forever,
            name="ac-command-server",
            daemon=True,
        )
        self._timer_thread = threading.Thread(
            target=self._time_sync_loop, name="backup-time-sync", daemon=True
        )

    def start(self) -> None:
        self.primary.start()
        if self.backup:
            self.backup.start()
        self._server_thread.start()
        self._timer_thread.start()
        LOGGER.info(
            "AC listening on TCP %s:%d; antenna state -> UDP %s:%d",
            *self._server.server_address,
            *self.state_destination,
        )

    def close(self) -> None:
        self._stop.set()
        self.primary.close()
        if self.backup:
            self.backup.close()
        self._server.shutdown()
        self._server.server_close()
        self.state_socket.close()

    def run(self) -> None:
        if self.tui:
            try:
                self._tui_loop()
            except KeyboardInterrupt:
                pass
            return
        self.start()
        try:
            while not self._stop.wait(1.0):
                pass
        except KeyboardInterrupt:
            pass
        finally:
            self.close()

    def _make_command_server(
        self, host: str, port: int
    ) -> socketserver.ThreadingTCPServer:
        controller = self

        class Handler(socketserver.StreamRequestHandler):
            def setup(self) -> None:
                super().setup()
                with controller._client_lock:
                    controller._client_count += 1
                controller._record_event(
                    "CLIENT",
                    f"connected {self.client_address[0]}:{self.client_address[1]}",
                    source="client",
                    route="tcp",
                )

            def finish(self) -> None:
                with controller._client_lock:
                    controller._client_count = max(
                        0, controller._client_count - 1
                    )
                controller._record_event(
                    "CLIENT",
                    f"disconnected {self.client_address[0]}:{self.client_address[1]}",
                    source="client",
                    route="tcp",
                )
                super().finish()

            def handle(self) -> None:
                for raw in self.rfile:
                    try:
                        request = json.loads(raw)
                        response = controller.handle_antenna_command(
                            request, external=True
                        )
                    except (json.JSONDecodeError, ValueError, TypeError) as exc:
                        response = {"ok": False, "error": str(exc)}
                        controller._record_event(
                            "ERROR",
                            f"client command rejected: {exc}",
                            source="client",
                            route="tcp",
                        )
                    self.wfile.write(json_line(response))

        return _ThreadingServer((host, port), Handler)

    def _handle_board_message(
        self, message: dict[str, Any], route: str
    ) -> None:
        self.calibration.ingest(message)
        if message.get("t") == "cs":
            return
        if message.get("t") == "ra":
            LOGGER.info("board response via %s: %s", route, message)
            self._record_event(
                "ACK",
                f"N{message.get('n', '?')} {message.get('c', 'command')} "
                f"e={message.get('e', 0)}",
                source=f"node{message.get('n', '?')}",
                route=route,
            )
            return
        state = self.states.update(message, route)
        if state is None:
            return
        self._publish_state(state)
        if message.get("t") == "rp":
            heading = "---" if state.heading_deg is None else f"{state.heading_deg:.1f}"
            target = "---" if state.target_deg is None else f"{state.target_deg:.1f}"
            self._record_event(
                "REPORT",
                f"N{state.node_id} az={heading} target={target} "
                f"moving={int(state.moving)}",
                source=f"node{state.node_id}",
                route=route,
            )
        elif message.get("t") == "rs":
            magnetic = _format_vector(state.magnetic_ut, 1)
            acceleration = _format_vector(state.acceleration_g, 3)
            self._record_event(
                "SENSOR",
                f"N{state.node_id} B={magnetic}uT A={acceleration}g",
                source=f"node{state.node_id}",
                route=route,
            )

    def _publish_state(self, state: NodeState) -> None:
        self.sequence += 1
        now_ms = int(time.time() * 1000)
        tilt = None
        if state.roll_deg is not None and state.pitch_deg is not None:
            tilt = round(math.hypot(state.roll_deg, state.pitch_deg), 1)
        sensor_valid = (state.sensor_flags & 0x0C) == 0x0C
        field_ok = (
            sensor_valid
            and state.field_strength_ut is not None
            and 20.0 <= state.field_strength_ut <= 70.0
        )
        message = {
            "schema_version": "1.0.0",
            "message_type": "antenna_state",
            "message_id": str(uuid.uuid4()),
            "source": "AntennaController",
            "source_instance_id": self.instance_id,
            "sequence_number": self.sequence,
            "generated_utc_ms": now_ms,
            "antenna": "SURV" if state.node_id == 1 else "REF",
            "measured_utc_ms": state.measured_utc_ms,
            "az_true_deg": state.heading_deg,
            "tilt_deg": tilt,
            "field_strength_ut": state.field_strength_ut,
            "field_strength_ok": field_ok,
            "tilt_ok": sensor_valid and tilt is not None and tilt <= 10.0,
            "moving": state.moving,
            "command_state": (
                "fault"
                if state.fault_code
                else "moving"
                if state.moving
                else "settled"
                if state.heading_deg is not None
                else "idle"
            ),
            "fault_code": state.fault_code,
            "node_last_heard_utc_ms": state.measured_utc_ms,
        }
        try:
            self.state_socket.sendto(
                json_line(message).rstrip(b"\n"), self.state_destination
            )
        except OSError as exc:
            LOGGER.warning("antenna-state UDP publish failed: %s", exc)

    def _select_route(self, node_id: int) -> tuple[str, Any]:
        backup_ready = self.backup is not None and self.backup.connected.is_set()
        if node_id == 2 and backup_ready:
            if self.prefer_backup_node2:
                return "backup", self.backup
            if (
                not self.states.route_fresh(2, "gateway")
                and self.states.route_fresh(2, "backup")
            ):
                return "backup", self.backup
        if self.primary.connected.is_set():
            return "gateway", self.primary
        if node_id == 2 and backup_ready:
            return "backup", self.backup
        raise RuntimeError(f"no command route available for node {node_id}")

    def _send_calibration(self, node_id: int, message: dict[str, Any]) -> str:
        route, endpoint = self._select_route(node_id)
        if not endpoint.send(message):
            raise RuntimeError(f"{route} calibration write failed")
        return route

    def _report_calibration(self, node_id: int, detail: str) -> None:
        self._record_event("CALIBRATION", detail, source=f"node{node_id}", route="calibration")

    def _send_rotator_command(
        self,
        node_id: int,
        command: str,
        *,
        azimuth_deg: float | None = None,
        delta_deg: float | None = None,
        execute_at_utc_s: int | None = None,
    ) -> str:
        self.command_sequence += 1
        message = compact_rotator_command(
            node_id,
            self.command_sequence,
            command,
            azimuth_deg=azimuth_deg,
            delta_deg=delta_deg,
            execute_at_utc_s=execute_at_utc_s,
        )
        route, endpoint = self._select_route(node_id)
        if not endpoint.send(message):
            raise RuntimeError(f"{route} route write failed")
        LOGGER.info("sent node %d %s via %s", node_id, command, route)
        self._record_event(
            "COMMAND",
            f"N{node_id} {command} "
            f"value={azimuth_deg if azimuth_deg is not None else delta_deg}",
            source="operator",
            route=route,
        )
        return route

    def handle_antenna_command(
        self, message: dict[str, Any], *, external: bool = False
    ) -> dict[str, Any]:
        if external and not self.accept_external_commands:
            raise ValueError("AC is OFFLINE; external commands are disabled")
        if message.get("message_type") != "antenna_command":
            raise ValueError("message_type must be antenna_command")
        antenna = message.get("antenna")
        if antenna not in {"SURV", "REF"}:
            raise ValueError("antenna must be SURV or REF")
        node_id = 1 if antenna == "SURV" else 2
        command = message.get("command")
        if command == "go_to":
            target = _finite_or_none(message.get("target_az_deg"))
            if target is None:
                raise ValueError("go_to requires target_az_deg")
            route = self._send_rotator_command(
                node_id, "goto", azimuth_deg=target
            )
        elif command == "stop":
            route = self._send_rotator_command(node_id, "stop")
        elif command == "cal_sweep":
            raise ValueError("cal_sweep is not enabled in simulator firmware")
        else:
            raise ValueError("command must be go_to, stop, or cal_sweep")
        return {
            "ok": True,
            "antenna": antenna,
            "command": command,
            "route": route,
        }

    @property
    def client_count(self) -> int:
        with self._client_lock:
            return self._client_count

    def set_online(self, online: bool) -> None:
        self.accept_external_commands = bool(online)
        self._record_event(
            "COMMAND",
            "external command acceptance "
            + ("enabled" if self.accept_external_commands else "disabled"),
            source="operator",
            route="local",
        )

    def recent_events(self, after_sequence: int = 0) -> list[dict[str, Any]]:
        with self._events_lock:
            return [
                dict(event)
                for event in self._events
                if event["sequence"] > after_sequence
            ]

    def _record_event(
        self,
        event_type: str,
        detail: str,
        *,
        source: str,
        route: str,
    ) -> None:
        with self._events_lock:
            self._event_sequence += 1
            self._events.append(
                {
                    "sequence": self._event_sequence,
                    "utc": time.time(),
                    "type": event_type,
                    "source": source,
                    "route": route,
                    "detail": detail,
                }
            )

    def _time_sync_loop(self) -> None:
        while not self._stop.wait(30.0):
            if self.backup and self.backup.connected.is_set():
                self.command_sequence += 1
                message = compact_rotator_command(
                    2,
                    self.command_sequence,
                    "time",
                    execute_at_utc_s=int(time.time()),
                )
                self.backup.send(message)

    @staticmethod
    def _format_state(state: NodeState) -> str:
        heading = "---" if state.heading_deg is None else f"{state.heading_deg:5.1f}"
        field = (
            "---"
            if state.field_strength_ut is None
            else f"{state.field_strength_ut:5.1f}"
        )
        return (
            f"N{state.node_id} {state.route:7s} az={heading}T "
            f"moving={int(state.moving)} |B|={field}uT"
        )

    def _tui_loop(self) -> None:
        try:
            from .tui import run_textual_tui
        except ImportError as exc:
            raise RuntimeError(
                "Textual TUI dependencies are missing; run `uv sync` or "
                "install textual and textual-plotext"
            ) from exc
        run_textual_tui(self)


class PiSerialRelay:
    """TCP 31995 to USB serial bridge for a nearby remote antenna node."""

    def __init__(
        self,
        serial_port: str,
        *,
        listen_host: str = "0.0.0.0",
        listen_port: int = PI_RELAY_PORT,
        serial_settings: SerialSettings | None = None,
    ) -> None:
        self.clients: set[socket.socket] = set()
        self.clients_lock = threading.Lock()
        self.serial = SerialJsonLink(
            "node_serial",
            serial_port,
            self._from_board,
            settings=serial_settings,
        )
        relay = self

        class Handler(socketserver.StreamRequestHandler):
            def setup(self) -> None:
                super().setup()
                with relay.clients_lock:
                    relay.clients.add(self.request)
                LOGGER.info("AC connected from %s", self.client_address[0])

            def finish(self) -> None:
                with relay.clients_lock:
                    relay.clients.discard(self.request)
                super().finish()

            def handle(self) -> None:
                for raw in self.rfile:
                    message = parse_board_line(raw)
                    if message is None or message.get("t") not in {"rc", "cc"}:
                        self.wfile.write(
                            json_line(
                                {
                                    "t": "ra",
                                    "e": 1,
                                    "detail": "expected compact rc or cc JSON",
                                }
                            )
                        )
                        continue
                    if not relay.serial.send(message):
                        self.wfile.write(
                            json_line(
                                {
                                    "t": "ra",
                                    "n": message.get("n"),
                                    "q": message.get("q"),
                                    "e": 1,
                                    "detail": "node serial unavailable",
                                }
                            )
                        )

        self.server = _ThreadingServer((listen_host, listen_port), Handler)

    def _from_board(self, message: dict[str, Any], _: str) -> None:
        payload = json_line(message)
        dead: list[socket.socket] = []
        with self.clients_lock:
            for client in self.clients:
                try:
                    client.sendall(payload)
                except OSError:
                    dead.append(client)
            for client in dead:
                self.clients.discard(client)
        print(
            f"N{message.get('n', '?')} {message.get('t', '?')} "
            f"{json.dumps(message, separators=(',', ':'))}",
            flush=True,
        )

    def run(self) -> None:
        self.serial.start()
        LOGGER.info(
            "Pi relay listening on TCP %s:%d",
            *self.server.server_address,
        )
        try:
            self.server.serve_forever()
        except KeyboardInterrupt:
            pass
        finally:
            self.serial.close()
            self.server.server_close()


def _add_serial_options(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--baud", type=int, default=SERIAL_BAUD, help="serial baud rate"
    )
    parser.add_argument(
        "--data-bits",
        type=int,
        choices=(5, 6, 7, 8),
        default=8,
        help="serial data bits",
    )
    parser.add_argument(
        "--parity",
        choices=("N", "E", "O", "M", "S"),
        default="N",
        help="serial parity: none, even, odd, mark, or space",
    )
    parser.add_argument(
        "--stop-bits",
        type=float,
        choices=(1.0, 1.5, 2.0),
        default=1.0,
        help="serial stop bits",
    )
    parser.add_argument(
        "--read-timeout",
        type=float,
        default=0.5,
        help="serial read timeout in seconds",
    )
    parser.add_argument(
        "--write-timeout",
        type=float,
        default=2.0,
        help="serial write timeout in seconds",
    )
    parser.add_argument("--xonxoff", action="store_true")
    parser.add_argument("--rtscts", action="store_true")
    parser.add_argument("--dsrdtr", action="store_true")


def _serial_settings_from_args(args: argparse.Namespace) -> SerialSettings:
    return SerialSettings(
        baud=args.baud,
        data_bits=args.data_bits,
        parity=args.parity,
        stop_bits=args.stop_bits,
        read_timeout_s=args.read_timeout,
        write_timeout_s=args.write_timeout,
        xonxoff=args.xonxoff,
        rtscts=args.rtscts,
        dsrdtr=args.dsrdtr,
    )


def scan_serial_ports(settings: SerialSettings | None = None) -> int:
    """Print discoverable serial devices without opening or resetting them."""
    try:
        from serial.tools import list_ports  # type: ignore
    except ImportError:
        print("pyserial is required to scan serial ports")
        return 1

    active_settings = settings or SerialSettings()
    ports = sorted(list_ports.comports(), key=lambda item: item.device)
    print(f"Application serial defaults: {active_settings.summary()}")
    print(
        "Port settings are applied when AC/relay opens a device; USB serial "
        "devices do not advertise a persistent baud/framing configuration."
    )
    if not ports:
        print("No serial devices found.")
        return 0
    for port_info in ports:
        identity = (
            f"VID:PID={port_info.vid:04X}:{port_info.pid:04X}"
            if port_info.vid is not None and port_info.pid is not None
            else "VID:PID=unknown"
        )
        serial_number = port_info.serial_number or "-"
        location = port_info.location or "-"
        print(
            f"{port_info.device}: {port_info.description} | {identity} | "
            f"serial={serial_number} | location={location}"
        )
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--log-level", default="INFO")
    parser.add_argument(
        "--scan-serial",
        action="store_true",
        help="list serial devices and default settings, then exit",
    )
    subparsers = parser.add_subparsers(dest="role")

    ac = subparsers.add_parser("ac", help="run the host Antenna Controller")
    ac.add_argument("--serial", required=True, help="CYD serial port")
    ac.add_argument("--command-host", default="127.0.0.1")
    ac.add_argument("--command-port", type=int, default=AC_COMMAND_PORT)
    ac.add_argument("--state-host", default="127.0.0.1")
    ac.add_argument("--state-port", type=int, default=ANTENNA_STATE_PORT)
    ac.add_argument("--backup-host")
    ac.add_argument("--backup-port", type=int, default=PI_RELAY_PORT)
    ac.add_argument("--prefer-backup-node2", action="store_true")
    ac.add_argument("--tui", action="store_true")
    _add_serial_options(ac)

    relay = subparsers.add_parser(
        "relay", help="run the Raspberry Pi TCP-to-node serial relay"
    )
    relay.add_argument("--serial", required=True, help="remote node serial port")
    relay.add_argument("--listen-host", default="0.0.0.0")
    relay.add_argument("--listen-port", type=int, default=PI_RELAY_PORT)
    _add_serial_options(relay)
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.scan_serial:
        return scan_serial_ports()
    if args.role is None:
        parser.error("a role is required unless --scan-serial is used")
    logging.basicConfig(
        level=getattr(logging, args.log_level.upper()),
        format="%(asctime)s %(levelname)s %(message)s",
    )
    LOGGER.info("starting on %s", platform.node())
    serial_settings = _serial_settings_from_args(args)
    if args.role == "relay":
        PiSerialRelay(
            args.serial,
            listen_host=args.listen_host,
            listen_port=args.listen_port,
            serial_settings=serial_settings,
        ).run()
    else:
        AntennaController(
            args.serial,
            command_host=args.command_host,
            command_port=args.command_port,
            state_host=args.state_host,
            state_port=args.state_port,
            backup_host=args.backup_host,
            backup_port=args.backup_port,
            prefer_backup_node2=args.prefer_backup_node2,
            tui=args.tui,
            serial_settings=serial_settings,
        ).run()
    return 0
