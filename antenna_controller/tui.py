"""Textual operator console for the Antenna Controller."""

from __future__ import annotations

import shlex
import logging
import time
from collections import deque
from dataclasses import dataclass
from typing import TYPE_CHECKING

from rich.markup import escape
from textual import on
from textual.app import App, ComposeResult
from textual.binding import Binding
from textual.containers import Horizontal, Vertical
from textual.widgets import Button, Footer, Header, Input, RichLog, Select, Static
from textual_plotext import PlotextPlot

if TYPE_CHECKING:
    from .bridge import AntennaController, NodeState


GRAPH_MODES = ("track", "magnetometer", "accelerometer", "rssi")
TIME_WINDOWS = (30, 60, 120, 300, 900)
NODE_FILTERS = ("all", "node 1", "node 2")
LOG_FILTERS = (
    "all",
    "system",
    "report",
    "sensor",
    "command",
    "ack",
    "error",
    "client",
)


@dataclass
class HistorySample:
    monotonic_s: float
    nodes: list["NodeState"]


class AntennaControllerApp(App[None]):
    """Radar-console-style Textual UI backed by a running controller."""

    TITLE = "AntennaCtl"
    SUB_TITLE = "Distributed antenna pointing console"

    CSS = """
    Screen {
        background: #020b12;
        color: #d8f3ff;
    }

    Header {
        background: #06334b;
        color: #e6fbff;
    }

    #status-bar {
        height: 3;
        padding: 0 1;
        border-bottom: solid #168aad;
        background: #041c2c;
        content-align: center middle;
    }

    #graph-controls {
        height: 3;
        padding: 0 1;
        background: #061721;
    }

    #graph-controls Select {
        width: 1fr;
        margin-right: 1;
    }

    #history {
        height: 2fr;
        min-height: 12;
        border: round #168aad;
        background: #020b12;
    }

    #lower {
        height: 2fr;
        min-height: 15;
    }

    #health-column {
        width: 2fr;
        min-width: 42;
    }

    .panel {
        border: round #1b6f8f;
        padding: 0 1;
        background: #03131d;
    }

    #node-health {
        height: 2fr;
    }

    #heartbeats {
        height: 7;
    }

    #calibration {
        height: 6;
    }

    #raw-log {
        width: 3fr;
        border: round #1b6f8f;
        background: #020d14;
        padding: 0 1;
    }

    #command-bar {
        height: 3;
        padding: 0 1;
        background: #061721;
    }

    #command-input {
        width: 1fr;
    }

    #send-command {
        width: 12;
        margin-left: 1;
    }

    Footer {
        background: #06334b;
    }
    """

    BINDINGS = [
        Binding("q", "quit", "Quit"),
        Binding("m", "cycle_mode", "Graph mode"),
        Binding("t", "cycle_window", "Time window"),
        Binding("n", "cycle_node", "Node"),
        Binding("o", "toggle_online", "Online/offline"),
        Binding("u", "focus_manual", "Manual"),
        Binding("left", "step_node(1,-5)", "N1 -5°", show=False),
        Binding("right", "step_node(1,5)", "N1 +5°", show=False),
        Binding("a", "step_node(2,-5)", "N2 -5°", show=False),
        Binding("d", "step_node(2,5)", "N2 +5°", show=False),
    ]

    def __init__(self, controller: "AntennaController") -> None:
        super().__init__()
        self.controller = controller
        self.history: deque[HistorySample] = deque(maxlen=3600)
        self.heartbeat_history: dict[tuple[int, str], deque[bool]] = {
            (node, route): deque(maxlen=30)
            for node in (1, 2)
            for route in ("gateway", "backup")
        }
        self.mode = "track"
        self.window_s = 60
        self.node_filter = "all"
        self.log_filter = "all"
        self._last_state_stamp: tuple[int, int] = (0, 0)
        self._last_event_sequence = 0

    def compose(self) -> ComposeResult:
        yield Header(show_clock=True)
        yield Static(id="status-bar")
        with Horizontal(id="graph-controls"):
            yield Select(
                [(label.upper(), label) for label in GRAPH_MODES],
                value="track",
                allow_blank=False,
                id="mode-select",
            )
            yield Select(
                [
                    ("30 seconds", 30),
                    ("60 seconds", 60),
                    ("2 minutes", 120),
                    ("5 minutes", 300),
                    ("15 minutes", 900),
                ],
                value=60,
                allow_blank=False,
                id="window-select",
            )
            yield Select(
                [(label.title(), label) for label in NODE_FILTERS],
                value="all",
                allow_blank=False,
                id="node-select",
            )
            yield Select(
                [(f"Log: {label.title()}", label) for label in LOG_FILTERS],
                value="all",
                allow_blank=False,
                id="log-select",
            )
        yield PlotextPlot(id="history")
        with Horizontal(id="lower"):
            with Vertical(id="health-column"):
                yield Static(id="node-health", classes="panel")
                yield Static(id="heartbeats", classes="panel")
                yield Static(id="calibration", classes="panel")
            yield RichLog(id="raw-log", markup=True, wrap=False, highlight=False)
        with Horizontal(id="command-bar"):
            yield Input(
                placeholder=(
                    "Manual: goto 2 180 | step 2 -5 | stop 2 | point 90 | park"
                ),
                id="command-input",
            )
            yield Button("SEND", id="send-command", variant="primary")
        yield Footer()

    def on_mount(self) -> None:
        self.set_interval(0.5, self.refresh_dashboard)
        self.refresh_dashboard()

    def refresh_dashboard(self) -> None:
        states = self.controller.states.snapshot()
        stamp = tuple(state.measured_utc_ms for state in states)
        if stamp != self._last_state_stamp:
            self.history.append(HistorySample(time.monotonic(), states))
            self._last_state_stamp = stamp

        for key, pulses in self.heartbeat_history.items():
            pulses.append(self.controller.states.route_fresh(*key))

        self._update_status(states)
        self._update_health(states)
        self._update_heartbeats()
        self._update_calibration(states)
        self._update_events()
        self._update_plot()

    def _update_status(self, states: list["NodeState"]) -> None:
        now_ms = int(time.time() * 1000)
        online_nodes = sum(
            state.measured_utc_ms > 0
            and now_ms - state.measured_utc_ms < 5000
            for state in states
        )
        serial_ok = self.controller.primary.connected.is_set()
        espnow_ok = any(
            self.controller.states.route_fresh(node, "gateway")
            for node in (1, 2)
        )
        backup_ok = (
            self.controller.backup is not None
            and self.controller.backup.connected.is_set()
        )
        overall = (
            "[bold green]ONLINE[/]"
            if serial_ok and online_nodes == 2
            else "[bold yellow]PARTIAL[/]"
            if serial_ok or online_nodes
            else "[bold red]OFFLINE[/]"
        )
        command_mode = (
            "[green]ONLINE[/]"
            if self.controller.accept_external_commands
            else "[yellow]LOCAL ONLY[/]"
        )
        text = (
            f"[bold cyan]ANTENNACTL v0.2[/]  {overall}  "
            f"Nodes: [bold]{online_nodes}/2[/]  "
            f"Clients: [bold]{self.controller.client_count}[/]  "
            f"ESP-NOW: {self._ok_text(espnow_ok)}  "
            f"SERIAL: {self._ok_text(serial_ok)}  "
            f"BACKUP: {self._ok_text(backup_ok, optional=self.controller.backup is None)}  "
            f"External: {command_mode}  "
            f"[bold]{time.strftime('%H:%M:%S UTC', time.gmtime())}[/]"
        )
        self.query_one("#status-bar", Static).update(text)

    def _update_health(self, states: list["NodeState"]) -> None:
        lines = ["[bold cyan]NODE STATUS / GEOMETRY[/]"]
        now_ms = int(time.time() * 1000)
        for state in states:
            age = (
                None
                if not state.measured_utc_ms
                else max(0.0, (now_ms - state.measured_utc_ms) / 1000.0)
            )
            age_text = "---" if age is None else f"{age:4.1f}s"
            heading = "---" if state.heading_deg is None else f"{state.heading_deg:6.1f}°"
            target = "---" if state.target_deg is None else f"{state.target_deg:6.1f}°"
            routes = [
                label
                for label, route in (("ESP-NOW", "gateway"), ("PI", "backup"))
                if self.controller.states.route_fresh(state.node_id, route)
            ]
            route_text = "+".join(routes) if routes else "NONE"
            rssi = (
                "---"
                if state.rssi_at_gateway_dbm is None
                else f"{state.rssi_at_gateway_dbm:.0f} dBm"
            )
            condition = (
                "[green]TRACKING[/]"
                if state.moving
                else "[green]LOCKED[/]"
                if age is not None and age < 5
                else "[red]LOST[/]"
            )
            lines.extend(
                [
                    f"[bold]Node {state.node_id}[/]  {condition}  "
                    f"last={age_text}  route={route_text}",
                    f"  Heading {heading}   Target {target}   RSSI {rssi}",
                ]
            )
        n1 = states[0].heading_deg
        n2 = states[1].heading_deg
        lines.append(
            "[dim]Geometry[/]  "
            f"N1 {self._arrow(n1)}  •  {self._arrow(n2)} N2"
        )
        self.query_one("#node-health", Static).update("\n".join(lines))

    def _update_heartbeats(self) -> None:
        lines = ["[bold cyan]LINK HEARTBEATS[/]"]
        for node, route, label in (
            (1, "gateway", "N1 ESP-NOW"),
            (2, "gateway", "N2 ESP-NOW"),
            (2, "backup", "N2 PI relay"),
        ):
            pulses = self.heartbeat_history[(node, route)]
            bar = "".join("[green]█[/]" if pulse else "[#33444c]░[/]" for pulse in pulses)
            lines.append(f"{label:12s} {bar}")
        self.query_one("#heartbeats", Static).update("\n".join(lines))

    def _update_calibration(self, states: list["NodeState"]) -> None:
        sensor = states[1]
        magnetic = self._vector_text(sensor.magnetic_ut, 1, "µT")
        acceleration = self._vector_text(sensor.acceleration_g, 3, "g")
        status = (
            "[yellow]UNCALIBRATED[/]"
            if sensor.sensor_flags & 0x08
            else "[red]NO SENSOR[/]"
        )
        self.query_one("#calibration", Static).update(
            "[bold cyan]N2 SENSOR / CALIBRATION[/]  "
            + status
            + "\n"
            + f"Bxyz {magnetic}\n"
            + f"Axyz {acceleration}\n"
            + "[dim]Hard/soft-iron and mounting calibration pending[/]"
        )

    def _update_events(self) -> None:
        log = self.query_one("#raw-log", RichLog)
        for event in self.controller.recent_events(self._last_event_sequence):
            self._last_event_sequence = max(
                self._last_event_sequence, int(event["sequence"])
            )
            event_type = str(event["type"]).lower()
            if self.log_filter != "all" and event_type != self.log_filter:
                continue
            color = {
                "report": "green",
                "sensor": "cyan",
                "client": "blue",
                "command": "yellow",
                "ack": "magenta",
                "error": "red",
                "system": "orange1",
            }.get(event_type, "grey62")
            stamp = time.strftime("%H:%M:%S", time.gmtime(float(event["utc"])))
            log.write(
                f"[dim]{stamp}[/] [{color}]{event['type']:<7}[/] "
                f"[bold]{escape(str(event['source']))}[/] "
                f"[dim]{escape(str(event['route']))}[/] "
                f"{escape(str(event['detail']))}"
            )

    def _update_plot(self) -> None:
        plot_widget = self.query_one("#history", PlotextPlot)
        plot = plot_widget.plt
        plot.clear_data()
        plot.grid(horizontal=True, vertical=True)
        plot.xlabel("seconds ago")
        cutoff = time.monotonic() - self.window_s
        samples = [sample for sample in self.history if sample.monotonic_s >= cutoff]
        now = time.monotonic()

        if self.mode == "track":
            plot.title("ANTENNA TRACKING — actual and commanded azimuth")
            plot.ylabel("degrees true")
            plot.ylim(0, 360)
            self._plot_state_value(plot, samples, now, 1, "heading_deg", "green", "N1 actual")
            self._plot_state_value(plot, samples, now, 1, "target_deg", "yellow", "N1 target")
            self._plot_state_value(plot, samples, now, 2, "heading_deg", "blue+", "N2 actual")
            self._plot_state_value(plot, samples, now, 2, "target_deg", "magenta", "N2 target")
        elif self.mode == "magnetometer":
            plot.title("MAGNETOMETER — raw Bx / By / Bz")
            plot.ylabel("µT")
            self._plot_vectors(plot, samples, now, "magnetic_ut")
        elif self.mode == "accelerometer":
            plot.title("ACCELEROMETER — raw Ax / Ay / Az")
            plot.ylabel("g")
            self._plot_vectors(plot, samples, now, "acceleration_g")
        else:
            plot.title("ESP-NOW RSSI — received at head / received at node")
            plot.ylabel("dBm")
            self._plot_state_value(
                plot, samples, now, 1, "rssi_at_gateway_dbm", "green", "N1→head"
            )
            self._plot_state_value(
                plot, samples, now, 1, "rssi_at_node_dbm", "yellow", "head→N1"
            )
            self._plot_state_value(
                plot, samples, now, 2, "rssi_at_gateway_dbm", "blue+", "N2→head"
            )
            self._plot_state_value(
                plot, samples, now, 2, "rssi_at_node_dbm", "magenta", "head→N2"
            )

        plot.xlim(-self.window_s, 0)
        plot_widget.refresh()

    def _plot_state_value(
        self,
        plot: object,
        samples: list[HistorySample],
        now: float,
        node_id: int,
        attribute: str,
        color: str,
        label: str,
    ) -> None:
        if not self._node_selected(node_id):
            return
        values = [
            (sample.monotonic_s - now, getattr(sample.nodes[node_id - 1], attribute))
            for sample in samples
        ]
        finite = [(x, y) for x, y in values if y is not None]
        if finite:
            plot.plot(
                [item[0] for item in finite],
                [item[1] for item in finite],
                color=color,
                label=label,
            )

    def _plot_vectors(
        self,
        plot: object,
        samples: list[HistorySample],
        now: float,
        attribute: str,
    ) -> None:
        colors = ("red+", "green+", "blue+")
        axes = ("x", "y", "z")
        for node_id in (1, 2):
            if not self._node_selected(node_id):
                continue
            for axis, color in enumerate(colors):
                values = [
                    (
                        sample.monotonic_s - now,
                        getattr(sample.nodes[node_id - 1], attribute),
                    )
                    for sample in samples
                ]
                finite = [
                    (x, vector[axis])
                    for x, vector in values
                    if vector is not None
                ]
                if finite:
                    plot.plot(
                        [item[0] for item in finite],
                        [item[1] for item in finite],
                        color=color,
                        label=f"N{node_id} {axes[axis]}",
                    )

    def _node_selected(self, node_id: int) -> bool:
        return self.node_filter == "all" or self.node_filter == f"node {node_id}"

    @on(Select.Changed, "#mode-select")
    def mode_changed(self, event: Select.Changed) -> None:
        if isinstance(event.value, str):
            self.mode = event.value
            self._update_plot()

    @on(Select.Changed, "#window-select")
    def window_changed(self, event: Select.Changed) -> None:
        if isinstance(event.value, int):
            self.window_s = event.value
            self._update_plot()

    @on(Select.Changed, "#node-select")
    def node_changed(self, event: Select.Changed) -> None:
        if isinstance(event.value, str):
            self.node_filter = event.value
            self._update_plot()

    @on(Select.Changed, "#log-select")
    def log_filter_changed(self, event: Select.Changed) -> None:
        if isinstance(event.value, str):
            self.log_filter = event.value
            log = self.query_one("#raw-log", RichLog)
            log.clear()
            self._last_event_sequence = 0
            self._update_events()

    @on(Button.Pressed, "#send-command")
    def send_button_pressed(self) -> None:
        self._submit_manual_command()

    @on(Input.Submitted, "#command-input")
    def command_submitted(self) -> None:
        self._submit_manual_command()

    def _submit_manual_command(self) -> None:
        command_input = self.query_one("#command-input", Input)
        command = command_input.value.strip()
        if not command:
            return
        command_input.value = ""
        try:
            result = self._execute_manual_command(command)
            self.notify(result, title="Antenna command", severity="information")
        except (ValueError, RuntimeError) as exc:
            self.controller._record_event(
                "ERROR", str(exc), source="operator", route="local"
            )
            self.notify(str(exc), title="Command rejected", severity="error")

    def _execute_manual_command(self, command: str) -> str:
        fields = shlex.split(command.lower())
        if not fields:
            raise ValueError("empty command")
        if fields[0] == "goto" and len(fields) == 3:
            route = self.controller._send_rotator_command(
                int(fields[1]), "goto", azimuth_deg=float(fields[2])
            )
            return f"Node {fields[1]} goto accepted via {route}"
        if fields[0] == "step" and len(fields) == 3:
            route = self.controller._send_rotator_command(
                int(fields[1]), "step", delta_deg=float(fields[2])
            )
            return f"Node {fields[1]} step accepted via {route}"
        if fields[0] == "stop" and len(fields) == 2:
            route = self.controller._send_rotator_command(
                int(fields[1]), "stop"
            )
            return f"Node {fields[1]} stop accepted via {route}"
        if fields[0] == "point" and len(fields) == 2:
            target = float(fields[1])
            routes = [
                self.controller._send_rotator_command(
                    node, "goto", azimuth_deg=target
                )
                for node in (1, 2)
            ]
            return f"Both nodes commanded via {', '.join(routes)}"
        if fields[0] == "park" and len(fields) == 1:
            routes = [
                self.controller._send_rotator_command(
                    node, "goto", azimuth_deg=0.0
                )
                for node in (1, 2)
            ]
            return f"Park commanded via {', '.join(routes)}"
        if fields[0] in {"online", "offline"} and len(fields) == 1:
            self.controller.set_online(fields[0] == "online")
            return f"External commands {fields[0]}"
        raise ValueError(
            "use: goto <node> <deg>, step <node> <deg>, stop <node>, "
            "point <deg>, park, online, or offline"
        )

    def action_cycle_mode(self) -> None:
        index = (GRAPH_MODES.index(self.mode) + 1) % len(GRAPH_MODES)
        self.query_one("#mode-select", Select).value = GRAPH_MODES[index]

    def action_cycle_window(self) -> None:
        index = (TIME_WINDOWS.index(self.window_s) + 1) % len(TIME_WINDOWS)
        self.query_one("#window-select", Select).value = TIME_WINDOWS[index]

    def action_cycle_node(self) -> None:
        index = (NODE_FILTERS.index(self.node_filter) + 1) % len(NODE_FILTERS)
        self.query_one("#node-select", Select).value = NODE_FILTERS[index]

    def action_toggle_online(self) -> None:
        self.controller.set_online(not self.controller.accept_external_commands)
        self.notify(
            "External commands "
            + ("enabled" if self.controller.accept_external_commands else "disabled")
        )

    def action_focus_manual(self) -> None:
        self.query_one("#command-input", Input).focus()

    def action_step_node(self, node_id: int, delta_deg: float) -> None:
        try:
            self.controller._send_rotator_command(
                int(node_id), "step", delta_deg=float(delta_deg)
            )
        except (ValueError, RuntimeError) as exc:
            self.notify(str(exc), severity="error")

    @staticmethod
    def _ok_text(value: bool, *, optional: bool = False) -> str:
        if value:
            return "[green]OK[/]"
        return "[dim]N/A[/]" if optional else "[red]LOST[/]"

    @staticmethod
    def _vector_text(
        vector: list[float] | None, precision: int, unit: str
    ) -> str:
        if vector is None:
            return "---"
        return (
            f"({vector[0]:.{precision}f}, {vector[1]:.{precision}f}, "
            f"{vector[2]:.{precision}f}) {unit}"
        )

    @staticmethod
    def _arrow(heading: float | None) -> str:
        if heading is None:
            return "?"
        arrows = ("↑", "↗", "→", "↘", "↓", "↙", "←", "↖")
        return f"{arrows[int((heading + 22.5) // 45) % 8]} {heading:.0f}°"


def run_textual_tui(controller: "AntennaController") -> None:
    """Run the interactive Textual dashboard until the operator exits."""
    root_logger = logging.getLogger()
    console_handlers = [
        handler
        for handler in root_logger.handlers
        if isinstance(handler, logging.StreamHandler)
        and not isinstance(handler, logging.FileHandler)
    ]
    tui_handler = _TextualEventHandler(controller)
    for handler in console_handlers:
        root_logger.removeHandler(handler)
    root_logger.addHandler(tui_handler)
    controller.start()
    try:
        AntennaControllerApp(controller).run()
    finally:
        controller.close()
        root_logger.removeHandler(tui_handler)
        for handler in console_handlers:
            root_logger.addHandler(handler)


class _TextualEventHandler(logging.Handler):
    """Send process logging to the TUI event pane instead of stderr."""

    def __init__(self, controller: "AntennaController") -> None:
        super().__init__()
        self.controller = controller

    def emit(self, record: logging.LogRecord) -> None:
        try:
            event_type = "ERROR" if record.levelno >= logging.ERROR else "SYSTEM"
            self.controller._record_event(
                event_type,
                self.format(record),
                source="system",
                route="process",
            )
        except Exception:
            self.handleError(record)
