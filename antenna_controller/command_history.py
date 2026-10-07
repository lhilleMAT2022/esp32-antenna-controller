"""Session command journal and portable, startup-relative CSV replay."""
from contextlib import contextmanager
from contextvars import ContextVar
import csv
from dataclasses import dataclass
from datetime import datetime, timezone
import math
from pathlib import Path
import shlex
import threading
import time

from .slew import parse_bearing

CSV_COLUMNS = ("utc_time", "elapsed_seconds", "command")


@dataclass(frozen=True)
class CommandEntry:
    utc: float | None
    elapsed: float
    command: str

    def csv_row(self):
        utc = datetime.fromtimestamp(self.utc, timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z")
        return (utc, f"{self.elapsed:.6f}", self.command)


@dataclass
class _Action:
    command: str
    recorded: bool = False


class CommandHistory:
    def __init__(self):
        self.started_utc = time.time()
        self.started_monotonic = time.monotonic()
        self.filename = f"antenna_commands_{int(self.started_utc)}.csv"
        self._entries: list[CommandEntry] = []
        self._recall: list[str] = []
        self._lock = threading.RLock()
        self._action = ContextVar("command_action", default=None)
        self._file = None
        self._writer = None
        self.path: Path | None = None
        self.save_error = ""

    def elapsed(self):
        return time.monotonic() - self.started_monotonic

    def entries(self, after=0):
        with self._lock:
            return list(self._entries[after:])

    def recall_entries(self):
        with self._lock:
            return list(self._recall)

    def remember_rejected(self, command):
        with self._lock:
            self._recall.append(command)

    def start_csv(self, directory=None):
        """Create a session file without overwriting an existing recording."""
        with self._lock:
            if self._file is not None:
                return self.path
            self.path = (Path(directory) if directory is not None else Path.cwd()) / self.filename
            self._file = self.path.open("x", newline="", encoding="utf-8")
            self._writer = csv.writer(self._file)
            self._writer.writerow(CSV_COLUMNS)
            self._writer.writerows(entry.csv_row() for entry in self._entries)
            self._file.flush()
            self.save_error = ""
            return self.path

    def save(self):
        """Flush automatic recording, or retry saving all in-memory entries."""
        with self._lock:
            if self._file is None:
                return self.start_csv(self.path.parent if self.path else Path.cwd())
            # Rewrite our own session file, also recovering any earlier failed append.
            self._file.seek(0)
            self._file.truncate()
            self._writer.writerow(CSV_COLUMNS)
            self._writer.writerows(entry.csv_row() for entry in self._entries)
            self._file.flush()
            self.save_error = ""
            return self.path

    def close(self):
        with self._lock:
            if self._file:
                self._file.close()
                self._file = None

    @contextmanager
    def action(self, command, *, local=False):
        """Group generated writes under one replayable operator command.

        Context-local grouping keeps concurrent TCP commands independent. A
        partial multi-node send is retained even if a later send fails.
        """
        action = self._action.get()
        token = None
        if action is None:
            action = _Action(command)
            token = self._action.set(action)
        try:
            yield
            if local:
                self.sent()
        finally:
            if token is not None:
                self._action.reset(token)

    def sent(self):
        action = self._action.get()
        if action is None or action.recorded:
            return
        with self._lock:
            action.recorded = True
            entry = CommandEntry(time.time(), self.elapsed(), action.command)
            self._entries.append(entry)
            self._recall.append(action.command)
            if self._writer and self._file:
                try:
                    self._writer.writerow(entry.csv_row())
                    self._file.flush()
                except OSError as exc:
                    # A recording failure must not turn a completed send into a
                    # reported command failure and encourage a duplicate move.
                    self.save_error = str(exc)


def validate_command(command: str):
    """Validate replay syntax before starting any links or executing any row."""
    prefix = command.strip().split(maxsplit=1)
    if prefix and prefix[0].lower() == "dump":
        from .event_log import parse_dump_path
        parse_dump_path(command)
        return
    fields = shlex.split(command.lower())
    if not fields:
        raise ValueError("Empty command")
    verb = fields[0]
    if verb in ("quit", "park", "online", "offline") and len(fields) == 1:
        return
    if verb == "point":
        parse_bearing(fields[1:])
        return
    if verb == "goto" and len(fields) >= 3 and fields[1] in ("1", "2"):
        parse_bearing(fields[2:])
        return
    if verb == "stop" and len(fields) == 2 and fields[1] in ("1", "2"):
        return
    if verb == "step" and len(fields) == 3 and fields[1] in ("1", "2"):
        if math.isfinite(float(fields[2])):
            return
    if verb in ("report", "reporting") and len(fields) in (2, 3):
        if fields[1] in ("continuous", "normal") and (len(fields) == 2 or int(fields[2]) == 0):
            return
        if fields[1] in ("quiet", "silent") and len(fields) == 3 and 1 <= int(fields[2]) <= 86400:
            return
    if verb == "cal" and len(fields) >= 3 and fields[1] in ("1", "2"):
        action, args = fields[2], fields[3:]
        if action in ("status", "save", "clear", "cancel", "export") and not args:
            return
        if action == "start" and args == ["mag"]:
            return
        if action == "face" and len(args) == 1 and args[0] in ("+x", "-x", "+y", "-y", "+z", "-z"):
            return
        if action in ("fit", "apply") and args in (["mag"], ["accel"]):
            return
        if action == "align" and len(args) == 3:
            if (all(a in ("+x", "-x", "+y", "-y", "+z", "-z") for a in args[:2])
                    and args[0][1] != args[1][1] and math.isfinite(float(args[2])) and abs(float(args[2])) <= 180):
                return
    raise ValueError("Invalid command syntax; use goto, point, step, stop, park, cal, report, online, offline, dump, or quit")


def load_history(path):
    entries = []
    with Path(path).open(newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames != list(CSV_COLUMNS):
            raise ValueError("CSV columns must be: " + ",".join(CSV_COLUMNS))
        for line, row in enumerate(reader, 2):
            try:
                if None in row or any(value is None for value in row.values()):
                    raise ValueError("Expected exactly three columns")
                elapsed = float(row["elapsed_seconds"])
                if not math.isfinite(elapsed) or elapsed < 0 or entries and elapsed < entries[-1].elapsed:
                    raise ValueError("Elapsed seconds must be finite, nonnegative, and nondecreasing")
                utc = None
                if row["utc_time"].strip():
                    stamp = datetime.fromisoformat(row["utc_time"].replace("Z", "+00:00"))
                    if stamp.tzinfo is None:
                        raise ValueError("UTC time must include Z or a timezone offset")
                    utc = stamp.timestamp()
                command = row["command"].strip()
                validate_command(command)
                if entries and entries[-1].command.lower() == "quit":
                    raise ValueError("quit must be the last command")
                entries.append(CommandEntry(utc, elapsed, command))
            except (ValueError, OverflowError) as exc:
                raise ValueError(f"CSV row {line}: {exc}") from exc
    return entries


def replay_command(entry: CommandEntry, startup_utc: float):
    """Shift recorded absolute slew starts into this session's timeline."""
    if entry.command.strip().split(maxsplit=1)[0].lower() == "dump":
        return entry.command
    fields = shlex.split(entry.command)
    start_index = 1 if fields[0].lower() == "point" else 2 if fields[0].lower() == "goto" else None
    if start_index is not None and len(fields) > start_index + 1 and not fields[start_index].startswith("+") and entry.utc is not None:
        original = int(fields[start_index])
        fields[start_index] = str(math.ceil(startup_utc + entry.elapsed + original - entry.utc))
        return " ".join(fields)
    return entry.command


class CommandReplay:
    """A nonblocking dispatcher, driven by the TUI timer and monotonic time."""
    def __init__(self, entries, history):
        self.entries = entries
        self.history = history
        self.index = 0
        self.error = ""
        self.cancelled = False

    def tick(self, execute):
        if self.error or self.cancelled:
            return
        while self.index < len(self.entries) and self.entries[self.index].elapsed <= self.history.elapsed():
            entry = self.entries[self.index]
            try:
                command = replay_command(entry, self.history.started_utc)
                execute(command)
            except (ValueError, RuntimeError, OSError) as exc:
                self.error = f"Replay stopped at row {self.index + 2}: {entry.command}: {exc}"
                return
            self.index += 1
            if entry.command.lower() == "quit":
                return

    def status(self):
        if self.error:
            return self.error
        if self.cancelled:
            return "Replay cancelled"
        return f"Replay: {self.index}/{len(self.entries)} commands" if self.entries else "No replay loaded"
