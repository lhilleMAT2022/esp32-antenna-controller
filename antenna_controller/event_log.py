"""Bounded event history formatting and CSV snapshots."""
import csv
from pathlib import Path

from .command_history import CSV_COLUMNS, CommandEntry

EVENT_HISTORY_LIMIT = 10_000


def parse_dump_path(command: str) -> str | None:
    """Preserve Windows backslashes and filename case; quotes are optional."""
    fields = command.strip().split(maxsplit=1)
    if len(fields) == 1:
        return None
    filename = fields[1].strip()
    if filename.startswith(('"', "'")):
        if len(filename) < 2 or filename[-1] != filename[0]:
            raise ValueError("Close the quote around the dump filename")
        filename = filename[1:-1]
    if not filename or any(c in filename for c in ('\0', '\r', '\n')):
        raise ValueError("Provide a nonempty dump filename on one line")
    return filename


def event_detail(event: dict) -> str:
    # Keep each event to one visual line so the row and widget limits agree.
    return " | ".join(str(event['detail']).splitlines())


def dump_events(events: list[dict], path: Path) -> int:
    with path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.writer(stream)
        writer.writerow(CSV_COLUMNS)
        for event in events:
            message = f"{event['type']:<7} {event['source']} {event['route']} {event_detail(event)}"
            writer.writerow(CommandEntry(event['utc'], event['elapsed_seconds'], message).csv_row())
    return len(events)
