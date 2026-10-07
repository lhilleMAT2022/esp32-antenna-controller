# Command recording and replay

The Python TUI's **Command History** tab sits beside Health and Calibration.
It shows each command with an `hh:mm:ss UTC` timestamp. Scroll with the mouse
wheel or focus the list and use Page Up/Down. New entries follow the bottom
only when you are already there, preserving your position while reviewing.

Each TUI session automatically creates this file in the working directory:

```text
antenna_commands_<UTC_epoch_seconds_at_startup>.csv
```

Rows are flushed after each command. **Save CSV** also saves the current
history and displays the full path. Recordings are ignored by Git. An existing
file with the same name is never overwritten by a new session. If writing
fails, the tab displays the error and retains the history in memory.

The three columns are:

| Column | Meaning |
| --- | --- |
| `utc_time` | Full UTC ISO 8601 timestamp with milliseconds and `Z` |
| `elapsed_seconds` | Monotonic seconds since controller startup, including fractions |
| `command` | Replayable operator command |

The history includes manual commands, left/right and A/D moves, calibration
commands (including local capture/fit/export actions), the Reporting dialog,
online/offline changes, external TCP movement commands, and quit. A manual
command is recorded once even if it sends to both nodes. Generated operations
are grouped under the originating command: for example, calibration's switch
to continuous reporting is included in `cal 2 start mag`, avoiding duplicate
reporting requests on replay. Periodic clock synchronization and telemetry are
housekeeping and do not become operator command rows.

A row records a successful host send or local action, not node acceptance or
physical completion. Partially sent multi-node commands are retained; failed
commands with no send/local action remain in the existing error log. Consult
the ACK, Slew and calibration logs for node outcomes.

## Replay a recording

```powershell
uv run python -m antenna_controller ac --serial COM10 --backup-host 127.0.0.1 --tui --command-file .\antenna_commands_1791395489.csv
```

The entire CSV is checked for syntax and ordered times before opening the
hardware links. Execution uses elapsed seconds from the new controller startup,
not the original date. Rows sharing a time execute in file order. The TUI
checks for due commands every 50 ms, so this is host scheduling, not a precise
real-time sequencer. Remote polynomial slews still execute on node UTC clocks.

`+seconds` starts are evaluated when their command runs. Recorded absolute
UTC slew starts shift to the new session's timeline, retaining the original
offset from the recorded command. For handwritten sequences, leave `utc_time`
blank and use `+seconds` for convenient future moves. A blank UTC field leaves
any absolute slew start unchanged.

Example handwritten CSV:

```csv
utc_time,elapsed_seconds,command
,5,report normal
,10,point +20 300 -0.5 dur 30 step 3
,70,quit
```

This requests normal reporting at startup +5 s, submits the plan at +10 s,
starts its targets around +30 s, finishes sending targets around +60 s, and
exits the TUI at +70 s. Allow time for serial connections and node UTC sync
before scheduling moves. Fractional timestamps and blank UTC fields are
supported; elapsed times must be nonnegative and nondecreasing.

A host validation or transport exception stops further replay and displays
the failed row. It does not retry or roll back commands already sent. Node
acknowledgments are asynchronous and do not delay the CSV timeline; leave
adequate spacing for reporting/calibration acknowledgments and check their
outcomes in the normal logs. Calibration replay still requires the expected
sensor movement, captured samples, and successful fits.

**Cancel replay** discards remaining CSV actions without changing node plans.
Use normal `stop <node>` commands if you also want to stop node movement.
Without a final `quit`, the TUI stays open when replay ends. A `quit` row must
be last; it closes the host application and links without sending a node stop
or reboot. Every replay creates its own new command recording.

## Manual recall and closing help

In the manual entry field, **Up/Down** walks through this session's submitted
commands from all sources: manual entries, keyboard steps, reporting controls,
calibration and external clients. Repeated steps remain separate entries;
manual sends are not duplicated. Rejected manual entries can also be recalled
and corrected. Moving
past the newest entry restores the draft you were typing. Recall never sends
a command until Enter or SEND. Left/Right and A/D edit text while the field has
focus; outside text entry they retain their node-movement bindings.

**Escape** closes the Ctrl+P command palette, key guide, or an open dialog.
The key guide also has a visible **Close keys (Esc)** button. `quit` in the
manual field has the same effect as the normal application quit action.

## Event log export

Use the manual command field to snapshot the right-hand scrolling log:

```text
dump
dump Diagnostics.CSV
dump "C:\Users\lhille\My Logs\antenna-check.csv"
```

Without a filename, the file is `antenna_events_<startup_epoch>.csv` in the
working directory, using the same startup epoch as the command recording.
Explicit filenames retain their case and extension; the contents are always
CSV. Quotes allow filenames with spaces. Parent directories must already
exist. Repeating a dump to the same filename replaces that snapshot; the
active command-history recording is protected from accidental overwrite.

The export contains the pane's retained history, not just the rows currently
visible in the viewport. It respects the selected **Log** filter; select
**Log: All** before dumping if you want all retained event types. Pending
events are added before taking the snapshot.

The columns match the command-history CSV exactly: `utc_time`,
`elapsed_seconds`, `command`. For event exports, the third column contains
the event's type, source, route, and message. UTC includes the full date and
milliseconds; elapsed time uses the monotonic clock at event creation, so PC
clock corrections do not disturb that column. CSV quoting preserves commas,
quotes, and Unicode. Multiline messages become one line with ` | ` separators.
Event exports are diagnostic records, not executable command sequences.
The `dump` action itself is recorded in command history and can be used in a
replay sequence, for example immediately before `quit`.

Both retention layers are bounded: the controller keeps the newest **10,000
events**, and the right-hand widget keeps **10,000 lines**, with oldest entries
discarded automatically. Each event occupies one unwrapped line. A filtered
pane can retain older matching events while other event types arrive; changing
the filter rebuilds it from the controller's latest 10,000 events. Dumps cannot
recover events already discarded. The earlier controller limit was 500 events,
while the scrolling widget previously had no explicit line limit.
