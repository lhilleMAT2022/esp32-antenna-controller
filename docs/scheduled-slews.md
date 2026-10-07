# Scheduled bearings and polynomial slews

The TUI manual commands retain their immediate forms:

```text
point 300
goto 2 300
```

`point` targets both nodes; `goto` targets the specified node. With additional
bearing values, the syntax is:

```text
point <start_epoch_s> C [X1 X2 ...] [dur <seconds>] [step <count>]
goto <node> <start_epoch_s> C [X1 X2 ...] [dur <seconds>] [step <count>]
```

The start is an **absolute integer UTC epoch second**, not a relative delay.
It must be at least one second in the future when submitted by the host, and
still future when accepted by the node. Defaults are `dur 30 step 3`.

At `t = UTC - start`, the requested bearing is
`C + X1*t + X2*t² + ...`. For duration D and step count N, the node samples the
polynomial at `t = 0, D/N, 2D/N, ..., D`. Thus there is one initial target and
N subsequent increments, including the endpoint. Evaluation uses double
precision and each resulting bearing wraps into [0°,360°).

Examples (replace the illustrative timestamp with a future time):

```text
point 1791395489 300 -0.5
point 1791395489 0 2 dur 60 step 30
goto 1 1791395489 350 2 0.1 dur 30 step 3
```

- First: targets 300°, 295°, 290°, 285° at 0, 10, 20, 30 seconds.
- Second: targets 0° through 120°, in **30 increments of 4° every 2 seconds**.
  The literal `dur 60` determines the duration. To request 12° every 4 seconds
  for two minutes, use `0 3 dur 120 step 30` instead.
- Third: a quadratic; its 10-second target is 20° after bearing wrap.

To obtain a start timestamp 30 seconds from now in PowerShell:

```powershell
[DateTimeOffset]::UtcNow.ToUnixTimeSeconds() + 30
```

## Execution and operator behavior

The whole plan is sent once to each node. It executes from the node's UTC
clock even if the host disconnects; there are no per-step RF commands. Existing
normal/quiet reporting schedules still apply. Planned movement reporting stays
active throughout a running plan, including pauses between target updates.

Each target invokes the existing position controller. This is a stepped target
schedule, not a guarantee of constant physical angular velocity or arrival at
each target on time. Current firmware drives only the non-actuating model.
At the end of the timeline, the node reports `targets_sent`; actual position
can still be approaching the last target. The existing movement/completion
reports show subsequent settling. Bearing wrap does not command a particular
multi-turn mechanical path; the existing reachable-position logic still applies.

Select **Log: Slew** or **Log: All** to see per-node acceptance, rejection,
start, final-target and cancellation messages. A successful host write is only
a request. Missing node acknowledgment is reported as outcome unknown after
approximately five seconds. For `point`, the two node acceptances are separate;
one can accept while the other rejects. Stop an accepted node if partial
execution is unwanted.

`stop`, an immediate `goto`/`point`, a manual `step`, and `park` cancel all
pending/running schedules on each affected node before issuing the immediate
action. No schedule is saved to flash; a reboot clears the queue.

Limits: four non-overlapping plans per node; up to six finite coefficients
(constant through fifth order), integer duration 1–86400 s, 1–3600 increments,
and at least 100 ms between targets. Touching endpoints count as overlap.
All target samples are validated before queueing. The legacy four-entry
single-target queue cannot be mixed with polynomial plans.

The last eight accepted request IDs and payloads are deduplicated in RAM.
Identical retransmissions are acknowledged without restarting, including
after completion/cancellation. Conflicting reuse is rejected. Clock steps
forward skip outdated intermediate targets; backward steps never replay an
already-issued increment. Startup without valid UTC rejects scheduled work.

## Transport contract

Protocol **6** requires matched CYD and node firmware. The existing status
Packet remains 62 bytes. New packed little-endian messages:

- Type 10, 70-byte `SlewCommand`: `type:u8,node:u8,version:u8,count:u8,
  request:u32,startMs:u64,durationMs:u32,steps:u16,coefficients:f64[6]`.
- Type 11, 9-byte `SlewReply`: `type:u8,node:u8,version:u8,phase:u8,
  request:u32,error:u8`. Phase 0/1/2/3/4 means accepted/started/targets_sent/
  cancelled/rejected. Node receive buffers are 70 bytes.

Compact serial and optional Pi relay JSON:

```json
{"t":"sc","n":1,"q":42,"at":1791395489,"coef":[300,-0.5],"dur":30,"steps":3}
{"t":"sa","n":1,"q":42,"phase":"accepted","e":0,"src":"espnow"}
```

Errors: 0 ok, 1 invalid plan, 2 unsynchronized UTC, 3 past start, 4 queue full,
5 overlap, 6 conflicting request ID, 7 legacy queue occupied, 255 gateway send
failure. Rejected syntax can produce a generic `ra` error; only a correlated
node `sa` confirms scheduling. Start/end/cancel lifecycle replies are immediate.
The Pi forwards `sc` plus the existing `rc`, `cc`, `rm`, `gt` commands.
The system TCP `antenna_command` and UDP `antenna_state` schemas are unchanged;
the extended syntax belongs to manual TUI commands and the compact node link.

## Verification

Host/native tests cover syntax/defaults, examples, non-finite input, polynomial
evaluation, wrapping, future-only scheduling, independent node acknowledgments,
queue limits, overlap/replay, cancellation and clock jumps. The live TUI parser
was checked for both scheduled command forms and unchanged immediate forms.

`tests/hardware_slew_smoke.py` issues targets to the non-actuating models and
checks scheduled start/end, target progression, retransmission, overlap
rejection, direct USB scheduling and stop cancellation. It leaves the models
stopped and nodes in normal reporting; it never applies/saves calibration.
