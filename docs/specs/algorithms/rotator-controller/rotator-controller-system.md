# Rotator Controller v0.1 — System Specification

Runtime update (2026-10-07): [Node reporting and timing](../../../node-runtime.md)
specifies the current continuous/normal/quiet states, >10° unexpected motion,
quiet timer and movement exceptions, clocks, protocol 4, and verification.
It supersedes fixed telemetry rates in this original rotator-model document.

## Status: Draft

**Last updated:** 2026-10-05
**Scope:** Host AC process, CYD operator console, redundant node transport,
ESP-NOW command/status contract, and the non-actuating ESP32 node model.

## Purpose

The controller provides a local CYD view of both antennas, showing each node's
reported true azimuth and whether it is slewing. It accepts immediate and
time-queued azimuth commands from the CYD's USB serial connection. The host AC
accepts system commands on TCP 31988, publishes state on UDP 31989, and can
reach node 2 through an optional Pi serial relay on TCP 31995. Version 0.1
models the rotator in real time and never drives a relay.

## External interface

| Interface | Inputs | Outputs |
|---|---|---|
| USB serial → CYD | Time setting, immediate azimuth, relative step, queue, stop, overlap configuration, panel selection | Command result, node status and diagnostics |
| CYD ↔ node | UTC time synchronization, command packets | 1 Hz azimuth/status, command acknowledgment, directional RSSI |
| CYD display | Two node cards, a time-of-day header, motion indication, and touch controls | Debug panel selected over USB and touch keypad entry |
| RM → host AC | Newline-delimited `antenna_command` JSON on TCP 31988 | Per-command acceptance/error response |
| Host AC → RM/RC/RD | — | ICD `antenna_state` JSON datagrams on UDP 31989 |
| Host AC ↔ Pi relay | Compact JSON lines on TCP 31995 | Redundant node-2 command, status, and sensor path |
| Pi relay ↔ node 2 | Compact JSON lines over USB serial at 115200 baud | Direct node status, sensor telemetry, and command responses |

Angles are true bearings in degrees, clockwise from north. On the wire they are
signed integer tenths of a degree. UTC is represented as Unix epoch seconds.

## V0.1 commands

| USB command | Effect |
|---|---|
| `TIME 2026-09-01T12:03:06Z` | Sets CYD UTC and immediately synchronizes nodes |
| `AZ 1 49` | Requests node 1 move to 49.0° true |
| `STEP 2 -10` | Requests a 10° counter-clockwise true-bearing step on node 2 |
| `QUEUE 1 2026-09-01T12:05:00Z 100` | Queues node 1 to target 100.0° true at UTC time |
| `STOP 1` | Cancels the active target on node 1 |
| `OVERLAP 1 180` | Sets the true bearing at the lower mechanical limit / start of the overlap region |
| `PANEL HOME` / `PANEL DEBUG` | Selects the operator view or diagnostic panel |
| `KEYPAD 1`, `KEY 4`, `KEY 9`, `KEY OK` | Exercises the calculator-style new-azimuth mock-up through serial |
| `STATUS`, `HELP` | Prints current CYD/node state or command help |

The command interface is deliberately simple for bring-up. Production commands
will require encrypted peers, authenticated command origin, command sequence
replay handling, and a safety-confirmed local UI.

## Goals and acceptance criteria

| Goal | Acceptance criterion |
|---|---|
| Operator situational awareness | Home panel displays both nodes, a true azimuth, online state, and a flashing `* SLEWING` indicator during a model move. |
| Scheduled operation | A queued target does not start before its UTC timestamp and starts within 1 s after it, with a valid time source. |
| Model fidelity | Model maximum speed is 15°/s (2.5 rpm), reaches/stops from that speed in 2 s, limits travel to 0–370°, and reflects at either mechanical end. |
| Alternating button constraint | Each model button press alternates direction; the controller models a short toggle press when it must reverse the available next press direction. |
| No hardware actuation | No v0.1 source path controls a relay GPIO. `modelButtonPressed` is telemetry only. |
| Redundant node-2 path | With CYD and Pi links present, AC prefers CYD/ESP-NOW; it selects the Pi path only when the gateway path is stale or when explicitly forced for a bench test. |
| Machine-readable serial | CYD and both remote boards emit valid compact JSON status; node 2 additionally emits raw sensor telemetry. |

## Operating modes

| Mode | Behavior |
|---|---|
| Time invalid | Immediate commands work; queued commands are rejected and the display shows `UTC --:--:--`. |
| Idle | Reports present azimuth and accepts commands. |
| Slewing | Node model presses/releases its virtual button to converge to target. |
| Toggle press | A 100 ms virtual press consumes the alternating-direction toggle before the commanded slew. |
| Queued | Stores up to four future commands and executes each when local UTC reaches its timestamp. |
| Primary communications | AC commands both nodes through CYD USB and ESP-NOW. |
| Backup communications | AC commands node 2 through TCP 31995 and Pi USB serial when the primary path is unavailable or explicitly overridden. |

## Non-goals

- Physical relay actuation, sensor-derived heading, calibration, or anti-stall
  control.
- Persistent RTC time across reset.
- Production local-command confirmation and authorization.
- Control authorization or encrypted ESP-NOW operation.
- Automatic active-active command duplication across both node-2 paths.

## Related documents

- [Architecture](rotator-controller-architecture.md)
- [Implementation plan](rotator-controller-implementation-plan.md)
- [Test plan](rotator-controller-test-plan.md)
- [State-machine contract](rotator-controller-state-machine.md)
