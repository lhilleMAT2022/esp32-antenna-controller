# Rotator Controller v0.1 — Architecture

## Status: Draft

**Last updated:** 2026-10-05
**Parent:** [System specification](rotator-controller-system.md)

## Functional decomposition

```text
RM --TCP 31988--> Host AC --USB--> CYD --ESP-NOW--> Nodes 1 and 2
                         |          ^                 |
                         |          +---- status -----+
                         |
                         +--TCP 31995--> Pi relay --USB--> Node 2

Host AC --UDP 31989--> RM / RF Collector / Report & Display
```

The CYD is the primary radio gateway and owns display state. The Python host
AC adapts the system ICD to the compact board protocol, merges telemetry from
either path, selects one command route, and publishes system state. Each node
owns its command queue, virtual press state, rotator dynamics, and position
state. The Pi relay is transport-only and owns no control policy.

## Host transport and routing

TCP 31988 is the authoritative `antenna_command` input to AC. UDP 31989 carries
periodic, loss-tolerant `antenna_state`. TCP 31995 is reserved for a
newline-delimited compact-JSON stream between AC and the optional Pi relay.

The command route is single-path:

1. Node 1 always uses CYD/ESP-NOW.
2. Node 2 normally uses CYD/ESP-NOW.
3. Node 2 uses the Pi route when primary telemetry is older than 3.5 s and Pi
   telemetry is fresh, or when `--prefer-backup-node2` is selected.
4. Commands are not broadcast over both paths, preventing duplicate physical
   button actions when relay actuation is introduced.

The TCP relay is intentionally unauthenticated for bench work on the isolated
rooftop network. Before relay actuation, bind it to the data-network interface,
apply a host firewall rule, and add authenticated command origin/replay
protection.

## Packet contract

All ESP-NOW traffic uses one packed, 20-byte `Packet` at protocol version 3.

| Field | Unit | Meaning |
|---|---|---|
| `packetType` | enum | Status, command, command acknowledgment, or time sync |
| `senderId` | ID | CYD=0; nodes=1,2 |
| `flags` | bitset | Valid time, moving, virtual button pressed, queue pending |
| `sequence` | count | Per-sender diagnostic sequence |
| `epochSeconds` | UTC s | Sender time; command execution time for `QueueAzimuth` |
| `azimuthDeciDegrees` | 0.1° true | Current node report |
| `targetDeciDegrees` | 0.1° true | Current target, or -1 |
| `commandValueDeciDegrees` | 0.1° | Command target, relative step, or overlap bearing |
| `commandType` | enum | Set, step, queue, overlap, or stop |
| `receiverRssiDbm` | dBm | Sender's most recent measured receive RSSI from recipient |

`N>G` on the CYD is its direct receive measurement. `G>N` is the node's
reported measurement. Neither is a calibrated RF power measurement.

## Serial JSON contract

All three ESP32 targets retain human-readable diagnostics and additionally
emit one compact JSON object per line:

| Type | Direction | Purpose |
|---|---|---|
| `rc` | AC → CYD or node | `goto`, `stop`, `step`, `queue`, `overlap`, or `time` command |
| `rp` | CYD or node → AC | Rotator status: node, sequence, UTC, heading, target, moving, error |
| `rs` | CYD or node → AC | Sensor vector, field magnitude, magnetic heading, roll, pitch, validity flags |
| `ra` | CYD or node → AC | Command syntax/acceptance result |

The gateway tags forwarded telemetry `src:"espnow"`; a directly attached node
tags it `src:"node_serial"`. The Pi passes objects unchanged.

## Node rotator model

The position state `q` is mechanical degrees on `[0, 370]`. The true bearing is

`azTrue = wrap360(overlapSouthTrue + q)`.

`overlapSouthTrue` is a calibratable site parameter. A requested true bearing
has a normal mechanical representation `[0, 360)` and, in the 10° overlap
region, a second representation `[360, 370]`; the node chooses the closest
current representation.

The virtual press produces a target velocity of `+15` or `−15°/s`. Velocity
changes at `7.5°/s²`, yielding two-second acceleration and deceleration. A
released button commands zero velocity. At 0° or 370°, the position and
velocity are reflected to model the rotator's end-stop bounce.

## Controller behavior

The controller is a target-seeking, bang-bang position controller with a
stopping-distance guard:

`dStop = v² / (2 * 7.5) + 0.5°`.

It releases the virtual button when remaining error is no larger than this
distance. When settled within 0.5° with velocity below 0.05°/s, it clears the
target. There is no integral term, so anti-windup is not applicable.

Because each real button press reverses the available direction for the next
press, a desired direction that differs from `nextPressDirection` performs a
100 ms virtual toggle press followed by coast-down, then starts the intended
press. This tiny modeled motion is intentional.

## Rates and initialization

| Component | Rate | Initialization |
|---|---:|---|
| Node plant/model | Every loop; elapsed time capped at 100 ms | Node 1 starts q=95°, node 2 q=270° |
| Node status | 1 Hz | Reports time-invalid until CYD sync |
| CYD time sync | Every 3 s and after `TIME` | UTC invalid after a CYD reset |
| Display refresh | On status/event change and each UTC second | Home panel selected |
| Sensor sample | 10 Hz on node 2 | Diagnostic only |
| Compact serial status | 1 Hz per attached node | Same values as ESP-NOW status |
| Host AC backup freshness | 3.5 s | Primary route preferred |

## Key decisions

| Decision | Rationale |
|---|---|
| USB sets UTC; nodes follow gateway | No RTC or network-time dependency in v0.1. |
| Queue resides on node | Commands execute even if the gateway serial client disconnects after queueing. |
| Control display is default; diagnostic panel is selected | Keeps operation-focused data primary while retaining radio observability. |
| XPT2046 four-point calibration plus serial analogs | Aura confirms the CYD touch controller is on separate VSPI pins 25/32/39/33 with IRQ 36. Calibration is persisted in Preferences and maps raw touch coordinates directly to the landscape UI. |
| Model-only actuation | Prevents any inadvertent rotator or mains-related action during development. |
| Pi is a transparent relay | Keeps control policy, ICD expansion, and failover selection in AC. |
| Single command route | Avoids duplicate button operations across redundant links. |

## API verification notes

Existing project patterns were retained for ESP-NOW peer registration, callback
queuing, and filtered promiscuous receive metadata. The installed
Arduino-ESP32 2.0.17 ESP-NOW receive callback provides no RSSI field, so the
project's existing `wifi_promiscuous_pkt_t.rx_ctrl.rssi` path remains the
source of measured RSSI.
