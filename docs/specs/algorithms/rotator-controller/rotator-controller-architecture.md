# Rotator Controller v0.1 — Architecture

## Status: Draft

**Last updated:** 2026-09-28  
**Parent:** [System specification](rotator-controller-system.md)

## Functional decomposition

```text
USB serial ──> CYD command parser ──> ESP-NOW command ──> Node supervisor
    │                    │                                     │
    │                    ├──> time synchronization ────────────┤
    │                    └──> CYD operator/debug/keypad panels  │
    │                                                          ▼
    └────────────────< status, ACK, RSSI <── Node status <─ Rotator model
```

The CYD is the sole radio gateway. It owns display state and a UTC offset
derived from USB serial. Each node owns its command queue, virtual press state,
rotator dynamics, and position state.

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

## Key decisions

| Decision | Rationale |
|---|---|
| USB sets UTC; nodes follow gateway | No RTC or network-time dependency in v0.1. |
| Queue resides on node | Commands execute even if the gateway serial client disconnects after queueing. |
| Control display is default; diagnostic panel is selected | Keeps operation-focused data primary while retaining radio observability. |
| XPT2046 four-point calibration plus serial analogs | Aura confirms the CYD touch controller is on separate VSPI pins 25/32/39/33 with IRQ 36. Calibration is persisted in Preferences and maps raw touch coordinates directly to the landscape UI. |
| Model-only actuation | Prevents any inadvertent rotator or mains-related action during development. |

## API verification notes

Existing project patterns were retained for ESP-NOW peer registration, callback
queuing, and filtered promiscuous receive metadata. The installed
Arduino-ESP32 2.0.17 ESP-NOW receive callback provides no RSSI field, so the
project's existing `wifi_promiscuous_pkt_t.rx_ctrl.rssi` path remains the
source of measured RSSI.
