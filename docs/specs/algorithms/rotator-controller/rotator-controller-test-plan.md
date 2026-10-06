# Rotator Controller v0.1 — Test Plan

## Status: Draft

**Last updated:** 2026-10-05
**Architecture:** [rotator-controller-architecture.md](rotator-controller-architecture.md)

## Test cases

| ID | Stimulus | Expected result |
|---|---|---|
| T1 | Boot all devices | CYD home panel shows both nodes, which report true azimuth at 1 Hz. |
| T2 | `AZ 1 49` | Node 1 flashes `* SLEWING`, virtual button state is reported, and final error is ≤0.5°. |
| T3 | `STEP 2 +10`, then `STEP 2 -10` | Each change has the requested true-bearing sign; reversal uses the alternating-press model. |
| T4 | Target across overlap region | Controller selects nearest 0–10° / 360–370° mechanical representation. |
| T5 | Command drives through 0° or 370° | Model reflects position/velocity and logs a bounce. |
| T6 | `QUEUE 1 <now+10s> 100` after `TIME` | Command starts no earlier than timestamp and no later than timestamp +1 s. |
| T7 | Queue without `TIME` | CYD rejects request and node queue remains unchanged. |
| T8 | `STOP 1` during slew | Button releases, target clears, and model coasts to stop. |
| T9 | `PANEL DEBUG`, `PANEL HOME`, `KEYPAD 1`, and physical `Az` | Debug only appears when selected; serial and touch keypad entry render correctly. |
| T9a | Four-point touch calibration | Large crosshairs accept touch, calibration persists across reset, and each on-screen button maps to its drawn region. |
| T10 | Inspect node GPIO behavior | No relay output is configured; built-in LED only mirrors virtual button state. |
| T11 | Boot node 2 with LSM303AGR on GPIO21/GPIO22 | Accelerometer and magnetometer are both detected; sensor flags report present and valid. |
| T12 | Rotate and tilt the disconnected sensor assembly | Magnetic vector, field magnitude, provisional magnetic heading, roll, and pitch change on node and CYD serial output. |
| T13 | Exercise sensor telemetry during a model move | Sensor data remains diagnostic only; the simulated azimuth remains the controller feedback source. |
| T14 | Monitor node 2 USB serial | Each second includes valid `rp` status and `rs` sensor JSON objects; human log lines do not prevent parsing. |
| T15 | Send canonical `rc` JSON directly to node 2 | Node accepts `goto`/`stop`, returns `ra`, and continues sending status without CYD involvement. |
| T16 | Run Pi relay on TCP 31995 | Node-2 `rp`/`rs` objects reach AC unchanged and AC time sync reaches the node. |
| T17 | Send `antenna_command` to AC TCP 31988 | AC validates and routes the command, then publishes schema-shaped `antenna_state` on UDP 31989. |
| T18 | Primary and backup links both healthy | Node-2 command uses CYD/ESP-NOW only; no duplicate command is sent. |
| T19 | Primary node-2 telemetry stale, backup fresh | Next node-2 command uses the Pi route; node 1 remains CYD-only. |

## Build validation

Run:

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" run
```

Pass criterion: all three PlatformIO targets compile without errors. The
TFT_eSPI `TOUCH_CS` warning is expected because touch uses the CYD's separate
XPT2046 SPI bus rather than TFT_eSPI's optional integrated touch path.
