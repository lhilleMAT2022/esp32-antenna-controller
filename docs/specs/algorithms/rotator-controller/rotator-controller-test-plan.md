# Rotator Controller v0.1 — Test Plan

## Status: Draft

**Last updated:** 2026-09-28  
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

## Build validation

Run:

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" run
```

Pass criterion: all three PlatformIO targets compile without errors. The
TFT_eSPI warning about `TOUCH_CS` is expected in v0.1 because touch input is
not yet calibrated or enabled.
