# Rotator Controller v0.1 — State-Machine Contract

## Status: Draft

**Last updated:** 2026-09-28  
**Parent:** [System specification](rotator-controller-system.md)

## States

| State | Entry / active behavior | Exit condition |
|---|---|---|
| Idle | Button released; no target; plant coasts to rest if necessary. | Immediate or queued target arrives. |
| Seek | Computes target error and starts/release presses using stopping distance. | Within 0.5° and below 0.05°/s. |
| TogglePress | Presses for 100 ms to consume a direction alternation. | 100 ms elapsed. |
| Coast | Button released; virtual plant decelerates. | Velocity below 0.05°/s, then returns to Seek or Idle. |
| Queued | Orthogonal condition: up to four `(UTC,target)` records await dispatch. | UTC reaches each record. |
| TimeInvalid | Orthogonal condition: status timestamp invalid, queue requests rejected. | Valid CYD time sync received. |

## Transition rules

| From | To | Guard / action |
|---|---|---|
| Idle | Seek | `SetAzimuth` or `StepAzimuth`; select nearest mechanical target. |
| Any active | Idle | `Stop`; clear target and release virtual button. |
| Seek | TogglePress | Required direction does not equal `nextPressDirection`; start 100 ms press. |
| TogglePress | Coast | Release button at expiry. |
| Seek | Coast | Remaining error <= stopping distance, or current press direction becomes wrong. |
| Coast | Seek | Plant stopped and target still outside tolerance. |
| Seek | Idle | Position and target differ <=0.5° after plant stops. |
| Queued | Seek | Local UTC >= queue time; consume record. |

## Arbitration

`Stop` has highest priority and clears only the active target, not queued
entries. A queued command due in the same loop is serviced after received
commands; a just-received `Stop` therefore wins for that loop. Queue capacity
is four records. A queue command is acknowledged even when rejected in v0.1;
the serial/node logs are the diagnostic source until a result-code field is
added.

## Boundary behavior

| Condition | Behavior |
|---|---|
| Mechanical q < 0° | Reflect position and velocity; log lower end-stop bounce. |
| Mechanical q > 370° | Reflect position and velocity; log upper end-stop bounce. |
| Desired true bearing in overlap | Choose 0–10° or 360–370° mechanical representation nearest current q. |
| UTC absent | Immediate target allowed; queued command rejected. |
| CYD disappears after queue upload | Node continues with its last synchronized UTC offset and queued record. |
| Sensor/relay unavailable | Not represented in v0.1; no hardware is actuated. |
