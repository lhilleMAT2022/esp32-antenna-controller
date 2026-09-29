# Rotator Controller v0.1 — Implementation Plan

## Status: In Progress

**Last updated:** 2026-09-28  
**Architecture:** [rotator-controller-architecture.md](rotator-controller-architecture.md)  
**Test plan:** [rotator-controller-test-plan.md](rotator-controller-test-plan.md)

## Progress

| Phase | Status | Deliverable |
|---|---|---|
| 0 — Interface freeze | Complete | Version-3 packet, serial grammar, units and flags |
| 1 — Node model | Complete | Non-actuating real-time plant, queue and status |
| 2 — CYD mock-up | Complete | Default operator display, keypad/debug panels and USB commands |
| 3 — Hardware smoke test | Complete | Build, flash and verify all three boards |
| 4 — Physical integration | Next | Magnetometer, isolated relay interface, calibrated touch, encryption |

## Parameters

| Parameter | Default | Unit | Calibratable | Source |
|---|---:|---|---|---|
| Max model speed | 15 | °/s | Yes | 2.5 rpm |
| Acceleration/deceleration | 7.5 | °/s² | Yes | 2 s ramp |
| Mechanical travel | 370 | ° | Yes | user-provided rotator behavior |
| Position tolerance | 0.5 | ° | Yes | initial controller target |
| Toggle press | 100 | ms | Yes | initial model approximation |
| Overlap south true bearing | 180 | ° true | Yes, per node | initial site assumption |
| Queue capacity | 4 | commands | No | v0.1 RAM bound |

## Deployment checkpoint

1. Build `cyd_gateway`, `antenna_node_1`, and `antenna_node_2`.
2. Flash CYD COM8, node 1 COM7, node 2 COM9.
3. Set a known time: `TIME 2026-09-01T12:03:06Z`.
4. Issue `AZ 1 49`, observe the flashing slew symbol and convergence.
5. Issue a near-future `QUEUE` command and verify it starts at the requested
   node UTC time.
6. Confirm no relay wiring is connected or driven.

## Next hardware increment

When the ordered parts arrive, add the LSM303AGR and STEMMA mini relay behind
disabled-by-default hardware-abstraction interfaces. Validate raw sensor
telemetry and a disconnected relay bench fixture before allowing either
interface to affect the rotator control path. Preserve the v0.1 plant model
as the selectable fallback for regression testing and safe development.
