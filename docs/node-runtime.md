# Node reporting, timing, and displays

Current implementation: 2026-10-07, ESP-NOW protocol **4**. This specification
supersedes earlier fixed-rate node reporting and integer-second clock behavior.
Nodes 1 (SURV) and 2 (REF) implement the same sensor/calibration/runtime features.
Rotator movement is still simulated; no physical relay is actuated.

## Reporting requirements

The operator selects one mode for both nodes with the TUI **Reporting** button
or a manual command:

```text
report continuous
report normal
report quiet 300
```

| Mode | Idle RF heartbeat | RF sensor reports | During commanded motion |
|---|---|---|---|
| continuous | 1 s | 1 s | Same rates |
| normal (boot default) | 10 s | 10 s | Sensors 1 s; heartbeat remains 10 s |
| quiet | 60 s | Only unexpected rotation >10° | Heartbeat/position and sensors 10 s, plus completion report |

Quiet duration is an integer from 1 to 86400 seconds. A monotonic timer returns
the node to normal at expiry, with an immediate status/sensor report. A move
does not reset or extend that timer. Completion reports are immediate in all
modes. Quiet expiry during a move changes to normal's moving rate.

Unexpected rotation uses the relative 3-D rotation of the corrected gravity
and magnetic vectors, including heading wrap. The reference is the last
reported orientation or the current orientation during commanded motion.
The threshold is strictly **greater than 10°**, as confirmed by the operator.
Gravity must be 0.8–1.2 g and horizontal magnetic field at least 1 µT; invalid
samples cannot trigger a report. Events are limited to one report per second.
Sensor failure prevents this detector from establishing motion; this is not
a mechanical motion interlock.

USB serial status, raw sensors and calibration diagnostics remain at 1 Hz in
every mode. Quiet suppresses periodic RF sensor/calibration packets, retaining
calibration flags in the heartbeat. Command replies remain immediate.
ESP-NOW MAC acknowledgments and gateway time synchronization also remain:
quiet reduces RF traffic but does not guarantee RF silence.

Mode requests are independently acknowledged by each node. The TUI displays
the actual reported modes and any partial failure, rather than assuming both
changed after a successful write. Timeout is five seconds, with outcome
unknown. Starting magnetometer or accelerometer-face capture requests
continuous globally and waits for both nodes to confirm. Reducing reporting
rate is blocked while capture is active or pending; cancel capture first.
Completing calibration leaves continuous enabled until the operator changes it.

## Clock ownership and diagnostics

The PC supplies UTC epoch milliseconds to the CYD over USB every three seconds.
CYD and each node maintain a free-running clock anchored to a 64-bit monotonic
timer. The CYD sends time to each node every 3/10/60 seconds in continuous/
normal/quiet mode, plus the first valid synchronization. If CYD USB is down,
the PC can seed node 2 through its serial relay. There is no battery-backed RTC;
UTC is invalid after reboot until synchronized. A correction steps the clock;
quiet expiry uses monotonic time and is unaffected by UTC corrections.

Legacy `ts` is UTC epoch seconds, not seconds since midnight. `ts_ms` adds
millisecond resolution and `tv` indicates validity. The previous host did not
periodically seed CYD UTC, so an integer-second field alone did not establish
that either clock was synchronized.

Health shows PC UTC, CYD UTC, each node's last reported UTC with age, last
sync correction and sync age. Node minus CYD offset uses CYD's radio receive
timestamp; node minus PC offset uses PC receipt time. The displayed standard
deviation uses the last 60 reports per route. These one-way differences include
link/queue delay and clock corrections; they are not a pure oscillator drift
measurement or a precision time-transfer guarantee.

Node link freshness thresholds are 5/25/90 seconds for continuous/normal/quiet.
The serial backup remains stale after 3.5 seconds. Unknown mode uses the legacy
host timeout until a status arrives. Mode is not persisted across reboot.

## Vectors and display

Both nodes appear in the calibration tab, independent of the selected help
node. A is acceleration in g; B is magnetic flux density in µT. Raw XYZ and MAE
describe the same sensor-frame vector. MAE magnitude is its Euclidean norm;
azimuth is `atan2(y,x)` in [0,360) degrees and elevation is
`atan2(z,hypot(x,y))` in [-90,90] degrees. Azimuth is undefined on the Z axis;
both angles are undefined for zero magnitude. MAE azimuth is not the antenna's
true bearing. Corrected calibration diagnostics are labeled separately.

The CYD debug panel displays each node's A and B over two lines per vector to
fit the 320×240 screen, using the degree symbol and omitting roll/pitch. Its
header displays advancing UTC after synchronization. Raw data availability
and sample age must be distinguished from node link health, especially during
quiet intervals with no periodic sensor packets.
Both debug views retain the last raw sample with its age during quiet; a
reported invalid sensor is shown as unavailable. These are last-known values,
not a claim of continuous measurement delivery.

## Wire contract

All three boards must run protocol 4 together; protocol 3 is incompatible.
Packed little-endian ESP32 layouts are defined in `espnow_smoke_config.h`,
`runtime_protocol.h`, and `calibration_protocol.h`. There is no JSON on air.

| Packet | Bytes | Type |
|---|---:|---|
| Status / rotator command / ACK / time sync | 55 | 1 / 2 / 3 / 4 |
| Raw sensors | 26 | 5 |
| Calibration command / report | 56 / 52 | 6 / 7 |
| Reporting command / ACK | 12 / 55 | 8 / 9 |

Reporting command layout is `type:u8,node:u8,version:u8,mode:u8,request:u32,
durationSeconds:u32`; modes are continuous=0, normal=1, quiet=2. Request ID is
nonzero. Non-quiet durations must be zero. The last eight request IDs/payloads
are retained in RAM: identical replay acknowledges current state without
extending quiet; conflicting payload is rejected. Reboot clears that cache.

The 55-byte Packet retains the original 20-byte prefix and appends
`utcMilliseconds:u64, uptimeMs:u32, reportingMode:u8, quietRemainingMs:u32,
clockSyncAgeMs:u32, clockCorrectionMs:i32, reportingRequest:u32,
reportingError:u8, boot:u32, calibrationFlags:u8`.

USB and the optional TCP relay use compact JSON lines:

```json
{"t":"rm","n":1,"q":42,"mode":"quiet","duration_s":300}
{"t":"gt","utc_ms":1791378000123}
```

`rm` is sent separately to each node with the same global request ID. `rp`
adds `ts_ms,tv,mode,quiet_left_ms,up_ms,sync_age_ms,sync_step_ms,mr,me,boot,cf`;
`mr` correlates a mode acknowledgment, `me` is 0 accepted or 2 rejected.
CYD adds `gw_rx_ms` when synchronized, plus radio RSSI `rg/rn`.
`gs` is CYD's one-second USB clock/status message with
`ts_ms,tv,up_ms,sync_age_ms,sync_step_ms,pv`. Invalid sync age is UINT32_MAX.
`rs` raw and `cs` corrected sensor messages remain distinct from `rp` status.

Host `antenna_state` is proposed schema **1.1.0**, adding optional
`node_time_utc_ms`, `reporting_mode`, `quiet_remaining_s`,
`node_cyd_offset_ms`, and `node_pc_offset_ms`. Node UTC is null until valid.
Legacy `measured_utc_ms` remains host receipt time; it is not retroactively
redefined as sensor sampling time. UDP publication follows accepted board
telemetry, including serial telemetry at 1 Hz; quiet limits apply to ESP-NOW.

## Verification and remaining acceptance

Native tests exercise cadence, timer wrap/expiry, motion completion, 3-D
rotation threshold, request replay, UTC arithmetic, and protocol parsing.
Host/TUI tests exercise both-node display, mode ACK/failure/timeout, automatic
continuous capture, route freshness and clock offsets. Build all three PIO
environments before coordinated firmware deployment.

Automated stationary bench check (requires exclusive serial access and leaves
both nodes in normal mode; does not change calibration or command movement):

```powershell
.\.venv\Scripts\python.exe tests/hardware_runtime_smoke.py --gateway-port COM10 --node1-port COM11 --node2-port COM7
```

This records raw JSON in ignored `runtime-smoke.log`. Enumerate ports first.

On hardware, verify both node IDs, protocol 4, advancing UTC, one-second USB
telemetry in every mode, normal ten-second RF reports, a static quiet window
over 60 seconds, timer expiry, and the physical display. Repeat with hand
rotation beyond 10° using valid sensors. Test commanded-motion cadence with
the simulator; physical relay control and calibration save/power-cycle
acceptance remain separate work.
