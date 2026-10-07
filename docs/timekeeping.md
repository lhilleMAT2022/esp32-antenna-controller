# Disciplined UTC and staggered reports

Implemented 2026-10-07, wire protocol **7** (clock discipline introduced in protocol 5). CYD and both remote nodes require
matching firmware. No additional time-sync traffic is introduced.

## Clock behavior

PC seeds CYD every 3 seconds; CYD seeds each node every 3/10/60 seconds in
continuous/normal/quiet mode. These remain one-way messages: a fixed delay
bias is acceptable, and this is not NTP or an absolute-accuracy guarantee.

Each board maps its 64-bit monotonic timer to UTC using an anchor, an estimated
frequency correction, and a bounded phase slew. `firmware/clock_discipline.h`
contains the tunable limits:

- Startup: step to the first valid UTC sample.
- Ordinary updates: median of the latest five errors (minimum three), then
  an exponential filter with weight 0.25. Re-anchor continuously without
  stepping UTC; estimate frequency with a slow integral term (900 s time
  constant) and request phase correction over 120 s.
- Estimated frequency correction is limited to ±200 ppm; combined frequency
  and phase correction is limited to ±500 ppm. UTC continues increasing while
  slewing, including during negative correction.
- Errors strictly greater than 2000 ms are excluded from normal filtering.
  Three consecutive fresh measurements within 250 ms of each other, separated
  by no more than 180 s, permit a recovery step. A normal sample clears the
  candidate sequence; inconsistent large errors restart it. The last actual
  step is recorded separately from filtered phase error.
- After 120 s without a new anchor, finite phase slewing stops; the estimated
  frequency remains active. After 180 s without an accepted sample, diagnostics
  show holdover. Reacquisition resets stale error-filter samples.

This suppresses isolated delayed packets and repeated clock jumps. It cannot
distinguish sustained link delay changes from clock error. Recovery steps can
still move UTC backward or forward; quiet duration always uses monotonic time.
Previously queued UTC movement commands retain their existing UTC semantics.

Health displays clock state (`unsynced`, `tracking`, `slewing`, `checking`,
`holdover`), signed current rate in ppm, filtered error, sync age, and last
actual step for nodes; CYD state/rate/error are also shown. Positive rate means
UTC runs faster than the local monotonic timer. No battery-backed RTC is added.

## Periodic radio scheduling

For period P=10 or 60 seconds, report at UTC interval boundary plus
`(node_id mod P)` seconds. For P=1 second, report at each whole UTC second plus
`(node_id mod 10) × 100 ms`.

| Period | Node 1 | Node 2 |
|---|---|---|
| 1 second | .100 each second | .200 each second |
| 10 seconds | :01, :11, :21, … | :02, :12, :22, … |
| 60 seconds | :01 each minute | :02 each minute |

The existing mode/motion rules choose the period independently for heartbeat
and sensor reports. Normal movement therefore uses 10-second heartbeat slots
and 1-second sensor slots; quiet movement uses 10-second slots for both.
Raw and corrected sensor packets are sent together at the sensor slot.

Before UTC is known, use 64-bit uptime with the same phase offsets. Different
boot times mean cross-node alignment cannot be guaranteed before sync.
On mode/period changes, initial UTC acquisition or a recovery step, select the
next future slot. Events do not reset periodic phase. ACKs, move completion,
unexpected-motion reports and quiet-expiry reports remain immediate.

Late loops may service a slot within 100 ms; older missed slots are skipped,
with no catch-up burst. A periodic transmission less than 500 ms (1 Hz) or
1000 ms (slower modes) after the preceding periodic transmission is skipped,
including when changing rates or recovering from a clock step. These guards
do not delay event reports. Timestamping and serial work after the scheduling
decision can add a small additional delay; targets are not hard real-time.

USB reports retain their independent 1 Hz schedule. Gateway synchronization
packets and MAC acknowledgments retain their existing behavior. No extra
sensor reports are introduced during stationary quiet operation.

## Protocol and verification

The protocol-5 status/command/time packet is **62 bytes**. It appends
`clockErrorMs:i32, clockRatePpm:i16, clockState:u8` to the protocol-4 55-byte
layout. Other packet sizes remain unchanged. The node receive buffer is
70 bytes in protocol 6 to accommodate scheduled-slew commands. Serial `rp` and gateway `gs` add `sync_error_ms`, `sync_rate_ppm`,
`sync_state`. `sync_step_ms` remains the last actual step (zero for startup).
`antenna_state` UDP schema remains 1.1.0: these new diagnostics are local to
the TUI/serial protocol.

Native tests cover six simulated hours of 100 ppm drift with packet-delay
noise at all synchronization cadences; final error relative to the biased
source was within 2 ms, with estimated drift about 96 ppm. This is simulation
evidence, not measured ESP32 oscillator performance. Tests also cover positive/
negative slewing, holdover, outliers, confirmed steps, all node slots, missed
slots, clock transitions, motion exceptions, and monotonic quiet expiry.

`tests/hardware_runtime_smoke.py` checks both physical nodes' UTC slot phases,
bounded clock progression, mode ACKs, normal/quiet rates, timer expiry and
unchanged 1 Hz USB reporting. It leaves both nodes normal. Use a distinct
`--output` log path to preserve earlier acceptance evidence.
