# Node sensor calibration

The host fits calibration from raw telemetry. Node 2 applies the coefficients
and can save them in ESP32 Preferences/NVS flash. A reboot reloads the saved
record. Updating coefficients in RAM does not write flash; `save` is explicit.
The CYD forwards calibration commands and node responses over ESP-NOW. The
optional Pi relay supports the same commands over direct node USB serial.

Update the CYD and node-2 firmware together before using these commands, then
restart AC with the updated Python environment (`uv sync --inexact`). Press
`c` or click **Cal** in the TUI for instructions. Enter commands in the manual
field (`u`). Status and progress appear in the calibration panel and log.
Node 1 currently has no sensor; it rejects calibration operations.

## Capture and fit

Stop antenna motion. Calibration involves turning the sensor assembly by
hand, with its mounting hardware and nearby electronics in their final
relative arrangement. AC checkpoints raw capture buffers to JSON files under
`calibration_captures/` in its working directory. Restarting AC loses the active
in-memory workflow; restarting the node cancels captures and clears fits.
Files already saved remain available for analysis, but are not automatically
loaded into a new workflow. See the diagnostics section below.

1. Enter `cal 2 start mag`. Slowly tumble the sensor through all three axes
   and eight octants for at least 120 samples (about two minutes at 1 Hz).
   Vary orientation rather than holding one pose. A level circle on the
   rotator cannot determine full 3-D calibration. The buffer holds at most
   1800 samples; restart capture if it fills before coverage is adequate.
2. Enter `cal 2 fit mag`. The fit rejects deficient coverage, excessive
   distortion and large residuals. A rejected fit leaves the capture running
   so more orientations can be collected. Enter `cal 2 apply mag` after a
   successful fit and wait for the node acknowledgment.
3. For the accelerometer, point the sensor's **+X axis upward**, hold still,
   and enter `cal 2 face +x`. Wait for the panel to confirm 12 samples.
   Repeat with `-x`, `+y`, `-y`, `+z`, and `-z` upward. Follow the sensor axes,
   not the antenna axes. Repeat a face to replace a bad capture.
4. Enter `cal 2 fit accel`, then `cal 2 apply accel`. Movement, incorrect
   orientation, implausible gains/bias, and inconsistent faces are rejected.
5. Install the sensor with a known axis along the Yagi and another pointing
   upward. Enter `cal 2 align <forward-axis> <up-axis> <declination-deg>`.
   Signed axes are `+x`, `-x`, `+y`, `-y`, `+z`, `-z`; they must be distinct.
   Declination is east-positive. For example, `cal 2 align +x +z -12` means
   +X forward, +Z up, and 12 degrees west declination. Use the actual mounting
   and local declination; these example values are not a site configuration.
6. Check the corrected heading at several surveyed bearings and tilts before
   saving. Investigate heading-dependent errors instead of absorbing them in
   the declination value. Arbitrary mounting rotations that do not align with
   two sensor axes are not supported by this first mounting interface.

The magnetic fit estimates a bias and a symmetric 3x3 ellipsoid correction.
Its determinant is normalized to one: it calibrates direction and relative
axis gains, not absolute field strength against a reference instrument. The
accelerometer fit estimates three biases and three diagonal gains in g.
Gravity remains part of acceleration. A 60 cm mounting radius matters for
acceleration during rotation, so orientation is evaluated while stationary.

## Capture files and failed-fit diagnostics

The host creates a separate JSON file for each magnetometer capture or
accelerometer face capture and checkpoints it after every accepted sample.
Writes replace the file atomically, keeping the previous checkpoint intact if
the update fails. Each fit attempt also creates a separate snapshot, whether
it passes or fails. Repeating a face or starting a new capture does not erase
earlier files. Cancellation and detected node restarts archive the host state.
These files are excluded from Git.

Files contain the node/boot IDs, UTC snapshot time, last raw sequence, latest
node status, raw samples with their capture kind, all completed accel faces,
successful fits, and the fit error when applicable. `accel_diagnostics` includes
per-face means and standard deviations, orientation checks, fitted offsets
and gains when computable, corrected vectors and vector errors, the worst face,
and acceptance limits. Acceleration uses g; magnetic raw vectors use microtesla.

Enter `cal 2 export` in AC's manual field to save an additional snapshot and
display its full path. This works without fresh node telemetry and sends
nothing to the node. Fit results also log the snapshot path. In PowerShell,
inspect the newest snapshot from the project directory with:

```powershell
$captureFile = Get-ChildItem .\calibration_captures\*.json |
    Sort-Object LastWriteTime -Descending | Select-Object -First 1
$captureRecord = Get-Content -Raw -LiteralPath $captureFile.FullName | ConvertFrom-Json
$captureRecord.accel_diagnostics | ConvertTo-Json -Depth 10
```

A stationary face can pass the initial checks and still fail the final fit:
the initial gates permit up to 0.025 g per-axis standard deviation and 0.25 g
of cross-axis mean, whereas the final corrected vector must be within 0.08 g
of the expected unit vector. For example, an otherwise ideal +X capture held
steadily at an 8-degree tilt passes the initial gates but has about 0.139 g
final error. Failure details identify faces exceeding the final limit. Support
the sensor so the named axis points vertically, recapture affected faces, and
run `cal 2 fit accel` again. Other completed faces remain in memory. A reported
face error can also reflect imperfect fitted coefficients, so inspect all face
means and gains if recapturing that face does not resolve it.

Disk errors appear as **Capture file NOT saved**; calibration continues in
memory. Check the error before closing AC. This feature requires restarting AC
after updating the Python code. Older running instances have no disk archive
or export command; updating files cannot recover their in-memory samples.
ESP32 NVS saves coefficients/configuration only, not these raw samples.

## Save, clear, and recover

- `cal 2 save`: save the currently applied settings. Wait for **Node confirmed
  save** and **SAVED**. Partial calibration can be saved and is marked PARTIAL.
- `cal 2 status`: request a fresh node report, including loaded/applied flags.
- `cal 2 clear`: atomically replace the saved record with defaults and reset
  active corrections. This clears calibration persistently.
- `cal 2 cancel`: stop host capture without changing the node or flash.
- `cal 2 export`: save host capture/diagnostic data to disk; does not write NVS.

The node returns a response with the original request ID. Sending bytes to a
gateway never counts as confirmation. A five-second timeout means the outcome
is unknown; query status before retrying. Calibration operations are absolute
and can be repeated; identical saves avoid another flash write. Telemetry
continues once per second, even without AC, so a reconnected host can discover
the state loaded at boot.

The NVS namespace is `sensor-cal`, key `config`. One versioned blob contains
coefficients, mounting axes, declination, flags, and an integrity checksum.
Boot checks its size, version, checksum, and coefficient bounds before use.
Invalid records fall back to uncalibrated defaults and report **NVS INVALID**.
Saving checks the write result and reads back the exact record. NVS provides
the atomic blob update; a normal power outage retains the last committed
record. A firmware upload that erases NVS or changes its partition is different
from a normal power outage and can remove calibration.

## Telemetry and limits

Existing `rs` vectors remain raw. New `cs` reports include corrected vectors
(`mc`, `ac`), corrected true heading (`ch`), and mounting inclination (`ti`).
The TUI shows calibration as UNKNOWN/STALE if reports stop. Separate mag,
accel, mount, and saved flags prevent a partial fit from being described as
fully calibrated. Heading is unavailable unless all calibrations are present,
both sensors are valid, gravity is near 1 g, horizontal field is sufficient,
and the forward axis is not near vertical. These checks do not compensate
arbitrary dynamic acceleration or changing magnetic interference.

Corrected heading remains diagnostic. Rotator control and the existing
`antenna_state.az_true_deg` still use the simulator; no relay output is enabled.
The broader measurement/command issues listed in the README checkpoint remain.

## Protocol and verification

`cc` commands have `n` (node), `q` (nonzero 32-bit request ID), `op`, and `v`.
Operations are `mag`/`accel` (3 offsets then a row-major 3x3 matrix), `align`
(forward axis, up axis, declination), and `save`/`clear`/`status` (empty array).
ESP-NOW type 6 carries the 56-byte command; type 7 carries the 52-byte report.
These extend protocol v3 without changing existing status/sensor packet sizes.
Older firmware cannot forward or act on calibration; update both ends.

`cs` includes `q` (node sequence), `boot` (boot ID), `req` (acknowledged request,
zero for periodic reports), `cf` (flags), and `e` (0 success, 1 unsupported
sensor, 2 invalid settings/operation, 3 storage failure). Flag bits are 1 mag,
2 accel, 4 mount, 8 saved, 16 invalid NVS, 32 live mag, 64 live accel, and
128 valid corrected heading. Late duplicate reports do not roll back host
state; duplicate raw samples from two transports are counted once.

Run `uv run --inexact python -m unittest discover -s tests -v` and `pio run`.
Native tests compile shared firmware math, storage and parsing using g++ and
the PlatformIO ArduinoJson dependency. Storage tests use an in-memory store,
covering boot reload, corrupt records, failed writes, repeated saves and clear.
They do not replace the physical acceptance test:

1. Complete capture/fit/apply over CYD/ESP-NOW and verify coefficients affect
   corrected vectors while raw vectors and simulated rotator behavior remain.
2. Save, power-cycle node 2, and verify the calibrated/saved flags and heading
   return without retransmitting coefficients.
3. Repeat status/save through the Pi route; test link loss while awaiting an
   acknowledgment and confirm the TUI does not invent success.
4. Clear, power-cycle, and verify the node remains uncalibrated.

No physical power-cycle validation has been performed for this increment yet.

The 2026-10-06 bench check flashed CYD COM10 and node 2 COM7 after matching
their configured MAC addresses. `hardware_calibration_smoke.py` passed direct
USB and CYD/ESP-NOW status queries and invalid-matrix rejection without
changing calibration. A live host AC check also received node-confirmed status
through the primary path and a same-PC Pi-style TCP relay. Node 2 reported both
sensors valid (`cf=96`) with no calibration applied or saved. Actual capture,
save, physical power-cycle, and clear acceptance remain pending.
