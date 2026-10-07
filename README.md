# Antenna Controller

Independent development repository for the Apple Hill passive-bistatic-radar
Antenna Controller (AC).

The component controls and reports the pointing of the SURV and REF
rotator-mounted Yagis. Its intended implementation comprises a Cheap Yellow
Display (CYD) USB gateway and local operator console, two ESP32 antenna nodes
connected over ESP-NOW, node firmware, and a host-side control adapter.

## Current development snapshot — 2026-10-07

Both nodes now appear in the TUI calibration panel with A/B XYZ and
magnitude/azimuth°/elevation° (MAE). Health shows actual reporting mode,
quiet time remaining, node/CYD/PC UTC comparisons, sync correction and age.
The CYD debug view shows both nodes' vectors and sample age, omits roll/pitch,
and has a live UTC header once the PC supplies time.

Use the **Reporting** button or `report continuous`, `report normal`,
`report quiet 300`. Normal is the boot default (10-second radio reports).
Quiet sends a heartbeat every minute and sensor reports for unexpected
rotation **>10°**; commanded movement gets 10-second updates and a completion
report. Its timer returns the nodes to normal. Serial remains at 1 Hz.
Calibration capture automatically requests continuous for both nodes and
waits for their acknowledgments.

Protocol **4 requires updating CYD and both remote nodes together**. The
runtime increment has passed 44 host/native/TUI tests and all three firmware
builds. CYD (COM10), node 1 (COM11), and node 2 (COM7) were flashed and verified
on 2026-10-07. The hardware check passed continuous/normal/quiet RF cadence,
quiet expiry, UTC synchronization, and 1 Hz USB telemetry in every mode.
Both nodes have valid accelerometer and magnetometer readings (`sf=15`) and
are left in normal reporting. The live TUI check passed both-node displays,
UTC, mode acknowledgments and quiet countdown. See the
[deployment evidence](docs/verification/2026-10-07-runtime.md).
Physical relay actuation and calibration save/power-cycle acceptance remain
unfinished. See [node runtime specification](docs/node-runtime.md) for rates,
wire fields, clock interpretation and verification. The snapshots below are
historical checkpoints, not the current feature list.

## Development checkpoint — 2026-10-06 (before calibration)

Recovered from session `01a10de7-c1c2-7c92-8e38-c253e108b109`.
This snapshot preserves the v0.2 host application and sensor bring-up before
the persistent-calibration increment; committed as `e630a6e`.

- Implemented: CYD/ESP-NOW gateway, two non-actuating rotator models,
  node-2 LSM303AGR raw vectors, compact JSON serial, host TCP commands/UDP
  state, optional Pi serial relay, and the Textual dashboard.
- Latest completed work: serial reconnection and COM reassignment handling,
  POSIX serial paths, configurable serial settings, `--scan-serial`, TUI
  `SYSTEM` logging, and one `LOST`/`RESTORED` report per link transition.
- Validation at recovery: all 13 Python tests pass and all three PlatformIO
  targets build. The earlier session recorded direct node-2 sensor telemetry
  and a same-PC forced-backup hardware smoke test passing. Automatic failover
  and complete disconnect/reconnect behavior still need dedicated validation.
- Last bench arrangement: node 2 powered separately and communicating through
  CYD/ESP-NOW; the optional Pi relay was off. The recovery scan found CH340
  on COM10 and no CP210x node. COM6/COM7 below are historical examples;
  enumerate devices before connecting or uploading.
- Calibration is not implemented at this checkpoint. The TUI's
  `UNCALIBRATED` label is a placeholder, magnetic heading is diagnostic,
  and rotator feedback remains simulated. No relay GPIO is assigned.

The next authorized increment is guided sensor capture, fitting and
validation, applying corrections on node 2, explicit save/clear commands,
ESP32 Preferences/NVS persistence across power loss, and calibration status
reported by the node to the TUI. The previous attempt stopped on tool failures
before changing files or flashing firmware. The intended sensor mounting is
a 60 cm non-ferrous extension behind the Yagi reflector; mounting alignment
and a true-bearing reference must be established during installation.

Separate review follow-ups remain: end-to-end command acknowledgment and
duplicate suppression; host-to-CYD clock synchronization; independent sensor
freshness and ordering across routes; explicit simulated/measured azimuth;
stronger board identity and non-USB Linux reconnection; mounting-aware tilt;
TUI graph-axis/log retention checks; and structured TCP errors when a command
route is unavailable. Physical relay control is a later increment.

## Persistent calibration increment

The calibration software now provides guided host capture, 3-D magnetometer
fitting, six-face accelerometer fitting, mounting-axis/declination setup, and
node-confirmed apply/save/clear operations through CYD or the optional relay.
Each sensor-equipped node stores a validated, versioned calibration record in
ESP32 NVS and reloads it at boot. Raw vectors remain available and rotator feedback remains
simulated. Physical capture and power-cycle acceptance are still pending.

Verification on 2026-10-06: 25 host/native tests pass, all three firmware
targets build, and the CYD (COM10) and node 2 (COM7) have been flashed.
Live calibration status and invalid-coefficient rejection passed over direct
USB and CYD/ESP-NOW. Host AC received node-confirmed status through both its
primary route and the same-PC Pi-style relay. No calibration was applied or
saved during these checks. Git origin is configured; commits remain local.

Update the CYD and node-2 firmware together, synchronize the Python environment
with `uv sync --inexact`, and press `c` or click **Cal** in the TUI. Follow the
[calibration procedure](docs/calibration.md) for capture, validation, explicit
saving, and the hardware acceptance checks. The checkpoint above records the
state before this increment.

The first operator calibration run passed the magnetometer fit (reported
residual 0.0484) but failed the final accelerometer consistency check with
"Six-face validation failed; check alignment and repeat". This indicates at
least one corrected face exceeded 0.08 g of vector error; the affected face and
physical cause have not yet been identified. The original host retained
raw captures only in memory, so this attempt has no automatic disk archive.
The host now checkpoints samples and fit attempts under `calibration_captures/`,
supports `cal 2 export`, and reports per-face accelerometer errors and limits.
This requires an AC restart to activate; it cannot recover samples from the
older running process. All 34 host/native/TUI tests pass, including a steady
tilt that passes preliminary gates, failed-fit archival, face replacement,
and disk-write failure recovery. Calibration save/power-cycle acceptance
remains pending. See the
[capture diagnostics](docs/calibration.md#capture-files-and-failed-fit-diagnostics).

## Node 1 sensor and CYD status update — 2026-10-07

Node 1 was identified by MAC `5c:01:3b:34:44:d8` on COM11 and flashed with
LSM303AGR sampling and calibration support. Both its magnetometer and
accelerometer report valid data (`sf=15`, `cf=96`). Calibration status and
invalid-matrix rejection passed over direct USB and through CYD/ESP-NOW.
The host calibration panel and help now follow the selected node or the node
specified in a calibration command; use `cal 1 ...` for this new sensor.

CYD COM10, MAC `04:b2:47:82:97:18`, was flashed with the link-status fix.
Previously, a zero-initialized receive timestamp falsely indicated ONLINE
at boot, and silent-node expiry did not request a screen redraw. The display
now starts OFFLINE and redraws on packet receipt, timeout, and recovery.

All 35 host/native/TUI tests and all three firmware builds passed. The live
link test held Node 1 in reset: CYD reported OFFLINE 4.32 seconds later
(five seconds after the last received packet), kept Node 2 ONLINE, and
restored Node 1 ONLINE with valid sensor data after reset was released.
This verified the serial link transitions and the firmware path that requests
the card redraw. Visual confirmation on the physical display remains an
operator check.
Node 2 was not reflashed or reset during this update. It was communicating
but reported no detected sensors (`sf=0`, `cf=0`); its sensor connection still
needs checking. No calibration was applied, saved, or cleared by these tests.

## Scope

- CYD gateway/operator-console and antenna-node firmware
- ESP-NOW and USB-serial protocol
- Heading sensing, calibration, safe rotator control, and telemetry
- Host-side controller API and integration tests
- An extensible device model for later environmental-sensor and remote-power
  nodes

The RF collector, N320, and Pluto azimuth-scan implementation stay in the
FlightTest repository. The controller must provide the stable command and
state interfaces that let those systems automate a scan.

## Controlled system references

System requirements, architecture, and message contracts are controlled in
the separate FlightTest system-engineering checkout:

- `docs/system/System_Architecture.md`
- `docs/system/Requirements.md` (SR-04)
- `docs/system/ICD_Messages.md` (§3.2–§3.4)
- `docs/system/SiteGeometry.md`

Those documents define system-level intent. This repository owns the
component design and implementation.

## Source material

`docs/design/antenna_controller.md` was imported from:

- repository: `https://github.com/pwilliamMAT/flightTest`
- branch: `feature/pluto-azimuth-environment-scan`
- commit: `48fec02bedbc4baec964797dd225ec2314383712`
- original path: `TestSetupTesting/antenna_controller.md`

The project origin is `https://github.com/lhilleMAT2022/esp32-antenna-controller.git`.
No FlightTest Git remote is configured for this repository.

## Firmware v0.1: control-panel and rotator-model prototype

The PlatformIO project contains three hardware prototype environments:

- `cyd_gateway`: default operator display for both nodes, a serial command
  gateway, visual keypad mock-up, selected-only debug panel, and ESP-NOW time
  synchronization.
- `antenna_node_1` and `antenna_node_2`: real-time, **non-actuating** rotator
  models. They report azimuth and virtual-button/motion state but do not drive
  a relay.

Build both with:

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" run
```

The current two-board bench mapping is CYD `COM6` (CH340) and node 2 `COM7`
(CP210x). Windows can reassign these ports, so they are intentionally not
stored in `platformio.ini`; verify them with `pio device list` before uploading:

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" run -e cyd_gateway -t upload --upload-port COM6
& "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" run -e antenna_node_2 -t upload --upload-port COM7
# Flash node 1 with its verified port when it is connected.
```

The peer MAC addresses and the fixed ESP-NOW channel used by this prototype
are in `firmware/espnow_smoke_config.h`. They are intentionally explicit so a
future deployed configuration does not silently choose a channel or peer.

The CYD begins on the two-node operator panel. Its `-10`, `Az`, and `+10`
controls are touch-active after the one-time four-point calibration, which is
stored in ESP32 nonvolatile preferences. `PANEL DEBUG` selects the diagnostic
panel with directional RSSI (`N>G` measured at the gateway and `G>N` measured
at the node). `PANEL HOME` restores the normal display.

Use `HELP` on the CYD's 115200 baud serial port for the v0.1 command grammar.
The detailed draft design is in
[`docs/specs/algorithms/rotator-controller`](docs/specs/algorithms/rotator-controller).
The packet format is versioned and deliberately unencrypted.

## Hardware-integration readiness

The v0.1 checkpoint has been built, flashed, and exercised as a three-board
prototype. The two antenna nodes remain simulator-only: no relay GPIO is
assigned and no attached hardware can be actuated.

The next increment is reserved for the incoming hardware:

- an Adafruit STEMMA non-latching mini relay (product 4409) to emulate the
  rotator's momentary toggle button through an isolated low-voltage interface;
- an Adafruit LSM303AGR accelerometer/magnetometer (product 4413) for
  measured mast orientation, tilt, and magnetic-quality telemetry.

Before enabling either interface, record the actual node-board pinout and
mounting arrangement, add a hardware-abstraction layer with outputs disabled
by default, and complete a bench test before connecting to a rotator.

### Remote-node LSM303AGR sensors

Both remote-node firmware targets enable the Adafruit LSM303AGR on their
default I2C bus:

| Qwiic wire | Signal | ESP32 pin |
|---|---|---|
| Red | 3.3 V | `3V3` |
| Black | Ground | `GND` |
| Blue | SDA | `GPIO21` |
| Yellow | SCL | `GPIO22` |

The node samples acceleration and magnetic field at 10 Hz and sends a
diagnostic packet once per second. The CYD home panel shows `Mag ...M` for
both nodes. The debug panel currently shows Node 2's magnetic heading,
`Bx/By/Bz` in microtesla, `Ax/Ay/Az` in g, field magnitude, roll, and pitch. USB serial output includes
the same raw vectors.

The CYD magnetic heading marked `M` remains a raw diagnostic. The calibration
increment supplies a separate corrected, tilt-compensated true heading after
sensor calibration and mounting/declination setup. Neither replaces the
simulated rotator feedback yet; see [calibration](docs/calibration.md).

The CYD home cards start **OFFLINE** until a valid packet arrives from the
configured node MAC. Status, command acknowledgments, raw sensor telemetry,
and calibration reports all count as communication. After five seconds
without a packet, the card redraws **OFFLINE** even when neither node sends
anything. Receipt of a new valid packet restores **ONLINE**. Raw sensor
readings expire separately after three seconds, and stale motion flags do not
keep the slewing indicator blinking while offline. Link transitions appear
in the CYD event log and USB serial output as `LINK N1 OFFLINE`, for example.

For an idle bench node with a standard ESP32 USB auto-reset circuit,
`tests/hardware_link_smoke.py --gateway-port COM10 --reset-node-port COM11
--node 1` exercises link loss and recovery by holding that node in reset.
This discards its unsaved RAM settings; it does not change NVS or send rotator
commands. The other node must remain connected. Verify port identities first.
`tests/hardware_calibration_smoke.py` also accepts `--node 1` or `--node 2`.

## Host Antenna Controller and backup relay

The `antenna_controller` Python module can run in either of two roles:

- `ac`: the Antenna Controller process on the RF Collection Desktop. It reads
  compact JSON telemetry from the CYD, accepts `antenna_command` JSON lines on
  TCP 31988, and publishes `antenna_state` datagrams on UDP 31989.
- `relay`: the optional Raspberry Pi process beside node 2. It bridges TCP
  31995 to node 2's USB serial port. Both node firmwares expose this same
  backup serial interface.

Create or synchronize the project environment:

```powershell
uv sync
```

For same-PC integration testing, start the node-2 relay and AC in separate
terminals (replace the COM ports if Windows reassigned them):

```powershell
uv run python -m antenna_controller relay --serial COM7 --listen-host 127.0.0.1
uv run python -m antenna_controller ac --serial COM6 --backup-host 127.0.0.1 --tui
```

Linux serial paths such as `/dev/ttyACM0`, `/dev/ttyUSB0`, and persistent
`/dev/serial/by-id/...` paths are accepted directly. To inspect available
devices without opening or resetting them:

```bash
uv run python -m antenna_controller --scan-serial
```

Both `ac` and `relay` accept `--baud`, `--data-bits`, `--parity`,
`--stop-bits`, `--read-timeout`, `--write-timeout`, `--xonxoff`, `--rtscts`,
and `--dsrdtr`. Defaults match the ESP32 firmware: 115200 baud, 8 data bits,
no parity, 1 stop bit, and no flow control.

The Textual TUI provides scrolling plots for antenna azimuth/target,
`Bx/By/Bz`, `Ax/Ay/Az`, and directional ESP-NOW RSSI. Use `m` to cycle graph
modes, `t` to cycle time windows, `n` to select nodes, `o` to allow or block
external client commands, and `u` to focus the manual-command field. All
selectors and the command entry are also mouse-accessible.

The AC uses the CYD/ESP-NOW path by default. For node 2 it fails over to the
Pi route when gateway telemetry is stale and backup telemetry is fresh.
`--prefer-backup-node2` forces the backup route for a bench test. On the Pi,
use `/dev/serial/by-id/...` rather than a changing `/dev/ttyUSB*` name.
Serial links reopen automatically after disconnect or stale telemetry. On
Windows, the process remembers each board's USB identity and follows it if
the operating system assigns a different COM number after reconnection.
Link warnings are routed to the Textual event pane as `SYSTEM` reports.
Missing serial or Pi-relay links produce one `LOST` report and remain quiet
until a `RESTORED` transition occurs.

Example TUI commands are `goto 2 180`, `step 2 -10`, `stop 2`, and `status`.
An ICD command can also be sent directly to TCP 31988 as newline-delimited
JSON:

```json
{"message_type":"antenna_command","antenna":"REF","command":"go_to","target_az_deg":180.0}
```

The board serial format is compact newline-delimited JSON: `rc` is a command,
`rp` is rotator status, `rs` is sensor telemetry, and `ra` is an acceptance or
error response. Human-readable firmware diagnostics remain present and are
ignored by the Python process.

The Pi listener is unauthenticated and is intended only for the isolated
rooftop data network. Bind it to the Pi's data-network address and restrict
TCP 31995 at the host firewall before physical relay actuation is enabled.
Example `systemd` units and environment files are in [`deploy`](deploy).
