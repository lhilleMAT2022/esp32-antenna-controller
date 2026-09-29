# Antenna Controller

Independent development repository for the Apple Hill passive-bistatic-radar
Antenna Controller (AC).

The component controls and reports the pointing of the SURV and REF
rotator-mounted Yagis. Its intended implementation comprises a Cheap Yellow
Display (CYD) USB gateway and local operator console, two ESP32 antenna nodes
connected over ESP-NOW, node firmware, and a host-side control adapter.

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

The current development-machine device mapping is expected to be CYD `COM8`
(CH340) and antenna node `COM7` (CP210x). These ports are intentionally not
stored in `platformio.ini`; use the appropriate port when uploading:

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" run -e cyd_gateway -t upload --upload-port COM8
& "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" run -e antenna_node_1 -t upload --upload-port COM7
& "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" run -e antenna_node_2 -t upload --upload-port COM9
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
