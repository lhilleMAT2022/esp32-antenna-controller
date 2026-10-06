# Antenna Pointing Controller (ESP32 / ESP-NOW) — Design Note

Status: proposed design, not built. Recorded 2026-09-25.

## Purpose

Measure and control the azimuth of each rotator-mounted Yagi (REF and SURV).
The existing rotators turn only while the controller's button is held,
alternate direction on each press, have no position feedback, and bounce at
the end of travel.

The local gateway is also an operator console: an operator at the RF
Collection Desktop can inspect status and point an antenna from its display
without logging in to the computer. Antenna control is the first device type;
the gateway and protocol are deliberately extensible to other low-rate remote
sensors and controllers.

## Existing equipment

- Each rotator controller sits in a weather enclosure about 6–8 ft from its antenna and runs from a 15 VAC wall supply.
- The rotator and the antenna's built-in LNA are powered up the coax from that controller.
- See the controlled [system flow-down references](../flowdown.md) for the antenna layout and pointing.

## Hardware

### Antenna node (one per antenna, in its weather enclosure)

- **ESP32**, powered from a 5 V USB adapter in the enclosure. It is mains-powered, so it can listen for commands continuously and needs no battery. Don't tap the rotator controller's supply or the coax.
- **Adafruit STEMMA mini relay (4409)**, wired across the controller's button contacts. It "presses" the button for the node, whatever the button's voltage or polarity, and keeps the ESP32 isolated from the controller.
- **Adafruit LSM303AGR accelerometer and magnetometer (4413)**, mounted on the rotating mast on a non-ferrous standoff at least 30 cm above the rotator:
  - The accelerometer provides tilt compensation. Earth's field dips about 66° here, so every 1° of mount tilt can cause up to about 2.3° of heading error, and that error varies with heading.
  - It connects by an 8 ft cable with a service loop for the rotation, using I2C slowed to 10–50 kHz. If that is unreliable, use a PCA9615 differential I2C extender over Cat5.
- **Optional:** an Adafruit MMC5603 (5579) as a second magnetometer for cross-checking. Its SET/RESET cancels offset drift with temperature.

Not recommended from the parts considered:

- **HMC5883L:** end-of-life, and many boards sold under that name carry the QMC5883L instead.
- **MAX4544 analog switch:** it needs the button circuit to be under 12 V and share a ground with the ESP32, which is unknown.
- **ICM-20948:** its magnetometer is the weakest of the group. Its gyro is a later upgrade if the rotator motor disturbs heading readings while turning.

### CYD gateway and operator console (at the collection PC)

- Use a **Cheap Yellow Display (CYD)** as the ESP32 USB gateway. It relays
  ESP-NOW packets between the remote nodes and the USB serial port, while its
  2.8-inch resistive touch display provides a local operator interface.
- The RF Collection Desktop talks to the CYD with `serialport`; the CYD is the
  one radio bridge for both computer-originated and display-originated
  commands. No special PC networking is required.
- The first supported display screens are:
  - overall gateway/link status and connected-device list;
  - SURV and REF status: heading, target, moving/settled/fault state, sensor
    quality, and last-heard time;
  - antenna controls: select antenna, choose a target or preset bearing,
    command `go_to`, and command `stop`;
  - a future-device view for environmental sensors and controlled-power nodes.
- The CYD shall continue to display status and issue local antenna commands if
  the USB link is absent. It shall visibly show that the host link is absent
  and buffer event records until the link returns.
- Place it near the stairwell glass. If the display location gives poor
  ESP-NOW coverage, use an external ESP-NOW antenna/radio extension rather
  than moving the operator console to an inaccessible location.
- The purchased board must be recorded before firmware is selected. The
  reference CYD is an `ESP32-2432S028`; the dual-USB (`CYD2USB`) variant has a
  different, colour-inverted display configuration. Its USB-C port may not
  work with a USB-C-to-USB-C cable, so the deployment cable/adapter must be
  qualified with the actual board.

## Radio link

- ESP-NOW (2.4 GHz, acknowledged), with all nodes on one fixed Wi-Fi channel at reduced transmit power.
- 2.4 GHz is well away from the UHF TV band, but REF's broadband LNA (20 MHz–4 GHz) covers it. Keep transmissions sparse (see Behaviour) and record heartbeat times in the capture metadata.
- The bring-up firmware shows directional link RSSI for each node: `N>G` is
  measured at the CYD gateway and `G>N` is measured at the node and reported
  in its subsequent packet. RSSI is a link-quality indicator, not a calibrated
  power measurement.
- ESP-NOW peer encryption is required before control commands are enabled.
  The bring-up heartbeat test is explicitly unencrypted. The deployed protocol
  shall provision a unique key per node, authenticate command origin, and
  reject replayed or stale command sequence numbers.

## Behaviour

### V0.1 CYD/operator-model prototype

The current firmware is a safe, non-actuating prototype. The default CYD
screen displays both nodes, their reported true bearing, online state, a
flashing `* SLEWING` indicator, and touch-active `-10`, `Az`, and `+10`
controls. `Az` opens the calculator-style keypad. The radio diagnostic screen
remains selected through the USB serial interface.

Touch uses the CYD's separate XPT2046 SPI bus (clock 25, MOSI 32, MISO 39,
CS 33, IRQ 36), not the display SPI bus. The four-point, landscape calibration
is stored in ESP32 nonvolatile preferences and therefore survives reset.

The CYD accepts `TIME`, `AZ`, `STEP`, `QUEUE`, `STOP`, `OVERLAP`, `PANEL`,
`KEYPAD`, `KEY`, `STATUS`, and `HELP` commands at 115200 baud. UTC time is
set by `TIME` and distributed to nodes every three seconds. Each node can hold
four future bearing requests and executes them against its synchronized local
time. This prototype's node code models the button/rotator only; it never
energizes a relay.

The model has a 2.5 rpm (15°/s) maximum rotation rate, two-second acceleration
and deceleration, 0–370° mechanical travel, and end-stop reflection. Its
virtual button alternates direction after each press. The mast-overlap start
is a per-node, configurable true bearing (`OVERLAP`; initial 180° true).
Nodes select the nearest representation of a target in the 0–10°/360–370°
overlap region.

The detailed draft specification and test plan are in
[`docs/specs/algorithms/rotator-controller`](../specs/algorithms/rotator-controller).

**Idle**

- Read the heading about once per second.
- Transmit only when the heading changes by more than about 0.5°, or as a 10 s heartbeat. Antennas don't move during captures, so the link is quiet then.

**`goTo(deg)`**

1. Close the relay and watch the heading at about 10 Hz.
2. If the antenna turns the wrong way, release, pause, and press again; the next press turns it the other way. The node learns the toggle state from the sign of the heading change, so it corrects itself if it loses track.
3. Release a few degrees early to allow for coast.
4. Failsafes: release if the heading stops changing (end of travel, before the bounce), if the link drops, or after a maximum hold time.

**Command ownership and safety**

- A command carries an origin (`host` or `local`), node ID, sequence number,
  and command ID. The CYD logs every accepted command and forwards the same
  status/event record to the desktop when USB is available.
- `stop` is always accepted immediately and has priority over any `go_to`
  command, independent of its origin.
- Motion control uses a per-node ownership lease. A local command takes a
  visible manual-control lease; host motion commands are rejected until the
  lease is released or expires. Either the host or local console may issue
  `stop`.
- The antenna node, not the CYD or desktop, owns the relay fail-safe. It must
  release the relay on its maximum-hold, stall, and link-loss conditions even
  if the gateway or desktop is unresponsive.

**Packet contents**

- Node ID and sequence number.
- True heading.
- Raw magnetometer and accelerometer vectors.
- Total field strength, which flags local disturbances such as a car parking near an antenna.
- A moving flag.

## Extensible remote-device boundary

The CYD is a small site gateway, not an antenna-only display. ESP-NOW device
packets shall identify the device ID, device type, sequence number, capability
set, command/result, UTC or gateway-relative timestamp, and health/fault
state. The compact antenna packet remains the first device profile.

Planned device profiles include:

- **antenna node:** heading, tilt, magnetic-field quality, relay state, and
  `go_to` / `stop` / calibration commands;
- **environmental node:** temperature, humidity, supply voltage, and sensor
  health;
- **controlled-power node:** commanded outlet state, measured/declared outlet
  state, and power-cycle result.

The CYD shall render an unknown device as status-only until it recognizes the
device profile and its capabilities. Adding a device profile must not change
the antenna-node safety behavior or packet contract.

### Controlled-power safety boundary

The CYD and ESP32 nodes must never switch 120 VAC directly. A later
controlled-power node will command an appropriately rated, enclosed,
approved-for-purpose switching device (for example, a certified remote outlet
or contactor interface) through its low-voltage control interface. Power-cycle
actions require an explicit device selection and confirmation, a configured
off interval, and a result/fault record; they must default to no action after a
restart or communications loss.

## Calibration

1. Sweep each antenna through its full travel while logging raw magnetometer data, and fit the hard- and soft-iron offsets (ellipse or ellipsoid fit).
2. Convert magnetic to true bearing with the local declination (about 14° W at Natick; check the current value with the NOAA calculator).
3. Anchor to RF truth by rotating each Yagi while logging channel levels. The main DTV towers are at about 83° true for both antennas. The Pluto injector's bearing gives a second reference once it is placed on the REF–SURV leg. Note that from SURV the stairwell shadows the Hudson transmitter (309°), so don't use channel 22 as a SURV reference.

Take heading readings only while the rotator is stopped, since the motor's field can disturb the magnetometer while it runs.

## MATLAB integration

- A `rotatorLink` class wrapping `serialport`, with `heading(node)`, `goTo(node, deg)` and a logger.
- The class uses the same command/status protocol as the CYD display, so a
  host command and a local command produce identical node behavior and audit
  records.
- `runPlutoAzimuthEnvironmentalScan` calls it instead of prompting the operator for bearings, which turns the azimuth scan into an unattended sweep.
- Every capture manifest records both antenna headings, with their field-strength and tilt quality flags.

## Open items before firmware

- Which CYD variant is on hand (board marking, USB connector(s), and display
  configuration), and whether its radio placement has acceptable ESP-NOW range.
- Which ESP32 boards are on hand for the antenna nodes (model or Adafruit
  product number), so the STEMMA QT port and pins can be set.
- Mounting details for the sensor standoff and the cable service loop on each mast.
- Local-screen preset bearings, manual-control lease duration, and the
  required confirmation/authorization policy for local and host commands.
- The approved low-voltage interface and switching hardware for any future
  120 VAC controlled-power node.

## v0.1 hardware-integration handoff

The v0.1 three-board prototype has been built, flashed, and exercised with
the CYD gateway and two identified antenna nodes. It establishes the
communication, operator UI, time synchronization, command queue, and
non-actuating rotator model needed for the next increment.

The incoming parts define that next increment:

1. **Relay interface:** integrate the Adafruit STEMMA non-latching mini relay
   (4409) as a momentary, isolated connection across the physical rotator
   button contacts. Add a node hardware-abstraction interface whose default
   state is released; test it first with the rotator disconnected.
2. **Orientation interface:** integrate the Adafruit LSM303AGR STEMMA QT/Qwiic
   accelerometer/magnetometer (4413). Add raw field, tilt, sensor-health, and
   calibrated true-bearing telemetry before making it the control source.
3. **Safety and calibration:** confirm the exact node pinout and power
   arrangement, document mast placement and cable routing, perform
   magnetometer hard-/soft-iron calibration, and retain the model as a
   selectable bench-test fallback.

No 120 VAC switching is in scope for this increment. Any later power-control
node remains a low-voltage controller for an approved external switching
device.

### LSM303AGR bring-up status

Node 2 enables the LSM303AGR accelerometer and LIS2MDL magnetometer on
`GPIO21` (SDA) and `GPIO22` (SCL). It reports sensor presence, raw acceleration,
raw magnetic field, field magnitude, roll, pitch, and a provisional magnetic
heading to the CYD and both USB serial consoles.

This first increment deliberately keeps sensor heading separate from the
simulated control azimuth. The magnetic heading is uncalibrated and not tilt
compensated; it must not drive a physical rotator. The next steps are to verify
axis signs and mounting orientation, collect full-rotation calibration data,
apply hard-/soft-iron correction and local magnetic declination, then validate
true heading before selecting the sensor as the controller feedback source.

### Redundant host communications

The host-side Python Antenna Controller is the adapter between the system ICD
and the ESP32 network. Its primary path is USB serial to the CYD followed by
ESP-NOW to either remote node. Node 2 also supports an optional backup path
over the rooftop Ethernet network to a Raspberry Pi, then USB serial directly
to node 2.

The AC listens for `antenna_command` on TCP 31988 and publishes
`antenna_state` on UDP 31989. The Pi serial relay listens on TCP 31995 and
passes compact newline-delimited JSON without making control decisions. AC
prefers the CYD path, uses only one command path at a time, and fails node 2
over only when primary telemetry is stale or a bench-test override is set.

This transport is development-only and unauthenticated. Before the physical
relay output can be enabled, TCP 31995 must be restricted to the isolated
data-network interface/firewall and command replay/authentication controls
must be added.
