# System Flow-Down References

This repository does not duplicate or own system requirements, interfaces, or
site configuration. Their controlled source is the FlightTest
system-engineering repository.

## Antenna Controller flow-down

| Controlled source | Relevance to this component |
| --- | --- |
| `System_Architecture.md`, Antenna Controller and Antenna Orientation Module | AC is hosted on the RF Collection Desktop; an ESP32 USB gateway communicates with SURV and REF antenna nodes over ESP-NOW. |
| `Requirements.md`, SR-04 | Point both antennas to the commanded azimuth within a TBD tolerance, report achieved azimuth, and hold both stationary during capture. |
| `ICD_Messages.md`, §3.2 | Input: `antenna_command` from RM to AC via TCP port 31988. |
| `ICD_Messages.md`, §3.3 | Output: `antenna_state` broadcast via UDP port 31989. |
| `ICD_Messages.md`, §3.4 | Packed ESP-NOW packets and compact serial JSON; global reporting control and UTC time transfer. Protocol-7 silent mode and addressed broadcast are specified in [node runtime](node-runtime.md). |
| `SiteGeometry.md` | Physical layout, antenna identities, known pointing, and calibration bearing constraints. |

The upstream documents remain authoritative. Any proposed change to these
requirements or interfaces must be made through the FlightTest
system-engineering change-request process.

## Proposed ICD clarification

The 2026-10-07 runtime change is recorded as CR-11 in the authoritative
`30_systems_engineering/docs/system` checkout: global reporting modes,
quiet duration and >10° motion threshold, clocks, and optional state fields
in proposed `antenna_state` 1.1.0. The controlled ICD and architecture were
updated there, preserving the existing redundant-transport edits.

Reserve TCP 31995 for the optional node-2 serial relay. It transports the same
compact newline-delimited `rotator_command`/`rotator_packet` objects as the
local CYD USB link. This is a redundant AC-to-node transport, not a new
system-level message or software-item requirement.
