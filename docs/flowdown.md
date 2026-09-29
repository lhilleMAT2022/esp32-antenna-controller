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
| `ICD_Messages.md`, §3.4 | Gateway/node protocol constraint: short-key ESP-NOW packets, acknowledged control, status at up to 10 Hz while moving and 10 s idle heartbeats. |
| `SiteGeometry.md` | Physical layout, antenna identities, known pointing, and calibration bearing constraints. |

The upstream documents remain authoritative. Any proposed change to these
requirements or interfaces must be made through the FlightTest
system-engineering change-request process.
