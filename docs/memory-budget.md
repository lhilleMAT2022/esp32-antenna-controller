# ESP32 memory budget — 2026-10-07

Protocol-5 firmware, from successful PlatformIO builds:

| Target | Code + read-only data | Application limit | Used | App space remaining | Static data RAM |
|---|---:|---:|---:|---:|---:|
| CYD | 831,581 bytes | 1,310,720 bytes | 63.4% | 479,139 bytes | 46,424 / 327,680 bytes (14.2%) |
| Node 1 | 804,261 bytes | 1,310,720 bytes | 61.4% | 506,459 bytes | 45,924 / 327,680 bytes (14.0%) |
| Node 2 | 804,265 bytes | 1,310,720 bytes | 61.4% | 506,455 bytes | 45,924 / 327,680 bytes (14.0%) |

Each board has **4 MiB (4,194,304 bytes) of flash**. The default ESP32 partition
layout gives each application slot **1.25 MiB**; there are two slots for OTA,
plus a 1.375 MiB filesystem partition, 20 KiB NVS, OTA metadata, core-dump and
boot/partition areas. The existence of OTA slots does not mean this project
has implemented over-the-air updates. Current uploads use USB serial.

The compiler's firmware percentage is relative to **one application slot**,
not the entire flash chip. A different partition layout could allocate more
flash to the application later; it is not needed for the current build.

Static RAM percentages cover linked data and BSS against the toolchain's
320 KiB application data-RAM budget. They do **not** include all runtime Wi-Fi
allocations, task stacks, JSON allocation or temporary buffers. The remaining
percentage is therefore not a measurement of live free heap. Runtime minimum
free heap, largest free block and stack high-water marks have not been measured
by this deployment check.

Compared with protocol 4, timing discipline and scheduling added 2,212 bytes
of code and 96 bytes of static RAM on CYD; on node 1, 2,660 bytes of code and
232 bytes of static RAM. Flash headroom is comfortable; no memory partition
or calibration-storage changes were made.
