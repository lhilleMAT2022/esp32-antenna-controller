# ESP32 memory budget — 2026-10-07

Protocol-6 firmware, from successful PlatformIO builds:

| Target | Code + read-only data | Application limit | Used | App space remaining | Static data RAM |
|---|---:|---:|---:|---:|---:|
| CYD | 835,889 bytes | 1,310,720 bytes | 63.8% | 474,831 bytes | 46,424 / 327,680 bytes (14.2%) |
| Node 1 | 809,849 bytes | 1,310,720 bytes | 61.8% | 500,871 bytes | 46,836 / 327,680 bytes (14.3%) |
| Node 2 | 809,865 bytes | 1,310,720 bytes | 61.8% | 500,855 bytes | 46,836 / 327,680 bytes (14.3%) |

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

The scheduled-slew increment added about 4.3 KiB of code on CYD and 5.5 KiB
on each node, plus 912 bytes of node static RAM for queue/replay state.
