# Antenna Controller TUI UX Specification

## Overview

The Antenna Controller (AC) TUI is an SSH-accessible operational console for a distributed ESP32-based antenna pointing system.

The system consists of:

- A Head Controller ESP32
- Multiple Remote AC ESP32 Rotator Nodes
- TCP-connected clients (part of RF Collection System)
- ESP-NOW wireless communications
- Optional Ethernet/UDP communications to Remote AC ESP32 Rotator Nodes

The TUI communicates with the Head Controller over a serial connection and provides a unified operational view of all connected nodes, clients, sensor data, commands, and system health.

Primary goals:

- Real-time situational awareness
- Manual antenna control
- Rotator Calibration workflows
- Sensor visualization
- Communications debugging
- Low-bandwidth operation over SSH
- Single-screen operation with mouse interaction to select subdisplays

---

# Design Principles

The interface should prioritize:

1. Antenna status over system status
2. Network health over infrastructure details
3. Operational awareness over debugging details
4. Keyboard + mouse operation
5. Readability over visual complexity
6. Robust operation slow SSH links

The overall design should feel more like a radar operations console than a conventional system monitor.

---

# Screen Layout

```text
+------------------------------------------------------------------+
| Status Bar                                                       |
+------------------------------------------------------------------+
|                                                                  |
|                                                                  |
|          AZIMUTH HISTORY / SENSOR DISPLAY                        |
|                                                                  |
|                                                                  |
+--------------------------------------+---------------------------+
|                                      |                           |
| Health / Node Status                 |                           |
|                                      |                           |
| Heartbeats                           |   Raw Message Log         |
|                                      |                           |
| Calibration Summary                  |                           |
|                                      |                           |
+--------------------------------------+                           |
| Control Bar                          |                           |
+--------------------------------------+---------------------------+
```

Layout Allocation:

```text
Header:                1 row
Main Graph Area:      33%
Bottom Left:          33%
Bottom Right:         66%
Footer:                1 row
```

---

# Header Status Bar

The header is always visible.

Example:

```text
AntennaCtl v0.8

ONLINE

Nodes: 2/2
Clients: 3

ESP-NOW: OK
UDP: OK
SERIAL: OK

14:03:31 UTC
```

Status Color Conventions:

## Green

```text
ONLINE
CONNECTED
LOCKED
CALIBRATED
TRACKING
```

## Yellow

```text
STALE
PARTIAL
DEGRADED
```

## Red

```text
OFFLINE
ERROR
LOST
UNCALIBRATED
```

---

# Main Graph Area

## Default View: Antenna Tracking

The primary display shows historical antenna pointing information.

Displayed Traces:

- Node 1 Actual Azimuth
- Node 1 Commanded Azimuth
- Node 2 Actual Azimuth
- Node 2 Commanded Azimuth

Example:

```text
360|
350|
340|
...
180|
...
  <------------------------------------0
          Historical Time (scrolling left - older to the left)
```

Color Scheme:

```text
Green     Node 1 Actual
Yellow    Node 1 Target

Blue      Node 2 Actual
Magenta   Node 2 Target
```

Available Time Windows:

```text
30 seconds
60 seconds
2 minutes
5 minutes
15 minutes
```

Display Characteristics:

- Horizontal scrolling history
- Auto-scaling optional
- Fixed 0-360° mode optional
- Zoom selection via keyboard

---

# Sensor Display Modes

The main graph may be switched from tracking mode into sensor mode.

Hotkey:

```text
m - Mode
```

Display cycle:

```text
TRACK
MAGNETOMETER Bx,By,Bz
ACCELEROMETER Ax, Ay, Az
RSSI for ESP-NOW packets
```

---

## Magnetometer View

Displays raw magnetic field for a selected node.

Signals:

```text
Bx
By
Bz
```

Units:

```text
uT (microtesla)
```

Purpose:

- Heading calibration
- Magnetic interference detection
- Sensor diagnostics
- Installation verification

Selectable Node:

```text
All
Node 1
Node 2
```

---

## Accelerometer View

Displays raw accelerometer readings.

Signals:

```text
Ax
Ay
Az
```

Purpose:

- Wind-induced motion
- Vibration monitoring
- Accidental movement detection
- Sensor diagnostics

Selectable Node:

```text
All
Node 1
Node 2
```

---

## RSSI View

Displays reported RSSI readings.

Signals:

```text
A. RSSI Head->Node 1
B. RSSI Head->Node 2
C. RSSI Node 1->Head
D. RSSI Node 2->Head
```

Purpose:

- debugging communications

Selectable Node:

```text
Single Signal
All Node 1 signals
All Node 2 signals
```

---

# Node Health Panel

Located in the lower-left section.

Purpose:

Provide an operational summary of all connected systems.

Example:

```text
NODE STATUS

Node 1
-------
Last Seen:     0.2 sec
Transport:     ESP-NOW
Heading:       42.5°
Target:        44.0°
RSSI:         -61 dBm

Node 2
-------
Last Seen:     0.1 sec
Transport:     ESP-NOW, UDP Serial
Heading:      225.1°
Target:       226.0°
RSSI:         -58 dBm

AC Client
--------------
192.168.7.41 - Port 54321
TCP Active
Last Seen:     0.4 sec
```

---

# Heartbeat Visualization

Located beneath Node Status.

Purpose:

Visualize communication health.

Example:

```text
Node1 ESP-NOW   ████████████████████████
Node2 ESP-NOW   ████████░███████████████
Node2 UDP       ████████░███████████████
AC Client TCP   ██████████████████░█████
```

Available Time Windows:

```text
30 seconds
60 seconds
2 minutes
5 minutes
```

Display:

```text
Green = heartbeat received
Red   = heartbeat missed
Gray  = no expected heartbeat
```

Benefits:

- Quickly identify intermittent links
- Visualize packet loss patterns
- Detect communication degradation

---

# Calibration Summary Panel

Located within the lower-left section.

Example:

```text
Node 1

Heading Offset: +3.2°
Last Cal:       12:43

MAG Span:       GOOD
Tilt Bias:      GOOD


Node 2

Heading Offset: -1.5°
Last Cal:       13:10

MAG Span:       FAIR
Tilt Bias:      GOOD
```

Future Diagnostics:

- Magnetometer calibration quality
- Hard iron compensation
- Soft iron compensation
- Accelerometer calibration quality

---

# Raw Message Console

Located on the right side of the display.

Purpose:

Provide a continuously scrolling operational log.

Example:

```text
14:03:12 N1 HB h=44.2
14:03:12 N2 HB h=221.8

14:03:13 CLIENT TRACK 120.2
14:03:13 N1 CMD 120.2
14:03:13 N2 CMD 300.2

14:03:13 N1 ACK

14:03:14 N1 REPORT
14:03:14 N2 REPORT
```

---

# Message Coloring

```text
Green     REPORT
Blue      CLIENT
Yellow    COMMAND
Magenta   ACK
Red       ERROR
Gray      DEBUG
```

---

# Log Filtering

Hotkey:

```text
f
```

Filter Categories:

## Source

```text
All
Head Controller
Node 1
Node 2
Client
```

## Message Type

```text
All
HB
REPORT
COMMAND
ACK
ERROR
DEBUG
CALIBRATION
```

## Transport

```text
All
ESP-NOW
UDP
SERIAL
```

---

# Footer Control Bar

Always visible.

Example:

```text
[h Help]
[g Graph]
[m Manual]
[o Online]
[c Cal]
[x Home]
[f Filter]
[q Quit]
```

---

# Manual Control Mode

Activated by:

```text
m
```

Purpose:

Direct operator control of antenna position.

Example Commands:

```text
az1 +5
az1 -5

az2 +5
az2 -5

point 180

park

stop

follow on
follow off
```

---

## Keyboard Shortcuts

Node 1:

```text
Left Arrow
Right Arrow
```

Node 2:

```text
A
D
```

Optional:

```text
Shift = larger step size
Ctrl  = fine step size
```

---

# Online / Offline Mode

Activated by:

```text
o
```

Purpose:

Enable or disable acceptance of client commands.

States:

## ONLINE

```text
Accept external client commands.
```

## OFFLINE

```text
Ignore client commands.
Allow local manual operation only.
```

Useful during:

- Maintenance
- Calibration
- Troubleshooting
- System testing

---

# Calibration Mode

Activated by:

```text
c
```

Potential Actions:

```text
Start Calibration
Save Calibration
Clear Calibration
Measure Offset
Level Sensor
Compass Verification
```

Display:

```text
Current Heading
Raw Magnetometer
Corrected Heading
Calibration Quality
```

---

# Geometry Widget

Small visual aid located in an unused corner of the display.

Purpose:

Provide immediate understanding of antenna orientation.

Example:

```text
             Aircraft

                 ^
                 |

Node1 ------- + ------- Node2

 42°                    221°
```

Alternative:

```text
N1 --->      <--- N2
```

Useful because operators often recognize geometry faster than numerical values.

---

# Packet Quality Trend

Optional compact panel.

Purpose:

Early detection of communication issues on ESP-NOW wireless links.

Metrics:

```text
Packet Age
Packet Loss
RSSI
Latency
Quality Metric (q)
```

Example:

```text
Q
100|██████████
 90|█████████
 80|███████
 70|██████
```

Benefits:

- Detect fading links
- Detect antenna problems
- Diagnose intermittent ESP-NOW issues
- Diagnose Ethernet network issues

---

# Future Enhancements

## Recording

Allow session recording.

Example:

```text
record start
record stop
```

Capture:

- Commands
- Reports
- Sensor data
- Calibration actions

---

## Playback

Replay recorded sessions.

Useful for:

- Troubleshooting
- Development
- Demonstrations

---

## Alarm System

Trigger visual warnings for:

```text
Node timeout
RSSI low
Calibration invalid
Heading error
Motor fault
Packet loss
```

---

## Target Tracking Summary

Display:

```text
Track Source
Current Target
Target Age
Tracking Error
```

Useful when external clients are providing tracking data.

---

# Success Criteria

An operator should be able to answer the following questions within 5 seconds of viewing the screen:

1. Are all nodes connected?
2. Are all antennas calibrated?
3. Are antennas pointing where expected?
4. Are tracking commands being received?
5. Is communication healthy?
6. Is a node reporting abnormal sensor behavior?
7. Can manual control be safely assumed?

If those questions can be answered quickly and reliably, the TUI is considered successful.

---
# Sample
JSON Messages from the nodes:

```text
N2 rs {"t":"rs","n":2,"q":594,"mh":141.3,"m":[-60.3,48.3,26.5],"a":[-0.016,-0.236,-0.977],"f":81.7,"r":-166.4,"p":0.9,"sf":15,"src":"node_serial"}
N2 rp {"t":"rp","n":2,"q":595,"ts":1791296898,"h":90.0,"tg":null,"mv":0,"e":0,"ack":0,"src":"node_serial"}
N2 rs {"t":"rs","n":2,"q":596,"mh":141.0,"m":[-59.2,48.0,27.0],"a":[-0.019,-0.236,-0.983],"f":80.9,"r":-166.5,"p":1.1,"sf":15,"src":"node_serial"}
N2 rp {"t":"rp","n":2,"q":597,"ts":1791296899,"h":90.0,"tg":null,"mv":0,"e":0,"ack":0,"src":"node_serial"}
N2 rs {"t":"rs","n":2,"q":598,"mh":141.4,"m":[-61.0,48.8,27.0],"a":[-0.018,-0.24,-0.979],"f":82.7,"r":-166.2,"p":1.0,"sf":15,"src":"node_serial"}
N2 rp {"t":"rp","n":2,"q":599,"ts":1791296900,"h":90.0,"tg":null,"mv":0,"e":0,"ack":0,"src":"node_serial"}
```
