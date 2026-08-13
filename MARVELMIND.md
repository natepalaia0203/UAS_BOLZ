# Marvelmind Indoor Positioning — Shared Hardware Guide

Covers the Marvelmind system as used by **both** the CrowDrone (aerial) and
GroundRobots (TurtleBot2) platforms.

**Read this first if you are switching the hedgehog between projects.** The two
platforms require mutually exclusive output protocols, and moving hardware between
them without reconfiguring is the single most likely way to lose an afternoon.

---

## Table of Contents

1. [The Shared Hardware Problem](#1-the-shared-hardware-problem)
2. [Protocol Matrix — Which Setting for Which Platform](#2-protocol-matrix--which-setting-for-which-platform)
3. [Switching Procedure](#3-switching-procedure)
4. [Device Roles](#4-device-roles)
5. [Beacon Placement](#5-beacon-placement)
6. [Dashboard Setup and Map Building](#6-dashboard-setup-and-map-building)
7. [Georeferencing (Drone Only)](#7-georeferencing-drone-only)
8. [Interfaces Panel Reference](#8-interfaces-panel-reference)
9. [NMEA Sentence Configuration (Drone)](#9-nmea-sentence-configuration-drone)
10. [Marvelmind Binary Protocol (Ground Robot)](#10-marvelmind-binary-protocol-ground-robot)
11. [Modem Exclusivity](#11-modem-exclusivity)
12. [Troubleshooting Decision Tree](#12-troubleshooting-decision-tree)
13. [Known Issues Log](#13-known-issues-log)
14. [License Codes and the UBX Trap](#14-license-codes-and-the-ubx-trap)
15. [Accuracy Benchmarks](#15-accuracy-benchmarks)
16. [Quick Reference Cards](#16-quick-reference-cards)

---

## 1. The Shared Hardware Problem

One hedgehog (Super-Beacon-2) currently serves two projects with **incompatible
requirements**:

| Platform | Consumer | Required protocol |
|---|---|---|
| CrowDrone | Pixhawk 6C / ArduPilot | `NMEA 0183` |
| Ground robot | `marvelmind_ros2` ROS 2 driver | `Marvelmind` (binary) |

The hedgehog has **one** output protocol setting governing its UART/USB stream. It
cannot speak both simultaneously. Switching projects means a dashboard trip.

### Why this bites

The failure is silent and misleading. With the wrong protocol set:

- The ROS 2 driver reports `Hedgehog is running!` and `Opened serial port` — then publishes nothing
- ArduPilot reports `No GPS` / `Sats: 0`

In both cases the connection *looks* healthy. Nothing in the logs says "wrong protocol."

> **Encountered August 11, 2026.** The hedgehog was left in `NMEA 0183` from drone
> work. The ground robot's driver connected cleanly and published empty topics for
> a considerable while before the protocol was identified as the cause.

### Long-term fix

Buy a second hedgehog. The kit supports two, each independently configured, and it
removes the switching step entirely. Until then, follow §3 every time.

---

## 2. Protocol Matrix — Which Setting for Which Platform

| Setting | Drone (Pixhawk) | Ground robot (ROS 2) |
|---|---|---|
| Protocol on UART/USB output | `NMEA 0183` | `Marvelmind` |
| UART speed | 115200 | 115200 |
| Streaming output | USB+UART | USB+UART |
| Stream location data | n/a | **enabled** (defaults to disabled) |
| `$GPGGA` | enabled | n/a |
| `$GPRMC` | enabled | n/a |
| `$GPVTG` / `$GPZDA` / `$GPGSA` / `$GPGSV` / `$GPGLL` / `$GPHDT` | disabled | n/a |
| Map GPS origin | **mandatory** | not required |
| Physical connection | UART solder pads → Pixhawk GPS1 | USB → Pi 5 |
| Device node | n/a | `/dev/ttyACM*` |

Baud is 115200 on both sides. The Marvelmind factory default is **500 kbps** — it
must be changed, and ArduPilot does not support 500k for this.

---

## 3. Switching Procedure

### Drone → Ground robot

1. Disconnect the hedgehog from the Pixhawk (or unmount from the drone)
2. Plug the **Super Modem** into the dashboard PC via USB
3. Open Marvelmind Dashboard, select the **hedgehog** in the device list — not the modem
4. Interfaces → **Protocol on UART/USB output** → change `NMEA 0183` → **`Marvelmind`**
5. Confirm **Stream location data** = `enabled`
6. Leave UART speed at 115200
7. Click **Write changes**, then **Read all** to verify it persisted
8. **Unplug the modem from the PC** (see §11 — this step is not optional)
9. Plug the hedgehog into the Pi via USB
10. Verify on the Pi:

```bash
ls -l /dev/ttyACM*
sudo dmesg | grep -i marvelmind
```

### Ground robot → Drone

Same sequence in reverse, plus:

- Set protocol back to **`NMEA 0183`**
- Verify `$GPGGA` and `$GPRMC` are enabled and everything else disabled (§9)
- Confirm the **map GPS origin** is still set (§7)
- Set **Sleep by timeout** and **Sleep with external power** to `never`

### Verify at the source before reconnecting

Fastest confirmation, ~30 seconds. Connect the hedgehog to a PC via USB and open a
serial terminal at 115200:

- **NMEA mode** — readable `$GPGGA` / `$GPRMC` text sentences with populated lat/lon
- **Marvelmind mode** — binary data, not human-readable

If you see the wrong one, the write did not take.

---

## 4. Device Roles

Configuration applied to the wrong device silently does nothing. Three device types:

| Device | Role | Needs config? |
|---|---|---|
| Stationary beacons (×5–6) | Fixed anchors around the room | Placement only |
| **Hedgehog** (Super-Beacon-2) | Mobile unit on the vehicle; computes own position | **Yes — this is the critical one** |
| Modem | Base station, coordinates network, USB to dashboard PC | Only if something is wired to its UART |

**The rule:** output settings must be applied to whichever device is physically wired
to the flight controller or robot computer. In both our builds that is the **hedgehog**.

### Data flow

```
[Stationary Beacons]
      |  ultrasonic ranging
      v
  [Hedgehog]  --radio-->  [Modem]  --USB-->  [Laptop: Dashboard]
      |
      |  UART/USB
      v
  [Pixhawk 6C]  or  [Raspberry Pi 5]
```

---

## 5. Beacon Placement

- Minimum 3–4 stationary beacons; we run 5–6
- **Spread across the full perimeter.** Clustered beacons give poor triangulation
  geometry and cause dropout
- **Vary the mounting heights** — materially improves Z-axis stability
- Mount **0.5–1 m off the ground**, not on the floor. Ultrasound is blocked by floor
  clutter and suffers multipath from ground reflections
- Keep beacons and hedgehog roughly in the same horizontal plane
- Avoid pointing beacons at large reflective surfaces

### Verification checklist

- All stationary beacons show solid green in the Dashboard
- Hedgehog shows a stable position on the map, not jumping
- Inter-beacon distances look sane, not all clustered within a couple metres

Validated configuration: 5 stationary beacons, 1 hedgehog, ~3 × 1 m working area,
all green. This geometry was confirmed adequate and used to rule out placement when
troubleshooting later problems.

---

## 6. Dashboard Setup and Map Building

1. Connect the modem to the PC via USB
2. Open the Marvelmind Dashboard — the modem appears, beacons join as they power on
3. Click the **hedgehog** device to configure it (not the modem)
4. Power all stationary beacons, confirm green
5. Let the system measure inter-beacon distances automatically
6. **Freeze the map** once geometry converges and stops shifting
7. **Save map** (lower left)

Relevant buttons: `Save map`, `Load map`, `Erase map`, `New`, `Unfreeze zones`,
`Unfreeze map`.

> Do not skip saving. Losing the map means re-surveying the room.

Record before unplugging the modem: corner coordinates, working-area dimensions, and
the robot's starting position. The ground robot's Nav2 static map is built from these
numbers.

---

## 7. Georeferencing (Drone Only)

**Mandatory for the NMEA path. Not needed for the ground robot**, which works
entirely in Marvelmind's local X/Y frame.

Marvelmind works in local X/Y/Z with an arbitrary origin. NMEA sentences carry
latitude and longitude. Without a geographic reference the hedgehog has nothing to
convert into, and emits sentences with empty or zeroed position fields.

### Symptom if skipped

```
GPS: No GPS
Sats: 0, hdop: 0.0
Coordinates: 0.0000000, 0.0000000
EKF3 waiting for GPS config data
```

### Procedure

1. Dashboard → map/zone settings (right-click the map area, or submap properties)
2. Find the GPS origin fields — labelling varies by firmware:
   - "GPS coordinates of the submap origin"
   - "GPS coordinates of zero point"
   - "Map origin latitude / longitude"
3. Enter the building's approximate lat/lon. Precision is not critical — the value
   simply needs to exist so the transform is defined
4. Set map rotation / north azimuth to align local with geographic frame
5. Write changes and power-cycle the hedgehog

---

## 8. Interfaces Panel Reference

Expand **Interfaces** on the hedgehog device page.

| Setting | Value | Notes |
|---|---|---|
| Streaming output | `USB+UART` | Both simultaneously is correct and does not conflict |
| UART speed, bps | `115200` | Default is 500 kbps — change it |
| Protocol on UART/USB output | see §2 | **Platform-dependent** |
| Stream location data | `enabled` | Required for ground robot; defaults to disabled |
| Quality and extended location data | `enabled` | Optional, useful |
| Stream realtime timestamps | `enabled` | |
| Discrete input instead of UART TX | `disabled` | Must stay disabled or TX is repurposed |
| Sleep by timeout | `never` | Default 60 s mimics a wiring fault mid-test |
| Sleep with external power | `never` | |

### Committing changes

Changes are **not applied live**. The Dashboard stages edits and requires an explicit write.

- **Write changes** (top of panel) commits
- **Cancel changes** discards staged edits
- **Read all** / **Write all** operate on the full parameter set
- The device may briefly disconnect or need a power cycle

> A staged-but-unwritten change looks identical to a committed one in the table.
> Always click **Read all** after writing to verify persistence.

### Reference values from our working unit

- Temperature: 23 °C
- Ultrasonic frequency: 28000 Hz
- Update rate: 6.4 Hz (dashboard-reported)
- Achieved on Pi over USB: **8 Hz** with correct driver params

---

## 9. NMEA Sentence Configuration (Drone)

Documented fix for the most time-consuming bug in the drone bring-up.

### The problem

Too many enabled sentence types flood the serial buffer. ArduPilot intermittently
misses the `$GPGGA` sentences it needs, producing a fix that appears for a couple of
seconds, drops to "no GPS," and cycles indefinitely.

### The fix

Enable **only**:

- `$GPGGA` — position fix
- `$GPRMC` — recommended minimum

Disable everything else: `$GPGSA`, `$GPGSV`, `$GPGLL`, `$GPVTG`, `$GPZDA`, `$GPHDT`.

### Root cause found in our case

Two compounding issues: excess sentence types flooding the buffer, **and** `$GPRMC`
had been disabled — ArduPilot requires it. Re-enabling RMC and disabling the extras
resolved the flickering completely.

### Update rate

6.4 Hz is fine — a new position every ~156 ms, well within ArduPilot's tolerance. If
buffer issues appear at higher rates, drop to 4–10 Hz in the Dashboard. **Do not
reflexively change `GPS_TIMEOUT`** — the 2000 ms default is more than adequate and is
almost never the real problem.

---

## 10. Marvelmind Binary Protocol (Ground Robot)

The `marvelmind_ros2` driver speaks **only** the Marvelmind binary protocol. It has
no NMEA parser.

### Driver installation

Official ROS 2 packages are in the `*_upstream` repos. The BitBucket
`ros_marvelmind_package` is ROS 1 / catkin and will not build. Several community
forks (`AngeloDamante`, `vlad-penkin`, `MarvelmindRobotics/marvelmind_ros2`) are
private and prompt for GitHub credentials.

```bash
cd ~/kobuki_ws/src
git clone https://github.com/MarvelmindRobotics/marvelmind_ros2_upstream.git
git clone https://github.com/MarvelmindRobotics/marvelmind_ros2_msgs_upstream.git
cd ~/kobuki_ws
colcon build --packages-select marvelmind_ros2_msgs marvelmind_ros2
source install/setup.bash
```

`stdcall` warnings during build are Windows-specific and harmless on Linux.

### Running — parameter names matter

The executable is `marvelmind_ros2`, not `hedge_rcv_bin`. Verify with
`ros2 pkg executables marvelmind_ros2`.

```bash
ros2 run marvelmind_ros2 marvelmind_ros2 --ros-args \
  -p marvelmind_tty_filename:=/dev/ttyACM1 \
  -p marvelmind_tty_baudrate:=115200 \
  -p marvelmind_publish_rate_in_hz:=8
```

> **The port parameter is `marvelmind_tty_filename`, NOT `port`.** Passing `port:=`
> is silently ignored — ROS 2 launch discards unknown args — and the driver falls back
> to its hardcoded default `/dev/ttyACM0`, which on our robot is the **MicroStrain IMU**.
>
> Confirm the port was accepted by reading the log line:
> `Prepare: tty: /dev/ttyACM1 and baud: 115200`
>
> Parameter names verified from source:
> ```bash
> grep -rn "declare_parameter" ~/kobuki_ws/src/marvelmind_ros2_upstream/ | grep -i "tty\|baud"
> ```

### The 1 Hz trap

Two independent causes produce 1 Hz when the dashboard reports 8 Hz:

1. **UART speed 9600** — cannot carry 8 Hz of position data. Fix in the dashboard.
2. **`marvelmind_publish_rate_in_hz` defaults to 1** — the driver's own publish timer,
   unrelated to serial rate. Must be passed explicitly.

Verify: `ros2 topic hz /hedgehog_pos` → expect ~8.

### Topics

Actual names differ from most documentation:

```
/hedgehog_pos             <- the one to use
/hedgehog_pos_ang
/hedgehog_pos_addressed
/hedgehog_imu_raw
/hedgehog_imu_fusion
/marvelmind_waypoint
/marvelmind_user_data
```

Sample `/hedgehog_pos`:

```yaml
timestamp_ms: 1786469190620
x_m: 1.666
y_m: -0.558
z_m: 0.357
flags: 2
```

`flags: 2` indicates a valid position.

### Message type in code

The correct import is `HedgePosition`, not `HedgehogPos`:

```python
from marvelmind_ros2_msgs.msg import HedgePosition
```

---

## 11. Modem Exclusivity

**The hedgehog streams to only one host at a time.**

If the Super Modem is connected to a dashboard PC, the Pi or Pixhawk receives nothing —
even though the driver reports a successful connection and the dashboard shows the
hedgehog tracking perfectly.

```
Setup phase:    Hedgehog -> Modem -> PC dashboard  (place beacons, freeze map, set fence)
Runtime phase:  Hedgehog -> USB/UART -> Pi or Pixhawk  (modem UNPLUGGED)
```

Confirmed with Marvelmind support during the ground robot bring-up.

---

## 12. Troubleshooting Decision Tree

Work top to bottom. Do not skip to parameter changes before confirming the layers beneath.

```
No position data
 |
 +- Are all stationary beacons green in Dashboard?
 |    NO -> Fix beacon power/placement (§5)
 |
 +- Is the hedgehog tracking stably on the Dashboard map?
 |    NO -> Beacon geometry or ultrasonic interference (§5)
 |
 +- Is the modem unplugged from all dashboard PCs?
 |    NO -> Unplug it (§11)
 |
 +- Is the OUTPUT PROTOCOL correct for this platform?          <-- CHECK EARLY
 |    Drone needs NMEA 0183; ground robot needs Marvelmind (§2)
 |
 +- Is the hedgehog the device that was configured?
 |    Settings on the modem do nothing (§4)
 |
 +- Raw stream on USB at 115200 — does it look right?
 |    NMEA: readable $GP... sentences with populated lat/lon
 |    Marvelmind: binary
 |
 +- [DRONE] Lat/lon blank or 0.0000?
 |    -> Map is not georeferenced (§7)
 |
 +- Do Dashboard settings match §8, and were they WRITTEN?
 |    Click Read all to verify persistence
 |
 +- [GROUND] Is the driver using marvelmind_tty_filename (not port)?
 |    Check the log line for the actual tty it opened (§10)
 |
 +- [GROUND] Is it pointed at the right /dev/ttyACM*?
 |    IMU and hedgehog both enumerate as ACM. Confirm with dmesg (§10)
 |
 +- [DRONE] Does SERIALx_BAUD match, and does the prefix match the PHYSICAL port?
 |
 +- CHECK THE WIRING / USB POWER
      TX->RX, RX->TX, common GND, pins fully seated
      Check dmesg for over-current events
```

> **Lesson learned, August 2026 drone bring-up.** The entire chain above was
> systematically verified — sentence config correct, parameters correct, beacons
> green, hedgehog tracking — and the root cause was the **wiring**. When every layer
> of configuration checks out and data still is not arriving, stop tuning parameters
> and inspect the physical connection.

---

## 13. Known Issues Log

| Symptom | Root cause | Fix | § |
|---|---|---|---|
| ROS driver connects, topics stay empty | Protocol set to NMEA, not Marvelmind | Switch protocol in dashboard | §2 |
| Driver opens wrong `/dev/ttyACM*` | Used `port:=` instead of `marvelmind_tty_filename:=` | Use the correct param name | §10 |
| Topics publish at 1 Hz not 8 Hz | UART at 9600 **and/or** driver publish rate default 1 | Fix both | §10 |
| Driver runs, topics frozen | Modem plugged into another PC | Unplug modem | §11 |
| `Stream location data` off | Dashboard default is disabled | Enable it | §8 |
| GPS fix flickers on/off | Too many NMEA sentences; `$GPRMC` disabled | Enable only GGA + RMC | §9 |
| `EKF3 waiting for GPS config data`, coords 0.0000 | Map not georeferenced | Set GPS origin | §7 |
| Everything configured, still no data | TX/RX swapped or unseated crimp | Inspect and reseat wiring | §12 |
| Settings applied but nothing changes | Changes staged but not written | Click Write changes, verify Read all | §8 |
| Hedgehog stops streaming after ~1 min | Sleep by timeout | Set sleep to `never` | §8 |
| Config applied but device unaffected | Applied to modem instead of hedgehog | Identify wired device | §4 |
| Baud mismatch | Marvelmind default is 500 kbps | Set both sides to 115200 | §8 |
| USB devices repeatedly disconnect/reconnect | Power budget exceeded | See ground robot README — Kinect draw | — |
| `PreArm: AHRS: EKF3 Yaw inconsistent` | Compass disagrees with EKF yaw | Recalibrate compass — unrelated to GPS | — |
| Dashboard crashes after license upload | Firmware/license version mismatch | Hard reset 10+ s, reflash, retry over USB | §14 |

Common Marvelmind default baud rates: 19200 / 38400 / 115200 / **500000**

---

## 14. License Codes and the UBX Trap

**Do not buy the UBX protocol license for an ArduPilot build.**

- UBX integration is specific to PX4's EKF2 estimator
- ArduPilot handles external positioning natively via MAVLink `GPS_INPUT` /
  `VISION_POSITION_ESTIMATE`, or simply as NMEA — no UBX license required
- The license is marketed toward PX4 users

If purchased in error, it is a legitimate case to raise with Marvelmind for a refund
or swap.

### License codes seen in the Dashboard

| Code | Gates |
|---|---|
| MMSW0001 | IMU fusion for location in NMEA |
| MMSW0002 | `$GPHDT` heading message |
| MMSW0003 | I²C compass emulation |
| MMSW0005 | Streaming mode |
| MMSW0006 | Alarm pin function |

**None of these are required** for either the standard NMEA integration or the ground
robot's binary path.

### If a license upload bricks the session

Symptoms: Dashboard quits after uploading and will not reopen.

Causes: license file does not match the exact firmware version (even a minor mismatch
breaks the session), upload corrupted flash mid-write, or the Dashboard dropped the
serial connection during transfer.

Recovery:

1. Hard reset the hedgehog — hold the button 10+ seconds for factory reset
2. Clean reflash of firmware before re-attempting
3. Use direct USB, never wireless, during license upload
4. Disable antivirus/firewall — the Windows dashboard has known conflicts where
   security software interrupts the serial write
5. Escalate to Marvelmind support or their forum/Discord

---

## 15. Accuracy Benchmarks

| Platform | Accuracy achieved | Notes |
|---|---|---|
| Drone (Marvelmind indoor nav) | **2 cm**, 95% success rate | Headline result |
| Ground robot (Pi 5) | **5 cm**, ~50 ft comm range | Earlier platform |
| Marvelmind spec (UWB general) | 2–10 cm | |

### Why Marvelmind was selected

| Modality | Accuracy | Latency | Infrastructure | Multipath | Cost |
|---|---|---|---|---|---|
| UWB (Marvelmind) | 2–10 cm | Low | Yes — anchors | Moderate | Low |
| IR mocap (OptiTrack) | <1 mm | Very low | Yes — cameras | None | Very high ($10k+) |
| Ultrasonic | 1–5 cm | Medium | Yes — beacons | High | Very low |
| BLE RSSI | 1–3 m | High | Yes | High | Very low |
| WiFi RSSI | 1–5 m | High | Yes | Very high | None |
| Visual SLAM | 5–20 cm | Medium | No | N/A | Low |
| LiDAR SLAM | 2–5 cm | Low | No | N/A | Med–high |

UWB gives the best cost/accuracy tradeoff for infrastructure-based indoor
localization. IR mocap is more accurate but is used as lab ground truth, not as a
deployable sensor, because of cost.

### Known weaknesses to design around

- Ultrasonic components are sensitive to motor/propeller noise
- Reflective lab surfaces cause multipath
- Requires fixed infrastructure — limits real-world deployability
- Coordinate frame mismatches (NED vs ENU) are a recurring integration hazard

---

## 16. Quick Reference Cards

### Ground robot (ROS 2)

```
Dashboard (hedgehog device)
  Streaming output ............ USB+UART
  UART speed .................. 115200
  Protocol .................... Marvelmind        <-- NOT NMEA
  Stream location data ........ enabled
  Sleep by timeout ............ never
  -> WRITE CHANGES, then READ ALL

Modem ......................... UNPLUGGED from all PCs

Pi launch
  ros2 run marvelmind_ros2 marvelmind_ros2 --ros-args \
    -p marvelmind_tty_filename:=/dev/ttyACM1 \
    -p marvelmind_tty_baudrate:=115200 \
    -p marvelmind_publish_rate_in_hz:=8

Verify
  ros2 topic hz /hedgehog_pos        -> ~8 Hz
  ros2 topic echo /hedgehog_pos --once -> x_m, y_m, flags: 2
```

### Drone (Pixhawk / ArduPilot)

```
Dashboard (hedgehog device)
  Streaming output ............ USB+UART
  UART speed .................. 115200
  Protocol .................... NMEA 0183         <-- NOT Marvelmind
  $GPRMC ...................... enabled
  $GPGGA ...................... enabled
  everything else ............. disabled
  Sleep by timeout ............ never
  Map GPS origin .............. SET (mandatory)
  -> WRITE CHANGES, then READ ALL

ArduPilot (GPS1 / SERIAL3)
  SERIAL3_PROTOCOL = 5
  SERIAL3_BAUD     = 115
  GPS_TYPE         = 5

Wiring (Super-Beacon-2 -> Pixhawk 6C)
  USART2_TX1 -> Pin 3 (RX)
  USART2_RX1 -> Pin 2 (TX)
  GND        -> Pin 6 (GND)
  Pins 1, 4, 5 unpopulated. Beacon on own battery.
```

### First things to check when it breaks

1. Beacons green? Hedgehog tracking on the map?
2. **Is the protocol right for the platform you're on?**
3. Modem unplugged from all dashboard PCs?
4. Which device did you actually configure — hedgehog or modem?
5. Were the changes **written**? (Read all to confirm)
6. Raw stream on USB — does it look like the protocol you expect?
7. The wiring. Always the wiring.

---

## Glossary

| Term | Meaning |
|---|---|
| Hedgehog | Marvelmind's term for the mobile beacon carried by the vehicle |
| Modem | Base station; coordinates the network, connects to PC |
| Stationary beacon | Fixed anchor providing a ranging reference |
| Submap | A defined coordinate region within the Marvelmind map |
| UWB | Ultra-wideband — radio ranging via time-of-flight |
| NMEA 0183 | Standard GPS text sentence protocol |
| GGA | NMEA sentence carrying position fix data |
| RMC | NMEA "recommended minimum" sentence |
| UART | Asynchronous point-to-point serial bus (TX/RX) |
| I²C | Synchronous multi-device serial bus (SCL/SDA) |
| JST-GH | Connector standard used on Pixhawk TELEM/GPS ports |
| BEC | Battery Eliminator Circuit — LiPo down to 5 V |
| EKF3 | ArduPilot's Extended Kalman Filter, third generation |
| ExternalNav | ArduPilot source type for non-GPS position input |
| VIO | Visual-Inertial Odometry |
| NED / ENU | North-East-Down / East-North-Up frame conventions |
| GCS | Ground Control Station (Mission Planner, QGroundControl) |
