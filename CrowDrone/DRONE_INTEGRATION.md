# CrowDrone — Marvelmind / Pixhawk Integration

Indoor positioning for the 10.5" quadcopter research platform. Marvelmind
Super-Beacon-2 feeds NMEA to a Pixhawk 6C running ArduPilot, which treats it as a GPS.

For shared hardware setup, protocol switching, beacon placement, and dashboard
configuration, see [MARVELMIND.md](../MARVELMIND.md).

**Achieved: 2 cm accuracy, 95% success rate.**

---

## Table of Contents

1. [Platform Specification](#1-platform-specification)
2. [Choosing the Integration Path](#2-choosing-the-integration-path)
3. [Super-Beacon-2 Pinout](#3-super-beacon-2-pinout)
4. [Pixhawk Port Selection](#4-pixhawk-port-selection)
5. [Cable Fabrication](#5-cable-fabrication)
6. [Power Architecture](#6-power-architecture)
7. [ArduPilot Parameters](#7-ardupilot-parameters)
8. [Bench Testing Procedure](#8-bench-testing-procedure)
9. [EKF3 Configuration for Indoor Flight](#9-ekf3-configuration-for-indoor-flight)
10. [Serial Protocol Primer](#10-serial-protocol-primer)
11. [Full Parameter Table](#11-full-parameter-table)

---

## 1. Platform Specification

| Component | Spec |
|---|---|
| Primary airframe | 10.5" prop quadcopter |
| Backup airframe | 7" prop quadcopter (FC previously failed) |
| Flight controller | Pixhawk 6C |
| Firmware | ArduPilot |
| GCS | Mission Planner |
| Positioning | Marvelmind Super-Beacon-2 on GPS1 |
| Telemetry | Microhard P900 on TELEM1 |
| Battery | 4S LiPo, 5000–6000 mAh |
| Companion compute | Jetson (vision workloads) |

Flight time: ~15–18 min without compute load; ~8–12 min with a companion computer
running vision.

---

## 2. Choosing the Integration Path

Three documented ways to get Marvelmind data into a Pixhawk. Pick one before wiring.

| Path | Protocol | ArduPilot setting | License | Verdict |
|---|---|---|---|---|
| **NMEA 0183** | Standard GPS sentences over UART | `SERIALx_PROTOCOL = 5` | None | **Recommended.** Path of least resistance — the FC treats Marvelmind as a GPS |
| Marvelmind native | Binary streaming format | `SERIALx_PROTOCOL = 36` | None | Richer data (raw distances, IMU) but needs the ArduPilot Marvelmind driver |
| I²C compass emulation | I²C via splitter | n/a | MMSW0003 | Super-Beacon only. Not our use case |

**We use NMEA.** Validate the hardware link with NMEA first; switching to native
protocol later for richer data is a straightforward follow-on.

> The ground robot uses the **Marvelmind binary** protocol instead. The hedgehog
> cannot serve both simultaneously — see [MARVELMIND.md §2](../MARVELMIND.md).

---

## 3. Super-Beacon-2 Pinout

The Super-Beacon-2 exposes **solder pads, not a connector**. There is no plug-in
cable option.

| Pad | Location | Function |
|---|---|---|
| `USART2_TX1` | Left side | Beacon transmit |
| `USART2_RX1` | Right side | Beacon receive |
| `GND` | Top-right corner | Ground |
| `I2C1_SCL1` | Bottom | I²C clock (unused in our path) |
| `I2C1_SDA1` | Bottom | I²C data (unused in our path) |

### Electrical characteristics

- Logic level: **CMOS 3.3 V** — same as Pixhawk, no level shifter required
- Default baud: **500 kbps** — must be changed to 115200 in the Dashboard; ArduPilot
  does not support 500k for this
- Minimum viable wiring: 2 wires (`GND` + `USART2_TX1`) for one-way position
  streaming; 3 wires (adding RX) for bidirectional

The I²C pads are only used for the compass-emulation route (requires MMSW0003
license). For the UART/NMEA approach documented here, ignore them entirely.

---

## 4. Pixhawk Port Selection

| Port | Typical reservation | Suitable for Marvelmind? |
|---|---|---|
| TELEM1 | MAVLink telemetry to GCS (Microhard P900) | Technically yes, but you lose the GCS link |
| TELEM2 | General purpose | Yes — conventional choice |
| TELEM3 | General purpose | Yes |
| **GPS1** | GNSS module | **Yes — current build**, with `SERIAL3_*` params |
| GPS2 | Secondary GNSS | Yes |
| CAN | DroneCAN bus (e.g. F9P) | **No** — not a UART, do not use |

TELEM ports are general-purpose serial ports assignable to any protocol via
`SERIALx_PROTOCOL`. GPS ports expect a real GNSS module, which the Marvelmind is
not — though NMEA mode works there too, since NMEA is what a GPS speaks.

### Port → parameter mapping

| Physical port | Parameter prefix |
|---|---|
| TELEM1 | `SERIAL1_*` |
| TELEM2 | `SERIAL2_*` |
| GPS1 | `SERIAL3_*` |

> Our current build uses **GPS1 → SERIAL3**. Earlier project documentation referenced
> TELEM2 (SERIAL2); both are valid, but the parameter prefix **must match the physical
> port**. This is a frequent error source.

---

## 5. Cable Fabrication

TELEM and GPS ports on the Pixhawk 6C use **6-pin JST-GH** connectors.

### TELEM2 pinout (verified against board documentation)

| Pin | Color | Signal | Voltage |
|---|---|---|---|
| 1 | red | VCC | +5 V |
| 2 | black | UART5_TX (out) | +3.3 V |
| 3 | black | UART5_RX (in) | +3.3 V |
| 4 | black | UART5_CTS (in) | +3.3 V |
| 5 | black | UART5_RTS (out) | +3.3 V |
| 6 | black | GND | GND |

### Wire map — Super-Beacon-2 → TELEM2

```
USART2_TX1  ->  Pin 3 (UART5_RX, in)
USART2_RX1  ->  Pin 2 (UART5_TX, out)
GND         ->  Pin 6 (GND)
```

**TX crosses to RX. RX crosses to TX.** Leave pins 1, 4, and 5 unpopulated.

> **Do not connect Pin 1 (+5 V) to the beacon.** The beacon runs on its own battery.
> See §6.

### Fabrication method

**Option 1 — Cut and solder (recommended).** Take a pre-crimped JST-GH cable, cut the
connector off one end, strip the three needed wires, and solder them directly to the
beacon pads. Keep the JST-GH connector intact on the Pixhawk end.

**Option 2 — Header pins + Dupont.** Solder pin headers to the beacon pads, then
bridge with Dupont jumpers. More modular but adds connection points that work loose
under drone vibration. Not recommended for flight hardware.

> **Before cutting, record which wire color maps to which JST-GH pin.** Once the
> connector is off, that information is gone and TX/RX gets guessed.

### Crimp seating

Crimped pins must seat **flush** in the JST-GH housing with an audible or tactile
click. The metal pin has a small barb that catches inside the housing. If the wire is
poking out past the flap, the pin is not fully seated — push further until it clicks.
An unseated pin is an intermittent connection waiting to happen.

---

## 6. Power Architecture

### Beacon power

The Super-Beacon-2 has its own battery and is designed to run independently — it must
operate whether or not the FC is connected. **Three wires only: TX, RX, GND.**

Powering from TELEM Pin 1 (+5 V) is technically possible but discouraged: the
Marvelmind UART logic is 3.3 V while the pin is 5 V, and it adds load to the
Pixhawk's 5 V rail.

### Drone power architecture

Motors and ESCs generate substantial electrical noise that corrupts serial
communication and can brown out compute. **Isolate everything.**

```
LiPo Battery (4S, 5000-6000 mAh)
  |
Power Distribution Board (PDB)
  +-- ESCs -> Motors (raw battery voltage, high current)
  +-- BEC 1 (5V 2-3A) -> Flight Controller
  +-- BEC 2 (5V 4-5A) -> Companion compute (Jetson)
  +-- BEC 3 (5V 1A)   -> Camera / peripherals
```

Recommendations:

- **Use a dedicated BEC per major device.** Shared power causes brownouts and
  mid-flight crashes
- Add a **35 V 1000 µF capacitor** across the PDB rails to absorb motor voltage
  spikes. Cheap insurance
- Linear BEC is fine for the FC; use a **switching BEC** for power-hungry compute
- **Never power companion compute off the same rail as the FC**

---

## 7. ArduPilot Parameters

### NMEA path — GPS1 / SERIAL3 (current build)

```
SERIAL3_PROTOCOL = 5      # GPS
SERIAL3_BAUD     = 115    # 115200
GPS_TYPE         = 5      # NMEA
```

### NMEA path — TELEM2 / SERIAL2 (alternate)

```
SERIAL2_PROTOCOL = 5
SERIAL2_BAUD     = 115
GPS_TYPE         = 5
```

### External nav path (position as ExternalNav rather than GPS)

```
GPS_TYPE       = 14   # MAVLink GPS
EK3_SRC1_POSXY = 6    # ExternalNav
EK3_SRC1_POSZ  = 6    # ExternalNav
```

### Native Marvelmind protocol (alternative)

```
SERIALx_PROTOCOL = 36     # Marvelmind native
SERIALx_BAUD     = 115
BCN_TYPE         = 2      # Marvelmind
BCN_LATITUDE     = <site lat>
BCN_LONGITUDE    = <site lon>
BCN_ALT          = <site alt>
```

`BCN_LATITUDE` / `BCN_LONGITUDE` / `BCN_ALT` serve the same purpose as the Dashboard's
map origin — they map Marvelmind's local coordinates into a global frame. They must be
set for the position to be usable.

Additional streams available in native mode (each currently n/a or license-gated on
our unit): stream location data, raw inertial sensors, processed IMU, raw distances,
quality and extended location data, telemetry stream, locations of other hedgehogs.

Beacon data appears in Mission Planner under the EKF/status page once flowing.

### Notes

- `SERIALx_BAUD` uses shorthand: **`115` means 115200**
- If using a second GPS port, `GPS_TYPE2` applies instead of `GPS_TYPE`
- **Confirm the parameter prefix matches the physical port in use** — frequent error source

---

## 8. Bench Testing Procedure

> The Pixhawk does not need to be mounted on a drone to validate the positioning
> link. It will read NMEA sitting on a bench. **Test on the bench first.**

1. Power the stationary beacon network; confirm all green in Dashboard
2. Power the hedgehog; confirm stable position on the Dashboard map
3. Connect hedgehog to Pixhawk per §5
4. Power the Pixhawk, connect Mission Planner
5. Open **MAVLink Inspector**, watch `GPS_RAW_INT` → `fix_type`
6. Physically move the hedgehog around the room and watch the position update

### Interpreting fix_type

| Value | Meaning | Implication |
|---|---|---|
| 0 | No fix | No valid data arriving — check wiring, sleep timeout, sentence config |
| 1 | Fix but no position | Data arriving, position invalid — check map georeferencing |
| 2 | 2D fix | Partial |
| 3 | 3D fix | **Working** |

A flat `0` that never twitches points to a **physical/link problem**. Bouncing values
point to a **data-quality problem**.

---

## 9. EKF3 Configuration for Indoor Flight

Once position is flowing, EKF3 must be tuned for a GPS-denied environment where the
"GPS" is actually an indoor positioning system.

- EKF3 source selection must trust the external position source
- For ExternalNav-style integration: `EK3_SRC1_POSXY = 6`, `EK3_SRC1_POSZ = 6`
- Indoor hover typically requires better than 10 cm accuracy for stability —
  Marvelmind's ~2 cm is comfortably within that
- Expect `EKF3 waiting for GPS config data` until a valid geographic position is
  being received

### Related pre-arm issue

```
PreArm: AHRS: EKF3 Yaw inconsistent NNN deg
```

Indicates the compass disagrees substantially with the EKF's yaw estimate. This is an
**independent problem from GPS lock** and does not block a fix from appearing, but it
will block arming. Recalibrate the compass once positioning is confirmed working.

---

## 10. Serial Protocol Primer

Both buses appear in this stack. Knowing which is which prevents wiring mistakes.

### UART — TX / RX

- TX = Transmit, RX = Receive
- **Asynchronous** — no shared clock; both sides must agree on baud rate in advance
- **Point-to-point only** (one device to one other)
- **Full duplex** — TX and RX are separate lines, both sides can talk simultaneously
- Wiring: 2 wires plus ground
- Used here for: **Marvelmind hedgehog → Pixhawk**

### I²C — SCL / SDA

- SCL = Serial Clock, SDA = Serial Data
- **Synchronous** — SCL is a shared clock keeping both sides in sync
- **Multi-device bus** — one master addresses many slaves on the same 2 wires
- **Half duplex** — data flows one direction at a time on SDA
- Requires pull-up resistors on both lines
- Used here for: IMUs, barometers, ICM-42688-P talking to the flight controller

### Mental model

- **UART** = a direct phone call between two specific people; no clock needed, just
  agree on speaking speed
- **I²C** = a shared intercom system with a clock coordinating who talks when, each
  device having an address

---

## 11. Full Parameter Table

| Parameter | Value | Applies to | Purpose |
|---|---|---|---|
| `SERIAL2_PROTOCOL` | 5 | TELEM2 | GPS protocol |
| `SERIAL2_BAUD` | 115 | TELEM2 | 115200 baud |
| `SERIAL3_PROTOCOL` | 5 | GPS1 | GPS protocol |
| `SERIAL3_BAUD` | 115 | GPS1 | 115200 baud |
| `GPS_TYPE` | 5 | — | NMEA |
| `GPS_TYPE` | 14 | — | MAVLink GPS (alt path) |
| `GPS_TYPE2` | 14 | — | Second GPS port |
| `GPS_TIMEOUT` | 2000 | — | Default; leave alone |
| `SERIALx_PROTOCOL` | 36 | — | Marvelmind native |
| `BCN_TYPE` | 2 | — | Marvelmind beacon |
| `BCN_LATITUDE` | site lat | — | Local→global mapping |
| `BCN_LONGITUDE` | site lon | — | Local→global mapping |
| `BCN_ALT` | site alt | — | Local→global mapping |
| `EK3_SRC1_POSXY` | 6 | — | ExternalNav XY |
| `EK3_SRC1_POSZ` | 6 | — | ExternalNav Z |

PX4 equivalents (not used in this build): `EKF2_AID_MASK`, `EKF2_HGT_MODE`.

---

## Research Context

### Alternatives evaluated

- **OptiTrack / Vicon mocap** — sub-millimetre, integrates cleanly via VRPN,
  eliminates all current problems. Rejected on cost ($10k+)
- **Visual-Inertial Odometry** — camera + IMU, no external infrastructure. Candidates:
  VINS-Mono, OpenVINS (best documented), Kimera-VIO. Drifts over time. Compute-heavy;
  Pi 4/5 marginal, Jetson preferable
- **Visual SLAM (ORB-SLAM3)** — builds a map and localizes within it; loop closure
  corrects drift. Highest novelty, no infrastructure
- **LiDAR SLAM** — very robust, heavy and expensive, likely overkill
- **DIY UWB (DW1000/DW3000)** — ~$30–50/node, white-box. At system level differs
  little from Marvelmind; reviewers will ask why not use the commercial unit
- **DIY IR mocap** — 4× modified PS3 Eye cameras (150 fps, ~$2–4 each) with IR pass
  filters, active 850 nm markers, OpenCV blob detection + triangulation, position over
  UDP. ~$50–60 for room-scale. Beyond ~10 m, active markers strongly outperform passive

### Publication angles (target: AIAA SciTech)

1. **Fused indoor localization + drone ID platform.** Most drone detection work
   assumes outdoor/GPS. Claim: detect, classify, and localize a drone in a GPS-denied
   environment
2. **Marvelmind as benchmarking ground truth.** Use the Super-Beacon-2 as a
   cm-accurate reference to evaluate VIO or other nav stacks. Platform contribution
3. **UWB + VIO fusion.** Marvelmind as an absolute anchor correcting VIO drift; VIO
   supplies high-rate estimates. Most fusion papers use expensive Vicon as ground
   truth, not accessible UWB — that is the gap
4. **Indoor acoustic classification.** Outdoor acoustic drone classification exists;
   indoor is far less studied since reverb and multipath change the problem

> **Where the real research lives:** modality choice is close to a solved problem —
> UWB wins for low-cost infrastructure-based positioning. The open problem is the
> **filtering and state estimation layer**. Raw UWB gives noisy discrete position
> pings; fusing that with high-rate drifting IMU data to produce smooth accurate pose
> is where papers are still being written.

Team: 5 students, spanning hardware setup through software integration.
