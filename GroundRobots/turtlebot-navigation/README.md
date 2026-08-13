# TurtleBot2 Indoor Autonomous Navigation Platform

Raspberry Pi 5 + ROS 2 Jazzy + Kobuki base + Marvelmind indoor positioning + LORD MicroStrain IMU.

A ground robot platform for GPS-denied indoor autonomous navigation. Position comes from
Marvelmind ultrasonic trilateration, heading from a MicroStrain inertial sensor, and path
planning from Nav2 — no LiDAR, no SLAM.

---

## Table of Contents

1. [System Overview](#1-system-overview)
2. [Hardware Inventory](#2-hardware-inventory)
3. [USB Port Assignment](#3-usb-port-assignment)
4. [Phase 1 — Raspberry Pi OS Installation](#4-phase-1--raspberry-pi-os-installation)
5. [Phase 2 — Network Configuration](#5-phase-2--network-configuration)
6. [Phase 3 — ROS 2 Jazzy Installation](#6-phase-3--ros-2-jazzy-installation)
7. [Phase 4 — Kobuki Base Setup](#7-phase-4--kobuki-base-setup)
8. [Phase 5 — Keyboard Teleoperation](#8-phase-5--keyboard-teleoperation)
9. [Phase 6 — Kinect Sensor (Optional)](#9-phase-6--kinect-sensor-optional)
10. [Phase 7 — Marvelmind Indoor Positioning](#10-phase-7--marvelmind-indoor-positioning)
11. [Phase 8 — Sensor Fusion Node](#11-phase-8--sensor-fusion-node)
12. [Phase 9 — Nav2 Autonomous Navigation](#12-phase-9--nav2-autonomous-navigation)
13. [Phase 10 — Yaw Calibration](#13-phase-10--yaw-calibration)
14. [Phase 11 — MicroStrain 3DM-GQ4-45 IMU Integration](#14-phase-11--microstrain-3dm-gq4-45-imu-integration)
15. [Topic Reference](#15-topic-reference)
16. [File Inventory](#16-file-inventory)
17. [Startup Sequence](#17-startup-sequence)
18. [Troubleshooting Log](#18-troubleshooting-log)
19. [Known Issues and Current State](#19-known-issues-and-current-state)
20. [Next Steps](#20-next-steps)

---

## 1. System Overview

### Architecture

```
Marvelmind Beacons (4-6 stationary)
        |  ultrasound trilateration
        v
   Hedgehog (mobile beacon on robot)
        |  USB serial
        v
   Raspberry Pi 5  <--- USB ---  MicroStrain 3DM-GQ4-45 (heading)
        |
        |  /hedgehog_pos (x, y)  +  /imu/data (yaw)
        v
   Fusion Node  -->  /marvelmind_odom  +  TF map->base_footprint
        |
        v
   Nav2 (planner + Regulated Pure Pursuit controller)
        |  /cmd_vel
        v
   relay --> /commands/velocity --> Kobuki base --> motion
```

### Design Rationale

| Decision | Reasoning |
|---|---|
| No LiDAR / SLAM | Marvelmind provides absolute position directly; the operating area is a known open space |
| Single hedgehog, not two | Two-hedgehog heading is noisy at low speed and caused control oscillation; a dedicated IMU is more reliable |
| MicroStrain over Kobuki IMU | Kobuki IMU drifts badly (observed ~178 deg persistent offset); MicroStrain drifts ~0.002 deg/s |
| Gyro integration over onboard filter | The complementary filter needs ~10 s to converge after a turn — far too slow for a 20 Hz Nav2 controller |
| Regulated Pure Pursuit | Simple, well-suited to differential drive without obstacle sensors |

---

## 2. Hardware Inventory

| Component | Details |
|---|---|
| Compute | Raspberry Pi 5, hostname `bolzpi2`, user `uas` |
| Base | TurtleBot2 / Kobuki — Hardware 1.0.4, Firmware 1.1.4 |
| Positioning | Marvelmind Super Beacon kit — 6 stationary beacons, 2 hedgehogs, 1 Super Modem |
| IMU | LORD MicroStrain 3DM-GQ4-45 (GNSS-aided inertial nav unit) |
| Camera | Microsoft Kinect (Xbox NUI) — optional |
| Power | Powered USB hub recommended (see notes below) |

### USB Device Identifiers

```
0483:5740  STMicroelectronics Virtual COM Port  -> MicroStrain 3DM-GQ4-45
                                                   ("Lord Inertial Sensor",
                                                    serial 0000__6250.88495)
0403:6001  FTDI FT232 Serial (UART) IC          -> Kobuki base
045e:02c2  Microsoft Corp. Kinect NUI Motor     -> Kinect
045e:02ad  Microsoft Corp. Xbox NUI Audio       -> Kinect
045e:02ae  Microsoft Corp. Xbox NUI Camera      -> Kinect
05e3:0610  Genesys Logic Hub                    -> USB hub
05e3:0616  Genesys Logic Hub                    -> USB hub
```

> The MicroStrain enumerates with an STMicroelectronics VID because of the STM32
> chip inside it. It is easy to mistake for a Marvelmind hedgehog, which uses the
> same VID/PID. Confirm by the USB product string (`Lord Inertial Sensor`) or the
> serial number, where `6250` is the GQ4-45 model code.

---

## 3. USB Port Assignment

The Pi 5 has two blue USB 3.0 ports (top, nearest the Ethernet jack) and two black USB 2.0 ports.

| Device | Port | Rationale |
|---|---|---|
| MicroStrain IMU | Blue USB 3.0, direct | Minimize latency/jitter on the attitude stream |
| Kobuki base | Blue USB 3.0, direct | Actuator path — a flaky hub connection drops velocity commands |
| Marvelmind hedgehog | Black USB 2.0, via powered hub | 8 Hz, low bandwidth, latency-tolerant |
| Kinect | Black USB 2.0, direct | Power-hungry, saturates a controller on its own |

### Power Note

If the Pi 5 PSU is not the official 27 W (5 V / 5 A) supply, the Pi caps total USB
current at 600 mA across all ports. With four devices attached this causes random
dropouts. Use a **powered** hub.

### Recommended: udev Rules

With three serial devices, `/dev/ttyUSB*` and `/dev/ttyACM*` assignments can swap
between boots. Identify each device:

```bash
udevadm info -a -n /dev/ttyACM0 | grep -E "serial|idVendor|idProduct"
```

Then create `/etc/udev/rules.d/99-robot.rules` with stable symlinks
(`/dev/imu`, `/dev/kobuki`, `/dev/hedgehog`) keyed on serial number, and point
launch files at those names.

> **Status: not yet implemented.** This is the single highest-value reliability
> improvement remaining.

---

## 4. Phase 1 — Raspberry Pi OS Installation

### Flash the SD Card

Use **Raspberry Pi Imager** (https://www.raspberrypi.com/software/):

- **Device:** Raspberry Pi 5
- **OS:** Other general-purpose OS -> Ubuntu -> **Ubuntu Server 24.04.4 LTS (64-bit)**
- **Storage:** microSD card

> Must be **Server**, not Desktop, and **64-bit**.

### Imager Advanced Settings (gear icon)

| Setting | Value |
|---|---|
| Hostname | `bolzpi2` |
| Username | `uas` |
| Password | (set your own — see security note) |
| WiFi SSID | Simple name, **no spaces or apostrophes** |
| WiFi Country | US |
| Services | Enable **SSH** with password authentication |

> **Security note:** the working password used during development was committed to
> chat logs and is weak. Change it with `passwd` before this repo goes public, and
> never commit credentials.

### Verify Boot

```bash
ip a                  # look for wlan0 with an inet address
ping -c 3 google.com  # confirm internet
```

### SSH In

```bash
ssh uas@bolzpi2.local
```

Falls back to IP if mDNS is unreliable over a phone hotspot:

```bash
arp -a | grep 172.20.10     # on the Mac
ssh uas@172.20.10.X
```

Set up key auth to skip the password prompt:

```bash
ssh-copy-id uas@bolzpi2.local
```

---

## 5. Phase 2 — Network Configuration

The robot runs headless over an iPhone Personal Hotspot (subnet `172.20.10.x`).

### Dual Hotspot Netplan

`/etc/netplan/50-cloud-init.yaml`:

```yaml
network:
  version: 2
  ethernets:
    eth0:
      optional: true
      dhcp4: true
      dhcp6: true
  wifis:
    wlan0:
      optional: true
      dhcp4: true
      regulatory-domain: "US"
      access-points:
        "primary_hotspot":
          auth:
            key-management: "psk"
            password: "<psk-hash>"
        "backup_hotspot":
          auth:
            key-management: "psk"
            password: "<password>"
```

Apply with:

```bash
sudo netplan apply
```

### Hotspot Gotchas

- **SSIDs must be simple.** Spaces and apostrophes (e.g. `"Mohammads iPhone"`) break the connection.
- **Enable "Maximize Compatibility"** in iPhone hotspot settings — forces 2.4 GHz, which the Pi joins far more reliably.
- **macOS Internet Sharing cannot share WiFi over WiFi.** The Mac must be on Ethernet to share to WiFi. Its subnet is `192.168.2.x`, not `172.20.10.x`.
- **Both Mac and Pi must be on the same hotspot** for SSH. Verify on the Mac with `ipconfig getifaddr en0`.
- If the Pi will not join, power cycle it — this resolved a persistent failure once.

---

## 6. Phase 3 — ROS 2 Jazzy Installation

### Locale

```bash
sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

### Universe Repository

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
```

### ROS 2 apt Source

```bash
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### Install

```bash
sudo apt update
sudo apt install ros-jazzy-desktop -y
```

Takes 10–20 minutes on a Pi 5 over hotspot.

### Source on Login

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "source ~/kobuki_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 7. Phase 4 — Kobuki Base Setup

### Build from Source

The Kobuki ROS 2 packages are not in the Jazzy apt repos and must be built.

```bash
mkdir -p ~/kobuki_ws/src
cd ~/kobuki_ws/src
# clone kobuki_core, kobuki_ros, ecl_* dependencies
cd ~/kobuki_ws
colcon build --symlink-install --cmake-args -DCMAKE_CXX_FLAGS="-w"
```

> **`-DCMAKE_CXX_FLAGS="-w"` is required.** Without it, `ecl_time_lite` fails on
> modern compiler strictness (warnings treated as errors).

> **Use `screen` for long builds** so they survive SSH drops:
> `screen -S build` then `colcon build ...`, reattach with `screen -r build`.

### Verify

```bash
ls ~/kobuki_ws/install/ | grep kobuki
```

Expected:

```
kobuki_auto_docking   kobuki_keyop           kobuki_ros
kobuki_bumper2pc      kobuki_node            kobuki_safety_controller
kobuki_core           kobuki_random_walker   kobuki_description
```

### Launch (stock)

```bash
ros2 launch kobuki_node kobuki_node-launch.py
```

Healthy output:

```
[kobuki]: Kobuki : using odom_frame [odom].
[kobuki]: Kobuki : using base_frame [base_footprint].
[kobuki]: Kobuki : configured for connection on device_port /dev/ttyUSB0
[kobuki]: Version info - Hardware: 1.0.4. Firmware: 1.1.4
```

### Custom Launch File

`~/kobuki_ws/kobuki_launch.py` — needed once Marvelmind is in the loop. It disables
Kobuki TF publishing (which conflicts with the fusion node) and remaps `/odom` so it
does not collide with Nav2's odometry source.

```python
import os
import ament_index_python.packages
import launch
import launch_ros.actions
import yaml

def generate_launch_description():
    share_dir = ament_index_python.packages.get_package_share_directory('kobuki_node')
    params_file = os.path.join(share_dir, 'config', 'kobuki_node_params.yaml')
    with open(params_file, 'r') as f:
        params = yaml.safe_load(f)['kobuki_ros_node']['ros__parameters']
    params['publish_tf'] = False
    kobuki_ros_node = launch_ros.actions.Node(
        package='kobuki_node',
        executable='kobuki_ros_node',
        output='both',
        parameters=[params],
        remappings=[('/odom', '/kobuki_odom')]
    )
    return launch.LaunchDescription([kobuki_ros_node])
```

Launch with:

```bash
ros2 launch ~/kobuki_ws/kobuki_launch.py
```

---

## 8. Phase 5 — Keyboard Teleoperation

### Critical: Topic Name

**The Kobuki subscribes to `/commands/velocity`, NOT `/cmd_vel`.** This was the single
biggest early time sink — teleop appeared to run fine while the robot never moved.

Diagnose with:

```bash
ros2 topic info /commands/velocity
# Subscription count: 1  <- base is listening
# Subscription count: 0  <- base is not running
```

### Standard Teleop

```bash
sudo apt install ros-jazzy-teleop-twist-keyboard -y
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args --remap cmd_vel:=/commands/velocity
```

Keys: `i` forward, `,` back, `j` left, `l` right, `k` stop.

### Custom WASD Script

`~/wasd.py` — publishes at 20 Hz with a dead-man timeout.

Key design points:

- **20 Hz publish rate.** The Kobuki watchdog zeroes velocity if commands do not
  arrive within 0.6 s. Publishing at 1 Hz (the `ros2 topic pub` default) makes the
  robot twitch and stop repeatedly — this looked like a hardware fault for a while.
- **0.4 s dead-man timeout.** Terminals send discrete keypresses, not key-down/key-up,
  so "hold to drive" relies on key autorepeat. Releasing stops the robot.
- `-` / `=` adjust speed live. Defaults: 0.15 m/s linear, 0.5 rad/s angular.

```bash
python3 ~/wasd.py
```

### Manual Velocity Command

```bash
timeout 5 ros2 topic pub -r 20 /commands/velocity geometry_msgs/msg/Twist "{angular: {z: 0.5}}"
```

`-r 20` is essential.

---

## 9. Phase 6 — Kinect Sensor (Optional)

Not part of the navigation stack — used for visual feedback only.

### System Library

```bash
sudo apt install ros-jazzy-cv-bridge -y
freenect-glview   # "Number of devices found: 1" confirms detection
```

`failed to open display` is expected headless and can be ignored.

### Build kinect_ros2

```bash
cd ~/kobuki_ws/src
git clone https://github.com/ros-drivers/freenect_stack.git
cd ~/kobuki_ws
colcon build --symlink-install --cmake-args -DCMAKE_CXX_FLAGS="-w" --packages-select kinect_ros2
```

### Two Required Source Patches

In `src/kinect_ros2/include/kinect_ros2/kinect_ros2_component.hpp`:

| Original | Change to | Why |
|---|---|---|
| `#include "libfreenect/libfreenect.h"` | `#include "libfreenect.h"` | Header installs flat, not in a subdirectory |
| `#include "cv_bridge/cv_bridge.h"` | `#include "cv_bridge/cv_bridge.hpp"` | Extension changed in ROS 2 Jazzy |

### Run

```bash
sudo -E bash -c "source /opt/ros/jazzy/setup.bash && \
  source /home/uas/kobuki_ws/install/setup.bash && \
  ros2 run kinect_ros2 kinect_ros2_node"
```

### Capture a Single Frame

```bash
# On the Pi (must run under the same sudo env as the node)
sudo -E bash -c "source /opt/ros/jazzy/setup.bash && \
  source /home/uas/kobuki_ws/install/setup.bash && python3 -c \"
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
rclpy.init()
node = Node('image_saver')
bridge = CvBridge()
def callback(msg):
    img = bridge.imgmsg_to_cv2(msg, 'bgr8')
    cv2.imwrite('/home/uas/kinect_image.jpg', img)
    print('Image saved!')
    rclpy.shutdown()
node.create_subscription(Image, '/image_raw', callback, 10)
rclpy.spin(node)
\""

# On the Mac
scp uas@bolzpi2.local:/home/uas/kinect_image.jpg ~/Desktop/
```

### MJPEG Live Stream

`~/kinect_stream.py` serves frames over Flask on port 5000
(`http://<pi-ip>:5000/video`).

> **Status: unreliable.** Stream loads very slowly or times out over the hotspot —
> frame size versus available bandwidth. Deprioritized.

---

## 10. Phase 7 — Marvelmind Indoor Positioning

### Beacon Placement

- **4 beacons in the corners** is sufficient for a rectangular space. 6 adds accuracy in the middle.
- **Mount 0.5–1 m off the ground**, not on the floor — ultrasound is blocked by floor
  clutter and suffers multipath from ground reflections.
- **Keep all beacons at roughly the same height**, and the hedgehog in a similar plane.
  Large height differences degrade 2D accuracy.
- Record the corner coordinates and area dimensions before unplugging the modem — they
  are needed for the Nav2 static map.

### Critical: Modem Exclusivity

**The hedgehog streams to only one host at a time.** If the Super Modem is connected to a
Windows or Mac dashboard, the Pi receives nothing. Symptom: driver reports
`Hedgehog is running!` but topics stay frozen and empty.

Workflow:

```
Setup:    Hedgehog -> Modem -> Windows/Mac dashboard (place beacons, freeze map, set fence)
Runtime:  Hedgehog -> USB -> Pi   (modem UNPLUGGED)
```

### Dashboard Settings

Set on the hedgehog before use:

| Setting | Value |
|---|---|
| Streaming output | `USB+UART` |
| UART speed | **115200** (9600 cannot sustain 8 Hz) |
| Protocol on UART/USB output | `Marvelmind` (binary) |
| Stream location data | **enabled** (defaults to disabled — a silent failure mode) |
| Quality and extended location data | enabled (optional) |
| UART Output | Mobile beacon position + Mobile beacon IMU |

### Driver Installation

The official ROS 2 packages live in the `*_upstream` repos. The BitBucket
`ros_marvelmind_package` is ROS 1 / catkin and will not build. Several community forks
(`AngeloDamante`, `vlad-penkin`, `MarvelmindRobotics/marvelmind_ros2`) are private and
prompt for GitHub credentials.

```bash
cd ~/kobuki_ws/src
git clone https://github.com/MarvelmindRobotics/marvelmind_ros2_upstream.git
git clone https://github.com/MarvelmindRobotics/marvelmind_ros2_msgs_upstream.git
cd ~/kobuki_ws
colcon build --packages-select marvelmind_ros2_msgs marvelmind_ros2
source install/setup.bash
ros2 pkg list | grep marvelmind
```

`stdcall` warnings during build are Windows-specific and harmless on Linux.

### Run

The executable is `marvelmind_ros2` — not `hedge_rcv_bin` as older docs suggest.
Check with `ros2 pkg executables marvelmind_ros2`.

```bash
ros2 run marvelmind_ros2 marvelmind_ros2 --ros-args \
  -p port:=/dev/ttyACM0 \
  -p marvelmind_tty_baudrate:=115200 \
  -p marvelmind_publish_rate_in_hz:=8
```

### The 1 Hz Trap

Two independent causes produce a 1 Hz stream when the dashboard reports 8 Hz:

1. **UART speed 9600** — cannot carry 8 Hz of position data. Fix in the dashboard.
2. **`marvelmind_publish_rate_in_hz` defaults to 1** — the driver's own publish timer,
   unrelated to the serial rate. Must be passed explicitly.

Verify:

```bash
ros2 topic hz /hedgehog_pos   # expect ~8.0
```

### Topics

Actual topic names differ from most documentation:

```
/hedgehog_pos             <- the one used by the fusion node
/hedgehog_pos_ang
/hedgehog_pos_addressed
/marvelmind_waypoint
/marvelmind_user_data
```

Sample `/hedgehog_pos` message:

```yaml
timestamp_ms: 1779378372137
x_m: 1.925
y_m: -2.095
z_m: 0.382
flags: 2
```

---

## 11. Phase 8 — Sensor Fusion Node

Custom ament_python package `marvelmind_fusion` at
`~/kobuki_ws/src/marvelmind_fusion/`.

### Purpose

Combine Marvelmind `x, y` with an IMU-derived `yaw` into a single
`nav_msgs/Odometry` on `/marvelmind_odom`, and broadcast TF `map -> base_footprint`.

### Package Layout

```
marvelmind_fusion/
├── marvelmind_fusion/
│   ├── fusion_node.py
│   ├── goto_goal.py
│   ├── perimeter.py
│   └── calibrate_yaw.py
├── resource/marvelmind_fusion
├── package.xml
└── setup.py
```

### setup.py

```python
from setuptools import setup, find_packages

package_name = 'marvelmind_fusion'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'fusion_node = marvelmind_fusion.fusion_node:main',
            'goto_goal = marvelmind_fusion.goto_goal:main',
            'perimeter = marvelmind_fusion.perimeter:main',
            'calibrate_yaw = marvelmind_fusion.calibrate_yaw:main',
        ],
    },
)
```

### package.xml

```xml
<?xml version="1.0"?>
<package format="3">
  <name>marvelmind_fusion</name>
  <version>0.0.1</version>
  <description>Marvelmind + IMU odometry fusion</description>
  <maintainer email="uas@todo.todo">uas</maintainer>
  <license>Apache-2.0</license>
  <depend>rclpy</depend>
  <depend>nav_msgs</depend>
  <depend>tf2_ros</depend>
  <depend>marvelmind_ros2_msgs</depend>
  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

### Message Type Note

The correct import is `HedgePosition`, not `HedgehogPos`:

```python
from marvelmind_ros2_msgs.msg import HedgePosition
```

### Build

```bash
cd ~/kobuki_ws
rm -rf build/marvelmind_fusion install/marvelmind_fusion
colcon build --packages-select marvelmind_fusion --symlink-install
source install/setup.bash
```

> Always `rm -rf` the build and install directories first. Incremental rebuilds of
> ament_python packages produce
> `error: [Errno 17] File exists: .../resource/marvelmind_fusion`.

Run via the installed binary:

```bash
~/kobuki_ws/install/marvelmind_fusion/bin/fusion_node
```

### TF Tree Design — Hard-Won

Several configurations were tried before finding one that works:

| Attempt | Result |
|---|---|
| Fusion publishes `map->base_footprint`, static TF publishes `map->odom`, Kobuki publishes `odom->base_footprint` | Nav2 resolves through `map->odom->base_footprint` and always sees (0,0) |
| Fusion publishes `odom->base_footprint` only | Nav2 cannot locate the robot in the map frame |
| **Fusion publishes `map->base_footprint`; Kobuki TF disabled (`publish_tf: False`); no static `map->odom`** | **Works** |

Symptom of the conflict: TF flickers each second between the true position
(2.330, -1.357) and near-origin (0.003, 0.000) — two publishers fighting over the
same transform.

---

## 12. Phase 9 — Nav2 Autonomous Navigation

### Install

```bash
sudo apt install -y ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-topic-tools
```

### cmd_vel Relay

Nav2 publishes `/cmd_vel`; Kobuki listens on `/commands/velocity`.

```bash
ros2 run topic_tools relay /cmd_vel /commands/velocity
```

> Do **not** relay odometry the same way. An earlier attempt to relay into `/odom`
> failed because Kobuki also publishes there and wins. Point Nav2 at
> `/marvelmind_odom` directly instead.

### Static Map Generation

No SLAM — generate a blank map with border walls sized to the beacon area plus margin.

```python
import math

x_min, x_max = -0.5, 5.5
y_min, y_max = -3.2, 0.5
resolution = 0.05

width_px  = int(math.ceil((x_max - x_min) / resolution))
height_px = int(math.ceil((y_max - y_min) / resolution))
pixels = [254] * (width_px * height_px)

border = 3
for x in range(width_px):
    for b in range(border):
        pixels[b * width_px + x] = 0
        pixels[(height_px - 1 - b) * width_px + x] = 0
for y in range(height_px):
    for b in range(border):
        pixels[y * width_px + b] = 0
        pixels[y * width_px + (width_px - 1 - b)] = 0

with open('/home/uas/nav2_config/map.pgm', 'wb') as f:
    f.write(f'P5\n{width_px} {height_px}\n255\n'.encode())
    f.write(bytes(pixels))
```

`~/nav2_config/map.yaml`:

```yaml
image: map.pgm
resolution: 0.05
origin: [-0.5, -3.2, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

> Size the map generously. An early map ending at `y = -2.645` caused the robot to
> drive out of bounds to (3.71, -2.65) and then loop forever failing to plan from
> outside the map.

### Key Nav2 Parameters

`~/nav2_config/nav2_params.yaml`:

```yaml
bt_navigator:
  ros__parameters:
    use_sim_time: false
    global_frame: map
    robot_base_frame: base_footprint
    odom_topic: /marvelmind_odom

controller_server:
  ros__parameters:
    use_sim_time: false
    controller_frequency: 10.0
    odom_topic: /marvelmind_odom
    costmap_update_timeout: 10.0
    plugins: ["FollowPath"]
    FollowPath:
      plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
      desired_linear_vel: 0.15
      lookahead_dist: 0.8
      min_lookahead_dist: 0.5
      max_lookahead_dist: 1.5
      lookahead_time: 1.5
      rotate_to_heading_angular_vel: 0.3
      transform_tolerance: 1.0
      use_collision_detection: false
      use_regulated_linear_velocity_scaling: true
      min_approach_linear_velocity: 0.05
```

Notes:

- `use_collision_detection: false` — no obstacle sensors exist.
- Local costmap plugins were disabled entirely for the same reason; they caused
  `Costmap timed out waiting for update` aborts.
- Lookahead was raised from 0.4 m to 0.8 m to stop controller ping-ponging at 0.15–0.2 m/s.

### Send a Goal

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{
  pose: {
    header: {frame_id: 'map'},
    pose: {
      position: {x: 2.8, y: -1.241, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    }
  }
}"
```

### Kill a Stuck Nav2

Ctrl-C is frequently ignored by the launch system.

```bash
sudo killall -9 lifecycle_manager bt_navigator controller_server \
  planner_server behavior_server waypoint_follower smoother_server map_server
```

### Square Loop Test

`~/square_loop.py` drives four waypoints via the `NavigateToPose` action —
the standard end-to-end regression test.

---

## 13. Phase 10 — Yaw Calibration

Gyro integration gives *relative* heading. The map frame needs *absolute* heading.
`calibrate_yaw` measures the offset between them.

### Method

1. Record Marvelmind start position
2. Drive 0.5 m forward on `/commands/velocity`
3. Record end position
4. `heading_truth = atan2(dy, dx)`
5. `yaw_offset = heading_truth - imu_raw_yaw`, normalized to +/-pi
6. Write to `/home/uas/yaw_offset.json`

The fusion node loads this file at startup.

### Run

Requires Marvelmind, Kobuki, and Fusion running. Nav2 and the cmd_vel relay are **not**
needed — `calibrate_yaw` publishes directly to `/commands/velocity`.

```bash
source ~/kobuki_ws/install/setup.bash
~/kobuki_ws/install/marvelmind_fusion/bin/calibrate_yaw
```

Sample result:

```
Start position: (2.550, -1.279)
End position:   (2.183, -1.258)
Distance moved: 0.368m
True heading:   176.7 degrees
IMU raw yaw:    0.1 degrees
YAW OFFSET:     176.6 degrees
Saved to /home/uas/yaw_offset.json
```

This revealed the robot's physical forward direction was **west (negative x)** while the
Kobuki IMU reported east — a 180-degree error that explained months of wrong-direction driving.

> Ensure ~0.5 m of clear space ahead. If the measured distance is near zero, the
> Kobuki is not running or not moving, and the calibration is invalid.

---

## 14. Phase 11 — MicroStrain 3DM-GQ4-45 IMU Integration

Replaces the Kobuki IMU as the heading source. This is the most recent work and the
most involved.

### Why

The Kobuki IMU showed a persistent ~178-degree offset that reasserted itself within
seconds of an odometry reset. A hardcoded `YAW_OFFSET` of -193 degrees was tried and
did not hold. The MicroStrain is a proper AHRS with far better bias stability.

### Device Detection

```bash
lsusb
sudo dmesg | tail -5
```

```
usb 2-1: Product: Lord Inertial Sensor
usb 2-1: Manufacturer: Lord Microstrain
usb 2-1: SerialNumber: 0000__6250.88495
cdc_acm 2-1:1.0: ttyACM0: USB ACM device
```

Enumerates as `/dev/ttyACM0`. Confirm serial permissions:

```bash
groups     # must include 'dialout'
sudo usermod -aG dialout uas    # if missing, then log out and back in
```

### Confirm the Data Stream

The device streams MIP protocol packets continuously with no configuration needed.

```bash
timeout 2 cat /dev/ttyACM0 > /tmp/imu_raw.bin; ls -l /tmp/imu_raw.bin
xxd /tmp/imu_raw.bin | head -20
```

`7565` is the MIP sync word (`0x75 0x65`). Approximately 5 KB/s when fully streaming.

> Do **not** pipe into `wc -c` with `timeout` — the pipe tears down before `wc`
> flushes and prints nothing, which looks like a dead device. Redirect to a file.

### Official Driver — Builds, Does Not Work

```bash
cd ~/kobuki_ws/src
git clone https://github.com/LORD-MicroStrain/microstrain_inertial.git -b ros2
cd microstrain_inertial
git submodule update --init --recursive     # REQUIRED — pulls the MIP SDK
cd ~/kobuki_ws
sudo apt update                              # REQUIRED — see 404 note below
sudo apt install ros-jazzy-nmea-msgs ros-jazzy-rtcm-msgs ros-jazzy-diagnostic-aggregator
colcon build --packages-select microstrain_inertial_driver microstrain_inertial_msgs
```

Builds cleanly (~3m43s on a Pi 5). Then:

```
[INFO]  Attempting to open serial port </dev/ttyACM0> at <115200>
[INFO]  Setting device to idle in order to configure
[ERROR] Unable to set device to idle
[ERROR]   Error(-4): Timed out
[FATAL] Failed to configure node
```

Findings during diagnosis:

- **The device IS supported.** `MODEL_3DM_GQ4_45 = 6250` appears in
  `mip_sdk/src/c/mip/mip_device_models.h`, and matches the `6250` in the USB serial number.
- **The launch file accepts no `port` or `baudrate` arguments.** Only `namespace`,
  `node_name`, `debug`, `params_file`. Passing `port:=` / `baudrate:=` is silently
  ignored — ROS 2 launch discards unknown args. Verify with `--show-args`.
- Parameter names in a params file are correct as `port` and `baudrate`.
- `debug:=true` only raises the ROS logger level; it does not enable MIP SDK byte tracing.
- The device streams fine but never answers commands. Root cause not fully determined —
  likely a firmware-level protocol difference on this older unit.

**Conclusion: bypass the driver.** The device streams everything needed unprompted.

### MIP Stream Contents

A descriptor scan of a 2-second capture:

| Set | Field | Len | Contents |
|---|---|---|---|
| 0x80 | 0x04 | 14 | Scaled accelerometer |
| 0x80 | 0x05 | 14 | **Scaled gyro (rad/s)** |
| 0x80 | 0x07 | 14 | Delta theta |
| 0x80 | 0x0C | 14 | **Complementary-filter Euler angles (rad)** |
| 0x80 | 0x10/0x11/0x12 | 14 | Additional sensor fields |
| 0x81 | 0x04–0x09 | var | GNSS data |
| 0x82 | 0x02 | 16 | NED velocity |
| 0x82 | 0x04 | 40 | Attitude matrix (9 floats) |
| 0x82 | 0x05 | 16 | EKF Euler angles |
| 0x82 | 0x11 | 14 | Filter status flags |

> **MIP field length includes the 2-byte header.** A field with `LEN 16` carries only
> 14 bytes of data. Getting this wrong produces a parser that silently matches nothing.

### The EKF Is Unusable Indoors

Descriptor set `0x82` (the GNSS-aided Kalman filter) reports `valid=0` and all zeros
indoors. The GQ4-45 initializes its EKF from a GNSS fix; without one it never converges,
and manual initialization requires the command interface that does not respond.

**Use `0x80/0x0C` and `0x80/0x05` instead** — these are the onboard complementary filter
and raw gyro, which need no initialization.

### Sensor Characterization Results

All measured on the actual robot with the IMU mounted at the base, between the plates.

**Static stability (complementary filter, 0x80/0x0C)**

```
20 s stationary: yaw held +14.89 to +14.94   (+/-0.03 deg)
```

**Convergence after a turn — the disqualifying problem**

```
t=25s  yaw=-3.33
t=27s  yaw=+26.60
t=30s  yaw=+47.42
t=35s  yaw=+53.17
t=40s  yaw=+54.73
t=45s  yaw=+54.91   <- ~10 seconds to converge
```

Nav2's controller runs at 20 Hz. A heading estimate that is wrong by tens of degrees for
ten seconds after every turn is what caused the earlier corner-spinning behavior.

**Accuracy is good, though.** A commanded turn produced 40.02 degrees reported against
~40 degrees actual. The shortfall versus the 143-degree command was Kobuki wheel slip,
not sensor error. The magnetometer is **not** meaningfully corrupted by the chassis
despite the low mounting position.

**Raw gyro bias (0x80/0x05)**

```
-0.148 deg/s constant, noise +/-0.02 deg/s
```

Uncorrected this is ~9 deg/min. Bias-compensated, residual drift is ~0.002 deg/s
(about 7 deg/hour) — a random walk rather than a steady creep.

### Solution: Bias-Compensated Gyro Integration

`~/microstrain_yaw_node.py` — standalone script, no ROS package needed.

Design:

1. Read MIP packets directly from `/dev/ttyACM0` — no driver, no handshake
2. Average `0x80/0x05` gyro-Z for 10 s stationary to estimate bias
3. Integrate `(gz - bias) * dt` for yaw
4. Publish `sensor_msgs/Imu` on `/imu/data` at ~25 Hz
5. Optional slow correction toward the CF heading, **disabled by default**

Run:

```bash
source /opt/ros/jazzy/setup.bash
python3 ~/microstrain_yaw_node.py
```

Hold the robot still for the first 10 seconds. Expected log:

```
Calibrating gyro bias - hold still for 10s...
Bias = -0.002583 rad/s (-0.148 deg/s), N samples. Publishing.
```

### Configuration

```python
PORT       = '/dev/ttyACM0'
BAUD       = 115200
CAL_SECS   = 10.0     # stationary calibration window
STILL_RATE = 0.01     # rad/s below which the robot counts as stationary
STILL_TIME = 2.0      # seconds of stillness before trusting the CF
CORRECT_K  = 0.0      # CF correction gain - KEEP AT 0.0
```

> **`CORRECT_K` must stay 0.0.** With correction enabled, the node chases the CF heading
> while the CF is itself still converging, reintroducing the exact lag the design removes.
> Marvelmind position will provide drift correction instead.

### Validated Performance

With `CORRECT_K = 0.0`:

| Phase | Result |
|---|---|
| 16 s baseline | 0.03 deg drift |
| 90-degree hand rotation | tracked in under 3 s, no lag |
| 27 s hold after turn | 0.04 deg drift |
| Second rotation | +96.3 deg, tracked cleanly |
| 13 s hold | 0.02 deg drift |

Under motor power (teleop driving):

| Phase | Result |
|---|---|
| Motors powered, idle | flat — no current-induced interference |
| Driving straight | ~1.3 deg change (real, Kobuki wheel imbalance) |
| Turn then hold 10 s | dead flat, no settling lag |

Conclusion: motor current and vibration do not disturb the gyro. **The IMU is validated
as a heading source.**

### rclpy Gotcha

`rclpy.Node` has a `handle` **property**. Defining a method named `handle()` on a Node
subclass shadows it, and `Node.__init__` fails with:

```
TypeError: 'method' object does not support the context manager protocol
```

This looks like a broken ROS install but is a name collision. Also avoid `context`,
`executor`, and `clock` as attribute names.

---

## 15. Topic Reference

| Topic | Type | Publisher | Notes |
|---|---|---|---|
| `/commands/velocity` | `geometry_msgs/Twist` | teleop / relay | **Kobuki's actual input** |
| `/cmd_vel` | `geometry_msgs/Twist` | Nav2 | Must be relayed |
| `/kobuki_odom` | `nav_msgs/Odometry` | Kobuki | Remapped from `/odom` |
| `/hedgehog_pos` | `HedgePosition` | marvelmind_ros2 | 8 Hz, x/y in metres |
| `/imu/data` | `sensor_msgs/Imu` | microstrain_yaw_node | 25 Hz, yaw quaternion |
| `/marvelmind_odom` | `nav_msgs/Odometry` | fusion_node | Nav2's odom source |
| `/sensors/imu_data` | `sensor_msgs/Imu` | Kobuki | Onboard IMU — superseded |
| `/commands/reset_odometry` | `std_msgs/Empty` | manual | Resets Kobuki odom |

Other Kobuki topics: `/events/bumper`, `/events/cliff`, `/events/wheel_drop`,
`/sensors/battery_state`, `/sensors/core`, `/commands/sound`, `/commands/led1`,
`/commands/led2`, `/commands/motor_power`.

---

## 16. File Inventory

```
~/kobuki_ws/
├── kobuki_launch.py                  # custom: publish_tf off, /odom remapped
├── src/
│   ├── kobuki_core/  kobuki_ros/  ecl_*/
│   ├── kinect_ros2/                  # 2 header patches applied
│   ├── freenect_stack/
│   ├── marvelmind_ros2_upstream/
│   ├── marvelmind_ros2_msgs_upstream/
│   ├── microstrain_inertial/         # builds; driver does not connect
│   └── marvelmind_fusion/
│       ├── marvelmind_fusion/
│       │   ├── fusion_node.py
│       │   ├── goto_goal.py
│       │   ├── perimeter.py
│       │   └── calibrate_yaw.py
│       ├── package.xml
│       └── setup.py
├── build/  install/  log/

~/nav2_config/
├── nav2_params.yaml
├── nav2_launch.py
├── map.pgm
└── map.yaml

~/
├── microstrain_yaw_node.py           # MIP parser -> /imu/data
├── mip_yaw_test.py                   # diagnostic: print yaw
├── mip_gyro_test.py                  # diagnostic: print gyro
├── mip_scan.py                       # diagnostic: descriptor inventory
├── log_yaw.py                        # logs /imu/data to /tmp/yaw_log.txt
├── wasd.py                           # WASD teleop
├── wasd_imu.py                       # WASD teleop + IMU logging
├── square_loop.py                    # 4-waypoint Nav2 test
├── kinect_stream.py                  # Flask MJPEG server
├── yaw_offset.json                   # calibration output
└── microstrain_config/gq4_params.yml
```

---

## 17. Startup Sequence

### Manual Driving

```bash
# T1
source ~/kobuki_ws/install/setup.bash
ros2 launch kobuki_node kobuki_node-launch.py

# T2
source ~/kobuki_ws/install/setup.bash
python3 ~/wasd.py
```

### IMU Only

```bash
source /opt/ros/jazzy/setup.bash
python3 ~/microstrain_yaw_node.py     # hold still 10 s

# verify
ros2 topic hz /imu/data               # ~25 Hz
ros2 topic echo /imu/data --field orientation.z
```

### Full Autonomous Stack

```bash
# T1 - Marvelmind
source ~/kobuki_ws/install/setup.bash
ros2 run marvelmind_ros2 marvelmind_ros2 --ros-args \
  -p port:=/dev/ttyACM0 -p marvelmind_tty_baudrate:=115200 \
  -p marvelmind_publish_rate_in_hz:=8

# T2 - Kobuki (custom launch)
source ~/kobuki_ws/install/setup.bash
ros2 launch ~/kobuki_ws/kobuki_launch.py

# T3 - IMU
source /opt/ros/jazzy/setup.bash
python3 ~/microstrain_yaw_node.py

# T4 - Fusion
source ~/kobuki_ws/install/setup.bash
~/kobuki_ws/install/marvelmind_fusion/bin/fusion_node

# T5 - Nav2 (wait for "Managed nodes are active")
source /opt/ros/jazzy/setup.bash
source ~/kobuki_ws/install/setup.bash
ros2 launch /home/uas/nav2_config/nav2_launch.py

# T6 - cmd_vel relay
source /opt/ros/jazzy/setup.bash
ros2 run topic_tools relay /cmd_vel /commands/velocity
```

> With both the hedgehog and the IMU on `ttyACM*`, device node assignment is not
> guaranteed across reboots. Verify before launching, or implement the udev rules.

---

## 18. Troubleshooting Log

### Robot does not move

```bash
ros2 topic info /commands/velocity
```

`Subscription count: 0` means the Kobuki node is not running. Also confirm you are
publishing to `/commands/velocity`, not `/cmd_vel`, and at 20 Hz — the base watchdog
zeroes velocity after 0.6 s of silence.

### `Malformed sub-payload detected` from Kobuki

Serial noise. Occasional occurrences are normal. Frequent ones improve on a black
USB 2.0 port rather than a hub.

### apt 404 errors on every ROS package

Stale package index — the ROS buildfarm removes superseded `.deb` files.

```bash
sudo apt update
```

Do **not** run `apt upgrade` mid-project; it can break working builds.

### `dpkg was interrupted`

```bash
sudo dpkg --configure -a
```

### Marvelmind topics exist but never publish

In order of likelihood:

1. Super Modem is plugged into another computer — unplug it
2. `Stream location data` is disabled in the dashboard
3. Wrong topic — use `/hedgehog_pos`, not `/marvelmind_waypoint`

### Marvelmind stuck at 1 Hz

Both must be fixed: UART speed to 115200 in the dashboard, **and**
`-p marvelmind_publish_rate_in_hz:=8` on the driver.

### Nav2 plans from (0,0)

TF conflict. Ensure `publish_tf: False` on Kobuki, no static `map->odom`, and the
fusion node publishing `map->base_footprint`.

### `Costmap timed out waiting for update`

Disable local costmap plugins — there are no obstacle sensors.

### Robot spins in place / ping-pongs

Increase `lookahead_dist` to ~0.8 m. Historically also caused by unreliable
position-derived yaw at low speed — resolved by the dedicated IMU.

### colcon `File exists` on ament_python rebuild

```bash
rm -rf build/<pkg> install/<pkg>
```

### `TypeError: 'method' object does not support the context manager protocol`

A method on your Node subclass is shadowing an rclpy property. Rename it (commonly
`handle`).

### SSH drops during long builds

```bash
screen -S build
# reattach: screen -r build
```

---

## 19. Known Issues and Current State

### Working

- Pi 5 with Ubuntu 24.04 + ROS 2 Jazzy, dual-hotspot networking
- Kobuki base driving under keyboard teleop
- Marvelmind position at 8 Hz
- Nav2 launching, planning, and executing goals
- Yaw calibration producing a valid map-frame offset
- **MicroStrain IMU heading — validated, 0.002 deg/s drift, no settling lag**

### Not Working / Incomplete

| Issue | Status |
|---|---|
| Marvelmind not physically set up in current rebuild | Blocks the full stack |
| MicroStrain official ROS driver never connects | Bypassed with custom parser |
| MicroStrain EKF (0x82) unusable indoors | By design — no GNSS; CF/gyro used instead |
| Fusion node still reads yaw from `/kobuki_odom` | Needs rewrite to `/imu/data` |
| Kinect MJPEG stream times out | Deprioritized |
| udev rules not written | Device nodes can swap between boots |
| Robot has left map bounds during navigation | Mitigated by a larger map; not fully solved |

### Open Question

The full square-loop test has not been run with the MicroStrain as heading source.
Four 90-degree turns returning to the start heading would quantify accumulated error
over a realistic path. Based on 0.002 deg/s drift and clean turn tracking, expect a
few degrees at most.

---

## 20. Next Steps

1. **Rewrite `fusion_node.py`** to subscribe to `/imu/data` instead of `/kobuki_odom`.
   Remove the `YAW_OFFSET` hardcode; the MicroStrain does not need it.

   ```python
   from sensor_msgs.msg import Imu
   from tf_transformations import euler_from_quaternion

   self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)

   def imu_callback(self, msg):
       q = msg.orientation
       _, _, self.yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
   ```

2. **Stand Marvelmind back up** — place beacons, freeze the map, verify 8 Hz on
   `/hedgehog_pos`.

3. **Write udev rules** before running three serial devices together.

4. **Re-run `calibrate_yaw`** with the MicroStrain in the loop.

5. **Run the square loop** end to end and record accumulated heading error.

6. **Add Marvelmind-based drift correction** — derive heading from position change
   during straight-line driving and use it to correct the integrated gyro yaw. Immune
   to magnetic interference and free of the CF's settling lag.

7. **Package `microstrain_yaw_node.py`** as a proper ROS 2 package with a launch file.

---

## Appendix — Quick Reference

```bash
# SSH
ssh uas@bolzpi2.local

# Serial devices
lsusb
sudo dmesg | tail -20
ls -l /dev/ttyACM* /dev/ttyUSB*

# Check IMU is streaming
timeout 2 cat /dev/ttyACM0 > /tmp/imu.bin; ls -l /tmp/imu.bin

# Topic checks
ros2 topic list
ros2 topic hz /hedgehog_pos      # expect 8
ros2 topic hz /imu/data          # expect 25
ros2 topic info /commands/velocity

# TF
ros2 run tf2_ros tf2_echo map base_footprint

# Emergency stop
ros2 topic pub --once /commands/velocity geometry_msgs/msg/Twist "{}"

# Kill Nav2
sudo killall -9 lifecycle_manager bt_navigator controller_server \
  planner_server behavior_server waypoint_follower smoother_server map_server
```
