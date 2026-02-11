# PX4 Keyboard & GUI Teleoperation (ROS 2 DDS + Isaac Sim)

This package provides **keyboard and GUI-based OFFBOARD position control** for PX4 using the **ROS 2 DDS bridge (Micro XRCE-DDS)**.  
It is designed to work **simultaneously with QGroundControl and Isaac Sim**. 

One drone and Many Drones in a Warehouse Digital Twin

You can:
- Arm / Disarm
- Switch to OFFBOARD
- Takeoff, Hold, Land
- Move **relative to the drone’s body frame** (W/A/S/D follows the nose)
- Control altitude and yaw
- Use either **terminal keyboard teleop** or an **interactive GUI**

---

## System Architecture

```
Isaac Sim ── DDS ── PX4 (SITL)
                     │
                     ├── MAVLink → QGroundControl
                     └── DDS → ROS 2 (Teleop / GUI)
```

All three run **simultaneously**.

---

## Prerequisites

### OS
- Ubuntu 22.04

### Software
- ROS 2 Humble
- PX4 Autopilot (SITL)
- Isaac Sim 5.x
- QGroundControl
- Micro XRCE-DDS Agent
- Python 3.10+

### ROS 2 packages
```bash
sudo apt install ros-humble-px4-msgs python3-pyqt5
```

---

## Workspace Layout

```
ws_px4_ros2/
├── src/
│   └── px4_keyboard_teleop/
│       ├── package.xml
│       ├── setup.py
│       ├── resource/
│       │   └── px4_keyboard_teleop
│       └── px4_keyboard_teleop/
│           ├── __init__.py
│           ├── teleop.py
│           └── gui.py
```

---

## Build Instructions

```bash
cd ~/ws_px4_ros2
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

Verify executables:
```bash
ros2 pkg executables px4_keyboard_teleop
```

Expected:
```
px4_keyboard_teleop teleop
px4_keyboard_teleop gui
```

---

## Running the Full Stack

### Terminal 1 – DDS Agent
```bash
MicroXRCEAgent udp4 -p 8888
```

### Terminal 2 – Isaac Sim (PX4 SITL)
```bash
cd PegasusSimulator
~/isaacsim/python.sh warehouse_drone.py --px4_dir /home/PX4-Autopilot
```

### Terminal 3 – QGroundControl
Launch QGC normally (auto-connects via MAVLink).

### Terminal 4 – GUI Teleop
```bash
source /opt/ros/humble/setup.bash
source ~/ws_px4_ros2/install/setup.bash
export ROS_DOMAIN_ID=0

ros2 run px4_keyboard_teleop gui
```

---

## GUI Controls

### Buttons
- Init SP = Current
- ARM
- DISARM
- OFFBOARD
- TAKEOFF
- HOLD
- LAND

### Keyboard Controls (GUI focused)

| Key | Action |
|---|---|
| W / S | Forward / Backward (body frame) |
| A / D | Left / Right (body frame) |
| R / F | Up / Down |
| Q / E | Yaw Left / Right |
| Space | Arm |
| Shift + D | Disarm |
| O | Offboard |
| T | Takeoff |
| H | Hold |
| L | Land |

---

## Notes

- Movement is **relative to drone heading**
- OFFBOARD requires continuous setpoint streaming (20 Hz)
- PX4 may refuse disarm while airborne
- DDS and MAVLink can run together safely

---

## License
Apache-2.0

---

## Author
Varun
