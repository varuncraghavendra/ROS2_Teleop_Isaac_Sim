# PX4 ROS 2 Offboard Control Workspace

This repository contains a **ROS 2 (Humble) workspace** for controlling a **PX4-based UAV in OFFBOARD mode** using ROS 2 and `px4_msgs`.  
It demonstrates how to send **offboard heartbeats, trajectory setpoints, and vehicle commands** from a custom ROS 2 node to PX4 (SITL or real hardware).

The project is designed for **learning, experimentation, and research** with PX4–ROS 2 integration.

---

## 🚀 Features

- ROS 2 Humble compatible  
- PX4 OFFBOARD mode control  
- Python-based ROS 2 node  
- Publishes:
  - `OffboardControlMode`
  - `TrajectorySetpoint`
  - `VehicleCommand`
- Works with PX4 SITL (Gazebo / Isaac Sim compatible)  
- Minimal, easy-to-extend codebase  

---

## 📁 Repository Structure

```
ws_px4_ros2/
├── src/
│   ├── my_px4_offboard/
│   │   └── src/
│   │       └── offboard_node.py
│   ├── px4_msgs/
│   └── px4_ros_com/
├── log/
├── build/
├── install/
└── README.md
```

---

## 🧠 How It Works

PX4 requires **continuous offboard setpoints** before and during OFFBOARD mode.

This node performs:
1. **OFFBOARD heartbeat publishing**  
2. **Trajectory setpoint streaming**  
3. **Vehicle command handling** (arming, mode switching, landing)

⚠️ PX4 will **exit OFFBOARD mode** if setpoints stop streaming.

---

## 🛠️ Prerequisites

- Ubuntu 22.04  
- ROS 2 Humble  
- Python 3.10  
- PX4 Autopilot (SITL recommended)  

---

## 📦 Installation

```bash
git clone <your-repo-url>
cd ws_px4_ros2
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -r -y
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

---

## ▶️ Running the System

### Start Isaac Sim to recieve ROS2 Commands

```bash
cd PegasusSimulator
~/isaacsim/python.sh 1_px4_single_vehicle.py 
```

### Run QGroundcontrol

```
 ./QGroundControl-x86_64.AppImage
```

---

### Run the Offboard Node

```bash
export ROS_DOMAIN_ID=0
ros2 run px4_keyboard_teleop gui
```

---

## 📡 Topics Used

| Topic | Type |
|------|------|
| `/fmu/in/offboard_control_mode` | `OffboardControlMode` |
| `/fmu/in/trajectory_setpoint` | `TrajectorySetpoint` |
| `/fmu/in/vehicle_command` | `VehicleCommand` |

---

## 🧪 Tested With

- PX4 v1.14+  
- ROS 2 Humble  
- Isaac Sim 5.1

---

## 👤 Author

**Varun Raghavendra**  
Robotics | UAV Systems | ROS 2 | PX4 | Autonomous Systems  

---

## 📄 License

MIT License
