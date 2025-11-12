# Dobot CR10 ROS2 Control

Custom ROS 2 hardware interface for Dobot CR10 collaborative robot with real-time TCP/IP control, trajectory execution, and RViz visualization.

![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)
![License](https://img.shields.io/badge/license-MIT-green)

## 📁 Repository Structure

### [`dobot_cr10_hardware/`](./dobot_cr10_hardware)
Custom hardware interface implementation for Dobot CR10

- **[`include/dobot_cr10_hardware/`](./dobot_cr10_hardware/include/dobot_cr10_hardware)**
  - [`dobot_cr10_hardware_interface.hpp`](./dobot_cr10_hardware/include/dobot_cr10_hardware/dobot_cr10_hardware_interface.hpp) - Header file for hardware interface
  
- **[`src/`](./dobot_cr10_hardware/src)**
  - [`dobot_cr10_hardware_interface.cpp`](./dobot_cr10_hardware/src/dobot_cr10_hardware_interface.cpp) - Main hardware interface implementation with TCP/IP communication

- [`CMakeLists.txt`](./dobot_cr10_hardware/CMakeLists.txt) - Build configuration
- [`package.xml`](./dobot_cr10_hardware/package.xml) - Package dependencies

### [`dobot_cr10_description/`](./dobot_cr10_description)
Robot description, launch files, and configurations

- **[`urdf/`](./dobot_cr10_description/urdf)**
  - [`dobot_cr10.urdf.xacro`](./dobot_cr10_description/urdf/dobot_cr10.urdf.xacro) - Robot URDF model with kinematics
  - [`dobot_cr10.ros2_control.xacro`](./dobot_cr10_description/urdf/dobot_cr10.ros2_control.xacro) - ROS2 Control configuration

- **[`launch/`](./dobot_cr10_description/launch)**
  - [`dobot_bringup.launch.py`](./dobot_cr10_description/launch/dobot_bringup.launch.py) - Main launch file for simulation and real robot

- **[`config/`](./dobot_cr10_description/config)**
  - [`dobot_controllers.yaml`](./dobot_cr10_description/config/dobot_controllers.yaml) - Controller parameters and tolerances
  - [`dobot.rviz`](./dobot_cr10_description/config/dobot.rviz) - RViz visualization config

- [`CMakeLists.txt`](./dobot_cr10_description/CMakeLists.txt) - Build configuration
- [`package.xml`](./dobot_cr10_description/package.xml) - Package dependencies

## ✨ Features

- ✅ Custom `ros2_control` hardware interface for Dobot CR10
- ✅ Real-time TCP/IP communication using ServoJ commands
- ✅ Joint trajectory controller for smooth motion planning
- ✅ URDF model with accurate kinematics
- ✅ RViz visualization
- ✅ Support for both simulation and real hardware

## 🔧 My Working System

- Ubuntu 22.04
- ROS 2 Humble
- Dobot CR10 robot (tested on real hardware)

## 📦 Installation

### 1. Clone the repository
mkdir -p ~/dobot_ws/src
cd ~/dobot_ws/src
git clone https://github.com/harshalRC/dobot_cr10_ros2.git .

### 2. Install dependencies
cd ~/dobot_ws
sudo apt update
sudo apt install -y ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-xacro ros-humble-joint-state-publisher ros-humble-joint-state-publisher-gui ros-humble-robot-state-publisher ros-humble-rviz2

### 3. Build the workspace
cd ~/dobot_ws
colcon build
source install/setup.bash

## 🚀 Quick Start

### Simulation (Fake Hardware)

Launch the robot in simulation mode:
ros2 launch dobot_cr10_description dobot_bringup.launch.py

See: [`dobot_bringup.launch.py`](./dobot_cr10_description/launch/dobot_bringup.launch.py)

### Real Robot Control

1. **Connect to Dobot CR10 network** (default: `192.168.5.1`)

2. **Launch with real hardware:**
ros2 launch dobot_cr10_description dobot_bringup.launch.py use_fake_hardware:=false robot_ip:=192.168.5.1

## 🔍 Hardware Interface Details

**Location:** [`dobot_cr10_hardware_interface.cpp`](./dobot_cr10_hardware/src/dobot_cr10_hardware_interface.cpp)

The custom hardware interface communicates with Dobot CR10 via TCP/IP (port 29999):

| Command | Description |
|---------|-------------|
| `EnableRobot()` | Enable robot motors |
| `DisableRobot()` | Disable robot motors |
| `GetAngle()` | Read current joint positions (returns degrees) |
| `ServoJ(j1,j2,j3,j4,j5,j6)` | Send joint position commands (in degrees) |

**Update rate:** 100 Hz  
**Communication:** TCP socket with 50ms response timeout

## 📊 Technical Specifications

- **Control Rate:** 100 Hz
- **Communication Protocol:** TCP/IP
- **Port:** 29999 (Dashboard/Command port)
- **Joint Limits:** Defined in [`dobot_cr10.urdf.xacro`](./dobot_cr10_description/urdf/dobot_cr10.urdf.xacro)
- **Degrees of Freedom:** 6

## 🧪 Tested On

- ✅ ROS 2 Humble
- ✅ Ubuntu 22.04 (WSL2)
- ✅ Dobot CR10 (real hardware validation)

## 🤝 Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](./LICENSE) file for details.

## 🙏 Acknowledgments

- Code tested and validated on real Dobot CR10 hardware
- Based on ROS 2 Control framework and best practices

## 👤 Author

**Harshal Salian**

- GitHub: [@harshalRC](https://github.com/harshalRC)
- Email: harshal22.hs@gmail.com

## ⚠️ Safety Notice

**Always ensure the robot workspace is clear before sending motion commands. Keep emergency stop accessible during testing.**

---
