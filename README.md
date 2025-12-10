# Dobot CR10 ROS2 Control with MoveIt2

Complete ROS 2 integration for Dobot CR10 collaborative robot with custom hardware interface, MoveIt2 motion planning, real-time TCP/IP control, and RViz visualization.

![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)
![MoveIt2](https://img.shields.io/badge/MoveIt2-Humble-orange)
![Jetson](https://img.shields.io/badge/Jetson-Orin%20AGX-76B900)
![License](https://img.shields.io/badge/license-MIT-green)

## Repository Structure

### [`dobot_cr10_hardware/`](./dobot_cr10_hardware)
Custom hardware interface implementation

- **[`include/dobot_cr10_hardware/`](./dobot_cr10_hardware/include/dobot_cr10_hardware)**
  - [`dobot_cr10_hardware_interface.hpp`](./dobot_cr10_hardware/include/dobot_cr10_hardware/dobot_cr10_hardware_interface.hpp) - Hardware interface header
  
- **[`src/`](./dobot_cr10_hardware/src)**
  - [`dobot_cr10_hardware_interface.cpp`](./dobot_cr10_hardware/src/dobot_cr10_hardware_interface.cpp) - TCP/IP communication implementation

- [`dobot_cr10_hardware.xml`](./dobot_cr10_hardware/dobot_cr10_hardware.xml) - Plugin description
- [`CMakeLists.txt`](./dobot_cr10_hardware/CMakeLists.txt) - Build configuration
- [`package.xml`](./dobot_cr10_hardware/package.xml) - Package dependencies

### [`dobot_cr10_description/`](./dobot_cr10_description)
Robot description and launch files

- **[`urdf/`](./dobot_cr10_description/urdf)**
  - [`dobot_cr10.urdf.xacro`](./dobot_cr10_description/urdf/dobot_cr10.urdf.xacro) - Robot URDF with kinematics

- **[`meshes/`](./dobot_cr10_description/meshes)** - STL mesh files

- **[`launch/`](./dobot_cr10_description/launch)**
  - [`dobot_bringup.launch.py`](./dobot_cr10_description/launch/dobot_bringup.launch.py) - Robot bringup
  - [`display.launch.py`](./dobot_cr10_description/launch/display.launch.py) - Visualization

- **[`config/`](./dobot_cr10_description/config)**
  - [`dobot_controllers.yaml`](./dobot_cr10_description/config/dobot_controllers.yaml) - Controller configuration

- [`CMakeLists.txt`](./dobot_cr10_description/CMakeLists.txt)
- [`package.xml`](./dobot_cr10_description/package.xml)

### [`dobot_cr10_moveit_config/`](./dobot_cr10_moveit_config)
MoveIt2 configuration for motion planning

- **[`config/`](./dobot_cr10_moveit_config/config)**
  - [`moveit_controllers.yaml`](./dobot_cr10_moveit_config/config/moveit_controllers.yaml) - MoveIt controller mappings
  - [`ros2_controllers.yaml`](./dobot_cr10_moveit_config/config/ros2_controllers.yaml) - Controller parameters
  - [`joint_limits.yaml`](./dobot_cr10_moveit_config/config/joint_limits.yaml) - Joint limits and velocities
  - [`kinematics.yaml`](./dobot_cr10_moveit_config/config/kinematics.yaml) - IK solver configuration
  - [`dobot_cr10.srdf`](./dobot_cr10_moveit_config/config/dobot_cr10.srdf) - Semantic robot description
  - [`moveit.rviz`](./dobot_cr10_moveit_config/config/moveit.rviz) - RViz config for MoveIt

- **[`launch/`](./dobot_cr10_moveit_config/launch)**
  - [`demo.launch.py`](./dobot_cr10_moveit_config/launch/demo.launch.py) - Complete MoveIt demo with RViz
  - [`move_group.launch.py`](./dobot_cr10_moveit_config/launch/move_group.launch.py) - MoveIt move_group node
  - [`moveit_rviz.launch.py`](./dobot_cr10_moveit_config/launch/moveit_rviz.launch.py) - RViz with MoveIt plugin

- [`CMakeLists.txt`](./dobot_cr10_moveit_config/CMakeLists.txt)
- [`package.xml`](./dobot_cr10_moveit_config/package.xml)

## Features

- Custom ros2_control hardware interface for Dobot CR10
- MoveIt2 integration with motion planning
- OMPL, CHOMP, and Pilz planners support
- Real-time TCP/IP communication using ServoJ commands
- Joint trajectory controller with path tolerance
- Accurate URDF model with collision geometry
- Interactive RViz visualization with MoveIt plugin
- Support for both simulation and real robot

## Installation

### Prerequisites
- Hardware: NVIDIA Jetson Orin AGX or Ubuntu 22.04 system
- OS: Ubuntu 22.04 (JetPack 6.0+ for Jetson)
- ROS: ROS 2 Humble
- Robot: Dobot CR10

### 1. Clone repository
mkdir -p ~/dobot_ws/src
cd ~/dobot_ws/src
git clone https://github.com/harshalRC/dobot_cr10_ros2.git .

### 2. Install dependencies
cd ~/dobot_ws
sudo apt update
sudo apt install -y ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-controller-manager ros-humble-joint-state-broadcaster ros-humble-joint-trajectory-controller ros-humble-xacro ros-humble-joint-state-publisher ros-humble-joint-state-publisher-gui ros-humble-robot-state-publisher ros-humble-rviz2 ros-humble-moveit ros-humble-moveit-ros-planning ros-humble-moveit-ros-planning-interface ros-humble-moveit-kinematics ros-humble-moveit-planners-ompl ros-humble-moveit-simple-controller-manager ros-humble-pilz-industrial-motion-planner

### 3. Build workspace
cd ~/dobot_ws
colcon build
source install/setup.bash
echo "source ~/dobot_ws/install/setup.bash" >> ~/.bashrc

## Quick Start

### Simulation with MoveIt2
ros2 launch dobot_cr10_moveit_config demo.launch.py use_fake_hardware:=true

Features:
- Interactive RViz window with MoveIt plugin
- Plan and execute trajectories
- Drag interactive markers to set goal pose
- Test motion planning algorithms

### Real Robot Control with MoveIt2

#### 1. Network Setup
sudo nmcli connection modify "Wired connection 1" ipv4.method manual ipv4.addresses 192.168.5.100/24
sudo nmcli connection down "Wired connection 1"
sudo nmcli connection up "Wired connection 1"
ping 192.168.5.1

#### 2. Launch MoveIt with Real Hardware
ros2 launch dobot_cr10_moveit_config demo.launch.py use_fake_hardware:=false

#### 3. Check Connection
ros2 topic echo /joint_states
ros2 control list_controllers

Expected output:
arm_controller          [joint_trajectory_controller] active
joint_state_broadcaster [joint_state_broadcaster]     active

### Basic Robot Bringup (Without MoveIt)

Simulation:
ros2 launch dobot_cr10_description dobot_bringup.launch.py use_fake_hardware:=true

Real robot:
ros2 launch dobot_cr10_description dobot_bringup.launch.py use_fake_hardware:=false robot_ip:=192.168.5.1

## Using MoveIt2

### In RViz:
1. Set Goal State: Drag the interactive marker to desired position
2. Plan: Click "Plan" button in MotionPlanning panel
3. Execute: Review planned path, then click "Execute"

### Planning Groups:
- arm - All 6 joints (joint_1 through joint_6)

### Supported Planners:
- OMPL: RRTConnect, RRT, PRM
- CHOMP: Covariant Hamiltonian Optimization
- Pilz: LIN, PTP, CIRC (industrial motion)

## Hardware Interface Details

**Implementation:** [`dobot_cr10_hardware_interface.cpp`](./dobot_cr10_hardware/src/dobot_cr10_hardware_interface.cpp)

### TCP/IP Command Protocol

Communicates with Dobot CR10 Dashboard Server (port 29999):

| Command | Description | Response |
|---------|-------------|----------|
| EnableRobot() | Enable robot motors | 0,{},EnableRobot(); |
| DisableRobot() | Disable robot motors | 0,{},DisableRobot(); |
| GetAngle() | Read joint positions | 0,{j1,j2,j3,j4,j5,j6},GetAngle(); |
| ServoJ(j1,j2,j3,j4,j5,j6) | Send position command | 0,{},ServoJ(); |
| RobotMode() | Get robot mode/state | 0,{mode},RobotMode(); |

Control Loop:
- Update Rate: 100 Hz
- Communication: Non-blocking TCP socket
- Timeout: 50ms per read operation
- Unit Conversion: Hardware interface converts rad to degrees

## Technical Specifications

### Robot
- Model: Dobot CR10
- DOF: 6
- Reach: 1300mm
- Payload: 10kg
- Repeatability: ±0.02mm

### Control System
- Control Rate: 100 Hz
- Protocol: TCP/IP
- Port: 29999 (Dashboard/Command)
- Command Mode: ServoJ (position control)
- Controller Type: JointTrajectoryController

### Motion Planning
- Framework: MoveIt2
- Default Planner: RRTConnect
- IK Solver: KDL
- Planning Group: arm (6 joints)

## Tested Environment

- ROS 2: Humble Hawksbill
- OS: Ubuntu 22.04 LTS
- Platform: NVIDIA Jetson Orin AGX (ARM64), x86_64, WSL2
- Hardware: Dobot CR10 (validated on real robot)
- MoveIt2: Humble distribution

## Troubleshooting

### Robot Not Connecting
ping 192.168.5.1
nc 192.168.5.1 29999

Type: RobotMode()
Type: GetErrorID()

### Controllers Not Loading
ros2 control list_controllers
ros2 control list_hardware_interfaces

### MoveIt Planning Fails
- Verify joint limits in joint_limits.yaml
- Check for collisions in planning scene
- Ensure start state is valid

## Contributing

Contributions welcome! Please:
1. Fork the repository
2. Create feature branch (git checkout -b feature/amazing-feature)
3. Commit changes (git commit -m 'Add amazing feature')
4. Push to branch (git push origin feature/amazing-feature)
5. Open Pull Request

## License

MIT License

## Acknowledgments

- Code tested and validated on real Dobot CR10 hardware
- Built on ROS 2 Control and MoveIt2 frameworks
- Based on ros2_control best practices

## Author

Harshal Salian
- GitHub: @harshalRC
- Email: harshal22.hs@gmail.com

## Safety Notice

Always ensure the robot workspace is clear before executing motion commands. Keep emergency stop accessible at all times.
