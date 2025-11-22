# Autonomous Warehouse Robot

A Complete ROS 2 implementation of an autonomous mobile manipulator for warehouse item pickup tasks. This project demonstrates integration of navigation (Nav2), manipulation (MoveIt), and perception in a simulated warehouse environment.

### Project Overview

**Goal:** To build a simulated robot that autonomously navigates through a warehouse, locating items on shelves, pick them up with a robotic arm, and deliver them to a drop-off zone.

**Key Features:** 
- Differential drive mobile base with autonomous navigation
- Robotic arm with motion planning for manipulation of tasks
- Simulated warehouse environment with realistic physics
- Integrating multiple ROS 2 subsytems

## Technology Stack
| Component | Technology | Purpose |
|-----------|-----------|---------|
| **Framework** | ROS 2 Humble | Robot Operating System |
| **Simulation** | Gazebo Classic | Physics simulation and 3D visualization |
| **Navigation** | Nav2 | Autonomous path planning and obstacle avoidance |
| **Manipulation** | MoveIt 2 | Motion planning for robotic arm |
| **Visualization** | RViz2 | Real-time robot state and sensor data visualization |

## Package Structure
autonomous_warehouse_robot/
├── warehouse_robot_description/    # Robot URDF models and configurations
├── warehouse_robot_gazebo/         # Gazebo worlds and simulation setup
├── warehouse_robot_navigation/     # Nav2 configuration and maps
├── warehouse_robot_manipulation/   # MoveIt configuration for arm control
└── warehouse_robot_core/           # Main mission logic and coordination

##  Current Progress

### Phase 1: Foundation (In Progress)
- [x] Project structure and package creation
- [x] Mobile base URDF with differential drive
- [ ] Gazebo world creation
- [ ] Basic teleoperation

### Phase 2: Navigation (Upcoming)
- [ ] Add laser scanner and sensors
- [ ] Create warehouse environment
- [ ] Map generation with SLAM
- [ ] Autonomous navigation with Nav2

### Phase 3: Manipulation (Planned)
- [ ] Add robotic arm to robot description
- [ ] Configure MoveIt for motion planning
- [ ] Implement pick and place actions
- [ ] Gripper control

### Phase 4: Integration (Planned)
- [ ] Mission state machine
- [ ] Object detection and localization
- [ ] Full workflow: navigate → detect → pick → return
- [ ] Error handling and recovery behaviors

## Prerequisites

- **OS:** Ubuntu 22.04 LTS
- **ROS:** ROS 2 Humble
- **Gazebo:** Gazebo Classic 11


- **OS:** Ubuntu 22.04 LTS
- **ROS:** ROS 2 Humble
- **Gazebo:** Gazebo Classic 11

## Installation

### 1. Install ROS 2 Humble

Follow the official [ROS 2 Humble installation guide](https://docs.ros.org/en/humble/Installation.html).

### 2. Install Dependencies
```bash
sudo apt update
sudo apt install -y \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-moveit \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-xacro \
  ros-humble-rviz2 \
  ros-humble-teleop-twist-keyboard
```

### 3. Clone and Build
```bash
# Create workspace
mkdir -p ~/autonomous_warehouse_robot/src
cd ~/autonomous_warehouse_robot/src

# Clone this repository
git clone https://github.com/OluomaOji/autonomous-warehouse-robot.git .

# Build
cd ..
colcon build

# Source the workspace
source install/setup.bash
```

## Usage

### View Robot Model in RViz
```bash
ros2 launch warehouse_robot_description display.launch.py
```

### Launch Gazebo Simulation (Coming Soon)
```bash
ros2 launch warehouse_robot_gazebo spawn_robot.launch.py
```

### Run Autonomous Navigation (Coming Soon)
```bash
ros2 launch warehouse_robot_navigation navigation.launch.py
```

## Documentation

Detailed documentation for each package can be found in their respective directories:
- [Robot Description](src/warehouse_robot_description/README.md)
- [Gazebo Simulation](src/warehouse_robot_gazebo/README.md) (Coming Soon)
- [Navigation](src/warehouse_robot_navigation/README.md) (Coming Soon)
- [Manipulation](src/warehouse_robot_manipulation/README.md) (Coming Soon)
- [Core Logic](src/warehouse_robot_core/README.md) (Coming Soon)

## Learning Goals

This project is designed to demonstrate:
- Professional ROS 2 package organization
- Robot description using URDF/Xacro
- Gazebo simulation setup and configuration
- Nav2 integration for autonomous navigation
- MoveIt integration for manipulation
- Multi-component system integration
- Software engineering best practices in robotics

## Contributing

This is a personal learning project, but suggestions and feedback are welcome! Feel free to open issues or submit pull requests.

## License

This project is licensed under the Apache License 2.0 - see the [LICENSE](LICENSE) file for details.

## Author

**Your Name**
- GitHub: [OluomaOji](https://github.com/OluomaOji)
- LinkedIn: [Oluoma Oji](https://linkedin.com/in/OluomaOji)
- Email: oluomaoji@gmail.com

---

**Note:** This project is under active development. Check back for update


