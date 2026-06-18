# Multi-Robot Formation Control using Sliding Mode Control (ROS2)

## Overview

This project implements a multi-robot formation control framework for omnidirectional mobile robots in ROS2 and Gazebo. The system utilizes Sliding Mode Control (SMC) for:

* Trajectory tracking
* Formation maintenance
* Obstacle avoidance
* Multi-robot coordination

The controller is designed for three Omniblue robots operating in a shared Gazebo environment.

---

## Prerequisites

Before using this package, ensure the following software is installed:

### Operating System

* Ubuntu 20.04 (ROS2 Foxy)
* Ubuntu 22.04 (ROS2 Humble)

### Required Software

* ROS2 Foxy or ROS2 Humble
* Gazebo
* RViz2
* Colcon Build System
* Git
* Eigen3

Verify ROS2 installation:

```bash
ros2 --version
```

---

## Creating a ROS2 Workspace

Create a new ROS2 workspace:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

---

## Cloning the Repository

Navigate to the source folder:

```bash
cd ~/ros2_ws/src
```

Clone the repository:

```bash
git clone <YOUR_REPOSITORY_URL>
```

Example:

```bash
git clone https://github.com/username/multi_robot_formation_control.git
```

---

## Building the Workspace

Move to the workspace root:

```bash
cd ~/ros2_ws
```

Source ROS2:

### ROS2 Foxy

```bash
source /opt/ros/foxy/setup.bash
```

### ROS2 Humble

```bash
source /opt/ros/humble/setup.bash
```

Build the workspace:

```bash
colcon build --symlink-install
```

After successful compilation:

```bash
source install/setup.bash
```

---

## Running the Simulation

Launch the multi-robot simulation:

```bash
ros2 launch gazebo_mecanum_plugins formation.launch.py
```

This will start:

* Gazebo simulation
* Three Omniblue robots
* Formation Manager node
* Sliding Mode Controllers
* RViz visualization

---

## Workspace Setup After Every New Terminal

Whenever a new terminal is opened:

```bash
cd ~/ros2_ws

source /opt/ros/humble/setup.bash
source install/setup.bash
```

or for Foxy:

```bash
source /opt/ros/foxy/setup.bash
source install/setup.bash
```

---

## Making Code Changes

After modifying any source code:

```bash
cd ~/ros2_ws
```

Rebuild the affected package:

```bash
colcon build --packages-select gazebo_mecanum_plugins
```

or rebuild the entire workspace:

```bash
colcon build
```

Source the workspace again:

```bash
source install/setup.bash
```

Run the simulation:

```bash
ros2 launch gazebo_mecanum_plugins formation.launch.py
```

---

## Cleaning the Workspace

If build issues occur:

```bash
cd ~/ros2_ws

rm -rf build/
rm -rf install/
rm -rf log/
```

Rebuild:

```bash
colcon build --symlink-install
```

---

## Package Structure

```text
ros2_ws/
└── src/
    ├── gazebo_mecanum_plugins/
    │   ├── src/
    │   ├── include/
    │   ├── launch/
    │   ├── config/
    │   └── CMakeLists.txt
    │
    ├── omniblue_description/
    │
    └── omniblue_gazebo/
```

---

## Main Components

### Formation Manager

Publishes the global formation goal:

```text
/formation_goal
```

### Formation SMC Controller

Responsible for:

* Formation maintenance
* Goal tracking
* Obstacle avoidance
* Sliding Mode Control implementation

### Gazebo Environment

Provides:

* Robot simulation
* Sensor simulation
* Physics engine

### RViz

Provides visualization of:

* Robot poses
* Formation behavior
* Sensor data

---

## Development Workflow

1. Modify source code.
2. Build the workspace.
3. Source the workspace.
4. Launch the simulation.
5. Test and verify controller behavior.
6. Commit and push changes.

Example:

```bash
git add .
git commit -m "Updated formation controller"
git push origin main
```

---

## Future Work

Potential improvements include:

* Robot-to-robot collision avoidance
* Adaptive Sliding Mode Control
* Dynamic formation reconfiguration
* Consensus-based formation control
* Distributed obstacle avoidance
* Multi-goal waypoint navigation
* Hardware deployment on real robots

---

## License

This project is intended for research and educational purposes.
