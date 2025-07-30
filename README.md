# MetaLimbs ROS2 Workspace

This workspace contains the ROS2 port of the MetaLimbs robotic system for Gazebo simulation.

## Prerequisites

- ROS2 (Humble, Galactic, or Foxy)
- Gazebo (usually installed with ROS2)
- Required ROS2 packages:
  ```bash
  sudo apt install ros-$ROS_DISTRO-gazebo-ros-pkgs
  sudo apt install ros-$ROS_DISTRO-gazebo-ros2-control
  sudo apt install ros-$ROS_DISTRO-xacro
  sudo apt install ros-$ROS_DISTRO-joint-state-publisher-gui
  sudo apt install ros-$ROS_DISTRO-robot-state-publisher
  sudo apt install ros-$ROS_DISTRO-rviz2
  sudo apt install ros-$ROS_DISTRO-ros2-controllers
  ```

## Building the Workspace

```bash
cd /media/nyamamura/Windows/Users/allex/source/repos/metalimbs-ros2-ws
./build.sh
```

## Running the Simulation

### Gazebo Simulation

To launch the robot in Gazebo:

```bash
source install/setup.bash
ros2 launch metalimbs_description gazebo.launch.py
```

### RViz Visualization

To visualize the robot model in RViz:

```bash
source install/setup.bash
ros2 launch metalimbs_description display.launch.py
```

## Package Structure

- `metalimbs_description`: Contains the URDF/XACRO files, meshes, and configuration for the robot
  - `robot/`: URDF/XACRO description files
  - `meshes/`: 3D model files
  - `launch/`: Launch files for Gazebo and RViz
  - `config/`: Configuration files for controllers
  - `rviz/`: RViz configuration files

## Robot Description

The MetaLimbs robot consists of:
- Dual robotic arms with 7 degrees of freedom each
- Left arm joints: shoulder (2 DOF), upper arm, elbow, forearm, wrist, handunit mount
- Right arm joints: shoulder (2 DOF), upper arm, elbow, forearm, wrist, handunit mount

## Controllers

The robot uses `joint_trajectory_controller` for controlling all joints simultaneously. The controller configuration can be found in `metalimbs_description/config/ros2_controllers.yaml`.