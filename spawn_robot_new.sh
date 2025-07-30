#!/bin/bash

# Source the environment setup
source ./setup_gazebo_env.sh

# Source ROS2 setup
source /opt/ros/humble/setup.bash
source install/setup.bash

# Generate URDF from xacro with absolute paths
echo "Generating URDF from xacro..."
WORKSPACE_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
xacro src/metalimbs_description/robot/metalimbs2_ign_fixed.urdf.xacro mesh_path:="file://${WORKSPACE_DIR}/src/metalimbs_description/meshes/metalimbs2" > /tmp/metalimbs2_gazebo.urdf

echo "Generated URDF saved to /tmp/metalimbs2_gazebo.urdf"

# Spawn the robot using ros_gz_sim
echo "Spawning robot in Ignition Gazebo..."
ros2 run ros_gz_sim create -name metalimbs -file /tmp/metalimbs2_gazebo.urdf -x 0 -y 0 -z 0.5

echo "Robot spawn command sent."