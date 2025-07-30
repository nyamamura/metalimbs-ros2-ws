#!/bin/bash

# Source the environment setup
source ./setup_gazebo_env.sh

# Source ROS2 setup
source /opt/ros/humble/setup.bash
source install/setup.bash

# Generate URDF from xacro with absolute paths
echo "Generating URDF from xacro..."
WORKSPACE_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
xacro src/metalimbs_description/robot/metalimbs2_ign.urdf.xacro mesh_path:="file://${WORKSPACE_DIR}/src/metalimbs_description/meshes/metalimbs2" > /tmp/metalimbs2_gazebo.urdf

echo "Generated URDF saved to /tmp/metalimbs2_gazebo.urdf"

# Convert URDF to SDF
echo "Converting URDF to SDF..."
ign sdf -p /tmp/metalimbs2_gazebo.urdf > /tmp/metalimbs2_gazebo.sdf

echo "Generated SDF saved to /tmp/metalimbs2_gazebo.sdf"

# Spawn the robot using Ignition service
echo "Spawning robot in Ignition Gazebo..."
ign service -s /world/default/create \
  --reqtype ignition.msgs.EntityFactory \
  --reptype ignition.msgs.Boolean \
  --timeout 5000 \
  --req "sdf_filename: '/tmp/metalimbs2_gazebo.sdf', name: 'metalimbs', pose: {position: {x: 0, y: 0, z: 0.5}}"

echo "Robot spawn command sent."