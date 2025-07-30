#!/bin/bash

# Test script to verify mesh loading in Ignition Gazebo

echo "=== Testing Metalimbs Ignition Gazebo Setup ==="
echo ""

# Source the environment
source ./setup_gazebo_env.sh
source /opt/ros/humble/setup.bash
source install/setup.bash

# Generate URDF
echo "1. Generating URDF with absolute mesh paths..."
WORKSPACE_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
xacro src/metalimbs_description/robot/metalimbs2_ign.urdf.xacro mesh_path:="file://${WORKSPACE_DIR}/src/metalimbs_description/meshes/metalimbs2" > /tmp/metalimbs2_test.urdf

echo "   URDF generated at: /tmp/metalimbs2_test.urdf"
echo ""

# Check if URDF contains correct paths
echo "2. Checking mesh paths in generated URDF..."
echo "   First few mesh references:"
grep -m 5 "mesh filename" /tmp/metalimbs2_test.urdf | head -5
echo ""

# Start Ignition Gazebo in background
echo "3. Starting Ignition Gazebo..."
echo "   Run this in a new terminal: ign gazebo -v 4"
echo ""

# Wait for user to start Gazebo
echo "Press Enter when Ignition Gazebo is running..."
read

# Spawn the robot
echo "4. Spawning robot..."
ros2 run ros_gz_sim create -name metalimbs_test -file /tmp/metalimbs2_test.urdf -x 0 -y 0 -z 0.5

echo ""
echo "=== Test Complete ==="
echo "Check if the robot appears with meshes in Ignition Gazebo."
echo "If meshes are not visible, check the Ignition Gazebo console for errors."