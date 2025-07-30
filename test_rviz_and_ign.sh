#!/bin/bash

echo "=== Testing Robot in RViz and Ignition Gazebo ==="
echo ""

# Source ROS2 setup
source /opt/ros/humble/setup.bash
source install/setup.bash

# 1. Launch RViz to verify robot structure
echo "1. Launching RViz to verify robot structure..."
echo "   Run this in a new terminal:"
echo "   ros2 launch metalimbs_description display.launch.py"
echo ""
echo "Press Enter when you've verified the robot looks correct in RViz..."
read

# 2. Kill all existing Gazebo instances
echo "2. Stopping any existing Gazebo instances..."
pkill -f "ign gazebo"
sleep 2

# 3. Launch Ignition Gazebo with the same robot
echo "3. Now launching the same robot in Ignition Gazebo..."
cd /media/nyamamura/Windows/Users/allex/source/repos/metalimbs-ros2-ws
./launch_ign_original.sh