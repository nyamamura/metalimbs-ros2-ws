#!/bin/bash

# Exit on error
set -e

# Source ROS2 setup
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
elif [ -f "/opt/ros/galactic/setup.bash" ]; then
    source /opt/ros/galactic/setup.bash
elif [ -f "/opt/ros/foxy/setup.bash" ]; then
    source /opt/ros/foxy/setup.bash
else
    echo "Error: No ROS2 installation found!"
    exit 1
fi

# Build the workspace
echo "Building ROS2 workspace..."
colcon build --symlink-install

# Source the workspace
source install/setup.bash

echo "Build complete!"
echo ""
echo "To launch the Gazebo simulation, run:"
echo "  source install/setup.bash"
echo "  ros2 launch metalimbs_description gazebo.launch.py"
echo ""
echo "To launch RViz visualization, run:"
echo "  source install/setup.bash"
echo "  ros2 launch metalimbs_description display.launch.py"