#!/bin/bash

echo "Checking if Gazebo is already running..."
if pgrep -x "ign gazebo" > /dev/null; then
    echo "Gazebo is already running. Killing existing process..."
    pkill -f "ign gazebo"
    sleep 2
fi

echo "Checking world file..."
if [ -f /tmp/metalimbs_human_world.sdf ]; then
    echo "World file exists at /tmp/metalimbs_human_world.sdf"
    echo "First 20 lines:"
    head -20 /tmp/metalimbs_human_world.sdf
else
    echo "ERROR: World file not found!"
    exit 1
fi

echo ""
echo "Launching Gazebo with verbose output..."
ign gazebo -v 4 /tmp/metalimbs_human_world.sdf