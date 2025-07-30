#!/bin/bash

# Generate URDF from xacro
xacro src/metalimbs_description/robot/metalimbs2.urdf.xacro > /tmp/metalimbs2.urdf

# Spawn using ros_ign_gazebo create tool
ros2 run ros_ign_gazebo create -name metalimbs -file /tmp/metalimbs2.urdf -x 0 -y 0 -z 0.5