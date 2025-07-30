#!/bin/bash

# Generate URDF from fixed xacro
xacro src/metalimbs_description/robot/metalimbs2_fixed.urdf.xacro > /tmp/metalimbs2_fixed.urdf

# Spawn using ros_gz_sim create tool
ros2 run ros_gz_sim create -name metalimbs -file /tmp/metalimbs2_fixed.urdf -x 0 -y 0 -z 0.5