#!/bin/bash

# Setup Ignition Gazebo environment for mesh loading
echo "Setting up Ignition Gazebo environment..."

# Get the workspace directory
WORKSPACE_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Set IGN_GAZEBO_RESOURCE_PATH to include the metalimbs_description package
export IGN_GAZEBO_RESOURCE_PATH="${WORKSPACE_DIR}/src/metalimbs_description/models:${WORKSPACE_DIR}/src/metalimbs_description:${IGN_GAZEBO_RESOURCE_PATH}"

# Also set GZ_SIM_RESOURCE_PATH for newer versions
export GZ_SIM_RESOURCE_PATH="${WORKSPACE_DIR}/src/metalimbs_description/models:${WORKSPACE_DIR}/src/metalimbs_description:${GZ_SIM_RESOURCE_PATH}"

# Set ROS package path
export ROS_PACKAGE_PATH="${WORKSPACE_DIR}/src:${ROS_PACKAGE_PATH}"

echo "Environment variables set:"
echo "IGN_GAZEBO_RESOURCE_PATH=${IGN_GAZEBO_RESOURCE_PATH}"
echo "GZ_SIM_RESOURCE_PATH=${GZ_SIM_RESOURCE_PATH}"
echo "ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}"