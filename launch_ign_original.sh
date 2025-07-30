#!/bin/bash

# Source the environment setup
source ./setup_gazebo_env.sh

# Source ROS2 setup
source /opt/ros/humble/setup.bash
source install/setup.bash

# Generate URDF from original xacro with absolute paths
echo "Generating URDF from original xacro..."
WORKSPACE_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Generate URDF directly from the original file
xacro ${WORKSPACE_DIR}/src/metalimbs_description/robot/metalimbs2_fixed.urdf.xacro > /tmp/metalimbs2_original.urdf

# Replace package:// with file:// in the generated URDF
sed -i "s|package://metalimbs_description/meshes/metalimbs2|file://${WORKSPACE_DIR}/src/metalimbs_description/meshes/metalimbs2|g" /tmp/metalimbs2_original.urdf

echo "Generated URDF saved to /tmp/metalimbs2_original.urdf"

# Convert URDF to SDF
echo "Converting URDF to SDF..."
ign sdf -p /tmp/metalimbs2_original.urdf > /tmp/metalimbs2_original.sdf

echo "Generated SDF saved to /tmp/metalimbs2_original.sdf"

# Create a world file that includes our robot
cat > /tmp/metalimbs_original_world.sdf << EOF
<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="default">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    
    <plugin
      filename="ignition-gazebo-physics-system"
      name="ignition::gazebo::systems::Physics">
    </plugin>
    <plugin
      filename="ignition-gazebo-user-commands-system"
      name="ignition::gazebo::systems::UserCommands">
    </plugin>
    <plugin
      filename="ignition-gazebo-scene-broadcaster-system"
      name="ignition::gazebo::systems::SceneBroadcaster">
    </plugin>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.8 0.8 0.8 1</specular>
          </material>
        </visual>
      </link>
    </model>

    <include>
      <uri>file:///tmp/metalimbs2_original.sdf</uri>
      <name>metalimbs</name>
      <pose>0 0 0.5 0 0 0</pose>
    </include>
  </world>
</sdf>
EOF

echo "World file created at /tmp/metalimbs_original_world.sdf"

# Launch Ignition Gazebo with the world file
echo "Launching Ignition Gazebo with original Metalimbs robot..."
# Suppress OBJ loader warnings by redirecting stderr
ign gazebo -v 4 /tmp/metalimbs_original_world.sdf 2>&1 | grep -v "Missing material for shape"