#!/bin/bash

# Source the environment setup
source ./setup_gazebo_env.sh

# Source ROS2 setup
source /opt/ros/humble/setup.bash
source install/setup.bash

# Generate URDF from original xacro with absolute paths
echo "Generating URDF from original xacro..."
WORKSPACE_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Generate URDF directly from the original file - use the correct fixed version
xacro ${WORKSPACE_DIR}/src/metalimbs_description/robot/metalimbs2_fixed.urdf.xacro > /tmp/metalimbs2_original.urdf

# Replace package:// with file:// in the generated URDF
sed -i "s|package://metalimbs_description/meshes/metalimbs2|file://${WORKSPACE_DIR}/src/metalimbs_description/meshes/metalimbs2|g" /tmp/metalimbs2_original.urdf

echo "Generated URDF saved to /tmp/metalimbs2_original.urdf"

# Convert URDF to SDF
echo "Converting URDF to SDF..."
ign sdf -p /tmp/metalimbs2_original.urdf > /tmp/metalimbs2_colored.sdf 2>/dev/null

# Add material definitions to SDF
echo "Adding material definitions to SDF..."
python3 << 'EOF'
import re

# Read the SDF file
with open('/tmp/metalimbs2_colored.sdf', 'r') as f:
    sdf_content = f.read()

# Define colors for different parts
colors = {
    'base_mount': {'ambient': '0.3 0.3 0.3 1', 'diffuse': '0.4 0.4 0.4 1'},
    'shoulder': {'ambient': '0.2 0.2 0.4 1', 'diffuse': '0.3 0.3 0.6 1'},
    'upper_arm': {'ambient': '0.3 0.2 0.2 1', 'diffuse': '0.5 0.3 0.3 1'},
    'elbow': {'ambient': '0.2 0.3 0.2 1', 'diffuse': '0.3 0.5 0.3 1'},
    'forearm': {'ambient': '0.3 0.3 0.2 1', 'diffuse': '0.5 0.5 0.3 1'},
    'wrist': {'ambient': '0.3 0.2 0.3 1', 'diffuse': '0.5 0.3 0.5 1'},
    'handunit': {'ambient': '0.2 0.2 0.3 1', 'diffuse': '0.3 0.3 0.5 1'},
}

# Function to add material after mesh tag
def add_material_to_visual(visual_block, part_type):
    color = None
    for key in colors:
        if key in part_type.lower():
            color = colors[key]
            break
    
    if not color:
        # Default color
        color = {'ambient': '0.5 0.5 0.5 1', 'diffuse': '0.7 0.7 0.7 1'}
    
    material = f"""
        <material>
          <ambient>{color['ambient']}</ambient>
          <diffuse>{color['diffuse']}</diffuse>
          <specular>0.1 0.1 0.1 1</specular>
        </material>"""
    
    # Insert material after </geometry>
    return visual_block.replace('</geometry>', '</geometry>' + material)

# Process each visual block
visual_pattern = r'(<visual[^>]*>.*?</visual>)'
visual_blocks = re.findall(visual_pattern, sdf_content, re.DOTALL)

for visual_block in visual_blocks:
    # Extract the mesh filename to determine part type
    mesh_match = re.search(r'<uri>.*?/([^/]+)\.obj</uri>', visual_block)
    if mesh_match:
        part_name = mesh_match.group(1)
        new_visual_block = add_material_to_visual(visual_block, part_name)
        sdf_content = sdf_content.replace(visual_block, new_visual_block)

# Write the modified SDF
with open('/tmp/metalimbs2_colored.sdf', 'w') as f:
    f.write(sdf_content)

print("Material definitions added to SDF")
EOF

echo "Generated SDF with materials saved to /tmp/metalimbs2_colored.sdf"

# Create a world file that includes our robot
cat > /tmp/metalimbs_colored_world.sdf << EOF
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
      <uri>file:///tmp/metalimbs2_colored.sdf</uri>
      <name>metalimbs</name>
      <pose>0 0 0.5 0 0 0</pose>
    </include>
  </world>
</sdf>
EOF

echo "World file created at /tmp/metalimbs_colored_world.sdf"

# Launch Ignition Gazebo with warnings suppressed
echo "Launching Ignition Gazebo with colored Metalimbs robot..."
ign gazebo -v 0 /tmp/metalimbs_colored_world.sdf 2>&1 | grep -v "OBJLoader.cc" | grep -v "Missing material"