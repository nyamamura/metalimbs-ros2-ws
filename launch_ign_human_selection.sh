#!/bin/bash

# Source the environment setup
source ./setup_gazebo_env.sh

# Source ROS2 setup
source /opt/ros/humble/setup.bash
source install/setup.bash

# Show available human models
echo "Available human models from Ignition Fuel:"
echo "1. Male Visitor (Phone) - 電話中の男性"
echo "2. Female Visitor - 女性の訪問者"
echo "3. Office Worker - オフィスワーカー"
echo "4. Walking person - 歩いている人"
echo "5. Standing person - 立っている人"
echo "6. Casual male - カジュアルな男性"
echo "7. Casual female - カジュアルな女性"
echo "8. Pedestrian - 歩行者"
echo "9. Rescue Randy - 救助訓練用マネキン"
echo "10. None - 人間モデルなし（ロボットのみ）"
echo ""
read -p "Select human model (1-10): " choice

# Generate URDF from original xacro with absolute paths
echo "Generating URDF from original xacro..."
WORKSPACE_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Generate URDF directly from the original file
xacro ${WORKSPACE_DIR}/src/metalimbs_description/robot/metalimbs2_fixed.urdf.xacro > /tmp/metalimbs2_human.urdf

# Replace package:// with file:// in the generated URDF
sed -i "s|package://metalimbs_description/meshes/metalimbs2|file://${WORKSPACE_DIR}/src/metalimbs_description/meshes/metalimbs2|g" /tmp/metalimbs2_human.urdf

echo "Generated URDF saved to /tmp/metalimbs2_human.urdf"

# Convert URDF to SDF
echo "Converting URDF to SDF..."
ign sdf -p /tmp/metalimbs2_human.urdf > /tmp/metalimbs2_human.sdf 2>/dev/null

# Add material definitions to SDF
echo "Adding material definitions to SDF..."
python3 << 'EOF'
import re

# Read the SDF file
with open('/tmp/metalimbs2_human.sdf', 'r') as f:
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
with open('/tmp/metalimbs2_human.sdf', 'w') as f:
    f.write(sdf_content)

print("Material definitions added to SDF")
EOF

# Set human model based on choice
case $choice in
  1)
    HUMAN_MODEL='<include>
      <uri>https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/MaleVisitorOnPhone</uri>
      <name>human</name>
      <pose>0 0 0 0 0 0</pose>
      <static>true</static>
    </include>'
    ROBOT_POSE="0 -0.2 1.4 0 0 3.14159"
    ;;
  2)
    HUMAN_MODEL='<include>
      <uri>https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/VisitorFemaleShirt</uri>
      <name>human</name>
      <pose>0 0 0 0 0 0</pose>
      <static>true</static>
    </include>'
    ROBOT_POSE="0 -0.2 1.4 0 0 3.14159"
    ;;
  3)
    HUMAN_MODEL='<include>
      <uri>https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/Office Worker</uri>
      <name>human</name>
      <pose>0 0 0 0 0 0</pose>
      <static>true</static>
    </include>'
    ROBOT_POSE="0 -0.2 1.4 0 0 3.14159"
    ;;
  4)
    HUMAN_MODEL='<include>
      <uri>https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/Walking person</uri>
      <name>human</name>
      <pose>0 0 0 0 0 0</pose>
    </include>'
    ROBOT_POSE="0 -0.2 1.4 0 0 3.14159"
    ;;
  5)
    HUMAN_MODEL='<include>
      <uri>https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/Standing person</uri>
      <name>human</name>
      <pose>0 0 0 0 0 0</pose>
    </include>'
    ROBOT_POSE="0 -0.2 1.4 0 0 3.14159"
    ;;
  6)
    HUMAN_MODEL='<include>
      <uri>https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/Casual male</uri>
      <name>human</name>
      <pose>0 0 0 0 0 0</pose>
    </include>'
    ROBOT_POSE="0 -0.2 1.4 0 0 3.14159"
    ;;
  7)
    HUMAN_MODEL='<include>
      <uri>https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/Casual female</uri>
      <name>human</name>
      <pose>0 0 0 0 0 0</pose>
    </include>'
    ROBOT_POSE="0 -0.2 1.3 0 0 3.14159"
    ;;
  8)
    HUMAN_MODEL='<include>
      <uri>https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/Pedestrian</uri>
      <name>human</name>
      <pose>0 0 0 0 0 0</pose>
      <static>true</static>
    </include>'
    ROBOT_POSE="0 -0.2 1.4 0 0 3.14159"
    ;;
  9)
    HUMAN_MODEL='<include>
      <uri>https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/Rescue Randy</uri>
      <name>human</name>
      <pose>0 0 0 0 0 0</pose>
      <static>true</static>
    </include>'
    ROBOT_POSE="0 -0.2 1.4 0 0 3.14159"
    ;;
  10)
    HUMAN_MODEL=""
    ROBOT_POSE="0 0 0.5 0 0 3.14159"
    ;;
  *)
    echo "Invalid choice. Using default (no human model)."
    HUMAN_MODEL=""
    ROBOT_POSE="0 0 0.5 0 0 3.14159"
    ;;
esac

# Create world file
cat > /tmp/metalimbs_human_world.sdf << EOF
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

    ${HUMAN_MODEL}

    <!-- Mount the robot -->
    <include>
      <uri>file:///tmp/metalimbs2_human.sdf</uri>
      <name>metalimbs</name>
      <pose>${ROBOT_POSE}</pose>
    </include>
  </world>
</sdf>
EOF

echo "World file created at /tmp/metalimbs_human_world.sdf"

# Launch Ignition Gazebo
echo "Launching Ignition Gazebo..."
ign gazebo -v 0 /tmp/metalimbs_human_world.sdf 2>&1 | grep -v "OBJLoader.cc" | grep -v "Missing material"