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
xacro ${WORKSPACE_DIR}/src/metalimbs_description/robot/metalimbs2_fixed.urdf.xacro > /tmp/metalimbs2_standing.urdf

# Replace package:// with file:// in the generated URDF
sed -i "s|package://metalimbs_description/meshes/metalimbs2|file://${WORKSPACE_DIR}/src/metalimbs_description/meshes/metalimbs2|g" /tmp/metalimbs2_standing.urdf

echo "Generated URDF saved to /tmp/metalimbs2_standing.urdf"

# Convert URDF to SDF
echo "Converting URDF to SDF..."
ign sdf -p /tmp/metalimbs2_standing.urdf > /tmp/metalimbs2_standing.sdf 2>/dev/null

# Add material definitions to SDF
echo "Adding material definitions to SDF..."
python3 << 'EOF'
import re

# Read the SDF file
with open('/tmp/metalimbs2_standing.sdf', 'r') as f:
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
with open('/tmp/metalimbs2_standing.sdf', 'w') as f:
    f.write(sdf_content)

print("Material definitions added to SDF")
EOF

echo "Generated SDF with materials saved to /tmp/metalimbs2_standing.sdf"

# Create world file with Standing person
cat > /tmp/metalimbs_standing_world.sdf << EOF
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

    <!-- User camera plugin for default view -->
    <plugin filename="ignition-gazebo-user-commands-system"
            name="ignition::gazebo::systems::UserCommands">
    </plugin>
    
    <!-- Scene plugin configuration -->
    <plugin filename="ignition-gazebo-scene-broadcaster-system"
            name="ignition::gazebo::systems::SceneBroadcaster">
      <scene>
        <ambient>0.4 0.4 0.4 1</ambient>
        <background>0.7 0.7 0.7 1</background>
        <grid>true</grid>
        <origin_visual>true</origin_visual>
      </scene>
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

    <!-- Standing person model -->
    <include>
      <uri>https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/Standing person</uri>
      <name>human</name>
      <pose>0 0 0 0 0 0</pose>
      <static>true</static>
    </include>

    <!-- MetaLimbs robot mounted on the back -->
    <include>
      <uri>file:///tmp/metalimbs2_standing.sdf</uri>
      <name>metalimbs</name>
      <!-- Position: behind the person, at shoulder height, rotated as specified -->
      <pose>0.0 0.0 1.5 3.1416 0.0 -1.5708</pose>
    </include>
  </world>
</sdf>
EOF

echo "World file created at /tmp/metalimbs_standing_world.sdf"

# Create GUI configuration with camera settings
cat > /tmp/gui_config.config << EOF
<?xml version="1.0"?>

<window>
  <width>1000</width>
  <height>845</height>
  <menus>
    <drawer default="false">
    </drawer>
  </menus>
  <plugins>
    <plugin filename="MinimalScene" name="3D View">
      <engine>ogre2</engine>
      <scene>scene</scene>
      <ambient_light>0.4 0.4 0.4</ambient_light>
      <background_color>0.8 0.8 0.8</background_color>
      <camera_pose>3 0 1.5 0 0.2 3.14159</camera_pose>
      <camera_clip>
        <near>0.1</near>
        <far>1000</far>
      </camera_clip>
    </plugin>
    <plugin filename="EntityTree" name="Entity tree">
    </plugin>
    <plugin filename="GzSceneManager" name="Scene Manager">
      <anchors target="3D View">
        <line own="right" target="right"/>
        <line own="top" target="top"/>
      </anchors>
    </plugin>
    <plugin filename="InteractiveViewControl" name="Interactive view control">
      <anchors target="3D View">
        <line own="right" target="right"/>
        <line own="top" target="top"/>
      </anchors>
    </plugin>
    <plugin filename="CameraTracking" name="Camera Tracking">
      <anchors target="3D View">
        <line own="right" target="right"/>
        <line own="top" target="top"/>
      </anchors>
    </plugin>
    <plugin filename="WorldControl" name="World control">
      <play_pause>true</play_pause>
      <step>true</step>
      <start_paused>true</start_paused>
    </plugin>
    <plugin filename="WorldStats" name="World stats">
      <sim_time>true</sim_time>
      <real_time>true</real_time>
      <real_time_factor>true</real_time_factor>
    </plugin>
    <plugin filename="ComponentInspector" name="Component inspector">
    </plugin>
  </plugins>
</window>
EOF

echo "GUI configuration created at /tmp/gui_config.config"

# Launch Ignition Gazebo with GUI config
echo "Launching Ignition Gazebo with Standing person and MetaLimbs robot..."
echo "Robot is positioned on the person's back at shoulder height"
echo "Camera positioned at front view"
ign gazebo -v 0 --gui-config /tmp/gui_config.config /tmp/metalimbs_standing_world.sdf 2>&1 | grep -v "OBJLoader.cc" | grep -v "Missing material" | grep -v "ODE Message"