#!/bin/bash

echo "Searching for human/person models in Ignition Fuel..."
echo ""

# Search using ign fuel command line tool
echo "Method 1: Using ign fuel tool (if available):"
if command -v ign &> /dev/null; then
    echo "Searching for 'person' models:"
    ign fuel list -t model -o OpenRobotics | grep -i person
    echo ""
    echo "Searching for 'human' models:"
    ign fuel list -t model -o OpenRobotics | grep -i human
    echo ""
    echo "Searching for 'man' or 'woman' models:"
    ign fuel list -t model -o OpenRobotics | grep -E -i "(man|woman|male|female)"
else
    echo "ign fuel command not found"
fi

echo ""
echo "Method 2: Direct API search URLs (open in browser):"
echo "https://fuel.ignitionrobotics.org/1.0/models?q=person"
echo "https://fuel.ignitionrobotics.org/1.0/models?q=human"
echo "https://fuel.ignitionrobotics.org/1.0/models?q=people"
echo "https://fuel.ignitionrobotics.org/1.0/models?q=character"

echo ""
echo "Method 3: Common human model URIs that might work:"
echo "- https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/MaleVisitorOnPhone"
echo "- https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/VisitorFemaleShirt"  
echo "- https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/Visitor"
echo "- https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/Casual_female"
echo "- https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/Office_worker"

echo ""
echo "Creating a test world with different human models..."

cat > /tmp/test_human_models.sdf << 'EOF'
<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="test_humans">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    
    <plugin filename="ignition-gazebo-physics-system" name="ignition::gazebo::systems::Physics"></plugin>
    <plugin filename="ignition-gazebo-user-commands-system" name="ignition::gazebo::systems::UserCommands"></plugin>
    <plugin filename="ignition-gazebo-scene-broadcaster-system" name="ignition::gazebo::systems::SceneBroadcaster"></plugin>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
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
          </material>
        </visual>
      </link>
    </model>

    <!-- Try different human models -->
    <include>
      <uri>https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/MaleVisitorOnPhone</uri>
      <name>human1</name>
      <pose>0 0 0 0 0 0</pose>
    </include>

    <include>
      <uri>https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/VisitorFemaleShirt</uri>
      <name>human2</name>
      <pose>2 0 0 0 0 0</pose>
    </include>

  </world>
</sdf>
EOF

echo "Test world created at /tmp/test_human_models.sdf"
echo ""
echo "To test human models, run:"
echo "ign gazebo -v 4 /tmp/test_human_models.sdf"