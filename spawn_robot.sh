#!/bin/bash

# Generate URDF from xacro
xacro src/metalimbs_description/robot/metalimbs2.urdf.xacro > /tmp/metalimbs2.urdf

# Convert to SDF
ign sdf -p /tmp/metalimbs2.urdf > /tmp/metalimbs2.sdf

# Spawn robot in Ignition Gazebo
ign service -s /world/empty/create \
  --reqtype ignition.msgs.EntityFactory \
  --reptype ignition.msgs.Boolean \
  --timeout 5000 \
  --req 'sdf_filename: "/tmp/metalimbs2.sdf", name: "metalimbs", pose: {position: {x: 0, y: 0, z: 0.5}}'