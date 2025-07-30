import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    # Package Directories
    pkg_metalimbs_description = get_package_share_directory('metalimbs_description')
    
    # Launch Configuration Variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Path to robot description
    robot_description_path = os.path.join(
        pkg_metalimbs_description,
        'robot',
        'metalimbs2.urdf.xacro'
    )
    
    # Process xacro file
    doc = xacro.process_file(robot_description_path)
    robot_description = {'robot_description': doc.toxml()}
    
    # Start Ignition Gazebo with empty world
    start_gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-v', '4', '-r', 'empty.sdf'],
        output='screen'
    )
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description,
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # Spawn robot - write URDF to file first
    urdf_file_path = '/tmp/metalimbs2.urdf'
    with open(urdf_file_path, 'w') as f:
        f.write(doc.toxml())
    
    # Spawn robot using ign service
    spawn_entity = ExecuteProcess(
        cmd=[
            'ign', 'service',
            '-s', '/world/empty/create',
            '--reqtype', 'ignition.msgs.EntityFactory',
            '--reptype', 'ignition.msgs.Boolean',
            '--timeout', '5000',
            '--req',
            f'sdf_filename: "{urdf_file_path}", name: "metalimbs", pose: {{position: {{x: 0, y: 0, z: 0.1}}}}'
        ],
        output='screen'
    )
    
    # Joint State Publisher GUI
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # ROS-IGN Bridge
    bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model'
        ],
        output='screen'
    )
    
    # Launch Description
    ld = LaunchDescription()
    
    # Add nodes
    ld.add_action(start_gazebo)
    ld.add_action(robot_state_publisher)
    ld.add_action(spawn_entity)
    ld.add_action(joint_state_publisher_gui)
    ld.add_action(bridge)
    
    return ld