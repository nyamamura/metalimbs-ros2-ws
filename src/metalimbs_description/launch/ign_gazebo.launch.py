import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    # Package Directories
    pkg_metalimbs_description = get_package_share_directory('metalimbs_description')
    
    # Launch Configuration Variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Declare Launch Arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # Path to robot description
    robot_description_path = os.path.join(
        pkg_metalimbs_description,
        'robot',
        'metalimbs2.urdf.xacro'
    )
    
    # Robot description
    robot_description = {'robot_description': Command(['xacro ', robot_description_path])}
    
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
    
    # Joint State Publisher GUI (for manual control)
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Launch Description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(declare_use_sim_time_cmd)
    
    # Add nodes
    ld.add_action(start_gazebo)
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher_gui)
    
    return ld