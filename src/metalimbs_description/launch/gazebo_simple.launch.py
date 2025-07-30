import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node


def generate_launch_description():
    # Package Directories
    pkg_metalimbs_description = get_package_share_directory('metalimbs_description')
    pkg_ros_ign_gazebo = get_package_share_directory('ros_ign_gazebo')
    
    # Path to robot description
    robot_description_path = os.path.join(
        pkg_metalimbs_description,
        'robot',
        'metalimbs2.urdf.xacro'
    )
    
    # Robot description
    robot_description = {'robot_description': Command(['xacro ', robot_description_path])}
    
    # Ignition Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_ign_gazebo, 'launch', 'ign_gazebo.launch.py')),
        launch_arguments={'ign_args': '-r empty.sdf'}.items(),
    )
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )
    
    # Joint State Publisher GUI
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )
    
    # Spawn entity using ros_ign_gazebo
    spawn_entity = Node(
        package='ros_ign_gazebo',
        executable='create',
        output='screen',
        arguments=['-name', 'metalimbs',
                   '-topic', 'robot_description',
                   '-x', '0',
                   '-y', '0', 
                   '-z', '0.5']
    )
    
    # ROS-IGN Bridge for joint states
    bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
        output='screen'
    )
    
    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        joint_state_publisher_gui,
        spawn_entity,
        bridge
    ])