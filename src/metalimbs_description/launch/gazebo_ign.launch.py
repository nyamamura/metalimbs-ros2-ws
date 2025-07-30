import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Package Directories
    pkg_metalimbs_description = get_package_share_directory('metalimbs_description')
    
    # Launch Configuration Variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = LaunchConfiguration('world')
    
    # Declare Launch Arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value='empty.sdf',
        description='World file name to load'
    )
    
    # Path to robot description
    robot_description_path = os.path.join(
        pkg_metalimbs_description,
        'robot',
        'metalimbs2.urdf.xacro'
    )
    
    # Robot description
    robot_description = {'robot_description': Command(['xacro ', robot_description_path])}
    
    # Start Ignition Gazebo
    start_gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', world, '-r'],
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
    
    # Joint State Publisher GUI
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Spawn robot using ROS2
    spawn_entity = Node(
        package='ros_ign_gazebo',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'metalimbs',
            '-x', '0',
            '-y', '0',
            '-z', '0.1',
        ],
        output='screen'
    )
    
    # Bridge between Ignition and ROS2
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
    
    # Add launch arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_world_cmd)
    
    # Add nodes
    ld.add_action(start_gazebo)
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher_gui)
    ld.add_action(spawn_entity)
    ld.add_action(bridge)
    
    return ld