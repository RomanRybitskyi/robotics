from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Set the path to the robot description file
    urdf_file = os.path.join(
        get_package_share_directory('ground_drone_description'),
        'urdf',
        'ground_drone.urdf.xacro'
    )
    
    # Set the path to Gazebo's launch file
    gazebo_launch_file = os.path.join(
        get_package_share_directory('gazebo_ros'),
        'launch',
        'gazebo.launch.py'
    )
    
    # Declare Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Load robot state publisher to publish the URDF
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_file]),
            'use_sim_time': use_sim_time
        }]
    )
    
    # Start Gazebo with an empty world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file),
        launch_arguments={'verbose': 'true'}.items(),
    )

    # Node to spawn the robot in Gazebo
    spawn_robot_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_robot',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'bot'
        ]
    )

    # Launch teleop_twist_keyboard node (optional)
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        prefix='xterm -e'  # Opens in a new terminal for ease of control
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time if true'),
        gazebo,
        robot_state_publisher_node,
        spawn_robot_node,  # Add spawn robot node here
        teleop_node
    ])

