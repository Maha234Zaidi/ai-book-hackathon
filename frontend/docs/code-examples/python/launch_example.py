from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Launch file for the Complex Robot Agent example."""
    
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        
        # Complex Robot Agent node
        Node(
            package='robot_examples',
            executable='complex_agent',
            name='complex_robot_agent',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'safety_distance': 0.5},
                {'linear_velocity': 0.3}
            ],
            remappings=[
                ('/cmd_vel', '/diff_drive_controller/cmd_vel_unstamped'),
                ('/scan', '/scan_raw')
            ],
            output='screen'
        ),
        
        # Example of multiple nodes
        Node(
            package='robot_examples',
            executable='sensor_processor',
            name='sensor_processor',
            parameters=[
                {'use_sim_time': use_sim_time}
            ],
            output='screen'
        )
    ])