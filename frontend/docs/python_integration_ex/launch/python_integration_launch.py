from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Launch file for Python integration examples."""
    
    # Declare launch arguments
    robot_name = LaunchConfiguration('robot_name')
    max_velocity = LaunchConfiguration('max_velocity')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument(
            'robot_name',
            default_value='turtlebot',
            description='Name of the robot'
        ),
        DeclareLaunchArgument(
            'max_velocity',
            default_value='0.5',
            description='Maximum velocity for the robot'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        
        # Parameter node with custom parameters
        Node(
            package='python_integration_ex',
            executable='param_node',
            name='parameter_example_node',
            parameters=[
                {'robot_name': robot_name},
                {'max_velocity': max_velocity},
                {'safety_distance': 0.5},
                {'publish_frequency': 2.0}
            ],
            output='screen'
        ),
        
        # Timer node
        Node(
            package='python_integration_ex',
            executable='timer_node',
            name='timer_example_node',
            output='screen'
        )
    ])