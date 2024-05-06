from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    obstacle_distance = LaunchConfiguration('obstacle')
    rotation_degrees = LaunchConfiguration('degrees')
    final_approach = LaunchConfiguration('final_approach')

    return LaunchDescription([
        Node(
            package='attach_shelf',
            executable='approach_shelf_server',
            name='approach_shelf_server_node',
        ),
        Node(
            package='attach_shelf',
            executable='pre_approach_v2',
            name='pre_approach_v2_node',
            parameters=[
                {'obstacle': obstacle_distance},
                {'degrees': rotation_degrees},
                {'final_approach': final_approach}
            ]
        )
    ])