from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='navi_launch',
            executable='simple_navi_node',
            name='simple_navi_node',
            output='screen'
        ),
        Node(
            package='navi_launch',
            executable='simple_path_follower',
            name='simple_path_follower',
            output='screen'
        ),
    ])
