from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='xrce_link_demo', executable='relay', name='relay', output='screen'),
        Node(package='xrce_link_demo', executable='processor', name='processor', output='screen'),
    ])
