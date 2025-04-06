import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():

    config = os.path.join(get_package_share_directory('arduagent'), 'config', 'rover.yaml')

    rover_node = Node(
        name='rover',
        executable='rover',
        package='arduagent',
        output='screen',
        parameters=[config]
    )

    return LaunchDescription([
        rover_node,
    ])