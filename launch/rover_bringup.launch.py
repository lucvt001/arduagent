import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import FrontendLaunchDescriptionSource

def generate_launch_description():

    param_file_name = 'mqtt_params_leader.yaml'
    config = os.path.join(get_package_share_directory('arduagent'), 'config', 'rover.yaml')

    rover_node = Node(
        name='rover',
        executable='rover',
        package='arduagent',
        output='screen',
        parameters=[config]
    )

    mqtt_client = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            os.path.join(get_package_share_directory('mqtt_client'), 'launch', 'standalone.launch.ros2.xml')
        ), launch_arguments={'params_file': os.path.join(get_package_share_directory('arduagent'), 'config', param_file_name)}.items()
    )

    return LaunchDescription([
        rover_node,
        TimerAction(period=2.0, actions=[mqtt_client]),
    ])