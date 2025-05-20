import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node, PushRosNamespace
from launch import LaunchDescription, Action
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument, OpaqueFunction, GroupAction
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition

launch_args = [
    DeclareLaunchArgument('ns', description='Namespace for the agent.'),
    DeclareLaunchArgument('is_leader', description='True will run origin_pub together with leader mqtt params, False otherwise.'),
    DeclareLaunchArgument('mqtt_params_file', default_value='/home/smarc2user/colcon_ws/src/tuper/arduagent/config/mqtt_params_leader1.yaml',
                         description='Path to the MQTT parameters file.'),
]

def generate_launch_description():

    config = os.path.join(get_package_share_directory('arduagent'), 'config', 'rover.yaml')

    ns = LaunchConfiguration('ns')
    is_leader = LaunchConfiguration('is_leader')
    mqtt_params_file = LaunchConfiguration('mqtt_params_file')

    rover_node = Node(
        name='rover',
        executable='rover',
        package='arduagent',
        namespace=ns,
        output='screen',
        parameters=[config]
    )

    mqtt_client_leader = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            os.path.join(get_package_share_directory('mqtt_client'), 'launch', 'standalone.launch.ros2.xml')
        ), condition=IfCondition(PythonExpression([is_leader])),
        launch_arguments={'params_file': mqtt_params_file}.items()
    )

    mqtt_client_follower = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            os.path.join(get_package_share_directory('mqtt_client'), 'launch', 'standalone.launch.ros2.xml')
        ), condition=UnlessCondition(PythonExpression([is_leader])),
        launch_arguments={'params_file': mqtt_params_file}.items()
    )

    mqtt_with_ns = GroupAction([
        PushRosNamespace(ns),
        mqtt_client_leader,
        mqtt_client_follower,
    ])

    # Specifically for the leader to publish the origin GPS for all agents to calculate their local positions
    origin_pub = Node(
        name='origin_pub',
        executable='origin_pub',
        package='arduagent',
        namespace=ns,
        output='screen',
        parameters=[{
            'input_topic': 'core/gps',
            'output_topic': 'origin_gps'
        }], condition=IfCondition(PythonExpression([is_leader])),
    )

    return LaunchDescription([
        *launch_args,
        rover_node,
        origin_pub,
        TimerAction(period=2.0, actions=[mqtt_with_ns]),
    ])