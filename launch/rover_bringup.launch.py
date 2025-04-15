import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node, PushRosNamespace
from launch import LaunchDescription, Action
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_context import LaunchContext

launch_args = [
    DeclareLaunchArgument('ns', description='Namespace for the agent. Should be agent0, agent1, agent2, etc.'),
    DeclareLaunchArgument('is_leader', description='True will run origin_pub together with leader mqtt params, False otherwise.'),
]

def launch_setup(context: LaunchContext) -> list[Action]:

    config = os.path.join(get_package_share_directory('arduagent'), 'config', 'rover.yaml')

    ns = LaunchConfiguration('ns')
    is_leader = LaunchConfiguration('is_leader')

    rover_node = Node(
        name='rover',
        executable='rover',
        package='arduagent',
        output='screen',
        parameters=[config]
    )

    mqtt_client_leader = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            os.path.join(get_package_share_directory('mqtt_client'), 'launch', 'standalone.launch.ros2.xml')
        ), condition=IfCondition(PythonExpression([is_leader])),
        launch_arguments={'params_file': os.path.join(get_package_share_directory('arduagent'), 'config', 'mqtt_params_leader.yaml')}.items()
    )

    mqtt_client_follower = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            os.path.join(get_package_share_directory('mqtt_client'), 'launch', 'standalone.launch.ros2.xml')
        ), condition=UnlessCondition(PythonExpression([is_leader])),
        launch_arguments={'params_file': os.path.join(get_package_share_directory('arduagent'), 'config', 'mqtt_params_follower.yaml')}.items()
    )

    # Specifically for the leader to publish the origin GPS for all agents to calculate their local positions
    origin_pub = Node(
        name='origin_pub',
        executable='origin_pub',
        package='arduagent',
        output='screen',
        parameters=[{
            'input_topic': 'core/gps',
            'output_topic': 'origin_gps'
        }], condition=IfCondition(PythonExpression([is_leader])),
    )

    return [
        PushRosNamespace(ns.perform(context)),
        rover_node,
        origin_pub,
        TimerAction(period=2.0, actions=[mqtt_client_leader, mqtt_client_follower]),
    ]

def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        *launch_args,
        OpaqueFunction(function=launch_setup)
    ])