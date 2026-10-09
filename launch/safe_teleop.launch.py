"""Joystick teleoperation with a dead-man switch and a collision monitor.

joy_node -> teleop_twist_joy (moves only while the enable button is held, at most
0.2 m/s and 0.5 rad/s) -> collision_monitor (slows, then stops, motion towards
what the laser and sonars see) -> cmd_vel -> naoqi_driver.

Needs the joy, teleop_twist_joy, nav2_collision_monitor and nav2_lifecycle_manager
packages, which naoqi_driver does not depend on.
"""

import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = launch.substitutions.PathJoinSubstitution(
        [FindPackageShare('naoqi_driver'), 'config', 'safe_teleop'])
    namespace = launch.substitutions.LaunchConfiguration('namespace')
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'namespace', default_value='',
            description='Namespace of the driver, whose topics these nodes use'),
        launch.actions.DeclareLaunchArgument(
            'joy_dev', default_value='0', description='Joystick device number'),
        launch.actions.DeclareLaunchArgument(
            'joy_config',
            default_value=launch.substitutions.PathJoinSubstitution(
                [config, 'teleop_twist_joy.yaml']),
            description='teleop_twist_joy parameters: axes, enable button, speeds'),
        launch.actions.DeclareLaunchArgument(
            'collision_monitor_config',
            default_value=launch.substitutions.PathJoinSubstitution(
                [config, 'collision_monitor.yaml']),
            description='nav2_collision_monitor parameters'),
        launch_ros.actions.Node(
            package='joy', executable='joy_node', name='joy_node', namespace=namespace,
            parameters=[{
                'device_id': launch.substitutions.LaunchConfiguration('joy_dev'),
                'deadzone': 0.1,
                # Repeats the last state, so that a stick held still keeps the robot
                # moving instead of tripping the driver's cmd_vel timeout.
                'autorepeat_rate': 20.0,
            }]),
        launch_ros.actions.Node(
            package='teleop_twist_joy', executable='teleop_node',
            name='teleop_twist_joy_node', namespace=namespace,
            parameters=[launch.substitutions.LaunchConfiguration('joy_config')],
            remappings=[('cmd_vel', 'cmd_vel_teleop')]),
        launch_ros.actions.Node(
            package='nav2_collision_monitor', executable='collision_monitor',
            name='collision_monitor', namespace=namespace,
            parameters=[launch.substitutions.LaunchConfiguration('collision_monitor_config')]),
        launch_ros.actions.Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_safe_teleop', namespace=namespace,
            parameters=[{'autostart': True, 'node_names': ['collision_monitor']}]),
    ])
