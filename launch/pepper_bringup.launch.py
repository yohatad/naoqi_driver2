"""
Top-level bringup: start the naoqi_driver2 node and the gscam2 Pepper front
camera together.

The naoqi driver publishes the robot state / TF tree; gscam2 consumes the
robot's front-camera GStreamer stream and publishes it into that TF tree
(frame: CameraTop_optical_frame).

Remember to start the stream on the robot before launching, e.g.:
    ssh nao@172.29.111.240 '~/start_camera.sh'

Example:
    ros2 launch naoqi_driver pepper_bringup.launch.py nao_ip:=172.29.111.240
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    naoqi_launch = os.path.join(
        get_package_share_directory('naoqi_driver'),
        'launch', 'naoqi_driver.launch.py')
    gscam_launch = os.path.join(
        get_package_share_directory('gscam2'),
        'launch', 'pepper_camera_launch.py')

    return LaunchDescription([
        # Re-declare the naoqi args here so they can be set on the bringup command
        # line and forwarded to the included naoqi launch file.
        DeclareLaunchArgument('nao_ip', default_value='127.0.0.1',
                              description='IP address of the robot'),
        DeclareLaunchArgument('nao_port', default_value='9559',
                              description='Port to be used for the connection'),
        DeclareLaunchArgument('user', default_value='nao',
                              description='Username for the connection'),
        DeclareLaunchArgument('password', default_value='no_password',
                              description='Password for the connection'),
        DeclareLaunchArgument('network_interface', default_value='eth0',
                              description='Network interface to be used'),
        DeclareLaunchArgument('qi_listen_url', default_value='tcp://0.0.0.0:0',
                              description='Endpoint to listen for incoming NAOqi connections'),
        DeclareLaunchArgument('namespace', default_value='',
                              description='Name of the namespace to be used'),
        DeclareLaunchArgument('publish_wheel_odom_tf', default_value='true',
                              description='Publish odom -> base_footprint from wheel odometry.'),
        DeclareLaunchArgument('use_camera', default_value='true',
                              description='Start the gscam2 Pepper front camera alongside the driver.'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(naoqi_launch),
            launch_arguments={
                'nao_ip': LaunchConfiguration('nao_ip'),
                'nao_port': LaunchConfiguration('nao_port'),
                'user': LaunchConfiguration('user'),
                'password': LaunchConfiguration('password'),
                'network_interface': LaunchConfiguration('network_interface'),
                'qi_listen_url': LaunchConfiguration('qi_listen_url'),
                'namespace': LaunchConfiguration('namespace'),
                'publish_wheel_odom_tf': LaunchConfiguration('publish_wheel_odom_tf'),
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gscam_launch),
            condition=IfCondition(LaunchConfiguration('use_camera')),
        ),
    ])
