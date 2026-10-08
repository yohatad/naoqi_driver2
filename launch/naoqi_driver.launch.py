import launch
import launch_ros
import launch.actions
import launch.substitutions
import launch_ros.actions
import launch_ros.parameter_descriptions

def generate_launch_description():
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'nao_ip',
            default_value="127.0.0.1",
            description='IP address of the robot'),
        launch.actions.DeclareLaunchArgument(
            'nao_port',
            default_value="9559",
            description='Port to be used for the connection'),
        launch.actions.DeclareLaunchArgument(
            'user',
            default_value="nao",
            description='Username for the connection'),
        launch.actions.DeclareLaunchArgument(
            'password',
            default_value="no_password",
            description='Password for the connection'),
        launch.actions.DeclareLaunchArgument(
            'network_interface',
            default_value="eth0",
            description='Network interface to be used'),
        launch.actions.DeclareLaunchArgument(
            'qi_listen_url',
            default_value="tcp://0.0.0.0:0",
            description='Endpoint to listen for incoming NAOqi connections (for audio)'),
        launch.actions.DeclareLaunchArgument(
            'namespace',
            default_value="",
            description='Name of the namespace to be used'),
        # FALSE by default, matching naoqi_driver_node's own C++ default.
        # Publishing this edge while a LIO/SLAM also owns odom -> base_footprint
        # gives that frame two parents and splits the TF tree; the flag is
        # latched in JointStateConverter's constructor with no parameter
        # callback, so it cannot be corrected at runtime. Wheel odometry is
        # still available as a TOPIC (/pepper_odom) either way.
        launch.actions.DeclareLaunchArgument(
            'publish_wheel_odom_tf',
            default_value="false",
            description='Publish odom -> base_footprint from wheel odometry. '
                         'Leave FALSE when an external localization source '
                         '(e.g. FAST-LIO) owns this edge, or the TF tree splits.'),
        launch.actions.DeclareLaunchArgument(
            'cmd_vel_timeout',
            default_value="0.5",
            description='Seconds without /cmd_vel before a moving base is stopped. '
                        '0 disables the watchdog (the base then keeps the last velocity).'),
        launch.actions.DeclareLaunchArgument(
            'moveto_max_distance',
            default_value="3.0",
            description='moveto goals farther than this (m) are rejected; 0 disables the check.'),
        launch.actions.DeclareLaunchArgument(
            'max_linear_speed',
            default_value="0.0",
            description='Cap on the /cmd_vel linear speed (m/s); 0 means no cap.'),
        launch.actions.DeclareLaunchArgument(
            'max_angular_speed',
            default_value="0.0",
            description='Cap on the /cmd_vel angular speed (rad/s); 0 means no cap.'),
        launch.actions.DeclareLaunchArgument(
            'listen_timeout',
            default_value="30.0",
            description='Seconds a listen goal waits for speech before it is aborted; 0 waits forever.'),
        launch.actions.DeclareLaunchArgument(
            'minidump_duration',
            default_value="0.0",
            description='Seconds of recent sensor data kept in memory for the '
                        '/naoqi_driver/minidump service; 0 turns it off.'),
        launch.actions.DeclareLaunchArgument(
            'respawn',
            default_value="true",
            description='Restart the driver when it exits on its own, e.g. after losing '
                        'the NAOqi connection (it then reconnects with backoff).'),
        launch_ros.actions.Node(
            package='naoqi_driver',
            executable='naoqi_driver_node',
            namespace=launch.substitutions.LaunchConfiguration('namespace'),
            parameters=[{
                'nao_ip': launch.substitutions.LaunchConfiguration('nao_ip'),
                'nao_port': launch.substitutions.LaunchConfiguration('nao_port'),
                'user': launch.substitutions.LaunchConfiguration('user'),
                'password': launch.substitutions.LaunchConfiguration('password'),
                'network_interface': launch.substitutions.LaunchConfiguration('network_interface'),
                'qi_listen_url': launch.substitutions.LaunchConfiguration('qi_listen_url'),
                'publish_wheel_odom_tf': launch.substitutions.LaunchConfiguration('publish_wheel_odom_tf'),
                'cmd_vel_timeout': launch_ros.parameter_descriptions.ParameterValue(
                    launch.substitutions.LaunchConfiguration('cmd_vel_timeout'), value_type=float),
                'moveto_max_distance': launch_ros.parameter_descriptions.ParameterValue(
                    launch.substitutions.LaunchConfiguration('moveto_max_distance'), value_type=float),
                'max_linear_speed': launch_ros.parameter_descriptions.ParameterValue(
                    launch.substitutions.LaunchConfiguration('max_linear_speed'), value_type=float),
                'max_angular_speed': launch_ros.parameter_descriptions.ParameterValue(
                    launch.substitutions.LaunchConfiguration('max_angular_speed'), value_type=float),
                'listen_timeout': launch_ros.parameter_descriptions.ParameterValue(
                    launch.substitutions.LaunchConfiguration('listen_timeout'), value_type=float),
                'minidump_duration': launch_ros.parameter_descriptions.ParameterValue(
                    launch.substitutions.LaunchConfiguration('minidump_duration'), value_type=float),
            }],
            output="screen",
            respawn=launch.substitutions.LaunchConfiguration('respawn'),
            respawn_delay=2.0),
    ])