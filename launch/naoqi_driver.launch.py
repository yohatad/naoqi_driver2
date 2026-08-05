import launch
import launch_ros
import launch.actions
import launch.substitutions
import launch_ros.actions

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
            }],
            output="screen"),
    ])