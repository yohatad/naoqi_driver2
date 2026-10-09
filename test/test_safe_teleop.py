"""launch/safe_teleop.launch.py with a fake joystick, laser, sonars and TF: what reaches
cmd_vel. Skipped unless the optional joy, teleop_twist_joy and nav2 packages are installed."""

import math
import os
import random
import signal
import subprocess
import time

from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import PackageNotFoundError
import pytest

SOURCE = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
N_RAYS = 61
FOV = 2.0944


def installed(*packages):
    try:
        for package in packages:
            get_package_share_directory(package)
    except PackageNotFoundError:
        return False
    return True


pytestmark = pytest.mark.skipif(
    not installed('joy', 'teleop_twist_joy', 'nav2_collision_monitor', 'nav2_lifecycle_manager'),
    reason='needs the optional safe_teleop packages')


@pytest.fixture(scope='module')
def domain():
    """The launch file running on a ROS domain of its own; this process joins it."""
    domain_id = str(random.randint(100, 200))
    os.environ['ROS_DOMAIN_ID'] = domain_id
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    config = os.path.join(SOURCE, 'config', 'safe_teleop')
    launch = subprocess.Popen(
        ['ros2', 'launch', os.path.join(SOURCE, 'launch', 'safe_teleop.launch.py'),
         'joy_config:=' + os.path.join(config, 'teleop_twist_joy.yaml'),
         'collision_monitor_config:=' + os.path.join(config, 'collision_monitor.yaml')],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
    yield
    for sig in (signal.SIGINT, signal.SIGKILL):  # the nodes too, not only ros2 launch
        try:
            os.killpg(launch.pid, sig)
            launch.wait(10)
            break
        except (ProcessLookupError, subprocess.TimeoutExpired):
            pass


@pytest.fixture(scope='module')
def pepper(domain):
    import rclpy
    rclpy.init()
    node = FakePepper()
    node.run(8.0)  # discovery, and the lifecycle manager activating the monitor
    yield node
    node.node.destroy_node()
    rclpy.shutdown()


class FakePepper(object):
    """Publishes joystick, laser, sonars and TF at 20 Hz; records cmd_vel."""

    def __init__(self):
        from geometry_msgs.msg import Twist
        from rclpy.node import Node
        from sensor_msgs.msg import Joy, LaserScan, Range
        from tf2_ros import StaticTransformBroadcaster
        self.msgs = (Joy, LaserScan, Range)
        self.node = Node('fake_pepper')
        self.joy = self.node.create_publisher(Joy, 'joy', 10)
        self.laser = self.node.create_publisher(LaserScan, 'laser', 10)
        self.sonars = [
            (self.node.create_publisher(Range, 'sonar/front', 10), 'SonarFront_frame'),
            (self.node.create_publisher(Range, 'sonar/back', 10), 'SonarBack_frame')]
        self.cmd_vel = []
        self.node.create_subscription(
            Twist, 'cmd_vel', lambda m: self.cmd_vel.append((time.monotonic(), m.linear.x)), 10)
        self.tf = StaticTransformBroadcaster(self.node)
        self.tf.sendTransform([
            self.transform('SonarFront_frame', 0.2, 0.0),
            self.transform('SonarBack_frame', -0.2, math.pi)])
        self.enable = False
        self.stick = 0.0
        self.obstacle_ahead = None
        self.node.create_timer(0.05, self.publish)

    def transform(self, child, x, yaw):
        from geometry_msgs.msg import TransformStamped
        t = TransformStamped()
        t.header.stamp = self.node.get_clock().now().to_msg()
        t.header.frame_id = 'base_footprint'
        t.child_frame_id = child
        t.transform.translation.x = x
        t.transform.rotation.z = math.sin(yaw / 2)
        t.transform.rotation.w = math.cos(yaw / 2)
        return t

    def publish(self):
        Joy, LaserScan, Range = self.msgs
        now = self.node.get_clock().now().to_msg()
        joy = Joy()
        joy.header.stamp = now
        joy.axes = [0.0] * 8
        joy.axes[1] = self.stick
        joy.buttons = [0] * 11
        joy.buttons[4] = int(self.enable)
        self.joy.publish(joy)

        scan = LaserScan()
        scan.header.stamp = now
        scan.header.frame_id = 'base_footprint'
        scan.angle_min, scan.angle_max = -FOV, FOV
        scan.angle_increment = 2 * FOV / N_RAYS
        scan.range_min, scan.range_max = 0.1, 3.0
        scan.ranges = [math.inf] * N_RAYS
        if self.obstacle_ahead is not None:
            for i in range(N_RAYS // 2 - 2, N_RAYS // 2 + 3):
                scan.ranges[i] = self.obstacle_ahead
        self.laser.publish(scan)

        for publisher, frame in self.sonars:
            sonar = Range()
            sonar.header.stamp = now
            sonar.header.frame_id = frame
            sonar.radiation_type = Range.ULTRASOUND
            sonar.field_of_view = 0.5236
            sonar.min_range, sonar.max_range = 0.25, 2.55
            sonar.range = 5.0  # nothing in range
            publisher.publish(sonar)

    def run(self, seconds):
        import rclpy
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def drive(self, seconds=2.0, **state):
        """Applies state for seconds; returns the cmd_vel x values received meanwhile."""
        for key, value in state.items():
            setattr(self, key, value)
        start = time.monotonic()
        self.run(seconds)
        return [(t - start, x) for t, x in self.cmd_vel if t > start]


def settled(received):
    """The x values of the second half of a drive() window."""
    return [x for t, x in received if t > 1.0]


def test_nothing_moves_without_the_enable_button(pepper):
    assert settled(pepper.drive(stick=1.0, enable=False, obstacle_ahead=None)) == []


def test_speed_is_capped(pepper):
    received = settled(pepper.drive(stick=1.0, enable=True, obstacle_ahead=None))
    assert received and all(x == pytest.approx(0.2, abs=1e-3) for x in received)


def test_slows_down_near_an_obstacle(pepper):
    received = settled(pepper.drive(stick=1.0, enable=True, obstacle_ahead=0.55))
    assert received and all(x == pytest.approx(0.1, abs=1e-3) for x in received)


def test_almost_stops_at_the_footprint(pepper):
    received = settled(pepper.drive(stick=1.0, enable=True, obstacle_ahead=0.37))
    assert received and all(0.0 <= x <= 0.03 for x in received)


def test_moving_away_stays_allowed(pepper):
    received = settled(pepper.drive(stick=-1.0, enable=True, obstacle_ahead=0.37))
    assert received and all(x == pytest.approx(-0.1, abs=1e-3) for x in received)


def test_releasing_the_button_stops_at_once(pepper):
    pepper.drive(stick=1.0, enable=True, obstacle_ahead=None)
    received = pepper.drive(enable=False)
    zeros = [t for t, x in received if x == 0.0]
    assert zeros and zeros[0] < 0.3, received
    assert all(x == 0.0 for t, x in received if t > zeros[0])
