#!/usr/bin/env python3
"""
Live checks for naoqi_driver against a real Pepper. Run while the driver is up.

Read-only by default. Anything that moves, lights up or speaks needs a flag:
  --leds         run the LED action (exercises goal accept / cancel)
  --speech       run the speech action (the robot talks)
  --motion       move the head and drive the base a few centimetres (asks first)
  --interactive  ask you to touch bumpers / hands / head

Every exit path, including Ctrl+C, sends a zero velocity and re-centres the head.
Exit code: 0 if nothing FAILed, 1 otherwise.

Example:
  python3 test/robot/robot_checks.py --leds --speech --motion --interactive
"""

import argparse
import math
import sys
import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
import rclpy.time

from action_msgs.msg import GoalStatus
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped, Twist, Vector3
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import CameraInfo, Image, Imu, JointState, LaserScan, Range
from std_msgs.msg import ColorRGBA
import tf2_ros

from naoqi_bridge_msgs.action import RunLed, SpeechWithFeedback
from naoqi_bridge_msgs.msg import (AudioBuffer, Bumper, HandTouch, HeadTouch,
                                   JointAnglesTrajectory, JointAnglesWithSpeed)
from naoqi_bridge_msgs.srv import GetRobotInfo, GetString

PASS, FAIL, WARN, SKIP = "PASS", "FAIL", "WARN", "SKIP"

# (relative topic, type, minimum rate in Hz, required). Topics that are not
# advertised are skipped (disabled in boot_config); advertised but too slow is a
# FAIL when required, a WARN otherwise (laser/sonar idle while the robot rests).
STREAMS = [
    ("/joint_states", JointState, 20.0, True),
    ("pepper_odom", Odometry, 20.0, True),
    ("imu/base", Imu, 40.0, True),
    ("imu/torso", Imu, 40.0, True),
    ("camera/front/image_raw", Image, 5.0, True),
    ("camera/bottom/image_raw", Image, 5.0, True),
    ("camera/depth/image_raw", Image, 3.0, True),
    ("audio", AudioBuffer, 3.0, True),
    ("/diagnostics", DiagnosticArray, 0.5, True),
    ("laser", LaserScan, 2.0, False),
    ("sonar/front", Range, 2.0, False),
    ("sonar/back", Range, 2.0, False),
]

# Distortion coefficients from converters/camera_info_definitions.hpp, by width.
CAMERA_D = {
    "front": {
        640: [-0.0545211535376379, 0.0691973423510287, -0.00241094929163055, -0.00112245009306511, 0.0],
        320: [-0.0870160932911717, 0.128210165050533, 0.003379500659424, -0.00106205540818586, 0.0],
        160: [-0.0843564504845967, 0.125733083790192, 0.00275901756247071, -0.00138645823460527, 0.0],
    },
    "bottom": {
        640: [-0.0648763971625288, 0.0612520196884308, 0.0038281538281731, -0.00551104078371959, 0.0],
        320: [-0.0481869853715082, 0.0201858398559121, 0.0030362056699177, -0.00172241952442813, 0.0],
        160: [-0.0688388724945936, 0.0697453843669642, 0.00309518737071049, -0.00570486993696543, 0.0],
    },
}


class RobotChecks(Node):

    def __init__(self, args):
        super().__init__("naoqi_driver_robot_checks")
        self.args = args
        self.results = []
        self.lock = threading.Lock()
        self.joints = {}
        self.odom_xy = None
        self.joint_stamps = []

        self.create_subscription(JointState, "/joint_states", self._on_joints, qos_profile_sensor_data)
        self.create_subscription(Odometry, self.name_of("pepper_odom"), self._on_odom, qos_profile_sensor_data)
        self.cmd_vel = self.create_publisher(Twist, "/cmd_vel", 10)
        self.joint_angles = self.create_publisher(JointAnglesWithSpeed, "/joint_angles", 10)
        self.joint_traj = self.create_publisher(JointAnglesTrajectory, "/joint_angles_trajectory", 10)
        self.moveto = self.create_publisher(PoseStamped, "/move_base_simple/goal", 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    # ------------------------------------------------------------ helpers

    def name_of(self, topic):
        if topic.startswith("/") or not self.args.namespace:
            return topic if topic.startswith("/") else "/" + topic
        return "/" + self.args.namespace.strip("/") + "/" + topic

    def driver_name(self, suffix):
        """Node-relative names (actions, parameter services) follow the namespace."""
        ns = "/" + self.args.namespace.strip("/") if self.args.namespace else ""
        return f"{ns}/naoqi_driver/{suffix}"

    def report(self, status, name, detail=""):
        self.results.append((status, name, detail))
        print(f"[{status}] {name}" + (f": {detail}" if detail else ""), flush=True)

    def _on_joints(self, msg):
        now = time.monotonic()
        with self.lock:
            self.joints.update(dict(zip(msg.name, msg.position)))
            self.joint_stamps = [t for t in self.joint_stamps if now - t < 10.0] + [now]

    def _on_odom(self, msg):
        with self.lock:
            self.odom_xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def joint(self, name):
        with self.lock:
            return self.joints.get(name)

    def odom(self):
        with self.lock:
            return self.odom_xy

    @staticmethod
    def wait(future, timeout):
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        return future.result() if future.done() else None

    def wait_for(self, predicate, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return True
            time.sleep(0.05)
        return predicate()

    # ------------------------------------------------------------ read-only

    def check_streams(self):
        for topic, msg_type, min_hz, required in STREAMS:
            name = self.name_of(topic)
            count = [0]
            sub = self.create_subscription(
                msg_type, name, lambda _m, c=count: c.__setitem__(0, c[0] + 1), qos_profile_sensor_data)
            time.sleep(1.0)  # discovery, and the driver notices the subscriber
            if self.count_publishers(name) == 0:
                self.report(SKIP, f"stream {name}", "not advertised (disabled in boot_config?)")
                self.destroy_subscription(sub)
                continue
            count[0] = 0
            window = 3.0
            time.sleep(window)
            rate = count[0] / window
            self.destroy_subscription(sub)
            if rate >= min_hz:
                self.report(PASS, f"stream {name}", f"{rate:.1f} Hz (min {min_hz})")
            else:
                self.report(FAIL if required else WARN, f"stream {name}", f"{rate:.1f} Hz, expected >= {min_hz}")

    def check_camera_info(self):
        for camera, table in CAMERA_D.items():
            name = self.name_of(f"camera/{camera}/camera_info")
            got = []
            sub = self.create_subscription(CameraInfo, name, got.append, qos_profile_sensor_data)
            # The driver only publishes a camera while its image topic has a subscriber.
            image_sub = self.create_subscription(Image, self.name_of(f"camera/{camera}/image_raw"),
                                                 lambda _m: None, qos_profile_sensor_data)
            time.sleep(1.0)
            if self.count_publishers(name) == 0:
                self.report(SKIP, f"camera_info {camera}", "not advertised")
                self.destroy_subscription(sub)
                self.destroy_subscription(image_sub)
                continue
            self.wait_for(lambda: got, 5.0)
            self.destroy_subscription(sub)
            self.destroy_subscription(image_sub)
            if not got:
                self.report(FAIL, f"camera_info {camera}", "no message in 5 s")
                continue
            info = got[0]
            expected = table.get(info.width)
            if expected is None:
                self.report(WARN, f"camera_info {camera}", f"no reference for width {info.width}")
            elif len(info.d) == 5 and all(math.isclose(a, b, abs_tol=1e-12) for a, b in zip(info.d, expected)):
                self.report(PASS, f"camera_info {camera}", f"{info.width}x{info.height}, distortion matches")
            else:
                self.report(FAIL, f"camera_info {camera}", f"d={list(info.d)}, expected {expected}")

    def check_tf(self):
        ok = self.wait_for(lambda: self.tf_buffer.can_transform(
            "base_footprint", "base_link", rclpy.time.Time()), 5.0)
        self.report(PASS if ok else FAIL, "tf base_footprint -> base_link",
                    "" if ok else "not available within 5 s")

    def check_parameter(self):
        client = self.create_client(GetParameters, self.driver_name("get_parameters"))
        if not client.wait_for_service(timeout_sec=5.0):
            self.report(FAIL, "parameter cmd_vel_timeout", "parameter service not available")
            return None
        req = GetParameters.Request()
        req.names = ["cmd_vel_timeout"]
        res = self.wait(client.call_async(req), 5.0)
        if res is None or not res.values or res.values[0].type == ParameterType.PARAMETER_NOT_SET:
            self.report(WARN, "parameter cmd_vel_timeout",
                        "not declared: this build predates the /cmd_vel watchdog")
            return None
        value = res.values[0]
        timeout = value.double_value if value.type == ParameterType.PARAMETER_DOUBLE else float(value.integer_value)
        self.report(PASS, "parameter cmd_vel_timeout", f"{timeout} s")
        return timeout

    def check_services(self):
        client = self.create_client(GetString, "/naoqi_driver/get_language")  # absolute in the driver
        if client.wait_for_service(timeout_sec=5.0):
            res = self.wait(client.call_async(GetString.Request()), 10.0)
            ok = res is not None and res.data != ""
            self.report(PASS if ok else FAIL, "service get_language", res.data if ok else "no answer")
        else:
            self.report(FAIL, "service get_language", "not available")

        client = self.create_client(GetRobotInfo, "/naoqi_driver/get_robot_config")
        if client.wait_for_service(timeout_sec=5.0):
            res = self.wait(client.call_async(GetRobotInfo.Request()), 10.0)
            ok = res is not None and res.info.model != ""
            self.report(PASS if ok else FAIL, "service get_robot_config",
                        f"model={res.info.model}" if ok else "no answer")
        else:
            self.report(FAIL, "service get_robot_config", "not available")

    # ------------------------------------------------------------ actions

    def _send_goal(self, client, goal):
        handle = self.wait(client.send_goal_async(goal), 5.0)
        return handle

    def check_led_action(self):
        client = ActionClient(self, RunLed, self.driver_name("run_led"))
        if not client.wait_for_server(timeout_sec=5.0):
            self.report(FAIL, "action run_led", "server not available")
            return
        fade = RunLed.Goal(target="FaceLeds", mode=RunLed.Goal.MODE_RGB_FADE,
                           color=ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0), duration=4.0)
        first = self._send_goal(client, fade)
        second = self._send_goal(client, fade)
        if first is None or not first.accepted:
            self.report(FAIL, "action run_led accept", "first goal not accepted")
            return
        self.report(PASS if second is not None and not second.accepted else FAIL,
                    "action run_led rejects a concurrent goal",
                    "" if second is not None and not second.accepted else "second goal was accepted")
        time.sleep(0.5)
        first.cancel_goal_async()
        result = self.wait(first.get_result_async(), 8.0)
        ok = result is not None and result.status == GoalStatus.STATUS_CANCELED
        self.report(PASS if ok else FAIL, "action run_led cancel",
                    f"final status {result.status}" if result else "no result in 8 s")
        restore = RunLed.Goal(target="FaceLeds", mode=RunLed.Goal.MODE_RGB,
                              color=ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0), duration=0.0)
        h = self._send_goal(client, restore)
        if h is not None and h.accepted:
            self.wait(h.get_result_async(), 5.0)

    def check_speech_action(self):
        client = ActionClient(self, SpeechWithFeedback, self.driver_name("speech_with_feedback"))
        if not client.wait_for_server(timeout_sec=5.0):
            self.report(FAIL, "action speech", "server not available")
            return
        short = SpeechWithFeedback.Goal(say="Driver test.", body_language_mode="disabled")
        h = self._send_goal(client, short)
        result = self.wait(h.get_result_async(), 20.0) if h is not None and h.accepted else None
        ok = result is not None and result.status == GoalStatus.STATUS_SUCCEEDED and result.result.success
        self.report(PASS if ok else FAIL, "action speech says a sentence",
                    "" if ok else f"status {result.status if result else 'none'}")

        long = SpeechWithFeedback.Goal(
            say="This is a long sentence that should be interrupted before it finishes, "
                "so if you hear all of it the cancel did not work.",
            body_language_mode="disabled")
        first = self._send_goal(client, long)
        second = self._send_goal(client, short)
        if first is None or not first.accepted:
            self.report(FAIL, "action speech accept", "long goal not accepted")
            return
        self.report(PASS if second is not None and not second.accepted else FAIL,
                    "action speech rejects a concurrent goal",
                    "" if second is not None and not second.accepted else "second goal was accepted")
        time.sleep(1.0)
        first.cancel_goal_async()
        result = self.wait(first.get_result_async(), 10.0)
        ok = result is not None and result.status == GoalStatus.STATUS_CANCELED
        self.report(PASS if ok else FAIL, "action speech cancel",
                    f"final status {result.status}" if result else "no result in 10 s")

    # ------------------------------------------------------------ motion

    def driver_present(self):
        """Joint states flowing and someone listening on /cmd_vel: the driver is up."""
        return self.joint("HeadYaw") is not None and self.count_subscribers("/cmd_vel") > 0

    def stop_base(self):
        for _ in range(5):
            self.cmd_vel.publish(Twist())
            time.sleep(0.05)

    def head_to(self, yaw, speed=0.1):
        self.joint_angles.publish(JointAnglesWithSpeed(joint_names=["HeadYaw"], joint_angles=[yaw],
                                                       speed=speed, relative=0))

    def wait_until_head_settles(self, timeout=5.0):
        deadline = time.monotonic() + timeout
        previous = self.joint("HeadYaw")
        while time.monotonic() < deadline:
            time.sleep(0.3)
            current = self.joint("HeadYaw")
            if abs(current - previous) < 0.003:
                return
            previous = current

    def check_head(self):
        if self.joint("HeadYaw") is None:
            self.report(FAIL, "head", "no HeadYaw in /joint_states")
            return
        self.head_to(0.0)
        self.wait_for(lambda: abs(self.joint("HeadYaw")) < 0.05, 5.0)

        self.head_to(0.25)
        ok = self.wait_for(lambda: abs(self.joint("HeadYaw") - 0.25) < 0.02, 8.0)
        self.report(PASS if ok else FAIL, "head follows JointAnglesWithSpeed",
                    f"HeadYaw={self.joint('HeadYaw'):.3f}" + ("" if ok else " (is the robot awake?)"))
        self.wait_until_head_settles()

        def stays_still(name, publish):
            before = self.joint("HeadYaw")
            publish()
            time.sleep(2.5)
            moved = abs(self.joint("HeadYaw") - before)
            self.report(PASS if moved < 0.03 else FAIL, name, f"moved {moved:.3f} rad")

        stays_still("speed > 1 is rejected", lambda: self.joint_angles.publish(JointAnglesWithSpeed(
            joint_names=["HeadYaw"], joint_angles=[-0.25], speed=1.5, relative=0)))
        stays_still("NaN angle is rejected", lambda: self.joint_angles.publish(JointAnglesWithSpeed(
            joint_names=["HeadYaw"], joint_angles=[float("nan")], speed=0.1, relative=0)))
        stays_still("relative Bezier trajectory is rejected", lambda: self.joint_traj.publish(JointAnglesTrajectory(
            joint_names=["HeadYaw"], joint_angles=[0.3], times=[1.5], relative=1, use_bezier=True)))
        stays_still("non-increasing trajectory times are rejected", lambda: self.joint_traj.publish(
            JointAnglesTrajectory(joint_names=["HeadYaw"], joint_angles=[-0.2, 0.1], times=[1.0, 1.0],
                                  relative=0, use_bezier=False)))

        self.joint_traj.publish(JointAnglesTrajectory(joint_names=["HeadYaw"], joint_angles=[-0.2, 0.0],
                                                      times=[1.5, 3.0], relative=0, use_bezier=False))
        reached = self.wait_for(lambda: abs(self.joint("HeadYaw") + 0.2) < 0.06, 4.0)
        back = self.wait_for(lambda: abs(self.joint("HeadYaw")) < 0.05, 5.0)
        self.report(PASS if reached and back else FAIL, "valid trajectory runs",
                    f"reached -0.2: {reached}, back to 0: {back}")

    def drive(self, speed, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.cmd_vel.publish(Twist(linear=Vector3(x=speed)))
            time.sleep(0.1)

    def displacement_over(self, seconds):
        start = self.odom()
        time.sleep(seconds)
        end = self.odom()
        if start is None or end is None:
            return None
        return math.hypot(end[0] - start[0], end[1] - start[1])

    def check_base(self, timeout):
        if self.odom() is None:
            self.report(FAIL, "base", "no pepper_odom, cannot observe motion")
            return

        start = self.odom()
        self.drive(0.1, 1.5)
        time.sleep(0.3)
        total = math.hypot(self.odom()[0] - start[0], self.odom()[1] - start[1])
        if total < 0.03:
            self.stop_base()
            self.report(WARN, "base moves on /cmd_vel",
                        f"moved {total:.3f} m: robot did not drive (obstacle, safety reflex, charging flap?)."
                        " Skipping the timeout check.")
            return
        self.report(PASS, "base moves on /cmd_vel", f"{total:.3f} m")

        if timeout is None or timeout <= 0:
            self.stop_base()
            self.report(SKIP, "cmd_vel timeout stops the base", "watchdog disabled or not in this build")
        else:
            # Stop publishing without sending zero: the driver must stop the base itself.
            time.sleep(timeout + 0.7)
            drift = self.displacement_over(1.0)
            self.stop_base()
            ok = drift is not None and drift < 0.01
            self.report(PASS if ok else FAIL, "cmd_vel timeout stops the base",
                        f"moved {drift:.3f} m in the second after the timeout" if drift is not None
                        else "no odometry")

        self.drive(0.1, 1.0)
        self.stop_base()
        time.sleep(0.5)  # let the base decelerate before checking it is at rest
        drift = self.displacement_over(1.0)
        self.report(PASS if drift is not None and drift < 0.01 else FAIL, "zero /cmd_vel stops the base",
                    f"moved {drift:.3f} m after the zero command" if drift is not None else "no odometry")

        nan = Twist()
        nan.linear.x = float("nan")
        self.cmd_vel.publish(nan)
        drift = self.displacement_over(1.5)
        self.stop_base()
        self.report(PASS if drift is not None and drift < 0.01 else FAIL, "NaN /cmd_vel is ignored",
                    f"moved {drift:.3f} m" if drift is not None else "no odometry")

    def check_moveto_does_not_stall(self):
        if self.tf_buffer.can_transform("base_footprint", "odom", rclpy.time.Time()):
            self.report(SKIP, "moveto(odom) does not stall the driver",
                        "odom TF exists, so the goal would really move the robot")
            return
        goal = PoseStamped()
        goal.header.frame_id = "odom"
        goal.pose.orientation.w = 1.0
        sent = time.monotonic()
        self.moveto.publish(goal)
        time.sleep(3.0)
        with self.lock:
            stamps = [t for t in self.joint_stamps if t > sent]
        # A stall starts right at the goal, so measure from it and up to now as well.
        points = [sent] + stamps + [time.monotonic()]
        gap = max(b - a for a, b in zip(points, points[1:])) if stamps else None
        ok = gap is not None and gap < 0.5
        self.report(PASS if ok else FAIL, "moveto(odom) does not stall the driver",
                    f"largest /joint_states gap {gap:.2f} s" if gap is not None else "no joint states")

    # ------------------------------------------------------------ interactive

    def check_touch(self):
        prompts = [
            ("bumper", Bumper, "Press any BUMPER at the base"),
            ("hand_touch", HandTouch, "Touch the BACK OF A HAND"),
            ("head_touch", HeadTouch, "Touch the TOP OF THE HEAD"),
        ]
        for topic, msg_type, prompt in prompts:
            name = self.name_of(topic)
            got = []
            sub = self.create_subscription(msg_type, name, got.append, 10)
            time.sleep(1.0)
            if self.count_publishers(name) == 0:
                self.report(SKIP, f"touch {topic}", "not advertised (disabled in boot_config?)")
                self.destroy_subscription(sub)
                continue
            print(f"\n>>> {prompt} within 15 s ...", flush=True)
            ok = self.wait_for(lambda: got, 15.0)
            self.destroy_subscription(sub)
            self.report(PASS if ok else FAIL, f"touch {topic}", f"{len(got)} message(s)")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--namespace", default="", help="namespace the driver was launched in")
    parser.add_argument("--leds", action="store_true")
    parser.add_argument("--speech", action="store_true")
    parser.add_argument("--motion", action="store_true")
    parser.add_argument("--interactive", action="store_true")
    parser.add_argument("--yes", action="store_true", help="do not ask before moving the robot")
    args = parser.parse_args()

    if args.motion and not args.yes:
        print("--motion turns the head and drives the base about 30 cm forward in total.\n"
              "Clear 1 m in front of the robot and keep the emergency stop in reach.")
        if input("Type 'go' to continue: ").strip().lower() != "go":
            print("Motion checks cancelled.")
            args.motion = False

    # rclpy's own SIGINT handler would invalidate the context before the finally
    # block below gets to stop the base; let Ctrl+C surface as KeyboardInterrupt.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = RobotChecks(args)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()

    try:
        time.sleep(1.0)
        node.check_streams()
        node.check_camera_info()
        node.check_tf()
        node.check_services()
        timeout = node.check_parameter()
        if args.leds:
            node.check_led_action()
        if args.speech:
            node.check_speech_action()
        if args.motion and not node.driver_present():
            node.report(FAIL, "motion checks", "driver not detected (no /joint_states or no /cmd_vel "
                        "subscriber); not sending any motion command")
            args.motion = False
        if args.motion:
            node.check_head()
            node.check_base(timeout)
            node.check_moveto_does_not_stall()
        if args.interactive:
            node.check_touch()
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        if args.motion and node.driver_present():
            node.stop_base()
            node.head_to(0.0)
            time.sleep(0.5)

    counts = {s: sum(1 for r in node.results if r[0] == s) for s in (PASS, FAIL, WARN, SKIP)}
    print("\nSummary: " + ", ".join(f"{v} {k}" for k, v in counts.items()))
    for status, name, detail in node.results:
        if status in (FAIL, WARN):
            print(f"  {status} {name}: {detail}")

    executor.shutdown(timeout_sec=2.0)
    spinner.join(timeout=2.0)
    node.destroy_node()
    rclpy.shutdown()
    return 1 if counts[FAIL] else 0


if __name__ == "__main__":
    sys.exit(main())
