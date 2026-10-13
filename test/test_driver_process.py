"""naoqi_driver_node as a process against fake_naoqi_server: Ctrl+C stops the robot
and the process exits cleanly, and the recording services write sensor data into a
bag. Unit tests cannot see main()'s start-up and exit, or the converter loop."""

import os
import random
import signal
import subprocess
import threading
import time

import pytest

SERVER = os.environ.get('FAKE_NAOQI_SERVER')
DRIVER = os.environ.get('DRIVER_NODE')

pytestmark = pytest.mark.skipif(not SERVER or not DRIVER, reason='run through colcon test')

# All converters off: the fake serves only what the rest of the driver uses.
CONVERTERS = [
    'info', 'audio', 'logs', 'diag', 'imu_torso', 'imu_base', 'front_camera',
    'bottom_camera', 'depth_camera', 'stereo_camera', 'ir_camera', 'joint_states',
    'laser', 'sonar', 'odom', 'bumper', 'touch_hand', 'touch_head',
]


class Output(object):
    """The lines a process prints, collected on a thread."""

    def __init__(self, stream):
        self.lines = []
        self._lock = threading.Lock()
        thread = threading.Thread(target=self._read, args=(stream,))
        thread.daemon = True
        thread.start()

    def _read(self, stream):
        for line in iter(stream.readline, ''):
            with self._lock:
                self.lines.append(line.rstrip('\n'))

    def count(self, text):
        with self._lock:
            return sum(text in line for line in self.lines)

    def wait_for(self, text, timeout):
        end = time.monotonic() + timeout
        while self.count(text) == 0 and time.monotonic() < end:
            time.sleep(0.05)
        return self.count(text) > 0

    def __str__(self):
        with self._lock:
            return '\n'.join(self.lines)


@pytest.fixture
def naoqi():
    server = subprocess.Popen([SERVER], stdout=subprocess.PIPE, text=True)
    output = Output(server.stdout)
    assert output.wait_for('listening on', 10), str(output)
    url = next(line for line in output.lines if 'listening on' in line).split()[-1]
    yield url, output
    server.send_signal(signal.SIGINT)
    server.wait(10)


def start_driver(url, folder, enabled=()):
    """The driver with every converter off but those in enabled, on a domain of its own."""
    host, port = url.replace('tcp://', '').split(':')
    params = folder / 'params.yaml'
    # The fake has only some of NAOqi's services: do not wait for the others.
    params.write_text('/**:\n  ros__parameters:\n    naoqi_startup_timeout: 0.0\n'
                      '    converters:\n' + ''.join(
        '      {}:\n        enabled: {}\n'.format(name, 'true' if name in enabled else 'false')
        for name in CONVERTERS))
    os.environ['ROS_DOMAIN_ID'] = str(random.randint(100, 200))  # also for rclpy below
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    env = dict(os.environ)
    return subprocess.Popen(
        [DRIVER, '--ros-args', '-p', 'nao_ip:=' + host, '-p', 'nao_port:=' + port,
         '--params-file', str(params)],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=env, cwd=str(folder))


def test_ctrl_c_stops_the_robot_and_exits_cleanly(naoqi, tmp_path):
    url, calls = naoqi
    driver = start_driver(url, tmp_path)
    log = Output(driver.stdout)
    try:
        assert log.wait_for('naoqi_driver initialized', 60), str(log)
        assert calls.wait_for('insertData(NaoqiDriver/Heartbeat)', 5), str(calls)
        driver.send_signal(signal.SIGINT)
        assert driver.wait(30) == 0, str(log)
    finally:
        if driver.poll() is None:
            driver.kill()
    assert 'Lost the connection' not in str(log), str(log)
    assert calls.count('stopMove()') == 1, str(calls)
    for service in ('ALTextToSpeech', 'ALAudioPlayer'):
        assert calls.count(service + '.stopAll()') == 1, str(calls)
    assert 'failed on shutdown' not in str(log), str(log)


def call_trigger(node, name):
    """Calls a std_srvs/Trigger service of the driver; returns its response."""
    import rclpy
    from std_srvs.srv import Trigger
    client = node.create_client(Trigger, '/naoqi_driver/' + name)
    assert client.wait_for_service(timeout_sec=10.0), name
    future = client.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
    assert future.done(), name
    return future.result()


def test_recording_writes_the_sensor_data(naoqi, tmp_path):
    import rclpy
    import rosbag2_py
    url, calls = naoqi
    driver = start_driver(url, tmp_path, enabled=('odom',))
    log = Output(driver.stdout)
    rclpy.init()
    node = rclpy.create_node('recording_test')
    try:
        assert log.wait_for('naoqi_driver initialized', 60), str(log)
        started = call_trigger(node, 'start_recording')
        assert started.success, started.message
        time.sleep(1.0)
        stopped = call_trigger(node, 'stop_recording')
        assert stopped.success and stopped.message == started.message, stopped.message
        info = rosbag2_py.Info().read_metadata(stopped.message, 'sqlite3')
        counts = {t.topic_metadata.name: t.message_count for t in info.topics_with_message_count}
        assert counts.get('/pepper_odom', 0) > 0, counts
        driver.send_signal(signal.SIGINT)
        assert driver.wait(30) == 0, str(log)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        if driver.poll() is None:
            driver.kill()
