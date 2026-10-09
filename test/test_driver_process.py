"""naoqi_driver_node as a process against fake_naoqi_server: Ctrl+C stops the robot
and the process exits cleanly. Unit tests cannot see main()'s start-up and exit."""

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


def start_driver(url, folder):
    host, port = url.replace('tcp://', '').split(':')
    params = folder / 'params.yaml'
    params.write_text('/**:\n  ros__parameters:\n    converters:\n' + ''.join(
        '      {}:\n        enabled: false\n'.format(name) for name in CONVERTERS))
    env = dict(os.environ, ROS_DOMAIN_ID=str(random.randint(100, 200)), ROS_LOCALHOST_ONLY='1')
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
    for service in ('ALAnimatedSpeech', 'ALTextToSpeech', 'ALAudioPlayer'):
        assert calls.count(service + '.stopAll()') == 1, str(calls)
