"""scripts/robot_watchdog.py against fake ALMemory, ALMotion and clock."""

import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'scripts'))

import robot_watchdog  # noqa: E402


class FakeMemory(object):

    def __init__(self, value=None):
        self.value = value  # None: the key does not exist yet
        self.fail = False

    def getData(self, key):
        assert key == robot_watchdog.KEY
        if self.value is None or self.fail:
            raise RuntimeError('ALMemory::getData Data not found')
        return self.value


class FakeMotion(object):

    def __init__(self):
        self.stops = 0
        self.failures_left = 0

    def stopMove(self):
        if self.failures_left:
            self.failures_left -= 1
            raise RuntimeError('ALMotion is busy')
        self.stops += 1


class Robot(object):
    """The watchdog, a fake driver writing the heartbeat, and a simulated clock."""

    def __init__(self, initial_value=None):
        self.now = 0.0
        self.memory = FakeMemory(initial_value)
        self.motion = FakeMotion()
        self.count = 0
        self.watchdog = robot_watchdog.HeartbeatWatchdog(
            self.memory, self.motion, timeout=1.0, clock=lambda: self.now)
        self.watchdog.poll()

    def run(self, seconds, beating):
        """Poll every POLL_PERIOD for seconds; a beating driver writes every 0.2 s."""
        for step in range(int(round(seconds / robot_watchdog.POLL_PERIOD))):
            self.now += robot_watchdog.POLL_PERIOD
            if beating and step % 2 == 0:
                self.count += 1
                self.memory.value = self.count
            self.watchdog.poll()

    def exit_cleanly(self):
        self.memory.value = robot_watchdog.STOPPED


def test_no_driver_never_stops_the_base():
    robot = Robot()
    robot.run(10, beating=False)
    assert robot.motion.stops == 0


def test_a_value_left_by_an_earlier_driver_is_not_a_heartbeat():
    robot = Robot(initial_value=42)
    robot.run(10, beating=False)
    assert robot.motion.stops == 0


def test_a_live_heartbeat_never_stops_the_base():
    robot = Robot()
    robot.run(10, beating=True)
    assert robot.motion.stops == 0


def test_a_stale_heartbeat_stops_the_base_once_after_the_timeout():
    robot = Robot()
    robot.run(2, beating=True)  # the last beat lands 0.1 s before the end
    robot.run(0.8, beating=False)
    assert robot.motion.stops == 0
    robot.run(0.4, beating=False)
    assert robot.motion.stops == 1
    robot.run(10, beating=False)
    assert robot.motion.stops == 1


def test_a_clean_exit_never_stops_the_base():
    robot = Robot()
    robot.run(2, beating=True)
    robot.exit_cleanly()
    robot.run(10, beating=False)
    assert robot.motion.stops == 0


def test_a_restarted_driver_is_watched_again():
    robot = Robot()
    robot.run(2, beating=True)
    robot.run(2, beating=False)
    assert robot.motion.stops == 1
    robot.count = 0  # the respawned driver counts from the start
    robot.run(2, beating=True)
    assert robot.motion.stops == 1
    robot.run(2, beating=False)
    assert robot.motion.stops == 2


def test_a_failed_stop_is_retried():
    robot = Robot()
    robot.motion.failures_left = 2
    robot.run(2, beating=True)
    robot.run(2, beating=False)
    assert robot.motion.stops == 1


def test_an_unreadable_heartbeat_counts_as_stale():
    robot = Robot()
    robot.run(2, beating=True)
    robot.memory.fail = True
    robot.run(2, beating=True)
    assert robot.motion.stops == 1


@pytest.mark.parametrize('argv, url, timeout', [
    ([], 'tcp://127.0.0.1:9559', 1.0),
    (['--url', 'tcp://10.0.0.2:9559', '--timeout', '2.5'], 'tcp://10.0.0.2:9559', 2.5),
    (['--pip', '127.0.0.1', '--pport', '9600', '--unknown'], 'tcp://127.0.0.1:9600', 1.0),
])
def test_arguments(argv, url, timeout):
    args = robot_watchdog.parse_args(argv)
    assert args.url == url
    assert args.timeout == timeout
