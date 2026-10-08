#!/usr/bin/env python
"""Stop Pepper's base when naoqi_driver stops sending its heartbeat.

ALMotion.move() keeps the last velocity until told otherwise. naoqi_driver stops the
base when /cmd_vel goes quiet and when it exits cleanly, but it cannot when it
crashes, hangs or loses the network. This script runs on the robot itself: it watches
the counter the driver writes to ALMemory every 0.2 s and calls ALMotion.stopMove()
when the counter has not changed for --timeout seconds.

It acts only after it has seen the counter change, and not after a clean exit (the
driver then writes -1), so it leaves other programs alone while no driver runs.

    python robot_watchdog.py [--url tcp://127.0.0.1:9559] [--timeout 1.0]

Runs on Python 2.7 (NAOqi 2.5) and Python 3; needs NAOqi's qi module.
"""

from __future__ import print_function

import argparse
import logging
import sys
import time

KEY = 'NaoqiDriver/Heartbeat'
STOPPED = -1
POLL_PERIOD = 0.1  # seconds
RECONNECT_DELAY = 2.0  # seconds

# Python 2.7 has no monotonic clock.
monotonic = getattr(time, 'monotonic', time.time)


class HeartbeatWatchdog(object):
    """Call motion.stopMove() when the heartbeat in memory stops changing."""

    def __init__(self, memory, motion, timeout, clock=monotonic, log=None):
        self._memory = memory
        self._motion = motion
        self._timeout = timeout
        self._clock = clock
        self._log = log or logging.getLogger('robot_watchdog')
        self._started = False
        self._last_value = None
        self._last_change = None
        self._armed = False

    def _read(self):
        """Return the heartbeat, or None if it cannot be read (no driver ran yet)."""
        try:
            return self._memory.getData(KEY)
        except Exception:  # NAOqi raises RuntimeError for a missing key
            return None

    def poll(self):
        """Check the heartbeat once. Return True if this call stopped the base."""
        value = self._read()
        now = self._clock()
        if not self._started:
            # A value left by an earlier driver is not a heartbeat.
            self._started = True
            self._last_value = value
            return False
        if value is not None and value != self._last_value:
            self._last_value = value
            if value == STOPPED:
                if self._armed:
                    self._log.info('naoqi_driver exited cleanly')
                self._armed = False
            else:
                if not self._armed:
                    self._log.info('naoqi_driver heartbeat found, watching it')
                self._armed = True
                self._last_change = now
            return False
        if not self._armed or now - self._last_change <= self._timeout:
            return False
        try:
            self._motion.stopMove()
        except Exception as e:
            self._log.error('ALMotion.stopMove() failed, retrying: %s', e)
            return False
        self._log.warning(
            'No naoqi_driver heartbeat for %.1f s: stopped the base', now - self._last_change)
        self._armed = False
        return True


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Stop Pepper's base when naoqi_driver stops sending its heartbeat.")
    parser.add_argument(
        '--url', default='tcp://127.0.0.1:9559',
        help='NAOqi to watch (default: %(default)s, the robot this runs on)')
    parser.add_argument(
        '--timeout', type=float, default=1.0,
        help='seconds without a heartbeat before the base is stopped (default: %(default)s)')
    # NAOqi passes its address as --pip/--pport to the scripts it starts itself.
    parser.add_argument('--pip', help=argparse.SUPPRESS)
    parser.add_argument('--pport', type=int, default=9559, help=argparse.SUPPRESS)
    args, _ = parser.parse_known_args(argv)
    if args.pip:
        args.url = 'tcp://%s:%d' % (args.pip, args.pport)
    return args


def run(args, qi, log):
    """Poll forever, connecting again whenever NAOqi restarts."""
    while True:
        session = qi.Session()
        try:
            session.connect(args.url)
            watchdog = HeartbeatWatchdog(
                session.service('ALMemory'), session.service('ALMotion'), args.timeout,
                log=log)
        except RuntimeError as e:
            log.warning('Cannot reach NAOqi at %s (%s), retrying', args.url, e)
            time.sleep(RECONNECT_DELAY)
            continue
        log.info('Watching %s at %s, timeout %.1f s', KEY, args.url, args.timeout)
        while session.isConnected():
            watchdog.poll()
            time.sleep(POLL_PERIOD)
        log.warning('Lost NAOqi at %s, reconnecting', args.url)


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    logging.basicConfig(level=logging.INFO, format='%(asctime)s robot_watchdog: %(message)s')
    import qi  # NAOqi's Python SDK: present on the robot
    try:
        run(args, qi, logging.getLogger('robot_watchdog'))
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == '__main__':
    sys.exit(main())
