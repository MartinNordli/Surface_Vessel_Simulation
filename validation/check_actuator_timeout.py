#!/usr/bin/env python3
"""Measure the live Gazebo actuator watchdog on a simulator-only partition.

Stop autonomy, command-guard and dynamics publishers before invoking this check.
The check applies physical thrust briefly. It does not stop other processes.

Example, inside the Gazebo container after the dynamics manoeuvre ends:
  python3 validation/check_actuator_timeout.py \
      --partition njord-dynamics --output /outputs/actuator_timeout.json

The result file and final stdout record are strict JSON. Exit 0 means all checks
passed; exit 1 means a failed assertion or unavailable simulator/transport.
"""

import argparse
from collections import deque
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import threading
import time


FORCES = '/njord/actuator_forces'
THRUSTERS = {'left': '/wamv/thrusters/left/thrust',
             'right': '/wamv/thrusters/right/thrust'}


class CheckFailure(RuntimeError):
    pass


class WatchdogProbe:
    def __init__(self, args, result):
        # Import after partition selection, before initializing Gazebo transport.
        from gz.transport13 import Node
        from gz.msgs10.double_pb2 import Double
        from gz.msgs10.twist_pb2 import Twist
        self.args, self.result, self.Twist = args, result, Twist
        self.condition = threading.Condition()
        self.samples = {side: deque(maxlen=20000) for side in THRUSTERS}
        self.node = Node()
        for side, topic in THRUSTERS.items():
            if not self.node.subscribe(Double, topic, lambda msg, side=side: self.receive(side, msg.data)):
                raise CheckFailure(f'Could not subscribe to {topic}')
        self.publisher = self.node.advertise(FORCES, Twist)
        self.last_publish = None

    def receive(self, side, value):
        with self.condition:
            self.samples[side].append((time.monotonic(), value))
            self.condition.notify_all()

    def publish(self, left, right):
        msg = self.Twist()
        msg.linear.x, msg.linear.y = float(left), float(right)
        sent = time.monotonic()
        if not self.publisher.publish(msg):
            raise CheckFailure('Gazebo rejected force publication')
        self.last_publish = sent
        return sent

    def latest_matches(self, expected, since):
        with self.condition:
            return all(self.samples[side] and self.samples[side][-1][0] >= since
                       and math.isfinite(self.samples[side][-1][1])
                       and abs(self.samples[side][-1][1] - target) <= 1e-6
                       for side, target in expected.items())

    def wait(self, seconds):
        with self.condition:
            self.condition.wait(timeout=max(0.0, seconds))

    def connect(self):
        deadline = time.monotonic() + self.args.discovery_s
        while time.monotonic() < deadline:
            with self.condition:
                observed_both = all(self.samples[side] for side in THRUSTERS)
            if observed_both and self.publisher.has_connections():
                return
            self.wait(0.025)
        raise CheckFailure('Discovery timed out: require a running simulator, matching partition, '
                           'watchdog subscription and both original thrust topics')

    def establish_finite(self):
        expected = {'left': self.args.left_force_n, 'right': self.args.right_force_n}
        started = time.monotonic()
        deadline, next_send = started + self.args.discovery_s, started
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_send:
                self.publish(expected['left'], expected['right'])
                next_send = now + 0.04
            if self.latest_matches(expected, started):
                # Final publication defines the start of publisher silence. Keep
                # the advertised endpoint alive; it must not refresh watchdog data.
                return self.publish(expected['left'], expected['right'])
            self.wait(0.01)
        raise CheckFailure('Distinct finite forces were not forwarded on both original thrust topics; '
                           'check paused world, competing publishers or force limits')

    def first_zero_times(self, since):
        with self.condition:
            return {side: next((stamp for stamp, value in self.samples[side]
                                if stamp >= since and math.isfinite(value) and abs(value) <= 1e-6), None)
                    for side in THRUSTERS}

    def measure_timeout(self):
        stopped = self.establish_finite()
        allowed = self.args.timeout_s + self.args.slack_s
        deadline = stopped + allowed
        zero_times = self.first_zero_times(stopped)
        while not all(value is not None for value in zero_times.values()) and time.monotonic() < deadline:
            self.wait(min(0.01, deadline - time.monotonic()))
            zero_times = self.first_zero_times(stopped)
        delays = {side: (stamp - stopped if stamp is not None else None)
                  for side, stamp in zero_times.items()}
        self.result['timeout_observed_s'] = delays
        if any(delay is None or delay > allowed for delay in delays.values()):
            raise CheckFailure(f'Both thrust outputs must become zero within {allowed:.3f} wall seconds '
                               'after the final finite force publication')
        # Reject a transient zero followed by renewed force from another source.
        settle_deadline = time.monotonic() + 0.15
        while time.monotonic() < settle_deadline:
            self.wait(0.01)
        if not self.latest_matches({'left': 0.0, 'right': 0.0}, stopped):
            raise CheckFailure('Thrust did not remain zero after publisher silence')
        self.result['checks']['publisher_silence_zeroes_both_thrusters'] = True

    def measure_nan(self):
        self.establish_finite()
        started = self.publish(math.nan, self.args.right_force_n)
        allowed = min(0.25, self.args.timeout_s * 0.5)
        deadline, next_send = started + allowed, started + 0.04
        zero_times = self.first_zero_times(started)
        while not all(value is not None for value in zero_times.values()) and time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_send:
                self.publish(math.nan, self.args.right_force_n)
                next_send = now + 0.04
            self.wait(0.005)
            zero_times = self.first_zero_times(started)
        delays = {side: (stamp - started if stamp is not None else None)
                  for side, stamp in zero_times.items()}
        self.result['nan_response_observed_s'] = delays
        self.result['nan_response_limit_s'] = allowed
        if any(delay is None or delay > allowed for delay in delays.values()):
            raise CheckFailure('NaN in one force must zero BOTH original outputs before normal timeout; '
                               f'allowed response {allowed:.3f} wall seconds')
        self.result['checks']['nan_zeroes_both_thrusters'] = True

    def cleanup(self):
        # Leave explicit zeros even when an assertion fails. No process is killed.
        for _ in range(3):
            try:
                self.publish(0.0, 0.0)
            except Exception:
                return
            time.sleep(0.02)


def positive(value):
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError('must be finite and positive')
    return number


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--partition', default=os.environ.get('GZ_PARTITION'),
                        help='Simulator-only Gazebo partition (defaults to GZ_PARTITION)')
    parser.add_argument('--output', type=Path, required=True, help='Strict JSON result path')
    parser.add_argument('--timeout-s', type=positive, default=0.5, help='Configured watchdog timeout, wall seconds')
    parser.add_argument('--slack-s', type=positive, default=0.25, help='Allowed transport/update scheduling overhead')
    parser.add_argument('--discovery-s', type=positive, default=10.0)
    parser.add_argument('--left-force-n', type=positive, default=80.0)
    parser.add_argument('--right-force-n', type=positive, default=120.0)
    args = parser.parse_args()
    result = {'schema_version': 1, 'status': 'failed',
              'started_utc': datetime.now(timezone.utc).isoformat(),
              'partition': args.partition, 'watchdog_timeout_s': args.timeout_s,
              'timeout_limit_s': args.timeout_s + args.slack_s,
              'finite_forces_n': {'left': args.left_force_n, 'right': args.right_force_n},
              'topics': {'input': FORCES, **THRUSTERS}, 'checks': {}, 'failures': []}
    probe = None
    try:
        if not args.partition:
            raise CheckFailure('Set --partition or GZ_PARTITION to the intended simulator-only partition')
        os.environ['GZ_PARTITION'] = args.partition
        probe = WatchdogProbe(args, result)
        probe.connect()
        probe.measure_timeout()
        result['checks']['finite_forces_forwarded'] = True
        probe.measure_nan()
        result['status'] = 'passed'
    except (Exception, KeyboardInterrupt) as error:
        result['failures'].append(f'{type(error).__name__}: {error}')
    finally:
        if probe is not None:
            probe.cleanup()
    result['finished_utc'] = datetime.now(timezone.utc).isoformat()
    serialized = json.dumps(result, indent=2, allow_nan=False)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(serialized + '\n', encoding='utf-8')
    print(serialized)
    return 0 if result['status'] == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
