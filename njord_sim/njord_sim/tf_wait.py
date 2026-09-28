"""Hold sensor messages until TF at their acquisition stamp exists (no ROS).

A lidar cloud or image is usable only with the vessel pose at its own
acquisition time. That transform can arrive after the message: the
estimator publishes TF at its own rate, and when the simulator runs slower
than real time Gazebo steps in bursts, so most messages arrive first (measured
at REAL_TIME_FACTOR 0.3: TF was available on arrival for 21 % of lidar
clouds, against 90 % at 1). Dropping such messages starves the map. This
queue keeps them, in stamp order, until the transform can be looked up or the
message is older than ``max_age_s`` of simulation time. Messages are never
re-stamped.
"""
from collections import deque


class TfWaitQueue:
    """FIFO of (stamp, message) released in stamp order once TF is available."""

    def __init__(self, max_age_s, capacity=20):
        if not max_age_s > 0 or capacity < 1:
            raise ValueError('max_age_s and capacity must be positive')
        self.max_age_s = max_age_s
        self.items = deque(maxlen=capacity)  # the oldest message falls out when full

    def push(self, stamp, message):
        """Queue a message with acquisition ``stamp`` (s); older than the newest is ignored."""
        if self.items and stamp <= self.items[-1][0]:
            return False
        self.items.append((stamp, message))
        return True

    def release(self, now, process):
        """Hand queued messages to ``process`` in stamp order at simulation time ``now``.

        ``process(message)`` returns False while the transform is not yet
        available; the message then stays queued with everything after it, so
        order is kept. Messages older than ``max_age_s`` (or from the future
        after a clock reset) are dropped. Returns the number processed.
        """
        done = 0
        while self.items:
            stamp, message = self.items[0]
            if not 0 <= now-stamp <= self.max_age_s:
                self.items.popleft()
                continue
            if not process(message):
                break
            self.items.popleft()
            done += 1
        return done

    def clear(self):
        self.items.clear()
