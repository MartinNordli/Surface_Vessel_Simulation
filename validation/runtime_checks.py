"""ROS-independent evidence checks; passing these is not live validation."""
import math
import numpy as np

from njord_sim.constants import BASE_FRAME


def image_varies(message):
    channels = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4}.get(message.encoding)
    if not channels or message.width <= 0 or message.height <= 0:
        return False
    stride = message.step
    data = np.frombuffer(bytes(message.data), dtype=np.uint8)
    if stride < message.width * channels or len(data) < stride * message.height:
        return False
    pixels = data[:stride * message.height].reshape(message.height, stride)
    pixels = pixels[:, :message.width * channels].reshape(-1, channels)[:, :3]
    # Variation across channels in a solid red frame is not spatial variation.
    return bool(np.any(np.std(pixels, axis=0) > 1.0))


def valid_odometry(message, parent='map', child=BASE_FRAME):
    p, q, t = message.pose.pose.position, message.pose.pose.orientation, message.twist.twist
    values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w,
              t.linear.x, t.linear.y, t.linear.z, t.angular.x, t.angular.y, t.angular.z)
    return (message.header.frame_id == parent and message.child_frame_id == child
            and all(math.isfinite(v) for v in values)
            and abs(sum(v*v for v in (q.x, q.y, q.z, q.w)) - 1.0) <= 0.01)


class StreamWindow:
    """Overlapping advancing acquisition stamps; invalid data resets coverage."""
    def __init__(self, names=('left', 'right', 'lidar', 'navigation'), duration=10.0, max_gap=0.5):
        self.names, self.duration, self.max_gap = names, duration, max_gap
        self.windows = {}

    def observe(self, name, stamp, now, valid):
        previous = self.windows.get(name)
        if (not valid or not math.isfinite(stamp) or stamp <= 0
                or not 0 <= now - stamp <= self.max_gap
                or (previous and stamp <= previous[1])):
            self.windows.pop(name, None)
            return False
        start = previous[0] if previous and stamp - previous[1] <= self.max_gap else stamp
        self.windows[name] = (start, stamp)
        return True

    def passed(self, now):
        if any(name not in self.windows for name in self.names):
            return False
        windows = [self.windows[name] for name in self.names]
        return (all(0 <= now-end <= self.max_gap for _, end in windows)
                and min(end for _, end in windows) - max(start for start, _ in windows) >= self.duration)
