"""Require 10 overlapping simulation seconds of advancing sensors and acquisition TF.

SMOKE_TIMEOUT_S bounds wall time. Truth mode is plumbing evidence only.
"""
import json
from collections import defaultdict, deque
from pathlib import Path
import os
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener

from njord_sim.constants import CAMERAS, LIDAR_POINTS_TOPIC, camera_topic
from rclpy.parameter import Parameter
from rclpy.time import Time
from runtime_checks import StreamWindow, image_varies, valid_odometry


class Check(Node):
    """Record in ``seen`` which of the four data sources have produced valid data."""
    def __init__(self):
        super().__init__('njord_runtime_check', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.seen = {}
        self.window = StreamWindow()
        self.pending = defaultdict(deque)
        for camera, side in zip(CAMERAS, ('left','right')):
            self.create_subscription(Image, camera_topic(camera, 'image_raw'),
                                     lambda m,s=side: self.camera(s,m), qos_profile_sensor_data)
        self.create_subscription(PointCloud2, LIDAR_POINTS_TOPIC, self.lidar, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/njord/odometry', self.odom, qos_profile_sensor_data)

    def observe(self, name, message, valid, details):
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        self.pending[name].append((stamp, message, valid, details))

    def drain(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        for name, pending in self.pending.items():
            while pending:
                stamp, message, valid, details = pending[0]
                frames = [message.header.frame_id]
                if name == 'navigation':
                    frames.append(message.child_frame_id)
                available = all(frame and self.tf.can_transform(
                    'map', frame, Time.from_msg(message.header.stamp)) for frame in frames)
                if valid and not available and 0 <= now-stamp <= self.window.max_gap:
                    break  # Wait for transport-delayed TF at the acquisition time.
                pending.popleft()
                accepted = self.window.observe(name, stamp, now, valid and available)
                self.seen[name] = dict(details, stamp_s=stamp, accepted=accepted)

    def camera(self, side, m):
        self.observe(side, m, image_varies(m), {'width': m.width, 'height': m.height,
                                             'frame': m.header.frame_id})

    def lidar(self, m):
        points = point_cloud2.read_points_numpy(m, field_names=('x','y','z'), skip_nans=True).reshape(-1,3)
        finite = points[np.isfinite(points).all(axis=1)]
        self.observe('lidar', m, len(finite) > 0, {'finite_points': len(finite), 'frame': m.header.frame_id})

    def odom(self, m):
        valid = valid_odometry(m)
        self.observe('navigation', m, valid, {'frame': m.header.frame_id})


def main():
    rclpy.init()
    node=None
    failure=None
    # Steady wall-time deadline, independent of simulation speed.
    end=time.monotonic()+float(os.environ.get('SMOKE_TIMEOUT_S', '90'))
    passed=False
    try:
        node=Check()
        while rclpy.ok() and time.monotonic()<end:
            rclpy.spin_once(node, timeout_sec=0.1)
            node.drain()
            if node.window.passed(node.get_clock().now().nanoseconds * 1e-9):
                passed=True
                break
    except (Exception, KeyboardInterrupt) as error:
        failure=f'{type(error).__name__}: {error}'
    finally:
        result={'passed':passed,'received':node.seen if node else {},
                'windows':node.window.windows if node else {}, 'failure':failure,
                'required_contiguous_sim_s':10.0,
                'evidence_level':'sensor_stream_and_tf_plumbing_only',
                'estimator_accuracy_validated':False}
        serialized=json.dumps(result,indent=2,allow_nan=False)
        path=Path(os.environ.get('SMOKE_OUTPUT','/outputs/sensor_validation.json'))
        path.parent.mkdir(parents=True,exist_ok=True)
        with path.open('x') as stream:
            stream.write(serialized+'\n')
        print(serialized)
        if node:
            node.destroy_node()
        rclpy.try_shutdown()

    raise SystemExit(0 if passed else 2)


if __name__=='__main__':
    main()
