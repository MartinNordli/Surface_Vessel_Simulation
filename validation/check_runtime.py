"""Check live GPU sensor data and sensor-based navigation; exits nonzero on timeout."""
import json
import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time


class Check(Node):
    def __init__(self):
        super().__init__('njord_runtime_check')
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.seen = {}
        for side in ('left','right'):
            self.create_subscription(Image, f'/wamv/sensors/cameras/front_{side}_camera_sensor/image_raw',
                                     lambda m,s=side: self.camera(s,m), qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/wamv/sensors/lidars/lidar_wamv_sensor/points', self.lidar, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/njord/odometry', self.odom, qos_profile_sensor_data)

    def camera(self, side, m):
        data = np.frombuffer(bytes(m.data), dtype=np.uint8)
        if m.width > 0 and m.height > 0 and data.size >= m.width*m.height*3 and np.std(data) > 1:
            self.seen[side] = {'width':m.width, 'height':m.height, 'frame':m.header.frame_id}

    def lidar(self, m):
        points = point_cloud2.read_points_numpy(m, field_names=('x','y','z'), skip_nans=True).reshape(-1,3)
        finite = points[np.isfinite(points).all(axis=1)]
        if len(finite):
            self.seen['lidar'] = {'finite_points':len(finite), 'frame':m.header.frame_id}

    def odom(self,m):
        q=m.pose.pose.orientation
        if m.header.frame_id == 'map' and all(math.isfinite(v) for v in (q.x,q.y,q.z,q.w,m.pose.pose.position.x)):
            self.seen['navigation'] = {'frame':m.header.frame_id}


def main():
    rclpy.init()
    node=Check()
    end=time.monotonic()+90
    passed=False
    try:
        while rclpy.ok() and time.monotonic()<end:
            rclpy.spin_once(node, timeout_sec=0.1)
            if len(node.seen)==4 and node.tf.can_transform('map','wamv/front_left_camera_link_optical',Time()):
                passed=True
                break
        print(json.dumps({'passed':passed,'received':node.seen},indent=2))
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    raise SystemExit(0 if passed else 2)


if __name__=='__main__':
    main()
