"""Sensor-only covariance and IMU attitude noise; no ground truth subscription."""
import copy
import math
import random
import time
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, NavSatFix
from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus


class SensorAdapter(Node):
    def __init__(self):
        super().__init__('sensor_adapter')
        self.declare_parameters('', [('seed', 0), ('orientation_noise_rad', 0.005),
                                     ('gps_xy_std_m', 0.3), ('gps_z_std_m', 0.5)])
        self.rng = random.Random(self.get_parameter('seed').value)
        self.gps_rng = random.Random(self.get_parameter('seed').value + 10000)
        self.received = {}
        self.imu_pub = self.create_publisher(Imu, '/wamv/sensors/imu/imu/data', qos_profile_sensor_data)
        self.gps_pub = self.create_publisher(NavSatFix, '/wamv/sensors/gps/gps/fix', qos_profile_sensor_data)
        self.status_pub = self.create_publisher(DiagnosticArray, '/njord/navigation_status', 1)
        self.create_subscription(Imu, '/wamv/sensors/imu/imu/data_raw', self.imu, qos_profile_sensor_data)
        self.create_subscription(NavSatFix, '/wamv/sensors/gps/gps/fix_raw', self.gps, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/njord/gps/odometry',
                                 lambda msg: self.record('gps_projected', msg), qos_profile_sensor_data)
        self.create_subscription(Odometry, '/njord/odometry', self.estimate, qos_profile_sensor_data)
        self.create_timer(0.1, self.status, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def record(self, name, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.received[name] = (time.monotonic(), stamp)

    def estimate(self, msg):
        pos = msg.pose.pose.position
        if (msg.header.frame_id == 'map' and all(math.isfinite(v) for v in (pos.x,pos.y,pos.z))
                and 0 <= msg.pose.covariance[0] < 4 and 0 <= msg.pose.covariance[7] < 4):
            self.record('estimate', msg)

    def imu(self, raw):
        q = raw.orientation
        values = [q.x, q.y, q.z, q.w, raw.angular_velocity.x, raw.angular_velocity.y,
                  raw.angular_velocity.z, raw.linear_acceleration.x,
                  raw.linear_acceleration.y, raw.linear_acceleration.z]
        if not all(math.isfinite(v) for v in values) or sum(v*v for v in values[:4]) < 0.5:
            return
        msg = copy.deepcopy(raw)
        std = self.get_parameter('orientation_noise_rad').value
        # Small isotropic rotation applied to the measured ENU orientation.
        x, y, z = [self.rng.gauss(0, std) / 2 for _ in range(3)]
        w = 1.0
        noisy = [w*q.x+x*q.w+y*q.z-z*q.y, w*q.y-x*q.z+y*q.w+z*q.x,
                 w*q.z+x*q.y-y*q.x+z*q.w, w*q.w-x*q.x-y*q.y-z*q.z]
        norm = math.sqrt(sum(v*v for v in noisy))
        msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = [v/norm for v in noisy]
        msg.orientation_covariance = [std*std if i in (0,4,8) else 0.0 for i in range(9)]
        msg.angular_velocity_covariance = [0.009**2 if i in (0,4,8) else 0.0 for i in range(9)]
        msg.linear_acceleration_covariance = [0.021**2 if i in (0,4,8) else 0.0 for i in range(9)]
        self.record('imu', msg)
        self.imu_pub.publish(msg)

    def gps(self, raw):
        if raw.status.status < 0 or not all(math.isfinite(v) for v in (raw.latitude, raw.longitude, raw.altitude)):
            return
        msg = copy.deepcopy(raw)
        xy = self.get_parameter('gps_xy_std_m').value**2
        z = self.get_parameter('gps_z_std_m').value**2
        # Local WGS84 metres per radian; avoid Gazebo's degree-valued noise.
        lat = math.radians(raw.latitude)
        a, e2 = 6378137.0, 6.69437999014e-3
        den = 1.0-e2*math.sin(lat)**2
        meridian = a*(1.0-e2)/(den**1.5)
        prime = a/math.sqrt(den)
        msg.latitude += math.degrees(self.gps_rng.gauss(0, math.sqrt(xy))/meridian)
        msg.longitude += math.degrees(self.gps_rng.gauss(0, math.sqrt(xy))/(prime*max(1e-6, math.cos(lat))))
        msg.altitude += self.gps_rng.gauss(0, math.sqrt(z))
        msg.position_covariance = [xy, 0.0, 0.0, 0.0, xy, 0.0, 0.0, 0.0, z]
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self.record('gps', msg)
        self.gps_pub.publish(msg)

    def status(self):
        now = self.get_clock().now()
        valid = all(k in self.received and time.monotonic()-self.received[k][0] < 0.5 and
                    -0.1 <= now.nanoseconds*1e-9-self.received[k][1] <= 0.5
                    for k in ('gps','imu','gps_projected','estimate'))
        msg = DiagnosticArray()
        msg.header.stamp = now.to_msg()
        msg.status = [DiagnosticStatus(name='navigation', hardware_id='gps_imu',
                                       level=DiagnosticStatus.OK if valid else DiagnosticStatus.ERROR,
                                       message='fresh GPS/IMU and initialized estimate' if valid else 'missing, stale or uninitialized navigation')]
        self.status_pub.publish(msg)


def main():
    rclpy.init()
    node = SensorAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
