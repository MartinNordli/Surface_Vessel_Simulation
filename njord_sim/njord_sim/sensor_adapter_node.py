"""Sensor-only covariance and IMU attitude noise; no ground truth subscription.

ROS node ``sensor_adapter``: sits between the raw Gazebo GPS/IMU and the
localisation EKFs. It adds seeded, metric measurement noise, fills in
covariances, and reports whether navigation is healthy.

Subscribes (best-effort sensor QoS):
    ``/wamv/sensors/imu/imu/data_raw`` (sensor_msgs/Imu): Gazebo IMU,
        orientation in ENU.
    ``/wamv/sensors/gps/gps/fix_raw`` (sensor_msgs/NavSatFix): Gazebo GPS
        with its own position noise disabled (see ``vessel.py``).
    ``/njord/gps/odometry`` (nav_msgs/Odometry): GPS fix projected into the
        local frame by navsat_transform; only checked for freshness.
    ``/njord/odometry`` (nav_msgs/Odometry): global EKF estimate in ``map``;
        only checked for freshness and plausibility.

Publishes:
    ``/wamv/sensors/imu/imu/data`` (sensor_msgs/Imu): orientation with added
        noise; rates and accelerations unchanged; diagonal covariances.
    ``/wamv/sensors/gps/gps/fix`` (sensor_msgs/NavSatFix): position with
        added metric Gaussian noise and a known diagonal covariance.
    ``/njord/navigation_status`` (diagnostic_msgs/DiagnosticArray): status
        ``navigation`` at 10 Hz of steady (wall) time; OK only while all four
        inputs are fresh, otherwise ERROR. Read by the command guard and the
        evaluator.

Parameters:
    ``seed``: RNG seed, set by the launch from the scenario so runs repeat.
    ``orientation_noise_rad``: per-axis IMU attitude noise std [rad].
    ``gps_xy_std_m``, ``gps_z_std_m``: GPS horizontal / vertical std [m].
    The noise defaults come from the vessel configuration (``defaults.py``).

Output messages keep the header (stamp and frame) of the raw measurement;
they are never re-stamped. Invalid inputs (no fix, non-finite values, zero
quaternion) are dropped, which lets the status go stale and turn ERROR.
"""
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

from njord_sim.defaults import node_defaults


class SensorAdapter(Node):
    """Add seeded GPS/IMU noise and publish the navigation health status."""

    def __init__(self):
        super().__init__('sensor_adapter')
        # Noise levels come from the vessel file (see defaults.py); the launch
        # always sets the seed from the scenario.
        self.declare_parameters('', [('seed', 0), *node_defaults('sensor_adapter')])
        # Separate, reproducible random streams for IMU and GPS, so the GPS
        # noise sequence does not depend on how many IMU messages arrived.
        self.rng = random.Random(self.get_parameter('seed').value)
        self.gps_rng = random.Random(self.get_parameter('seed').value + 10000)
        # name -> (steady receipt time [s], message stamp [s, simulation time])
        self.received = {}
        self.imu_pub = self.create_publisher(Imu, '/wamv/sensors/imu/imu/data',
                                             qos_profile_sensor_data)
        self.gps_pub = self.create_publisher(NavSatFix, '/wamv/sensors/gps/gps/fix',
                                             qos_profile_sensor_data)
        self.status_pub = self.create_publisher(DiagnosticArray, '/njord/navigation_status', 1)
        self.create_subscription(Imu, '/wamv/sensors/imu/imu/data_raw', self.imu,
                                 qos_profile_sensor_data)
        self.create_subscription(NavSatFix, '/wamv/sensors/gps/gps/fix_raw', self.gps,
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, '/njord/gps/odometry',
                                 lambda msg: self.record('gps_projected', msg), qos_profile_sensor_data)
        self.create_subscription(Odometry, '/njord/odometry', self.estimate, qos_profile_sensor_data)
        # Health watchdog runs on steady time so it keeps reporting even if
        # /clock stops.
        self.create_timer(0.1, self.status, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def record(self, name, msg):
        """Note when input ``name`` was received (steady) and its stamp (simulation)."""
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.received[name] = (time.monotonic(), stamp)

    def estimate(self, msg):
        """Record the EKF estimate only if it looks initialised.

        Requires frame ``map``, a finite position and x/y position variances
        in [0, 4) m^2 (standard deviation below 2 m).
        """
        pos = msg.pose.pose.position
        # Pose covariance is a row-major 6x6: [0] is var(x), [7] is var(y).
        if (msg.header.frame_id == 'map' and all(math.isfinite(v) for v in (pos.x,pos.y,pos.z))
                and 0 <= msg.pose.covariance[0] < 4 and 0 <= msg.pose.covariance[7] < 4):
            self.record('estimate', msg)

    def imu(self, raw):
        """Add attitude noise to a raw IMU message and republish it.

        The noise is a small random rotation with independent per-axis angles
        ``N(0, orientation_noise_rad)``, drawn fresh for each message (white
        noise, no bias). It is built as the small-angle quaternion
        ``dq = (ax/2, ay/2, az/2, 1)`` and applied on the left,
        ``q_noisy = normalize(dq * q)``, i.e. about the axes of the ENU world
        frame. Angular velocity and linear acceleration pass through
        unchanged; only their covariances are filled in.
        """
        q = raw.orientation
        values = [q.x, q.y, q.z, q.w, raw.angular_velocity.x, raw.angular_velocity.y,
                  raw.angular_velocity.z, raw.linear_acceleration.x,
                  raw.linear_acceleration.y, raw.linear_acceleration.z]
        # Drop non-finite data and near-zero (uninitialised) quaternions.
        if not all(math.isfinite(v) for v in values) or sum(v*v for v in values[:4]) < 0.5:
            return
        # Deep copy keeps the raw header: same acquisition stamp and frame.
        msg = copy.deepcopy(raw)
        std = self.get_parameter('orientation_noise_rad').value
        # Small isotropic rotation applied to the measured ENU orientation.
        # A rotation by angle a has quaternion (sin(a/2) axis, cos(a/2)),
        # approximately (a/2 axis, 1) for small a.
        x, y, z = [self.rng.gauss(0, std) / 2 for _ in range(3)]
        w = 1.0
        # Hamilton product dq * q, with dq = (x, y, z, w).
        noisy = [w*q.x+x*q.w+y*q.z-z*q.y, w*q.y-x*q.z+y*q.w+z*q.x,
                 w*q.z+x*q.y-y*q.x+z*q.w, w*q.w-x*q.x-y*q.y-z*q.z]
        norm = math.sqrt(sum(v*v for v in noisy))
        msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = [v/norm for v in noisy]
        # Row-major 3x3 covariances; indices 0, 4, 8 are the diagonal.
        # Orientation [rad^2], angular velocity [(rad/s)^2], acceleration [(m/s^2)^2].
        msg.orientation_covariance = [std*std if i in (0,4,8) else 0.0 for i in range(9)]
        msg.angular_velocity_covariance = [0.009**2 if i in (0,4,8) else 0.0 for i in range(9)]
        msg.linear_acceleration_covariance = [0.021**2 if i in (0,4,8) else 0.0 for i in range(9)]
        self.record('imu', msg)
        self.imu_pub.publish(msg)

    def gps(self, raw):
        """Add metric Gaussian noise to a raw GPS fix and republish it.

        Horizontal noise is drawn in metres, north and east separately with
        std ``gps_xy_std_m``, and converted to degrees using the local WGS84
        radii of curvature at the fix latitude ``lat``:

        * meridian (north-south) radius ``M = a (1 - e^2) / (1 - e^2 sin^2 lat)^1.5``,
          so ``dlat = north / M`` [rad];
        * prime-vertical radius ``N = a / sqrt(1 - e^2 sin^2 lat)``, so
          ``dlon = east / (N cos lat)`` [rad] (cos clamped to 1e-6 near the
          poles).

        Altitude gets ``N(0, gps_z_std_m)`` metres. The covariance is the
        diagonal ``[xy, xy, z]`` in m^2 (east, north, up), marked as known.
        Fixes with a negative status (no fix) or non-finite values are dropped.
        """
        if raw.status.status < 0 or not all(math.isfinite(v)
                                            for v in (raw.latitude, raw.longitude, raw.altitude)):
            return
        # Deep copy keeps the raw header: same acquisition stamp and frame.
        msg = copy.deepcopy(raw)
        # Variances [m^2].
        xy = self.get_parameter('gps_xy_std_m').value**2
        z = self.get_parameter('gps_z_std_m').value**2
        # Local WGS84 metres per radian; avoid Gazebo's degree-valued noise.
        lat = math.radians(raw.latitude)
        # WGS84 semi-major axis [m] and first eccentricity squared.
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
        """Publish the ``navigation`` health status (steady 10 Hz timer).

        OK only if every input (noisy GPS, noisy IMU, projected GPS odometry
        and an initialised EKF estimate) was received less than 0.5 s ago in
        steady time *and* its stamp is at most 0.5 s behind and 0.1 s ahead
        of the current simulation time. The first check catches stopped
        publishers, the second stale or future data. The status itself is
        stamped with the current simulation time; it is a heartbeat, not a
        re-stamped measurement.
        """
        now = self.get_clock().now()
        valid = all(k in self.received and time.monotonic()-self.received[k][0] < 0.5 and
                    -0.1 <= now.nanoseconds*1e-9-self.received[k][1] <= 0.5
                    for k in ('gps','imu','gps_projected','estimate'))
        msg = DiagnosticArray()
        msg.header.stamp = now.to_msg()
        msg.status = [DiagnosticStatus(
            name='navigation', hardware_id='gps_imu',
            level=DiagnosticStatus.OK if valid else DiagnosticStatus.ERROR,
            message=('fresh GPS/IMU and initialized estimate' if valid
                     else 'missing, stale or uninitialized navigation'))]
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
