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
        noise in attitude, angular rate and acceleration; diagonal covariances.
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
import time
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, NavSatFix
from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

from njord_sim.defaults import node_defaults
from njord_sim.geometry import valid_odometry
from njord_sim.sensor_noise_core import SensorNoise, covariance, noisy_orientation, noisy_fix


class SensorAdapter(Node):
    """Add seeded GPS/IMU noise and publish the navigation health status."""

    def __init__(self):
        super().__init__('sensor_adapter')
        # Noise levels come from the vessel file (see defaults.py); the launch
        # always sets the seed from the scenario.
        self.declare_parameters('', [('seed', 0), *node_defaults('sensor_adapter')])
        self.noise = SensorNoise(self.get_parameter('seed').value)
        self.received = {}
        self.last_clock = None
        self.clock_progress_wall = time.monotonic()
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

    def observe_clock(self):
        now = self.get_clock().now().nanoseconds*1e-9
        if self.last_clock is None or now != self.last_clock:
            if self.last_clock is not None and now < self.last_clock:
                self.received.clear()
                self.noise = SensorNoise(self.get_parameter('seed').value)
            self.clock_progress_wall = time.monotonic()
        self.last_clock = now
        return now

    def record(self, name, msg):
        """Accept advancing acquisition stamps; duplicates never renew health."""
        now = self.observe_clock()
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if (not 0 <= now-stamp <= self.get_parameter('stale_after_s').value
                or stamp <= self.received.get(name, (0., -math.inf))[1]):
            return False
        self.received[name] = (time.monotonic(), stamp)
        return True

    def estimate(self, msg):
        """Record the EKF estimate only if it looks initialised.

        Requires frame ``map``, a finite position and x/y position variances
        in [0, 4) m^2 (standard deviation below 2 m).
        """
        # Pose covariance is a row-major 6x6: [0] is var(x), [7] is var(y).
        if (valid_odometry(msg, 'map', 'wamv/base_link')
                and 0 <= msg.pose.covariance[0] < 4 and 0 <= msg.pose.covariance[7] < 4):
            self.record('estimate', msg)

    def imu(self, raw):
        """Add attitude noise to a raw IMU message and republish it.

        Independent streams add Gaussian rotation-vector, rate and acceleration
        errors. Covariance diagonals are exactly the configured SI variances.
        Headers and raw inputs are preserved.
        """
        q = raw.orientation
        values = [q.x, q.y, q.z, q.w, raw.angular_velocity.x, raw.angular_velocity.y,
                  raw.angular_velocity.z, raw.linear_acceleration.x,
                  raw.linear_acceleration.y, raw.linear_acceleration.z]
        # Drop non-finite data and near-zero (uninitialised) quaternions.
        if not all(math.isfinite(v) for v in values) or sum(v*v for v in values[:4]) < 0.5:
            return
        if not self.record('imu', raw):
            return
        msg = copy.deepcopy(raw)
        std = self.get_parameter('orientation_noise_rad').value
        angular = self.get_parameter('angular_velocity_noise_rad_s').value
        acceleration = self.get_parameter('linear_acceleration_noise_m_s2').value
        errors = self.noise.sample('orientation', (std,)*3)
        (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w) = noisy_orientation(
            (q.x, q.y, q.z, q.w), errors)
        for channel, vector, sigma in (
                ('angular_velocity', msg.angular_velocity, angular),
                ('linear_acceleration', msg.linear_acceleration, acceleration)):
            for axis, error in zip(('x', 'y', 'z'), self.noise.sample(channel, (sigma,)*3)):
                setattr(vector, axis, getattr(vector, axis)+error)
        msg.orientation_covariance = covariance((std,)*3)
        msg.angular_velocity_covariance = covariance((angular,)*3)
        msg.linear_acceleration_covariance = covariance((acceleration,)*3)
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
        if not self.record('gps', raw):
            return
        msg = copy.deepcopy(raw)
        xy = self.get_parameter('gps_xy_std_m').value
        z = self.get_parameter('gps_z_std_m').value
        msg.latitude, msg.longitude, msg.altitude = noisy_fix(
            raw.latitude, raw.longitude, raw.altitude, self.noise.sample('gps', (xy, xy, z)))
        msg.position_covariance = covariance((xy, xy, z))
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self.gps_pub.publish(msg)

    def status(self):
        """Publish the ``navigation`` health status (steady 10 Hz timer).

        OK only while all inputs advance and remain fresh in simulation time,
        and the simulation clock has progressed within the wall-time watchdog.
        Replayed identical stamps cannot renew health.
        """
        simulation_now = self.observe_clock()
        now = self.get_clock().now()
        valid = (time.monotonic()-self.clock_progress_wall < self.get_parameter('clock_stall_after_s').value
                 and all(k in self.received and
                         0 <= simulation_now-self.received[k][1] <= self.get_parameter('stale_after_s').value
                         for k in ('gps', 'imu', 'gps_projected', 'estimate')))
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
