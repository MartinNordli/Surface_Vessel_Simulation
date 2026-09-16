#!/usr/bin/env python3
"""Actual rclpy/DDS smoke tests without Gazebo; run in a dedicated subprocess.

These checks establish ROS message interoperability and fail-safe command flow,
not GPU sensor rendering, vehicle dynamics, or successful course completion.
"""
import math
import os
from pathlib import Path
import sys
import tempfile
import time
import unittest

os.environ.setdefault('ROS_DOMAIN_ID', str(100 + os.getpid() % 100))
os.environ.setdefault('ROS_LOCALHOST_ONLY', '1')
_ros_logs = tempfile.TemporaryDirectory(prefix='njord-ros-smoke-')
os.environ.setdefault('ROS_LOG_DIR', _ros_logs.name)
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'njord_sim'))

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path as RosPath
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Float64, Header
from tf2_ros import StaticTransformBroadcaster

from njord_sim.command_guard_node import CommandGuard
from njord_sim.guidance_node import Guidance
from njord_sim.mapper_node import Mapper
from njord_sim.planner_node import Planner


class RosCase(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.executor = SingleThreadedExecutor()
        self.nodes = []
        self.driver = self.add(Node('runtime_smoke_driver'))
        self.outputs = []
        self.clock_tick = None
        self.driver.create_subscription(Twist, '/njord/actuator_forces',
                                        lambda msg: self.outputs.append((msg.linear.x, msg.linear.y)), 10)
        self.mission = self.driver.create_publisher(DiagnosticArray, '/njord/mission_status', 1)
        self.navigation = self.driver.create_publisher(DiagnosticArray, '/njord/navigation_status', 1)
        self.race = self.driver.create_publisher(Bool, '/njord/race_active',
                         QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def add(self, node):
        self.nodes.append(node)
        self.executor.add_node(node)
        return node

    def tearDown(self):
        for node in reversed(self.nodes):
            self.executor.remove_node(node)
            node.destroy_node()
        self.executor.shutdown()
        rclpy.shutdown()

    def pump(self, duration, publish=None):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            if self.clock_tick:
                self.clock_tick()
            if publish:
                publish()
            self.executor.spin_once(timeout_sec=0.01)
            # Drain ready subscriptions so frequent input cannot starve timers.
            for _ in range(12):
                self.executor.spin_once(timeout_sec=0.0)
            time.sleep(0.01)

    def until(self, predicate, publish=None, timeout=4.0):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            self.pump(0.05, publish)
        details = ''
        if not predicate() and hasattr(self, 'planner'):
            details = repr({'planner_error': self.planner.error, 'goal': self.planner.goal,
                            'position': self.planner.position, 'points': len(self.planner.points),
                            'goal_cell': self.planner.geometry.cell(self.planner.goal) if self.planner.geometry and self.planner.goal else None,
                            'goal_occupancy': self.planner.data[425] if self.planner.data else None,
                            'map_stamp': self.planner.map_stamp, 'odom_stamp': self.planner.odom_stamp,
                            'clock': self.planner.get_clock().now().nanoseconds * 1e-9,
                            'guidance_stamps': [self.guidance.path_stamp, self.guidance.odom_stamp, self.guidance.grid_stamp, self.guidance.status_stamp],
                            'guard_values': self.guard.values,
                            'paths': [len(p.poses) for p in getattr(self, 'paths', [])[-5:]],
                            'statuses': [(m.status[0].level,m.status[0].message) for m in getattr(self, 'statuses', [])[-5:]]})
        self.assertTrue(predicate(), 'ROS condition was not met before timeout: ' + details)

    def status(self, publisher, name, stamp=None):
        msg = DiagnosticArray()
        msg.header.frame_id = 'map'
        msg.header.stamp = stamp or self.driver.get_clock().now().to_msg()
        record = DiagnosticStatus()
        record.name, record.level = name, DiagnosticStatus.OK
        msg.status = [record]
        publisher.publish(msg)

    def authority(self, stamp=None):
        self.status(self.mission, 'mission', stamp)
        self.status(self.navigation, 'navigation', stamp)
        self.race.publish(Bool(data=True))

    def assert_zero(self):
        self.assertGreaterEqual(len(self.outputs), 3)
        self.assertTrue(all(values == (0.0, 0.0) for values in self.outputs[-3:]), self.outputs[-5:])


class PlannerGuidanceTransportTests(RosCase):
    def setUp(self):
        super().setUp()
        self.planner = self.add(Planner())
        self.guidance = self.add(Guidance())
        self.guard = self.add(CommandGuard())
        # Exercise production's simulation-time contract. WSL wall-clock
        # corrections must not manufacture stale/future sensor acquisitions.
        # The guard still enforces its independent steady-clock watchdog.
        for node in (self.driver, self.planner, self.guidance, self.guard):
            result = node.set_parameters([Parameter('use_sim_time', value=True)])
            self.assertTrue(result[0].successful)
        self.clock_pub = self.driver.create_publisher(Clock, '/clock', 1)
        self.clock_origin = time.monotonic()

        def tick_clock():
            elapsed_ns = int((time.monotonic() - self.clock_origin) * 1e9)
            clock = Clock()
            clock.clock.sec = 1000 + elapsed_ns // 1000000000
            clock.clock.nanosec = elapsed_ns % 1000000000
            self.clock_pub.publish(clock)
            # /clock reaches each node through a separate DDS subscription.
            # Synchronize the test acquisition batch to one delivered instant.
            target_ns = clock.clock.sec * 1000000000 + clock.clock.nanosec
            deadline = time.monotonic() + 1.0
            while any(node.get_clock().now().nanoseconds < target_ns for node in self.nodes):
                self.assertLess(time.monotonic(), deadline, '/clock delivery timed out')
                self.executor.spin_once(timeout_sec=0.001)

        self.clock_tick = tick_clock
        self.grid_pub = self.driver.create_publisher(OccupancyGrid, '/njord/occupancy', 1)
        self.odom_pub = self.driver.create_publisher(Odometry, '/njord/odometry', 1)
        self.goal_pub = self.driver.create_publisher(PoseStamped, '/njord/goal', 1)
        self.paths, self.statuses = [], []
        self.driver.create_subscription(RosPath, '/njord/path', self.paths.append, 10)
        self.driver.create_subscription(DiagnosticArray, '/njord/planner_status', self.statuses.append, 10)
        self.blocked = False
        self.odom_age = 0.0
        self.until(lambda: self.grid_pub.get_subscription_count() >= 2
                   and self.goal_pub.get_subscription_count() >= 1
                   and all(node.get_clock().now().nanoseconds >= 1000000000000
                           for node in (self.driver, self.planner, self.guidance, self.guard)))
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.driver.get_clock().now().to_msg()
        goal.pose.orientation.w = 1.0
        goal.pose.position.x, goal.pose.position.y = 25.5, 10.5
        self.goal_message = goal
        self.goal_pub.publish(goal)

    def inputs(self):
        stamp = self.driver.get_clock().now().to_msg()
        grid = OccupancyGrid()
        grid.header.frame_id, grid.header.stamp = 'map', stamp
        grid.info.width = grid.info.height = 40
        grid.info.resolution, grid.info.origin.orientation.w = 1.0, 1.0
        grid.data = [0] * 1600
        if self.blocked:
            grid.data[10 * 40 + 25] = 100
        self.grid_pub.publish(grid)
        odom = Odometry()
        odom.header.frame_id, odom.child_frame_id = 'map', 'wamv/base_link'
        odom.header.stamp = self.driver.get_clock().now().to_msg()
        odom.header.stamp.sec -= int(self.odom_age)
        odom.pose.pose.position.x, odom.pose.pose.position.y = 10.5, 10.5
        odom.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(odom)
        self.authority()

    def moving(self):
        return bool(self.outputs) and self.outputs[-1][0] > 1 and self.outputs[-1][1] > 1

    def test_blocked_goal_publishes_empty_path_zeroes_actuators_and_recovers(self):
        self.until(self.moving, self.inputs)
        self.assertTrue(self.paths[-1].poses)
        self.blocked = True
        # Path and diagnostic messages arrive on separate DDS subscriptions;
        # observing one does not establish delivery of the other under load.
        self.until(lambda: self.paths and not self.paths[-1].poses and self.statuses
                   and self.statuses[-1].status[0].level == DiagnosticStatus.ERROR,
                   self.inputs)
        self.pump(0.2, self.inputs)
        self.assertFalse(self.paths[-1].poses)
        self.assertEqual(self.statuses[-1].status[0].level, DiagnosticStatus.ERROR)
        self.assert_zero()
        self.blocked = False
        self.until(self.moving, self.inputs)
        self.assertTrue(self.paths[-1].poses)

    def test_republished_old_odometry_cannot_freshen_commands(self):
        self.until(self.moving, self.inputs)
        self.odom_age = 3.0
        self.pump(0.5, self.inputs)
        self.assertFalse(self.paths[-1].poses)
        self.assert_zero()

    def test_identical_goal_heartbeat_preserves_valid_path(self):
        self.until(self.moving, self.inputs)
        self.paths.clear()
        self.statuses.clear()

        def heartbeat():
            self.inputs()
            self.goal_message.header.stamp = self.driver.get_clock().now().to_msg()
            self.goal_pub.publish(self.goal_message)

        self.pump(0.6, heartbeat)
        self.assertTrue(self.paths)
        self.assertTrue(self.statuses)
        self.assertTrue(all(path.poses for path in self.paths), [(m.status[0].level,m.status[0].message) for m in self.statuses])
        self.assertTrue(all(message.status[0].level == DiagnosticStatus.OK for message in self.statuses))
        self.assertTrue(self.moving())

    def test_guidance_process_loss_expires_commands(self):
        self.until(self.moving, self.inputs)
        self.executor.remove_node(self.guidance)
        self.nodes.remove(self.guidance)
        self.guidance.destroy_node()
        self.pump(0.85, self.inputs)
        self.assert_zero()


class GuardSteadyClockTests(RosCase):
    def test_source_timeout_with_frozen_ros_clock(self):
        guard = self.add(CommandGuard())
        result = guard.set_parameters([Parameter('use_sim_time', value=True)])
        self.assertTrue(result[0].successful)
        clock_pub = self.driver.create_publisher(Clock, '/clock', 1)
        left = self.driver.create_publisher(Float64, '/njord/thrusters/left/thrust', 1)
        right = self.driver.create_publisher(Float64, '/njord/thrusters/right/thrust', 1)
        planner = self.driver.create_publisher(DiagnosticArray, '/njord/planner_status', 1)
        clock = Clock()
        clock.clock.sec = 123

        def feed_clock():
            clock_pub.publish(clock)

        def feed():
            feed_clock()
            left.publish(Float64(data=80.0))
            right.publish(Float64(data=100.0))
            self.status(planner, 'njord/planner', clock.clock)
            self.authority(clock.clock)

        self.until(lambda: guard.get_clock().now().nanoseconds == 123000000000, feed_clock)
        self.until(lambda: self.outputs and self.outputs[-1] == (80.0, 100.0), feed)
        self.pump(0.85, feed_clock)
        self.assertEqual(guard.get_clock().now().nanoseconds, 123000000000)
        self.assert_zero()


class MapperTransportTests(RosCase):
    def test_timestamped_tf_cloud_obstacle_and_map_heartbeat(self):
        mapper = self.add(Mapper())
        publisher = self.driver.create_publisher(PointCloud2,
                    '/wamv/sensors/lidars/lidar_wamv_sensor/points', 1)
        grids = []
        self.driver.create_subscription(OccupancyGrid, '/njord/occupancy', grids.append, 10)
        broadcaster = StaticTransformBroadcaster(self.driver)
        boat = TransformStamped()
        boat.header.frame_id, boat.child_frame_id = 'map', 'wamv/base_link'
        boat.header.stamp = self.driver.get_clock().now().to_msg()
        boat.transform.translation.x = boat.transform.translation.y = 10.5
        boat.transform.rotation.w = 1.0
        lidar = TransformStamped()
        lidar.header.frame_id, lidar.child_frame_id = 'wamv/base_link', 'smoke_lidar'
        lidar.header.stamp = boat.header.stamp
        lidar.transform.translation.z, lidar.transform.rotation.w = 2.0, 1.0
        broadcaster.sendTransform([boat, lidar])
        self.until(lambda: mapper.tf.can_transform('map', 'smoke_lidar', rclpy.time.Time())
                           and publisher.get_subscription_count() >= 1)
        header = Header()
        header.frame_id, header.stamp = 'smoke_lidar', self.driver.get_clock().now().to_msg()
        cloud = point_cloud2.create_cloud_xyz32(header, [(10.0, 0.0, -1.0)])
        publisher.publish(cloud)
        self.until(lambda: bool(grids))
        grid = grids[-1]

        def at(x, y):
            col = math.floor((x - grid.info.origin.position.x) / grid.info.resolution)
            row = math.floor((y - grid.info.origin.position.y) / grid.info.resolution)
            return grid.data[row * grid.info.width + col]

        self.assertEqual(at(20.5, 10.5), 100)
        self.assertEqual(at(12.5, 10.5), 0)
        self.assertEqual(at(12.5, 25.5), -1)
        self.pump(0.5)
        self.assertEqual(grids[-1].header.stamp, header.stamp)


if __name__ == '__main__':
    unittest.main(verbosity=2)
