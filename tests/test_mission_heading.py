"""Mission heading and camera-FOV approach regressions without ROS transport."""
import importlib.util
import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'njord_sim'))
from njord_sim.mission_core import ERROR, OK, GateMission
from njord_sim.perception_core import choose_gate

SETTINGS = dict(min_gate_width_m=8., max_gate_width_m=30., approach_m=5., exit_m=6.,
                arrival_tolerance_m=2., detection_max_age_s=2., camera_max_age_s=0.5,
                odometry_max_age_s=0.5, crossing_memory_s=45., crossing_entry_m=15.,
                search_after_s=2., search_radius_m=8., search_goal_s=10., search_timeout_s=60.,
                max_gate_retries=1, retry_clearance_m=6.)


def pose_message():
    return NS(header=NS(frame_id='', stamp=None),
              pose=NS(position=NS(x=0., y=0., z=0.), orientation=NS(x=0., y=0., z=0., w=1.)))


class MissionHeadingTests(unittest.TestCase):
    def setUp(self):
        self.mission = GateMission(3, 0., SETTINGS)

    def test_first_gate_uses_measured_north_heading(self):
        self.mission.odometry((0., 0.), math.pi/2, 10.)
        self.assertAlmostEqual(self.mission.heading, math.pi/2)
        self.assertIsNotNone(choose_gate([('red', (-7, 20)), ('green', (7, 20))],
                                         self.mission.position, self.mission.heading))

    def test_selected_and_passed_gate_heading_is_retained(self):
        self.mission.heading, self.mission.gate = 0.4, object()
        self.mission.odometry((0., 0.), math.pi/2, 10.)
        self.assertEqual(self.mission.heading, 0.4)
        self.mission.gate, self.mission.passed = None, [np.array([10., 0.])]
        self.mission.odometry((0., 0.), math.pi, 11.)
        self.assertEqual(self.mission.heading, 0.4)

    def test_camera_fov_loss_at_ten_metres_allows_bounded_remembered_approach(self):
        mission = self.mission
        mission.odometry((38., 0.79), 0., 10.)
        mission.camera_stamp = 10.
        mission.observations = {'red': ('red', np.array([50., 10.]), 10.),
                                'green': ('green', np.array([50., -4.]), 10.)}
        decision = mission.step(10.)
        self.assertEqual((decision.level, decision.message), (OK, 'valid'))
        mission.odometry((40., 0.79), 0., 13.)
        mission.camera_stamp = 13.  # Fresh images, but the gate left their FOV.
        decision = mission.step(13.)
        self.assertEqual(mission.observations, {})
        self.assertEqual(mission.phase, 'approach')
        self.assertEqual((decision.level, decision.message), (OK, 'valid'))
        self.assertEqual(decision.goal[0], 45.)
        mission.odometry((40., 0.79), 0., 56.)
        mission.camera_stamp = 56.
        decision = mission.step(56.)
        self.assertEqual((decision.level, decision.message), (ERROR, 'tracked gate expired'))

    def test_memory_still_requires_fresh_images_and_bounded_lateral_position(self):
        mission = self.mission
        mission.odometry((38., 0.79), 0., 10.)
        mission.camera_stamp = 10.
        mission.observations = {'red': ('red', np.array([50., 10.]), 10.),
                                'green': ('green', np.array([50., -4.]), 10.)}
        mission.step(10.)
        mission.odometry((40., 0.79), 0., 13.)
        decision = mission.step(13.)
        self.assertEqual((decision.level, decision.message), (ERROR, 'camera fusion unavailable or stale'))
        mission.camera_stamp = 13.
        mission.position = np.array([40., -5.])
        decision = mission.step(13.)
        self.assertEqual((decision.level, decision.message), (ERROR, 'tracked gate expired'))


class MissionNodeOdometryTests(unittest.TestCase):
    """The node rejects poses the core must never see."""
    @classmethod
    def setUpClass(cls):
        spec = importlib.util.spec_from_file_location('mission_node_under_test',
                Path(__file__).resolve().parents[1] / 'njord_sim/njord_sim/mission_node.py')
        module = importlib.util.module_from_spec(spec)
        doubles = {'rclpy': NS(), 'rclpy.node': NS(Node=object),
                   'diagnostic_msgs': NS(), 'diagnostic_msgs.msg': NS(DiagnosticArray=NS,
                        DiagnosticStatus=NS(OK=0, WARN=1, ERROR=2), KeyValue=NS),
                   'geometry_msgs': NS(), 'geometry_msgs.msg': NS(PoseStamped=pose_message),
                   'nav_msgs': NS(), 'nav_msgs.msg': NS(Odometry=NS),
                   'vision_msgs': NS(), 'vision_msgs.msg': NS(Detection3DArray=NS)}
        with patch.dict(sys.modules, doubles):
            spec.loader.exec_module(module)
        cls.Mission = module.Mission

    def node(self):
        node = self.Mission.__new__(self.Mission)
        node.p = {'map_frame': 'map'}.__getitem__
        node.core = GateMission(3, 0., SETTINGS)
        return node

    @staticmethod
    def odom(x=0., orientation=None, variance=0.03):
        covariance = [0.]*36
        covariance[0] = covariance[7] = variance
        return NS(header=NS(frame_id='map', stamp=NS(sec=10, nanosec=0)),
                  pose=NS(pose=NS(position=NS(x=x, y=0.), orientation=orientation or NS(x=0., y=0., z=0., w=1.)),
                          covariance=covariance))

    def test_invalid_orientation_does_not_initialize_navigation(self):
        node = self.node()
        for orientation in (NS(x=0., y=0., z=0., w=0.), NS(x=0., y=0., z=math.nan, w=1.)):
            node.on_odom(self.odom(orientation=orientation))
            self.assertIsNone(node.core.odom_stamp)
            self.assertIsNone(node.core.position)

    def test_uninitialized_estimate_is_ignored(self):
        # Measured at real-time factor 0.3: ~1e6 m off with a variance of ~300 m^2.
        node = self.node()
        node.on_odom(self.odom(x=106329., variance=298.))
        self.assertIsNone(node.core.position)
        node.on_odom(self.odom(x=0.5))
        self.assertEqual(node.core.position[0], 0.5)


if __name__ == '__main__':
    unittest.main()
