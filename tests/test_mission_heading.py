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
from njord_sim.perception_core import choose_gate


def pose_message():
    return NS(header=NS(frame_id='', stamp=None),
              pose=NS(position=NS(x=0., y=0., z=0.), orientation=NS(x=0., y=0., z=0., w=1.)))


class MissionHeadingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        spec = importlib.util.spec_from_file_location('mission_heading_under_test',
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

    def setUp(self):
        self.node = self.Mission.__new__(self.Mission)
        self.params = dict(map_frame='map', initial_heading_rad=0., expected_gates=3,
                           min_gate_width_m=8., max_gate_width_m=30., approach_m=5., exit_m=6.,
                           arrival_tolerance_m=2., detection_max_age_s=2., camera_max_age_s=0.5,
                           odometry_max_age_s=0.5, crossing_memory_s=45., crossing_entry_m=15.)
        self.node.p = self.params.__getitem__
        self.node.heading = 0.
        self.node.gate = self.node.gate_seen = None
        self.node.position = self.node.odom_stamp = self.node.camera_stamp = None
        self.node.passed, self.node.observations = [], {}
        self.node.phase, self.node.crossed, self.node.previous_signed = 'seek', False, None
        self.node.last_clock = None
        self.now = 10.
        self.node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=int(self.now*1e9), to_msg=lambda: NS()))
        self.statuses, self.goals = [], []
        self.node.status = lambda level, message: self.statuses.append((level, message))
        self.node.goal_pub = NS(publish=self.goals.append)

    def odom(self, yaw, stamp=10., point=(0., 0.)):
        return NS(header=NS(frame_id='map', stamp=NS(sec=int(stamp), nanosec=int((stamp-int(stamp))*1e9))),
                  pose=NS(pose=NS(position=NS(x=point[0], y=point[1]),
                                 orientation=NS(x=0., y=0., z=math.sin(yaw/2), w=math.cos(yaw/2)))))

    def test_first_gate_uses_measured_north_heading(self):
        self.node.on_odom(self.odom(math.pi/2))
        self.assertAlmostEqual(self.node.heading, math.pi/2)
        self.assertIsNotNone(choose_gate([('red', (-7, 20)), ('green', (7, 20))],
                                         self.node.position, self.node.heading))

    def test_invalid_orientation_does_not_initialize_navigation(self):
        for orientation in (NS(x=0., y=0., z=0., w=0.), NS(x=0., y=0., z=math.nan, w=1.)):
            message = self.odom(math.pi/2)
            message.pose.pose.orientation = orientation
            self.node.on_odom(message)
            self.assertIsNone(self.node.odom_stamp)
            self.assertIsNone(self.node.position)

    def test_selected_and_passed_gate_heading_is_retained(self):
        self.node.heading, self.node.gate = 0.4, object()
        self.node.on_odom(self.odom(math.pi/2))
        self.assertEqual(self.node.heading, 0.4)
        self.node.gate, self.node.passed = None, [np.array([10., 0.])]
        self.node.on_odom(self.odom(math.pi, 11.))
        self.assertEqual(self.node.heading, 0.4)

    def test_camera_fov_loss_at_ten_metres_allows_bounded_remembered_approach(self):
        self.node.on_odom(self.odom(0., point=(38., 0.79)))
        self.node.camera_stamp = 10.
        self.node.observations = {'red': ('red', np.array([50., 10.]), 10.),
                                  'green': ('green', np.array([50., -4.]), 10.)}
        self.node.step()
        self.assertEqual(self.statuses[-1], (0, 'valid'))
        self.now = 13.
        self.node.on_odom(self.odom(0., 13., (40., 0.79)))
        self.node.camera_stamp = 13.  # Fresh images, but the gate left their FOV.
        self.node.step()
        self.assertEqual(self.node.observations, {})
        self.assertEqual(self.node.phase, 'approach')
        self.assertEqual(self.statuses[-1], (0, 'valid'))
        self.assertEqual(self.goals[-1].pose.position.x, 45.)
        self.now = 56.
        self.node.on_odom(self.odom(0., 56., (40., 0.79)))
        self.node.camera_stamp = 56.
        self.node.step()
        self.assertEqual(self.statuses[-1], (2, 'tracked gate expired'))

    def test_memory_still_requires_fresh_images_and_bounded_lateral_position(self):
        self.node.on_odom(self.odom(0., point=(38., 0.79)))
        self.node.camera_stamp = 10.
        self.node.observations = {'red': ('red', np.array([50., 10.]), 10.),
                                  'green': ('green', np.array([50., -4.]), 10.)}
        self.node.step()
        self.now = 13.
        self.node.on_odom(self.odom(0., 13., (40., 0.79)))
        self.node.step()
        self.assertEqual(self.statuses[-1], (2, 'camera fusion unavailable or stale'))
        self.node.camera_stamp = 13.
        self.node.position = np.array([40., -5.])
        self.node.step()
        self.assertEqual(self.statuses[-1], (2, 'tracked gate expired'))


if __name__ == '__main__':
    unittest.main()
