"""Truth mode relays ground truth unchanged and drops anything unusable."""
import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'njord_sim'))
from njord_sim.geometry import valid_odometry


def odometry(frame='map', child='base_link', x=3.0, w=1.0, sec=12):
    return NS(header=NS(frame_id=frame, stamp=NS(sec=sec, nanosec=250000000)), child_frame_id=child,
              pose=NS(pose=NS(position=NS(x=x, y=-4.0, z=0.1), orientation=NS(x=0.0, y=0.0, z=0.0, w=w))),
              twist=NS(twist=NS(linear=NS(x=1.5, y=0.0, z=0.0), angular=NS(x=0.0, y=0.0, z=0.2))))


class ValidOdometryTests(unittest.TestCase):
    def test_accepts_expected_frames_and_rejects_everything_else(self):
        self.assertTrue(valid_odometry(odometry(), 'map', 'base_link'))
        for bad in (odometry(frame='odom'), odometry(child='wamv/base_link'), odometry(x=float('nan')),
                    odometry(w=0.5), odometry(sec=float('inf'))):
            self.assertFalse(valid_odometry(bad, 'map', 'base_link'))


class TruthRelayCallbackTests(unittest.TestCase):
    """Run the relay callback with ROS transport replaced by small doubles."""

    @classmethod
    def setUpClass(cls):
        class Transform:
            def __init__(self):
                self.header = NS(stamp=None, frame_id='', child_frame_id='')
                self.transform = NS(translation=NS(x=0.0, y=0.0, z=0.0), rotation=None)
        doubles = {'rclpy': NS(), 'rclpy.node': NS(Node=object),
                   'rclpy.qos': NS(qos_profile_sensor_data=object()),
                   'geometry_msgs': NS(), 'geometry_msgs.msg': NS(TransformStamped=Transform),
                   'nav_msgs': NS(), 'nav_msgs.msg': NS(Odometry=NS),
                   'tf2_ros': NS(StaticTransformBroadcaster=object, TransformBroadcaster=object)}
        source = ROOT / 'njord_sim/njord_sim/truth_relay_node.py'
        spec = importlib.util.spec_from_file_location('truth_relay_callback_test', source)
        module = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, doubles):
            spec.loader.exec_module(module)
        cls.TruthRelay = module.TruthRelay

    def setUp(self):
        self.node = self.TruthRelay.__new__(self.TruthRelay)
        parameters = {'map_frame': 'map', 'odom_frame': 'odom', 'base_frame': 'base_link'}
        self.node.p = parameters.__getitem__
        self.published, self.transforms = [], []
        self.node.pub = NS(publish=self.published.append)
        self.node.tf = NS(sendTransform=self.transforms.append)

    def test_relays_the_same_message_and_its_acquisition_stamp(self):
        message = odometry()
        self.node.relay(message)
        self.assertEqual(self.published, [message])  # unchanged, never re-stamped
        transform = self.transforms[0]
        self.assertIs(transform.header.stamp, message.header.stamp)
        self.assertEqual((transform.header.frame_id, transform.child_frame_id), ('odom', 'base_link'))
        self.assertEqual(transform.transform.translation.x, 3.0)

    def test_invalid_truth_is_dropped_so_navigation_goes_stale(self):
        self.node.relay(odometry(x=float('nan')))
        self.node.relay(odometry(frame='odom'))
        self.assertEqual((self.published, self.transforms), ([], []))


if __name__ == '__main__':
    unittest.main()
