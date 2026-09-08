"""Real ROS messages and node methods with transport replaced by local doubles.

No DDS context or sockets are opened. These regressions test callback/state logic,
not sensor rendering, middleware delivery or physical race completion.
"""
from contextlib import contextmanager, ExitStack
from pathlib import Path
from types import SimpleNamespace
import sys
import unittest
from unittest.mock import patch
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/"njord_sim"))
try:
    from builtin_interfaces.msg import Time
    from diagnostic_msgs.msg import DiagnosticStatus
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import NavSatFix, Imu
    from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose
    from rclpy.node import Node
    from njord_sim.mission_node import Mission
    from njord_sim.sensor_adapter_node import SensorAdapter
    HAS_ROS = True
except ImportError:
    HAS_ROS = False


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class TestClock:
    def __init__(self):
        self.seconds = 10.

    def now(self):
        ns = round(self.seconds*1e9)
        return SimpleNamespace(nanoseconds=ns, to_msg=lambda: Time(sec=ns//10**9, nanosec=ns%10**9))


@contextmanager
def local_node(constructor, overrides=None):
    """Keep actual constructor defaults, publishers, messages and callbacks."""
    parameters, publishers = {}, {}
    clock = TestClock()

    def declare(node, namespace, values):
        parameters.update(values)
        parameters.update(overrides or {})

    def publisher(node, message_type, topic, qos, **kwargs):
        publishers[topic] = Publisher()
        return publishers[topic]

    with ExitStack() as stack:
        for name, replacement in {
            "__init__": lambda self, *args, **kwargs: None,
            "declare_parameters": declare,
            "get_parameter": lambda self, name: SimpleNamespace(value=parameters[name]),
            "create_publisher": publisher,
            "create_subscription": lambda *args, **kwargs: None,
            "create_timer": lambda *args, **kwargs: None,
            "get_clock": lambda self: clock,
        }.items():
            stack.enter_context(patch.object(Node, name, replacement))
        yield constructor(), clock, publishers


@unittest.skipUnless(HAS_ROS, "ROS messages unavailable; run with sourced Jazzy /usr/bin/python3")
class SensorAdapterTests(unittest.TestCase):
    @staticmethod
    def fix():
        msg = NavSatFix()
        msg.header.frame_id = "wamv/gps_wamv_link"
        msg.header.stamp.sec = 123
        msg.header.stamp.nanosec = 456
        msg.status.status = 0
        msg.latitude, msg.longitude, msg.altitude = 63., 10., 0.
        return msg

    @staticmethod
    def ecef(latitude, longitude, altitude):
        """Independent ellipsoid-coordinate check of geographic output units."""
        lat, lon = np.radians(latitude), np.radians(longitude)
        flattening = 1./298.257223563
        e2 = flattening*(2.-flattening)
        n = 6378137./np.sqrt(1.-e2*np.sin(lat)**2)
        return np.array([(n+altitude)*np.cos(lat)*np.cos(lon),
                         (n+altitude)*np.cos(lat)*np.sin(lon),
                         ((1.-e2)*n+altitude)*np.sin(lat)])

    def test_gps_noise_is_metres_at_trondheim_latitude(self):
        with local_node(SensorAdapter) as (node, _, pubs):
            raw = self.fix()
            baseline = self.ecef(raw.latitude, raw.longitude, raw.altitude)
            lat, lon = np.radians([raw.latitude, raw.longitude])
            enu = np.array([[-np.sin(lon), np.cos(lon), 0.],
                            [-np.sin(lat)*np.cos(lon), -np.sin(lat)*np.sin(lon), np.cos(lat)],
                            [np.cos(lat)*np.cos(lon), np.cos(lat)*np.sin(lon), np.sin(lat)]])
            for _ in range(4000):
                node.gps(raw)
            messages = pubs['/wamv/sensors/gps/gps/fix'].messages
            errors = np.array([enu@(self.ecef(m.latitude, m.longitude, m.altitude)-baseline) for m in messages])
            np.testing.assert_allclose(np.std(errors, axis=0), [.3, .3, .5], rtol=.05)
            np.testing.assert_allclose(np.mean(errors, axis=0), [0., 0., 0.], atol=.025)
            self.assertEqual((raw.latitude, raw.longitude, raw.altitude), (63., 10., 0.))
            self.assertTrue(all(m.header == raw.header for m in messages))
            np.testing.assert_allclose(messages[-1].position_covariance, [.09,0,0,0,.09,0,0,0,.25])

    def test_imu_callback_rate_does_not_change_gps_noise_stream(self):
        raw = self.fix()
        sequences = []
        for imu_count in (0, 100):
            with local_node(SensorAdapter) as (node, _, pubs):
                imu = Imu()
                imu.orientation.w = 1.
                for _ in range(imu_count):
                    node.imu(imu)
                for _ in range(10):
                    node.gps(raw)
                sequences.append([(m.latitude, m.longitude, m.altitude) for m in pubs['/wamv/sensors/gps/gps/fix'].messages])
        self.assertEqual(sequences[0], sequences[1])

    def test_invalid_fix_is_not_freshened_or_published(self):
        with local_node(SensorAdapter) as (node, _, pubs):
            raw = self.fix()
            raw.status.status = -1
            node.gps(raw)
            self.assertEqual(pubs['/wamv/sensors/gps/gps/fix'].messages, [])
            self.assertNotIn('gps', node.received)


@unittest.skipUnless(HAS_ROS, "ROS messages unavailable; run with sourced Jazzy /usr/bin/python3")
class MissionCrossingTests(unittest.TestCase):
    def update(self, node, clock, x, detections=False, camera=True, odometry=True):
        if odometry:
            msg = Odometry()
            msg.header.frame_id = 'map'
            msg.header.stamp = clock.now().to_msg()
            msg.pose.pose.position.x = float(x)
            msg.pose.pose.orientation.w = 1.
            node.on_odom(msg)
        if camera:
            array = Detection3DArray()
            array.header.frame_id = 'map'
            array.header.stamp = clock.now().to_msg()
            if detections:
                for color, y in [('red', 7.), ('green', -7.)]:
                    detection = Detection3D()
                    detection.id = color
                    detection.bbox.center.position.x = 25.
                    detection.bbox.center.position.y = y
                    result = ObjectHypothesisWithPose()
                    result.hypothesis.class_id = color
                    result.hypothesis.score = .9
                    detection.results = [result]
                    array.detections.append(detection)
            node.on_buoys(array)
        node.step()

    def test_slow_crossing_with_fresh_empty_camera_frames_completes(self):
        with local_node(Mission, {'expected_gates': 1}) as (node, clock, pubs):
            self.update(node, clock, 17., detections=True)
            self.assertEqual(node.p('crossing_memory_s'), 45.)
            for index in range(1, 301):
                clock.seconds = 10.+index/10.
                self.update(node, clock, 17.+14.*index/300.)
                status = pubs['/njord/mission_status'].messages[-1].status[0]
                self.assertNotEqual(status.level, DiagnosticStatus.ERROR, status.message)
            self.assertEqual(len(node.passed), 1)
            self.assertEqual(pubs['/njord/mission_status'].messages[-1].status[0].message, 'complete')
            self.assertEqual(node.gate_seen, 10.)  # Observations were never freshened.

    def test_memory_still_expires_with_live_camera(self):
        with local_node(Mission) as (node, clock, pubs):
            self.update(node, clock, 17., detections=True)
            clock.seconds = 55.1
            self.update(node, clock, 20.)
            status = pubs['/njord/mission_status'].messages[-1].status[0]
            self.assertEqual(status.level, DiagnosticStatus.ERROR)
            self.assertEqual(status.message, 'tracked gate expired')

    def test_remembered_gate_does_not_override_camera_or_odom_failure(self):
        for failed_sensor in ('camera', 'odometry'):
            with self.subTest(sensor=failed_sensor), local_node(Mission) as (node, clock, pubs):
                self.update(node, clock, 17., detections=True)
                clock.seconds = 11.
                self.update(node, clock, 18., camera=failed_sensor != 'camera', odometry=failed_sensor != 'odometry')
                status = pubs['/njord/mission_status'].messages[-1].status[0]
                self.assertEqual(status.level, DiagnosticStatus.ERROR)
                self.assertIn(failed_sensor, status.message)


if __name__ == '__main__':
    unittest.main()
