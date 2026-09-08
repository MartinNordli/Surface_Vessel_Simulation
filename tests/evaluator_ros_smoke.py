"""Real DDS regression for evaluator readiness and contact-stream failure."""
import json
import os
from pathlib import Path
import sys
import tempfile
import time
import unittest
from types import SimpleNamespace
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "njord_sim"))
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from ros_gz_interfaces.msg import Contacts
from std_msgs.msg import Bool, Float64
from builtin_interfaces.msg import Time
from test_sensor_runtime import local_node
from njord_sim.evaluator_node import Evaluator

ROOT = Path(__file__).resolve().parents[1]


class EvaluatorReadinessTests(unittest.TestCase):
    def test_diagnostic_ok_byte_starts_race_and_contact_loss_stops_it(self):
        with tempfile.TemporaryDirectory(prefix="njord-evaluator-") as output:
            result = Path(output) / "metrics.json"
            rclpy.init(args=["--ros-args", "-p", f"scenario_file:={ROOT / 'scenarios/reference.yaml'}",
                            "-p", f"output:={result}"])
            driver = Node("evaluator_smoke_driver")
            evaluator = Evaluator()
            executor = SingleThreadedExecutor()
            executor.add_node(driver)
            executor.add_node(evaluator)
            states = []
            driver.create_subscription(Bool, "/njord/race_active", lambda m: states.append(m.data),
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            odom = driver.create_publisher(Odometry, "/wamv/ground_truth/odometry", 10)
            contacts = driver.create_publisher(Contacts, "/njord/contacts", 10)
            topics = [("mission", "/njord/mission_status"), ("navigation", "/njord/navigation_status"),
                      ("njord/planner", "/njord/planner_status")]
            status_publishers = [(name, driver.create_publisher(DiagnosticArray, topic, 10))
                                 for name, topic in topics]

            def feed(level=DiagnosticStatus.OK, with_contacts=True, contact_delay_s=0):
                stamp = driver.get_clock().now().to_msg()
                truth = Odometry()
                truth.header.stamp, truth.header.frame_id = stamp, "world"
                truth.child_frame_id, truth.pose.pose.orientation.w = "wamv/base_link", 1.0
                odom.publish(truth)
                if with_contacts:
                    message = Contacts()
                    message.header.stamp = Time(sec=stamp.sec-contact_delay_s, nanosec=stamp.nanosec)
                    contacts.publish(message)
                for name, pub in status_publishers:
                    message = DiagnosticArray()
                    message.header.stamp = stamp
                    record = DiagnosticStatus()
                    record.name, record.level = name, level
                    message.status = [record]
                    pub.publish(message)

            def pump(duration, level=DiagnosticStatus.OK, with_contacts=True, contact_delay_s=0):
                end = time.monotonic() + duration
                while time.monotonic() < end:
                    feed(level, with_contacts, contact_delay_s)
                    for _ in range(15):
                        executor.spin_once(timeout_sec=0.001)
                    time.sleep(0.01)

            try:
                # Actual generated ROS byte constants must be used, not integer0.
                self.assertIsInstance(DiagnosticStatus.OK, bytes)
                self.assertNotEqual(DiagnosticStatus.OK, 0)
                pump(0.7, DiagnosticStatus.ERROR)
                self.assertFalse(evaluator.started)
                self.assertFalse(any(states))
                pump(0.6, DiagnosticStatus.ERROR, with_contacts=False)
                pump(0.7, contact_delay_s=2)
                self.assertFalse(evaluator.started)  # Receipt freshness is insufficient.
                pump(0.8)
                self.assertTrue(evaluator.started)
                self.assertTrue(any(states))
                self.assertFalse(evaluator.done)
                pump(0.9, contact_delay_s=2)
                self.assertTrue(evaluator.done)
                self.assertFalse(states[-1])
                metrics = json.loads(result.read_text())
                self.assertEqual(metrics["status"], "contact_monitor_timeout")
                self.assertEqual(evaluator.exit_code, 2)
            finally:
                executor.remove_node(evaluator)
                executor.remove_node(driver)
                evaluator.destroy_node()
                driver.destroy_node()
                executor.shutdown()
                rclpy.shutdown()


class EvaluatorCallbackTests(unittest.TestCase):
    """Actual ROS messages; local doubles keep targeted checks free of DDS."""
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix="njord-evaluator-callback-")
        self.addCleanup(temporary.cleanup)
        self.result = Path(temporary.name)/"metrics.json"
        context = local_node(Evaluator, {
            "scenario_file": str(ROOT/"scenarios/reference.yaml"), "output": str(self.result)})
        self.node, self.clock, self.publishers = context.__enter__()
        self.addCleanup(context.__exit__, None, None, None)
        logger = patch.object(Node, "get_logger", return_value=SimpleNamespace(info=lambda _: None))
        logger.start()
        self.addCleanup(logger.stop)
        self.clock.seconds = 100.
        truth = Odometry()
        truth.header.stamp = self.clock.now().to_msg()
        truth.pose.pose.orientation.w = 1.
        self.node.on_odom(truth)
        for name in ("mission", "navigation", "njord/planner"):
            message = DiagnosticArray()
            message.header.stamp = self.clock.now().to_msg()
            message.status = [DiagnosticStatus(name=name, level=DiagnosticStatus.OK)]
            self.node.on_readiness(message)

    def contact(self, seconds):
        ns = round(seconds*1e9)
        message = Contacts()
        message.header.stamp = Time(sec=ns//10**9, nanosec=ns%10**9)
        self.node.on_contacts(message)

    def test_old_and_future_contacts_cannot_start_or_count_as_evidence(self):
        for stamp in (1., 99.49, 101.):
            self.contact(stamp)
            self.assertFalse(self.node.ready())
            self.assertEqual(self.node.scorer.contact_messages, 0)
        self.contact(100.)
        self.assertTrue(self.node.ready())
        self.assertEqual(self.node.scorer.contact_messages, 1)

    def test_repeated_stamp_does_not_refresh_receipt_watchdog(self):
        self.contact(100.)
        received = self.node.contact_last_wall
        with patch("njord_sim.evaluator_node.time.monotonic", return_value=received+0.6):
            self.contact(100.)
            self.assertEqual(self.node.contact_last_wall, received)
            self.assertEqual(self.node.scorer.contact_messages, 1)
            self.assertFalse(self.node.contact_fresh())

    def test_sim_time_expiry_stops_race_despite_recent_wall_receipt(self):
        self.contact(100.)
        self.node.check_timeout()
        self.assertTrue(self.node.started)
        self.clock.seconds = 100.6
        self.node.check_timeout()
        self.assertTrue(self.node.done)
        self.assertEqual(json.loads(self.result.read_text())["status"], "contact_monitor_timeout")
        self.assertFalse(self.publishers["/njord/race_active"].messages[-1].data)

    def test_completion_between_timer_ticks_requires_fresh_contacts(self):
        self.contact(100.)
        self.clock.seconds = 100.6
        self.node.scorer.status = "completed"
        self.node.finish()
        self.assertEqual(self.node.exit_code, 2)
        self.assertEqual(json.loads(self.result.read_text())["status"], "contact_monitor_timeout")

    def test_path_messages_and_computations_are_counted_separately(self):
        for _ in range(5):
            self.node.on_path(None)
        for latency in (1., 2., float("nan"), -1.):
            self.node.on_latency(Float64(data=latency))
        self.node.scorer.status = "interrupted"
        self.node.finish()
        metrics = json.loads(self.result.read_text())
        self.assertEqual(metrics["path_messages"], 5)
        self.assertEqual(metrics["replans"], 2)
        self.assertEqual(metrics["plan_samples"], 2)


if __name__ == "__main__":
    unittest.main(verbosity=2)
