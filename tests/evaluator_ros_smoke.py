"""Real DDS regression for evaluator readiness and contact-stream failure."""
import json
import os
from pathlib import Path
import sys
import tempfile
import time
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "njord_sim"))
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from ros_gz_interfaces.msg import Contacts
from std_msgs.msg import Bool
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

            def feed(level=DiagnosticStatus.OK, with_contacts=True):
                stamp = driver.get_clock().now().to_msg()
                truth = Odometry()
                truth.header.stamp, truth.header.frame_id = stamp, "world"
                truth.child_frame_id, truth.pose.pose.orientation.w = "wamv/base_link", 1.0
                odom.publish(truth)
                if with_contacts:
                    message = Contacts()
                    message.header.stamp = stamp
                    contacts.publish(message)
                for name, pub in status_publishers:
                    message = DiagnosticArray()
                    message.header.stamp = stamp
                    record = DiagnosticStatus()
                    record.name, record.level = name, level
                    message.status = [record]
                    pub.publish(message)

            def pump(duration, level=DiagnosticStatus.OK, with_contacts=True):
                end = time.monotonic() + duration
                while time.monotonic() < end:
                    feed(level, with_contacts)
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
                pump(0.8)
                self.assertTrue(evaluator.started)
                self.assertTrue(any(states))
                self.assertFalse(evaluator.done)
                pump(0.9, with_contacts=False)
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


if __name__ == "__main__":
    unittest.main(verbosity=2)
