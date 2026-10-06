"""Real DDS regression for evaluator readiness and contact-stream failure."""
import json
import math
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
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from ros_gz_interfaces.msg import Contact, Contacts, Entity
from std_msgs.msg import Bool, Float64
from builtin_interfaces.msg import Time
from test_sensor_runtime import local_node
from njord_sim.constants import GZ_MODEL_NAME, PROCESS_LIVENESS_S
from njord_sim.evaluator_node import Evaluator
from njord_sim.scenario_core import wall_budget_s

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
            odom = driver.create_publisher(Odometry, "/sim/ground_truth/odometry", 10)
            contacts = driver.create_publisher(Contacts, "/njord/contacts", 10)
            topics = [("mission", "/njord/mission_status"), ("navigation", "/njord/navigation_status"),
                      ("njord/planner", "/njord/planner_status")]
            status_publishers = [(name, driver.create_publisher(DiagnosticArray, topic, 10))
                                 for name, topic in topics]

            def feed(level=DiagnosticStatus.OK, with_contacts=True, contact_delay_s=0):
                stamp = driver.get_clock().now().to_msg()
                truth = Odometry()
                truth.header.stamp, truth.header.frame_id = stamp, "world"
                truth.child_frame_id, truth.pose.pose.orientation.w = "base_link", 1.0
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


class ExternalAutonomyReadinessTests(unittest.TestCase):
    """An external stack's race starts on navigation alone (AUTONOMY=external)."""
    def test_only_required_heartbeats_gate_the_start(self):
        with tempfile.TemporaryDirectory(prefix="njord-evaluator-external-") as temporary:
            context = local_node(Evaluator, {"scenario_file": str(ROOT/"scenarios/reference.yaml"),
                                             "output": str(Path(temporary)/"metrics.json"),
                                             "required_status": ["navigation"]})
            node, clock, _ = context.__enter__()
            try:
                clock.seconds = 100.
                truth = Odometry()
                truth.header.stamp = clock.now().to_msg()
                truth.pose.pose.orientation.w = 1.
                node.on_odom(truth)
                contacts = Contacts()
                contacts.header.stamp = Time(sec=100, nanosec=0)
                node.on_contacts(contacts)
                self.assertFalse(node.ready())
                message = DiagnosticArray()
                message.header.stamp = clock.now().to_msg()
                message.status = [DiagnosticStatus(name="navigation", level=DiagnosticStatus.OK)]
                node.on_readiness(message)
                self.assertTrue(node.ready())
            finally:
                context.__exit__(None, None, None)


class WallBudgetTests(unittest.TestCase):
    def test_budget_leaves_room_for_the_simulation_timeout_at_slow_factors(self):
        self.assertEqual(wall_budget_s(600, 360, 1.0), 720)
        self.assertAlmostEqual(wall_budget_s(600, 360, 0.3), 2400)
        self.assertEqual(wall_budget_s(600, 100, 3.0), 600)


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

    def contact(self, seconds, pairs=()):
        """Deliver a contact message; ``pairs`` are scoped collision names."""
        ns = round(seconds*1e9)
        message = Contacts()
        message.header.stamp = Time(sec=ns//10**9, nanosec=ns%10**9)
        message.contacts = [Contact(collision1=Entity(name=first), collision2=Entity(name=second))
                            for first, second in pairs]
        self.node.on_contacts(message)

    def test_only_contacts_of_the_vessel_model_count_as_collision(self):
        # Scoped names as Gazebo reports them: model::link::collision.
        self.contact(99.9, [("obstacle_1::link::collision", "ground_plane::link::collision"),
                            ("obstacle_1::link::collision", f"not_{GZ_MODEL_NAME}::link::collision")])
        self.assertEqual(self.node.scorer.contact_events, 0)
        self.assertFalse(self.node.done)
        self.contact(100., [("obstacle_1::link::collision",
                               f"{GZ_MODEL_NAME}::base_link::base_link_fixed_joint_lump__top_base_collision_24")])
        self.assertEqual(self.node.scorer.contact_events, 1)
        self.assertEqual(self.node.scorer.status, "collision")
        self.assertTrue(self.node.done)

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
        with patch("njord_sim.evaluator_node.time.monotonic", return_value=received+PROCESS_LIVENESS_S+0.1):
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

    def test_clock_rollback_ends_scoring_and_replayed_odom_is_ignored(self):
        previous = self.node.latest_ground_truth
        truth = Odometry()
        truth.header.stamp = Time(sec=99)
        truth.pose.pose.orientation.w = 1.
        self.node.on_odom(truth)
        self.assertEqual(self.node.latest_ground_truth, previous)
        self.contact(100.)
        self.node.check_timeout()
        self.clock.seconds = 90.
        self.node.check_timeout()
        self.assertEqual(json.loads(self.result.read_text())["status"], "clock_reset")

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


def truth(stamp_s, x=0.0, y=0.0, yaw=0.0):
    """Ground-truth odometry at simulation time ``stamp_s``."""
    message = Odometry()
    ns = round(stamp_s * 1e9)
    message.header.stamp = Time(sec=ns // 10**9, nanosec=ns % 10**9)
    message.pose.pose.position.x, message.pose.pose.position.y = x, y
    message.pose.pose.orientation.z, message.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return message


class SetpointEvaluatorTests(unittest.TestCase):
    """Referee and observer modes on a setpoint course, without DDS."""
    def evaluator(self, **overrides):
        temporary = tempfile.TemporaryDirectory(prefix="njord-evaluator-setpoint-")
        self.addCleanup(temporary.cleanup)
        self.output = Path(temporary.name)
        context = local_node(Evaluator, {"scenario_file": str(ROOT/"scenarios/goto_square.yaml"),
                                         "output": str(self.output/"run_metrics.json"), **overrides})
        node, clock, publishers = context.__enter__()
        self.addCleanup(context.__exit__, None, None, None)
        logger = patch.object(Node, "get_logger", return_value=SimpleNamespace(
            info=lambda _: None, warning=lambda _: None))
        logger.start()
        self.addCleanup(logger.stop)
        self.subscribers = 1
        listeners = patch.object(Node, "count_subscribers", lambda _, topic: self.subscribers, create=True)
        listeners.start()
        self.addCleanup(listeners.stop)
        return node, clock, publishers

    def contacts(self, node, seconds):
        message = Contacts()
        message.header.stamp = Time(sec=int(seconds), nanosec=round(seconds % 1 * 1e9))
        node.on_contacts(message)

    def test_referee_issues_targets_in_order_and_writes_the_report(self):
        node, clock, publishers = self.evaluator(required_status=["navigation", "controller"])
        targets = node.scenario["setpoints"]
        clock.seconds = 100.
        node.on_odom(truth(100.))
        self.contacts(node, 100.)
        for name in ("navigation", "controller"):
            message = DiagnosticArray()
            message.header.stamp = clock.now().to_msg()
            message.status = [DiagnosticStatus(name=name, level=DiagnosticStatus.OK)]
            node.on_readiness(message)
        self.subscribers = 0
        node.check_timeout()
        self.assertFalse(node.started)  # nobody listens for targets yet
        self.subscribers = 1
        node.check_timeout()
        self.assertTrue(node.started)
        issued = publishers["/njord/setpoint"].messages
        self.assertEqual(len(issued), 1)
        node.repeat_setpoint()  # the 1 Hz repeat keeps the issue stamp
        self.assertIs(issued[-1], issued[0])
        del issued[1:]
        self.assertEqual((issued[0].header.frame_id, issued[0].pose.position.x), ("map", targets[0]["position"][0]))
        sequence = publishers["/njord/setpoint_sequence"].messages[-1]
        self.assertEqual(len(sequence.poses), len(targets))
        # Sit on each target with its heading for longer than the hold.
        t = 100.
        for index, target in enumerate(targets):
            yaw = math.radians(target["heading_deg_enu"])
            for _ in range(int(target["hold_s"] / 0.1) + 2):
                t += 0.1
                clock.seconds = t
                self.contacts(node, t)
                node.on_odom(truth(t, *target["position"], yaw))
            if index + 1 < len(targets):
                self.assertEqual(len(publishers["/njord/setpoint"].messages), index + 2)
        self.assertTrue(node.done)
        self.assertEqual(node.exit_code, 0)
        metrics = json.loads((self.output/"run_metrics.json").read_text())
        self.assertEqual((metrics["status"], metrics["course"]), ("completed", "setpoints"))
        self.assertEqual([s["outcome"] for s in metrics["setpoints"]], ["reached"] * len(targets))
        self.assertTrue((self.output/"timeseries.csv").read_text().startswith("t_s,x_m"))
        report = (self.output/"report.html").read_text()
        self.assertIn("<svg", report)
        self.assertIn(targets[-1]["name"], report)
        self.assertFalse(publishers["/njord/race_active"].messages[-1].data)

    def test_observer_scores_targets_sent_from_outside(self):
        node, clock, publishers = self.evaluator(mode="observe")
        self.assertNotIn("/njord/race_active", publishers)
        self.assertNotIn("/njord/setpoint", publishers)
        clock.seconds = 50.
        node.on_odom(truth(50.))
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.pose.position.x, goal.pose.orientation.w = 3.0, 1.0
        node.on_setpoint(goal)
        node.on_setpoint(goal)  # a duplicate of the same message
        wrong = PoseStamped()
        wrong.header.frame_id = "base_link"
        node.on_setpoint(wrong)
        self.assertEqual(len(node.scorer.tracks), 1)
        t = 50.
        for _ in range(60):
            t += 0.1
            clock.seconds = t
            node.on_odom(truth(t, 3.0, 0.0, 0.0))
        self.assertFalse(node.done)
        interim = json.loads((self.output/"run_metrics.json").read_text())
        self.assertEqual(interim["setpoints"][0]["outcome"], "reached")
        node.check_timeout()  # no watchdog ends an observation
        self.assertFalse(node.done)
        node.interrupt()
        metrics = json.loads((self.output/"run_metrics.json").read_text())
        self.assertEqual((metrics["status"], metrics["mode"], node.exit_code), ("stopped", "observe", 0))
        self.assertTrue((self.output/"report.html").is_file())


if __name__ == "__main__":
    unittest.main(verbosity=2)
