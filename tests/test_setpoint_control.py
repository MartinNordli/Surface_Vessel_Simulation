"""Reference setpoint controller: allocation, braking, freshness and closed loop.

The closed-loop checks use a simple planar rigid-body model with linear
damping, not Gazebo; they verify the control logic and sign conventions, not
the simulated or real boat's behaviour.
"""
from contextlib import ExitStack, contextmanager
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "njord_sim"))
from njord_sim.configuration import autonomy_parameters, resolve_configuration
from njord_sim.control_core import allocation_matrix
from njord_sim.setpoint_control_core import STATION_KEEPING, TRANSIT, SetpointController

try:
    from builtin_interfaces.msg import Time
    from diagnostic_msgs.msg import DiagnosticStatus
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from njord_sim.setpoint_controller_node import SetpointControllerNode
    HAS_ROS = True
except ImportError:
    HAS_ROS = False

KEYS = ('approach_radius_m', 'align_radius_m', 'kp_surge', 'kp_yaw', 'kd_yaw', 'kp_position', 'kd_position',
        'braking_deceleration_mps2', 'reaction_time_s', 'max_speed', 'max_thrust')


def parameters(vessel):
    resolved = resolve_configuration(ROOT / f"njord_sim/config/vessels/{vessel}.yaml",
                                     ROOT / "scenarios/goto_square.yaml", ROOT / "njord_sim/config/algorithms.yaml")
    return autonomy_parameters(resolved)['setpoint_controller']


def controller(vessel):
    p = parameters(vessel)
    return SetpointController({k: p[k] for k in KEYS}, p['thruster_positions'], p['thruster_axes'],
                              p['thruster_forward_limits'], p['thruster_reverse_limits'])


def body_wrench(core, thrusts):
    rows = allocation_matrix(core.positions, core.axes)
    return [sum(a * t for a, t in zip(row, thrusts)) for row in rows]


def simulate(core, target, seconds=150.0, dt=0.05, mass=250.0, inertia=300.0, damping=(60.0, 120.0, 200.0),
             side_force=0.0):
    """Planar 3-DOF model: body-frame forces, linear damping, optional constant ENU +y force."""
    x = y = yaw = u = v = r = 0.0
    for _ in range(int(seconds / dt)):
        X, Y, N = body_wrench(core, core.thrusts((x, y, yaw), (u, v, r), target))
        # Constant world-frame side force (e.g. wind), rotated into the body.
        X += side_force * math.sin(yaw)
        Y += side_force * math.cos(yaw)
        u += dt * (X - damping[0] * u) / mass
        v += dt * (Y - damping[1] * v) / mass
        r += dt * (N - damping[2] * r) / inertia
        yaw = math.atan2(math.sin(yaw + dt * r), math.cos(yaw + dt * r))
        x += dt * (u * math.cos(yaw) - v * math.sin(yaw))
        y += dt * (u * math.sin(yaw) + v * math.cos(yaw))
    return x, y, yaw


class SetpointControllerTests(unittest.TestCase):
    def test_layouts(self):
        self.assertTrue(controller('munin_v0').fully_actuated)
        self.assertFalse(controller('wamv').fully_actuated)

    def test_x_layout_corrects_a_lateral_error_with_pure_sway(self):
        core = controller('munin_v0')
        surge, sway, yaw = body_wrench(core, core.thrusts((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 2.0, 0.0)))
        self.assertEqual(core.mode, STATION_KEEPING)
        self.assertGreater(sway, 100.0)
        self.assertAlmostEqual(surge, 0.0, places=6)
        self.assertAlmostEqual(yaw, 0.0, places=6)

    def test_transit_turns_first_and_brakes_near_the_target(self):
        core = controller('wamv')
        surge, _, yaw = core.wrench((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 30.0, 0.0))
        self.assertEqual(core.mode, TRANSIT)
        self.assertGreater(yaw, 0.0)          # target to port: turn counter-clockwise
        self.assertLessEqual(surge, 1e-9)     # 90 deg off the bow: no forward speed yet
        surge, _, _ = core.wrench((0.0, 0.0, 0.0), (2.0, 0.0, 0.0), (6.0, 0.0, 0.0))
        self.assertLess(surge, 0.0)           # 2 m/s with 6 m left: brake

    def test_hysteresis_keeps_station_keeping_near_the_radius(self):
        core = controller('munin_v0')
        core.wrench((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (4.0, 0.0, 0.0))
        self.assertEqual(core.mode, STATION_KEEPING)
        core.wrench((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (6.0, 0.0, 0.0))
        self.assertEqual(core.mode, STATION_KEEPING)
        core.wrench((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (8.0, 0.0, 0.0))
        self.assertEqual(core.mode, TRANSIT)

    def test_thrust_respects_limits(self):
        for vessel in ('wamv', 'munin_v0'):
            core = controller(vessel)
            for thrust in core.thrusts((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, -3.0, math.pi / 2)):
                self.assertLessEqual(abs(thrust), core.settings['max_thrust'] + 1e-9)

    def test_closed_loop_reaches_position_and_heading(self):
        for vessel, target in (('munin_v0', (20.0, 10.0, math.pi / 2)), ('munin_v0', (-15.0, 5.0, math.pi)),
                               ('wamv', (20.0, 10.0, math.pi / 2)), ('wamv', (-15.0, -5.0, 0.0))):
            with self.subTest(vessel=vessel, target=target):
                x, y, yaw = simulate(controller(vessel), target)
                self.assertLess(math.dist((x, y), target[:2]), 1.0)
                self.assertLess(abs(math.atan2(math.sin(yaw - target[2]), math.cos(yaw - target[2]))),
                                math.radians(10))

    def test_fully_actuated_holds_against_a_side_force(self):
        x, y, yaw = simulate(controller('munin_v0'), (10.0, 0.0, 0.0), side_force=40.0)
        self.assertLess(math.dist((x, y), (10.0, 0.0)), 1.0)
        self.assertLess(abs(yaw), math.radians(10))

    def test_invalid_settings_rejected(self):
        p = parameters('wamv')
        for key, value in (('align_radius_m', 10.0), ('max_thrust', 0.0), ('braking_deceleration_mps2', 0.0)):
            settings = {k: p[k] for k in KEYS}
            settings[key] = value
            with self.subTest(key=key), self.assertRaises(ValueError):
                SetpointController(settings, p['thruster_positions'], p['thruster_axes'],
                                   p['thruster_forward_limits'], p['thruster_reverse_limits'])


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


@contextmanager
def local_node(overrides=None):
    """Construct the node with real callbacks and messages but no DDS transport."""
    parameters, publishers = {}, {}
    clock = SimpleNamespace(seconds=10.0)

    def now():
        ns = round(clock.seconds * 1e9)
        return SimpleNamespace(nanoseconds=ns, to_msg=lambda: Time(sec=ns // 10**9, nanosec=ns % 10**9))

    def declare(node, namespace, values):
        parameters.update(values)
        parameters.update(overrides or {})

    def publisher(node, message_type, topic, qos, **kwargs):
        publishers[topic] = Publisher()
        return publishers[topic]

    logger = SimpleNamespace(info=lambda *a, **k: None, warning=lambda *a, **k: None)
    with ExitStack() as stack:
        for name, replacement in {
            "__init__": lambda self, *args, **kwargs: None,
            "declare_parameters": declare,
            "get_parameter": lambda self, name: SimpleNamespace(value=parameters[name]),
            "create_publisher": publisher,
            "create_subscription": lambda *args, **kwargs: None,
            "create_timer": lambda *args, **kwargs: None,
            "get_clock": lambda self: SimpleNamespace(now=now),
            "get_logger": lambda self: logger,
        }.items():
            stack.enter_context(patch.object(Node, name, replacement))
        yield SetpointControllerNode(), clock, publishers


@unittest.skipUnless(HAS_ROS, "ROS messages unavailable; run with sourced Jazzy /usr/bin/python3")
class SetpointControllerNodeTests(unittest.TestCase):
    @staticmethod
    def odometry(stamp=10.0, x=0.0):
        msg = Odometry()
        msg.header.frame_id, msg.child_frame_id = "map", "base_link"
        msg.header.stamp.sec, msg.header.stamp.nanosec = int(stamp), round(stamp % 1 * 1e9)
        msg.pose.pose.position.x = x
        msg.pose.pose.orientation.w = 1.0
        return msg

    @staticmethod
    def setpoint(x=10.0, frame="map"):
        msg = PoseStamped()
        msg.header.frame_id = frame
        msg.pose.position.x = x
        msg.pose.orientation.w = 1.0
        return msg

    def last(self, publishers):
        return [publishers[f"/thruster_{i}/command"].messages[-1].data for i in (1, 2)]

    def test_drives_only_with_target_and_fresh_odometry(self):
        with local_node() as (node, clock, publishers):
            node.on_odom(self.odometry())
            node.step()
            self.assertEqual(self.last(publishers), [0.0, 0.0])      # no target
            node.on_setpoint(self.setpoint())
            node.step()
            self.assertTrue(all(t > 0 for t in self.last(publishers)))
            clock.seconds = 11.0                                      # odometry 1 s old
            node.step()
            self.assertEqual(self.last(publishers), [0.0, 0.0])
            node.heartbeat()
            status = publishers["/njord/controller_status"].messages[-1].status[0]
            self.assertEqual((status.name, status.level), ("controller", DiagnosticStatus.WARN))

    def test_invalid_target_or_odometry_stops_at_once(self):
        with local_node() as (node, clock, publishers):
            node.on_odom(self.odometry())
            node.on_setpoint(self.setpoint())
            node.on_setpoint(self.setpoint(frame="base_link"))
            self.assertIsNone(node.target)
            self.assertEqual(self.last(publishers), [0.0, 0.0])
            node.on_setpoint(self.setpoint())
            node.on_odom(self.odometry(x=float("nan")))
            self.assertEqual(self.last(publishers), [0.0, 0.0])
            node.step()
            self.assertEqual(self.last(publishers), [0.0, 0.0])

    def test_heartbeat_ok_without_target(self):
        with local_node() as (node, clock, publishers):
            node.on_odom(self.odometry())
            node.heartbeat()
            status = publishers["/njord/controller_status"].messages[-1].status[0]
            self.assertEqual(status.level, DiagnosticStatus.OK)
            self.assertEqual(status.message, "holding no target")


if __name__ == "__main__":
    unittest.main()
