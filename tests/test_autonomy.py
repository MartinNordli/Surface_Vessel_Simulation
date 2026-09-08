"""Independent safety regressions for pure planner/controller interfaces."""

import math
import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'njord_sim'))
from njord_sim.control_core import (clearance, mix_thrusters, segment_is_free,
                                    speed_limit, tracking_corridor)
from njord_sim.dstar_lite import DStarLite, Grid, INF, astar
from njord_sim.planner_core import Geometry, IncrementalPlanner, fresh


class PlannerSafetyTests(unittest.TestCase):
    def test_invalid_endpoints_including_blocked_same_cell(self):
        for start, goal, blocked in [((-1, 0), (3, 3), set()),
                                     ((0, 0), (4, 3), set()),
                                     ((1, 1), (1, 1), {(1, 1)}),
                                     ((0, 0), (3, 3), {(3, 3)})]:
            grid = Grid(4, 4)
            grid.blocked = blocked
            planner = DStarLite(grid, start, goal)
            planner.compute_shortest_path()
            self.assertEqual(planner.path(), [])
            self.assertEqual(planner.path_cost(), INF)
            self.assertEqual(astar(grid, start, goal), INF)

    def test_block_and_unblock_goal_preserves_incremental_search(self):
        core = IncrementalPlanner()
        geometry = Geometry(7, 5, 1, 0, 0)
        data = [0] * 35
        self.assertTrue(core.plan(geometry, data, (0.5, 0.5), (6.5, 4.5)))
        search = core.search
        data[-1] = 100
        self.assertEqual(core.plan(geometry, data, (0.5, 0.5), (6.5, 4.5)), [])
        data[-1] = 0
        self.assertTrue(core.plan(geometry, data, (0.5, 0.5), (6.5, 4.5)))
        self.assertIs(core.search, search)
        self.assertAlmostEqual(core.search.path_cost(), astar(core.search.grid, (0, 0), (4, 6)))

    def test_wall_invalidates_old_path_and_recovery_matches_astar(self):
        geometry = Geometry(8, 8, 1, 0, 0)
        data = [0] * 64
        core = IncrementalPlanner()
        self.assertTrue(core.plan(geometry, data, (1.5, 1.5), (6.5, 6.5)))
        for row in range(8):
            data[row * 8 + 4] = 100
        self.assertEqual(core.plan(geometry, data, (2.5, 2.5), (6.5, 6.5)), [])
        data[4 * 8 + 4] = 0
        self.assertTrue(core.plan(geometry, data, (2.5, 2.5), (6.5, 6.5)))
        self.assertAlmostEqual(core.search.path_cost(), astar(core.search.grid, (2, 2), (6, 6)))

    def test_complete_geometry_changes_reinitialize_search(self):
        core = IncrementalPlanner()
        variants = [Geometry(4, 4, 1, 0, 0), Geometry(4, 5, 1, 0, 0),
                    Geometry(4, 5, 1, 0, -1), Geometry(4, 5, 1, 0, -1, 2),
                    Geometry(4, 5, 1, 0, -1, 2, 0.25), Geometry(4, 5, 1, 0, -1, 2, 0.25, 'alternate')]
        previous = None
        for geometry in variants:
            result = core.plan(geometry, [0] * (geometry.width * geometry.height),
                               geometry.world((0, 0)), geometry.world((3, 3)))
            self.assertTrue(result)
            self.assertIsNot(core.search, previous)
            previous = core.search

    def test_rotated_grid_and_malformed_data(self):
        geometry = Geometry(10, 10, 0.5, -3, 8, 0, math.pi / 3)
        for cell in [(0, 0), (4, 7), (9, 9)]:
            self.assertEqual(geometry.cell(geometry.world(cell)), cell)
        with self.assertRaises(ValueError):
            geometry.validate_data([0] * 99)
        with self.assertRaises(ValueError):
            geometry.validate_data([101] * 100)
        with self.assertRaises(ValueError):
            Geometry(0, 3, 1, 0, 0)
        msg = NS(header=NS(frame_id='odom'), info=NS(origin=NS(position=NS(x=0, y=0, z=0),
                 orientation=NS(x=0, y=0, z=0, w=1)), width=2, height=2, resolution=1))
        with self.assertRaises(ValueError):
            Geometry.from_message(msg, 'map')
        msg.header.frame_id = 'map'
        msg.info.origin.orientation.x = 0.2
        with self.assertRaises(ValueError):
            Geometry.from_message(msg, 'map')


class GuidanceSafetyTests(unittest.TestCase):
    def setUp(self):
        self.geometry = Geometry(20, 20, 1, 0, 0)
        self.data = [0] * 400

    def test_unknown_allowed_in_global_plan_but_forbidden_to_controller(self):
        core = IncrementalPlanner()
        self.data[10 * 20 + 10] = -1
        path = core.plan(self.geometry, self.data, (8.5, 10.5), (12.5, 10.5))
        self.assertTrue(path)
        self.assertFalse(segment_is_free(self.geometry, self.data, (8.5, 10.5), (12.5, 10.5)))
        target, distance = tracking_corridor(self.geometry, self.data, path, (8.5, 10.5), 8)
        self.assertEqual(target, (9.5, 10.5))
        self.assertEqual(distance, 1.0)

    def test_shortcuts_do_not_cut_occupied_corner(self):
        self.data[1 * 20 + 2] = 100
        self.assertFalse(segment_is_free(self.geometry, self.data, (1.5, 1.5), (2.5, 2.5)))
        self.assertFalse(segment_is_free(self.geometry, self.data, (1.9, 1.1), (2.1, 1.9)))
        self.assertFalse(segment_is_free(self.geometry, self.data, (-0.1, 1), (1.5, 1.5)))
        self.assertTrue(segment_is_free(self.geometry, self.data, (1.5, 1.5), (1.5, 5.5)))

    def test_boundary_parallel_segment_checks_both_sides(self):
        self.data[3 * 20 + 4] = 100
        self.assertFalse(segment_is_free(self.geometry, self.data, (5.0, 2.5), (5.0, 5.5)))
        self.assertTrue(segment_is_free(self.geometry, self.data, (5.01, 2.5), (5.01, 5.5)))

    def test_freshness_uses_acquisition_time_and_rejects_clock_reset(self):
        self.assertTrue(fresh(10, 9.5, 1))
        self.assertFalse(fresh(10, 8, 1))
        self.assertFalse(fresh(0.1, 10, 1))
        self.assertFalse(fresh(10, None, 1))
        self.assertFalse(fresh(10, math.nan, 1))

    def test_speed_respects_stopping_distance_turn_and_clearance(self):
        args = dict(max_speed=3, heading_error=0, free_distance=5, clearance_m=5,
                    current_speed=1, deceleration=0.25, reaction_s=1, margin_m=3)
        self.assertAlmostEqual(speed_limit(**args), math.sqrt(0.5))
        self.assertLess(speed_limit(**dict(args, heading_error=1.4)), 0.1)
        self.assertEqual(speed_limit(**dict(args, free_distance=3)), 0)
        self.assertEqual(speed_limit(**dict(args, clearance_m=0)), 0)
        self.data[10 * 20 + 12] = 100
        self.assertLess(clearance(self.geometry, self.data, (10.5, 10.5)), 2)

    def test_thruster_moment_uses_physical_separation_and_saturation(self):
        left, right = mix_thrusters(100, 40, 2.4, 500)
        self.assertAlmostEqual(left + right, 100)
        self.assertAlmostEqual((right - left) * 1.2, 40)
        left, right = mix_thrusters(1000, 1000, 2.4, 100)
        self.assertLessEqual(max(abs(left), abs(right)), 100)
        self.assertAlmostEqual((right - left) * 1.2 / (left + right), 1)
        with self.assertRaises(ValueError):
            mix_thrusters(1, 1, 0, 500)


class NodeCallbackTests(unittest.TestCase):
    """Exercise failure callbacks with ROS transport replaced by tiny doubles."""

    @classmethod
    def setUpClass(cls):
        messages = NS(OccupancyGrid=NS, Odometry=NS, Path=NS)
        diagnostic = NS(DiagnosticArray=NS, DiagnosticStatus=NS(OK=0, ERROR=2), KeyValue=NS)
        doubles = {'rclpy': NS(), 'rclpy.node': NS(Node=object),
                   'rclpy.qos': NS(qos_profile_sensor_data=object()),
                   'diagnostic_msgs': NS(), 'diagnostic_msgs.msg': diagnostic,
                   'nav_msgs': NS(), 'nav_msgs.msg': messages,
                   'std_msgs': NS(), 'std_msgs.msg': NS(Float64=NS)}
        source = Path(__file__).resolve().parents[1] / 'njord_sim/njord_sim/guidance_node.py'
        spec = importlib.util.spec_from_file_location('guidance_callback_test', source)
        module = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, doubles):
            spec.loader.exec_module(module)
        cls.Guidance = module.Guidance

    def setUp(self):
        self.node = self.Guidance.__new__(self.Guidance)
        self.node.p = lambda name: {'map_frame': 'map', 'base_frame': 'wamv/base_link',
                                   'stale_after_s': 1.0}[name]
        self.commands = []
        self.node.left = self.node.right = NS(publish=lambda msg: self.commands.append(msg.data))

    def test_empty_path_removes_previous_route_and_zeroes_commands(self):
        self.node.path = [(1, 1), (2, 2)]
        self.node.on_path(NS(header=NS(frame_id='map', stamp=NS(sec=10, nanosec=0)), poses=[]))
        self.assertEqual(self.node.path, [])
        self.assertEqual(self.commands, [0.0, 0.0])

    def test_freshly_delivered_old_odometry_keeps_old_stamp(self):
        pose = NS(position=NS(x=1, y=2), orientation=NS(x=0, y=0, z=0, w=1))
        twist = NS(linear=NS(x=1), angular=NS(z=0))
        self.node.on_odom(NS(header=NS(frame_id='map', stamp=NS(sec=1, nanosec=500000000)),
                            child_frame_id='wamv/base_link', pose=NS(pose=pose), twist=NS(twist=twist)))
        self.assertEqual(self.node.odom_stamp, 1.5)
        self.node.path = [(1, 1), (5, 5)]
        self.node.geometry = Geometry(10, 10, 1, 0, 0)
        self.node.status_valid = True
        self.node.path_stamp = self.node.grid_stamp = self.node.status_stamp = 10.0
        self.node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=10000000000))
        self.node.step()
        self.assertEqual(self.commands, [0.0, 0.0])

    def test_failed_planner_status_zeroes_commands(self):
        self.node.on_status(NS(header=NS(frame_id='map', stamp=NS(sec=10, nanosec=0)),
                               status=[NS(name='njord/planner', level=2)]))
        self.assertFalse(self.node.status_valid)
        self.assertEqual(self.commands, [0.0, 0.0])


if __name__ == '__main__':
    unittest.main()
