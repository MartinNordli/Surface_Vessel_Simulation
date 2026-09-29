"""algorithms.yaml owns the speed profiles and the reference occupancy grid."""
from pathlib import Path
import sys
import tempfile
import unittest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'njord_sim'))
from njord_sim.configuration import (autonomy_parameters, convert_algorithm_schema, guard_requirements,
                                     resolve_configuration, speed_profile_names)

ROOT = Path(__file__).resolve().parents[1]
ALGORITHMS = ROOT/'njord_sim/config/algorithms.yaml'
# Scenario files that are not scored races and never run the reference mapper.
NON_RACE_SCENARIOS = {'dynamics'}
# Room a race needs inside the grid around every buoy, obstacle and the start
# (m): the mission's 5 m approach and 6 m exit plus arrival tolerance.
COURSE_MARGIN_M = 10.0


class AlgorithmsConfigTests(unittest.TestCase):
    def resolve(self, algorithms):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'algorithms.yaml'
            path.write_text(yaml.safe_dump(algorithms))
            return resolve_configuration(ROOT/'njord_sim/config/vessels/wamv.yaml', ROOT/'scenarios/reference.yaml',
                                         path)

    def shipped(self):
        return yaml.safe_load(ALGORITHMS.read_text())

    def test_speed_profile_names_come_from_the_file(self):
        self.assertEqual(speed_profile_names(ALGORITHMS), sorted(self.shipped()['speed_profiles_mps']))
        algorithms = self.shipped()
        algorithms['speed_profiles_mps']['survey'] = 0.5
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'algorithms.yaml'
            path.write_text(yaml.safe_dump(algorithms))
            self.assertIn('survey', speed_profile_names(path))
            path.write_text(yaml.safe_dump({**algorithms, 'speed_profiles_mps': {}}))
            with self.assertRaises(ValueError):
                speed_profile_names(path)

    def test_grid_reaches_the_mapper_as_ros_doubles(self):
        algorithms = self.shipped()
        algorithms['mapping'].update(grid_resolution_m=1, grid_size_m=200, grid_origin_m=[-50, -100])
        mapper = autonomy_parameters(self.resolve(algorithms))['mapper']
        self.assertEqual((mapper['resolution'], mapper['size_m'], mapper['origin_x'], mapper['origin_y']),
                         (1.0, 200.0, -50.0, -100.0))
        for key in ('resolution', 'size_m', 'origin_x', 'origin_y'):
            self.assertIs(type(mapper[key]), float)

    def test_schema_2_converts_to_the_previously_built_in_grid(self):
        old = self.shipped()
        old['schema_version'] = 2
        for key in ('grid_resolution_m', 'grid_size_m', 'grid_origin_m'):
            del old['mapping'][key]
        del old['mission']
        converted = convert_algorithm_schema(old)
        self.assertEqual(converted['schema_version'], 4)
        self.assertEqual(converted['mapping'], self.shipped()['mapping'])
        self.assertNotIn('grid_size_m', old['mapping'])
        resolved = self.resolve(old)['algorithms']
        self.assertEqual({k: v for k, v in resolved.items() if k != 'mission'},
                         {k: v for k, v in self.resolve(self.shipped())['algorithms'].items() if k != 'mission'})
        old['mapping']['grid_size_m'] = 100.0
        with self.assertRaisesRegex(ValueError, 'schema 3'):
            convert_algorithm_schema(old)

    def test_schema_3_converts_with_search_and_retry_disabled(self):
        old = self.shipped()
        old['schema_version'] = 3
        del old['mission']
        mission = convert_algorithm_schema(old)['mission']
        # The geometry the mission node used to build in; no search, no retry.
        shipped = self.shipped()['mission']
        for key in ('min_gate_width_m', 'max_gate_width_m', 'approach_m', 'exit_m', 'arrival_tolerance_m',
                    'detection_max_age_s', 'crossing_memory_s', 'crossing_entry_m'):
            self.assertEqual(mission[key], shipped[key])
        self.assertEqual(mission['max_gate_retries'], 0)
        self.assertGreater(mission['search_after_s'], 1e6)
        self.resolve(old)

    def test_invalid_mission_rejected(self):
        for key, value in [('max_gate_retries', 1.5), ('max_gate_retries', True), ('max_gate_retries', -1),
                           ('min_gate_width_m', 40.0), ('search_goal_s', 30.0), ('search_radius_m', 0.0),
                           ('approach_m', float('nan'))]:
            algorithms = self.shipped()
            algorithms['mission'][key] = value
            with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                self.resolve(algorithms)
        algorithms = self.shipped()
        algorithms['mission']['typo_m'] = 1.0
        with self.assertRaises(ValueError):
            self.resolve(algorithms)

    def test_retry_point_must_lie_outside_the_obstacle_inflation(self):
        algorithms = self.shipped()
        algorithms['mission']['retry_clearance_m'] = 4.0  # WAM-V inflation is 4.0 m
        with self.assertRaisesRegex(ValueError, 'retry_clearance_m'):
            self.resolve(algorithms)
        algorithms['mission']['retry_clearance_m'] = 5.1
        self.resolve(algorithms)

    def test_mission_reaches_the_node_with_navigation_freshness(self):
        mission = autonomy_parameters(self.resolve(self.shipped()))['mission']
        self.assertEqual({k: v for k, v in mission.items() if k not in ('camera_max_age_s', 'odometry_max_age_s')},
                         {k: v for k, v in self.shipped()['mission'].items()})
        self.assertIs(type(mission['max_gate_retries']), int)
        self.assertEqual(mission['camera_max_age_s'], self.shipped()['navigation']['stale_after_s'])

    def test_invalid_grid_rejected(self):
        for key, value in [('grid_resolution_m', 0), ('grid_size_m', -1), ('grid_size_m', float('nan')),
                           ('grid_origin_m', [0.0]), ('grid_origin_m', [0.0, float('inf')]),
                           ('grid_origin_m', 'corner')]:
            algorithms = self.shipped()
            algorithms['mapping'][key] = value
            with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                self.resolve(algorithms)
        algorithms = self.shipped()
        del algorithms['mapping']['grid_origin_m']
        with self.assertRaises(ValueError):
            self.resolve(algorithms)

    def resolve_vessel(self, vessel, algorithms):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'algorithms.yaml'
            path.write_text(yaml.safe_dump(algorithms))
            return resolve_configuration(ROOT/f'njord_sim/config/vessels/{vessel}.yaml',
                                         ROOT/'scenarios/reference.yaml', path)

    def test_vessel_override_applies_only_to_its_vessel(self):
        algorithms = self.shipped()
        algorithms['vessel_overrides'] = {'njord_analytic': {'guidance': {'kp_yaw': 123.0},
                                                             'mission': {'approach_m': 7.0}}}
        njord = self.resolve_vessel('njord_v1', algorithms)
        self.assertEqual(njord['algorithms']['vessel_override'], 'njord_analytic')
        self.assertEqual(njord['algorithms']['guidance']['kp_yaw'], 123.0)
        self.assertEqual(autonomy_parameters(njord)['mission']['approach_m'], 7.0)
        # Other guidance values stay shared.
        self.assertEqual(njord['algorithms']['guidance']['kp_surge'], self.shipped()['guidance']['kp_surge'])
        wamv = self.resolve_vessel('wamv', algorithms)
        self.assertIsNone(wamv['algorithms']['vessel_override'])
        self.assertEqual(wamv['algorithms']['guidance']['kp_yaw'], self.shipped()['guidance']['kp_yaw'])
        algorithms['vessel_overrides'] = {'wamv': {'guidance': {'kp_yaw': 99.0}}}
        self.assertEqual(self.resolve_vessel('wamv', algorithms)['algorithms']['guidance']['kp_yaw'], 99.0)

    def test_invalid_vessel_overrides_are_rejected_for_every_vessel(self):
        for overrides in ({'njord_analytic': {'navigation': {'stale_after_s': 1.0}}},  # not overridable
                          {'njord_analytic': {'guidance': {'kp_yaww': 1.0}}},  # misspelled key
                          {'someone_else': {'guidance': {'typo': 1.0}}},  # checked even if not applied
                          {'njord_analytic': {'guidance': {'kp_yaw': -1.0}}},  # merged value invalid
                          {'njord_analytic': {'guidance': {'max_thrust': 5000.0}}},  # above the vessel limit
                          {'njord_analytic': 'kp_yaw'}, 'not a mapping'):
            algorithms = self.shipped()
            algorithms['vessel_overrides'] = overrides
            with self.subTest(overrides=overrides), self.assertRaises(ValueError):
                self.resolve_vessel('njord_v1', algorithms)

    def test_guard_requirements(self):
        self.assertEqual(guard_requirements({'autonomy': 'reference'}, 'race'),
                         {'required_status': ['navigation', 'planner', 'mission'], 'require_race_active': True})
        self.assertEqual(guard_requirements({'autonomy': 'external'}, 'free'),
                         {'required_status': ['navigation'], 'require_race_active': False})
        self.assertEqual(guard_requirements({}, 'race')['required_status'], ['navigation', 'planner', 'mission'])
        for components, run_mode in (({'autonomy': 'reference'}, 'lab'), ({'autonomy': 'typo'}, 'race')):
            with self.subTest(components=components, run_mode=run_mode), self.assertRaises(ValueError):
                guard_requirements(components, run_mode)

    def test_race_courses_fit_inside_the_grid(self):
        mapping = self.shipped()['mapping']
        low = mapping['grid_origin_m']
        high = [corner + mapping['grid_size_m'] for corner in low]
        scenarios = [path for path in sorted((ROOT/'scenarios').glob('*.yaml')) if path.stem not in NON_RACE_SCENARIOS]
        self.assertTrue(scenarios)
        for path in scenarios:
            course = yaml.safe_load(path.read_text())
            points = [course['start'][:2]]
            points += [gate[color] for gate in course['gates'] for color in ('red', 'green')]
            points += [obstacle['position'] for obstacle in course.get('obstacles') or []]
            for point in points:
                with self.subTest(course=path.stem, point=point):
                    for axis in (0, 1):
                        self.assertGreaterEqual(point[axis] - low[axis], COURSE_MARGIN_M)
                        self.assertGreaterEqual(high[axis] - point[axis], COURSE_MARGIN_M)


if __name__ == '__main__':
    unittest.main()
