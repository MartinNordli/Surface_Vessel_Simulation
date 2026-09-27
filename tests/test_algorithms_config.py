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
        converted = convert_algorithm_schema(old)
        self.assertEqual(converted['schema_version'], 3)
        self.assertEqual(converted['mapping'], self.shipped()['mapping'])
        self.assertNotIn('grid_size_m', old['mapping'])
        self.assertEqual(self.resolve(old)['algorithms'], self.resolve(self.shipped())['algorithms'])
        old['mapping']['grid_size_m'] = 100.0
        with self.assertRaisesRegex(ValueError, 'schema 3'):
            convert_algorithm_schema(old)

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
