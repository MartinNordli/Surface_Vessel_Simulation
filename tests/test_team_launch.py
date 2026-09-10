"""Check team component selection without starting ROS nodes or Gazebo.

These tests evaluate real launch substitutions and capture requested processes;
they do not establish DDS delivery, sensor rendering, or vessel dynamics.
"""
import importlib.util
import itertools
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import yaml

try:
    from launch import LaunchContext
    HAS_LAUNCH = True
except ImportError:
    HAS_LAUNCH = False


REPO = Path(__file__).resolve().parents[1]
SHARE = REPO / 'njord_sim'
sys.path.insert(0, str(SHARE))


@unittest.skipUnless(HAS_LAUNCH, 'ROS launch unavailable; run with sourced Jazzy /usr/bin/python3')
class TeamLaunchTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        logs = tempfile.TemporaryDirectory(prefix='njord-team-launch-')
        cls.addClassCleanup(logs.cleanup)
        cls.enterClassContext(patch.dict(os.environ, {'ROS_LOG_DIR': logs.name}))
        spec = importlib.util.spec_from_file_location(
            'team_launch_under_test', SHARE / 'launch/dstar_demo.launch.py')
        cls.module = importlib.util.module_from_spec(spec)
        try:
            spec.loader.exec_module(cls.module)
        except ImportError as error:
            raise unittest.SkipTest(f'ROS launch dependencies unavailable: {error}')

    def launch(self, **overrides):
        context = LaunchContext()
        context.launch_configurations.update({
            'profile': 'fast', 'seed': '7', 'expected_gates': '5',
            'autonomy': 'reference', 'controller': 'reference',
            'perception': 'reference', 'mapping': 'reference',
            'params_file': '', 'vessel_config': str(SHARE / 'config/vessel.yaml'),
            **overrides,
        })
        with patch.object(self.module, 'get_package_share_directory', return_value=str(SHARE)), \
                patch.object(self.module, 'Node', side_effect=lambda **kwargs: kwargs):
            return self.module.launch(context)

    def test_default_launch_keeps_complete_reference_stack(self):
        nodes = self.launch()
        self.assertCountEqual([node['executable'] for node in nodes], [
            'sensor_adapter', 'ekf_node', 'ekf_node', 'navsat_transform_node',
            'mapper', 'perception', 'mission', 'planner', 'guidance', 'command_guard',
        ])

    def test_every_team_combination_removes_only_owned_reference_publishers(self):
        for controller, perception, mapping in itertools.product(('reference', 'external'), repeat=3):
            with self.subTest(controller=controller, perception=perception, mapping=mapping):
                nodes = self.launch(controller=controller, perception=perception, mapping=mapping)
                executables = [node['executable'] for node in nodes]
                for choice, executable in [(controller, 'guidance'), (perception, 'perception'), (mapping, 'mapper')]:
                    self.assertEqual(executables.count(executable), int(choice == 'reference'))
                for executable in ['sensor_adapter', 'navsat_transform_node', 'mission', 'planner', 'command_guard']:
                    self.assertEqual(executables.count(executable), 1)
                self.assertEqual(executables.count('ekf_node'), 2)

    def test_full_external_stack_retains_estimation_and_single_guard(self):
        self.assertCountEqual([node['executable'] for node in self.launch(autonomy='external')], [
            'sensor_adapter', 'ekf_node', 'ekf_node', 'navsat_transform_node', 'command_guard',
        ])

    def test_bad_mode_never_silently_launches_reference_component(self):
        for argument in ['autonomy', 'controller', 'perception', 'mapping']:
            with self.subTest(argument=argument), self.assertRaises(ValueError):
                self.launch(**{argument: 'typo'})

    def test_bad_component_mode_is_rejected_even_for_full_external_stack(self):
        for argument in ['controller', 'perception', 'mapping']:
            with self.subTest(argument=argument), self.assertRaises(ValueError):
                self.launch(autonomy='external', **{argument: 'typo'})

    def test_parameter_file_can_override_defaults_but_not_simulation_clock(self):
        with tempfile.TemporaryDirectory(prefix='njord-team-params-') as directory:
            params_file = Path(directory) / 'team.yaml'
            params_file.write_text(yaml.safe_dump({'/**': {'ros__parameters': {'use_sim_time': False}}}))
            nodes = self.launch(params_file=str(params_file))
            for node in nodes:
                with self.subTest(executable=node['executable'], name=node.get('name')):
                    parameters = node['parameters']
                    files = [str(value) for value in parameters if isinstance(value, (str, Path))]
                    self.assertIn(str(params_file), files)
                    team_index = next(index for index, value in enumerate(parameters) if str(value) == str(params_file))
                    clock_settings = [(index, value['use_sim_time']) for index, value in enumerate(parameters)
                                      if isinstance(value, dict) and 'use_sim_time' in value]
                    self.assertTrue(clock_settings)
                    self.assertGreater(clock_settings[-1][0], team_index)
                    self.assertIs(clock_settings[-1][1], True)


if __name__ == '__main__':
    unittest.main()
