"""Public run handoff and reproducible team settings without starting ROS."""
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'njord_sim'))
spec = importlib.util.spec_from_file_location('team_runner_under_test', ROOT / 'scripts/run_autonomy.py')
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)


class TeamRunnerTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory(prefix='njord-team-runner-')
        self.addCleanup(self.directory.cleanup)
        self.output = Path(self.directory.name)
        (self.output / 'run_ready.json').write_text(json.dumps({'run_id': 'run-1', 'expected_gates': 5}))
        self.vessel = self.output / 'vessel_config.yaml'
        self.vessel.write_text('camera_rate: 8.0\nmax_thrust_n: 350.0\n')

    def metadata(self):
        return json.loads((self.output / 'autonomy_config.json').read_text())

    def test_generated_vessel_and_public_gate_count_override_cli(self):
        command = runner.prepare(self.output, 'run-1', {}, [
            'vessel_config:=incorrect.yaml', 'expected_gates:=99'])
        self.assertEqual(command[-1], f'vessel_config:={self.vessel}')
        self.assertLess(command.index('expected_gates:=99'), command.index('expected_gates:=5'))
        metadata = self.metadata()
        self.assertEqual(metadata['expected_gates'], 5)
        self.assertEqual(metadata['vessel_config_sha256'], hashlib.sha256(self.vessel.read_bytes()).hexdigest())
        self.assertNotIn('scenario', metadata)
        self.assertNotIn('gates', metadata)

    def test_params_are_snapshotted_and_modes_provenance_match_command(self):
        original = self.output / 'requested.yaml'
        payload = b'guidance:\n  ros__parameters:\n    kp_surge: 125.0\n'
        original.write_bytes(payload)
        command = runner.prepare(self.output, 'run-1', {
            'ROS_PARAMS_FILE': str(original), 'CONTROLLER': 'external', 'PERCEPTION': 'external',
            'PROFILE': 'conservative', 'SEED': '9', 'IMAGE_ID': 'sha256:test',
            'NJORD_IMAGE_SOURCE_COMMIT': 'source-commit', 'RUNNER_GIT_COMMIT': 'runner-commit',
        })
        snapshot = self.output / 'ros_params.yaml'
        self.assertEqual(snapshot.read_bytes(), payload)
        original.write_text('changed after run preparation')
        self.assertEqual(snapshot.read_bytes(), payload)
        self.assertIn(f'params_file:={snapshot}', command)
        metadata = self.metadata()
        self.assertEqual(metadata['params_sha256'], hashlib.sha256(payload).hexdigest())
        self.assertEqual(metadata['controller'], 'external')
        self.assertEqual(metadata['perception'], 'external')
        self.assertEqual(metadata['mapping'], 'reference')
        self.assertEqual(metadata['profile'], 'conservative')
        self.assertEqual(metadata['seed'], 9)
        self.assertEqual(metadata['image_identity'], 'sha256:test')
        self.assertEqual(metadata['image_source_commit'], 'source-commit')
        self.assertEqual(metadata['runner_git_commit'], 'runner-commit')
        self.assertEqual(metadata['command'], command)

    def test_cli_override_is_snapshotted_and_recorded_as_effective_setting(self):
        params = self.output / 'cli.yaml'
        params.write_text('/**:\n  ros__parameters:\n    use_sim_time: true\n')
        extra = ['perception:=external', 'profile:=conservative', 'seed:=17', f'params_file:={params}']
        command = runner.prepare(self.output, 'run-1', {'ROS_PARAMS_FILE': '/missing/env.yaml'}, extra)
        metadata = self.metadata()
        self.assertEqual(metadata['extra_launch_args'], extra)
        self.assertEqual(metadata['perception'], 'external')
        self.assertEqual(metadata['profile'], 'conservative')
        self.assertEqual(metadata['seed'], 17)
        self.assertEqual(command[-2], f'params_file:={self.output / "ros_params.yaml"}')

    def test_no_requested_params_omits_malformed_empty_launch_argument(self):
        command = runner.prepare(self.output, 'run-1', {})
        self.assertFalse(any(argument.endswith(':=') for argument in command))
        self.assertIsNone(self.metadata()['params_sha256'])
        self.assertFalse((self.output / 'ros_params.yaml').exists())

    def test_missing_generated_vessel_fails_without_provenance(self):
        self.vessel.unlink()
        with self.assertRaises(FileNotFoundError):
            runner.prepare(self.output, 'run-1', {})
        self.assertFalse((self.output / 'autonomy_config.json').exists())

    def test_missing_requested_params_fails_without_provenance(self):
        with self.assertRaises(FileNotFoundError):
            runner.prepare(self.output, 'run-1', {'ROS_PARAMS_FILE': '/missing/team.yaml'})
        self.assertFalse((self.output / 'autonomy_config.json').exists())

    def test_invalid_mode_fails_without_provenance(self):
        with self.assertRaisesRegex(ValueError, 'controller'):
            runner.prepare(self.output, 'run-1', {'CONTROLLER': 'typo'})
        self.assertFalse((self.output / 'autonomy_config.json').exists())

    def test_failed_handoff_never_starts_or_writes_team_configuration(self):
        with patch.object(runner, 'wait_ready', side_effect=TimeoutError('wrong run')) as wait:
            with self.assertRaises(TimeoutError):
                runner.prepare(self.output, 'run-1', {})
        wait.assert_called_once_with(self.output, 'run-1')
        self.assertFalse((self.output / 'autonomy_config.json').exists())

    def test_real_ros_launch_parser_accepts_defaults_and_enforces_generated_config(self):
        try:
            from ros2launch.api.api import parse_launch_arguments
        except ImportError:
            self.skipTest('ROS launch parser unavailable; run with sourced Jazzy /usr/bin/python3')
        command = runner.prepare(self.output, 'run-1', {}, ['vessel_config:=wrong.yaml'])
        parsed = dict(parse_launch_arguments(command[4:]))
        self.assertEqual(parsed['vessel_config'], str(self.vessel))
        self.assertEqual(parsed['expected_gates'], '5')
        self.assertEqual(parsed['autonomy'], 'reference')


if __name__ == '__main__':
    unittest.main()
