"""Recorder handoff and output preservation without starting ROS transport."""
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'njord_sim'))
spec = importlib.util.spec_from_file_location('njord_recorder_script', ROOT/'scripts/run_recorder.py')
recorder = importlib.util.module_from_spec(spec)
spec.loader.exec_module(recorder)


class RecorderTests(unittest.TestCase):
    def manifest(self, output):
        (output/'run_ready.json').write_text(json.dumps({'run_id': 'current-run', 'expected_gates': 3}))

    def test_matching_run_records_source_provenance_and_latched_tf(self):
        with tempfile.TemporaryDirectory(prefix='njord-recorder-') as temporary:
            output = Path(temporary)
            self.manifest(output)
            command = recorder.prepare(output, 'current-run', {
                'IMAGE_ID': 'sha256:test', 'NJORD_IMAGE_SOURCE_COMMIT': 'image-commit',
                'NJORD_IMAGE_SOURCE_DIGEST': 'source-digest', 'RUNNER_GIT_COMMIT': 'runner-commit'})
            metadata = json.loads((output/'recording.json').read_text())
            self.assertEqual(metadata['run_id'], 'current-run')
            self.assertEqual(metadata['image_source_commit'], 'image-commit')
            self.assertEqual(metadata['runner_git_commit'], 'runner-commit')
            self.assertIn('--use-sim-time', command)
            self.assertEqual(command[command.index('--output')+1], str(output/'bag'))
            self.assertEqual(metadata['qos_overrides']['/tf_static']['durability'], 'transient_local')
            self.assertIn('/wamv/ground_truth/odometry', metadata['topics'])
            self.assertIn('/njord/contacts', metadata['topics'])
            self.assertFalse((output/'bag').exists())  # rosbag2 owns directory creation.

    def test_existing_bag_is_never_replaced(self):
        with tempfile.TemporaryDirectory(prefix='njord-recorder-') as temporary:
            output = Path(temporary)
            self.manifest(output)
            (output/'bag').mkdir()
            previous = output/'bag/metadata.yaml'
            previous.write_text('valuable recording')
            with self.assertRaises(FileExistsError):
                recorder.prepare(output, 'current-run', {})
            self.assertEqual(previous.read_text(), 'valuable recording')
            self.assertFalse((output/'recording.json').exists())

    def test_failed_run_handoff_creates_no_recording_artifacts(self):
        with tempfile.TemporaryDirectory(prefix='njord-recorder-') as temporary:
            output = Path(temporary)
            with patch.object(recorder, 'wait_ready', side_effect=TimeoutError('wrong run')) as wait:
                with self.assertRaises(TimeoutError):
                    recorder.prepare(output, 'current-run', {})
            wait.assert_called_once_with(output, 'current-run')
            self.assertEqual(list(output.iterdir()), [])


if __name__ == '__main__':
    unittest.main()
