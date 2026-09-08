"""Run handoff must never consume an earlier scenario or reveal gate geometry."""
import json
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'njord_sim'))
from njord_sim.run_manifest import publish_ready, wait_ready


class RunHandoffTests(unittest.TestCase):
    def test_custom_gate_count_without_truth_coordinates(self):
        with tempfile.TemporaryDirectory() as output:
            publish_ready(output, 'new-run', {'gates': [{'red': [99, 7]}] * 4})
            metadata = wait_ready(output, 'new-run', timeout=0)
            self.assertEqual(metadata, {'run_id': 'new-run', 'expected_gates': 4})
            self.assertFalse((Path(output)/'run_ready.json.tmp').exists())

    def test_previous_or_partial_run_does_not_authorize_start(self):
        with tempfile.TemporaryDirectory() as output:
            path = Path(output)/'run_ready.json'
            for text in (json.dumps({'run_id': 'old', 'expected_gates': 3}), '{"run_id":'):
                path.write_text(text)
                with self.assertRaises(TimeoutError):
                    wait_ready(output, 'new', timeout=0)

    def test_run_identity_is_mandatory(self):
        with tempfile.TemporaryDirectory() as output:
            with self.assertRaises(ValueError):
                wait_ready(output, '', timeout=0)


if __name__ == '__main__':
    unittest.main()
