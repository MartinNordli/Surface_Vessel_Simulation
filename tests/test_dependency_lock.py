"""Offline regressions for consistent Docker base and dependency locks."""
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('dependency_lock', ROOT / 'scripts/lock-dependencies.py')
dependency_lock = importlib.util.module_from_spec(spec)
spec.loader.exec_module(dependency_lock)


class DependencyLockTests(unittest.TestCase):
    def test_refresh_changes_base_and_lock_together_preserving_other_stages(self):
        previous = 'ros:jazzy-ros-base@sha256:' + 'a' * 64
        current = 'ros:jazzy-ros-base@sha256:' + 'b' * 64
        dockerfile = f'# attribution\nFROM {previous} AS vrx-base\nRUN true\nFROM vrx-base AS vrx-runtime\n'
        lock = {'ros_image': current, 'vrx_commit': 'c' * 40,
                'vendors': {'example': {'url': 'https://example.org/vendor.git', 'commit': 'd' * 40}}}
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / 'Dockerfile').write_text(dockerfile)
            dependency_lock.write_lock(root, lock)
            self.assertEqual((root / 'Dockerfile').read_text(), dockerfile.replace(previous, current))
            self.assertEqual(json.loads((root / 'docker/dependencies.lock.json').read_text()), lock)
            vendor = json.loads((root / 'docker/gz.repos').read_text())['repositories']['gz_libs/example']
            self.assertEqual(vendor['version'], lock['vendors']['example']['commit'])

    def test_unexpected_dockerfile_is_rejected_before_lock_mutation(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / 'Dockerfile').write_text('FROM ubuntu:24.04\n')
            (root / 'docker').mkdir()
            lock_path = root / 'docker/dependencies.lock.json'
            lock_path.write_text('original\n')
            with self.assertRaises(ValueError):
                dependency_lock.write_lock(root, {'ros_image': 'ros:jazzy-ros-base@sha256:' + 'b' * 64,
                                                  'vendors': {}})
            self.assertEqual(lock_path.read_text(), 'original\n')

    def test_non_digest_base_is_rejected(self):
        with self.assertRaises(ValueError):
            dependency_lock.update_base_image('', 'ros:jazzy-ros-base:latest')


if __name__ == '__main__':
    unittest.main()
