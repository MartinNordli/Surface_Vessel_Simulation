"""Source fingerprints and immutable image pinning, with no Docker dependency."""
import importlib.util
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, ROOT / path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


metadata = load('scripts/build_metadata.py', 'build_metadata')
benchmark = load('scripts/benchmark.py', 'benchmark_metadata')


class BuildMetadataTests(unittest.TestCase):
    def fixture(self, root):
        (root / 'scripts').mkdir()
        (root / 'scripts/run.py').write_text('print("hello")\n')
        (root / 'Dockerfile').write_text('FROM example\nCOPY scripts /scripts\n')
        (root / '.dockerignore').write_text('**/__pycache__\n.git\noutputs\n')

    def test_digest_ignores_timestamps_outputs_docs_and_caches(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.fixture(root)
            before = metadata.source_digest(root)
            os.utime(root / 'scripts/run.py', (1, 1))
            for folder in ('outputs', '.git', 'docs', 'scripts/__pycache__'):
                (root / folder).mkdir(parents=True, exist_ok=True)
                (root / folder / 'generated').write_text('not a Docker input')
            self.assertEqual(before, metadata.source_digest(root))

    def test_dirty_source_new_files_and_executable_modes_change_digest(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.fixture(root)
            original = metadata.source_digest(root)
            (root / 'scripts/run.py').write_text('print("dirty edit")\n')
            dirty = metadata.source_digest(root)
            self.assertNotEqual(original, dirty)
            (root / 'scripts/untracked.py').write_text('new source\n')
            added = metadata.source_digest(root)
            self.assertNotEqual(dirty, added)
            (root / 'scripts/run.py').chmod(0o755)
            self.assertNotEqual(added, metadata.source_digest(root))

    def test_umask_dependent_permissions_do_not_change_digest(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.fixture(root)
            before = metadata.source_digest(root)
            (root / 'scripts/run.py').chmod(0o664)
            (root / 'scripts').chmod(0o775)
            self.assertEqual(before, metadata.source_digest(root))

    def test_build_recipe_is_part_of_digest(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.fixture(root)
            before = metadata.source_digest(root)
            with (root / 'Dockerfile').open('a') as recipe:
                recipe.write('RUN echo different-image\n')
            self.assertNotEqual(before, metadata.source_digest(root))

    def test_copy_parser_covers_directory_file_and_stage_inputs(self):
        recipe = ('COPY njord_sim /app/njord_sim\n'
                  'COPY docker/gz.repos /tmp/gz.repos\n'
                  'COPY --from=build /built /app\n'
                  'COPY ["scripts", "/scripts"]\n')
        self.assertEqual(metadata.copy_sources(recipe), ['docker/gz.repos', 'njord_sim', 'scripts'])

    def test_missing_copy_input_fails(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / 'Dockerfile').write_text('COPY absent /app\n')
            with self.assertRaises(ValueError):
                metadata.source_digest(root)


class ImagePinningTests(unittest.TestCase):
    def test_pin_reports_image_source_separately_from_runner(self):
        image_id = 'sha256:' + 'a'*64
        environment = {'RUNNER_GIT_COMMIT': 'runner-revision'}
        calls = []

        def fake_command(command, env):
            calls.append(command)
            if command[:3] == ['docker', 'compose', 'config']:
                return json.dumps({'services': {service: {'image': env.get('NJORD_IMAGE', 'njord-sim:local')}
                                  for service in ('simulator', 'autonomy', 'evaluator')}})
            return json.dumps([{'Id': image_id, 'Config': {'Labels': {
                'org.opencontainers.image.revision': 'image-revision', 'io.njord.source.digest': 'source-digest'}}}])

        with patch.object(benchmark, 'command_output', fake_command):
            result = benchmark.pin_image(environment)
        self.assertEqual(environment['NJORD_IMAGE'], image_id)
        self.assertEqual(environment['IMAGE_ID'], image_id)
        self.assertEqual(environment['RUNNER_GIT_COMMIT'], 'runner-revision')
        self.assertEqual(result['image_source_commit'], 'image-revision')
        self.assertEqual(result['image_source_digest'], 'source-digest')
        self.assertEqual(len(calls), 3)

    def test_hardcoded_compose_image_cannot_bypass_pinning(self):
        configuration = {'services': {service: {'image': 'mutable-tag'}
                                      for service in ('simulator', 'autonomy', 'evaluator')}}
        def fake_command(command, env):
            if command[:3] == ['docker', 'compose', 'config']:
                return json.dumps(configuration)
            return json.dumps([{'Id': 'sha256:' + 'b'*64, 'Config': {'Labels': {}}}])

        with patch.object(benchmark, 'command_output', fake_command), self.assertRaises(ValueError):
            benchmark.pin_image({})


if __name__ == '__main__':
    unittest.main()
