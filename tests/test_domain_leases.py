"""Real advisory-lock regressions for independent benchmark invocations."""
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import unittest
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('benchmark_domains', ROOT / 'scripts/benchmark.py')
benchmark = importlib.util.module_from_spec(spec)
spec.loader.exec_module(benchmark)


class DomainLeaseTests(unittest.TestCase):
    def child(self, directory, count, pool, abrupt=False):
        script = '''
import json, os, sys
sys.path.insert(0, sys.argv[1])
from benchmark import DomainLeases, DomainUnavailable
try:
    lease = DomainLeases(int(sys.argv[3]), sys.argv[2], json.loads(sys.argv[4])).acquire()
except DomainUnavailable as error:
    print(str(error), flush=True)
    sys.exit(2)
print(json.dumps(lease.domains), flush=True)
if sys.argv[5] == 'abrupt':
    os._exit(0)
lease.close()
'''
        return subprocess.run([sys.executable, '-c', script, str(ROOT / 'scripts'), directory,
                               str(count), json.dumps(pool), 'abrupt' if abrupt else 'normal'],
                              capture_output=True, text=True, timeout=10)

    def test_two_processes_receive_disjoint_domains(self):
        with tempfile.TemporaryDirectory() as directory:
            with benchmark.DomainLeases(1, directory, [60, 61, 62]) as parent:
                child = self.child(directory, 2, [60, 61, 62])
                self.assertEqual(child.returncode, 0, child.stderr)
                child_domains = json.loads(child.stdout)
                self.assertEqual(parent.domains, [60])
                self.assertEqual(child_domains, [61, 62])
                self.assertTrue(set(parent.domains).isdisjoint(child_domains))
            with benchmark.DomainLeases(3, directory, [60, 61, 62]) as fresh:
                self.assertEqual(fresh.domains, [60, 61, 62])

    def test_exhaustion_is_clear_and_releases_partial_acquisition(self):
        with tempfile.TemporaryDirectory() as directory:
            with benchmark.DomainLeases(1, directory, [61]):
                child = self.child(directory, 2, [60, 61])
                self.assertEqual(child.returncode, 2)
                self.assertIn('Need 2 free ROS domains; only 1 available', child.stdout)
                with benchmark.DomainLeases(1, directory, [60]) as available:
                    self.assertEqual(available.domains, [60])

    def test_locks_release_on_process_exit_without_python_cleanup(self):
        with tempfile.TemporaryDirectory() as directory:
            child = self.child(directory, 1, [60], abrupt=True)
            self.assertEqual(child.returncode, 0, child.stderr)
            with benchmark.DomainLeases(1, directory, [60]) as fresh:
                self.assertEqual(fresh.domains, [60])
            self.assertTrue((Path(directory) / 'domain-60.lock').exists())

    def test_context_error_releases_all_leases(self):
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaisesRegex(RuntimeError, 'failed run'):
                with benchmark.DomainLeases(2, directory, [60, 61]):
                    raise RuntimeError('failed run')
            with benchmark.DomainLeases(2, directory, [60, 61]) as fresh:
                self.assertEqual(fresh.domains, [60, 61])

    def test_run_saves_resolved_compose_with_actual_domain_and_image(self):
        configuration = {'services': {'simulator': {'image': 'sha256:test',
            'environment': {'ROS_DOMAIN_ID': '73', 'FASTDDS_BUILTIN_TRANSPORTS': 'UDPv4'}}}}
        process = Mock(returncode=0)
        process.poll.return_value = 0
        with tempfile.TemporaryDirectory() as directory:
            with patch.object(benchmark, 'command_output', return_value=json.dumps(configuration)) as config, \
                    patch.object(benchmark.subprocess, 'Popen', return_value=process), \
                    patch.object(benchmark.subprocess, 'run', return_value=Mock(returncode=0)):
                benchmark.run_one(0, ('calm', 1, 'fast'), 1, Path(directory),
                                  {'NJORD_IMAGE': 'sha256:test', 'IMAGE_ID': 'sha256:test'},
                                  10, 73, threading.Event())
            saved = json.loads((Path(directory) / 'calm-1-fast/compose.resolved.json').read_text())
            self.assertEqual(saved, configuration)
            command, environment = config.call_args.args
            self.assertEqual(command[-3:], ['config', '--format', 'json'])
            self.assertEqual(environment['ROS_DOMAIN_ID'], '73')
            self.assertEqual(environment['NJORD_IMAGE'], 'sha256:test')
            self.assertEqual(environment['RUN_ID'], environment['COMPOSE_PROJECT_NAME'])

    def test_config_failure_still_runs_project_cleanup(self):
        with tempfile.TemporaryDirectory() as directory:
            with patch.object(benchmark, 'command_output', side_effect=subprocess.CalledProcessError(42, ['config'], output='invalid configuration')), \
                    patch.object(benchmark.subprocess, 'Popen') as start, \
                    patch.object(benchmark.subprocess, 'run', return_value=Mock(returncode=0)) as cleanup:
                metrics = benchmark.run_one(0, ('calm', 1, 'fast'), 1, Path(directory), {},
                                            10, 73, threading.Event())
            start.assert_not_called()
            cleanup.assert_called_once()
            self.assertIn('down', cleanup.call_args.args[0])
            self.assertEqual(metrics['status'], 'compose_config_exit_42')


if __name__ == '__main__':
    unittest.main()
