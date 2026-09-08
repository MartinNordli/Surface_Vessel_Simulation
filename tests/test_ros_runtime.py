"""Subprocess boundary keeps actual ROS transport tests separate from doubles."""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest


class RosRuntimeIntegration(unittest.TestCase):
    def test_actual_ros_transport(self):
        python = os.environ.get('NJORD_ROS_PYTHON', '/usr/bin/python3')
        probe = subprocess.run([python, '-c', 'import rclpy, sensor_msgs_py, tf2_ros'],
                               capture_output=True, text=True)
        if probe.returncode:
            self.skipTest('ROS Python dependencies unavailable; run inside the Jazzy container')
        repo = Path(__file__).resolve().parents[1]
        env = os.environ.copy()
        env['ROS_DOMAIN_ID'] = str(100 + os.getpid() % 100)
        env['ROS_LOCALHOST_ONLY'] = '1'
        with tempfile.TemporaryDirectory(prefix='njord-ros-test-') as logs:
            env['ROS_LOG_DIR'] = logs
            result = subprocess.run([python, str(repo / 'validation/runtime_smoke.py')],
                                    cwd=repo, env=env, capture_output=True, text=True, timeout=90)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)


if __name__ == '__main__':
    unittest.main()
