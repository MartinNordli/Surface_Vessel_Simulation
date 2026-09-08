"""Keep real rclpy evaluator tests in an isolated process and ROS domain."""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest


class EvaluatorRuntimeIntegration(unittest.TestCase):
    def test_actual_evaluator_readiness(self):
        python = os.environ.get("NJORD_ROS_PYTHON", "/usr/bin/python3")
        probe = subprocess.run([python, "-c", "import rclpy, ros_gz_interfaces"],
                               capture_output=True, text=True)
        if probe.returncode:
            self.skipTest("Jazzy ROS dependencies unavailable; run inside the simulator container")
        root = Path(__file__).resolve().parents[1]
        environment = os.environ.copy()
        environment["ROS_DOMAIN_ID"] = str(160 + os.getpid() % 40)
        environment["ROS_LOCALHOST_ONLY"] = "1"
        with tempfile.TemporaryDirectory(prefix="njord-evaluator-log-") as logs:
            environment["ROS_LOG_DIR"] = logs
            result = subprocess.run([python, str(root / "tests/evaluator_ros_smoke.py")], cwd=root,
                                    env=environment, capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)


if __name__ == "__main__":
    unittest.main()
