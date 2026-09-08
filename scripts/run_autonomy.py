#!/usr/bin/env python3
"""Start autonomy after the simulator publishes sanitized mission metadata."""
import os
import sys
from njord_sim.run_manifest import wait_ready

metadata = wait_ready(os.environ.get('OUTPUT_DIR', '/outputs'), os.environ.get('RUN_ID', ''))
args = ['ros2', 'launch', 'njord_sim', 'dstar_demo.launch.py',
        'expected_gates:=' + str(metadata['expected_gates']),
        'autonomy:=' + os.environ.get('AUTONOMY', 'reference'), *sys.argv[1:]]
os.execvp(args[0], args)
