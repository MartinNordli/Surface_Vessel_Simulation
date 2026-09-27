#!/usr/bin/env python3
"""Wait for resolved scenario before starting the scorer; bound infrastructure wait.

Entry point of the Compose ``evaluator`` service (runs inside the container).
The ``simulator`` service must be running with the same OUTPUT_DIR and RUN_ID.
Waits up to 120 s of steady wall time for its run_ready.json (a TimeoutError
ends the service), then replaces itself with the ``evaluator`` node on
simulation time.

Inputs (environment): OUTPUT_DIR (default /outputs), RUN_ID (required),
RUN_LABEL (default "demo"), PROFILE (default "fast") and STATE_SOURCE
(default "estimate"), all recorded in the metrics. The evaluator reads OUTPUT_DIR/resolved_scenario.json (ground truth,
evaluation only) and writes OUTPUT_DIR/run_metrics.json.

Exit code: the evaluator's; 0 means the course was completed, 2 that it was
not (collision, timeout, invalid gate order, ...). ``scripts/njord
demo`` and the benchmark use it as the race result.
"""
import os
from pathlib import Path
from njord_sim.run_manifest import wait_ready

path = Path(os.environ.get('OUTPUT_DIR', '/outputs'))
# Also verifies the run manifest checksums; raises TimeoutError after 120 s.
wait_ready(path, os.environ.get('RUN_ID', ''))
args = ['ros2', 'run', 'njord_sim', 'evaluator', '--ros-args', '-p', 'use_sim_time:=true',
        '-p', f'scenario_file:={path}/resolved_scenario.json', '-p', f'output:={path}/run_metrics.json',
        '-p', 'run_label:='+os.environ.get('RUN_LABEL', 'demo'),
        '-p', 'profile:='+os.environ.get('PROFILE', 'fast'),
        '-p', 'state_source:='+os.environ.get('STATE_SOURCE', 'estimate')]
# exec replaces this process, so the node's exit code is the container's.
os.execvp(args[0], args)
