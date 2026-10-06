#!/usr/bin/env python3
"""Wait for resolved scenario before starting the scorer; bound infrastructure wait.

Entry point of the Compose ``evaluator`` service (runs inside the container).
The ``simulator`` service must be running with the same OUTPUT_DIR and RUN_ID.
Waits up to 120 s of steady wall time for its run_ready.json (a TimeoutError
ends the service), then replaces itself with the ``evaluator`` node on
simulation time.

Inputs (environment): OUTPUT_DIR (default /outputs), RUN_ID (required),
RUN_LABEL (default "demo"), PROFILE (default "fast") and STATE_SOURCE
(default "estimate"), all recorded in the metrics; AUTONOMY and CONTROLLER,
which with the course kind from run_ready.json decide the heartbeats the race
waits for (configuration.guard_requirements); and EVALUATOR_MODE: "race"
(default) scores the scenario, "observe" (./scripts/njord lab) scores targets
sent during a free run and never ends it. The evaluator reads
OUTPUT_DIR/resolved_scenario.json (ground truth, evaluation only) and writes
OUTPUT_DIR/run_metrics.json, timeseries.csv and report.html.

Exit code: the evaluator's; 0 means the course was completed (or the
observation stopped cleanly), 2 that it was not (collision, timeout, invalid
gate order, missed target, ...). ``scripts/njord demo`` and the benchmark use
it as the race result.
"""
import os
from pathlib import Path
from ament_index_python.packages import get_package_prefix
from njord_sim.configuration import guard_requirements
from njord_sim.run_manifest import wait_ready

path = Path(os.environ.get('OUTPUT_DIR', '/outputs'))
# Also verifies the run manifest checksums; raises TimeoutError after 120 s.
metadata = wait_ready(path, os.environ.get('RUN_ID', ''))
components = {'autonomy': os.environ.get('AUTONOMY', 'reference'),
              'controller': os.environ.get('CONTROLLER', 'reference'),
              'course': metadata.get('course', 'gates')}
# The executable itself, not `ros2 run`: SIGTERM from `docker compose stop`
# must reach the evaluator so it writes the result of an interrupted run.
executable = str(Path(get_package_prefix('njord_sim')) / 'lib' / 'njord_sim' / 'evaluator')
args = [executable, '--ros-args', '-p', 'use_sim_time:=true',
        '-p', f'scenario_file:={path}/resolved_scenario.json', '-p', f'output:={path}/run_metrics.json',
        '-p', 'run_label:='+os.environ.get('RUN_LABEL', 'demo'),
        '-p', 'profile:='+os.environ.get('PROFILE', 'fast'),
        '-p', 'state_source:='+os.environ.get('STATE_SOURCE', 'estimate'),
        '-p', 'mode:='+os.environ.get('EVALUATOR_MODE', 'race'),
        '-p', 'required_status:=' + str(guard_requirements(components, 'race')['required_status'])]
# exec replaces this process, so the node's exit code is the container's.
os.execvp(args[0], args)
