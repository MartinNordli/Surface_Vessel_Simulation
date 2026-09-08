#!/usr/bin/env python3
"""Wait for resolved scenario before starting the scorer; bound infrastructure wait."""
import os
from pathlib import Path
import time

path = Path(os.environ.get('OUTPUT_DIR', '/outputs'))
deadline = time.monotonic() + 120
while not (path/'resolved_scenario.json').exists():
    if time.monotonic() > deadline:
        raise SystemExit('Simulator did not generate its scenario within 120 seconds')
    time.sleep(0.1)
args = ['ros2', 'run', 'njord_sim', 'evaluator', '--ros-args', '-p', 'use_sim_time:=true',
        '-p', f'scenario_file:={path}/resolved_scenario.json', '-p', f'output:={path}/run_metrics.json',
        '-p', 'run_label:='+os.environ.get('RUN_LABEL', 'demo'),
        '-p', 'profile:='+os.environ.get('PROFILE', 'fast')]
os.execvp(args[0], args)
