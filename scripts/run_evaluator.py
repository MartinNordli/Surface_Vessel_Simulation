#!/usr/bin/env python3
"""Wait for resolved scenario before starting the scorer; bound infrastructure wait."""
import os
from pathlib import Path
from njord_sim.run_manifest import wait_ready

path = Path(os.environ.get('OUTPUT_DIR', '/outputs'))
wait_ready(path, os.environ.get('RUN_ID', ''))
args = ['ros2', 'run', 'njord_sim', 'evaluator', '--ros-args', '-p', 'use_sim_time:=true',
        '-p', f'scenario_file:={path}/resolved_scenario.json', '-p', f'output:={path}/run_metrics.json',
        '-p', 'run_label:='+os.environ.get('RUN_LABEL', 'demo'),
        '-p', 'profile:='+os.environ.get('PROFILE', 'fast')]
os.execvp(args[0], args)
