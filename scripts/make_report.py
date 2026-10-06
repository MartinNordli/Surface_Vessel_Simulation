#!/usr/bin/env python3
"""Rebuild report.html of a finished run from its metrics and time series.

Usage: ``./scripts/njord report [run-dir]`` or
``python3 scripts/make_report.py outputs/run-<stamp>``. Runs on the host with
plain Python 3 (no ROS, no third-party packages). Reads
``<run-dir>/run_metrics.json`` and ``<run-dir>/timeseries.csv`` (written by
the evaluator) and writes ``<run-dir>/report.html``; the evaluator already
writes the same report at the end of each run.
"""
import argparse
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'njord_sim'))
from njord_sim.report_core import write_report  # noqa: E402


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('run_dir', type=Path, help='run output directory, e.g. outputs/run-20261006T120000')
    args = parser.parse_args()
    metrics = args.run_dir / 'run_metrics.json'
    if not metrics.is_file():
        parser.error(f'{metrics} does not exist; the run has no evaluator result')
    print(write_report(metrics, args.run_dir / 'timeseries.csv', args.run_dir / 'report.html'))


if __name__ == '__main__':
    main()
