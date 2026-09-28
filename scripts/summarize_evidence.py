#!/usr/bin/env python3
"""Write a compact, reviewable evidence summary to docs/evidence/<name>.json.

Raw runs stay in the git-ignored outputs/. A summary keeps what a reader needs
to check a claim in the documentation without the local files: for every JSON
file directly in a run directory (or given directly) its scalar results, with
nested values kept to MAX_DEPTH and lists longer than MAX_LIST dropped (sample
tables), and for a log file its last lines. Every source is recorded with its
path and SHA-256, so the summary can be matched to the retained raw file.

Usage (on the host, from the checkout):
    python3 scripts/summarize_evidence.py --name njord-debug-race outputs/njord-debug-race
    python3 scripts/summarize_evidence.py --name final-container-suite outputs/x.log

Summaries are documentation: create them only with this script, commit them
with the documentation that cites them, and never edit them by hand.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[1]
SCHEMA_VERSION = 1
MAX_DEPTH = 3  # nesting kept from each JSON source
MAX_LIST = 10  # longer lists are sample data and are dropped
LOG_TAIL_LINES = 8
NAME = re.compile(r'^[a-z0-9][a-z0-9-]*$')
# An absolute path up to and including an ``outputs/`` directory.
OUTPUTS_PATH = re.compile(r'(?:/[^/\s"\']+)+/outputs/')


def scrub(text):
    """Local paths cut to ``outputs/...``; any other home-directory path as ``~``."""
    text = OUTPUTS_PATH.sub('outputs/', text)
    return text.replace(str(Path.home()), '~')


def compact(value, depth=0):
    """Scalars and short structures of ``value``, paths scrubbed; None marks dropped data."""
    if isinstance(value, str):
        return scrub(value)
    if isinstance(value, dict):
        if depth >= MAX_DEPTH:
            return None
        kept = {scrub(key): compact(item, depth+1) for key, item in value.items()}
        return {key: item for key, item in kept.items() if item is not None}
    if isinstance(value, list):
        if len(value) > MAX_LIST or depth >= MAX_DEPTH:
            return None
        return [compact(item, depth+1) for item in value]
    return value


def display_path(path):
    """Path from ``outputs/`` onward (or relative to the checkout); never a home directory."""
    parts = path.resolve().parts
    if 'outputs' in parts:
        return str(Path(*parts[len(parts)-1-parts[::-1].index('outputs'):]))
    if path.resolve().is_relative_to(ROOT):
        return str(path.resolve().relative_to(ROOT))
    return path.name


def source(path):
    """Summary of one JSON or text file, with its repository path and digest."""
    data = path.read_bytes()
    entry = {'path': display_path(path), 'sha256': hashlib.sha256(data).hexdigest(), 'bytes': len(data)}
    if path.suffix == '.json':
        entry['content'] = compact(json.loads(data))
    else:
        entry['tail'] = [scrub(line) for line in data.decode(errors='replace').splitlines()[-LOG_TAIL_LINES:]]
    return entry


def collect(paths):
    """Files to summarize: given files, and the JSON files directly in given directories."""
    files = []
    for path in paths:
        if path.is_dir():
            files += sorted(p for p in path.iterdir() if p.suffix == '.json' and p.is_file())
        elif path.is_file():
            files.append(path)
        else:
            raise FileNotFoundError(path)
    if not files:
        raise ValueError('no JSON or log files to summarize')
    return files


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--name', required=True, help='summary name: lower case, digits and hyphens')
    parser.add_argument('--note', default='', help='one sentence on what the evidence shows')
    parser.add_argument('--output-dir', type=Path, default=ROOT / 'docs/evidence')
    parser.add_argument('paths', nargs='+', type=Path)
    args = parser.parse_args(argv)
    if not NAME.match(args.name):
        parser.error('--name must be lower case letters, digits and hyphens')
    summary = {'schema_version': SCHEMA_VERSION, 'name': args.name, 'note': args.note,
               'created_utc': datetime.now(timezone.utc).isoformat(timespec='seconds'),
               'sources': [source(path) for path in collect(args.paths)]}
    args.output_dir.mkdir(parents=True, exist_ok=True)
    output = args.output_dir / f'{args.name}.json'
    output.write_text(json.dumps(summary, indent=2, allow_nan=False, sort_keys=True) + '\n')
    print(f'Wrote {output}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
