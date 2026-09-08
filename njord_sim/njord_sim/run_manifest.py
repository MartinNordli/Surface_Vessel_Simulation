"""Atomic run handoff; autonomy receives only public mission metadata."""
import json
from pathlib import Path
import time


def atomic_text(path, text):
    path = Path(path)
    temporary = path.with_name(path.name + '.tmp')
    temporary.write_text(text)
    temporary.replace(path)


def publish_ready(output, run_id, scenario):
    if not run_id:
        raise ValueError('RUN_ID is required; use scripts/njord or set a unique run identifier')
    # Do not expose hidden gate positions to the mission process.
    metadata = {'run_id': run_id, 'expected_gates': len(scenario['gates'])}
    atomic_text(Path(output) / 'run_ready.json', json.dumps(metadata) + '\n')


def wait_ready(output, run_id, timeout=120.0):
    if not run_id:
        raise ValueError('RUN_ID is required; use scripts/njord or set a unique run identifier')
    deadline = time.monotonic() + timeout
    while True:
        try:
            metadata = json.loads((Path(output) / 'run_ready.json').read_text())
            if (metadata.get('run_id') == run_id
                    and type(metadata.get('expected_gates')) is int
                    and metadata['expected_gates'] > 0):
                return metadata
        except (OSError, ValueError):
            pass
        if time.monotonic() >= deadline:
            raise TimeoutError('Simulator did not publish matching run metadata within the startup timeout')
        time.sleep(min(0.1, max(0.0, deadline-time.monotonic())))
