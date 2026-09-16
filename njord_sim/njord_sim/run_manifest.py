"""Atomic run handoff; autonomy receives only public mission metadata."""
import json
import hashlib
import os
from pathlib import Path
import time


def atomic_text(path, text):
    path = Path(path)
    temporary = path.with_name(path.name + '.tmp')
    temporary.write_text(text)
    temporary.replace(path)


def publish_ready(output, run_id, scenario, manifest_sha256=None):
    if not run_id:
        raise ValueError('RUN_ID is required; use scripts/njord or set a unique run identifier')
    # Do not expose hidden gate positions to the mission process.
    metadata = {'run_id': run_id, 'expected_gates': len(scenario['gates'])}
    if manifest_sha256 is not None:
        metadata['manifest_sha256'] = manifest_sha256
    atomic_text(Path(output) / 'run_ready.json', json.dumps(metadata) + '\n')


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def freeze_resources(output, resolved):
    """Copy checksummed inputs; generated meshes reference only the run copy."""
    output = Path(output)
    directory = output/'resources'
    directory.mkdir(exist_ok=True)
    replacements = {}
    for source, expected in resolved.get('resources', {}).items():
        source = Path(source)
        data = source.read_bytes()
        if hashlib.sha256(data).hexdigest() != expected:
            raise ValueError(f'configuration resource changed during resolution: {source}')
        destination = directory / (expected + source.suffix)
        destination.write_bytes(data)
        replacements[str(source)] = str(destination.resolve())
    def rewrite(value):
        if isinstance(value, dict):
            for key, child in value.items():
                if key == 'uri' and isinstance(child, str):
                    value[key] = replacements.get(child, child)
                else:
                    rewrite(child)
        elif isinstance(value, list):
            for child in value:
                rewrite(child)
    rewrite(resolved['vessel'])


def write_manifest(output, run_id, resolved, sources, public_parameters):
    """Freeze inputs and generated artifacts before publishing the run handoff.

    The full manifest is evaluation-only. Autonomy reads public_parameters.json,
    which contains no scenario geometry, and receives only the manifest digest.
    """
    output = Path(output)
    source_dir = output / 'source_config'
    source_dir.mkdir(exist_ok=True)
    snapshots = {}
    for name, path in sources.items():
        destination = source_dir / (name + '.yaml')
        destination.write_bytes(Path(path).read_bytes())
        expected = resolved.get('resources', {}).get(str(Path(path).resolve()))
        if expected is not None and sha256(destination) != expected:
            raise ValueError(f'source configuration changed during startup: {path}')
        snapshots[name] = {'path': str(destination.relative_to(output)), 'sha256': sha256(destination)}
    atomic_text(output/'resolved_configuration.json', json.dumps(resolved, indent=2, allow_nan=False)+'\n')
    atomic_text(output/'public_parameters.json', json.dumps(public_parameters, indent=2, allow_nan=False)+'\n')
    immutable = {'wamv.sdf', 'wamv.urdf', 'bridges.yaml', 'vessel_config.yaml',
                 'njord_course.sdf', 'resolved_configuration.json', 'resolved_scenario.json',
                 'scenario.sha256', 'public_parameters.json'}
    artifacts = {str(path.relative_to(output)): sha256(path)
                 for path in sorted(output.rglob('*')) if path.is_file()
                 and (path.parent == output and path.name in immutable
                      or path.relative_to(output).parts[0] in ('resources', 'source_config'))}
    manifest = {
        'schema_version': 1, 'run_id': run_id, 'seed': resolved['scenario']['seed'],
        'sources': snapshots, 'artifacts': artifacts,
        'resources': resolved.get('resources', {}),
        'image_identity': os.environ.get('IMAGE_ID', 'unknown'),
        'image_source_commit': os.environ.get('NJORD_IMAGE_SOURCE_COMMIT', 'unknown'),
        'image_source_digest': os.environ.get('NJORD_IMAGE_SOURCE_DIGEST', 'unknown'),
        'runner_git_commit': os.environ.get('RUNNER_GIT_COMMIT', 'unknown'),
    }
    atomic_text(output/'run_manifest.json', json.dumps(manifest, indent=2, allow_nan=False)+'\n')
    return sha256(output/'run_manifest.json')


def verify_public_handoff(output, metadata):
    """Verify public inputs against the run identity without loading scenario data."""
    output = Path(output)
    if 'manifest_sha256' not in metadata:
        return  # Compatibility with pre-manifest runs.
    if sha256(output/'run_manifest.json') != metadata['manifest_sha256']:
        raise ValueError('run manifest changed after readiness')
    manifest = json.loads((output/'run_manifest.json').read_text())
    if manifest['run_id'] != metadata['run_id']:
        raise ValueError('run manifest identity mismatch')
    for name, expected in manifest['artifacts'].items():
        if sha256(output/name) != expected:
            raise ValueError(f'{name} changed after readiness')


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
                verify_public_handoff(output, metadata)
                return metadata
        except (OSError, ValueError):
            pass
        if time.monotonic() >= deadline:
            raise TimeoutError('Simulator did not publish matching run metadata within the startup timeout')
        time.sleep(min(0.1, max(0.0, deadline-time.monotonic())))
