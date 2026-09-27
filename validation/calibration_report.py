"""Validate real measurement provenance and preregistered holdout comparisons.

Template is deliberately incomplete. Never substitutes simulated measurements
for boat data. Each channel needs unit, frame and standard_uncertainty.
Run files are relative to the manifest, SHA256 sealed, and explicitly real_boat.
Comparisons require measured, predicted, combined_standard_uncertainty and
run_id/channel; acceptance uses only preregistered absolute_error and
uncertainty_sigma limits, with all holdout runs/channels required.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def report(manifest, root, comparisons=()):
    failures = []
    for key in ('boat_revision', 'load_revision', 'frame_convention'):
        if not manifest.get(key):
            failures.append('missing ' + key)
    timing = manifest.get('timing', {})
    if (not timing.get('clock_source') or not timing.get('synchronization_method')
            or not finite(timing.get('maximum_error_s')) or timing['maximum_error_s'] < 0):
        failures.append('missing timing and synchronization uncertainty')
    environment = manifest.get('environment', {})
    if any(environment.get(key) is None for key in ('water_density_kg_m3','wind_mps','current_mps','waves')):
        failures.append('missing environmental observations')
    pre = manifest.get('preregistration', {})
    fit, holdout = pre.get('fit_run_ids', []), pre.get('holdout_run_ids', [])
    if not fit or not holdout or set(fit) & set(holdout):
        failures.append('fit/holdout must be nonempty and disjoint')
    if not pre.get('registered_utc') or len(pre.get('protocol_sha256', '') or '') != 64:
        failures.append('missing sealed preregistration')
    channels = manifest.get('channels', {})
    if not channels:
        failures.append('no measured channels')
    for name, channel in channels.items():
        if not channel.get('unit') or not channel.get('frame') or not finite(channel.get('standard_uncertainty')) or channel['standard_uncertainty'] < 0:
            failures.append('invalid channel metadata: ' + name)
    ids = []
    for run in manifest.get('runs', []):
        ids.append(run.get('id'))
        path = (Path(root) / run.get('path', '')).resolve()
        if (run.get('source') != 'real_boat' or not path.is_relative_to(Path(root).resolve())
                or not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest() != run.get('sha256')):
            failures.append('missing/unsealed real boat run: ' + str(run.get('id')))
    if len(ids) != len(set(ids)) or not set(fit+holdout) <= set(ids):
        failures.append('run ids duplicate or missing fit/holdout files')
    checks, seen = [], set()
    limits = pre.get('acceptance_limits', {})
    for row in comparisons:
        key = (row.get('run_id'), row.get('channel'))
        if key in seen or key[0] not in holdout or key[1] not in channels:
            failures.append('duplicate or non-holdout comparison: ' + str(key))
            continue
        seen.add(key)
        limit = limits.get(key[1], {})
        values = [row.get(k) for k in ('measured','predicted','combined_standard_uncertainty')]
        if (not all(finite(x) for x in values) or values[2] < 0
                or not all(finite(limit.get(k)) and limit[k] >= 0 for k in ('absolute_error','uncertainty_sigma'))):
            failures.append('missing finite values or preregistered limits: ' + str(key))
            continue
        error = abs(values[0]-values[1])
        checks.append({'run_id':key[0], 'channel':key[1], 'absolute_error':error,
                       'passed':error <= limit['absolute_error'] and error <= limit['uncertainty_sigma']*values[2]})
    required = {(run, channel) for run in holdout for channel in channels}
    if not required or seen != required:
        failures.append('incomplete holdout channel comparisons')
    passed = not failures and bool(checks) and all(c['passed'] for c in checks)
    return {'passed':passed, 'status':'holdout_comparison_passed' if passed else 'uncalibrated_or_incomplete',
            'failures':failures, 'comparisons':checks, 'scope':'declared boat/load/environment only'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('manifest', type=Path)
    parser.add_argument('--comparisons', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    try:
        manifest = json.loads(args.manifest.read_text())
        comparisons = json.loads(args.comparisons.read_text()) if args.comparisons else []
        result = report(manifest, args.manifest.parent, comparisons)
    except (OSError, ValueError, TypeError, KeyError) as error:
        result = {'passed':False, 'status':'invalid_input', 'failures':[str(error)]}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x') as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps(result, indent=2, allow_nan=False))
    return 0 if result['passed'] else 2

if __name__ == '__main__':
    raise SystemExit(main())
