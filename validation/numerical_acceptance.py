"""Compare isolated simulator reports at 4, 2 and 1 ms; never infer sea fidelity.

Input JSON: {"metrics": {metric_name: "m"|"m/s"|"rad/s"}, "runs": [...]}
Each run provides experiment, step_s, repetition, comparison_group, provenance,
complete and metrics. comparison_group identifies identical resolved settings
except time step; provenance records each run's manifest hash. Requires >=3
matched repetitions per experiment and step. All matched 2/1 ms pairs must pass.

This is a time-step convergence check: it shows that halving the physics step
from 2 ms to 1 ms no longer changes the measured result by more than the
tolerance. It says nothing about how well the model matches real water.

Where to run: anywhere with Python 3 (host or container); no ROS or simulator.
``python3 validation/numerical_acceptance.py comparison.json`` prints the
result as JSON. ``scripts/dynamics_campaign.py`` imports ``compare`` directly.
Exit code: 0 when accepted, 2 when not (including any incomplete run).

Tolerance per metric: max(absolute tolerance for its unit, 2 % of the 1 ms
value). The 4 ms runs must exist and be matched, but are not compared.
"""
import argparse
import json
import math
from pathlib import Path

# Absolute tolerance floor per unit, used when 2 % of the value is smaller.
ABSOLUTE = {"m": 0.02, "m/s": 0.01, "rad/s": 0.001}
# Physics time steps of a campaign in seconds (4, 2 and 1 ms).
STEPS = (0.004, 0.002, 0.001)


def compare(runs, metrics):
    """Check 2 ms against 1 ms results for every matched repetition.

    Args:
        runs: list of run dicts (see the module docstring for the fields).
        metrics: {metric name: unit}; each unit must be a key of ABSOLUTE.

    Runs are grouped by (experiment, comparison_group) and matched by
    repetition index across the three steps. Returns a dict with ``passed``
    (True only if at least one comparison was made, nothing is incomplete and
    every comparison passed), ``incomplete_reasons``, the individual
    ``comparisons`` and the tolerances used. Raises ValueError for an unknown
    unit.
    """
    if not metrics or any(unit not in ABSOLUTE for unit in metrics.values()):
        raise ValueError("explicit metric units must be m, m/s, or rad/s")
    groups = {}
    reasons = []
    for run in runs:
        key = (run.get("experiment"), run.get("comparison_group"))
        step, rep = run.get("step_s"), run.get("repetition")
        if not all(key) or not run.get("provenance") or step not in STEPS or not isinstance(rep, int) or isinstance(rep, bool) or rep < 0:
            reasons.append("missing provenance/group/experiment or invalid timestep/repetition")
            continue
        group = groups.setdefault(key, {s: {} for s in STEPS})
        if rep in group[step]:
            reasons.append(f"duplicate run: {key}, {step}, {rep}")
        group[step][rep] = run
        if run.get("complete") is not True:
            reasons.append(f"incomplete run: {key}, {step}, {rep}")
        for name in metrics:
            value = run.get("metrics", {}).get(name)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                reasons.append(f"missing/nonfinite metric {name}: {key}, {step}, {rep}")
    comparisons = []
    if not groups:
        reasons.append("no experiments")
    for key, steps in groups.items():
        ids = [set(steps[s]) for s in STEPS]
        if min(map(len, ids)) < 3 or not ids[0] == ids[1] == ids[2]:
            reasons.append(f"need >=3 matched repetitions at 4/2/1 ms: {key}")
        # Compare each repetition present at both 2 ms and 1 ms.
        for rep in sorted(ids[1] & ids[2]):
            for name, unit in metrics.items():
                coarse = steps[0.002][rep].get("metrics", {}).get(name)
                fine = steps[0.001][rep].get("metrics", {}).get(name)
                if any(isinstance(v, bool) or not isinstance(v, (float, int)) or not math.isfinite(v) for v in (coarse, fine)):
                    continue
                tolerance = max(ABSOLUTE[unit], 0.02 * abs(fine))
                delta = abs(coarse - fine)
                comparisons.append({"experiment": key[0], "comparison_group": key[1],
                                    "repetition": rep, "metric": name, "difference": delta,
                                    "tolerance": tolerance, "passed": delta <= tolerance})
    return {"passed": bool(comparisons) and not reasons and all(c["passed"] for c in comparisons),
            "incomplete_reasons": reasons, "comparisons": comparisons,
            "relative_tolerance": 0.02, "absolute_tolerances": ABSOLUTE,
            "evidence_level": "numerical_convergence_only"}


def main():
    """Read the comparison JSON file named on the command line and exit 0 or 2."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    args = parser.parse_args()
    data = json.loads(args.input.read_text())
    result = compare(data["runs"], data["metrics"])
    print(json.dumps(result, indent=2, allow_nan=False))
    raise SystemExit(0 if result["passed"] else 2)


if __name__ == "__main__":
    main()
