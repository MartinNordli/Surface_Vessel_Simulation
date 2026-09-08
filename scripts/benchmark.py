#!/usr/bin/env python3
"""Run paired, isolated Docker races and report completion and time evidence.

Defaults: 10 seeds x 2 environments x 2 profiles (40 races), one job at a time.
Use --jobs 4 to run four races concurrently. Each race uses a
fresh Compose project, Gazebo partition, ROS domain and output directory.
COMPOSE_FILE supports additional platform overlays, e.g. compose.wsl.yaml.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import statistics
import queue
import threading
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]


def safe_run(metrics):
    return (metrics.get("status") == "completed" and metrics.get("collision") is False
            and metrics.get("contact_status") == "observed" and not metrics.get("geometric_overlap", True))


def summarize(runs):
    groups = {}
    for run in runs:
        key = f'{run["environment"]}/{run["profile"]}'
        groups.setdefault(key, []).append(run)
    summary = {}
    for key, group in groups.items():
        successful = [r for r in group if safe_run(r)]
        summary[key] = {"runs": len(group), "verified_completions": len(successful),
                        "completion_rate": len(successful) / len(group),
                        "median_time_s": statistics.median(r["time_s"] for r in successful) if successful else None,
                        "contacts": sum(bool(r.get("collision")) for r in group),
                        "contact_unavailable_runs": sum(r.get("contact_status") != "observed" for r in group),
                        "failed_labels": [r["label"] for r in group if not safe_run(r)]}
    comparisons = {}
    for environment in sorted({r["environment"] for r in runs}):
        by_key = {(r["seed"], r["profile"]): r for r in runs if r["environment"] == environment}
        paired = [(by_key[(seed, "conservative")], by_key[(seed, "fast")])
                  for seed in sorted({r["seed"] for r in runs if r["environment"] == environment})
                  if (seed, "conservative") in by_key and (seed, "fast") in by_key]
        safe_pairs = [(a, b) for a, b in paired if safe_run(a) and safe_run(b)]
        gains = [a["time_s"] - b["time_s"] for a, b in safe_pairs]
        conservative = [a["time_s"] for a, _ in safe_pairs]
        fast = [b["time_s"] for _, b in safe_pairs]
        comparisons[environment] = {
            "pairs": len(paired), "verified_pairs": len(safe_pairs),
            "median_paired_time_saving_s": statistics.median(gains) if gains else None,
            "fast_improves_median_without_failures": bool(paired) and len(safe_pairs) == len(paired)
            and statistics.median(fast) < statistics.median(conservative),
        }
    return {"groups": summary, "comparisons": comparisons,
            "all_runs_verified": bool(runs) and all(safe_run(r) for r in runs)}


def command_output(args, env=None):
    return subprocess.check_output(args, cwd=ROOT, env=env, text=True, stderr=subprocess.STDOUT).strip()


def pin_image(environment):
    """Resolve once and pin every race service to immutable image contents."""
    def race_images():
        configuration = json.loads(command_output(['docker', 'compose', 'config', '--format', 'json'], environment))
        return {configuration['services'][service]['image'] for service in ('simulator', 'autonomy', 'evaluator')}

    images = race_images()
    if len(images) != 1:
        raise ValueError('Benchmark race services must use the same simulator image')
    inspection = json.loads(command_output(['docker', 'image', 'inspect', images.pop()], environment))[0]
    image_id = inspection['Id']
    labels = inspection.get('Config', {}).get('Labels') or {}
    environment['NJORD_IMAGE'] = image_id
    environment['IMAGE_ID'] = image_id
    if race_images() != {image_id}:
        raise ValueError('Compose overrides must honor NJORD_IMAGE for all race services')
    return {'image_identity': image_id,
            'image_source_commit': labels.get('org.opencontainers.image.revision', 'unknown')
            if 'io.njord.source.digest' in labels else 'unknown',
            'image_source_digest': labels.get('io.njord.source.digest', 'unknown')}


def run_one(index, item, total, output, base_env, wall_timeout, domain_id, cancelled):
    environment, seed, profile = item
    label = f"{environment}-{seed}-{profile}"
    run_dir = output / label
    run_dir.mkdir()
    project = f"njord-bench-{os.getpid()}-{index}"
    env = base_env | {"OUTPUT_HOST": str(run_dir), "OUTPUT_DIR": "/outputs", "SEED": str(seed),
                      "ENVIRONMENT": environment, "PROFILE": profile, "RUN_LABEL": label,
                      "COMPOSE_PROJECT_NAME": project, "GZ_PARTITION": project, "RUN_ID": project,
                      "ROS_DOMAIN_ID": str(domain_id)}
    command = ["docker", "compose", "-p", project]
    wall_start = time.monotonic()
    reason = None
    print(f"[{index + 1}/{total}] {label}", flush=True)
    try:
        with (run_dir / "compose.log").open("w") as log:
            process = subprocess.Popen(command + ["up", "--abort-on-container-exit", "--exit-code-from",
                                                   "evaluator", "simulator", "autonomy", "evaluator"],
                                       cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT)
            try:
                while process.poll() is None:
                    if cancelled.wait(0.2):
                        reason = "interrupted"
                        break
                    if time.monotonic() - wall_start >= wall_timeout:
                        reason = "infrastructure_wall_timeout"
                        break
                if reason:
                    process.terminate()
                    try:
                        process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait(timeout=5)
                elif process.returncode:
                    reason = f"compose_exit_{process.returncode}"
            finally:
                if process.poll() is None:
                    process.kill()
                    process.wait(timeout=5)
    except subprocess.TimeoutExpired:
        reason = "infrastructure_wall_timeout"
    except OSError as error:
        reason = f"infrastructure_error: {error}"
    finally:
        # Also runs after Ctrl-C; only this uniquely named project is stopped.
        with (run_dir / "cleanup.log").open("w") as log:
            try:
                cleanup = subprocess.run(command + ["down", "--remove-orphans", "--timeout", "10"],
                                         cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT, timeout=45)
                if cleanup.returncode:
                    reason = reason or "cleanup_failed"
            except subprocess.TimeoutExpired:
                reason = reason or "cleanup_timeout"
            except OSError as error:
                reason = reason or f"cleanup_error: {error}"
    metrics_path = run_dir / "run_metrics.json"
    try:
        metrics = json.loads(metrics_path.read_text(), parse_constant=lambda s: (_ for _ in ()).throw(ValueError(s)))
        if reason:
            metrics["infrastructure_failure"] = reason
            if metrics.get("status") == "completed":
                metrics["status"] = reason
    except (OSError, ValueError):
        metrics = {"status": reason or "missing_metrics", "collision": None,
                   "contact_status": "unavailable", "geometric_overlap": None}
    metrics.update({"label": label, "environment": environment, "seed": seed, "profile": profile,
                    "runner_git_commit": base_env.get("RUNNER_GIT_COMMIT", "unknown"),
                    "image_identity": base_env.get("IMAGE_ID", "unknown"),
                    "benchmark_wall_time_s": time.monotonic() - wall_start})
    return metrics


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seeds", type=int, nargs="+", default=list(range(1, 11)))
    parser.add_argument("--environments", choices=["calm", "moderate"], nargs="+", default=["calm", "moderate"])
    parser.add_argument("--profiles", choices=["conservative", "fast"], nargs="+", default=["conservative", "fast"])
    parser.add_argument("--wall-timeout", type=float, default=600)
    parser.add_argument("--jobs", type=int, default=1, help="Concurrent races (1-100); each holds a distinct ROS domain")
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    if any(seed <= 0 for seed in args.seeds) or args.wall_timeout <= 0:
        parser.error("seeds and timeout must be positive")
    if len(set(args.seeds)) != len(args.seeds):
        parser.error("seeds must be unique")
    if not 1 <= args.jobs <= 100:
        parser.error("jobs must be between 1 and 100")
    if len(set(args.environments)) != len(args.environments) or len(set(args.profiles)) != len(args.profiles):
        parser.error("environments and profiles must be unique")
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S")
    output = (args.output_dir or ROOT / "outputs" / f"benchmark-{stamp}-{os.getpid()}").resolve()
    matrix = [(environment, seed, profile) for environment in args.environments
              for seed in args.seeds for profile in args.profiles]
    if args.dry_run:
        print(json.dumps({"output": str(output), "runs": matrix, "jobs": args.jobs,
                          "command": ["docker", "compose", "up", "--abort-on-container-exit",
                                      "--exit-code-from", "evaluator", "simulator", "autonomy", "evaluator"]}, indent=2))
        return 0
    output.mkdir(parents=True, exist_ok=False)
    base_env = os.environ.copy()
    base_env.setdefault("COMPOSE_FILE", str(ROOT / "compose.yaml"))
    base_env["RUNNER_GIT_COMMIT"] = command_output(["git", "rev-parse", "HEAD"])
    image_metadata = pin_image(base_env)
    manifest = {"runner_git_commit": base_env["RUNNER_GIT_COMMIT"],
                "runner_git_dirty": bool(command_output(["git", "status", "--porcelain"])),
                **image_metadata, "matrix": matrix, "wall_timeout_s": args.wall_timeout, "jobs": args.jobs,
                "compose_file": base_env["COMPOSE_FILE"], "created_utc": stamp}
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    runs = []
    cancelled = threading.Event()
    domains = queue.Queue()
    for domain_id in range(60, 60 + args.jobs):
        domains.put(domain_id)

    def execute(index, item):
        domain_id = domains.get()
        try:
            if cancelled.is_set():
                return None
            return run_one(index, item, len(matrix), output, base_env, args.wall_timeout, domain_id, cancelled)
        finally:
            domains.put(domain_id)

    def write_summary():
        runs.sort(key=lambda run: (args.environments.index(run["environment"]),
                                  args.seeds.index(run["seed"]), args.profiles.index(run["profile"])))
        temporary = output / "summary.json.tmp"
        temporary.write_text(json.dumps({"manifest": manifest, "runs": runs,
                                         **summarize(runs)}, indent=2, allow_nan=False) + "\n")
        temporary.replace(output / "summary.json")

    executor = ThreadPoolExecutor(max_workers=args.jobs)
    futures = [executor.submit(execute, index, item) for index, item in enumerate(matrix)]
    collected = set()
    interrupted = False
    try:
        for future in as_completed(futures):
            metrics = future.result()
            collected.add(future)
            if metrics is not None:
                runs.append(metrics)
                write_summary()
    except KeyboardInterrupt:
        interrupted = True
        cancelled.set()
        print("Stopping benchmark projects and cancelling queued races...", flush=True)
    finally:
        cancelled.set()
        executor.shutdown(wait=True, cancel_futures=True)
        for future in futures:
            if future not in collected and not future.cancelled() and future.exception() is None:
                metrics = future.result()
                if metrics is not None:
                    runs.append(metrics)
        write_summary()
    if interrupted:
        return 130
    report = summarize(runs)
    print(json.dumps(report, indent=2, allow_nan=False))
    print(f"Results: {output / 'summary.json'}")
    paired = set(args.profiles) == {"conservative", "fast"}
    improvements = all(c["fast_improves_median_without_failures"] for c in report["comparisons"].values())
    return 0 if report["all_runs_verified"] and (not paired or improvements) else 2


if __name__ == "__main__":
    sys.exit(main())
