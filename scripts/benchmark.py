#!/usr/bin/env python3
"""Run paired, isolated Docker races and report completion and time evidence.

Defaults: 10 seeds x 2 environments x every speed profile in algorithms.yaml
(40 races with the shipped fast and conservative), one job at a time.
Use --jobs 4 to run four races concurrently. Each race uses a
fresh Compose project, Gazebo partition, ROS domain and output directory.
COMPOSE_FILE supports additional platform overlays, e.g. compose.wsl.yaml.

Where to run: on the host, from a Git checkout, normally through
``./scripts/njord benchmark [course] [options]`` (which selects the platform
overlay and the course). It needs Docker Compose and a built simulator image;
it starts and stops its own ``simulator``, ``autonomy`` and ``evaluator``
services, so no stack has to be running beforehand.

Isolation per race: the Compose project, GZ_PARTITION and RUN_ID are all
``njord-bench-<pid>-<index>``, and ROS_DOMAIN_ID is a domain leased
exclusively from 60..159 (see DomainLeases), so concurrent races and other
benchmark processes never see each other's topics or containers.

Inputs: the options below plus the caller's environment (COMPOSE_FILE,
SCENARIO, NJORD_IMAGE, VESSEL_CONFIG, ...), which is passed to every race.

Outputs, under --output-dir (default outputs/benchmark-<UTC stamp>-<pid>/):
    manifest.json               runner commit, dirty flag, pinned image, matrix
    summary.json                all run metrics plus the summarize() report;
                                rewritten after every finished race
    <env>-<seed>-<profile>/     per-race directory mounted as /outputs:
        compose.resolved.json   exact Compose configuration used
        compose.log, cleanup.log
        run_metrics.json        written by the evaluator (if the race got that far)

Exit codes: 0 when every run is a verified safe completion and, if both
profiles ran, "fast" has a lower median time than "conservative" in every
environment; 2 otherwise (also when no ROS domains are free); 130 on Ctrl-C.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
from datetime import datetime, timezone
import fcntl
import json
import os
from pathlib import Path
import statistics
import queue
import threading
import subprocess
import sys
import time

import yaml

ROOT = Path(__file__).resolve().parents[1]


def speed_profiles(environment):
    """PROFILE names in the algorithms file the races read, or None if it is not on the host.

    Races read ALGORITHMS_CONFIG (default /config/algorithms.yaml) inside the
    container, where /config is the host's CONFIG_HOST (default
    njord_sim/config). A file elsewhere in the container cannot be checked here;
    the simulator still rejects an unknown profile before Gazebo starts.
    """
    container = environment.get("ALGORITHMS_CONFIG") or "/config/algorithms.yaml"
    if not container.startswith("/config/"):
        return None
    path = ROOT / environment.get("CONFIG_HOST", "njord_sim/config") / container.removeprefix("/config/")
    return sorted(yaml.safe_load(path.read_text())["speed_profiles_mps"])


class DomainUnavailable(RuntimeError):
    """Fewer free ROS domains are left in the lease pool than races requested."""


class DomainLeases:
    """Keep ROS domains exclusive across benchmark processes for this user.

    Each domain in ``pool`` has a lock file ``domain-<id>.lock`` in
    ``directory`` (default /tmp/njord-ros-domains-<uid>). A domain is leased by
    holding a non-blocking exclusive flock on its file, so two benchmark
    processes running at the same time never pick the same ROS_DOMAIN_ID.
    ``acquire`` takes the first ``count`` free domains, or raises
    DomainUnavailable and releases everything it took. Use as a context
    manager; the leased IDs are in ``domains``.

    Lock files remain in place; deleting them could create two independently
    locked inodes for the same domain. The kernel releases locks on exit.
    """
    # The pool 60..159 stays clear of the image default ROS_DOMAIN_ID (42) used
    # by interactive runs. dynamics_campaign.py does not take these leases.
    def __init__(self, count, directory=None, pool=range(60, 160)):
        self.count = count
        self.directory = Path(directory or f'/tmp/njord-ros-domains-{os.getuid()}')
        self.pool = tuple(pool)
        self.domains = []
        self.descriptors = []
        if not 1 <= count <= len(self.pool):
            raise ValueError('Requested domain count exceeds the lease pool')

    def acquire(self):
        """Lock ``count`` free domains from the pool and return self.

        Domains already locked by another process are skipped. Each held lock
        file is overwritten with ``pid=... domain=...`` to help debugging.
        """
        if self.descriptors:
            raise RuntimeError('Domain leases are already held')
        self.directory.mkdir(mode=0o700, parents=True, exist_ok=True)
        try:
            for domain in self.pool:
                descriptor = os.open(self.directory / f'domain-{domain}.lock',
                                     os.O_CREAT | os.O_RDWR | os.O_CLOEXEC, 0o600)
                try:
                    # Non-blocking: a held lock means another benchmark owns it.
                    fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
                except BlockingIOError:
                    os.close(descriptor)
                    continue
                except BaseException:
                    os.close(descriptor)
                    raise
                self.descriptors.append(descriptor)
                self.domains.append(domain)
                os.ftruncate(descriptor, 0)
                os.write(descriptor, f'pid={os.getpid()} domain={domain}\n'.encode())
                if len(self.domains) == self.count:
                    return self
            raise DomainUnavailable(
                f'Need {self.count} free ROS domains; only {len(self.domains)} available '
                f'in {self.pool[0]}..{self.pool[-1]}. '
                'Wait for another benchmark to finish or reduce --jobs.')
        except BaseException:
            self.close()
            raise

    def close(self):
        """Release every held lease (closing the descriptor drops the flock)."""
        for descriptor in self.descriptors:
            os.close(descriptor)
        self.descriptors.clear()
        self.domains.clear()

    def __enter__(self):
        return self.acquire()

    def __exit__(self, *_):
        self.close()


def safe_run(metrics):
    """Return True only for a race that is verified complete and contact free.

    All four must hold; a missing field counts as a failure:
    - status == "completed": the evaluator saw every gate crossed in order in
      the right direction, and no infrastructure failure overrode the status
      (see run_one);
    - collision is False: explicitly no contact (None means unknown);
    - contact_status == "observed": the contact monitor actually reported, so
      "no collision" is measured rather than assumed;
    - geometric_overlap is falsy: the evaluator's independent ground-truth
      check (hull rectangle swept against scenario obstacles) never reached
      zero clearance (a missing value defaults to True, i.e. unsafe).
    """
    return (metrics.get("status") == "completed" and metrics.get("collision") is False
            and metrics.get("contact_status") == "observed" and not metrics.get("geometric_overlap", True))


def summarize(runs):
    """Aggregate run metrics into per-group statistics and profile comparisons.

    Returns a dict with:
    - groups["<environment>/<profile>"]: runs, verified_completions (safe_run
      count), completion_rate (fraction 0..1), median_time_s (median course
      time of safe runs in simulation seconds, first ground-truth sample to
      last gate; None if none), contacts (runs with a reported
      collision), contact_unavailable_runs (contact monitor not observed) and
      failed_labels;
    - comparisons[<environment>]: pairs (seeds that ran both profiles),
      verified_pairs (both runs safe), median_paired_time_saving_s (median of
      conservative minus fast time per seed; positive means fast is quicker)
      and fast_improves_median_without_failures (every pair safe and the fast
      median time is lower);
    - all_runs_verified: at least one run and every run passes safe_run.
    """
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
    sources = sorted({r.get("state_source", "unknown") for r in runs})
    return {"groups": summary, "comparisons": comparisons,
            "state_source": sources[0] if len(sources) == 1 else "mixed",
            "estimator_validation_passed": sources == ["estimate"] and bool(runs) and all(safe_run(r) for r in runs),
            "truth_course_validation_passed": sources == ["truth"] and bool(runs) and all(safe_run(r) for r in runs),
            "all_runs_verified": bool(runs) and all(safe_run(r) for r in runs)}


def command_output(args, env=None):
    """Run a command in the repo root and return its stripped stdout+stderr.

    Raises subprocess.CalledProcessError on a nonzero exit.
    """
    return subprocess.check_output(args, cwd=ROOT, env=env, text=True, stderr=subprocess.STDOUT).strip()


def default_compose_files(environment):
    """Apply the same renderer overlay as scripts/njord for direct entrypoints."""
    paths = [ROOT / 'compose.yaml']
    if environment.get('NJORD_CPU') == '1':
        paths.append(ROOT / 'compose.cpu.yaml')
    elif 'microsoft' in Path('/proc/sys/kernel/osrelease').read_text().lower():
        paths.append(ROOT / 'compose.wsl.yaml')
    return ':'.join(map(str, paths))


def pin_image(environment):
    """Resolve once and pin every race service to immutable image contents.

    A tag such as njord-sim:local can be rebuilt or re-pulled while a long
    benchmark runs. This resolves the tag the race services use to its image
    ID and writes it into ``environment`` as NJORD_IMAGE and IMAGE_ID, so every
    race in the matrix runs the exact same image. Raises ValueError if the
    services disagree on the image or an overlay ignores NJORD_IMAGE.

    Returns image provenance for the manifest: the image ID, and the source
    commit and source digest labels stamped by ``scripts/njord build`` or CI
    ("unknown" when the image carries no io.njord.source.digest label).
    """
    def race_images():
        configuration = json.loads(command_output(['docker', 'compose', 'config', '--format', 'json'], environment))
        return {configuration['services'][service]['image'] for service in ('simulator', 'autonomy', 'evaluator')}

    images = race_images()
    if len(images) != 1:
        raise ValueError('Benchmark race services must use the same simulator image')
    inspection = json.loads(command_output(['docker', 'image', 'inspect', images.pop()], environment))[0]
    image_id = inspection['Id']
    labels = inspection.get('Config', {}).get('Labels') or {}
    sys.path.insert(0, str(ROOT / "scripts"))
    from build_metadata import verify_executable
    verify_executable(labels, ROOT)
    environment['NJORD_IMAGE'] = image_id
    environment['IMAGE_ID'] = image_id
    # Re-resolve with the pinned ID to prove every service now uses it.
    if race_images() != {image_id}:
        raise ValueError('Compose overrides must honor NJORD_IMAGE for all race services')
    return {'image_identity': image_id,
            'image_source_commit': labels.get('org.opencontainers.image.revision', 'unknown')
            if 'io.njord.source.digest' in labels else 'unknown',
            'image_source_digest': labels.get('io.njord.source.digest', 'unknown'),
            'image_executable_digest': labels['io.njord.executable.digest']}


def run_one(index, item, total, output, base_env, wall_timeout, domain_id, cancelled):
    """Run one race in its own Compose project and return its metrics dict.

    Args:
        index: position in the matrix; makes the project name unique.
        item: (environment, seed, profile) tuple.
        total: matrix size, for the progress line only.
        output: benchmark directory; the race writes to output/<label>/.
        base_env: environment shared by all races (pinned image, commit, ...).
        wall_timeout: steady wall-clock budget for the race in seconds.
        domain_id: ROS_DOMAIN_ID leased for this race.
        cancelled: threading.Event set on Ctrl-C to stop the race early.

    Race and infrastructure failures (timeouts, Compose errors) are recorded in
    the returned metrics ("status" and "infrastructure_failure") instead of
    raised, so a failed
    race is kept in the report instead of disappearing. The project is always
    torn down with ``docker compose down`` before returning.
    """
    environment, seed, profile = item
    label = f"{environment}-{seed}-{profile}"
    run_dir = output / label
    run_dir.mkdir()
    # One unique name isolates the Compose project (containers/networks), the
    # Gazebo transport partition and the run handoff (RUN_ID); the leased ROS
    # domain isolates DDS traffic.
    project = f"njord-bench-{os.getpid()}-{index}"
    env = base_env | {"OUTPUT_HOST": str(run_dir), "OUTPUT_DIR": "/outputs", "SEED": str(seed),
                      "ENVIRONMENT": environment, "PROFILE": profile, "RUN_LABEL": label,
                      "COMPOSE_PROJECT_NAME": project, "GZ_PARTITION": project, "RUN_ID": project,
                      "ROS_DOMAIN_ID": str(domain_id)}
    command = ["docker", "compose", "-p", project]
    # Infrastructure watchdog: steady wall time, independent of /clock.
    wall_start = time.monotonic()
    reason = None  # Set when infrastructure (not the autonomy) ends the race.
    print(f"[{index + 1}/{total}] {label}", flush=True)
    try:
        # Save the fully resolved Compose file so the race can be reproduced.
        configuration = json.loads(command_output(command + ['config', '--format', 'json'], env))
        (run_dir / 'compose.resolved.json').write_text(json.dumps(configuration, indent=2, allow_nan=False) + '\n')
        with (run_dir / "compose.log").open("w") as log:
            # The evaluator exits when the race ends; its exit code is the result.
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
    except subprocess.CalledProcessError as error:
        reason = f"compose_config_exit_{error.returncode}"
        (run_dir / 'compose_config_error.log').write_text(str(error.output or error))
    except ValueError as error:
        reason = f"invalid_compose_config: {error}"
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
        # parse_constant rejects NaN/Infinity so corrupt metrics count as missing.
        metrics = json.loads(metrics_path.read_text(), parse_constant=lambda s: (_ for _ in ()).throw(ValueError(s)))
        # An infrastructure failure always wins: a "completed" result from a
        # race that timed out or failed cleanup is not accepted as verified.
        if reason:
            metrics["infrastructure_failure"] = reason
            if metrics.get("status") == "completed":
                metrics["status"] = reason
    except (OSError, ValueError):
        # No usable evaluator output: record an explicitly unsafe placeholder.
        metrics = {"status": reason or "missing_metrics", "collision": None,
                   "contact_status": "unavailable", "geometric_overlap": None}
    metrics.setdefault("state_source", base_env.get("STATE_SOURCE", "estimate"))
    metrics.update({"label": label, "environment": environment, "seed": seed, "profile": profile,
                    "runner_git_commit": base_env.get("RUNNER_GIT_COMMIT", "unknown"),
                    "image_identity": base_env.get("IMAGE_ID", "unknown"),
                    "benchmark_wall_time_s": time.monotonic() - wall_start})
    return metrics


def main():
    """Parse options, lease ROS domains and run the matrix; return the exit code."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seeds", type=int, nargs="+", default=list(range(1, 11)))
    parser.add_argument("--environments", choices=["calm", "moderate"], nargs="+", default=["calm", "moderate"])
    parser.add_argument("--profiles", nargs="+",
                        help="speed_profiles_mps names from algorithms.yaml (default: all of them)")
    # Per-race steady wall-clock limit in seconds (the course has its own
    # simulation-time timeout inside the evaluator).
    parser.add_argument("--wall-timeout", type=float, default=600)
    parser.add_argument("--jobs", type=int, default=1, help="Concurrent races (1-100); each holds a distinct ROS domain")
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    known_profiles = speed_profiles(os.environ)
    if args.profiles is None:
        if known_profiles is None:
            parser.error("--profiles is required when ALGORITHMS_CONFIG is not under /config/")
        args.profiles = known_profiles
    elif known_profiles is not None and not set(args.profiles) <= set(known_profiles):
        parser.error(f"unknown profiles; algorithms.yaml defines {known_profiles}")
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
    # --dry-run only prints the plan; it starts nothing and leases nothing.
    if args.dry_run:
        print(json.dumps({"output": str(output), "runs": matrix, "jobs": args.jobs,
                          "command": ["docker", "compose", "up", "--abort-on-container-exit",
                                      "--exit-code-from", "evaluator", "simulator", "autonomy", "evaluator"]}, indent=2))
        return 0
    try:
        leases = DomainLeases(args.jobs).acquire()
    except DomainUnavailable as error:
        parser.exit(2, f"benchmark: {error}\n")
    try:
        return run_benchmark(args, matrix, output, stamp, leases.domains)
    finally:
        # Workers finish their Compose cleanup before run_benchmark returns.
        leases.close()


def run_benchmark(args, matrix, output, stamp, leased_domains):
    """Run every matrix entry on a pool of ``args.jobs`` workers.

    Each worker borrows one leased ROS domain from a queue for the duration of
    a race, so at most ``jobs`` races run and no two share a domain. Writes
    manifest.json first and summary.json after every finished race. Returns
    the process exit code described in the module docstring.
    """
    # exist_ok=False: never mix results into an earlier benchmark directory.
    output.mkdir(parents=True, exist_ok=False)
    base_env = os.environ.copy()
    base_env.setdefault("COMPOSE_FILE", default_compose_files(base_env))
    base_env["RUNNER_GIT_COMMIT"] = command_output(["git", "rev-parse", "HEAD"])
    image_metadata = pin_image(base_env)
    # Provenance: which code, image, matrix and domains produced these results.
    manifest = {"runner_git_commit": base_env["RUNNER_GIT_COMMIT"],
                "runner_git_dirty": bool(command_output(["git", "status", "--porcelain"])),
                **image_metadata, "matrix": matrix, "wall_timeout_s": args.wall_timeout, "jobs": args.jobs,
                "compose_file": base_env["COMPOSE_FILE"], "created_utc": stamp,
                "ros_domain_ids": list(leased_domains)}
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    runs = []
    cancelled = threading.Event()
    domains = queue.Queue()
    for domain_id in leased_domains:
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
        # Stable order regardless of completion order; atomic replace so a
        # reader never sees a half-written file.
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
        # Queued races are cancelled; running ones see `cancelled`, stop and
        # still tear down their project. Their metrics are collected below.
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
    # The fast-vs-conservative requirement only applies when both profiles ran.
    paired = set(args.profiles) == {"conservative", "fast"}
    improvements = all(c["fast_improves_median_without_failures"] for c in report["comparisons"].values())
    return 0 if report["all_runs_verified"] and (not paired or improvements) else 2


if __name__ == "__main__":
    sys.exit(main())
