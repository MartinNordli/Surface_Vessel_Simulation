#!/usr/bin/env python3
"""Probe Njord applied-force telemetry on an isolated simulator-only partition.

No guard/controller/other force publisher may run during this check. Applied
force is simulation evidence, never an autonomy input. This checks actual plugin
response, invalid-input behavior and steady-wall expiry; it does not establish
vessel inertia, hydrodynamic fidelity, or calibrated propeller response.
"""

import argparse
from collections import deque
from datetime import datetime, timezone
import json
import hashlib
import math
import os
from pathlib import Path
import threading
import time
import yaml


class Probe:
    def __init__(self, args):
        from gz.transport13 import Node
        from gz.msgs10.twist_pb2 import Twist

        self.args, self.Twist = args, Twist
        self.node = Node()
        self.condition = threading.Condition()
        self.samples = deque(maxlen=30000)
        self.publisher = self.node.advertise("/njord/actuator_forces", Twist)
        if not self.node.subscribe(Twist, "/njord/actuator_applied", self.receive):
            raise RuntimeError("Cannot subscribe applied force telemetry")

    def receive(self, msg):
        sample = {
            "wall": time.monotonic(),
            "sim": msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9,
            "forces": [msg.linear.x, msg.linear.y],
            "targets": [msg.angular.x, msg.angular.y],
            "live": bool(msg.angular.z),
        }
        with self.condition:
            self.samples.append(sample)
            self.condition.notify_all()

    def send(self, values):
        msg = self.Twist()
        msg.linear.x, msg.linear.y = values
        sent = time.monotonic()
        if not self.publisher.publish(msg):
            raise RuntimeError("Command publication failed")
        return sent

    def wait(self, duration=0.01):
        with self.condition:
            self.condition.wait(duration)

    def snapshot(self):
        with self.condition:
            return list(self.samples)

    def connect(self):
        deadline = time.monotonic() + self.args.discovery_s
        while time.monotonic() < deadline:
            if self.samples and self.publisher.has_connections():
                return
            self.wait()
        raise RuntimeError("No Njord force telemetry / command subscriber in partition")

    def hold(self, values, duration):
        start = time.monotonic()
        deadline = start + max(10, duration * 10)
        initial_sim = self.snapshot()[-1]["sim"]
        last_send = 0
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now - last_send >= 0.04:
                last_send = self.send(values)
            if self.snapshot()[-1]["sim"] - initial_sim >= duration:
                return start, last_send
            self.wait()
        raise RuntimeError("Simulation time did not advance enough to measure response")

    def wait_invalid(self, since, limit):
        deadline = since + limit
        while time.monotonic() < deadline:
            for sample in self.snapshot():
                if (
                    sample["wall"] >= since
                    and not sample["live"]
                    and sample["targets"] == [0.0, 0.0]
                ):
                    return sample
            self.wait()
        raise AssertionError(
            "Both targets did not become invalid zero within wall deadline"
        )

    def response_errors(self, since, taus, target):
        samples = [
            s for s in self.snapshot() if s["wall"] >= since and s["targets"] == target
        ]
        errors, transient_pairs = [], 0
        for a, b in zip(samples, samples[1:]):
            dt = b["sim"] - a["sim"]
            if not 0 < dt < 0.2:
                continue
            for index, tau in enumerate(taus):
                expected = (
                    target[index]
                    if tau == 0
                    else target[index]
                    + (a["forces"][index] - target[index]) * math.exp(-dt / tau)
                )
                errors.append(abs(b["forces"][index] - expected))
                if abs(a["forces"][index] - target[index]) > 0.1:
                    transient_pairs += 1
        if not errors or (any(taus) and transient_pairs < 2):
            raise AssertionError(
                "Insufficient transient samples to verify configured response"
            )
        if max(errors) > 0.02:
            raise AssertionError(
                f"Applied-force response differs from configured model by {max(errors):.6g} N"
            )
        return {
            "sample_pairs": len(errors) // 2,
            "transient_pairs": transient_pairs,
            "max_error_n": max(errors),
        }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--partition", default=os.environ.get("GZ_PARTITION"))
    parser.add_argument(
        "--resolved",
        type=Path,
        required=True,
        help="Same resolved_configuration.json as running model (JSON or YAML accepted)",
    )
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--discovery-s", type=float, default=15.0)
    parser.add_argument("--timeout-s", type=float, default=0.5)
    parser.add_argument("--slack-s", type=float, default=0.35)
    args = parser.parse_args()
    result = {
        "schema_version": 1,
        "status": "failed",
        "checks": {},
        "failures": [],
        "started_utc": datetime.now(timezone.utc).isoformat(),
        "partition": args.partition,
        "resolved_path": str(args.resolved),
        "scope": "applied actuator forces only",
    }
    probe = None
    try:
        if not args.partition:
            raise ValueError("Specify isolated simulator --partition")
        os.environ["GZ_PARTITION"] = args.partition
        resolved = yaml.safe_load(args.resolved.read_text())
        vessel = resolved["vessel"]
        if vessel["profile"] != "njord":
            raise ValueError("Probe requires Njord profile")
        result["config_sha256"] = resolved.get("config_sha256")
        result["resolved_sha256"] = hashlib.sha256(
            args.resolved.read_bytes()
        ).hexdigest()
        manifest = args.resolved.parent / "run_manifest.json"
        if manifest.is_file():
            result["manifest_sha256"] = hashlib.sha256(manifest.read_bytes()).hexdigest()
        taus = [t["response_time_s"] for t in vessel["thrusters"]]
        targets = [
            min(80.0 + i * 20.0, t["forward_limit_n"] * 0.25)
            for i, t in enumerate(vessel["thrusters"])
        ]
        result["response_time_s"], result["requested_force_n"] = taus, targets
        probe = Probe(args)
        probe.connect()
        probe.hold([0.0, 0.0], max(0.3, 8 * max(taus)))
        start, _ = probe.hold(targets, max(0.4, 6 * max(taus)))
        result["checks"]["configured_response"] = probe.response_errors(
            start, taus, targets
        )
        sent = probe.send([math.nan, targets[1]])
        invalid = probe.wait_invalid(sent, min(0.25, args.timeout_s * 0.5))
        result["checks"]["nonfinite_invalidates_both_targets"] = {
            "wall_delay_s": invalid["wall"] - sent
        }
        decay_start = invalid["wall"]
        probe.hold([math.nan, targets[1]], max(0.4, 6 * max(taus)))
        result["checks"]["invalid_command_residual_decay"] = probe.response_errors(
            decay_start, taus, [0.0, 0.0]
        )
        _, stopped = probe.hold(targets, max(0.4, 6 * max(taus)))
        expired = probe.wait_invalid(stopped, args.timeout_s + args.slack_s)
        delay = expired["wall"] - stopped
        if delay < args.timeout_s - 0.08:
            raise AssertionError(
                "Premature expiry: competing command source or configured timeout mismatch"
            )
        result["checks"]["steady_wall_timeout"] = {
            "observed_s": delay,
            "configured_s": args.timeout_s,
        }
        samples = probe.snapshot()
        if any(
            not math.isfinite(f)
            for sample in samples
            for f in sample["forces"] + sample["targets"]
        ):
            raise AssertionError("Nonfinite applied or target force telemetry")
        result["checks"]["all_observed_forces_finite"] = True
        result["sample_count"] = len(samples)
        result["status"] = "passed"
    except (Exception, KeyboardInterrupt) as error:
        result["failures"].append(f"{type(error).__name__}: {error}")
    finally:
        if probe:
            for _ in range(3):
                try:
                    probe.send([0.0, 0.0])
                except Exception:
                    break
                time.sleep(0.02)
    result["finished_utc"] = datetime.now(timezone.utc).isoformat()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    serialized = json.dumps(result, indent=2, allow_nan=False)
    args.output.write_text(serialized + "\n")
    print(serialized)
    return 0 if result["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
