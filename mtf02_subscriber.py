#!/usr/bin/env python3
"""
Zenoh subscriber for MTF-02 sensor data.
Reads zenoh_topic from config.cfg (default: fdcl/mtf).

Install: pip install eclipse-zenoh
"""

import json
import os
import sys
import time
import argparse
import threading
import zenoh


def load_config(path: str) -> dict:
    cfg = {"zenoh_topic": "fdcl/mtf", "sensor_hz": 50}
    try:
        with open(path) as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith("#"):
                    continue
                if "=" not in line:
                    continue
                key, _, val = line.partition("=")
                key, val = key.strip(), val.strip()
                if key == "zenoh_topic":
                    cfg["zenoh_topic"] = val
                elif key == "sensor_hz":
                    cfg["sensor_hz"] = int(val)
    except FileNotFoundError:
        pass
    return cfg


class Stats:
    def __init__(self):
        self._lock = threading.Lock()
        self.count = 0
        self.window_start = time.monotonic()
        self.hz = 0.0

    def tick(self):
        with self._lock:
            self.count += 1
            elapsed = time.monotonic() - self.window_start
            if elapsed >= 1.0:
                self.hz = self.count / elapsed
                self.count = 0
                self.window_start = time.monotonic()

    def rate(self) -> float:
        with self._lock:
            return self.hz


def make_callback(stats: Stats, verbose: bool):
    def on_sample(sample):
        stats.tick()
        try:
            raw = bytes(sample.payload.to_bytes())
            d = json.loads(raw)
        except Exception as e:
            print(f"[warn] failed to parse payload: {e}", file=sys.stderr)
            return

        if verbose:
            print(
                f"\033[2J\033[H"
                f"=== MTF-02 Subscriber  ({stats.rate():.1f} Hz) ===\n\n"
                f"  Rangefinder\n"
                f"    distance  = {d.get('distance_mm', '?')} mm"
                f"   [status={d.get('dis_status', '?')}  strength={d.get('strength', '?')}]\n\n"
                f"  Optical Flow\n"
                f"    vel x     = {d.get('flow_vel_x', '?')} cm/s@1m\n"
                f"    vel y     = {d.get('flow_vel_y', '?')} cm/s@1m\n"
                f"    quality   = {d.get('flow_quality', '?')}   [status={d.get('flow_status', '?')}]\n\n"
                f"  time_ms     = {d.get('time_ms', '?')} ms",
                end="", flush=True,
            )
        else:
            print(json.dumps(d))

    return on_sample


def main():
    parser = argparse.ArgumentParser(description="MTF-02 Zenoh subscriber")
    parser.add_argument(
        "--config", default=None,
        help="Path to config.cfg (default: searches ../config.cfg then config.cfg)"
    )
    parser.add_argument(
        "--topic", default=None,
        help="Override zenoh key expression (default: from config.cfg)"
    )
    parser.add_argument(
        "--raw", action="store_true",
        help="Print raw JSON lines instead of the formatted display"
    )
    args = parser.parse_args()

    # Locate config.cfg relative to this script
    script_dir = os.path.dirname(os.path.abspath(__file__))
    cfg_candidates = [
        args.config,
        os.path.join(script_dir, "config.cfg"),
        os.path.join(script_dir, "..", "config.cfg"),
        "config.cfg",
    ]
    cfg = {}
    for path in cfg_candidates:
        if path and os.path.exists(path):
            cfg = load_config(path)
            break

    topic = args.topic or cfg.get("zenoh_topic", "fdcl/mtf")

    print(f"Connecting to Zenoh, subscribing to '{topic}' ...")

    stats = Stats()
    callback = make_callback(stats, verbose=not args.raw)

    with zenoh.open(zenoh.Config()) as session:
        sub = session.declare_subscriber(topic, callback)  # noqa: F841
        print(f"Subscribed. Waiting for data on '{topic}'. Ctrl+C to quit.\n")
        try:
            while True:
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\nDone.")


if __name__ == "__main__":
    main()
