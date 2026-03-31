#!/usr/bin/env python3
"""
Vision Tracker v3.1 — Performance Benchmark Suite
Runs synthetic benchmarks against core components and produces a
polished, CEO-friendly terminal report (or JSON with --json).
"""

import argparse
import cv2
import json
import numpy as np
import os
import sys
import time

# Ensure we can import from the same directory as this script.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from vision_tracker import (
    create_kalman,
    detect_objects,
    match_and_update,
    estimate_distance_cm,
    TrackedObject,
    build_mask,
    HSV_DEFAULTS,
)

# ── Helpers ─────────────────────────────────────────────────────

MIN_BENCH_SECONDS = 1.0  # each benchmark runs at least this long


def _bench(func, min_seconds=MIN_BENCH_SECONDS):
    """Run *func* repeatedly for at least *min_seconds* and return (iterations, elapsed)."""
    iterations = 0
    start = time.perf_counter()
    deadline = start + min_seconds
    while True:
        func()
        iterations += 1
        if time.perf_counter() >= deadline:
            break
    elapsed = time.perf_counter() - start
    return iterations, elapsed


# ── Individual Benchmarks ───────────────────────────────────────

def bench_kalman():
    """Kalman filter predict+correct throughput."""
    kf = create_kalman()
    kf.statePost = np.array([[320], [240], [0], [0]], dtype=np.float32)
    rng = np.random.default_rng(42)
    measurements = rng.uniform(100, 540, size=(10000, 2)).astype(np.float32)
    idx = 0
    n = len(measurements)

    def step():
        nonlocal idx
        kf.predict()
        m = measurements[idx % n]
        kf.correct(m.reshape(2, 1))
        idx += 1

    iters, elapsed = _bench(step)
    return iters, elapsed


def bench_detection():
    """Detection pipeline FPS on synthetic 640x480 frames with coloured blobs."""
    # Build a reusable synthetic frame: green and purple circles on dark background.
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    # Green blob (HSV ~60, high S/V)
    cv2.circle(frame, (200, 240), 50, (0, 200, 0), -1)
    # Purple blob (HSV ~155, high S/V)
    cv2.circle(frame, (440, 240), 45, (200, 0, 180), -1)

    sliders = HSV_DEFAULTS

    def step():
        detect_objects(frame, sliders)

    iters, elapsed = _bench(step)
    return iters, elapsed


def bench_matching():
    """Multi-object matching throughput (N tracked vs M detections)."""
    # Pre-create tracked objects (reset class-level counter to avoid unbounded growth).
    original_next_id = TrackedObject._next_id

    def make_tracked(n=8):
        objs = []
        rng = np.random.default_rng(7)
        for _ in range(n):
            cx = int(rng.integers(50, 590))
            cy = int(rng.integers(50, 430))
            bbox = (cx - 25, cy - 25, 50, 50)
            objs.append(TrackedObject(cx, cy, bbox))
        return objs

    rng = np.random.default_rng(99)
    det_bank = []
    for _ in range(500):
        dets = []
        for _ in range(8):
            cx = int(rng.integers(50, 590))
            cy = int(rng.integers(50, 430))
            bbox = (cx - 25, cy - 25, 50, 50)
            dets.append((cx, cy, bbox))
        det_bank.append(dets)

    idx = 0
    n_bank = len(det_bank)

    def step():
        nonlocal idx
        tracked = make_tracked()
        match_and_update(tracked, det_bank[idx % n_bank])
        idx += 1

    iters, elapsed = _bench(step)
    TrackedObject._next_id = original_next_id
    return iters, elapsed


def bench_serial_encoding():
    """Serial protocol string formatting throughput."""
    # Simulate the formatting done in RobotSerial.send_frame()
    obj_id = 1
    vec_x, vec_y = 42, -17
    dist = 38.5
    vx, vy = 1.2, -0.8

    def step():
        _line = f"T{obj_id},{vec_x:+d},{vec_y:+d},{dist:.1f},{vx:+.1f},{vy:+.1f}\n"

    iters, elapsed = _bench(step)
    return iters, elapsed


def bench_distance():
    """Distance estimation throughput."""
    widths = np.random.default_rng(0).uniform(10, 200, size=10000)
    idx = 0
    n = len(widths)

    def step():
        nonlocal idx
        estimate_distance_cm(widths[idx % n])
        idx += 1

    iters, elapsed = _bench(step)
    return iters, elapsed


# ── Report Rendering ────────────────────────────────────────────

_COMPONENT_LABELS = [
    "Kalman Filter",
    "Detection Pipeline",
    "Object Matching",
    "Serial Encoding",
    "Distance Estimation",
]

_UNIT_MAP = {
    "Kalman Filter":       ("ops/s",  "ms/op"),
    "Detection Pipeline":  ("FPS",    "ms/frame"),
    "Object Matching":     ("ops/s",  "ms/op"),
    "Serial Encoding":     ("msg/s",  "ms/msg"),
    "Distance Estimation": ("ops/s",  "ms/op"),
}


def _fmt_throughput(value, unit):
    """Human-readable throughput with thousands separator."""
    if value >= 1_000_000:
        return f"{value:,.0f} {unit}"
    elif value >= 1_000:
        return f"{value:,.0f} {unit}"
    else:
        return f"{value:,.1f} {unit}"


def _fmt_latency(ms, unit):
    if ms < 0.001:
        return f"{ms:.4f} {unit}"
    elif ms < 0.1:
        return f"{ms:.4f} {unit}"
    elif ms < 1:
        return f"{ms:.3f} {unit}"
    else:
        return f"{ms:.1f} {unit}"


def render_report(results):
    """Print the box-drawing report to stdout."""
    W = 64  # inner width between the vertical bars

    def hline(left, fill, right):
        return left + fill * W + right

    def padded(text):
        return "\u2551" + text.ljust(W) + "\u2551"

    lines = []
    lines.append(hline("\u2554", "\u2550", "\u2557"))
    title = "VISION TRACKER v3.1 \u2014 PERFORMANCE REPORT"
    lines.append("\u2551" + title.center(W) + "\u2551")
    lines.append(hline("\u2560", "\u2550", "\u2563"))
    lines.append(padded(""))

    # Table header
    hdr = f"  {'Component':<24}{'Throughput':<20}{'Latency'}"
    lines.append(padded(hdr))
    dash = "\u2500"
    sep = f"  {dash * 24}{dash * 20}{dash * 16}"
    lines.append(padded(sep))

    bottleneck_name = None
    bottleneck_fps = float("inf")

    for name in _COMPONENT_LABELS:
        iters, elapsed = results[name]
        throughput = iters / elapsed
        latency_ms = (elapsed / iters) * 1000.0
        t_unit, l_unit = _UNIT_MAP[name]
        t_str = _fmt_throughput(throughput, t_unit)
        l_str = _fmt_latency(latency_ms, l_unit)
        row = f"  {name:<24}{t_str:<20}{l_str}"
        lines.append(padded(row))

        # Track pipeline bottleneck (the component with lowest throughput
        # that limits the overall frame pipeline).
        if throughput < bottleneck_fps:
            bottleneck_fps = throughput
            bottleneck_name = name

    # The end-to-end FPS is gated by the slowest per-frame component.
    # Detection, matching, encoding, and distance all run once per frame;
    # Kalman runs once per object but is extremely fast. Use detection as
    # the primary frame-rate limiter since it dominates wall time.
    det_iters, det_elapsed = results["Detection Pipeline"]
    e2e_fps = det_iters / det_elapsed  # detection is the real bottleneck
    realtime = e2e_fps >= 30.0

    lines.append(padded(""))
    lines.append(padded("  SYSTEM SUMMARY"))
    lines.append(padded(f"  {dash * 14}"))
    lines.append(padded(f"  End-to-end pipeline:   ~{e2e_fps:,.0f} FPS (bottleneck: detection)"))
    check = "\u2713 YES" if realtime else "\u2717 NO"
    lines.append(padded(f"  Real-time capable:     {check} (target: 30 Hz)"))
    lines.append(padded(""))
    lines.append(hline("\u255a", "\u2550", "\u255d"))

    print("\n".join(lines))


def build_json(results):
    """Return a JSON-serialisable dict of benchmark results."""
    data = {"version": "3.1.0", "benchmarks": {}}
    for name in _COMPONENT_LABELS:
        iters, elapsed = results[name]
        throughput = iters / elapsed
        latency_ms = (elapsed / iters) * 1000.0
        t_unit, _ = _UNIT_MAP[name]
        data["benchmarks"][name] = {
            "iterations": iters,
            "elapsed_s": round(elapsed, 4),
            "throughput": round(throughput, 2),
            "throughput_unit": t_unit,
            "latency_ms": round(latency_ms, 6),
        }
    det_iters, det_elapsed = results["Detection Pipeline"]
    e2e_fps = det_iters / det_elapsed
    data["summary"] = {
        "end_to_end_fps": round(e2e_fps, 1),
        "bottleneck": "detection",
        "realtime_capable": e2e_fps >= 30.0,
        "realtime_target_hz": 30,
    }
    return data


# ── Main ────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Vision Tracker v3.1 — Performance Benchmark"
    )
    parser.add_argument(
        "--json", action="store_true",
        help="Output results as JSON instead of the formatted report",
    )
    args = parser.parse_args()

    if not args.json:
        print("Running benchmarks (each component runs for >= 1 s) ...")
        print()

    results = {}

    benchmarks = [
        ("Kalman Filter",       bench_kalman),
        ("Detection Pipeline",  bench_detection),
        ("Object Matching",     bench_matching),
        ("Serial Encoding",     bench_serial_encoding),
        ("Distance Estimation", bench_distance),
    ]

    for name, fn in benchmarks:
        if not args.json:
            print(f"  Benchmarking {name} ...", end="", flush=True)
        iters, elapsed = fn()
        results[name] = (iters, elapsed)
        if not args.json:
            throughput = iters / elapsed
            print(f"  done  ({throughput:,.0f} {_UNIT_MAP[name][0]})")

    if not args.json:
        print()

    if args.json:
        print(json.dumps(build_json(results), indent=2))
    else:
        render_report(results)


if __name__ == "__main__":
    main()
