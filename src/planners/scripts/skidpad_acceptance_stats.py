#!/usr/bin/env python3
"""Compute the skidpad acceptance metrics from a recorded run directory.

The recorder (test harness, not part of the package) writes:
  tf.csv          t_wall, stamp, x, y, yaw_deg        (track -> base_link)
  paths.csv       t_wall, stamp, n_poses              (PathWithSpeeds receipts)
  states.csv      t_wall, json                        (planning/skidpad_state)
  velocity.csv    t_wall, v                           (/vehicle/velocity)
  gt_cones.csv    x, y, color                         (/ground_truth/global_map)
  full_path.csv / yolo_full_path.csv   x, y           (latched full path)

Usage:
  python3 src/planners/scripts/skidpad_acceptance_stats.py /tmp/skid_gt_ff0
"""

import argparse
import csv
import json
import math
import os

import numpy as np

from planners.skidpad_fitting import fit_skidpad, select_boundary_cones


def read_csv(path):
    if not os.path.exists(path):
        return []
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def read_states(path):
    out = []
    for row in read_csv(path):
        try:
            out.append((float(row["t_wall"]), json.loads(row["json"])))
        except (KeyError, TypeError, ValueError):
            continue
    return out


def nearest_and_mask(tf, points, centres, radius):
    d = np.linalg.norm(tf[:, None, :] - points[None, :, :], axis=2)
    nn = d.min(axis=1)
    idx = d.argmin(axis=1)
    mask = np.zeros(len(tf), dtype=bool)
    for c in centres:
        rc = np.linalg.norm(points[idx] - c, axis=1)
        mask |= np.abs(rc - radius) < 0.15
    return nn, mask


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run_dir")
    ap.add_argument("--cone-hit-threshold", type=float, default=0.8)
    args = ap.parse_args()
    d = args.run_dir

    states = read_states(os.path.join(d, "states.csv"))
    if not states:
        raise SystemExit("no states.csv")
    seg_seq, lap_seq = [], []
    for _, s in states:
        if not seg_seq or seg_seq[-1] != s.get("segment"):
            seg_seq.append(s.get("segment"))
        if not lap_seq or lap_seq[-1] != s.get("lap"):
            lap_seq.append(s.get("lap"))
    final = states[-1][1]

    path_rows = read_csv(os.path.join(d, "paths.csv"))
    t_path = np.array([float(r["t_wall"]) for r in path_rows])
    gaps = np.diff(t_path) if len(t_path) > 1 else np.array([])

    full_name = "yolo_full_path.csv"
    if not os.path.exists(os.path.join(d, full_name)):
        full_name = "full_path.csv"
    points = np.array(
        [[float(r["x"]), float(r["y"])] for r in read_csv(os.path.join(d, full_name))]
    )
    tf = np.array(
        [[float(r["x"]), float(r["y"])] for r in read_csv(os.path.join(d, "tf.csv"))]
    )
    velocity = np.array(
        [float(r["v"]) for r in read_csv(os.path.join(d, "velocity.csv"))]
    )

    # Estimate the true track circles from the GT cone map for a run-independent
    # circle mask; the entry heading is the first path chord.
    cones = read_csv(os.path.join(d, "gt_cones.csv"))
    centres, radius = [], float("nan")
    if cones and len(points) > 1:
        positions = np.array([[float(r["x"]), float(r["y"])] for r in cones])
        colors = np.array([int(r["color"]) for r in cones])
        boundary = select_boundary_cones(positions, colors)
        e = points[1] - points[0]
        e = e / np.linalg.norm(e)
        gt_fit, _ = fit_skidpad(boundary, e)
        if gt_fit is not None:
            centres = [gt_fit.c_left, gt_fit.c_right]
            radius = gt_fit.R

    if len(tf) and len(points):
        nn, circle_mask = nearest_and_mask(tf, points, centres, radius)
        all_rms = float(np.sqrt(np.mean(nn**2)))
        all_max = float(nn.max())
        if circle_mask.any():
            circle_rms = float(np.sqrt(np.mean(nn[circle_mask] ** 2)))
            circle_max = float(nn[circle_mask].max())
        else:
            circle_rms = circle_max = float("nan")
    else:
        all_rms = all_max = circle_rms = circle_max = float("nan")

    min_cone = float("nan")
    hit_count = 0
    if cones and len(tf):
        cone_xy = np.array([[float(r["x"]), float(r["y"])] for r in cones])
        cd = np.linalg.norm(tf[:, None, :] - cone_xy[None, :, :], axis=2).min(axis=1)
        min_cone = float(cd.min())
        hit_count = int((cd < args.cone_hit_threshold).sum())

    print(f"run_dir: {d}")
    print(f"segments: {seg_seq}")
    print(f"laps: {lap_seq}")
    print(f"final: state={final.get('state')} segment={final.get('segment')} "
          f"s_prog={final.get('s_prog')} total={final.get('total_length')}")
    print(f"fit: {dict((k, final.get(k)) for k in ('fit_rms','fit_p90','fit_R','fit_ring_width','fit_centre_dist','fit_removed','boundary_cones'))}")
    print(f"path messages={len(t_path)} max_gap={gaps.max() if len(gaps) else float('nan'):.3f} "
          f"mean_gap={gaps.mean() if len(gaps) else float('nan'):.3f}")
    print(f"lateral all: rms={all_rms:.3f} max={all_max:.3f} | "
          f"circle: rms={circle_rms:.3f} max={circle_max:.3f}")
    print(f"cones: n={len(cones)} min_centre_cone={min_cone:.3f} "
          f"hits<{args.cone_hit_threshold}={hit_count}")
    print(f"velocity: final={velocity[-1] if len(velocity) else float('nan'):.3f} "
          f"max={velocity.max() if len(velocity) else float('nan'):.3f}")


if __name__ == "__main__":
    main()
