#!/usr/bin/env python3
"""
render_red_waypoint.py

Offline post-processing script.  For each episode, reads traj_body.csv and
the original images, selects an active waypoint per frame, projects it to
pixel coordinates, and saves annotated images to images_red_wp/.

Also writes a waypoint_meta.csv with per-frame projection metadata.

Usage:
    conda activate dbc241
    cd /home/l/uav_kd_project

    # Single episode
    python kd_uav/datasets/render_red_waypoint.py --episodes ep_test_003

    # Multiple episodes
    python kd_uav/datasets/render_red_waypoint.py \
        --episodes ep_test_003 ep_test_004 ep_test_005 ep_test_007

    # Custom lookahead
    python kd_uav/datasets/render_red_waypoint.py \
        --episodes ep_test_003 --lookahead 1.5

    # Inspection: copy first N annotated frames to a single folder
    python kd_uav/datasets/render_red_waypoint.py \
        --episodes ep_test_003 --inspect 20 --inspect_dir /tmp/red_wp_inspect
"""

import argparse
import csv
import os
import shutil
import sys
from pathlib import Path

import cv2
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT))

from kd_uav.utils.waypoint_projection import (
    select_active_waypoint,
    project_body_to_pixel,
    draw_red_waypoint,
)

RAW_DIR = PROJECT_ROOT / 'data' / 'raw'

TRAJ_BODY_COLS = ['frame_id', 'timestamp'] + [
    f'd{ax}{i}' for i in range(10) for ax in ['x', 'y', 'z']
]


def process_episode(ep_dir: Path, lookahead: float, K: int,
                    inspect: int, inspect_out: Path):
    """Process one episode. Returns per-frame metadata list."""

    img_dir     = ep_dir / 'images'
    out_dir     = ep_dir / 'images_red_wp'
    traj_csv    = ep_dir / 'traj_body.csv'
    meta_csv    = ep_dir / 'waypoint_meta.csv'

    if not img_dir.exists():
        print(f"  [SKIP] {ep_dir.name}: images/ not found")
        return []
    if not traj_csv.exists():
        print(f"  [SKIP] {ep_dir.name}: traj_body.csv not found")
        return []

    out_dir.mkdir(exist_ok=True)

    # Load traj_body
    traj_rows = {}
    with open(traj_csv) as f:
        reader = csv.DictReader(f)
        for row in reader:
            fid = int(row['frame_id'])
            vals = np.array([float(row[f'd{ax}{i}'])
                             for i in range(K) for ax in ['x', 'y', 'z']],
                            dtype=np.float32)
            traj_rows[fid] = vals

    meta_rows = []
    n_visible = 0
    n_total   = 0
    inspect_count = 0

    img_files = sorted(img_dir.glob('*.png'))
    for img_path in img_files:
        fid = int(img_path.stem)
        if fid not in traj_rows:
            continue

        img = cv2.imread(str(img_path))
        if img is None:
            print(f"  [WARN] Could not read {img_path}")
            continue

        n_total += 1

        # Select active waypoint
        row_vals = traj_rows[fid]
        dx, dy, dz, wp_idx = select_active_waypoint(row_vals, lookahead, K)
        dist_b = float(np.sqrt(dx**2 + dy**2 + dz**2))

        # Project to pixel
        u, v, visible = project_body_to_pixel(dx, dy, dz)

        # Annotate
        if visible:
            annotated = draw_red_waypoint(img, u, v, dist_b)
            n_visible += 1
        else:
            annotated = img.copy()

        # Save
        out_path = out_dir / img_path.name
        cv2.imwrite(str(out_path), annotated)

        meta_rows.append({
            'frame_id':  fid,
            'wp_index':  wp_idx,
            'wp_dx_b':   round(dx, 6),
            'wp_dy_b':   round(dy, 6),
            'wp_dz_b':   round(dz, 6),
            'wp_dist_b': round(dist_b, 6),
            'wp_u':      round(u, 2),
            'wp_v':      round(v, 2),
            'wp_visible': int(visible),
        })

        # Collect inspection images (from spread across episode)
        if inspect > 0 and inspect_out is not None:
            step = max(1, n_total // inspect) if n_total > 0 else 1
            if inspect_count < inspect and (n_total % step == 0 or inspect_count == 0):
                dst = inspect_out / f"{ep_dir.name}_{img_path.name}"
                shutil.copy(str(out_path), str(dst))
                inspect_count += 1

    # Write metadata CSV
    if meta_rows:
        fieldnames = list(meta_rows[0].keys())
        with open(meta_csv, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(meta_rows)

    vis_pct = 100.0 * n_visible / n_total if n_total > 0 else 0
    print(f"  {ep_dir.name}: {n_total} frames, "
          f"wp_visible={n_visible} ({vis_pct:.1f}%), "
          f"lookahead={lookahead}m → "
          f"saved to images_red_wp/")
    return meta_rows


def main():
    parser = argparse.ArgumentParser(description='Render red waypoint onto UAV images')
    parser.add_argument('--episodes',    nargs='+', required=True)
    parser.add_argument('--raw_dir',     default=str(RAW_DIR))
    parser.add_argument('--lookahead',   type=float, default=1.0,
                        help='Min dx_b (m) for active waypoint selection')
    parser.add_argument('--K',           type=int,   default=10)
    parser.add_argument('--inspect',     type=int,   default=0,
                        help='If > 0, copy this many annotated frames to inspect_dir')
    parser.add_argument('--inspect_dir', default='/tmp/red_wp_inspect')
    args = parser.parse_args()

    raw_dir = Path(args.raw_dir)

    inspect_out = None
    if args.inspect > 0:
        inspect_out = Path(args.inspect_dir)
        inspect_out.mkdir(parents=True, exist_ok=True)
        print(f"Inspection images → {inspect_out}")

    print(f"lookahead_dist = {args.lookahead} m")
    print(f"Episodes       : {args.episodes}")
    print()

    for ep in args.episodes:
        ep_dir = raw_dir / ep
        if not ep_dir.exists():
            print(f"  [SKIP] {ep}: directory not found")
            continue
        process_episode(ep_dir, args.lookahead, args.K,
                        args.inspect, inspect_out)

    if args.inspect > 0:
        print(f"\nInspection images saved to: {inspect_out}")
        print("Open them with any image viewer to verify red dot position.")


if __name__ == '__main__':
    main()
