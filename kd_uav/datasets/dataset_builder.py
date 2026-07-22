#!/usr/bin/env python3
"""
dataset_builder.py

Converts raw teacher demonstration episodes into a processed .npz dataset
ready for PyTorch training.

Usage:
  conda activate dbc_bc
  cd /home/l/uav_kd_project

  # Debug dataset (single episode, all to train):
  python kd_uav/datasets/dataset_builder.py \
      --raw_dir data/raw \
      --output_dir data/processed/uav_kd_debug_v000 \
      --episodes ep_test_003 \
      --K 10 \
      --dataset_type debug

  # Future: formal dataset (multiple episodes, episode-level split):
  python kd_uav/datasets/dataset_builder.py \
      --raw_dir data/raw \
      --output_dir data/processed/uav_kd_v001 \
      --episodes ep_001 ep_002 ep_003 ep_004 ep_005 \
      --K 10 \
      --dataset_type train \
      --val_episodes ep_004 \
      --test_episodes ep_005

Per-sample .npz keys:
  image           (H, W, 3)  uint8  BGR
  ctrl_body       (4,)       float32  [vx_b, vy_b, vz_b, yaw_rate]
  ctrl_world      (4,)       float32  [vx_w, vy_w, vz_w, yaw_rate]
  traj_body       (K, 4)     float32  [dx, dy, dz, dyaw]  dyaw=0 if has_dyaw=False
  traj_world      (K, 4)     float32  [x,  y,  z,  yaw]   yaw=0 if has_dyaw=False
  traj_body_xyz   (K, 3)     float32  [dx, dy, dz]
  traj_world_xyz  (K, 3)     float32  [x,  y,  z]
  pose            (4,)       float32  [x, y, z, yaw]
  timestamp       scalar     float64
  frame_id        scalar     int32
  valid_motion    scalar     bool
  has_dyaw        scalar     bool
  episode_id      0-d array  str
"""

import argparse
import csv
import json
import os
import sys
from pathlib import Path

# Red-waypoint mode extra npz keys:
#   wp_u       float32  pixel u coordinate
#   wp_v       float32  pixel v coordinate
#   wp_dist_b  float32  distance to waypoint in body frame (m)
#   wp_index   int32    index of selected waypoint in traj_body
#   wp_visible bool     True (always True when use_red_wp, since non-visible frames are filtered)

import cv2
import numpy as np
import yaml

# ── Constants ─────────────────────────────────────────────────────────────────

DEFAULT_SPEED_THRESHOLD = 0.05   # m/s: below this → valid_motion = False
DEFAULT_YAW_THRESHOLD   = 0.3    # rad/s: above this → valid_motion = True even if speed < threshold
DEFAULT_TRAJ_MAGNITUDE_THRESHOLD = 0.02  # sum(|dx_i|+|dy_i|) over K waypoints: below this → traj considered stale/degenerate


# ── CSV helpers ───────────────────────────────────────────────────────────────

def read_csv_by_frame(path):
    """
    Read a CSV file into a dict keyed by frame_id (int).
    Each value is a dict of column_name → string_value.
    Raises FileNotFoundError if the file does not exist.
    """
    path = Path(path)
    if not path.exists():
        raise FileNotFoundError(f"Required CSV not found: {path}")
    rows = {}
    with open(path, newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            fid = int(row['frame_id'])
            rows[fid] = row
    return rows


def infer_K_from_traj_csv(path):
    """
    Infer K from a trajectory CSV header.
    Header format: frame_id, timestamp, dx0, dy0, dz0, ..., dx{K-1}, dy{K-1}, dz{K-1}
    """
    with open(path, newline='') as f:
        header = next(csv.reader(f))
    n_data_cols = len(header) - 2   # subtract frame_id and timestamp
    assert n_data_cols % 3 == 0, f"Unexpected traj CSV column count: {len(header)}"
    return n_data_cols // 3


# ── Episode processor ─────────────────────────────────────────────────────────

def load_waypoint_meta(ep_dir):
    """Load waypoint_meta.csv into a dict keyed by frame_id."""
    path = Path(ep_dir) / 'waypoint_meta.csv'
    if not path.exists():
        raise FileNotFoundError(
            f"waypoint_meta.csv not found in {ep_dir}. "
            "Run render_red_waypoint.py first."
        )
    rows = {}
    with open(path, newline='') as f:
        for row in csv.DictReader(f):
            fid = int(row['frame_id'])
            rows[fid] = {
                'wp_u':       float(row['wp_u']),
                'wp_v':       float(row['wp_v']),
                'wp_dist_b':  float(row['wp_dist_b']),
                'wp_index':   int(row['wp_index']),
                'wp_visible': int(row['wp_visible']) == 1,
            }
    return rows


def process_episode(ep_dir, ep_id, K, speed_threshold, yaw_threshold=DEFAULT_YAW_THRESHOLD,
                     traj_magnitude_threshold=DEFAULT_TRAJ_MAGNITUDE_THRESHOLD, use_red_wp=False):
    """
    Read one episode directory and return a list of sample dicts.

    Returns:
        samples (list[dict])  — one dict per valid frame
        metadata (dict)       — contents of metadata.yaml
        report (dict)         — summary counts for this episode
    """
    ep_dir = Path(ep_dir)

    # ── Check required files ─────────────────────────────────────────────────
    required_files = [
        'metadata.yaml',
        'ctrl_body.csv', 'ctrl_world.csv',
        'traj_body.csv', 'traj_world.csv',
        'pose.csv',
    ]
    if use_red_wp:
        required_files.append('waypoint_meta.csv')
    missing = [f for f in required_files if not (ep_dir / f).exists()]
    if missing:
        raise FileNotFoundError(
            f"[ERROR] Episode '{ep_id}' is missing required files:\n"
            + "\n".join(f"  {m}" for m in missing)
        )

    # ── metadata.yaml ────────────────────────────────────────────────────────
    with open(ep_dir / 'metadata.yaml') as f:
        metadata = yaml.safe_load(f)

    # ── Load waypoint meta (red-wp mode only) ────────────────────────────────
    wp_meta = load_waypoint_meta(ep_dir) if use_red_wp else {}

    # ── Choose image directory ────────────────────────────────────────────────
    img_subdir = 'images_red_wp' if use_red_wp else 'images'

    # ── Load CSVs ────────────────────────────────────────────────────────────
    ctrl_body_csv  = read_csv_by_frame(ep_dir / 'ctrl_body.csv')
    ctrl_world_csv = read_csv_by_frame(ep_dir / 'ctrl_world.csv')
    traj_body_csv  = read_csv_by_frame(ep_dir / 'traj_body.csv')
    traj_world_csv = read_csv_by_frame(ep_dir / 'traj_world.csv')
    pose_csv       = read_csv_by_frame(ep_dir / 'pose.csv')

    # ── Validate K ───────────────────────────────────────────────────────────
    csv_K = infer_K_from_traj_csv(ep_dir / 'traj_body.csv')
    if csv_K != K:
        print(f"  [WARN] {ep_id}: traj_body.csv has K={csv_K}, but requested K={K}. "
              f"Using K={csv_K} for this episode.")
        K = csv_K

    # ── Frame alignment ──────────────────────────────────────────────────────
    frame_ids = sorted(
        set(ctrl_body_csv)
        & set(ctrl_world_csv)
        & set(traj_body_csv)
        & set(traj_world_csv)
        & set(pose_csv)
    )

    raw_frames = metadata.get('total_frames', len(frame_ids))
    missing_images = []
    samples = []

    for fid in frame_ids:
        # ── Red-wp filter: skip frames where waypoint is not visible ──────────
        if use_red_wp:
            wp = wp_meta.get(fid)
            if wp is None or not wp['wp_visible']:
                continue

        img_path = ep_dir / img_subdir / f'{fid:06d}.png'
        if not img_path.exists():
            missing_images.append(fid)
            continue

        # ── ctrl_body ──────────────────────────────────────────────────────
        cb = ctrl_body_csv[fid]
        ctrl_body = np.array([
            float(cb['vx_b']), float(cb['vy_b']),
            float(cb['vz_b']), float(cb['yaw_rate']),
        ], dtype=np.float32)

        # ── ctrl_world ─────────────────────────────────────────────────────
        cw = ctrl_world_csv[fid]
        ctrl_world = np.array([
            float(cw['vx_w']), float(cw['vy_w']),
            float(cw['vz_w']), float(cw['yaw_rate']),
        ], dtype=np.float32)

        # ── pose ───────────────────────────────────────────────────────────
        p = pose_csv[fid]
        pose = np.array([
            float(p['x']), float(p['y']),
            float(p['z']), float(p['yaw']),
        ], dtype=np.float32)

        # ── traj_body_xyz (K, 3) ───────────────────────────────────────────
        tb = traj_body_csv[fid]
        traj_body_xyz = np.array(
            [[float(tb[f'dx{i}']), float(tb[f'dy{i}']), float(tb[f'dz{i}'])]
             for i in range(K)],
            dtype=np.float32,
        )

        # ── traj_world_xyz (K, 3) ──────────────────────────────────────────
        tw = traj_world_csv[fid]
        traj_world_xyz = np.array(
            [[float(tw[f'x{i}']), float(tw[f'y{i}']), float(tw[f'z{i}'])]
             for i in range(K)],
            dtype=np.float32,
        )

        # ── traj_body / traj_world (K, 4) — dyaw=0 ─────────────────────────
        # bspline_trajectory PoseStamped orientation is not stored in the CSV.
        # dyaw will be filled with 0; has_dyaw = False.
        # To add dyaw: reprocess from rosbag using the waypoint quaternions.
        traj_body  = np.zeros((K, 4), dtype=np.float32)
        traj_world = np.zeros((K, 4), dtype=np.float32)
        traj_body[:, :3]  = traj_body_xyz
        traj_world[:, :3] = traj_world_xyz

        # ── Motion filter ──────────────────────────────────────────────────
        # A frame is valid if the UAV is moving forward (speed >= threshold)
        # OR actively turning in place (|yaw_rate| >= yaw_threshold).
        # The latter preserves spot-turn frames so the model can learn
        # rotation-tracking behavior from pure visual input.
        speed_norm    = float(np.linalg.norm(ctrl_body[:3]))
        yaw_rate_abs  = abs(float(ctrl_body[3]))
        turning       = yaw_rate_abs >= yaw_threshold
        valid_motion  = (speed_norm >= speed_threshold) or turning
        filter_reason = '' if valid_motion else 'speed_below_threshold'

        # ── Trajectory validity filter ─────────────────────────────────
        # valid_traj = True when (a) the first extracted waypoint is ahead
        # of the UAV (dx0 >= 0 in body frame) AND (b) the trajectory is not
        # stale/degenerate. bspline_trajectory is not republished while the
        # UAV turns in place without translating, so during a sustained
        # spot-turn all K waypoints can freeze at ~the UAV's own position
        # (near-zero magnitude). This passes dx0 >= 0 trivially but carries
        # no planning signal, so it must be excluded from Trajectory/Joint KD.
        traj_magnitude = float(np.sum(np.abs(traj_body_xyz[:, :2])))
        traj_ahead     = bool(traj_body_xyz[0, 0] >= 0.0)
        traj_not_stale = traj_magnitude >= traj_magnitude_threshold
        valid_traj     = traj_ahead and traj_not_stale
        if not traj_ahead:
            filter_reason = (filter_reason + ',traj_behind_uav') if filter_reason else 'traj_behind_uav'
        if not traj_not_stale:
            filter_reason = (filter_reason + ',traj_degenerate') if filter_reason else 'traj_degenerate'

        sample = {
            'frame_id':      fid,
            'episode_id':    ep_id,
            'timestamp':     float(cb['timestamp']),
            'image_path':    str(img_path),
            'ctrl_body':     ctrl_body,
            'ctrl_world':    ctrl_world,
            'traj_body':     traj_body,
            'traj_world':    traj_world,
            'traj_body_xyz': traj_body_xyz,
            'traj_world_xyz': traj_world_xyz,
            'pose':          pose,
            'speed_body_norm': speed_norm,
            'valid_motion':  valid_motion,
            'valid_traj':    valid_traj,
            'filter_reason': filter_reason,
            'has_dyaw':      False,
            'K':             K,
        }

        if use_red_wp:
            wp = wp_meta[fid]
            sample['wp_u']      = np.float32(wp['wp_u'])
            sample['wp_v']      = np.float32(wp['wp_v'])
            sample['wp_dist_b'] = np.float32(wp['wp_dist_b'])
            sample['wp_index']  = np.int32(wp['wp_index'])

        samples.append(sample)

    report = {
        'episode_id':    ep_id,
        'raw_frames':    raw_frames,
        'aligned_frames': len(frame_ids),
        'missing_images': len(missing_images),
        'valid_samples': len(samples),
        'world':         metadata.get('world', 'unknown'),
        'K':             K,
    }

    if missing_images:
        print(f"  [WARN] {ep_id}: {len(missing_images)} frames missing image files "
              f"(first 5: {missing_images[:5]})")

    return samples, metadata, report


# ── Split assignment ──────────────────────────────────────────────────────────

def assign_splits(all_samples, dataset_type, val_episodes, test_episodes):
    """
    Assign 'split' field to each sample.

    debug:  all → train
    train:  episode-level split using val_episodes / test_episodes lists
    """
    for s in all_samples:
        ep = s['episode_id']
        if dataset_type == 'debug':
            s['split'] = 'train'
        else:
            if ep in test_episodes:
                s['split'] = 'test'
            elif ep in val_episodes:
                s['split'] = 'val'
            else:
                s['split'] = 'train'


# ── Writer ────────────────────────────────────────────────────────────────────

MANIFEST_COLS = [
    'sample_id', 'episode_id', 'frame_id', 'timestamp',
    'image_path', 'sample_path', 'split', 'valid',
    'valid_motion', 'valid_traj', 'speed_body_norm', 'has_dyaw', 'filter_reason',
]

MANIFEST_COLS_RED_WP = MANIFEST_COLS + ['wp_u', 'wp_v', 'wp_dist_b', 'wp_index']


def write_dataset(all_samples, out_dir, dataset_name, dataset_type,
                  episode_reports, speed_threshold, yaw_threshold=DEFAULT_YAW_THRESHOLD,
                  traj_magnitude_threshold=DEFAULT_TRAJ_MAGNITUDE_THRESHOLD, use_red_wp=False):
    out_dir   = Path(out_dir)
    samp_dir  = out_dir / 'samples'
    samp_dir.mkdir(parents=True, exist_ok=True)

    manifest_cols = MANIFEST_COLS_RED_WP if use_red_wp else MANIFEST_COLS
    manifest_rows = []
    image_shapes  = set()

    print(f"[Builder] Writing {len(all_samples)} samples → {samp_dir}")

    for i, s in enumerate(all_samples):
        # Read image
        img = cv2.imread(s['image_path'])
        if img is None:
            print(f"[WARN] Cannot read image at sample {i}: {s['image_path']}")
            continue

        image_shapes.add(img.shape)
        sample_path = samp_dir / f'{i:06d}.npz'

        npz_kwargs = dict(
            image          = img,                         # (H, W, 3) uint8 BGR
            ctrl_body      = s['ctrl_body'],              # (4,)  float32
            ctrl_world     = s['ctrl_world'],             # (4,)  float32
            traj_body      = s['traj_body'],              # (K,4) float32
            traj_world     = s['traj_world'],             # (K,4) float32
            traj_body_xyz  = s['traj_body_xyz'],          # (K,3) float32
            traj_world_xyz = s['traj_world_xyz'],         # (K,3) float32
            pose           = s['pose'],                   # (4,)  float32
            timestamp      = np.float64(s['timestamp']),
            frame_id       = np.int32(s['frame_id']),
            valid_motion   = np.bool_(s['valid_motion']),
            valid_traj     = np.bool_(s['valid_traj']),
            has_dyaw       = np.bool_(s['has_dyaw']),
            episode_id     = np.array(s['episode_id']),  # 0-d str array
        )
        if use_red_wp:
            npz_kwargs['wp_u']      = s['wp_u']
            npz_kwargs['wp_v']      = s['wp_v']
            npz_kwargs['wp_dist_b'] = s['wp_dist_b']
            npz_kwargs['wp_index']  = s['wp_index']

        np.savez_compressed(sample_path, **npz_kwargs)

        mrow = {
            'sample_id':      i,
            'episode_id':     s['episode_id'],
            'frame_id':       s['frame_id'],
            'timestamp':      round(s['timestamp'], 6),
            'image_path':     s['image_path'],
            'sample_path':    str(sample_path),
            'split':          s['split'],
            'valid':          True,
            'valid_motion':   s['valid_motion'],
            'valid_traj':     s['valid_traj'],
            'speed_body_norm': round(s['speed_body_norm'], 4),
            'has_dyaw':       s['has_dyaw'],
            'filter_reason':  s['filter_reason'],
        }
        if use_red_wp:
            mrow['wp_u']      = round(float(s['wp_u']), 2)
            mrow['wp_v']      = round(float(s['wp_v']), 2)
            mrow['wp_dist_b'] = round(float(s['wp_dist_b']), 4)
            mrow['wp_index']  = int(s['wp_index'])
        manifest_rows.append(mrow)

        if (i + 1) % 100 == 0:
            print(f"  {i+1}/{len(all_samples)} ...")

    # ── manifest.csv ─────────────────────────────────────────────────────────
    with open(out_dir / 'manifest.csv', 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=manifest_cols)
        w.writeheader()
        w.writerows(manifest_rows)

    # ── split index files ─────────────────────────────────────────────────────
    for split_name in ('train', 'val', 'test'):
        ids = [r['sample_id'] for r in manifest_rows if r['split'] == split_name]
        with open(out_dir / f'{split_name}_index.csv', 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['sample_id'])
            for sid in ids:
                w.writerow([sid])

    # ── statistics ───────────────────────────────────────────────────────────
    train_rows = [r for r in manifest_rows if r['split'] == 'train']
    val_rows   = [r for r in manifest_rows if r['split'] == 'val']
    test_rows  = [r for r in manifest_rows if r['split'] == 'test']

    ctrl_arr  = np.stack([s['ctrl_body'] for s in all_samples])
    traj_arr  = np.stack([s['traj_body'] for s in all_samples])
    vm_count  = sum(1 for s in all_samples if s['valid_motion'])
    vt_count  = sum(1 for s in all_samples if s['valid_traj'])

    K = all_samples[0]['K']
    img_res = list(image_shapes)[0] if len(image_shapes) == 1 else list(image_shapes)

    summary = {
        'dataset_name':     dataset_name,
        'dataset_type':     dataset_type,
        'source_episodes':  episode_reports,
        'total_raw_frames': sum(r['raw_frames'] for r in episode_reports),
        'total_samples':    len(manifest_rows),
        'train_samples':    len(train_rows),
        'val_samples':      len(val_rows),
        'test_samples':     len(test_rows),
        'K':                K,
        'has_dyaw':         False,
        'dyaw_note':        (
            'dyaw is currently 0 in all samples. '
            'bspline_trajectory waypoint orientations are not stored in the CSV. '
            'To enable: reprocess from rosbag using waypoint quaternions.'
        ),
        'image_resolution': list(img_res) if isinstance(img_res, tuple) else img_res,
        'ctrl_body_mean':   ctrl_arr.mean(axis=0).tolist(),
        'ctrl_body_std':    ctrl_arr.std(axis=0).tolist(),
        'ctrl_body_labels': ['vx_b', 'vy_b', 'vz_b', 'yaw_rate'],
        'traj_body_mean':   traj_arr.reshape(-1, 4).mean(axis=0).tolist(),
        'traj_body_std':    traj_arr.reshape(-1, 4).std(axis=0).tolist(),
        'traj_body_labels': ['dx', 'dy', 'dz', 'dyaw'],
        'valid_motion_count': vm_count,
        'valid_motion_ratio': round(vm_count / len(manifest_rows), 4),
        'valid_traj_count':  vt_count,
        'valid_traj_ratio':  round(vt_count / len(manifest_rows), 4),
        'valid_traj_note':   (
            'valid_traj=True when traj_body dx0>=0 (first waypoint is ahead of UAV) '
            'AND sum(|dx_i|+|dy_i|) over K waypoints >= traj_magnitude_threshold. '
            'False (traj_behind_uav) occurs near goal when all waypoints are behind UAV. '
            'False (traj_degenerate) occurs during sustained spot-turns, where '
            'bspline_trajectory is not republished while the UAV rotates without '
            'translating, so all K waypoints freeze near the UAV\'s own position. '
            'Use filter_valid_traj=True in UavKdDataset for Trajectory/Joint KD training.'
        ),
        'filter_config':    {
            'speed_threshold': speed_threshold,
            'yaw_threshold': yaw_threshold,
            'traj_magnitude_threshold': traj_magnitude_threshold,
        },
    }

    if dataset_type == 'debug':
        summary['warning'] = 'single episode only; not valid for formal evaluation'

    with open(out_dir / 'dataset_summary.json', 'w') as f:
        json.dump(summary, f, indent=2)

    return manifest_rows, summary


# ── CLI ───────────────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(description='Build UAV KD processed dataset from raw episodes')
    p.add_argument('--raw_dir',         default='data/raw',
                   help='Root directory containing raw episode folders')
    p.add_argument('--output_dir',      required=True,
                   help='Output directory for processed dataset')
    p.add_argument('--episodes',        nargs='+', required=True,
                   help='Episode IDs to include (folder names under raw_dir)')
    p.add_argument('--K',               type=int, default=10,
                   help='Number of future trajectory waypoints (default: 10)')
    p.add_argument('--dataset_type',    default='debug', choices=['debug', 'train'],
                   help='"debug": all to train. "train": episode-level split')
    p.add_argument('--val_episodes',    nargs='*', default=[],
                   help='Episodes to assign to val split (train mode only)')
    p.add_argument('--test_episodes',   nargs='*', default=[],
                   help='Episodes to assign to test split (train mode only)')
    p.add_argument('--speed_threshold', type=float, default=DEFAULT_SPEED_THRESHOLD,
                   help='Speed threshold for valid_motion flag (m/s)')
    p.add_argument('--yaw_threshold', type=float, default=DEFAULT_YAW_THRESHOLD,
                   help='Yaw-rate threshold: frames with |yaw_rate| >= this are kept even if speed < speed_threshold (rad/s)')
    p.add_argument('--traj_magnitude_threshold', type=float, default=DEFAULT_TRAJ_MAGNITUDE_THRESHOLD,
                   help='Min sum(|dx_i|+|dy_i|) over K waypoints; below this the trajectory is considered stale/degenerate (valid_traj=False)')
    p.add_argument('--use_red_wp', action='store_true',
                   help='Use images_red_wp/ and waypoint_meta.csv; filter to wp_visible=True only')
    return p.parse_args()


def main():
    args = parse_args()

    raw_dir      = Path(args.raw_dir)
    output_dir   = Path(args.output_dir)
    dataset_name = output_dir.name

    print(f"\n{'='*60}")
    print(f"  UAV KD Dataset Builder")
    print(f"  dataset_type : {args.dataset_type}")
    print(f"  output       : {output_dir}")
    print(f"  episodes     : {args.episodes}")
    print(f"  K            : {args.K}")
    print(f"{'='*60}\n")

    all_samples      = []
    episode_reports  = []

    for ep_id in args.episodes:
        ep_dir = raw_dir / ep_id
        if not ep_dir.exists():
            print(f"[ERROR] Episode directory not found: {ep_dir}")
            sys.exit(1)

        print(f"[Builder] Processing '{ep_id}' ...")
        samples, meta, report = process_episode(
            ep_dir, ep_id, args.K, args.speed_threshold,
            yaw_threshold=args.yaw_threshold,
            traj_magnitude_threshold=args.traj_magnitude_threshold,
            use_red_wp=args.use_red_wp,
        )
        episode_reports.append(report)
        all_samples.extend(samples)

        print(f"  raw={report['raw_frames']}  aligned={report['aligned_frames']}  "
              f"missing_img={report['missing_images']}  valid={report['valid_samples']}")

    if not all_samples:
        print("[ERROR] No samples collected. Check episode directories.")
        sys.exit(1)

    assign_splits(all_samples, args.dataset_type, args.val_episodes, args.test_episodes)

    manifest_rows, summary = write_dataset(
        all_samples, output_dir, dataset_name, args.dataset_type,
        episode_reports, args.speed_threshold, yaw_threshold=args.yaw_threshold,
        traj_magnitude_threshold=args.traj_magnitude_threshold,
        use_red_wp=args.use_red_wp,
    )

    # ── Final report ─────────────────────────────────────────────────────────
    vm_count = summary['valid_motion_count']
    total    = summary['total_samples']

    print(f"\n{'='*60}")
    print(f"  Done.")
    print(f"  Output dir    : {output_dir}")
    print(f"  Total samples : {total}")
    print(f"  Train / Val / Test : {summary['train_samples']} / {summary['val_samples']} / {summary['test_samples']}")
    print(f"  valid_motion  : {vm_count}/{total} ({100*vm_count/total:.1f}%)  ← filter with filter_static=True")
    vt = summary['valid_traj_count']
    print(f"  valid_traj    : {vt}/{total} ({100*vt/total:.1f}%)  ← filter with filter_valid_traj=True (Traj/Joint KD)")
    print(f"  has_dyaw      : {summary['has_dyaw']}  ← fill with 0; reprocess from rosbag to fix")
    print(f"  image_res     : {summary['image_resolution']}")
    print(f"  ctrl_body mean: {[round(v,3) for v in summary['ctrl_body_mean']]}")
    print(f"  ctrl_body std : {[round(v,3) for v in summary['ctrl_body_std']]}")
    if summary.get('warning'):
        print(f"\n  ⚠ WARNING: {summary['warning']}")
    print(f"{'='*60}\n")


if __name__ == '__main__':
    main()
