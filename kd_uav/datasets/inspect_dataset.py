#!/usr/bin/env python3
"""
inspect_dataset.py

Quick visual and numerical check of a processed UAV KD dataset.

Usage:
  conda activate dbc_bc
  cd /home/l/uav_kd_project

  python kd_uav/datasets/inspect_dataset.py \
      --processed_dir data/processed/uav_kd_debug_v000 \
      --split train \
      --num_samples 10

Output:
  - Printed statistics (sample count, shapes, ctrl/traj ranges, valid_motion ratio)
  - Annotated sample images saved to results/figures/dataset_inspection/
  - DataLoader batch shape verification
"""

import argparse
import json
import random
import sys
from pathlib import Path

import cv2
import numpy as np
import torch
from torch.utils.data import DataLoader

# Allow running from project root without installing the package
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from kd_uav.datasets.uav_kd_dataset import UavKdDataset


# ── CLI ───────────────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(description='Inspect a UAV KD processed dataset')
    p.add_argument('--processed_dir', required=True,
                   help='Path to processed dataset directory')
    p.add_argument('--split',         default='train',
                   choices=['train', 'val', 'test'],
                   help='Which split to inspect')
    p.add_argument('--num_samples',   type=int, default=10,
                   help='Number of random samples to save as images')
    p.add_argument('--image_size',    type=int, nargs=2, default=[224, 224],
                   metavar=('H', 'W'),
                   help='Image resize target (default: 224 224)')
    p.add_argument('--no_filter',     action='store_true',
                   help='Disable valid_motion filter (include static frames)')
    p.add_argument('--out_dir',       default='results/figures/dataset_inspection',
                   help='Directory to save inspection images')
    p.add_argument('--batch_size',    type=int, default=4,
                   help='Batch size for DataLoader test')
    return p.parse_args()


# ── Helpers ───────────────────────────────────────────────────────────────────

def _sep(title=''):
    w = 62
    if title:
        pad = (w - len(title) - 2) // 2
        print('─' * pad + f' {title} ' + '─' * (w - pad - len(title) - 2))
    else:
        print('─' * w)


def _stat_row(name, arr, fmt='{:+.4f}'):
    mn, mx, mu, sd = arr.min(), arr.max(), arr.mean(), arr.std()
    print(f"  {name:<18s} min={fmt.format(mn)}  max={fmt.format(mx)}"
          f"  mean={fmt.format(mu)}  std={fmt.format(sd)}")


def annotate_image(img_rgb_uint8, ctrl, traj, sample_id, episode_id):
    """Draw stats overlay on image for visual inspection."""
    img = cv2.cvtColor(img_rgb_uint8, cv2.COLOR_RGB2BGR)
    lines = [
        f"ep={episode_id}  sid={sample_id}",
        f"vx_b={ctrl[0]:+.2f} vy_b={ctrl[1]:+.2f} vz_b={ctrl[2]:+.2f} yr={ctrl[3]:+.3f}",
        f"traj dx[0]={traj[0,0]:+.2f}  dx[-1]={traj[-1,0]:+.2f}",
        f"traj dy[0]={traj[0,1]:+.2f}  dz[0]={traj[0,2]:+.3f}",
    ]
    y = 18
    for line in lines:
        cv2.putText(img, line, (5, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 60), 1, cv2.LINE_AA)
        y += 18
    return img


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    args = parse_args()

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    print()
    _sep('UAV KD Dataset Inspection')

    # ── Load dataset summary ──────────────────────────────────────────────────
    summary_path = Path(args.processed_dir) / 'dataset_summary.json'
    if summary_path.exists():
        with open(summary_path) as f:
            summary = json.load(f)
        _sep('dataset_summary.json')
        fields = [
            ('dataset_name',   summary.get('dataset_name')),
            ('dataset_type',   summary.get('dataset_type')),
            ('total_samples',  summary.get('total_samples')),
            ('train/val/test', f"{summary.get('train_samples')} / "
                               f"{summary.get('val_samples')} / "
                               f"{summary.get('test_samples')}"),
            ('K',              summary.get('K')),
            ('has_dyaw',       summary.get('has_dyaw')),
            ('image_res',      summary.get('image_resolution')),
            ('valid_motion',   f"{summary.get('valid_motion_count')} / "
                               f"{summary.get('total_samples')} "
                               f"({100*summary.get('valid_motion_ratio',0):.1f}%)"),
        ]
        for k, v in fields:
            print(f"  {k:<20s}: {v}")
        if summary.get('warning'):
            print(f"\n  ⚠  WARNING: {summary['warning']}")
    else:
        print(f"  [WARN] dataset_summary.json not found in {args.processed_dir}")
        summary = {}

    # ── Load dataset ─────────────────────────────────────────────────────────
    _sep(f"Loading split='{args.split}'")
    filter_static = not args.no_filter

    dataset = UavKdDataset(
        processed_dir=args.processed_dir,
        split=args.split,
        image_size=tuple(args.image_size),
        label='both',
        filter_static=filter_static,
    )
    print(f"  {dataset}")
    print(f"  Loaded {len(dataset)} samples  (filter_static={filter_static})")

    if len(dataset) == 0:
        print("  [ERROR] No samples found — nothing to inspect.")
        sys.exit(1)

    # ── Collect statistics over all samples ───────────────────────────────────
    _sep('Computing statistics (full split)')
    print("  (iterating all samples — may take a moment for large datasets)")

    ctrl_all  = []
    traj_all  = []
    pose_all  = []

    for i in range(len(dataset)):
        item = dataset[i]
        ctrl_all.append(item['ctrl_body'].numpy())
        traj_all.append(item['traj_body'].numpy())
        pose_all.append(item['pose'].numpy())

    ctrl_arr = np.stack(ctrl_all)   # (N, 4)
    traj_arr = np.stack(traj_all)   # (N, K, 4)
    pose_arr = np.stack(pose_all)   # (N, 4)

    # ── Image shape ──────────────────────────────────────────────────────────
    _sep('Image')
    sample0 = dataset[0]
    img0    = sample0['image']
    print(f"  shape (C,H,W) : {tuple(img0.shape)}")
    print(f"  dtype         : {img0.dtype}")
    print(f"  value range   : [{img0.min():.4f}, {img0.max():.4f}]")

    # ── ctrl_body ────────────────────────────────────────────────────────────
    _sep('ctrl_body  [vx_b, vy_b, vz_b, yaw_rate]')
    for i, name in enumerate(['vx_b', 'vy_b', 'vz_b', 'yaw_rate']):
        _stat_row(name, ctrl_arr[:, i])

    # ── traj_body ────────────────────────────────────────────────────────────
    K = traj_arr.shape[1]
    _sep(f'traj_body  ({K}×4: dx, dy, dz, dyaw)')
    print(f"  shape (N,K,4) : {traj_arr.shape}")
    print(f"  --- first waypoint (i=0) ---")
    for j, name in enumerate(['dx', 'dy', 'dz', 'dyaw']):
        _stat_row(f'  {name}[0]', traj_arr[:, 0, j])
    print(f"  --- last waypoint (i={K-1}) ---")
    for j, name in enumerate(['dx', 'dy', 'dz', 'dyaw']):
        _stat_row(f'  {name}[{K-1}]', traj_arr[:, K-1, j])
    dyaw_max = np.abs(traj_arr[:, :, 3]).max()
    if dyaw_max < 1e-6:
        print(f"  dyaw: all zero  (has_dyaw=False — expected)")

    # ── pose ─────────────────────────────────────────────────────────────────
    _sep('pose  [x, y, z, yaw]')
    for i, name in enumerate(['x', 'y', 'z', 'yaw']):
        _stat_row(name, pose_arr[:, i])

    # ── valid_motion ─────────────────────────────────────────────────────────
    # Re-count from the unfiltered manifest to show true ratio
    import csv as csv_mod
    manifest_path = Path(args.processed_dir) / 'manifest.csv'
    all_vm, split_vm_total, split_vm_valid = 0, 0, 0
    with open(manifest_path) as f:
        for row in csv_mod.DictReader(f):
            all_vm += 1
            if row['split'] == args.split:
                split_vm_total += 1
                if row['valid_motion'].lower() == 'true':
                    split_vm_valid += 1
    _sep('valid_motion (all frames in manifest)')
    print(f"  split='{args.split}' total frames : {split_vm_total}")
    print(f"  valid_motion=True               : {split_vm_valid} "
          f"({100*split_vm_valid/max(split_vm_total,1):.1f}%)")
    print(f"  static frames                   : {split_vm_total - split_vm_valid}")

    # ── Save inspection images ────────────────────────────────────────────────
    n_save = min(args.num_samples, len(dataset))
    _sep(f'Saving {n_save} annotated samples → {out_dir}')
    indices = random.sample(range(len(dataset)), n_save)

    for j, idx in enumerate(indices):
        item = dataset[idx]
        img_np  = (item['image'].permute(1, 2, 0).numpy() * 255).astype(np.uint8)
        ctrl_np = item['ctrl_body'].numpy()
        traj_np = item['traj_body'].numpy()

        annotated = annotate_image(
            img_np, ctrl_np, traj_np,
            item['sample_id'], item['episode_id']
        )
        fname = out_dir / f'inspect_{args.split}_{j:03d}_sid{item["sample_id"]}.png'
        cv2.imwrite(str(fname), annotated)
        print(f"  [{j+1:2d}/{n_save}] {fname.name}  "
              f"vx_b={ctrl_np[0]:+.2f}  traj_dx0={traj_np[0,0]:+.2f}")

    # ── DataLoader test ───────────────────────────────────────────────────────
    _sep(f'DataLoader test (batch_size={args.batch_size})')
    loader = DataLoader(
        dataset, batch_size=args.batch_size,
        shuffle=True, num_workers=0, drop_last=False,
    )
    batch = next(iter(loader))
    print(f"  image     : {tuple(batch['image'].shape)}   dtype={batch['image'].dtype}")
    print(f"  ctrl_body : {tuple(batch['ctrl_body'].shape)}")
    print(f"  traj_body : {tuple(batch['traj_body'].shape)}")
    print(f"  pose      : {tuple(batch['pose'].shape)}")
    print(f"  ✓ DataLoader OK")

    # ── Final ─────────────────────────────────────────────────────────────────
    _sep('Done')
    print(f"  Inspection images : {out_dir}")
    print(f"  Samples inspected : {n_save}")
    print(f"  DataLoader        : OK")
    _sep()
    print()


if __name__ == '__main__':
    main()
