#!/usr/bin/env python3
"""
uav_kd_dataset.py

PyTorch Dataset class for the UAV Knowledge Distillation project.

Usage:
    from kd_uav.datasets.uav_kd_dataset import UavKdDataset

    dataset = UavKdDataset(
        processed_dir="data/processed/uav_kd_debug_v000",
        split="train",
        image_size=(224, 224),
        label="ctrl_body",          # 'ctrl_body' | 'traj_body' | 'both'
        filter_static=True,         # exclude valid_motion=False frames
        filter_valid_traj=False,    # exclude valid_traj=False frames (set True for Traj/Joint KD)
    )

    item = dataset[0]
    # item['image']     : Tensor(3, H, W)  float32  RGB  [0, 1]
    # item['ctrl_body'] : Tensor(4,)       float32  [vx_b, vy_b, vz_b, yaw_rate]
    # item['traj_body'] : Tensor(K, 4)     float32  [dx, dy, dz, dyaw]
    # item['pose']      : Tensor(4,)       float32  [x, y, z, yaw]
    # item['sample_id'] : str
    # item['episode_id']: str
"""

import csv
import json
from pathlib import Path

import cv2
import numpy as np
import torch
from torch.utils.data import Dataset


class UavKdDataset(Dataset):
    """
    UAV KD processed dataset.

    Args:
        processed_dir     : path to processed dataset (e.g. data/processed/uav_kd_debug_v000)
        split             : 'train' | 'val' | 'test'
        image_size        : (H, W) to resize images; None = keep original size
        label             : which labels to include — 'ctrl_body' | 'traj_body' | 'both'
        filter_static     : if True, exclude frames where valid_motion=False
        filter_valid_traj : if True, exclude frames where valid_traj=False
                            (dx0 < 0, UAV past all trajectory waypoints).
                            Use True for Trajectory KD and Joint KD training;
                            leave False for BC training.
        return_raw_image  : if True, also include 'image_raw_bgr' — the
                            original-resolution (H,W,3) uint8 BGR image,
                            undecorated. Needed by callers that draw on the
                            image themselves (e.g. SelfRedWP's online red-dot
                            rendering), since waypoint_projection.py's pinhole
                            projection is calibrated for the original 640x480
                            resolution, not the resized training resolution.
    """

    def __init__(
        self,
        processed_dir,
        split='train',
        image_size=(224, 224),
        label='ctrl_body',
        filter_static=True,
        filter_valid_traj=False,
        return_raw_image=False,
    ):
        self.processed_dir     = Path(processed_dir)
        self.split             = split
        self.image_size        = image_size
        self.label             = label
        self.filter_static     = filter_static
        self.filter_valid_traj = filter_valid_traj
        self.return_raw_image  = return_raw_image

        # Validate
        if label not in ('ctrl_body', 'traj_body', 'both'):
            raise ValueError(f"label must be 'ctrl_body', 'traj_body', or 'both'; got '{label}'")

        manifest_path = self.processed_dir / 'manifest.csv'
        if not manifest_path.exists():
            raise FileNotFoundError(
                f"manifest.csv not found in: {processed_dir}\n"
                f"Run dataset_builder.py first."
            )

        # Load and filter manifest
        self._rows = []
        with open(manifest_path, newline='') as f:
            reader = csv.DictReader(f)
            for row in reader:
                if row['split'] != split:
                    continue
                if filter_static and row['valid_motion'].lower() == 'false':
                    continue
                if filter_valid_traj and row.get('valid_traj', 'true').lower() == 'false':
                    continue
                self._rows.append(row)

        if not self._rows:
            print(
                f"[UavKdDataset] WARNING: 0 samples for split='{split}' "
                f"filter_static={filter_static}. "
                f"Check manifest.csv or try filter_static=False."
            )

    # ── Dataset interface ─────────────────────────────────────────────────────

    def __len__(self):
        return len(self._rows)

    def __getitem__(self, idx):
        row  = self._rows[idx]
        data = np.load(row['sample_path'], allow_pickle=True)

        # ── Image ─────────────────────────────────────────────────────────────
        img_raw_bgr = data['image']                      # (H, W, 3)  uint8  BGR, original resolution
        img = cv2.cvtColor(img_raw_bgr, cv2.COLOR_BGR2RGB)  # → RGB
        if self.image_size is not None:
            img = cv2.resize(img, (self.image_size[1], self.image_size[0]),
                             interpolation=cv2.INTER_LINEAR)
        img_t = torch.from_numpy(img).permute(2, 0, 1).float() / 255.0  # (3, H, W) [0,1]

        # ── Labels ────────────────────────────────────────────────────────────
        ctrl_body = torch.from_numpy(data['ctrl_body'].astype(np.float32))   # (4,)
        traj_body = torch.from_numpy(data['traj_body'].astype(np.float32))   # (K, 4)
        pose      = torch.from_numpy(data['pose'].astype(np.float32))        # (4,)

        item = {
            'image':      img_t,
            'ctrl_body':  ctrl_body,
            'traj_body':  traj_body,
            'pose':       pose,
            'sample_id':  row['sample_id'],
            'episode_id': row['episode_id'],
        }
        if self.return_raw_image:
            # Original-resolution BGR image, undecorated — for callers that need
            # to draw on the image themselves (waypoint_projection.py's pinhole
            # math is calibrated for the original 640x480 resolution, not the
            # resized training resolution). Kept uint8 (not normalized/resized).
            item['image_raw_bgr'] = img_raw_bgr
        return item

    # ── Utility ───────────────────────────────────────────────────────────────

    def get_summary(self):
        """Return the dataset_summary.json as a dict (or {} if missing)."""
        p = self.processed_dir / 'dataset_summary.json'
        if p.exists():
            with open(p) as f:
                return json.load(f)
        return {}

    def get_normalization_stats(self):
        """
        Return (ctrl_mean, ctrl_std, traj_mean, traj_std) as numpy arrays,
        sourced from dataset_summary.json.
        Returns None if summary is not available.
        """
        summary = self.get_summary()
        if not summary or 'ctrl_body_mean' not in summary:
            return None
        return (
            np.array(summary['ctrl_body_mean'], dtype=np.float32),
            np.array(summary['ctrl_body_std'],  dtype=np.float32),
            np.array(summary['traj_body_mean'], dtype=np.float32),
            np.array(summary['traj_body_std'],  dtype=np.float32),
        )

    def __repr__(self):
        return (
            f"UavKdDataset("
            f"split='{self.split}', "
            f"n={len(self)}, "
            f"label='{self.label}', "
            f"image_size={self.image_size}, "
            f"filter_static={self.filter_static}, "
            f"filter_valid_traj={self.filter_valid_traj})"
        )
