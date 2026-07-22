#!/usr/bin/env python3
"""
train_self_red_wp.py — SelfRedWP Training

Two-pass, single shared backbone:
  Pass 1: raw image -> predict_traj -> K waypoints
  Pass 2: image + red dot (drawn at a selected waypoint) -> predict_ctrl -> cmd_vel

Curriculum: the dot source is chosen per-sample by a coin flip with
probability p_self(epoch) of using the model's own (detached) predicted
waypoint instead of ground truth. p_self ramps 0->1 over training so the
control head first learns a clean mapping on correct guidance, then adapts
to the model's own (biased) prediction distribution before deployment.
Validation always uses self-predicted waypoints (matches deployment).

Usage:
  conda activate dbc241
  cd /home/l/uav_kd_project
  python kd_uav/train/train_self_red_wp.py --config configs/train/self_red_wp.yaml
"""

import argparse
import json
import random
import sys
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import torch
import torch.nn as nn
from torch.utils.data import DataLoader
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from kd_uav.datasets.uav_kd_dataset import UavKdDataset
from kd_uav.models.self_red_wp_policy import SelfRedWPPolicy
from kd_uav.losses.bc_loss import BCLoss
from kd_uav.losses.trajectory_kd_loss import TrajectoryKDLoss
from kd_uav.utils.waypoint_projection import (
    select_active_waypoint, project_body_to_pixel, draw_red_waypoint,
)


# ── Helpers ───────────────────────────────────────────────────────────────────

def load_config(path):
    with open(path) as f:
        return yaml.safe_load(f)


def make_run_dir(runs_dir):
    run_id  = datetime.now().strftime('%Y%m%d_%H%M%S')
    run_dir = Path(runs_dir) / run_id
    run_dir.mkdir(parents=True, exist_ok=True)
    return run_dir, run_id


def save_config(cfg, run_dir):
    with open(run_dir / 'config.yaml', 'w') as f:
        yaml.dump(cfg, f, default_flow_style=False, sort_keys=False)


def make_scheduler(optimizer, cfg, steps_per_epoch):
    name = cfg['train'].get('lr_scheduler', 'cosine')
    epochs = cfg['train']['epochs']
    warmup = cfg['train'].get('lr_warmup_epochs', 0)

    if name == 'cosine':
        def lr_lambda(epoch):
            if epoch < warmup:
                return (epoch + 1) / max(warmup, 1)
            progress = (epoch - warmup) / max(epochs - warmup, 1)
            return 0.5 * (1 + np.cos(np.pi * progress))
        return torch.optim.lr_scheduler.LambdaLR(optimizer, lr_lambda)
    elif name == 'step':
        return torch.optim.lr_scheduler.StepLR(optimizer, step_size=20, gamma=0.5)
    else:
        return None


def p_self_for_epoch(epoch, start_epoch, ramp_end_epoch):
    """Curriculum schedule: 0 for epoch<=start_epoch, linear ramp to 1 by
    ramp_end_epoch, held at 1.0 afterward."""
    if epoch <= start_epoch:
        return 0.0
    if epoch >= ramp_end_epoch:
        return 1.0
    return (epoch - start_epoch) / (ramp_end_epoch - start_epoch)


def build_annotated_batch(traj_gt, traj_pred, image_raw_bgr, p_self,
                          lookahead, K, image_size, device, force_self=False):
    """
    Per-sample: pick ground-truth or self-predicted (detached) waypoint
    (coin flip with prob p_self, unless force_self=True), select the active
    waypoint, draw it as a red dot on the original-resolution image, resize
    to training resolution.

    Args:
        traj_gt       : (B, K, 3) tensor, ground-truth body-frame waypoints
        traj_pred     : (B, K, 3) tensor, model's own predicted waypoints
        image_raw_bgr : (B, H, W, 3) uint8 tensor, original-resolution BGR
        p_self        : float, probability of using traj_pred this batch
        force_self    : if True, always use traj_pred (used for validation)

    Returns:
        (B, 3, image_size[0], image_size[1]) float32 tensor, RGB, [0,1]
    """
    B = traj_gt.shape[0]
    traj_gt_np   = traj_gt.detach().cpu().numpy()
    traj_pred_np = traj_pred.detach().cpu().numpy()
    raw_np       = image_raw_bgr.numpy() if torch.is_tensor(image_raw_bgr) else image_raw_bgr

    out = np.empty((B, image_size[0], image_size[1], 3), dtype=np.float32)
    for i in range(B):
        use_self = force_self or (random.random() < p_self)
        src = traj_pred_np[i] if use_self else traj_gt_np[i]   # (K, 3)
        dx, dy, dz, _ = select_active_waypoint(src.reshape(-1), lookahead, K)
        u, v, visible = project_body_to_pixel(dx, dy, dz)

        img = raw_np[i]
        if visible:
            dist_b = float(np.sqrt(dx**2 + dy**2 + dz**2))
            annotated = draw_red_waypoint(img, u, v, dist_b)
        else:
            annotated = img.copy()

        rgb = cv2.cvtColor(annotated, cv2.COLOR_BGR2RGB)
        resized = cv2.resize(rgb, (image_size[1], image_size[0]), interpolation=cv2.INTER_LINEAR)
        out[i] = resized.astype(np.float32) / 255.0

    out_t = torch.from_numpy(out).permute(0, 3, 1, 2).to(device)  # (B, 3, H, W)
    return out_t


# ── Train / Val loops ─────────────────────────────────────────────────────────

def run_epoch(model, loader, ctrl_loss_fn, traj_loss_fn, lambda_traj,
               curriculum_cfg, image_size, epoch, optimizer, device, train=True):
    model.train(train)
    total_loss      = 0.0
    total_ctrl_loss = 0.0
    total_traj_loss = 0.0
    dim_mse         = torch.zeros(4, device=device)
    total_ate       = 0.0
    all_yr          = []
    n               = 0

    p_self = 1.0 if not train else p_self_for_epoch(
        epoch, curriculum_cfg['p_self_start_epoch'], curriculum_cfg['p_self_ramp_end_epoch'])

    with torch.set_grad_enabled(train):
        for batch in loader:
            img       = batch['image'].to(device)              # (B, 3, H, W) raw, resized
            ctrl      = batch['ctrl_body'].to(device)           # (B, 4)
            traj_full = batch['traj_body'].to(device)           # (B, K, 4)
            traj_gt   = traj_full[:, :, :3]                     # (B, K, 3)
            img_raw   = batch['image_raw_bgr']                  # (B, H0, W0, 3) uint8

            # Pass 1: predict trajectory from raw image
            traj_pred = model.predict_traj(img)

            # Build annotated batch for pass 2 (curriculum-mixed if training, always self if val)
            annotated = build_annotated_batch(
                traj_gt, traj_pred, img_raw, p_self,
                curriculum_cfg['lookahead'], traj_gt.shape[1], image_size, device,
                force_self=not train,
            )

            # Pass 2: predict control from annotated image
            ctrl_pred = model.predict_ctrl(annotated)

            ctrl_loss = ctrl_loss_fn(ctrl_pred, ctrl)
            traj_loss = traj_loss_fn(traj_pred, traj_gt)
            loss      = lambda_traj * traj_loss + ctrl_loss

            if train:
                optimizer.zero_grad()
                loss.backward()
                nn.utils.clip_grad_norm_(model.parameters(), max_norm=1.0)
                optimizer.step()

            bs               = img.size(0)
            total_loss      += loss.item() * bs
            total_ctrl_loss += ctrl_loss.item() * bs
            total_traj_loss += traj_loss.item() * bs
            dim_mse         += ctrl_loss_fn.per_dim_mse(ctrl_pred, ctrl) * bs
            total_ate       += traj_loss_fn.ate(traj_pred, traj_gt).item() * bs
            all_yr.append(ctrl_pred[:, 3].detach().cpu().numpy())
            n                += bs

    all_yr = np.concatenate(all_yr)
    yr_std  = float(all_yr.std())
    yr_frac = float((np.abs(all_yr) > 0.15).mean())

    return (
        total_loss / n,
        total_ctrl_loss / n,
        total_traj_loss / n,
        (dim_mse / n).cpu().numpy(),
        total_ate / n,
        p_self,
        yr_std,
        yr_frac,
    )


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', default='configs/train/self_red_wp.yaml')
    args = parser.parse_args()

    cfg = load_config(args.config)
    run_dir, run_id = make_run_dir(cfg['output']['runs_dir'])
    save_config(cfg, run_dir)

    log_path = run_dir / 'train.log'
    log_f    = open(log_path, 'w')

    def log(msg):
        print(msg)
        log_f.write(msg + '\n')
        log_f.flush()

    log(f"Run ID  : {run_id}")
    log(f"Run dir : {run_dir}")
    log(f"Config  : {args.config}")
    log("")

    # ── Device ────────────────────────────────────────────────────────────────
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    log(f"Device  : {device}")

    # ── Dataset ───────────────────────────────────────────────────────────────
    dcfg  = cfg['dataset']
    h, w  = dcfg['image_size']

    train_ds = UavKdDataset(
        processed_dir    = dcfg['processed_dir'],
        split            = 'train',
        image_size       = (h, w),
        label            = 'both',
        filter_static    = dcfg['filter_static'],
        filter_valid_traj= dcfg['filter_valid_traj'],
        return_raw_image = True,
    )
    val_ds = UavKdDataset(
        processed_dir    = dcfg['processed_dir'],
        split            = 'val',
        image_size       = (h, w),
        label            = 'both',
        filter_static    = dcfg['filter_static'],
        filter_valid_traj= dcfg['filter_valid_traj'],
        return_raw_image = True,
    )

    tcfg = cfg['train']
    train_loader = DataLoader(
        train_ds, batch_size=tcfg['batch_size'],
        shuffle=True,  num_workers=tcfg['num_workers'], drop_last=True,
    )
    val_loader = DataLoader(
        val_ds,   batch_size=tcfg['batch_size'],
        shuffle=False, num_workers=tcfg['num_workers'], drop_last=False,
    )

    log(f"Train   : {len(train_ds)} samples  ({len(train_loader)} batches)")
    log(f"Val     : {len(val_ds)} samples  ({len(val_loader)} batches)")

    # ── Model ─────────────────────────────────────────────────────────────────
    mcfg  = cfg['model']
    model = SelfRedWPPolicy.build(
        pretrained       = mcfg['pretrained'],
        freeze_backbone  = mcfg['freeze_backbone'],
        hidden_dim       = mcfg['hidden_dim'],
        dropout          = mcfg['dropout'],
        K                = mcfg['K'],
    ).to(device)

    n_params = sum(p.numel() for p in model.parameters()) / 1e6
    log(f"Model   : SelfRedWPPolicy  {n_params:.2f}M params  (K={mcfg['K']})")

    # ── Loss & Optimizer ──────────────────────────────────────────────────────
    lcfg         = cfg['loss']
    ctrl_loss_fn = BCLoss(weights=lcfg['ctrl_weights']).to(device)
    traj_loss_fn = TrajectoryKDLoss(weights=lcfg['traj_weights']).to(device)
    lambda_traj  = lcfg['lambda_traj']
    curriculum_cfg = cfg['curriculum']

    optimizer = torch.optim.Adam(
        model.parameters(),
        lr           = tcfg['lr'],
        weight_decay = tcfg['weight_decay'],
    )
    scheduler = make_scheduler(optimizer, cfg, len(train_loader))

    # ── Training loop ─────────────────────────────────────────────────────────
    epochs   = tcfg['epochs']
    best_val = float('inf')
    history  = {'train_loss': [], 'val_loss': [], 'lr': []}

    log(f"\n{'Epoch':>5}  {'Train':>10}  {'Val':>10}  {'ctrl':>8}  {'traj':>8}  "
        f"{'vx':>8}  {'vy':>8}  {'vz':>8}  {'yr':>8}  {'ATE':>8}  {'p_self':>7}  "
        f"{'yr_std':>7}  {'yr>.15':>7}  {'LR':>10}")
    log('─' * 150)

    for epoch in range(1, epochs + 1):
        t0 = time.time()

        (train_loss, train_ctrl, train_traj, _, _, train_p_self, _, _) = run_epoch(
            model, train_loader, ctrl_loss_fn, traj_loss_fn, lambda_traj,
            curriculum_cfg, (h, w), epoch, optimizer, device, train=True,
        )
        (val_loss, val_ctrl, val_traj, dim_mse, val_ate, _, val_yr_std, val_yr_frac) = run_epoch(
            model, val_loader, ctrl_loss_fn, traj_loss_fn, lambda_traj,
            curriculum_cfg, (h, w), epoch, optimizer, device, train=False,
        )

        lr_now = optimizer.param_groups[0]['lr']
        if scheduler is not None:
            scheduler.step()

        history['train_loss'].append(train_loss)
        history['val_loss'].append(val_loss)
        history['lr'].append(lr_now)

        log(f"{epoch:>5}  {train_loss:>10.6f}  {val_loss:>10.6f}  "
            f"{val_ctrl:>8.4f}  {val_traj:>8.4f}  "
            f"{dim_mse[0]:>8.4f}  {dim_mse[1]:>8.4f}  {dim_mse[2]:>8.4f}  {dim_mse[3]:>8.4f}  "
            f"{val_ate:>8.4f}  {train_p_self:>7.2f}  "
            f"{val_yr_std:>7.4f}  {val_yr_frac:>7.3f}  "
            f"{lr_now:>10.2e}  ({time.time()-t0:.1f}s)")

        if val_loss < best_val:
            best_val = val_loss
            torch.save(model.state_dict(), run_dir / 'best_model.pt')

        if epoch % cfg['output']['save_every'] == 0:
            torch.save(model.state_dict(), run_dir / f'ckpt_ep{epoch:03d}.pt')

    # ── Save final ────────────────────────────────────────────────────────────
    torch.save(model.state_dict(), run_dir / 'last_model.pt')

    metrics = {
        'best_val_loss': best_val,
        'final_train_loss': history['train_loss'][-1],
        'final_val_loss':   history['val_loss'][-1],
        'final_val_ctrl_loss': val_ctrl,
        'final_val_traj_loss': val_traj,
        'final_val_ate':       val_ate,
        'final_val_dim_mse':   dim_mse.tolist(),
        'final_val_yr_std':    val_yr_std,
        'final_val_yr_frac_gt_0.15': val_yr_frac,
        'lambda_traj': lambda_traj,
        'K': mcfg['K'],
        'curriculum': curriculum_cfg,
        'epochs': epochs,
        'train_samples': len(train_ds),
        'val_samples':   len(val_ds),
        'dim_labels': ['vx_b', 'vy_b', 'vz_b', 'yaw_rate'],
    }
    with open(run_dir / 'metrics.json', 'w') as f:
        json.dump(metrics, f, indent=2)

    # ── Loss curve ────────────────────────────────────────────────────────────
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt

        fig, ax = plt.subplots(figsize=(8, 4))
        ax.plot(history['train_loss'], label='train')
        ax.plot(history['val_loss'],   label='val')
        ax.set_xlabel('Epoch')
        ax.set_ylabel(f'Loss (lambda_traj={lambda_traj} * L_traj + L_ctrl)')
        ax.set_title(f'SelfRedWP — {run_id}')
        ax.legend()
        ax.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.savefig(run_dir / 'loss_curve.png', dpi=120)
        plt.close()
        log(f"\nLoss curve → {run_dir / 'loss_curve.png'}")
    except Exception as e:
        log(f"[WARN] Could not save loss curve: {e}")

    log(f"\nDone.  best_val={best_val:.6f}  run_dir={run_dir}")
    log_f.close()


if __name__ == '__main__':
    main()
