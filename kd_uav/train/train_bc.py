#!/usr/bin/env python3
"""
train_bc.py — BC Baseline Training

Input  : front camera image  (3, 224, 224)
Output : ctrl_body           (4,)  [vx_b, vy_b, vz_b, yaw_rate]
Loss   : weighted MSE(pred, teacher_ctrl_body)

Usage:
  conda activate dbc241
  cd /home/l/uav_kd_project
  python kd_uav/train/train_bc.py --config configs/train/bc.yaml
"""

import argparse
import json
import os
import sys
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn
from torch.utils.data import DataLoader
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from kd_uav.datasets.uav_kd_dataset import UavKdDataset
from kd_uav.models.control_head import BCPolicy
from kd_uav.losses.bc_loss import BCLoss


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
        # Linear warmup + cosine annealing
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


# ── Train / Val loops ─────────────────────────────────────────────────────────

def run_epoch(model, loader, loss_fn, optimizer, device, train=True):
    model.train(train)
    total_loss = 0.0
    dim_mse    = torch.zeros(4, device=device)
    n          = 0

    with torch.set_grad_enabled(train):
        for batch in loader:
            img  = batch['image'].to(device)        # (B, 3, H, W)
            ctrl = batch['ctrl_body'].to(device)    # (B, 4)

            pred = model(img)
            loss = loss_fn(pred, ctrl)

            if train:
                optimizer.zero_grad()
                loss.backward()
                nn.utils.clip_grad_norm_(model.parameters(), max_norm=1.0)
                optimizer.step()

            bs          = img.size(0)
            total_loss += loss.item() * bs
            dim_mse    += loss_fn.per_dim_mse(pred, ctrl) * bs
            n          += bs

    return total_loss / n, (dim_mse / n).cpu().numpy()


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', default='configs/train/bc.yaml')
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
        label            = 'ctrl_body',
        filter_static    = dcfg['filter_static'],
        filter_valid_traj= dcfg['filter_valid_traj'],
    )
    val_ds = UavKdDataset(
        processed_dir    = dcfg['processed_dir'],
        split            = 'val',
        image_size       = (h, w),
        label            = 'ctrl_body',
        filter_static    = dcfg['filter_static'],
        filter_valid_traj= dcfg['filter_valid_traj'],
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
    model = BCPolicy.build(
        pretrained       = mcfg['pretrained'],
        freeze_backbone  = mcfg['freeze_backbone'],
        hidden_dim       = mcfg['hidden_dim'],
        dropout          = mcfg['dropout'],
    ).to(device)

    n_params = sum(p.numel() for p in model.parameters()) / 1e6
    log(f"Model   : BCPolicy  {n_params:.2f}M params")

    # ── Loss & Optimizer ──────────────────────────────────────────────────────
    lcfg    = cfg['loss']
    loss_fn = BCLoss(weights=lcfg['weights']).to(device)

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

    log(f"\n{'Epoch':>5}  {'Train':>10}  {'Val':>10}  {'vx':>8}  {'vy':>8}  {'vz':>8}  {'yr':>8}  {'LR':>10}")
    log('─' * 80)

    for epoch in range(1, epochs + 1):
        t0 = time.time()

        train_loss, _        = run_epoch(model, train_loader, loss_fn, optimizer, device, train=True)
        val_loss,   dim_mse  = run_epoch(model, val_loader,   loss_fn, optimizer, device, train=False)

        lr_now = optimizer.param_groups[0]['lr']
        if scheduler is not None:
            scheduler.step()

        history['train_loss'].append(train_loss)
        history['val_loss'].append(val_loss)
        history['lr'].append(lr_now)

        log(f"{epoch:>5}  {train_loss:>10.6f}  {val_loss:>10.6f}  "
            f"{dim_mse[0]:>8.4f}  {dim_mse[1]:>8.4f}  {dim_mse[2]:>8.4f}  {dim_mse[3]:>8.4f}  "
            f"{lr_now:>10.2e}  ({time.time()-t0:.1f}s)")

        # Save best
        if val_loss < best_val:
            best_val = val_loss
            torch.save(model.state_dict(), run_dir / 'best_model.pt')

        # Periodic checkpoint
        if epoch % cfg['output']['save_every'] == 0:
            torch.save(model.state_dict(), run_dir / f'ckpt_ep{epoch:03d}.pt')

    # ── Save final ────────────────────────────────────────────────────────────
    torch.save(model.state_dict(), run_dir / 'last_model.pt')

    metrics = {
        'best_val_loss': best_val,
        'final_train_loss': history['train_loss'][-1],
        'final_val_loss':   history['val_loss'][-1],
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
        ax.set_ylabel('Loss (weighted MSE)')
        ax.set_title(f'BC Baseline — {run_id}')
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
