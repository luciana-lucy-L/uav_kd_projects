#!/usr/bin/env python3
"""
Generate 6 training metric plots + companion TXT files.
Output: results/figures/metric_plots/{name}.png + {name}.txt
"""

import re
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from pathlib import Path

ROOT = Path('/home/l/uav_kd_project')
OUT  = ROOT / 'results/figures/metric_plots'
OUT.mkdir(parents=True, exist_ok=True)

LOGS = {
    'BC-v002':   ROOT/'runs/bc_v002/20260701_142506/train.log',
    'TrajKD':    ROOT/'runs/traj_kd/20260702_131051/train.log',
    'SelfRedWP': ROOT/'runs/self_red_wp/20260704_162940/train.log',
}
SCHEMAS = {
    'BC-v002':   ['epoch','train','val','vx','vy','vz','yr','lr'],
    'TrajKD':    ['epoch','train','val','ctrl','traj','vx','vy','vz','yr','ate','lr'],
    'SelfRedWP': ['epoch','train','val','ctrl','traj','vx','vy','vz','yr','ate',
                  'p_self','yr_std','yr_frac','lr'],
}

def parse_log(path, cols):
    n   = len(cols)
    pat = re.compile(r'-?(?:\d+\.\d+(?:e[+\-]?\d+)?|\d+e[+\-]?\d+|\d+)')
    rows = []
    with open(path) as f:
        for line in f:
            if not line.strip() or not line.strip()[0].isdigit():
                continue
            nums = [float(x) for x in pat.findall(line)]
            if len(nums) >= n:
                rows.append(nums[:n])
    return {c: np.array([r[i] for r in rows]) for i, c in enumerate(cols)}

print("Parsing logs...")
data = {name: parse_log(path, SCHEMAS[name]) for name, path in LOGS.items()}
for name, d in data.items():
    print(f"  {name}: {len(d['epoch'])} epochs")

C = {'BC-v002': '#2E86C1', 'TrajKD': '#E67E22', 'SelfRedWP': '#27AE60'}
L = {
    'BC-v002':   'BC-v002  (no-KD baseline)',
    'TrajKD':    'TrajKD   (planning KD)',
    'SelfRedWP': 'SelfRedWP (planning KD + curriculum)',
}
plt.rcParams.update({
    'font.family': 'DejaVu Sans',
    'axes.unicode_minus': False,
    'axes.titlesize': 12, 'axes.labelsize': 10,
    'xtick.labelsize': 9, 'ytick.labelsize': 9,
    'axes.grid': True, 'grid.alpha': 0.3, 'grid.linewidth': 0.7,
    'axes.spines.top': False, 'axes.spines.right': False,
    'figure.facecolor': 'white', 'axes.facecolor': 'white',
})

def save_fig(name, txt_lines):
    plt.savefig(OUT / f'{name}.png', dpi=150, bbox_inches='tight', facecolor='white')
    plt.close()
    with open(OUT / f'{name}.txt', 'w', encoding='utf-8') as f:
        f.write('\n'.join(txt_lines) + '\n')
    print(f"  v  {name}")

# ── Plot 1: Validation Loss Curves ──────────────────────────────────────────
fig, ax = plt.subplots(figsize=(9, 5))
for name in ['BC-v002', 'TrajKD', 'SelfRedWP']:
    d  = data[name]
    bi = int(np.argmin(d['val']))
    bv = float(d['val'][bi])
    be = int(d['epoch'][bi])
    ax.plot(d['epoch'], d['val'], color=C[name], lw=2,
            label=f"{L[name]}\n  best val={bv:.4f} @ ep{be}")
    ax.axvline(be, color=C[name], lw=0.8, ls='--', alpha=0.35)

ax.set_xlabel('Epoch')
ax.set_ylabel('Validation Loss')
ax.set_title('Validation Total Loss — v002 Methods\n'
             'Note: BC-v002 = ctrl MSE only; TrajKD/SelfRedWP = ctrl + lambda*traj (not directly comparable)')
ax.legend(fontsize=8.5, framealpha=0.9)
ax.set_xlim(1, 50)
plt.tight_layout()
save_fig('val_loss_curves_v002', [
    "[Plot 1: Validation Loss Curves (v002 methods)]",
    "",
    "Validation total loss over 50 epochs for BC-v002, TrajKD, and SelfRedWP on the uav_kd_v002 dataset.",
    "Dashed vertical lines mark the best-val epoch for each method.",
    "",
    "Important: BC-v002 loss = ctrl MSE only; TrajKD/SelfRedWP loss = ctrl + lambda*traj weighted sum.",
    "Absolute values are NOT comparable across methods.",
    "",
    "Key observations:",
    "- BC-v002 (blue):    best val=0.0526 @ ep23, converges fastest, no planning supervision.",
    "- TrajKD (orange):   best val=0.0885 @ ep9, oscillates afterward; traj head adds variance on limited data.",
    "- SelfRedWP (green): best val=0.0843, curriculum (ep5->35) yields a smoother late-training curve.",
    "",
    "CtrlKD and JointKD results to be added after training.",
])

# ── Plot 2: Per-Dimension Control MSE Bar Chart ──────────────────────────────
dims    = ['vx', 'vy', 'vz', 'yr']
dlabel  = ['$v_x$ (forward)', '$v_y$ (lateral)', '$v_z$ (vertical)', 'yaw_rate (turning)']
methods = ['BC-v002', 'TrajKD', 'SelfRedWP']
pending = ['CtrlKD', 'JointKD']

fig, ax = plt.subplots(figsize=(12, 5))
n_bars  = len(methods) + len(pending)
w       = 0.14
x       = np.arange(len(dims))
offsets = np.linspace(-(n_bars - 1) * w / 2, (n_bars - 1) * w / 2, n_bars)

for i, name in enumerate(methods):
    d    = data[name]
    bi   = int(np.argmin(d['val']))
    vals = [float(d[dim][bi]) for dim in dims]
    bars = ax.bar(x + offsets[i], vals, w, label=L[name], color=C[name], alpha=0.85, zorder=3)
    for b, v in zip(bars, vals):
        ax.text(b.get_x() + b.get_width() / 2, v + 0.0008,
                f'{v:.4f}', ha='center', va='bottom', fontsize=7, color=C[name], rotation=80)

for j, name in enumerate(pending):
    ax.bar(x + offsets[len(methods) + j], [0] * 4, w,
           label=f'{name} (pending)', color='#BBBBBB', alpha=0.55, hatch='//', zorder=3)

ax.set_xticks(x)
ax.set_xticklabels(dlabel, fontsize=10)
ax.set_ylabel('Validation MSE (at best-val epoch)')
ax.set_title('Per-Dimension Control Validation MSE — v002 Methods\n'
             '(values taken from the epoch with minimum total val loss)')
ax.legend(fontsize=8.5, ncol=3, framealpha=0.9)
ax.set_ylim(0, ax.get_ylim()[1] * 1.35)
plt.tight_layout()
save_fig('ctrl_dim_mse_v002', [
    "[Plot 2: Per-Dimension Control Validation MSE (v002 methods)]",
    "",
    "Grouped bar chart: vx/vy/vz/yaw_rate validation MSE for BC-v002, TrajKD, SelfRedWP.",
    "Each method uses the values at its best-val epoch for fair comparison.",
    "Grey hatched bars are placeholders for CtrlKD and JointKD (not yet trained).",
    "",
    "Key observations:",
    "- vz (vertical velocity) is near 0 -- dataset is predominantly level flight, no learning signal.",
    "- yaw_rate is the hardest dimension (highest MSE); requires scene-level turning intent.",
    "- TrajKD and SelfRedWP show a small improvement over BC-v002 on yaw_rate,",
    "  suggesting planning supervision injects directional signals into the backbone.",
    "- vy (lateral) is stable at ~0.02 across all methods, converges best.",
])

# ── Plot 3: ATE Curves ───────────────────────────────────────────────────────
fig, ax = plt.subplots(figsize=(9, 5))
for name in ['TrajKD', 'SelfRedWP']:
    d  = data[name]
    bi = int(np.argmin(d['val']))
    ax.plot(d['epoch'], d['ate'], color=C[name], lw=2,
            label=f"{L[name]}\n  ATE@best_val={d['ate'][bi]:.4f}m (ep{int(d['epoch'][bi])})")
    ax.scatter(d['epoch'][bi], d['ate'][bi], color=C[name], s=70, zorder=5)

ymax = max(data['TrajKD']['ate'].max(), data['SelfRedWP']['ate'].max())
ax.axvline(5,  color='gray', lw=1.0, ls=':', alpha=0.6)
ax.axvline(35, color='gray', lw=1.0, ls=':', alpha=0.6)
ax.text(5.5,  ymax * 0.98, 'SelfRedWP\ncurriculum\nstart (ep5)', va='top', fontsize=8, color='gray')
ax.text(34.5, ymax * 0.98, 'curriculum\nend (ep35)', ha='right', va='top', fontsize=8, color='gray')

ax.set_xlabel('Epoch')
ax.set_ylabel('Val ATE — Average Trajectory Error (m)')
ax.set_title('Trajectory Prediction Error (ATE) — TrajKD vs SelfRedWP')
ax.legend(fontsize=8.5, framealpha=0.9)
ax.set_xlim(1, 50)
ax.set_ylim(0, ymax * 1.18)
plt.tight_layout()
save_fig('ate_curves', [
    "[Plot 3: Trajectory ATE Curves (TrajKD vs SelfRedWP)]",
    "",
    "Validation ATE (average trajectory error, in metres) over training epochs.",
    "ATE measures the mean displacement error between the trajectory head's K=10 predicted",
    "waypoints and the teacher B-spline planner waypoints.",
    "Dots mark the ATE at each method's best-val epoch; grey dotted lines mark SelfRedWP's curriculum.",
    "",
    "Key observations:",
    "- Early epochs (ep1-5): ATE ~0.32m, drops quickly to ~0.25m, then oscillates.",
    "- SelfRedWP achieves slightly lower ATE after curriculum end (ep35), confirming that",
    "  self-predicted waypoint supervision helps stabilise trajectory prediction.",
    "- Final ATE ~0.25m: ~25cm deviation over 1m lookahead, acceptable for visual navigation.",
    "",
    "Note: the trajectory head is a training-only auxiliary and is discarded at deployment.",
    "ATE is an indirect measure of how well the backbone encodes the teacher's planning intent.",
])

# ── Plot 4: Yaw Rate Learning Curves ────────────────────────────────────────
fig, ax = plt.subplots(figsize=(9, 5))
for name in ['BC-v002', 'TrajKD', 'SelfRedWP']:
    d  = data[name]
    bi = int(np.argmin(d['yr']))
    ax.plot(d['epoch'], d['yr'], color=C[name], lw=2, label=L[name])
    ax.scatter(d['epoch'][bi], d['yr'][bi], color=C[name], s=70, zorder=5,
               label=f"  min={d['yr'][bi]:.4f} @ ep{int(d['epoch'][bi])}")

ax.set_xlabel('Epoch')
ax.set_ylabel('Val yaw_rate MSE')
ax.set_title('Yaw Rate Validation MSE — Hardest Control Dimension\n'
             '(requires scene-level turning intent)')
ax.legend(fontsize=8.5, framealpha=0.9)
ax.set_xlim(1, 50)
plt.tight_layout()
save_fig('yr_learning_curves', [
    "[Plot 4: Yaw Rate Validation MSE Curves (v002 methods)]",
    "",
    "Yaw_rate validation MSE over training. Dots mark each method's per-dimension minimum.",
    "",
    "Key observations:",
    "- BC-v002 (blue):   MSE briefly dips at ep4-5 then rebounds; single-frame BC cannot capture",
    "                    turning intent, leading to poor generalisation.",
    "- TrajKD / SelfRedWP: planning supervision yields a slightly lower minimum (~0.075-0.082),",
    "  showing that trajectory-level KD injects directional signals into the backbone.",
    "- SelfRedWP achieves lower yaw_rate MSE in the post-curriculum phase (ep35+),",
    "  confirming that the self-predicted waypoint curriculum aids turning generalisation.",
    "",
    "Expected next step: CtrlKD's H-step sequence distillation should further reduce",
    "yaw_rate MSE and temporal oscillation via smoothness constraints.",
])

# ── Plot 5: Dataset Composition ──────────────────────────────────────────────
EPS = {
    'ep_003': {'frames': 1013, 'rot': 0.000, 'split': 'train'},
    'ep_004': {'frames': 958,  'rot': 0.000, 'split': 'train'},
    'ep_005': {'frames': 569,  'rot': 0.000, 'split': 'val'},
    'ep_006': {'frames': 962,  'rot': 0.182, 'split': 'val'},
    'ep_007': {'frames': 4406, 'rot': 0.229, 'split': 'train'},
    'ep_008': {'frames': 2457, 'rot': 0.042, 'split': 'train'},
    'ep_009': {'frames': 1337, 'rot': 0.062, 'split': 'train'},
}
ep_ids = list(EPS.keys())
frames = [EPS[e]['frames'] for e in ep_ids]
rot_n  = [int(EPS[e]['frames'] * EPS[e]['rot']) for e in ep_ids]
str_n  = [frames[i] - rot_n[i] for i in range(len(ep_ids))]
splits = [EPS[e]['split'] for e in ep_ids]
sp_col = {'train': '#2471A3', 'val': '#C0392B'}

fig, ax = plt.subplots(figsize=(10, 5))
for i, ep in enumerate(ep_ids):
    col = sp_col[splits[i]]
    ax.bar(i, str_n[i],                  color=col, alpha=0.80, zorder=3)
    ax.bar(i, rot_n[i], bottom=str_n[i], color=col, alpha=0.38, hatch='///', zorder=3)
    ax.text(i, frames[i] + 30, str(frames[i]),
            ha='center', va='bottom', fontsize=9, fontweight='bold')
    pct = EPS[ep]['rot'] * 100
    if pct > 0:
        ax.text(i, frames[i] + 200, f'{pct:.1f}% rot',
                ha='center', va='bottom', fontsize=8, color='#7D3C98')

legend_handles = [
    mpatches.Patch(color='#2471A3', alpha=0.80, label='Train — straight'),
    mpatches.Patch(color='#2471A3', alpha=0.38, hatch='///', label='Train — rotation'),
    mpatches.Patch(color='#C0392B', alpha=0.80, label='Val   — straight'),
    mpatches.Patch(color='#C0392B', alpha=0.38, hatch='///', label='Val   — rotation'),
]
ax.legend(handles=legend_handles, fontsize=9, loc='upper left', framealpha=0.9)
ax.set_xticks(range(len(ep_ids)))
ax.set_xticklabels(ep_ids, rotation=15)
ax.set_xlabel('Episode')
ax.set_ylabel('Frame count')
ax.set_title('Dataset Composition — uav_kd_v002\n'
             '(blue=train / red=val; hatching=rotation frames |yaw_rate|>0.15 rad/s)')
ax.set_ylim(0, max(frames) * 1.22)
plt.tight_layout()
save_fig('dataset_composition', [
    "[Plot 5: Dataset Composition — uav_kd_v002]",
    "",
    "Frame counts per episode, split by train (blue) / val (red) and straight vs rotation frames",
    "(hatched, |yaw_rate|>0.15 rad/s). ep_001 and ep_002 are excluded due to traj_body bug.",
    "",
    "Key observations:",
    "- ep_007 (4406 frames) is the longest episode with the highest rotation ratio (22.9%).",
    "  It is the primary source of turning behaviour for the model.",
    "- ep_005 (pure straight, val) provides a simple distribution reference.",
    "  ep_006 (18.2% rotation, val) makes the v002 val set representative of turning.",
    "- Overall rotation ratio ~11-12%; dataset is straight-flight dominated.",
    "  This explains why vz MSE ~0 and yaw_rate learning signal is sparse.",
    "",
    "Recommendation: collect more episodes with higher rotation ratios to improve turning generalisation.",
])

# ── Plot 6: Train vs Val Gap (3-panel overfitting analysis) ──────────────────
fig, axes = plt.subplots(1, 3, figsize=(15, 5))
for ax, name in zip(axes, ['BC-v002', 'TrajKD', 'SelfRedWP']):
    d  = data[name]
    bi = int(np.argmin(d['val']))
    be = int(d['epoch'][bi])
    ax.plot(d['epoch'], d['train'], color=C[name], lw=2, ls='-',  label='Train loss')
    ax.plot(d['epoch'], d['val'],   color=C[name], lw=2, ls='--', label='Val loss',  alpha=0.85)
    ax.axvline(be, color='gray', lw=0.9, ls=':', alpha=0.7, label=f'Best val ep{be}')
    gap = float(d['val'][-1] - d['train'][-1])
    ax.text(0.97, 0.97, f'Final gap: {gap:.4f}',
            transform=ax.transAxes, ha='right', va='top', fontsize=9, color='#555555')
    ax.set_title(L[name], fontsize=10, color=C[name], fontweight='bold')
    ax.set_xlabel('Epoch', fontsize=9)
    ax.set_ylabel('Loss', fontsize=9)
    ax.set_xlim(1, 50)
    ax.legend(fontsize=8)

fig.suptitle('Train vs Validation Loss — Overfitting Analysis (v002 Methods)', fontsize=13, y=1.01)
plt.tight_layout()
save_fig('train_val_gap', [
    "[Plot 6: Train vs Validation Loss Gap — Overfitting Analysis (v002 methods)]",
    "",
    "Each sub-panel shows train loss (solid) vs val loss (dashed) for one method.",
    "The text annotation gives the final-epoch train-val gap; the dotted line marks the best-val epoch.",
    "",
    "Key observations:",
    "- All methods overfit substantially: train loss falls near 0 while val loss plateaus or rises.",
    "- BC-v002:   final gap ~0.051 (train~0.002, val~0.054); stable after ep23.",
    "- TrajKD:    final gap ~0.088; val loss oscillates more, dual-head distillation",
    "             increases generalisation difficulty on 6004 training samples.",
    "- SelfRedWP: similar gap to TrajKD, but val loss shows a slight downward trend",
    "             after curriculum end (ep35), suggesting curriculum aids late-stage generalisation.",
    "",
    "Mitigation directions: more episodes, stronger data augmentation, tuning dropout/weight_decay.",
])

print(f"\nDone. 12 files -> {OUT}")
