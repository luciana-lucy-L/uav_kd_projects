#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import glob
import numpy as np

import torch
import torch.nn as nn
from torch.utils.data import Dataset, DataLoader


DATA_DIR = os.path.join(os.path.dirname(__file__), "data")
MODEL_DIR = os.path.join(os.path.dirname(__file__), "models")

TEACHER_MODEL_PATH = os.path.join(MODEL_DIR, "bc_policy_vel.pt")
STATS_PATH = os.path.join(MODEL_DIR, "bc_vel_stats.npz")
TEACHER_ACTIONS_PATH = os.path.join(MODEL_DIR, "teacher_actions_vel.npz")


def _ensure_np_str_array(arr):
    arr = np.asarray(arr)
    if arr.dtype.kind in ("S", "a"):
        return arr.astype(str)
    if arr.dtype == object:
        return arr.astype(str)
    return arr


class BCDatasetVel(Dataset):
    def __init__(
        self,
        files,
        only_source="cmd_vel",
        stats_path=None,
        teacher_actions_path=None,
    ):
        raw_states = []
        raw_actions = []

        for f in files:
            data = np.load(f, allow_pickle=True)

            pos = data["position"]
            ori = data["orientation"]
            lin = data["lin_vel"]
            ang = data["ang_vel"]

            act = data["action"]
            src = _ensure_np_str_array(data["action_source"])
            mask = (src == only_source)

            if mask.sum() == 0:
                continue

            state = np.concatenate([pos, ori, lin, ang], axis=-1)
            raw_states.append(state[mask])
            raw_actions.append(act[mask])

        assert raw_states, "No valid samples found."

        raw_states = np.concatenate(raw_states, axis=0).astype(np.float32)
        raw_actions = np.concatenate(raw_actions, axis=0).astype(np.float32)

        # === load normalization stats (must match teacher) ===
        stats = np.load(stats_path)
        self.state_mean = stats["state_mean"]
        self.state_std = stats["state_std"]
        self.action_mean = stats["action_mean"]
        self.action_std = stats["action_std"]

        states = (raw_states - self.state_mean) / self.state_std
        actions = (raw_actions - self.action_mean) / self.action_std

        self.states = torch.from_numpy(states).float()
        self.actions = torch.from_numpy(actions).float()

        # === optional teacher actions ===
        self.teacher_actions = None
        if teacher_actions_path is not None:
            ta = np.load(teacher_actions_path)["teacher_actions"]
            assert len(ta) == len(self.states), "Teacher actions size mismatch"
            self.teacher_actions = torch.from_numpy(ta).float()

        print(f"[Dataset] samples: {len(self.states)}")
        print(f"[Dataset] KD enabled: {self.teacher_actions is not None}")

    def __len__(self):
        return len(self.states)

    def __getitem__(self, idx):
        if self.teacher_actions is None:
            return self.states[idx], self.actions[idx]
        else:
            return (
                self.states[idx],
                self.actions[idx],
                self.teacher_actions[idx],
            )


class BCPolicy(nn.Module):
    def __init__(self, state_dim=13, action_dim=3, hidden_dim=64):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(state_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, action_dim),
            nn.Tanh(),
        )

    def forward(self, x):
        return self.net(x)


# ------------------------------------------------------------
# Step 1: Precompute teacher actions (normalized)
# ------------------------------------------------------------
def precompute_teacher_actions(device):
    if os.path.exists(TEACHER_ACTIONS_PATH):
        print("[KD] Found cached teacher actions, skip precompute.")
        return

    print("[KD] Precomputing teacher actions...")

    files = sorted(glob.glob(os.path.join(DATA_DIR, "expert_*.npz")))
    dataset = BCDatasetVel(
        files,
        stats_path=STATS_PATH,
        teacher_actions_path=None,
    )
    loader = DataLoader(dataset, batch_size=256, shuffle=False)

    teacher = BCPolicy(hidden_dim=64).to(device)
    teacher.load_state_dict(torch.load(TEACHER_MODEL_PATH, map_location=device))
    teacher.eval()
    for p in teacher.parameters():
        p.requires_grad_(False)

    teacher_actions = []

    with torch.no_grad():
        for s, _ in loader:
            s = s.to(device)
            a_t = teacher(s)
            teacher_actions.append(a_t.cpu().numpy())

    teacher_actions = np.concatenate(teacher_actions, axis=0)

    np.savez(
        TEACHER_ACTIONS_PATH,
        teacher_actions=teacher_actions.astype(np.float32),
    )

    print("[KD] Saved teacher actions:", TEACHER_ACTIONS_PATH)


# ------------------------------------------------------------
# Step 2: Train student with KD
# ------------------------------------------------------------
def train_kd():
    device = "cuda" if torch.cuda.is_available() else "cpu"
    print("[Train] device:", device)

    precompute_teacher_actions(device)

    files = sorted(glob.glob(os.path.join(DATA_DIR, "expert_*.npz")))

    dataset = BCDatasetVel(
        files,
        stats_path=STATS_PATH,
        teacher_actions_path=TEACHER_ACTIONS_PATH,
    )
    loader = DataLoader(dataset, batch_size=64, shuffle=True)

    # === student (smaller model) ===
    student = BCPolicy(hidden_dim=32).to(device)
    optim = torch.optim.Adam(student.parameters(), lr=1e-3)
    loss_fn = nn.MSELoss()

    alpha = 0.2  # KD weight

    for epoch in range(1, 51):
        total_loss = 0.0

        for s, a_gt, a_t in loader:
            s = s.to(device)
            a_gt = a_gt.to(device)
            a_t = a_t.to(device)

            pred = student(s)

            loss_bc = loss_fn(pred, a_gt)
            loss_kd = loss_fn(pred, a_t)
            loss = (1 - alpha) * loss_bc + alpha * loss_kd

            optim.zero_grad()
            loss.backward()
            optim.step()

            total_loss += loss.item() * s.size(0)

        print(
            f"[Epoch {epoch:03d}] "
            f"loss={total_loss / len(dataset):.6f} "
            f"(alpha={alpha})"
        )

    model_path = os.path.join(MODEL_DIR, "bc_policy_vel_kd.pt")
    torch.save(student.state_dict(), model_path)
    print("[Train] Saved KD student:", model_path)


if __name__ == "__main__":
    os.makedirs(MODEL_DIR, exist_ok=True)
    train_kd()
