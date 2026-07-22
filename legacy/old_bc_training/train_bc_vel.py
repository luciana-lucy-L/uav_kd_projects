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


def _ensure_np_str_array(arr):
    """
    action_source 可能是 dtype=object / bytes / str
    统一转成 numpy 的 str 数组，方便比较。
    """
    arr = np.asarray(arr)
    if arr.dtype.kind in ("S", "a"):  # bytes
        return arr.astype(str)
    if arr.dtype == object:
        return arr.astype(str)
    return arr


class BCDatasetVel(Dataset):
    """
    训练目标：学习专家的 cmd_vel (v_x, v_y, v_z)
    训练空间：使用标准化后的 state / action
    """
    def __init__(self, files, only_source="cmd_vel", save_stats_path=None):
        raw_states = []
        raw_actions = []

        for f in files:
            data = np.load(f, allow_pickle=True)

            pos = data["position"]        # (N, 3)
            ori = data["orientation"]     # (N, 4)
            lin = data["lin_vel"]         # (N, 3)
            ang = data["ang_vel"]         # (N, 3)

            # ✅ 正确：动作来自 recorder 保存的 action（专家命令）
            act = data["action"]          # (N, 3)

            # ✅ 只训练 cmd_vel 的样本，避免混入 cmd_acc
            src = _ensure_np_str_array(data["action_source"])  # (N,)
            mask = (src == only_source)

            if mask.sum() == 0:
                continue

            state = np.concatenate([pos, ori, lin, ang], axis=-1)  # (N, 13)

            raw_states.append(state[mask])
            raw_actions.append(act[mask])

        assert len(raw_states) > 0, f"No samples found for action_source == '{only_source}'."

        raw_states = np.concatenate(raw_states, axis=0).astype(np.float32)
        raw_actions = np.concatenate(raw_actions, axis=0).astype(np.float32)

        print(f"[Dataset] samples: {raw_states.shape[0]}")
        print(f"[Dataset] state dim: {raw_states.shape[1]}, action dim: {raw_actions.shape[1]}")

        # ✅ 标准化（非常推荐）
        state_mean = raw_states.mean(axis=0)
        state_std = raw_states.std(axis=0) + 1e-6

        action_mean = raw_actions.mean(axis=0)
        action_std = raw_actions.std(axis=0) + 1e-6

        states = (raw_states - state_mean) / state_std
        actions = (raw_actions - action_mean) / action_std

        # 保存统计量，部署时也要用（同样的归一化/反归一化）
        if save_stats_path is not None:
            os.makedirs(os.path.dirname(save_stats_path), exist_ok=True)
            np.savez(
                save_stats_path,
                state_mean=state_mean.astype(np.float32),
                state_std=state_std.astype(np.float32),
                action_mean=action_mean.astype(np.float32),
                action_std=action_std.astype(np.float32),
            )
            print(f"[Dataset] Saved stats to: {save_stats_path}")

        self.states = torch.from_numpy(states).float()
        self.actions = torch.from_numpy(actions).float()

    def __len__(self):
        return len(self.states)

    def __getitem__(self, idx):
        return self.states[idx], self.actions[idx]


class BCPolicy(nn.Module):
    def __init__(self, state_dim=13, action_dim=3, hidden_dim=64):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(state_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, action_dim),
             nn.Tanh(),   # ✅ 核心
        )

    def forward(self, x):
        return self.net(x)


def train():
    files = sorted(glob.glob(os.path.join(DATA_DIR, "expert_*.npz")))
    assert files, f"No expert data found in: {DATA_DIR}"

    os.makedirs(MODEL_DIR, exist_ok=True)

    stats_path = os.path.join(MODEL_DIR, "bc_vel_stats.npz")
    dataset = BCDatasetVel(files, only_source="cmd_vel", save_stats_path=stats_path)
    loader = DataLoader(dataset, batch_size=64, shuffle=True, drop_last=False)

    device = "cuda" if torch.cuda.is_available() else "cpu"
    print("[Train] device:", device)

    model = BCPolicy().to(device)
    optim = torch.optim.Adam(model.parameters(), lr=1e-3)
    loss_fn = nn.MSELoss()

    for epoch in range(1, 51):
        total_loss = 0.0
        for s, a in loader:
            s, a = s.to(device), a.to(device)
            pred = model(s)
            loss = loss_fn(pred, a)

            optim.zero_grad()
            loss.backward()
            optim.step()

            total_loss += loss.item() * s.size(0)

        print(f"[Epoch {epoch:03d}] loss = {total_loss / len(dataset):.6f}")

    model_path = os.path.join(MODEL_DIR, "bc_policy_vel.pt")
    torch.save(model.state_dict(), model_path)
    print("[Train] Saved model:", model_path)
    print("[Train] Saved stats:", stats_path)


if __name__ == "__main__":
    train()
