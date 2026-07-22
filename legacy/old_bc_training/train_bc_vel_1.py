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


# class BCDatasetVel(Dataset):
#     def __init__(self, files):
#         states = []
#         actions = []

#         for f in files:
#             data = np.load(f)

#             pos = data["position"]      # (N, 3)
#             ori = data["orientation"]   # (N, 4)
#             lin = data["lin_vel"]       # (N, 3)
#             ang = data["ang_vel"]       # (N, 3)

#             # state: 13 维
#             state = np.concatenate([pos, ori, lin, ang], axis=-1)

#             # action: 直接学“期望速度”
#             action = lin.copy()

#             states.append(state)
#             actions.append(action)

#         states = np.concatenate(states, axis=0)
#         actions = np.concatenate(actions, axis=0)

#         print(f"[Dataset] samples: {states.shape[0]}")
#         print(f"[Dataset] state dim: {states.shape[1]}, action dim: {actions.shape[1]}")

#         self.states = torch.from_numpy(states).float()
#         self.actions = torch.from_numpy(actions).float()

#     def __len__(self):
#         return len(self.states)

#     def __getitem__(self, idx):
#         return self.states[idx], self.actions[idx]
class BCDatasetVel(Dataset):
    def __init__(self, files):
        all_states = []
        all_actions = []

        total_samples = 0
        total_cmd_vel = 0
        total_cmd_acc = 0

        for f in files:
            data = np.load(f)

            pos = data["position"]      # (N, 3)
            ori = data["orientation"]   # (N, 4)
            lin = data["lin_vel"]       # (N, 3)
            ang = data["ang_vel"]       # (N, 3)
            act = data["action"]        # (N, 3)
            src = data["action_source"] # (N,)

            # state: 13 维
            state = np.concatenate([pos, ori, lin, ang], axis=-1)

            # 统计一下原始样本数
            total_samples += state.shape[0]

            # src 是一个字符串数组，比如 ["cmd_vel", "cmd_vel", "cmd_acc", ...]
            # 我们只保留来源是 "cmd_vel" 的样本
            mask_cmd_vel = (src == "cmd_vel")
            num_cmd_vel = np.sum(mask_cmd_vel)
            num_cmd_acc = state.shape[0] - num_cmd_vel

            total_cmd_vel += int(num_cmd_vel)
            total_cmd_acc += int(num_cmd_acc)

            if num_cmd_vel == 0:
                # 这个文件里没有任何 cmd_vel 样本，直接跳过也行
                continue

            state = state[mask_cmd_vel]
            act = act[mask_cmd_vel]

            all_states.append(state)
            all_actions.append(act)

            print(f"[Dataset] file {os.path.basename(f)}: total={state.shape[0]+num_cmd_acc}, "
                  f"cmd_vel={num_cmd_vel}, cmd_acc={num_cmd_acc}")

        if not all_states:
            raise RuntimeError("No cmd_vel samples found in given files. "
                               "Please check your recordings or topics.")

        states = np.concatenate(all_states, axis=0)
        actions = np.concatenate(all_actions, axis=0)

        print(f"[Dataset] total raw samples: {total_samples}")
        print(f"[Dataset] total cmd_vel samples: {total_cmd_vel}")
        print(f"[Dataset] total cmd_acc samples: {total_cmd_acc}")
        print(f"[Dataset] used samples: {states.shape[0]}")
        print(f"[Dataset] state dim: {states.shape[1]}, action dim: {actions.shape[1]}")

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
        )

    def forward(self, x):
        return self.net(x)


def train():
    files = sorted(glob.glob(os.path.join(DATA_DIR, "expert_*.npz")))
    assert files, "No expert data found."

    dataset = BCDatasetVel(files)
    loader = DataLoader(dataset, batch_size=64, shuffle=True)

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

    os.makedirs(MODEL_DIR, exist_ok=True)
    path = os.path.join(MODEL_DIR, "bc_policy_vel.pt")
    torch.save(model.state_dict(), path)
    print("[Train] Saved:", path)


if __name__ == "__main__":
    train()

