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


class BCDataset(Dataset):
    def __init__(self, files):
        states = []
        actions = []

        for f in files:
            data = np.load(f)
            pos = data["position"]      # (N, 3)
            ori = data["orientation"]   # (N, 4)
            lin = data["lin_vel"]       # (N, 3)
            ang = data["ang_vel"]       # (N, 3)
            act = data["action"]        # (N, 3)

            # 拼成 state: [pos, ori, lin, ang] → (N, 13)
            st = np.concatenate([pos, ori, lin, ang], axis=-1)

            states.append(st)
            actions.append(act)

        states = np.concatenate(states, axis=0)
        actions = np.concatenate(actions, axis=0)

        print(f"[Dataset] Loaded {states.shape[0]} samples from {len(files)} file(s)")
        print(f"[Dataset] state shape: {states.shape}, action shape: {actions.shape}")

        self.states = torch.from_numpy(states).float()
        self.actions = torch.from_numpy(actions).float()

    def __len__(self):
        return self.states.shape[0]

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


def load_data(test_ratio=0.1, batch_size=64):
    files = sorted(glob.glob(os.path.join(DATA_DIR, "expert_*.npz")))
    if not files:
        raise RuntimeError(f"No expert_*.npz found in {DATA_DIR}")

    dataset = BCDataset(files)

    n = len(dataset)
    idx = np.random.permutation(n)
    n_test = max(1, int(n * test_ratio))

    test_idx = idx[:n_test]
    train_idx = idx[n_test:]

    train_subset = torch.utils.data.Subset(dataset, train_idx.tolist())
    test_subset = torch.utils.data.Subset(dataset, test_idx.tolist())

    train_loader = DataLoader(train_subset, batch_size=batch_size, shuffle=True)
    test_loader = DataLoader(test_subset, batch_size=batch_size, shuffle=False)

    print(f"[Data] train size: {len(train_subset)}, test size: {len(test_subset)}")

    return train_loader, test_loader


def train_bc(num_epochs=50, lr=1e-3, device=None):
    if device is None:
        device = "cuda" if torch.cuda.is_available() else "cpu"
    print(f"[Train] Using device: {device}")

    train_loader, test_loader = load_data()

    model = BCPolicy().to(device)
    optimizer = torch.optim.Adam(model.parameters(), lr=lr)
    criterion = nn.MSELoss()

    for epoch in range(1, num_epochs + 1):
        # ----- train -----
        model.train()
        total_loss = 0.0
        for states, actions in train_loader:
            states = states.to(device)
            actions = actions.to(device)

            pred = model(states)
            loss = criterion(pred, actions)

            optimizer.zero_grad()
            loss.backward()
            optimizer.step()

            total_loss += loss.item() * states.size(0)

        avg_train_loss = total_loss / len(train_loader.dataset)

        # ----- eval -----
        model.eval()
        total_loss = 0.0
        with torch.no_grad():
            for states, actions in test_loader:
                states = states.to(device)
                actions = actions.to(device)
                pred = model(states)
                loss = criterion(pred, actions)
                total_loss += loss.item() * states.size(0)

        avg_test_loss = total_loss / len(test_loader.dataset)
        print(f"[Epoch {epoch:03d}] train_loss = {avg_train_loss:.6f}, "
              f"test_loss = {avg_test_loss:.6f}")

    # ----- save model -----
    os.makedirs(MODEL_DIR, exist_ok=True)
    model_path = os.path.join(MODEL_DIR, "bc_policy.pt")
    torch.save(model.state_dict(), model_path)
    print(f"[Train] Saved model to {model_path}")


if __name__ == "__main__":
    train_bc()
