#!/usr/bin/env python3
"""
trajectory_kd_loss.py

Trajectory distillation loss: MSE between predicted and teacher body-frame
future waypoints.

Label: traj_body_xyz = [[dx0,dy0,dz0], ..., [dx{K-1},dy{K-1},dz{K-1}]]
"""

import torch
import torch.nn as nn


class TrajectoryKDLoss(nn.Module):
    """
    Per-axis weighted MSE loss over K future waypoints.

    Args:
        weights : (3,) tensor of per-axis [dx, dy, dz] weights.
                  Default [1, 1, 1] (uniform).
    """

    def __init__(self, weights=None):
        super().__init__()
        if weights is None:
            weights = [1.0, 1.0, 1.0]
        self.register_buffer('weights', torch.tensor(weights, dtype=torch.float32))

    def forward(self, pred, target):
        """
        Args:
            pred   : (B, K, 3)
            target : (B, K, 3)
        Returns:
            scalar loss
        """
        sq = (pred - target) ** 2          # (B, K, 3)
        weighted = sq * self.weights       # (B, K, 3)
        return weighted.mean()

    def per_dim_mse(self, pred, target):
        """Return (3,) per-axis MSE, averaged over K waypoints and batch — for logging."""
        with torch.no_grad():
            return ((pred - target) ** 2).mean(dim=(0, 1))

    def ate(self, pred, target):
        """Average Trajectory Error: mean L2 distance per waypoint, averaged over K and batch."""
        with torch.no_grad():
            dist = torch.norm(pred - target, dim=-1)   # (B, K)
            return dist.mean()
