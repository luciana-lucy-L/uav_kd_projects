#!/usr/bin/env python3
"""
bc_loss.py

Behaviour Cloning loss: MSE between predicted and teacher control commands.

Label: ctrl_body = [vx_b, vy_b, vz_b, yaw_rate]
"""

import torch
import torch.nn as nn


class BCLoss(nn.Module):
    """
    Per-dimension weighted MSE loss.

    Args:
        weights : (4,) tensor of per-output weights.
                  Default [1, 1, 1, 1] (uniform).
                  Increase yaw_rate weight if yaw supervision matters more.
    """

    def __init__(self, weights=None):
        super().__init__()
        if weights is None:
            weights = [1.0, 1.0, 1.0, 1.0]
        self.register_buffer('weights', torch.tensor(weights, dtype=torch.float32))

    def forward(self, pred, target):
        """
        Args:
            pred   : (B, 4)
            target : (B, 4)
        Returns:
            scalar loss
        """
        sq = (pred - target) ** 2          # (B, 4)
        weighted = sq * self.weights       # (B, 4)
        return weighted.mean()

    def per_dim_mse(self, pred, target):
        """Return (4,) per-dimension MSE — for logging."""
        with torch.no_grad():
            return ((pred - target) ** 2).mean(dim=0)
