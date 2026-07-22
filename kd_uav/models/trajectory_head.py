#!/usr/bin/env python3
"""
trajectory_head.py

MLP head that maps a visual feature vector to K future body-frame waypoints.
Auxiliary head for TrajKD (Option B) / JointKD: used only during training to
force the backbone to encode planning intent; discarded at deployment.

Input : (B, feat_dim) float32
Output: (B, K, 3)     float32  [dx, dy, dz] per waypoint
"""

import torch
import torch.nn as nn


class TrajectoryHead(nn.Module):
    """
    Two-layer MLP trajectory head.

    Args:
        feat_dim   : input feature dimension (must match backbone output)
        hidden_dim : hidden layer width (default 256)
        K          : number of future waypoints (default 10)
        dropout    : dropout probability (default 0.3)
    """

    def __init__(self, feat_dim=1280, hidden_dim=256, K=10, dropout=0.3):
        super().__init__()
        self.K = K
        self.net = nn.Sequential(
            nn.Linear(feat_dim, hidden_dim),
            nn.ReLU(inplace=True),
            nn.Dropout(dropout),
            nn.Linear(hidden_dim, K * 3),
        )

    def forward(self, feat):
        out = self.net(feat)             # (B, K*3)
        return out.view(-1, self.K, 3)   # (B, K, 3)


class TrajKDPolicy(nn.Module):
    """
    TrajKD (Option B) policy: shared backbone + trajectory head (auxiliary,
    training only) + control head (main, deployed).

    Input : (B, 3, H, W)  image
    Output: ctrl (B, 4), traj (B, K, 3)
    """

    def __init__(self, backbone, traj_head, ctrl_head):
        super().__init__()
        self.backbone  = backbone
        self.traj_head = traj_head
        self.ctrl_head = ctrl_head

    def forward(self, image):
        feat = self.backbone(image)
        ctrl = self.ctrl_head(feat)
        traj = self.traj_head(feat)
        return ctrl, traj

    def forward_ctrl_only(self, image):
        """Deployment path: backbone + control head only (trajectory head unused)."""
        feat = self.backbone(image)
        return self.ctrl_head(feat)

    @staticmethod
    def build(pretrained=True, freeze_backbone=False,
              hidden_dim=256, dropout=0.3, K=10):
        from kd_uav.models.student_backbone import StudentBackbone
        from kd_uav.models.control_head import ControlHead
        backbone  = StudentBackbone(pretrained=pretrained, freeze=freeze_backbone)
        ctrl_head = ControlHead(feat_dim=backbone.feat_dim, hidden_dim=hidden_dim, dropout=dropout)
        traj_head = TrajectoryHead(feat_dim=backbone.feat_dim, hidden_dim=hidden_dim, K=K, dropout=dropout)
        return TrajKDPolicy(backbone, traj_head, ctrl_head)
