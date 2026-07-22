#!/usr/bin/env python3
"""
control_head.py

MLP head that maps a visual feature vector to a UAV control command.

Input : (B, feat_dim) float32
Output: (B, 4)        float32  [vx_b, vy_b, vz_b, yaw_rate]
"""

import torch
import torch.nn as nn


class ControlHead(nn.Module):
    """
    Two-layer MLP control head.

    Args:
        feat_dim    : input feature dimension (must match backbone output)
        hidden_dim  : hidden layer width (default 256)
        output_dim  : control output dimension (default 4)
        dropout     : dropout probability (default 0.3)
    """

    def __init__(self, feat_dim=1280, hidden_dim=256, output_dim=4, dropout=0.3):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(feat_dim, hidden_dim),
            nn.ReLU(inplace=True),
            nn.Dropout(dropout),
            nn.Linear(hidden_dim, output_dim),
        )

    def forward(self, feat):
        return self.net(feat)   # (B, 4)


class BCPolicy(nn.Module):
    """
    Full BC policy: backbone + control head.

    Input : (B, 3, H, W)  image
    Output: (B, 4)         [vx_b, vy_b, vz_b, yaw_rate]
    """

    def __init__(self, backbone, head):
        super().__init__()
        self.backbone = backbone
        self.head     = head

    def forward(self, image):
        feat = self.backbone(image)
        return self.head(feat)

    @staticmethod
    def build(pretrained=True, freeze_backbone=False,
              hidden_dim=256, dropout=0.3):
        from kd_uav.models.student_backbone import StudentBackbone
        backbone = StudentBackbone(pretrained=pretrained, freeze=freeze_backbone)
        head     = ControlHead(feat_dim=backbone.feat_dim,
                               hidden_dim=hidden_dim,
                               dropout=dropout)
        return BCPolicy(backbone, head)
