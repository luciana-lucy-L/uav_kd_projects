#!/usr/bin/env python3
"""
student_backbone.py

Lightweight CNN visual encoder for the UAV KD student policy.
Uses MobileNetV2 as the backbone (pretrained on ImageNet).

Input : (B, 3, H, W)  float32  RGB  [0, 1]
Output: (B, feat_dim) float32  visual feature vector
"""

import torch
import torch.nn as nn
import torchvision.models as models


class StudentBackbone(nn.Module):
    """
    MobileNetV2 encoder with global average pooling.

    Args:
        pretrained  : load ImageNet weights (default True)
        freeze      : freeze all backbone weights (default False)
        feat_dim    : output feature dimension (1280 for MobileNetV2)
    """

    FEAT_DIM = 1280   # MobileNetV2 last-conv output channels

    def __init__(self, pretrained=True, freeze=False):
        super().__init__()

        base = models.mobilenet_v2(
            weights=models.MobileNet_V2_Weights.IMAGENET1K_V1 if pretrained else None
        )
        # Drop the original classifier; keep features + global pool
        self.features = base.features          # (B, 1280, H/32, W/32)
        self.pool     = nn.AdaptiveAvgPool2d(1) # (B, 1280, 1, 1)

        if freeze:
            for p in self.features.parameters():
                p.requires_grad = False

    def forward(self, x):
        x = self.features(x)          # (B, 1280, h, w)
        x = self.pool(x)              # (B, 1280, 1, 1)
        x = x.flatten(1)              # (B, 1280)
        return x

    @property
    def feat_dim(self):
        return self.FEAT_DIM
