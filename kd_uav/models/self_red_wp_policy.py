#!/usr/bin/env python3
"""
self_red_wp_policy.py

SelfRedWP: self-guided red-waypoint policy.

Two-pass, single shared backbone:
  Pass 1 (predict_traj): raw image -> backbone -> traj_head -> K waypoints
  Pass 2 (predict_ctrl):  annotated image (red dot drawn at the selected
                          waypoint) -> backbone (same weights) -> ctrl_head

Unlike TrajKDPolicy (Option B), the trajectory head's output is not a pure
training-time auxiliary loss discarded at deployment. Instead, a waypoint
selected from the predicted trajectory is rendered onto the image as a red
dot (kd_uav/utils/waypoint_projection.py), and the control head conditions
on that annotated image directly. At deployment there is no ground truth,
so the dot always comes from the model's own prediction — training uses a
curriculum that ramps from ground-truth waypoints to self-predicted
(detached) waypoints so the control head adapts to its own prediction
distribution before deployment (see kd_uav/train/train_self_red_wp.py).

The drawing step happens in image/numpy space between the two passes, so it
is NOT part of this module's forward graph — this class only exposes the
two head-specific passes; the training loop / deployment node orchestrates
predict_traj -> select waypoint -> draw -> predict_ctrl.
"""

import torch
import torch.nn as nn


class SelfRedWPPolicy(nn.Module):
    def __init__(self, backbone, traj_head, ctrl_head):
        super().__init__()
        self.backbone  = backbone
        self.traj_head = traj_head
        self.ctrl_head = ctrl_head

    def predict_traj(self, image):
        """raw image (B,3,H,W) -> traj_pred (B,K,3)"""
        feat = self.backbone(image)
        return self.traj_head(feat)

    def predict_ctrl(self, annotated_image):
        """red-dot-annotated image (B,3,H,W) -> ctrl_pred (B,4), same backbone weights"""
        feat = self.backbone(annotated_image)
        return self.ctrl_head(feat)

    @staticmethod
    def build(pretrained=True, freeze_backbone=False,
              hidden_dim=256, dropout=0.3, K=10):
        from kd_uav.models.student_backbone import StudentBackbone
        from kd_uav.models.control_head import ControlHead
        from kd_uav.models.trajectory_head import TrajectoryHead
        backbone  = StudentBackbone(pretrained=pretrained, freeze=freeze_backbone)
        ctrl_head = ControlHead(feat_dim=backbone.feat_dim, hidden_dim=hidden_dim, dropout=dropout)
        traj_head = TrajectoryHead(feat_dim=backbone.feat_dim, hidden_dim=hidden_dim, K=K, dropout=dropout)
        return SelfRedWPPolicy(backbone, traj_head, ctrl_head)
