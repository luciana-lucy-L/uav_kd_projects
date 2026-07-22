# Legacy Archive

This directory archives the preliminary state-based BC implementation that was attempted before the formal KD project was established.

## Why This Exists

An initial behavior cloning model was tested prior to this project. The model used a 13-dimensional state vector (position + orientation + linear velocity + angular velocity) as input and predicted a 3-dimensional velocity command. The resulting flight behavior was **unstable and insufficient** as a formal baseline.

## What Is Archived Here

### old_bc_training/

Old training scripts and their associated data and models.

| File | Description |
|---|---|
| `train_bc.py` | State-based MLP BC: 13D state → 3D action |
| `train_bc_vel.py` | Variant: 13D state → 3D velocity (TwistStamped) |
| `train_bc_vel_1.py` | Variant with different normalization |
| `train_bc_vel_kd.py` | First KD attempt via teacher action distribution matching |
| `plot_traj3d.py` | Trajectory visualization utility |
| `record_pose_after_takeoff.py` | Pose recording utility |
| `data/expert_*.npz` | Old state-based expert demonstrations (Dec 2025) |
| `models/bc_policy*.pt` | Old trained model checkpoints |
| `models/bc_vel_stats.npz` | Normalization statistics for old state-based model |
| `models/teacher_actions_vel.npz` | Cached teacher actions from old recording session |

**Data schema of old expert_*.npz files:**
```
keys: t, position (N,3), orientation (N,4), lin_vel (N,3), ang_vel (N,3),
      action (N,3), action_source (N,)
```
No image key. These files are **not usable** for the image-based student training.

### old_bc_tools/

Old ROS package scripts from `catkin_ws/src/uav_bc_tools/`.

| File | Description |
|---|---|
| `scripts/record_expert.py` | Records CERLAB odom + cmd_vel to .npz (no camera) |
| `scripts/bc_policy_node*.py` | State-based deployment nodes (read odom, publish cmd_vel) |
| `scripts/twist_to_twiststamped_bridge.py` | Topic type bridge utility |
| `launch/record_expert.launch` | Launch file for old expert recording |
| `launch/play_bc_policy.launch` | Launch file for old BC policy deployment |

Note: `record_expert.py` is a useful design reference for the new `uav_kd_data_collector` package, but must be extended to record camera images and teacher trajectories.

### old_figures/

Figures generated during the old BC experiments.

| File | Description |
|---|---|
| `Figure_1.png` | Training loss curve from old BC run |
| `traj_3d.png` | 3D trajectory plot (old workflow) |
| `traj_3d_multi.png` | Multi-episode 3D trajectory plot |
| `frame0000.jpg` | Example camera frame captured during testing |

## Formal Baseline Definition (for reference)

The **formal BC baseline** for this project is defined as:

- **Input**: front-facing RGB camera image only
- **Output**: teacher control command `[vx, vy, vz, yaw_rate]`
- **Training signal**: CERLAB automatic flight control command
- **Loss**: `L_BC = MSE(pred_control, teacher_control)`
- **No teacher signals at deployment time**

It will be implemented in `kd_uav/train/train_bc.py` after the CERLAB teacher demonstration dataset is collected.

## Thesis Reference

In any thesis text, the old preliminary BC attempt should be described as:

> A preliminary behavior cloning model was previously attempted using a state-based input representation (position, orientation, and velocity). The resulting policy showed unstable flight behavior and was insufficient as a formal baseline. Therefore, the BC baseline was reconstructed using newly collected CERLAB automatic flight demonstrations, with front-camera images as the sole input, under the same data collection and evaluation protocol as the KD methods.

Do not cite the old model performance numbers as official results.
