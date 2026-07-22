# UAV Knowledge Distillation Project

**Research Title:** Knowledge Distillation from a Classical Autonomous Flight System for Lightweight Vision-Based UAV Navigation

## Research Overview

This project transfers navigation capability from the CERLAB Autonomous Flight classical robotics stack (the teacher) to a lightweight vision-based neural network policy (the student). The student uses only a front-facing RGB camera image as input and outputs velocity control commands.

This is **not** standard neural-to-neural knowledge distillation. The teacher is a classical autonomy stack — perception, planning, and control — not a neural network.

## Teacher System

**CERLAB Autonomous Flight** (`/home/l/cerlab_ws/src/CERLAB-UAV-Autonomy/`)

The teacher provides:
- **Trajectory knowledge**: planned future waypoints from `trajectory_planner`
- **Control knowledge**: velocity commands from `tracking_controller` via `/CERLAB/quadcopter/cmd_vel`
- **State reference**: odometry and pose via `/CERLAB/quadcopter/odom`

The CERLAB source code is an external dependency. It must not be modified.

## Student Policy

- **Input**: front-facing RGB camera image only (`/camera/color/image_raw`)
- **Output**: velocity command `[vx, vy, vz, yaw_rate]`
- **No teacher signals at deployment time**

## Experimental Methods

| Method | Input | Output | Loss | Status |
|---|---|---|---|---|
| BC Baseline (rebuilt) | front image | control cmd | MSE(pred, teacher_ctrl) | **Done** — trained on `uav_kd_v002`, deployed & flight-tested in Gazebo (2026-07-01) |
| BC-RedWP (image + projected waypoint) | front image | control cmd | MSE(pred, teacher_ctrl) | Done — offline metrics on par with BC baseline |
| Trajectory KD | front image | future trajectory | MSE(pred, teacher_traj) | Pending — next up |
| Control KD | front image | control cmd | MSE(pred, teacher_ctrl) | Pending |
| Joint KD | front image | traj + control | weighted combination | Pending |

## Implementation Pipeline

```
CERLAB teacher system (cerlab_ws)
    ↓
Run teacher in Gazebo simulation
    ↓
Record synchronized teacher demonstrations
(front image + pose + odom + cmd_vel + teacher trajectory)
    ↓
Build processed dataset (data/processed/uav_kd_v001/)
    ↓
Train rebuilt BC baseline   →   Train Trajectory KD
    ↓
Train Control KD            →   Train Joint KD
    ↓
Offline evaluation + Gazebo deployment evaluation
    ↓
Thesis tables and figures
```

## Repository Layout

```
uav_kd_project/
├── ros_ws/src/             ROS packages (data collector, student policy, eval tools)
├── kd_uav/                 Python training package (models, losses, train, eval, utils)
├── configs/                YAML config files (data, model, train, eval)
├── data/raw/               Raw teacher demonstration episodes
├── data/processed/         Synchronized dataset for training
├── data/splits/            Train/val/test index files
├── runs/                   Training run outputs (checkpoints, logs, metrics)
├── results/                Evaluation outputs (tables, figures, trajectories, videos)
├── legacy/                 Archive of old state-based BC work (not the formal baseline)
└── docs/                   Phase logs, topic mapping, data schema, experiment log
```

## External Dependencies

| Dependency | Path | Purpose |
|---|---|---|
| CERLAB Autonomy Stack | `/home/l/cerlab_ws/src/CERLAB-UAV-Autonomy/` | Teacher system — do not modify |
| UAV Simulator | `/home/l/catkin_ws/src/uav_simulator/` | Gazebo worlds and UAV plugin |

## Important Notes

- The `legacy/` directory contains a preliminary state-based BC implementation. It is **not** the formal baseline. See `legacy/README.md`.
- The formal BC baseline will be rebuilt from CERLAB teacher demonstrations using front-camera images.
- The student policy must use **only** the front camera image during deployment.
- Train/val/test splits must be done **by episode**, not by random frame.

## Phase Status

| Phase | Goal | Status |
|---|---|---|
| 0 | Project audit and environment check | Done (2026-06-28) |
| 1 | Reproduce CERLAB teacher automatic flight | Done (2026-06-28) |
| 2 | Define teacher knowledge and topics | Done |
| 3 | Collect teacher demonstration data | Done — 9 episodes (`ep_test_003`–`007`, `ep_006`, `ep_008`, `ep_009`) |
| 4 | Build synchronized dataset | Done — `uav_kd_v001` (2026-06-29), superseded by `uav_kd_v002` (2026-07-01, adds rotation-diverse episodes + fixes degenerate spot-turn trajectory labels) |
| 5 | Train rebuilt BC baseline | **Done (baseline phase closed 2026-07-01)** — trained on `uav_kd_v002`, deployed & flight-tested in Gazebo (stable, non-degenerate yaw_rate output). See `docs/04_experiment_log.md`. |
| 6 | Train Trajectory KD | Pending — next up |
| 7 | Train Control KD + Joint KD | Pending |
| 8 | Evaluation and thesis writing | Pending |

## See Also

- `CLAUDE.md` — full project specification and rules for Claude
- `docs/01_environment_log.md` — environment audit result
- `docs/02_topic_mapping.md` — CERLAB ROS topic map (to be filled in Phase 1)
- `docs/04_experiment_log.md` — running experiment log
