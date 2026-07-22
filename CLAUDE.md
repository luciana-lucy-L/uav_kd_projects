UAV-KD Project Implementation Guide for Claude

> Research design, framework, distillation strategies, evaluation metrics, and related work:
> → See `docs/../research.md` (i.e., `/home/l/uav_kd_project/research.md`)
> This file covers implementation planning only. Do not duplicate research design content here.

6. Overall Implementation Pipeline
The full project pipeline should be:
CERLAB Autonomous Flight teacher system
        ↓
Run teacher in Gazebo simulation
        ↓
Record synchronized teacher demonstrations:
front image + pose/odom + teacher control + teacher trajectory
        ↓
Build processed teacher knowledge dataset
        ↓
Train BC baseline (no-KD baseline)                    [DONE]
        ↓
Train TrajKD (ablation A — planning knowledge only)   [trained + Gazebo-tested;
                                                        offline ctrl loss beat BC,
                                                        but yaw_rate std in deployment
                                                        was 4x lower than BC — the
                                                        implicit aux-loss signal never
                                                        reached the control head strongly
                                                        enough to produce sharp turns]
        ↓
Train SelfRedWP (new direction) — instead of an implicit trajectory loss,
render the predicted waypoint onto the image as a red dot and condition the
control head on it directly, so the plan is explicit, not just backbone
regularization. Trained with a curriculum (ground-truth waypoint → model's
own predicted waypoint) to avoid train/deploy mismatch. Motivated directly
by TrajKD's turning problem above. [NEXT — see docs/04_experiment_log.md]
        ↓
Train CtrlKD (ablation B — execution knowledge only)
        ↓
Train JointKD (proposed method)
        ↓
Offline ablation evaluation
        ↓
Gazebo deployment evaluation
        ↓
Generate thesis tables and figures

Experiment roles (see research.md Section 7 for design rationale):
- BC:        no-KD baseline
- TrajKD:    ablation A (planning knowledge only, Option B architecture) — trained
  and Gazebo-tested; see pipeline note above on why SelfRedWP followed it
- SelfRedWP: self-predicted waypoint rendered as a red dot, control head
  conditions on it directly (explicit guidance, not an implicit aux loss);
  ground-truth→self-predicted curriculum during training
- CtrlKD:    ablation B (execution knowledge only, H-step sequence)
- JointKD:   proposed method (planning + execution, traj head auxiliary)

Current priority:
1. Implement and train SelfRedWP (see docs/04_experiment_log.md for design rationale).
2. Implement and train CtrlKD (H-step control sequence).
3. Implement and train JointKD.
4. Run Gazebo ablation comparison.

7. Repository Structure Recommendation
Please organize or inspect the project according to the following structure.
uav_kd_project/
│
├── README.md
├── CLAUDE.md
├── PROJECT_GUIDE.md
├── requirements.txt
├── environment.yml
│
├── ros_ws/
│   └── src/
│       ├── CERLAB-UAV-Autonomy/
│       ├── uav_kd_data_collector/
│       ├── uav_kd_student_policy/
│       └── uav_kd_eval_tools/
│
├── kd_uav/
│   ├── datasets/
│   │   ├── dataset_builder.py
│   │   ├── sync_image_traj_ctrl.py
│   │   └── uav_kd_dataset.py
│   │
│   ├── models/
│   │   ├── student_backbone.py
│   │   ├── trajectory_head.py
│   │   ├── control_head.py
│   │   └── joint_policy.py
│   │
│   ├── losses/
│   │   ├── bc_loss.py
│   │   ├── trajectory_kd_loss.py
│   │   ├── control_kd_loss.py
│   │   └── joint_loss.py
│   │
│   ├── train/
│   │   ├── train_bc.py
│   │   ├── train_traj_kd.py
│   │   ├── train_ctrl_kd.py
│   │   └── train_joint_kd.py
│   │
│   ├── eval/
│   │   ├── eval_offline.py
│   │   ├── eval_gazebo.py
│   │   ├── compute_metrics.py
│   │   └── plot_trajectory.py
│   │
│   └── utils/
│       ├── coordinate_transform.py
│       ├── rosbag_utils.py
│       └── config_utils.py
│
├── configs/
│   ├── data/
│   ├── model/
│   ├── train/
│   └── eval/
│
├── data/
│   ├── raw/
│   ├── processed/
│   ├── splits/
│   └── dataset_manifest.csv
│
├── runs/
│   ├── bc/
│   ├── traj_kd/
│   ├── ctrl_kd/
│   └── joint_kd/
│
├── results/
│   ├── tables/
│   ├── figures/
│   ├── trajectories/
│   └── videos/
│
└── docs/
    ├── 00_project_history.md
    ├── 01_environment_log.md
    ├── 02_topic_mapping.md
    ├── 03_data_schema.md
    ├── 04_experiment_log.md
    ├── 05_error_log.md
    └── 06_thesis_notes.md

8. Strict Rules for Claude
Please follow these rules strictly.
1. Do not rewrite the whole project.
2. Do not modify CERLAB source code unless explicitly instructed.
3. Before editing files, first explain which files you plan to change and why.
4. After editing, list all changed files.
5. After editing, provide the exact command to run.
6. After editing, provide the expected output.
7. Always add or update documentation under docs/.
8. Keep raw data and processed data separate.
9. Never use teacher trajectory, LiDAR, map, or odometry during student deployment unless the experiment explicitly allows it.
10. The student policy should use front-camera image only during deployment.

9. Phase-by-Phase Implementation Plan
Phase 0: Project Audit and Environment Check
Goal:
Understand the current project state without modifying files.
Claude should run inspection commands such as:
pwd
git status
git branch
git remote -v
find . -maxdepth 3 -type f | sort | head -200
conda env list
python --version
rosversion -d
gazebo --version
Claude should output:
1. repository tree summary
2. detected ROS packages
3. detected Python training scripts
4. detected launch files
5. detected data folders
6. detected existing BC-related files
7. missing components
8. recommended next step
Record the result in:
docs/01_environment_log.md

Phase 1: Reproduce CERLAB Teacher Automatic Flight
Goal:
Run CERLAB Autonomous Flight in Gazebo and confirm that the teacher system can produce stable autonomous flight behavior.
Known useful topics may include:
/CERLAB/quadcopter/cmd_vel
/CERLAB/quadcopter/odom
/CERLAB/quadcopter/pose
/CERLAB/quadcopter/pose_raw
/camera/color/image_raw
/camera/depth/image_raw
/camera/depth/points
Manual check:
1. UAV takes off successfully.
2. CERLAB teacher starts planning.
3. UAV follows the teacher system.
4. cmd_vel is published.
5. front camera image is available.
6. pose/odom is available.
7. topic frequency is stable.
8. no unexpected crash or missing frame.
Record:
1. launch command
2. world name
3. successful screenshots
4. rostopic list
5. topic frequency
6. failure cases
Save to:
docs/02_topic_mapping.md
docs/04_experiment_log.md

Phase 2: Define Teacher Knowledge
The teacher system provides three types of useful signals.
1. Trajectory knowledge:
   future waypoints / planned trajectory

2. Control knowledge:
   velocity command / yaw command / cmd_vel

3. State reference:
   odometry, pose, velocity, orientation
Recommended trajectory label:
Y_traj(t) = [
  [dx_1, dy_1, dz_1, dyaw_1],
  [dx_2, dy_2, dz_2, dyaw_2],
  ...
  [dx_K, dy_K, dz_K, dyaw_K]
]
Recommended control label:
Y_ctrl(t) = [vx, vy, vz, yaw_rate]
The trajectory label should preferably be represented in the UAV-relative coordinate frame, not directly in the world frame.
Reason:
If the student receives only front-camera images, world-frame labels may cause the model to memorize map positions instead of learning vision-based navigation behavior.

Phase 3: Teacher Demonstration Data Collection
Goal:
Collect synchronized CERLAB teacher demonstrations.
Each episode should contain:
front camera image
pose
odom
teacher control command
teacher trajectory if available
metadata
event log
Recommended raw data format:
data/raw/
└── 20260628_001_corridor_teacher/
    ├── metadata.yaml
    ├── rosbag/
    │   └── flight.bag
    ├── images/
    │   ├── 000000.png
    │   ├── 000001.png
    │   └── ...
    ├── pose.csv
    ├── odom.csv
    ├── cmd_vel.csv
    ├── teacher_traj.csv
    └── event_log.csv
Recommended metadata.yaml:
episode_id: 20260628_001_corridor_teacher
date: 2026-06-28
world: corridor_dynamic_9_nohuman
teacher_system: CERLAB Autonomous Flight
student_used: false
record_type: teacher_demonstration
camera_topic: /camera/color/image_raw
control_topic: /CERLAB/quadcopter/cmd_vel
pose_topic: /CERLAB/quadcopter/pose
odom_topic: /CERLAB/quadcopter/odom
frequency_target_hz: 20
result:
  success: true
  collision: false
  completed_route: true
notes: "Stable teacher run. No collision."
Manual check after every episode:
1. Images are saved correctly.
2. Images are not black or empty.
3. cmd_vel has non-zero values after takeoff.
4. pose/odom is continuous.
5. timestamps are reasonable.
6. collision or failure is marked.
7. takeoff-only useless data is filtered or marked.

Phase 4: Build Synchronized Dataset
Goal:
Convert raw teacher demonstrations into a processed dataset.
Each training sample should contain:
sample_i:
  image_t
  pose_t
  odom_t
  teacher_control_t
  teacher_future_trajectory_t_to_t+K
  episode_id
  timestamp
  validity_flag
Recommended processed dataset structure:
data/processed/
└── uav_kd_v001/
    ├── manifest.csv
    ├── train_index.csv
    ├── val_index.csv
    ├── test_index.csv
    ├── samples/
    │   ├── 000000.npz
    │   ├── 000001.npz
    │   └── ...
    └── dataset_summary.json
manifest.csv should include:
sample_id
episode_id
timestamp
image_path
pose
teacher_cmd_vx
teacher_cmd_vy
teacher_cmd_vz
teacher_cmd_yaw_rate
teacher_future_trajectory
split
valid
notes
Important split rule:
Train/val/test should preferably be split by episode, not random frame.
Reason:
Random frame-level split may place almost identical neighboring frames in both train and test sets, making evaluation unreliable.
Manual check:
1. Randomly visualize 10 samples.
2. Check image-control timestamp gap.
3. Check image-trajectory timestamp gap.
4. Plot teacher trajectory.
5. Confirm future trajectory starts from current UAV state.
6. Confirm labels are in the intended coordinate frame.
7. Confirm train/val/test are episode-level splits.

10. Rebuilt BC Baseline Implementation
This is the first formal model to train after dataset construction.
Goal
Build a clean BC baseline using the new CERLAB teacher demonstration dataset.
Definition
Input:
front camera image

Output:
teacher control command

Loss:
L_BC = MSE(pred_control, teacher_control)
Implementation Requirements
Claude should implement or verify:
kd_uav/train/train_bc.py
kd_uav/models/student_backbone.py
kd_uav/models/control_head.py
kd_uav/losses/bc_loss.py
configs/train/bc.yaml
configs/model/bc_model.yaml
BC Output
The BC model should output:
[vx, vy, vz, yaw_rate]
or match the exact control representation extracted from CERLAB cmd_vel.
BC Training Records
Each BC run must save:
runs/bc/
└── 20260628_001/
    ├── config.yaml
    ├── train.log
    ├── metrics.json
    ├── best_model.pt
    ├── last_model.pt
    ├── loss_curve.png
    ├── val_prediction_examples/
    └── notes.md
Required BC Metrics
For offline evaluation:
train loss
val loss
control MSE
vx MSE
vy MSE
vz MSE
yaw_rate MSE
model size
FPS
For Gazebo deployment:
success rate
collision rate
average flight time
trajectory error if reference exists
qualitative trajectory plot
failure notes
BC Manual Check
After BC training:
1. Check whether train loss decreases.
2. Check whether val loss decreases.
3. Check whether output commands collapse to zero.
4. Check whether output commands are too large.
5. Plot predicted control vs teacher control.
6. Run at least one Gazebo deployment test.
7. Record whether the student uses only the front camera during deployment.
BC Thesis Role
The BC baseline should be described as:
A rebuilt behavior cloning baseline trained from CERLAB automatic flight demonstrations. It directly maps front-camera images to teacher control commands without explicit trajectory-level distillation.
Do not describe the old poor BC model as the official baseline.
The old BC result should be described only in the project history:
A preliminary BC implementation was previously attempted and showed unstable behavior. Based on this observation, the baseline was reconstructed using a cleaner CERLAB teacher demonstration dataset and a unified evaluation protocol.

11. TrajKD Implementation (Ablation A — Planning Knowledge Only)
Role: ablation that isolates planning-level KD. See research.md Section 7.2 for design rationale.

Architecture (Option B):
- Training: backbone → trajectory head (auxiliary, L_traj) + control head (main, L_ctrl single-step)
- Deployment: control head only → cmd_vel directly (trajectory head discarded)
- No pure pursuit required at deployment

Definition:
  Input:       front camera image I_t
  Traj head:   K=10 body-frame waypoints τ_student
  Ctrl head:   [vx_b, vy_b, vz_b, yaw_rate]
  Loss:        L = λ_traj · MSE(τ_student, τ_teacher) + MSE(u_student, u_teacher)
  Deploy:      ctrl head output only

Implementation files:
  kd_uav/models/trajectory_head.py     — MLP: 1280 → 256 → K*3 (K=10 waypoints × 3 dims)
  kd_uav/models/control_head.py        — reuse existing (1280 → 256 → 4)
  kd_uav/losses/trajectory_kd_loss.py  — L_traj = MSE(τ_student, τ_teacher)
  kd_uav/train/train_traj_kd.py        — joint training loop, both heads
  configs/train/traj_kd.yaml
  runs/traj_kd/<timestamp>/

Required records per run:
  train loss (L_traj + L_ctrl)
  val trajectory MSE (per-waypoint and aggregate ATE)
  val control MSE (vx, vy, vz, yaw_rate)
  predicted vs teacher trajectory plot
  model size, FPS

Manual check:
1. Trajectory head output range reasonable (not NaN, not collapsed to zero).
2. Control head output consistent with BC baseline range.
3. λ_traj initial value = 1.0; log sensitivity to λ ∈ {0.1, 1.0, 5.0}.

12. CtrlKD Implementation (Ablation B — Execution Knowledge Only)
Role: ablation that isolates execution-level KD. See research.md Section 7.3 for design rationale.

Architecture:
- Training: backbone → control sequence head, L_ctrl_seq (H-step weighted MSE)
- Deployment: predict H steps, execute u_t only, re-predict at t+1 (receding horizon)
- No trajectory supervision

Definition:
  Input:       front camera image I_t
  Output:      U_student = [u_t, u_{t+1}, ..., u_{t+H}]  (H-step sequence)
  Labels:      sliding window over ctrl_body.csv (H rows starting at frame t)
  Loss:        L_ctrl_seq = Σ_{k=0}^{H-1} w_k · MSE(u_pred(t+k), u_teacher(t+k))
               w_k = 1.0 (uniform initial); can decay with k
  Deploy:      execute u_t only; re-predict each frame

Data preparation:
  No new data collection needed.
  Build sliding window index over existing ctrl_body.csv:
    each sample t → label rows [t, t+1, ..., t+H-1]
    handle episode boundaries (do not cross episode boundary)
  Initial H = 5; log sensitivity to H ∈ {3, 5, 10}

Implementation files:
  kd_uav/models/control_head.py         — extend to output H*4 (reshape to [H, 4])
  kd_uav/losses/control_kd_loss.py      — weighted MSE over H steps
  kd_uav/train/train_ctrl_kd.py
  configs/train/ctrl_kd.yaml
  runs/ctrl_kd/<timestamp>/

Required records per run:
  train loss, val loss (L_ctrl_seq)
  per-step MSE: u_t, u_{t+1}, ..., u_{t+H-1}
  per-dimension MSE: vx, vy, vz, yaw_rate
  model size, FPS

Manual check:
1. Label sliding window does not cross episode boundary.
2. u_t prediction quality comparable to BC (sanity check).
3. Smoothness of predicted sequence (no sudden jumps between steps).

13. JointKD Implementation (Proposed Method)
Role: proposed method combining planning-level and execution-level KD. See research.md Section 7.4 for design rationale.

Architecture:
  Front Camera Image I_t
          ↓
    MobileNetV2 Backbone (shared)
          ├── Trajectory Head  [training only — discarded at deployment]
          │     Output: K=10 body-frame waypoints τ_student
          │     Loss:   L_traj = MSE(τ_student, τ_teacher)
          └── Control Sequence Head  [deployed]
                Output: [u_t, u_{t+1}, ..., u_{t+H}]
                Loss:   L_ctrl_seq = Σ_k w_k · MSE(u_pred(t+k), u_teacher(t+k))

  Joint Loss:  L = λ_traj · L_traj + λ_ctrl · L_ctrl_seq
  Deploy:      Control Sequence Head only; receding horizon (execute u_t, re-predict each frame)

Initial hyperparameters:
  λ_traj = 1.0, λ_ctrl = 1.0, H = 5, K = 10
  Ablation: λ ∈ {0.1, 1.0, 5.0} for both

Implementation files:
  kd_uav/models/trajectory_head.py      — reuse from TrajKD
  kd_uav/models/control_head.py         — reuse H-step version from CtrlKD
  kd_uav/models/joint_policy.py         — combines backbone + both heads
  kd_uav/losses/joint_loss.py           — λ_traj·L_traj + λ_ctrl·L_ctrl_seq
  kd_uav/train/train_joint_kd.py
  configs/train/joint_kd.yaml
  runs/joint_kd/<timestamp>/

Required records per run:
  train loss (total, L_traj component, L_ctrl_seq component)
  val trajectory MSE (ATE)
  val control MSE per step and per dimension
  model size, FPS
  λ sensitivity results

Deployment note:
  At inference, trajectory head is not loaded.
  Only backbone + control sequence head weights are needed.
  Saves a checkpoint with trajectory head stripped for clean deployment.

Manual check:
1. Both loss components decreasing (not one dominating).
2. Trajectory head output reasonable (even though not deployed — signals healthy backbone).
3. Control sequence output smoother than CtrlKD alone (expected due to planning regularization).

14. Evaluation Metrics
The evaluation should include both offline and deployment metrics.
Offline Metrics
Control MSE
Trajectory Error
Average Trajectory Error
Smoothness
Validation Loss
Model Size
FPS
Deployment Metrics
Success Rate
Collision Rate
Average Flight Time
Trajectory Error
Real-time FPS
Failure Case Notes
Recommended tables:
Dataset Summary Table
  Dataset version | Episodes | Frames | Duration | World | Successful | Failed | Collision count

Ablation Summary Table (main result table)
  Method  | Role              | Teacher signal        | Deploy output         | Loss
  BC      | No-KD baseline    | u_t (single step)     | cmd_vel (single step) | L_BC
  TrajKD  | Ablation A        | τ_{t:t+K} + u_t       | cmd_vel (single step) | λ·L_traj + L_ctrl
  CtrlKD  | Ablation B        | U_{t:t+H} (ctrl seq)  | cmd_vel (receding H)  | L_ctrl_seq
  JointKD | Proposed method   | τ_{t:t+K} + U_{t:t+H}| cmd_vel (receding H)  | λ·L_traj + λ·L_ctrl_seq

Offline Evaluation Table
  Method | Val Loss | Traj ATE | Ctrl MSE (vx/vy/vz/yaw) | Smoothness | Model size | FPS

Gazebo Deployment Table
  Method | Runs | Success Rate | Collision Rate | Avg Traj Error | Avg Time | FPS

15. Minimum Midterm Deliverable
If time is limited, the minimum useful result should be:
1. CERLAB teacher automatic flight reproduced.                          [DONE]
2. Teacher demonstration data collected.                                [DONE]
3. Synchronized dataset v001 built.                                     [DONE]
4. BC baseline trained and deployed.                                    [DONE]
5. TrajKD (Option B) trained.                                           [DONE — trained + Gazebo-tested; turning behavior issue found, motivated SelfRedWP]
6. SelfRedWP trained (new direction, see pipeline note above).
7. CtrlKD trained.
8. BC vs TrajKD vs SelfRedWP vs CtrlKD offline comparison.
9. Preliminary Gazebo deployment for at least BC, TrajKD, and SelfRedWP.
10. JointKD training started with clear λ ablation plan.

The midterm presentation can say:
We propose JointKD, a dual-layer knowledge distillation method that transfers planning-level (trajectory) and execution-level (control sequence) knowledge from a classical UAV autonomy stack to a lightweight front-camera visual policy. BC and ablations (TrajKD, CtrlKD) have been trained and compared offline. JointKD training is underway.

16. Project History Writing Template
Use this structure for recording implementation history:
Problem → Decision → Implementation → Observation → Next Step
Example:
Stage: Preliminary BC Attempt

Problem:
An initial behavior cloning model was previously tested, but the resulting flight behavior was unstable and the performance was not sufficient as a formal baseline.

Decision:
The BC baseline should be rebuilt using newly collected CERLAB automatic flight demonstrations, so that BC, Trajectory KD, Control KD, and Joint KD can be evaluated under the same data and evaluation protocol.

Implementation:
The CERLAB Autonomous Flight system is used to generate teacher demonstrations. Front-camera images are synchronized with teacher control commands and teacher trajectories.

Observation:
The rebuilt BC baseline will serve as a clean no-distillation comparison. Any improvement from Trajectory KD or Joint KD can then be interpreted more reliably.

Next Step:
Construct the synchronized teacher demonstration dataset and train the rebuilt BC baseline.

17. Experiment Log Template
Every implementation step should update:
docs/04_experiment_log.md
Use this format:
# Experiment Log

## Date
2026-06-28

## Goal
Build synchronized dataset from CERLAB teacher run.

## Changed Files
- ros_ws/src/uav_kd_data_collector/scripts/record_teacher_data.py
- kd_uav/datasets/sync_image_traj_ctrl.py

## Commands
```bash
roslaunch ...
python sync_image_traj_ctrl.py --input ... --output ...
Result
Recorded 3 teacher episodes.
Total frames: 12430
Valid synchronized samples: 11802
Average image-control timestamp gap: 0.018s
Manual Check
Camera images normal: yes
cmd_vel non-zero: yes
trajectory label visualized: yes
collision data removed: yes
Problems
One episode had missing image frames.
Need to improve timestamp tolerance.
Decision
Use max sync tolerance = 0.05s for v001 dataset.
Next Step
Train rebuilt BC baseline on uav_kd_v001.

---

# 18. First Task for Claude

Please start with project audit only.

Do not modify files yet.

Please output:

```text
1. repository tree summary
2. detected ROS packages
3. detected launch files
4. detected Python scripts
5. detected data folders
6. detected existing BC-related files
7. detected CERLAB-related files
8. missing components for data collection
9. missing components for rebuilt BC baseline
10. recommended next step
After the audit, wait for my confirmation before editing code.
