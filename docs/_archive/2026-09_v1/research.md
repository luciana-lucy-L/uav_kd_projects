# Research Design
# Knowledge Distillation from a Classical Autonomous Flight System for Lightweight Vision-Based UAV Navigation

---

## 1. Abstract

Recent advances in deep learning have significantly improved autonomous UAV navigation. However, many vision-based navigation models still rely on large neural networks and substantial computational resources, making deployment on lightweight UAV platforms challenging. In contrast, classical autonomous flight systems provide reliable navigation performance through carefully designed perception, planning, and control modules, but often require complex system architectures and high computational costs.

This research proposes a Knowledge Distillation (KD) framework that transfers navigation capability from a classical autonomous flight system to a lightweight vision-based UAV policy. Unlike conventional KD (large NN → small NN), the teacher here is a complete classical robotics autonomy stack (CERLAB Autonomous Flight), not a neural network. The student is a lightweight neural network that uses only monocular front-camera images as input at inference time.

We propose JointKD, a dual-layer distillation method that simultaneously transfers planning-level knowledge (future trajectory from the teacher planner) and execution-level knowledge (control sequence from the teacher controller) to the student. The trajectory supervision acts as an auxiliary training signal that injects planning intent into the shared backbone, while the control sequence head serves as the deployed output. BC, TrajKD, and CtrlKD are included as ablations to validate each component's contribution. The goal is to preserve navigation capability while significantly reducing model complexity and deployment cost.

---

## 2. Research Positioning

### 2.1 Novelty: Classical System as Teacher

| Type | Teacher | Student | Distilled Knowledge |
|---|---|---|---|
| Conventional KD | Large neural network | Small neural network | Soft logits / features |
| Policy distillation | Expert RL policy | Student policy | Actions / policy |
| **This research** | **Classical UAV autonomy stack** | **Lightweight visual policy** | **Trajectory + control** |

The teacher is not a neural network. It is a complete classical autonomy pipeline consisting of Perception → Planning → Control modules. The distilled knowledge is not soft classification logits, but structured planning outputs (trajectories) and control commands.

### 2.2 Distinction from Behavior Cloning

| | BC | This Research |
|---|---|---|
| Teacher signal | Final action labels | Intermediate planning output + control |
| What is learned | What action was taken | Where to go (trajectory) + how to move (control) |
| Planning structure | Implicit | Explicit via trajectory supervision |

Pure BC only learns a direct observation-to-action mapping. This framework additionally uses the planner's trajectory output as structured supervision, encoding the teacher's navigation intent rather than just its final actions.

### 2.3 Core Research Question and Proposed Method

We propose **JointKD**: a dual-layer knowledge distillation method that simultaneously transfers planning-level knowledge (trajectory) and execution-level knowledge (control sequence) from a classical UAV autonomy stack to a lightweight front-camera visual policy.

**Design rationale:** The classical teacher encodes two fundamentally different forms of navigation intelligence — the planner's spatial intent (where to go) and the controller's temporal execution behavior (how to move). Distilling only one form leaves the student with an incomplete picture of the teacher's navigation capability. JointKD is designed to capture both.

> **Why does jointly distilling trajectory (planning) and control sequence (execution) knowledge produce a more capable lightweight visual policy than distilling either form alone?**

---

## 3. Overall Framework

```
  ╔═══════════════════════════════════════════════════════════════╗
  ║             CERLAB Teacher System  (Gazebo, run-time)         ║
  ║   LiDAR · RGB-D · IMU · Odometry · Map                       ║
  ║                                                               ║
  ║   Perception ──▶ B-spline Planner ──▶ Tracking Controller    ║
  ╚══════════════════════╤════════════════════════╤══════════════╝
                         │                        │
              τ_t: K=10 body-frame        U_t: H-step ctrl seq
              waypoints (planner)         [u_t … u_{t+H}] (controller)
                         │                        │
                ─────────────────────────────────────────
                         ↓  synchronized offline recording
  ┌──────────────────────────────────────────────────────────────┐
  │                  Teacher Knowledge Dataset                    │
  │                                                              │
  │    Image I_t  │  Trajectory τ_t  │  Ctrl Sequence U_t       │
  │  (front cam)  │  (planner out)   │  (controller out)        │
  └───────────────────────────┬──────────────────────────────────┘
                              │  KD supervision labels
                  ════════════╪════════════════════════════════════
                  TRAINING    ↓
  ╔═════════════════════════════════════════════════════════════╗
  ║                JointKD Student Network                      ║
  ║                                                             ║
  ║     Input: front-camera image I_t  (only)                  ║
  ║                       ↓                                     ║
  ║           MobileNetV2 Backbone  (1280-d)                   ║
  ║                  ┌────┴────┐                                ║
  ║                  ↓         ↓                                ║
  ║   ┌──────────────────┐  ┌──────────────────────┐           ║
  ║   │  Trajectory Head │  │  Ctrl Sequence Head  │           ║
  ║   │  τ_student       │  │  [u_t … u_{t+H}]    │           ║
  ║   │                  │  │                      │           ║
  ║   │  ▶ TRAIN ONLY    │  │  ▶ TRAIN + DEPLOY    │           ║
  ║   │  (aux supervisor)│  │  (receding horizon)  │           ║
  ║   └────────┬─────────┘  └──────────┬───────────┘           ║
  ║            │                       │                        ║
  ║        L_traj (aux)           L_ctrl_seq (main)             ║
  ╚════════════╪═══════════════════════╪════════════════════════╝
               └───────────┬───────────┘
                           ↓
         L = λ_traj · L_traj + λ_ctrl · L_ctrl_seq

         L_traj     = MSE(τ_student, τ_teacher)
         L_ctrl_seq = Σ_{k=0}^{H-1} w_k · MSE(u_pred(t+k), u_teacher(t+k))

                  ════════════════════════════════════
                  DEPLOYMENT  (teacher not running)

              Front-camera I_t
                    ↓
          MobileNetV2 + Ctrl Sequence Head only
                    ↓
         [u_t, u_{t+1}, …, u_{t+H}]
                    ↓  execute u_t, re-predict next frame
                 cmd_vel → UAV
```

**Ablation study:** BC, TrajKD, and CtrlKD isolate each component of JointKD.

```
  ┌──────────┬───────────────────────────┬────────────────────────────┐
  │ Method   │ Training supervision      │ Deployment output          │
  ├──────────┼───────────────────────────┼────────────────────────────┤
  │ BC       │ u_t (single step)         │ cmd_vel (single step)      │
  │ TrajKD   │ τ_t (aux) + u_t (main)   │ cmd_vel (single step)      │
  │ CtrlKD   │ U_t (H-step seq)         │ cmd_vel (receding horizon) │
  │ JointKD  │ τ_t (aux) + U_t (main)   │ cmd_vel (receding horizon) │
  └──────────┴───────────────────────────┴────────────────────────────┘
```

**Key principle:** Teacher uses rich multi-sensor state (LiDAR, maps, odometry) at data collection time. Student uses only front-camera image at deployment. Teacher signals are KD supervision labels only — never available to the student at inference.

---

## 4. Teacher System: CERLAB Autonomous Flight

### 4.1 System Overview

CERLAB Autonomous Flight is a classical robotics UAV autonomy stack with three modules:

**Perception Module**
- Sensors: RGB-D camera, LiDAR / point cloud, IMU, odometry
- Functions: dynamic obstacle tracking, local/global map construction, depth estimation, collision risk estimation

**Planning Module**
- Algorithms: B-spline trajectory optimization, incremental PRM, MPC-like planning
- Output: future trajectory τ_t = {p_{t+1}, ..., p_{t+K}} (local path, future waypoints)

**Control Module**
- Output: velocity / attitude commands u_t = [vx, vy, vz, yaw_rate]
- Deployed via: `/CERLAB/quadcopter/cmd_vel` (geometry_msgs/TwistStamped)

### 4.2 Teacher Outputs Used for Distillation

| Signal | ROS Topic | Role |
|---|---|---|
| Front camera image | `/camera/color/image_raw` | Student input |
| Teacher trajectory | `/dynamicNavigation/bspline_trajectory` | Planning KD label |
| Teacher control | `/autonomous_flight/target_state` | Control KD label |
| Pose / odometry | `/CERLAB/quadcopter/pose`, `/odom` | Synchronization reference |

### 4.3 Simulation Environment

- Simulator: **Gazebo 11** (ROS Noetic)
- Primary world: `corridor_dynamic_9_nohuman`
- Teacher runs fully autonomously via 2D Nav Goal input

---

## 5. Student Network

### 5.1 Design Principles

- **Training:** receives teacher trajectory labels + control labels as supervision
- **Inference:** front-camera image only — no LiDAR, no map, no online planner, no teacher signal

### 5.2 Architecture

```
Front Camera Image I_t (480×640×3)
        │
        ▼
  MobileNetV2 Backbone
  (lightweight CNN encoder, 1280-dim feature)
        │
        ▼
  Policy Head(s)
  ├── Trajectory Head → τ_student = {(dx_1,dy_1,dz_1), ..., (dx_K,dy_K,dz_K)}
  └── Control Head   → u_student = [vx_b, vy_b, vz_b, yaw_rate]
```

- Control head: MLP (1280 → 256 → 4)
- Temporal modeling (optional): RNN / TCN over image sequence
- Output coordinate: **body frame** (relative positions, not world frame)

### 5.3 Body Frame Rationale

Trajectory labels are expressed in UAV body frame (relative displacements). Using world-frame labels would cause the student to memorize absolute map positions rather than learning vision-based navigation behavior. Body-frame labels generalize across different starting positions.

Camera-to-body frame mapping: `x_c = -dy_b, y_c = -dz_b, z_c = dx_b`

---

## 6. Teacher Knowledge Dataset

### 6.1 Per-Frame Synchronized Data

Each training sample contains:

| Field | Content |
|---|---|
| `image_t` | Front-camera RGB frame |
| `traj_body_t` | K=10 future waypoints in body frame: [[dx_1,dy_1,dz_1], ..., [dx_10,dy_10,dz_10]] |
| `ctrl_body_t` | [vx_b, vy_b, vz_b, yaw_rate] |
| `pose_t`, `odom_t` | For synchronization and reference |
| `episode_id`, `timestamp` | Metadata |

### 6.2 Label Definitions

**Trajectory label** `Y_traj(t)`:
```
Y_traj(t) = [[dx_1, dy_1, dz_1],
             [dx_2, dy_2, dz_2],
             ...
             [dx_K, dy_K, dz_K]]   (K=10, body frame relative)
```

**Control label** `Y_ctrl(t)`:
```
Y_ctrl(t) = [vx_b, vy_b, vz_b, yaw_rate]
```

### 6.3 Dataset Split Rule

Split by **episode** (not random frame). Random frame-level splits place nearly identical neighboring frames in both train and test, making evaluation unreliable.

### 6.4 Current Dataset: uav_kd_v001

| Split | Episodes | Samples |
|---|---|---|
| Train | ep_003, ep_004, ep_007 | 6,376 |
| Val | ep_005 | 569 |

Control stats — mean: [0.386, 0.004, 0.000, -0.003], std: [0.361, 0.194, 0.002, 0.314]

---

## 7. Distillation Strategies

**Overview:** JointKD is the proposed method. BC, TrajKD, and CtrlKD are ablations that isolate the contribution of each design component.

| Role | Method | What is ablated |
|---|---|---|
| No-KD baseline | BC | both knowledge forms removed |
| Ablation A | TrajKD | only planning knowledge; execution knowledge removed |
| Ablation B | CtrlKD | only execution knowledge; planning knowledge removed |
| **Proposed method** | **JointKD** | full method |

---

### 7.1 BC — No-Distillation Baseline

```
Input:   I_t
Output:  u_student = [vx_b, vy_b, vz_b, yaw_rate]
Label:   u_teacher (CERLAB controller output, single step)
Loss:    L_BC = MSE(u_student, u_teacher)
```

**Role:** Establishes the ceiling of pure action imitation without any KD. Encodes no planning structure; vulnerable to distribution shift in closed-loop deployment.

---

### 7.2 Ablation A — TrajKD (Planning Knowledge Only)

TrajKD isolates the contribution of planning-level KD by removing execution-level supervision. It uses the **trajectory-regularized backbone** approach (Option B architecture): the trajectory head is auxiliary during training and discarded at deployment, forcing the backbone to encode planning intent without requiring a separate geometric controller.

```
Training:
  Input:   I_t
  Backbone → Trajectory Head (auxiliary) + Control Head (main)
  Labels:  τ_teacher (K=10 body-frame waypoints) + u_teacher (single step)
  Loss:    L_traj_only = λ·L_traj + L_ctrl   (L_ctrl = single-step MSE, fixes deployment interface)

Deployment:
  Only Control Head is used → cmd_vel directly
  Trajectory Head is discarded
```

**Design note:** Using the trajectory head as auxiliary (rather than as the deployment output) avoids the need for a separate geometric controller (e.g., pure pursuit) at deployment, keeping the deployment interface identical to BC. An alternative implementation (TrajKD+PP: trajectory as output → pure pursuit → cmd_vel) is noted for reference but not included in the main experiment sequence.

**What this ablation answers:** Does adding planning-level supervision (trajectory KD) to the backbone improve navigation over pure BC?

---

### 7.3 Ablation B — CtrlKD (Execution Knowledge Only)

CtrlKD isolates the contribution of execution-level KD by removing planning-level supervision. The student learns the teacher controller's temporal execution pattern over a future horizon H.

```
Input:   I_t
Output:  U_student = [u_t, u_{t+1}, ..., u_{t+H}]
Label:   U_teacher (sliding window over ctrl_body.csv)
Loss:    L_ctrl_seq = Σ_k w_k · MSE(u_student(t+k), u_teacher(t+k))
```

**Inference:** receding horizon — predict H steps, execute u_t only, re-predict at t+1.

**Critical requirement:** Labels must come from CERLAB controller's real output (`ctrl_body.csv`), not derived from trajectory finite-differencing. The controller encodes dynamic constraints, tracking error feedback, and inertia absent from the trajectory.

**Data:** Sliding window over existing `ctrl_body.csv` — no new data collection required.

**What this ablation answers:** Does distilling temporal execution knowledge improve navigation over pure BC?

---

### 7.4 JointKD — Proposed Method (Planning + Execution)

JointKD combines both knowledge forms. The shared MobileNetV2 backbone receives gradient signals from both the trajectory auxiliary loss and the control sequence loss, learning visual representations that encode both planning intent and execution behavior.

```
Architecture:
  Front Camera Image I_t
          ↓
    MobileNetV2 Backbone (shared)
          ├── Trajectory Head  [training only, discarded at deployment]
          │     Output: K=10 body-frame waypoints τ_student
          │     Loss:   L_traj = MSE(τ_student, τ_teacher)
          │     Role:   injects planning intent into backbone representations
          └── Control Sequence Head  [deployed]
                Output: [u_t, u_{t+1}, ..., u_{t+H}]
                Loss:   L_ctrl_seq = Σ_k w_k · MSE(u_pred(t+k), u_teacher(t+k))
                Role:   distills temporal execution behavior

Joint Loss:  L = λ_traj · L_traj + λ_ctrl · L_ctrl_seq
Deployment:  Control Sequence Head only; receding horizon execution
```

**Initial hyperparameters:** λ_traj = 1.0, λ_ctrl = 1.0

**Ablation over λ:** {0.1, 1.0, 5.0} for both λ_traj and λ_ctrl

**Why trajectory head is auxiliary (not deployed):** Deploying the trajectory head would require a separate waypoint-tracking controller, complicating deployment and introducing additional failure modes. Using it as auxiliary supervision achieves the same planning-level backbone regularization while maintaining the same deployment interface as BC and CtrlKD.

**Distinction from TCP (Wu et al., NeurIPS 2022):** TCP is multi-task imitation learning trained on ground-truth human driving data. JointKD is knowledge distillation — teacher supervision signals come from the outputs of a classical UAV autonomy stack, not human demonstrations. TCP deploys both heads; JointKD deploys only the ctrl head.

---

## 8. Evaluation

### 8.1 Experimental Design

All experiments use identical conditions to ensure fair ablation:
- Same simulation environment and routes
- Same student backbone (MobileNetV2)
- Same front-camera-only input at deployment
- Same data split (uav_kd_v001)

| Method | Role | Teacher Signal | Deployment Output | Loss |
|---|---|---|---|---|
| BC | No-KD baseline | u_t (single step) | cmd_vel (single step) | L_BC |
| TrajKD | Ablation A (planning only) | τ_{t:t+K} + u_t | cmd_vel (single step) | λ·L_traj + L_ctrl |
| CtrlKD | Ablation B (execution only) | U_{t:t+H} (controller) | cmd_vel (receding horizon) | L_ctrl_seq |
| **JointKD** | **Proposed method** | τ_{t:t+K} + U_{t:t+H} | cmd_vel (receding horizon) | λ·L_traj + λ·L_ctrl_seq |

### 8.2 Evaluation Metrics

**Offline metrics:**
| Metric | Description |
|---|---|
| Control MSE | Per-dimension: vx, vy, vz, yaw_rate |
| Trajectory Error (ATE) | Average trajectory error |
| Smoothness | Control / trajectory smoothness |
| Model Size | Parameter count (MB) |
| Inference FPS | Frames per second |

**Gazebo deployment metrics:**
| Metric | Description |
|---|---|
| Success Rate (SR) | % of missions completed |
| Collision Rate (CR) | % of missions with collision |
| Trajectory Error | vs teacher reference path |
| Completion Time | Average time per mission |
| Real-time FPS | Actual inference speed |

### 8.3 Expected Outcome

**Primary hypothesis:** JointKD outperforms all ablations, validating that both planning-level and execution-level knowledge are necessary components of the proposed method.

**Ablation findings expected:**
- TrajKD > BC: planning-level supervision improves backbone representations beyond pure action imitation
- CtrlKD > BC: temporal execution knowledge improves smoothness and consistency over single-step BC
- JointKD > TrajKD and JointKD > CtrlKD: each knowledge form contributes independently; their combination is complementary

---

## 9. Related Work

| Category | Representative Work | Relation to This Research |
|---|---|---|
| Knowledge Distillation | Hinton KD; Teacher-Assistant KD | Foundation: KD paradigm and loss design |
| Planner-to-Policy | PlanNetX; MPNet (Qureshi et al., IEEE T-RO 2020, arXiv:1907.06013); MonoMPC | Classical planner output as neural policy supervision; MPNet directly motivates TrajKD's classical→neural paradigm |
| Motion Planner + Policy Distillation | Liu et al. (PMLR 2022) — distilling motion planner augmented policies into visual control | Closest conceptual match; manipulation domain |
| Trajectory-guided BC / Multi-task IL | TCP (Wu et al., NeurIPS 2022, arXiv:2206.08129); DBC — Driver Behavioral Cloning for Route Following Using Task KD (jizefeng0810, CARLA); CILRS (Codevilla et al., 2019, arXiv:1904.08980) | TCP: multi-task IL (not KD) with dual-branch traj+ctrl architecture trained on ground-truth labels; provides architectural reference for joint trajectory+control training (Option B / JointKD); DBC: classical→neural KD paradigm for driving BC, direct structural reference for TrajKD+PP; CILRS: auxiliary task supervision (speed prediction) to improve BC backbone, motivates using trajectory as auxiliary loss |
| BC / Imitation for UAV | SATMN; PPO+BC; LSTM-TD3+BC | BC/IL for UAV navigation and control |
| Classical UAV Autonomy (Teacher) | CERLAB B-spline traj opt; dynamic obstacle tracking; tunnel inspection; incremental PRM | Direct teacher system reference |
| UAV + KD | FHE-aware KD for UAV (arXiv:2411.00403) | UAV-specific KD work |

**Positioning statement:** Existing KD work mostly distills NN→NN. Existing UAV visual policies rely on BC/RL without explicit planning knowledge. This research occupies the intersection of KD, planner-to-policy, and UAV visual navigation — with a classical autonomy stack as teacher.

---

## 10. Expected Contributions

1. **Method — JointKD:** A dual-layer knowledge distillation method that transfers planning-level knowledge (trajectory) and execution-level knowledge (control sequence) from a classical UAV autonomy stack to a lightweight front-camera visual policy. The trajectory head acts as auxiliary supervision during training to inject planning intent into the backbone; the control sequence head is the deployed output. This is the first method to jointly distill structured planning and execution knowledge from a non-neural UAV teacher into a lightweight visual student.

2. **Dataset:** A synchronized teacher knowledge dataset pairing front-camera observations with CERLAB planner trajectories and controller outputs, collected in Gazebo. Enables reproducible evaluation of KD from a classical autonomy stack.

3. **Ablation study:** A controlled ablation (BC / TrajKD / CtrlKD / JointKD) under identical conditions, validating that both knowledge forms contribute independently and that their combination in JointKD achieves the best lightweight navigation performance.
