# Topic Mapping — CERLAB Dynamic Navigation

**Date:** 2026-06-28
**Phase:** 1 — Reproduce CERLAB Teacher Flight
**Status:** Derived from source code inspection. Live verification required (see Section 6).

---

## 1. Correct Launch Sequence

The teacher system requires two independent workspaces launched in order.

### Terminal 1 — Gazebo Simulator (catkin_ws)

```bash
source /home/l/catkin_ws/devel/setup.bash
roslaunch uav_simulator start.launch
```

**What this starts:**
- Gazebo 11 with the default world (`corridor_dynamic_9.world`)
- UAV quadcopter model (URDF spawn with depth camera plugin)
- Four topic throttle nodes: `odom_raw` → `odom`, `pose_raw` → `pose`, `vel_raw` → `vel`, `acc_raw` → `acc` at **30 Hz**
- TF broadcaster (`setupTF.launch`)
- Keyboard control node (`keyboard_control`)

**To use a different world**, edit `start.launch` and uncomment the desired `world_name` argument before launching. Recommended world for data collection:

```bash
# Option A: corridor with dynamic obstacles (9 moving boxes, no humans)
worlds/corridor/corridor_dynamic_9_nohuman.world

# Option B: floorplan environments
worlds/floorplan1/floorplan1_dynamic_16.world
worlds/floorplan2/floorplan2_dynamic_12.world
```

### Terminal 2 — CERLAB Teacher (cerlab_ws)

```bash
source /home/l/catkin_ws/devel/setup.bash
source /home/l/cerlab_ws/devel/setup.bash
roslaunch autonomous_flight dynamic_navigation.launch
```

**Important:** Source catkin_ws first. The `tracking_controller::Target` message type is defined in cerlab_ws but the controller links against catkin packages.

**What this starts:**
- `tracking_controller_node` — cascade PID controller
- `dynamic_navigation_node` — B-spline trajectory planner + replanner

**The UAV takeoff is automatic.** After `odom` is received, the drone takes off to `takeoff_height = 1.0 m` (set in `cfg/dynamic_navigation/flight_base.yaml`).

### Terminal 3 — RViz (optional, for goal input)

```bash
source /home/l/cerlab_ws/devel/setup.bash
roslaunch remote_control dynamic_navigation_rviz.launch
```

**Send a navigation goal:** Use the **2D Nav Goal** tool in RViz to click a target position. This publishes to `/move_base_simple/goal` which `flightBase` subscribes to.

---

## 2. Complete Topic Map

### 2.1 UAV State Topics (from simulator)

| Topic | Message Type | Publisher | Frequency | Notes |
|---|---|---|---|---|
| `/CERLAB/quadcopter/odom_raw` | `nav_msgs/Odometry` | Gazebo quadcopter plugin | ~200+ Hz | Raw, high-frequency ground truth |
| `/CERLAB/quadcopter/odom` | `nav_msgs/Odometry` | topic throttle node | **30 Hz** | Throttled from `odom_raw` — use this |
| `/CERLAB/quadcopter/pose_raw` | `geometry_msgs/PoseStamped` | Gazebo plugin | ~200+ Hz | Raw pose |
| `/CERLAB/quadcopter/pose` | `geometry_msgs/PoseStamped` | topic throttle node | **30 Hz** | Throttled — use this |
| `/CERLAB/quadcopter/vel_raw` | `geometry_msgs/TwistStamped` | Gazebo plugin | ~200+ Hz | Raw velocity |
| `/CERLAB/quadcopter/vel` | `geometry_msgs/TwistStamped` | topic throttle node | **30 Hz** | Throttled — use this |
| `/CERLAB/quadcopter/acc_raw` | `geometry_msgs/TwistStamped` | Gazebo plugin | ~200+ Hz | Raw acceleration |
| `/CERLAB/quadcopter/acc` | `geometry_msgs/TwistStamped` | topic throttle node | **30 Hz** | Throttled — use this |

### 2.2 Camera Topics (from URDF depth camera plugin)

Camera sensor is embedded in `quadcopter.urdf` via `libgazebo_ros_openni_kinect.so`.

| Topic | Message Type | Publisher | Frequency | Notes |
|---|---|---|---|---|
| `/camera/color/image_raw` | `sensor_msgs/Image` | Gazebo camera plugin | **~30 Hz** | **Primary input for student** — RGB front camera |
| `/camera/color/camera_info` | `sensor_msgs/CameraInfo` | Gazebo camera plugin | ~30 Hz | Intrinsics |
| `/camera/depth/image_raw` | `sensor_msgs/Image` | Gazebo camera plugin | ~30 Hz | Depth image — teacher only |
| `/camera/depth/camera_info` | `sensor_msgs/CameraInfo` | Gazebo camera plugin | ~30 Hz | Depth intrinsics |
| `/camera/depth/points` | `sensor_msgs/PointCloud2` | Gazebo camera plugin | ~30 Hz | Used by CERLAB map_manager |

**Note:** The camera publishes from the moment the UAV model is spawned, independent of whether the teacher is flying. Images may appear black during early startup if Gazebo is still loading textures.

### 2.3 Teacher Control Topics

| Topic | Message Type | Publisher | Frequency | Notes |
|---|---|---|---|---|
| `/autonomous_flight/target_state` | `tracking_controller/Target` | `dynamic_navigation_node` | **~100 Hz** | **Best source for teacher control label** |
| `/CERLAB/quadcopter/cmd_acc` | `mavros_msgs/PositionTarget` | `tracking_controller_node` | ~100 Hz | Acceleration setpoint — low-level, harder to learn |
| `/CERLAB/quadcopter/cmd_vel` | `geometry_msgs/TwistStamped` | keyboard control only | N/A during auto | **NOT published during autonomous flight** |

**Critical note on cmd_vel:** The `/CERLAB/quadcopter/cmd_vel` topic is the velocity interface for the simulator plugin but is only published by keyboard control. During autonomous flight, the tracking controller sends acceleration commands to `/CERLAB/quadcopter/cmd_acc`, not velocity commands to `cmd_vel`.

The **old `record_expert.py` subscribed to both `cmd_vel` and `cmd_acc`**. During CERLAB teacher flight, only `cmd_acc` was active. This is why the old expert data used `cmd_acc` as the action source.

**For this project, use `/autonomous_flight/target_state` as the teacher control label.**

### Target message schema (`tracking_controller/Target`):

```
std_msgs/Header header
uint8 type_mask
uint8 IGNORE_ACC = 1
uint8 IGNORE_ACC_VEL = 2
geometry_msgs/Vector3 position    # desired position [x, y, z] in world frame
geometry_msgs/Vector3 velocity    # desired velocity [vx, vy, vz] in world frame ← USE THIS
geometry_msgs/Vector3 acceleration # desired acceleration [ax, ay, az]
float32 yaw                        # desired yaw angle [rad] ← compute yaw_rate from this
```

**Derived control label for training:**
```
teacher_ctrl[t] = [
    target_state.velocity.x,   # vx (m/s)
    target_state.velocity.y,   # vy (m/s)
    target_state.velocity.z,   # vz (m/s)
    (yaw[t] - yaw[t-1]) / dt   # yaw_rate (rad/s) — computed from consecutive yaw values
]
```

### 2.4 Teacher Trajectory Topics

All published by `dynamic_navigation_node` at ~30 Hz (visualization timer `Duration(0.033)`).

| Topic | Message Type | Content | Use for KD |
|---|---|---|---|
| `dynamicNavigation/bspline_trajectory` | `nav_msgs/Path` | **Final executed B-spline trajectory** — the path CERLAB actually flies | **YES — primary trajectory label** |
| `dynamicNavigation/poly_traj` | `nav_msgs/Path` | Polynomial trajectory (input to B-spline optimizer) | Optional |
| `dynamicNavigation/input_trajectory` | `nav_msgs/Path` | Adjusted poly trajectory fed to B-spline | Reference |
| `dynamicNavigation/pwl_trajectory` | `nav_msgs/Path` | Piecewise-linear fallback (used when dynamic obstacles present) | Reference |
| `dynamicNavigation/rrt_path` | `nav_msgs/Path` | Global RRT path (only if global planner enabled) | Not needed |

**Key property of `bspline_trajectory`:** It contains the full planned path ahead of the UAV as a sequence of `geometry_msgs/PoseStamped` waypoints. For the trajectory distillation label, extract the next K waypoints relative to the current UAV position.

**Trajectory label construction:**
```
bspline_trajectory.poses[i+1 .. i+K]   →  extract positions
transform from world frame to UAV-relative frame (body frame at current pose)
→  teacher_traj[t] = [[dx1, dy1, dz1], [dx2, dy2, dz2], ..., [dxK, dyK, dzK]]
```

**Recommended K = 10 at 0.1–0.2 m spacing** → covers ~1–2 m look-ahead horizon.

### 2.5 Additional Topics (visualization / internal use)

| Topic | Type | Notes |
|---|---|---|
| `/tracking_controller/robot_pose` | `geometry_msgs/PoseStamped` | Tracking controller's pose estimate |
| `/tracking_controller/trajectory_history` | `nav_msgs/Path` | Executed trajectory history |
| `/tracking_controller/target_pose` | `geometry_msgs/PoseStamped` | Current target position |
| `/tracking_controller/vel_and_acc_info` | `visualization_msgs/Marker` | Velocity/accel visualization |
| `/move_base_simple/goal` | `geometry_msgs/PoseStamped` | RViz goal click input |
| `/CERLAB/quadcopter/setpoint_pose` | `geometry_msgs/PoseStamped` | Pose setpoint during takeoff |

---

## 3. Data Flow Diagram

```
Gazebo World
    │
    ├── quadcopterPlugin → /CERLAB/quadcopter/odom_raw (200+ Hz)
    │                          │
    │                   topic_throttle → /CERLAB/quadcopter/odom (30 Hz)
    │
    ├── camera_plugin → /camera/color/image_raw (30 Hz)  ← STUDENT INPUT
    │                → /camera/depth/points             ← used by CERLAB map
    │
CERLAB Teacher
    ├── dynamic_navigation_node
    │       ├── subscribes: /CERLAB/quadcopter/odom (flightBase)
    │       ├── subscribes: /move_base_simple/goal (RViz click)
    │       ├── publishes:  /autonomous_flight/target_state (100 Hz)  ← TEACHER CONTROL
    │       └── publishes:  dynamicNavigation/bspline_trajectory (30 Hz)  ← TEACHER TRAJ
    │
    └── tracking_controller_node
            ├── subscribes: /CERLAB/quadcopter/odom (remapped from mavros)
            ├── subscribes: /autonomous_flight/target_state
            └── publishes:  /CERLAB/quadcopter/cmd_acc (100 Hz) → simulator
```

---

## 4. Topics Required for This Project

### For the rebuilt BC baseline (image → control)

| Topic | Required | Source | Notes |
|---|---|---|---|
| `/camera/color/image_raw` | **YES** | Gazebo plugin | Student input — confirmed in URDF |
| `/autonomous_flight/target_state` | **YES** | dynamic_navigation_node | Teacher control: velocity + yaw → [vx, vy, vz, yaw_rate] |
| `/CERLAB/quadcopter/odom` | YES (sync) | topic throttle | For timestamp synchronization and state context |

### For trajectory KD (image → future trajectory)

| Topic | Required | Source | Notes |
|---|---|---|---|
| `/camera/color/image_raw` | **YES** | Gazebo plugin | Student input |
| `dynamicNavigation/bspline_trajectory` | **YES** | dynamic_navigation_node | Teacher trajectory label |
| `/CERLAB/quadcopter/odom` | YES (sync) | topic throttle | For converting world-frame traj to body-frame |

### For state reference / synchronization

| Topic | Required | Source | Notes |
|---|---|---|---|
| `/CERLAB/quadcopter/odom` | YES | topic throttle | 30 Hz, stable, used for all synchronization |
| `/CERLAB/quadcopter/pose` | Optional | topic throttle | Alternative state reference |

---

## 5. BC Baseline Sufficiency Analysis

**Question:** Are the available topics sufficient to rebuild the image-based BC baseline?

**Answer: YES, all required topics are confirmed present in source code.**

| Requirement | Topic | Availability | Confidence |
|---|---|---|---|
| Front-camera RGB image | `/camera/color/image_raw` | Defined in `quadcopter.urdf`, lines 87 | **Confirmed** (code) |
| Teacher velocity command (vx, vy, vz) | `/autonomous_flight/target_state` `.velocity` | Published by `flightBase.cpp`, line 52 | **Confirmed** (code) |
| Teacher desired yaw (→ yaw_rate) | `/autonomous_flight/target_state` `.yaw` | Same topic | **Confirmed** (code) |
| UAV odometry for synchronization | `/CERLAB/quadcopter/odom` | Throttled from `odom_raw` in `start.launch` | **Confirmed** (code) |
| Teacher trajectory (for Traj KD) | `dynamicNavigation/bspline_trajectory` | Published in `registerPub()` | **Confirmed** (code) |

**Conclusion:** The CERLAB teacher system provides all signals needed for:
- BC baseline training (image → control)
- Trajectory KD training (image → future waypoints)
- Joint KD training (image → control + trajectory)

No additional topics or external tools are needed.

---

## 6. Live Verification Checklist (to run during first teacher flight)

When running the teacher for the first time, verify each item below and record results in `docs/04_experiment_log.md`.

```bash
# After launching both terminals:

# 1. List all active topics
rostopic list

# 2. Verify camera is publishing
rostopic hz /camera/color/image_raw
# Expected: ~30 Hz

# 3. Verify odom is publishing
rostopic hz /CERLAB/quadcopter/odom
# Expected: ~30 Hz

# 4. Verify target_state is publishing (after UAV takes off and goal is given)
rostopic hz /autonomous_flight/target_state
# Expected: ~100 Hz

# 5. Verify bspline_trajectory is publishing (after goal is given)
rostopic hz /dynamicNavigation/bspline_trajectory
# Expected: ~30 Hz

# 6. Check camera image is non-black (requires display)
rosrun image_view image_view image:=/camera/color/image_raw

# 7. Check target state values are non-zero after takeoff
rostopic echo /autonomous_flight/target_state | head -30

# 8. Check bspline trajectory has waypoints
rostopic echo /dynamicNavigation/bspline_trajectory | head -30

# 9. Check that cmd_vel is NOT published during autonomous flight
rostopic hz /CERLAB/quadcopter/cmd_vel
# Expected: 0 Hz (or topic not present)
```

**Record results in:** `docs/04_experiment_log.md`

---

## 7. Known Issues and Warnings

1. **cmd_vel is NOT the teacher control during autonomous flight.** Do not subscribe to `cmd_vel` for BC/KD training. Use `/autonomous_flight/target_state` instead. The old `record_expert.py` subscribing to `cmd_vel` was ineffective during teacher flights; it only captured data from the keyboard control mode.

2. **yaw_rate is not published as a separate topic.** It must be derived: `yaw_rate = (yaw[t] - yaw[t-1]) / (timestamp[t] - timestamp[t-1])`. Use a running window filter to reduce noise.

3. **B-spline trajectory may not publish before a goal is given.** The `visCB` only publishes if the trajectory message is non-empty. During takeoff and hover (before first goal), the trajectory topic may be silent.

4. **`use_global_planner: false` in the config.** The RRT path topic (`dynamicNavigation/rrt_path`) will be empty unless global planning is enabled. This is fine for our purposes.

5. **Source order matters.** Always source `catkin_ws` before `cerlab_ws` to avoid message type conflicts. The `tracking_controller::Target` message must be found from the cerlab_ws overlay.

6. **Takeoff delay.** The UAV takes off automatically when `dynamic_navigation.launch` starts, but requires odom to be received first. There is a brief wait loop in `flightBase`. Expect 2–5 seconds before takeoff begins.
