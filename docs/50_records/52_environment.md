# 环境、话题映射与命令速查

> 层级：记录层。**环境变更时修改本文件。**
> 合并自 `_archive/2026-09_v1/` 的 `01_environment_log.md` + `02_topic_mapping.md` + `commands.txt`
> 环境部分实测核实于 **2026-09-07**

---

## 1. 系统环境（2026-09-07 实测）

| 项 | 值 |
|---|---|
| OS | Ubuntu 20.04（Linux 5.15.0-139-generic, x86_64） |
| ROS | Noetic |
| Gazebo | 11.15.1 |
| **系统 Python（ROS 用）** | **3.8.10**（`/usr/bin/python3`） |
| GPU | **NVIDIA RTX 2080 Ti，11264 MiB** |
| NVIDIA 驱动 | **570.133.07** |

### conda 环境 `dbc241`（训练用）

| 包 | 版本 |
|---|---|
| torch | **2.4.1+cu121**（CUDA available: **True**） |
| torchvision | **0.19.1+cu121** |
| numpy | 1.24.4 |
| opencv (cv2) | 4.11.0 |
| pyyaml | 6.0.3 |
| pandas | 2.0.3 |
| matplotlib | 3.7.5 |
| tqdm | 4.67.3 |

```bash
source ~/miniconda3/etc/profile.d/conda.sh && conda activate dbc241
```

### ⛔ 环境使用铁律

| 任务 | 环境 |
|---|---|
| ROS / Gazebo / 采集节点 / 部署节点 | **系统 Python，先 `conda deactivate`** |
| 数据集构建 / 训练 / 评估 / 诊断 | **`dbc241`** |

**绝不在 conda 激活状态下启动 ROS 节点**（原因见 `50_records/51_issues.md` 第一条）。
部署节点的 launch 文件用 `launch-prefix` 指定 dbc241 的 python，绕过 catkin wrapper 的 shebang。

---

## 2. ROS 工作区

| 工作区 | 路径 | 内容 | 可否修改 |
|---|---|---|---|
| `catkin_ws` | `/home/l/catkin_ws/` | `uav_simulator`（Gazebo world、UAV 模型、插件） | ⛔ 不可 |
| `cerlab_ws` | `/home/l/cerlab_ws/` | CERLAB 教师系统 | ⛔ **绝不可修改** |
| `ros_ws` | `/home/l/uav_kd_project/ros_ws/` | 本项目的 ROS 包 | ✅ 本项目所有 |

### cerlab_ws 的包

| 包 | 作用 |
|---|---|
| `autonomous_flight` | 教师飞行模式（navigation / dynamic_navigation / exploration / inspection） |
| `trajectory_planner` | B 样条与多项式轨迹规划 |
| `tracking_controller` | 底层跟踪控制器 |
| `map_manager` | ESDF / 动态 / 占据地图 |
| `onboard_detector` | 障碍物检测 |
| `global_planner` | RRT / RRT* |
| `remote_control` | RViz 交互控制 |

---

## 3. 话题映射

### 3.1 采集必需（v2）

| 话题 | 类型 | 频率 | 角色 |
|---|---|---|---|
| `/camera/color/image_raw` | `sensor_msgs/Image` | ~30 Hz | **学生输入** |
| `/autonomous_flight/target_state` | `tracking_controller/Target` | ~100 Hz | **教师动作标签** |
| `/dynamicNavigation/bspline_trajectory` | `nav_msgs/Path` | ~30 Hz | **教师轨迹标签** |
| `/move_base_simple/goal` | `geometry_msgs/PoseStamped` | 事件 | ⭐ **v2 新增：目标点** |
| `/CERLAB/quadcopter/odom` | `nav_msgs/Odometry` | 30 Hz | 同步 + 目标向量换算 |
| `/CERLAB/quadcopter/pose` | `geometry_msgs/PoseStamped` | 30 Hz | yaw 提取 |

### 3.2 `tracking_controller/Target` 消息结构

```
std_msgs/Header header
uint8 type_mask
geometry_msgs/Vector3 position      # 世界系期望位置
geometry_msgs/Vector3 velocity      # 世界系期望速度 ← 用这个
geometry_msgs/Vector3 acceleration
float32 yaw                          # 期望偏航角 ← 差分得 yaw_rate
```

**动作标签**：
```
u_t = [velocity.x, velocity.y, velocity.z, wrap(yaw[t]-yaw[t-1])/dt]
（再旋转到机体系，见 20_architecture/21_data_spec.md §1）
```

### 3.3 状态与相机话题

| 话题 | 频率 | 说明 |
|---|---|---|
| `/CERLAB/quadcopter/odom_raw` / `pose_raw` / `vel_raw` / `acc_raw` | ~200+ Hz | 原始，**不用** |
| `/CERLAB/quadcopter/odom` / `pose` / `vel` / `acc` | **30 Hz** | 已限频，**用这些** |
| `/camera/depth/image_raw`、`/camera/depth/points` | ~30 Hz | 教师专用，学生不可用 |
| `/camera/color/camera_info` | ~30 Hz | 内参 |

### 3.4 教师的其他知识层输出（2026-09-07 源码扫描新增）

**当前未使用，但可作为第 5 臂（感知层）的蒸馏标签** —— 见 `20_architecture/20_architecture.md` §1.2 与 §3.2。

| 层 | 话题 | 类型 |
|---|---|---|
| 感知-静态 | `<ns>/2D_occupancy_map` | `nav_msgs/OccupancyGrid` ⭐ 最适合 dense prediction |
| | `<ns>/esdf` | `sensor_msgs/PointCloud2` |
| | `<ns>/voxel_map`, `/inflated_voxel_map`, `/explored_voxel_map` | `PointCloud2` |
| 感知-动态 | `<ns>/tracked_bboxes`, `/dynamic_bboxes` | `MarkerArray` |
| | `<ns>/history_trajectories`, `/velocity_visualizaton` | `MarkerArray` |
| | `<ns>/dynamic_point_cloud`, `/filtered_depth_cloud` | `PointCloud2` |
| | `<ns>/detected_depth_map`, `/bird_view` | `sensor_msgs/Image` |
| 全局规划 | `dynamicNavigation/rrt_path` | `nav_msgs/Path`（需开 `use_global_planner`） |
| 局部规划中间阶段 | `dynamicNavigation/poly_traj`, `/input_trajectory`, `/pwl_trajectory` | `nav_msgs/Path` |

**两个可主动查询的 service**（对 DAgger 有价值，可直接问教师而不只是回放录制）：

| Service | 用途 |
|---|---|
| `<ns>/check_pos_collision` | 查询任意位置是否碰撞 |
| `<ns>/raycast` | 射线投射 |

⚠️ 上表的 `<ns>` 需在实机确认实际命名空间；`MarkerArray` 类型的话题是**可视化格式**，若要作训练标签需从中解析几何量（或改用对应的 PointCloud2 / OccupancyGrid 话题）。

### 3.5 ⚠️ 不要用的话题

| 话题 | 原因 |
|---|---|
| `/CERLAB/quadcopter/cmd_vel` | **自主飞行时不发布**（只有键盘控制才发） |
| `/CERLAB/quadcopter/cmd_acc` | 加速度设定点，低层，难学 |
| `dynamicNavigation/poly_traj` / `pwl_trajectory` / `rrt_path` | 中间产物，不作为标签 |

### 3.5 学生部署用

| 话题 | 类型 | 说明 |
|---|---|---|
| `/CERLAB/quadcopter/cmd_vel` | `geometry_msgs/TwistStamped` | linear.x/y/z = 机体系速度，angular.z = yaw_rate |
| `/CERLAB/quadcopter/takeoff` | `std_msgs/Empty` | 起飞 |
| `/CERLAB/quadcopter/vel_mode` | `std_msgs/Bool` | 切速度模式 |

---

## 4. 数据流

```
Gazebo World
  ├── quadcopterPlugin → odom_raw (200+Hz) → throttle → odom (30Hz)
  └── camera_plugin    → /camera/color/image_raw (30Hz)   ← 学生输入
                       → /camera/depth/points              ← 教师建图用

CERLAB Teacher
  ├── dynamic_navigation_node
  │     ├── sub: /CERLAB/quadcopter/odom
  │     ├── sub: /move_base_simple/goal          ← RViz 2D Nav Goal
  │     ├── pub: /autonomous_flight/target_state (100Hz)   ← 教师动作
  │     └── pub: /dynamicNavigation/bspline_trajectory (30Hz) ← 教师轨迹
  └── tracking_controller_node
        └── pub: /CERLAB/quadcopter/cmd_acc (100Hz) → 仿真器
```

---

## 5. 启动序列

**⚠️ 顺序不能变；每个终端都要先 `conda deactivate`。**

### 终端 1 — Gazebo 仿真器

```bash
source ~/miniconda3/etc/profile.d/conda.sh && conda deactivate
source /home/l/catkin_ws/devel/setup.bash
roslaunch uav_simulator start.launch
```

启动：Gazebo 11 + 四旋翼模型（含深度相机插件）+ 话题限频节点 + TF。

**换 world**：编辑 `start.launch` 注释行（不能用命令行覆盖，见 `50_records/51_issues.md`）。
数据采集推荐 `worlds/corridor/corridor_dynamic_9_nohuman.world`。

### 终端 2 — CERLAB 教师

```bash
source ~/miniconda3/etc/profile.d/conda.sh && conda deactivate
source /home/l/catkin_ws/devel/setup.bash      # 必须先 source 这个
source /home/l/cerlab_ws/devel/setup.bash
roslaunch autonomous_flight dynamic_navigation.launch
```

收到 odom 后**自动起飞**到 1.0 m（约 2–5 秒）。

### 终端 3 — RViz（下发目标点）

```bash
source ~/miniconda3/etc/profile.d/conda.sh && conda deactivate
source /home/l/cerlab_ws/devel/setup.bash
roslaunch remote_control dynamic_navigation_rviz.launch
```

用 **2D Nav Goal** 工具点击目标 → 发布到 `/move_base_simple/goal`。

### 终端 4 — 数据采集

```bash
bash scripts/launch_collector.sh <episode_id>
```

### 封装脚本

| 脚本 | 作用 |
|---|---|
| `scripts/launch_sim.sh` | 终端 1 |
| `scripts/launch_teacher.sh` | 终端 2 |
| `scripts/launch_rviz.sh` | 终端 3 |
| `scripts/launch_collector.sh [episode_id]` | 终端 4 |
| `scripts/check_topics.sh` | 话题频率检查 |
| `scripts/view_camera.sh` | 实时查看相机画面 |

---

## 6. 话题健康检查

```bash
rostopic list

rostopic hz /camera/color/image_raw                    # 期望 ~30 Hz
rostopic hz /CERLAB/quadcopter/odom                    # 期望 ~30 Hz
rostopic hz /autonomous_flight/target_state            # 期望 ~100 Hz（给目标后）
rostopic hz /dynamicNavigation/bspline_trajectory      # 期望 ~30 Hz（给目标后）
rostopic echo /move_base_simple/goal                   # 点击 2D Nav Goal 后应有输出

rostopic hz /CERLAB/quadcopter/cmd_vel                 # 期望 0 Hz（自主飞行时不发）
rosrun image_view image_view image:=/camera/color/image_raw   # 确认画面非黑
```

---

## 7. 常用命令

```bash
# 训练（v2 目标形态：config 驱动单入口）
conda activate dbc241
python -m kd_uav.train --arm bc --capacity l

# 数据集构建
python -m kd_uav.datasets.dataset_builder --config configs/data/v2.yaml

# 数据集检查
python -m kd_uav.datasets.inspect_dataset --dataset data/processed/uav_kd_v201

# 诊断实验（R1）
python -m kd_uav.diagnostics.t1_train_set_recall
python -m kd_uav.diagnostics.t4_neighbor_label_variance

# 批量部署评估（R1-W2 建成后）
bash ros_ws/src/uav_kd_eval_tools/scripts/batch_deploy.sh <model_path> <n_runs>
```

---

## 8. 相机内参（`corridor_dynamic_9_nohuman`）

| 参数 | 值 |
|---|---|
| 分辨率 | 640 × 480 |
| HFoV | 60°（1.047198 rad） |
| fx = fy | ≈ 554.3 |
| cx, cy | 320, 240 |
| 挂载 | base_link 原点，无偏移，光轴沿机头 |
| body → camera | `x_c = -dy_b, y_c = -dz_b, z_c = dx_b` |
