# Data Schema

---

## Episode Directory Layout

Each recording session produces one episode directory:

```
data/raw/<episode_id>/
├── metadata.yaml          # written on Ctrl+C shutdown
├── rosbag/
│   └── flight.bag         # raw source-of-truth bag (all 5 topics)
├── images/
│   ├── 000000.png
│   ├── 000001.png
│   └── ...                # BGR, 640×480 (or camera native resolution)
├── ctrl_world.csv
├── ctrl_body.csv
├── traj_world.csv
├── traj_body.csv
├── odom.csv
└── pose.csv
```

---

## CSV Schemas

All CSVs share `frame_id` and `timestamp` as the first two columns.
`frame_id` is a zero-based integer counting saved frames.
`timestamp` is wall-clock time (`rospy.get_time()`) at the image callback, in seconds.

### ctrl_world.csv

Teacher velocity command in **world frame**, plus yaw_rate.

| Column       | Unit  | Source                                       |
|---|---|---|
| `frame_id`   | —     | frame index                                  |
| `timestamp`  | s     | wall clock at image callback                 |
| `vx_w`       | m/s   | `target_state.velocity.x`                   |
| `vy_w`       | m/s   | `target_state.velocity.y`                   |
| `vz_w`       | m/s   | `target_state.velocity.z`                   |
| `yaw_rate`   | rad/s | `wrap(yaw[t] - yaw[t-1]) / dt` from target_state |
| `target_yaw` | rad   | `target_state.yaw` (desired heading, world frame) |

### ctrl_body.csv

Same signals rotated into **UAV body frame** (x-forward, y-left, z-up).

| Column       | Unit  | Derivation                                   |
|---|---|---|
| `vx_b`       | m/s   | `vx_w·cos(yaw) + vy_w·sin(yaw)`             |
| `vy_b`       | m/s   | `-vx_w·sin(yaw) + vy_w·cos(yaw)`            |
| `vz_b`       | m/s   | same as `vz_w` (z-axis unchanged)            |
| `yaw_rate`   | rad/s | same scalar as ctrl_world                    |
| `target_yaw` | rad   | same as ctrl_world                           |

`yaw` used for rotation is the actual UAV yaw extracted from `/CERLAB/quadcopter/pose`.

**BC / Control KD training label:** `[vx_b, vy_b, vz_b, yaw_rate]` from `ctrl_body.csv`.

### traj_world.csv

K future B-spline waypoints in **world frame** (absolute coordinates).
Column pattern: `x0, y0, z0, x1, y1, z1, ..., x{K-1}, y{K-1}, z{K-1}`.

| Column       | Unit | Source                                        |
|---|---|---|
| `x{i}`       | m    | `bspline_trajectory.poses[i].position.x`     |
| `y{i}`       | m    | `bspline_trajectory.poses[i].position.y`     |
| `z{i}`       | m    | `bspline_trajectory.poses[i].position.z`     |

If the trajectory has fewer than K poses, the last available pose is repeated.
Default K = 10.

### traj_body.csv

Same waypoints in **UAV body frame** (relative coordinates).
Column pattern: `dx0, dy0, dz0, dx1, dy1, dz1, ..., dx{K-1}, dy{K-1}, dz{K-1}`.

| Column       | Unit | Derivation                                    |
|---|---|---|
| `dx{i}`      | m    | body-x component of (wp_i − uav_pos)         |
| `dy{i}`      | m    | body-y component of (wp_i − uav_pos)         |
| `dz{i}`      | m    | `wp_i.z − uav_z` (z same in both frames)     |

**Trajectory KD training label:** `[[dx0,dy0,dz0], ..., [dx{K-1},dy{K-1},dz{K-1}]]` from `traj_body.csv`.

### odom.csv

| Column | Unit | Source                                        |
|---|---|---|
| `x/y/z` | m  | `odom.pose.pose.position`                    |
| `vx/vy/vz` | m/s | `odom.twist.twist.linear`                |

### pose.csv

| Column      | Unit | Source                                         |
|---|---|---|
| `x/y/z`     | m    | `pose.pose.position`                          |
| `qx/qy/qz/qw` | —  | `pose.pose.orientation`                      |
| `yaw`       | rad  | extracted via `atan2(2(qw·qz+qx·qy), 1-2(qy²+qz²))` |

---

## metadata.yaml Schema

```yaml
episode_id:       ep_20260628_120000
world:            corridor_dynamic_9_nohuman
teacher_system:   CERLAB Autonomous Flight / dynamic_navigation
student_used:     false
record_type:      teacher_demonstration
K:                10
sync_tolerance_s: 1.0
downsample:       false
downsample_step:  "N/A"
camera_topic:     /camera/color/image_raw
control_topic:    /autonomous_flight/target_state
trajectory_topic: /dynamicNavigation/bspline_trajectory
odom_topic:       /CERLAB/quadcopter/odom
pose_topic:       /CERLAB/quadcopter/pose
total_frames:     1834
result:
  success:   true          # fill in after visual inspection
  collision: false
  notes:     "Stable teacher run."
```

---

## Coordinate Frames

```
World frame (ROS standard):
  x — forward (roughly East in corridor world)
  y — left
  z — up

UAV body frame:
  x — forward (UAV nose direction)
  y — left
  z — up

Rotation: world → body
  [vx_b]   [ cos(yaw)  sin(yaw)  0 ] [vx_w]
  [vy_b] = [-sin(yaw)  cos(yaw)  0 ] [vy_w]
  [vz_b]   [    0          0     1 ] [vz_w]

where yaw = angle from world-x to body-x, counterclockwise.
```

---

## 处理后数据集 Schema（Phase 4，已构建）

### 目录结构

```
data/processed/<dataset_name>/
├── manifest.csv           # 所有 sample 的索引，含 split/valid_motion 标记
├── train_index.csv        # 训练集 sample_id 列表
├── val_index.csv          # 验证集 sample_id 列表
├── test_index.csv         # 测试集 sample_id 列表
├── samples/
│   ├── 000000.npz         # 单个 sample（见下方 npz schema）
│   ├── 000001.npz
│   └── ...
└── dataset_summary.json   # 数据集全局统计信息
```

**Split 规则：** 按 episode 划分 train/val/test，不做随机帧分割。
原因：相邻帧几乎完全相同，随机分割会导致 val 分数虚高。

---

### npz 文件 Schema（每个 sample）

| 键名 | 形状 | dtype | 说明 |
|---|---|---|---|
| `image` | (H, W, 3) | uint8 | BGR 原始分辨率图像（由 camera 原始话题保存） |
| `ctrl_world` | (5,) | float32 | [vx_w, vy_w, vz_w, yaw_rate, target_yaw] |
| `ctrl_body` | (4,) | float32 | [vx_b, vy_b, vz_b, yaw_rate]，**BC / Control KD 训练标签** |
| `traj_world` | (K, 3) | float32 | K 个未来路点（世界坐标，绝对值）[x, y, z] |
| `traj_body` | (K, 4) | float32 | K 个未来路点（机体坐标，相对值）[dx, dy, dz, dyaw] |
| `pose` | (4,) | float32 | [x, y, z, yaw]，UAV 当前世界坐标系位姿 |
| `odom` | (6,) | float32 | [x, y, z, vx, vy, vz] |
| `episode_id` | scalar | str | episode 名称（如 "ep_test_003"） |
| `frame_id` | scalar | int | 帧编号（episode 内从 0 开始） |
| `timestamp` | scalar | float64 | 图像回调时的 wall-clock 时间（秒） |
| `valid_motion` | scalar | bool | True = 速度 > 0.05 m/s 或 \|yaw_rate\| >= 0.3 rad/s（原地转保留）；False = 静止帧（保留但可过滤） |
| `valid_traj` | scalar | bool | True = 首个路点在前方（dx0>=0）且轨迹未退化（见下）；False = 应在 Trajectory/Joint KD 训练时过滤 |

**注意：**
- `traj_body[:, 3]`（dyaw）**全为 0**（has_dyaw=False）。
  原因：bspline_trajectory 中 PoseStamped.orientation 未被存入 traj_body.csv，需从 rosbag 重新提取才可获得真实 dyaw。
- `ctrl_body[3]`（yaw_rate）在走廊直线飞行时全为 0，因无转弯，属正常现象。
- **`valid_traj` 退化路点问题（2026-07-01 发现并修复）**：UAV 原地旋转（不平移）时 bspline_trajectory 不会重新发布，导致该时段全部 K 个路点冻结在 UAV 自身位置附近（幅值≈0）。`dataset_builder.py` 已加入 `traj_magnitude_threshold`（默认 0.02，检查 `sum(|dx_i|+|dy_i|)` over K）过滤此类帧，`filter_reason` 标记为 `traj_degenerate`。

---

### manifest.csv Schema

| 列名 | 说明 |
|---|---|
| `sample_id` | 全局唯一 sample 编号（如 "000000"） |
| `episode_id` | 所属 episode |
| `frame_id` | episode 内帧编号 |
| `timestamp` | wall-clock 时间戳（秒） |
| `sample_path` | npz 文件绝对路径 |
| `split` | "train" / "val" / "test" |
| `valid_motion` | True / False |
| `valid_traj` | True / False（见上方 npz schema 的退化路点说明） |
| `filter_reason` | 若 valid_motion/valid_traj=False，记录原因：`speed_below_threshold` / `traj_behind_uav` / `traj_degenerate`（可组合，逗号分隔） |

---

### dataset_summary.json Schema

| 字段 | 说明 |
|---|---|
| `dataset_name` | 数据集名称（如 "uav_kd_debug_v000"） |
| `dataset_type` | "debug" / "train" / "full" |
| `total_samples` | 总 sample 数 |
| `train_samples` | 训练集 sample 数 |
| `val_samples` | 验证集 sample 数 |
| `test_samples` | 测试集 sample 数 |
| `episodes` | episode 列表 |
| `K` | 轨迹路点数 |
| `has_dyaw` | False（当前 dyaw 全为 0） |
| `image_resolution` | 原始图像分辨率（如 [480, 640]） |
| `valid_motion_count` | valid_motion=True 的 sample 数 |
| `valid_motion_ratio` | valid_motion 比例 |
| `valid_traj_count` | valid_traj=True 的 sample 数 |
| `valid_traj_ratio` | valid_traj 比例 |
| `filter_config` | `{speed_threshold, yaw_threshold, traj_magnitude_threshold}` |
| `ctrl_body_mean` | ctrl_body 各维均值 [4] |
| `ctrl_body_std` | ctrl_body 各维标准差 [4] |
| `traj_body_mean` | traj_body 均值 [K, 4] |
| `traj_body_std` | traj_body 标准差 [K, 4] |
| `warning` | 数据质量警告（如 dyaw=0） |

---

### 已构建数据集记录

| 数据集 | 类型 | Episodes（train / val） | Samples（train / val） | valid_traj | 用途 |
|---|---|---|---|---|---|
| `uav_kd_debug_v000` | debug | ep_test_003 | 790 | — | pipeline 验证，不用于正式训练 |
| `uav_kd_v001` | train | 003+004+007 / 005 | 6376 / 569 | — | BC baseline 正式训练集；val 100% 直飞，无旋转样本 |
| `uav_kd_v002` | train | 003+004+007+008+009 / 005+006 | 10170 / 1531 | 9980/11701 (85.3%) | 并入未使用的 006/008/009，val 混合直飞+旋转（~11.4%旋转帧），修复 traj 退化过滤 |

---

### UavKdDataset PyTorch 接口

```python
from kd_uav.datasets.uav_kd_dataset import UavKdDataset

dataset = UavKdDataset(
    processed_dir="data/processed/uav_kd_debug_v000",
    split="train",
    image_size=(224, 224),   # BGR→RGB，resize，归一化到 [0,1]
    label="both",            # 'ctrl_body' | 'traj_body' | 'both'
    filter_static=True,      # 过滤 valid_motion=False 的静止帧
)

item = dataset[0]
# item['image']      : Tensor(3, 224, 224)  float32  RGB  [0, 1]
# item['ctrl_body']  : Tensor(4,)           float32  [vx_b, vy_b, vz_b, yaw_rate]
# item['traj_body']  : Tensor(K, 4)         float32  [dx, dy, dz, dyaw=0]
# item['pose']       : Tensor(4,)           float32  [x, y, z, yaw]
# item['sample_id']  : str
# item['episode_id'] : str
```

运行环境：`conda activate dbc241`（需要 torch + cv2 + yaml）
