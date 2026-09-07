# 数据规格与采集协议

> 层级：架构层。**数据格式变更时修改本文件。**
> 版本：v2（2026-09-07）。v1 见 `_archive/2026-09_v1/03_data_schema.md`、`06_data_collection_protocol.md`
> ⚠️ v2 新增**目标条件化字段**，v1 的 9 个 episode 无这些字段，**不能用于 v2 训练**（但保留用于 R1 诊断）

---

## 1. 坐标系约定

```
世界系（ROS 标准）：       机体系（UAV body）：
  x — 前（走廊方向）         x — 机头方向
  y — 左                     y — 左
  z — 上                     z — 上

世界 → 机体旋转：
  [vx_b]   [ cos(yaw)  sin(yaw)  0 ] [vx_w]
  [vy_b] = [-sin(yaw)  cos(yaw)  0 ] [vy_w]
  [vz_b]   [    0         0      1 ] [vz_w]

相机系映射：x_c = -dy_b,  y_c = -dz_b,  z_c = dx_b
```

**为什么标签用机体系**：世界系标签会让学生记住地图上的绝对位置，而不是学习基于视觉的导航行为。机体系标签能跨起始位置泛化。

### 相机内参（`corridor_dynamic_9_nohuman`）

| 参数 | 值 |
|---|---|
| 分辨率 | 640 × 480 |
| HFoV | 60°（1.047198 rad） |
| fx = fy | ≈ 554.3 |
| cx, cy | 320, 240 |
| 挂载 | base_link 原点，无偏移，光轴沿机头方向 |

---

## 2. Episode 目录结构

```
data/raw/<episode_id>/
├── metadata.yaml
├── rosbag/flight.bag          原始 bag（全部话题，source of truth）
├── images/000000.png ...      BGR, 640×480
├── ctrl_world.csv
├── ctrl_body.csv
├── traj_world.csv
├── traj_body.csv
├── goal.csv                   ⭐ v2 新增
├── odom.csv
└── pose.csv
```

所有 CSV 前两列均为 `frame_id`（从 0 计的整数）和 `timestamp`（图像回调时的 wall-clock 秒）。

---

## 3. CSV Schema

### 3.1 `goal.csv` ⭐ v2 新增

| 列 | 单位 | 来源 |
|---|---|---|
| `goal_id` | — | 本帧生效的目标点编号（一个 episode 内可切换多个目标） |
| `goal_x_w`, `goal_y_w`, `goal_z_w` | m | 目标点世界坐标（来自 `/move_base_simple/goal`） |
| `goal_dx_b`, `goal_dy_b`, `goal_dz_b` | m | **机体系相对目标向量 —— 学生的目标输入** |
| `goal_dist` | m | 到目标的欧氏距离 |
| `goal_dir_x`, `goal_dir_y`, `goal_dir_z` | — | 归一化方向向量（Loquercio 式备选输入） |
| `goal_reached` | bool | 本帧是否已到达该目标 |

**换算**：`goal_d_b = R(yaw)ᵀ · (goal_w − uav_pos_w)`，`R(yaw)` 取自 `pose.csv` 的 yaw。

### 3.2 `ctrl_world.csv` / `ctrl_body.csv`

| 列 | 单位 | 来源 / 推导 |
|---|---|---|
| `vx_w, vy_w, vz_w` | m/s | `target_state.velocity` |
| `vx_b` | m/s | `vx_w·cos(yaw) + vy_w·sin(yaw)` |
| `vy_b` | m/s | `-vx_w·sin(yaw) + vy_w·cos(yaw)` |
| `vz_b` | m/s | 同 `vz_w` |
| `yaw_rate` | rad/s | `wrap(yaw[t] − yaw[t−1]) / dt`，来自 `target_state.yaw` |
| `target_yaw` | rad | `target_state.yaw` |

**动作 / 执行层标签**：`[vx_b, vy_b, vz_b, yaw_rate]` 取自 `ctrl_body.csv`。

⚠️ `yaw_rate` 不是独立话题，必须由连续 yaw 差分得到，建议加滑窗滤波降噪。

### 3.3 `traj_world.csv` / `traj_body.csv`

- 世界系列名：`x0,y0,z0, ..., x{K-1},y{K-1},z{K-1}`
- 机体系列名：`dx0,dy0,dz0, ..., dx{K-1},dy{K-1},dz{K-1}`
- K = 10；若轨迹路点不足 K，重复最后一个
- **规划层标签**：`traj_body.csv`

⚠️ **dyaw 缺失**：`bspline_trajectory` 的 `PoseStamped.orientation` 未存入 CSV，v1 数据中 dyaw 恒为 0。**v2 采集时应一并记录朝向**（若不需要可显式声明放弃）。

### 3.4 `odom.csv` / `pose.csv`

| 文件 | 列 |
|---|---|
| `odom.csv` | `x, y, z, vx, vy, vz` |
| `pose.csv` | `x, y, z, qx, qy, qz, qw, yaw` |

`yaw = atan2(2(qw·qz + qx·qy), 1 − 2(qy² + qz²))`

---

## 4. 处理后数据集

```
data/processed/<dataset_name>/
├── manifest.csv
├── train_index.csv / val_index.csv / test_index.csv
├── samples/000000.npz ...
└── dataset_summary.json
```

### 4.1 npz Schema

| 键 | 形状 | dtype | 说明 |
|---|---|---|---|
| `image` | (H,W,3) | uint8 | BGR 原始分辨率 |
| `ctrl_body` | (4,) | float32 | **动作标签** `[vx_b, vy_b, vz_b, yaw_rate]` |
| `ctrl_seq_body` | (H_max,4) | float32 | ⭐ v2 新增：H 步控制序列标签（滑窗，不跨 episode） |
| `traj_body` | (K,3) | float32 | **规划层标签** |
| `goal_body` | (3,) | float32 | ⭐ v2 新增：**机体系相对目标向量（学生输入）** |
| `goal_dist` | scalar | float32 | ⭐ v2 新增 |
| `pose` | (4,) | float32 | `[x, y, z, yaw]` |
| `odom` | (6,) | float32 | `[x, y, z, vx, vy, vz]` |
| `episode_id` | scalar | str | |
| `frame_id` | scalar | int | episode 内帧号 |
| `timestamp` | scalar | float64 | |
| `valid_motion` | scalar | bool | 速度 > 0.05 m/s 或 \|yaw_rate\| ≥ 0.3（保留原地转） |
| `valid_traj` | scalar | bool | `dx0 ≥ 0` 且轨迹未退化 |
| `is_turning` | scalar | bool | ⭐ v2 新增：\|yaw_rate\| > 0.15，用于平衡采样与分层统计 |

### 4.2 `manifest.csv`

| 列 | 说明 |
|---|---|
| `sample_id` | 全局唯一编号 |
| `episode_id` / `frame_id` / `timestamp` | |
| `sample_path` | npz 路径 |
| `split` | train / val / test |
| `valid_motion` / `valid_traj` / `is_turning` | 过滤与分层标记 |
| `goal_id` | ⭐ v2 新增 |
| `filter_reason` | `speed_below_threshold` / `traj_behind_uav` / `traj_degenerate`（逗号分隔） |

### 4.3 划分规则

**按 episode 划分，绝不随机分帧。** 相邻帧几乎完全相同，随机分帧会让验证分数虚高。

### 4.4 已知的退化轨迹问题（v1 修复，v2 继承）

UAV 原地旋转（不平移）时 `bspline_trajectory` 不重新发布 → 该时段 K 个路点冻结在 UAV 自身位置附近（幅值 ≈ 0）。
**修复**：`dataset_builder.py` 的 `traj_magnitude_threshold`（默认 0.02，检查 `Σ(|dx_i|+|dy_i|)` over K），`valid_traj = (dx0 ≥ 0) AND 未退化`。
影响主要集中在旋转密集的 episode。对 BC/CtrlKD 无影响（动作标签本身有效），对 TrajKD/JointKD 训练需过滤。

---

## 5. v2 采集协议

> ⚠️ **本节的具体参数待 R1 诊断结论确定**（见 `40_experiments/41_diagnostics.md`）。诊断结果直接决定要采什么数据。

### 5.1 诊断结论 → 采集策略的映射

| 诊断结论 | 采集策略 |
|---|---|
| M1（类别不平衡）为主 | **刻意采集转向密集的 episode**：多设岔路目标、S 形路线、往返掉头。目标转向帧占比 ≥ 30% |
| M2（标签多峰）为主 | 目标点记录是必需的；数据量不必大，但**目标点覆盖度**要高（同一走廊配多个不同目标） |
| M3（梯度干扰）为主 | 与采集无关，是 loss 权重问题 |
| 混合（最可能） | 三条合并执行 |

### 5.2 采集流程（固定不变的部分）

1. 启动顺序见 `50_records/52_environment.md`（Gazebo → CERLAB teacher → RViz）
2. 等待自动起飞完成（约 2–5 秒，起飞高度 1.0 m）
3. 用 RViz 的 **2D Nav Goal** 下发目标点 → **同时被采集节点记录**
4. 一个 episode 内可连续下发多个目标点，`goal_id` 递增
5. Ctrl+C 结束，写出 `metadata.yaml`

### 5.3 每个 episode 的质检清单

| 检查项 | 判据 |
|---|---|
| 图像非黑、无缺帧 | 抽查 10 帧 |
| `goal.csv` 字段齐全 | `goal_dx_b` 非全零，`goal_id` 有切换 |
| `ctrl_body` 起飞后非零 | |
| 时间戳连续 | 相邻帧间隔稳定 |
| `valid_traj` 比例 | > 85% |
| 转向帧占比 | 达到协议目标 |
| 碰撞 / 失败标记 | 记入 `metadata.yaml` |

### 5.4 `metadata.yaml`

```yaml
episode_id:       ep_v2_001
date:             2026-XX-XX
world:            corridor_dynamic_9_nohuman
teacher_system:   CERLAB Autonomous Flight / dynamic_navigation
record_type:      teacher_demonstration
student_used:     false
K:                10
H_max:            10            # 控制序列标签的最大步数
sync_tolerance_s: 1.0
camera_topic:     /camera/color/image_raw
control_topic:    /autonomous_flight/target_state
trajectory_topic: /dynamicNavigation/bspline_trajectory
goal_topic:       /move_base_simple/goal      # v2 新增
odom_topic:       /CERLAB/quadcopter/odom
pose_topic:       /CERLAB/quadcopter/pose
total_frames:     0
n_goals:          0             # v2 新增：本 episode 下发的目标点数
turning_ratio:    0.0           # v2 新增：|yaw_rate|>0.15 的帧占比
result:
  success:   true
  collision: false
  notes:     ""
```

---

## 6. Legacy 数据（v1，仅用于诊断）

| 数据集 | train / val | Episodes | 用途 |
|---|---|---|---|
| `uav_kd_v001` | 6376 / 569 | 003+004+007 / 005 | ⛔ 不用于 v2 训练 |
| `uav_kd_v002` | 10170 / 1531 | 003+004+007+008+009 / 005+006 | ✅ **R1 诊断实验（T1/T4）使用** |
| `uav_kd_red_wp_v001` | 3040 / 475 | — | ⛔ SelfRedWP 方向已废弃 |
| `uav_kd_debug_v000/v001` | — | — | ⛔ 仅 pipeline 验证 |

**v1 的 9 个 raw episode 与 5 个训练好的模型必须保留**，R1 诊断直接依赖它们。详见 `40_experiments/41_diagnostics.md`。

### v1 数据的 episode 统计

| Episode | 帧数 | 转向帧占比 | 状态 |
|---|---|---|---|
| ep_test_001 / 002 | 864 / 875 | — | ❌ traj 无效（早期 bug） |
| ep_test_003 | 1013 | ~0% | ✅ 直飞 |
| ep_test_004 | 958 | ~0% | ✅ 直飞 |
| ep_test_005 | 569 | ~0% | ✅ 直飞 |
| ep_test_006 | 962 | 18.2% | ✅ |
| ep_test_007 | 4405 | 22.9% | ✅ 最长 |
| ep_008 | 2457 | 4.2% | ✅ |
| ep_009 | 1337 | 6.2% | ✅ 含一段原地转 |
