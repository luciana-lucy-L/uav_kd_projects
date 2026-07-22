# Experiment Log

---

## 2026-06-29 — Phase 1: CERLAB Teacher Autonomous Navigation Verified

### Goal
Reproduce CERLAB autonomous navigation in Gazebo and confirm stable execution before data collection.

### Environment
- OS: Ubuntu 20.04, ROS Noetic, Gazebo 11.15.1
- World: `corridor_dynamic_9.world` (9 dynamic box obstacles)
- Branch: `autonomous_flight` on `simulation`

### Launch Sequence (confirmed working)
```bash
# Terminal 1
bash /home/l/uav_kd_project/scripts/launch_sim.sh

# Terminal 2
bash /home/l/uav_kd_project/scripts/launch_rviz.sh

# Terminal 3
bash /home/l/uav_kd_project/scripts/launch_teacher.sh
```
Then: send **2D Nav Goal** in RViz.

### Result
- UAV took off to 1.0 m automatically ✓
- Trajectory generated and executed successfully ✓
- UAV navigated to goal while avoiding obstacles ✓
- No crash, no Gazebo freeze ✓
- System stable end-to-end ✓

### Issues Encountered and Fixed
| Issue | Cause | Fix |
|---|---|---|
| `spawn_model` died: `No module named 'yaml'` | conda base active, Python 3.13 intercepting ROS Python 3.8 | `conda deactivate` before sourcing ROS — handled by wrapper scripts |
| `RLException: cannot override arg world_name` | `start.launch` uses `value=` not `default=` for world_name | Removed `world_name:=` argument from `launch_sim.sh` |
| UAV not taking off | CERLAB teacher not launched yet | Must run `launch_teacher.sh` — takeoff is automatic |

### Confirmed Active Topics (live measured — 2026-06-28)
| Topic | Type | Measured Hz | Notes |
|---|---|---|---|
| `/camera/color/image_raw` | `sensor_msgs/Image` | **30.0** | Stable, std dev ~0.011s |
| `/CERLAB/quadcopter/odom` | `nav_msgs/Odometry` | **29.412** | Rock-solid, std dev 0.000s (exact timer) |
| `/CERLAB/quadcopter/pose` | `geometry_msgs/PoseStamped` | **29.412** | Same throttle node as odom |
| `/autonomous_flight/target_state` | `tracking_controller/Target` | **~200** | Source says 100Hz; actual sim runs at 200Hz |
| `dynamicNavigation/bspline_trajectory` | `nav_msgs/Path` | **~30.3** | 36 poses per message during active flight |

### Sample target_state values (UAV actively flying)
```
velocity.x = -0.786 m/s
velocity.y = -0.149 m/s
velocity.z = -0.000 m/s
yaw        =  3.054 rad
position.z =  0.998 m  (hovering at ~1m)
```

### bspline_trajectory content
- 36 `position` entries per message during active flight
- Represents the upcoming B-spline path in world frame

### Phase 1 Verdict: PASS ✓
All 5 required topics confirmed active and at expected rates.

### Next Step
Phase 3 — Test `uav_kd_data_collector` with a live teacher run and collect first episode.

---

## 2026-06-29 — Phase 3: uav_kd_data_collector Built and Compiled

### Goal
Build a ROS package to record synchronized teacher demonstrations for knowledge distillation training.

### Files Created
| File | Purpose |
|---|---|
| `ros_ws/src/uav_kd_data_collector/package.xml` | ROS package declaration |
| `ros_ws/src/uav_kd_data_collector/CMakeLists.txt` | catkin build config |
| `ros_ws/src/uav_kd_data_collector/launch/record_teacher.launch` | Launch collector + rosbag record |
| `ros_ws/src/uav_kd_data_collector/scripts/record_teacher_data.py` | Main collector node |
| `scripts/launch_collector.sh` | Wrapper: pre-create dirs, source workspaces, launch |
| `docs/03_data_schema.md` | Full schema for all CSVs, metadata, coordinate frames |

### Build Result
```bash
cd /home/l/uav_kd_project/ros_ws
catkin_make   # after sourcing catkin_ws + cerlab_ws
# → Build succeeded, 1 package processed
rospack find uav_kd_data_collector  # → found
```

### Design Decisions
- **rosbag**: recorded by `rosbag record` node in launch file (raw source of truth)
- **ctrl**: world frame + body frame both saved; yaw_rate from consecutive `target_state.yaw` diffs
- **traj**: world frame (absolute) + body frame (relative to UAV) both saved; K=10 default
- **sync**: wall-clock freshness check (1s threshold); `bspline_trajectory` gates saving (only saves after 2D Nav Goal sent)
- **downsample**: disabled by default; configurable via `downsample:=true downsample_step:=3`

### Next Step
Phase 3 live test — Build completed. CERLAB sim needed to verify topic naming.

### TODO Before First Data Collection
1. Launch sim → teacher → rviz (see launch_sim/teacher/rviz.sh)
2. Send 2D Nav Goal
3. In a 4th terminal: `bash scripts/launch_collector.sh ep_20260629_001`
4. After Ctrl+C: inspect `data/raw/ep_20260629_001/`
5. Fill in `metadata.yaml` result fields
6. Record frame count and visual inspection result here

### Collector Launch Command (when sim is running)
```bash
# Terminal 4
bash /home/l/uav_kd_project/scripts/launch_collector.sh ep_20260629_001

# Or with custom episode ID:
bash /home/l/uav_kd_project/scripts/launch_collector.sh ep_corridor_001
```

### Expected Output Structure (after one episode)
```
data/raw/ep_20260629_001/
├── metadata.yaml
├── rosbag/flight.bag
├── images/000000.png ... (≈30 frames/sec × flight_duration)
├── ctrl_world.csv      # vx_w, vy_w, vz_w, yaw_rate, target_yaw
├── ctrl_body.csv       # vx_b, vy_b, vz_b, yaw_rate, target_yaw
├── traj_world.csv      # 10 future waypoints (absolute world coords)
├── traj_body.csv       # 10 future waypoints (body frame, relative)
├── odom.csv
└── pose.csv
```

### Verification Checklist (after first episode)
- [ ] `images/` not empty, first frame not black
- [ ] `ctrl_world.csv`: vx_w non-zero during flight
- [ ] `ctrl_body.csv`: vx_b ≈ positive during forward flight
- [ ] `traj_world.csv`: x0/y0/z0 values are near UAV position
- [ ] `traj_body.csv`: dx0 ≈ positive (next waypoint is ahead)
- [ ] `rosbag/flight.bag` exists and `rosbag info flight.bag` shows all 5 topics
- [ ] `metadata.yaml` written (not missing)

### Episode Summary

| Episode | Frames | rosbag | traj_body dx0 | Status |
|---|---|---|---|---|
| ep_test_001 | 864 | 769 MB | BUG: all points behind UAV (−1.25m) | traj labels invalid |
| ep_test_002 | 875 | 776 MB | BUG: ±0.08m oscillating (50% negative) | traj labels invalid |
| ep_test_003 | 790 | 1.4 GB | FIXED: min=0.150m, 0/790 negative ✅ | **VALID** |

### Phase 3 Status: COMPLETE ✅
Build: DONE | Live test: DONE (3 episodes) | Trajectory fix: VERIFIED

### ep_test_003 Verified Values
- traj_body dx0: min=0.150m, max=0.287m (all positive, lookahead=0.15m working)
- traj_body dx0~dx4 means: 0.208 → 0.336 → 0.467 → 0.600 → 0.733 (correctly increasing)
- ctrl_body vx_b: ~0.7 m/s (UAV flying forward in body frame)
- All 6 CSVs: 791 rows (790 frames + header), perfectly synchronized

### Bug History
| Bug | Episodes Affected | Fix |
|---|---|---|
| `traj` takes poses[0..K-1] (all behind UAV) | ep_test_001 | Added closest-point search |
| Closest point at UAV feet (±0.08m, 50% negative) | ep_test_002 | Changed to `traj_lookahead=0.15m` scan |
| Fixed | ep_test_003 ✅ | — |

### Pending Phase 3 Status
Build: DONE | Live test: DONE | Fix verified: ep_test_003 ✅

---

---

## 2026-06-29 — Phase 4：Debug Dataset Pipeline 构建与验证

### 目标
构建 debug 级处理数据集，验证从原始 CSV/图像到 PyTorch DataLoader 的完整 pipeline，为正式 `uav_kd_v001` 训练集奠定基础。

### 使用数据
| Episode | 帧数 | traj_body 状态 | 是否有效 |
|---|---|---|---|
| ep_test_003 | 790 | min dx0=0.150m，0 帧负值 ✅ | **有效** |
| ep_test_001 | 864 | 所有路点在 UAV 身后（bug） | traj 无效，不用 |
| ep_test_002 | 875 | dx0 振荡 ±0.08m（bug） | traj 无效，不用 |

### 新建文件
| 文件 | 用途 |
|---|---|
| `kd_uav/datasets/dataset_builder.py` | 读取原始 CSV + 图像 → 写入 .npz + manifest.csv + dataset_summary.json |
| `kd_uav/datasets/uav_kd_dataset.py` | PyTorch Dataset 类，DataLoader 接口 |
| `kd_uav/datasets/inspect_dataset.py` | 统计检查 + 保存标注图像 + 测试 DataLoader |

### 构建命令
```bash
conda activate dbc241
cd /home/l/uav_kd_project

python kd_uav/datasets/dataset_builder.py \
    --raw_dir data/raw \
    --output_dir data/processed/uav_kd_debug_v000 \
    --episodes ep_test_003 \
    --K 10 \
    --dataset_type debug
```

### 检查命令
```bash
python kd_uav/datasets/inspect_dataset.py \
    --processed_dir data/processed/uav_kd_debug_v000 \
    --split train \
    --num_samples 10
```

### 构建结果

| 项目 | 值 |
|---|---|
| 数据集路径 | `data/processed/uav_kd_debug_v000/` |
| 总 samples | 790 |
| valid_motion=True | 753 / 790（95.3%） |
| 静止帧（valid_motion=False） | 37 帧（保留，filter_static=True 时过滤） |
| split | 全部归 train（debug 模式，单 episode） |
| K | 10 |
| has_dyaw | **False**（traj_body dyaw 列全为 0） |

### DataLoader 验证结果

| 张量 | 形状 | dtype | 说明 |
|---|---|---|---|
| `image` | (4, 3, 224, 224) | float32 | BGR→RGB，resize 到 224×224，归一化 [0,1] |
| `ctrl_body` | (4, 4) | float32 | [vx_b, vy_b, vz_b, yaw_rate] |
| `traj_body` | (4, 10, 4) | float32 | [dx, dy, dz, dyaw=0] × 10 个路点 |
| `pose` | (4, 4) | float32 | [x, y, z, yaw] |

✅ DataLoader batch_size=4 正常运行，无报错。

### 数据统计（ep_test_003，走廊直线飞行）

**ctrl_body：**
| 维度 | min | max | mean | std |
|---|---|---|---|---|
| vx_b | ~0.00 | ~1.20 | ~0.65 | — |
| vy_b | — | — | ~0.00 | — |
| vz_b | — | — | ~0.00 | — |
| yaw_rate | 0.0 | 0.0 | 0.0 | 0.0 |

**说明：**
- `yaw_rate=0`：走廊直线飞行，无转弯，属正常现象。有弯道的 episode 会自然出现非零值。
- `traj_body dyaw=0`：bspline_trajectory 的 PoseStamped.orientation 未被存入 CSV（has_dyaw=False）。如需真实 dyaw，需从 rosbag 重新提取。

### 环境确认（2026-06-29）

| 环境 | torch | cv2 | yaml | 用途 |
|---|---|---|---|---|
| `dbc241` | 2.4.1+cpu | 4.11.0 | ✓ | **官方训练/检查环境** |
| `dbc_bc` | ✗ | ✓ | ✓ | 仅可运行 dataset_builder.py（无 torch） |

**规则：** `inspect_dataset.py`、所有模型训练 → 一律使用 `dbc241`。

### Phase 4 结论

| 检查项 | 状态 |
|---|---|
| dataset_builder 构建无报错 | ✅ |
| manifest.csv / dataset_summary.json 写入正确 | ✅ |
| npz 文件 keys 齐全 | ✅ |
| UavKdDataset 加载正常 | ✅ |
| DataLoader batch 形状正确 | ✅ |
| 标注图像保存到 results/figures/dataset_inspection/ | ✅ |
| dyaw=0（已知限制，已记录） | ⚠️ |

### Phase 4 结论：PASS ✅
Debug dataset pipeline 端到端验证通过。`uav_kd_debug_v000` 仅供 pipeline 验证，**不用于正式训练**。

### 下一步
1. 用仿真环境多跑 5–10 个 episode（含弯道，以获得非零 yaw_rate）
2. 用 `dataset_builder.py` 构建正式数据集 `uav_kd_v001`（episode 级别 8:1:1 train/val/test 划分）
3. 实现 BC baseline 模型 + 训练脚本（`kd_uav/models/` + `kd_uav/train/train_bc.py`）

---

## 2026-06-29 — Phase 5: 正式数据集构建（uav_kd_v001）

### 目标
在 Phase 4 debug pipeline 验证完成后，用更多有效 episode 构建正式训练数据集。

### 使用 Episode

| Episode | 帧数 | 用途 | 说明 |
|---|---|---|---|
| ep_test_001 | 864 | ❌ 排除 | traj_body bug：所有路点在 UAV 身后 |
| ep_test_002 | 875 | ❌ 排除 | traj_body bug：dx0 振荡 ±0.08m |
| ep_test_003 | 1013 | ✅ train | 首个修复后有效 episode |
| ep_test_004 | 958 | ✅ train | 有效 |
| ep_test_005 | 569 | ✅ val | 有效，用作验证集 |
| ep_test_006 | 962 | ❌ 排除 | 未纳入 v001（保留备用）|
| ep_test_007 | 4406 | ✅ train | 最长 episode，1帧图像缺失（4405张图）|

### 构建命令
```bash
conda activate dbc241
cd /home/l/uav_kd_project
python kd_uav/datasets/dataset_builder.py \
    --raw_dir data/raw \
    --output_dir data/processed/uav_kd_v001 \
    --episodes ep_test_003 ep_test_004 ep_test_007 ep_test_005 \
    --K 10 \
    --dataset_type train \
    --val_episodes ep_test_005
```

### 数据集统计

| 项目 | 值 |
|---|---|
| 数据集路径 | `data/processed/uav_kd_v001/` |
| 总样本数 | 6945 |
| train 样本 | 6376（ep_test_003/004/007） |
| val 样本 | 569（ep_test_005） |
| test 样本 | 0 |
| K（未来路点数）| 10 |
| 图像分辨率 | 480 × 640 × 3 |
| has_dyaw | False（bspline_trajectory dyaw 未存储）|

### ctrl_body 统计（全量）

| 维度 | mean | std | min | max |
|---|---|---|---|---|
| vx_b | 0.386 | 0.361 | -0.449 | 1.088 |
| vy_b | 0.004 | 0.194 | -0.734 | 0.709 |
| vz_b | 0.000 | 0.002 | -0.009 | 0.008 |
| yaw_rate | -0.003 | 0.314 | -3.000 | 2.500 |

### 障碍回避信号分析（7 episodes 合计 7907 帧）

| 指标 | 值 |
|---|---|
| \|vy_b\| > 0.1（侧向回避） | 34.8% 帧 |
| \|vy_b\| > 0.3（强侧向） | 14.4% 帧 |
| \|yaw_rate\| > 0.05（转弯）| 15.0% 帧 |
| vx_b < 0.1（几乎停止，ep_007）| 56.6% 帧 |

### 结论
- 数据集包含有意义的障碍回避信号（vy_b 非零）
- yaw_rate 有非零帧但 vz 基本为 0（走廊飞行，无高度变化）
- **数据集不含目标点信息**（2D Nav Goal 未记录）—— 纯 BC 无法实现 goal-reaching

### Phase 5 结论：PASS ✅

---

## 2026-06-29 — Phase 6: BC Baseline 训练

### 目标
在 uav_kd_v001 上训练正式 BC baseline，作为后续 KD 实验的对比基准。

### 模型架构

| 组件 | 细节 |
|---|---|
| Backbone | MobileNetV2（ImageNet 预训练，全参数微调）|
| Head | 2 层 MLP：Linear(1280,256)→ReLU→Dropout(0.3)→Linear(256,4) |
| 参数量 | 2.55M |
| 输入 | RGB image (3, 224, 224)，BGR→RGB，/255.0 |
| 输出 | [vx_b, vy_b, vz_b, yaw_rate]（body frame）|

### 训练配置（configs/train/bc.yaml）

| 参数 | 值 |
|---|---|
| epochs | 50 |
| batch_size | 32 |
| lr | 1e-4 |
| weight_decay | 1e-4 |
| lr_scheduler | cosine（3 epoch warmup）|
| loss | weighted MSE，权重 [1,1,1,1] |
| num_workers | 2 |
| device | CUDA（GTX 1080 Ti，11GB）|
| filter_static | True |
| filter_valid_traj | False |

### 训练过程

| Epoch | Train Loss | Val Loss | vx MSE | vy MSE | vz MSE | yr MSE |
|---|---|---|---|---|---|---|
| 1 | 0.030627 | 0.018481 | 0.0358 | 0.0341 | 0.0021 | 0.0019 |
| 5 | 0.006823 | 0.015405 | 0.0344 | 0.0262 | 0.0004 | 0.0006 |
| 15 | 0.003875 | 0.011672 | 0.0258 | 0.0207 | 0.0002 | 0.0001 |
| 30 | 0.002386 | 0.013028 | 0.0331 | 0.0190 | 0.0000 | 0.0000 |
| 45 | 0.001694 | **0.011639** | 0.0287 | 0.0178 | 0.0000 | 0.0000 |
| 50 | 0.001609 | 0.011841 | 0.0295 | 0.0179 | 0.0000 | 0.0000 |

### 训练结果

| 指标 | 值 |
|---|---|
| best val loss | **0.011639**（epoch 45）|
| final train loss | 0.001609 |
| best model | `runs/bc/20260629_151702/best_model.pt` |
| 每 epoch 耗时 | ~13–15 秒 |
| 总训练时间 | ~12 分钟 |

### 观察与分析

- vx_b 误差最大（0.0295），说明前向速度预测最难——这是最主要的控制维度
- vy_b 误差次之（0.0179），有障碍回避信号但不完整
- vz_b、yaw_rate 误差接近 0（数据中几乎无高度变化和转弯）
- val loss 在 epoch 15 附近达到最低，之后轻微过拟合
- **train/val gap 显著（0.0016 vs 0.0116）**：模型记忆了训练数据，泛化有限

### 部署验证
- 构建了 `ros_ws/src/uav_kd_student_policy/` ROS 包
- 节点：`run_bc_policy.py`，shebang 使用 dbc241 python，launch-prefix 绕过 catkin wrapper
- 发布话题：`/CERLAB/quadcopter/cmd_vel`（TwistStamped，body frame）
- 自动起飞 + 切换 vel_mode，之后纯视觉控制
- 初步飞行测试：**直线飞行正常，无目标点感知**

### BC Baseline 能力分析

| 能力 | 结论 |
|---|---|
| 直线向前飞行 | ✅ 已实现 |
| 被动障碍侧移（vy_b）| ⚠️ 部分（34.8% 训练帧有信号，但泛化未验证）|
| 到达指定目标点 | ❌ 不可能（模型无目标输入）|
| 过弯道/复杂轨迹 | ⚠️ 未知（需测试）|

### BC 控制分工说明

```
BC 模型负责：高层速度决策 [vx_b, vy_b, vz_b, yaw_rate]
Gazebo PID 自动处理：姿态稳定（roll/pitch）、电机混控、低层物理
```

### 训练命令（复现）
```bash
source ~/miniconda3/etc/profile.d/conda.sh && conda activate dbc241
cd /home/l/uav_kd_project
python kd_uav/train/train_bc.py --config configs/train/bc.yaml
```

### 部署命令
```bash
# Terminal 1
bash /home/l/uav_kd_project/scripts/launch_sim.sh
# Terminal 2
bash /home/l/uav_kd_project/scripts/launch_student_bc.sh
# Terminal 3（可选，监视）
bash /home/l/uav_kd_project/scripts/launch_rviz.sh
```

### Phase 6 结论：PASS ✅
BC Baseline 训练完成，部署节点就绪。作为后续 Traj KD / Ctrl KD / Joint KD 的对比基准。

---

## 第一阶段总结（Phase 0–6）

### 完成内容

| Phase | 内容 | 结论 |
|---|---|---|
| 0 | 工程审计 + 环境确认 | ✅ |
| 1 | CERLAB teacher 自主飞行验证 | ✅ |
| 2 | Teacher knowledge 定义（ctrl + traj label）| ✅ |
| 3 | 数据采集器构建 + traj bug 修复 + 7 episodes 采集 | ✅ |
| 4 | Debug dataset pipeline 验证（uav_kd_debug_v000）| ✅ |
| 5 | 正式数据集构建（uav_kd_v001，6945 样本）| ✅ |
| 6 | BC Baseline 训练 + 部署节点 | ✅ |

### 关键数据路径

| 类型 | 路径 |
|---|---|
| 原始数据 | `data/raw/ep_test_003~007/` |
| 正式数据集 | `data/processed/uav_kd_v001/` |
| BC 模型权重 | `runs/bc/20260629_151702/best_model.pt` |
| BC 训练 log | `runs/bc/20260629_151702/train.log` |
| BC loss 曲线 | `runs/bc/20260629_151702/loss_curve.png` |

### 环境说明

| 项目 | 值 |
|---|---|
| 训练环境 | conda `dbc241`，Python 3.8，torch 2.4.1+cu121 |
| GPU | NVIDIA GTX 1080 Ti，11GB，CUDA 12.2 |
| ROS 环境 | Noetic，system Python 3.8（训练时不激活）|
| 仿真 | Gazebo 11.15.1，world: corridor_dynamic_9_nohuman |

### 第二阶段计划

按优先级：

1. **Traj KD 训练** — image → traj（K=10），Loss = MSE(pred_traj, teacher_traj)
2. **Ctrl KD 训练** — image → ctrl，蒸馏框架，Loss = MSE(pred_ctrl, teacher_ctrl)
3. **Joint KD 训练** — image → traj + ctrl，L_total = L_traj + L_ctrl
4. **Gazebo 部署对比** — 4 个方法各跑多次，记录成功率/碰撞率
5. **（可选）红点视觉路点 BC** — 在图像上叠加当前路点红点，扩展 goal-aware 能力

### 已知问题与限制

| 问题 | 说明 |
|---|---|
| dyaw=0 | bspline_trajectory 路点方向未存储，traj_body dyaw 列全为 0 |
| 无目标点记录 | 采集时未记录 2D Nav Goal 坐标，纯 BC 无 goal-reaching 能力 |
| ep_test_007 少 1 帧 | aligned_frames=4405，少 1 帧（已处理，不影响训练）|
| ep_001/002 traj 无效 | 已排除出训练集，原始数据保留备查 |
| ep_test_006 未纳入 v001 | 保留备用，可加入 v002 |

---

## 2026-06-29 — 第二阶段 Module 1-3: BC-RedWP 训练完成

### 目标
在 BC Baseline 基础上，引入视觉路点引导（红点叠加图像），提升模型的方向感知能力。

### 设计原则
- 离线/在线使用完全相同的投影函数（`kd_uav/utils/waypoint_projection.py`），避免 train-test mismatch
- 只保留 `wp_visible=True` 的样本训练（wp 在图像范围内）
- 输出接口不变：`[vx_b, vy_b, vz_b, yaw_rate]`
- Student 部署时从 `/dynamicNavigation/bspline_trajectory` 获取路点（teacher-planned visual waypoint following）

### 模块实现

**Module 1: 投影工具 (`kd_uav/utils/waypoint_projection.py`)**
- `project_body_to_pixel(dx_b, dy_b, dz_b)` → (u, v, visible)
- `draw_red_waypoint(img, u, v, dist_b)` — 距离自适应半径
- `select_active_waypoint(row, lookahead=1.0, K=10)` — 选第一个 dx_b >= 1.0m 的路点
- 坐标变换：body → camera optical frame (Z_opt=dx_c, X_opt=-dy_c, Y_opt=-dz_c)

**Module 2: 数据集构建 (`kd_uav/datasets/render_red_waypoint.py` + `dataset_builder.py`)**
- 离线为每帧叠加红点，输出至 `images_red_wp/`，写 `waypoint_meta.csv`
- `dataset_builder.py --use_red_wp` 过滤 wp_visible=False 样本

### 数据集统计 (`uav_kd_red_wp_v001`)

| 分集 | 样本数 | vx_b mean | vx_b std | yaw_rate std |
|---|---|---|---|---|
| train | 3040 | 0.588 | 0.290 | 0.206 |
| val   | 475  | 0.730 | 0.111 | 0.000 |

*红点过滤效果：vx mean 从 0.356 升至 0.588（移除悬停帧），yaw_rate std 从 0.328 降至 0.206（移除急转帧）*

### Sanity Checks（训练前）

1. **视觉检查** — 抽取 train/val 各 20 张图像，红点位置正常：
   - 直线飞行时红点在图像中央 ✓
   - 绕障时红点偏向目标侧 ✓
   - 无左右/上下反向 ✓
   - 靠近墙壁时红点投影到墙面（符合体坐标系投影逻辑）✓

2. **Config 检查** — `configs/train/bc_red_wp.yaml` ✓
   - processed_dir: `data/processed/uav_kd_red_wp_v001`
   - 所有超参与 `bc.yaml` 完全一致（epochs=50, batch=32, lr=1e-4, cosine sched）
   - runs_dir: `runs/bc_red_wp`

3. **Action 统计** — 无异常，分布符合预期 ✓

### Module 3: 训练结果

**命令**
```bash
conda activate dbc241
cd /home/l/uav_kd_project
python kd_uav/train/train_bc.py --config configs/train/bc_red_wp.yaml
```

**Run ID**: `20260629_180554`
**Run dir**: `runs/bc_red_wp/20260629_180554/`

**Epoch 损失表（关键节点）**

| Epoch | Train  | Val    | vx     | vy     | vz     | yr     |
|-------|--------|--------|--------|--------|--------|--------|
| 1     | 0.0272 | 0.0154 | 0.0317 | 0.0228 | 0.0048 | 0.0022 |
| 10    | 0.0041 | 0.0126 | 0.0297 | 0.0202 | 0.0002 | 0.0002 |
| 20    | 0.0030 | 0.0138 | 0.0365 | 0.0186 | 0.0001 | 0.0001 |
| 31    | 0.0024 | **0.0116** | 0.0290 | 0.0174 | 0.0001 | 0.0000 |
| 40    | 0.0019 | 0.0123 | 0.0318 | 0.0173 | 0.0001 | 0.0000 |
| 50    | 0.0018 | 0.0125 | 0.0326 | 0.0174 | 0.0001 | 0.0000 |

**最佳 val**: `0.011610` @ Epoch 31

### 对比 BC Baseline

| 模型         | 数据集           | 最佳 val loss | 最佳 epoch |
|---|---|---|---|
| BC Baseline  | uav_kd_v001      | 0.011639     | —          |
| **BC-RedWP** | uav_kd_red_wp_v001 | **0.011610** | 31         |

离线指标基本持平（差异 < 0.03%）。关键区别在于模型输入包含路点方向引导——真实飞行效果的差异需要 Gazebo 部署实验验证。

### 模型文件
```
runs/bc_red_wp/20260629_180554/
├── best_model.pt       ← 用于部署
├── last_model.pt
├── loss_curve.png
└── config.yaml
```

### 下一步
- **Module 4+5**: 更新 `run_bc_policy.py` — 订阅 `/dynamicNavigation/bspline_trajectory`，实时投影红点后推理，无路点时悬停
- **Gazebo 部署**: BC-RedWP vs BC Baseline 对比飞行

---

## 2026-07-01 — 数据集旋转样本审计 + v002 重建

### Goal
排查"BC 学不会旋转"的根因：确认是数据缺失还是模型问题，并据此决定是否需要补录新 episode。

### 背景
CERLAB teacher 自身在直线走廊里几乎不做连续弯道飞行，"转弯"主要靠原地小幅偏航（避开动态障碍物）实现。

### 审计结果（原始 7+2 个 episode 的 yaw_rate 统计）
| Episode | 帧数 | mean(\|yaw_rate\|) | \|yaw_rate\|>0.15 占比 | v001 用途 |
|---|---|---|---|---|
| ep_003/004 | 1013/958 | ≈0 | 0% | train |
| ep_005 | 569 | ≈0 | 0% | **val（100% 直飞，旋转 MSE 无法真实衡量）** |
| ep_006 | 962 | 0.130 | 18.2% | ❌ 未使用（保留备用） |
| ep_007 | 4405 | 0.167 | 22.9% | train |
| ep_008（新采集）| 2457 | 0.030 | 4.2% | ❌ 未入库 |
| ep_009（新采集）| 1337 | 0.036 | 6.2% | ❌ 未入库 |

结论：不需要立即补录——已有 3 个采集完整但未使用的 episode（006/008/009），traj_body 质量与 003/004/007 同一水平（dx0<0 占比 5~18%，正常范围）。

### 发现的数据 bug：spot-turn 期间 traj_body 退化
`bspline_trajectory` 在 UAV 原地旋转（不平移）时不会重新发布，导致该时段内全部 K=10 个路点冻结在 UAV 自身位置附近（body frame 幅值 ≈0）。原有 `valid_traj` 规则仅检查 `dx0>=0`，这类冻结路点会被误判为 valid。

统计（旋转帧中 traj 退化的比例）：
| Episode | 旋转帧数 | 退化 traj 占比 |
|---|---|---|
| ep_006 | 175 | 0% |
| ep_007 | 1008 | 1.9% |
| ep_008 | 103 | 0% |
| ep_009 | 83 | **92.8%**（frame 200–218 一段长时间原地转，第一次真正移动前的探索转向） |

### 修复
`kd_uav/datasets/dataset_builder.py`：
- 新增 `DEFAULT_TRAJ_MAGNITUDE_THRESHOLD = 0.02`（`sum(|dx_i|+|dy_i|)` over K waypoints）
- `valid_traj` 改为 `dx0>=0 AND traj_magnitude>=threshold`；新增 `filter_reason='traj_degenerate'`
- 新增 CLI 参数 `--traj_magnitude_threshold`
- 阈值选取依据：对全部 11701 帧的 traj magnitude 做百分位分析，p1.8 仍为 0.000000，p2.0 跳到 0.006，之间有清晰的 gap，0.02 可安全隔离退化帧而不误伤真实的近目标短轨迹帧

### 数据集重建：uav_kd_v002
```bash
python kd_uav/datasets/dataset_builder.py \
  --raw_dir data/raw \
  --output_dir data/processed/uav_kd_v002 \
  --episodes ep_test_003 ep_test_004 ep_test_007 ep_008 ep_009 ep_test_005 ep_test_006 \
  --K 10 --dataset_type train \
  --val_episodes ep_test_005 ep_test_006
```

划分策略：train = 003+004+007+008+009（10170），val = 005+006（1531，混合直飞+旋转，而非 v001 那样 100% 直飞）。

### 结果
| | v001 | v002 |
|---|---|---|
| train | 6376 | 10170 |
| val | 569 | 1531 |
| val 旋转样本占比 | 0% | ~11.4% |
| train 旋转样本占比 | ~16% | ~11.8% |
| valid_traj_count | — | 9980/11701（85.3%，退化路点修复前为 10246） |

### 视觉抽检
从 ep_006/007/008/009 各随机抽取 1 个高旋转帧（\|yaw_rate\|>0.2），导出图像人工检查：4 张图均呈现明显水平运动模糊条纹，与标注的 yaw_rate（vx/vy/vz≈0，纯原地旋转）吻合，确认标签真实可信，非采集/同步错误。

### Manual Check
- [x] 新 episode（008/009）traj_body 质量与训练用 episode 同一水平
- [x] 旋转样本视觉核验通过（运动模糊 vs yaw_rate 标注一致）
- [x] valid_traj 退化路点过滤生效（10246 → 9980）
- [x] BC baseline 在 v002 上重新训练，验证 yaw_rate val MSE 是否从 ≈0 变为有意义数值

### BC on uav_kd_v002 — 训练结果（2026-07-01）
```bash
python kd_uav/train/train_bc.py --config configs/train/bc_v002.yaml
```
Run: `runs/bc_v002/20260701_142506/`，50 epoch，train=6722，val=935（filter_static 后）

| | best epoch(23) | v001 baseline（旧val，100%直飞）|
|---|---|---|
| val loss | 0.052565 | 0.011639 |
| vx MSE | 0.1014 | 0.0295 |
| vy MSE | 0.0209 | 0.0179 |
| vz MSE | ≈0 | ≈0 |
| yaw_rate MSE | 0.0879 | ≈0（无旋转样本可评估）|

**trivial baseline 诚实检查**（val 集上"预测常数"能拿到的 MSE，用于判断模型是否真的学到东西）：
- yaw_rate：模型 0.0879 vs 预测 val 均值(-0.133) 得 0.0999，vs 预测 0 得 0.1177 → 模型确实优于两个 trivial 基线（+12%~+25%），旋转信号不再退化
- vx：模型 0.1014 vs 预测 val 均值(0.549) 得 0.0899 → **模型比 trivial 基线还差**，说明不只是"任务变难"，更可能是 episode 级泛化问题（ep_006 作为完全未见过的 val episode，行为模式/视觉分布与 train episode 有差异，单帧 BC 未能很好泛化）

**结论**：v001 与 v002 的 loss 数值不可直接比较（v001 val 是 100% 直飞的简单分布，BC-RedWP 的 0.011610 同理）。yaw_rate 学习信号已从"退化"变为"真实但偏弱"，vx 出现的泛化 gap 是新发现的问题，但不影响后续 KD 实验的推进——反而印证了纯单帧 BC 的局限性，是 TrajKD/CtrlKD/JointKD 的动机所在。**后续 TrajKD/CtrlKD/JointKD 应以 uav_kd_v002 上的 BC 结果（val loss 0.0526，yr MSE 0.0879）作为公平基线，而非旧的 v001/BC-RedWP 数值。**

### 下一步
1. 继续推进 TrajKD（Option B），使用 uav_kd_v002 作为统一训练/评估数据集
2. 注意 spot-turn 帧的 traj 标签在 `filter_valid_traj=True` 下会被正确排除（退化路点已修复）
3.（可选，非阻塞）后续如需专门排查 vx 泛化 gap，可视化 ep_006 上的预测 vs 真值轨迹，或检查 ep_006 是否有画面风格/障碍物布局与 train episode 显著不同的因素

### Gazebo 部署飞行测试（2026-07-01）
```bash
# Terminal 1
bash scripts/launch_sim.sh
# Terminal 2
bash scripts/launch_student_bc.sh runs/bc_v002/20260701_142506/best_model.pt
```
World: `corridor_dynamic_9.world`（含人体障碍模型）。

结果：
- 自动起飞 + vel_mode 生效，cmd_vel 持续发布，运行 2+ 分钟无崩溃、无报错
- yaw_rate 输出真实变化（-0.02 ~ -0.39 区间），多次出现 `vx≈0 且 yr 较大` 的类 spot-turn 时刻，与训练数据模式吻合
- 高度稳定在 ~0.44m（低于 teacher 常见 1m 悬停高度，但无下坠/震荡）
- x 位置持续增长，确认 UAV 在向前推进，非卡死原地

Manual check:
- [x] 起飞成功
- [x] cmd_vel 非零、持续发布
- [x] 无崩溃/无 ROS 报错
- [x] 仅使用前置相机（无 teacher 信号参与部署）
- [ ] 定量指标（成功率/碰撞率/平均飞行时长）—— 留待正式 ablation 对比阶段统一测量，本次为基线阶段的定性验收

**BC baseline 阶段结论**：离线指标 + 部署测试均确认 v002 上训练的 BC 已具备非退化的旋转输出能力。基线阶段到此结束，转入 TrajKD 实现。

---

## 2026-07-02 — TrajKD（Option B）实现与训练

### Goal
按 `research.md` §7.2 实现 TrajKD：backbone → trajectory head（辅助，仅训练用）+ control head（主输出，部署用），验证轨迹监督能否提升 backbone 表征质量，从而改善 control head 的表现（对比 BC）。

### 新增文件
- `kd_uav/models/trajectory_head.py` — `TrajectoryHead`（`1280→256→K*3`→`(B,K,3)`，K=10）+ `TrajKDPolicy`（backbone + traj_head(aux) + ctrl_head(部署)，`forward_ctrl_only()` 为部署路径）—— 上一 session 写的草稿，本次经用户逐项确认架构后使用
- `kd_uav/losses/trajectory_kd_loss.py` — `TrajectoryKDLoss`（逐轴加权 MSE，权重 `[1,1,1]`）+ `ate()` 辅助函数（Average Trajectory Error，逐路点 L2 距离均值）
- `kd_uav/train/train_traj_kd.py` — 复用 `train_bc.py` 的脚手架（config/日志/cosine调度/checkpoint），扩展为双头前向 + 组合损失
- `configs/train/traj_kd.yaml` — `processed_dir: uav_kd_v002`，`filter_valid_traj: true`，`lambda_traj: 1.0`，`K: 10`

### 训练命令
```bash
python kd_uav/train/train_traj_kd.py --config configs/train/traj_kd.yaml
```
Run: `runs/traj_kd/20260702_131051/`，50 epoch

数据：`filter_valid_traj=true` 后 train=6004（BC 用 6722，少了约700个 traj 无效/退化帧），val=861

### 结果

| | BC(v002) best (ep23) | TrajKD best (ep9，按total loss选) | TrajKD final (ep50) |
|---|---|---|---|
| ctrl loss（部署头）| **0.052565** | **0.0482** | 0.0487 |
| vx MSE | 0.1014 | 0.0866 | 0.0916 |
| vy MSE | 0.0209 | 0.0227 | 0.0211 |
| vz MSE | ≈0 | 0.0001 | ≈0 |
| yaw_rate MSE | 0.0879 | 0.0835 | 0.0820 |
| Trajectory ATE | n/a | 0.257m | 0.257m |

**TrajKD 部署头（ctrl-only）的 loss 优于 BC**（0.0482 vs 0.0526，约低8%；final epoch 也保持优势 0.0487 vs 0.0541），主要来自 vx 和 yaw_rate 的提升，vy 基本持平。符合 `research.md` §8.3 的预期结论（TrajKD > BC）。

### Manual Check（`CLAUDE.md` §11）
- [x] Trajectory head 输出未出现 NaN / 坍缩为 0；ATE 全程稳定在 0.25~0.32m
- [x] 两个 loss 分量全程量级相当（ctrl ~0.047-0.066，traj ~0.040-0.055），无一方压制另一方
- [x] Control head 输出范围与 BC 一致，无爆炸/坍缩

### Gazebo 部署冒烟测试（2026-07-02）
```bash
# Terminal 1
bash scripts/launch_sim.sh
# Terminal 2
bash scripts/launch_student_traj_kd.sh runs/traj_kd/20260702_131051/best_model.pt
```
新增部署文件：
- `ros_ws/src/uav_kd_student_policy/scripts/run_traj_kd_policy.py`（仿照 `run_bc_policy.py`，推理路径为 `model.forward_ctrl_only()`，trajectory head 完全不参与部署）
- `ros_ws/src/uav_kd_student_policy/launch/traj_kd_policy.launch`
- `scripts/launch_student_traj_kd.sh`
- `CMakeLists.txt` 加入新脚本，`catkin_make --pkg uav_kd_student_policy` 重新构建

World: `corridor_dynamic_9.world`。

结果：
- ROS 层面：运行 ~3.5 分钟无崩溃、无报错、无 NaN，cmd_vel 持续发布，高度稳定在 ~0.40m（与 BC(v002) 测试同一水平）
- **行为层面（诚实记录）**：前 ~40s 正常推进（x: 0→4.6m，vx 峰值0.57），此后**趋于停滞**——约1分钟起进入稳态爬行（vx≈0.08，vy≈-0.04，yr≈0.02-0.05），后续2.5分钟内 x 基本不再增长（4.6→4.57m）。推测是在障碍物前卡住：前进和转向指令小幅抵消，未能绕过障碍物继续导航。不是崩溃，但导航能力不完整。

Manual check:
- [x] 起飞成功、cmd_vel 持续发布
- [x] 无崩溃/无 ROS 报错/无 NaN
- [x] 仅用前置相机（ctrl-only 推理，trajectory head 不参与部署）
- [ ] 定量指标（成功率/碰撞率/平均飞行时长）—— 留待正式 ablation 阶段统一测量
- [ ] "卡住不前"问题的根因排查 —— 后续工作（不阻塞当前 TrajKD 计划收尾）

**TrajKD 5-phase 计划结论**：离线指标确认 TrajKD 的控制头优于 BC（§本文档上一条目）；部署冒烟测试确认系统级稳定性达标，但发现了一个真实的行为局限（近障碍物时可能卡住）。此计划到此收尾。

---

## 2026-07-02 — 设计讨论：SelfRedWP（新方向）

### Problem
TrajKD 部署测试暴露的问题不是"崩溃"，而是"不敢转弯"：部署时 yaw_rate 标准差只有 BC(v002) 的 1/4（0.027 vs 0.104），\|yr\|>0.15 的强转弯指令占比从 BC 的 10.4% 降到 0.9%，导致 UAV 在障碍物前卡住。根因：轨迹知识只作为辅助 loss 隐式影响 backbone 特征，从未真正传递到 control head 能直接利用的信号里。

### Decision
不再用隐式辅助 loss，改为**显式视觉引导**：预测轨迹 → 选取一个路点 → 以红点形式画在图像上（复用 `kd_uav/utils/waypoint_projection.py` 已有的 BC-RedWP 机制）→ control head 直接基于这张带红点的图像做决策。与 BC-RedWP 不同的是，红点来自**模型自己预测的轨迹**，不依赖部署时运行 CERLAB 规划器——比 BC-RedWP（部署时仍需要 `dynamicNavigation` 提供实时 `/dynamicNavigation/bspline_trajectory`）更彻底地满足"仅用前置相机"的约束。

关键设计决策（讨论详情见对话记录，此处只记结论）：
- **架构**：单一共享 backbone，一次 forward 里跑两次——第一次用原图预测轨迹，第二次用画好红点的图预测控制。不是两个独立网络。
- **训练/部署不一致问题（exposure bias / covariate shift，DAgger 同源问题）**：如果训练时红点永远来自真值路点，control head 学到的是"给定精确引导点该怎么做"，但部署时红点来自轨迹头自己的（偏保守的）预测——分布不一致，可能导致失效。解法：**课程学习**——训练时按 epoch 递增的概率 `p_self` 从真值路点切换到模型自己预测的（detached，不回传梯度）路点，前期用真值让 control head 先学会干净的映射，后期切到自预测让它适应真实部署分布。
- **不需要可微渲染**：预测路点在画点前 detach，`L_traj` 和 `L_ctrl` 各自独立对真值监督，梯度不需要穿过不可微的 `cv2.circle` 画点步骤。
- **初始化**：全新初始化（不复用已训练的 TrajKD 权重），保持消融实验的独立性。
- **本轮范围**：沿用 `uav_kd_v002` 现有的 K=10 局部窗口 `traj_body`（不到达终点）先验证"自引导红点机制"本身能否解决转弯问题；真正"含终点轨迹"需要重新处理 rosbag（已确认 `/dynamicNavigation/bspline_trajectory` 完整记录在每个 episode 的 `rosbag/flight.bag` 里，可行但推迟到后续迭代）。

### Next Step
按 5-phase 计划实现：架构确认 → loss/课程学习设计 → 训练脚本 → 训练 → Gazebo 部署验证，重点验证 yaw_rate std / 强转弯占比能否恢复到接近 BC(v002) 的水平。

---

## 2026-07-04 — SelfRedWP 实现、训练与部署测试

### 实现
- `kd_uav/models/self_red_wp_policy.py` — `SelfRedWPPolicy`：单一共享 backbone，暴露 `predict_traj()` / `predict_ctrl()` 两个方法（不是单一 forward，因为画点步骤要离开 tensor 空间）
- `kd_uav/datasets/uav_kd_dataset.py` — 新增 `return_raw_image` 选项（向后兼容，默认关闭），额外返回原始分辨率（640×480）BGR 图像，供在线画点使用（`waypoint_projection.py` 的针孔投影是按原始分辨率标定的，不能直接用训练分辨率的图像）
- `kd_uav/train/train_self_red_wp.py` — 两遍前向 + 课程学习（`p_self` 按 epoch 从 0 线性升到 1，5→35 epoch 区间渐变，35+ 保持 1.0）；验证集始终用自预测路点（对标真实部署条件）
- `configs/train/self_red_wp.yaml`
- 部署节点：`run_self_red_wp_policy.py` + `self_red_wp_policy.launch` + `launch_student_self_red_wp.sh`（每帧 predict_traj→选点→画点→predict_ctrl，完全自包含，不需要实时规划器）

Dry run 通过；红点投影经视觉核验，几何方向正确（dy=-0.39→图像右侧，dy=+0.20→图像左侧，符合 body frame 左正右负的约定）。

### 训练
```bash
python kd_uav/train/train_self_red_wp.py --config configs/train/self_red_wp.yaml
```
Run: `runs/self_red_wp/20260704_162940/`，50 epoch，耗时约3.5小时（单个epoch ~252s，比TrajKD慢约11倍——单样本Python/cv2画点循环是CPU瓶颈，GPU和CPU串行而非并行，画点步骤依赖刚算出的traj_pred，无法预先在DataLoader worker里并行处理）。

**离线结果（final epoch，对比 BC(v002) / TrajKD）：**

| | BC(v002) | TrajKD | SelfRedWP |
|---|---|---|---|
| ctrl loss | 0.0541 | 0.0487 | **0.0459** |
| vx MSE | 0.0957 | 0.0916 | **0.0867** |
| vy MSE | 0.0197 | 0.0211 | 0.0214 |
| yaw_rate MSE | 0.1009 | 0.0820 | **0.0757** |
| traj ATE | n/a | 0.2569m | 0.2508m |
| **val yr_std**（自预测路点）| — | 0.027（部署时坍缩）| **0.133** |
| **val frac(\|yr\|>0.15)** | — | 0.9% | **15.9%** |

课程学习期间（epoch 6-35，p_self渐增到1.0）及之后（36-50，纯自预测），yr_std全程稳定在0.12-0.14区间，**没有出现TrajKD那种随训练推进而坍缩的现象**——offline层面看，机制似乎奏效了。

### Gazebo 部署测试
```bash
bash scripts/launch_sim.sh
bash scripts/launch_student_self_red_wp.sh runs/self_red_wp/20260704_162940/best_model.pt
```
World: `corridor_dynamic_9.world`。

**结果（用户直接观察 Gazebo 画面后判定）：仍然卡在第一个障碍物前，和 TrajKD 同样的失败模式。**

日志显示：起飞正常，前 ~30s 曾尝试一次真实转弯（yr 从 -0.170 逐渐回落到 -0.03，同时预测路点 dx 从 0.86 变化到 0.72 附近），但随后**预测路点冻结**——`wp=(dx,dy)` 稳定在 (0.72~0.76, -0.22~-0.27) 这个窄区间不再变化，vx 降到 ~0.11-0.15，yr 在 0 附近小幅震荡。最终位置 x=4.6m，**和 TrajKD 卡住的位置几乎一样**。

**实际部署 yr 统计（与 offline 验证集诊断的对比）：**

| | BC(v002) 部署 | TrajKD 部署 | SelfRedWP 部署（实际）| SelfRedWP val（offline）|
|---|---|---|---|---|
| yr std | 0.104 | 0.027 | **0.0415** | 0.133 |
| frac(\|yr\|>0.15) | 10.4% | 0.9% | **3.4%** | 15.9% |

**关键发现：offline 诊断（val yr_std=0.133）严重高估了实际部署表现（实际只有0.0415），更接近 TrajKD 的坍缩水平，而不是训练时看到的健康数字。**

### 根因分析（假设，未验证）
一旦前进停滞，相机画面帧间变化很小，轨迹头倾向于反复预测几乎相同的路点——这又反馈成几乎相同（不足够）的控制动作，导致画面持续不变。自我强化的停滞循环，不是崩溃。这和课程学习要解决的"训练/部署路点来源不一致"是**不同的问题**：课程学习解决了"控制头有没有见过自己预测的（有偏的）路点"，但没有解决"模型陷入自己造成的静止状态后能否脱困"——这是闭环部署特有的、offline 验证集（哪怕用自预测路点）完全无法捕捉到的失效模式，因为验证集是固定的、非交互的历史帧，不会出现"模型的动作导致画面停止变化"这种自反馈。

### Manual Check
- [x] 起飞成功、cmd_vel 持续发布、无崩溃/无 NaN
- [x] 红点几何投影视觉核验正确
- [x] 课程学习期间 yr_std 未出现 TrajKD 式的训练期坍缩
- [ ] **实际部署未能解决"卡住"问题**——offline 诊断具有误导性，不能替代真实部署测试
- [ ] 停滞后脱困机制的根因排查与解决方案——留待后续

### 结论
SelfRedWP 在**offline 指标**上全面优于 BC(v002) 和 TrajKD（ctrl loss、大部分维度 MSE、yr_std 未坍缩），但**实际部署行为仍然是同一个失败模式**：卡在第一个障碍物前，位置几乎和 TrajKD 一样。这是一个重要的方法论教训：**offline 验证集诊断（即使刻意设计为"自预测路点、对标部署条件"）不足以预测闭环部署表现**，因为它无法捕捉"模型自身动作导致的静止状态"这种反馈循环。5-phase 计划到此收尾；根因（停滞脱困机制缺失）和后续方向留待讨论。

---

## [TEMPLATE] 未来实验记录模板

### 下一步
Phase 3 — Build `uav_kd_data_collector` ROS package to record synchronized:
- `/camera/color/image_raw`
- `/autonomous_flight/target_state` → [vx, vy, vz, yaw_rate]
- `dynamicNavigation/bspline_trajectory` → future waypoints
- `/CERLAB/quadcopter/odom`
