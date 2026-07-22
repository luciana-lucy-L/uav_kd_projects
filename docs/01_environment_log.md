# Environment Log

**Date:** 2026-06-28
**Phase:** 0 — Project Audit

---

## System Environment

| Item | Value |
|---|---|
| OS | Ubuntu (Linux 5.15.0-139-generic, x86_64) |
| ROS | Noetic |
| Gazebo | 11.15.1 |
| Python (system) | 3.13.12 |
| Shell | bash |

## Conda Environments

| 环境 | 用途 | 状态 |
|---|---|---|
| `dbc241` | **官方 UAV-KD pipeline 环境** | **使用此环境** |
| `base` | Conda base | 不用于 ROS 或训练 |
| `dbc_bc` | 不完整（缺 torch） | 弃用，改用 dbc241 |
| `torchdistill` | 无关库实验 | 不使用 |

### dbc241 包版本（已验证 2026-06-29）

| 包 | 版本 | 用途 |
|---|---|---|
| `torch` | 2.4.1+cpu | 模型训练/推理 |
| `torchvision` | 0.19.1+cpu | 图像变换、预训练 backbone |
| `numpy` | 1.24.4 | 数组运算 |
| `cv2` (opencv) | 4.11.0 | 图像读取/处理 |
| `yaml` (pyyaml) | 6.0.3 | 配置文件 / metadata.yaml |
| `pandas` | 2.0.3 | CSV 读写 |
| `matplotlib` | 3.7.5 | 训练曲线、结果可视化 |
| `tqdm` | 4.67.3 | 训练进度条 |

所有 UAV-KD pipeline 所需包均已就绪，无需额外安装。

### 环境使用规则（确认 2026-06-29）

| 任务 | 环境 | 说明 |
|---|---|---|
| ROS / Gazebo / 数据采集节点 | 系统 Python（无 conda） | 启动前执行 `conda deactivate` |
| `dataset_builder.py` | `dbc241` | `conda activate dbc241` |
| `inspect_dataset.py` | `dbc241` | `conda activate dbc241` |
| BC / KD 模型训练 | `dbc241` | `conda activate dbc241` |
| 所有 Python 数据/训练工作 | `dbc241` | `conda activate dbc241` |

**规则：** 绝不在任何 conda 环境激活状态下启动 ROS 节点。
**规则：** 所有 Python 训练和数据处理工作统一使用 `dbc241`。

## ROS Workspaces

### catkin_ws (`/home/l/catkin_ws/`)
| Package | Role |
|---|---|
| `uav_bc_tools` | Old expert recording + BC deployment nodes |
| `uav_simulator` | Gazebo UAV simulator with worlds, models, plugin `.so` files |

### cerlab_ws (`/home/l/cerlab_ws/`)
This is the CERLAB Autonomous Flight teacher system. Do not modify.

| Package | Role |
|---|---|
| `autonomous_flight` | Teacher flight modes: navigation, dynamic navigation, exploration, inspection |
| `global_planner` | RRT / RRT* global path planning |
| `map_manager` | ESDF, dynamic, and occupancy map management |
| `onboard_detector` | Object/obstacle detection |
| `remote_control` | RViz-based interactive control |
| `time_optimizer` | Trajectory time-scaling optimization |
| `tracking_controller` | Low-level tracking controller (publishes `cmd_vel`) |
| `trajectory_planner` | B-spline and polynomial trajectory planning |
| `uav_simulator` | Shared simulator package |

## Known ROS Topics (from code inspection — not yet verified live)

| Topic | Type | Source |
|---|---|---|
| `/CERLAB/quadcopter/cmd_vel` | `geometry_msgs/TwistStamped` | `tracking_controller` |
| `/CERLAB/quadcopter/odom` | `nav_msgs/Odometry` | UAV simulator |
| `/CERLAB/quadcopter/odom_raw` | `nav_msgs/Odometry` | UAV simulator |
| `/CERLAB/quadcopter/cmd_acc` | `geometry_msgs/Vector3` | `tracking_controller` |
| `/camera/color/image_raw` | `sensor_msgs/Image` | UAV simulator (camera plugin) |
| `/camera/depth/image_raw` | `sensor_msgs/Image` | UAV simulator |
| `/camera/depth/points` | `sensor_msgs/PointCloud2` | UAV simulator |
| Trajectory planner topic | **Unknown — must verify in live sim** | `trajectory_planner` |

**Action required:** Run CERLAB `dynamic_navigation.launch` in Gazebo, run `rostopic list`, and fill in the trajectory planner topic. Record results in `docs/02_topic_mapping.md`.

## Detected Data

| Location | Content | Usable for KD? |
|---|---|---|
| `legacy/old_bc_training/data/` | 4 state-based `.npz` files, ~280 KB total | No — no image key |
| `data/raw/candidate_bags/` | 5 rosbags (~693 MB total, symlinked) | Unverified — inspect before use |
| `/home/l/datasets/cifar10/` | CIFAR-10 | No — unrelated |

## Detected BC-Related Files (all archived as legacy)

| File | Location | Status |
|---|---|---|
| `train_bc.py` | `legacy/old_bc_training/` | Archived — state-based, not image-based |
| `train_bc_vel.py` | `legacy/old_bc_training/` | Archived |
| `train_bc_vel_1.py` | `legacy/old_bc_training/` | Archived |
| `train_bc_vel_kd.py` | `legacy/old_bc_training/` | Archived |
| `bc_policy*.pt` (4 models) | `legacy/old_bc_training/models/` | Archived — do not reload |
| `record_expert.py` | `legacy/old_bc_tools/scripts/` | Archived — design reference only |
| `bc_policy_node*.py` (4 nodes) | `legacy/old_bc_tools/scripts/` | Archived — state-based only |

## 组件完成状态（更新至 2026-06-29）

### 数据采集组件
| 组件 | 状态 |
|---|---|
| `ros_ws/src/uav_kd_data_collector/` | ✅ 已实现并通过测试 |
| 轨迹 topic 名称 | ✅ 已确认：`/dynamicNavigation/bspline_trajectory` |
| 同步 image + ctrl + traj 采集 | ✅ 已验证（ep_test_003，790 帧） |

### 数据集组件
| 组件 | 状态 |
|---|---|
| `kd_uav/datasets/dataset_builder.py` | ✅ 已实现，debug dataset 构建通过 |
| `kd_uav/datasets/uav_kd_dataset.py` | ✅ 已实现，DataLoader 验证通过 |
| `kd_uav/datasets/inspect_dataset.py` | ✅ 已实现，统计和图像输出正常 |
| `data/processed/uav_kd_debug_v000/` | ✅ 790 samples，DataLoader OK |

### 模型训练组件（待实现）
| 组件 | 状态 |
|---|---|
| `kd_uav/models/student_backbone.py` | ❌ 未实现 |
| `kd_uav/models/control_head.py` | ❌ 未实现 |
| `kd_uav/models/trajectory_head.py` | ❌ 未实现 |
| `kd_uav/losses/bc_loss.py` | ❌ 未实现 |
| `kd_uav/train/train_bc.py` | ❌ 未实现 |
| `configs/train/bc.yaml` | ❌ 未实现 |

### 数据状态
| 项目 | 状态 |
|---|---|
| 有效 episode | 1 个（ep_test_003，790 帧） |
| 正式 dataset `uav_kd_v001` | ❌ 未构建，等待 5–10 个 episode 后进行 |
| dyaw 标签 | ❌ 全为 0（has_dyaw=False），需从 rosbag 重提取 |

## 当前推荐下一步

1. 用仿真环境多跑几次采集（目标 5–10 个 episode，含弯道飞行）
2. 构建正式 `uav_kd_v001` dataset（episode 级别 train/val/test 划分）
3. 实现 BC baseline 模型和训练脚本
