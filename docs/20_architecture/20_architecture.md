# 系统与模型架构

> 层级：架构层。**模型结构变更时修改本文件。**
> 版本：v2（2026-09-07）。研究动机见 `10_research/10_research.md`，实验执行见 `40_experiments/40_experiments.md`
> ⚠️ 本文件描述**目标架构**（R1–R2 实现）。当前代码尚未达到此状态，进度见 `30_plan/30_plan.md`

---

## 1. 教师系统：CERLAB Autonomous Flight

**外部依赖，位于 `/home/l/cerlab_ws/`，严禁修改。**

经典机器人自主飞行栈，三个物理模块：

| 模块 | 算法 | 输出 | 可蒸馏知识 |
|---|---|---|---|
| **感知** | RGB-D / 点云、动态障碍跟踪、ESDF / 占据地图 | 局部地图、障碍物 | —（不蒸馏） |
| **规划** | B 样条轨迹优化、增量式 PRM | 未来轨迹 τ | **规划层知识** |
| **控制** | 级联 PID 跟踪控制器 | 速度/加速度指令 u | **执行层知识** |

### 1.1 用于蒸馏的教师信号

| 信号 | ROS 话题 | 频率 | 角色 |
|---|---|---|---|
| 前视 RGB 图像 | `/camera/color/image_raw` | ~30 Hz | **学生唯一视觉输入** |
| 教师控制 | `/autonomous_flight/target_state` | ~100 Hz | 动作/执行层标签 |
| 教师轨迹 | `/dynamicNavigation/bspline_trajectory` | ~30 Hz | 规划层标签 |
| 目标点 | `/move_base_simple/goal` | 事件触发 | **目标条件化输入（v2 新增）** |
| 位姿 / 里程计 | `/CERLAB/quadcopter/pose`, `/odom` | 30 Hz | 同步基准 + 目标向量换算 |

### 1.2 教师暴露的完整知识层（2026-09-07 源码扫描）⭐

**扫描结论：教师的可分解性远超 v1 的假设。** v1 只用了 2 层（轨迹 + 控制），实际暴露 **5 层**：

| 层 | 话题 | 消息类型 | 蒸馏可行性 |
|---|---|---|---|
| **感知-静态几何** | `<ns>/2D_occupancy_map` | `nav_msgs/OccupancyGrid` | ⭐ **最适合做 dense prediction 标签**（规则网格） |
| | `<ns>/esdf` | `sensor_msgs/PointCloud2` | 欧氏符号距离场（障碍距离 + 梯度） |
| | `<ns>/voxel_map`, `/inflated_voxel_map`, `/explored_voxel_map` | `PointCloud2` | 占据栅格 |
| **感知-动态障碍** | `<ns>/tracked_bboxes`, `/dynamic_bboxes` | `MarkerArray` | 跟踪的动态障碍框 |
| | `<ns>/history_trajectories`, `/velocity_visualizaton` | `MarkerArray` | 障碍历史轨迹与速度 |
| | `<ns>/dynamic_point_cloud`, `/filtered_depth_cloud` | `PointCloud2` | |
| **全局规划** | `dynamicNavigation/rrt_path` | `nav_msgs/Path` | 全局路径（需开 `use_global_planner`） |
| **局部规划** | `dynamicNavigation/bspline_trajectory` | `nav_msgs/Path` | ✅ 当前使用 |
| | `dynamicNavigation/poly_traj`, `/input_trajectory`, `/pwl_trajectory` | `nav_msgs/Path` | **B样条优化前的中间阶段** —— 支持"规划层内部再分"的更细消融 |
| **控制** | `/autonomous_flight/target_state` | `tracking_controller/Target` | ✅ 当前使用 |

**两个可主动查询的 ROS service**（不只是被动录制）：

| Service | 用途 |
|---|---|
| `<ns>/check_pos_collision` | 查询任意位置是否碰撞 |
| `<ns>/raycast` | 射线投射查询 |

🔶 **对研究的影响**：
1. **"可分解教师"不是勉强的说法，是一个有 5 个真实层次的系统** —— 这直接强化 `10_research/10_research.md` 的核心立论
2. **感知层有真实产物可蒸馏**（`2D_occupancy_map` / `tracked_bboxes`），不需要人造"教师表征" —— 见 §3.2
3. 两个 service 对 **DAgger** 特别有价值：可直接询问教师"这个状态该怎么做/会不会撞"，而不只是回放录制数据

⚠️ **`/CERLAB/quadcopter/cmd_vel` 在自主飞行期间不发布**，只有键盘控制才发。教师控制标签必须取自 `target_state`。详见 `50_records/52_environment.md`。

### 1.3 版本状态（2026-09-07 核实）

| 项 | 值 |
|---|---|
| 仓库 | [github.com/Zhefan-Xu/CERLAB-UAV-Autonomy](https://github.com/Zhefan-Xu/CERLAB-UAV-Autonomy) |
| 本地 HEAD | `e045ce55`（2025-04-02） |
| 上游 HEAD | `e045ce55` —— **本地已是最新，无可用更新** |
| 各模块最后提交 | onboard_detector 2025-04 / map_manager 2024-10 / autonomous_flight 2024-04 / time_optimizer 2024-03 / global_planner 2024-01 / trajectory_planner、tracking_controller 2023-12 |

结论：上游基本处于停更状态，**不存在需要跟进的更新**。教师系统可视为稳定冻结的依赖。

### 1.4 特权信息落差

| | 教师 | 学生（部署时） |
|---|---|---|
| 深度 / 点云 | ✅ | ❌ |
| 局部地图 | ✅ | ❌ |
| 全局位姿 | ✅ | ❌ |
| 目标位置 | ✅ | ✅（**仅相对目标向量**） |
| 前视 RGB | ✅ | ✅ |
| 在线规划器 | ✅ | ❌ |

**这个落差是研究对象本身**，不是需要消除的缺陷。教师信号仅作为训练期监督标签，推理时一律不可用。

---

## 2. 学生统一底座

**所有实验臂共用同一底座，只在"头"和"损失"上不同。这是公平对照的前提。**

```
前视 RGB 图像 I_t  (480×640×3)
        │
        ▼
  ┌──────────────────────┐
  │  Backbone            │  MobileNetV2(width_mult=α)
  │  (容量轴 B)           │  → 特征维度 D(α)
  └──────────┬───────────┘
             │                    相对目标向量 g_t = [dx, dy, dz]（机体系）
             │                              │
             │                              ▼
             │                    ┌──────────────────┐
             │                    │  Goal Encoder    │  MLP(3 → 64)
             │                    └────────┬─────────┘
             ▼                             ▼
        ┌────────────────────────────────────┐
        │      concat → Temporal (LSTM)      │  窗口 T 帧
        └──────────────────┬─────────────────┘
                           │  h_t
        ┌──────────────────┴──────────────────────┐
        ▼                  ▼                      ▼
  ┌───────────┐    ┌──────────────┐    ┌─────────────────┐
  │ Ctrl Head │    │ CtrlSeq Head │    │ Trajectory Head │
  │  → u_t    │    │ → u_{t..t+H} │    │  → τ (K×3)      │
  │  (4,)     │    │  (H, 4)      │    │  训练期辅助      │
  └───────────┘    └──────────────┘    └─────────────────┘
```

### 2.1 各组件规格

| 组件 | 文件 | 输入 → 输出 | 说明 |
|---|---|---|---|
| **Backbone** | `kd_uav/models/backbone.py` | (B,3,H,W) → (B,D) | MobileNetV2，`width_mult` 可配。α=1.0 用 ImageNet 预训练；α<1.0 **无预训练权重，从头训练** |
| **Goal Encoder** | `kd_uav/models/goal_encoder.py` | (B,3) → (B,64) | 机体系相对目标向量 → MLP。备选：Loquercio 式归一化方向向量 |
| **Temporal** | `kd_uav/models/temporal.py` | (B,T,D+64) → (B,Dh) | LSTM，窗口 T 可配（初值 T=8） |
| **Ctrl Head** | `kd_uav/models/heads/control_head.py` | (B,Dh) → (B,4) | `[vx_b, vy_b, vz_b, yaw_rate]` 单步 |
| **CtrlSeq Head** | `kd_uav/models/heads/control_seq_head.py` | (B,Dh) → (B,H,4) | H 步控制序列，初值 H=5 |
| **Traj Head** | `kd_uav/models/heads/trajectory_head.py` | (B,Dh) → (B,K,3) | K=10 机体系路点，**部署时丢弃** |
| **Policy 容器** | `kd_uav/models/policy.py` | 由 YAML 组装 | 决定挂哪些头、用哪个 loss |

### 2.2 容量轴（B 轴）配置

| 配置 | width_mult α | 预训练 | 备注 |
|---|---|---|---|
| L（大） | 1.0 | ImageNet | 基准配置 |
| M（中） | 0.5 | 无，从头训练 | |
| S（小） | 0.35 | 无，从头训练 | |

⚠️ **风险**：α<1.0 从头训练收敛更慢，可能需要更多 epoch 或不同学习率。R2-W9 的压缩探针提前暴露此风险（比 R3 正式扫描早 3 周）。
⚠️ **待定**：骨干网络的最终选择（继续用 MobileNetV2 width_mult，还是换更适合从头训练的轻量骨干）待用户读完论文后决定。

---

## 3. 实验臂

### 3.0 什么是"臂"

**"臂"（arm）= 对照实验中的一组。** 本研究要回答"教师的哪一层知识最值得蒸馏"，做法是：

> **让同一个学生网络，用不同的教师知识去训练，然后比较结果。**
> **每一种训练方式 = 一个臂。**

所有臂共用**完全相同**的：学生网络结构、数据集、划分、优化器、部署接口。
**唯一的差别是：训练时教师给什么监督信号。**

这就是为什么臂的划分必须对应**教师的模块**——教师有几个模块，就有几种"层级知识"可以蒸馏：

```
CERLAB 教师                          对应的臂
─────────────────────────────────────────────────────
感知模块  → 深度/占据栅格/动态障碍  →  第 5 臂（待定）
   ↓
规划模块  → B样条轨迹 τ            →  TrajKD
   ↓
控制模块  → 控制序列 U             →  CtrlKD
   ↓
最终动作  → 单步指令 u             →  BC（无蒸馏基线）

           规划 + 控制 组合         →  JointKD
```

| 臂 | 教师给什么 | 直觉解释 |
|---|---|---|
| **BC** | 最终动作 `u_t` | "教师**做了**什么" —— 只学结果，不知道为什么 |
| **TrajKD** | 轨迹 `τ`（+ 动作） | "教师**打算走**哪条路" —— 学规划意图 |
| **CtrlKD** | 控制序列 `U_{t:t+H}` | "教师接下来**一连串**怎么操作" —— 学执行节奏 |
| **JointKD** | `τ` + `U` | 规划 + 执行，两层一起 |
| **第 5 臂** | 感知输出（待定） | "教师**看到**了什么" —— 学几何理解 |

**统一底座 + 不同监督信号。部署接口全部相同：图像 + 相对目标向量 → 速度指令。**

| 臂 | 训练监督 | 挂载的头 | 损失 | 部署输出 |
|---|---|---|---|---|
| **BC** | u_t（单步动作） | Ctrl | `L_BC = MSE(u_s, u_t)` | 单步 cmd_vel |
| **TrajKD** | τ（辅助）+ u_t（主） | Traj + Ctrl | `λ_traj·L_traj + L_ctrl` | 单步 cmd_vel（Traj 头丢弃） |
| **CtrlKD** | U_{t:t+H}（H 步序列） | CtrlSeq | `L_ctrl_seq = Σ_k w_k·MSE(u_s(t+k), u_T(t+k))` | receding horizon，执行 u_t |
| **JointKD** | τ（辅助）+ U（主） | Traj + CtrlSeq | `λ_traj·L_traj + λ_ctrl·L_ctrl_seq` | receding horizon（Traj 头丢弃） |
| **第 5 臂（感知层）** | 🟡 **待定**，见 §3.2 | — | — | — |

### 3.1 各臂的角色

| 臂 | 对应教师模块 | 在 A 轴上的角色 |
|---|---|---|
| BC | —（无蒸馏） | 基线：只有最终动作 |
| TrajKD | 规划（局部） | 规划层知识 |
| CtrlKD | 控制 | 执行层知识 |
| JointKD | 规划 + 控制 | 两层组合 |
| **第 5 臂** | **感知** | **感知层知识 —— 待定** |

⚠️ **JointKD 在 v2 中是消融臂，不是"提出的方法"**。原因见 `10_research/10_research.md` §1。

### 3.2 🟡 第 5 臂（感知层）—— 待定，R1 诊断后决策

**背景**：原计划设 FeatKD 臂，复现 Li & Zhao 2026 的 InfoNCE latent 对齐。**该方案已被否决**，原因：

> Li & Zhao 的 InfoNCE 对齐的是**教师网络的 embedding**。CERLAB 是 B 样条规划器 + PID 控制器，**没有 embedding**。任何"教师表征"都是人造产物，不是教师的知识 —— 这破坏 A 轴"教师各模块真实中间产物"的内在一致性。

**§1.2 的扫描结果解决了这个困境**：教师的**感知模块有真实的输出可蒸馏**，不需要人造表征。

#### 候选方案

| 方案 | 蒸馏标签 | 成本 | 说明 |
|---|---|---|---|
| **A. DepthAux** | `/camera/depth/image_raw`（降采样） | ~2–3 天 | 深度预测辅助头 + L1。最便宜，直接检验"单目缺几何信息"假设 |
| **B. OccAux** | `<ns>/2D_occupancy_map` (`OccupancyGrid`) | ~3–4 天 | ⭐ 规则网格，天然适合 dense prediction；且是教师**加工后**的产物（比原始深度更"像知识"） |
| **C. DynObsAux** | `<ns>/tracked_bboxes` + 速度 | ~4–5 天 | 动态障碍框与速度预测。最贴合"动态走廊"的应用叙事，也最能回答"为什么不用 VT&R" |
| **D. 不设第 5 臂** | — | 0 | A 轴保持 4 臂（动作/规划/执行/组合），在 discussion 中说明感知层未纳入的原因 |

#### 决策依据（R1 诊断后判定）

| 若 R1 诊断显示 | 倾向 |
|---|---|
| **M2 主导**（标签多峰 / 信息瓶颈） | 感知层臂的必要性强 → 选 A 或 B |
| **M1 主导**（类别不平衡） | 感知层臂必要性弱 → 可选 D 省时间 |
| 应用叙事需要强化"动态障碍"差异化 | → 选 C |

⚠️ **决策时点：R1 结束（W6）之后、R2-W8 之前。** 在此之前不投入实现工时。

---

## 4. 部署接口

```
部署时（教师不运行）：

  前视 RGB I_t  +  相对目标向量 g_t
            │
            ▼
   Backbone + Goal Encoder + LSTM
            │
            ▼
      Ctrl / CtrlSeq Head
            │
            ▼
  [vx_b, vy_b, vz_b, yaw_rate]
            │
            ▼
  /CERLAB/quadcopter/cmd_vel  (geometry_msgs/TwistStamped)
  linear.x/y/z = 机体系速度，angular.z = yaw_rate
```

- Gazebo 插件自动处理姿态稳定，学生只负责高层速度决策
- 起飞：`/CERLAB/quadcopter/takeoff` (Empty) + `/CERLAB/quadcopter/vel_mode` (Bool)
- **轨迹头在推理时不加载**；JointKD/TrajKD 保存一份剥离轨迹头的部署检查点
- **相对目标向量来自里程计**，不需要 3D 度量地图、不需要 LiDAR

---

## 5. 相对既有工作的架构差异

| | Li & Zhao 2026 | Loquercio 2021 | 本研究 |
|---|---|---|---|
| 教师 | RL 策略（SAC，全向深度） | 采样式规划器（完整 3D 地图） | **经典自主栈（模块化）** |
| 学生输入 | 单目 RGB + 目标 + LSTM | 深度图 + 状态 + 方向向量 | 单目 RGB + 相对目标 + LSTM |
| 蒸馏的知识 | 动作 + latent (InfoNCE) | 轨迹（多假设 R-WTA） | **动作 / 轨迹 / 控制序列 / latent（可切换）** |
| 平台 | 地面全向机器人 | 四旋翼 | 四旋翼 |
| 容量扫描 | ❌ | ❌ | **✅（B 轴）** |
| 扰动鲁棒性评估 | ❌ | ❌ | **✅（C 轴）** |

**学生底座（目标条件 + LSTM）借鉴 Li & Zhao，本研究的差异在于教师的可分解性以及 B/C 两轴。**

---

## 6. 代码结构（目标状态）

```
kd_uav/
├── models/
│   ├── backbone.py            MobileNetV2(width_mult)
│   ├── goal_encoder.py        相对目标向量编码
│   ├── temporal.py            LSTM 时序融合
│   ├── heads/
│   │   ├── control_head.py        单步 (4,)
│   │   ├── control_seq_head.py    H 步 (H,4)
│   │   └── trajectory_head.py     K 路点 (K,3)
│   └── policy.py              由 YAML 组装的统一容器
├── losses/
│   ├── bc_loss.py             加权 MSE
│   ├── traj_kd_loss.py
│   ├── ctrl_kd_loss.py        H 步加权 MSE
│   ├── joint_loss.py
│   └── feat_kd_loss.py        InfoNCE
├── datasets/
│   ├── dataset_builder.py     raw → processed（含 goal 字段、退化轨迹过滤）
│   ├── uav_kd_dataset.py      序列采样、平衡采样
│   └── inspect_dataset.py
├── train.py                   ⭐ 单一入口，YAML 驱动
├── eval/
│   ├── eval_offline.py
│   ├── eval_robustness.py     扰动套件
│   └── compute_metrics.py
└── diagnostics/               R1 的 T1–T5

configs/
├── base.yaml                  共享默认值
├── arms/{bc,traj_kd,ctrl_kd,joint_kd,feat_kd}.yaml
└── capacity/{l,m,s}.yaml      α = 1.0 / 0.5 / 0.35
```

**关键工程决策：单一 `train.py` + YAML 组合。**
v1 是"每个方法一个训练脚本"，但 v2 需要 5 臂 × 3 容量 = 15 个配置，那种结构会崩。
**验收标准：换臂或换容量只改 YAML，不改代码。**

### ROS 包

| 包 | 状态 | v2 任务 |
|---|---|---|
| `uav_kd_data_collector` | 已实现（432 行） | **改造**：增加目标点记录（约 +50 行） |
| `uav_kd_student_policy` | 已实现（BC 推理） | **改造**：支持 goal + LSTM；废弃 red_wp / self_red_wp 节点 |
| `uav_kd_eval_tools` | **空包** | **新建**：SR/CR/轨迹误差自动测量 + 批量部署脚本 |
