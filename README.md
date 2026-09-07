# UAV Knowledge Distillation Project

**从可分解的经典自主飞行系统向轻量单目视觉导航策略的知识蒸馏**

> 状态更新：2026-09-07（v2 研究方向确立） · 详细文档见 [`docs/00_INDEX.md`](docs/00_INDEX.md)

---

## 一句话

用 CERLAB 经典自主飞行栈（感知 + B样条规划 + 跟踪控制）作为**可分解的教师**，系统研究：**它内部的哪一层知识最值得蒸馏给资源受限的单目视觉学生，以及这个答案如何随学生容量和输入扰动而变化。**

---

## 研究设计

### 三个轴

| 轴 | 内容 | 取值 |
|---|---|---|
| **A. 知识层级** | 训练监督信号来源 | BC（动作）/ TrajKD（规划层）/ CtrlKD（执行层）/ JointKD（规划+执行）/ FeatKD（表征层） |
| **B. 学生容量** | 网络规模（压缩） | α = 1.0 / 0.5 / 0.35 |
| **C. 扰动鲁棒性** | 评估时的输入扰动 | 干净 / 噪声 / 模糊 / 亮度 / 对比度 /（保留：稀疏对抗） |

**核心假设**：结构化的教师知识（轨迹 / 控制序列），相比纯动作模仿，让小容量学生在压缩后与受扰动时退化得更慢。

**为什么只有本研究能问这个问题**：经典教师是模块化的，中间产物（轨迹、控制序列）语义清晰、可分别摘取。神经教师（RL 策略、特权网络）内部没有"规划层"可摘。

### 教师 / 学生

| | 教师（CERLAB） | 学生（部署时） |
|---|---|---|
| 深度 / 点云 / 地图 / 全局位姿 | ✅ | ❌ |
| 前视 RGB | ✅ | ✅ |
| 目标 | 绝对坐标 | 仅**机体系相对目标向量** |
| 输出 | 轨迹 τ + 控制指令 u | `[vx_b, vy_b, vz_b, yaw_rate]` |

---

## 当前进度

| 轮次 | 周次 | 主题 | 状态 |
|---|---|---|---|
| **R1** | W1–W6 | 诊断 + 地基 | 🔄 **进行中（W1）** |
| R2 | W7–W11 | 知识层级轴 | ⬜ |
| R3 | W12–W16 | 压缩轴 + 鲁棒性轴 | ⬜ |
| R4 | W18–W21 | 保留项 + 定稿 | ⬜ |

**下一个动作**：诊断实验 T1 / T2 / T4（[`docs/41_diagnostics.md`](docs/41_diagnostics.md)）
**下一个决策门**：G0（W2 末）

### R1 要解决的核心问题

v1 的三个学生模型全部无法复现教师的转向能力：

| | 教师 | BC(v002) | TrajKD | SelfRedWP |
|---|---|---|---|---|
| yaw_rate std | **0.340** | 0.104 | 0.027 | 0.042 |
| frac(\|yr\|>0.15) | **17.7%** | 10.4% | 0.9% | 3.4% |

三个候选机制（类别不平衡 / 标签多峰 / 辅助 loss 梯度干扰）需要先用五个廉价实验分开，**再决定采什么数据**。

---

## 目录结构

```
uav_kd_project/
├── README.md              本文件（一页现状）
├── CLAUDE.md              工作规则 + 文档索引
├── docs/                  全部文档（见 docs/00_INDEX.md）
│   └── _archive/          v1 历史版本（只读）
├── kd_uav/                Python 训练包
├── configs/               YAML 配置（arms / capacity）
├── ros_ws/src/            ROS 包（采集 / 部署 / 评估）
├── data/raw|processed/    原始 / 处理后数据
├── runs/                  训练输出
├── results/               表格 / 图表 / 部署记录
├── scripts/               启动脚本
└── legacy/                v0 的状态式 BC 存档（非正式基线）
```

---

## 外部依赖（不可修改）

| 依赖 | 路径 | 作用 |
|---|---|---|
| CERLAB Autonomy Stack | `/home/l/cerlab_ws/src/CERLAB-UAV-Autonomy/` | **教师系统** |
| UAV Simulator | `/home/l/catkin_ws/src/uav_simulator/` | Gazebo world 与 UAV 模型 |

---

## 快速开始

```bash
# 环境（训练）
source ~/miniconda3/etc/profile.d/conda.sh && conda activate dbc241

# 环境（ROS）—— 必须先 conda deactivate
source ~/miniconda3/etc/profile.d/conda.sh && conda deactivate
source /home/l/catkin_ws/devel/setup.bash
source /home/l/cerlab_ws/devel/setup.bash
```

完整启动序列、话题映射、常用命令见 [`docs/52_environment.md`](docs/52_environment.md)。

---

## 时间线

2026-09-07 → 2027-01-31（21 周），4–5 天/周 × 5–8 h。
四轮螺旋：R1+R2 完成即构成合格论文主体，R3 补齐压缩与鲁棒两轴，R4 整轮可牺牲。
详见 [`docs/30_plan.md`](docs/30_plan.md) 与 [`docs/31_schedule.md`](docs/31_schedule.md)。
