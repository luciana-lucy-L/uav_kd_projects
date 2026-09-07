# DONE — 项目现状快照

> 快照日期：2026-07-01。Baseline（BC）阶段刚收尾，下一步是 TrajKD 实现。
> 本文档是对 `research.md`（研究设计）和 `CLAUDE.md`（实现规范）的一次回顾整理，
> 汇总现有数据、未来对比需要的数据、以及**已明确确认**的模型结构。
> 完整逐条实验记录见 `docs/04_experiment_log.md`；数据格式定义见 `docs/03_data_schema.md`。

---

## 1. 现有数据

### 1.1 原始 episode（`data/raw/`）

| Episode | 帧数 | 状态 |
|---|---|---|
| ep_test_001 | 864 | ❌ traj_body 无效（bug：路点在身后）|
| ep_test_002 | 875 | ❌ traj_body 无效（bug：dx0 振荡）|
| ep_test_003 | 1013 | ✅ 直飞为主 |
| ep_test_004 | 958 | ✅ 直飞为主 |
| ep_test_005 | 569 | ✅ 直飞为主 |
| ep_test_006 | 962 | ✅ 旋转帧占比 18.2%（\|yaw_rate\|>0.15）|
| ep_test_007 | 4405 | ✅ 旋转帧占比 22.9%，最长一条 |
| ep_008 | 2457 | ✅ 旋转帧占比 4.2% |
| ep_009 | 1337 | ✅ 旋转帧占比 6.2%（含一段约19帧长退化 traj，已被过滤）|

### 1.2 已构建数据集（`data/processed/`）

| 数据集 | train/val | 使用的 episode | ctrl_body mean [vx,vy,vz,yr] | valid_traj 比例 | 用途 |
|---|---|---|---|---|---|
| uav_kd_debug_v000 | 790/0 | ep_003 | [0.674,-0.023,0.001,0.008] | — | pipeline 验证，不用于训练 |
| uav_kd_debug_v001 | 1971/0 | ep_003+004 | [0.629,0.003,0.001,0.0001] | 0.926 | pipeline 验证，不用于训练 |
| uav_kd_v001 | 6376/569 | train: 003+004+007 / val: 005 | [0.386,0.004,0.0004,-0.003] | 0.867 | 旧 BC baseline 数据集；val 100% 直飞，无旋转样本 |
| **uav_kd_v002** | 10170/1531 | train: 003+004+007+008+009 / val: 005+006 | [0.326,0.005,0.0004,-0.011] | 0.853 | **当前推荐使用**；val 含 ~11.4% 旋转样本，且修复了 spot-turn 退化 traj 问题 |
| uav_kd_red_wp_v001 | 3040/475 | train: 003+004+007 / val: 005 | [0.607,-0.012,0.0004,-0.007] | 1.0 | BC-RedWP 专用（图像叠加红色路点，仅保留 wp_visible=True 帧）|

**遗留已知问题：**
- `dyaw` 在所有数据集里恒为 0（bspline_trajectory 的路点朝向信息未被存入 CSV，需从 rosbag 重新提取才能补全）
- 所有数据集均按 **episode** 划分 train/val（不做随机帧划分），符合 `research.md` §6.3 的规则
- ep_001/ep_002 的 traj_body 有 bug，未被任何正式数据集使用
- v002 已修复"UAV 原地旋转时 bspline 不重新发布导致 traj 全零"的退化路点问题（`traj_magnitude_threshold` 过滤）

---

## 2. 未来可能需要用于对比的数据

`research.md` §8 定义了统一的评估方案，目前只有 BC 一个方法产出了部分数据，其余留空待补。

### 2.1 离线指标（Offline Metrics）

| Metric | BC (v001) | BC-RedWP | BC (v002) | TrajKD | CtrlKD | JointKD |
|---|---|---|---|---|---|---|
| Val loss | 0.011639 | 0.011610 | 0.052565 | — | — | — |
| vx MSE | 0.0290 | 0.0290 | 0.1014 | — | — | — |
| vy MSE | 0.0174 | 0.0174 | 0.0209 | — | — | — |
| vz MSE | ~0.0001 | 0.0001 | ~0 | — | — | — |
| yaw_rate MSE | ~0 | 0.0000 | 0.0879 | — | — | — |
| Trajectory ATE | n/a | n/a | n/a | — | — | — |
| Model size | 2.55M params | 2.55M params | 2.55M params | — | — | — |
| Inference FPS | 未测量 | 未测量 | 未测量 | — | — | — |

> 注：v001/BC-RedWP 的 val 集 100% 直飞（旋转 MSE≈0 是"预测0就赢"的假象），不能和 v002 直接比较。
> **后续 TrajKD/CtrlKD/JointKD 应统一在 uav_kd_v002 上评估**，以 BC(v002) 的结果（val loss 0.0526，yr MSE 0.0879）作为公平基线。

### 2.2 Gazebo 部署指标（Deployment Metrics）

`research.md` §8.2 要求的指标：Success Rate、Collision Rate、Trajectory Error（vs teacher 参考路径）、Completion Time、Real-time FPS。

**现状：尚未对任何方法正式测量。** 仅对 BC(v002) 做过一次定性冒烟测试（2026-07-01，`corridor_dynamic_9.world`，稳定飞行 2+ 分钟无崩溃，yaw_rate 输出非退化，详见 `docs/04_experiment_log.md`），未按固定路线跑多次、未记录成功率/碰撞率等定量数字。

**这是当前最大的空缺**：在做 ablation 定量对比之前，需要先设计一套可重复的多轮部署测试协议（固定路线、固定 N 次运行/方法），否则 SR/CR 等数字无意义。

### 2.3 待补的超参数扫描（research.md 已指定）

- TrajKD：λ ∈ {0.1, 1.0, 5.0}
- CtrlKD：H ∈ {3, 5, 10}
- JointKD：λ_traj × λ_ctrl 网格，{0.1, 1.0, 5.0}²

---

## 3. 已完成的模型结构

> **范围说明**：本节只收录**已经过明确确认**（训练完成、结果已讨论并认可）的模型，不包括仅存在于代码库中但未训练/未确认的结构。

目前只有 **BC** 一个模型满足这个标准——已用三份不同数据训练（v001 / BC-RedWP / v002）并在 Gazebo 中部署验证过，结果均已在对话中讨论确认。

### 3.1 StudentBackbone（`kd_uav/models/student_backbone.py`）

- **输入**：`(B, 3, H, W)` RGB float32，范围 `[0, 1]`
- **输出**：`(B, 1280)`
- **结构**：MobileNetV2（`torchvision.models.mobilenet_v2`，ImageNet1K_V1 预训练权重）的 `.features` → `AdaptiveAvgPool2d(1)` → flatten
- **超参数**：`pretrained=True`，`freeze=False`（所有已跑的训练中 backbone 均整体微调，未冻结）

### 3.2 ControlHead + BCPolicy（`kd_uav/models/control_head.py`）

- **输入**：`(B, 1280)` 视觉特征
- **输出**：`(B, 4)` = `[vx_b, vy_b, vz_b, yaw_rate]`
- **网络结构**：`Linear(1280,256) → ReLU → Dropout(0.3) → Linear(256,4)`
- **BCPolicy** = StudentBackbone + ControlHead，端到端图像→控制指令，共 **2.55M 参数**
- **损失函数**：`BCLoss`（`kd_uav/losses/bc_loss.py`）——逐维加权 MSE，所有已跑训练权重均为 `[1,1,1,1]`（均匀）
- **训练超参数**（`configs/train/bc*.yaml`）：Adam 优化器，lr=1e-4，weight_decay=1e-4，cosine 学习率调度（3 epoch warmup），batch_size=32，50 epochs，dropout=0.3

---

## 4. 已完成的训练记录

| Run | 数据集 | Epochs | Best val loss | Final val loss | 备注 |
|---|---|---|---|---|---|
| `runs/bc/20260629_151702` | uav_kd_v001 | 50 | 0.011639 | 0.011841 | 旧 baseline；val 无旋转样本 |
| `runs/bc_red_wp/20260629_180554` | uav_kd_red_wp_v001 | 50 | 0.011610 (epoch 31) | 0.012517 | 图像叠加红点变体；离线指标与 v001 基本持平 |
| `runs/bc_v002/20260701_142506` | uav_kd_v002 | 50 | 0.052565 (epoch 23) | 0.054072 | val 含真实旋转样本；yaw_rate 学习信号从退化变为真实但偏弱；vx 出现 episode 级泛化 gap（已知问题，未阻塞）；已在 Gazebo 部署验证 |

---

## 5. 目前进度

在第1-4节记录的 BC baseline 之后，又实现并测试了两个新方向（架构细节暂不并入第3节，避免改动已确认内容——完整细节见 `docs/04_experiment_log.md` 对应日期条目）：

- **TrajKD**（2026-07-02）：backbone 共享 + trajectory head（辅助 loss）+ control head（部署输出）。离线 ctrl loss 优于 BC(v002)（0.0487 vs 0.0541），但 Gazebo 部署测试发现 yaw_rate 标准差比 BC 低 4 倍，几乎不转弯，卡在障碍物前。
- **SelfRedWP**（2026-07-04）：不用隐式辅助 loss，改为把预测轨迹渲染成红点、control head 直接基于带红点的图像做决策，配合课程学习（真值路点→自预测路点，解决 train/deploy 分布不一致）。离线指标全面优于前两者（ctrl loss 0.0459，各维 MSE 更低，且训练全程 yr_std 未坍缩），**但实际 Gazebo 部署仍然卡在和 TrajKD 几乎同一个位置**。

三个模型（BC(v002) / TrajKD / SelfRedWP）的离线指标对比图、部署 yaw_rate 诊断对比图（含教师参考线）、训练损失曲线对比图，见 `results/figures/compare_*.png`。

## 6. 目前问题

**核心问题：三个学生模型都卡在障碍物前，没有一个真正学会教师级别的转弯能力。**

把三个模型的部署期 yaw_rate 统计和教师（CERLAB）自己录制数据的统计放在一起看，差距非常悬殊：

| | 教师（CERLAB 录制）| BC(v002) 部署 | TrajKD 部署 | SelfRedWP 部署 |
|---|---|---|---|---|
| yaw_rate std | **0.340** | 0.104 | 0.027 | 0.042 |
| frac(\|yr\|>0.15) | **17.7%** | 10.4% | 0.9% | 3.4% |

表现最好的 BC(v002) 也只达到教师 std 的约 30%、强转弯占比的约 59%。TrajKD 和 SelfRedWP 更低。也就是说，"哪个方法更好"这个问题目前次要于"没有一个方法接近教师水平"——三者之间的差距，远小于任一学生和教师之间的差距。

**方法论教训**：SelfRedWP 专门设计了"验证集用自预测路点"的诊断指标，希望能提前发现部署时的问题，但该指标（离线 yr_std=0.133）严重高估了实际部署表现（实际 yr_std=0.042）。原因是离线验证集是固定的历史帧，无法复现"模型自己的动作导致画面停止变化，从而卡住"这种闭环反馈——这类失效模式目前只能靠真实部署测试发现，offline 指标（哪怕专门设计过）不能替代它。

详见 `docs/04_experiment_log.md` 2026-07-02（TrajKD）、2026-07-04（SelfRedWP）条目。

## 7. 下一步

按 `CLAUDE.md` 当前优先级：CtrlKD、JointKD 待实现。但鉴于第6节的发现，在继续堆方法之前，可能更值得先排查"为什么所有学生都远低于教师转弯能力"这个共性根因（是数据分布问题、单帧视觉输入的信息瓶颈，还是别的），否则新方法大概率重复同样的失败模式。
