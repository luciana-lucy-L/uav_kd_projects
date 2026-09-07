# 相关工作对照与威胁分析

> 层级：研究层。**读到新的相关论文时修改本文件。**
> 版本：v2（2026-09-07）。研究定位见 `10_research/10_research.md`
> PDF 存放于 `/home/l/Downloads/papers/`

---

## 0. 可信度标签

| 标签 | 含义 |
|---|---|
| ✅ **核实** | 抓取论文原文/摘要页读到的具体细节，可直接引用 |
| ⚠️ **二手** | 来自搜索摘要、代码库标题或页面片段，细节需查 PDF 复核 |
| 🔶 **判断** | 推理与评估，**不是事实**，可反驳 |

---

## 1. 威胁矩阵总览

| 想声称的东西 | 占位者 | 等级 | 结论 |
|---|---|---|---|
| 经典系统（非神经）当教师 | Loquercio 2021、Tube-NeRF 2023 | 🔴 高 | ❌ 不可作 novelty |
| 蒸馏轨迹/规划知识到 UAV 视觉策略 | Loquercio 2021 | 🔴 高 | ❌ |
| 辅助监督优于纯动作模仿 | Li & Zhao 2026、TCP 2022、CILRS 2019 | 🔴 高 | ❌ |
| 轨迹头 + 多步控制头联合训练 | TCP 2022 | 🔴 高 | ❌ JointKD 方法层归零 |
| 特权教师 → 纯视觉学生范式 | LBC 2019、Lee 2020 | 🔴 高 | ❌ |
| 轻量 MobileNet 学生 | Loquercio 2021（MobileNet-V3） | 🟠 中 | ❌ 不可当卖点 |
| UAV 导航的极限压缩 | Tiny-PULP-Dronet 系列 | 🔴 高 | ❌ 不在压缩率上竞争 |
| 卖通用预训练导航模型 | GNM / ViNT / NoMaD | 🔴 高 | ❌ 放弃该路线 |
| 对比学习提升**场景迁移**鲁棒性 | Xing 2024、arXiv:2606.05506 | 🟠 中 | ⚠️ 必须限定为"**扰动/对抗**鲁棒性" |
| "教师飞一圈、学生重复作业"应用形态 | VT&R 全家 | 🟠 中 | ⚠️ 可说，须答"为什么不用 VT&R" |
| 单目 RGB（无深度）学生 | Tube-NeRF 部分占用 | 🟡 低 | ⚠️ 不能单独支撑差异 |
| 动态障碍走廊 + 3D 机动 | 无 | 🟡 低 | ✅ 弱，需组合 |
| **可分解教师的知识层级对照** | **无人占位** | 🟢 空白 | ✅ **可支撑** |
| **知识层级 × 学生容量的交互** | **无人占位** | 🟢 空白 | ✅ **可支撑** |
| **蒸馏知识类型对扰动鲁棒性的影响** | **无人占位** | 🟢 空白 | ✅ **可支撑** |

---

## 2. A 组：特权/经典教师 → 视觉学生

### 2.1 Loquercio et al. 2021 — *Learning High-Speed Flight in the Wild* 🔴 最高威胁

**出处**：Science Robotics 6(59) · [arXiv:2110.05113](https://arxiv.org/abs/2110.05113) · [Science](https://www.science.org/doi/10.1126/scirobotics.abg5810) · 本地 `01 Learning High-Speed Flight in the Wild.pdf`

#### 核心事实 ✅

| 组件 | 内容 |
|---|---|
| **教师** | **采样式运动规划器（Metropolis-Hastings 采样）**，特权信息 = 完整平台状态 + 环境完整 3D 地图/点云，且**无计算预算限制** |
| 教师输出 | 从 `P(τ\|τ_ref, C)` 采样碰撞-free 轨迹，三次 B 样条（3 控制点），取代价最低的 **3 条**作训练标签 |
| **学生输入** | 深度图 640×480（SGM）+ 平台速度 ℝ³ + 姿态（9 维旋转矩阵）+ **期望飞行方向**（指向 1 秒后参考点的归一化向量） |
| **学生输出** | **M=3 条候选轨迹**（各 10 个位置/1 秒）+ 每条的碰撞代价 |
| 骨干 | **预训练 MobileNet-V3** |
| **损失** | Relaxed Winner-Takes-All：`L = λ₁·R-WTA(T_e,T_n) + λ₂·Σ‖c_k − C_collision(τ_nk)‖²` |
| **训练** | **DAgger**：滚动学生策略 → 教师标注学生访问状态；每 30 个环境放宽跟踪阈值；约 **90K 样本** |
| 目标指定 | 上层给一条**不必碰撞-free**的参考轨迹，网络用"指向 1 秒后参考点的归一化向量" |
| 验证 | 纯仿真训练，零样本迁移到真实森林/雪地/废弃火车/坍塌建筑 |

#### 对本研究的影响 🔶

**占用**：经典规划器当教师 + 轨迹蒸馏 + 轻量 MobileNet + 四旋翼避障 —— v1 定位的核心组合被完整覆盖。

**未占用**：单目 RGB（他们用深度）、执行层（控制器）知识的蒸馏、**知识层级的对照实验**（他们只蒸馏一层）、容量扫描、扰动鲁棒性。

**可借用的三样工具**：
| 他们的做法 | 对本研究的意义 |
|---|---|
| **R-WTA 多假设输出** | 对"标签多峰 → MSE 回归到均值"的标准解法。本项目 yaw_rate 坍缩正是此病（`40_experiments/41_diagnostics.md` M2） |
| **DAgger** | 本项目教师在同一 Gazebo 中在线可跑，DAgger 近乎免费，且解决"学生卡住后回不来"的闭环失效 |
| **方向向量作目标输入** | 廉价目标指定的存在证明之一 |

---

### 2.2 Li & Zhao 2026 — *Omni-View and Cross-Modality KD for Mobile Robots* 🔴 最高相似度

**出处**：[arXiv:2603.20679](https://arxiv.org/abs/2603.20679)，2026-03-21，**已接收 ICRA 2026** ⚠️ · 作者单位：浙江大学 / 西湖大学 · 本地 `02 Enhancing Vision-Based Policies...`

#### 核心事实 ✅

| 组件 | 内容 |
|---|---|
| **教师** | **RL 策略（SAC，5M 步）**，输入 = 全向深度图（DepthAnythingV2 把 4 路 RGB 转深度后拼接）→ **不是经典规划器，是黑箱** |
| **学生** | 单目 RGB → ResNet → 与 **goal embedding** 拼接 → **LSTM** → MLP |
| **目标指定** | **机体局部系 2D 目标点**，MLP 编码后与图像特征拼接 |
| **损失** | `L_s = λ₀·L_act + λ₁·L_InfoNCE`；正样本对 = 全局位姿空间接近的师生 embedding；λ₁ 从 0.9 线性衰减到 0.1 |
| 平台 | 地面**全向移动机器人**，Jetson Orin NX（25 W） |
| 环境 | ROS Gazebo，**4 m × 4 m** |
| 数据 | 50,000 组 (state, image, action)，4 个仿真环境 + 1 真实实验室 |
| 评估 | 80 次仿真 + 20 次真机；Action Error / SR（0.3 m 无碰撞）/ Moving Distance / 板载延迟 |
| 结果 | **72% SR**（MAE 50%，单视角 RGB 基线 40%） |
| 基线 | ResNet18/50 各模态、DINOv2 / CLIP / MAE / MultiMAE、RoboSaGA、VISARL |
| **代码** | [github.com/xiaowei1015/robot-kd](https://github.com/xiaowei1015/robot-kd) —— ⚠️ **仓库基本为空**（README + 1 commit），板载代码"待论文接收后上传"。**FeatKD 按从零实现估工时** |

#### 关键洞察 🔶

**他们的教师是 RL 黑箱，内部没有"规划层""执行层"之分。可提取的知识只有动作与中间 embedding —— 用 InfoNCE 对齐 latent 不是设计偏好，是被教师结构逼出来的唯一选择。**

这正是本研究的立足点：可分解的经典教师能问一个他们结构上问不出来的问题（`10_research/10_research.md` §2.1）。

#### 可借用

| 他们的做法 | 对本研究的意义 |
|---|---|
| **相对 2D 目标点 + MLP 编码** | 廉价目标指定的存在证明，只需里程计。本项目 `traj_body` 已是机体系相对坐标，改造成本极小 |
| **LSTM 时序融合** | 补上"单帧无记忆"的缺陷 |
| **InfoNCE latent 对齐** | 作为 FeatKD 臂（表征层知识）的对照基线 |

⚠️ **动机段落几乎逐字撞车**（"可迁移性/算力/传感器成本三难"）。本研究的 Introduction 必须换一种切入方式。

---

### 2.3 Chen et al. 2019 — *Learning by Cheating*

**出处**：CoRL 2019 · [arXiv:1912.12294](https://arxiv.org/abs/1912.12294) · [PMLR](https://proceedings.mlr.press/v100/chen20a/chen20a.pdf) · [代码](https://github.com/dotchen/LearningByCheating) · 本地 `03 Learning by cheating.pdf`

- ✅ 两阶段：先训**特权智能体**（观察环境真值布局与所有交通参与者位置），再由它作为教师训练**纯视觉感知-运动智能体**
- ✅ CARLA benchmark 100% 成功率，刷新 NoCrash 记录
- ⚠️ 学生预测**路点**再由低层控制器跟踪（据代码库标题，正文未核实）

🔶 **威胁**：这是"特权教师 → 纯视觉学生"两阶段范式的祖先。审稿人最可能问的一句是"**这不就是 LBC 换到无人机上吗**"。
⚠️ **v1 的 `research.md` 甚至没有引用 LBC —— 这是最紧急的引用缺口，必须在论文中正面区分。**

---

### 2.4 Lee et al. 2020 — *Learning Quadrupedal Locomotion over Challenging Terrain*

**出处**：Science Robotics 5(47) · [arXiv:2010.11251](https://arxiv.org/abs/2010.11251) · 本地 `05 ...pdf`

- ✅ 特权教师（地形真值）→ 仅本体感受学生；零样本迁移到泥地/雪地/碎石/植被

🔶 **用途**：机器人领域 teacher-student 特权蒸馏的标杆引用。它的存在直接否定"KD 大多是 NN→NN"这一前提。**不引它会被认为文献调研不足。**

---

### 2.5 Tube-NeRF 2023 — *Imitation Learning of Visuomotor Policies from MPC*

**出处**：[arXiv:2311.14153](https://arxiv.org/abs/2311.14153) · 本地 `06 Tube-NeRF...pdf`

- ⚠️ 以 MPC（经典最优控制器）为专家，tube-guided 数据增强 + NeRF 合成视角
- ✅ **用户核实（2026-09-07）：平台为四旋翼，学生输入 RGB，有真机实验**

🔶 **威胁等级上调**：它占用了"四旋翼 + 单目 RGB + 经典控制器教师 + 真机"这一组合。
**剩余差异**：其教师是 MPC（纯控制器，**无可分解的规划层**），任务是视觉运动控制而非避障导航；其 tube-guided + NeRF 增强是解决协变量偏移的另一路径。
**结论：单目 RGB 不能单独作为本研究的差异点，必须靠三轴交叉。**

---

## 3. B 组：模型压缩 × UAV 导航（教授指派题目的对标）

### 3.1 Tiny-PULP-Dronet 系列 🔴 压缩轴直接竞品

| 论文 | 链接 |
|---|---|
| *Distilling Tiny and Ultra-fast DNNs for Autonomous Navigation on Nano-UAVs* | [arXiv:2407.12675](https://arxiv.org/abs/2407.12675) |
| *Tiny-PULP-Dronets* | [arXiv:2407.02405](https://arxiv.org/abs/2407.02405) |

⚠️ 核心事实：蒸馏得到比 PULP-Dronet 更强的 CNN，**内存压缩 168×（降至 2.9 kB）**，139 fps，在"狭窄有障碍走廊 + 180° 转弯"的未见路径上 **100% 成功率** —— 任务形态与本项目走廊几乎一样。

🔶 **对本研究的意义**：
- **不要在压缩率上竞争**，这条路已被走到头
- 但它是 **NN→NN 蒸馏**，**不问"蒸馏哪一层知识"** —— 这正是本研究的差异
- 是压缩轴必须引用的对标

### 3.2 KD 的容量差距理论（压缩轴的理论支点）

| 论文 | 链接 | 对本研究的价值 |
|---|---|---|
| Cho & Hariharan, *On the Efficacy of Knowledge Distillation* (ICCV 2019) | [arXiv:1910.01348](https://arxiv.org/abs/1910.01348) | ⚠️ **更大的教师不一定是更好的教师**；容量不匹配使小学生无法模仿大教师。本研究的经典教师是"无限大教师"的极端情形 —— 这是连接压缩题目最自然的理论入口 |
| Mirzadeh et al., *Improved KD via Teacher Assistant* (AAAI 2020) | [arXiv:1902.03393](https://arxiv.org/abs/1902.03393) · [代码](https://github.com/imirzadeh/Teacher-Assistant-Knowledge-Distillation) | ⚠️ 用中间尺寸"助教"桥接容量差。🔶 延伸想法：能否用一个"中等学生"当助教？ |
| Stanton et al., *Does Knowledge Distillation Really Work?* (NeurIPS 2021) | [arXiv:2106.05945](https://arxiv.org/abs/2106.05945) | ⚠️ **学生即使有足够容量也匹配不上教师**，区分 fidelity 与 generalization。🔶 **本项目"学生 yaw_rate std 只有教师 30%"正是 fidelity 问题** |
| *Student Capacity Moderates KD Effectiveness* (2026) | [arXiv:2605.31191](https://arxiv.org/abs/2605.31191) | ⚠️ 学生容量如何调节 KD 效果的系统研究，方法论可借鉴 |

---

## 4. C 组：鲁棒性 × 蒸馏（实验室主轴）

| 论文 | 链接 | 关系 |
|---|---|---|
| Goldblum et al., *Adversarially Robust Distillation* (AAAI 2020) | [AAAI](https://ojs.aaai.org/index.php/AAAI/article/view/5816) · [代码](https://github.com/goldblum/AdversariallyRobustDistillation) | ⚠️ **鲁棒性可通过蒸馏从教师传给学生**，即使只在干净图像上蒸馏。🔶 本研究的新问题：**经典教师没有"对抗鲁棒性"可传，但有几何知识 —— 几何知识能否带来鲁棒性？** |
| *How and When Adversarial Robustness Transfers in KD* | [arXiv:2110.12072](https://arxiv.org/abs/2110.12072) | ⚠️ 鲁棒性传递的条件分析 |
| Su, Vargas, Sakurai, *One Pixel Attack* | [arXiv:1710.08864](https://arxiv.org/abs/1710.08864) · 本地 `10 ...pdf` | ⚠️ **本实验室的招牌工作**。🔶 把稀疏对抗攻击用在**连续控制策略**上（现有工作几乎都是分类），攻击目标改为最大化 \|Δyaw_rate\| —— 实验室主场 + 学术空白 |

### ⚠️ 措辞警戒：场景迁移 ≠ 扰动鲁棒性

| 论文 | 链接 | 占用了什么 |
|---|---|---|
| Xing et al., *Contrastive Learning for Robust Scene Transfer in Vision-based Agile Flight* (ICRA 2024) | [arXiv:2309.09865](https://arxiv.org/abs/2309.09865) | 四旋翼 + 对比学习 + 零样本**场景迁移** + 真机 |
| *Robust Scene Transfer for PointGoal Navigation via Privileged-Sensor-Guided Contrastive Learning* (2026) | [arXiv:2606.05506](https://arxiv.org/abs/2606.05506) | 特权传感器 + 对比学习 + 目标条件导航 |

🔶 **结论：C 轴必须限定为"输入扰动鲁棒性与对抗鲁棒性"**（输入被破坏时策略是否失效），**不要笼统说"鲁棒性"**，那一块已被"场景迁移"占据。

---

## 5. D 组：轨迹 + 控制双头（JointKD 的直接前身）

### 5.1 Wu et al. 2022 — *TCP* 🔴 对 JointKD 最直接的威胁

**出处**：NeurIPS 2022 · [arXiv:2206.08129](https://arxiv.org/abs/2206.08129) · 本地 `04 ...pdf`

摘要原文 ✅：
> "our integrated approach has two branches for trajectory planning and direct control... the **control branch involves a novel multi-step prediction scheme**... **The two branches are connected so that the control branch receives corresponding guidance from the trajectory branch at each time step.** The outputs from two branches are then fused..."

| 本研究的 JointKD | TCP |
|---|---|
| 轨迹头（辅助）+ H 步控制序列头 | 轨迹分支 + **多步预测**控制分支 |
| 共享 backbone | 共享 backbone |
| 两个 loss 加权 | 两分支**相互连接**（耦合更深） |
| 部署丢弃轨迹头 | 部署融合两分支 |

🔶 **JointKD ≈ TCP 减去分支间连接与融合。架构层面是做减法，不是提出。方法层 novelty 归零。**
→ v2 中 JointKD 降级为消融臂（`10_research/10_research.md` §7）。

### 5.2 CILRS (Codevilla et al. 2019)

[arXiv:1904.08980](https://arxiv.org/abs/1904.08980) —— 用辅助任务（速度预测）改善 BC backbone。与 TCP、Li & Zhao 一起构成"辅助监督改善 BC"这一命题的三重占位。

### 5.3 *Exploring the Limitations of Behavior Cloning for Autonomous Driving*

本地 `09 ...pdf` ⚠️ 未详读。BC 的局限性分析（数据偏差、泛化、因果混淆），🔶 与本研究的诊断章节（`40_experiments/41_diagnostics.md`）直接相关，**建议优先读**。

---

## 6. E 组：应用形态与目标指定

### 6.1 Visual Teach & Repeat

| 论文 | 链接 | 本地 |
|---|---|---|
| Clement et al., *Monocular VT&R Aided by Local Ground Planarity* | [arXiv:1707.08989](https://arxiv.org/abs/1707.08989) | `0701 ...pdf` |
| Warren et al., *Towards VT&R for GPS-Denied Flight of a Fixed-Wing UAV* (FSR 2017) | [PDF](https://www.dynsyslab.org/wp-content/papercite-data/pdf/warren-fsr17.pdf) | `0701 ...pdf` |
| *UAV See, UGV Do* (2025) | [arXiv:2505.16912](https://arxiv.org/abs/2505.16912) | `0703 ...pdf` |

⚠️ VT&R = teach 阶段（驾驶并沿途采图）+ repeat 阶段（仅用相机重复路线），不需 GPS 与全局度量地图；单目实现可达公里级路线、厘米级精度。

🔶 **本研究的应用叙事（"教师飞一圈、学生重复送货"）逐字就是 VT&R 的定义。**

**必须能回答**："VT&R 用几何方法早就能做路线重复，为什么要换成神经网络？"
候选答案（需实验支撑）：**动态障碍物**（VT&R 重复固定路线，遇临时障碍需额外避障；本研究教师 CERLAB 自带动态障碍处理，学生可继承）——本项目 world 名为 `corridor_dynamic_9`，天然支持这个论证。

### 6.2 目标指定 / 导航基础模型

| 论文 | 链接 | 本地 |
|---|---|---|
| ViNG | [Semantic Scholar](https://www.semanticscholar.org/paper/ViNG:-Learning-Open-World-Navigation-with-Visual-Shah-Eysenbach/0fd47bf484a05001ce787747cf5a879b9202ebfa) | `0801 ...pdf` |
| GNM | [arXiv:2210.03370](https://arxiv.org/abs/2210.03370) | `0802 ...pdf` |
| NoMaD | [arXiv:2310.07896](https://arxiv.org/abs/2310.07896) · [代码](https://github.com/robodhruv/visualnav-transformer) | `0803 ...pdf` |
| SIGN (2025) | [arXiv:2508.12394](https://arxiv.org/abs/2508.12394) | `0804 ...pdf` |
| Warehouse MAV line following | [arXiv:2310.00950](https://arxiv.org/abs/2310.00950) | `0804 ...pdf` |

🔶 **双重影响**：
- 正面：ViNG 的图像拓扑记忆是"教师飞一圈"的低成本产物形态之一
- 负面：**GNM / ViNT / NoMaD 封死了"卖通用预训练模型"路线** —— 跨场景零样本导航基础模型已是大数据大团队赛道

---

## 7. F 组：关联较弱（仅需一句带过）

| 论文 | 链接 | 判断 |
|---|---|---|
| *Driving policy distillation in autonomous racing with adaptive racing vocabulary* (Expert Systems with Applications, 2025) | [ScienceDirect](https://www.sciencedirect.com/science/article/abs/pii/S0957417425028076) | ⚠️ 自适应轨迹词表 + policy transformer 从候选轨迹中选一条。🔶 **多峰问题的第三种解法**（离散化 + 分类，避开回归到均值）。若 R1 诊断指向 M2，这是备选技术路线 |
| *Proximal Policy Distillation* | [arXiv:2407.15134](https://arxiv.org/abs/2407.15134) | ⚠️ 学生驱动蒸馏 + PPO，RL 设定。🔶 **关联低**（本研究是 IL 设定），仅"学生在线收集数据"概念与 DAgger 同源 |

---

## 8. 待读清单

| 优先级 | 论文 | 原因 |
|---|---|---|
| 高 | *Exploring the Limitations of Behavior Cloning* | 直接关联诊断章节 |
| 高 | Stanton et al. *Does KD Really Work?* | fidelity 概念是本项目失败现象的理论框架 |
| 中 | Cho & Hariharan | 压缩轴理论支点 |
| 中 | Tiny-PULP-Dronet | 压缩轴对标数字 |
| 中 | ARD (Goldblum) | 鲁棒性轴理论支点 |
