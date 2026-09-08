# 研究设计

> 层级：研究层（最稳定）。**研究方向变更时才修改本文件。**
> 版本：v2（2026-09-07）。v1 见 `_archive/2026-09_v1/research.md`
> 相关：文献对照见 `10_research/11_related_work.md`，架构实现见 `20_architecture/20_architecture.md`

---

## 1. 为什么有 v2：v1 的三个主张已被推翻

v1 的定位陈述是：

> "现有 KD 工作大多是 NN→NN 蒸馏。现有无人机视觉策略依赖 BC/RL，不具备显式规划知识。本研究处于 KD、planner-to-policy、UAV 视觉导航三者的未探索交叉点——以经典自主飞行系统作为教师。"

经 2026-09 文献核查，三句全部不成立（详细证据见 `10_research/11_related_work.md`）：

| v1 主张 | 推翻证据 |
|---|---|
| "KD 大多是 NN→NN" | 机器人领域的特权教师-学生蒸馏是标准范式：Lee et al. (Science Robotics 2020)、Chen et al. (CoRL 2019) |
| "UAV 视觉策略无显式规划知识" | Loquercio et al. (Science Robotics 2021) 明确从采样式经典规划器蒸馏轨迹到视觉策略 |
| "该交叉点未被探索" | 同上，且该交叉点已有 Science Robotics 论文占位 |

同时，v1 的方法 JointKD（轨迹辅助头 + 多步控制头）在架构上等价于 TCP (NeurIPS 2022) 减去分支间连接与融合，**方法层面的新颖性为零**。

**结论：v2 放弃"方法创新"定位，改为"系统性实证研究"定位。**

---

## 2. v2 的研究定位

### 2.1 核心洞察：教师的可分解性是研究工具，不是历史包袱

| 已有工作的教师 | 可提取的知识 | 为什么受限 |
|---|---|---|
| Li & Zhao 2026：RL 黑箱策略 | 动作 + latent embedding | 黑箱内部无"规划层"可摘，只能做表征对齐 |
| Loquercio 2021：采样式规划器 | 轨迹 | 教师不含控制器层，只能蒸馏一层 |
| Chen et al. 2019：特权神经智能体 | 动作 / 路点 | 神经教师无模块分界 |
| **本研究：CERLAB 经典自主栈** | **轨迹 τ（规划器）+ 控制序列 U（控制器）+ 动作 u** | **模块化，各层语义清晰，可分别摘取** |

经典教师由 感知 → 规划 → 控制 三个物理模块组成，其中间产物是人类可解释的。**这让本研究能提出一个神经教师结构上无法定义的问题。**

### 2.2 研究问题

> **当教师是一个可分解的经典自主系统时，它内部的哪一层知识最值得蒸馏给资源受限的单目视觉学生？**
> 这种"最优知识层级"如何随学生容量（压缩）和输入扰动（鲁棒性）而变化？

**English (for thesis):**
> When the teacher is a decomposable classical autonomy stack, which layer of its internal knowledge is most worth distilling into a resource-constrained monocular visual student, and how does this depend on student capacity and input perturbation?

### 2.3 三个研究轴

| 轴 | 内容 | 取值 | 归属 |
|---|---|---|---|
| **A. 知识层级** | 训练时的监督信号来源 | BC / TrajKD / CtrlKD / JointKD / FeatKD | 本研究的方法框架 |
| **B. 学生容量** | 学生网络规模（压缩） | α = 1.0 / 0.5 / 0.35 | 教授指派的题目 |
| **C. 扰动鲁棒性** | 评估时施加的输入扰动 | 干净 / 噪声 / 模糊 / 亮度 / 对比度 /（保留：稀疏对抗） | 实验室主轴 |

**A × B 构成训练网格**（每格一次训练，共 13 个配置；第 5 臂未启用则 12，且第 5 臂只在 α=1.0 设一格，不做容量扫描）；**C 是评估维度**（不增加训练次数）。

### 2.4 核心假设（可证伪）

> **H1**：结构化的教师知识（规划层 τ / 执行层 U），相比纯动作模仿，让小容量学生在**压缩后**和**受扰动时**退化得更慢。

**先验合理性**：
- 容量侧：KD 文献已知容量不匹配会让小学生学不动大教师（Cho & Hariharan 2019）。但那说的是 NN→NN 的 logits。**结构化几何监督是否是一种更容易被小模型吸收的知识形式，尚无人测量。**
- 鲁棒性侧：动作模仿容易学到表面纹理捷径（causal confusion）；轨迹监督强迫 backbone 编码几何结构，理论上更抗视觉扰动。ARD (Goldblum et al. 2020) 已证明鲁棒性可随蒸馏传递，但那是分类任务。

**H1 成立 → 正结果**："在给定压缩预算下应选择结构化蒸馏"
**H1 不成立 → 负结果**："知识层级对可压缩性无影响，瓶颈在观测信息而非监督形式"

**两头都能写成论文。这是本设计最主要的安全性来源。**

### 2.5 与教授指派题目（模型压缩）的接口

传统 KD 压缩范式是：大 NN 教师 → 小 NN 学生，**蒸馏本身即压缩手段**。
本研究的教师是经典系统，**不是神经网络，不可压缩**。因此"压缩"只能定义为**学生容量扫描**，研究问题相应地变为：

> **知识的可压缩性**：把策略网络压到多小时，某一类教师知识开始传不过去？

理论支点：
- Cho & Hariharan, *On the Efficacy of Knowledge Distillation* (arXiv:1910.01348)：容量不匹配使小学生无法模仿大教师
- Stanton et al., *Does Knowledge Distillation Really Work?* (arXiv:2106.05945)：学生即使有足够容量也匹配不上教师（fidelity ≠ generalization）

**经典教师是"无限大教师"的极端情形**——这是本研究与压缩文献最自然的连接点。

---

## 3. 待解释的核心现象（v1 遗留的失败数据）

v1 的三个学生模型在部署时全部无法复现教师的转向能力：

| | 教师（CERLAB） | BC(v002) | TrajKD | SelfRedWP |
|---|---|---|---|---|
| yaw_rate std | **0.340** | 0.104 | 0.027 | 0.042 |
| frac(\|yr\|>0.15) | **17.7%** | 10.4% | 0.9% | 3.4% |

训练集转向帧占比 ~11.8%，验证集 ~11.4%。

**关键观察**：BC 的转向**频率**（10.4%）几乎等于训练数据的转向帧占比（11.8%），但**幅度**只有教师的 30%；而 TrajKD / SelfRedWP 是频率与幅度双双坍缩。

### 三个候选机制

| 机制 | 描述 | 能解释 | 不能解释 |
|---|---|---|---|
| **M1 类别不平衡** | 转向样本仅约 12%，未加权 MSE → 梯度被直飞样本主导 → 幅值低估 | BC 的"频率对、幅度小" | 为什么 TrajKD 更差 |
| **M2 标签多峰 / 目标不可观测** | 无目标输入，视觉相同的岔路帧对应左/右矛盾标签 → MSE 最优解 = 均值 ≈ 0 | 所有模型的整体压缩；闭环卡死 | 为什么 BC 频率还对 |
| **M3 辅助 loss 梯度干扰** | 轨迹 loss 主导共享 backbone，轨迹标签方差低（多为直飞）→ 抑制控制头输出方差 | TrajKD / SelfRedWP 比 BC 更差 | BC 自身的压缩 |

**三者很可能同时存在，权重未知。** R1 的诊断实验（`40_experiments/41_diagnostics.md`）用于测量三者的相对贡献，其结论直接决定数据采集协议。

**这一节的分析本身是论文的一个章节**，不是前期调试的副产品。

---

## 4. 幸存空间：可以声称什么，不可以声称什么

### 4.1 已被占用（不可作为 novelty）

| 主张 | 占位者 |
|---|---|
| 经典系统（非神经）当教师 | Loquercio 2021、Tube-NeRF 2023 |
| 蒸馏轨迹/规划知识到 UAV 视觉策略 | Loquercio 2021 |
| 辅助监督优于纯动作模仿 | Li & Zhao 2026、TCP 2022、CILRS 2019 |
| 轨迹头 + 多步控制头联合训练 | TCP 2022 |
| 特权教师 → 纯视觉学生范式 | Chen et al. 2019、Lee et al. 2020 |
| 轻量 MobileNet 学生 | Loquercio 2021（MobileNet-V3） |
| UAV 导航的极限模型压缩 | Tiny-PULP-Dronet 系列（压至 2.9 kB） |
| 卖通用预训练导航模型 | GNM / ViNT / NoMaD |
| 对比学习提升**场景迁移**鲁棒性 | Xing et al. 2024、arXiv:2606.05506 |

### 4.2 灰色地带（可说，措辞需精确）

| 说法 | 允许的措辞 | 禁止的措辞 |
|---|---|---|
| 单目 RGB | "与依赖深度感知的既有工作不同，我们研究仅用单目 RGB 的可行边界" | "首个单目方案"（Tube-NeRF 已是四旋翼+RGB+真机） |
| 教师形态 | "教师是部署级的完整自主栈，而非为蒸馏专门构造的离线 oracle" | "首个用经典系统当教师" |
| 鲁棒性 | "**扰动鲁棒性与对抗鲁棒性**" | "场景迁移鲁棒性"（已被占用） |

### 4.3 真正的空白（本研究的立足点）

1. **可分解教师的知识层级对照实验** —— 只有模块化的经典教师能定义这个问题
2. **知识层级 × 学生容量的交互效应** —— 压缩文献研究 NN→NN 的容量差距，未研究"知识类型"这一维度
3. **蒸馏知识类型对扰动鲁棒性的影响** —— 现有 KD-导航工作均未评估输入扰动下的策略退化
4. **单目学生的可蒸馏性边界** —— 教师的哪些决策原理上无法传递给单目观测

---

## 5. 预期贡献（论文用）

1. **方法论贡献**：提出以**可分解的经典自主栈作为研究工具**，将"教师知识"拆解为规划层、执行层、表征层三种可分别蒸馏的形式，并在统一条件下系统对照。这一分解在神经教师上无法定义。

2. **实证贡献**：给出知识层级 × 学生容量的交互效应测量，回答"在给定压缩预算下应选择何种蒸馏监督"。

3. **鲁棒性贡献**：首次评估不同蒸馏知识类型对轻量视觉导航策略在输入扰动下的退化速度。

4. **诊断贡献**：量化单目学生无法复现经典教师转向能力的机制构成（类别不平衡 / 标签多峰 / 梯度干扰），给出可蒸馏性的信息边界。

**English (contribution statement draft):**
> We use a decomposable classical autonomy stack as a research instrument to separate teacher knowledge into planning-level (trajectory), execution-level (control sequence), and representation-level (latent) forms, and systematically compare their distillability into a resource-constrained monocular visual policy under matched conditions. We measure how the most effective knowledge layer interacts with student capacity and input perturbation, and quantify the mechanisms by which a monocular student fails to reproduce the classical teacher's turning behaviour.

---

## 6. 应用背景（仅用于动机段落，不作为贡献）

一次性用配备全套传感器的经典无人机对场地做初次示范 → 后续重复性作业由仅有前视摄像头的轻量无人机执行，省去每台无人机的传感器与建图算力成本。

⚠️ **写作纪律**：
- 该场景与 Visual Teach & Repeat 高度重合，必须在相关工作中引用 VT&R 并说明差异（动态障碍物处理）
- **不声称** zero-shot 跨场地泛化（GNM/ViNT 已占据该赛道，且本研究数据量不支持）
- 商业形态按"每场地教师示范一次 + 学生本地训练"表述，不说"厂家无需训练"

---

## 7. 明确排除的方向

| 方向 | 排除原因 |
|---|---|
| JointKD 作为方法创新 | 架构等价于 TCP 减去融合，novelty 归零。降级为消融臂之一 |
| SelfRedWP | v1 的探索方向，部署失败，已废弃 |
| TrajKD + Pure Pursuit | v1 已降级为实现注记，不进入实验 |
| Teach-and-Repeat 神经化作为主线 | 撞 VT&R 15 年积累；且需实现几何基线，超出时间预算。仅保留在动机段落 |
| 卖通用预训练模型 | 需 zero-shot 跨场景泛化，本研究数据量与体量不支持 |
| 极限模型压缩（追求压缩率） | Tiny-PULP-Dronet 已压至 2.9 kB，不在压缩率上竞争 |
