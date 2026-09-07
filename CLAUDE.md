# CLAUDE.md — 工作规则与文档索引

> 本文件只放**工作规则**和**文档入口**。研究内容、架构细节、实验协议一律在 `docs/` 下。
> v1 版本（678 行，含旧方案实现细节）已归档至 `docs/_archive/2026-09_v1/CLAUDE_v1.md`

---

## 当前状态

| 项 | 值 |
|---|---|
| 阶段 | **R1「诊断 + 地基」W1**（2026-09-07 起） |
| 下一个动作 | 诊断实验 T1 / T2 / T4（见 `docs/41_diagnostics.md`） |
| 下一个决策门 | **G0**（W2 末，诊断结论是否明确） |
| 研究定位 | 系统性实证研究：知识层级 × 学生容量 × 扰动鲁棒性 |

---

## 文档索引

**先读 `docs/00_INDEX.md`**（文档地图与维护规则）。

| 想知道什么 | 看哪里 |
|---|---|
| 研究问题、定位、贡献、核心假设 | `docs/10_research.md` |
| 文献威胁分析、能声称什么 | `docs/11_related_work.md` |
| 教师系统、学生架构、五个实验臂 | `docs/20_architecture.md` |
| 数据 schema、采集协议、坐标系 | `docs/21_data_spec.md` |
| 四轮螺旋计划、决策门、风险 | `docs/30_plan.md` |
| 逐周排期、周会汇报点 | `docs/31_schedule.md` |
| 实验矩阵、命名、评估协议 | `docs/40_experiments.md` |
| R1 诊断实验 T1–T5 | `docs/41_diagnostics.md` |
| 实验流水账 | `docs/50_log.md` |
| 踩坑记录 | `docs/51_issues.md` |
| 环境、ROS 话题、启动命令 | `docs/52_environment.md` |

---

## 严格规则

### 不可触碰

1. **绝不修改 CERLAB 源码**（`/home/l/cerlab_ws/`）
2. **绝不修改 uav_simulator**（`/home/l/catkin_ws/`）
3. **R1 结束前，绝不删除或移动** `runs/*`、`data/raw/*`、`data/processed/uav_kd_v002`
   —— 诊断实验直接依赖它们（`docs/41_diagnostics.md` §1）

### 研究约束

4. **学生部署时只用前视 RGB 图像 + 相对目标向量**。不得使用教师轨迹、LiDAR、深度、地图、全局位姿
5. 教师信号仅作训练期监督标签
6. **train/val/test 按 episode 划分**，绝不随机分帧
7. `data/raw/` 与 `data/processed/` 严格分离

### 环境

8. **绝不在 conda 激活状态下启动 ROS 节点**（先 `conda deactivate`）
9. ROS 用系统 Python 3.8；训练与数据处理用 conda `dbc241`
10. source 顺序：先 `catkin_ws` 再 `cerlab_ws`

### 实验纪律

11. **跨臂比较只用可比指标**（Ctrl MSE、分层指标、SR/CR、相对退化率），**绝不比 val loss** —— 不同臂的 loss 定义不同
12. 每个配置 **≥ 2 个随机种子**，报告均值±标准差
13. 公平对照的七条红线见 `docs/40_experiments.md` §7，违反任意一条整组作废
14. 失败实验**不删记录**，负结果是本研究的合法产出

### 工作方式

15. 改文件前先说明**改哪些、为什么**
16. 改完列出**所有变更文件** + **可复现的运行命令** + **预期输出**
17. 每次实验后追加 `docs/50_log.md`；踩坑后追加 `docs/51_issues.md`
18. **改一件事只改一个文件**：研究方向变 → `10`；进度滑 → `31`；加实验臂 → `40`

---

## 已废弃的方向（不要再提议）

| 方向 | 废弃原因 |
|---|---|
| JointKD 作为"提出的方法" | 架构等价于 TCP 减去融合，novelty 归零；降级为消融臂 |
| SelfRedWP | 部署失败，方向已废 |
| TrajKD + Pure Pursuit | v1 已降级为实现注记 |
| Teach-and-Repeat 神经化作为主线 | 撞 VT&R，超时间预算；仅保留在动机段落 |
| 卖通用预训练模型 | 需 zero-shot 跨场景泛化，数据量不支持 |
| 追求极限压缩率 | Tiny-PULP-Dronet 已压至 2.9 kB，不在此竞争 |

详见 `docs/10_research.md` §7。
