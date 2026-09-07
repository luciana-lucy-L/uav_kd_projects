# 实验日志（v2）

> 层级：记录层。**每次实验后追加。新条目放在最上面（倒序）。**
> 版本：v2 起始于 2026-09-07。v1 的实验记录见 `_archive/2026-09_v1/04_experiment_log.md`（方法论已废，仅作历史证据引用）
> 记录规范见 `40_experiments/40_experiments.md` §6

---

## 条目模板

```markdown
## YYYY-MM-DD — <臂>_<容量> / <任务名>

**轮次**：R? / W?
**目的**：一句话说明这次要回答什么问题

**配置**：`runs/<path>/config.yaml`
**命令**：
```bash
<可复现的完整命令>
```

**结果**：
| 指标 | 值 |
|---|---|
| ... | ... |

**观察**：异常、意外、值得注意的现象

**结论 / 下一步**：对研究问题的影响，是否触发决策门判定
```

---

## 记录纪律

1. **每次训练和每批部署都要记**，包括失败的
2. 失败的实验**不要删除记录** —— 负结果是本研究的合法产出
3. 触发决策门（G0/G1/G2/G3）时，判定结果必须写进日志并注明依据
4. 进度滑动时在日志中记录原因，同步更新 `30_plan/31_schedule.md` 的进度追踪表
5. 跨臂比较只用 `40_experiments/40_experiments.md` §4.1 规定的可比指标

---

# 日志正文

<!-- 新条目插入到这一行下方 -->

## 2026-09-07 — 教师系统（CERLAB）源码扫描与替代方案评估

**轮次**：R1 前置
**目的**：确认教师是否有可用更新；评估是否应更换 expert；核查教师暴露的全部知识层

### 1. 版本状态

| 项 | 值 |
|---|---|
| 本地 HEAD | `e045ce55`（2025-04-02） |
| 上游 HEAD（`git ls-remote`） | `e045ce55` —— **完全一致，无更新可用** |
| 各模块最后提交 | onboard_detector 2025-04 / map_manager 2024-10 / autonomous_flight 2024-04 / time_optimizer 2024-03 / global_planner 2024-01 / trajectory_planner、tracking_controller 2023-12 |

**结论**：上游基本停更，教师可视为稳定冻结的依赖，无跟进成本。

### 2. ⭐ 重大发现：教师的可分解性远超 v1 假设

源码 `advertise` 扫描显示 CERLAB 暴露 **5 个知识层**，v1 只用了其中 2 个：

| 层 | 代表话题 | v1 是否使用 |
|---|---|---|
| 感知-静态几何 | `2D_occupancy_map`、`esdf`、`voxel_map` | ❌ |
| 感知-动态障碍 | `tracked_bboxes`、`history_trajectories`、`velocity_visualizaton` | ❌ |
| 全局规划 | `rrt_path` | ❌ |
| 局部规划 | `bspline_trajectory`（+ `poly_traj`/`pwl_trajectory` 等优化前中间阶段） | ✅ |
| 控制 | `target_state` | ✅ |

另发现两个 ROS service：`check_pos_collision`、`raycast` —— 可**主动查询**教师，对 DAgger 有价值。

**对研究的影响**：
1. "可分解教师"从一个勉强的说法变成有 5 个真实层次的系统，直接强化 `10_research/10_research.md` 的核心立论
2. **解决了第 5 臂的困境** —— 感知层有真实产物可蒸馏，不需要人造"教师表征"
3. 局部规划内部还可再分（优化前 / 优化后），支持更细的消融

### 3. 替代教师评估：不更换

| 候选 | 结论 |
|---|---|
| **EGO-Planner / ego-planner-swarm** (ZJU FAST Lab) | ❌ **planner-only**，无独立跟踪控制器、无动态障碍检测、无 ESDF 地图模块 → 层数更少，**直接削弱可分解性立论** |
| **Fast-Planner** (HKUST) | ❌ 同上，是 EGO-Planner / FUEL / RACER 的基础框架，仍是规划器而非完整栈 |
| **DYNUS** (MIT ACL, 2025, arXiv:2504.16734) | ❌ ROS 2 Humble / Ubuntu 22.04（本机为 ROS Noetic / Ubuntu 20.04）；**后端优化器依赖 Gurobi 商业许可** → 迁移成本极高 |

**结论：保留 CERLAB。** 它恰恰因为是"完整模块化栈"而非"规划器"，才适合本研究的可分解性立论。换成任何 planner-only 方案都会让 A 轴的层数减少。

### 4. 决策：第 5 臂（感知层）标为待定

原 FeatKD（InfoNCE latent 对齐）方案**否决** —— 经典教师无神经 embedding，人造"教师表征"会破坏 A 轴一致性。
改为待定，候选 DepthAux / OccAux / DynObsAux / 不设，**R1 诊断后决策**（`20_architecture/20_architecture.md` §3.2）。

**下一步**：R1-W1 启动诊断实验 T1/T2/T4

---

## 2026-09-07 — 文档体系 v2 建立

**轮次**：R1 前置
**目的**：研究方向变更后，重建文档结构

**变更**：
- v1 全部文档归档至 `docs/_archive/2026-09_v1/`
- 建立 v2 十位数分层文档体系（见 `00_INDEX.md`）
- 研究定位从"方法创新（JointKD）"改为"系统性实证研究（知识层级 × 容量 × 扰动鲁棒性）"，依据见 `10_research/10_research.md` §1

**环境核实修正**：
| 项 | 旧记录 | 实测（2026-09-07） |
|---|---|---|
| GPU | GTX 1080 Ti，驱动 535.230.02 | **RTX 2080 Ti，11264 MiB，驱动 570.133.07** |
| torch | 2.4.1+cpu | **2.4.1+cu121，CUDA available: True** |
| torchvision | 0.19.1+cpu | **0.19.1+cu121** |

**下一步**：R1-W1 启动诊断实验 T1/T2/T4（见 `40_experiments/41_diagnostics.md`）
