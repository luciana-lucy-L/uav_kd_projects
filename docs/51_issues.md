# 错误与踩坑记录

> 层级：记录层。**每次踩坑后追加。**
> 从 `_archive/2026-09_v1/05_error_log.md` 迁入仍然有效的条目。

---

## 条目模板

```markdown
## YYYY-MM-DD — <一句话症状>

**症状**：报错信息或异常现象
**根因**：
**修复**：
**预防**：以后怎么避免（如果有对应的规则，写进 CLAUDE.md）
```

---

# 已知问题（仍然有效）

## ROS 与 conda 的 Python 冲突 ⭐ 最常踩

**症状**：`roslaunch uav_simulator start.launch` 时 Gazebo 启动但 UAV 模型未生成：

```
[spawn_gazebo_model-N] process has died
ModuleNotFoundError: No module named 'yaml'
```

`/CERLAB/quadcopter/odom` 永远不发布。

**根因**：`spawn_model` 用 `#!/usr/bin/env python3`。conda 激活时 PATH 指向 conda 的 Python 3.13，它没有 PyYAML，也无法 import ROS 的编译扩展（ABI 绑定 Python 3.8）。

**诊断命令**：
```bash
which python3        # 应为 /usr/bin/python3
python3 --version    # 应为 Python 3.8.10
python3 -c "import yaml"
```

**修复**：启动 ROS 前必须 `conda deactivate`。已封装进 `scripts/launch_*.sh`。

**预防规则**：**绝不在 conda 激活状态下启动任何 ROS 节点。**（已写入 `CLAUDE.md`）

---

## `world_name` 无法通过命令行覆盖

**症状**：
```
RLException: Invalid <arg> tag: cannot override arg 'world_name', which has already been set.
```

**根因**：`start.launch` 用 `value=`（编译期常量）而非 `default=` 声明 `world_name`，ROS 禁止命令行覆盖。该文件属于 `uav_simulator`，**不得修改**。

**修复**：`launch_sim.sh` 不再传 world 参数。要换 world 需手动编辑 `start.launch` 的注释行。

---

## `cmd_vel` 在自主飞行期间不发布

**症状**：订阅 `/CERLAB/quadcopter/cmd_vel` 采集教师动作，得到空数据。

**根因**：自主飞行时跟踪控制器发的是加速度指令到 `/CERLAB/quadcopter/cmd_acc`，`cmd_vel` 只有键盘控制才发布。

**修复**：教师控制标签取自 `/autonomous_flight/target_state`（`.velocity` + `.yaw`）。

**预防**：见 `52_environment.md` §3 的话题角色表。

---

## `yaw_rate` 不是独立话题

**根因**：必须从连续的 `target_state.yaw` 差分得到：`yaw_rate = wrap(yaw[t] − yaw[t−1]) / dt`。

**修复**：采集节点内计算，建议加滑窗滤波降噪。

---

## `bspline_trajectory` 在给目标前不发布

**根因**：`visCB` 只在轨迹消息非空时发布。起飞与悬停阶段（首个目标点之前）该话题静默。

**影响**：这些帧的 traj 标签无效，需过滤。

---

## 原地旋转时 traj 退化 ⭐ 已修复但需继承

**症状**：UAV 原地旋转（不平移）时段的 K 个路点全部冻结在 UAV 自身位置附近，幅值 ≈ 0，但旧的 `valid_traj`（仅查 `dx0 ≥ 0`）判其有效。

**根因**：不平移时 `bspline_trajectory` 不重新发布。

**修复**：`dataset_builder.py` 增加 `traj_magnitude_threshold`（默认 0.02，检查 `Σ(|dx_i|+|dy_i|)` over K），`valid_traj = (dx0 ≥ 0) AND 未退化`，`filter_reason` 增加 `traj_degenerate`。

**注意**：v2 的 builder 必须继承此修复。

---

## workspace source 顺序

**规则**：必须先 source `catkin_ws` 再 source `cerlab_ws`，否则 `tracking_controller::Target` 消息类型解析冲突。

---

## 环境记录曾经有误（2026-09-07 修正）

| 项 | 旧记录 | 实测 |
|---|---|---|
| GPU | GTX 1080 Ti，驱动 535.230.02 | **RTX 2080 Ti，驱动 570.133.07** |
| torch | 2.4.1+cpu | **2.4.1+cu121，CUDA 可用** |

**预防**：环境信息以实测为准，写入 `52_environment.md` 时附核实日期。

<!-- 新条目追加到下方 -->
