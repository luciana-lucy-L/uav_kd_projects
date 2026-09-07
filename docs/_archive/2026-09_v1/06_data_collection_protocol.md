# 正式数据采集协议 — uav_kd_v001

**目标：** 采集 5–10 个有效教师演示 episode，构建正式训练数据集 `uav_kd_v001`。

---

## 一、Episode 命名规范

格式：`ep_v001_<编号>_<类型>`

| Episode ID | 场景类型 | World | 备注 |
|---|---|---|---|
| `ep_v001_001_straight_short` | 直线短距离 | corridor_dynamic_9_nohuman | ~5–10s 飞行 |
| `ep_v001_002_straight_long` | 直线长距离 | corridor_dynamic_9_nohuman | ~15–20s 飞行 |
| `ep_v001_003_left_turn` | 左转 | corridor_dynamic_9_nohuman | 2D Nav Goal 放在走廊左侧 |
| `ep_v001_004_right_turn` | 右转 | corridor_dynamic_9_nohuman | 2D Nav Goal 放在走廊右侧 |
| `ep_v001_005_obstacle_avoidance` | 障碍物回避 | corridor_dynamic_9_nohuman | 有动态障碍物干扰 |
| `ep_v001_006_straight_long_2` | 直线长距离（第2次） | corridor_dynamic_9_nohuman | 补充数据 |
| `ep_v001_007_left_turn_2` | 左转（第2次） | corridor_dynamic_9_nohuman | 补充数据 |
| `ep_v001_008_right_turn_2` | 右转（第2次） | corridor_dynamic_9_nohuman | 补充数据 |

**命名规则：**
- 前缀 `ep_v001_` 固定，表示 v001 数据集
- 编号三位数字，从 001 开始
- 类型用下划线，全小写英文
- 每次重跑同类型场景，在末尾加 `_2`、`_3` 等

---

## 二、采集前系统准备清单

每次采集前，确保以下 4 个终端全部正常运行：

```
终端 1：bash /home/l/uav_kd_project/scripts/launch_sim.sh
终端 2：bash /home/l/uav_kd_project/scripts/launch_rviz.sh
终端 3：bash /home/l/uav_kd_project/scripts/launch_teacher.sh
终端 4：（待发 2D Nav Goal 后）bash launch_collector.sh <episode_id>
```

**系统就绪检查（终端 5，可选）：**
```bash
bash /home/l/uav_kd_project/scripts/check_topics.sh
```

| 检查项 | 期望值 | 确认 |
|---|---|---|
| Gazebo 仿真窗口已打开 | ✓ | |
| RViz 窗口已打开，UAV 模型可见 | ✓ | |
| UAV 自动起飞到约 1.0m | ✓ | |
| `/camera/color/image_raw` 话题存在且约 30 Hz | ✓ | |
| `/autonomous_flight/target_state` 话题存在 | ✓ | |
| `/dynamicNavigation/bspline_trajectory` 话题存在 | ✓ | |
| `/CERLAB/quadcopter/odom` 话题存在 | ✓ | |
| Conda 已 deactivate（系统 Python 3.8） | ✓ | |

---

## 三、单 Episode 采集流程

### 步骤

```
1. 确认系统就绪（见上方清单）
2. 在 RViz 中点击 "2D Nav Goal"，在地图上设置目标点
3. 确认 bspline_trajectory 话题开始发布（UAV 开始规划轨迹）
4. 打开终端 4，启动采集器：
   bash /home/l/uav_kd_project/scripts/launch_collector.sh <episode_id>
5. 等待 UAV 完成飞行（到达目标或发生碰撞）
6. 按 Ctrl+C 停止采集（metadata.yaml 在此时写入）
7. 执行下方 "采集后立即检查" 命令
8. 填写本页 Episode 记录表
```

### 采集命令示例

```bash
# 以 ep_v001_001_straight_short 为例
bash /home/l/uav_kd_project/scripts/launch_collector.sh ep_v001_001_straight_short
```

---

## 四、采集后立即检查命令

每个 episode 采集完成后，运行以下命令：

```bash
# === 替换为实际 episode ID ===
EP="ep_v001_001_straight_short"
RAW_DIR="/home/l/uav_kd_project/data/raw/$EP"

# 1. 目录和文件完整性
echo "=== 目录结构 ===" && ls "$RAW_DIR"
echo "=== 图像帧数 ===" && ls "$RAW_DIR/images/" | wc -l
echo "=== rosbag 大小 ===" && du -sh "$RAW_DIR/rosbag/"
echo "=== metadata ===" && cat "$RAW_DIR/metadata.yaml"

# 2. CSV 行数（含表头应为 frames+1）
echo "=== CSV 行数 ==="
wc -l "$RAW_DIR"/*.csv

# 3. ctrl_body 基本统计（检查 vx_b 非零、yaw_rate 是否有变化）
echo "=== ctrl_body 前 5 行 ===" && head -6 "$RAW_DIR/ctrl_body.csv"
conda run -n dbc241 python3 -c "
import pandas as pd, sys
df = pd.read_csv('$RAW_DIR/ctrl_body.csv')
print('ctrl_body shape:', df.shape)
print(df[['vx_b','vy_b','vz_b','yaw_rate']].describe().round(4))
"

# 4. traj_body dx0 检查（确认无负值）
conda run -n dbc241 python3 -c "
import pandas as pd
df = pd.read_csv('$RAW_DIR/traj_body.csv')
dx0 = df['dx0']
print(f'traj_body dx0: min={dx0.min():.3f}  max={dx0.max():.3f}  negative={( dx0 < 0).sum()}/{len(dx0)}')
"

# 5. 图像抽样检查（查看第 1 帧）
echo "=== 第 1 帧图像 ===" && ls "$RAW_DIR/images/" | head -3
```

---

## 五、Episode 有效性标准

| 条件 | 要求 | 不满足时处理 |
|---|---|---|
| 帧数 | ≥ 100 帧 | 丢弃，重新采集 |
| traj_body dx0 负值帧数 | = 0 | 若 > 0 则检查采集器参数 |
| ctrl_body vx_b 均值 | > 0.1 m/s（飞行期间） | 丢弃（UAV 未实际移动） |
| 图像是否黑帧 | 目视检查第 1、中间、最后 1 帧 | 有黑帧则丢弃 |
| rosbag 文件存在 | ✓ | 若无则采集失败 |
| metadata.yaml 存在 | ✓ | 若无则 Ctrl+C 异常，手动补写 |

---

## 六、Episode 记录表

### ep_v001_001_straight_short

| 项目 | 记录 |
|---|---|
| 采集日期 | |
| World | corridor_dynamic_9_nohuman |
| Goal 类型 | 走廊直线，短距离 |
| 2D Nav Goal 是否被接受 | |
| UAV 是否到达目标 | |
| 是否发生碰撞 | |
| 总帧数 | |
| traj_body dx0 负值帧数 | |
| yaw_rate 是否有非零值 | |
| 图像是否正常（非黑） | |
| target_state 是否活跃 | |
| bspline_trajectory 是否非空 | |
| 是否有效（用于 v001） | |
| 备注 | |

---

### ep_v001_002_straight_long

| 项目 | 记录 |
|---|---|
| 采集日期 | |
| World | corridor_dynamic_9_nohuman |
| Goal 类型 | 走廊直线，长距离 |
| 2D Nav Goal 是否被接受 | |
| UAV 是否到达目标 | |
| 是否发生碰撞 | |
| 总帧数 | |
| traj_body dx0 负值帧数 | |
| yaw_rate 是否有非零值 | |
| 图像是否正常（非黑） | |
| target_state 是否活跃 | |
| bspline_trajectory 是否非空 | |
| 是否有效（用于 v001） | |
| 备注 | |

---

### ep_v001_003_left_turn

| 项目 | 记录 |
|---|---|
| 采集日期 | |
| World | corridor_dynamic_9_nohuman |
| Goal 类型 | 左转 |
| 2D Nav Goal 是否被接受 | |
| UAV 是否到达目标 | |
| 是否发生碰撞 | |
| 总帧数 | |
| traj_body dx0 负值帧数 | |
| yaw_rate 是否有非零值 | |
| 图像是否正常（非黑） | |
| target_state 是否活跃 | |
| bspline_trajectory 是否非空 | |
| 是否有效（用于 v001） | |
| 备注 | |

---

### ep_v001_004_right_turn

| 项目 | 记录 |
|---|---|
| 采集日期 | |
| World | corridor_dynamic_9_nohuman |
| Goal 类型 | 右转 |
| 2D Nav Goal 是否被接受 | |
| UAV 是否到达目标 | |
| 是否发生碰撞 | |
| 总帧数 | |
| traj_body dx0 负值帧数 | |
| yaw_rate 是否有非零值 | |
| 图像是否正常（非黑） | |
| target_state 是否活跃 | |
| bspline_trajectory 是否非空 | |
| 是否有效（用于 v001） | |
| 备注 | |

---

### ep_v001_005_obstacle_avoidance

| 项目 | 记录 |
|---|---|
| 采集日期 | |
| World | corridor_dynamic_9_nohuman |
| Goal 类型 | 障碍物回避 |
| 2D Nav Goal 是否被接受 | |
| UAV 是否到达目标 | |
| 是否发生碰撞 | |
| 总帧数 | |
| traj_body dx0 负值帧数 | |
| yaw_rate 是否有非零值 | |
| 图像是否正常（非黑） | |
| target_state 是否活跃 | |
| bspline_trajectory 是否非空 | |
| 是否有效（用于 v001） | |
| 备注 | |

---

### ep_v001_006（备用 / 补充）

| 项目 | 记录 |
|---|---|
| 采集日期 | |
| World | |
| Goal 类型 | |
| 2D Nav Goal 是否被接受 | |
| UAV 是否到达目标 | |
| 是否发生碰撞 | |
| 总帧数 | |
| traj_body dx0 负值帧数 | |
| yaw_rate 是否有非零值 | |
| 图像是否正常（非黑） | |
| target_state 是否活跃 | |
| bspline_trajectory 是否非空 | |
| 是否有效（用于 v001） | |
| 备注 | |

---

## 七、v001 数据集构建触发条件

满足以下条件后，执行 `dataset_builder.py` 构建正式数据集：

- [ ] 有效 episode 数量 ≥ 5
- [ ] 至少 1 个含左转（yaw_rate 有非零值）的 episode
- [ ] 至少 1 个含右转的 episode
- [ ] 所有有效 episode 的 traj_body dx0 负值帧数 = 0
- [ ] 总帧数 ≥ 3000（约 100 秒飞行时间）

**构建命令（满足条件后执行）：**

```bash
conda activate dbc241
cd /home/l/uav_kd_project

# 将 <ep_list> 替换为空格分隔的有效 episode 列表
python kd_uav/datasets/dataset_builder.py \
    --raw_dir data/raw \
    --output_dir data/processed/uav_kd_v001 \
    --episodes ep_v001_001_straight_short ep_v001_002_straight_long ep_v001_003_left_turn ep_v001_004_right_turn ep_v001_005_obstacle_avoidance \
    --K 10 \
    --dataset_type train \
    --val_ratio 0.1 \
    --test_ratio 0.1
```

---

## 八、汇总表（采集完成后填写）

| Episode ID | 场景 | 帧数 | yaw_rate 非零 | dx0 负值 | 有效 | 备注 |
|---|---|---|---|---|---|---|
| ep_v001_001_straight_short | 直线短 | | | | | |
| ep_v001_002_straight_long | 直线长 | | | | | |
| ep_v001_003_left_turn | 左转 | | | | | |
| ep_v001_004_right_turn | 右转 | | | | | |
| ep_v001_005_obstacle_avoidance | 障碍回避 | | | | | |
| ep_v001_006 | 备用 | | | | | |
| ep_v001_007 | 备用 | | | | | |
| ep_v001_008 | 备用 | | | | | |
| **合计** | | | | | | |
