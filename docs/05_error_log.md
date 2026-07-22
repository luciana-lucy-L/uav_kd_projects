# Error Log

---

## 2026-06-29 — RLException: cannot override arg 'world_name' in launch_sim.sh

### Symptom

Running `bash scripts/launch_sim.sh corridor_dynamic_9_nohuman` failed immediately:

```
RLException: Invalid <arg> tag: cannot override arg 'world_name', which has already been set.
Arg xml is <arg name="world_name" value="$(find uav_simulator)/worlds/corridor/corridor_dynamic_9.world"/>
```

### Root Cause

`start.launch` declares `world_name` with `value=` (a compile-time constant), not `default=`. ROS forbids callers from overriding `value=` args via command-line arguments (`world_name:=...`). This is a constraint of `uav_simulator` which must not be modified.

### Fix

Removed the world-selection argument from `launch_sim.sh`. The script now runs `roslaunch uav_simulator start.launch` with no extra arguments. It prints the active world at startup for reference.

To change the world: edit `start.launch` and comment/uncomment the desired `world_name` line.

Currently active world: `worlds/corridor/corridor_dynamic_9.world`

For data collection, switch to: `worlds/corridor/corridor_dynamic_9_nohuman.world`

---

## 2026-06-29 — ModuleNotFoundError: No module named 'yaml' in spawn_model

### Symptom

When launching `roslaunch uav_simulator start.launch` from a terminal with conda base active, Gazebo started but the UAV model was not spawned. The error:

```
[spawn_gazebo_model-N] process has died
ModuleNotFoundError: No module named 'yaml'
  File "/opt/ros/noetic/lib/gazebo_ros/spawn_model", line ...
```

The quadcopter never appeared in Gazebo. `/CERLAB/quadcopter/odom` was never published.

### Root Cause

`spawn_model` uses `#!/usr/bin/env python3`. With conda base active, PATH is:

```
/home/l/miniconda3/bin:... → python3 = Python 3.13.12 (conda)
```

Conda's Python 3.13 has no `yaml` (PyYAML) installed, and cannot import ROS's compiled `.so` packages because they are ABI-linked to Python 3.8 (the Ubuntu 20.04 system Python that ROS Noetic was built against).

### Diagnosis Commands

```bash
which python3          # → /home/l/miniconda3/bin/python3   (wrong)
python3 --version      # → Python 3.13.12                   (wrong for ROS)
python3 -c "import yaml"   # → ModuleNotFoundError
```

After conda deactivate:
```bash
which python3          # → /usr/bin/python3                 (correct)
python3 --version      # → Python 3.8.10                    (correct for ROS Noetic)
python3 -c "import yaml"   # → OK
```

### Fix

Deactivate conda before sourcing ROS and launching. The required sequence is:

```bash
source /home/l/miniconda3/etc/profile.d/conda.sh
conda deactivate
source /home/l/catkin_ws/devel/setup.bash        # for simulator
# and/or
source /home/l/cerlab_ws/devel/setup.bash        # for CERLAB teacher
```

### Permanent Fix — Wrapper Scripts

Created launcher scripts in `uav_kd_project/scripts/` that handle this automatically:

| Script | Purpose |
|---|---|
| `scripts/launch_sim.sh` | Deactivates conda, sources catkin_ws, launches simulator |
| `scripts/launch_teacher.sh` | Deactivates conda, sources both workspaces, launches CERLAB teacher |
| `scripts/launch_rviz.sh` | Deactivates conda, launches RViz for goal input |
| `scripts/check_topics.sh` | Deactivates conda, checks all required topic frequencies |

### Rule Going Forward

**Never launch ROS nodes with conda active.**

Every terminal used for ROS must run the full sequence:
```bash
source ~/miniconda3/etc/profile.d/conda.sh && conda deactivate
source /home/l/catkin_ws/devel/setup.bash
[source /home/l/cerlab_ws/devel/setup.bash]   # if using CERLAB
```

Or simply use the wrapper scripts in `uav_kd_project/scripts/`.

Python training environments (conda `dbc_bc`, future `uav_kd_env`) are separate and should be used only in terminals not running ROS.

### Verification

After applying the fix, all three packages import correctly:
```bash
python3 -c "import yaml, rospy, rospkg; print('OK')"
# → yaml + rospy + rospkg: all OK
rospack find uav_simulator    # → found
rospack find autonomous_flight # → found
rospack find tracking_controller # → found
```
