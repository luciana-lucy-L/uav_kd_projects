#!/bin/bash
# Launch the UAV Gazebo simulator.
# Must be run in a terminal where conda base is active (it will deactivate conda first).
#
# Usage:
#   bash launch_sim.sh
#
# The active world is whichever line is uncommented in:
#   /home/l/catkin_ws/src/uav_simulator/launch/start.launch
#
# To change the world: open start.launch, comment the current world_name arg line,
# and uncomment the desired one. The arg uses 'value=' (not 'default='), so it
# cannot be overridden from the command line — this is a uav_simulator constraint.
#
# Recommended world for data collection:
#   worlds/corridor/corridor_dynamic_9_nohuman.world

set -e

# Step 1: deactivate conda to restore system Python 3.8 for ROS
if [ -f "/home/l/miniconda3/etc/profile.d/conda.sh" ]; then
    source /home/l/miniconda3/etc/profile.d/conda.sh
    conda deactivate
    echo "[launch_sim] Conda deactivated. python3 is now: $(which python3)"
fi

# Step 2: verify system Python is active
PY_VER=$(python3 --version 2>&1)
if [[ "$PY_VER" != *"3.8"* ]]; then
    echo "[WARNING] Expected Python 3.8 for ROS Noetic but got: $PY_VER"
    echo "          Run 'conda deactivate' manually if this persists."
fi

# Step 3: source catkin workspace
source /home/l/catkin_ws/devel/setup.bash
echo "[launch_sim] Sourced catkin_ws. ROS_DISTRO=$ROS_DISTRO"

# Step 4: show active world
ACTIVE_WORLD=$(grep -v '<!--' /home/l/catkin_ws/src/uav_simulator/launch/start.launch \
    | grep 'world_name' | grep 'value=' | head -1 | sed 's/.*value="\(.*\)".*/\1/')
echo "[launch_sim] Active world: $ACTIVE_WORLD"

# Step 5: launch
echo "[launch_sim] Launching uav_simulator/start.launch ..."
roslaunch uav_simulator start.launch
