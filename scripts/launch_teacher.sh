#!/bin/bash
# Launch the CERLAB teacher (dynamic navigation).
# Run this in a second terminal AFTER launch_sim.sh is running.
#
# Usage:
#   bash launch_teacher.sh

set -e

# Step 1: deactivate conda
if [ -f "/home/l/miniconda3/etc/profile.d/conda.sh" ]; then
    source /home/l/miniconda3/etc/profile.d/conda.sh
    conda deactivate
    echo "[launch_teacher] Conda deactivated. python3 is now: $(which python3)"
fi

PY_VER=$(python3 --version 2>&1)
if [[ "$PY_VER" != *"3.8"* ]]; then
    echo "[WARNING] Expected Python 3.8 for ROS Noetic but got: $PY_VER"
fi

# Step 2: source catkin_ws first (for message types like uav_simulator/uav_bc_tools),
#         then overlay cerlab_ws
source /home/l/catkin_ws/devel/setup.bash
source /home/l/cerlab_ws/devel/setup.bash
echo "[launch_teacher] Sourced catkin_ws + cerlab_ws. ROS_DISTRO=$ROS_DISTRO"

# Step 3: launch
echo "[launch_teacher] Launching autonomous_flight/dynamic_navigation.launch ..."
roslaunch autonomous_flight dynamic_navigation.launch
