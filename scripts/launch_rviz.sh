#!/bin/bash
# Launch RViz for dynamic navigation (goal input + visualization).
# Run in a third terminal after launch_sim.sh and launch_teacher.sh are running.
#
# Usage:
#   bash launch_rviz.sh

set -e

if [ -f "/home/l/miniconda3/etc/profile.d/conda.sh" ]; then
    source /home/l/miniconda3/etc/profile.d/conda.sh
    conda deactivate
    echo "[launch_rviz] Conda deactivated."
fi

source /home/l/catkin_ws/devel/setup.bash
source /home/l/cerlab_ws/devel/setup.bash

echo "[launch_rviz] Launching remote_control/dynamic_navigation_rviz.launch ..."
echo "[launch_rviz] Use the '2D Nav Goal' tool in RViz to send a navigation target."
roslaunch remote_control dynamic_navigation_rviz.launch
