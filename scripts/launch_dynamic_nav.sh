#!/bin/bash
# Launch dynamic_navigation_node (planner only, no tracking_controller).
# Provides /dynamicNavigation/bspline_trajectory for BC-RedWP student policy.
#
# Usage:
#   bash /home/l/uav_kd_project/scripts/launch_dynamic_nav.sh
#
# Run AFTER launch_sim.sh, BEFORE bc_red_wp_policy.launch.
# Send a 2D Nav Goal in RViz to activate the planner.

# Deactivate conda to restore system Python 3.8 for ROS
if [ -f "/home/l/miniconda3/etc/profile.d/conda.sh" ]; then
    source /home/l/miniconda3/etc/profile.d/conda.sh
    conda deactivate 2>/dev/null || true
fi

# Source catkin_ws and ros_ws (chain)
source /home/l/catkin_ws/devel/setup.bash
source /home/l/uav_kd_project/ros_ws/devel/setup.bash

# Manually extend the environment with cerlab_ws without resetting the chain.
# cerlab_ws was built as a sibling of ros_ws (both extend catkin_ws),
# so sourcing cerlab_ws/setup.bash would drop ros_ws from CMAKE_PREFIX_PATH.
export CMAKE_PREFIX_PATH=/home/l/cerlab_ws/devel:$CMAKE_PREFIX_PATH
export ROS_PACKAGE_PATH=/home/l/cerlab_ws/src/CERLAB-UAV-Autonomy:$ROS_PACKAGE_PATH
export LD_LIBRARY_PATH=/home/l/cerlab_ws/devel/lib:$LD_LIBRARY_PATH
export PKG_CONFIG_PATH=/home/l/cerlab_ws/devel/lib/pkgconfig:$PKG_CONFIG_PATH

echo "[launch_dynamic_nav] cerlab added to environment"
echo "[launch_dynamic_nav] Launching dynamic_nav_only.launch ..."

roslaunch uav_kd_student_policy dynamic_nav_only.launch
