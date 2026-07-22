#!/bin/bash
# Launch the UAV KD teacher data collector.
#
# Usage:
#   bash launch_collector.sh [episode_id]
#
# If episode_id is omitted, one is generated from the current timestamp.
#
# Prerequisites (must all be running before launching this script):
#   Terminal 1: bash launch_sim.sh
#   Terminal 2: bash launch_rviz.sh
#   Terminal 3: bash launch_teacher.sh
#   Then send a 2D Nav Goal from RViz to start the teacher trajectory.
#
# Stop collection: Ctrl+C in this terminal.
# The collector writes metadata.yaml on shutdown.

set -e

# ── Episode ID ───────────────────────────────────────────────────────────────
EPISODE_ID="${1:-ep_$(date +%Y%m%d_%H%M%S)}"
OUTPUT_DIR="/home/l/uav_kd_project/data/raw"
WORLD="corridor_dynamic_9_nohuman"

echo "============================================================"
echo "  UAV KD Data Collector"
echo "  episode_id : $EPISODE_ID"
echo "  output_dir : $OUTPUT_DIR"
echo "  world      : $WORLD"
echo "============================================================"

# ── Pre-create directories so rosbag record never races ─────────────────────
mkdir -p "$OUTPUT_DIR/$EPISODE_ID/images"
mkdir -p "$OUTPUT_DIR/$EPISODE_ID/rosbag"

# ── Deactivate conda, restore system Python 3.8 for ROS ─────────────────────
if [ -f "/home/l/miniconda3/etc/profile.d/conda.sh" ]; then
    source /home/l/miniconda3/etc/profile.d/conda.sh
    conda deactivate
    echo "[launch_collector] conda deactivated — python3: $(which python3)"
fi

# ── Source all workspaces ────────────────────────────────────────────────────
source /home/l/catkin_ws/devel/setup.bash
source /home/l/cerlab_ws/devel/setup.bash
source /home/l/uav_kd_project/ros_ws/devel/setup.bash
# Sourcing ros_ws last drops cerlab_ws from Python/lib paths; restore manually.
export CMAKE_PREFIX_PATH=/home/l/cerlab_ws/devel:$CMAKE_PREFIX_PATH
export ROS_PACKAGE_PATH=/home/l/cerlab_ws/src/CERLAB-UAV-Autonomy:$ROS_PACKAGE_PATH
export LD_LIBRARY_PATH=/home/l/cerlab_ws/devel/lib:$LD_LIBRARY_PATH
export PKG_CONFIG_PATH=/home/l/cerlab_ws/devel/lib/pkgconfig:$PKG_CONFIG_PATH
export PYTHONPATH=/home/l/cerlab_ws/devel/lib/python3/dist-packages:$PYTHONPATH

echo "[launch_collector] ROS_DISTRO=$ROS_DISTRO"
echo "[launch_collector] Launching collector..."

# ── Launch ───────────────────────────────────────────────────────────────────
roslaunch uav_kd_data_collector record_teacher.launch \
    episode_id:="$EPISODE_ID"    \
    output_dir:="$OUTPUT_DIR"    \
    world:="$WORLD"              \
    K:=10                        \
    sync_tolerance:=1.0          \
    downsample:=false
