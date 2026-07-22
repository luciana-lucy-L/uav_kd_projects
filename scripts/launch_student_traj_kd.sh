#!/bin/bash
# Launch TrajKD student policy node (ctrl-head-only deployment, standalone, without teacher)
# Usage: bash scripts/launch_student_traj_kd.sh [model_path]
#
# Run order:
#   Terminal 1: bash scripts/launch_sim.sh
#   Terminal 2: bash scripts/launch_rviz.sh      (optional, for monitoring)
#   Terminal 3: bash scripts/launch_student_traj_kd.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
ROS_WS="$PROJECT_ROOT/ros_ws"

# Default to the confirmed TrajKD run's best_model.pt if not specified
MODEL_PATH="${1:-$PROJECT_ROOT/runs/traj_kd/20260702_131051/best_model.pt}"

if [ ! -f "$MODEL_PATH" ]; then
    echo "[ERROR] Model not found: $MODEL_PATH"
    exit 1
fi

echo "[student_traj_kd] model : $MODEL_PATH"

# Source ROS + workspaces (NO conda — ROS uses system Python 3.8)
source /opt/ros/noetic/setup.bash
source "$HOME/catkin_ws/devel/setup.bash"
source "$HOME/cerlab_ws/devel/setup.bash"
source "$ROS_WS/devel/setup.bash"

# Prepend project root so run_traj_kd_policy.py can import kd_uav
# PyTorch must be on system Python 3.8 — use dbc241 torch installed there
export PYTHONPATH="$PROJECT_ROOT:$PYTHONPATH"

roslaunch uav_kd_student_policy traj_kd_policy.launch \
    model_path:="$MODEL_PATH" \
    --screen
