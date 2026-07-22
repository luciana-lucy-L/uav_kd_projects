#!/bin/bash
# Launch SelfRedWP student policy node (self-predicted red-dot guidance,
# standalone, no live planner needed, without teacher)
# Usage: bash scripts/launch_student_self_red_wp.sh [model_path]
#
# Run order:
#   Terminal 1: bash scripts/launch_sim.sh
#   Terminal 2: bash scripts/launch_rviz.sh      (optional, for monitoring)
#   Terminal 3: bash scripts/launch_student_self_red_wp.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
ROS_WS="$PROJECT_ROOT/ros_ws"

MODEL_PATH="${1:-$PROJECT_ROOT/runs/self_red_wp/20260704_162940/best_model.pt}"

if [ ! -f "$MODEL_PATH" ]; then
    echo "[ERROR] Model not found: $MODEL_PATH"
    exit 1
fi

echo "[student_self_red_wp] model : $MODEL_PATH"

source /opt/ros/noetic/setup.bash
source "$HOME/catkin_ws/devel/setup.bash"
source "$HOME/cerlab_ws/devel/setup.bash"
source "$ROS_WS/devel/setup.bash"

export PYTHONPATH="$PROJECT_ROOT:$PYTHONPATH"

roslaunch uav_kd_student_policy self_red_wp_policy.launch \
    model_path:="$MODEL_PATH" \
    --screen
