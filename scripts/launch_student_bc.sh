#!/bin/bash
# Launch BC student policy node (standalone, without teacher)
# Usage: bash scripts/launch_student_bc.sh [model_path]
#
# Run order:
#   Terminal 1: bash scripts/launch_sim.sh
#   Terminal 2: bash scripts/launch_rviz.sh      (optional, for monitoring)
#   Terminal 3: bash scripts/launch_student_bc.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
ROS_WS="$PROJECT_ROOT/ros_ws"

# Default to latest best_model.pt if not specified
MODEL_PATH="${1:-$PROJECT_ROOT/runs/bc/20260629_151702/best_model.pt}"

if [ ! -f "$MODEL_PATH" ]; then
    echo "[ERROR] Model not found: $MODEL_PATH"
    exit 1
fi

echo "[student_bc] model : $MODEL_PATH"

# Source ROS + workspaces (NO conda — ROS uses system Python 3.8)
source /opt/ros/noetic/setup.bash
source "$HOME/catkin_ws/devel/setup.bash"
source "$HOME/cerlab_ws/devel/setup.bash"
source "$ROS_WS/devel/setup.bash"

# Prepend project root so run_bc_policy.py can import kd_uav
# PyTorch must be on system Python 3.8 — use dbc241 torch installed there
export PYTHONPATH="$PROJECT_ROOT:$PYTHONPATH"

# Use the dbc241 conda env's torch (CUDA-enabled) via explicit Python path
CONDA_PYTHON="$HOME/miniconda3/envs/dbc241/bin/python3"

roslaunch uav_kd_student_policy bc_policy.launch \
    model_path:="$MODEL_PATH" \
    --screen
