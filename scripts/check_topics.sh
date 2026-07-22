#!/bin/bash
# Verify that required CERLAB teacher topics are active.
# Run this in a terminal while launch_sim.sh + launch_teacher.sh are running.
#
# Usage:
#   bash check_topics.sh

set -e

if [ -f "/home/l/miniconda3/etc/profile.d/conda.sh" ]; then
    source /home/l/miniconda3/etc/profile.d/conda.sh
    conda deactivate
fi

source /home/l/catkin_ws/devel/setup.bash
source /home/l/cerlab_ws/devel/setup.bash

echo "============================================================"
echo "  CERLAB Topic Verification"
echo "============================================================"

echo ""
echo "--- Active topics (filtered) ---"
rostopic list 2>/dev/null | grep -E "CERLAB|camera|autonomous_flight|dynamicNavigation|tracking_controller" | sort

echo ""
echo "--- /CERLAB/quadcopter/odom (expect ~30 Hz) ---"
timeout 4 rostopic hz /CERLAB/quadcopter/odom 2>/dev/null || echo "(no data in 4s)"

echo ""
echo "--- /camera/color/image_raw (expect ~30 Hz) ---"
timeout 4 rostopic hz /camera/color/image_raw 2>/dev/null || echo "(no data in 4s)"

echo ""
echo "--- /autonomous_flight/target_state (expect ~100 Hz, only after goal given) ---"
timeout 4 rostopic hz /autonomous_flight/target_state 2>/dev/null || echo "(no data in 4s - give a 2D Nav Goal first)"

echo ""
echo "--- /dynamicNavigation/bspline_trajectory (expect ~30 Hz after goal) ---"
timeout 4 rostopic hz /dynamicNavigation/bspline_trajectory 2>/dev/null || echo "(no data in 4s - give a 2D Nav Goal first)"

echo ""
echo "--- /CERLAB/quadcopter/cmd_vel (expect 0 Hz during autonomous flight) ---"
timeout 3 rostopic hz /CERLAB/quadcopter/cmd_vel 2>/dev/null || echo "(not publishing - correct for autonomous mode)"

echo ""
echo "============================================================"
echo "  Done. Record results in docs/04_experiment_log.md"
echo "============================================================"
