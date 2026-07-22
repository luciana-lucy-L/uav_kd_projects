#!/bin/bash
# Display UAV front camera in a real-time window.
# Run this in any terminal while the simulator is running.
#
# Usage:
#   bash view_camera.sh
#
# Opens an OpenCV window showing /camera/color/image_raw at ~30 Hz.
# Press Q or Ctrl+C to close.

set -e

if [ -f "/home/l/miniconda3/etc/profile.d/conda.sh" ]; then
    source /home/l/miniconda3/etc/profile.d/conda.sh
    conda deactivate
fi

source /home/l/catkin_ws/devel/setup.bash

echo "[view_camera] Displaying /camera/color/image_raw — press Q to quit"
rosrun image_view image_view image:=/camera/color/image_raw
