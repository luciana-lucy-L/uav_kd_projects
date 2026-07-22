#!/bin/bash
# record_flight.sh — Record Gazebo + RViz + Camera and stitch into one video.
#
# Usage:
#   bash record_flight.sh MODEL_NAME [CAMERA_TOPIC]
#
# Examples:
#   bash record_flight.sh bc_red_wp
#   bash record_flight.sh bc_baseline /camera/color/image_raw
#
# Prerequisites:
#   - Gazebo simulation window must be open
#   - RViz window must be open
#   - ROS policy node must be running (publishing CAMERA_TOPIC)
#   - sudo apt install xdotool  (if not already installed)
#
# Output:
#   results/videos/MODEL_NAME_test001.mp4  (auto-increments if exists)
#   Layout: [Gazebo | RViz | Camera] side-by-side, all scaled to 480p height

set -e

MODEL_NAME="${1:-bc_red_wp}"
CAMERA_TOPIC="${2:-/bc_red_wp/debug_image}"
FPS=20

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
RESULTS_DIR="$PROJECT_DIR/results/videos"
TMP_DIR="/tmp/flight_record_$$"

mkdir -p "$TMP_DIR" "$RESULTS_DIR"

# ── Auto-increment test number ───────────────────────────────────────────────
N=1
while [ -f "$RESULTS_DIR/${MODEL_NAME}_test$(printf '%03d' $N).mp4" ]; do
    N=$((N+1))
done
OUT_NAME="${MODEL_NAME}_test$(printf '%03d' $N)"
OUT_PATH="$RESULTS_DIR/${OUT_NAME}.mp4"

echo "=================================================="
echo " record_flight.sh"
echo "  Model  : $MODEL_NAME"
echo "  Camera : $CAMERA_TOPIC"
echo "  Output : $OUT_PATH"
echo "=================================================="
echo ""

# ── Check dependencies ───────────────────────────────────────────────────────
for cmd in ffmpeg xdotool; do
    if ! command -v "$cmd" &>/dev/null; then
        echo "[ERROR] '$cmd' not found."
        echo "  Install: sudo apt install $cmd"
        exit 1
    fi
done

# ── Find window geometry ─────────────────────────────────────────────────────
get_geometry() {
    # Usage: get_geometry "window_name_pattern"
    # Returns: "X Y W H" or empty string on failure
    local pattern="$1"
    local wid
    # Try exact name first, then class
    wid=$(xdotool search --name "$pattern" 2>/dev/null | tail -1)
    if [ -z "$wid" ]; then
        wid=$(xdotool search --class "$pattern" 2>/dev/null | tail -1)
    fi
    if [ -z "$wid" ]; then
        echo ""
        return 1
    fi
    local geom
    geom=$(xdotool getwindowgeometry "$wid" 2>/dev/null)
    local x y w h
    x=$(echo "$geom" | grep -oP 'Position: \K[0-9]+(?=,)')
    y=$(echo "$geom" | grep -oP 'Position: [0-9]+,\K[0-9]+(?=\s)')
    w=$(echo "$geom" | grep -oP 'Geometry: \K[0-9]+(?=x)')
    h=$(echo "$geom" | grep -oP 'Geometry: [0-9]+x\K[0-9]+')
    if [ -z "$x" ] || [ -z "$w" ]; then
        echo ""
        return 1
    fi
    # Clamp to screen bounds so x11grab doesn't fail
    local sw sh
    read sw sh <<< "$(xdotool getdisplaygeometry 2>/dev/null)"
    if [ -n "$sw" ] && [ -n "$sh" ]; then
        [ $((x + w)) -gt "$sw" ] && w=$(( sw - x ))
        [ $((y + h)) -gt "$sh" ] && h=$(( sh - y ))
    fi
    # Ensure even dimensions (required by libx264)
    w=$(( (w / 2) * 2 ))
    h=$(( (h / 2) * 2 ))
    echo "$x $y $w $h"
}

echo "[record_flight] Searching for windows..."

GAZEBO_GEO=$(get_geometry "Gazebo")
if [ -z "$GAZEBO_GEO" ]; then
    echo "[ERROR] Gazebo window not found. Is Gazebo open?"
    echo "  Tip: check window title with: xdotool search --onlyvisible --name ''"
    rm -rf "$TMP_DIR"
    exit 1
fi

RVIZ_GEO=$(get_geometry "RViz")
if [ -z "$RVIZ_GEO" ]; then
    echo "[ERROR] RViz window not found. Is RViz open?"
    rm -rf "$TMP_DIR"
    exit 1
fi

read -r G_X G_Y G_W G_H <<< "$GAZEBO_GEO"
read -r R_X R_Y R_W R_H <<< "$RVIZ_GEO"

echo "[record_flight] Gazebo  : ${G_W}x${G_H} at (${G_X}, ${G_Y})"
echo "[record_flight] RViz    : ${R_W}x${R_H} at (${R_X}, ${R_Y})"
echo "[record_flight] Camera  : ${CAMERA_TOPIC}"
echo ""

# ── Temp file paths (MKV: no moov-atom issue, safe to kill mid-recording) ───
GAZEBO_TMP="$TMP_DIR/gazebo.mkv"
RVIZ_TMP="$TMP_DIR/rviz.mkv"
CAMERA_TMP="$TMP_DIR/camera.mp4"

# ── Cleanup + stitch on Ctrl+C ───────────────────────────────────────────────
cleanup() {
    echo ""
    echo "[record_flight] Stopping recordings..."

    # SIGINT → ffmpeg stops gracefully and finalizes the MKV file
    kill -INT "$PID_GAZEBO" 2>/dev/null
    kill -INT "$PID_RVIZ"   2>/dev/null
    # SIGTERM → Python script exits loop and calls writer.release()
    kill -TERM "$PID_CAM"   2>/dev/null

    echo "[record_flight] Waiting for processes to finish..."
    wait "$PID_GAZEBO" 2>/dev/null || true
    wait "$PID_RVIZ"   2>/dev/null || true
    wait "$PID_CAM"    2>/dev/null || true
    sleep 1   # Extra buffer for file flush

    echo "[record_flight] Recordings saved to $TMP_DIR"
    echo ""

    # ── Check files exist ────────────────────────────────────────────────────
    MISSING=""
    [ ! -s "$GAZEBO_TMP" ] && MISSING="$MISSING gazebo(mkv)"
    [ ! -s "$RVIZ_TMP"   ] && MISSING="$MISSING rviz(mkv)"
    [ ! -s "$CAMERA_TMP" ] && MISSING="$MISSING camera(mp4)"
    if [ -n "$MISSING" ]; then
        echo "[WARN] Missing/empty recordings:$MISSING"
        echo "       Skipping stitch. Temp files kept in $TMP_DIR"
        exit 1
    fi

    # ── Stitch ───────────────────────────────────────────────────────────────
    # Match Gazebo and RViz height to camera's actual height (proportional)
    CAM_H=$(ffprobe -v quiet -select_streams v:0 \
            -show_entries stream=height -of csv=p=0 "$CAMERA_TMP" 2>/dev/null)
    CAM_H=$(( (${CAM_H:-480} / 2) * 2 ))   # ensure even; fallback 480
    echo "[record_flight] Stitching three views at height=${CAM_H}px..."
    ffmpeg -loglevel warning \
        -i "$GAZEBO_TMP" \
        -i "$RVIZ_TMP"   \
        -i "$CAMERA_TMP" \
        -filter_complex "
            [0:v]scale=-2:${CAM_H},setsar=1[v0];
            [1:v]scale=-2:${CAM_H},setsar=1[v1];
            [2:v]scale=-2:${CAM_H},setsar=1[v2];
            [v0][v1][v2]hstack=inputs=3[out]
        " \
        -map "[out]" \
        -c:v libx264 -preset medium -crf 22 \
        "$OUT_PATH"

    if [ -f "$OUT_PATH" ]; then
        DURATION=$(ffprobe -v quiet -show_entries format=duration \
                   -of csv=p=0 "$OUT_PATH" 2>/dev/null | cut -d. -f1)
        SIZE=$(du -h "$OUT_PATH" | cut -f1)
        echo ""
        echo "=================================================="
        echo " DONE"
        echo "  File     : $OUT_PATH"
        echo "  Duration : ${DURATION}s"
        echo "  Size     : ${SIZE}"
        echo "=================================================="
        rm -rf "$TMP_DIR"
    else
        echo "[ERROR] Stitch failed. Temp files kept in $TMP_DIR"
        exit 1
    fi

    exit 0
}

trap cleanup SIGINT SIGTERM

# ── Start recordings ─────────────────────────────────────────────────────────
echo "[record_flight] Recording... Press Ctrl+C to stop."
echo ""

# setsid isolates ffmpeg from the terminal's Ctrl+C signal group.
# We send SIGINT explicitly in cleanup() so ffmpeg can finalize cleanly.
setsid ffmpeg -loglevel warning \
    -f x11grab -r "$FPS" -video_size "${G_W}x${G_H}" \
    -i ":0.0+${G_X},${G_Y}" \
    -c:v libx264 -preset ultrafast -crf 23 -f matroska \
    "$GAZEBO_TMP" &
PID_GAZEBO=$!

setsid ffmpeg -loglevel warning \
    -f x11grab -r "$FPS" -video_size "${R_W}x${R_H}" \
    -i ":0.0+${R_X},${R_Y}" \
    -c:v libx264 -preset ultrafast -crf 23 -f matroska \
    "$RVIZ_TMP" &
PID_RVIZ=$!

# Source ROS for camera recorder, then run with dbc241 Python
source /opt/ros/noetic/setup.bash 2>/dev/null
source /home/l/catkin_ws/devel/setup.bash 2>/dev/null
/home/l/miniconda3/envs/dbc241/bin/python3 \
    "$SCRIPT_DIR/record_camera_topic.py" \
    "$CAMERA_TMP" "$CAMERA_TOPIC" "$FPS" &
PID_CAM=$!

echo "[record_flight] PIDs  gazebo=$PID_GAZEBO  rviz=$PID_RVIZ  cam=$PID_CAM"
echo ""

# Wait until Ctrl+C
wait
