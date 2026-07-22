#!/home/l/miniconda3/envs/dbc241/bin/python3
"""
record_camera_topic.py — Subscribe to a ROS Image topic and write frames to MP4.

Usage (called by record_flight.sh):
  python3 record_camera_topic.py <output.mp4> [<topic>] [<fps>]

Does NOT use cv_bridge (avoids libffi conflict with conda).
"""

import sys
import signal
import numpy as np

# Inject ROS Python packages so dbc241 conda python can import rospy
sys.path.insert(0, '/opt/ros/noetic/lib/python3/dist-packages')

import cv2
import rospy
from sensor_msgs.msg import Image

output_path = sys.argv[1] if len(sys.argv) > 1 else '/tmp/flight_camera.mp4'
topic       = sys.argv[2] if len(sys.argv) > 2 else '/bc_red_wp/debug_image'
fps         = float(sys.argv[3]) if len(sys.argv) > 3 else 20.0

writer  = None
running = True
frame_count = 0


def shutdown(*_):
    global running
    running = False


signal.signal(signal.SIGTERM, shutdown)
signal.signal(signal.SIGINT,  shutdown)


def image_cb(msg):
    global writer, frame_count
    try:
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width, 3)
    except Exception as e:
        print(f"[CamRec] decode error: {e}", flush=True)
        return

    if writer is None:
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        writer = cv2.VideoWriter(output_path, fourcc, fps,
                                 (msg.width, msg.height))
        print(f"[CamRec] {msg.width}x{msg.height} @{fps:.0f}fps → {output_path}",
              flush=True)

    writer.write(img)
    frame_count += 1
    if frame_count % 100 == 0:
        print(f"[CamRec] {frame_count} frames written", flush=True)


rospy.init_node('flight_camera_recorder', anonymous=True)
sub = rospy.Subscriber(topic, Image, image_cb, queue_size=2)
print(f"[CamRec] Subscribed to {topic}", flush=True)

try:
    r = rospy.Rate(200)
    while not rospy.is_shutdown() and running:
        r.sleep()
except (rospy.exceptions.ROSInterruptException, KeyboardInterrupt):
    pass

sub.unregister()
if writer is not None:
    writer.release()
    print(f"[CamRec] Done — {frame_count} frames → {output_path}", flush=True)
else:
    print(f"[CamRec] WARNING: no frames received from {topic}", flush=True)
