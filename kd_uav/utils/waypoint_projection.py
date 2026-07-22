"""
waypoint_projection.py

Shared utility for projecting a 3D body-frame waypoint onto the UAV front
camera image plane.  Used by BOTH the offline data-processing pipeline and
the online ROS inference node so that train and deploy use identical logic.

=============================================================================
Coordinate frame conventions
=============================================================================

UAV body frame  (from /CERLAB/quadcopter/pose, traj_body.csv)
  +X  forward
  +Y  left
  +Z  up

Gazebo camera sensor  (from quadcopter.urdf)
  Mounted at base_link with pose: x=0.09m forward, y=0.0m, z=0.095m up
  Rotation: none (0 0 0) — optical axis aligned with body +X

  Gazebo depth camera (`libgazebo_ros_openni_kinect`) looks along the
  sensor's +X direction, which equals body +X (forward).

ROS image / OpenCV pixel convention
  u  increases to the right   →  corresponds to body  -Y  direction
  v  increases downward       →  corresponds to body  -Z  direction

Body-frame point (dx_b, dy_b, dz_b) relative to UAV origin:

  Step 1 — shift to camera origin (subtract camera offset):
    dx_c = dx_b - CAM_X   (CAM_X =  0.09 m)
    dy_c = dy_b - CAM_Y   (CAM_Y =  0.00 m)
    dz_c = dz_b - CAM_Z   (CAM_Z =  0.095 m)

  Step 2 — express in camera optical frame:
    Z_opt =  dx_c   (depth, forward)
    X_opt = -dy_c   (right  = negative left)
    Y_opt = -dz_c   (down   = negative up)

  Step 3 — pinhole projection:
    u = cx + fx * (X_opt / Z_opt)
    v = cy + fy * (Y_opt / Z_opt)

Camera intrinsics (from URDF: width=640, height=480, hfov=60°=1.047198 rad):
  fx = fy = (width/2) / tan(hfov/2) = 320 / tan(30°) ≈ 554.26
  cx = 320,  cy = 240

=============================================================================
"""

import math
import numpy as np
import cv2

# ── Camera mounting offset (body frame, metres) ────────────────────────────
CAM_X = 0.09    # forward
CAM_Y = 0.00    # lateral
CAM_Z = 0.095   # vertical

# ── Camera intrinsics ──────────────────────────────────────────────────────
IMG_W  = 640
IMG_H  = 480
HFOV   = 1.047198        # radians (60°)
FX     = (IMG_W / 2.0) / math.tan(HFOV / 2.0)   # ≈ 554.26
FY     = FX
CX     = IMG_W / 2.0    # 320.0
CY     = IMG_H / 2.0    # 240.0


def project_body_to_pixel(dx_b: float, dy_b: float, dz_b: float):
    """
    Project a 3D waypoint in UAV body frame to image pixel coordinates.

    Args:
        dx_b: forward offset from UAV origin (metres, body +X)
        dy_b: lateral offset from UAV origin (metres, body +Y = left)
        dz_b: vertical offset from UAV origin (metres, body +Z = up)

    Returns:
        (u, v, visible)
          u, v   : float pixel coordinates (may be outside image bounds)
          visible: True if point is in front of camera AND inside image
    """
    # Step 1: translate to camera origin
    dx_c = dx_b - CAM_X
    dy_c = dy_b - CAM_Y
    dz_c = dz_b - CAM_Z

    # Point must be in front of the camera (positive depth)
    if dx_c <= 0.0:
        return 0.0, 0.0, False

    # Step 2: camera optical frame
    Z_opt =  dx_c
    X_opt = -dy_c
    Y_opt = -dz_c

    # Step 3: pinhole projection
    u = CX + FX * (X_opt / Z_opt)
    v = CY + FY * (Y_opt / Z_opt)

    visible = (0 <= u < IMG_W) and (0 <= v < IMG_H)
    return float(u), float(v), visible


def draw_red_waypoint(img: np.ndarray, u: float, v: float,
                      dist_b: float, min_radius: int = 6,
                      max_radius: int = 20, max_dist: float = 5.0) -> np.ndarray:
    """
    Draw a filled red circle on the image at pixel (u, v).

    Radius scales with distance: closer waypoints appear larger.

    Args:
        img       : HxWx3 uint8 BGR image (modified in-place copy)
        u, v      : pixel coordinates
        dist_b    : distance from UAV to waypoint in body frame (metres)
        min_radius: minimum circle radius in pixels
        max_radius: maximum circle radius in pixels
        max_dist  : distance at which radius equals min_radius

    Returns:
        Annotated image (copy of input).
    """
    out = img.copy()
    t = max(0.0, 1.0 - dist_b / max_dist)
    radius = int(round(min_radius + (max_radius - min_radius) * t))
    radius = max(min_radius, radius)

    cx_px = int(round(u))
    cy_px = int(round(v))

    # Filled red circle (BGR: 0, 0, 255), thin white border for contrast
    cv2.circle(out, (cx_px, cy_px), radius + 2, (255, 255, 255), -1)
    cv2.circle(out, (cx_px, cy_px), radius,     (0,   0,   255), -1)
    return out


def select_active_waypoint(traj_body_row: np.ndarray,
                           lookahead_dist: float = 1.0,
                           K: int = 10):
    """
    Select the active waypoint from a traj_body row.

    Chooses the first waypoint with dx_b >= lookahead_dist.
    Falls back to the farthest available waypoint if none qualifies.

    Args:
        traj_body_row : 1D array of length 3*K  [dx0,dy0,dz0, dx1,dy1,dz1, ...]
        lookahead_dist: minimum forward distance threshold (metres)
        K             : number of waypoints

    Returns:
        (dx, dy, dz, wp_index)  — selected waypoint in body frame and its index
    """
    best_idx = K - 1   # fallback: farthest waypoint
    for i in range(K):
        dx = traj_body_row[i * 3]
        if dx >= lookahead_dist:
            best_idx = i
            break

    dx = float(traj_body_row[best_idx * 3])
    dy = float(traj_body_row[best_idx * 3 + 1])
    dz = float(traj_body_row[best_idx * 3 + 2])
    return dx, dy, dz, best_idx
