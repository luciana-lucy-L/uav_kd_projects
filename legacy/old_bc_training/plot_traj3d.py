import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
import os

# === 1) 多个 CSV 路径 ===
csv_paths = [
    "~/bc_logs/pose_after_takeoff_20260119_125745.csv",
    "~/bc_logs/pose_after_takeoff_20260109_140143.csv",
    "~/bc_logs/pose_after_takeoff_20260109_140939.csv",
]

# 展开 ~
csv_paths = [os.path.expanduser(p) for p in csv_paths]

# === 2) 创建画布（只创建一次） ===
fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")

def pick_col(df, candidates):
    for c in candidates:
        if c in df.columns:
            return c
    for c in df.columns:
        if any(k in c for k in candidates):
            return c
    raise ValueError(f"Cannot find columns: {candidates}\nAvailable columns:\n{df.columns}")

# === 3) 逐条轨迹画 ===
for csv_path in csv_paths:
    df = pd.read_csv(csv_path)

    x_col = pick_col(df, ["pose.position.x", "pose.pose.position.x", "position.x"])
    y_col = pick_col(df, ["pose.position.y", "pose.pose.position.y", "position.y"])
    z_col = pick_col(df, ["pose.position.z", "pose.pose.position.z", "position.z"])

    x = df[x_col].astype(float).to_numpy()
    y = df[y_col].astype(float).to_numpy()
    z = df[z_col].astype(float).to_numpy()

    # 每条轨迹单独平移到原点
    x -= x[0]
    y -= y[0]
    z -= z[0]

    label = os.path.basename(csv_path)
    ax.plot(x, y, z, label=label)

# === 4) 统一设置 ===
ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")
ax.set_zlabel("Z (m)")
ax.set_title("UAV 3D Trajectories (Multiple Runs)")
ax.legend()

plt.tight_layout()
plt.savefig("traj_3d_multi.png", dpi=200)
plt.show()

print("Saved: traj_3d_multi.png")
