#!/usr/bin/env python3
import csv
import sys
import subprocess
from datetime import datetime

TOPIC = "/pose_20hz"
Z_THRESHOLD = 0.30   # 起飞判定高度（米）
OUT = f"/home/l/bc_logs/pose_after_takeoff_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"


# 启动 rostopic echo -p
p = subprocess.Popen(
    ["rostopic", "echo", "-p", TOPIC],
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE,
    text=True,
    bufsize=1,
)

start_time = None
started = False
writer = None
f = open(OUT, "w", newline="")
try:
    reader = csv.reader(p.stdout)
    header = next(reader)  # 第一行是表头

    # 找 z 列
    z_candidates = ["field.pose.position.z", "pose.position.z", "pose.pose.position.z"]

    z_idx = None
    for name in z_candidates:
        if name in header:
            z_idx = header.index(name)
            break
    if z_idx is None:
        raise RuntimeError(f"Cannot find z column in header: {header}")

    t_idx = header.index("%time") if "%time" in header else None
    if t_idx is None:
        raise RuntimeError("Cannot find %time column in header")

    # 输出新表头：把时间变成 t_rel
    new_header = ["t_rel"] + [h for i, h in enumerate(header) if i != t_idx]
    writer = csv.writer(f)
    writer.writerow(new_header)
    f.flush()

    for row in reader:
        if not row:
            continue
        try:
            t = float(row[t_idx])
            z = float(row[z_idx])
        except Exception:
            continue

        if not started:
            if z > Z_THRESHOLD:
                started = True
                start_time = t

        if started:
            t_rel = t - start_time
            new_row = [f"{t_rel:.6f}"] + [v for i, v in enumerate(row) if i != t_idx]
            writer.writerow(new_row)
            f.flush()
            # 也实时打印高度（可选）
            # print(f"t={t_rel:.2f}s z={z:.3f}m")

finally:
    f.close()
    p.terminate()
    print(f"Saved: {OUT}")
