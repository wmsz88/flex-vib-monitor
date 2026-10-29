#!/usr/bin/env python3
"""10Hz 采集闭合轨迹，并按"角度平台"自动分段。

用途：验证"夹空气 vs 夹物体"的轨迹差异（方案 §13）。
平台判据：角度连续 >=1.5s 变化 <3°，视为停在某个角度。

用法：python -X utf8 traj_capture.py --port COM8 --secs 90 --out data\\traj3
"""

import argparse
import csv
import os
import re
import time

import serial

RE = re.compile(
    r"#SWEEP t=(\d+)s F=\s*([0-9.]+|\?)Hz A=\s*([0-9.]+|\?)mg "
    r"D=\s*([0-9.]+|\?)mg ang=\s*([0-9.]+|\?)deg"
)


def num(x):
    try:
        return float(x)
    except (TypeError, ValueError):
        return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--secs", type=float, default=90)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    ser = serial.Serial(args.port, args.baud, timeout=0.05)
    time.sleep(0.5)
    ser.reset_input_buffer()

    rows = []
    t0 = time.time()
    nxt = 0.0
    while time.time() - t0 < args.secs:
        el = time.time() - t0
        if el >= nxt:
            ser.write(b"w")
            nxt = el + 0.1
        chunk = ser.read(256)
        if not chunk:
            continue
        for line in chunk.decode("utf-8", "replace").splitlines():
            m = RE.search(line)
            if m:
                rows.append((round(el, 2), m.group(5), m.group(2), m.group(3)))
    ser.close()

    with open(args.out + ".csv", "w", newline="", encoding="utf-8-sig") as fp:
        w = csv.writer(fp)
        w.writerow(["t", "ang", "F", "A"])
        w.writerows(rows)
    print(f"rows {len(rows)}")

    # 平台分段
    seg_start = 0
    segs = []
    for i in range(1, len(rows)):
        a0 = num(rows[seg_start][1])
        a1 = num(rows[i][1])
        if a0 is None or a1 is None or abs(a1 - a0) >= 3:
            if num(rows[i - 1][0]) - num(rows[seg_start][0]) >= 1.5:
                amps = [num(r[3]) for r in rows[seg_start:i] if num(r[3]) is not None]
                segs.append((num(rows[seg_start][0]), num(rows[i - 1][0]), a0,
                             max(amps) if amps else 0.0))
            seg_start = i

    print("%-9s %-9s %6s %8s" % ("from", "to", "ang", "A_max"))
    for a, b, ang, amax in segs:
        if ang is not None and ang > 40:
            print("%8.1fs %8.1fs %6.0f %8.0f" % (a, b, ang, amax))


if __name__ == "__main__":
    main()
