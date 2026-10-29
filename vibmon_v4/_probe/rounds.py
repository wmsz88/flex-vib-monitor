#!/usr/bin/env python3
"""三轮「平放 -> 夹空气 -> 平放 -> 夹物体」，每段平放自动重采基准。

为什么要自动按 t：D 是"相对基准的姿态变化"，换一次位置基准就失效。
在每个平放段的末尾按一次 t（此时机械手已静止），后面的闭合才是从
这个位置量起的，不同轮次之间才能比较。

用法：
  python -X utf8 rounds.py --sensor COM8 --out data\\R3
"""

import argparse
import csv
import os
import re
import time

import serial

SWEEP_RE = re.compile(
    r"#SWEEP t=(?P<t>\d+)s F=\s*(?P<f>[0-9.]+|\?)Hz A=\s*(?P<a>[0-9.]+|\?)mg "
    r"D=\s*(?P<d>[0-9.]+|\?)mg tr=\s*(?P<tr>[0-9.]+|\?)mg vib=(?P<vib>\d+) "
    r"ct=(?P<ct>\d+) grip=(?P<grip>\d+) cliplim=(?P<clim>\d+) ax=(?P<ax>\d+) ok=(?P<ok>\d+)"
)

FIELDS = ["elapsed_s", "round", "phase", "Freq_Hz", "Amp_mg", "FlatDist_mg",
          "RefSwing_mg", "vib", "contact", "gripNoObject", "clipLimited",
          "axis", "ok"]

# (时长s, 阶段名, 是否在本段末尾按 t)
SEGMENTS = [
    (14, "R1 平放（等基准）", True),
    (22, "R1 夹空气", False),
    (12, "R1 平放", True),
    (22, "R1 夹物体#1", False),
    (12, "R2 平放", True),
    (22, "R2 夹空气", False),
    (12, "R2 平放", True),
    (22, "R2 夹物体#2", False),
    (12, "R3 平放", True),
    (22, "R3 夹空气", False),
    (12, "R3 平放", True),
    (22, "R3 夹物体#3", False),
]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sensor", required=True)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    ser = serial.Serial(args.sensor, args.baud, timeout=0.2)
    # 【开串口会复位板子】ESP32-C3 的 USB CDC 在端口打开时会被复位，
    # 之后是 ~3.2s 标定 + 0.8s 采基准。必须等它真正开始应答再计时，
    # 否则整个时间表会错位，而且标定期间的数据全是废的。
    print("等待板子启动并完成标定（最长 15s，期间请保持静止）...", flush=True)
    boot_deadline = time.time() + 15.0
    ready = False
    while time.time() < boot_deadline:
        ser.write(b"w")
        time.sleep(0.5)
        chunk = ser.read(2048)
        if chunk and b"#SWEEP" in chunk:
            ready = True
            break
    if not ready:
        raise SystemExit("板子 15s 内没有应答，检查串口/固件")
    ser.write(b"0")           # 关绘图组
    time.sleep(0.4)
    ser.reset_input_buffer()
    print("就绪，开始计时。", flush=True)

    rows = []
    raw = open(args.out + ".raw.txt", "w", encoding="utf-8", errors="replace")
    csv_fp = open(args.out + ".csv", "w", newline="", encoding="utf-8-sig")
    csv_w = csv.DictWriter(csv_fp, fieldnames=FIELDS)
    csv_w.writeheader()
    csv_fp.flush()
    t0 = time.time()
    try:
        for dur, name, do_t in SEGMENTS:
            seg_start = time.time()
            print(f"[{time.time()-t0:6.1f}s] {name}（{dur}s）", flush=True)
            nxt_poll = 0.0
            while time.time() - seg_start < dur:
                el = time.time() - seg_start
                if el >= nxt_poll:
                    ser.write(b"w")
                    nxt_poll = el + 1.0
                chunk = ser.read(512)
                if not chunk:
                    time.sleep(0.02)
                    continue
                text = chunk.decode("utf-8", "replace")
                raw.write(text)
                raw.flush()          # 被中断也不丢已采数据
                for line in text.splitlines():
                    m = SWEEP_RE.search(line)
                    if not m:
                        continue
                    row = {
                        "elapsed_s": f"{time.time()-t0:.1f}",
                        "round": name.split()[0],
                        "phase": name,
                        "Freq_Hz": "" if m["f"] == "?" else m["f"],
                        "Amp_mg": "" if m["a"] == "?" else m["a"],
                        "FlatDist_mg": "" if m["d"] == "?" else m["d"],
                        "RefSwing_mg": "" if m["tr"] == "?" else m["tr"],
                        "vib": m["vib"], "contact": m["ct"],
                        "gripNoObject": m["grip"], "clipLimited": m["clim"],
                        "axis": m["ax"], "ok": m["ok"],
                    }
                    rows.append(row)
                    csv_w.writerow(row)
                    csv_fp.flush()      # 逐行落盘，中断也不丢
            if do_t:
                ser.write(b"t")   # 本段末尾采基准（此时应为静止的平放姿态）
                time.sleep(1.0)
                ser.reset_input_buffer()
    finally:
        raw.close()
        csv_fp.close()
        ser.close()
    print(f"完成：{len(rows)} 行 -> {args.out}.csv")


if __name__ == "__main__":
    main()
