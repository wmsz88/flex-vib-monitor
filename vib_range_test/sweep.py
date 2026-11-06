#!/usr/bin/env python3
"""duty 扫频：同时控振源(COM7)与检测模块(COM9)，逐档记录。

振源指令（vibsource 固件）：
  0~9  占空比 = 数字x10%   h=100%   p=55%(约80Hz工作点)   z=停   ?=帮助

模块输出（vib_range_test 固件）：
  100Hz CSV : t_ms,freq_hz,env_mg,axis,clip,fx,fy,fz

每档：settle 秒后进入 hold 段才计入统计（起转/变速的过渡丢掉）。

用法：
  python -X utf8 sweep.py --source COM7 --sensor COM9 --out <out.csv> --arm 30
"""
import argparse
import csv
import os
import re
import sys
import time

import serial

# (振源指令, 说明, 占空比%)
LEVELS = [
    ("z", "停(基线)", 0.0),
    ("h", "100%", 100.0),
    ("9", "90%", 90.0),
    ("8", "80%", 80.0),
    ("7", "70%", 70.0),
    ("p", "55%", 55.0),
]

# 细扫：从 55% 起一路按 '+' 每步 +5%，直到 100%（避免低于启转门槛导致停转）
FINE_LEVELS = [("p", "55%", 55.0)] + [("+", f"{d}%", float(d)) for d in range(60, 101, 5)]

CSV_RE = re.compile(r"^(?P<t>\d+),(?P<f>-?[\d.]+),(?P<a>-?[\d.]+),(?P<ax>\d+),(?P<cl>\d+),")
DUTY_RE = re.compile(rb"\[DUTY\]\s*(\d+)%")
FIELDS = ["level", "duty", "phase", "t_ms", "freq_hz", "env_mg", "axis", "clip"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", required=True)      # 振源 COM7
    ap.add_argument("--sensor", required=True)      # 检测模块 COM9
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--out", required=True)
    ap.add_argument("--arm", type=float, default=30.0)    # 开跑前就位倒计时
    ap.add_argument("--settle", type=float, default=3.0)  # 变速后过渡丢弃
    ap.add_argument("--hold", type=float, default=8.0)    # 每档有效记录时长
    ap.add_argument("--fine", action="store_true")        # 5%/步 细扫
    args = ap.parse_args()
    levels = FINE_LEVELS if args.fine else LEVELS

    outdir = os.path.dirname(os.path.abspath(args.out))
    if outdir:
        os.makedirs(outdir, exist_ok=True)

    src = serial.Serial(args.source, args.baud, timeout=0.2)
    time.sleep(0.6)
    src.reset_input_buffer()
    src.write(b"z")          # 先从停转开始
    time.sleep(0.3)

    # 等检测模块启动 + 标定完成（开串口会复位它）。
    # USB-CDC 端点有时会僵住（例如上一个占用者被杀掉），所以没看见 CSV 就重开串口重试。
    sen = None
    ready = False
    for attempt in range(1, 4):
        if sen is not None:
            try:
                sen.close()
            except Exception:
                pass
        sen = serial.Serial(args.sensor, args.baud, timeout=0.2)
        print(f"等待检测模块启动/标定（第 {attempt} 次，最长 15s，保持静止）...", flush=True)
        deadline = time.time() + 15.0
        while time.time() < deadline:
            chunk = sen.read(4096)
            if chunk and CSV_RE.search(chunk.decode("utf-8", "replace")):
                ready = True
                break
            time.sleep(0.05)
        if ready:
            print("模块就绪。", flush=True)
            break
        print("  没看到 CSV，重开串口重试。", flush=True)
        time.sleep(1.0)
    if not ready:
        print("错误：模块 45s 内没有输出 CSV，放弃。", flush=True)
        try:
            src.write(b"z")
        finally:
            sen.close()
            src.close()
        sys.exit(2)
    sen.reset_input_buffer()

    if args.arm > 0:
        print(f"就位倒计时 {args.arm:.0f}s —— 请把模块压到 4cm 方块上并保持住。", flush=True)
        time.sleep(args.arm)

    rows = []
    raw = open(args.out + ".raw.txt", "w", encoding="utf-8", errors="replace")
    t0 = time.time()
    try:
        for i, (cmd, name, duty) in enumerate(levels):
            key = f"L{i:02d}"
            src.reset_input_buffer()   # 丢掉上一条指令的旧应答，避免读出上一个占空比
            src.write(cmd.encode())
            time.sleep(0.25)
            src.write(b"q")          # 查询实际占空比（'+' 是逐步累加的）
            qb = b""
            qt = time.time()
            while time.time() - qt < 0.35:
                qb += src.read(256)
                time.sleep(0.01)
            qm = DUTY_RE.search(qb)
            if qm:
                duty = float(qm.group(1))
            seg = f"{cmd}({duty:.0f}%)"
            print(f"[{time.time() - t0:6.1f}s] 设置 {key} {seg}", flush=True)
            seg_start = time.time()
            phase = "settle"
            while True:
                el = time.time() - seg_start
                if phase == "settle" and el >= args.settle:
                    phase = "hold"
                    print(f"[{time.time() - t0:6.1f}s]   {seg} 稳定，记录 {args.hold:.0f}s", flush=True)
                if el >= args.settle + args.hold:
                    break
                chunk = sen.read(2048)
                if not chunk:
                    time.sleep(0.01)
                    continue
                text = chunk.decode("utf-8", "replace")
                raw.write(text)
                for line in text.splitlines():
                    m = CSV_RE.match(line)
                    if not m:
                        continue
                    rows.append({
                        "level": key, "duty": duty, "phase": phase,
                        "t_ms": m["t"], "freq_hz": m["f"], "env_mg": m["a"],
                        "axis": m["ax"], "clip": m["cl"],
                    })
    finally:
        try:
            src.write(b"z")
        except Exception:
            pass
        raw.close()
        sen.close()
        src.close()

    with open(args.out, "w", newline="", encoding="utf-8-sig") as fp:
        w = csv.DictWriter(fp, fieldnames=FIELDS)
        w.writeheader()
        w.writerows(rows)
    print(f"完成：{len(rows)} 行 -> {args.out}", flush=True)

    print("\n档位,占空比,N,freq中位,freq[min],freq[max],env中位,env[max],freq>0比例", flush=True)
    for i, (cmd, name, duty) in enumerate(levels):
        key = f"L{i:02d}"
        hs = [r for r in rows if r["level"] == key and r["phase"] == "hold"]
        seg = f"{cmd}({duty:.0f}%)"
        n = len(hs)
        if n == 0:
            print(f"{seg},{duty},0,-,-,-,-,-,-", flush=True)
            continue
        fs = sorted(float(r["freq_hz"]) for r in hs if float(r["freq_hz"]) > 0)
        es = sorted(float(r["env_mg"]) for r in hs)
        fmed = fs[len(fs) // 2] if fs else 0.0
        fr = f"{fs[0]:.1f},{fs[-1]:.1f}" if fs else "-, -"
        print(f"{seg},{duty},{n},{fmed:.1f},{fr},"
              f"{es[len(es) // 2]:.1f},{es[-1]:.1f},{len(fs)}/{n}", flush=True)


if __name__ == "__main__":
    main()
