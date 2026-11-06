#!/usr/bin/env python3
"""接触判据扫描：控振源(COM7)逐档升占空比，同时采 vibmon 绘图行(COM9)。

回答的问题：用当前振子，接触判据 ct 从哪一档（哪个频率/幅值）开始成立。
机械要求：全程保持同一姿态——把模块稳稳压住被测物体，别松手。

vibmon 绘图行（PLOT_RESULT）：
  Freq_Hz:%.1f,Amp_mg:%.1f,SS_Hz_x10:%.1f,Vib_x50:%d,Contact_x100:%d

用法：
  python -X utf8 contact_sweep.py --source COM7 --sensor COM9 --out <out.csv> --arm 25
"""
import argparse
import csv
import os
import re
import sys
import time

import serial

# 细扫：55% 起步（'p' 带起步踢一脚），之后 '+' 每步 +5% 到 100%
LEVELS = [("p", "55%", 55.0)] + [("+", f"{d}%", float(d)) for d in range(60, 101, 5)]

PLOT_RE = re.compile(
    r"Freq_Hz:(?P<f>-?[\d.]+),Amp_mg:(?P<a>-?[\d.]+),"
    r"SS_Hz_x10:(?P<ss>-?[\d.]+),Vib_x50:(?P<vib>\d+),Contact_x100:(?P<ct>\d+)"
)
DUTY_RE = re.compile(rb"\[DUTY\]\s*(\d+)%")
FIELDS = ["level", "duty", "phase", "freq_hz", "amp_mg", "ss_hz", "vib", "ct"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", required=True)
    ap.add_argument("--sensor", required=True)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--out", required=True)
    ap.add_argument("--arm", type=float, default=25.0)
    ap.add_argument("--settle", type=float, default=3.0)
    ap.add_argument("--hold", type=float, default=8.0)
    args = ap.parse_args()

    outdir = os.path.dirname(os.path.abspath(args.out))
    if outdir:
        os.makedirs(outdir, exist_ok=True)

    src = serial.Serial(args.source, args.baud, timeout=0.2)
    time.sleep(0.6)
    src.reset_input_buffer()
    src.write(b"z")
    time.sleep(0.3)

    # 等模块启动 + 标定完成（开串口会复位它）；USB-CDC 端点偶有僵住，没数据就重开重试
    sen = None
    ready = False
    for attempt in range(1, 4):
        if sen is not None:
            try:
                sen.close()
            except Exception:
                pass
        sen = serial.Serial(args.sensor, args.baud, timeout=0.2)
        print(f"等待模块启动/标定（第 {attempt} 次，最长 15s，保持静止）...", flush=True)
        deadline = time.time() + 15.0
        while time.time() < deadline:
            chunk = sen.read(4096)
            if chunk and PLOT_RE.search(chunk.decode("utf-8", "replace")):
                ready = True
                break
            time.sleep(0.05)
        if ready:
            print("模块就绪。", flush=True)
            break
        print("  没看到绘图行，重开串口重试。", flush=True)
        time.sleep(1.0)
    if not ready:
        print("错误：模块 45s 内没有输出绘图行，放弃。", flush=True)
        sen.close(); src.close()
        sys.exit(2)
    sen.reset_input_buffer()

    if args.arm > 0:
        print(f"就位倒计时 {args.arm:.0f}s —— 请压稳被测物，全程别松手。", flush=True)
        time.sleep(args.arm)

    rows = []
    raw = open(args.out + ".raw.txt", "w", encoding="utf-8", errors="replace")
    t0 = time.time()
    try:
        for i, (cmd, name, duty) in enumerate(LEVELS):
            key = f"L{i:02d}"
            src.reset_input_buffer()      # 丢掉上一条指令的旧应答
            src.write(cmd.encode())
            time.sleep(0.25)
            src.write(b"q")
            qb = b""
            qt = time.time()
            while time.time() - qt < 0.35:
                qb += src.read(256)
                time.sleep(0.01)
            qm = DUTY_RE.search(qb)
            if qm:
                duty = float(qm.group(1))
            print(f"[{time.time() - t0:6.1f}s] 设置 {key} {cmd}({duty:.0f}%)", flush=True)
            seg_start = time.time()
            phase = "settle"
            while True:
                el = time.time() - seg_start
                if phase == "settle" and el >= args.settle:
                    phase = "hold"
                if el >= args.settle + args.hold:
                    break
                chunk = sen.read(2048)
                if not chunk:
                    time.sleep(0.01)
                    continue
                text = chunk.decode("utf-8", "replace")
                raw.write(text)
                for line in text.splitlines():
                    m = PLOT_RE.search(line)
                    if not m:
                        continue
                    rows.append({
                        "level": key, "duty": duty, "phase": phase,
                        "freq_hz": m["f"], "amp_mg": m["a"], "ss_hz": m["ss"],
                        "vib": int(m["vib"]) // 50, "ct": int(m["ct"]) // 100,
                    })
    finally:
        try:
            src.write(b"z")
        except Exception:
            pass
        raw.close(); sen.close(); src.close()

    with open(args.out, "w", newline="", encoding="utf-8-sig") as fp:
        w = csv.DictWriter(fp, fieldnames=FIELDS)
        w.writeheader(); w.writerows(rows)
    print(f"完成：{len(rows)} 行 -> {args.out}", flush=True)

    print("\n档位,占空比,N,freq中位,freq[min],freq[max],amp中位,amp[max],ct比例,vib比例", flush=True)
    for i, (cmd, name, duty) in enumerate(LEVELS):
        key = f"L{i:02d}"
        hs = [r for r in rows if r["level"] == key and r["phase"] == "hold"]
        n = len(hs)
        if n == 0:
            print(f"{cmd}({duty:.0f}%),{duty},0,-,-,-,-,-,-,-", flush=True)
            continue
        fs = sorted(float(r["freq_hz"]) for r in hs if float(r["freq_hz"]) > 0)
        am = sorted(float(r["amp_mg"]) for r in hs)
        ct = sum(int(r["ct"]) for r in hs)
        vib = sum(int(r["vib"]) for r in hs)
        fmed = fs[len(fs) // 2] if fs else 0.0
        fr = f"{fs[0]:.1f},{fs[-1]:.1f}" if fs else "-, -"
        print(f"{cmd}({duty:.0f}%),{duty},{n},{fmed:.1f},{fr},"
              f"{am[len(am) // 2]:.1f},{am[-1]:.1f},{ct / n:.0%},{vib / n:.0%}", flush=True)


if __name__ == "__main__":
    main()
