#!/usr/bin/env python3
r"""持续采集 vibmon 的 Plot 结果行（含 SS_Hz），直到进程被杀。

Plot 组 1 输出：
  Freq_Hz:...,Amp_mg:...,SS_Hz_x10:...,Vib_x50:...,Contact_x100:...

用法：
  python -X utf8 -u plot_logger.py --sensor COM9 --out data\startstop_live.csv
"""

import argparse
import csv
import os
import re
import time
from datetime import datetime

import serial

PLOT_RE = re.compile(
    r"Freq_Hz:(?P<f>-?[\d.]+),Amp_mg:(?P<a>-?[\d.]+),"
    r"SS_Hz_x10:(?P<ss>-?[\d.]+),Vib_x50:(?P<vib>\d+),"
    r"Contact_x100:(?P<ct>\d+)"
)
FIELDS = ["wall", "epoch", "Freq_Hz", "Amp_mg", "SS_Hz", "Vib", "Contact"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sensor", required=True)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--out", required=True)
    ap.add_argument("--secs", type=float, default=1800.0)
    args = ap.parse_args()

    outdir = os.path.dirname(os.path.abspath(args.out))
    if outdir:
        os.makedirs(outdir, exist_ok=True)

    sen = serial.Serial(args.sensor, args.baud, timeout=0.2)
    time.sleep(0.5)
    sen.write(b"1")          # 绘图组 1：Freq/Amp/SS/Vib/Contact
    time.sleep(0.3)
    sen.reset_input_buffer()
    print("plot_logger 启动（开串口已复位模块，请保持张开/静止）",
          flush=True)

    fp = open(args.out, "w", newline="", encoding="utf-8-sig")
    writer = csv.DictWriter(fp, fieldnames=FIELDS)
    writer.writeheader()
    fp.flush()
    raw = open(args.out + ".raw.txt", "w", encoding="utf-8", errors="replace")

    t0 = time.time()
    next_plot_cmd = 1.0
    seen_data = False
    last_data = time.time()
    try:
        while time.time() - t0 < args.secs:
            if not seen_data and time.time() - t0 >= next_plot_cmd:
                sen.write(b"1")     # 启动早期命令可能丢，重复发到出数据
                next_plot_cmd += 1.0
            chunk = sen.read(2048)
            if not chunk:
                if time.time() - last_data > 3.0:
                    print("[!!] 3s 无数据（链路可能掉了）", flush=True)
                    last_data = time.time()
                time.sleep(0.01)
                continue
            last_data = time.time()
            text = chunk.decode("utf-8", "replace")
            raw.write(text)
            raw.flush()
            for line in text.splitlines():
                m = PLOT_RE.search(line)
                if not m:
                    continue
                seen_data = True
                now = datetime.now()
                writer.writerow({
                    "wall": now.strftime("%H:%M:%S.%f")[:-3],
                    "epoch": f"{time.time():.3f}",
                    "Freq_Hz": m["f"],
                    "Amp_mg": m["a"],
                    "SS_Hz": f"{float(m['ss']) / 10.0:.2f}",
                    "Vib": int(m["vib"]) // 50,
                    "Contact": int(m["ct"]) // 100,
                })
                fp.flush()
    except KeyboardInterrupt:
        pass
    finally:
        fp.close()
        raw.close()
        sen.close()
        print("plot_logger 结束", flush=True)


if __name__ == "__main__":
    main()
