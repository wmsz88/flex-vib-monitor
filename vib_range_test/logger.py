#!/usr/bin/env python3
"""持续采集 vibmon 的 #SWEEP：带墙钟时间戳一直记，直到进程被杀。

用途：配合"我播报一步、你做一步"的节奏 —— 边界由对话决定，不靠秒表。
每一步的时间段事后按墙钟时间切分即可。

用法：
  python -X utf8 -u logger.py --sensor COM9 --out <out.csv> [--secs 1800]
"""
import argparse
import csv
import os
import re
import sys
import time
from datetime import datetime

import serial

SWEEP_RE = re.compile(
    r"#SWEEP t=(?P<t>\d+)s "
    r"F=\s*(?P<f>[0-9.]+|\?)Hz "
    r"A=\s*(?P<a>[0-9.]+|\?)mg "
    r"D=\s*(?P<d>[0-9.]+|\?)mg "
    r"(?:ang=\s*(?P<ang>[0-9.]+|\?)deg )?"
    r"tr=\s*(?P<tr>[0-9.]+|\?)mg "
    r"vib=(?P<vib>\d+) ct=(?P<ct>\d+) grip=(?P<grip>\d+) cl=(?P<cl>\d+) "
    r"w=(?P<w>[\d.]+) cliplim=(?P<clim>\d+) ax=(?P<ax>\d+) ok=(?P<ok>\d+)"
)
FIELDS = ["wall", "epoch", "t_s", "Freq_Hz", "Amp_mg", "FlatDist_mg", "ang_deg",
          "tr_mg", "vib", "ct", "grip", "closed", "w_dps", "cliplim", "axis", "ok"]


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
    sen.write(b"1")
    time.sleep(0.3)
    sen.reset_input_buffer()
    print("logger 启动（开串口已复位模块，请保持张开/静止）", flush=True)

    fp = open(args.out, "w", newline="", encoding="utf-8-sig")
    w = csv.DictWriter(fp, fieldnames=FIELDS)
    w.writeheader()
    fp.flush()
    raw = open(args.out + ".raw.txt", "w", encoding="utf-8", errors="replace")

    t0 = time.time()
    nxt = 0.0
    last_data = time.time()
    try:
        while time.time() - t0 < args.secs:
            el = time.time() - t0
            if el >= nxt:
                sen.write(b"w")
                nxt = el + 1.0
            chunk = sen.read(512)
            if not chunk:
                if time.time() - last_data > 3.0:
                    print(f"[{el:6.1f}s] !! 3s 无数据（链路可能掉了）", flush=True)
                    last_data = time.time()
                time.sleep(0.02)
                continue
            last_data = time.time()
            text = chunk.decode("utf-8", "replace")
            raw.write(text)
            raw.flush()
            for line in text.splitlines():
                m = SWEEP_RE.search(line)
                if not m:
                    continue
                now = datetime.now()
                row = {
                    "wall": now.strftime("%H:%M:%S.%f")[:-3],
                    "epoch": f"{time.time():.3f}",
                    "t_s": m["t"],
                    "Freq_Hz": "" if m["f"] == "?" else m["f"],
                    "Amp_mg": "" if m["a"] == "?" else m["a"],
                    "FlatDist_mg": "" if m["d"] == "?" else m["d"],
                    "ang_deg": "" if m["ang"] in (None, "?") else m["ang"],
                    "tr_mg": "" if m["tr"] == "?" else m["tr"],
                    "vib": m["vib"], "ct": m["ct"], "grip": m["grip"],
                    "closed": m["cl"], "w_dps": m["w"], "cliplim": m["clim"],
                    "axis": m["ax"], "ok": m["ok"],
                }
                w.writerow(row)
                fp.flush()
    except KeyboardInterrupt:
        pass
    finally:
        fp.close(); raw.close(); sen.close()
        print("logger 结束", flush=True)


if __name__ == "__main__":
    main()
