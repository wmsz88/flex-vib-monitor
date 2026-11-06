#!/usr/bin/env python3
"""振源 C3-KEY 串口控制（T013 实板体检用）。

脚本串一次开串口完成（开串口会复位振源，中途不能再开一次）：
  段之间用 ';' 分隔，段要么是命令字符串（原样写出），要么是 wait:N（等 N 秒）。

用法：
  python -X utf8 -u 振源控制.py --port COM7 --script "b;T2;wait:35;z"
"""
import argparse
import time
from datetime import datetime

import serial


def log(msg):
    print(f"{datetime.now().strftime('%H:%M:%S.%f')[:-3]} {msg}", flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--script", required=True)
    ap.add_argument("--boot", type=float, default=2.5, help="开串口后等复位启动的秒数")
    args = ap.parse_args()

    s = serial.Serial(args.port, 115200, timeout=0.2)
    time.sleep(args.boot)
    s.reset_input_buffer()
    log("振源就绪（开串口已复位它）")
    log("RX " + "\nRX ".join(x for x in s.read(4000).decode("utf-8", "replace").splitlines() if x.strip()))

    for seg in args.script.split(";"):
        seg = seg.strip()
        if not seg:
            continue
        if seg.startswith("wait:"):
            hold = float(seg[5:])
            log(f"-- 保持 {hold}s --")
            t = time.time()
            while time.time() - t < hold:
                d = s.read(500)
                if d:
                    log("RX " + "\nRX ".join(x for x in d.decode("utf-8", "replace").splitlines() if x.strip()))
                time.sleep(0.05)
        else:
            s.write(seg.encode())
            log(f"TX {seg!r}")
            time.sleep(0.25)
            d = s.read(3000)
            if d:
                log("RX " + "\nRX ".join(x for x in d.decode("utf-8", "replace").splitlines() if x.strip()))
    s.close()
    log("振源串口关闭")


if __name__ == "__main__":
    main()
