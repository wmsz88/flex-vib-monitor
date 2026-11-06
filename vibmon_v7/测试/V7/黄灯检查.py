#!/usr/bin/env python3
"""T010 改动 A 实板检查：黄灯三态 + 回显 + 重标定两灯亮（人目视灯态，脚本记串口）。

用法：python -X utf8 -u 黄灯检查.py --port COM9 --out 黄灯.txt
时序（相对开串口后）：
   4s  'y'  -> 期望 [LEDY] ON    （黄灯常亮）
   9s  'S'  -> 状态行应含 "黄灯 [LEDY] ON"
  11s  'y'  -> [LEDY] OFF        （黄灯常灭）
  16s  'y'  -> [LEDY] AUTO       （回到接触闪）
  18s  '?'  -> 帮助应含 "y 黄灯"
  20s  'c'  -> 重标定约 3.2s，期间两灯同时亮（CAL）
  36s  结束
"""
import argparse
import sys
import time
from datetime import datetime

import serial


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    plan = [(4, "y"), (9, "S"), (11, "y"), (16, "y"), (18, "?"), (20, "c")]
    end = 36.0

    sen = serial.Serial(args.port, 115200, timeout=0.2)
    fp = open(args.out, "w", encoding="utf-8")
    t0 = time.time()

    def stamp():
        return datetime.now().strftime("%H:%M:%S.%f")[:-3]

    def emit(tag, text):
        line = f"{stamp()} {tag} {text}"
        fp.write(line + "\n")
        fp.flush()
        print(line, flush=True)          # 先落文件再打印，控制台编码问题不影响记录

    emit("##", f"打开 {args.port}（会复位模块，请保持张开/静止）")
    time.sleep(3.0)
    sen.reset_input_buffer()
    sen.write(b"d")                      # 打开调试行
    emit("TX", "d")
    buf = ""
    i = 0
    nxt = time.time()
    while time.time() - t0 < end:
        now = time.time() - t0
        while i < len(plan) and now >= plan[i][0]:
            c = plan[i][1]
            sen.write(c.encode())
            emit("TX", repr(c))
            i += 1
        chunk = sen.read(512)
        if chunk:
            buf += chunk.decode("utf-8", "replace")
            while "\n" in buf:
                line, buf = buf.split("\n", 1)
                if line.strip():
                    emit("RX", line.strip("\r"))
        else:
            time.sleep(0.02)
    fp.close()
    sen.close()
    emit("##", "结束")


if __name__ == "__main__":
    # 控制台码页可能是 cp437/cp936，中文/≈ 打不出来时用替换字符，别让脚本崩
    try:
        sys.stdout.reconfigure(errors="replace")
    except Exception:
        pass
    main()
