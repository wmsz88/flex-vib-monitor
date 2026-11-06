#!/usr/bin/env python3
"""vibmon_v6 出发体检：带墙钟时间戳的连续采集（T013 实板检查 A~D 用）。

和先前 V6\\*.txt 的格式一致（每行 "HH:MM:SS.mmm $VD,..."），
所以 test\\ss_eval.py、测试\\V6\\_win.py 可以直接吃新文件。

用法：
  python -X utf8 -u 采集.py --port COM4 --out 检查A.txt --secs 60

注意：
  1. 打开串口会复位模块 —— 上电/复位时必须处于张开(START)姿态，否则 ang 恒 0、ct 永远 0。
  2. 默认先发 'd' 打开调试行（#V6 closed= ang= lpW= settled= moved=）；
     不想要调试行就加 --no-debug。
"""
import argparse
import sys
import time
from datetime import datetime

import serial


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--out", required=True)
    ap.add_argument("--secs", type=float, default=60.0)
    ap.add_argument("--no-debug", action="store_true", help="不发 'd'，只采 $VD 帧")
    args = ap.parse_args()

    sen = serial.Serial(args.port, args.baud, timeout=0.2)
    time.sleep(0.5)
    if not args.no_debug:
        sen.write(b"d")          # 单字符命令；不要带换行（换行会被当成未知命令）
        time.sleep(0.2)
    sen.reset_input_buffer()
    print(f"采集启动：{args.port} -> {args.out}（开串口已复位模块，请保持张开/静止）", flush=True)

    fp = open(args.out, "w", encoding="utf-8")
    t0 = time.time()
    buf = ""
    n = 0
    try:
        while time.time() - t0 < args.secs:
            chunk = sen.read(512)
            if not chunk:
                continue
            buf += chunk.decode("utf-8", "replace")
            while "\n" in buf:
                line, buf = buf.split("\n", 1)
                line = line.strip("\r")
                if not line:
                    continue
                fp.write(f"{datetime.now().strftime('%H:%M:%S.%f')[:-3]} {line}\n")
                n += 1
            fp.flush()
    except KeyboardInterrupt:
        pass
    finally:
        fp.close()
        sen.close()
        print(f"采集结束：{n} 行，用时 {time.time() - t0:.1f}s", flush=True)


if __name__ == "__main__":
    try:
        main()
    except serial.SerialException as e:
        print(f"串口错误：{e}", file=sys.stderr)
        sys.exit(2)
