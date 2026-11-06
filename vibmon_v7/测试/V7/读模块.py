#!/usr/bin/env python3
"""读 vibmon_v7 调试行（#F / #V6），并保证调试行真的是开着的。

命令 d 是开关：如果上一轮结束时调试行还开着，再发一次就关掉了。
所以这里先发 d，1.5s 内看不到 #F= 就再发一次。

用法：python -X utf8 -u 读模块.py --port COM9 --secs 20 --out x.txt
"""
import argparse
import statistics
import time

import serial


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--secs", type=float, default=20)
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    s = serial.Serial(args.port, 115200, timeout=0.2)
    time.sleep(3.0)
    s.reset_input_buffer()

    def probe(secs):
        t0 = time.time()
        buf = ""
        while time.time() - t0 < secs:
            c = s.read(512)
            if c:
                buf += c.decode("utf-8", "replace")
        return buf

    s.write(b"d")
    head = probe(1.5)
    if "#F=" not in head:
        s.write(b"d")
        head += probe(1.5)
    print("调试行 " + ("已开" if "#F=" in head else "没开（两次 d 都没出 #F=）"))

    t0 = time.time()
    buf = head
    rows = []
    while time.time() - t0 < args.secs:
        c = s.read(512)
        if c:
            buf += c.decode("utf-8", "replace")
        while "\n" in buf:
            line, buf = buf.split("\n", 1)
            rows.append(line)
    s.close()

    if args.out:
        with open(args.out, "w", encoding="utf-8") as fp:
            fp.write("\n".join(rows))

    amps, cts, closed, settled = [], [], [], []
    for line in rows:
        d = {}
        for p in line.split():
            if "=" in p:
                k, v = p.split("=", 1)
                d[k.lstrip("#")] = v
        if "#F=" in line:
            if "A" in d:
                try:
                    amps.append(float(d["A"].rstrip("(x")))
                except ValueError:
                    pass
            if "ct" in d:
                try:
                    cts.append(float(d["ct"].rstrip("(x")))
                except ValueError:
                    pass
        elif "#V6" in line:
            if "closed" in d:
                closed.append(int(d["closed"]))
            if "settled" in d:
                settled.append(int(d["settled"]))

    print(f"#F 帧={len(amps)}  #V6 帧={len(closed)}")
    if amps:
        print(f"A: 中位={statistics.median(amps):.0f}  最大={max(amps):.0f} mg")
    if cts:
        print(f"ct=1: {int(sum(1 for x in cts if x > 0))} / {len(cts)} 帧")
    if closed:
        print(f"closed=1: {sum(closed)} / {len(closed)} 帧")
    if settled:
        print(f"settled=1: {sum(settled)} / {len(settled)} 帧")


if __name__ == "__main__":
    main()
