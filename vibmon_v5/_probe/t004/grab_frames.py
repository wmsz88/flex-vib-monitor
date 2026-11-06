#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""抓比赛帧 $VD（补 T001 遗留：实板确认默认构建下 35 字节格式）。

注意：$VD 从**比赛口 Serial1** 发出（默认引脚 TX=GPIO21 / RX=GPIO20），
不走 USB-CDC 调试口。所以需要一根 USB-TTL 接到比赛口才能抓，PC 会多出一个 COMx。

接法（默认构建、UART_SWAP=1）：
  USB-TTL 的 RX  ->  模块 GPIO21（比赛口 TX）
  USB-TTL 的 GND ->  模块 GND
  （只收不发，模块的 GPIO20/RX 可以不接）

用法：
  python -X utf8 grab_frames.py --port COM12 --count 10 --out frames.txt

产物：文本里每行一帧 + 统计（帧数、字节长度分布、逐帧 XOR 校验核对）。
"""

import argparse
import time

import serial


def split_frame(frame):
    """'...CHK\\r\\n' -> (正文不含 CHK, CHK)。失败返回 None。"""
    core = frame.rstrip("\r\n")
    if not core.startswith("$VD,"):
        return None
    parts = core.rsplit(",", 1)
    if len(parts) != 2:
        return None
    return parts[0], parts[1].upper()


def calc_chk(text):
    x = 0
    for b in text.encode("ascii", "replace"):
        x ^= b
    return f"{x:02X}"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True, help="USB-TTL 接比赛口后的 COM 口")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--count", type=int, default=10)
    ap.add_argument("--timeout", type=float, default=15.0)
    ap.add_argument("--out", default="frames.txt")
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.3)
    print(f"打开 {args.port}，等待 {args.count} 帧 $VD（最长 {args.timeout:.0f}s）...",
          flush=True)
    frames = []
    buf = ""
    t0 = time.time()
    try:
        while len(frames) < args.count and time.time() - t0 < args.timeout:
            chunk = ser.read(4096)
            if not chunk:
                continue
            buf += chunk.decode("ascii", "replace")
            while "\n" in buf:
                line, buf = buf.split("\n", 1)
                line = line.strip("\r")
                if line.startswith("$VD,"):
                    frames.append(line)
                    if len(frames) >= args.count:
                        break
    finally:
        ser.close()

    with open(args.out, "w", encoding="utf-8") as fp:
        for f in frames:
            fp.write(f + "\n")

    print(f"\n抓到 {len(frames)} 帧 -> {args.out}", flush=True)
    lens = {}
    for f in frames:
        L = len(f) + 2  # + '\r\n'
        lens[L] = lens.get(L, 0) + 1
    print(f"字节长度分布（含 CR LF）：{lens}", flush=True)
    bad = 0
    for f in frames:
        sf = split_frame(f)
        if not sf:
            bad += 1
            print(f"  无法解析：{f}", flush=True)
            continue
        text, given = sf
        got = calc_chk(text)
        flag = "OK" if got == given else "!!"
        if got != given:
            bad += 1
        print(f"  len={len(f) + 2:2d}  calc={got} given={given}  {flag}", flush=True)
    print(f"\n校验不符/无法解析：{bad}/{len(frames)}", flush=True)
    if frames:
        fields = frames[0].split(",")
        print(f"首帧字段数：{len(fields)}  （CL=0 应为 10 段；CL=1 应为 11 段）",
              flush=True)


if __name__ == "__main__":
    main()
