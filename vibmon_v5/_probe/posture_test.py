#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""两阶段姿态诊断测试：先采 START 基线，再等待操作者切到 HONGEST。"""

from __future__ import annotations

import sys
import time

import serial


def read_lines(ser: serial.Serial, seconds: float) -> list[str]:
    deadline = time.time() + seconds
    lines: list[str] = []
    while time.time() < deadline:
        line = ser.readline().decode("utf-8", "replace").strip()
        if line:
            lines.append(line)
    return lines


def main() -> int:
    port = sys.argv[1] if len(sys.argv) > 1 else "COM8"
    ser = serial.Serial(port, 115200, timeout=0.25)
    try:
        print(f"打开 {port}，等待启动/标定 8 s，请保持 START", flush=True)
        time.sleep(8.0)
        ser.reset_input_buffer()
        ser.write(b"P\n")
        print("已发送 P，等待 START 基线完成...", flush=True)
        lines = read_lines(ser, 5.0)
        for line in lines:
            if "[POST]" in line or "#P" in line:
                print(line, flush=True)
        print("BASELINE_STAGE_DONE", flush=True)
        input()  # 由外层在确认操作者切到 HONGEST 后发送换行
        print("已确认切位，等待 8 s 后读取 S", flush=True)
        time.sleep(8.0)
        ser.reset_input_buffer()
        ser.write(b"S\n")
        for line in read_lines(ser, 1.2):
            if "姿态诊断" in line or "闭合角" in line:
                print(line, flush=True)
        # HONGEST 上再次按 P，必须被 START 防误采拒绝。
        ser.reset_input_buffer()
        ser.write(b"P\n")
        print("HONGEST 上发 P，检查拒绝...", flush=True)
        for line in read_lines(ser, 1.2):
            if "P 拒绝" in line or "[POST]" in line or "姿态诊断" in line:
                print(line, flush=True)
    finally:
        ser.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
