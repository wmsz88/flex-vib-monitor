#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""记录固件姿态诊断在 START/HONGEST 过渡段中的 d/class。"""

from __future__ import annotations

import csv
import re
import sys
import time
from pathlib import Path

import serial


POST_RE = re.compile(
    r"#POST cal=(?P<cal>\d+) act=(?P<act>\d+) valid=(?P<valid>\d+) "
    r"cls=(?P<cls>\d+) d=(?P<d>-?[0-9.]+) "
    r"x=(?P<x>-?[0-9.]+) y=(?P<y>-?[0-9.]+) z=(?P<z>-?[0-9.]+) "
    r"bx=(?P<bx>-?[0-9.]+) by=(?P<by>-?[0-9.]+) bz=(?P<bz>-?[0-9.]+) "
    r"clsd=(?P<clsd>\d+) stable=(?P<stable>\d+) scnt=(?P<scnt>\d+) n=(?P<n>\d+)"
)


def main() -> int:
    port = sys.argv[1] if len(sys.argv) > 1 else "COM8"
    out = Path(sys.argv[2] if len(sys.argv) > 2 else "posture_transition.csv")
    ser = serial.Serial(port, 115200, timeout=0.2)
    rows: list[list[str]] = []
    try:
        print("保持 START；等待启动 8 s 后采 P 基线", flush=True)
        time.sleep(8.0)
        ser.reset_input_buffer()
        ser.write(b"P\n")
        time.sleep(2.4)
        ser.write(b"S\n")
        time.sleep(0.8)
        baseline = ser.read_all().decode("utf-8", "replace")
        if "[POST] START 基线有效" not in baseline:
            print("P 后未得到有效 START 基线，停止", flush=True)
            return 1
        print("START 基线有效；BASELINE_STAGE_DONE", flush=True)
        input()  # 由外层确认操作者开始动作后发送换行
        print("开始采样；现在切 HONGEST 5 s，再回 START", flush=True)
        # 采样 18 s。
        t0 = time.time()
        next_sample = t0
        while time.time() - t0 < 18.0:
            now = time.time()
            if now < next_sample:
                time.sleep(0.05)
                continue
            next_sample = now + 0.8
            ser.reset_input_buffer()
            ser.write(b"Q\n")
            time.sleep(0.7)
            text = ser.read_all().decode("utf-8", "replace")
            for line in text.splitlines():
                m = POST_RE.search(line)
                if not m:
                    continue
                rows.append([
                    f"{now - t0:.2f}",
                    m.group("cal"),
                    m.group("act"),
                    m.group("valid"),
                    m.group("cls"),
                    m.group("d"),
                    m.group("x"),
                    m.group("y"),
                    m.group("z"),
                    m.group("bx"),
                    m.group("by"),
                    m.group("bz"),
                    m.group("clsd"),
                    m.group("stable"),
                    m.group("scnt"),
                    m.group("n"),
                ])
    finally:
        ser.close()

    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w", newline="", encoding="utf-8-sig") as fp:
        writer = csv.writer(fp)
        writer.writerow(["t_s", "cal", "act", "valid", "cls", "d_mg",
                         "x", "y", "z", "bx", "by", "bz",
                         "clsd", "stable", "scnt", "n"])
        writer.writerows(rows)
    print(f"记录 {len(rows)} 个姿态诊断点 -> {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
