#!/usr/bin/env python3
"""一次跑完六格：同时控振源(COM7)与检测模块(COM8)，按时间表打标签。

为什么要时间表而不是一条一条来：三态切换靠人手，而 `w` 采样是 1Hz，
只要相位分段足够长（>=40s），事后按 elapsed 就能切回来。
振源在脚本里按时开/关，避免"以为开着其实没开"。

用法：
  python -X utf8 session.py --sensor COM8 --source COM7 --out data\\S1

产物：
  <out>.csv       每行一个采样点，含 phase 列
  <out>.raw.txt   设备原始行，取证用
"""

import argparse
import csv
import os
import re
import time

import serial

SWEEP_RE = re.compile(
    r"#SWEEP t=(?P<t>\d+)s "
    r"F=\s*(?P<f>[0-9.]+|\?)Hz "
    r"A=\s*(?P<a>[0-9.]+|\?)mg "
    r"D=\s*(?P<d>[0-9.]+|\?)mg (?:ang=\s*(?P<ang>[0-9.]+|\?)deg )?"
    r"tr=\s*(?P<tr>[0-9.]+|\?)mg "
    r"vib=(?P<vib>\d+) ct=(?P<ct>\d+) grip=(?P<grip>\d+) "
    r"cliplim=(?P<clim>\d+) ax=(?P<ax>\d+) ok=(?P<ok>\d+)"
)

FIELDS = ["elapsed_s", "phase", "cell", "Freq_Hz", "Amp_mg", "FlatDist_mg",
          "RefSwing_mg", "vib", "contact", "gripNoObject", "clipLimited",
          "axis", "ok"]

# (起始秒, 阶段名, 格号)  —— 前 20s 是留给人看指令的时间
PLAN = [
    (20, "C1 松手 + 振源关", "C1"),
    (60, "C3 握紧无物 + 振源关", "C3"),
    (100, "C5 接触(压住振源) + 振源关", "C5"),
    (140, "C1 松手 + 振源关(回落检查)", "C1"),
    (180, "C4 握紧无物 + 振源开", "C4"),
    (220, "C6 接触(压住振源) + 振源开", "C6"),
    (260, "C2 松手 + 振源开", "C2"),
]
END_S = 300
MOTOR_ON_S = 170          # 提前 10s 起转，让转速稳定后再进 C4
MOTOR_CMD = b"h"          # 100% 占空比（实测约 178Hz）


def phase_at(elapsed):
    current = ("准备（先别动，读指令）", "-")
    for start, name, cell in PLAN:
        if elapsed >= start:
            current = (name, cell)
        else:
            break
    return current


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sensor", required=True)
    ap.add_argument("--source", required=True)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    sen = serial.Serial(args.sensor, args.baud, timeout=0.2)
    src = serial.Serial(args.source, args.baud, timeout=0.2)
    time.sleep(0.8)
    sen.reset_input_buffer()
    src.reset_input_buffer()
    src.write(b"z")                     # 确保从停转开始
    time.sleep(0.5)
    sen.write(b"0")                     # 关绘图组，避免混入非采样行
    time.sleep(0.3)
    sen.reset_input_buffer()

    rows = []
    motor_on = False
    t0 = time.time()
    next_poll = 0.0
    phase_seen = None
    raw = open(args.out + ".raw.txt", "w", encoding="utf-8", errors="replace")

    try:
        while True:
            elapsed = time.time() - t0
            if elapsed >= END_S:
                break

            name, cell = phase_at(elapsed)
            if name != phase_seen:
                phase_seen = name
                print(f"[{elapsed:6.1f}s] {name}", flush=True)

            if not motor_on and elapsed >= MOTOR_ON_S:
                src.write(MOTOR_CMD)
                motor_on = True
                print(f"[{elapsed:6.1f}s] 振源开 {MOTOR_CMD!r}", flush=True)

            if elapsed >= next_poll:
                sen.write(b"w")
                next_poll = elapsed + 1.0

            chunk = sen.read(512)
            if not chunk:
                time.sleep(0.02)
                continue
            text = chunk.decode("utf-8", "replace")
            raw.write(text)
            for line in text.splitlines():
                m = SWEEP_RE.search(line)
                if not m:
                    continue
                rows.append({
                    "elapsed_s": f"{elapsed:.1f}",
                    "phase": name,
                    "cell": cell,
                    "Freq_Hz": "" if m["f"] == "?" else m["f"],
                    "Amp_mg": "" if m["a"] == "?" else m["a"],
                    "FlatDist_mg": "" if m["d"] == "?" else m["d"],
                    "RefSwing_mg": "" if m["tr"] == "?" else m["tr"],
                    "vib": m["vib"], "contact": m["ct"],
                    "gripNoObject": m["grip"], "clipLimited": m["clim"],
                    "axis": m["ax"], "ok": m["ok"],
                })
    finally:
        try:
            src.write(b"z")
        except Exception:
            pass
        raw.close()
        sen.close()
        src.close()

    with open(args.out + ".csv", "w", newline="", encoding="utf-8-sig") as fp:
        w = csv.DictWriter(fp, fieldnames=FIELDS)
        w.writeheader()
        w.writerows(rows)
    print(f"完成：{len(rows)} 行 -> {args.out}.csv")


if __name__ == "__main__":
    main()
