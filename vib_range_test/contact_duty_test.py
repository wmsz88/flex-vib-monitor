#!/usr/bin/env python3
"""接触判据 × 振源档位：一次跑完"静止接触"和"低占空比掉档点"。

时间表（脚本自动控振源，人只做 3 个动作）：
  A  0-10s   张开保持(START)，振源关     —— 建立闭合角参考点
  B 10-16s   闭合夹住，振源关            —— "静止无振动接触"（设计上刻意不判定）
  C 16-72s   保持夹住，振源从 100% 逐档降到 65%（每档 7s）
  D 72-78s   保持夹住，振源关            —— 回落检查：ct 应回到 0
  E 78-86s   松手回 START

关键：脚本一开串口模块就复位标定，**这段时间必须保持张开**，否则 ang 参考点是错的。

用法：
  python -X utf8 contact_duty_test.py --source COM7 --sensor COM9 --out <out.csv> --arm 20
"""
import argparse
import csv
import os
import re
import sys
import time

import serial

PHASES = [
    (0.0,  "B 闭合+振源关"),
    (8.0,  "C 闭合+扫档"),
    (64.0, "D 闭合+振源关"),
    (72.0, "E 松手回START"),
]
END_S = 80.0

# 从 100% 用 '-' 逐档降到 65%（降档比升档可靠：马达一直在转）
LEVELS = [("h", 100.0)] + [("-", float(d)) for d in range(95, 64, -5)]
LEVEL_START = 8.0
LEVEL_DUR = 7.0          # 每档 2s 过渡 + 5s 记录

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
FIELDS = ["elapsed_s", "phase", "level", "Freq_Hz", "Amp_mg", "FlatDist_mg",
          "ang_deg", "vib", "ct", "closed", "cliplim", "axis", "ok"]


def phase_at(el):
    cur = PHASES[0][1]
    for start, name in PHASES:
        if el >= start:
            cur = name
        else:
            break
    return cur


def wait_go(gofile, msg):
    """显式等待点：打印 WAIT，等外部放行文件出现后继续（供人/上位脚本掐时机）。"""
    if os.path.exists(gofile):
        os.remove(gofile)
    print(f"WAIT {msg}", flush=True)
    while not os.path.exists(gofile):
        time.sleep(0.2)
    os.remove(gofile)
    print("GO", flush=True)


def level_at(el):
    if el < LEVEL_START:
        return ""
    i = int((el - LEVEL_START) // LEVEL_DUR)
    if i >= len(LEVELS):
        return ""
    return f"L{i:02d}({LEVELS[i][1]:.0f}%)"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", required=True)
    ap.add_argument("--sensor", required=True)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--out", required=True)
    ap.add_argument("--go-file", default="")
    args = ap.parse_args()

    outdir = os.path.dirname(os.path.abspath(args.out))
    if outdir:
        os.makedirs(outdir, exist_ok=True)

    src = serial.Serial(args.source, args.baud, timeout=0.2)
    src.write(b"z")
    time.sleep(0.3)

    sen = None
    ready = False
    for attempt in range(1, 4):
        if sen is not None:
            try:
                sen.close()
            except Exception:
                pass
        sen = serial.Serial(args.sensor, args.baud, timeout=0.2)
        print(f"复位模块并等待标定（第 {attempt} 次，最长 15s）—— 保持张开、静止！", flush=True)
        deadline = time.time() + 15.0
        while time.time() < deadline:
            chunk = sen.read(4096)
            if chunk and b"Freq_Hz:" in chunk:
                ready = True
                break
            time.sleep(0.05)
        if ready:
            break
        print("  没看到绘图行，重开串口重试。", flush=True)
        time.sleep(1.0)
    if not ready:
        print("错误：模块 45s 内没有输出，放弃。", flush=True)
        sen.close(); src.close()
        sys.exit(2)

    sen.write(b"1")          # 绘图组=结果；保持模块一直在流式输出，别把它关成静默
    time.sleep(0.4)
    sen.reset_input_buffer()

    gofile = args.go_file or (args.out + ".go")
    wait_go(gofile,
            "模块已复位并完成标定；请保持张开(START)。收到 GO 后立即闭合夹住被测物，"
            "并保持到本轮结束（约 70s），期间振源会自动逐档降。")

    rows = []
    raw = open(args.out + ".raw.txt", "w", encoding="utf-8", errors="replace")
    t0 = time.time()
    next_poll = 0.0
    last_idx = None        # 当前生效的档位下标（None = 振源关）
    last_data = time.time()   # 最近一次收到模块数据的时刻（断流检测）
    try:
        while True:
            el = time.time() - t0
            if el >= END_S:
                break

            # ---- 振源指令：按时间表决定该发什么 ----
            idx = None
            if LEVEL_START <= el < LEVEL_START + LEVEL_DUR * len(LEVELS):
                idx = int((el - LEVEL_START) // LEVEL_DUR)
            if idx != last_idx:
                src.reset_input_buffer()
                if idx is None:
                    src.write(b"z")
                    print(f"[{el:6.1f}s] 振源关", flush=True)
                else:
                    src.write(LEVELS[idx][0].encode())
                    print(f"[{el:6.1f}s] 振源 {LEVELS[idx][0]!r} ({LEVELS[idx][1]:.0f}%)", flush=True)
                last_idx = idx

            if el >= next_poll:
                print(f"[{el:6.1f}s] {phase_at(el)}", flush=True)
                sen.write(b"w")
                next_poll = el + 1.0

            chunk = sen.read(512)
            if not chunk:
                time.sleep(0.02)
                continue
            text = chunk.decode("utf-8", "replace")
            raw.write(text)
            last_data = time.time()
            for line in text.splitlines():
                m = SWEEP_RE.search(line)
                if not m:
                    continue
                rows.append({
                    "elapsed_s": f"{el:.1f}", "phase": phase_at(el), "level": level_at(el),
                    "Freq_Hz": "" if m["f"] == "?" else m["f"],
                    "Amp_mg": "" if m["a"] == "?" else m["a"],
                    "FlatDist_mg": "" if m["d"] == "?" else m["d"],
                    "ang_deg": "" if m["ang"] in (None, "?") else m["ang"],
                    "vib": m["vib"], "ct": m["ct"], "closed": m["cl"],
                    "cliplim": m["clim"], "axis": m["ax"], "ok": m["ok"],
                })
            if time.time() - last_data > 3.0:
                print(f"[{el:6.1f}s] !! 模块 3s 无数据（链路可能掉了）", flush=True)
                last_data = time.time()
    finally:
        try:
            src.write(b"z")
        except Exception:
            pass
        raw.close(); sen.close(); src.close()

    with open(args.out, "w", newline="", encoding="utf-8-sig") as fp:
        w = csv.DictWriter(fp, fieldnames=FIELDS)
        w.writeheader(); w.writerows(rows)
    print(f"完成：{len(rows)} 行 -> {args.out}", flush=True)

    def med(v):
        v = sorted(v)
        return v[len(v) // 2] if v else 0.0

    print("\n分组,N,ang中位,A中位,A最大,F中位,ct,vib", flush=True)
    keys = [(f"L{i:02d}({d:.0f}%)") for i, (c, d) in enumerate(LEVELS)]
    for k in keys + ["B 闭合+振源关", "D 闭合+振源关", "E 松手回START"]:
        if k.startswith("L"):
            rs = [r for r in rows if r["level"] == k]
        else:
            rs = [r for r in rows if r["phase"] == k]
        n = len(rs)
        if not n:
            print(f"{k},{0}", flush=True)
            continue
        ang = [float(r["ang_deg"]) for r in rs if r["ang_deg"] != ""]
        A = [float(r["Amp_mg"]) for r in rs if r["Amp_mg"] != ""]
        F = [float(r["Freq_Hz"]) for r in rs if r["Freq_Hz"] != ""]
        print(f"{k},{n},{med(ang):.0f},{med(A):.1f},{max(A) if A else 0:.1f},"
              f"{med(F):.1f},{sum(int(r['ct']) for r in rs) / n:.0%},"
              f"{sum(int(r['vib']) for r in rs) / n:.0%}", flush=True)


if __name__ == "__main__":
    main()
