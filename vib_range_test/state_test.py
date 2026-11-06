#!/usr/bin/env python3
"""三态验收（按动作分段时间表）：脚本负责复位模块、控振源、每秒采一条 #SWEEP。

为什么要时间表：三态切换靠人手，动作分段足够长，事后按 elapsed 就能切回来，
而且能避免"以为开着了其实没开"。

关键前提：脚本一开串口，模块就会复位并重新标定 ——
**这段时间必须保持"张开(START)、未夹物、静止"**，否则闭合角参考点是错的（ang 永远 0）。

用法：
  python -X utf8 state_test.py --source COM7 --sensor COM9 --out <out.csv> --arm 20 [--duty h]
"""
import argparse
import csv
import os
import re
import sys
import time

import serial

# (起始秒, 阶段名)  —— 前面留 arm 秒给人就位
PLAN = [
    (0,  "A 张开保持(START)"),
    (12, "B 闭合夹住物体(振源关)"),
    (30, "C 保持夹住 + 振源开"),
    (50, "D 松手回 START"),
]
END_S = 62
SRC_ON_AT = 30.0     # 振源开启时刻（= 阶段 C 起点）
SRC_OFF_AT = 50.0

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
FIELDS = ["elapsed_s", "phase", "Freq_Hz", "Amp_mg", "FlatDist_mg", "ang_deg",
          "RefSwing_mg", "vib", "ct", "grip", "closed", "cliplim", "axis", "ok"]


def phase_at(elapsed):
    cur = PLAN[0][1]
    for start, name in PLAN:
        if elapsed >= start:
            cur = name
        else:
            break
    return cur


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", required=True)
    ap.add_argument("--sensor", required=True)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--out", required=True)
    ap.add_argument("--arm", type=float, default=20.0)
    ap.add_argument("--duty", default="h")     # 振源开启时用的指令
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
        print(f"复位模块并等待标定（第 {attempt} 次，最长 15s）—— 请保持张开、静止！", flush=True)
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

    sen.write(b"1")           # 绘图组=结果；保持模块一直在流式输出（#SWEEP 另有前缀，不会混）
    time.sleep(0.4)
    sen.reset_input_buffer()

    if args.arm > 0:
        print(f"就位倒计时 {args.arm:.0f}s —— 保持张开(START)、静止。", flush=True)
        time.sleep(args.arm)

    rows = []
    raw = open(args.out + ".raw.txt", "w", encoding="utf-8", errors="replace")
    t0 = time.time()
    next_poll = 0.0
    src_on = False
    try:
        while True:
            elapsed = time.time() - t0
            if elapsed >= END_S:
                break

            name = phase_at(elapsed)
            if (not src_on) and SRC_ON_AT <= elapsed < SRC_OFF_AT:
                src.reset_input_buffer()
                src.write(args.duty.encode())
                src_on = True
                print(f"[{elapsed:6.1f}s] 振源开 {args.duty!r}", flush=True)
            if src_on and elapsed >= SRC_OFF_AT:
                src.write(b"z")
                src_on = False
                print(f"[{elapsed:6.1f}s] 振源关", flush=True)

            if elapsed >= next_poll:
                print(f"[{elapsed:6.1f}s] {name}", flush=True)
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
                    "elapsed_s": f"{elapsed:.1f}", "phase": name,
                    "Freq_Hz": "" if m["f"] == "?" else m["f"],
                    "Amp_mg": "" if m["a"] == "?" else m["a"],
                    "FlatDist_mg": "" if m["d"] == "?" else m["d"],
                    "ang_deg": "" if m["ang"] in (None, "?") else m["ang"],
                    "RefSwing_mg": "" if m["tr"] == "?" else m["tr"],
                    "vib": m["vib"], "ct": m["ct"], "grip": m["grip"],
                    "closed": m["cl"], "cliplim": m["clim"],
                    "axis": m["ax"], "ok": m["ok"],
                })
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

    def med(vals):
        v = sorted(vals)
        return v[len(v) // 2] if v else 0.0

    print("\n阶段,N,ang中位,closed比例,ct比例,vib比例,A中位,A最大,F中位", flush=True)
    for start, name in PLAN:
        rs = [r for r in rows if r["phase"] == name]
        n = len(rs)
        if not n:
            continue
        angs = [float(r["ang_deg"]) for r in rs if r["ang_deg"] != ""]
        amps = [float(r["Amp_mg"]) for r in rs if r["Amp_mg"] != ""]
        fs = [float(r["Freq_Hz"]) for r in rs if r["Freq_Hz"] != ""]
        print(f"{name},{n},{med(angs):.0f},"
              f"{sum(int(r['closed']) for r in rs) / n:.0%},"
              f"{sum(int(r['ct']) for r in rs) / n:.0%},"
              f"{sum(int(r['vib']) for r in rs) / n:.0%},"
              f"{med(amps):.1f},{(max(amps) if amps else 0):.1f},"
              f"{med(fs):.1f}", flush=True)


if __name__ == "__main__":
    main()
