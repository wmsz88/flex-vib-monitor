#!/usr/bin/env python3
# -*- coding: utf-8 -*-
r"""T008 平台模拟：USB-TTL 从比赛口(GPIO21)收 $VD 帧，逐帧校验 + 统计。

波特率以 config.h 的 COMP_BAUD 为准（当前 = 115200）。默认帧 35 字节；CL=1 构建 37 字节。

用法：
  python -X utf8 platform_sim.py --port COM12 --secs 60 --out "..\..\data\t008_20261102\p1_default"
分段：p0_link(10s) / p1_default(60s) / p2_cl(30s) / p3_scene(30s)
产物：<out>.frames.txt(原始帧) / <out>.csv(解析) / <out>.summary.txt(统计)
"""
import argparse, csv, os, time
from collections import Counter
import serial

FIELDS_CL0 = ["$VD","SEQ","FREQx10","AMP_MG","VIB","CT","SSx100","VALID","CHK"]
FIELDS_CL1 = ["$VD","SEQ","FREQx10","AMP_MG","VIB","CT","CL","SSx100","VALID","CHK"]

def xor_chk(text):
    x = 0
    for b in text.encode("ascii","replace"):
        x ^= b
    return f"{x:02X}"

def parse(frame):
    core = frame.rstrip("\r\n")
    if not core.startswith("$VD,"):
        return None
    body, chk = core.rsplit(",",1)
    calc = xor_chk(body)
    parts = core.split(",")
    return {"body": body, "chk_given": chk.upper(), "chk_calc": calc,
            "chk_ok": calc == chk.upper(), "nfields": len(parts), "parts": parts}

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=115200, help="以 config.h COMP_BAUD 为准")
    ap.add_argument("--secs", type=float, default=60.0)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()

    outdir = os.path.dirname(os.path.abspath(a.out))
    if outdir: os.makedirs(outdir, exist_ok=True)
    ser = serial.Serial(a.port, a.baud, timeout=0.3)
    frames_fp = open(a.out + ".frames.txt", "w", encoding="utf-8")
    csv_fp = open(a.out + ".csv", "w", newline="", encoding="utf-8-sig")
    w = csv.writer(csv_fp); w.writerow(["wall","epoch","bytes","nfields","chk_given","chk_calc","chk_ok"]+FIELDS_CL1[1:])

    n = ok = 0; lens = Counter(); nf = Counter(); chk_bad = 0
    buf = ""; t0 = time.time(); first = None
    print(f"打开 {a.port}@{a.baud}，收 {a.secs:.0f}s ...", flush=True)
    try:
        while time.time() - t0 < a.secs:
            c = ser.read(4096)
            if not c:
                continue
            if first is None: first = time.time()
            buf += c.decode("ascii","replace")
            while "\n" in buf:
                line, buf = buf.split("\n",1)
                if not line.startswith("$VD,"): continue
                frame = line.rstrip("\r")     # 去掉行尾 CR，帧文本不含 CRLF
                n += 1
                L = len(frame) + 2  # + \r\n
                lens[L] += 1
                p = parse(frame)
                nf[p["nfields"] if p else 0] += 1
                if p and p["chk_ok"]: ok += 1
                if p and not p["chk_ok"]: chk_bad += 1
                frames_fp.write(frame + "\r\n")
                cols = [f"{time.time():.3f}"]
                w.writerow([time.strftime("%H:%M:%S"), cols[0], L, p["nfields"] if p else "",
                            p["chk_given"] if p else "", p["chk_calc"] if p else "",
                            p["chk_ok"] if p else ""] + (p["parts"][1:] if p else []))
    finally:
        frames_fp.close(); csv_fp.close(); ser.close()

    dur = (time.time()-t0)
    rate = n/ (dur - (first - t0)) if first and (dur-(first-t0))>0 else 0
    lines = [
        f"port={a.port} baud={a.baud} secs={dur:.1f}",
        f"总帧数={n}  校验OK={ok}  校验错={chk_bad}",
        f"帧率≈{rate:.1f} Hz  （理论 50Hz，COMP_PERIOD_MS=20）",
        f"帧长分布(含CRLF)={dict(lens)}",
        f"域数分布={dict(nf)}",
    ]
    txt = "\n".join(lines)
    print("\n"+txt, flush=True)
    with open(a.out + ".summary.txt","w",encoding="utf-8") as f: f.write(txt+"\n")

if __name__ == "__main__":
    main()
