#!/usr/bin/env python3
"""分步接触验收：每一步先播报，等人做完再采集（显式等待点）。

流程（脚本停在 wait_go，等外部放行文件才继续）：
  对每一步：
    1) 打印 "STEP n: <动作说明>  —— 做完回我 好了"
    2) 等放行文件  3) 采集并打印该步小结  4) 进入下一步

用法：
  python -X utf8 contact_steps.py --source COM7 --sensor COM9 --out <out.csv> --go-file <path>
"""
import argparse
import csv
import os
import re
import sys
import time

import serial

# (编号, 动作说明, 脚本动作, 采集秒数)
STEPS = [
    ("1", "保持张开(START)、静止不动",              None,   8.0),
    ("2", "闭合夹住物体（本步振源保持关）",          None,   8.0),
    ("3", "保持夹住；脚本自动把振源 100%→65% 逐档降", "SCAN", 0.0),
    ("4", "保持夹住；脚本关振源，看 ct 回落",        "OFF",  8.0),
    ("5", "松手回到 START",                        None,   8.0),
]
SCAN_LEVELS = [("h", 100.0)] + [("-", float(d)) for d in range(95, 64, -5)]
SCAN_DUR = 7.0     # 每档 2s 过渡 + 5s 记录

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
FIELDS = ["step", "step_name", "sub", "Freq_Hz", "Amp_mg", "FlatDist_mg", "ang_deg",
          "vib", "ct", "closed", "axis", "ok"]


def wait_go(_gofile, msg):
    """显式等待点：提示打在终端上，人做完动作按回车才继续。"""
    print(f"WAIT {msg}", flush=True)
    try:
        input(">>> 做完后按【回车】继续: ")
    except EOFError:
        # 没接终端时的兜底：直接往下走
        time.sleep(1.0)
    print("GO", flush=True)


def parse_line(line, step, name, sub, rows):
    m = SWEEP_RE.search(line)
    if not m:
        return
    rows.append({
        "step": step, "step_name": name, "sub": sub,
        "Freq_Hz": "" if m["f"] == "?" else m["f"],
        "Amp_mg": "" if m["a"] == "?" else m["a"],
        "FlatDist_mg": "" if m["d"] == "?" else m["d"],
        "ang_deg": "" if m["ang"] in (None, "?") else m["ang"],
        "vib": m["vib"], "ct": m["ct"], "closed": m["cl"],
        "axis": m["ax"], "ok": m["ok"],
    })


def med(v):
    v = sorted(v)
    return v[len(v) // 2] if v else 0.0


def summarize(rows, step):
    rs = [r for r in rows if r["step"] == step]
    n = len(rs)
    if not n:
        return "0 行"
    ang = [float(r["ang_deg"]) for r in rs if r["ang_deg"] != ""]
    A = [float(r["Amp_mg"]) for r in rs if r["Amp_mg"] != ""]
    F = [float(r["Freq_Hz"]) for r in rs if r["Freq_Hz"] != ""]
    return (f"N={n} ang中位={med(ang):.0f}deg A中位={med(A):.1f}mg "
            f"A最大={(max(A) if A else 0):.1f} F中位={med(F):.1f}Hz "
            f"ct={sum(int(r['ct']) for r in rs) / n:.0%} "
            f"vib={sum(int(r['vib']) for r in rs) / n:.0%}")


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
    sen.write(b"1")           # 保持绘图输出
    time.sleep(0.3)
    sen.reset_input_buffer()
    print("模块就绪。", flush=True)

    rows = []
    raw = open(args.out + ".raw.txt", "w", encoding="utf-8", errors="replace")

    def poll_capture(seconds, step, name, sub="", src_switch=None):
        t0 = time.time()
        nxt = 0.0
        while time.time() - t0 < seconds:
            el = time.time() - t0
            if src_switch is not None and el >= 0 and src_switch != "__done__":
                src.reset_input_buffer()
                src.write(src_switch.encode())
                src_switch = "__done__"
            if el >= nxt:
                sen.write(b"w")
                nxt = el + 1.0
            chunk = sen.read(512)
            if not chunk:
                time.sleep(0.02)
                continue
            text = chunk.decode("utf-8", "replace")
            raw.write(text)
            for line in text.splitlines():
                parse_line(line, step, name, sub, rows)

    try:
        for step, name, action, secs in STEPS:
            print(f"STEP {step}: {name}   —— 做完回我 \"好了\"", flush=True)
            wait_go(args.go_file, f"step {step} 等待动作完成")
            print(f"--- STEP {step} 开始采集 ---", flush=True)
            if action == "OFF":
                src.write(b"z")
                poll_capture(secs, step, name)
            elif action == "SCAN":
                for i, (cmd, duty) in enumerate(SCAN_LEVELS):
                    src.reset_input_buffer()
                    src.write(cmd.encode())
                    print(f"    档位 {i + 1}/{len(SCAN_LEVELS)}  {cmd!r} = {duty:.0f}%", flush=True)
                    poll_capture(SCAN_DUR, step, name, f"{duty:.0f}%")
            else:
                poll_capture(secs, step, name)
            print(f"--- STEP {step} 小结: {summarize(rows, step)} ---", flush=True)
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

    print("\n分步小结", flush=True)
    for step, name, action, secs in STEPS:
        print(f"  {step} {name}: {summarize(rows, step)}", flush=True)


if __name__ == "__main__":
    main()
