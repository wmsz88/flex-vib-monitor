#!/usr/bin/env python3
"""按《实板测试模式.md》跑一轮"接触判据 x 振源下扫"。

流程：
  语音播报 -> 执行一条命令 -> sleep -> 当场读 live.csv 报数

组件：
  - logger.py 后台采 COM9 的 #SWEEP（开串口会复位模块，必须保持张开）
  - PowerShell System.Speech + Microsoft Huihui 播报
  - COM7 单字符控振源

用法：
  python -X utf8 -u run_contact_sweep.py
"""

import argparse
import base64
import csv
import json
import os
import statistics
import subprocess
import sys
import time

import serial

BASE = os.path.dirname(os.path.abspath(__file__))
LOGGER = os.path.join(BASE, "logger.py")


def speak(text):
    """用 PowerShell System.Speech 播报；Huihui 不存在就用默认音色。"""
    print(f"[SPEAK] {text}", flush=True)
    ps = (
        "Add-Type -AssemblyName System.Speech; "
        "$s = New-Object System.Speech.Synthesis.SpeechSynthesizer; "
        "try { $s.SelectVoice('Microsoft Huihui') } catch {}; "
        "$s.Rate = 0; "
        "$s.Speak('" + text.replace("'", "''") + "')"
    )
    enc = base64.b64encode(ps.encode("utf-16le")).decode("ascii")
    subprocess.run(["powershell", "-NoProfile", "-EncodedCommand", enc],
                   check=False)


def wait_logger_ready(live, timeout_s=30.0):
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if os.path.exists(live):
            try:
                with open(live, encoding="utf-8-sig") as fp:
                    n = sum(1 for _ in fp)
                if n >= 3:
                    return True
            except OSError:
                pass
        time.sleep(0.5)
    return False


def show_tail(live, label, n=3):
    print(f"--- {label} live tail ---", flush=True)
    if not os.path.exists(live):
        print("(no live.csv)", flush=True)
        return
    with open(live, encoding="utf-8-sig") as fp:
        lines = fp.readlines()
    for line in lines[-n:]:
        print(line.rstrip(), flush=True)


def load_rows(live):
    rows = []
    with open(live, encoding="utf-8-sig") as fp:
        for row in csv.DictReader(fp):
            try:
                row["epoch_f"] = float(row["epoch"])
            except (KeyError, ValueError):
                continue
            rows.append(row)
    return rows


def summarize(live, markers):
    rows = load_rows(live)
    print("\n=== 分段汇总 ===", flush=True)
    names = [m["name"] for m in markers]
    for i, m in enumerate(markers):
        start = float(m["epoch"])
        end = float(markers[i + 1]["epoch"]) if i + 1 < len(markers) else None
        seg = [r for r in rows
               if r["epoch_f"] >= start and (end is None or r["epoch_f"] < end)]
        if not seg:
            print(f"{m['name']:12s}: 无数据", flush=True)
            continue
        ct = sum(1 for r in seg if r.get("ct") == "1")
        closed = sum(1 for r in seg if r.get("closed") == "1")
        amps = [float(r["Amp_mg"]) for r in seg if r.get("Amp_mg")]
        freqs = [float(r["Freq_Hz"]) for r in seg if r.get("Freq_Hz")]
        a_med = f"{statistics.median(amps):7.1f}" if amps else "   n/a"
        f_med = f"{statistics.median(freqs):7.1f}" if freqs else "   n/a"
        print(f"{m['name']:12s}: n={len(seg):3d} ct={ct:3d}/{len(seg):<3d} "
              f"cl={closed:3d}/{len(seg):<3d} A_med={a_med}mg "
              f"F_med={f_med}Hz", flush=True)

    off = [m for m in markers if m["name"] == "source_off"]
    if off:
        start = float(off[0]["epoch"])
        seg = [r for r in rows if r["epoch_f"] >= start]
        first_zero = None
        for r in seg:
            if r.get("ct") == "0":
                first_zero = r["epoch_f"] - start
                break
        if first_zero is not None:
            print(f"source_off 首次 ct=0: {first_zero:.1f}s", flush=True)
        else:
            print("source_off 首次 ct=0: 未在窗口内出现", flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sensor", default="COM9")
    ap.add_argument("--source", default="COM7")
    ap.add_argument("--out", default=os.path.join(
        BASE, "data", "contact_duty_sweep_live_20261102.csv"))
    ap.add_argument("--secs", type=float, default=1800.0)
    args = ap.parse_args()

    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    live = os.path.abspath(args.out)
    markers_path = live + ".markers.json"
    for path in (live, live + ".raw.txt", markers_path):
        try:
            os.remove(path)
        except OSError:
            pass

    src = serial.Serial(args.source, 115200, timeout=0.3)
    time.sleep(0.6)
    src.reset_input_buffer()

    def send_source(cmd):
        src.write(cmd.encode("ascii"))
        time.sleep(0.3)
        try:
            resp = src.read(1024).decode("utf-8", "replace")
        except Exception:
            resp = ""
        print(f"[SRC] {cmd} {resp.strip()}", flush=True)

    send_source("z")

    markers = []

    def mark(name):
        markers.append({"name": name, "epoch": time.time()})

    logger = None
    try:
        speak("请张开保持，十二秒后我复位模块")
        time.sleep(15.0)

        mark("start_logger")
        logger = subprocess.Popen(
            [sys.executable, "-X", "utf8", "-u", LOGGER,
             "--sensor", args.sensor, "--out", live, "--secs", str(args.secs)],
            cwd=BASE)
        if not wait_logger_ready(live):
            print("警告：logger 30s 内没有写出数据行。", flush=True)
        show_tail(live, "baseline")

        speak("基线就绪，请闭合夹住四厘米方块，二十五秒")
        mark("close")
        time.sleep(25.0)
        show_tail(live, "closed")

        speak("把振源开到百分之百，保持二十秒")
        mark("duty100")
        send_source("h")
        time.sleep(20.0)
        show_tail(live, "100pct")

        speak("降到百分之八十，保持十八秒")
        mark("duty80")
        send_source("8")
        time.sleep(18.0)
        show_tail(live, "80pct")

        speak("降到百分之六十五，保持十八秒")
        mark("duty65")
        send_source("p")
        send_source("+")
        send_source("+")
        time.sleep(18.0)
        show_tail(live, "65pct")

        speak("降到百分之五十五，约八十赫兹，保持十八秒")
        mark("duty55")
        send_source("p")
        time.sleep(18.0)
        show_tail(live, "55pct")

        speak("我关振源，看接触回落，十二秒")
        mark("source_off")
        send_source("z")
        time.sleep(12.0)
        show_tail(live, "off")

        speak("可以松手了")
        mark("end")
    finally:
        send_source("z")
        if logger is not None:
            logger.terminate()
            try:
                logger.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                logger.kill()
        src.close()
        with open(markers_path, "w", encoding="utf-8") as fp:
            json.dump(markers, fp, ensure_ascii=False, indent=2)
        print("DONE", flush=True)

    summarize(live, markers)


if __name__ == "__main__":
    main()
