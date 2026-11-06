#!/usr/bin/env python3
"""按《实板测试模式.md》跑 0~3Hz 启停测试。

振源固件提供：
  T0/T1/T2/T3  自动通断 0.5/1/2/3 Hz
  T            停止通断

流程：
  张开复位 -> 压住 4cm 源并保持 -> T0/T1/T2/T3 各 20s -> 关源
  后台用 plot_logger.py 记录 SS_Hz（含启停频率）。
"""

import argparse
import csv
import json
import os
import statistics
import subprocess
import sys
import time

import serial

import run_contact_sweep as rcs

BASE = os.path.dirname(os.path.abspath(__file__))


def wait_contact(live, timeout_s=40.0):
    """等待 live.csv 最近样本：A 中位数 > 300mg 且 ct 多数为 1。"""
    print("等待闭合夹住：A 中位数 > 300mg 且 ct=1 ...", flush=True)
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        try:
            with open(live, encoding="utf-8-sig") as fp:
                rows = list(csv.DictReader(fp))
        except OSError:
            rows = []
        recent = rows[-100:]
        amps = []
        cts = 0
        for r in recent:
            try:
                amps.append(float(r["Amp_mg"]))
            except (KeyError, ValueError):
                pass
            if r.get("Contact") == "1":
                cts += 1
        if amps:
            med = statistics.median(amps)
            ct_ratio = cts / len(recent)
            print(f"  A 中位 {med:.1f}mg  ct {ct_ratio:.0%}", flush=True)
            if med > 300.0 and ct_ratio > 0.5:
                return True
        time.sleep(1.0)
    return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sensor", default="COM9")
    ap.add_argument("--source", default="COM7")
    ap.add_argument("--hold", type=float, default=20.0)
    ap.add_argument("--out", default=os.path.join(
        BASE, "data", "startstop_live_20261102.csv"))
    args = ap.parse_args()

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

    def send(cmd):
        src.write(cmd.encode("ascii"))
        time.sleep(0.3)
        try:
            resp = src.read(1024).decode("utf-8", "replace")
        except Exception:
            resp = ""
        print(f"[SRC] {cmd} {resp.strip()}", flush=True)

    send("z")

    logger = subprocess.Popen(
        [sys.executable, "-X", "utf8", "-u",
         os.path.join(BASE, "plot_logger.py"),
         "--sensor", args.sensor, "--out", live],
        cwd=BASE)

    # 等标定完成并出现 Plot 行。
    deadline = time.time() + 25.0
    ready = False
    while time.time() < deadline:
        if os.path.exists(live):
            try:
                with open(live, encoding="utf-8-sig") as fp:
                    n = sum(1 for _ in fp)
                if n >= 3:
                    ready = True
                    break
            except OSError:
                pass
        time.sleep(0.5)
    if not ready:
        print("警告：plot_logger 25s 内没有数据。", flush=True)

    markers = []

    def mark(name):
        markers.append({"name": name, "epoch": time.time()})

    try:
        rcs.speak("基线就绪，请闭合夹住四厘米方块并保持不动。"
                  "振源将依次以零点五、一、二、三赫兹自动通断。")
        send("8")          # 先持续开 80%，等 A 上来确认已耦合
        if not wait_contact(live):
            print("40s 内没有形成 A>300mg 且 ct=1，本轮不继续。",
                  flush=True)
            return

        for cmd, hz in [("T0", 0.5), ("T1", 1.0), ("T2", 2.0), ("T3", 3.0)]:
            print(f"\n=== 启停 {hz:g}Hz（{cmd}，保持 {args.hold:g}s）===",
                  flush=True)
            mark(f"ss_{hz:g}hz")
            send(cmd)
            time.sleep(args.hold)
            send("T")
            time.sleep(3.0)
    finally:
        send("z")
        logger.terminate()
        try:
            logger.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            logger.kill()
        src.close()
        with open(markers_path, "w", encoding="utf-8") as fp:
            json.dump(markers, fp, ensure_ascii=False, indent=2)
        print("DONE", flush=True)

    rows = []
    with open(live, encoding="utf-8-sig") as fp:
        for row in csv.DictReader(fp):
            try:
                row["epoch_f"] = float(row["epoch"])
                row["ss_f"] = float(row["SS_Hz"])
            except (KeyError, ValueError):
                continue
            rows.append(row)

    print("\n=== 启停结果 ===", flush=True)
    print("设定Hz,n,ss有效数,ss中位,误差,ss_valid比例,vib比例,ct比例",
          flush=True)
    for i, m in enumerate(markers):
        expected = float(m["name"].split("_")[1].replace("hz", ""))
        start = float(m["epoch"])
        end = float(markers[i + 1]["epoch"]) if i + 1 < len(markers) else None
        seg = [r for r in rows
               if r["epoch_f"] >= start and (end is None or r["epoch_f"] < end)]
        ss = [r["ss_f"] for r in seg if r["ss_f"] > 0]
        med = statistics.median(ss) if ss else 0.0
        err = med - expected if ss else float("nan")
        vib = sum(1 for r in seg if r.get("Vib") == "1")
        ct = sum(1 for r in seg if r.get("Contact") == "1")
        print(f"{expected:5.1f},{len(seg)},{len(ss)},{med:6.2f},{err:+6.2f},"
              f"{(len(ss) / len(seg) if seg else 0):.0%},"
              f"{(vib / len(seg) if seg else 0):.0%},"
              f"{(ct / len(seg) if seg else 0):.0%}", flush=True)


if __name__ == "__main__":
    main()
