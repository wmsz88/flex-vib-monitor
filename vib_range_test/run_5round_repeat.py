#!/usr/bin/env python3
"""5 轮接触重复性测试（单 session，语音提示）。

每轮：
  START 静止 -> t 采基准 -> 闭合夹住 4cm 方块（源关）
  -> 源开 20s -> 源关 8s -> 松开回 START

脚本直接占用 COM9，发送 w/t/S 并记录 #SWEEP；另占 COM7 控源。
"""

import argparse
import csv
import os
import statistics
import sys
import time

import serial

BASE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, BASE)
sys.path.insert(0, r"D:\大学\课程\工科赛\代码\vibmon_v5\_probe")

import run_contact_sweep as rcs
import source_contact_check as scc

FIELDS = ["round"] + scc.FIELDS


def angle_of(row):
    v = row.get("CloseAngle_deg", "")
    if v in ("", "?"):
        return None
    try:
        return float(v)
    except ValueError:
        return None


def is_open(row):
    a = angle_of(row)
    return row.get("closed") == "0" and a is not None and a <= 10.0


def is_closed(row):
    a = angle_of(row)
    return row.get("closed") == "1" and a is not None and a >= 60.0


def rows_from(text, round_no, phase, elapsed):
    out = []
    for row in scc.parse_rows(text, phase, elapsed):
        row["round"] = str(round_no)
        out.append(row)
    return out


def poll(sen, raw, writer, csv_fp, rows, round_no, phase, seconds):
    t0 = time.time()
    nxt = 0.0
    while time.time() - t0 < seconds:
        elapsed = time.time() - t0
        if elapsed >= nxt:
            sen.write(b"w")
            nxt += 1.0
        text = scc.read_chunk(sen)
        if not text:
            time.sleep(0.02)
            continue
        scc.log_raw(raw, text)
        for row in rows_from(text, round_no, phase, elapsed):
            rows.append(row)
            writer.writerow(row)
            csv_fp.flush()


def wait_for(sen, raw, writer, csv_fp, rows, round_no, phase,
             predicate, timeout_s, prompt):
    print(f"\n[轮{round_no}] {prompt}", flush=True)
    deadline = time.time() + timeout_s
    t0 = time.time()
    nxt = 0.0
    while time.time() < deadline:
        elapsed = time.time() - t0
        if elapsed >= nxt:
            sen.write(b"w")
            nxt += 0.5
        text = scc.read_chunk(sen)
        if not text:
            time.sleep(0.02)
            continue
        scc.log_raw(raw, text)
        for row in rows_from(text, round_no, phase, elapsed):
            rows.append(row)
            writer.writerow(row)
            csv_fp.flush()
            if predicate(row):
                return True
    return False


def summarize(rows, rounds):
    print("\n=== 5 轮汇总 ===", flush=True)
    print("轮,源开n,源开ct,源开cl,A中位,F中位,关源n,关源ct,关源cl,首次ct0(s)",
          flush=True)
    for r in range(1, rounds + 1):
        on = [x for x in rows if x["round"] == str(r) and x["phase"] == "source_on"]
        off = [x for x in rows if x["round"] == str(r) and x["phase"] == "source_off"]
        on_ct = sum(1 for x in on if x["contact"] == "1")
        on_cl = sum(1 for x in on if x["closed"] == "1")
        off_ct = sum(1 for x in off if x["contact"] == "1")
        off_cl = sum(1 for x in off if x["closed"] == "1")
        amps = [float(x["Amp_mg"]) for x in on if x["Amp_mg"]]
        freqs = [float(x["Freq_Hz"]) for x in on if x["Freq_Hz"]]
        a_med = statistics.median(amps) if amps else 0.0
        f_med = statistics.median(freqs) if freqs else 0.0
        first0 = None
        for x in off:
            if x["contact"] == "0":
                first0 = float(x["elapsed_s"])
                break
        print(f"{r},{len(on)},{on_ct},{on_cl},{a_med:.1f},{f_med:.1f},"
              f"{len(off)},{off_ct},{off_cl},"
              f"{'' if first0 is None else f'{first0:.1f}'}", flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sensor", default="COM9")
    ap.add_argument("--source", default="COM7")
    ap.add_argument("--rounds", type=int, default=5)
    ap.add_argument("--on", type=float, default=20.0)
    ap.add_argument("--off", type=float, default=8.0)
    ap.add_argument("--closed-off", type=float, default=5.0)
    ap.add_argument("--out", default=os.path.join(
        BASE, "data", "5round_repeat_20261102.csv"))
    args = ap.parse_args()

    live = os.path.abspath(args.out)
    os.makedirs(os.path.dirname(live), exist_ok=True)
    for p in (live, live + ".raw.txt"):
        try:
            os.remove(p)
        except OSError:
            pass

    rcs.speak("五轮接触重复性测试即将开始，请先保持张开静止")
    src = serial.Serial(args.source, 115200, timeout=0.3)
    sen = serial.Serial(args.sensor, 115200, timeout=0.3)
    time.sleep(0.8)
    src.reset_input_buffer()
    sen.reset_input_buffer()
    src.write(b"z")
    time.sleep(0.4)
    sen.write(b"0")
    time.sleep(0.3)
    sen.reset_input_buffer()

    rows = []
    raw = open(live + ".raw.txt", "w", encoding="utf-8", errors="replace")
    csv_fp = open(live, "w", newline="", encoding="utf-8-sig")
    writer = csv.DictWriter(csv_fp, fieldnames=FIELDS)
    writer.writeheader()
    csv_fp.flush()

    try:
        # 等第一次 #SWEEP，确认模块已完成启动/标定。
        if not wait_for(sen, raw, writer, csv_fp, rows, 0, "boot",
                        lambda r: True, 20.0,
                        "模块启动/标定中，请保持张开静止..."):
            raise SystemExit("模块 20s 内没有 #SWEEP 应答")

        for r in range(1, args.rounds + 1):
            print(f"\n================ 第 {r}/{args.rounds} 轮 ================",
                  flush=True)
            rcs.speak(f"第{r}轮，请保持张开静止")
            if not wait_for(sen, raw, writer, csv_fp, rows, r, "wait_open",
                            is_open, 60.0,
                            "请保持张开/START 静止（等 cl=0, ang<=10°）..."):
                print(f"[轮{r}] 未等到 START，停止。", flush=True)
                break
            sen.write(b"t")
            time.sleep(1.5)

            rcs.speak(f"第{r}轮，请闭合夹住四厘米方块，振源保持关闭")
            if not wait_for(sen, raw, writer, csv_fp, rows, r, "wait_closed",
                            is_closed, 60.0,
                            "请缓慢闭合夹住 4cm 方块（源关，等 cl=1, ang>=60°）..."):
                print(f"[轮{r}] 未检测到闭合，停止。", flush=True)
                break
            poll(sen, raw, writer, csv_fp, rows, r, "closed_off",
                 args.closed_off)

            rcs.speak(f"第{r}轮，振源开，保持夹住二十秒")
            src.write(b"h")
            poll(sen, raw, writer, csv_fp, rows, r, "source_on", args.on)

            rcs.speak(f"第{r}轮，振源关，保持夹住八秒")
            src.write(b"z")
            poll(sen, raw, writer, csv_fp, rows, r, "source_off", args.off)

            rcs.speak(f"第{r}轮，请松开回到张开位")
            if not wait_for(sen, raw, writer, csv_fp, rows, r, "wait_open_after",
                            is_open, 60.0,
                            "请松开回 START（等 cl=0, ang<=10°）..."):
                print(f"[轮{r}] 未回到 START，停止。", flush=True)
                break
    finally:
        src.write(b"z")
        time.sleep(0.3)
        csv_fp.close()
        raw.close()
        src.close()
        sen.close()
        print("DONE", flush=True)

    summarize(rows, args.rounds)
    print(f"\n数据：{live}", flush=True)


if __name__ == "__main__":
    main()
