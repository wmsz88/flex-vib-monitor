#!/usr/bin/env python3
"""单 session 正式多轮接触测试。

每轮顺序（与 README §7 / 现场输出文档 §5.3 对齐）：
  START 静止 -> t 采基准 -> 张开基线（源关）
  -> 空握闭合 -> 回 START -> t
  -> 夹/压 4cm 源（源关）-> 源开 -> 源关 -> 回 START -> t

与 source_contact_check.py 的区别：
  - 不每轮重开串口，避免每轮复位重新标定；
  - 每轮都采空握对照；
  - 每步等 cl/ang 条件成立后才继续；
  - 每轮前后发 S 检查 I2C错/读失败/缺样。

用法：
  python -X utf8 source_contact_rounds.py --sensor COM9 --source COM7 `
    --rounds 5 --out data\4cm_5round_20261102
"""

import argparse
import csv
import os
import re
import statistics
import time

import serial

import source_contact_check as scc


ROUND_FIELDS = ["round"] + scc.FIELDS


def add_rows(rows, writer, csv_fp, text, round_no, phase, elapsed):
    for row in scc.parse_rows(text, phase, elapsed):
        row["round"] = str(round_no)
        rows.append(row)
        writer.writerow(row)
        csv_fp.flush()


def poll(sen, raw, csv_fp, writer, rows, round_no, phase,
         seconds, prompt=None):
    if prompt:
        print(f"\n[轮{round_no}] {prompt}", flush=True)
    t_start = time.time()
    next_poll = 0.0
    while time.time() - t_start < seconds:
        elapsed = time.time() - t_start
        if elapsed >= next_poll:
            sen.write(b"w")
            next_poll += 1.0
        text = scc.read_chunk(sen)
        if not text:
            time.sleep(0.02)
            continue
        scc.log_raw(raw, text)
        add_rows(rows, writer, csv_fp, text, round_no, phase, elapsed)


def _angle(row):
    v = row.get("CloseAngle_deg", "")
    if v in ("", "?"):
        return None
    try:
        return float(v)
    except ValueError:
        return None


def wait_condition(sen, raw, csv_fp, writer, rows, round_no,
                   phase, predicate, prompt, timeout_s=30.0):
    print(f"\n[轮{round_no}] {prompt}", flush=True)
    deadline = time.time() + timeout_s
    t_start = time.time()
    next_poll = 0.0
    while time.time() < deadline:
        elapsed = time.time() - t_start
        if elapsed >= next_poll:
            sen.write(b"w")
            next_poll += 0.5
        text = scc.read_chunk(sen)
        if not text:
            time.sleep(0.02)
            continue
        scc.log_raw(raw, text)
        new = []
        for row in scc.parse_rows(text, phase, elapsed):
            row["round"] = str(round_no)
            new.append(row)
        for row in new:
            rows.append(row)
            writer.writerow(row)
            csv_fp.flush()
            if predicate(row):
                return True
    return False


def wait_start(sen, raw, csv_fp, writer, rows, round_no,
               phase, open_max, timeout_s=30.0):
    return wait_condition(
        sen, raw, csv_fp, writer, rows, round_no, phase,
        lambda r: r["closed"] == "0" and _angle(r) is not None
                  and _angle(r) <= open_max,
        f"请回到 START/张开位并保持静止（等 cl=0 且 ang<={open_max:.0f}°）...",
        timeout_s)


def wait_closed(sen, raw, csv_fp, writer, rows, round_no,
                phase, angle_min, prompt, timeout_s=30.0):
    return wait_condition(
        sen, raw, csv_fp, writer, rows, round_no, phase,
        lambda r: r["closed"] == "1" and _angle(r) is not None
                  and _angle(r) >= angle_min,
        prompt, timeout_s)


def health(sen, raw, label):
    text = scc.query_status(sen)
    scc.log_raw(raw, f"\n===== STATUS {label} =====\n" + text)

    def num(pat):
        m = re.search(pat, text)
        return int(m.group(1)) if m else None

    i2c = num(r"I2C\s*错\s*(\d+)")
    read_fail = num(r"读失败\s*(\d+)")
    gap = num(r"不连续\s*(\d+)\(缺\s*(\d+)")
    miss = None
    m = re.search(r"不连续\s*(\d+)\(缺\s*(\d+)", text)
    if m:
        gap = int(m.group(1))
        miss = int(m.group(2))
    print(f"  [HEALTH {label}] I2C错={i2c} 读失败={read_fail} "
          f"不连续={gap}(缺={miss})", flush=True)
    return {"i2c": i2c, "read_fail": read_fail,
            "gap": gap, "miss": miss}


def send_p_q(sen, raw):
    print("\n采 START 姿态基线：发 P，等基线有效 ...", flush=True)
    sen.write(b"P")
    time.sleep(2.5)
    sen.write(b"Q")
    time.sleep(0.8)
    text = sen.read(16384).decode("utf-8", "replace")
    scc.log_raw(raw, "\n===== POST P/Q =====\n" + text)
    print(text if text else "(no POST output)", flush=True)


def round_summary(rows, round_no):
    print(f"\n--- 轮 {round_no} 摘要 ---")
    for phase in ["empty_close", "object_off", "source_on", "source_off"]:
        seg = [r for r in rows
               if r["round"] == str(round_no) and r["phase"] == phase]
        if not seg:
            print(f"  {phase:12s}: 无数据")
            continue
        ct = sum(1 for r in seg if r["contact"] == "1")
        cl = sum(1 for r in seg if r["closed"] == "1")
        amps = [float(r["Amp_mg"]) for r in seg if r["Amp_mg"] != ""]
        freqs = [float(r["Freq_Hz"]) for r in seg if r["Freq_Hz"] != ""]
        a_s = f"{statistics.median(amps):7.1f}" if amps else "   n/a"
        f_s = f"{statistics.median(freqs):7.1f}" if freqs else "   n/a"
        print(f"  {phase:12s}: n={len(seg):2d} ct={ct:2d}/{len(seg):2d} "
              f"cl={cl:2d}/{len(seg):2d} A_med={a_s}mg F_med={f_s}Hz")
    off = [r for r in rows
           if r["round"] == str(round_no) and r["phase"] == "source_off"]
    exit_s = None
    for r in off:
        if r["contact"] == "0":
            exit_s = float(r["elapsed_s"])
            break
    if exit_s is not None:
        print(f"  source_off 首次 ct=0: {exit_s:.1f}s")
    else:
        print("  source_off 首次 ct=0: 未在采样窗口内出现")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sensor", required=True)
    ap.add_argument("--source", required=True)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--rounds", type=int, default=5)
    ap.add_argument("--on", type=float, default=20.0)
    ap.add_argument("--off", type=float, default=8.0)
    ap.add_argument("--start-baseline", type=float, default=3.0)
    ap.add_argument("--empty", type=float, default=5.0)
    ap.add_argument("--object-off", type=float, default=5.0)
    ap.add_argument("--round-end", type=float, default=3.0)
    ap.add_argument("--angle-min", type=float, default=60.0)
    ap.add_argument("--open-angle-max", type=float, default=10.0)
    ap.add_argument("--action-timeout", type=float, default=60.0)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    src = serial.Serial(args.source, args.baud, timeout=0.2)
    sen = serial.Serial(args.sensor, args.baud, timeout=0.2)
    time.sleep(0.8)
    src.reset_input_buffer()
    sen.reset_input_buffer()
    src.write(b"z")
    time.sleep(0.5)

    print("等待板子启动并完成标定（最长 15s，保持 START 静止）...",
          flush=True)
    deadline = time.time() + 15.0
    ready = False
    while time.time() < deadline:
        sen.write(b"w")
        time.sleep(0.5)
        if b"#SWEEP" in sen.read(4096):
            ready = True
            break
    if not ready:
        src.close()
        sen.close()
        raise SystemExit("板子 15s 内没有应答，检查 COM9 / 固件")

    sen.write(b"0")
    time.sleep(0.3)
    sen.reset_input_buffer()

    rows = []
    raw = open(args.out + ".raw.txt", "w", encoding="utf-8", errors="replace")
    csv_fp = open(args.out + ".csv", "w", newline="", encoding="utf-8-sig")
    writer = csv.DictWriter(csv_fp, fieldnames=ROUND_FIELDS)
    writer.writeheader()
    csv_fp.flush()

    try:
        health(sen, raw, "boot")
        send_p_q(sen, raw)

        for r in range(1, args.rounds + 1):
            print(f"\n================ 第 {r}/{args.rounds} 轮 ================",
                  flush=True)

            if not wait_start(sen, raw, csv_fp, writer, rows, r,
                              "start_wait_1", args.open_angle_max,
                              args.action_timeout):
                print(f"[轮{r}] 未等到 START，跳过本轮。", flush=True)
                continue

            sen.write(b"t")
            time.sleep(1.5)
            health(sen, raw, f"r{r}_start")
            poll(sen, raw, csv_fp, writer, rows, r, "start_off",
                 args.start_baseline, "START 基线（源关）...")

            if not wait_closed(sen, raw, csv_fp, writer, rows, r,
                               "empty_wait", args.angle_min,
                               "请闭合空握（不要接触物体）...",
                               args.action_timeout):
                print(f"[轮{r}] 未检测到空握闭合，跳过本轮。", flush=True)
                continue
            poll(sen, raw, csv_fp, writer, rows, r, "empty_close",
                 args.empty, "保持空握（源关）...")

            if not wait_start(sen, raw, csv_fp, writer, rows, r,
                              "return_start", args.open_angle_max,
                              args.action_timeout):
                print(f"[轮{r}] 空握后未回到 START，跳过本轮。", flush=True)
                continue
            sen.write(b"t")
            time.sleep(1.5)

            if not wait_closed(sen, raw, csv_fp, writer, rows, r,
                               "object_wait", args.angle_min,
                               "请缓慢闭合到 4cm 振源并保持不动（源关）...",
                               args.action_timeout):
                print(f"[轮{r}] 未检测到夹物闭合，跳过本轮。", flush=True)
                continue
            poll(sen, raw, csv_fp, writer, rows, r, "object_off",
                 args.object_off, "保持夹住 4cm 源（源关）...")

            print(f"\n[轮{r}] 振源开（保持夹住，不要动）...", flush=True)
            src.write(b"h")
            poll(sen, raw, csv_fp, writer, rows, r, "source_on", args.on)

            print(f"\n[轮{r}] 振源关（保持夹住，观察 ct 退出）...",
                  flush=True)
            src.write(b"z")
            poll(sen, raw, csv_fp, writer, rows, r, "source_off", args.off)

            if not wait_start(sen, raw, csv_fp, writer, rows, r,
                              "round_end_wait", args.open_angle_max,
                              args.action_timeout):
                print(f"[轮{r}] 结束时未回到 START。", flush=True)
            else:
                sen.write(b"t")
                time.sleep(1.5)
            poll(sen, raw, csv_fp, writer, rows, r, "round_end",
                 args.round_end)
            health(sen, raw, f"r{r}_end")
            round_summary(rows, r)
    finally:
        src.write(b"z")
        final = scc.query_status(sen)
        scc.log_raw(raw, "\n===== STATUS final =====\n" + final)
        csv_fp.close()
        raw.close()
        src.close()
        sen.close()

    print("\n===== STATUS final =====")
    print(final)
    print(f"\n完成：{len(rows)} 行 -> {args.out}.csv")


if __name__ == "__main__":
    main()
