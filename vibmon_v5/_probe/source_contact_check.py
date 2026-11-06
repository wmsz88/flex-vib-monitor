#!/usr/bin/env python3
"""单轮接触检查：张开 -> 缓慢闭合 -> 振源开 -> 振源关 -> 松开。

与 session.py 的区别：
  1. 只跑一轮，专门验证当前物体/振源是否真的耦合上；
  2. 会先等闭合角恢复 [V]，避免陀螺超量程锁存时白采；
  3. 会等检测到 cl=1 且 ang>=60 后才开振源；
  4. 每段保存原始行，并打印 ct/A/F 统计。

用法：
  python -X utf8 source_contact_check.py --sensor COM9 --source COM7 `
    --out data\4cm_source_check_20261102
"""

import argparse
import csv
import os
import re
import statistics
import time

import serial

SWEEP_RE = re.compile(
    r"#SWEEP t=(?P<t>\d+)s "
    r"F=\s*(?P<f>[0-9.]+|\?)Hz "
    r"A=\s*(?P<a>[0-9.]+|\?)mg "
    r"D=\s*(?P<d>[0-9.]+|\?)mg "
    r"(?:ang=\s*(?P<ang>[0-9.]+|\?)deg )?"
    r"tr=\s*(?P<tr>[0-9.]+|\?)mg "
    r"vib=(?P<vib>\d+) ct=(?P<ct>\d+) grip=(?P<grip>\d+) "
    r"cl=(?P<cl>\d+) w=(?P<w>[0-9.]+|\?) "
    r"cliplim=(?P<clim>\d+) ax=(?P<ax>\d+) ok=(?P<ok>\d+)"
)

FIELDS = [
    "elapsed_s", "phase", "Freq_Hz", "Amp_mg", "FlatDist_mg",
    "CloseAngle_deg", "RefSwing_mg", "vib", "contact", "gripNoObject",
    "closed", "gyroW", "clipLimited", "axis", "ok",
]

ANGLE_OPEN_RE = re.compile(r"闭合角\s+([0-9.]+)°.*\[V\]")


def parse_rows(text, phase, elapsed):
    rows = []
    for line in text.splitlines():
        m = SWEEP_RE.search(line)
        if not m:
            continue
        rows.append({
            "elapsed_s": f"{elapsed:.1f}",
            "phase": phase,
            "Freq_Hz": "" if m["f"] == "?" else m["f"],
            "Amp_mg": "" if m["a"] == "?" else m["a"],
            "FlatDist_mg": "" if m["d"] == "?" else m["d"],
            "CloseAngle_deg": "" if m["ang"] in (None, "?") else m["ang"],
            "RefSwing_mg": "" if m["tr"] == "?" else m["tr"],
            "vib": m["vib"],
            "contact": m["ct"],
            "gripNoObject": m["grip"],
            "closed": m["cl"],
            "gyroW": "" if m["w"] == "?" else m["w"],
            "clipLimited": m["clim"],
            "axis": m["ax"],
            "ok": m["ok"],
        })
    return rows


def read_chunk(sen):
    chunk = sen.read(4096)
    return chunk.decode("utf-8", "replace") if chunk else ""


def log_raw(raw_fp, text):
    if text:
        raw_fp.write(text)
        raw_fp.flush()


def query_status(sen):
    sen.reset_input_buffer()
    sen.write(b"S")
    time.sleep(0.8)
    return sen.read(16384).decode("utf-8", "replace")


def status_angle_line(text):
    for line in text.splitlines():
        if "闭合角" in line:
            return line.strip()
    return ""


def wait_open_ready(sen, raw_fp, angle_max, timeout_s=30.0):
    print(f"\n请回到张开/START 位并保持静止，"
          f"等待闭合角回到 <= {angle_max:.0f}° 且显示 [V] ...", flush=True)
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        text = query_status(sen)
        log_raw(raw_fp, "\n===== STATUS wait_open =====\n" + text)
        line = status_angle_line(text)
        print(f"  {line if line else '(无闭合角行)'}", flush=True)
        m = ANGLE_OPEN_RE.search(text)
        if m:
            try:
                if float(m.group(1)) <= angle_max:
                    return True
            except ValueError:
                pass
        time.sleep(0.5)
    return False


def poll_phase(sen, raw_fp, csv_fp, writer, rows,
               phase, seconds, prompt=None):
    if prompt:
        print(f"\n{prompt}", flush=True)
    t_start = time.time()
    t_end = t_start + seconds
    next_poll = 0.0
    while time.time() < t_end:
        elapsed = time.time() - t_start
        if elapsed >= next_poll:
            sen.write(b"w")
            next_poll += 1.0
        text = read_chunk(sen)
        if not text:
            time.sleep(0.02)
            continue
        log_raw(raw_fp, text)
        for row in parse_rows(text, phase, elapsed):
            rows.append(row)
            writer.writerow(row)
            csv_fp.flush()


def wait_closed(sen, raw_fp, csv_fp, writer, rows, angle_min, timeout_s=30.0):
    print(f"\n请缓慢闭合到 4cm 振源，保持不动（振源关）；"
          f"脚本等到 cl=1 且 ang>={angle_min:.0f}° 才继续 ...", flush=True)
    deadline = time.time() + timeout_s
    t_start = time.time()
    next_poll = 0.0
    while time.time() < deadline:
        elapsed = time.time() - t_start
        if elapsed >= next_poll:
            sen.write(b"w")
            next_poll += 0.5
        text = read_chunk(sen)
        if not text:
            time.sleep(0.02)
            continue
        log_raw(raw_fp, text)
        for row in parse_rows(text, "closed_wait", elapsed):
            rows.append(row)
            writer.writerow(row)
            csv_fp.flush()
            if row["closed"] == "1" and row["CloseAngle_deg"] not in ("", "?"):
                try:
                    if float(row["CloseAngle_deg"]) >= angle_min:
                        print(f"  检测到闭合：ang={row['CloseAngle_deg']}° "
                              f"cl=1 w={row['gyroW']}dps", flush=True)
                        return True
                except ValueError:
                    pass
    return False


def summarize(rows, phases):
    print("\n=== 分段统计 ===")
    for name in phases:
        seg = [r for r in rows if r["phase"] == name]
        if not seg:
            print(f"{name:12s}: 无数据")
            continue
        amps = [float(r["Amp_mg"]) for r in seg if r["Amp_mg"] != ""]
        freqs = [float(r["Freq_Hz"]) for r in seg if r["Freq_Hz"] != ""]
        angs = [float(r["CloseAngle_deg"]) for r in seg
                if r["CloseAngle_deg"] not in ("", "?")]
        cts = [int(r["contact"]) for r in seg]
        closed = [int(r["closed"]) for r in seg]
        amp_s = f"{statistics.median(amps):7.1f}" if amps else "   n/a"
        freq_s = f"{statistics.median(freqs):7.1f}" if freqs else "   n/a"
        ang_s = f"{statistics.median(angs):5.1f}" if angs else "  n/a"
        print(f"{name:12s}: n={len(seg):2d} ct={sum(cts):2d}/{len(cts):2d}"
              f" cl={sum(closed):2d}/{len(closed):2d} ang_med={ang_s}°"
              f" A_med={amp_s}mg F_med={freq_s}Hz")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sensor", required=True)
    ap.add_argument("--source", required=True)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--open-baseline", type=float, default=5.0)
    ap.add_argument("--closed-baseline", type=float, default=5.0)
    ap.add_argument("--on", type=float, default=15.0)
    ap.add_argument("--off", type=float, default=8.0)
    ap.add_argument("--open-after", type=float, default=5.0)
    ap.add_argument("--angle-min", type=float, default=60.0)
    ap.add_argument("--open-angle-max", type=float, default=10.0)
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

    # 打开 COM 口会让 ESP32-C3 USB CDC 复位，等待标定完成。
    print("等待板子启动并完成标定（最长 15s，期间保持张开、保持静止）...",
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
    writer = csv.DictWriter(csv_fp, fieldnames=FIELDS)
    writer.writeheader()
    csv_fp.flush()

    try:
        boot = query_status(sen)
        log_raw(raw, "\n===== STATUS boot =====\n" + boot)
        print("\n===== STATUS boot =====")
        print(boot, flush=True)

        if not wait_open_ready(sen, raw, args.open_angle_max):
            print("\n30s 内没有等到“闭合角回到 0° 附近且 [V]”，本轮不继续。"
                  "请回到张开/START 位静止 >0.5s 后重跑。", flush=True)
            return

        print("\n正在重采平放/START 基准：保持张开静止 1.5s ...", flush=True)
        sen.write(b"t")
        time.sleep(1.5)
        sen.reset_input_buffer()

        poll_phase(sen, raw, csv_fp, writer, rows,
                   "open_off", args.open_baseline,
                   "保持张开静止（振源关）...")

        if not wait_closed(sen, raw, csv_fp, writer, rows, args.angle_min):
            print("\n30s 内没有检测到闭合，本轮不继续。"
                  "请缓慢闭合到 ang>=60°，别甩手，重跑。", flush=True)
            return

        poll_phase(sen, raw, csv_fp, writer, rows,
                   "closed_off", args.closed_baseline,
                   "已闭合。保持不动，振源仍关，采闭合基线 ...")

        print("\n振源开：保持压住 4cm 振源，不要动。", flush=True)
        src.write(b"h")
        poll_phase(sen, raw, csv_fp, writer, rows,
                   "source_on", args.on)

        print("\n振源关：保持压住，观察 ct 是否在 0.4~1s 内退出。",
              flush=True)
        src.write(b"z")
        poll_phase(sen, raw, csv_fp, writer, rows,
                   "source_off", args.off)

        poll_phase(sen, raw, csv_fp, writer, rows,
                   "open_after", args.open_after,
                   "请缓慢松开回到张开位，保持静止 ...")
    finally:
        src.write(b"z")
        final = query_status(sen)
        log_raw(raw, "\n===== STATUS final =====\n" + final)
        csv_fp.close()
        raw.close()
        src.close()
        sen.close()

    summarize(rows, ["open_off", "closed_wait", "closed_off",
                     "source_on", "source_off", "open_after"])
    print("\n===== STATUS final =====")
    print(final)
    print(f"\n完成：{len(rows)} 行 -> {args.out}.csv")


if __name__ == "__main__":
    main()
