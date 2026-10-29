#!/usr/bin/env python3
"""vibmon v4 实测数据采集工具。

两条采集通道，对应固件现有能力，不修改固件：

  sweep  : 每秒发一次 'w'，记录固件 #SWEEP 行
           字段 F(主频) / A(幅值) / D(到平放基准距离) / tr / vib / ct / grip / ax / ok
           带有效性标记，"测不出"会写成 '?'，不会被当成 0
           用途：给 CT_MODE=3 的六个阈值采样

  label  : 发 k0~k6 进打标模式，固件以 100Hz 输出 CSV
           字段 seq,t_ms,ctX,ctY,ctZ,ctDev,ampMg,vib,contact,cvalid,gap,label,ctTouchDist
           用途：看时序、看接触判定翻转过程
           注意：这一路**没有 freqHz / flatDist**，阈值采样请用 sweep

用法（PowerShell，注意带 -X utf8 输出中文才不乱码）：

  python -X utf8 capture_measure.py --list
  python -X utf8 capture_measure.py --port COM5 --mode sweep \
      --steps 0:60,4:60,6:60,3:30 --out run01

  python -X utf8 capture_measure.py --port COM5 --mode label \
      --steps 0:20,4:20,6:20 --out run01_ts

--steps 的格式是 "状态号:秒数"，状态号沿用固件 k0~k6 的语义：
  0 空闲（手指伸直、未接触）   1 接触中
  2 保持+振动暂停              3 释放
  4 无接触转动 / 接触但不给振动 5 周期振动
  6 接触中+振动

产物（都落在 --out 前缀上）：
  <out>.csv      解析后的表格（state 列由脚本写入）
  <out>.raw.txt  设备原始行，解析失败时的取证依据
"""

import argparse
import csv
import os
import re
import statistics
import sys
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.exit("需要 pyserial：python -m pip install pyserial")


SWEEP_RE = re.compile(
    r"#SWEEP t=(?P<t>\d+)s "
    r"F=\s*(?P<f>[0-9.]+|\?)Hz "
    r"A=\s*(?P<a>[0-9.]+|\?)mg "
    r"D=\s*(?P<d>[0-9.]+|\?)mg "
    r"(?:ang=\s*(?P<ang>[0-9.]+|\?)deg )?"
    r"tr=\s*(?P<tr>[0-9.]+|\?)mg "
    r"vib=(?P<vib>\d+) ct=(?P<ct>\d+) grip=(?P<grip>\d+) "
    r"cliplim=(?P<clim>\d+) ax=(?P<ax>\d+) ok=(?P<ok>\d+)"
)

SWEEP_HEADER = ["state", "wall_s", "t_dev_s", "Freq_Hz", "Amp_mg", "FlatDist_mg",
                "Angle_deg", "RefSwing_mg", "vib", "contact", "gripNoObject",
                "clipLimited", "axis", "ok"]

LABEL_HEADER = ["state", "seq", "t_ms", "ctX", "ctY", "ctZ", "ctDev",
                "ampMg", "vib", "contact", "cvalid", "gap", "label",
                "ctTouchDist"]

STATE_NAMES = {
    0: "空闲", 1: "接触中", 2: "保持+振动暂停", 3: "释放",
    4: "无接触转动", 5: "周期振动", 6: "接触中+振动",
}


# 与固件 sweepLine / csvLine 逐字一致的样例行，用于无硬件时验证解析链路。
# 注意 '?' 是固件的"无效"语义（区别于测得 0），必须解析成空而不是 0。
SELFTEST_SWEEP = [
    "#SWEEP t=1s F=      ?Hz A=     0.00mg D=     0mg tr=   12mg vib=0 ct=0 grip=0 cliplim=0 ax=2 ok=0",
    "#SWEEP t=2s F=  140.1Hz A=    81.9mg D=   756mg tr=   12mg vib=1 ct=1 grip=0 cliplim=0 ax=0 ok=1",
    "#SWEEP t=3s F=  140.1Hz A=    81.9mg D=   756mg tr=   12mg vib=1 ct=1 grip=0 cliplim=1 ax=0 ok=0",
]

SELFTEST_LABEL = [
    "1,1234,0.12,-0.34,981.20,1.50,40.20,1,0,1,0,6,0.00",
    "EV,t_ms=1300,label=6",
    "#打标模式：仅输出 CSV",
]


def parse_steps(text):
    steps = []
    for item in text.split(","):
        item = item.strip()
        if not item:
            continue
        if ":" not in item:
            raise argparse.ArgumentTypeError(f"步进要写成 状态:秒数，收到 {item!r}")
        label, secs = item.split(":", 1)
        label = int(label)
        secs = float(secs)
        if not 0 <= label <= 6:
            raise argparse.ArgumentTypeError(f"状态号只能是 0~6，收到 {label}")
        if secs <= 0:
            raise argparse.ArgumentTypeError("每个状态的时长必须 > 0")
        steps.append((label, secs))
    if not steps:
        raise argparse.ArgumentTypeError("--steps 不能为空")
    return steps


def cmd(text):
    return text.encode("ascii")


def fmt(value):
    return "" if value is None else f"{value:.2f}"


def collect(args):
    steps = args.steps
    ser = serial.Serial(args.port, args.baud, timeout=0.25)
    rows = []
    raw_path = args.out + ".raw.txt"
    # 输出目录可能还不存在：先建，否则打开 raw 文件就抛 FileNotFoundError
    os.makedirs(os.path.dirname(os.path.abspath(raw_path)), exist_ok=True)
    raw = open(raw_path, "w", encoding="utf-8", errors="replace")

    try:
        ser.reset_input_buffer()
        # 上电横幅 + 自动标定 3.2s + 首次平放基准采集 0.8s，留足余量
        deadline = time.time() + args.settle
        while time.time() < deadline:
            line = ser.readline()
            if line:
                raw.write(line.decode("utf-8", "replace"))
                raw.flush()
        ser.write(cmd("0"))          # 关绘图组，避免混入非采样行
        time.sleep(0.3)
        ser.reset_input_buffer()

        if args.mode == "label":
            ser.write(cmd(f"k{steps[0][0]}"))

        for index, (state, secs) in enumerate(steps):
            if args.mode == "label" and index > 0:
                ser.write(cmd(f"k{state}"))
            print(f"[{index + 1}/{len(steps)}] 状态 {state}（{STATE_NAMES[state]}）"
                  f" 采集 {secs:.0f}s", flush=True)

            step_start = time.time()
            next_cmd = step_start
            while True:
                now = time.time()
                if now - step_start >= secs:
                    break
                if args.mode == "sweep" and now >= next_cmd:
                    ser.write(cmd("w"))
                    next_cmd = now + args.rate
                line = ser.readline()
                if not line:
                    continue
                text = line.decode("utf-8", "replace").rstrip("\r\n")
                raw.write(text + "\n")
                raw.flush()
                if args.mode == "sweep":
                    row = parse_sweep(text, state, step_start)
                else:
                    row = parse_label(text, state)
                if row is not None:
                    rows.append(row)
    finally:
        if args.mode == "label":
            try:
                ser.write(cmd("K"))
            except Exception:
                pass
        raw.close()
        ser.close()
    return rows


def parse_sweep(text, state, step_start):
    m = SWEEP_RE.search(text)
    if not m:
        return None
    get = m.group
    return {
        "state": state,
        "wall_s": f"{time.time():.3f}",
        "t_dev_s": f"{time.time() - step_start:.3f}",
        "Freq_Hz": "" if get("f") == "?" else get("f"),
        "Amp_mg": "" if get("a") == "?" else get("a"),
        "FlatDist_mg": "" if get("d") == "?" else get("d"),
        "Angle_deg": "" if get("ang") in (None, "?") else get("ang"),
        "RefSwing_mg": "" if get("tr") == "?" else get("tr"),
        "vib": get("vib"),
        "contact": get("ct"),
        "gripNoObject": get("grip"),
        "clipLimited": get("clim"),
        "axis": get("ax"),
        "ok": get("ok"),
    }


def parse_label(text, state):
    if not text or text.startswith("#") or text.startswith("EV,"):
        return None
    parts = text.split(",")
    if len(parts) != 13:
        return None
    try:
        int(parts[0])
    except ValueError:
        return None
    return dict(zip(LABEL_HEADER[1:], parts), state=state)


def numbers(rows, field):
    out = []
    for row in rows:
        value = row.get(field, "")
        if value in ("", None):
            continue
        try:
            out.append(float(value))
        except ValueError:
            continue
    return out


def report(rows, mode):
    print("\n=== 汇总 ===")
    states = []
    for row in rows:
        if row["state"] not in states:
            states.append(row["state"])

    if mode == "sweep":
        fields = ["Freq_Hz", "Amp_mg", "FlatDist_mg"]
    else:
        fields = ["ctDev", "ampMg", "ctTouchDist"]

    for state in states:
        subset = [r for r in rows if r["state"] == state]
        print(f"\n状态 {state}（{STATE_NAMES[state]}）：{len(subset)} 行")
        for field in fields:
            values = numbers(subset, field)
            if not values:
                print(f"  {field:12s} 全部无效")
                continue
            valid_ratio = 100.0 * len(values) / len(subset)
            print(f"  {field:12s} n={len(values):4d} ({valid_ratio:5.1f}%) "
                  f"min={min(values):8.2f}  median={statistics.median(values):8.2f}  "
                  f"max={max(values):8.2f}")

    if mode == "sweep":
        suggest(rows)


def suggest(rows):
    """按 README 的取阈原则给一版建议值；数据不足时只说明缺什么。"""
    def span(state, field):
        values = numbers([r for r in rows if r["state"] == state], field)
        if len(values) < 5:
            return None
        return min(values), statistics.median(values), max(values)

    print("\n=== 阈值建议（仅供参考，需人工复核余量）===")
    d_idle = span(0, "FlatDist_mg")
    d_touch = span(6, "FlatDist_mg") or span(4, "FlatDist_mg")
    a_quiet = span(4, "Amp_mg")
    a_vib = span(6, "Amp_mg") or span(5, "Amp_mg")
    f_quiet = span(4, "Freq_Hz")
    f_vib = span(6, "Freq_Hz") or span(5, "Freq_Hz")

    def gap(name, low, high, on_key, off_key, ratio=0.75):
        if not low or not high:
            print(f"  {name}: 数据不足（需要状态 0/4/6 中至少两种、各 >=5 条有效值）")
            return
        lo, hi = low[2], high[0]
        if hi <= lo:
            print(f"  {name}: 两状态区间重叠（低态上限 {lo:.1f} ≥ 高态下限 {hi:.1f}），"
                  f"不能只靠这一路分开")
            return
        on = lo + (hi - lo) * ratio
        print(f"  {name}: {off_key}={on * 0.7:.1f}  {on_key}={on:.1f}   "
              f"(低态上限 {lo:.1f} / 高态下限 {hi:.1f}，余量 {hi - lo:.1f})")

    gap("姿态 D", d_idle, d_touch, "CT_FLAT_D_ON_MG", "CT_FLAT_D_OFF_MG")
    gap("幅值 A", a_quiet, a_vib, "CT_FLAT_A_ON_MG", "CT_FLAT_A_OFF_MG")
    gap("频率 F", f_quiet, f_vib, "CT_FLAT_F_ON_HZ", "CT_FLAT_F_OFF_HZ")


def selftest():
    """无硬件时验证：解析 -> 统计 -> 阈值建议 整条链路。"""
    fails = 0

    def check(ok, label, detail=""):
        nonlocal fails
        if not ok:
            fails += 1
        print(f"[{'PASS' if ok else 'FAIL'}] {label}{('  ' + detail) if detail else ''}")

    rows = []
    for state, line in zip((0, 6, 6), SELFTEST_SWEEP):
        row = parse_sweep(line, state, 0.0)
        check(row is not None, "sweep 行可解析", line[:40])
        if row:
            rows.append(row)

    check(rows[0]["Freq_Hz"] == "", "无效 F 解析为空（不是 0）", repr(rows[0]["Freq_Hz"]))
    check(rows[0]["Amp_mg"] == "0.00", "A 为 0 时保留数值 0", repr(rows[0]["Amp_mg"]))
    check(rows[1]["Freq_Hz"] == "140.1", "有效 F 解析正确", repr(rows[1]["Freq_Hz"]))
    check(rows[1]["contact"] == "1", "contact 解析正确", repr(rows[1]["contact"]))
    check(rows[2]["clipLimited"] == "1", "cliplim 解析正确", repr(rows[2]["clipLimited"]))
    check(rows[2]["ok"] == "0", "ok 解析正确", repr(rows[2]["ok"]))

    values = numbers(rows, "Freq_Hz")
    check(values == [140.1, 140.1], "统计跳过空值", repr(values))

    parsed = [parse_label(line, 6) for line in SELFTEST_LABEL]
    check(parsed[0] is not None and parsed[0]["ctDev"] == "1.50", "label CSV 可解析")
    check(parsed[1] is None, "EV 事件行被跳过")
    check(parsed[2] is None, "注释行被跳过")

    print("\n--- 统计输出（样例行，非实测）---")
    report(rows, "sweep")

    print(f"\n{'全部通过' if fails == 0 else str(fails) + ' 项失败'}")
    return 1 if fails else 0


def main():
    parser = argparse.ArgumentParser(description="vibmon v4 实测采集")
    parser.add_argument("--list", action="store_true", help="列出串口后退出")
    parser.add_argument("--selftest", action="store_true", help="无硬件时验证解析与统计链路")
    parser.add_argument("--port", help="串口号，例如 COM5")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--mode", choices=["sweep", "label"], default="sweep")
    parser.add_argument("--steps", type=parse_steps, default=parse_steps("0:60,4:60,6:60,3:30"))
    parser.add_argument("--rate", type=float, default=1.0, help="sweep 模式发 w 的间隔（秒）")
    parser.add_argument("--settle", type=float, default=8.0, help="开串口后等待设备就绪（秒）")
    parser.add_argument("--out", default="measure", help="输出文件前缀")
    args = parser.parse_args()

    if args.selftest:
        return selftest()

    if args.list:
        ports = list(list_ports.comports())
        if not ports:
            print("没有发现串口。检查 USB 线、驱动，或换一根能传数据的线。")
        for port in ports:
            print(f"{port.device:8s} {port.description}")
        return

    if not args.port:
        parser.error("需要 --port（或用 --list 查看）")

    rows = collect(args)
    if not rows:
        print("\n没有解析到任何数据行。检查：波特率、是否已烧 vibmon_v4、"
              "串口是否被串口监视器占用。原始行见 " + args.out + ".raw.txt")
        return

    path = args.out + ".csv"
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    header = SWEEP_HEADER if args.mode == "sweep" else LABEL_HEADER
    with open(path, "w", newline="", encoding="utf-8-sig") as fp:
        writer = csv.DictWriter(fp, fieldnames=header)
        writer.writeheader()
        writer.writerows(rows)

    report(rows, args.mode)
    print(f"\n解析数据：{path}\n原始数据：{args.out}.raw.txt")


if __name__ == "__main__":
    main()
