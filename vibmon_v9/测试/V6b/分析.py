#!/usr/bin/env python3
"""统计 采集.py 的日志：$VD 帧、#F 调试行、#V6 调试行。

用法：
  python -X utf8 分析.py 检查A-旧2Hz.txt [--from 19:09:32.4] [--to 19:10:07.6]
"""
import argparse
import re
import statistics as st
from datetime import datetime, timedelta

RE_V6 = re.compile(r"#V6 closed=(\d) ang=(-?[\d.]+) lpW=([\d.]+) settled=(\d) moved=([\d.]+)")
RE_F = re.compile(r"#F=([\d.]+)\S* A=([\d.]+)\S* .*?vib=(\d+).*? SS=([\d.]+)\S* ct=(\d+)")
RE_VD = re.compile(r"\$VD,(\d{4}),(\d{5}),(\d{5}),(\d),(\d),(\d{3}),([0-9A-F]),([0-9A-F]{2})")


def parse(path):
    v6, fr, vd = [], [], []
    for line in open(path, encoding="utf-8", errors="replace"):
        if len(line) < 12:
            continue
        try:
            t = datetime.strptime(line[:12], "%H:%M:%S.%f")
        except ValueError:
            continue
        m = RE_V6.search(line)
        if m:
            v6.append((t, *(float(x) for x in m.groups())))
        m = RE_F.search(line)
        if m:
            fr.append((t, *(float(x) for x in m.groups())))
        m = RE_VD.search(line)
        if m:
            seq, f, a, vib, ct, ss, valid, chk = m.groups()
            vd.append((t, int(seq), int(f) / 10.0, int(a), int(vib), int(ct), int(ss) / 100.0, valid, chk))
    return v6, fr, vd


def window(rows, t0, t1):
    return [r for r in rows if (t0 is None or r[0] >= t0) and (t1 is None or r[0] <= t1)]


def stats(rows, idx, label):
    xs = [r[idx] for r in rows]
    if not xs:
        print(f"  {label}: 无数据")
        return
    print(f"  {label}: min={min(xs):g} 中位={st.median(xs):g} max={max(xs):g}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("file")
    ap.add_argument("--from", dest="t0")
    ap.add_argument("--to", dest="t1")
    args = ap.parse_args()

    def tt(x):
        return datetime.strptime(x, "%H:%M:%S.%f") if x else None

    t0, t1 = tt(args.t0), tt(args.t1)
    v6, fr, vd = parse(args.file)
    v6, fr, vd = window(v6, t0, t1), window(fr, t0, t1), window(vd, t0, t1)

    print(f"{args.file}: #V6={len(v6)}  #F={len(fr)}  $VD={len(vd)}")
    if v6:
        print("#V6:")
        stats(v6, 3, "lpW(dps)")
        stats(v6, 5, "moved(deg)")
        print(f"  settled=1 占比: {sum(1 for r in v6 if r[4]) / len(v6):.1%}")
        print(f"  closed=1  占比: {sum(1 for r in v6 if r[1]) / len(v6):.1%}")
    if fr:
        print("#F:")
        print(f"  ct=1 占比: {sum(1 for r in fr if r[5]) / len(fr):.1%}  ({sum(1 for r in fr if r[5])}/{len(fr)} 帧)")
        stats(fr, 2, "A(mg)")
        stats(fr, 1, "F(Hz)")
        stats(fr, 4, "SS(Hz)")
    if vd:
        # 采集脚本按"读到就写"打时间戳，一坨帧会同毫秒。用 20ms 帧周期把批内时间拉平，
        # 让每帧时刻≈该批最后到达时刻减去它在批内的位置。
        fixed, i, n = [], 0, len(vd)
        batches = []
        while i < n:
            j = i
            while j + 1 < n and vd[j + 1][0] == vd[i][0]:
                j += 1
            m = j - i + 1
            batches.append(m)
            for k in range(i, j + 1):
                fixed.append((vd[i][0] - timedelta(milliseconds=20 * (j - k)),) + vd[k][1:])
            i = j + 1

        print("$VD:")
        print(f"  首帧 {fixed[0][0].strftime('%H:%M:%S.%f')[:-3]}  末帧 {fixed[-1][0].strftime('%H:%M:%S.%f')[:-3]}"
              f"  时长 {(fixed[-1][0] - fixed[0][0]).total_seconds():.1f}s")
        print(f"  帧数={len(fixed)}  平均批大小={sum(batches) / len(batches):.1f}  ct=1 占比={sum(1 for r in fixed if r[5]) / len(fixed):.1%}")
        stats(fixed, 2, "FREQ(Hz)")
        stats(fixed, 3, "AMP(mg)")
        stats(fixed, 6, "SS(Hz)")
        gaps = sum(1 for a, b in zip(fixed, fixed[1:]) if (b[1] - a[1]) % 10000 != 1)
        print(f"  SEQ 不连续次数={gaps}")
        # 帧正文长度（$VD,... 到 CHK 之前）
        lens = {}
        for r in fixed[:200]:
            L = len(f"$VD,{r[1]:04d},{int(round(r[2] * 10)):05d},{r[3]:05d},{int(r[4])},{int(r[5])},{int(round(r[6] * 100)):03d},{r[7]}")
            lens[L] = lens.get(L, 0) + 1
        print(f"  正文长度分布（抽 200 帧）: {lens}")
        bad = 0
        for r in fixed[:5]:
            body = f"$VD,{r[1]:04d},{int(round(r[2] * 10)):05d},{r[3]:05d},{int(r[4])},{int(r[5])},{int(round(r[6] * 100)):03d},{r[7]}"
            x = 0
            for ch in body:
                x ^= ord(ch)
            if f"{x:02X}" != r[8]:
                bad += 1
        print(f"  抽 5 帧重算 CHK：{5 - bad}/5 对")
        print("  ct 跳变：")
        for a, b in zip(fixed, fixed[1:]):
            if a[5] != b[5]:
                print(f"    {b[0].strftime('%H:%M:%S.%f')[:-3]}  ct {int(a[5])}->{int(b[5])}"
                      f"  (AMP={b[3]:d}mg, FREQ={b[2]:g}Hz, SEQ={b[1]})")

if __name__ == "__main__":
    main()
