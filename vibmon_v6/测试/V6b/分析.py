#!/usr/bin/env python3
"""统计 采集.py 的日志：$VD 帧、#F 调试行、#V6 调试行。

用法：
  python -X utf8 分析.py 检查A-旧2Hz.txt [--from 19:09:32.4] [--to 19:10:07.6]
"""
import argparse
import re
import statistics as st
from datetime import datetime

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
        good = [r for r in vd if r[7] != "0"]
        print("$VD:")
        print(f"  帧数={len(vd)}  ct=1 占比={sum(1 for r in vd if r[5]) / len(vd):.1%}")
        stats(vd, 2, "FREQ(Hz)")
        stats(vd, 3, "AMP(mg)")
        stats(vd, 6, "SS(Hz)")
        # 连续帧序列号的缺口
        gaps = sum(1 for a, b in zip(vd, vd[1:]) if (b[1] - a[1]) % 10000 != 1)
        print(f"  SEQ 不连续次数={gaps}")

    if len(vd) >= 4:
        body = "".join(f"{r[1]:04d},{int(r[2] * 10):05d},{r[3]:05d},{r[4]},{r[5]},{int(r[6] * 100):03d},{r[7]}"
                       for r in vd[:0])
        # 逐帧重算 CHK（正文 XOR），抽前 5 帧
        bad = 0
        for r in vd[:5]:
            body = f"{r[1]:04d},{int(round(r[2] * 10)):05d},{r[3]:05d},{r[4]},{r[5]},{int(round(r[6] * 100)):03d},{r[7]}"
            x = 0
            for ch in body:
                x ^= ord(ch)
            if f"{x:02X}" != r[8]:
                bad += 1
        print(f"  抽 5 帧重算 CHK：{5 - bad}/5 对")


if __name__ == "__main__":
    main()
