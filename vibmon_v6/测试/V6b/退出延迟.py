#!/usr/bin/env python3
"""量「松开手 -> ct 掉 0」的延迟（T013 检查 B）。

口语提示的时间戳会带上语音合成启动延迟和人的反应（实测约 2s），不能当"松手时刻"。
所以改用 $VD 的 AMP 列找机械松开的时刻：手一松、模块不再压在振子上，AMP 会从
振动态（数百~上千 mg）塌到几百 mg 以下。取"塌陷前最后一个 AMP >= 峰值一半的帧"
作为松手时刻。

用法：python -X utf8 退出延迟.py 检查B-松手.txt 检查B-松手2.txt
"""
import importlib.util
import sys
from datetime import timedelta

spec = importlib.util.spec_from_file_location("m", "分析.py")
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)


def load(path):
    _, _, vd = m.parse(path)
    fixed, i, n = [], 0, len(vd)
    while i < n:
        j = i
        while j + 1 < n and vd[j + 1][0] == vd[i][0]:
            j += 1
        for k in range(i, j + 1):
            fixed.append((vd[i][0] - timedelta(milliseconds=20 * (j - k)),) + vd[k][1:])
        i = j + 1
    return fixed


def report(path):
    rows = load(path)
    print(f"== {path}（{len(rows)} 帧）")
    for a, b in zip(rows, rows[1:]):
        if a[5] == 1 and b[5] == 0:
            idx = rows.index(b)
            prev = [r for r in rows[max(0, idx - 100):idx]]          # 前 2s
            peak = max(r[3] for r in prev)
            half = 0.5 * peak
            rel = None
            for r in reversed(prev):
                if r[3] >= half:
                    rel = r
                    break
            if rel is None:
                print(f"  {b[0].strftime('%H:%M:%S.%f')[:-3]}  找不到松手帧")
                continue
            dt = (b[0] - rel[0]).total_seconds() * 1000
            # 再给一个"塌到 400mg 以下"的参考
            low = None
            for r in prev[prev.index(rel):]:
                if r[3] < 400:
                    low = r
                    break
            d2 = (b[0] - low[0]).total_seconds() * 1000 if low else float("nan")
            print(f"  ct 1->0 @ {b[0].strftime('%H:%M:%S.%f')[:-3]}"
                  f"  峰值 {peak}mg -> 松手帧 {rel[0].strftime('%H:%M:%S.%f')[:-3]}({rel[3]}mg)"
                  f"  => {dt:.0f}ms"
                  + (f"  （塌到<400mg 后 {d2:.0f}ms）" if low else ""))


if __name__ == "__main__":
    for p in sys.argv[1:]:
        report(p)
