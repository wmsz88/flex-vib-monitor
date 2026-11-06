"""Per-second timeline of $VD frames: amp max, VIB ratio, CT ratio, SS values, freq median."""
import re, sys, statistics as st
from collections import Counter
pat = re.compile(r'(\d\d):(\d\d):(\d\d)\.(\d{3}) .*?\$VD,(\d{4}),(\d{5}),(\d{5}),(\d),(\d),(\d{3}),([0-9A-F]),')
for p in sys.argv[1:]:
    rows = []
    for line in open(p, encoding='utf-8', errors='ignore'):
        m = pat.search(line)
        if m:
            t = int(m[1]) * 3600 + int(m[2]) * 60 + int(m[3]) + int(m[4]) / 1000
            rows.append((t, int(m[6]) / 10, int(m[7]), int(m[8]), int(m[9]), int(m[10])))
    t0 = rows[0][0]
    print(f'== {p} frames={len(rows)} CT=1:{sum(r[4] for r in rows)} span={rows[-1][0]-t0:.0f}s')
    b = {}
    for r in rows: b.setdefault(int(r[0] - t0), []).append(r)
    for s in sorted(b):
        g = b[s]
        ss = Counter(r[5] for r in g if r[5])
        f = [r[1] for r in g if r[1] > 0]
        print(f'{s:4d}s amp<={max(r[2] for r in g):5d} vib={sum(r[3] for r in g)/len(g):4.0%} '
              f'ct={sum(r[4] for r in g)/len(g):4.0%} ss={dict(ss.most_common(2))} f~{st.median(f) if f else 0:.0f}')
