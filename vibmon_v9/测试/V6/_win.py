"""按绝对时钟 4s 窗汇总 $VD：多个文件重叠部分只算一次。用法：python _win.py 3.txt 4.txt 5.txt"""
import re, sys
from collections import Counter
pat = re.compile(r'(\d\d):(\d\d):(\d\d)\.(\d{3}) .*?\$VD,(\d{4}),(\d{5}),(\d{5}),(\d),(\d),(\d{3}),')
seen = {}
for fn in sys.argv[1:]:
    for l in open(fn, encoding='utf-8', errors='ignore'):
        m = pat.search(l)
        if m:
            t = int(m[1]) * 3600 + int(m[2]) * 60 + int(m[3]) + int(m[4]) / 1000
            seen[round(t, 2)] = (int(m[6]) / 10, int(m[7]), int(m[8]), int(m[9]), int(m[10]))
w = {}
for t in sorted(seen):
    w.setdefault(int(t // 4) * 4, []).append(seen[t])
for k in sorted(w):
    g = w[k]
    if max(r[1] for r in g) < 100 and not any(r[3] for r in g): continue
    ss = Counter(r[4] for r in g if r[4])
    f = sorted(r[0] for r in g if r[0] > 0)
    hh, mm, s = k // 3600, k % 3600 // 60, k % 60
    print(f'{hh:02d}:{mm:02d}:{s:02d} amp<={max(r[1] for r in g):4d} vib={sum(r[2] for r in g)/len(g):4.0%} '
          f'ct={sum(r[3] for r in g)/len(g):4.0%} ssV={sum(1 for r in g if r[4])/len(g):4.0%} '
          f'ss={dict(ss.most_common(2))} f~{f[len(f)//2] if f else 0:.0f}')
