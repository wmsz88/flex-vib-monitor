"""Replay 6.1/6.2: per 2s window, dominant SS value, CT ratio as logged, and
the ratio if contact were also entered whenever SS>=1.5Hz (then held 1200ms).
Posture is not in the frame; start-stop files assume the finger stays closed."""
import re, sys
from collections import Counter
pat = re.compile(r'(\d\d):(\d\d):(\d\d)\.(\d{3}) .*?\$VD,(\d{4}),(\d{5}),(\d{5}),(\d),(\d),(\d{3}),([0-9A-F]),')
for p in sys.argv[1:]:
    rows = []
    for line in open(p, encoding='utf-8', errors='ignore'):
        m = pat.search(line)
        if m:
            t = int(m[1]) * 3600 + int(m[2]) * 60 + int(m[3]) + int(m[4]) / 1000
            rows.append((t, int(m[7]), int(m[9]), int(m[10]), int(m[6]) / 10, int(m[8])))
    sim, last, cur = [], -1e9, 0
    for t, a, ct, ss, f, vib in rows:
        if ss >= 150: last = t
        # once in contact, ON bursts with valid freq (>63Hz) or amp>300 refresh the hold
        if cur and ((vib and f > 63) or a > 300): last = max(last, t)
        cur = 1 if (ct or t - last <= 1.2) else 0
        sim.append(cur)
    print(f'== {p}')
    t0, i = rows[0][0], 0
    while i < len(rows):
        j = i
        while j < len(rows) and rows[j][0] - rows[i][0] < 4.0: j += 1
        seg = rows[i:j]
        ssn = [r[3] for r in seg if r[3]]
        dom = Counter(ssn).most_common(1)[0][0] if ssn else 0
        if max(r[1] for r in seg) > 150:
            print(f'  t={seg[0][0]-t0:6.1f}s ss~{dom/100:.2f} ssValid={len(ssn)/len(seg):4.0%}'
                  f' CT_old={sum(r[2] for r in seg)/len(seg):4.0%} CT_new={sum(sim[i:j])/len(seg):4.0%}')
        i = j
