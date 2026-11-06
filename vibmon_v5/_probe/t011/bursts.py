import re, sys, statistics as st
pat = re.compile(r'\$VD,(\d{4}),(\d{5}),(\d{5}),(\d),(\d),(\d{3}),([0-9A-F]),')
def bursts(path):
    rows = []
    for line in open(path, encoding='utf-8', errors='ignore'):
        m = pat.search(line)
        if m:
            rows.append((int(m[2]) / 10, int(m[3]), int(m[4]), int(m[5]), int(m[6])))
    out, cur = [], []
    for r in rows:
        if r[1] >= 100:
            cur.append(r)
        elif cur:
            out.append(cur); cur = []
    if cur: out.append(cur)
    return rows, out
for p in sys.argv[1:]:
    rows, bs = bursts(p)
    print(f'== {p}: frames={len(rows)} bursts(amp>=100mg)={len(bs)}')
    # histogram by burst length (frames @50Hz) and peak amp
    for lo, hi in [(1, 3), (3, 6), (6, 10), (10, 15), (15, 30), (30, 10**9)]:
        g = [b for b in bs if lo <= len(b) < hi]
        if not g: continue
        pk = [max(x[1] for x in b) for b in g]
        f = [st.median([x[0] for x in b if x[0] > 0] or [0]) for b in g]
        ct = sum(1 for b in g if any(x[3] for x in b))
        n200 = sum(1 for b in g if sum(1 for x in b if x[1] >= 200) >= 3)
        n250 = sum(1 for b in g if sum(1 for x in b if x[1] >= 250) >= 3)
        print(f'  len {lo*20}-{hi*20}ms n={len(g):3d} peak p10/p50/max={sorted(pk)[len(pk)//10]}/{int(st.median(pk))}/{max(pk)}'
              f' fmed={st.median(f):.0f} anyCT={ct} >=3fr@200mg={n200} >=3fr@250mg={n250}')
