"""v6 进入门限标定：滑动窗内"振动证据帧"占比。
证据帧 = AMP > A 且 (EV_FREQ 时还要 FREQ > 75Hz)。
负样本（不应进入）：单测 1/2/3（移动、空握、夹不振动物体），取全文件最大占比。
正样本（应进入）：2/3Hz 启停与持续振动段，取进入时间（段起点到占比首次 ≥P）。
注意：没有陀螺数据，未做"手指静止"过滤 —— 负样本占比是上界。"""
import re
pat = re.compile(r'(\d\d):(\d\d):(\d\d)\.(\d{3}) .*?\$VD,(\d{4}),(\d{5}),(\d{5}),(\d),(\d),(\d{3}),')
B = '../../结果/T009/'
def load(fn):
    rows = []
    for l in open(B + fn, encoding='utf-8', errors='ignore'):
        m = pat.search(l)
        if m:
            t = int(m[1]) * 3600 + int(m[2]) * 60 + int(m[3]) + int(m[4]) / 1000
            rows.append((t, int(m[6]) / 10, int(m[7])))
    t0 = rows[0][0]
    return [(t - t0, f, a) for t, f, a in rows]
NEG = ['单测/1.txt', '单测/2.txt', '单测/3.txt']
POS = [('新/4.txt', 156, 185, '2Hz'), ('新/4.txt', 192, 212, '3Hz'),
       ('单测/6.1.txt', 914, 930, '2Hz'), ('单测/6.1.txt', 934, 950, '3Hz'),
       ('单测/6.2.txt', 173, 189, '2Hz'), ('单测/6.2.txt', 189, 201, '3Hz'),
       ('单测/6.2.txt', 48, 64, '3Hz')]
cache = {}
def get(fn):
    if fn not in cache: cache[fn] = load(fn)
    return cache[fn]
def fracs(rows, A, W, ev_freq):
    n = int(W / 0.02)
    ev = [1 if (a > A and (not ev_freq or f > 75)) else 0 for _, f, a in rows]
    s, out = 0, []
    for i, e in enumerate(ev):
        s += e
        if i >= n: s -= ev[i - n]
        out.append(s / n)
    return out
print('A(mg) W(s) freq | 负样本最大占比 | 正样本进入耗时(s) @P=负样本最大+10%')
for ev_freq in (False, True):
    for A in (150, 200, 250):
        for W in (0.6, 1.0, 1.5):
            neg = max(max(fracs(get(fn), A, W, ev_freq)) for fn in NEG)
            P = min(neg + 0.10, 0.95)
            res = []
            for fn, t0, t1, lab in POS:
                rows = get(fn); fr = fracs(rows, A, W, ev_freq)
                hit = next((rows[i][0] - t0 for i in range(len(rows)) if t0 <= rows[i][0] <= t1 and fr[i] >= P), None)
                res.append(f'{lab}:{hit:.1f}' if hit is not None else f'{lab}:--')
            print(f'{A:4d} {W:4.1f} {"F" if ev_freq else "-"} | {neg:4.0%} | P={P:4.0%} ' + ' '.join(res))
