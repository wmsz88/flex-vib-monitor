"""启停参数回放评估。对每个构建，把实板段喂进 ss_replay，统计：
  0.5Hz 起振到首读数延迟、启停段读数覆盖率（读数在目标 ±10%）、固定/空握段的假读数秒数。
用法：python test/ss_eval.py <exe1> <exe2> ...（在 vibmon_v6 目录下运行）"""
import re, subprocess, sys
pat = re.compile(r'(\d\d):(\d\d):(\d\d)\.(\d{3}) .*?\$VD,(\d{4}),(\d{5}),(\d{5}),(\d),(\d),(\d{3}),')
V5 = '../vibmon_v5/结果/T009/'
def load(files):
    seen = {}
    for fn in files:
        for l in open(fn, encoding='utf-8', errors='ignore'):
            m = pat.search(l)
            if m:
                t = int(m[1]) * 3600 + int(m[2]) * 60 + int(m[3]) + int(m[4]) / 1000
                seen[round(t, 2)] = (int(m[7]), int(m[9]))
    return [(t,) + seen[t] for t in sorted(seen)]
H = 11 * 3600
def hm(m, s): return H + m * 60 + s
SRC = {
    'v6_45': load(['测试/V6/4.txt', '测试/V6/5.txt']),
    'v6_3':  load(['测试/V6/3.txt']),
    'v6_1':  load(['测试/V6/1.txt']),
    'u61':   load([V5 + '单测/6.1.txt']),
    'u62':   load([V5 + '单测/6.2.txt']),
    'u4':    load([V5 + '单测/4.txt']),
    'u5':    load([V5 + '单测/5.txt']),
}
def rel(src, a, b):   # 文件内相对秒
    t0 = SRC[src][0][0]; return (t0 + a, t0 + b)
# (名称, 源, 起, 止, 目标Hz)；目标 0 = 不应有读数
SEG = [
    ('新0.5Hz', 'v6_45', hm(33, 10), hm(33, 30), 0.5),
    ('新1Hz',   'v6_45', hm(33, 34), hm(33, 52), 1.0),
    ('新3Hz',   'v6_45', hm(33, 55), hm(34, 7), 3.0),
    ('旧0.5Hz', 'v6_45', hm(36, 29), hm(36, 59), 0.5),
    ('旧1Hz',   'v6_45', hm(37, 4), hm(37, 19), 1.0),
    ('旧2Hz',   'v6_45', hm(37, 22), hm(37, 46), 2.0),
    ('旧3Hz败', 'v6_45', hm(37, 50), hm(38, 2), 3.0),
    ('新2Hz',   'v6_45', hm(38, 3), hm(38, 14), 2.0),
    ('新3Hz末', 'v6_45', hm(38, 23), hm(38, 31), 3.0),
    ('6.1-2Hz', 'u61', *rel('u61', 914, 930), 2.0),
    ('6.1-3Hz', 'u61', *rel('u61', 934, 950), 3.0),
    ('6.2-2Hz', 'u62', *rel('u62', 173, 189), 2.0),
    ('6.2-3Hz', 'u62', *rel('u62', 189, 201), 3.0),
    ('固定新旧', 'v6_45', hm(30, 40), hm(32, 2), 0),
    ('固定u4',  'u4', *rel('u4', 0, 10**5), 0),
    ('固定u5',  'u5', *rel('u5', 0, 10**5), 0),
    ('空握v6',  'v6_1', *rel('v6_1', 0, 697), 0),
]
def run(exe, src):
    rows = SRC[src]; t0 = rows[0][0]
    inp = '\n'.join(f'{t - t0:.3f} {a} {c}' for t, a, c in rows)
    out = subprocess.run([exe], input=inp, capture_output=True, text=True).stdout.split('\n')
    return {round(float(x.split()[0]) + t0, 2): (int(x.split()[1]), float(x.split()[2])) for x in out if x.strip()}
res = {exe: {s: run(exe, s) for s in SRC} for exe in sys.argv[1:]}
print('段'.ljust(9) + ''.join(e.split('/')[-1].replace('.exe', '').rjust(16) for e in sys.argv[1:]))
for name, src, a, b, hz in SEG:
    line = name.ljust(9)
    for exe in sys.argv[1:]:
        r = res[exe][src]
        ts = [t for t in sorted(r) if a <= t <= b]
        if not ts: line += '无数据'.rjust(16); continue
        if hz == 0:
            bad = len({int(t) for t in ts if r[t][0]})
            line += f'假读数{bad:4d}s'.rjust(16)
        else:
            ok = [t for t in ts if r[t][0] and abs(r[t][1] - hz) <= 0.1 * hz]
            on = next((t for t in ts if SRC[src][0] and True), ts[0])
            first = f'{ok[0] - ts[0]:4.1f}s' if ok else '  --'
            line += f'{len(ok) / len(ts):4.0%} 首{first}'.rjust(16)
    print(line)
