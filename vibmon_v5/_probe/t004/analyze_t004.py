#!/usr/bin/env python3
# -*- coding: utf-8 -*-
r"""T004 数据判读：读 run.csv / run.status.csv / run.markers.json，按任务卡验收标准出结论。

用法：
  python -X utf8 analyze_t004.py --dir "D:\大学\课程\工科赛\代码\vibmon_v5\data\t004_20261102\run1"
  python -X utf8 analyze_t004.py --dir <run_dir> --out summary.md

只做判读，不做任何硬件操作。输出对齐 T004「验收标准」的逐项判定 + RT004 数据草稿。
"""

import argparse
import csv
import json
import os
import statistics
import sys


def load_csv(path):
    if not os.path.exists(path):
        return []
    with open(path, encoding="utf-8-sig") as fp:
        return list(csv.DictReader(fp))


def fnum(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def median(vals):
    vals = [v for v in vals if v is not None]
    return statistics.median(vals) if vals else None


def pct(n, d):
    return f"{100.0 * n / d:.0f}%" if d else "n/a"


def by_phase(rows):
    out = {}
    for r in rows:
        out.setdefault(r["phase"], []).append(r)
    return out


def phase_is_ct1(rows):
    return sum(1 for r in rows if r.get("contact") == "1")


def count_delta(status_rows, field):
    """返回 (first, last, delta)。计数器是上电累计量，判读要看增量而非绝对值。"""
    vals = [int(s[field]) for s in status_rows if s.get(field) not in ("", None)]
    if not vals:
        return None, None, None
    return vals[0], vals[-1], vals[-1] - vals[0]


def first_ct0(rows):
    """返回 phase 内第一条 contact=0 的 elapsed_s（秒）。"""
    for r in rows:
        if r.get("contact") == "0":
            return fnum(r.get("elapsed_s"))
    return None


def analyze(run_dir):
    base = os.path.join(run_dir, "run")
    rows = load_csv(base + ".csv")
    status = load_csv(base + ".status.csv")
    markers = {}
    mp = base + ".markers.json"
    if os.path.exists(mp):
        with open(mp, encoding="utf-8") as fp:
            markers = json.load(fp)

    ph = by_phase(rows)
    res = []

    def verdict(item, ok, detail):
        res.append((item, "PASS" if ok else "FAIL", detail))

    # ---- P0 ----
    # 注意：readFail / i2cErr / 不连续 都是**上电累计**计数器（启动+标定期就会攒一笔），
    # 所以判读看"段内增量"而非绝对值。I2C 健康以 readFail/i2cErr 增量为准，
    # 不连续增量单独列出供人工判断（启动基线通常已有数百）。
    h1 = [s for s in status if s["phase"] == "p0_h1_static"]
    if len(h1) >= 2:
        rf0, rfN, rfD = count_delta(h1, "readFail")
        i0, iN, iD = count_delta(h1, "i2cErr")
        g0, gN, gD = count_delta(h1, "gaps")
        ok = rfD == 0 and iD == 0
        verdict("H1 I2C 静态 60s", ok,
                f"{len(h1)} 次 S 快照；读失败 {rf0}->{rfN}(Δ{rfD})，"
                f"I2C错 {i0}->{iN}(Δ{iD})，不连续 {g0}->{gN}(Δ{gD}，上电累计基数)")
    elif h1:
        verdict("H1 I2C 静态 60s", False, f"仅 {len(h1)} 次快照，不足以判增量")
    else:
        verdict("H1 I2C 静态 60s", False, "无 p0_h1_static 状态记录")

    h3ph = ph.get("p0_h3_bend", [])
    faults = [f for f in markers.get("faults", []) if f["phase"] == "p0_h3_bend"]
    h3 = [s for s in status if s["phase"] == "p0_h3_bend"]
    if len(h3) >= 2 or h3ph:
        rf0, rfN, rfD = count_delta(h3, "readFail")
        i0, iN, iD = count_delta(h3, "i2cErr")
        g0, gN, gD = count_delta(h3, "gaps")
        h3_bad = [s for s in h3 if s["sensor"] == "故障"]
        ok = (not faults) and (not h3_bad) and (rfD == 0) and (iD == 0)
        verdict("H3 弯折 3min", ok,
                f"FAULT={len(faults)}，S 快照 {len(h3)} 次；读失败 Δ{rfD}，"
                f"I2C错 Δ{iD}，不连续 {g0}->{gN}(Δ{gD})")
    else:
        verdict("H3 弯折 3min", False, "无 p0_h3_bend 数据")

    # ---- P1 / A5 ----
    rounds = sorted({r["round"] for r in rows if r["phase"].startswith("r") and r["round"]},
                    key=lambda x: int(x))
    round_pass = {}
    for r in rounds:
        rn = r
        a1 = ph.get(f"r{rn}_open", [])
        a2 = ph.get(f"r{rn}_empty", [])
        a3 = ph.get(f"r{rn}_contact_on", [])
        a4 = ph.get(f"r{rn}_exit_off", [])

        a1_ok = a1 and all(x["closed"] == "0" and x["contact"] == "0" for x in a1)
        a2_ok = a2 and all(x["closed"] == "1" and x["contact"] == "0" for x in a2)
        a3_rate = pct(phase_is_ct1(a3), len(a3)) if a3 else "n/a"
        a3_ok = bool(a3) and (phase_is_ct1(a3) / len(a3)) >= 0.90
        exit_s = first_ct0(a4)
        a4_ok = bool(a4) and exit_s is not None and exit_s <= 0.5
        round_pass[rn] = bool(a1_ok and a2_ok and a3_ok and a4_ok)

        verdict(f"轮{rn} A1 松手", bool(a1_ok),
                f"n={len(a1)} ct=1数={phase_is_ct1(a1)}" if a1 else "无数据")
        verdict(f"轮{rn} A2 空握", bool(a2_ok),
                f"n={len(a2)} 误报ct=1数={phase_is_ct1(a2)}" if a2 else "无数据")
        verdict(f"轮{rn} A3 接触", bool(a3_ok),
                f"n={len(a3)} ct=1占比={a3_rate}" if a3 else "无数据")
        verdict(f"轮{rn} A4 退出", bool(a4_ok),
                f"首次 ct=0 @ {exit_s:.2f}s" if exit_s is not None else "未见 ct=0")

    # ---- T1 ----
    t1 = ph.get("r1_t1_pull", [])
    t_pull = markers.get("t_pull_epoch")
    if t1:
        # 优先用「相对播报『抽』那一刻」的时间；缺 epoch 时退回段内 elapsed_s。
        ex = None
        ex_epoch = None
        if t_pull is not None:
            for r in t1:
                if r.get("contact") == "0" and fnum(r.get("epoch")) is not None:
                    ex_epoch = fnum(r["epoch"]) - float(t_pull)
                    break
        ex = ex_epoch if ex_epoch is not None else first_ct0(t1)
        if ex is not None and ex <= 0.3:
            band = "100~300ms 正常"
        elif ex is None or ex > 5.0:
            band = ">5s 未退出/规则过宽(开返工卡)"
        else:
            band = "0.3~5s 中间带（含人反应延迟，需结合波形判）"
        src = "相对『抽』" if ex_epoch is not None else "段内 elapsed（无 t_pull 记录）"
        verdict("T1 抽走物体", ex is not None and ex <= 5.0,
                f"首次 ct=0 @ {ex:.2f}s（{src}）→ {band}" if ex is not None
                else f"整段 ct 未退出 → {band}")
    else:
        verdict("T1 抽走物体", False, "无 r1_t1_pull 数据")

    # ---- A5 汇总 ----
    passed = [k for k, v in round_pass.items() if v]
    verdict("A5 五轮重复", len(passed) == len(round_pass) and len(round_pass) >= 5,
            f"{len(passed)}/{len(round_pass)} 轮全过（需 >=5 且全过）")

    # ---- 双振源 ----
    for tag, name in (("dual_orig_on", "B-orig 原振子"),
                      ("dual_new_on", "B-new 新低速振子")):
        seg = ph.get(tag, [])
        if seg:
            rate = pct(phase_is_ct1(seg), len(seg))
            fmed = median([fnum(x["Freq_Hz"]) for x in seg])
            amed = median([fnum(x["Amp_mg"]) for x in seg])
            verdict(name, bool(seg) and (phase_is_ct1(seg) / len(seg)) >= 0.90,
                    f"n={len(seg)} ct=1占比={rate} F中位={fmed:.1f}Hz "
                    f"A中位={amed:.1f}mg" if fmed is not None else f"n={len(seg)}")
        else:
            verdict(name, False, "无数据")

    # ---- dump ----
    dump_txt = base + ".dump.txt"
    dump_ok = os.path.exists(dump_txt) and "#DUMP," in open(
        dump_txt, encoding="utf-8", errors="replace").read()
    verdict("D 命令实板样例", dump_ok, "已抓取 #DUMP 行" if dump_ok else "未抓到 #DUMP")

    return rows, status, markers, res


def emit_report(run_dir, rows, status, markers, res, out=None):
    lines = []
    lines.append(f"# T004 判读报告 — {os.path.basename(os.path.abspath(run_dir))}")
    lines.append("")
    lines.append("## 逐项判定")
    lines.append("")
    lines.append("| 检验项 | 判定 | 依据 |")
    lines.append("|---|---|---|")
    for item, v, detail in res:
        mark = "✅" if v == "PASS" else "❌"
        lines.append(f"| {item} | {mark} {v} | {detail} |")
    lines.append("")
    npass = sum(1 for _, v, _ in res if v == "PASS")
    lines.append(f"**合计**：{npass}/{len(res)} 项 PASS")
    lines.append("")
    lines.append("## FAULT 记录")
    lines.append("")
    faults = markers.get("faults", [])
    if faults:
        for f in faults:
            lines.append(f"- {f['wall']} [{f.get('phase')}] {f.get('line')}")
    else:
        lines.append("- 无")
    lines.append("")
    lines.append("## 数据规模")
    lines.append("")
    lines.append(f"- 采样行 run.csv：{len(rows)}")
    lines.append(f"- 状态快照 run.status.csv：{len(status)}")
    lines.append("")
    text = "\n".join(lines)
    print(text)
    if out:
        with open(out, "w", encoding="utf-8") as fp:
            fp.write(text)
        print(f"\n报告已写入：{out}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", required=True, help="run_t004.py 的 --out-dir")
    ap.add_argument("--out", default="", help="可选：报告写入文件")
    args = ap.parse_args()
    if not os.path.isdir(args.dir):
        raise SystemExit(f"目录不存在：{args.dir}")
    rows, status, markers, res = analyze(args.dir)
    if not rows and not status:
        raise SystemExit("没有找到 run.csv / run.status.csv，确认 --dir 指向 run_t004 的输出目录")
    emit_report(args.dir, rows, status, markers, res, args.out or None)


if __name__ == "__main__":
    main()
