#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""离线分析 START / HONGEST 静态姿态，并提取 FINISH 回落事件。

输入是当前固件 label 模式 CSV，至少包含：
  t_ms, ctX, ctY, ctZ, gyroW, closeAngleDeg

输出：
  *.static_runs.csv       稳定保持段及三轴均值
  *.static_summary.txt    每个 session 的类中心、距离和留一验证
  *.finish_events.csv     FINISH 回落事件特征（releaseStable 等）

本脚本只做离线分析，不修改固件。
"""

from __future__ import annotations

import argparse
import csv
import math
import statistics
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Optional


HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import event_analyze as ea


@dataclass
class StableRun:
    session: str
    run_id: int
    start_idx: int
    end_idx: int
    t_start_ms: float
    t_end_ms: float
    mean: tuple[float, float, float]
    std: tuple[float, float, float]
    label: str = "OTHER"
    dist_start: Optional[float] = None
    dist_hongest: Optional[float] = None

    @property
    def duration_ms(self) -> float:
        return self.t_end_ms - self.t_start_ms


def mean_std(values: list[float]) -> tuple[float, float]:
    return statistics.fmean(values), statistics.pstdev(values)


def find_stable_runs(
    samples: list[ea.Sample],
    window_ms: float,
    min_hold_ms: float,
    range_mg: float,
    gyro_max: float,
    max_gap_ms: float,
) -> list[tuple[int, int]]:
    """找出连续稳定保持段；返回首尾 sample index。"""
    flags: list[bool] = []
    window: list[ea.Sample] = []
    for sample in samples:
        if window and sample.t_ms - window[-1].t_ms > max_gap_ms:
            window = []
        window = [s for s in window if sample.t_ms - s.t_ms <= window_ms]
        window.append(sample)
        if len(window) < 10 or sample.gyro is None:
            flags.append(False)
            continue
        xs = [s.x for s in window if s.x is not None]
        ys = [s.y for s in window if s.y is not None]
        zs = [s.z for s in window if s.z is not None]
        if len(xs) < 10 or len(ys) < 10 or len(zs) < 10:
            flags.append(False)
            continue
        rng = max(
            max(xs) - min(xs),
            max(ys) - min(ys),
            max(zs) - min(zs),
        )
        flags.append(sample.gyro < gyro_max and rng <= range_mg)

    runs: list[tuple[int, int]] = []
    start: Optional[int] = None
    for index, ok in enumerate(flags):
        if ok and start is None:
            start = index
        if not ok and start is not None:
            if samples[index - 1].t_ms - samples[start].t_ms >= min_hold_ms:
                runs.append((start, index - 1))
            start = None
    if start is not None and samples[-1].t_ms - samples[start].t_ms >= min_hold_ms:
        runs.append((start, len(samples) - 1))
    return runs


def make_run(session: str, samples: list[ea.Sample], start: int, end: int, run_id: int) -> StableRun:
    seg = samples[start : end + 1]
    xs = [s.x for s in seg if s.x is not None]
    ys = [s.y for s in seg if s.y is not None]
    zs = [s.z for s in seg if s.z is not None]
    mx, sx = mean_std(xs)
    my, sy = mean_std(ys)
    mz, sz = mean_std(zs)
    return StableRun(
        session=session,
        run_id=run_id,
        start_idx=start,
        end_idx=end,
        t_start_ms=seg[0].t_ms,
        t_end_ms=seg[-1].t_ms,
        mean=(mx, my, mz),
        std=(sx, sy, sz),
    )


def dist(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


def classify_runs(
    runs: list[StableRun],
    start_radius_mg: float,
    start_norm_max: float,
    hongest_distance_mg: float,
) -> Optional[StableRun]:
    """用三轴范数最小的稳定段作为本 session 的 START 候选。

    如果本次采集没有真正稳定的 START，不把 FINISH 误标成 START。
    """
    def norm3(v: tuple[float, float, float]) -> float:
        return math.sqrt(sum(x * x for x in v))

    candidates = [r for r in runs if norm3(r.mean) <= start_norm_max]
    baseline = min(candidates, key=lambda r: norm3(r.mean)) if candidates else None
    start_center = baseline.mean if baseline is not None else None
    for run in runs:
        run.dist_start = None if start_center is None else dist(run.mean, start_center)
        if run.dist_start is not None and run.dist_start >= hongest_distance_mg:
            run.label = "HONGEST"
        elif run.dist_start is not None and run.dist_start <= start_radius_mg:
            run.label = "START"
        else:
            run.label = "OTHER/FINISH"
    return baseline


def centroid(runs: Iterable[StableRun]) -> Optional[tuple[float, float, float]]:
    group = list(runs)
    if not group:
        return None
    return tuple(statistics.fmean(r.mean[i] for r in group) for i in range(3))


def loo_validation(runs: list[StableRun]) -> tuple[int, int, list[tuple[str, str]]]:
    labeled = [r for r in runs if r.label in ("START", "HONGEST")]
    correct = 0
    total = 0
    mistakes: list[tuple[str, str]] = []
    for run in labeled:
        train = [r for r in labeled if r is not run and r.label == run.label]
        other_label = "HONGEST" if run.label == "START" else "START"
        other = [r for r in labeled if r.label == other_label]
        c_self = centroid(train)
        c_other = centroid(other)
        if c_self is None or c_other is None:
            continue
        total += 1
        pred = run.label if dist(run.mean, c_self) <= dist(run.mean, c_other) else other_label
        if pred == run.label:
            correct += 1
        else:
            mistakes.append((run.label, pred))
    return correct, total, mistakes


def run_row(run: StableRun) -> dict[str, str]:
    return {
        "session": run.session,
        "runId": str(run.run_id),
        "tStartMs": f"{run.t_start_ms:.1f}",
        "tEndMs": f"{run.t_end_ms:.1f}",
        "durationMs": f"{run.duration_ms:.1f}",
        "ctXMean": f"{run.mean[0]:.3f}",
        "ctYMean": f"{run.mean[1]:.3f}",
        "ctZMean": f"{run.mean[2]:.3f}",
        "ctXStd": f"{run.std[0]:.3f}",
        "ctYStd": f"{run.std[1]:.3f}",
        "ctZStd": f"{run.std[2]:.3f}",
        "label": run.label,
        "distStart": "" if run.dist_start is None else f"{run.dist_start:.3f}",
        "distHongest": "" if run.dist_hongest is None else f"{run.dist_hongest:.3f}",
    }


def finish_rows(session: str, samples: list[ea.Sample]) -> list[dict[str, str]]:
    cfg = ea.DetectorConfig(
        start_deg=30.0,
        still_dps=20.0,
        plateau_ms=250.0,
        commit_ms=300.0,
        min_event_deg=20.0,
    )
    events = ea.detect_events(samples, cfg, session)
    rows: list[dict[str, str]] = []
    for event in events:
        f = event.features
        rows.append(
            {
                "session": session,
                "eventId": str(event.event_id),
                "startMs": f"{event.start_ms:.1f}",
                "commitMs": f"{event.commit_ms:.1f}",
                "releaseStable": ea.fmt(f.get("releaseStable"), 0),
                "releaseSlowMs": ea.fmt(f.get("releaseSlowMs")),
                "releaseSlowAngle": ea.fmt(f.get("releaseSlowAngle")),
                "settleAngle": ea.fmt(f.get("settleAngle")),
                "cycleZeroAngle": ea.fmt(f.get("cycleZeroAngle")),
                "closureAngleRelative": ea.fmt(f.get("closureAngleRelative")),
            }
        )
    return rows


def analyze_file(path: Path, args: argparse.Namespace) -> tuple[list[StableRun], list[dict[str, str]]]:
    samples = ea.read_samples(path)
    intervals = find_stable_runs(
        samples,
        window_ms=args.stable_window_ms,
        min_hold_ms=args.min_hold_ms,
        range_mg=args.stable_range_mg,
        gyro_max=args.stable_gyro_dps,
        max_gap_ms=args.max_gap_ms,
    )
    runs = [
        make_run(path.stem, samples, start, end, i + 1)
        for i, (start, end) in enumerate(intervals)
    ]
    if args.sequence:
        labels = [item.strip().upper() for item in args.sequence.split(",") if item.strip()]
        if not labels or any(item not in ("START", "HONGEST") for item in labels):
            raise ValueError("--sequence 只能是 START/HONGEST，例如 START,HONGEST")
        for index, run in enumerate(runs):
            run.label = labels[index % len(labels)]
    elif args.single_position:
        # 单段采集由操作者声明位置；避免用跨会话不稳定的绝对 X/Z 阈值。
        for run in runs:
            run.label = args.single_position
        if len(runs) > 1:
            print(
                f"[WARN] {path.name}: 检测到 {len(runs)} 个稳定段，"
                f"单段模式全部标为 {args.single_position}",
                file=sys.stderr,
            )
    else:
        classify_runs(
            runs,
            args.start_radius_mg,
        args.start_norm_max,
        args.hongest_distance_mg,
        )
    starts = centroid(r for r in runs if r.label == "START")
    hongests = centroid(r for r in runs if r.label == "HONGEST")
    for run in runs:
        run.dist_start = None if starts is None else dist(run.mean, starts)
        run.dist_hongest = None if hongests is None else dist(run.mean, hongests)
    return runs, finish_rows(path.stem, samples)


def summary_text(all_runs: list[StableRun], finish: list[dict[str, str]]) -> str:
    lines: list[str] = []
    sessions = sorted({r.session for r in all_runs})
    for session in sessions:
        runs = [r for r in all_runs if r.session == session]
        starts = [r for r in runs if r.label == "START"]
        hongests = [r for r in runs if r.label == "HONGEST"]
        others = [r for r in runs if r.label == "OTHER/FINISH"]
        lines.append(f"\n[{session}] stable={len(runs)} START={len(starts)} HONGEST={len(hongests)} OTHER/FINISH={len(others)}")
        for name, group in (("START", starts), ("HONGEST", hongests)):
            c = centroid(group)
            if c is None:
                lines.append(f"  {name}: 无样本")
                continue
            max_std = max(max(r.std) for r in group)
            lines.append(
                f"  {name}: center=({c[0]:.1f},{c[1]:.1f},{c[2]:.1f}) "
                f"max_std={max_std:.1f} n={len(group)}"
            )
        cs, ch = centroid(starts), centroid(hongests)
        if cs is not None and ch is not None:
            lines.append(f"  distance START-HONGEST: {dist(cs, ch):.1f} mg")
        correct, total, mistakes = loo_validation(runs)
        if total:
            lines.append(f"  leave-one-run-out: {correct}/{total}")
            if mistakes:
                lines.append(f"  mistakes: {mistakes}")
        else:
            lines.append("  leave-one-run-out: 样本不足")
    if finish:
        stable = sum(1 for r in finish if r["releaseStable"] == "1")
        lines.append(f"\nFINISH events: {len(finish)}, releaseStable=1: {stable}")
    return "\n".join(lines) + "\n"


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="离线 START/HONGEST 静态分类与 FINISH 回落分析")
    parser.add_argument("inputs", nargs="+", help="label 模式 CSV")
    parser.add_argument("--out", default="static_position_analysis", help="输出前缀")
    parser.add_argument("--stable-window-ms", type=float, default=500.0)
    parser.add_argument("--min-hold-ms", type=float, default=800.0)
    parser.add_argument("--stable-range-mg", type=float, default=30.0)
    parser.add_argument("--stable-gyro-dps", type=float, default=4.0)
    parser.add_argument("--max-gap-ms", type=float, default=1000.0)
    parser.add_argument("--start-radius-mg", type=float, default=150.0)
    parser.add_argument("--start-norm-max", type=float, default=200.0)
    parser.add_argument("--hongest-distance-mg", type=float, default=500.0)
    parser.add_argument(
        "--single-position",
        choices=("START", "HONGEST"),
        help="单段采集时由操作者声明该文件属于 START 或 HONGEST",
    )
    parser.add_argument(
        "--sequence",
        help="按稳定段顺序循环标注，例如 START,HONGEST",
    )
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    all_runs: list[StableRun] = []
    all_finish: list[dict[str, str]] = []
    for item in args.inputs:
        path = Path(item)
        if not path.exists():
            print(f"[FAIL] 文件不存在: {path}", file=sys.stderr)
            return 1
        try:
            runs, finish = analyze_file(path, args)
        except Exception as exc:
            print(f"[FAIL] {path}: {exc}", file=sys.stderr)
            return 1
        all_runs.extend(runs)
        all_finish.extend(finish)

    prefix = Path(args.out)
    prefix.parent.mkdir(parents=True, exist_ok=True)
    runs_path = Path(str(prefix) + ".static_runs.csv")
    finish_path = Path(str(prefix) + ".finish_events.csv")
    summary_path = Path(str(prefix) + ".static_summary.txt")

    run_fields = [
        "session", "runId", "tStartMs", "tEndMs", "durationMs",
        "ctXMean", "ctYMean", "ctZMean", "ctXStd", "ctYStd", "ctZStd",
        "label", "distStart", "distHongest",
    ]
    with runs_path.open("w", newline="", encoding="utf-8-sig") as fp:
        writer = csv.DictWriter(fp, fieldnames=run_fields)
        writer.writeheader()
        writer.writerows(run_row(r) for r in all_runs)

    finish_fields = [
        "session", "eventId", "startMs", "commitMs", "releaseStable",
        "releaseSlowMs", "releaseSlowAngle", "settleAngle",
        "cycleZeroAngle", "closureAngleRelative",
    ]
    with finish_path.open("w", newline="", encoding="utf-8-sig") as fp:
        writer = csv.DictWriter(fp, fieldnames=finish_fields)
        writer.writeheader()
        writer.writerows(all_finish)

    summary = summary_text(all_runs, all_finish)
    summary_path.write_text(summary, encoding="utf-8")
    print(summary)
    print(f"静态段: {runs_path}")
    print(f"FINISH 事件: {finish_path}")
    print(f"汇总: {summary_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
