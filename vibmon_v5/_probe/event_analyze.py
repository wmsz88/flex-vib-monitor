#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""vibmon_v5 闭合事件离线分析。

输入优先使用当前固件 label 模式生成的 16 列 CSV：
  seq,t_ms,ctX,ctY,ctZ,ctDev,ampMg,vib,contact,cvalid,gap,label,
  ctTouchDist,closed,closeAngleDeg,gyroW

输出：
  *.events.csv       一次闭合一行的事件特征
  *.events.summary.txt
  *.events.png       --plot 时生成

本脚本只做离线特征提取，不改变固件判据，也不产生正式 contact。
"""

from __future__ import annotations

import argparse
import csv
import math
import statistics
import sys
import tempfile
import warnings
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Optional


LABEL_COLUMNS = [
    "seq",
    "t_ms",
    "ctX",
    "ctY",
    "ctZ",
    "ctDev",
    "ampMg",
    "vib",
    "contact",
    "cvalid",
    "gap",
    "label",
    "ctTouchDist",
    "closed",
    "closeAngleDeg",
    "gyroW",
]

OUT_FIELDS = [
    "eventId",
    "class",
    "label",
    "startMs",
    "plateauMs",
    "commitMs",
    "openMs",
    "closureMs",
    "startAngle",
    "plateauAngle",
    "angleMoved",
    "gyroPeak",
    "gyroEnergy",
    "impactA",
    "ampPost100",
    "ringdownDb",
    "decayTauMs",
    "ctXMean",
    "ctYMean",
    "ctZMean",
    "angleSlopeDegS",
    "releaseFastMs",
    "releaseSlowMs",
    "releaseSlowAngle",
    "releaseStableMs",
    "returnMinAngle",
    "returnMinMs",
    "settleAngle",
    "settleSlopeDegS",
    "cycleZeroAngle",
    "closureAngleRelative",
    "releaseStable",
    "closedRatio",
    "gapCount",
    "sampleCount",
    "touchScore",
    "scoreValid",
]


@dataclass
class Sample:
    index: int
    t_ms: float
    angle: Optional[float]
    gyro: Optional[float]
    amp: Optional[float]
    x: Optional[float]
    y: Optional[float]
    z: Optional[float]
    closed: Optional[int]
    label: Optional[str]
    gap: int


@dataclass
class Event:
    event_id: int
    class_name: str
    label: Optional[str]
    start_idx: int
    plateau_idx: int
    commit_idx: int
    start_ms: float
    plateau_ms: float
    commit_ms: float
    open_ms: Optional[float] = None
    features: dict[str, Optional[float]] = None

    def __post_init__(self) -> None:
        if self.features is None:
            self.features = {}


@dataclass
class DetectorConfig:
    start_deg: float = 10.0
    move_dps: float = 3.0
    still_dps: float = 20.0
    plateau_angle_tol_deg: float = 5.0
    plateau_ms: float = 250.0
    commit_ms: float = 300.0
    open_deg: float = 20.0
    open_still_ms: float = 500.0
    open_search_ms: float = 3000.0
    fast_drop_dps: float = 20.0
    slow_drop_dps: float = 5.0
    stable_slope_dps: float = 2.5
    stable_tol_deg: float = 2.5
    stable_ms: float = 500.0
    release_search_ms: float = 6000.0
    release_window_ms: float = 200.0
    release_min_drop_deg: float = 15.0
    min_event_deg: float = 20.0
    max_gap_ms: float = 1000.0


def fnum(value: object) -> Optional[float]:
    if value is None:
        return None
    text = str(value).strip()
    if text in ("", "?", "nan", "NaN"):
        return None
    try:
        out = float(text)
    except ValueError:
        return None
    return out if math.isfinite(out) else None


def inum(value: object) -> Optional[int]:
    val = fnum(value)
    return int(val) if val is not None else None


def _field(row: dict[str, str], names: Iterable[str]) -> Optional[str]:
    for name in names:
        value = row.get(name)
        if value is not None and str(value).strip() != "":
            return value
    return None


def _row_value(row: dict[str, str], names: Iterable[str]) -> Optional[str]:
    lowered = {str(k).strip().lower(): v for k, v in row.items()}
    for name in names:
        value = lowered.get(name.lower())
        if value is not None:
            return value
    return None


def _can_float(text: str) -> bool:
    try:
        float(text)
        return True
    except ValueError:
        return False


def read_samples(path: Path) -> list[Sample]:
    """读取 label CSV；兼容带/不带 state 表头的 16 列数据。"""
    with path.open("r", encoding="utf-8-sig", newline="") as fp:
        lines = [
            line.rstrip("\r\n")
            for line in fp
            if line.strip() and not line.startswith("#") and not line.startswith("EV,")
        ]
    if not lines:
        raise ValueError(f"文件没有数据行: {path}")

    first_cells = next(csv.reader([lines[0]]))
    has_header = not _can_float(first_cells[0].strip())
    if has_header:
        reader = csv.DictReader(lines)
        raw_rows = [dict(row) for row in reader]
    else:
        # 无表头时按固件 label 输出顺序解释。
        header = LABEL_COLUMNS
        raw_rows = []
        for line in lines:
            values = next(csv.reader([line]))
            if len(values) != len(header):
                continue
            raw_rows.append(dict(zip(header, values)))

    if not raw_rows:
        raise ValueError(f"没有解析到 CSV 数据行: {path}")

    samples: list[Sample] = []
    for raw in raw_rows:
        t_ms_text = _row_value(raw, ["t_ms", "time_ms"])
        t_s_text = _row_value(raw, ["t_dev_s", "t_s", "t"])
        if t_ms_text is not None:
            t_ms = fnum(t_ms_text)
        elif t_s_text is not None:
            t_sec = fnum(t_s_text)
            t_ms = t_sec * 1000.0 if t_sec is not None else None
        else:
            t_ms = None
        if t_ms is None:
            continue

        angle = fnum(_row_value(raw, ["closeangledeg", "angle_deg", "ang", "angle"]))
        gyro = fnum(_row_value(raw, ["gyrow", "gyro_dps", "gyrow_dps", "w"]))
        amp = fnum(_row_value(raw, ["ampmg", "amp_mg", "a"]))
        x = fnum(_row_value(raw, ["ctx", "x"]))
        y = fnum(_row_value(raw, ["cty", "y"]))
        z = fnum(_row_value(raw, ["ctz", "z"]))
        closed = inum(_row_value(raw, ["closed", "cl"]))
        gap = inum(_row_value(raw, ["gap"])) or 0
        label = _row_value(raw, ["label", "state"])

        samples.append(
            Sample(
                index=len(samples),
                t_ms=float(t_ms),
                angle=angle,
                gyro=gyro,
                amp=amp,
                x=x,
                y=y,
                z=z,
                closed=closed,
                label=str(label).strip() if label is not None else None,
                gap=int(gap),
            )
        )

    if not samples:
        raise ValueError(f"没有带时间戳的数据行: {path}")
    if not any(s.angle is not None for s in samples):
        raise ValueError("缺少 closeAngleDeg/ang 列，无法做闭合事件分析")
    if not any(s.gyro is not None for s in samples):
        raise ValueError("缺少 gyroW 列，无法判断 plateau；请用当前固件 label 模式重采")
    return samples


def mean_valid(values: Iterable[Optional[float]]) -> Optional[float]:
    vals = [v for v in values if v is not None and math.isfinite(v)]
    return statistics.fmean(vals) if vals else None


def max_valid(values: Iterable[Optional[float]]) -> Optional[float]:
    vals = [v for v in values if v is not None and math.isfinite(v)]
    return max(vals) if vals else None


def linreg_slope(points: Iterable[tuple[float, float]]) -> Optional[float]:
    pts = list(points)
    if len(pts) < 2:
        return None
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    mx = statistics.fmean(xs)
    my = statistics.fmean(ys)
    den = sum((x - mx) ** 2 for x in xs)
    if den <= 0:
        return None
    return sum((x - mx) * (y - my) for x, y in pts) / den


def range_indices(samples: list[Sample], start_idx: int, end_idx: int) -> list[Sample]:
    return samples[max(0, start_idx) : min(len(samples), end_idx + 1)]


def time_indices(samples: list[Sample], start_ms: float, end_ms: float) -> list[Sample]:
    return [s for s in samples if start_ms <= s.t_ms <= end_ms]


def make_event(
    samples: list[Sample],
    start_idx: int,
    plateau_idx: int,
    commit_idx: int,
    event_id: int,
    class_name: str,
) -> Event:
    start = samples[start_idx]
    plateau = samples[plateau_idx]
    commit = samples[commit_idx]
    move_segment = range_indices(samples, start_idx, plateau_idx)
    plateau_segment = range_indices(samples, plateau_idx, commit_idx)
    plateau_t = plateau.t_ms

    start_angle = start.angle
    move_angles = [s.angle for s in move_segment if s.angle is not None]
    angle_moved = None
    if move_angles and start_angle is not None:
        angle_moved = max(move_angles) - start_angle

    gyro_peak = max_valid(s.gyro for s in move_segment)
    gyro_energy = None
    energy_terms: list[float] = []
    for prev, cur in zip(move_segment, move_segment[1:]):
        if prev.gyro is None or cur.gyro is None:
            continue
        dt_s = (cur.t_ms - prev.t_ms) / 1000.0
        if dt_s > 0:
            energy_terms.append(((prev.gyro + cur.gyro) * 0.5) ** 2 * dt_s)
    if energy_terms:
        gyro_energy = sum(energy_terms)

    # 停止冲击可能落在 plateau 起点本身，因此窗口同时覆盖前后。
    impact_amp = time_indices(samples, plateau_t - 20.0, plateau_t + 50.0)
    impact_a = max_valid(s.amp for s in impact_amp)
    post_amp = time_indices(samples, plateau_t, plateau_t + 100.0)
    amp_post = mean_valid(s.amp for s in post_amp)
    ringdown_db: Optional[float] = None
    if impact_a is not None and amp_post is not None and impact_a > 0.0 and amp_post > 0.0:
        ringdown_db = 20.0 * math.log10(amp_post / impact_a)

    decay_points: list[tuple[float, float]] = []
    for s in plateau_segment:
        if s.amp is not None and s.amp > 0.0 and s.t_ms >= plateau_t:
            decay_points.append(((s.t_ms - plateau_t) / 1000.0, math.log(s.amp)))
    decay_slope = linreg_slope(decay_points)
    decay_tau_ms = (
        -1000.0 / decay_slope
        if decay_slope is not None and decay_slope < -1e-9
        else None
    )

    angle_points = [
        ((s.t_ms - plateau_t) / 1000.0, s.angle)
        for s in time_indices(samples, plateau_t - 100.0, plateau_t)
        if s.angle is not None
    ]
    angle_slope = linreg_slope(angle_points)

    plateau_angles = [s.angle for s in plateau_segment if s.angle is not None]
    plateau_angle = mean_valid(plateau_angles)
    if plateau_angle is None and commit.angle is not None:
        plateau_angle = commit.angle

    closed_values = [s.closed for s in plateau_segment if s.closed is not None]
    closed_ratio = (
        sum(1 for v in closed_values if v != 0) / len(closed_values)
        if closed_values
        else None
    )

    steady = time_indices(samples, commit.t_ms, commit.t_ms + 500.0)
    x_mean = mean_valid(s.x for s in steady)
    y_mean = mean_valid(s.y for s in steady)
    z_mean = mean_valid(s.z for s in steady)

    features = {
        "closureMs": plateau_t - start.t_ms,
        "startAngle": start_angle,
        "plateauAngle": plateau_angle,
        "angleMoved": angle_moved,
        "gyroPeak": gyro_peak,
        "gyroEnergy": gyro_energy,
        "impactA": impact_a,
        "ampPost100": amp_post,
        "ringdownDb": ringdown_db,
        "decayTauMs": decay_tau_ms,
        "ctXMean": x_mean,
        "ctYMean": y_mean,
        "ctZMean": z_mean,
        "angleSlopeDegS": angle_slope,
        "closedRatio": closed_ratio,
    }
    return Event(
        event_id=event_id,
        class_name=class_name,
        label=start.label,
        start_idx=start_idx,
        plateau_idx=plateau_idx,
        commit_idx=commit_idx,
        start_ms=start.t_ms,
        plateau_ms=plateau_t,
        commit_ms=commit.t_ms,
        features=features,
    )


def _trailing_slope(samples: list[Sample], index: int, window_ms: float) -> Optional[float]:
    if not samples or index < 0 or index >= len(samples):
        return None
    end = samples[index].t_ms
    window = [
        s
        for s in samples[: index + 1]
        if s.t_ms >= end - window_ms and s.angle is not None
    ]
    if len(window) < 3 or window[-1].t_ms - window[0].t_ms < window_ms * 0.5:
        return None
    return linreg_slope(
        ((s.t_ms - end) / 1000.0, s.angle) for s in window
    )


def analyze_release(
    samples: list[Sample],
    event: Event,
    end_idx: int,
    cfg: DetectorConfig,
) -> dict[str, Optional[float]]:
    """分析 plateau 之后的 快速下降 -> 缓降 -> 稳定 回落。

    cycleZeroAngle 只有在稳定窗口满足条件后才输出。这里只做离线参考，
    不修改固件的绝对角度。
    """
    commit_ms = event.commit_ms
    peak_angle = event.features.get("plateauAngle")
    if peak_angle is None:
        peak_angle = samples[event.commit_idx].angle
    if peak_angle is None:
        return {}
    min_drop = max(cfg.release_min_drop_deg, 0.15 * peak_angle)

    search_end = min(
        end_idx,
        len(samples) - 1,
    )
    release = [
        s
        for s in samples[event.commit_idx : search_end + 1]
        if s.t_ms - commit_ms <= cfg.release_search_ms
    ]
    if len(release) < 3:
        return {}

    min_sample = min(
        (s for s in release if s.angle is not None),
        key=lambda s: s.angle,
        default=None,
    )

    fast_sample: Optional[Sample] = None
    slow_sample: Optional[Sample] = None
    stable_sample: Optional[Sample] = None
    settle_window: list[Sample] = []

    for release_index, s in enumerate(release):
        if s.angle is None:
            continue
        slope = _trailing_slope(release, release_index, cfg.release_window_ms)
        if slope is None:
            continue
        if fast_sample is None:
            if slope <= -cfg.fast_drop_dps and s.angle < peak_angle - min_drop:
                fast_sample = s
            continue
        if slow_sample is None:
            if slope >= -cfg.slow_drop_dps and s.angle < peak_angle - min_drop:
                slow_sample = s
            continue
        # 下一轮动作可能刚好在稳定窗口末尾开始；允许 20% 的边界余量，
        # 但稳定窗口本身仍按 stable_ms 计算。
        if s.t_ms - slow_sample.t_ms < cfg.stable_ms * 0.8:
            continue
        window = [
            x
            for x in release
            if s.t_ms - cfg.stable_ms <= x.t_ms <= s.t_ms and x.angle is not None
        ]
        if len(window) < 3:
            continue
        window_angles = [x.angle for x in window]
        window_slope = linreg_slope(
            ((x.t_ms - s.t_ms) / 1000.0, x.angle) for x in window
        )
        if (
            max(window_angles) - min(window_angles) <= cfg.stable_tol_deg
            and window_slope is not None
            and abs(window_slope) <= cfg.stable_slope_dps
        ):
            stable_sample = s
            settle_window = window
            break

    out: dict[str, Optional[float]] = {
        "releaseFastMs": (
            fast_sample.t_ms - commit_ms if fast_sample is not None else None
        ),
        "releaseSlowMs": (
            slow_sample.t_ms - commit_ms if slow_sample is not None else None
        ),
        "releaseSlowAngle": (
            slow_sample.angle if slow_sample is not None else None
        ),
        "releaseStableMs": (
            stable_sample.t_ms - commit_ms if stable_sample is not None else None
        ),
        "returnMinAngle": min_sample.angle if min_sample is not None else None,
        "returnMinMs": (
            min_sample.t_ms - commit_ms if min_sample is not None else None
        ),
        "settleAngle": None,
        "settleSlopeDegS": None,
        "cycleZeroAngle": None,
        "closureAngleRelative": None,
        "releaseStable": 0.0,
    }

    if stable_sample is not None and settle_window:
        settle_angle = mean_valid(x.angle for x in settle_window)
        settle_slope = linreg_slope(
            ((x.t_ms - stable_sample.t_ms) / 1000.0, x.angle)
            for x in settle_window
        )
        out["settleAngle"] = settle_angle
        out["settleSlopeDegS"] = settle_slope
        out["cycleZeroAngle"] = settle_angle
        if settle_angle is not None:
            out["closureAngleRelative"] = peak_angle - settle_angle
        out["releaseStable"] = 1.0
    return out


def annotate_release(
    events: list[Event],
    samples: list[Sample],
    cfg: DetectorConfig,
) -> list[Event]:
    for index, event in enumerate(events):
        if index + 1 < len(events):
            end_idx = events[index + 1].start_idx - 1
        else:
            end_idx = len(samples) - 1
        event.features.update(analyze_release(samples, event, end_idx, cfg))
    return events


class EventDetector:
    def __init__(self, samples: list[Sample], cfg: DetectorConfig, class_name: str):
        self.samples = samples
        self.cfg = cfg
        self.class_name = class_name
        self.state = "ARMED"
        self.events: list[Event] = []
        self._start_idx: Optional[int] = None
        self._start_t: Optional[float] = None
        self._start_angle: Optional[float] = None
        self._peak_angle: Optional[float] = None
        self._still_start_idx: Optional[int] = None
        self._still_start_t: Optional[float] = None
        self._plateau_idx: Optional[int] = None
        self._plateau_t: Optional[float] = None
        self._open_still_idx: Optional[int] = None
        self._open_still_t: Optional[float] = None
        self._last_t: Optional[float] = None
        self._last_event: Optional[Event] = None

    def _begin(self, sample: Sample) -> None:
        self.state = "MOVING"
        self._start_idx = sample.index
        self._start_t = sample.t_ms
        self._start_angle = sample.angle
        self._peak_angle = sample.angle
        self._still_start_idx = None
        self._still_start_t = None
        self._plateau_idx = None
        self._plateau_t = None
        self._open_still_idx = None
        self._open_still_t = None

    def _reset_motion(self) -> None:
        self.state = "ARMED" if self.state != "COMMIT" else "COMMIT"
        self._start_idx = None
        self._start_t = None
        self._start_angle = None
        self._peak_angle = None
        self._still_start_idx = None
        self._still_start_t = None
        self._plateau_idx = None
        self._plateau_t = None

    def _gap_before(self, sample: Sample) -> bool:
        return sample.gap == 1 or (
            self._last_t is not None
            and (
                sample.t_ms < self._last_t
                or sample.t_ms - self._last_t > self.cfg.max_gap_ms
            )
        )

    def _update_motion_peak(self, sample: Sample) -> None:
        if sample.angle is not None and (
            self._peak_angle is None or sample.angle > self._peak_angle
        ):
            self._peak_angle = sample.angle

    def feed(self, sample: Sample) -> None:
        if self._gap_before(sample):
            if self.state in ("MOVING", "PLATEAU"):
                self._reset_motion()
            self._still_start_idx = None
            self._still_start_t = None
            self._open_still_idx = None
            self._open_still_t = None
        self._last_t = sample.t_ms

        angle = sample.angle
        gyro = sample.gyro
        moving = (
            (angle is not None and angle >= self.cfg.start_deg)
            or (gyro is not None and gyro >= self.cfg.move_dps)
        )
        still = gyro is not None and gyro < self.cfg.still_dps

        if self.state == "ARMED":
            if moving:
                self._begin(sample)
            return

        if self.state == "MOVING":
            self._update_motion_peak(sample)
            if angle is not None and angle >= self.cfg.start_deg:
                moving = True
            # 回到张开角度附近就取消未完成的动作。这里不能要求 gyro 已静止：
            # 实际开合时张开动作本身仍有几十 dps，旧条件会把状态永久卡在 MOVING。
            if (
                angle is not None
                and angle < self.cfg.open_deg
                and self._start_t is not None
                and sample.t_ms - self._start_t >= self.cfg.plateau_ms
            ):
                self._reset_motion()
                return
            # 小动作或半途退回张开位：没有形成 event，不能把下一次闭合并进来。
            angle_moved_now = None
            if self._peak_angle is not None and self._start_angle is not None:
                angle_moved_now = self._peak_angle - self._start_angle
            if (
                still
                and angle is not None
                and angle < self.cfg.open_deg
                and self._start_t is not None
                and sample.t_ms - self._start_t >= self.cfg.plateau_ms
                and (
                    angle_moved_now is None
                    or angle_moved_now < self.cfg.min_event_deg
                )
            ):
                self._reset_motion()
                return
            if gyro is None:
                self._still_start_idx = None
                self._still_start_t = None
                return
            if still:
                if self._still_start_t is None:
                    self._still_start_idx = sample.index
                    self._still_start_t = sample.t_ms
                angle_moved = None
                if self._peak_angle is not None and self._start_angle is not None:
                    angle_moved = self._peak_angle - self._start_angle
                still_ms = sample.t_ms - (self._still_start_t or sample.t_ms)
                if (
                    still_ms >= self.cfg.plateau_ms
                    and angle_moved is not None
                    and angle_moved >= self.cfg.min_event_deg
                ):
                    self.state = "PLATEAU"
                    self._plateau_idx = self._still_start_idx
                    self._plateau_t = self._still_start_t
            else:
                self._still_start_idx = None
                self._still_start_t = None
            return

        if self.state == "PLATEAU":
            self._update_motion_peak(sample)
            if gyro is None:
                return
            if not still:
                self.state = "MOVING"
                self._still_start_idx = None
                self._still_start_t = None
                self._plateau_idx = None
                self._plateau_t = None
                return
            still_ms = sample.t_ms - (self._plateau_t or sample.t_ms)
            if still_ms >= self.cfg.plateau_ms + self.cfg.commit_ms:
                if self._start_idx is None or self._plateau_idx is None:
                    self._reset_motion()
                    return
                self.events.append(
                    make_event(
                        self.samples,
                        self._start_idx,
                        self._plateau_idx,
                        sample.index,
                        len(self.events) + 1,
                        self.class_name,
                    )
                )
                self._last_event = self.events[-1]
                self.state = "COMMIT"
                self._open_still_idx = None
                self._open_still_t = None
            return

        if self.state == "COMMIT":
            self._update_motion_peak(sample)
            # 重新张开时角速度仍可能很高，不能把 gyro 死区作为 event 结束条件。
            # 角度回到低值即可认为本次闭合结束；500 ms 只用于稳定确认。
            open_now = angle is not None and angle < self.cfg.open_deg
            if open_now:
                if self._open_still_t is None:
                    self._open_still_idx = sample.index
                    self._open_still_t = sample.t_ms
                    if self._last_event is not None:
                        self._last_event.open_ms = sample.t_ms
                if sample.t_ms - (self._open_still_t or sample.t_ms) >= self.cfg.open_still_ms:
                    self.state = "ARMED"
                    self._reset_motion()
                    self._last_event = None
            else:
                # 动作连续、没有停满 openStillMs 时，只要从低角度重新抬起，
                # 就结束上一轮并开始下一轮，避免整段循环被合成一个事件。
                if (
                    self._open_still_t is not None
                    and angle is not None
                    and angle >= self.cfg.start_deg
                ):
                    self.state = "ARMED"
                    self._reset_motion()
                    self._last_event = None
                    if moving:
                        self._begin(sample)
                    return
                self._open_still_idx = None
                self._open_still_t = None

    def finish(self) -> list[Event]:
        # 没有等到重新张开的事件仍然保留；openMs 留空。
        return self.events


def detect_events(
    samples: list[Sample],
    cfg: DetectorConfig,
    class_name: str,
) -> list[Event]:
    """默认用离线分段，避免连续开合时状态机卡在 COMMIT。

    每段从角度越过 start_deg 开始，到角度回落/时间断点结束；在段内从峰值
    向前寻找持续 plateau_ms 的角度稳定窗口。这样即使张开动作本身仍有几十 dps，
    也不会把多次开合合并成一个事件。
    """
    regions: list[list[Sample]] = []
    current: list[Sample] = []
    for sample in samples:
        is_high = sample.angle is not None and sample.angle >= cfg.start_deg
        if is_high:
            if current and (
                sample.t_ms < current[-1].t_ms
                or sample.t_ms - current[-1].t_ms > cfg.max_gap_ms
            ):
                regions.append(current)
                current = []
            current.append(sample)
        elif current:
            regions.append(current)
            current = []
    if current:
        regions.append(current)

    events: list[Event] = []
    for region in regions:
        if not region:
            continue
        angles = [s.angle for s in region if s.angle is not None]
        if not angles:
            continue
        start_angle = region[0].angle
        if start_angle is None:
            continue
        if max(angles) - start_angle < cfg.min_event_deg:
            continue

        peak_idx = max(
            range(len(region)),
            key=lambda i: region[i].angle if region[i].angle is not None else -math.inf,
        )
        plateau_idx = peak_idx
        for i in range(peak_idx + 1):
            if region[peak_idx].t_ms - region[i].t_ms < cfg.plateau_ms:
                continue
            window = [
                s.angle
                for s in region[i : peak_idx + 1]
                if s.angle is not None
            ]
            if window and max(window) - min(window) <= cfg.plateau_angle_tol_deg:
                plateau_idx = i
                break

        start_sample = region[0]
        peak_sample = region[peak_idx]
        event = make_event(
            samples,
            start_sample.index,
            region[plateau_idx].index,
            peak_sample.index,
            len(events) + 1,
            class_name,
        )
        for after in samples[peak_sample.index + 1 :]:
            if after.angle is not None and after.angle < cfg.open_deg:
                event.open_ms = after.t_ms
                break
            if after.t_ms - peak_sample.t_ms > cfg.open_search_ms:
                break
        events.append(event)
    return annotate_release(events, samples, cfg)


def detect_events_state(
    samples: list[Sample],
    cfg: DetectorConfig,
    class_name: str,
) -> list[Event]:
    """保留原状态机路径，供调试 ARMED/MOVING/PLATEAU/COMMIT/OPEN 使用。"""
    detector = EventDetector(samples, cfg, class_name)
    for sample in samples:
        detector.feed(sample)
    return annotate_release(detector.finish(), samples, cfg)


def fmt(value: Optional[float], digits: int = 3) -> str:
    return "" if value is None else f"{value:.{digits}f}"


def event_row(event: Event) -> dict[str, str]:
    f = event.features
    return {
        "eventId": str(event.event_id),
        "class": event.class_name,
        "label": event.label or "",
        "startMs": fmt(event.start_ms),
        "plateauMs": fmt(event.plateau_ms),
        "commitMs": fmt(event.commit_ms),
        "openMs": fmt(event.open_ms),
        "closureMs": fmt(f.get("closureMs")),
        "startAngle": fmt(f.get("startAngle")),
        "plateauAngle": fmt(f.get("plateauAngle")),
        "angleMoved": fmt(f.get("angleMoved")),
        "gyroPeak": fmt(f.get("gyroPeak")),
        "gyroEnergy": fmt(f.get("gyroEnergy")),
        "impactA": fmt(f.get("impactA")),
        "ampPost100": fmt(f.get("ampPost100")),
        "ringdownDb": fmt(f.get("ringdownDb")),
        "decayTauMs": fmt(f.get("decayTauMs")),
        "ctXMean": fmt(f.get("ctXMean")),
        "ctYMean": fmt(f.get("ctYMean")),
        "ctZMean": fmt(f.get("ctZMean")),
        "angleSlopeDegS": fmt(f.get("angleSlopeDegS")),
        "releaseFastMs": fmt(f.get("releaseFastMs")),
        "releaseSlowMs": fmt(f.get("releaseSlowMs")),
        "releaseSlowAngle": fmt(f.get("releaseSlowAngle")),
        "releaseStableMs": fmt(f.get("releaseStableMs")),
        "returnMinAngle": fmt(f.get("returnMinAngle")),
        "returnMinMs": fmt(f.get("returnMinMs")),
        "settleAngle": fmt(f.get("settleAngle")),
        "settleSlopeDegS": fmt(f.get("settleSlopeDegS")),
        "cycleZeroAngle": fmt(f.get("cycleZeroAngle")),
        "closureAngleRelative": fmt(f.get("closureAngleRelative")),
        "releaseStable": fmt(f.get("releaseStable"), 0),
        "closedRatio": fmt(f.get("closedRatio")),
        "gapCount": "",
        "sampleCount": str(event.commit_idx - event.start_idx + 1),
        "touchScore": "",
        "scoreValid": "0",
    }

def write_events(path: Path, events: list[Event], samples: list[Sample]) -> None:
    with path.open("w", newline="", encoding="utf-8-sig") as fp:
        writer = csv.DictWriter(fp, fieldnames=OUT_FIELDS)
        writer.writeheader()
        for event in events:
            row = event_row(event)
            seg = range_indices(samples, event.start_idx, event.commit_idx)
            row["gapCount"] = str(sum(1 for s in seg if s.gap))
            writer.writerow(row)


def summary_text(events: list[Event], cfg: DetectorConfig) -> str:
    lines = [
        f"事件数: {len(events)}",
        (
            "参数: "
            f"start={cfg.start_deg:g}deg move={cfg.move_dps:g}dps "
            f"still={cfg.still_dps:g}dps plateau={cfg.plateau_ms:g}ms "
            f"commit={cfg.commit_ms:g}ms open={cfg.open_deg:g}deg "
            f"openStill={cfg.open_still_ms:g}ms"
        ),
    ]
    by_class: dict[str, list[Event]] = {}
    for event in events:
        by_class.setdefault(event.class_name, []).append(event)
    for class_name, group in by_class.items():
        lines.append(f"\n[{class_name}] n={len(group)}")
        for field in (
            "closureMs",
            "plateauAngle",
            "gyroPeak",
            "gyroEnergy",
            "impactA",
            "ringdownDb",
            "ctXMean",
            "ctYMean",
            "ctZMean",
            "angleSlopeDegS",
            "releaseSlowAngle",
            "settleAngle",
            "settleSlopeDegS",
            "cycleZeroAngle",
            "closureAngleRelative",
            "returnMinAngle",
        ):
            values = [
                e.features.get(field)
                for e in group
                if e.features.get(field) is not None
            ]
            if not values:
                lines.append(f"  {field:16s} 无有效值")
                continue
            med = statistics.median(values)
            lines.append(
                f"  {field:16s} n={len(values):3d} "
                f"min={min(values):9.3f} median={med:9.3f} max={max(values):9.3f}"
            )
    return "\n".join(lines) + "\n"


def plot_events(
    path: Path,
    samples: list[Sample],
    events: list[Event],
    class_name: str,
) -> bool:
    try:
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", UserWarning)
            import matplotlib

            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
    except Exception as exc:
        print(f"[WARN] matplotlib 不可用，跳过图: {exc}")
        return False

    if not samples:
        return False
    t0 = samples[0].t_ms
    ts = [(s.t_ms - t0) / 1000.0 for s in samples]

    fig = None
    try:
        fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
        series = [
            ("closeAngleDeg", "angle (deg)", [s.angle for s in samples]),
            ("gyroW", "gyroW (dps)", [s.gyro for s in samples]),
            ("ampMg", "amplitude (mg)", [s.amp for s in samples]),
        ]
        for ax, (name, ylabel, values) in zip(axes, series):
            ax.plot(ts, values, linewidth=0.9)
            ax.set_ylabel(ylabel)
            ax.grid(True, alpha=0.25)
            ax.set_title(f"{class_name}: {name}")
            for event in events:
                for t_ms, style in (
                    (event.start_ms, ":"),
                    (event.plateau_ms, "--"),
                    (event.commit_ms, "-."),
                ):
                    ax.axvline((t_ms - t0) / 1000.0, color="0.35", linestyle=style, alpha=0.7)
            ax.legend(["signal"] + ["start", "plateau", "commit"] * len(events), fontsize=7)
        axes[-1].set_xlabel("time since first sample (s)")
        # 某些 Python 3.14/numpy 组合的 matplotlib 在 tight_layout 会报错；
        # 固定边距足够本次分析使用，也避免因绘图失败让分析结果丢失。
        fig.subplots_adjust(left=0.08, right=0.98, top=0.96, bottom=0.06, hspace=0.35)
        fig.savefig(path, dpi=150)
        return True
    except Exception as exc:
        print(f"[WARN] 绘图失败，跳过图: {exc}")
        return False
    finally:
        if fig is not None:
            try:
                plt.close(fig)
            except Exception:
                pass


def synthetic_samples() -> list[Sample]:
    """生成两段闭合事件：空握 100 deg，夹物 65 deg。"""
    rows: list[Sample] = []
    t = 0.0

    def add(angle: float, gyro: float, amp: float, closed: int) -> None:
        nonlocal t
        rows.append(
            Sample(
                index=len(rows),
                t_ms=t,
                angle=angle,
                gyro=gyro,
                amp=amp,
                x=10.0 + angle * 0.02,
                y=100.0 + angle * 0.1,
                z=-50.0 + angle * 0.05,
                closed=closed,
                label="synth",
                gap=0,
            )
        )
        t += 10.0

    def rest(duration_ms: float, angle: float = 0.0) -> None:
        for _ in range(int(round(duration_ms / 10.0))):
            add(angle, 0.0, 5.0, 1 if angle > 60.0 else 0)

    def close_to(target: float, duration_ms: float) -> None:
        steps = int(round(duration_ms / 10.0))
        for i in range(steps + 1):
            angle = target * i / steps
            add(angle, 200.0 if i < steps else 0.0, 8.0, 1 if angle > 60.0 else 0)

    def plateau(angle: float, duration_ms: float) -> None:
        for i in range(int(round(duration_ms / 10.0))):
            # 停止瞬间有一个冲击，随后回落。
            amp = 600.0 if i < 4 else 80.0
            add(angle, 0.0, amp, 1 if angle > 60.0 else 0)

    def open_from(angle: float, duration_ms: float) -> None:
        steps = int(round(duration_ms / 10.0))
        for i in range(steps + 1):
            a = angle * (1.0 - i / steps)
            add(a, -200.0 if i < steps else 0.0, 8.0, 1 if a > 60.0 else 0)

    rest(600.0)
    close_to(100.0, 500.0)
    plateau(100.0, 800.0)
    open_from(100.0, 400.0)
    rest(600.0)
    close_to(65.0, 500.0)
    plateau(65.0, 800.0)
    open_from(65.0, 400.0)
    rest(600.0)
    return rows


def selftest() -> int:
    samples = synthetic_samples()
    cfg = DetectorConfig(
        plateau_ms=200.0,
        commit_ms=200.0,
        open_still_ms=300.0,
    )
    events = detect_events(samples, cfg, "synthetic")
    failures = 0

    def check(ok: bool, label: str) -> None:
        nonlocal failures
        if not ok:
            failures += 1
        print(f"[{'PASS' if ok else 'FAIL'}] {label}")

    check(len(events) == 2, f"检测到 2 个事件（实际 {len(events)}）")
    if len(events) == 2:
        check(abs((events[0].features["plateauAngle"] or 0) - 100.0) < 5.0, "事件1平台角约 100 deg")
        check(abs((events[1].features["plateauAngle"] or 0) - 65.0) < 5.0, "事件2平台角约 65 deg")
        check(events[0].open_ms is not None, "事件1记录到重新张开")
        check(events[1].open_ms is not None, "事件2记录到重新张开")
        check(events[0].features.get("releaseStable") == 1.0, "事件1回落稳定")
        check(events[1].features.get("releaseStable") == 1.0, "事件2回落稳定")
        zero = events[0].features.get("cycleZeroAngle")
        check(zero is not None and abs(zero) < 2.0, "事件1相对零点约 0 deg")
        check(
            abs(
                (events[0].features.get("closureAngleRelative") or 0)
                - (events[0].features.get("plateauAngle") or 0)
            )
            < 2.0,
            "事件1相对闭合角有效",
        )

    with tempfile.TemporaryDirectory() as temp_dir:
        csv_path = Path(temp_dir) / "synthetic.csv"
        with csv_path.open("w", newline="", encoding="utf-8-sig") as fp:
            writer = csv.DictWriter(fp, fieldnames=["state"] + LABEL_COLUMNS)
            writer.writeheader()
            for s in samples:
                writer.writerow(
                    {
                        "state": "4",
                        "seq": s.index + 1,
                        "t_ms": f"{s.t_ms:.1f}",
                        "ctX": "0",
                        "ctY": f"{s.y:.2f}",
                        "ctZ": f"{s.z:.2f}",
                        "ctDev": "0",
                        "ampMg": f"{s.amp:.2f}",
                        "vib": "0",
                        "contact": "0",
                        "cvalid": "1",
                        "gap": "0",
                        "label": "4",
                        "ctTouchDist": "0",
                        "closed": str(s.closed or 0),
                        "closeAngleDeg": f"{s.angle:.2f}",
                        "gyroW": f"{s.gyro:.2f}",
                    }
                )
        parsed = read_samples(csv_path)
        check(len(parsed) == len(samples), "16 列 label CSV 可解析")
        check(detect_events(parsed, cfg, "parsed").__len__() == 2, "解析后仍检测到 2 个事件")

    print(f"\n{'全部通过' if failures == 0 else str(failures) + ' 项失败'}")
    return 1 if failures else 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="vibmon_v5 闭合事件离线分析")
    parser.add_argument("input", nargs="?", help="label 模式 16 列 CSV")
    parser.add_argument("--out", help="输出文件前缀；默认在输入文件旁生成 *.events")
    parser.add_argument("--class-name", help="类别名；默认使用输入文件名")
    parser.add_argument("--plot", action="store_true", help="生成角度/角速度/幅值图")
    parser.add_argument("--selftest", action="store_true", help="无硬件自测")
    parser.add_argument(
        "--detector",
        choices=("segment", "state"),
        default="segment",
        help="segment=离线分段（默认），state=事件状态机调试",
    )
    parser.add_argument("--start-deg", type=float, default=10.0)
    parser.add_argument("--move-dps", type=float, default=3.0)
    parser.add_argument("--still-dps", type=float, default=20.0)
    parser.add_argument("--angle-tol", type=float, default=5.0)
    parser.add_argument("--plateau-ms", type=float, default=250.0)
    parser.add_argument("--commit-ms", type=float, default=300.0)
    parser.add_argument("--open-deg", type=float, default=20.0)
    parser.add_argument("--open-still-ms", type=float, default=500.0)
    parser.add_argument("--open-search-ms", type=float, default=3000.0)
    parser.add_argument("--fast-drop-dps", type=float, default=20.0)
    parser.add_argument("--slow-drop-dps", type=float, default=5.0)
    parser.add_argument("--stable-slope-dps", type=float, default=2.5)
    parser.add_argument("--stable-tol", type=float, default=2.5)
    parser.add_argument("--stable-ms", type=float, default=500.0)
    parser.add_argument("--release-search-ms", type=float, default=6000.0)
    parser.add_argument("--release-window-ms", type=float, default=200.0)
    parser.add_argument("--release-min-drop-deg", type=float, default=15.0)
    parser.add_argument("--min-event-deg", type=float, default=20.0)
    parser.add_argument("--max-gap-ms", type=float, default=1000.0)
    return parser


def main(argv: Optional[list[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.selftest:
        return selftest()
    if not args.input:
        parser.error("需要输入 CSV，或使用 --selftest")

    input_path = Path(args.input)
    if not input_path.exists():
        parser.error(f"文件不存在: {input_path}")
    cfg = DetectorConfig(
        start_deg=args.start_deg,
        move_dps=args.move_dps,
        still_dps=args.still_dps,
        plateau_angle_tol_deg=args.angle_tol,
        plateau_ms=args.plateau_ms,
        commit_ms=args.commit_ms,
        open_deg=args.open_deg,
        open_still_ms=args.open_still_ms,
        open_search_ms=args.open_search_ms,
        fast_drop_dps=args.fast_drop_dps,
        slow_drop_dps=args.slow_drop_dps,
        stable_slope_dps=args.stable_slope_dps,
        stable_tol_deg=args.stable_tol,
        stable_ms=args.stable_ms,
        release_search_ms=args.release_search_ms,
        release_window_ms=args.release_window_ms,
        release_min_drop_deg=args.release_min_drop_deg,
        min_event_deg=args.min_event_deg,
        max_gap_ms=args.max_gap_ms,
    )
    class_name = args.class_name or input_path.stem
    try:
        samples = read_samples(input_path)
        if args.detector == "state":
            events = detect_events_state(samples, cfg, class_name)
        else:
            events = detect_events(samples, cfg, class_name)
    except Exception as exc:
        print(f"[FAIL] {exc}", file=sys.stderr)
        return 1

    prefix = Path(args.out) if args.out else input_path.with_suffix(".events")
    prefix.parent.mkdir(parents=True, exist_ok=True)
    events_path = Path(str(prefix) + ".csv")
    summary_path = Path(str(prefix) + ".summary.txt")
    write_events(events_path, events, samples)
    summary = summary_text(events, cfg)
    summary_path.write_text(summary, encoding="utf-8")

    print(summary)
    print(f"事件表: {events_path}")
    print(f"汇总: {summary_path}")

    if args.plot:
        plot_path = Path(str(prefix) + ".png")
        if plot_events(plot_path, samples, events, class_name):
            print(f"图: {plot_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
