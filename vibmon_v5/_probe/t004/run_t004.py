#!/usr/bin/env python3
# -*- coding: utf-8 -*-
r"""T004 实板检验第一轮：一轮跑完（语音引导 + 后台采集）。

覆盖任务卡 T004 的四组证据：
  P0   硬件证据：H1 静止 60s、H3 弯折 3min（逐次 'S' 记录 I2C错/读失败/不连续）
  P1   核心判据 + A5：A1 松手 / A2 空握 / A3 接触 / A4 关源退出，重复 N 轮
  T1   工单：A3 稳定后抽走物体但保持振源开启，看 ct 退出（第 1 轮内做）
  dual 双振源：原振子(中高频) 与 新低速振子(低频) 各一段
  dump 顺带：'D' 命令实板样例（空握 + 接触）

模式：脚本占用 检测模块口 + 振源口，System.Speech 中文播报，人按提示做动作。
每个测量段先"等状态到位"再"保持固定时长"，不用秒表、不需要按回车。

振源：默认**同一块双路振源板**（COM7，GPIO3=旧/原振子、GPIO4=新低速振子），
用通道选择字符切换，发送占空比指令前自动加选择字符：
  a = 旧/原振子(GPIO3)   b = 新低速振子(GPIO4)   z = 两路全停
若改用两个独立 COM 口，给 --source-new 指定新口、并把 --old-sel/--new-sel 留空。

用法（默认全跑 P0+P1+dual+dump）：
  python -X utf8 run_t004.py --sensor COM9 --source COM7 ^
      --out-dir "D:\大学\课程\工科赛\代码\vibmon_v5\data\t004_20261102\run1"

只跑部分组：
  --groups p0          # 只跑 P0
  --groups p1          # 只跑 P1 五轮 + T1
  --groups dual        # 只跑双振源
  --groups dump        # 只抓 D 命令样例

产物（在 --out-dir 下）：
  run.csv          主表：每行一个采样点（round/phase/elapsed + #SWEEP 全字段）
  run.raw.txt      设备原始输出（取证）
  run.status.csv   每次 'S' 解析出的计数（H1/H3 量化证据）
  run.dump.txt     'D' 命令输出（含 #DUMPH 表头 + #DUMP 行）
  run.markers.json 关键时间点（phase 起止、振源指令、T1 抽走时刻），供 analyze_t004.py 判读
"""

import argparse
import base64
import csv
import json
import os
import re
import subprocess
import sys
import time
from datetime import datetime

import serial

# ---------------------------------------------------------------- 解析 ----

SWEEP_RE = re.compile(
    r"#SWEEP t=(?P<t>\d+)s "
    r"F=\s*(?P<f>[0-9.]+|\?)Hz "
    r"A=\s*(?P<a>[0-9.]+|\?)mg "
    r"D=\s*(?P<d>[0-9.]+|\?)mg "
    r"(?:ang=\s*(?P<ang>[0-9.]+|\?)deg )?"
    r"tr=\s*(?P<tr>[0-9.]+|\?)mg "
    r"vib=(?P<vib>\d+) ct=(?P<ct>\d+) grip=(?P<grip>\d+) "
    r"cl=(?P<cl>\d+) w=(?P<w>[0-9.]+|\?) "
    r"cliplim=(?P<clim>\d+) ax=(?P<ax>\d+) ok=(?P<ok>\d+)"
)

# 与 state.cpp 的 outPrintStatus 输出一致：
#   起 N / 停 N | 读失败 N  缓冲溢出 N  不连续 N(缺 N, 抖动 N)  削顶 N  I2C错 N
STATUS_STATS_RE = re.compile(
    r"起 (\d+) / 停 (\d+) \| 读失败 (\d+)\s+缓冲溢出 (\d+)\s+"
    r"不连续 (\d+)\(缺 (\d+), 抖动 (\d+)\)\s+削顶 (\d+)\s+I2C错 (\d+)"
)
STATUS_SENSOR_RE = re.compile(r"传感器\s+(在线|故障)\s+标定\s+(\S+)/(\S+)")

SWEEP_FIELDS = ["wall", "epoch", "t_global_s", "round", "phase", "elapsed_s",
                "Freq_Hz", "Amp_mg", "FlatDist_mg", "CloseAngle_deg", "RefSwing_mg",
                "vib", "contact", "gripNoObject", "closed", "gyroW",
                "clipLimited", "axis", "ok"]
STATUS_FIELDS = ["wall", "epoch", "tag", "phase", "sensor", "cal", "nStart", "nStop",
                 "readFail", "ringDrop", "gaps", "missed", "jitter", "clip", "i2cErr"]


def now_hms():
    return datetime.now().strftime("%H:%M:%S.%f")[:-3]


def speak(text, rate=0):
    """用 PowerShell System.Speech 中文播报；Huihui 不存在就用默认音色。"""
    print(f"[SPEAK] {text}", flush=True)
    ps = (
        "Add-Type -AssemblyName System.Speech; "
        "$s = New-Object System.Speech.Synthesis.SpeechSynthesizer; "
        "try { $s.SelectVoice('Microsoft Huihui') } catch {}; "
        f"$s.Rate = {rate}; "
        "$s.Speak('" + text.replace("'", "''") + "')"
    )
    enc = base64.b64encode(ps.encode("utf-16le")).decode("ascii")
    subprocess.run(["powershell", "-NoProfile", "-EncodedCommand", enc], check=False)


def parse_sweep(m):
    return {
        "Freq_Hz": "" if m["f"] == "?" else m["f"],
        "Amp_mg": "" if m["a"] == "?" else m["a"],
        "FlatDist_mg": "" if m["d"] == "?" else m["d"],
        "CloseAngle_deg": "" if m["ang"] in (None, "?") else m["ang"],
        "RefSwing_mg": "" if m["tr"] == "?" else m["tr"],
        "vib": m["vib"], "contact": m["ct"], "gripNoObject": m["grip"],
        "closed": m["cl"], "gyroW": "" if m["w"] == "?" else m["w"],
        "clipLimited": m["clim"], "axis": m["ax"], "ok": m["ok"],
    }


def row_open(row):
    a = row.get("CloseAngle_deg", "")
    if a in ("", "?"):
        return False
    try:
        return row.get("closed") == "0" and float(a) <= 10.0
    except ValueError:
        return False


def row_closed(row):
    a = row.get("CloseAngle_deg", "")
    if a in ("", "?"):
        return False
    try:
        return row.get("closed") == "1" and float(a) >= 60.0
    except ValueError:
        return False


# -------------------------------------------------------------- Session ----

class Session:
    def __init__(self, sensor, source_old, source_new, sel_old, sel_new, out_dir):
        self.out_dir = os.path.abspath(out_dir)
        os.makedirs(self.out_dir, exist_ok=True)
        base = os.path.join(self.out_dir, "run")
        self.base = base
        self.csv_fp = open(base + ".csv", "w", newline="", encoding="utf-8-sig")
        self.writer = csv.DictWriter(self.csv_fp, fieldnames=SWEEP_FIELDS)
        self.writer.writeheader()
        self.csv_fp.flush()
        self.raw = open(base + ".raw.txt", "w", encoding="utf-8", errors="replace")
        self.status_fp = open(base + ".status.txt", "w", encoding="utf-8")
        self.status_csv = open(base + ".status.csv", "w", newline="", encoding="utf-8-sig")
        self.status_writer = csv.DictWriter(self.status_csv, fieldnames=STATUS_FIELDS)
        self.status_writer.writeheader()
        self.status_csv.flush()
        self.dump_fp = open(base + ".dump.txt", "w", encoding="utf-8")
        self.markers = {"sensor": sensor, "source_old": source_old,
                        "source_new": source_new or source_old,
                        "sel_old": sel_old, "sel_new": sel_new, "phases": [],
                        "source_events": [], "faults": [], "t_pull_epoch": None,
                        "status_rows": []}
        self.faults = self.markers["faults"]

        # 振源：可能是「同一 COM 口的两路通道」（用选择字符 a/b 切），
        # 也可能是两个独立口（选择字符留空）。每个"源"= (句柄, 选择字符)。
        self._src_handles = {}

        def _open(port):
            if port not in self._src_handles:
                self._src_handles[port] = serial.Serial(port, 115200, timeout=0.2)
            return self._src_handles[port]

        self.sources = {
            "old": (_open(source_old), sel_old or ""),
            "new": (_open(source_new or source_old), sel_new or ""),
        }
        self.sen = serial.Serial(sensor, 115200, timeout=0.2)
        time.sleep(0.8)
        self.sen.reset_input_buffer()
        for p in self._src_handles.values():
            p.reset_input_buffer()
            p.write(b"z")           # 开机确保振源全停
        time.sleep(0.4)
        self.t_global = time.time()

    # -- 低层 I/O --
    def read_for(self, seconds):
        buf = []
        t0 = time.time()
        while time.time() - t0 < seconds:
            chunk = self.sen.read(4096)
            if chunk:
                buf.append(chunk.decode("utf-8", "replace"))
            else:
                time.sleep(0.02)
        return "".join(buf)

    def consume(self, text, phase, round_no, elapsed):
        if not text:
            return
        self.raw.write(text)
        self.raw.flush()
        for line in text.splitlines():
            if "[FAULT]" in line:
                self.faults.append({"wall": now_hms(), "phase": phase,
                                    "elapsed_s": round(elapsed, 2),
                                    "line": line.strip()})
                print(f"[!! FAULT] {phase} @ {elapsed:.1f}s: {line.strip()}", flush=True)
        for m in SWEEP_RE.finditer(text):
            row = {"wall": now_hms(), "epoch": f"{time.time():.3f}",
                   "t_global_s": f"{time.time() - self.t_global:.1f}",
                   "round": round_no, "phase": phase, "elapsed_s": f"{elapsed:.2f}"}
            row.update(parse_sweep(m))
            self.writer.writerow(row)
            self.csv_fp.flush()

    def send_source(self, cmd, which="old"):
        p, sel = self.sources[which]
        p.reset_input_buffer()
        # 'z'=两路全停，直接发；其余占空比指令先发通道选择字符再发指令。
        sent = cmd if cmd == "z" else (f"{sel}{cmd}" if sel else cmd)
        p.write(sent.encode())
        self.markers["source_events"].append({"wall": now_hms(),
                                              "epoch": round(time.time(), 3),
                                              "cmd": cmd, "which": which, "sent": sent})
        print(f"[SRC {which}] sent={sent!r} @ {now_hms()}", flush=True)

    # -- 高层动作 --
    def boot(self, timeout=20.0):
        speak("模块复位中，请保持张开静止")
        deadline = time.time() + timeout
        while time.time() < deadline:
            self.sen.write(b"w")
            text = self.read_for(0.5)
            if "#SWEEP" in text:
                self.consume(text, "boot", 0, time.time() - self.t_global)
                self.sen.write(b"0")          # 关绘图组，避免波形行混入
                time.sleep(0.3)
                self.sen.reset_input_buffer()
                print("模块就绪（已完成标定）。", flush=True)
                return True
            self.consume(text, "boot", 0, time.time() - self.t_global)
        raise SystemExit("模块 20s 内没有 #SWEEP 应答，检查串口/固件")

    def poll(self, phase, seconds, round_no=0, interval=1.0,
             status_every=None, note=""):
        if note:
            print(f"\n[{phase}] {note}", flush=True)
        start = time.time()
        self.markers["phases"].append({"phase": phase, "round": round_no,
                                       "start_epoch": round(start, 3),
                                       "should_s": seconds})
        nxt = 0.0
        nxt_status = status_every if status_every else None
        t_last_status = 0.0
        while True:
            el = time.time() - start
            if el >= seconds:
                break
            if el >= nxt:
                self.sen.write(b"w")
                nxt += interval
            if nxt_status and el - t_last_status >= nxt_status:
                self.status(tag=f"{phase}@{el:.0f}s", phase=phase, round_no=round_no)
                t_last_status = el
            text = self.sen.read(4096)
            if text:
                self.consume(text.decode("utf-8", "replace"), phase, round_no, el)
            else:
                time.sleep(0.01)
        self.markers["phases"][-1]["end_epoch"] = round(time.time(), 3)

    def wait_state(self, predicate, timeout, phase, round_no=0, prompt="",
                   interval=0.5):
        if prompt:
            print(f"\n[{phase}] {prompt}", flush=True)
        start = time.time()
        self.markers["phases"].append({"phase": phase, "round": round_no,
                                       "start_epoch": round(start, 3),
                                       "should_s": f"wait<={timeout:.0f}"})
        nxt = 0.0
        while time.time() - start < timeout:
            el = time.time() - start
            if el >= nxt:
                self.sen.write(b"w")
                nxt += interval
            text = self.sen.read(4096)
            if not text:
                time.sleep(0.01)
                continue
            text = text.decode("utf-8", "replace")
            hit = None
            for m in SWEEP_RE.finditer(text):
                row = parse_sweep(m)
                if predicate(row):
                    hit = row
            self.consume(text, phase, round_no, el)
            if hit is not None:
                self.markers["phases"][-1]["end_epoch"] = round(time.time(), 3)
                print(f"  到位：{phase} ang={hit.get('CloseAngle_deg')} "
                      f"cl={hit.get('closed')} @ {el:.1f}s", flush=True)
                return True
        self.markers["phases"][-1]["end_epoch"] = round(time.time(), 3)
        print(f"  !! 未到位：{phase}（{timeout:.0f}s 超时）", flush=True)
        return False

    def rebaseline(self, phase):
        self.sen.write(b"t")
        time.sleep(1.2)
        text = self.read_for(0.6)
        self.consume(text, phase, 0, 0.0)
        print(f"[BASELINE] 't' 已发送 @ {now_hms()}", flush=True)

    def status(self, tag, phase="p0", round_no=0):
        self.sen.reset_input_buffer()
        self.sen.write(b"S")
        text = self.read_for(1.4)
        self.consume(text, phase, round_no, time.time() - self.t_global)
        self.status_fp.write(f"\n===== [{tag}] wall={now_hms()} =====\n{text}\n")
        self.status_fp.flush()
        ms = STATUS_STATS_RE.search(text)
        msen = STATUS_SENSOR_RE.search(text)
        row = {"wall": now_hms(), "epoch": f"{time.time():.3f}",
               "tag": tag, "phase": phase,
               "sensor": "", "cal": "", "nStart": "", "nStop": "",
               "readFail": "", "ringDrop": "", "gaps": "", "missed": "",
               "jitter": "", "clip": "", "i2cErr": ""}
        if ms:
            row.update({"nStart": ms.group(1), "nStop": ms.group(2),
                        "readFail": ms.group(3), "ringDrop": ms.group(4),
                        "gaps": ms.group(5), "missed": ms.group(6),
                        "jitter": ms.group(7), "clip": ms.group(8),
                        "i2cErr": ms.group(9)})
        if msen:
            row["sensor"] = msen.group(1)
            row["cal"] = f"{msen.group(2)}/{msen.group(3)}"
        self.status_writer.writerow(row)
        self.status_csv.flush()
        self.markers["status_rows"].append(row)
        if ms:
            print(f"[STATUS {tag}] sensor={row['sensor'] or '?'} cal={row['cal'] or '?'} "
                  f"readFail={row['readFail']} 不连续={row['gaps']} I2C错={row['i2cErr']}",
                  flush=True)
        return text

    def dump(self, tag):
        self.sen.reset_input_buffer()
        self.sen.write(b"D")
        text = self.read_for(1.2)
        self.consume(text, f"dump_{tag}", 0, time.time() - self.t_global)
        self.dump_fp.write(f"\n===== [{tag}] wall={now_hms()} =====\n{text}\n")
        self.dump_fp.flush()
        print(f"[DUMP {tag}] 已抓取 @ {now_hms()}", flush=True)

    def close(self):
        for p in self._src_handles.values():
            try:
                p.write(b"z")       # 退出前确保振源全停
            except Exception:
                pass
        self.markers["fault_count"] = len(self.faults)
        with open(self.base + ".markers.json", "w", encoding="utf-8") as fp:
            json.dump(self.markers, fp, ensure_ascii=False, indent=2)
        self.csv_fp.close()
        self.raw.close()
        self.status_fp.close()
        self.status_csv.close()
        self.dump_fp.close()
        for p in self._src_handles.values():
            try:
                p.close()
            except Exception:
                pass
        self.sen.close()
        print(f"\n数据已写入：{self.base}.*", flush=True)


# --------------------------------------------------------------- 各组 ----

def run_p0(s):
    print("\n================ P0 硬件证据 ================", flush=True)
    speak("开始 P0 硬件证据。请先保持张开，静止不动")
    s.wait_state(row_open, 30.0, "p0_wait_open", prompt="等张开/START（cl=0, ang<=10°）")
    s.status("p0_h1_start", "p0_h1_static")
    speak("H1：请保持张开静止六十秒，我会每十秒记录一次状态")
    s.poll("p0_h1_static", 60.0, interval=1.0, status_every=10.0,
           note="H1 静止 60s（合格：全程 I2C错=0 读失败=0 不连续=0）")
    s.status("p0_h1_end", "p0_h1_static")

    speak("H3：请模拟仿生手指弯折，持续三分钟，动作不用很快，反复来回弯就行")
    speak("现在开始")
    s.poll("p0_h3_bend", 180.0, interval=1.0, status_every=30.0,
           note="H3 弯折 3min（合格：无 [FAULT]、I2C错=0）")
    s.status("p0_h3_end", "p0_h3_bend")
    speak("P0 结束，请把手指放回张开位")


def run_p1(s, rounds, orig_cmd):
    print("\n================ P1 核心判据 + A5 ================", flush=True)
    for r in range(1, rounds + 1):
        print(f"\n---------------- 第 {r}/{rounds} 轮 ----------------", flush=True)
        speak(f"第{r}轮。请张开手指，保持静止")
        if not s.wait_state(row_open, 40.0, f"r{r}_open_wait", round_no=r,
                            prompt="A1 准备：等张开/START（cl=0, ang<=10°）"):
            print(f"[轮{r}] 未回到 START，中断。", flush=True)
            break
        s.rebaseline(f"r{r}_open_wait")

        speak(f"第{r}轮，A1 松手：保持张开静止十秒")
        s.poll(f"r{r}_open", 10.0, round_no=r, interval=1.0,
               note="A1 松手（合格：closed=0 ct=0 全 10 条）")

        speak(f"第{r}轮，A2 空握：请闭合但不夹任何东西，保持十秒")
        if not s.wait_state(row_closed, 40.0, f"r{r}_empty_wait", round_no=r,
                            prompt="A2 准备：等闭合（cl=1, ang>=60°）"):
            print(f"[轮{r}] 未检测到闭合，中断。", flush=True)
            break
        s.poll(f"r{r}_empty", 10.0, round_no=r, interval=1.0,
               note="A2 空握（关键：cl=1 且 ct=0 全 10 条，不许误报）")

        speak(f"第{r}轮，请张开")
        if not s.wait_state(row_open, 40.0, f"r{r}_open2_wait", round_no=r,
                            prompt="等张开/START"):
            print(f"[轮{r}] 未回到 START，中断。", flush=True)
            break
        s.rebaseline(f"r{r}_open2_wait")

        speak(f"第{r}轮，A3：请夹住四厘米方块，夹稳")
        if not s.wait_state(row_closed, 40.0, f"r{r}_contact_wait", round_no=r,
                            prompt="A3 准备：等夹住（cl=1, ang>=60°）"):
            print(f"[轮{r}] 未检测到夹住，中断。", flush=True)
            break
        s.send_source(orig_cmd, "old")
        s.poll(f"r{r}_contact_on", 10.0, round_no=r, interval=1.0,
               note="A3 接触+原振子开（合格：ct=1 保持率>=90%）")

        if r == 1:
            speak("T1 工单：保持闭合不动，听我说『抽』就把方块抽走，振源继续开着")
            time.sleep(1.0)
            s.markers["t_pull_epoch"] = round(time.time() + 0.0, 3)
            speak("抽")
            s.poll("r1_t1_pull", 10.0, round_no=1, interval=0.5,
                   note="T1 抽走物体(振源仍开)：记录 ct 退出时间（100~300ms 正常 / >5s 规则过宽）")

        speak(f"第{r}轮，A4：我关振源，请继续夹住，看接触是否退出")
        s.send_source("z", "old")
        s.poll(f"r{r}_exit_off", 5.0, round_no=r, interval=0.25,
               note="A4 关源退出（合格：ct 在 0.5s 内变 0）")

        speak(f"第{r}轮完成。请张开")
        s.wait_state(row_open, 40.0, f"r{r}_after_wait", round_no=r,
                     prompt="等张开/START")
    speak("五轮结束")


def run_dual(s, orig_cmd, new_cmd):
    """两段频带：old 口=原振子(中高频)、new 口=新低速振子(低频)。两个口同时在线，不用换马达。"""
    print("\n================ 双振源两段频带 ================", flush=True)
    s.send_source("z", "old")     # 'z' 两路全停

    speak("双振源第一段：原振子，中高频。请张开手指静止")
    if not s.wait_state(row_open, 40.0, "dual_orig_open_wait",
                        prompt="等张开/START"):
        return
    s.rebaseline("dual_orig_open_wait")
    speak("请夹住四厘米方块")
    if not s.wait_state(row_closed, 40.0, "dual_orig_closed_wait",
                        prompt="等夹住（cl=1, ang>=60°）"):
        return
    s.send_source(orig_cmd, "old")
    s.poll("dual_orig_on", 12.0, interval=1.0,
           note="B-orig 原振子（合格：ct=1 稳定，F 与振源一致 ±5Hz）")
    s.send_source("z", "old")
    s.poll("dual_orig_off", 4.0, interval=0.5, note="原振子关源")

    speak("双振源第二段：新低速振子，低频。请张开手指静止")
    if not s.wait_state(row_open, 40.0, "dual_new_open_wait",
                        prompt="等张开/START"):
        return
    s.rebaseline("dual_new_open_wait")
    speak("请夹住四厘米方块")
    if not s.wait_state(row_closed, 40.0, "dual_new_closed_wait",
                        prompt="等夹住（cl=1, ang>=60°）"):
        return
    s.send_source(new_cmd, "new")
    s.poll("dual_new_on", 12.0, interval=1.0,
           note="B-new 新低速振子（合格：ct=1 稳定，F 与振源一致 ±5Hz）")
    s.send_source("z", "new")
    s.poll("dual_new_off", 4.0, interval=0.5, note="新振子关源")
    speak("双振源结束")


def run_dump(s):
    print("\n================ D 命令实板样例 ================", flush=True)
    speak("最后是 D 命令样例。请张开手指静止")
    if s.wait_state(row_open, 40.0, "dump_open_wait", prompt="等张开/START"):
        s.dump("empty_open")
    speak("请空握，保持闭合十秒")
    if s.wait_state(row_closed, 40.0, "dump_empty_wait", prompt="等闭合（cl=1）"):
        time.sleep(2.0)
        s.dump("empty_grip")
    speak("好，D 命令样例结束")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sensor", default="COM9")
    ap.add_argument("--source", default="COM7",
                    help="振源口（默认 COM7）")
    ap.add_argument("--source-new", default="",
                    help="新振子口；留空（默认）=与 --source 同口，用通道选择字符切换")
    ap.add_argument("--old-sel", default="a",
                    help="旧/原振子通道选择字符（同口时用，默认 a=GPIO3）")
    ap.add_argument("--new-sel", default="b",
                    help="新低速振子通道选择字符（同口时用，默认 b=GPIO4）")
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--groups", default="p0,p1,dual,dump")
    ap.add_argument("--rounds", type=int, default=5)
    ap.add_argument("--orig-cmd", default="h", help="原振子指令（默认 h=100%%≈178Hz）")
    ap.add_argument("--new-cmd", default="p", help="新低速振子指令（默认 p=55%%≈80Hz）")
    args = ap.parse_args()

    groups = {g.strip() for g in args.groups.split(",") if g.strip()}
    print(f"T004 分组：{sorted(groups)}  轮数={args.rounds}  "
          f"旧/原振子={args.source}#{args.old_sel}({args.orig_cmd!r})  "
          f"新振子={args.source_new or args.source}#{args.new_sel}({args.new_cmd!r})", flush=True)

    s = Session(args.sensor, args.source, args.source_new,
                args.old_sel, args.new_sel, args.out_dir)
    try:
        s.boot()
        if "p0" in groups:
            run_p0(s)
        if "p1" in groups:
            run_p1(s, args.rounds, args.orig_cmd)
        if "dual" in groups:
            run_dual(s, args.orig_cmd, args.new_cmd)
        if "dump" in groups:
            run_dump(s)
    except KeyboardInterrupt:
        print("\n[中断] 收到 Ctrl-C，正在保存已采数据 ...", flush=True)
    finally:
        s.close()
    print("全部完成。", flush=True)


if __name__ == "__main__":
    main()
