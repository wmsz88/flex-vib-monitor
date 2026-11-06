#!/usr/bin/env python3
# -*- coding: utf-8 -*-
r"""T005 实板：D2 关源追中间量 + 边界端点/提档补测 + D-contact + START 基线。

分组（--groups）：
  d2    关源后连续敲 D（≥2Hz）追中间量，重复 3 次
  sweep 4cm 方块夹持下，旧/新两路逐档扫描（占空比 -> F/A/ct），覆盖 E-80/E-180/E-mid/P-new
  dcon  接触稳定时敲 D 一次（补 T004 缺项）
  all   以上全跑

默认端口：COM9=检测模块；COM7=双路振源板（a=GPIO3旧, b=GPIO4新, z=两路停）。
用法：
  python -X utf8 run_t005.py --sensor COM9 --source COM7 --out-dir "D:\大学\课程\工科赛\代码\vibmon_v5\data\t005_20261102"
"""
import argparse, base64, csv, json, os, re, subprocess, time
from datetime import datetime
import serial

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
SWEEP_FIELDS = ["wall","epoch","t_global_s","phase","elapsed_s","Freq_Hz","Amp_mg",
                "FlatDist_mg","CloseAngle_deg","RefSwing_mg","vib","contact",
                "gripNoObject","closed","gyroW","clipLimited","axis","ok"]
DUMP_FIELDS = ["wall","epoch","phase","t_ms","closed","flatDist","flatDistValid",
               "closeAngleDeg","angleValid","condPosture","condAmp","condFreq",
               "ct","ctValid","vib","freqHz","freqValid","ampMg","ampValid",
               "ssHz","ssValid","grip","gyroW","ax","cliplim","ok"]

def now_hms(): return datetime.now().strftime("%H:%M:%S.%f")[:-3]

def speak(text):
    print(f"[SPEAK] {text}", flush=True)
    ps = ("Add-Type -AssemblyName System.Speech; "
          "$s = New-Object System.Speech.Synthesis.SpeechSynthesizer; "
          "try { $s.SelectVoice('Microsoft Huihui') } catch {}; $s.Rate = 0; "
          "$s.Speak('" + text.replace("'", "''") + "')")
    enc = base64.b64encode(ps.encode("utf-16le")).decode("ascii")
    subprocess.run(["powershell", "-NoProfile", "-EncodedCommand", enc], check=False)

def parse_sweep(m):
    return {"Freq_Hz": "" if m["f"]=="?" else m["f"],
            "Amp_mg": "" if m["a"]=="?" else m["a"],
            "FlatDist_mg": "" if m["d"]=="?" else m["d"],
            "CloseAngle_deg": "" if m["ang"] in (None,"?") else m["ang"],
            "RefSwing_mg": "" if m["tr"]=="?" else m["tr"],
            "vib": m["vib"], "contact": m["ct"], "gripNoObject": m["grip"],
            "closed": m["cl"], "gyroW": "" if m["w"]=="?" else m["w"],
            "clipLimited": m["clim"], "axis": m["ax"], "ok": m["ok"]}

def row_open(r):
    a = r.get("CloseAngle_deg","")
    try: return r.get("closed")=="0" and a not in ("","?") and float(a)<=10.0
    except ValueError: return False

def row_closed(r):
    a = r.get("CloseAngle_deg","")
    try: return r.get("closed")=="1" and a not in ("","?") and float(a)>=60.0
    except ValueError: return False

class Sess:
    def __init__(self, sensor, source, out_dir):
        os.makedirs(out_dir, exist_ok=True)
        self.base = os.path.join(os.path.abspath(out_dir), "run")
        self.csv_fp = open(self.base+".csv","w",newline="",encoding="utf-8-sig")
        self.writer = csv.DictWriter(self.csv_fp, fieldnames=SWEEP_FIELDS); self.writer.writeheader(); self.csv_fp.flush()
        self.raw = open(self.base+".raw.txt","w",encoding="utf-8",errors="replace")
        self.dump_fp = open(self.base+".dump.csv","w",newline="",encoding="utf-8-sig")
        self.dump_w = csv.DictWriter(self.dump_fp, fieldnames=DUMP_FIELDS); self.dump_w.writeheader(); self.dump_fp.flush()
        self.dump_txt = open(self.base+".dump.txt","w",encoding="utf-8")
        self.markers = {"phases":[], "src_events":[], "faults":[]}
        self.src = serial.Serial(source,115200,timeout=0.2)
        self.sen = serial.Serial(sensor,115200,timeout=0.2)
        time.sleep(0.8); self.src.reset_input_buffer(); self.sen.reset_input_buffer()
        self.src.write(b"z"); time.sleep(0.3)
        self.tg = time.time()
    def read_for(self, s):
        buf=[]; t0=time.time()
        while time.time()-t0<s:
            c=self.sen.read(4096)
            if c: buf.append(c.decode("utf-8","replace"))
            else: time.sleep(0.01)
        return "".join(buf)
    def consume(self, text, phase, el):
        if not text: return
        self.raw.write(text); self.raw.flush()
        for ln in text.splitlines():
            if "[FAULT]" in ln:
                self.markers["faults"].append({"wall":now_hms(),"phase":phase,"line":ln.strip()})
                print(f"[!! FAULT] {phase}: {ln.strip()}", flush=True)
        for m in SWEEP_RE.finditer(text):
            r={"wall":now_hms(),"epoch":f"{time.time():.3f}","t_global_s":f"{time.time()-self.tg:.1f}",
               "phase":phase,"elapsed_s":f"{el:.2f}"}; r.update(parse_sweep(m))
            self.writer.writerow(r); self.csv_fp.flush()
    def boot(self, timeout=20.0):
        speak("模块复位中，请保持张开静止")
        dl=time.time()+timeout
        while time.time()<dl:
            self.sen.write(b"w"); t=self.read_for(0.5)
            if "#SWEEP" in t:
                self.consume(t,"boot",time.time()-self.tg); self.sen.write(b"0"); time.sleep(0.3)
                self.sen.reset_input_buffer(); print("模块就绪。",flush=True); return True
            self.consume(t,"boot",time.time()-self.tg)
        raise SystemExit("模块 20s 内无 #SWEEP")
    def send(self, cmd, which="old"):
        self.src.reset_input_buffer()
        sel = b"a" if which=="old" else b"b"
        if cmd=="z": self.src.write(b"z")
        else: self.src.write(sel+cmd.encode())
        self.markers["src_events"].append({"wall":now_hms(),"which":which,"cmd":cmd})
        print(f"[SRC {which}] {cmd!r}", flush=True)
    def status(self, tag, phase="p0"):
        self.sen.reset_input_buffer(); self.sen.write(b"S"); t=self.read_for(1.2)
        self.consume(t,phase,time.time()-self.tg)
        open(self.base+".__status.tmp","a",encoding="utf-8").write(f"\n== {tag} ==\n{t}")
        return t
    def poll(self, phase, secs, interval=1.0, note=""):
        if note: print(f"\n[{phase}] {note}", flush=True)
        t0=time.time(); self.markers["phases"].append({"phase":phase,"start":round(t0,3)})
        nxt=0.0
        while time.time()-t0<secs:
            el=time.time()-t0
            if el>=nxt: self.sen.write(b"w"); nxt+=interval
            c=self.sen.read(4096)
            if c: self.consume(c.decode("utf-8","replace"),phase,el)
            else: time.sleep(0.01)
        self.markers["phases"][-1]["end"]=round(time.time(),3)
    def wait(self, pred, timeout, phase, prompt=""):
        if prompt: print(f"\n[{phase}] {prompt}", flush=True)
        t0=time.time(); self.markers["phases"].append({"phase":phase,"start":round(t0,3)})
        nxt=0.0
        while time.time()-t0<timeout:
            el=time.time()-t0
            if el>=nxt: self.sen.write(b"w"); nxt+=0.5
            c=self.sen.read(4096)
            if not c: time.sleep(0.01); continue
            txt=c.decode("utf-8","replace"); hit=False
            for m in SWEEP_RE.finditer(txt):
                if pred(parse_sweep(m)): hit=True
            self.consume(txt,phase,el)
            if hit:
                print(f"  到位 {phase} @ {el:.1f}s", flush=True); return True
        print(f"  !! 超时 {phase}", flush=True); return False
    def rebase(self, phase):
        self.sen.write(b"t"); time.sleep(1.2); self.consume(self.read_for(0.6),phase,0.0)
    def start_baseline(self, phase):
        self.sen.write(b"P"); t=self.read_for(2.5); self.consume(t,phase,time.time()-self.tg)
        open(self.base+".__start.txt","w",encoding="utf-8").write(t)
        print("[START基线] 'P' 已发送", flush=True)
    def dump_series(self, phase, secs, interval=0.4):
        t0=time.time(); self.markers["phases"].append({"phase":phase,"start":round(t0,3)})
        nxt=0.0; n=0
        while time.time()-t0<secs:
            el=time.time()-t0
            if el>=nxt:
                self.sen.write(b"D"); nxt+=interval
            c=self.sen.read(4096)
            if not c: time.sleep(0.01); continue
            txt=c.decode("utf-8","replace"); self.consume(txt,phase,el)
            for ln in txt.splitlines():
                if ln.startswith("#DUMP,"):
                    parts=ln.split(",")
                    if len(parts)==24:
                        row={"wall":now_hms(),"epoch":f"{time.time():.3f}","phase":phase}
                        for k,v in zip(DUMP_FIELDS[3:],parts[1:]): row[k]=v
                        self.dump_w.writerow(row); self.dump_fp.flush(); n+=1
        self.markers["phases"][-1]["end"]=round(time.time(),3)
        print(f"[DUMP {phase}] {n} 行", flush=True)
    def dump_once(self, phase):
        self.sen.reset_input_buffer(); self.sen.write(b"D"); t=self.read_for(1.0)
        self.consume(t,phase,time.time()-self.tg)
        self.dump_txt.write(f"\n== {phase} {now_hms()} ==\n{t}\n"); self.dump_txt.flush()
    def close(self):
        try: self.src.write(b"z")
        except Exception: pass
        with open(self.base+".markers.json","w",encoding="utf-8") as fp:
            json.dump(self.markers,fp,ensure_ascii=False,indent=2)
        for f in (self.csv_fp,self.raw,self.dump_fp,self.dump_txt): f.close()
        self.src.close(); self.sen.close()
        print(f"\n数据 -> {self.base}.*", flush=True)

DUTIES = [("p","55"),("6","60"),("7","70"),("8","80"),("9","90"),("h","100")]

def run_d2(s):
    print("\n======== D2 关源追中间量（3 次） ========", flush=True)
    for k in (1,2,3):
        speak(f"D2 第{k}次：请张开手指静止")
        if not s.wait(row_open,40,f"d2{k}_open"): break
        s.rebase(f"d2{k}_open")
        speak(f"第{k}次：请夹住四厘米方块")
        if not s.wait(row_closed,40,f"d2{k}_closed"): break
        s.send("h","old")
        s.poll(f"d2{k}_on",6,1.0,note=f"D2#{k} 接触稳定(原振子)")
        speak(f"第{k}次：我关振源，保持夹住，连续追状态 20 秒")
        s.send("z","old")
        s.dump_series(f"d2{k}_off",20,0.4)
        speak(f"第{k}次完成，请张开")
        s.wait(row_open,40,f"d2{k}_after")

def run_sweep(s):
    print("\n======== 补测扫描：旧/新两路逐档 ========", flush=True)
    speak("补测：请张开手指静止")
    if not s.wait(row_open,40,"sw_open"): return
    s.rebase("sw_open")
    speak("请夹住四厘米方块，夹稳后整段保持不动，直到我说结束")
    if not s.wait(row_closed,40,"sw_closed"): return
    for which,label in (("old","旧/原振子 GPIO3"),("new","新低速振子 GPIO4")):
        speak(f"现在测{label}，请继续夹住")
        for cmd,duty in DUTIES:
            s.send(cmd,which)
            s.poll(f"sw_{which}_{duty}",8,1.0)
        s.send("z",which)
        s.poll(f"sw_{which}_off",3,0.5)
    speak("补测扫描结束，请松开")
    s.wait(row_open,40,"sw_after")
    speak("请空握，保持闭合")
    if s.wait(row_closed,40,"dcon_closed"):
        time.sleep(1.5); s.dump_once("dcon_empty")
    speak("补测全部结束")

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--sensor",default="COM9")
    ap.add_argument("--source",default="COM7")
    ap.add_argument("--out-dir",required=True)
    ap.add_argument("--groups",default="d2,sweep")
    a=ap.parse_args()
    g={x.strip() for x in a.groups.split(",") if x.strip()}
    print(f"T005 分组 {sorted(g)}", flush=True)
    s=Sess(a.sensor,a.source,a.out_dir)
    try:
        s.boot()
        speak("开场：请张开手指，先采 START 基线")
        if s.wait(row_open,40,"start_open"): s.start_baseline("start")
        if "d2" in g or "all" in g: run_d2(s)
        if "sweep" in g or "all" in g: run_sweep(s)
    except KeyboardInterrupt:
        print("\n[中断] 保存已采数据", flush=True)
    finally:
        s.close()
    print("DONE", flush=True)

if __name__ == "__main__":
    main()
