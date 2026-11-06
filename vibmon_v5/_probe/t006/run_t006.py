#!/usr/bin/env python3
# -*- coding: utf-8 -*-
r"""T006 实板：通道标签对调确认 + E-180 端点排查（纯硬件侧，不改码/不改阈值）。

阶段：
  p1  通道对调确认：两块马达各取 55%/100% 两档各 8s，读 F/A/ct（看频段跟着通道还是跟着标签）
  p2c E-180 分支C：取最高档（实测约165Hz）夹持 10s 正式数据（F/A/ct 逐条）
用法：
  python -X utf8 run_t006.py --sensor COM9 --source COM7 --out-dir "D:\大学\课程\工科赛\代码\vibmon_v5\data\t006_20261102"
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
FIELDS = ["wall","epoch","t_global_s","phase","elapsed_s","Freq_Hz","Amp_mg",
          "FlatDist_mg","CloseAngle_deg","RefSwing_mg","vib","contact",
          "gripNoObject","closed","gyroW","clipLimited","axis","ok"]

def now_hms(): return datetime.now().strftime("%H:%M:%S.%f")[:-3]
def speak(t):
    print(f"[SPEAK] {t}", flush=True)
    ps=("Add-Type -AssemblyName System.Speech; $s = New-Object System.Speech.Synthesis.SpeechSynthesizer; "
        "try { $s.SelectVoice('Microsoft Huihui') } catch {}; $s.Rate=0; $s.Speak('"+t.replace("'","''")+"')")
    subprocess.run(["powershell","-NoProfile","-EncodedCommand",base64.b64encode(ps.encode("utf-16le")).decode("ascii")],check=False)
def ps(m):
    return {"Freq_Hz":"" if m["f"]=="?" else m["f"],"Amp_mg":"" if m["a"]=="?" else m["a"],
            "FlatDist_mg":"" if m["d"]=="?" else m["d"],"CloseAngle_deg":"" if m["ang"] in (None,"?") else m["ang"],
            "RefSwing_mg":"" if m["tr"]=="?" else m["tr"],"vib":m["vib"],"contact":m["ct"],"gripNoObject":m["grip"],
            "closed":m["cl"],"gyroW":"" if m["w"]=="?" else m["w"],"clipLimited":m["clim"],"axis":m["ax"],"ok":m["ok"]}
def is_open(r):
    # 角度有效时要求 ang<=10；角度失效（陀螺削顶/超量程）时只看 closed 位。
    a=r.get("CloseAngle_deg","")
    if a in ("","?"):
        return r.get("closed")=="0"
    try: return r.get("closed")=="0" and float(a)<=10.0
    except ValueError: return r.get("closed")=="0"
def is_closed(r):
    a=r.get("CloseAngle_deg","")
    if a in ("","?"):
        return r.get("closed")=="1"
    try: return r.get("closed")=="1" and float(a)>=60.0
    except ValueError: return r.get("closed")=="1"

class S:
    def __init__(self,sensor,source,out):
        os.makedirs(out,exist_ok=True); self.base=os.path.join(os.path.abspath(out),"run")
        self.csv=open(self.base+".csv","w",newline="",encoding="utf-8-sig")
        self.w=csv.DictWriter(self.csv,fieldnames=FIELDS); self.w.writeheader(); self.csv.flush()
        self.raw=open(self.base+".raw.txt","w",encoding="utf-8",errors="replace")
        self.markers={"phases":[],"src":[]}
        self.src=serial.Serial(source,115200,timeout=0.2); self.sen=serial.Serial(sensor,115200,timeout=0.2)
        time.sleep(0.8); self.src.reset_input_buffer(); self.sen.reset_input_buffer(); self.src.write(b"z"); time.sleep(0.3)
        self.tg=time.time()
    def rd(self,s):
        b=[]; t0=time.time()
        while time.time()-t0<s:
            c=self.sen.read(4096); b.append(c.decode("utf-8","replace")) if c else time.sleep(0.01)
        return "".join(b)
    def con(self,t,ph,el):
        if not t: return
        self.raw.write(t); self.raw.flush()
        for m in SWEEP_RE.finditer(t):
            r={"wall":now_hms(),"epoch":f"{time.time():.3f}","t_global_s":f"{time.time()-self.tg:.1f}",
               "phase":ph,"elapsed_s":f"{el:.2f}"}; r.update(ps(m)); self.w.writerow(r); self.csv.flush()
    def boot(self,tl=20.0):
        speak("模块复位，请保持张开静止")
        dl=time.time()+tl
        while time.time()<dl:
            self.sen.write(b"w"); t=self.rd(0.5)
            if "#SWEEP" in t:
                self.con(t,"boot",time.time()-self.tg); self.sen.write(b"0"); time.sleep(0.3); self.sen.reset_input_buffer(); print("就绪",flush=True); return True
            self.con(t,"boot",time.time()-self.tg)
        raise SystemExit("无 #SWEEP")
    def src_cmd(self,cmd,which="old"):
        self.src.reset_input_buffer()
        self.src.write(b"z" if cmd=="z" else (b"a" if which=="old" else b"b")+cmd.encode())
        self.markers["src"].append({"wall":now_hms(),"which":which,"cmd":cmd}); print(f"[SRC {which}] {cmd!r}",flush=True)
    def poll(self,ph,s,it=1.0,note=""):
        if note: print(f"\n[{ph}] {note}",flush=True)
        t0=time.time(); self.markers["phases"].append({"phase":ph,"start":round(t0,3)}); nxt=0.0
        while time.time()-t0<s:
            el=time.time()-t0
            if el>=nxt: self.sen.write(b"w"); nxt+=it
            c=self.sen.read(4096)
            if c: self.con(c.decode("utf-8","replace"),ph,el)
            else: time.sleep(0.01)
        self.markers["phases"][-1]["end"]=round(time.time(),3)
    def wait(self,pred,tl,ph,pr=""):
        if pr: print(f"\n[{ph}] {pr}",flush=True)
        t0=time.time(); self.markers["phases"].append({"phase":ph,"start":round(t0,3)}); nxt=0.0
        while time.time()-t0<tl:
            el=time.time()-t0
            if el>=nxt: self.sen.write(b"w"); nxt+=0.5
            c=self.sen.read(4096)
            if not c: time.sleep(0.01); continue
            t=c.decode("utf-8","replace"); hit=any(pred(ps(m)) for m in SWEEP_RE.finditer(t))
            self.con(t,ph,el)
            if hit: print(f"  到位 {ph} @ {el:.1f}s",flush=True); return True
        print(f"  !! 超时 {ph}",flush=True); return False
    def rebase(self,ph): self.sen.write(b"t"); time.sleep(1.2); self.con(self.rd(0.6),ph,0.0)
    def startbase(self):
        self.sen.write(b"P"); t=self.rd(2.5); self.con(t,"start",time.time()-self.tg)
        open(self.base+".__start.txt","w",encoding="utf-8").write(t); print("[START基线] P",flush=True)
    def close(self):
        try: self.src.write(b"z")
        except Exception: pass
        json.dump(self.markers,open(self.base+".markers.json","w",encoding="utf-8"),ensure_ascii=False,indent=2)
        for f in (self.csv,self.raw): f.close()
        self.src.close(); self.sen.close(); print(f"\n数据 -> {self.base}.*",flush=True)

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--sensor",default="COM9"); ap.add_argument("--source",default="COM7")
    ap.add_argument("--out-dir",required=True); ap.add_argument("--groups",default="p1,p2c")
    a=ap.parse_args(); g={x.strip() for x in a.groups.split(",") if x.strip()}
    print(f"T006 分组 {sorted(g)}",flush=True)
    s=S(a.sensor,a.source,a.out_dir)
    try:
        s.boot()
        speak("请张开手指，先采 START 基线")
        if not s.wait(is_open,15,"start_open"):
            speak("检测到当前不是张开位。请把手指摊开到 START 位、保持静止，我正在重采姿态基线")
            time.sleep(1.0)
            s.startbase()          # P：重采 START 姿态基线
            s.rebase("rebase_fix")  # t：重采平放基准
            s.wait(is_open,30,"start_open2")
        else:
            s.startbase()
        if "p1" in g or "all" in g:
            print("\n======== P1 通道对调确认 ========",flush=True)
            speak("通道确认：请夹住四厘米方块并整段保持")
            if s.wait(is_closed,40,"p1_closed"):
                for which in ("old","new"):
                    for cmd,duty in (("p","55"),("h","100")):
                        s.src_cmd(cmd,which); s.poll(f"p1_{which}_{duty}",8,1.0)
                    s.src_cmd("z",which); s.poll(f"p1_{which}_off",2,0.5)
                speak("通道确认结束，请松开")
                s.wait(is_open,40,"p1_after")
        if "p2c" in g or "all" in g:
            print("\n======== P2C 最高档正式数据(约165Hz) ========",flush=True)
            speak("最高档正式数据：请张开")
            if s.wait(is_open,40,"p2c_open"):
                s.rebase("p2c_open")
                speak("请夹住四厘米方块")
                if s.wait(is_closed,40,"p2c_closed"):
                    s.src_cmd("h","new")   # GPIO4 最高档(实测约166Hz)
                    s.poll("p2c_165hz",10,1.0,note="165Hz 档正式数据（夹持10s）")
                    s.src_cmd("z","new"); s.poll("p2c_off",3,0.5)
                    speak("最高档数据结束")
    except KeyboardInterrupt:
        print("\n[中断] 保存",flush=True)
    finally:
        s.close()
    print("DONE",flush=True)

if __name__ == "__main__":
    main()
