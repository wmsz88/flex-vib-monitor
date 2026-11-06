#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""T008 P3 平台场景模拟：控振源(COM7) + 收比赛口 $VD(COM4)，看 VIB/CT/SS/FREQ/AMP 语义。"""
import argparse, base64, csv, os, subprocess, time
import serial

def speak(t):
    print(f"[SPEAK] {t}", flush=True)
    ps=("Add-Type -AssemblyName System.Speech; $s = New-Object System.Speech.Synthesis.SpeechSynthesizer; "
        "try { $s.SelectVoice('Microsoft Huihui') } catch {}; $s.Rate=0; $s.Speak('"+t.replace("'","''")+"')")
    subprocess.run(["powershell","-NoProfile","-EncodedCommand",base64.b64encode(ps.encode("utf-16le")).decode("ascii")],check=False)

def xor_chk(text):
    x=0
    for b in text.encode("ascii","replace"): x^=b
    return f"{x:02X}"

def parse(core):
    parts=core.split(",")
    if parts[0]!="$VD": return None
    body,chk=core.rsplit(",",1)
    d={"seq":parts[1],"freq":None,"amp":None,"vib":None,"ct":None,"cl":"","ss":None,"valid":None,"chk_ok":xor_chk(body)==chk.upper()}
    try: d["freq"]=int(parts[2])/10.0
    except: pass
    try: d["amp"]=int(parts[3])
    except: pass
    d["vib"]=parts[4]; d["ct"]=parts[5]
    # parts[6] 可能是 CL(1位) 或 SS(3位)
    p6=parts[6]
    if len(p6)==1 and parts[7].isdigit() and len(parts[7])==3:
        d["cl"]=p6; ss=parts[7]
    else:
        ss=p6
    try: d["ss"]=int(ss)/100.0
    except: pass
    d["valid"]=parts[-2]
    return d

def capture(comp, phase, secs, rows, fp, src=None, on_cmd=None, on_after=None):
    if on_cmd and src:
        comp.reset_input_buffer()   # 丢掉等待期间积压的帧，保证从指令时刻开始计时
        src.reset_input_buffer(); src.write(on_cmd); print(f"[SRC] {phase} -> {on_cmd!r}", flush=True)
    t0=time.time(); buf=""; n=0
    while time.time()-t0 < secs:
        if on_after and time.time()-t0 >= on_after:
            src.reset_input_buffer(); src.write(b"z"); print(f"[SRC] {phase} -> 'z'", flush=True); on_after=None
        c=comp.read(4096)
        if not c: continue
        buf+=c.decode("ascii","replace")
        while "\n" in buf:
            line,buf=buf.split("\n",1)
            if not line.startswith("$VD,"): continue
            core=line.rstrip("\r"); d=parse(core)
            if not d: continue
            n+=1
            r={"wall":time.strftime("%H:%M:%S"),"epoch":f"{time.time():.3f}","phase":phase}
            r.update(d)
            rows.append(r); fp.writerow(r)
    print(f"[{phase}] {n} 帧", flush=True)
    return n

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--comp",default="COM4"); ap.add_argument("--source",default="COM7")
    ap.add_argument("--out-dir",required=True)
    a=ap.parse_args()
    os.makedirs(a.out_dir,exist_ok=True)
    base=os.path.join(a.out_dir,"scene")
    comp=serial.Serial(a.comp,115200,timeout=0.3)
    src=serial.Serial(a.source,115200,timeout=0.3)
    time.sleep(0.5); src.reset_input_buffer(); src.write(b"z"); time.sleep(0.3)
    fp=open(base+".csv","w",newline="",encoding="utf-8-sig")
    w=csv.DictWriter(fp,fieldnames=["wall","epoch","phase","seq","freq","amp","vib","ct","cl","ss","valid","chk_ok"])
    w.writeheader(); fp.flush(); rows=[]
    try:
        speak("场景1：请夹住四厘米方块，振源将开启三十秒")
        time.sleep(8)                       # 留时间给人夹好
        capture(comp,"s1_contact",30.0,rows,w,src,on_cmd=b"bh")   # b=GPIO4原振子, h=100%
        speak("场景2：我关振源，请继续夹住十秒，观察接触退出")
        capture(comp,"s2_off",10.0,rows,w,src,on_cmd=b"z")
        speak("场景3：启停1赫兹档，请继续夹住三十秒")
        time.sleep(5)
        capture(comp,"s3_cycle",30.0,rows,w,src,on_cmd=b"bT1")   # 原振子 1Hz 通断
        speak("场景结束，请松开")
    finally:
        try: src.write(b"z")
        except Exception: pass
        fp.close(); comp.close(); src.close()
    # 汇总
    import statistics
    def med(vals):
        vals=[v for v in vals if v is not None]
        return statistics.median(vals) if vals else None
    print("\n==== 场景汇总 ====")
    for ph in ("s1_contact","s2_off","s3_cycle"):
        seg=[r for r in rows if r["phase"]==ph]
        if not seg: continue
        F=med([r["freq"] for r in seg]); A=med([r["amp"] for r in seg])
        vib=sum(1 for r in seg if r["vib"]=="1"); ct=sum(1 for r in seg if r["ct"]=="1")
        ssvals=sorted({r["ss"] for r in seg if r["ss"]})
        bad=sum(1 for r in seg if not r["chk_ok"])
        print(f"{ph}: n={len(seg)} Fmed={F} Amed={A} VIB1={vib}/{len(seg)} CT1={ct}/{len(seg)} SS值集={ssvals} 校验错={bad}")
        # 首个 ct=0（仅 s2）
        if ph=="s2_off":
            for i,r in enumerate(seg):
                if r["ct"]=="0":
                    print(f"   s2 首个 CT=0 在第 {i+1} 帧 (~{(i)*0.02:.2f}s 量级)"); break
    print(f"\n数据 -> {base}.*", flush=True)

if __name__=="__main__":
    main()
