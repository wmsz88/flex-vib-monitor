# T004 实板检验第一轮 — 操作手册与脚本

> 对应任务卡：`规划/tasks/T004-实板检验第一轮.md`
> 模式：**一轮跑完** —— 脚本后台连续采集 + 中文语音播报逐步提示 + 人按提示做动作。
> 不掐秒表、不按回车。

---

## 1. 脚本清单

| 脚本 | 作用 |
|---|---|
| `run_t004.py` | 主流程：P0 硬件证据 + P1 五轮(A5) + T1 工单 + 双振源 + D 命令样例，语音引导 |
| `analyze_t004.py` | 判读：读 `run.csv/status.csv/markers.json`，按 T004 验收标准出逐项 PASS/FAIL 报告 |
| `grab_frames.py` | 抓比赛口 `$VD` 帧（补 T001 遗留，需一根 USB-TTL 接比赛口） |

---

## 2. 接线与串口分工

| 口 | 设备 | 用途 |
|---|---|---|
| `COM9` | 检测模块 vibmon_v5（USB-CDC 调试口） | 脚本主体：`w`/`S`/`D`/`t` |
| `COM7` | 双路振源 vibsource_dual | `a`=选 GPIO3(旧/原振子) / `b`=选 GPIO4(新低速振子) / `h` `p` `z` 等 |
| `COMx` | USB-TTL 接**比赛口**(GPIO21) | 只给抓 `$VD` 帧用（可选） |

双路振源固件在 `C:\Users\Lenovo\Desktop\STM32project\vibsource_esp32_dual\`（已烧到 COM7）：
`a`=选通道A(GPIO3，旧)，`b`=选通道B(GPIO4，新)，之后的 `h/p/0~9/+/-` 作用于当前通道，`z`=两路全停，`q`=查状态。
脚本默认单口 COM7（`--source COM7`，`--old-sel a`，`--new-sel b`）；
若改用两个独立 COM 口：给 `--source-new` 指新口、并 `--old-sel "" --new-sel ""`。

比赛口 = `Serial1`，引脚默认 TX=GPIO21 / RX=GPIO20（`config.h`：`PIN_COMP_TX=21`）。

---

## 3. 怎么跑（一次跑完）

```powershell
cd "D:\大学\课程\工科赛\代码\vibmon_v5\_probe\t004"
python -X utf8 run_t004.py --sensor COM9 --source COM7 `
    --out-dir "D:\大学\课程\工科赛\代码\vibmon_v5\data\t004_20261102\run1"
```

跑完立刻判读：

```powershell
python -X utf8 analyze_t004.py --dir "D:\大学\课程\工科赛\代码\vibmon_v5\data\t004_20261102\run1" `
    --out "D:\大学\课程\工科赛\代码\vibmon_v5\data\t004_20261102\run1\判读.md"
```

某一组没跑成、想单独补跑（失败重跑不受影响，写不同 `--out-dir`）：

```powershell
python -X utf8 run_t004.py --sensor COM9 --source COM7 --groups p0   --out-dir ...\run_p0
python -X utf8 run_t004.py --sensor COM9 --source COM7 --groups p1   --out-dir ...\run_p1
python -X utf8 run_t004.py --sensor COM9 --source COM7 --groups dual --out-dir ...\run_dual
python -X utf8 run_t004.py --sensor COM9 --source COM7 --groups dump --out-dir ...\run_dump
```

抓比赛帧（单独一根 USB-TTL，抓 10 帧）：

```powershell
python -X utf8 grab_frames.py --port COM12 --count 10 --out ...\run1\frames.txt
```

---

## 4. 语音引导脚本（脚本会自动播报，这里供人先看一遍）

### P0 硬件证据（约 4 分钟）
1. 「开始 P0 硬件证据。请先保持张开，静止不动」
2. 「H1：请保持张开静止六十秒，我会每十秒记录一次状态」
3. 「H3：请模拟仿生手指弯折，持续三分钟…现在开始」

### P1 五轮（A5）+ T1（每轮约 1 分钟，共约 5~6 分钟）
每轮：张开静止 →（重采基准）→ **A1 松手 10s** → **A2 空握 10s** → 张开 →（重采基准）
→ **A3 夹住 4cm 方块 + 振源开 10s** →（第 1 轮插入 **T1：听「抽」抽走方块、振源继续开 10s**）
→ **A4 关振源、看接触退出 5s（0.25s 分辨率）** → 张开。

### 双振源（约 2 分钟 + 换马达等待）
1. 「双振源第一段：原振子，中高频。请张开手指静止」
2. 张开 → 重采基准 → 夹住 → 发 `a`+`h` 让 **GPIO3 旧/原振子**转 12s（**B-orig**）→ `z`
3. 「双振源第二段：新低速振子，低频。请张开手指静止」
4. 张开 → 重采基准 → 夹住 → 发 `b`+`p` 让 **GPIO4 新低速振子**转 12s（**B-new**）→ `z`

两路都在同一块 COM7 板上，脚本按通道字符切换，**不用换马达、不用等**。

### D 命令样例
张开敲 `D`（空握前）→ 空握 → 敲 `D`（空握态）→ 结束。

---

## 5. 产物与判读口径

`run.csv`（每行一个 `#SWEEP` 采样点，列含 `round/phase/elapsed_s` + 全字段）；
`run.status.csv`（每次 `S` 解析出的 `readFail/不连续/I2C错/传感器/标定`）；
`run.dump.txt`（`#DUMPH` 表头 + `#DUMP` 行）；`run.markers.json`（phase 起止、振源指令、`t_pull_epoch`、FAULT）。

`analyze_t004.py` 逐项对齐任务卡验收标准：

| 检验项 | PASS 口径 |
|---|---|
| H1 | p0_h1_static 段内 `读失败`/`I2C错` **增量 Δ=0**（`不连续` 是上电累计量，看增量） |
| H3 | p0_h3_bend 内无 `[FAULT]`，`S` 快照无 `传感器 故障` |
| A1 | 每轮 opened 段 `closed=0 且 ct=0` 全部成立 |
| A2 | 每轮 empty 段 `closed=1 且 ct=0` 全部成立（不许误报） |
| A3 | 每轮 contact_on 段 `ct=1` 占比 ≥ 90% |
| A4 | 每轮关源后首次 `ct=0` ≤ 0.5s |
| T1 | r1 抽走后 `ct` 退出 ≤ 5s（100~300ms=正常；>5s=规则过宽，开返工卡） |
| A5 | ≥5 轮且全部通过 |
| 双振源 | B-orig/B-new 各自 `ct=1` 占比 ≥ 90%，`F` 中位落在对应频带 |
| dump | 抓到 `#DUMP` 行 |

---

## 6. 已知注意点

- **开串口会复位模块**：脚本启动后先等标定（语音提示保持张开静止），别在标定期间动作，否则 `ang` 参考点错、`ct` 恒 0。
- **T1 的抽走时刻有人反应延迟**：脚本在播报「抽」的瞬间记 `t_pull_epoch`，判读的退出时间是「从播报到首次 ct=0」，含人反应（约 0.2~0.4s）。判读带只看 100~300ms 与 >5s 两档，受此影响可接受；介于中间档的需结合 `run.csv` 波形人工复核。
- **振源两路**：默认同一块双路板（`COM7`，`a`=GPIO3 旧、`b`=GPIO4 新）。若旧/新接反了，把 `--old-sel`/`--new-sel` 对调即可（改的是软件选的通道，不用重接线）。要换成两个独立 COM 口：`--source-new COMx --old-sel "" --new-sel ""`。
- **H1/H3 的 `不连续` 是上电累计量**：启动+标定期就已攒到数百，判读看**段内增量**（判读脚本按 Δ 判定）。
- **`$VD` 不在 USB 口**：抓帧要用第二根 USB-TTL 接比赛口 GPIO21（见 `grab_frames.py` 头注）。
- 现场发现代码问题**不要当场改**：写进 `结果/RT004` 的「异常与未解决项」，由规划侧开返工卡。
