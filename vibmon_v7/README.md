# vibmon_v7 —— 现场适配版（= v6 接触三条 + 黄灯/`y` 命令 + 三只预案宏）

由 `vibmon_v6` 拷贝（2026-11-06），**判据链一字未改**（`state.cpp` / `dsp.cpp` / `posture.cpp` / `sampler.cpp` / `mpu.cpp` 与 v6 逐字节相同）。

为什么要单独一个目录：`vibmon_v6` 是 T013 已经上板验收过的那一版，**保持原样归档**；T010 的黄灯语义改动与现场预案宏全部落在本目录，避免"改了的代码"和"验过的代码"混在一起。

## 相对 v6 改了什么

只动 `output.cpp` 与 `config.h`：

1. **黄灯不再"振动中常亮"**，改成接触联动：CT=1 时 2Hz 闪、CT=0 灭；`LED_FAULT`（4Hz 快闪）与 `LED_CAL` 不变，绿灯 1Hz 心跳不变。
2. 新增 **`y` 命令**三态循环 AUTO → ON（常亮）→ OFF（常灭），回显 `[LEDY] AUTO|ON|OFF`；`S` 状态行加黄灯行，`?` 帮助加 `y`。
3. 新增三只**现场预案宏**（默认全部保持出发行为）：

| 宏 | 默认 | 作用 |
|---|---|---|
| `COMP_FRAME_CSV` | 0 | 1 = 比赛口改发 CSV 文本行 `ct,vib,freq,amp,ss` |
| `COMP_FRAME_DIV` | 1 | 2 = 25Hz、5 = 10Hz（只降帧率，SEQ 仍连续） |
| `COMP_FRAME_GATED` | 0 | 1 = 比赛口收 `S` 才开始发、`X` 停止 |

## 继承自 v6 的接触三条（未改动）

手指收拢并停下后振动才算接触；接触后不看加速度；只有陀螺判出张开或再转动 10° 才分离。

现场开关（`config.h`）：

| 宏 | 默认 | 作用 |
|---|---|---|
| `CT_V6_ENABLE` | 1 | 0 = 回到 v5 + T011 行为 |
| `CT_SETTLE_ENABLE` | 1 | 0 = 不要求手指停下 |
| `CT_SETTLE_DPS` / `CT_SETTLE_MS` | 10 / 150 | 停下门限 |
| `CT_RELEASE_DEG` | 10 | 接触后转过多少度算分离 |
| `GYRO_LP_HZ` | 4 | 陀螺低通截止 |

串口按 `d` 打开调试行：`#F=...` 与 `#V6 closed= ang= lpW= settled= moved=`。

## 测试与预编译

- 宿主：`test\run_all.ps1`（contact 89/89、gyro 8/8、comp_frame 22/22、dump 15/19、led 64/64、ss_replay 可运行）
- 预编译变体：`_variants\`（def/cl/csv/div2/gate/v5sem/uart）+ `_variants\固件变体清单.md`
- 现场手册：`..\vibmon_v5\答辩\现场适配预案手册_20261106.md`

> 本目录**不带** `测试\`（T013 的实板采集语料与 `采集.py`/`分析.py` 等脚本留在 `..\vibmon_v6\测试\`）。
> v7 的判据链与 v6 逐字节相同，回放/复现直接用 v6 那份语料即可；实板体检也沿用 v6 的脚本，把输出目录指到这边就行。

## 编译

arduino-cli 的 `--build-path` 不能含中文，先把 sketch 拷到英文临时目录（文件夹名保持 `vibmon_v7`）再编；产物按需拷回。
