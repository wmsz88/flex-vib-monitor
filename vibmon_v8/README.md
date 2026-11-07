# vibmon_v8

> 任务：T014 · 分层检测实验版  
> 基线：由 vibmon_v6 复制  
> 状态：T014 初版已接入分层检测；宿主测试通过；**尚未实板验证**

## 1. 版本关系

| 版本 | 角色 |
|---|---|
| vibmon_v6 | **最终版 / 出发版** |
| vibmon_v7 | 现场适配参考版 |
| vibmon_v8 | 分层检测实验版 |

V8 不替代 V6，也不修改 V6/V7。

## 2. 目标架构

```text
sampler
  ↓
dsp
  ↓
state
  ├─ RealtimeDetector  → 当前 VIB / CT
  └─ UniformDetector   → 稳定性 / 离散度 / 有效率
  ↓
output
  ├─ $VD 35 字节比赛帧（不改）
  └─ #LAYER 调试行（按 d 开启）
```

## 3. 预期调试行

按 `d` 后输出：

```text
#LAYER vib= ct= stable= fmed= fspread= amed= aspread= ssmed= valid=
```

## 4. 纪律

- 不改 `$VD` 帧格式
- 不改 V6 接触判据
- 分层输出不反向影响 `Result`
- `LAYERED_DETECT_ENABLE=0` 时回到 V6 行为

## 5. 当前测试结果

```text
contact_test      89/89
gyro_test          8/8
comp_frame_test   22/22
dump_test        15/19（4 项既有列数问题）
ss_replay         可运行
layered_detection  8/8
```

全部 0 warning。

## 5. 编译

arduino-cli 的 build path 不能包含中文，先复制到英文临时目录再编译。

```bash
arduino-cli compile --fqbn esp32:esp32:esp32c3 --warnings all "<英文临时目录>/vibmon_v8"
```

CDC 构建：

```bash
arduino-cli compile --fqbn "esp32:esp32:esp32c3:CDCOnBoot=cdc" --warnings all "<英文临时目录>/vibmon_v8"
```
